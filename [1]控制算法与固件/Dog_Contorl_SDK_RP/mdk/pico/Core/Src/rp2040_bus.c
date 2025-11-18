/**
  ******************************************************************************
  * @file    rp2040_bus.c
  * @brief   Source file for the BSP BUS IO driver for RP2040 platform.
  * @note    This file replaces the G4's custom_bus.c and provides the
  *          necessary low-level I2C and Tick functions for the sensor drivers.
  ******************************************************************************
  */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "rp2040_bus.h" // 包含我们自己的头文件
#include <string.h>    // For memcpy

/* Private Defines -----------------------------------------------------------*/

// --- I2C Configuration based on PDF ---
#define I2C_INSTANCE        i2c1
#define I2C_SDA_PIN         2  
#define I2C_SCL_PIN         3  

// G4的I2C配置为1000KHz (Fast Mode Plus)，我们也匹配这个速率
#define I2C_BAUDRATE        1000 * 1000

/* External Variables --------------------------------------------------------*/

// G4代码中HAL_GetTick()返回的是毫秒数。
// 需要一个在RP2040项目中由1ms定时器维护的全局变量。
extern volatile uint32_t sysTickMs;

/* Public Functions ----------------------------------------------------------*/

/**
  * @brief  Initialize I2C HAL for RP2040.
  * @retval BSP status
  */
int32_t BSP_I2C1_Init(void) {
    // Pico SDK 的 i2c_init 会返回实际设置的波特率，但我们这里简化处理。
    // 正常情况下不会失败。
		set_sys_clock_khz(125 * 1000, true);
    i2c_init(I2C_INSTANCE, I2C_BAUDRATE);

    // 设置GPIO功能并启用内部上拉，这对应于G4的开漏模式(AF_OD)
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    return BSP_ERROR_NONE;
}

/**
  * @brief  DeInitialize I2C HAL for RP2040.
  * @retval BSP status
  */
int32_t BSP_I2C1_DeInit(void) {
    i2c_deinit(I2C_INSTANCE);
    
    // 将GPIO引脚恢复为默认状态
    gpio_disable_pulls(I2C_SDA_PIN);
    gpio_disable_pulls(I2C_SCL_PIN);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_NULL);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_NULL);

    return BSP_ERROR_NONE;
}

/**
  * @brief  Write a value in a register of the device through the I2C bus.
  * @param  Addr    Device address on Bus.
  * @param  Reg     The target register address to write
  * @param  pData   Pointer to data buffer to write
  * @param  len     Data Length
  * @retval BSP status
  */
int32_t BSP_I2C1_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len) {
    // Pico SDK的I2C写函数需要一个连续的缓冲区，第一个字节是寄存器地址
    uint8_t buffer[len + 1];
    buffer[0] = (uint8_t)Reg;
    memcpy(buffer + 1, pData, len);
    
    // i2c_write_blocking 返回写入的字节数，如果出错则返回一个错误码
    int ret = i2c_write_blocking(I2C_INSTANCE, (uint8_t)Addr, buffer, len + 1, false);
    
    if (ret == PICO_ERROR_GENERIC) {
        return BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE; // 最常见的错误是设备无应答 (NAK)
    } else if (ret < 0) {
        return BSP_ERROR_BUS_FAILURE; // 其他I2C总线错误
    }
    
    // 写入的字节数不等于期望的字节数，也视为失败
    if (ret != (len + 1)) {
        return BSP_ERROR_PERIPH_FAILURE;
    }

    return BSP_ERROR_NONE;
}

/**
  * @brief  Read a register of the device through the I2C bus.
  * @param  Addr    Device address on Bus.
  * @param  Reg     The target register address to read
  * @param  pData   Pointer to data buffer to read
  * @param  len     Data Length
  * @retval BSP status
  */
int32_t BSP_I2C1_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len) {
    uint8_t reg_addr = (uint8_t)Reg;

    // 步骤1: 向设备写入要读取的寄存器地址。
    // `nostop = true` 表示在写操作后不发送STOP信号，保持总线活动，准备进行读操作。
    int ret_write = i2c_write_blocking(I2C_INSTANCE, (uint8_t)Addr, &reg_addr, 1, true);
    
    if (ret_write < 0) {
        return (ret_write == PICO_ERROR_GENERIC) ? BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE : BSP_ERROR_BUS_FAILURE;
    }
    
    // 步骤2: 从设备读取数据。
    int ret_read = i2c_read_blocking(I2C_INSTANCE, (uint8_t)Addr, pData, len, false);

    if (ret_read < 0) {
        return (ret_read == PICO_ERROR_GENERIC) ? BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE : BSP_ERROR_BUS_FAILURE;
    }
    
    // 读取的字节数不等于期望的字节数，视为失败
    if (ret_read != len) {
        return BSP_ERROR_PERIPH_FAILURE;
    }

    return BSP_ERROR_NONE;
}

/**
  * @brief  Return system tick in ms.
  * @retval Current system tick timestamp.
  */
int32_t BSP_GetTick(void) {
  return (int32_t)sysTickMs;
}