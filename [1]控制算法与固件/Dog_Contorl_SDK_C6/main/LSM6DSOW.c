/* LSM6DSOX 陀螺仪的驱动文件 */
#include <stdio.h>
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "freertos/FreeRTOS.h"

#define SDA_PIN    6                        // GPIO6 作为 I2C 的 SDA 引脚
#define SCL_PIN    7                        // GPIO7 作为 I2C 的 SCL 引脚

/************* LSM6DSOX 内部寄存器 *************/
#define LSM6DSOX_Address       0x6B            // SDO/SA0 接地为 0x6A，接高为 0x6B
#define LSM6DSOX_Device_ID_Ad  0x0F            // 设备 ID 寄存器地址
#define LSM6DSOX_CTRL1_XL      0x10            // 加速度计控制寄存器1
#define LSM6DSOX_CTRL2_G       0x11            // 陀螺仪控制寄存器2
#define LSM6DSOX_CTRL3_C       0x12            // 控制寄存器3
#define LSM6DSOX_OUTX_L_G      0x22            // 陀螺仪X轴低字节寄存器
#define LSM6DSOX_OUTX_L_A      0x28            // 加速度X轴低字节寄存器
/***********************************************/

#define LSM6DSOX_Device_ID     0x6C         // 设备 ID 寄存器值

i2c_master_bus_handle_t bus_handle;         // I2C 总线句柄
i2c_master_dev_handle_t lsm6dsow_handle;    // LSM6DSOW 设备句柄

/**
 * @brief  读取 LSM6DSOX 设备 ID 并验证连接状态
 *
 * 读取 WHO_AM_I 寄存器（0x0F），判断返回值是否为 0x6C。
 * 如果返回 0x6C，说明 LSM6DSOX 连接正常，否则连接失败。
 */
void lsm6dsow_read_id(void) {
    uint8_t reg_addr = LSM6DSOX_Device_ID_Ad;
    uint8_t id = 0;

    // 先写寄存器地址，再读1字节
    ESP_ERROR_CHECK(i2c_master_transmit_receive(lsm6dsow_handle, &reg_addr, 1, &id, 1, 100));
    if (id == LSM6DSOX_Device_ID) {
        printf("LSM6DSOX connected successfully!\n");
    } else {
        printf("Failed to connect to LSM6DSOX!\n");
    }
}

/**
 * @brief  向 LSM6DSOX 指定寄存器写入一个字节数据
 *
 * @param  dev_handle I2C 设备句柄
 * @param  reg        目标寄存器地址
 * @param  data       要写入的数据
 */
static void lsm6dsow_write_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, 100));
}

/**
 * @brief  初始化 LSM6DSOX 传感器
 *
 * 配置 I2C 总线并添加 LSM6DSOX 设备，完成以下初始化步骤：
 * - 复位芯片
 * - 配置加速度计 ODR=104Hz，量程±2g
 * - 配置陀螺仪 ODR=104Hz，量程±2000dps
 * - 使能数据锁存和自动地址递增
 * 初始化完成后自动读取并验证设备ID。
 */
void lsm6dsow_init(void) {
    // 配置 I2C 主机参数
    i2c_master_bus_config_t conf = {
        .sda_io_num = SDA_PIN,                   // SDA 引脚
        .scl_io_num = SCL_PIN,                   // SCL 引脚
        .clk_source = I2C_CLK_SRC_DEFAULT,       // 时钟源
        .glitch_ignore_cnt = 7,                  // 消抖滤波
        .i2c_port = I2C_NUM_0,                   // I2C 端口号
        .flags.enable_internal_pullup = true,    // 启用内部上拉
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &bus_handle));
    
    // 添加一个 lsm6dsow 设备
    i2c_device_config_t dev_conf = {
        .device_address = LSM6DSOX_Address,      // 设备地址
        .dev_addr_length = I2C_ADDR_BIT_7,       // 地址长度
        .scl_speed_hz = 100000,                  // SCL 频率
    };
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_conf, &lsm6dsow_handle));

    lsm6dsow_read_id();

/************************** lsm6dsow 初始化 **************************/
    // 1. 复位
    lsm6dsow_write_reg(lsm6dsow_handle, LSM6DSOX_CTRL3_C, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2. 加速度计 ODR=104Hz, ±2g
    lsm6dsow_write_reg(lsm6dsow_handle, LSM6DSOX_CTRL1_XL, 0x40);

    // 3. 陀螺仪 ODR=104Hz, ±2000dps
    lsm6dsow_write_reg(lsm6dsow_handle, LSM6DSOX_CTRL2_G, 0x4C);

    // 4. 数据锁存/自动地址递增
    lsm6dsow_write_reg(lsm6dsow_handle, LSM6DSOX_CTRL3_C, 0x44);

/********************************************************************/
}

/**
 * @brief  读取 LSM6DSOX 三轴加速度计和陀螺仪的原始数据
 *
 * 从 LSM6DSOX 连续读取 12 字节，分别获取三轴陀螺仪和加速度计的原始数据（单位：LSB）。
 * 
 * @param[out] accel 三轴加速度计原始数据数组，accel[0]=X，accel[1]=Y，accel[2]=Z
 * @param[out] gyro  三轴陀螺仪原始数据数组，gyro[0]=X，gyro[1]=Y，gyro[2]=Z
 *
 * @note 结果为原始 LSB，需根据量程配置换算为物理单位（g、°/s）。
 */
void lsm6dsow_read_accel_gyro(int16_t accel[3], int16_t gyro[3]) {
    uint8_t reg_addr = LSM6DSOX_OUTX_L_G;
    uint8_t buf[12] = {0};

    // 连续读取陀螺仪和加速度计的12字节数据
    ESP_ERROR_CHECK(i2c_master_transmit_receive(lsm6dsow_handle, &reg_addr, 1, buf, 12, 100));

    // 陀螺仪
    gyro[0] = (int16_t)(buf[1] << 8 | buf[0]);    // X
    gyro[1] = (int16_t)(buf[3] << 8 | buf[2]);    // Y
    gyro[2] = (int16_t)(buf[5] << 8 | buf[4]);    // Z
    // 加速度计
    accel[0] = (int16_t)(buf[7] << 8 | buf[6]);   // X
    accel[1] = (int16_t)(buf[9] << 8 | buf[8]);   // Y
    accel[2] = (int16_t)(buf[11] << 8 | buf[10]); // Z
}
