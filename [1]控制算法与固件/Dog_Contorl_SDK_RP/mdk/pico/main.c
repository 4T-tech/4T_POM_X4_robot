/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body for RP2040 port - Framework Only (No USB)
 ******************************************************************************
 * @attention
 *
 * This code provides a structural framework for migrating a robotic dog project
 * from STM32G4 to RP2040. It mirrors the original main.c's logic flow.
 *
 * Copyright (c) 2025 YourCompany. All rights reserved.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
// Standard RP2040 SDK Includes
#include "pico/stdlib.h"
#include "hardware/timer.h" // For repeating_timer

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "reg_protocol.h"
#include "Dog_Sport.h"
#include "leg_ik.h"
#include "POM_lsm6dso.h"


#include "gpio.h" 
#include "usart.h" 
#include "timers.h"
#include "esp32_control.h"
#include "MadgwickAHRS.h" // 使用新的姿态解算头文件

// Keil / CMSIS / SDK 底层头文件
#include "cmsis_compiler.h"
#include "RTE_Components.h"

#include "hardware/clocks.h"
#include "hardware/resets.h"

// 如果您想将printf重定向到MDK的调试窗口
#if defined(RTE_Compiler_EventRecorder) && defined(RTE_Compiler_IO_STDOUT_EVR)
#   include <EventRecorder.h>
#endif


#define USE_DEBUG
#define USE_INFO
#define USE_PRINT
#define BUF_SIZE 1024

uint8_t uart_reg_buf[BUF_SIZE];      // 对应 G4 的 uart_rx2_buf1
uint8_t uart_esp_buf[BUF_SIZE];      // 对应 G4 的 uart_rx4_buf1
uint8_t* BUF_addr = uart_esp_buf;     // 保持与G4代码的兼容性

extern uint16_t REG_ADDR[REG_LEN];
extern uint16_t ADDR;
extern uint16_t UP;

volatile uint8_t Mode = 0;

unsigned char ESP_RX_Result = 0;   // 对应 G4 的 UART4_RX_Result
unsigned char REG_RX_Result = 0;   // 对应 G4 的 UART2_RX_Result

uint8_t  FIFO_TX_BUF[50];
uint8_t FIFO_NUM = 0;
uint8_t FIFO_BUF[60][7];

uint32_t Pos_Stand_Time = 0;

float LSM6DSOW_Yaw;
float LSM6DSOW_Roll;
float LSM6DSOW_Pitch;

volatile uint32_t sysTickMs = 0;
uint8_t TIMER_20MS_FLAG = 0;


void RP_GPIO_Init(void);
void Madgwick_Process_FIFO(uint8_t FIFO_data[][7], uint8_t fifo_num);


/**
 * @brief  Performs the absolute essential low-level system initializations.
 * @note   This function MUST be called as the very first thing in main().
 */
static void system_init(void)
{
    // (1) Update the SystemCoreClock variable to match the hardware clock speed.
    // This is crucial for peripherals like UART to calculate correct baud rates.
    void SystemCoreClockUpdate (void);
    SystemCoreClockUpdate();
		gpio_put(25,0);
    // (2) Initialize the Keil Event Recorder for printf redirection.
    // This requires the "Compiler:I/O:STDOUT:EVR" component in RTE.
    #if defined(RTE_Compiler_EventRecorder) && defined(RTE_Compiler_IO_STDOUT_EVR)
        EventRecorderInitialize(0, 1);
    #endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    
    // 手动配置系统时钟到全速 (125MHz)
    set_sys_clock_khz(125 * 1000, true);


    // 使用 Keil 的 Event Recorder (需要RTE配置)
    #if defined(RTE_Compiler_EventRecorder) && defined(RTE_Compiler_IO_STDOUT_EVR)
        EventRecorderInitialize(0, 1);
    #endif
   
    // 短暂延时，等待所有时钟稳定
    sleep_ms(100);

    // 开始测试
		printf("\n--- RP2040 Dog Project Starting ---\n");
    /* MCU Configuration--------------------------------------------------------*/
    stdio_init_all();
	
		unreset_block_wait(RESETS_RESET_IO_BANK0_BITS);
    unreset_block_wait(RESETS_RESET_PADS_BANK0_BITS);
    unreset_block_wait(RESETS_RESET_UART0_BITS); 
    unreset_block_wait(RESETS_RESET_UART1_BITS); 
    unreset_block_wait(RESETS_RESET_I2C1_BITS);   
    unreset_block_wait(RESETS_RESET_TIMER_BITS); 

    /* Initialize all configured peripherals by calling the declared functions */
    RP_GPIO_Init();
    RP_UARTS_Init();
		printf("\n--- RP2040 UARTS INIT ---\n");
    RP_Timers_Init();
		printf("\n--- RP2040 Timeer INIT ---\n");

    Reset_ESP32_C3_To_Run();
    sleep_ms(1000);
	
    if(LSM6DSO_Initialization() != POM_LSM6DSO_OK)
    {
      printf("LSM6DSO Init Failed\r\n");
			printf("\n--- RP2040 LSM INIT Failed ---\n");
    }
    if(LSM6DSO_Set_FIFO_Mode() != POM_LSM6DSO_OK)
    {
      printf("LSM6DSO Set Failed\r\n");
    }

			printf("\n--- RP2040 LSM INIT ---\n");
		

    Stand_Up_Smoothly();


    while (1)
    {
        // --- 核心姿态解算 ---
        // 检查GPIO中断是否已从IMU获取到新的FIFO数据
        if(FIFO_NUM > 0)
        {
            // 调用基于Madgwick的FIFO处理函数
            Madgwick_Process_FIFO(FIFO_BUF, FIFO_NUM);
            
            // 清除标志位，等待下一次中断
            FIFO_NUM = 0;
        }

        // --- 20ms 定时任务 ---
        if(TIMER_20MS_FLAG)
        {
             TIMER_20MS_FLAG = 0;
             
             // G4代码中的逻辑，用于蓝牙APP等查询
             switch(ADDR)
            {
             case 43:
                 Read_Battery_Volt();
             break;
             case 44:
                 Read_Temp();
             break;
            }
            ADDR = 0;
        }
        
        // --- UART 指令处理 ---
        if(ESP_RX_Result == 1) // 来自ESP的指令
        {
             // G4代码中的逻辑
             switch(ADDR)
            {
                case 43:
                    REG_6Byte_Send(79 , 43);
                break;
                case 44:
                    REG_6Byte_Send(79 , 44);
                break;
            }
            ESP_RX_Result = 0;
        }

        if(REG_RX_Result == 1) // 来自舵机驱动板的返回数据 未添加此串口接收函数
        {
             REG_RX_Result = 0;
        }

    }
}




/**
 * @brief 20ms定时器回调 
 */
bool repeating_timer_20ms_callback(struct repeating_timer *t)
{
    Dog_Control();
    TIMER_20MS_FLAG = 1;															
    return true; // 返回true以保持定时器运行
}

/**
 * @brief 1ms定时器回调 
 */
bool repeating_timer_1ms_callback(struct repeating_timer *t)
{
    static uint32_t mode_control_counter = 0;

    Pos_Stand_Time++;
    sysTickMs++;
    mode_control_counter++;

    if(mode_control_counter >= 20)
    {
        mode_control_counter = 0;
        Mode_Control();
    }
    return true; // 返回true以保持定时器运行
}

/**
 * @brief GPIO中断回调 
 * @note  这个函数需要在您的GPIO初始化代码中通过 gpio_set_irq_enabled_with_callback 注册。
 */
void on_gpio_irq(uint gpio, uint32_t events)
{
 if (gpio == IMU_INT1_PIN) 
    {
      FIFO_NUM = LSM6DSO_FIFO_Buf_Get(FIFO_BUF);
    }
}

void Madgwick_Process_FIFO(uint8_t FIFO_data[][7], uint8_t fifo_num)
{
    static uint32_t last_timestamp = 0;
    
    float sensitivity_acc = 0.0f;
    float sensitivity_gyro = 0.0f;
    
    // 1. 获取传感器的灵敏度，用于将原始数据转换为物理单位
    if (LSM6DSO_ACC_GetSensitivity(&lsm6dso_obj, &sensitivity_acc) != LSM6DSO_OK) return;
    if (LSM6DSO_GYRO_GetSensitivity(&lsm6dso_obj, &sensitivity_gyro) != LSM6DSO_OK) return;

    // 定义临时变量来存储从FIFO解析出的数据
    float ax_g, ay_g, az_g;
    float gx_rps, gy_rps, gz_rps;
    uint32_t current_timestamp;
    
    // 标志位，确保我们在一组数据中同时获得了加速度和陀螺仪
    bool acc_valid = false;
    bool gyro_valid = false;

    // 2. 遍历FIFO缓冲区中的每一个数据包
    for (int i = 0; i < fifo_num; i++)
    {
        // 使用指针转换来从字节数组中提取16位数据
        int16_t* data_ptr = (int16_t*)&FIFO_data[i][1];

        // 3. 根据数据包的标签(TAG)来解析数据
        switch (FIFO_data[i][0])
        {
            case LSM6DSO_XL_NC_TAG: // 加速度计数据包
                // 单位换算：从 LSB -> mg -> g
                ax_g = (float)data_ptr[0] * sensitivity_acc / 1000.0f;
                ay_g = (float)data_ptr[1] * sensitivity_acc / 1000.0f;
                az_g = (float)data_ptr[2] * sensitivity_acc / 1000.0f;
                acc_valid = true;
                break;

            case LSM6DSO_GYRO_NC_TAG: // 陀螺仪数据包
                // 单位换算：从 LSB -> mdps -> rad/s
                #define MDPS_TO_RADPS (0.001f * 3.1415926535f / 180.0f)
                gx_rps = (float)data_ptr[0] * sensitivity_gyro * MDPS_TO_RADPS;
                gy_rps = (float)data_ptr[1] * sensitivity_gyro * MDPS_TO_RADPS;
                gz_rps = (float)data_ptr[2] * sensitivity_gyro * MDPS_TO_RADPS;
                gyro_valid = true;
                break;

            case LSM6DSO_TIMESTAMP_TAG: // 时间戳数据包
                // 从4个字节重构32位时间戳
                current_timestamp = (uint32_t)FIFO_data[i][1] |
                                    (uint32_t)(FIFO_data[i][2] << 8) |
                                    (uint32_t)(FIFO_data[i][3] << 16) |
                                    (uint32_t)(FIFO_data[i][4] << 24);

                // 4. 当集齐了加速度、陀螺仪和时间戳后，进行姿态解算
                if (acc_valid && gyro_valid)
                {
                    if (last_timestamp == 0) {
                        last_timestamp = current_timestamp;
                        break; // 第一次，只记录时间
                    }

                    // 计算时间差 (dt)
                    uint32_t delta_ticks;
                    if (current_timestamp >= last_timestamp) {
                        delta_ticks = current_timestamp - last_timestamp;
                    } else {
                        // 处理32位时间戳翻转的情况
                        delta_ticks = (0xFFFFFFFF - last_timestamp + 1) + current_timestamp;
                    }
                    
                    // LSM6DSO的时间戳分辨率是 25 微秒/LSB
                    float dt = (float)delta_ticks * 25.0f * 1e-6f;
                    
                    // 调用【优化后】的Madgwick算法
                    MadgwickAHRSupdateIMU(gx_rps, gy_rps, gz_rps, ax_g, ay_g, az_g, dt);
                    
                    last_timestamp = current_timestamp;
                    
                    // 重置标志位，准备接收下一组数据
                    acc_valid = false;
                    gyro_valid = false;
                }
                break;
            
            default:
                break;
        }
    }
    
    // 5. 在处理完所有FIFO数据后，最后计算一次欧拉角并更新全局变量
    Madgwick_GetYawPitchRoll(&LSM6DSOW_Yaw, &LSM6DSOW_Pitch, &LSM6DSOW_Roll);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{

  while (1)
  {
      // __breakpoint();
  }
  
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    panic("assertion failed: file %s, line %u\n", file, line);
}
#endif /* USE_FULL_ASSERT */