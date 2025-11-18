// usart.h

#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "hardware/timer.h" // For repeating_timer

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// TODO: 请将您项目特定的头文件包含在此处
#include "reg_protocol.h"
#include "Dog_Sport.h"
#include "leg_ik.h"
#include "POM_lsm6dso.h"


// TODO: 您应该创建一个新的头文件 (例如, "rp2040_hal.h") 来声明您的外设初始化函数
#include "gpio.h" // 假设所有 MX_..._Init() 声明都在此
#include "usart.h" 
#include "timers.h"
#include "esp32_control.h"
#include "MadgwickAHRS.h" // 使用新的姿态解算头文件

// Keil / CMSIS / SDK 底层头文件
#include "cmsis_compiler.h"
#include "RTE_Components.h"

// 如果您想将printf重定向到MDK的调试窗口
#if defined(RTE_Compiler_EventRecorder) && defined(RTE_Compiler_IO_STDOUT_EVR)
#   include <EventRecorder.h>
#endif

// 主初始化函数，在 main() 中调用
void RP_UARTS_Init(void);

#ifdef __cplusplus
}
#endif

#endif // __USART_H__