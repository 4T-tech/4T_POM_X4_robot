#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"            // GPIO 驱动
#include "driver/uart.h"            // UART 驱动
#include "driver/gptimer.h"         // 包含GPTimer驱动头文件
#include "driver/usb_serial_jtag.h" // USB CDC 驱动
#include "freertos/FreeRTOS.h"      // FreeRTOS 头文件
#include "freertos/task.h"          // FreeRTOS 任务头文件
#include "esp_log.h"                // ESP32 日志头文件

#include "LSM6DSOW.h"               // LSM6DSOW 头文件
#include "my_uart.h"                // 串口初始化头文件
#include "LIS2MDL.h"                // LIS2MDL 头文件
#include "MadgwickAHRS.h"           // MadgwickAHRS 头文件
#include "Dog_Sport.h"              // 运动控制头文件
#include "reg_protocol.h"           // 寄存器协议头文件
#include "leg_ik.h"                 // 逆运动学头文件
#include "my_timer.h"               // 定时器头文件
#include "USB_CDC.h"                // USB CDC 头文件

#include "BLEt.h"

#endif