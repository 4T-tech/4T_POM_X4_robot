#ifndef __GPIO_H__
#define __GPIO_H__
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// --- 外部声明 ---
// 声明在main.c或其他地方定义的GPIO中断回调函数
extern void on_gpio_irq(uint gpio, uint32_t events);


// --- 引脚定义 (根据PDF映射) ---

// G4的LED_Pin, 请根据您的RP2040板卡硬件连接进行定义
// 例如，如果您外接了一个LED到GPIO15
#define USER_LED_PIN        15

// G4的ESP32_IO9_Pin (来自 GPIOC) -> RP GPIO25
#define ESP_IO9_PIN         25

// G4的ESP32_EN_Pin (来自 GPIOC) -> RP GPIO23
#define ESP_EN_PIN          23


// G4的PB2 (EXIT 2) -> IMU_INT1 -> RP GPIO21
#define IMU_INT1_PIN        21

// G4的PB12 (EXIT 12) -> IMU_INT2 -> RP GPIO22
#define IMU_INT2_PIN        22




#endif