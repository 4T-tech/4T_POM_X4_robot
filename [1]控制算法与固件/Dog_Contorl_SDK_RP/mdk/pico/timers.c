/**
  ******************************************************************************
  * @file    timers.c
  * @brief   Provides code for the timer configuration on RP2040.
  * @note    This replaces the G4's TIM1 and TIM2 functionality using the
  *          Pico SDK's repeating timer API.
  ******************************************************************************
  */

#include "pico/stdlib.h"
#include "hardware/timer.h"

#include "timers.h"

/* External Callback Functions -----------------------------------------------*/
// 声明在 main.c 中实现的定时器回调函数
extern bool repeating_timer_20ms_callback(struct repeating_timer *t);
extern bool repeating_timer_1ms_callback(struct repeating_timer *t);


/* Private Variables ---------------------------------------------------------*/
static struct repeating_timer timer_20ms;
static struct repeating_timer timer_1ms;


/* Public Functions ----------------------------------------------------------*/

/**
  * @brief  Initializes two repeating timers to match the frequencies of G4's TIM1 and TIM2.
  * @note   This single function replaces both MX_TIM1_Init and MX_TIM2_Init.
  */
void RP_Timers_Init(void)
{
    // --- 配置 20ms 定时器 
    // 第一个参数是周期（毫秒）。使用负数可以让SDK尝试更精确地安排定时器，避免漂移。
    // 第二个参数是定时器触发时要调用的回调函数。
    // 第三个参数是用户数据指针（这里我们不需要，所以是NULL）。
    // 第四个参数是指向 repeating_timer 结构体的指针。
    add_repeating_timer_ms(-20, repeating_timer_20ms_callback, NULL, &timer_20ms);


    // --- 配置 1ms 定时器 (替代 G4 的 TIM2) ---
    add_repeating_timer_ms(-1, repeating_timer_1ms_callback, NULL, &timer_1ms);
}


void Delay_MS(uint32_t ms)
{
    // RP2040 SDK提供的标准阻塞延时
    sleep_ms(ms);
}