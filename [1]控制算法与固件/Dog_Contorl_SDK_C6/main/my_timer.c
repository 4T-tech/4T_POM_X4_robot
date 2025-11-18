#include "include.h"

extern uint32_t Pos_Stand_Time;
extern uint8_t TIMER_20MS_FLAG;

/**
 * @brief 定时器1的中断服务回调函数 (ISR)
 * 
 * @note  !!! 致命错误警告 !!!
 *        此函数在中断上下文中执行，其可用的栈空间非常小。
 *        你在这里直接调用了 `Dog_Control()`，这是一个包含大量复杂计算的函数，
 *        它会瞬间耗尽中断栈空间，导致 "Stack protection fault" (栈溢出) 错误并使系统崩溃。
 * 
 *        正确做法是：在此回调函数中只设置一个标志位（例如你已经做的 `TIMER_20MS_FLAG = 1;`），
 *        然后在一个高优先级的普通任务中检查这个标志位，并由该任务来执行 `Dog_Control()`。
 * 
 * @param timer     触发此回调的定时器句柄。
 * @param edata     包含报警事件数据的结构体指针。
 * @param user_data 用户在注册回调时传入的自定义数据指针。
 * 
 * @return bool     用于指示此ISR是否唤醒了一个更高优先级的任务，从而需要进行任务切换。
 *                  - 如果你没有调用任何 `...FromISR()` 结尾的FreeRTOS函数来唤醒任务，应返回 `false`。
 *                  - 注意：这个返回值**不控制**定时器是否继续运行。定时器的自动重载是由
 *                    `gptimer_alarm_config_t` 中的 `.flags.auto_reload_on_alarm = true;` 控制的。
 */
static bool Timer1_Callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    
    return pdFALSE; 
}

static bool Timer2_Callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    Pos_Stand_Time++;
    if(Pos_Stand_Time >=  20)
    {   
        Pos_Stand_Time -= 20;
        Mode_Control();
    }

    return pdFALSE; 
}

/**
 * @brief   初始化定时器1 (GPTimer)
 * @details 配置GPTimer1为1MHz频率，20ms周期自动重载，并注册回调函数
 */
void Timer1_Init(void)
{
    // 初始化定时器句柄
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer1 = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,     // 时钟源选择
        .direction = GPTIMER_COUNT_UP,          // 计数方向(向上计数)
        .resolution_hz = 1000000,               // 1 MHz -> 1 tick = 1us （不能太小，CPU主频 / resolution_hz < 65536）
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer1, &gptimer));

    // 注册回调函数
    gptimer_event_callbacks_t timer1_cbs = {
        .on_alarm = Timer1_Callback, // 可以在这里设置一个回调函数
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &timer1_cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    // 配置定时器为20ms周期自动重载
    gptimer_alarm_config_t timer1_config = {
        .alarm_count = 20000,                   // 20 ticks = 20 ms
        .reload_count = 0,                      // 自动重载值
        .flags.auto_reload_on_alarm = true,     // 到达报警值后自动重载
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &timer1_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

/**
 * @brief   初始化定时器2 (GPTimer)
 * @details 配置GPTimer2为1MHz频率，20ms周期自动重载，并注册回调函数
 */
void Timer2_Init(void)
{
    // 初始化定时器句柄
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer2 = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz -> 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer2, &gptimer));

    // 注册回调函数
    gptimer_event_callbacks_t timer2_cbs = {
        .on_alarm = Timer2_Callback, // 可以在这里设置一个回调函数
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &timer2_cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    // 配置定时器为20ms周期自动重载
    gptimer_alarm_config_t timer2_config = {
        .alarm_count = 20000, // 20 ms
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &timer2_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}