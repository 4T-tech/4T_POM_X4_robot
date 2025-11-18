#include "gpio.h"
/**
  * @brief 初始化所有在G4代码中配置的GPIO引脚，并设置中断
  * @note  此函数替代了原G4版本的 RP_GPIO_Init
  */
void RP_GPIO_Init(void)
{
    // 在RP2040中, GPIO时钟是默认使能的, 无需类似 __HAL_RCC_GPIOC_CLK_ENABLE() 的操作

    // --- 配置输出引脚 ---
    // G4: HAL_GPIO_WritePin(GPIOC, LED_Pin|ESP32_IO9_Pin|ESP32_EN_Pin, GPIO_PIN_RESET);
    // G4: HAL_GPIO_Init(GPIOC, &GPIO_InitStruct) with MODE_OUTPUT_PP
    
    // 配置 USER_LED_PIN
    gpio_init(USER_LED_PIN);
    gpio_set_dir(USER_LED_PIN, GPIO_OUT);
    gpio_put(USER_LED_PIN, 0); // 设置初始电平为低

    // 配置 ESP_IO9_PIN
    gpio_init(ESP_IO9_PIN);
    gpio_set_dir(ESP_IO9_PIN, GPIO_OUT);
    gpio_put(ESP_IO9_PIN, 0); // 设置初始电平为低

    // 配置 ESP_EN_PIN
    gpio_init(ESP_EN_PIN);
    gpio_set_dir(ESP_EN_PIN, GPIO_OUT);
    gpio_put(ESP_EN_PIN, 0); // 设置初始电平为低


    // --- 配置中断输入引脚 ---
    // G4: HAL_GPIO_Init(GPIOA/GPIOB, &GPIO_InitStruct) with MODE_IT_RISING, NOPULL
    // G4: HAL_NVIC_EnableIRQ(...)
    
    // 配置 IMU_INT1_PIN (对应 G4 的 PB2)
    gpio_init(IMU_INT1_PIN);
    gpio_set_dir(IMU_INT1_PIN, GPIO_IN);
    // G4代码中为 GPIO_NOPULL。如果外部电路没有上拉/下拉，您可能需要在这里启用一个
    // 例如: gpio_pull_up(IMU_INT1_PIN);
    gpio_disable_pulls(IMU_INT1_PIN); 

    // 配置 IMU_INT2_PIN (对应 G4 的 PB12)
    gpio_init(IMU_INT2_PIN);
    gpio_set_dir(IMU_INT2_PIN, GPIO_IN);
    gpio_disable_pulls(IMU_INT2_PIN);



    // --- 统一使能所有GPIO中断并注册回调函数 ---
    // 这一个函数调用替代了所有 HAL_NVIC_SetPriority 和 HAL_NVIC_EnableIRQ 的功能
    // on_gpio_irq 是您需要在main.c或其它地方实现的全局回调函数
    // 它会处理来自所有GPIO引脚的中断
    gpio_set_irq_enabled_with_callback(IMU_INT1_PIN, GPIO_IRQ_EDGE_RISE, true, &on_gpio_irq);
    gpio_set_irq_enabled(IMU_INT2_PIN, GPIO_IRQ_EDGE_RISE, true); // 已经注册过回调，只需使能
   
}