/**
  ******************************************************************************
  * @file    usart.c
  * @brief   Provides code for UART configuration and interrupt handling on RP2040.
  * @note    Corrected mapping based on datasheet:
  *          - UART1 TX -> GPIO4 (for Servo Driver)
  *          - UART1 RX -> GPIO9 (for ESP)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include <string.h>

#include "usart.h" 

/* External Variables & Functions --------------------------------------------*/
// String_parsing 现在将由 uart1 的中断调用
extern unsigned char String_parsing(uint8_t *buf, uint32_t len);
extern volatile unsigned char ESP_RX_Result;

// REG_FeedBytes 和 REG_RX_Result 相关的逻辑需要重新评估，因为没有双向通信
// extern unsigned char REG_FeedBytes(uint8_t *buf, uint32_t len);
// extern volatile unsigned char REG_RX_Result;

/* Private Defines (Corrected Mapping) ---------------------------------------*/

// --- 所有通信都由 uart1 处理 ---
#define ASYMMETRIC_UART_ID   uart1
#define SERVO_TX_PIN         4  // UART1 TX
#define ESP_RX_PIN           9  // UART1 RX

#define ESP_RX_BUF_SIZE     128 

/* Private Variables ---------------------------------------------------------*/
static uint8_t esp_rx_buffer[ESP_RX_BUF_SIZE];
static uint32_t esp_rx_pos = 0;

/* Private Function Prototypes -----------------------------------------------*/
void on_uart1_rx_irq(void);

/* Public Functions ----------------------------------------------------------*/

/**
 * @brief 主UART初始化函数。
 *        现在它只初始化一个UART实例 (uart1)，但将其TX和RX映射到不同的引脚。
 */
void RP_UARTS_Init(void) {
    // 1. 初始化 uart1 实例，并设置一个通用的波特率（以最高者为准）
    uart_init(ASYMMETRIC_UART_ID, 460800);

    // 2. 分别设置TX和RX引脚的功能
    gpio_set_function(SERVO_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(ESP_RX_PIN, GPIO_FUNC_UART);

    // 3. 设置UART的帧格式 (8-N-1)
    uart_set_format(ASYMMETRIC_UART_ID, 8, 1, UART_PARITY_NONE);

    // 4. 关闭硬件流控
    uart_set_hw_flow(ASYMMETRIC_UART_ID, false, false);

    // 5. 使能FIFO
    uart_set_fifo_enabled(ASYMMETRIC_UART_ID, true);

    // 6. 配置并使能【只针对接收】的中断
    // UART1_IRQ 是 uart1 对应的中断号
    irq_set_exclusive_handler(UART1_IRQ, on_uart1_rx_irq);
    irq_set_enabled(UART1_IRQ, true);

    // 只使能 uart1 的“接收”中断，不使能“发送”中断
    uart_set_irq_enables(ASYMMETRIC_UART_ID, true, false);
}


/* Interrupt Service Routine -------------------------------------------------*/

/**
  * @brief  UART1的中断服务程序。
  * @note   此函数只在从ESP (GPIO9) 接收到数据时被触发。
  */
void on_uart1_rx_irq(void) {
    // 循环读取所有可用的字符
    while (uart_is_readable(ASYMMETRIC_UART_ID)) {
        uint8_t ch = uart_getc(ASYMMETRIC_UART_ID);

        if (esp_rx_pos < ESP_RX_BUF_SIZE) {
            esp_rx_buffer[esp_rx_pos++] = ch;
        }

        // 检查包尾，并调用解析函数
        if (ch == '@' && esp_rx_pos > 1) {
            if (String_parsing(esp_rx_buffer, esp_rx_pos)) {
                ESP_RX_Result = 1;
            }
            esp_rx_pos = 0; // 重置缓冲区
        }
        
        // 缓冲区溢出保护
        if (esp_rx_pos >= ESP_RX_BUF_SIZE) {
            esp_rx_pos = 0; 
        }
    }
}