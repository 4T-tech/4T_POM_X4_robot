#include "include.h"

#define PARSER_BUF_SIZE 128
static uint8_t parser_buf[PARSER_BUF_SIZE];
static uint32_t parser_len = 0;
static bool in_string_frame = false;

extern uint16_t ADDR;

/**
 * @brief 解析收到的单个字节，并分发给不同的解析器
 * @param b 收到的字节
 */
static void parse_byte(uint8_t b)
{
    // 优先处理 '@...@' 格式的字符串命令
    if (b == '@') {
        if (!in_string_frame) {
            // 帧开始
            in_string_frame = true;
            parser_len = 0;
            parser_buf[parser_len++] = b;
        } else {
            // 帧结束
            if (parser_len < PARSER_BUF_SIZE - 1) {
                parser_buf[parser_len++] = b;
            }
            // 调用你的字符串解析函数
            String_parsing(parser_buf, parser_len);
            in_string_frame = false;
            parser_len = 0;
        }
    } else if (in_string_frame) {
        // 在字符串帧内部，缓存字符
        if (parser_len < PARSER_BUF_SIZE - 1) {
            parser_buf[parser_len++] = b;
        } else {
            // 缓冲区溢出，丢弃此帧
            in_string_frame = false;
            parser_len = 0;
            ESP_LOGW("PARSER", "String command too long, dropped.");
        }
    } else {
        // 如果不在字符串帧内，则认为是二进制数据，送入6字节解析器
        REG_FeedBytes(&b, 1);
    }

    // 读取电压和温度
    if (ADDR == 43)
        REG_6Byte_Send(79 , 43);
    else if (ADDR == 44)
        REG_6Byte_Send(79 , 44);                // 读取温度
}

/**
 * @brief 任务：通过原生USB(CDC)循环读取数据并调用解析函数
 * @details 该任务持续运行，从USB虚拟串口读取数据，并逐字节调用解析函数进行处理。
 *          并且每次接收到一个控制命令后，通过串口发送读取电压和温度的指令。
 */
void usb_rx_task(void *arg)
{
    ESP_LOGI("MAIN", "USB receive task started.");
    uint8_t temp_buf[64]; // 临时缓冲区，用于从驱动读取数据

    // **重要：在使用读写函数前，必须先安装驱动**
    usb_serial_jtag_driver_config_t usb_cfg = {
        .tx_buffer_size = 256,  // set desired TX buffer size
        .rx_buffer_size = 256   // set desired RX buffer size
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    while (1) {
        // 从USB虚拟串口读取数据，超时时间为20ms
        int len = usb_serial_jtag_read_bytes(temp_buf, sizeof(temp_buf), pdMS_TO_TICKS(20));
        if (len > 0) {
            // 逐个字节送去解析
            for (int i = 0; i < len; ++i) {
                parse_byte(temp_buf[i]);
            }
        }
    }
}