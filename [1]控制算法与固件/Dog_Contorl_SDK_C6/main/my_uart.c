/* 用户串口的驱动文件 */
#include "include.h"

#define RX_BUF_SIZE         (1024)          // UART 接收缓冲区大小
#define USER_UART_NUM       UART_NUM_1      // 使用 UART1
#define TXD_PIN             GPIO_NUM_20     // UART1 TX 引脚
#define RXD_PIN             GPIO_NUM_19     // UART1 RX 引脚
#define USER_UART_BAUDRATE  460800          // UART 波特率
#define FRAME_LEN           6               // 帧长度        

static uint8_t uart_rx_buff[RX_BUF_SIZE];   // UART 接收缓冲区
static uint8_t command1[6] = {0xAA,0x00,0x01,0x00,0x2C,0xD7};          
static uint8_t command2[6] = {0xAA,0x00,0x01,0x00,0x5F,0x0A}; 

extern uint16_t ADDR;

/*
    GPIO20 ----> TXD
    GPIO19 ----> RXD
*/

/**
 * @brief 解析UART连续字节流，提取固定长度6字节、0x55为帧头的数据帧。
 * 帧格式: [0]=0x55 | [1]=Addr_H | [2]=Addr_L | [3]=Data_H | [4]=Data_L | [5]=Sum(前5字节累加8位)
 * @param data 输入的字节缓冲指针
 * @param len  本次输入的字节数
 */
static void parse_stream(const uint8_t *data, int len){
    static uint8_t frame[FRAME_LEN];
    static int idx = 0;

    for(int i=0;i<len;i++){
        uint8_t b = data[i];

        if(idx == 0){                 // 等待帧头
            if(b == 0x55){
                frame[0] = b;
                idx = 1;
            }
            continue;
        } else {
            frame[idx++] = b;
            if(idx == FRAME_LEN){
                // 校验
                uint8_t sum = 0;
                for(int k=0;k<FRAME_LEN-1;k++) sum += frame[k];
                if(frame[0]==0x55 && sum == frame[5]){
                    uint16_t addr = ((uint16_t)frame[1] << 8) | frame[2];
                    uint16_t data16 = ((uint16_t)frame[3] << 8) | frame[4];
                    REG_ADDR[addr] = data16;
                    // 可加 ESP_LOGD("UART","addr=%u data=%u", addr, data16);
                }
                // 重新同步：当前字节已用完，重置；若最后字节本身也是0x55，可当作新帧开端
                uint8_t last = frame[FRAME_LEN-1];
                idx = (last == 0x55) ? 1 : 0;
                if(idx == 1) frame[0] = 0x55;
            }
        }
    }
    if (ADDR == 43)
        Read_Battery_Volt();                    // 读取电池电压
    else if (ADDR == 44)
        Read_Temp();                            // 读取温度
    ADDR = 0;
}

/**
 * @brief 解析UART连续字节流，提取固定长度6字节、0x55为帧头的数据帧。
 * @details 当解析出数据帧后会更新寄存器数组REG_ADDR，并且通过CDC发送电压或者温度读取数据。
 * 帧格式: [0]=0x55 | [1]=Addr_H | [2]=Addr_L | [3]=Data_H | [4]=Data_L | [5]=Sum(前5字节累加8位)
 */
void UART_Rx_Task(void *arg){
    while(1){
        int rec = uart_read_bytes(USER_UART_NUM, uart_rx_buff, sizeof(uart_rx_buff),pdMS_TO_TICKS(20)); // 较短超时提高响应
        if(rec > 0){
            parse_stream(uart_rx_buff, rec);
        }
    }
}

// UART 初始化
void uart_init(void){

    uart_config_t uart_config = {
        .baud_rate = USER_UART_BAUDRATE,                // 波特率
        .data_bits = UART_DATA_8_BITS,                  // 数据位
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,          // 硬件流控
        .parity = UART_PARITY_DISABLE,                  // 无奇偶校验
        .stop_bits = UART_STOP_BITS_1,                  // 1 位停止位
        .source_clk = UART_SCLK_DEFAULT,                // 默认时钟
    };
    uart_param_config(UART_NUM_1, &uart_config);            // 配置 UART1 参数
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE, 0, 0, NULL, 0);   // 安装 UART1 驱动
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, -1, -1);     // 设置 UART1 引脚 (-1 表示不改变 RTS 和 CTS 引脚)
}

// UART 发送数据
void uart_send_bytes(uint8_t *s, int len){
    uart_write_bytes(USER_UART_NUM, s, len); // 发送数据
}