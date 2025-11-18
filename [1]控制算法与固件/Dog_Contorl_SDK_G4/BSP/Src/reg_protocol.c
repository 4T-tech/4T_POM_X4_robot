#include "reg_protocol.h"
#include <string.h>
#include "leg_ik.h"
#include "usart.h"
#include "Dog_Sport.h"
#include "usbd_cdc_if.h"

#define MAX_FIELD_LEN 32  // 增大字段长度限制

uint16_t REG_ADDR[REG_LEN] = {0}; 
static uint8_t  rx6[6] = {0};
static uint8_t  pos = 0;

extern int8_t UART2_TX_State;

uint16_t ADDR = 0;

float Stand_Data[6] = {0,0,0,0,0,160.0};
uint16_t Motion_Data[6] = {0};

extern DogCtrl_t DogCtrl;// 运动控制结构体

void send_str(char *s)
{
    CDC_Transmit_FS((uint8_t *)s, strlen(s));
}

void Send_BLE(char *s)
{
    HAL_UART_Transmit(&huart4,(uint8_t *)s,strlen(s),5);  
}


/*--------------------接收六字节数据解析函数------------------*/
/*
    * 解析格式 : 帧头0x55 + 地址 + 数据 + 校验和 的6字节数据
 * 返回值：1-成功， 0-失败 ,2-解析错误
 */
unsigned char REG_FeedBytes(uint8_t *buf, uint32_t len )
{

    for (uint32_t i = 0; i < len; ++i)
    {
        rx6[pos++] = buf[i];
        if (pos == 6)               
        {
            pos = 0;
            uint8_t sum = rx6[0] + rx6[1] + rx6[2] + rx6[3] + rx6[4];
            if (rx6[0] == 0x55 && sum == rx6[5])
            {
                uint16_t addr = (rx6[1] << 8) | rx6[2];
                uint16_t data = (rx6[3] << 8) | rx6[4];
                REG_ADDR[addr] = data;
                return 1;
            }
            else
            {                     
                return 0;
            }
        }
    }
    return 0;
}

/**
 * 解析格式：@<字母>=<浮点数>@
 * 字母：x y z r p w → 对应全局变量
 * 返回值：1-成功，0-格式错误
 */
unsigned char String_parsing(uint8_t *buf, uint32_t len)
{
	send_str(buf);
    unsigned char Parsing_Result = 0;
		if (len > 0 && buf[len - 1] == '\0') {   // 操作：将有效长度减一，从而在后续的逻辑中忽略这个空字符
        
        len--;
    }
    if (len == 0 || buf[0] != '@') {
        send_str("ERROR: Invalid header\r\n");
        return Parsing_Result;
    }
    if (len < 5 || buf[len - 1] != '@') {   // 最短：@x=0@
        send_str("ERROR: No tail\r\n");
        return Parsing_Result;
    }

    /* 把中间部分变成 C 字符串以便 sscanf 用 */
    char tmp[32] = {0};
    uint32_t content_len = len - 2;          // 去掉两端的 '@'
    if (content_len >= sizeof(tmp)) {
        send_str("ERROR: Too long\r\n");
        return Parsing_Result;
    }
    memcpy(tmp, buf + 1, content_len);
    tmp[content_len] = '\0';

    /* 解析：首字符 + '=' + 浮点数 */
    char axis;
    float value;
    if (sscanf(tmp, "%c=%f", &axis, &value) != 2) {
        send_str("ERROR: Format\r\n");
        return Parsing_Result;
    }

    /* 根据字母写入对应全局变量 */
    switch (axis) {
        case 'x': case 'X': 
            DogCtrl.X_data   = value; 
            Parsing_Result = 1;
        break;
        case 'y': case 'Y': 
            DogCtrl.Y_data   = value; 
            Parsing_Result = 1;
        break;
        case 'z': case 'Z': 
            DogCtrl.Z_data   = value; 
            Parsing_Result = 1;
        break;
        case 'r': case 'R': 
            DogCtrl.Roll_data  = value; 
            Parsing_Result = 1;
        break;
        case 'p': case 'P': 
            DogCtrl.Pitch_data = value;
            Parsing_Result = 1;
        break;
        case 'w': case 'W': 
            DogCtrl.Yaw_data = value; 
            Parsing_Result = 1;
        break;
        case 's': case 'S': 
            DogCtrl.Stride_Length  = value; 
            Parsing_Result = 1;
        break;
        case 'l': case 'L': 
            DogCtrl.Side_Length  = value; 
            Parsing_Result = 1;
        break;
        case 't': case 'T': 
            DogCtrl.Turn_Speed = value; 
            Parsing_Result = 1;
        break;
            case 'h': case 'H': 
            DogCtrl.Raise_Leg_Height = value; 
            Parsing_Result = 1;
        break;
        case 'a': case 'A': 
            ADDR = (uint16_t)value; 
            Parsing_Result = 1;
        break;
        case 'm': case 'M': 
            DogCtrl.Gait_Mode = (uint16_t)value; 
            Parsing_Result = 1;
        break;
        case 'v': case 'V': 
            DogCtrl.Motion_Speed = value; 
            Parsing_Result = 1;
        break;
        default:
            send_str("ERROR: Unknown axis\r\n");
            return Parsing_Result;
    }

    /* 可选：回显调试 */
    char echo[24];
    sprintf(echo, "OK: %c=%.2f\r\n", axis, value);
    send_str(echo);
    Send_BLE(echo);
    return Parsing_Result;   // 成功
}


uint16_t REG_Read(uint16_t addr)
{
    return (addr < REG_LEN) ? REG_ADDR[addr] : 0;
}


/*---------------6字节数据发送函数--------------- */
/*
    *发送的数据格式 : 帧头0xAA + 地址 + 数据 + 校验和 的6字节数据
    *用于填入地址和数据转化为16进制6字节数据发送给舵机驱动板
 */
void REG_6Byte_Send(uint16_t Addr, uint16_t Data) 
{
 
    if (UART2_TX_State == 0)
    {
        UART2_TX_State = 1;
        uint8_t frame[6] = {
            0xAA,
            (Addr >> 8) & 0xFF,
            Addr & 0xFF,
            (Data >> 8) & 0xFF,
            Data & 0xFF,
            0   // 校验和占位
        };

        uint16_t sum = 0;
        for (int i = 0; i < 5; ++i) sum += frame[i];
        frame[5] = (uint8_t)(sum & 0xFF);

        if (HAL_UART_Transmit(&huart2, frame, 6, 5) != HAL_OK) {
            Send_BLE("6Byte Send ERROR!!\r\n");
            send_str("6Byte Send ERROR!!\r\n");
        }
        UART2_TX_State = 0;
    }
}

void Send_Frame_UART2(uint8_t *frame)
{
    if (UART2_TX_State == 0)
    {
        UART2_TX_State = 1;
        if (HAL_UART_Transmit(&huart2, frame, 65, 5) != HAL_OK)
        {
            Send_BLE("Frame_Send_ERROR\r\n");
            send_str("Frame_Send_ERROR\r\n");
        }
        UART2_TX_State = 0;
    }
}