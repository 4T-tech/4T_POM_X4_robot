/**
  ******************************************************************************
  * @file    reg_protocol.c
  * @brief   Communication protocol handling for RP2040 platform.
  * @note    This file is a port of the G4 version. All HAL calls have been
  *          replaced with their Pico-SDK equivalents.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <string.h>
#include <stdio.h>

#include "reg_protocol.h"
#include "leg_ik.h"
#include "Dog_Sport.h"

/* Defines from usart.c - for identifying UART instances */
#define UART_REG_INSTANCE   uart1
//#define UART_ESP_INSTANCE   uart1

/* Global Variables ----------------------------------------------------------*/
uint16_t REG_ADDR[REG_LEN] = {0};
static uint8_t  rx6[6] = {0};
static uint8_t  pos = 0;

// TX State flag - used to prevent re-entrant transmission calls.
// Using volatile as it might be accessed from different contexts in the future.
volatile int8_t UART_REG_TX_State = 0;

uint16_t ADDR = 0;




/* Helper Functions for Sending Data -----------------------------------------*/

/**
 * @brief Sends a string for debugging purposes.
 * @note  Replaces CDC_Transmit_FS. It now uses printf, which is redirected
 *        to the default STDIO UART on the RP2040.
 *        WARNING: This might conflict with the UART used for motor control (uart0).
 *                 Disable if communication issues arise.
 */
void send_str(char *s)
{
    // CDC_Transmit_FS((uint8_t *)s, strlen(s));
   // printf("%s", s);
}

/**
 * @brief Sends a string to the ESP32 (via Bluetooth Low Energy).
 * @note  Replaces HAL_UART_Transmit with uart_puts for the ESP UART.
 */
void Send_BLE(char *s)
{
    // HAL_UART_Transmit(&huart4,(uint8_t *)s,strlen(s),5);
//    uart_puts(UART_ESP_INSTANCE, s);
}

/* Data Parsing Functions (Platform Independent - No changes needed) ---------*/

/**
 * @brief Feeds bytes into the 6-byte register protocol parser.
 * @note  This function is platform-independent and requires no changes.
 */
unsigned char REG_FeedBytes(uint8_t *buf, uint32_t len)
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
                if (addr < REG_LEN) {
                    REG_ADDR[addr] = data;
                }
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
 * @brief Parses string commands in the format "@<char>=<float>@".
 * @note  This function is platform-independent and requires no changes.
 */
unsigned char String_parsing(uint8_t *buf, uint32_t len)
{
	
	   if (len > 0 && buf[len - 1] == '\0') {
        // 操作：将有效长度减一，从而在后续的逻辑中忽略这个空字符
        len--;
    }
    // ... (函数体与G4版本完全相同，无需任何修改) ...
    unsigned char Parsing_Result = 0;
    if (len == 0 || buf[0] != '@') {
        // send_str("ERROR: Invalid header\r\n"); // 可选地禁用调试输出
        return Parsing_Result;
    }
    if (len < 5 || buf[len - 1] != '@') {   // 最短：@x=0@
        // send_str("ERROR: No tail\r\n");
        return Parsing_Result;
    }

    char tmp[32] = {0};
    uint32_t content_len = len - 2;
    if (content_len >= sizeof(tmp)) {
        // send_str("ERROR: Too long\r\n");
        return Parsing_Result;
    }
    memcpy(tmp, buf + 1, content_len);
    tmp[content_len] = '\0';

    char axis;
    float value;
    if (sscanf(tmp, "%c=%f", &axis, &value) != 2) {
        // send_str("ERROR: Format\r\n");
        return Parsing_Result;
    }
    
    switch (axis) {
        case 'x': case 'X': DogCtrl.X_data = value; Parsing_Result = 1; break;
        case 'y': case 'Y': DogCtrl.Y_data = value; Parsing_Result = 1; break;
        case 'z': case 'Z': DogCtrl.Z_data = value; Parsing_Result = 1; break;
        case 'r': case 'R': DogCtrl.Roll_data = value; Parsing_Result = 1; break;
        case 'p': case 'P': DogCtrl.Pitch_data = value; Parsing_Result = 1; break;
        case 'w': case 'W': DogCtrl.Yaw_data = value; Parsing_Result = 1; break;
        case 's': case 'S': DogCtrl.Stride_Length = value; Parsing_Result = 1; break;
        case 'l': case 'L': DogCtrl.Side_Length = value; Parsing_Result = 1; break;
        case 't': case 'T': DogCtrl.Turn_Speed = value; Parsing_Result = 1; break;
        case 'h': case 'H': DogCtrl.Raise_Leg_Height = value; Parsing_Result = 1; break;
        case 'a': case 'A': ADDR = (uint16_t)value; Parsing_Result = 1; break;
        case 'm': case 'M': DogCtrl.Gait_Mode = (uint16_t)value; Parsing_Result = 1; break;
        case 'v': case 'V': DogCtrl.Motion_Speed = value; Parsing_Result = 1; break;
        default: /* send_str("ERROR: Unknown axis\r\n"); */ return Parsing_Result;
    }
    
    // char echo[24];
    // sprintf(echo, "OK: %c=%.2f\r\n", axis, value);
    // send_str(echo);
    // Send_BLE(echo);
    return Parsing_Result;   // 成功
}

/* Register Access and Sending Functions -------------------------------------*/

uint16_t REG_Read(uint16_t addr)
{
    return (addr < REG_LEN) ? REG_ADDR[addr] : 0;
}


/**
 * @brief Sends a 6-byte command frame to the motor driver board.
 * @note  Replaces HAL_UART_Transmit with uart_write_blocking.
 */
void REG_6Byte_Send(uint16_t Addr, uint16_t Data)
{
    if (UART_REG_TX_State == 0)
    {
        UART_REG_TX_State = 1;
        uint8_t frame[6] = {
            0xAA,
            (Addr >> 8) & 0xFF,
            Addr & 0xFF,
            (Data >> 8) & 0xFF,
            Data & 0xFF,
            0   // Checksum placeholder
        };

        uint16_t sum = 0;
        for (int i = 0; i < 5; ++i) sum += frame[i];
        frame[5] = (uint8_t)(sum & 0xFF);

        // HAL_UART_Transmit(&huart2, frame, 6, 5);
        uart_write_blocking(UART_REG_INSTANCE, frame, 6);
        
        UART_REG_TX_State = 0;
    }
}


void Send_Frame_UART2(uint8_t *frame)
{
    if (UART_REG_TX_State == 0)
    {
        UART_REG_TX_State = 1;
        
        // HAL_UART_Transmit(&huart2, frame, 65, 5);
        uart_write_blocking(UART_REG_INSTANCE, frame, 65);
        
        UART_REG_TX_State = 0;
    }
}