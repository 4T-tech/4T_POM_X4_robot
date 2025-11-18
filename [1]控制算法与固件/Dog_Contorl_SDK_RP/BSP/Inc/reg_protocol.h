#ifndef REG_PROTOCOL_H
#define REG_PROTOCOL_H

#include <stdint.h>

#define REG_LEN  256

extern uint16_t REG_ADDR[REG_LEN];

unsigned char REG_FeedBytes(uint8_t *buf, uint32_t len );  // 接收端一次性喂数据
uint16_t REG_Read(uint16_t addr);                  // 主循环查询
void send_str(char *s);

void Send_BLE(char *s);
unsigned char String_parsing(uint8_t *buf, uint32_t len );

void REG_6Byte_Send(uint16_t Addr, uint16_t Data);

void Send_Frame_UART2(uint8_t *frame);
#endif