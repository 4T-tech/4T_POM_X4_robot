#ifndef __MY_UART_H__
#define __MY_UART_H__

void UART_Rx_Task(void *arg);
void uart_init(void);
void uart_send_bytes(uint8_t *s, int len);

#endif  // __MY_UART_H__