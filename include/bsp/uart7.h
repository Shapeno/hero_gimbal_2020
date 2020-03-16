#ifndef _UART8_H_
#define _UART8_H_

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"
#include "main.h"

#define USART_REC_LEN  200
#define EN_UART7_RX      1

#ifdef USE_UART7_TO_REPORT
#define USART_RX_BUF UART_RX_BUF_7
#define USART_RX_STA UART_RX_STA_7
#endif

extern u8  UART_RX_BUF_7[USART_REC_LEN];
extern u16 UART_RX_STA_7;

void UART7_Init(u32 bound);
void UART7_SendChar(unsigned char b);

#endif
