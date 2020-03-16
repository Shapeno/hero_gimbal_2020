#ifndef _USART8_H_
#define _USART8_H_

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"
#include "main.h"

#define USART_REC_LEN  200
#define EN_UART8_RX      1

#ifdef USE_UART8_TO_REPORT
#define USART_RX_BUF UART_RX_BUF_8
#define USART_RX_STA UART_RX_STA_8
#endif

extern u8  UART_RX_BUF_8[USART_REC_LEN];
extern u16 UART_RX_STA_8;

void UART8_Init(u32 bound);
void UART8_SendChar(unsigned char b);

#endif
