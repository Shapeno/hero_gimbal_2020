#ifndef __USART3_H
#define __USART3_H

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"
#include "main.h"

#define USART_REC_LEN  200
#define EN_USART3_RX 	 1

#ifdef USE_USART3_TO_REPORT
#define USART_RX_BUF USART_RX_BUF_3
#define USART_RX_STA USART_RX_STA_3
#endif

extern u8  USART_RX_BUF_3[USART_REC_LEN];
extern u16 USART_RX_STA_3;

void USART3_Init(u32 bound);
void USART3_SendChar(unsigned char b);

#endif


