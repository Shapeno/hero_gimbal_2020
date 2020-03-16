/*
***************************************************************
*******USART2接裁判系统，除特殊情况最好不要作用户接口**********
***************************************************************
*/
#ifndef __USART_H
#define __USART_H

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"

void USART2_Init(void);
void USART2_SendChar(uint8_t b);

#endif


