/*
***************************************************************
*******USART2�Ӳ���ϵͳ�������������ò�Ҫ���û��ӿ�**********
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


