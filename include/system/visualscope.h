#ifndef __VISUALSCOPE_H__
#define __VISUALSCOPE_H__

#include "stm32f4xx.h"

void RS232_VisualScope(USART_TypeDef* USARTx, u8*, u16);
void VisualScope(USART_TypeDef* USARTx,int16_t CH1,int16_t CH2,int16_t CH3,int16_t CH4);

#endif
