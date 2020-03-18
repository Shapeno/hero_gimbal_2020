#include "usart3.h"

#if SYSTEM_SUPPORT_OS
#include "freeRTOS.h"
#endif

#ifdef USE_USART3_TO_REPORT		//要使用请在main.h进行修改
#pragma import(__use_no_semihosting)
struct __FILE {
    int handle;
};
void _ttywrch(int ch) 
{ 
ch = ch; 
} 
FILE __stdout;
void _sys_exit(int x) {
    x = x;
}

int fputc(int ch, FILE *f) {
    while ((USART3->SR & 0X40) == 0);
    USART3->DR = (u8)ch;
    return ch;
}
#endif


void USART3_Init(u32 bound){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
#if EN_USART3_RX	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

#if EN_USART3_RX	
u8 USART_RX_BUF_3[USART_REC_LEN];
u16 USART_RX_STA_3 = 0;

void USART3_IRQHandler(void){
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
		Res =USART_ReceiveData(USART3);
		if((USART_RX_STA_3&0x8000)==0){
			if(USART_RX_STA_3&0x4000){
				if(Res!=0x0a)USART_RX_STA_3=0;
				else USART_RX_STA_3|=0x8000;
			}
			else{	
				if(Res==0x0d)
					USART_RX_STA_3|=0x4000;
				else{
					USART_RX_BUF_3[USART_RX_STA_3&0X3FFF]=Res ;
					USART_RX_STA_3++;
					if(USART_RX_STA_3>(USART_REC_LEN-1))
						USART_RX_STA_3=0; 
				}		 
			}
		}
	}
}
#endif	

void USART3_SendChar(unsigned char b){
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)== RESET)
		;
	USART_SendData(USART3,b);
}
