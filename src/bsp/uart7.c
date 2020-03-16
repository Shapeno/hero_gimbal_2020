#include "uart7.h"

#ifdef USE_UART7_TO_REPORT	//要使用请在main.h进行修改
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
    while ((UART7->SR & 0X40) == 0);
    UART7->DR = (u8)ch;
    return ch;
}
#endif

void UART7_Init(u32 bound){
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_UART8);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_UART8);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE,&GPIO_InitStructure);
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode =USART_Mode_Tx|USART_Mode_Rx;
    USART_Init(UART7, &USART_InitStructure);
    USART_Cmd(UART7, ENABLE);
#if EN_UART7_RX    
    USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

#if EN_UART7_RX 
u8 UART_RX_BUF_7[USART_REC_LEN];
u16 UART_RX_STA_7=0;

void UART7_IRQHandler(void){
    u8 Res;  
    if(USART_GetITStatus(UART7, USART_IT_RXNE) != RESET){
        Res =USART_ReceiveData(UART7);
        if((UART_RX_STA_7&0x8000)==0){
            if(UART_RX_STA_7&0x4000){
                if(Res!=0x0a)UART_RX_STA_7=0;
                else UART_RX_STA_7|=0x8000;
            }
            else{    
                if(Res==0x0d)
                    UART_RX_STA_7|=0x4000;
                else{
                    UART_RX_BUF_7[UART_RX_STA_7&0X3FFF]=Res ;
                    UART_RX_STA_7++;
                    if(UART_RX_STA_7>(USART_REC_LEN-1))
                        UART_RX_STA_7=0; 
                }         
            }
        }
    }
}
#endif

void UART7_SendChar(unsigned char b){
    while(USART_GetFlagStatus(UART7,USART_FLAG_TXE)== RESET)
		;
	USART_SendData(UART7,b);
}
