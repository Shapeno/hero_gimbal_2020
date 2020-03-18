#include "usart6.h"

#include "FIFO.h"
#include "led.h"
#include "command_task.h"

static UpperMonitor_Ctr_t upperMonitorCmd = { 0,NO_CMD,0.0,0.0 };

FIFO_S_t* UART_TranFifo;

void USART6_Init(u32 bound){
  USART_InitTypeDef usart6;
  GPIO_InitTypeDef  gpio;
  NVIC_InitTypeDef  nvic;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
  gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_100MHz;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG,&gpio);
  usart6.USART_BaudRate = bound;
  usart6.USART_WordLength = USART_WordLength_8b;
  usart6.USART_StopBits = USART_StopBits_1;
  usart6.USART_Parity = USART_Parity_No;
  usart6.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
  usart6.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART6,&usart6);
  USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
  USART_Cmd(USART6,ENABLE);
  nvic.NVIC_IRQChannel = USART6_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 0;
  nvic.NVIC_IRQChannelSubPriority = 0;
  nvic.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&nvic);
  UART_TranFifo = FIFO_S_Create(100);
}

void USART6_sendChar(uint8_t ch) {
    while ((USART6->SR & 0X40) == 0);
    USART6->DR = (u8)ch;
}

void USART6_Print(uint8_t* ch, int len) {
    for (int i = 0; i < len; i++) {
        USART6_sendChar((u8)ch[i]); 
    }
}

void USART6_IRQHandler(void){
  if(USART_GetITStatus(USART6, USART_IT_TXE) != RESET){
    if(!FIFO_S_IsEmpty(UART_TranFifo)){
			uint16_t data = (uint16_t)FIFO_S_Get(UART_TranFifo);
			USART_SendData(USART6, data);
    }
    else{
			USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
    }
  }
  else if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET){
    static unsigned char rx_buffer[12];
    static uint8_t receivedBytes = 0;
    uint8_t tmp = USART_ReceiveData(USART6);
    switch(receivedBytes){
			case 0:{
        if(tmp == 0x0a){
          receivedBytes++;
        }
				else{
          receivedBytes = 0;
        }
        break;
      }
      case 1:{
        if(tmp == 0x0d){
          receivedBytes++;
        }
				else{
          receivedBytes = 0;
        }
        break;
      }
      case 2:{
        receivedBytes++;
        rx_buffer[0] = tmp;
        break;
      }
      case 3:{
        receivedBytes++;
        rx_buffer[1] = tmp;
        break;
      }
      case 4:{
        receivedBytes++;
        rx_buffer[2] = tmp;
        break;
      }
      case 5:{
        receivedBytes++;
        rx_buffer[3] = tmp;
        break;
      }
      case 6:{
        receivedBytes++;
        rx_buffer[4] = tmp;
        break;
      }
      case 7:{
        if(tmp == 0x0d){
          receivedBytes++;
        }
				else{
          receivedBytes = 0;
        }
        break;
      }
      case 8:{
        if(tmp == 0x0a){
          UpperMonitorDataProcess(rx_buffer);
//					LED1=~LED1;
        }
        receivedBytes = 0;
        break;
      }
      default:
        receivedBytes = 0;
    }
  }
}

void UpperMonitorDataProcess(uint8_t *pData){
	int16_t d1 = *((int16_t *)(pData + 1));
	int16_t d2 = *((int16_t *)(pData + 3)); 
		switch (pData[0]){
			case GIMBAL_MOVEBY:{
				upperMonitorCmd.d1 = d1 * 0.001f;
				upperMonitorCmd.d2 = d2 * 0.001f;
				upperMonitorCmd.cmdid = GIMBAL_MOVEBY;
			}break;
			case GIMBAL_STOP:{
				upperMonitorCmd.cmdid = GIMBAL_STOP;
			}break;
			case REQUEST_PITCH:{
//				miniPC_ACK_status();			////////
			}break;
			case ACK:{
//            Set_Flag(PC_online);
			}break;
			default:{
			}break;
	  }
}


UpperMonitor_Ctr_t GetUpperMonitorCmd(void){
	return upperMonitorCmd;
}

void ResetUpperMonitorCmd(void){
	upperMonitorCmd.d1 = 0;
	upperMonitorCmd.d2 = 0;
}

