#include "visualscope.h"

#define CHECK_CRC_16

#ifdef CHECK_CRC_16
static u16 RS232_VisualScope_CRC16( u8 *Array, u16 Len ){
  u16 USART_IX, USART_IY, USART_CRC;
  USART_CRC = 0xffff;
  for(USART_IX=0; USART_IX<Len; USART_IX++){
    USART_CRC = USART_CRC^(u16)(Array[USART_IX]);
    for(USART_IY=0; USART_IY<=7; USART_IY++){
      if((USART_CRC&1)!=0)
        USART_CRC = (USART_CRC>>1)^0xA001;
      else
        USART_CRC = USART_CRC>>1;
    }
  }
  return(USART_CRC);
}
#endif

#ifdef CHECK_SUM
static u8 RS232_VisualScope_CHKSUM(u8 *Array,u16 Len){
  u8 sum=0;
  u8 i=0;
  for(i=0; i<Len; i++)
    sum+=Array[i];
  return sum;
}
#endif

void VisualScope(USART_TypeDef* USARTx,int16_t CH1,int16_t CH2,int16_t CH3,int16_t CH4){
  u8 i = 0;
  uint8_t Buffer[10];
  uint16_t Temp=0;
  Buffer[0]=CH1&0xff;
  Buffer[1]=CH1>>8;
  Buffer[2]=CH2&0xff;;
  Buffer[3]=CH2>>8;
  Buffer[4]=CH3&0xff;;
  Buffer[5]=CH3>>8;
  Buffer[6]=CH4&0xff;;
  Buffer[7]=CH4>>8;
#ifdef CHECK_CRC_16
  Temp = RS232_VisualScope_CRC16(Buffer, 8);
  Buffer[8] = Temp&0x00ff;
  Buffer[9] = (Temp&0xff00)>>8;
  for(i=0; i<10; i++){
    USART_SendData(USARTx, Buffer[i]);
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
	    ;
  }
#endif
#ifdef CHECK_SUM
  Temp = RS232_VisualScope_CHKSUM(Buffer, 8);
  Buffer[8] = Temp&0x00ff;
  for(i=0; i<9; i++){
    USART_SendData(USARTx, Buffer[i]);
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
			;
  }
#endif
}

void RS232_VisualScope( USART_TypeDef* USARTx, u8 *pWord, u16 Len ){
  u8 i = 0;
  u16 Temp = 0;
#ifdef CHECK_CRC_16
  Temp = RS232_VisualScope_CRC16(pWord, Len);
  pWord[8] = Temp&0x00ff;
  pWord[9] = (Temp&0xff00)>>8;
  for(i=0; i<10; i++){
    USART_SendData(USARTx, (uint8_t)*pWord);
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
	    ;
    pWord++;
  }
#endif
#ifdef CHECK_SUM
  Temp = RS232_VisualScope_CHKSUM(pWord, Len);
  pWord[8] = Temp&0xff;
  for(i=0; i<9; i++){
    USART_SendData(USARTx, (uint8_t)*pWord);
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
			;
    pWord++;
  }
#endif
}
