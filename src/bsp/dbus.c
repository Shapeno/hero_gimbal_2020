#include "dbus.h"
/*****************************************************************************
*  @file     dbus.c                                                       	 *
*  @brief    用于对遥控器数据解码                                            *
*  包含了遥控器数据处理的初始化函数，遥控器信息获取函数，遥控器上一次信息的	 *
*  获取函数                                                                  *
*                                                                            *
*  @author   DENG		                                                     *
*  @version  1.0.1.1		                                                 *
*  @date     19/9/6				                                             *
*                                                                            *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2019/09/6  | 1.0.0.1   | DengXY	       | Create file                     *
*----------------------------------------------------------------------------*
*  2019/09/10 | 1.0.0.2   | DengXY	       | Function test                   *
*----------------------------------------------------------------------------*
*  2019/09/11 | 1.0.1.1   | DengXY	       | Function test                   *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/
#include <string.h>
#include "delay.h"
#include "sys.h"

static uint8_t DBUS_rx_buf[2][DBUS_RX_BUF_NUM];
static RC_Data_t	rc_data;
static RC_Data_t	rc_data_last;
static portTickType remote_data_time;
	portTickType GetRemoteDataTime(void){return remote_data_time;}

//------------------------------------------------------------
//用户函数
//------------------------------------------------------------

/**	
*@brief	遥控器信息获取初始化函数
*/
void Dbus_Init(void){
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); //PB7  usart1 rx
	/* -------------- Configure GPIO ---------------------------------------*/
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		USART_DeInit(USART1);

		USART_InitStructure.USART_BaudRate = 100000;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_Even;
		USART_InitStructure.USART_Mode = USART_Mode_Rx;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART1, &USART_InitStructure);

		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

		USART_ClearFlag(USART1, USART_FLAG_IDLE);
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

		USART_Cmd(USART1, ENABLE);
	}

	/* -------------- Configure NVIC ---------------------------------------*/
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}

	//DMA2 stream5 ch4  or (DMA2 stream2 ch4)    !!!!!!! P206 of the datasheet
	/* -------------- Configure DMA -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA2_Stream2);

		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DBUS_rx_buf[0];
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = 36;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA2_Stream2, &DMA_InitStructure);
		DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)DBUS_rx_buf[1], DMA_Memory_0);
		DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
		DMA_Cmd(DMA2_Stream2, DISABLE); //Add a disable
		DMA_Cmd(DMA2_Stream2, ENABLE);
	}
}

/**	
*@brief	获取遥控器控制信息
*@return RC_Data_t
*/
RC_Data_t GetRcData(void){
	return rc_data;
}

/**	
*@brief	获取遥控器上一次控制信息
*@return RC_Data_t
*/
RC_Data_t GetLastRcData(void){
	return rc_data_last;
}



//------------------------------------------------------------
//非用户函数
//------------------------------------------------------------
void remote_data_encode(uint8_t *pData);
void remote_swich_encode(Sw_Data_t *SW,uint8_t sw);


void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART1);
    }
    else if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART1);

        if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream2, DISABLE);
            this_time_rx_len = DBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream2);
            DMA_SetCurrDataCounter(DMA2_Stream2, DBUS_RX_BUF_NUM);
            DMA2_Stream2->CR |= DMA_SxCR_CT;
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA2_Stream2, ENABLE);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                remote_data_encode(DBUS_rx_buf[0]);
            }
        }
        else
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream2, DISABLE);
            this_time_rx_len = DBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream2);
            DMA_SetCurrDataCounter(DMA2_Stream2, DBUS_RX_BUF_NUM);
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA2_Stream2, ENABLE);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                remote_data_encode(DBUS_rx_buf[1]);
            }

        }
    }
}

void remote_data_encode(uint8_t *pData){
	rc_data_last=rc_data;
	uint8_t s1,s2;
	rc_data.rc.ch[0]  = (pData[0] | (pData[1]<<8)) & 0x07FF; 
	rc_data.rc.ch[1]  = ((pData[1]>>3) | (pData[2]<<5)) & 0x07FF;
	rc_data.rc.ch[2]  = ((pData[2]>>6) | (pData[3]<<2) |(pData[4] << 10)) & 0x07FF
						;
	rc_data.rc.ch[3]  = ((pData[4]>>1) | (pData[5]<<7)) & 0x07FF;
	s1   = ((pData[5] >> 4) & 0x000C)>>2;
	s2   = ((pData[5] >> 4) & 0x0003);
	remote_swich_encode(&rc_data.rc.s1,s1);
	remote_swich_encode(&rc_data.rc.s2,s2);
	memcpy(&rc_data.mouse, &pData[6], sizeof(Mouse_Data_t));
	memcpy(&rc_data.key,&pData[14],sizeof(Key_Data_t));
	rc_data.rc.ch[4]  =	pData[16]|(pData[17]<<8);
	
	rc_data.rc.ch[0] -= REMOTE_CONTROLLER_OFFSET;
	rc_data.rc.ch[1] -= REMOTE_CONTROLLER_OFFSET;
	rc_data.rc.ch[2] -= REMOTE_CONTROLLER_OFFSET;
	rc_data.rc.ch[3] -= REMOTE_CONTROLLER_OFFSET;
	rc_data.rc.ch[4] -= REMOTE_CONTROLLER_OFFSET;
	
	remote_data_time=xTaskGetTickCount();
}

void remote_swich_encode(Sw_Data_t *SW,uint8_t sw){
	switch (sw)
	{
		case 1:{
			SW->up=1;
			SW->mid=0;
			SW->down=0;
		}break;
		case 3:{
			SW->up=0;
			SW->mid=1;
			SW->down=0;
		}break;
		case 2:{
			SW->up=0;
			SW->mid=0;
			SW->down=1;
		}break;
	}
}


