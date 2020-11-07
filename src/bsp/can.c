#include "can.h"
/*****************************************************************************
*  @file     can.c                                                       	 *
*  @brief    CANͨ�Ű弶֧��                                                 *
*  ������CANͨ�ŵ����ú�������ʼ������������ͨ�ź���						 *
*  			                                                                 *
*                                                                            *
*  @author   DENG		                                                     *
*  @version  1.0.0.1		                                                 *
*  @date     19/9/8				                                             *
*                                                                            *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2019/09/8  | 1.0.0.1   | DengXY	       | Create file                     *
*----------------------------------------------------------------------------*
*  2019/09/11 | 1.0.0.2   | DengXY	       | Function test                   *
*----------------------------------------------------------------------------*
*  2019/09/13 | 1.0.0.2   | DengXY	       | Function add                    *
*----------------------------------------------------------------------------*
*  2019/09/18 | 1.0.0.3   | DengXY	       | Function test                   *
*----------------------------------------------------------------------------*
*  2020/08/3  | 1.0.0.4   | DengXY	       | renew sendmotorcurrent()        *
*----------------------------------------------------------------------------*
*  2020/08/3  | 1.0.1.0   | DengXY	       | separate motor funtions         *
*----------------------------------------------------------------------------*
*****************************************************************************/
#include "sys.h"
#include <string.h>
#include <stdio.h>
#include "mw_motor.h"

/// @brief �ӵ�������ȡ����ϵͳǹ������
static gun_data_t Gun_Data={0};
gun_data_t Get_Gun_Data(){return Gun_Data;}
/// @brief �ӵ�������ȡ����ϵͳ������״̬����
static robot_status_t Robot_Status={0};
robot_status_t Get_Robot_Status(){return Robot_Status;}

//------------------------------------------------------------
//������غ���
//------------------------------------------------------------

/**
@brief ���͵����ٶ�
@param forward_back_target	ǰ���ٶ�
@param left_right_target	�����ٶ�
@param rotate_target		�����ٶ�
@param chasis_heat			��������
*/
void SendChassisSpeed(CAN_TypeDef *CANx, uint8_t mode, int16_t Vx, int16_t Vy, int16_t W){
    CanTxMsg tx_message;
    tx_message.StdId = Gimbal_ID;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = MoveData;
    tx_message.Data[1] = mode;
    tx_message.Data[2] = (uint8_t)(Vx >> 8);
    tx_message.Data[3] = (uint8_t)Vx;
    tx_message.Data[4] = (uint8_t)(Vy >> 8);
    tx_message.Data[5] = (uint8_t)Vy;
    tx_message.Data[6] = (uint8_t)(W >> 8);
    tx_message.Data[7] = (uint8_t)W;
    CAN_Transmit(CANx,&tx_message);
}

void RecieveChassisData(CanRxMsg * msg)
{
	switch (msg->Data[0])
	{
		case GunData:
		{
			Gun_Data.bulletFreq=msg->Data[1];
			Gun_Data.bulletSpeed=(msg->Data[2]<<24)|(msg->Data[3]<<16)|(msg->Data[4]<<8)|msg->Data[5];
			Gun_Data.shooterHeat=(msg->Data[6]<<8)|msg->Data[7];
		}break;
		case RoboStateData:
		{
			Robot_Status.robot_id=msg->Data[1];
			Robot_Status.robot_level=msg->Data[2];
			Robot_Status.gun_cooling_rate=(msg->Data[3]<<8)|msg->Data[4];
			Robot_Status.gun_cooling_limit=(msg->Data[5]<<8)|msg->Data[6];
			Robot_Status.gun_speed_limit=msg->Data[7];
		}break;
		
	}
	
}

//------------------------------------------------------------
//mw_motor���ͺ����ײ�ʵ��
//------------------------------------------------------------
void CAN_Data_Tx(uint32_t StdId,uint8_t Data[8],Can_Channel_e CAN_X){
	CanTxMsg tx_message;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	tx_message.StdId=StdId;
	memcpy(tx_message.Data,Data,sizeof(tx_message.Data));
	if(CAN_X==CAN_1)
		CAN_Transmit(CAN1,&tx_message);
	else if(CAN_X==CAN_2)
		CAN_Transmit(CAN2,&tx_message);
}

//------------------------------------------------------------
//��ʼ������
//------------------------------------------------------------

/**
@brief CAN1ͨ�ų�ʼ��
*/
void CAN1_Init(void){
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	GPIO_InitTypeDef       gpio;
	NVIC_InitTypeDef       nvic;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOD, &gpio);
	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);    
	CAN_DeInit(CAN1);
	CAN_StructInit(&can);
	can.CAN_TTCM = DISABLE;
	can.CAN_ABOM = DISABLE;
	can.CAN_AWUM = DISABLE;
	can.CAN_NART = DISABLE;
	can.CAN_RFLM = DISABLE;
	can.CAN_TXFP = ENABLE;
	can.CAN_Mode = CAN_Mode_Normal;
	can.CAN_SJW  = CAN_SJW_1tq;
	can.CAN_BS1 = CAN_BS1_9tq;
	can.CAN_BS2 = CAN_BS2_4tq;
	can.CAN_Prescaler = 3;
	CAN_Init(CAN1, &can);
	can_filter.CAN_FilterNumber=0;
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=CAN_FilterFIFO0;
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}

/**
@brief CAN2ͨ�ų�ʼ��
*/
void CAN2_Init(void){
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	GPIO_InitTypeDef       gpio;
	NVIC_InitTypeDef       nvic;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1|RCC_APB1Periph_CAN2, ENABLE);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 
	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &gpio);
	nvic.NVIC_IRQChannel = CAN2_RX1_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	CAN_DeInit(CAN2);
	CAN_StructInit(&can);
	can.CAN_TTCM = DISABLE;
	can.CAN_ABOM = DISABLE;    
	can.CAN_AWUM = DISABLE;    
	can.CAN_NART = DISABLE;    
	can.CAN_RFLM = DISABLE;    
	can.CAN_TXFP = ENABLE;     
	can.CAN_Mode = CAN_Mode_Normal; 
	can.CAN_SJW  = CAN_SJW_1tq;
	can.CAN_BS1 = CAN_BS1_9tq;
	can.CAN_BS2 = CAN_BS2_4tq;
	can.CAN_Prescaler = 3;
	CAN_Init(CAN2, &can);
	
	can_filter.CAN_FilterNumber=14;	//CAN1��CAN2����28����������CAN1��0~13
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=CAN_FilterFIFO1;	
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}

//------------------------------------------------------------
//���û�����
//------------------------------------------------------------

void CAN1_TX_IRQHandler(void){
	if(CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET){
	  CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
}

void CAN1_RX0_IRQHandler(void){
	CanRxMsg rx_message;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET){
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		
		///���յ�������
		if(rx_message.StdId==Chassis_ID)RecieveChassisData(&rx_message);
		///���յ������
		else CAN_MSG_Encode(rx_message.StdId,rx_message.Data,CAN_1);
		
	}
}

void CAN2_TX_IRQHandler(void){
  if(CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET){
	  CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
  }
}

void CAN2_RX1_IRQHandler(void){
	CanRxMsg rx_message;
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP1)!= RESET){
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
		CAN_Receive(CAN2, CAN_FIFO1, &rx_message);
		///���յ�������
		if(rx_message.StdId==Chassis_ID)RecieveChassisData(&rx_message);
		///���յ������
		else CAN_MSG_Encode(rx_message.StdId,rx_message.Data,CAN_2);
		
	}
}

