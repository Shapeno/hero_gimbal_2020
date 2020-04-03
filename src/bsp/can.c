#include "can.h"
/*****************************************************************************
*  @file     can.c                                                       	 *
*  @brief    ���ڶ�CANͨ�����ݽ���                                           *
*  ������CANͨ�ŵ����ú�������ʼ��������CANͨ�ŵ�����ݵĻ�ȡ����			 *
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
*****************************************************************************/
#include "sys.h"
#include <string.h>
#include "usart3.h"

/// @brief CANͨ���豸����������Ϣ
static Can_Cfg_Info_t 	can_cfg_info[CAN_DEVICE_NUM];
/// @brief CANͨ���豸��������(ecd_angleȡ4000��ֹ�������ʱ������λ�õ���cycles++)
static Motor_Data_t 	motor_data[CAN_DEVICE_NUM]={4000,0,0,0,0};
static Motor_Data_t 	motor_last_data[CAN_DEVICE_NUM]={4000,0,0,0,0};

/// @brief CANͨ���豸��������
static uint8_t data_CAN1_0x200[8]={0};
static uint8_t data_CAN1_0x1FF[8]={0};
static uint8_t data_CAN1_0x2FF[8]={0};
static uint8_t data_CAN2_0x200[8]={0};
static uint8_t data_CAN2_0x1FF[8]={0};
static uint8_t data_CAN2_0x2FF[8]={0};

/// @brief �ӵ�������ȡ����ϵͳǹ������
static gun_data_t Gun_Data={0};
gun_data_t Get_Gun_Data(){return Gun_Data;}
/// @brief �ӵ�������ȡ����ϵͳ������״̬����
static robot_status_t Robot_Status={0};
robot_status_t Get_Robot_Status(){return Robot_Status;}
//------------------------------------------------------------
//�����غ���
//------------------------------------------------------------

/**
@brief CAN�����Ϣ����
@param device_seq	�豸���к�,���������豸������С��CAN_DEVICE_NUM
@param ID			������뿨��ID,��Ϊ������Ϊ���̵Ľ���ID
@param device		�豸���ͣ���ϸ�ο�Motor_Data_t�ṹ��ע��
@param CAN_x 		CAN_1��CAN_2
@param bias			���̽Ƕȼ���Ĳο�ƫ����
*/
void CAN_Motor_Config(uint8_t device_seq,uint32_t ID,Device_Type_e device,Can_Channel_e CAN_x,int16_t bias){
	if(device_seq<=CAN_DEVICE_NUM){
		can_cfg_info[device_seq-1].id=ID;
		can_cfg_info[device_seq-1].ch=CAN_x;
		can_cfg_info[device_seq-1].type=device;
		can_cfg_info[device_seq-1].ecd_bias=bias;
		switch(device){
			case C610:{
				if(ID<=4){
					can_cfg_info[device_seq-1].id_send=0x200;
					can_cfg_info[device_seq-1].id_recieve=0x200+ID;
				}
				else if(ID<=8){
					can_cfg_info[device_seq-1].id_send=0x1FF;
					can_cfg_info[device_seq-1].id_recieve=0x200+ID;
				}
				else printf("ID wrong\r\n");
			}break;
			case C620:{
				if(ID<=4){
					can_cfg_info[device_seq-1].id_send=0x200;
					can_cfg_info[device_seq-1].id_recieve=0x200+ID;
				}
				else if(ID<=8){
					can_cfg_info[device_seq-1].id_send=0x1FF;
					can_cfg_info[device_seq-1].id_recieve=0x200+ID;
				}
				else printf("ID wrong\r\n");
			}break;
			case GM6020:{
				if(ID>=1&&ID<=4){
					can_cfg_info[device_seq-1].id_send=0x1FF;
					can_cfg_info[device_seq-1].id_recieve=0x204+ID;
				}
				else if(ID<=7){
					can_cfg_info[device_seq-1].id_send=0x2FF;
					can_cfg_info[device_seq-1].id_recieve=0x204+ID;
				}
				else printf("ID wrong\r\n");
			}break;
			
			case RM6623:{
				if(ID<=4){
					can_cfg_info[device_seq-1].id_send=0x1FF;
					can_cfg_info[device_seq-1].id_recieve=0x205+ID;
				}
				else if(ID<=8){
					can_cfg_info[device_seq-1].id_send=0x2FF;
					can_cfg_info[device_seq-1].id_recieve=0x205+ID;
				}
				else printf("ID wrong\r\n");	
			}break;
			
			case GM3510:{
				if(ID>=1&&ID<=3){
					can_cfg_info[device_seq-1].id_send=0x1FF;
					can_cfg_info[device_seq-1].id_recieve=0x204+ID;
				}
				else printf("ID wrong\r\n");
			}break;
			
			case RM820R:{
				if(ID>=1&&ID<=4){
					can_cfg_info[device_seq-1].id_send=0x200;
					can_cfg_info[device_seq-1].id_recieve=0x200+ID;
				}
				else printf("ID wrong\r\n");
			}break;
			
			case Chassis:{
				can_cfg_info[device_seq-1].id_send=ID;
			}break;
			
			default:{
				printf("no such device\r\n");
			}break;
		}
	}
	else printf("seq����С��CAN_DEVICE_NUM\r\n");
}

/**
@brief �ж��Ƿ����ظ�ID
ֻ���������豸�������ִ��
*/
void CAN_ID_CHECK(void){
	for(int i=0;i<CAN_DEVICE_NUM;i++){
		int id_i=0;
		if(can_cfg_info[i].type==RM6623)id_i=can_cfg_info[i].id+1;
		else id_i=can_cfg_info[i].id;
		for(int j=i+1;j<CAN_DEVICE_NUM;j++){
			int id_j=0;
			if(can_cfg_info[j].type==RM6623)id_j=can_cfg_info[j].id+1;
			else id_j=can_cfg_info[j].id;
			if(can_cfg_info[i].ch==can_cfg_info[j].ch){
				if(can_cfg_info[i].id_send==can_cfg_info[j].id_send){
					if(id_i==id_j)printf("����%d�豸����ID�ظ�\r\n",i+1);
				}
				if(can_cfg_info[i].id_recieve==can_cfg_info[j].id_recieve){
					printf("����%d�豸����ID�ظ�\r\n",i+1);
				}
			}
		}
	}
}
/**
@brief ��ӡ�豸����ID
ֻ���������豸�������ִ��
*/
void CAN_id_send_Print(void){
	for(int i=0;i<CAN_DEVICE_NUM;i++){
		if(can_cfg_info[i].ch==CAN_1)
			printf("�豸����:%d\tid_send:%#X\tChannle:CAN1\r\n",i+1,can_cfg_info[i].id_send);
		else if(can_cfg_info[i].ch==CAN_2)
			printf("�豸����:%d\tid_send:%#X\tChannle:CAN2\r\n",i+1,can_cfg_info[i].id_send);
	}
}
/**
@brief ��ȡ���������
@param device_seq�����ǵ�����豸���к�
@param last_data:false-��ǰ���ݣ�true-��һ������
@return �豸��Ӧ��Motor_Data_t���͵����ݣ�
	���е�����нǶ���Ϣ�������Ƕ���������Ϣ
	��ϸ�ο�Motor_Data_t�ṹ��ע��
*/
Motor_Data_t GetMotorData(uint8_t device_seq,bool last_data){
	if(can_cfg_info[device_seq-1].type>Chassis){
		if(last_data)return motor_last_data[device_seq-1];
		return motor_data[device_seq-1];
	}
	else {
		printf("device %d is not a motor\r\n",device_seq);
		Motor_Data_t no_data={0};
		return no_data;
	}
}

/**
@brief �趨����Ŀ�������(δ����)
���Ʊ���ID��ɶ�Ӧ���
0x200	C610,C620
0x1FF	C610,C620,GM6020,RM6623,GM3510
0x2FF	GM6020,RM6623
@param device_seq�����ǵ�����豸���к�
@param current�ǵ���Ŀ��Ƶ���
*/
void SetMotorCurrent(uint8_t device_seq, int16_t current){
	if(can_cfg_info[device_seq-1].type>Chassis){
		///<�޷�
		switch(can_cfg_info[device_seq-1].type){
		case C610:		VAL_LIMIT(current, -10000, 10000);break;
		case C620:		VAL_LIMIT(current, -16384, 16384);break;
		case GM6020:	VAL_LIMIT(current, -30000, 30000);break;
		case RM6623:	VAL_LIMIT(current, -29000, 29000);break;
		case GM3510:	VAL_LIMIT(current, -5000, 5000);break;
		case RM820R:	VAL_LIMIT(current, -32768, 32767);break;
		default:break;
		}
		///<�ж�
		if(can_cfg_info[device_seq-1].ch==CAN_1){
			switch(can_cfg_info[device_seq-1].id_send){
				case 0x200:{
					data_CAN1_0x200[2*can_cfg_info[device_seq-1].id-2]	=(uint8_t)(current >> 8);
					data_CAN1_0x200[2*can_cfg_info[device_seq-1].id-1]	=(uint8_t)current;
				}break;
				case 0x1FF:{
					if(can_cfg_info[device_seq-1].type==C610||can_cfg_info[device_seq-1].type==C620){
						data_CAN1_0x1FF[2*(can_cfg_info[device_seq-1].id-4)-2]	=(uint8_t)(current >> 8);
						data_CAN1_0x1FF[2*(can_cfg_info[device_seq-1].id-4)-1]	=(uint8_t)current;
					}
					else if(can_cfg_info[device_seq-1].type==GM6020||can_cfg_info[device_seq-1].type==GM6020){
						data_CAN1_0x1FF[2*can_cfg_info[device_seq-1].id-2]	=(uint8_t)(current >> 8);
						data_CAN1_0x1FF[2*can_cfg_info[device_seq-1].id-1]	=(uint8_t)current;
					}
					else if(can_cfg_info[device_seq-1].type==RM6623){
						data_CAN1_0x1FF[2*can_cfg_info[device_seq-1].id]	=(uint8_t)(current >> 8);
						data_CAN1_0x1FF[2*can_cfg_info[device_seq-1].id+1]	=(uint8_t)current;
					}
				}break;
				case 0x2FF:{
					if(can_cfg_info[device_seq-1].type==GM6020){
						data_CAN1_0x2FF[2*(can_cfg_info[device_seq-1].id-4)-2]	=(uint8_t)(current >> 8);
						data_CAN1_0x2FF[2*(can_cfg_info[device_seq-1].id-4)-1]	=(uint8_t)current;
					}
					else if(can_cfg_info[device_seq-1].type==RM6623){
						data_CAN1_0x2FF[2*(can_cfg_info[device_seq-1].id-4)]	=(uint8_t)(current >> 8);
						data_CAN1_0x2FF[2*(can_cfg_info[device_seq-1].id-4)+1]	=(uint8_t)current;
					}
				}break;
			}
		}
		else if(can_cfg_info[device_seq-1].ch==CAN_2){
			switch(can_cfg_info[device_seq-1].id_send){
				case 0x200:{
					data_CAN2_0x200[2*can_cfg_info[device_seq-1].id-2]	=(uint8_t)(current >> 8);
					data_CAN2_0x200[2*can_cfg_info[device_seq-1].id-1]	=(uint8_t)current;
				}break;
				case 0x1FF:{
					if(can_cfg_info[device_seq-1].type==C610||can_cfg_info[device_seq-1].type==C620){
						data_CAN2_0x1FF[2*(can_cfg_info[device_seq-1].id-4)-2]	=(uint8_t)(current >> 8);
						data_CAN2_0x1FF[2*(can_cfg_info[device_seq-1].id-4)-1]	=(uint8_t)current;
					}
					else if(can_cfg_info[device_seq-1].type==GM6020||can_cfg_info[device_seq-1].type==GM6020){
						data_CAN2_0x1FF[2*can_cfg_info[device_seq-1].id-2]	=(uint8_t)(current >> 8);
						data_CAN2_0x1FF[2*can_cfg_info[device_seq-1].id-1]	=(uint8_t)current;
					}
					else if(can_cfg_info[device_seq-1].type==RM6623){
						data_CAN2_0x1FF[2*can_cfg_info[device_seq-1].id]	=(uint8_t)(current >> 8);
						data_CAN2_0x1FF[2*can_cfg_info[device_seq-1].id+1]	=(uint8_t)current;
					}
				}break;
				case 0x2FF:{
					if(can_cfg_info[device_seq-1].type==GM6020){
						data_CAN2_0x2FF[2*(can_cfg_info[device_seq-1].id-4)-2]	=(uint8_t)(current >> 8);
						data_CAN2_0x2FF[2*(can_cfg_info[device_seq-1].id-4)-1]	=(uint8_t)current;
					}
					else if(can_cfg_info[device_seq-1].type==RM6623){
						data_CAN2_0x2FF[2*(can_cfg_info[device_seq-1].id-4)]	=(uint8_t)(current >> 8);
						data_CAN2_0x2FF[2*(can_cfg_info[device_seq-1].id-4)+1]	=(uint8_t)current;
					}
				}break;
			}
		}
	}
	else printf("device %d is not a motor\r\n",device_seq);
}
/**
@brief ����ĳһ������ڿ���ID���еĿ�������
ĳһ����ID�ڵ�����������ݸ�������ٵ��ã����Լ��ٷ��ʹ���
0x200	C610,C620
0x1FF	C610,C620,GM6020,RM6623,GM3510
0x2FF	GM6020,RM6623
@param device_seq�����ǵ�����豸���к�
*/
void SendMotorCurrent(uint8_t device_seq){
	if(can_cfg_info[device_seq-1].type>Chassis){
		CanTxMsg tx_message;
		tx_message.StdId = can_cfg_info[device_seq-1].id_send;
		  //gimbal_yaw_iq = 0;
		tx_message.IDE = CAN_Id_Standard;
		tx_message.RTR = CAN_RTR_Data;
		tx_message.DLC = 0x08;
		switch(can_cfg_info[device_seq-1].id_send){
			case 0x200:{
				if(can_cfg_info[device_seq-1].ch==CAN_1){
					memcpy(tx_message.Data,data_CAN1_0x200,sizeof(tx_message.Data));
					CAN_Transmit(CAN1,&tx_message);
				}
				else{
					memcpy(tx_message.Data,data_CAN2_0x200,sizeof(tx_message.Data));
					CAN_Transmit(CAN2,&tx_message);
				}
			}break;
			case 0x1FF:{
				if(can_cfg_info[device_seq-1].ch==CAN_1){
					memcpy(tx_message.Data,data_CAN1_0x1FF,sizeof(tx_message.Data));
					CAN_Transmit(CAN1,&tx_message);
				}
				else{
					memcpy(tx_message.Data,data_CAN2_0x1FF,sizeof(tx_message.Data));
					CAN_Transmit(CAN2,&tx_message);
				}
			}break;
			case 0x2FF:{
				if(can_cfg_info[device_seq-1].ch==CAN_1){
					memcpy(tx_message.Data,data_CAN1_0x2FF,sizeof(tx_message.Data));
					CAN_Transmit(CAN1,&tx_message);
				}
				else{
					memcpy(tx_message.Data,data_CAN2_0x2FF,sizeof(tx_message.Data));
					CAN_Transmit(CAN2,&tx_message);
				}
			}break;
		}
	}
}
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
	nvic.NVIC_IRQChannelSubPriority = 1;
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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 
	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &gpio);
	nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
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
	
//	can_filter.CAN_FilterIdHigh= (((u32)0x427<<3)&0xFFFF0000)>>16;    //Ҫ���˵�ID��λ
//  can_filter.CAN_FilterIdLow= (((u32)0x427<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //Ҫ���˵�ID��λ
//  can_filter.CAN_FilterMaskIdHigh= 0xFFFF;   //��������16λÿλ����ƥ��
//  can_filter.CAN_FilterMaskIdLow= 0xFFFF;   //��������16λÿλ����ƥ��
	
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0400;
	can_filter.CAN_FilterFIFOAssignment=CAN_FilterFIFO0;	
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}

//------------------------------------------------------------
//���û�����
//------------------------------------------------------------
void angle_convert(uint8_t seq);

void can_msg_encode( CanRxMsg * msg, Can_Channel_e CAN_x){
	int i=0;
	///���յ�������
	if(msg->StdId==Chassis_ID)RecieveChassisData(msg);
	///���յ������
	for(;i<CAN_DEVICE_NUM;i++){
		if(can_cfg_info[i].ch==CAN_x){
			if(msg->StdId==can_cfg_info[i].id_recieve){
				switch(can_cfg_info[i].type){
					case C610:{
						motor_last_data[i]=motor_data[i];
						motor_data[i].ecd_angle	=(msg->Data[0]<<8)|msg->Data[1];
						motor_data[i].speed	=(msg->Data[2]<<8)|msg->Data[3];
						motor_data[i].torque=(msg->Data[4]<<8)|msg->Data[5];
						angle_convert(i+1);//����Ƕ�
					}break;
					case C620:{
						motor_last_data[i]=motor_data[i];
						motor_data[i].ecd_angle	=(msg->Data[0]<<8)|msg->Data[1];
						motor_data[i].speed	=(msg->Data[2]<<8)|msg->Data[3];
						motor_data[i].torque=(msg->Data[4]<<8)|msg->Data[5];
						motor_data[i].temperature=msg->Data[6];
						angle_convert(i+1);
					}break;
					case GM6020:{
						motor_last_data[i]=motor_data[i];
						motor_data[i].ecd_angle	=(msg->Data[0]<<8)|msg->Data[1];
						motor_data[i].speed	=(msg->Data[2]<<8)|msg->Data[3];
						motor_data[i].torque=(msg->Data[4]<<8)|msg->Data[5];
						motor_data[i].temperature=msg->Data[6];
						angle_convert(i+1);
					}break;
					case RM6623:{
						motor_last_data[i]=motor_data[i];
						motor_data[i].ecd_angle	=(msg->Data[0]<<8)|msg->Data[1];
						motor_data[i].torque=(msg->Data[2]<<8)|msg->Data[3];
						angle_convert(i+1);
					}
					case GM3510:{
						motor_last_data[i]=motor_data[i];
						motor_data[i].ecd_angle	=(msg->Data[0]<<8)|msg->Data[1];
						motor_data[i].torque=(msg->Data[2]<<8)|msg->Data[3];
						angle_convert(i+1);
					}break;
					case RM820R:{
						motor_last_data[i]=motor_data[i];
						motor_data[i].ecd_angle	=(msg->Data[0]<<8)|msg->Data[1];
						motor_data[i].speed	=(msg->Data[2]<<8)|msg->Data[3];
						angle_convert(i+1);
					}
					case Chassis:{
						
					}break;
					default:break;
				}
				i=CAN_DEVICE_NUM;	///<����ѭ��
			}
		}
	}
	if(i==CAN_DEVICE_NUM)printf("����δʶ���豸,ID=%#X\r\n",msg->StdId);///<���������и��豸���������˴�ʱi=CAN_DEVICE_NUM+1
}

void angle_convert(uint8_t seq){
	motor_data[seq-1].angle=(motor_data[seq-1].ecd_angle-can_cfg_info[seq-1].ecd_bias)*360.00/Full_Ecd_Angle;
	if(motor_data[seq-1].angle>180){
		motor_data[seq-1].angle-=360.0;
	}
	else if(motor_data[seq-1].angle<-180){
		motor_data[seq-1].angle+=360.0;
	}
	if((motor_data[seq-1].angle-motor_last_data[seq-1].angle)>300)motor_data[seq-1].cycles--;
	else if((motor_data[seq-1].angle-motor_last_data[seq-1].angle)<-300)motor_data[seq-1].cycles++;
}
void CAN1_TX_IRQHandler(void){
	if(CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET){
	  CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
}

void CAN1_RX0_IRQHandler(void){
	CanRxMsg rx_message;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET){
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		can_msg_encode(&rx_message,CAN_1);
	}
}

void CAN2_TX_IRQHandler(void){
  if(CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET){
	  CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
  }
}

void CAN2_RX0_IRQHandler(void){
	CanRxMsg rx_message;
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET){
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_Receive(CAN2, CAN_FIFO0, &rx_message);
		can_msg_encode(&rx_message,CAN_2);
	}
}

