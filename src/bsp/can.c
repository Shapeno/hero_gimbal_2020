#include "can.h"
/*****************************************************************************
*  @file     can.c                                                       	 *
*  @brief    用于对CAN通信数据解码                                           *
*  包含了CAN通信的配置函数，初始化函数，CAN通信电机数据的获取函数			 *
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

/// @brief CAN通信设备配置配置信息
static Can_Cfg_Info_t 	can_cfg_info[CAN_DEVICE_NUM];
/// @brief CAN通信设备接收数据(ecd_angle取4000防止电机启动时在特殊位置导致cycles++)
static Motor_Data_t 	motor_data[CAN_DEVICE_NUM]={4000,0,0,0,0};
static Motor_Data_t 	motor_last_data[CAN_DEVICE_NUM]={4000,0,0,0,0};

/// @brief CAN通信设备发送数据
static uint8_t data_CAN1_0x200[8]={0};
static uint8_t data_CAN1_0x1FF[8]={0};
static uint8_t data_CAN1_0x2FF[8]={0};
static uint8_t data_CAN2_0x200[8]={0};
static uint8_t data_CAN2_0x1FF[8]={0};
static uint8_t data_CAN2_0x2FF[8]={0};

/// @brief 从底盘来获取裁判系统枪口数据
static gun_data_t Gun_Data={0};
gun_data_t Get_Gun_Data(){return Gun_Data;}
/// @brief 从底盘来获取裁判系统机器人状态数据
static robot_status_t Robot_Status={0};
robot_status_t Get_Robot_Status(){return Robot_Status;}
//------------------------------------------------------------
//电机相关函数
//------------------------------------------------------------

/**
@brief CAN电机信息配置
@param device_seq	设备序列号,用于区分设备，必须小于CAN_DEVICE_NUM
@param ID			电调拨码卡关ID,若为底盘则为底盘的接收ID
@param device		设备类型，详细参考Motor_Data_t结构体注释
@param CAN_x 		CAN_1或CAN_2
@param bias			码盘角度计算的参考偏移量
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
	else printf("seq必须小于CAN_DEVICE_NUM\r\n");
}

/**
@brief 判断是否有重复ID
只能在所有设备配置完后执行
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
					if(id_i==id_j)printf("序列%d设备发送ID重复\r\n",i+1);
				}
				if(can_cfg_info[i].id_recieve==can_cfg_info[j].id_recieve){
					printf("序列%d设备接收ID重复\r\n",i+1);
				}
			}
		}
	}
}
/**
@brief 打印设备控制ID
只能在所有设备配置完后执行
*/
void CAN_id_send_Print(void){
	for(int i=0;i<CAN_DEVICE_NUM;i++){
		if(can_cfg_info[i].ch==CAN_1)
			printf("设备序列:%d\tid_send:%#X\tChannle:CAN1\r\n",i+1,can_cfg_info[i].id_send);
		else if(can_cfg_info[i].ch==CAN_2)
			printf("设备序列:%d\tid_send:%#X\tChannle:CAN2\r\n",i+1,can_cfg_info[i].id_send);
	}
}
/**
@brief 获取电机的数据
@param device_seq必须是电机的设备序列号
@param last_data:false-当前数据，true-上一次数据
@return 设备对应的Motor_Data_t类型的数据，
	所有电机都有角度信息，但不是都有其他信息
	详细参考Motor_Data_t结构体注释
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
@brief 设定电机的控制数据(未发送)
控制报文ID与可对应电机
0x200	C610,C620
0x1FF	C610,C620,GM6020,RM6623,GM3510
0x2FF	GM6020,RM6623
@param device_seq必须是电机的设备序列号
@param current是电机的控制电流
*/
void SetMotorCurrent(uint8_t device_seq, int16_t current){
	if(can_cfg_info[device_seq-1].type>Chassis){
		///<限幅
		switch(can_cfg_info[device_seq-1].type){
		case C610:		VAL_LIMIT(current, -10000, 10000);break;
		case C620:		VAL_LIMIT(current, -16384, 16384);break;
		case GM6020:	VAL_LIMIT(current, -30000, 30000);break;
		case RM6623:	VAL_LIMIT(current, -29000, 29000);break;
		case GM3510:	VAL_LIMIT(current, -5000, 5000);break;
		case RM820R:	VAL_LIMIT(current, -32768, 32767);break;
		default:break;
		}
		///<判断
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
@brief 发送某一电机所在控制ID所有的控制数据
某一控制ID内电机的所有数据更新完后再调用，可以减少发送次数
0x200	C610,C620
0x1FF	C610,C620,GM6020,RM6623,GM3510
0x2FF	GM6020,RM6623
@param device_seq必须是电机的设备序列号
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
//底盘相关函数
//------------------------------------------------------------

/**
@brief 发送底盘速度
@param forward_back_target	前后速度
@param left_right_target	左右速度
@param rotate_target		自旋速度
@param chasis_heat			底盘热量
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
//初始化函数
//------------------------------------------------------------

/**
@brief CAN1通信初始化
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
@brief CAN2通信初始化
*/
void CAN2_Init(void){
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	GPIO_InitTypeDef       gpio;
	NVIC_InitTypeDef       nvic;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//测试
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
	
	can_filter.CAN_FilterNumber=14;	//CAN1、CAN2共用28各过滤器，CAN1用0~13
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
	
//	can_filter.CAN_FilterIdHigh= (((u32)0x427<<3)&0xFFFF0000)>>16;    //要过滤的ID高位
//  can_filter.CAN_FilterIdLow= (((u32)0x427<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //要过滤的ID低位
//  can_filter.CAN_FilterMaskIdHigh= 0xFFFF;   //过滤器高16位每位必须匹配
//  can_filter.CAN_FilterMaskIdLow= 0xFFFF;   //过滤器低16位每位必须匹配
	
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
//非用户函数
//------------------------------------------------------------
void angle_convert(uint8_t seq);

void can_msg_encode( CanRxMsg * msg, Can_Channel_e CAN_x){
	int i=0;
	///接收底盘数据
	if(msg->StdId==Chassis_ID)RecieveChassisData(msg);
	///接收电机数据
	for(;i<CAN_DEVICE_NUM;i++){
		if(can_cfg_info[i].ch==CAN_x){
			if(msg->StdId==can_cfg_info[i].id_recieve){
				switch(can_cfg_info[i].type){
					case C610:{
						motor_last_data[i]=motor_data[i];
						motor_data[i].ecd_angle	=(msg->Data[0]<<8)|msg->Data[1];
						motor_data[i].speed	=(msg->Data[2]<<8)|msg->Data[3];
						motor_data[i].torque=(msg->Data[4]<<8)|msg->Data[5];
						angle_convert(i+1);//换算角度
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
				i=CAN_DEVICE_NUM;	///<结束循环
			}
		}
	}
	if(i==CAN_DEVICE_NUM)printf("存在未识别设备,ID=%#X\r\n",msg->StdId);///<若数组内有该设备，运行至此处时i=CAN_DEVICE_NUM+1
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

