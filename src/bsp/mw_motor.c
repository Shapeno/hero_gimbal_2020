#include "mw_motor.h"
/*****************************************************************************
*  @file     mw_motor.c                                                  	 *
*  @brief    官方电机的中间层(MiddleWare)									 *
*  包含了电机的配置函数，数据获取函数，底层解包函数							 *
*  			                                                                 *
*                                                                            *
*  @author   DENG		                                                     *
*  @version  1.0.0.1		                                                 *
*  @date     20/8/3				                                             *
*                                                                            *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2020/08/3  | 1.0.0.1   | DengXY	       | Create file                     *
*----------------------------------------------------------------------------*
*****************************************************************************/
#include <stdio.h>
#include <string.h>
///<限幅宏定义
#ifndef VAL_LIMIT
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\
else {val = val;}
#endif

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
		}
	}
	else printf("seq必须小于CAN_DEVICE_NUM\r\n");
}

/**
@brief 判断是否有重复ID
只能在所有设备配置完后执行
*/
void CAN_Motor_ID_CHECK(void){
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
void CAN_Motor_Send_ID_Print(void){
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
	所有电机都有角度信息，不一定有其他信息
	详细参考Motor_Data_t结构体注释
*/
Motor_Data_t GetMotorData(uint8_t device_seq,bool last_data){
		if(last_data)return motor_last_data[device_seq-1];
		return motor_data[device_seq-1];
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
	///<限幅（按照手册上的幅值限幅）
    switch(can_cfg_info[device_seq-1].type){
    case C610:		VAL_LIMIT(current, -10000, 10000);break;
    case C620:		VAL_LIMIT(current, -16384, 16384);break;
    case GM6020:	VAL_LIMIT(current, -30000, 30000);break;
    case RM6623:	VAL_LIMIT(current, -29000, 29000);break;
    case GM3510:	VAL_LIMIT(current, -5000, 5000);break;
    case RM820R:	VAL_LIMIT(current, -32768, 32767);break;
    default:break;
    }
    ///<判断并存储
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
/**
@brief  同时发送所有电机的数据，建议放在1ms独立循环内。
0x200	C610,C620
0x1FF	C610,C620,GM6020,RM6623,GM3510
0x2FF	GM6020,RM6623
*/
void SendMotorCurrent(){
	static uint8_t can_send_id_flags=0x00;//7-null;6-state flag;5:3-CAN2-0x200/0x1FF/0x2FF;2:0-CAN1-0x200/0x1FF/0x2FF;
	///<获得有哪些发送id需要发送，以及在哪条can总线上，只需进入一次获取can_send_id_flags各标志位
	if((can_send_id_flags&0x40)==0x00){
		for(uint8_t device_seq=1;device_seq<=CAN_DEVICE_NUM;device_seq++){
			switch(can_cfg_info[device_seq-1].id_send){
				case 0x200:{
					if(can_cfg_info[device_seq-1].ch==CAN_1){can_send_id_flags|=(0x01<<2);}
					else{can_send_id_flags|=(0x01<<5);}
				}break;
				case 0x1FF:{
					if(can_cfg_info[device_seq-1].ch==CAN_1){can_send_id_flags|=(0x01<<1);}
					else{can_send_id_flags|=(0x01<<4);}
				}break;
				case 0x2FF:{
					if(can_cfg_info[device_seq-1].ch==CAN_1){can_send_id_flags|=(0x01<<0);}
					else{can_send_id_flags|=(0x01<<3);}
				}break;
			}
		}
		can_send_id_flags|=0x40;//获得所有电机标志位后不再进入
	}
    ///<按照标志位发送数据
	{
		CanTxMsg tx_message;
		tx_message.IDE = CAN_Id_Standard;
		tx_message.RTR = CAN_RTR_Data;
		tx_message.DLC = 0x08;
		if((can_send_id_flags&(0x01<<0))){//CAN1_0x2FF
			tx_message.StdId=0x2FF;
			memcpy(tx_message.Data,data_CAN1_0x2FF,sizeof(tx_message.Data));
			CAN_Transmit(CAN1,&tx_message);
		}
		if((can_send_id_flags&(0x01<<1))){//CAN1_0x1FF
			tx_message.StdId=0x1FF;
			memcpy(tx_message.Data,data_CAN1_0x1FF,sizeof(tx_message.Data));
			CAN_Transmit(CAN1,&tx_message);
		}
		if((can_send_id_flags&(0x01<<2))){//CAN1_0x200
			tx_message.StdId=0x200;
			memcpy(tx_message.Data,data_CAN1_0x200,sizeof(tx_message.Data));
			CAN_Transmit(CAN1,&tx_message);
		}
		if((can_send_id_flags&(0x01<<3))){//CAN2_0x2FF
			tx_message.StdId=0x2FF;
			memcpy(tx_message.Data,data_CAN2_0x2FF,sizeof(tx_message.Data));
			CAN_Transmit(CAN2,&tx_message);
		}
		if((can_send_id_flags&(0x01<<4))){//CAN2_0x1FF
			tx_message.StdId=0x1FF;
			memcpy(tx_message.Data,data_CAN2_0x1FF,sizeof(tx_message.Data));
			CAN_Transmit(CAN2,&tx_message);
		}
		if((can_send_id_flags&(0x01<<5))){//CAN2_0x200
			tx_message.StdId=0x200;
			memcpy(tx_message.Data,data_CAN2_0x200,sizeof(tx_message.Data));
			CAN_Transmit(CAN2,&tx_message);
		}
	}
}

///角度转换函数（内部函数）
void angle_convert(uint8_t seq){
	motor_data[seq-1].angle=(motor_data[seq-1].ecd_angle-can_cfg_info[seq-1].ecd_bias)*360.00/Full_Ecd_Angle;
	if(motor_data[seq-1].angle>180){
		motor_data[seq-1].angle-=360.0f;
	}
	else if(motor_data[seq-1].angle<-180){
		motor_data[seq-1].angle+=360.0f;
	}
	if((motor_data[seq-1].angle-motor_last_data[seq-1].angle)>300)motor_data[seq-1].cycles--;
	else if((motor_data[seq-1].angle-motor_last_data[seq-1].angle)<-300)motor_data[seq-1].cycles++;
}
/**
 * @brief 电机数据解码函数，放在CAN通信中断回调函数中
 * 
 * @param msg CAN通信对应收到的消息
 * @param CAN_x CAN1或CAN2，取决于在哪个中断回调函数中
 */
void CAN_MSG_Encode( CanRxMsg * msg, Can_Channel_e CAN_x){
	int i=0;
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
					default:break;
				}
				i=CAN_DEVICE_NUM;	///<结束循环
			}
		}
	}
	if(i==CAN_DEVICE_NUM)printf("存在未识别设备,ID=%#X\r\n",msg->StdId);///<若数组内有该设备，运行至此处时i=CAN_DEVICE_NUM+1
}
