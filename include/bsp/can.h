#ifndef _CAN1_H_
#define _CAN1_H_
/*****************************************************************************
*  @file     can.h                                                       	 *
*  @brief    用于对CAN通信数据解码                                           *
*  包含了CAN通信的配置列表，以及相关枚举和结构体的定义						 *
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
*  2019/09/11 | 1.0.1.1   | DengXY	       | Function test                   *
*----------------------------------------------------------------------------*
*****************************************************************************/
#include "stm32f4xx.h"
#include "sys.h"
#define Full_Ecd_Angle (8192.00)
//------------------------------------------------------------
//配置列表
//CAN_DEVICE_NUM 配置设备数目，列表有几个设备就设为几
//格式：
//#define YAW_MOTOR			1		///<电机序列，1-CAN_DEVICE_NUM，不可重复
//#define YAW_MOTOR_ID		0		///<电调拨码开关ID,若为底盘则为底盘的接收ID
//			一般电机拨码开关只算前n-1位，位数对应权值最后一位先择是否接入Can通信末端电阻
//			例如：1-4编码为0011则对应二进制数100即4,一般必须大于0
//			RM6623编码从0开始,拨码开关在电调保护壳内，拨码开关后2-4位设置ID
//#define YAW_MOTOR_TYPE	RM6623	///<电机/电调类型
//#define YAW_MOTOR_CH		CAN_1	///<CAN总线
//------------------------------------------------------------
#define CAN_DEVICE_NUM 6
////Yaw轴电机
#define YAW_MOTOR		4
#define	YAW_MOTOR_ID	0
#define YAW_MOTOR_TYPE	RM6623
#define YAW_MOTOR_CH	CAN_2
#define YAW_MOTOR_BIAS	7260
//Pitch轴电机
#define PIT_MOTOR		5
#define	PIT_MOTOR_ID	2
#define PIT_MOTOR_TYPE	GM6020
#define PIT_MOTOR_CH	CAN_2
#define PIT_MOTOR_BIAS	1450
//拨弹电机
#define RAMMER_MOTOR		6
#define	RAMMER_MOTOR_ID		1
#define RAMMER_MOTOR_TYPE	C610
#define RAMMER_MOTOR_CH		CAN_2
#define RAMMER_MOTOR_BIAS	0
//垂直摩擦轮
#define FRIC0_MOTOR			1
#define	FRIC0_MOTOR_ID		1
#define FRIC0_MOTOR_TYPE	C610
#define FRIC0_MOTOR_CH		CAN_1
#define FRIC0_MOTOR_BIAS	0
//水平摩擦轮
#define FRIC1_MOTOR			2
#define	FRIC1_MOTOR_ID		2
#define FRIC1_MOTOR_TYPE	C620
#define FRIC1_MOTOR_CH		CAN_1
#define FRIC1_MOTOR_BIAS	0

#define FRIC2_MOTOR			3
#define	FRIC2_MOTOR_ID		3
#define FRIC2_MOTOR_TYPE	C620
#define FRIC2_MOTOR_CH		CAN_1
#define FRIC2_MOTOR_BIAS	0

//------------------------------------------------------------
//枚举类型与数据结构体定义
//------------------------------------------------------------
/**
@brief CAN通道枚举
*/
typedef enum
{
	CAN_1	= 1,///<C610电调,主要驱动M2006
	CAN_2	= 2,///<C620电调，主要驱动M3510
}Can_Channel_e;

/**
@brief CAN设备枚举
*/
typedef enum
{
	Chassis	= 0,
	C610    = 1,///<C610电调,主要驱动M2006
	C620	= 2,///<C620电调，主要驱动M3510
	GM6020	= 3,
	RM6623	= 4,
	GM3510	= 5,
	RM820R	= 6,
}Device_Type_e;
/**
@brief CAN设备配置信息
*/
typedef struct
{
	Can_Channel_e	ch;				///<CAN设备通道
	uint8_t			id;				///<CAN电调拨码开关ID
	uint32_t		id_send;		///<CAN控制ID
	uint32_t 		id_recieve;		///<CAN反馈ID
	Device_Type_e	type;			///<CAN设备类型
	int16_t 		ecd_bias;		///<码盘角度计算的参考偏移量
}Can_Cfg_Info_t;
/**
@brief 电机数据结构体(*为有这个数据)
type	angle	speed	torque	temperature
C610	*		*		*
C620	*		*		*		*
GM6020	*		*		*		*
RM6623	*				*		
GM3510	*				*
RM820R	*		*
Chassis	(Not a motor)
*/
typedef struct
{
	int16_t  ecd_angle;
	int16_t  cycles;
	float    angle;
	int16_t  speed;
	int16_t  torque;
	int8_t  temperature;		
}Motor_Data_t;

void CAN1_Init(void);
void CAN2_Init(void);

void CAN_Motor_Config(uint8_t seq,uint32_t can_id,Device_Type_e device,Can_Channel_e Can_x,int16_t bias);
void CAN_ID_CHECK(void);
void CAN_id_send_Print(void);
Motor_Data_t GetMotorData(uint8_t device_seq);
void SetMotorCurrent(uint8_t device_seq, int16_t current);
void SendMotorCurrent(uint8_t device_seq);

void SendChassisSpeed(CAN_TypeDef *CANx, int16_t forward_back_target, int16_t left_right_target, int16_t rotate_target, int16_t chasis_heat);

#endif 
