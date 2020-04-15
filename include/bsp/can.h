#ifndef _CAN1_H_
#define _CAN1_H_
/*****************************************************************************
*  @file     can.h                                                       	 *
*  @brief    ���ڶ�CANͨ�����ݽ���                                           *
*  ������CANͨ�ŵ������б��Լ����ö�ٺͽṹ��Ķ���						 *
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
//�����б�
//CAN_DEVICE_NUM �����豸��Ŀ���б��м����豸����Ϊ��
//��ʽ��
//#define YAW_MOTOR			1		///<������У�1-CAN_DEVICE_NUM�������ظ�
//#define YAW_MOTOR_ID		0		///<������뿪��ID,��Ϊ������Ϊ���̵Ľ���ID
//			һ�������뿪��ֻ��ǰn-1λ��λ����ӦȨֵ���һλ�����Ƿ����Canͨ��ĩ�˵���
//			���磺1-4����Ϊ0011���Ӧ��������100��4,һ��������0
//			RM6623�����0��ʼ,���뿪���ڵ���������ڣ����뿪�غ�2-4λ����ID
//#define YAW_MOTOR_TYPE	RM6623	///<���/�������
//#define YAW_MOTOR_CH		CAN_1	///<CAN����
//------------------------------------------------------------
#define CAN_DEVICE_NUM 		6
//Yaw����
#define YAW_MOTOR			1
#define	YAW_MOTOR_ID		0
#define YAW_MOTOR_TYPE		RM6623
#define YAW_MOTOR_CH		CAN_1
#define YAW_MOTOR_BIAS		7260
//Pitch����
#define PIT_MOTOR			2
#define	PIT_MOTOR_ID		2
#define PIT_MOTOR_TYPE		GM6020
#define PIT_MOTOR_CH		CAN_1
#define PIT_MOTOR_BIAS		2380
//��ֱĦ����
#define FRIC_DOWN_MOTOR			3
#define	FRIC_DOWN_MOTOR_ID		1
#define FRIC_DOWN_MOTOR_TYPE	C620
#define FRIC_DOWN_MOTOR_CH		CAN_1
#define FRIC_DOWN_MOTOR_BIAS	0

#define FRIC_UP_MOTOR			4
#define	FRIC_UP_MOTOR_ID		2
#define FRIC_UP_MOTOR_TYPE		C620
#define FRIC_UP_MOTOR_CH		CAN_1
#define FRIC_UP_MOTOR_BIAS		0
//ˮƽĦ����
#define FRIC_MID_MOTOR			5
#define	FRIC_MID_MOTOR_ID		3
#define FRIC_MID_MOTOR_TYPE	C610
#define FRIC_MID_MOTOR_CH		CAN_1
#define FRIC_MID_MOTOR_BIAS		0
//������
#define RAMMER_MOTOR			6
#define	RAMMER_MOTOR_ID			
#define RAMMER_MOTOR_TYPE		
#define RAMMER_MOTOR_CH			
#define RAMMER_MOTOR_BIAS		0

#define Chassis_ID	0x402
#define Gimbal_ID	0x401

//------------------------------------------------------------
//ö�����������ݽṹ�嶨��
//------------------------------------------------------------
/**
@brief CANͨ��ö��
*/
typedef enum
{
	CAN_1	= 1,///<C610���,��Ҫ����M2006
	CAN_2	= 2,///<C620�������Ҫ����M3510
}Can_Channel_e;

/**
@brief CAN�豸ö��
*/
typedef enum
{
	Chassis	= 0,
	C610    = 1,///<C610���,��Ҫ����M2006
	C620	= 2,///<C620�������Ҫ����M3510
	GM6020	= 3,
	RM6623	= 4,
	GM3510	= 5,
	RM820R	= 6,
}Device_Type_e;
/**
@brief CAN�豸������Ϣ
*/
typedef struct
{
	Can_Channel_e	ch;				///<CAN�豸ͨ��
	uint8_t			id;				///<CAN������뿪��ID
	uint32_t		id_send;		///<CAN����ID
	uint32_t 		id_recieve;		///<CAN����ID
	Device_Type_e	type;			///<CAN�豸����
	int16_t 		ecd_bias;		///<���̽Ƕȼ���Ĳο�ƫ����
}Can_Cfg_Info_t;
/**
@brief ������ݽṹ��(*Ϊ����������)
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
/**
 * @brief ���յ�����Ϣ��������
 * 
 */
typedef enum
{
	MoveData = 0x00,
}Gimbal_Command_ID_e;
/**
 * @brief 
 * 
 */
typedef enum
{
	GunData 	= 	0x01,
	RoboStateData =	0x02,
}Chassis_Command_ID_e;
/**
 * @brief ����ϵͳ��ǹ������
 * 
 */
typedef __packed struct
{
    uint8_t 	bulletFreq;
    float  	bulletSpeed;
    uint16_t   shooterHeat;
}gun_data_t;
/**
 * @brief ����ϵͳ������״̬����
 * 
 */
typedef __packed struct
{
    uint8_t 	robot_id;
    uint8_t 	robot_level;
    uint16_t 	gun_cooling_rate;
    uint16_t	gun_cooling_limit;
    uint8_t 	gun_speed_limit;
}robot_status_t;

void CAN1_Init(void);
void CAN2_Init(void);

void CAN_Motor_Config(uint8_t seq,uint32_t can_id,Device_Type_e device,Can_Channel_e Can_x,int16_t bias);
void CAN_ID_CHECK(void);
void CAN_id_send_Print(void);
Motor_Data_t GetMotorData(uint8_t device_seq,bool last_data);
void SetMotorCurrent(uint8_t device_seq, int16_t current);
void SendMotorCurrent(uint8_t device_seq);

void SendChassisSpeed(CAN_TypeDef *CANx, uint8_t mode, int16_t Vx, int16_t Vy, int16_t W);
gun_data_t Get_Gun_Data(void);
robot_status_t Get_Robot_Status(void);

#endif 
