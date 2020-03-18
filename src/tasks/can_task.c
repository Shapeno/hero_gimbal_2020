#include "can_task.h"
/*****************************************************************************
*  @file     can_task.c                                                      *
*  @brief    用于对CAN通信初始化                                             *
*  在can.h内的can设备配置列表中电机设备ID,类型,总线,CAN_DEVICE_NUM必须等于	 *
*  设备数目,设备序列必须从1开始依次排列,不允许跳过			                 *
*  使用参数列表的宏定义在CAN_Device_Init()中配置电机,初始化后即可使用can.c中 *
*  相关接口函数来获取电机数据                                                *
*                                                                            *
*  @author   DENG		                                                     *
*  @version  1.0.0.1		                                                 *
*  @date     19/9/18				                                         *
*                                                                            *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2019/09/18 | 1.0.0.2   | DengXY	       | Function add                    *
*----------------------------------------------------------------------------*
*****************************************************************************/
#include "can.h"


void CAN_Device_Init(){
	CAN_Motor_Config(YAW_MOTOR,YAW_MOTOR_ID,YAW_MOTOR_TYPE,YAW_MOTOR_CH,YAW_MOTOR_BIAS);
	CAN_Motor_Config(PIT_MOTOR,PIT_MOTOR_ID,PIT_MOTOR_TYPE,PIT_MOTOR_CH,PIT_MOTOR_BIAS);
	CAN_Motor_Config(RAMMER_MOTOR,RAMMER_MOTOR_ID,RAMMER_MOTOR_TYPE,RAMMER_MOTOR_CH,RAMMER_MOTOR_BIAS);
	CAN_Motor_Config(FRIC0_MOTOR,FRIC0_MOTOR_ID,FRIC0_MOTOR_TYPE,FRIC0_MOTOR_CH,FRIC0_MOTOR_BIAS);
	CAN_Motor_Config(FRIC1_MOTOR,FRIC1_MOTOR_ID,FRIC1_MOTOR_TYPE,FRIC1_MOTOR_CH,FRIC1_MOTOR_BIAS);
	CAN_Motor_Config(FRIC2_MOTOR,FRIC2_MOTOR_ID,FRIC2_MOTOR_TYPE,FRIC2_MOTOR_CH,FRIC2_MOTOR_BIAS);
	CAN_ID_CHECK();
	CAN_id_send_Print();
	CAN1_Init();
	CAN2_Init();
}

void CanMsgPrc(){
//	SendMotorCurrent(YAW_MOTOR);
//	SendMotorCurrent(RAMMER_MOTOR);
	SendMotorCurrent(FRIC0_MOTOR);
}
