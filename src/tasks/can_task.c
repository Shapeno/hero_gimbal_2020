#include "can_task.h"
/*****************************************************************************
*  @file     can_task.c                                                      *
*  @brief    ���ڶ�CANͨ�ų�ʼ��                                             *
*  ��can.h�ڵ�can�豸�����б��е���豸ID,����,����,CAN_DEVICE_NUM�������	 *
*  �豸��Ŀ,�豸���б����1��ʼ��������,����������			                 *
*  ʹ�ò����б�ĺ궨����CAN_Device_Init()�����õ��,��ʼ���󼴿�ʹ��can.c�� *
*  ��ؽӿں�������ȡ�������                                                *
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
	CAN_Motor_Config(FRIC_DOWN_MOTOR,FRIC_DOWN_MOTOR_ID,FRIC_DOWN_MOTOR_TYPE,FRIC_DOWN_MOTOR_CH,FRIC_DOWN_MOTOR_BIAS);
	CAN_Motor_Config(FRIC_UP_MOTOR,FRIC_UP_MOTOR_ID,FRIC_UP_MOTOR_TYPE,FRIC_UP_MOTOR_CH,FRIC_UP_MOTOR_BIAS);
	CAN_Motor_Config(FRIC_MID_MOTOR,FRIC_MID_MOTOR_ID,FRIC_MID_MOTOR_TYPE,FRIC_MID_MOTOR_CH,FRIC_MID_MOTOR_BIAS);
	CAN_Motor_Config(RAMMER_MOTOR,RAMMER_MOTOR_ID,RAMMER_MOTOR_TYPE,RAMMER_MOTOR_CH,RAMMER_MOTOR_BIAS);
	CAN_ID_CHECK();
	CAN_id_send_Print();
	CAN1_Init();
	CAN2_Init();
}

void CanMsgPrc(){
	SendMotorCurrent();
}
