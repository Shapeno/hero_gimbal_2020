#ifndef _COMMAND_TASK_H_
#define _COMMAND_TASK_H_

#include "sys.h"
#include "dbus.h"

///	@brief	pitch��Ƕ�����
#define PITCH_MAX 15.0f
#define PITCH_MIN -30.0f
///	@brief	ң�������Ƶ�һЩ����
#define STICK_TO_CHASSIS_SPEED_REF_FACT     0.30f
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.004f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.002f
#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		  -0.025f
#define MOUSE_TO_YAW_ANGLE_INC_FACT 		    -0.025f
///	@brief	���̲�ͬģʽ�ٶ�
#define NORMAL_FORWARD_BACK_SPEED 			    207
#define NORMAL_LEFT_RIGHT_SPEED   			    207
#define HIGH_FORWARD_BACK_SPEED 			      330
#define HIGH_LEFT_RIGHT_SPEED   			      230
#define SUPER_HIGH_FORWARD_BACK_SPEED 			414
#define SUPER_HIGH_LEFT_RIGHT_SPEED   			414

///	@brief	����ģʽö��
typedef enum{
	REMOTE_INPUT    = 1,///<ң������
	KEY_MOUSE_INPUT = 3,///<��������
	STOP            = 2,
}InputMode_e;

///	@brief	��̨Ŀ��λ�ýṹ��
typedef struct{
	float pitch_angle_target;
	float yaw_angle_target;
}Gimbal_Target_t;

///	@brief	����Ŀ���ٶȽṹ��
typedef __packed struct{
	int16_t forward_back_target;
	int16_t left_right_target;
	int16_t rotate_target;
	int16_t rotate_target360;
}ChassisSpeed_Target_t;

//------------------------------------------------------------
//�ǽӿں���
//------------------------------------------------------------
void CommandVariableInit(void);
void RcCmdPrc(void);
void SetInputMode(void);
//ң������������
void RemoteCmdPrc(Remote_Data_t *rc);
void RemoteShootCmdPrc(Remote_Data_t *sw);
//������������
void MouseKeyCmdPrc(Mouse_Data_t *mouse, Key_Data_t *key);
void MouseShootControl(Mouse_Data_t *mouse);
void GimbalAngleLimit(void);
void MouseShootControl(Mouse_Data_t *mouse);
//------------------------------------------------------------
//�ӿں���
//------------------------------------------------------------
InputMode_e GetInputMode(void);
Gimbal_Target_t GetGimbalTarget(void);
ChassisSpeed_Target_t GetChassisSpeedTarget(void);

#endif
