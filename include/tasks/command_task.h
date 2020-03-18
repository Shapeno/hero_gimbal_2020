#ifndef _COMMAND_TASK_H_
#define _COMMAND_TASK_H_

#include "sys.h"
#include "dbus.h"

///	@brief	pitch轴角度限制
#define PITCH_MAX 15.0f
#define PITCH_MIN -30.0f
///	@brief	遥控器控制的一些定义
#define STICK_TO_CHASSIS_SPEED_REF_FACT     0.30f
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.004f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.002f
#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		  -0.025f
#define MOUSE_TO_YAW_ANGLE_INC_FACT 		    -0.025f
///	@brief	底盘不同模式速度
#define NORMAL_FORWARD_BACK_SPEED 			    207
#define NORMAL_LEFT_RIGHT_SPEED   			    207
#define HIGH_FORWARD_BACK_SPEED 			      330
#define HIGH_LEFT_RIGHT_SPEED   			      230
#define SUPER_HIGH_FORWARD_BACK_SPEED 			414
#define SUPER_HIGH_LEFT_RIGHT_SPEED   			414

///	@brief	输入模式枚举
typedef enum{
	REMOTE_INPUT    = 1,///<遥控输入
	KEY_MOUSE_INPUT = 3,///<键盘输入
	STOP            = 2,
}InputMode_e;

///	@brief	云台目标位置结构体
typedef struct{
	float pitch_angle_target;
	float yaw_angle_target;
}Gimbal_Target_t;

///	@brief	底盘目标速度结构体
typedef __packed struct{
	int16_t forward_back_target;
	int16_t left_right_target;
	int16_t rotate_target;
	int16_t rotate_target360;
}ChassisSpeed_Target_t;

//------------------------------------------------------------
//非接口函数
//------------------------------------------------------------
void CommandVariableInit(void);
void RcCmdPrc(void);
void SetInputMode(void);
//遥控器命令处理相关
void RemoteCmdPrc(Remote_Data_t *rc);
void RemoteShootCmdPrc(Remote_Data_t *sw);
//键鼠命令处理相关
void MouseKeyCmdPrc(Mouse_Data_t *mouse, Key_Data_t *key);
void MouseShootControl(Mouse_Data_t *mouse);
void GimbalAngleLimit(void);
void MouseShootControl(Mouse_Data_t *mouse);
//------------------------------------------------------------
//接口函数
//------------------------------------------------------------
InputMode_e GetInputMode(void);
Gimbal_Target_t GetGimbalTarget(void);
ChassisSpeed_Target_t GetChassisSpeedTarget(void);

#endif
