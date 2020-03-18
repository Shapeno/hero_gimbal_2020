#include "command_task.h"
/*****************************************************************************
*  @file     command_task.c                                                  *
*  @brief    通过遥控器数据做出命令                                          *
*  由CommandVariableInit()初始化相关参数,RcCmdPrc()在CMD_Task中以周期7ms运行 *
*  实现选择命令模式,获取云台目标,获取底盘速度,获取射击控制命令的功能         *
*                                                                            *
*  @author   DENG		                                                     *
*  @version  1.0.1.1		                                                 *
*  @date     19/9/6				                                             *
*                                                                            *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2019/09/18 | 1.0.1.1   | DengXY	       | Function test                   *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/
#include "sys.h"
#include "bsp.h"

#include "usart6.h"
#include "ramp.h"
#include "pid_regulator.h"
#include "control_task.h"

///	@brief	输入模式
static InputMode_e input_mode = REMOTE_INPUT;
	InputMode_e GetInputMode(void){return REMOTE_INPUT;}
static InputMode_e input_mode_last = REMOTE_INPUT;
///	@brief	云台目标位置
static Gimbal_Target_t	Gimbal_Target = {0};
	Gimbal_Target_t GetGimbalTarget(void){return Gimbal_Target;}
///	@brief	底盘目标速度
static ChassisSpeed_Target_t ChassisSpeed_Target = {0};
	ChassisSpeed_Target_t GetChassisSpeedTarget(void){return ChassisSpeed_Target;}



/**
@brief 命令变量初始化
*/
void CommandVariableInit(void){
	ChassisSpeed_Target.forward_back_target = 0.0f;
	ChassisSpeed_Target.left_right_target   = 0.0f;
	ChassisSpeed_Target.rotate_target       = 0.0f;
	Gimbal_Target.pitch_angle_target = 0.0f;
	Gimbal_Target.yaw_angle_target   = 0.0f;
}

/**
@brief 控制命令程序
*/
void RcCmdPrc(void){
	RC_Data_t rc_data;
	rc_data=GetRcData();
	#if Monitor_Remoter == 1
//	printf("ch0:%d\t", GetRcData().rc.ch[0]);
//	printf("ch1:%d\t", GetRcData().rc.ch[1]);
//	printf("ch2:%d\t", GetRcData().rc.ch[2]);
//	printf("ch3:%d\t", GetRcData().rc.ch[3]);
//	printf("ch4:%d\t", GetRcData().rc.ch[4]);
//	printf("s1U:%u\t", GetRcData().rc.s1.up);
//	printf("s1M:%u\t", GetRcData().rc.s1.mid);
//	printf("s1D:%u\t", GetRcData().rc.s1.down);
//	printf("s2U:%u\t", GetRcData().rc.s2.up);
//	printf("s2M:%u\t", GetRcData().rc.s2.mid);
//	printf("s2D:%u\r\n", GetRcData().rc.s2.down);

//	printf("x:%d\t", GetRcData().mouse.x);
//	printf("y:%d\t", GetRcData().mouse.y);
//	printf("z:%d\t", GetRcData().mouse.z);
//	printf("l:%u\t", GetRcData().mouse.press_l);
//	printf("r:%u\t", GetRcData().mouse.press_r);
//	printf("W:%d\t", GetRcData().key.W);
//	printf("S:%d\t", GetRcData().key.S);
//	printf("A:%d\t", GetRcData().key.A);
//	printf("D:%d\t", GetRcData().key.D);
//	printf("SHIFT:%d\t", GetRcData().key.SHIFT);
//	printf("CTRL:%d\t", GetRcData().key.CONTRL);
//	printf("Q:%d\t", GetRcData().key.Q);
//	printf("E:%d\t", GetRcData().key.E);
//	printf("R:%d\t", GetRcData().key.R);
//	printf("F:%d\t", GetRcData().key.F);
//	printf("G:%d\t", GetRcData().key.G);
//	printf("Z:%d\t", GetRcData().key.Z);
//	printf("X:%d\t", GetRcData().key.X);
//	printf("C:%d\t", GetRcData().key.C);
//	printf("V:%d\t", GetRcData().key.V);
//	printf("B:%d\r\n", GetRcData().key.B);
	#endif
	SetInputMode();
	switch(GetInputMode()){
		case REMOTE_INPUT:
			RemoteCmdPrc(&rc_data.rc);break;
		case KEY_MOUSE_INPUT:
			MouseKeyCmdPrc(&rc_data.mouse,&rc_data.key);break;
		default:break;
	}
}
/**
@brief 选择遥控器输入模式
*/
void SetInputMode(void){
	input_mode_last=input_mode;
	if (GetRcData().rc.s2.up)input_mode=REMOTE_INPUT;
	if (GetRcData().rc.s2.mid)input_mode=KEY_MOUSE_INPUT;
	if (input_mode!=input_mode_last){	///<输入模式发生改变
		
	}
}
/**
@brief 遥控器命令程序
*/
void RemoteCmdPrc(Remote_Data_t *rc){
	ChassisSpeed_Target.forward_back_target = rc->ch[3] * STICK_TO_CHASSIS_SPEED_REF_FACT;
	ChassisSpeed_Target.left_right_target   = rc->ch[2] * STICK_TO_CHASSIS_SPEED_REF_FACT;
	Gimbal_Target.pitch_angle_target += rc->ch[1] * STICK_TO_PITCH_ANGLE_INC_FACT;
	Gimbal_Target.yaw_angle_target   += rc->ch[0] * STICK_TO_YAW_ANGLE_INC_FACT;
	GimbalAngleLimit();
	if(rc->s2.down){
	Gimbal_Target.pitch_angle_target =10;
	Gimbal_Target.yaw_angle_target   =0;	
	}
	
	RemoteShootCmdPrc(rc);
}

/**
@brief 遥控器射击命令程序
*/
void RemoteShootCmdPrc(Remote_Data_t *rc){	
	if(rc->s1.up == 1){
		SetSingleShootMode(false);
		StopBigFric();
		ShootCMD(false);
		LASER_OFF();
	}
	else if(rc->s1.mid == 1){
		SetSingleShootMode(true);
		LASER_ON();
		StartBigFric();
		ShootCMD(false);
	}
	else if(rc->s1.down == 1){
		SetSingleShootMode(false);
		StartBigFric();
		ShootCMD(true);
		LASER_ON();
	}
}

/**
@brief 键鼠命令程序
*/
void MouseKeyCmdPrc(Mouse_Data_t *mouse, Key_Data_t *key){
	///<底盘目标速度
	if(key->SHIFT){
		ChassisSpeed_Target.forward_back_target	=	key->W * HIGH_FORWARD_BACK_SPEED;
		ChassisSpeed_Target.forward_back_target	=	-key->S * HIGH_FORWARD_BACK_SPEED;
		ChassisSpeed_Target.left_right_target	=	key->D * HIGH_LEFT_RIGHT_SPEED;
		ChassisSpeed_Target.left_right_target	=	-key->A * HIGH_LEFT_RIGHT_SPEED;
	}
	else{
		ChassisSpeed_Target.forward_back_target	=	key->W * NORMAL_FORWARD_BACK_SPEED;
		ChassisSpeed_Target.forward_back_target	=	-key->S * NORMAL_FORWARD_BACK_SPEED;
		ChassisSpeed_Target.left_right_target	=	key->D * NORMAL_LEFT_RIGHT_SPEED;
		ChassisSpeed_Target.left_right_target	=	-key->A * NORMAL_LEFT_RIGHT_SPEED;
	}
	///<云台目标位置
	VAL_LIMIT(mouse->x, -150, 150);
	VAL_LIMIT(mouse->y, -150, 150);
	Gimbal_Target.pitch_angle_target -= mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
	Gimbal_Target.yaw_angle_target   += mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
	GimbalAngleLimit();
	
	MouseShootControl(mouse);
}

/**
@brief 云台角度限制
*/
void GimbalAngleLimit(void){
	VAL_LIMIT(Gimbal_Target.pitch_angle_target, PITCH_MIN, PITCH_MAX); 
}

/**
@brief 键鼠射击命令程序
*/
void MouseShootControl(Mouse_Data_t *mouse){
}


