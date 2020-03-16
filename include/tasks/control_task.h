#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_

#include "sys.h"
//��̨���PID����(λ�û�)
#define YAW_POSITION_KP_DEFAULTS    1300
#define YAW_POSITION_KI_DEFAULTS    0
#define YAW_POSITION_KD_DEFAULTS    1000

#define INTEGRAL_THRESHOLD			1.0
#define INTEGRAL_SLOPE				0.5

#define PITCH_POSITION_KP_DEFAULTS  5000
#define PITCH_POSITION_KI_DEFAULTS  0
#define PITCH_POSITION_KD_DEFAULTS  1000
//С���貦���������(��Ҫ�޸�)
#define RAMMER_SPEED_KP_DEFAULTS    15
#define RAMMER_SPEED_KI_DEFAULTS    0
#define RAMMER_SPEED_KD_DEFAULTS    5

#define RAMMER_POSITION_KP_DEFAULTS 150
#define RAMMER_POSITION_KI_DEFAULTS 0.15
#define RAMMER_POSITION_KD_DEFAULTS 6

//���貦���������
#define BIG_RAMMER_SPEED_KP_DEFAULTS    10
#define BIG_RAMMER_SPEED_KI_DEFAULTS    0
#define BIG_RAMMER_SPEED_KD_DEFAULTS    0

//Ħ����PID����(ˮƽ)
#define HFRICTION1_SPEED_KP_DEFAULTS 100
#define HFRICTION1_SPEED_KI_DEFAULTS 0
#define HFRICTION1_SPEED_KD_DEFAULTS 0

#define HFRICTION2_SPEED_KP_DEFAULTS 100
#define HFRICTION2_SPEED_KI_DEFAULTS 0
#define HFRICTION2_SPEED_KD_DEFAULTS 0
//Ħ����PID����(��ֱ)
#define VFRICTION0_SPEED_KP_DEFAULTS 10
#define VFRICTION0_SPEED_KI_DEFAULTS 0
#define VFRICTION0_SPEED_KD_DEFAULTS 5

#define VFRICTION0_POSITION_KP_DEFAULTS 10
#define VFRICTION0_POSITION_KI_DEFAULTS 0
#define VFRICTION0_POSITION_KD_DEFAULTS 50
//�����ٶ�(m/s)���㵽Ħ����ת���ٶ�(rpm)1.357*
#define BULLET_SPEED_TO_MOTOR_SPEED	(272.837f)

/// @brief ��̨�Ŀ���ģʽ
typedef enum{
	PREPARE_STATE,	///<׼��ģʽ
	NORMAL_STATE,	///<����ģʽ
	AUTOAIM_STATE,	///<����ģʽ
	BS_STATE,		///<���ģʽ
	STOP_STATE,		///<ֹͣģʽ
}WorkState_e;

#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	4.50f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	PITCH_POSITION_KP_DEFAULTS,\
	PITCH_POSITION_KI_DEFAULTS,\
	PITCH_POSITION_KD_DEFAULTS,\
	1,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	30000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	PITCH_SPEED_KP_DEFAULTS,\
	PITCH_SPEED_KI_DEFAULTS,\
	PITCH_SPEED_KD_DEFAULTS,\
	1,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	15000,\
	12,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	YAW_POSITION_KP_DEFAULTS,\
	YAW_POSITION_KI_DEFAULTS,\
	YAW_POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	YAW_SPEED_KP_DEFAULTS,\
	YAW_SPEED_KI_DEFAULTS,\
	YAW_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	4900,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define RAMMER_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	RAMMER_SPEED_KP_DEFAULTS,\
	RAMMER_SPEED_KI_DEFAULTS,\
	RAMMER_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	10000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define RAMMER_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	RAMMER_POSITION_KP_DEFAULTS,\
	RAMMER_POSITION_KI_DEFAULTS,\
	RAMMER_POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	10000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define BIG_RAMMER_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	BIG_RAMMER_SPEED_KP_DEFAULTS,\
	BIG_RAMMER_SPEED_KI_DEFAULTS,\
	BIG_RAMMER_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	10000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define HFRICTION1_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	HFRICTION1_SPEED_KP_DEFAULTS,\
	HFRICTION1_SPEED_KI_DEFAULTS,\
	HFRICTION1_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	30000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define HFRICTION2_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	HFRICTION2_SPEED_KP_DEFAULTS,\
	HFRICTION2_SPEED_KI_DEFAULTS,\
	HFRICTION2_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	30000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define VFRICTION0_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	VFRICTION0_SPEED_KP_DEFAULTS,\
	VFRICTION0_SPEED_KI_DEFAULTS,\
	VFRICTION0_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	11000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define VFRICTION0_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	VFRICTION0_POSITION_KP_DEFAULTS,\
	VFRICTION0_POSITION_KI_DEFAULTS,\
	VFRICTION0_POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	11000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}
///���ƺ���
void ControlVariableInit(void);
void ControlPrc(void);
void GimbalControlModeSwitch(void);
void WorkStateChange(void);
void GMPitchControlLoop(void);
void GMYawControlLoop(void);
void GimbalMotorOutput(void);
void CMControlLoop(void);
void ShootLimitSwitch(void);
///�ӿں���
void StartBigFric(void);
void StopBigFric(void);
void SetWorkState(WorkState_e state);
WorkState_e GetWorkState(void);
void SetSingleShootMode(bool flag);
void ShootCMD(bool flag);
void SetRammerInversionTime(int16_t t);
void SetFricAccelOK(bool flag);
uint16_t GetFricWheelMaxDuty(void);
void SmallBulletFirc_Start_Prc(void);
void SmallBulletFirc_Stop_Prc(void);
void BigBulletRammer_Control_Prc(void);
void BigBulletFric_Control_Prc(void);

/*debug fuctons of PID
PID���ߵ���
*/
void GMP_PID_PLUS(int x,int y);
void GMP_PID_MIN(int x,int y);
void GMY_PID_PLUS(int x,int y);
void GMY_PID_MIN(int x,int y);
void FRIC_PID_PLUS(int x,int y);
void FRIC_PID_MIN(int x,int y);

#endif
