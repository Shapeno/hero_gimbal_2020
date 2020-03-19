#include "control_task.h"
/*****************************************************************************
*  @file     control_task.c                                                  *
*  @brief    ���豸���п���                                                  *
*  ControlVariableInit()��ʼ����ز���,ControlPrc()��Control_Task��          *
*  ������1ms����                                                             *
*  ʵ��ѡ�����ģʽ,������豸PID,��������Ŀ���,�Լ���command_task�ṩ����  *
*  �ӿ�                                                                      *
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
#include "gun.h"

#include "ramp.h"
#include "pid_regulator.h"

#include "judge_task.h"
#include "command_task.h"
#include "can_task.h"
#include "main.h"

/// @brief PID��ʼ��
PID_Regulator_t GMPPositionPID    = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;     
//PID_Regulator_t GMPSpeedPID       = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID    = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;	
//PID_Regulator_t GMYSpeedPID       = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;
PID_Regulator_t RAMMERSpeedPID    = RAMMER_SPEED_PID_DEFAULT;
PID_Regulator_t RAMMERPositionPID = RAMMER_POSITION_PID_DEFAULT;
PID_Regulator_t CMRotatePID       = CHASSIS_MOTOR_ROTATE_PID_DEFAULT;
PID_Regulator_t BigRammerPID   =  BIG_RAMMER_SPEED_PID_DEFAULT;
PID_Regulator_t HFric1_SpeedPID   = HFRICTION1_SPEED_PID_DEFAULT;
PID_Regulator_t HFric2_SpeedPID   = HFRICTION2_SPEED_PID_DEFAULT;
PID_Regulator_t VFric_SpeedPID   = VFRICTION0_SPEED_PID_DEFAULT;
PID_Regulator_t VFric_PositionPID   = VFRICTION0_POSITION_PID_DEFAULT;

/// @brief б�º�����ʼ��
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;///<��̨����б�º���
RampGen_t GMYawRamp   = RAMP_GEN_DAFAULT;
RampGen_t CMRamp      = RAMP_GEN_DAFAULT;

/// @brief	����״̬�ӿں���
static WorkState_e work_state_last = PREPARE_STATE;
static WorkState_e work_state     = PREPARE_STATE;
	void SetWorkState(WorkState_e state){if(work_state!=PREPARE_STATE)work_state=state;}
	WorkState_e GetWorkState(void){return work_state;}
/// @biref	������Ħ����
static bool big_fric=false;
	void StartBigFric(void){big_fric=true;}
	void StopBigFric(void){big_fric=false;}
/// @biref	�������ģʽ
static bool single_shoot_mode=false;
	void SetSingleShootMode(bool flag){single_shoot_mode=flag;}
	
/// @biref	�������(����ģʽ����Ҫ�����������)
static bool shoot=false;
	void ShootCMD(bool flag){shoot=flag;}
	
/// @brief	���̷�ת
static int16_t volatile rammer_inversion = 0;
	void SetRammerInversionTime(int16_t t){rammer_inversion = t;}
	
/// @brief	Ħ���������ɹ���־�����Ϊ�����̲���������
static bool fric_accel_ok=false;
	void SetFricAccelOK(bool flag){fric_accel_ok=flag;}

	
/// @brief	Ħ���ֵ�PWMռ�ձ��趨(snail���ר��)
static uint16_t friction_wheel_max_duty;
	uint16_t GetFricWheelMaxDuty(void){return friction_wheel_max_duty;}
	
/// @brief	Ħ������������(snail���ר��)
static uint16_t friction_counter;
	
/// @brief  ǹ����������
static volatile uint16_t HeatLimit  = 90;
	
/// @brief  ǹ����������
static volatile uint8_t  ShootSpeed = 16;
	
/// @brief  ��Ƶ
static volatile int16_t ShootFre   = 1600;

	
/// @brief  ���Ʊ�����ʼ��
void ControlVariableInit(void){
	SetWorkState(PREPARE_STATE);
	
	GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_MS);
	GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_MS);
	CMRamp.SetScale(&CMRamp, PREPARE_TIME_MS);
	
	GMPitchRamp.ResetCounter(&GMPitchRamp);
	GMYawRamp.ResetCounter(&GMYawRamp);
	CMRamp.ResetCounter(&CMRamp);
	
	BigRammerPID.Reset(&BigRammerPID);
	RAMMERSpeedPID.Reset(&RAMMERSpeedPID);
	GMPPositionPID.Reset(&GMPPositionPID);
	GMYPositionPID.Reset(&GMYPositionPID);
	CMRotatePID.Reset(&CMRotatePID);
}

/// @brief ����������
void ControlPrc(void){
	GimbalControlModeSwitch();
	GMPitchControlLoop();
	GMYawControlLoop();
	GimbalMotorOutput();
  	CMControlLoop();
	BigBulletFric_Control_Prc();
	BigBulletRammer_Control_Prc();
}

/// @brief ��̨��ͬģʽ����
void GimbalControlModeSwitch(void){
	work_state_last=work_state;
	static float YawAngleBias = 0.0f;
	if(work_state!=work_state_last)
		WorkStateChange();
	switch(GetWorkState()){
		case PREPARE_STATE:{
			GMYPositionPID.ref = 0.0f;
			GMYPositionPID.fdb = -(GetMotorData(YAW_MOTOR).angle+360*GetMotorData(YAW_MOTOR).cycles)* GMYawRamp.Calc(&GMYawRamp);
			YawAngleBias = 0;//imu_yaw_angle;				///< @brief ��ȡ�����ǳ�ʼλ��
			if(xTaskGetTickCount() > 4000)	///< @brief ����̨���������ǿ���
				work_state = NORMAL_STATE;
		}break;
		case NORMAL_STATE:{
			if((GetGimbalTarget().yaw_angle_target-GMYPositionPID.ref)>INTEGRAL_THRESHOLD)
				GMYPositionPID.ref += INTEGRAL_SLOPE;
			else if((GetGimbalTarget().yaw_angle_target-GMYPositionPID.ref)<-INTEGRAL_THRESHOLD)
				GMYPositionPID.ref -= INTEGRAL_SLOPE;
			else GMYPositionPID.ref=GetGimbalTarget().yaw_angle_target;
//			GMYPositionPID.fdb = -(imu_yaw_angle-YawAngleBias);
			GMYPositionPID.fdb = -(GetMotorData(YAW_MOTOR).angle+360*GetMotorData(YAW_MOTOR).cycles);
		}break;
		case AUTOAIM_STATE:{}break;
		case BS_STATE:{}break;
		case STOP_STATE:{}break;
	}
}

/** 
@brief ����״̬�ı�ʱ����
*/
void WorkStateChange(void){
	
}

/** 
@brief Pitch���ƻ�
*/
void GMPitchControlLoop(void){
	GMPPositionPID.ref = GetGimbalTarget().pitch_angle_target;
	GMPPositionPID.fdb = (GetMotorData(PIT_MOTOR).angle+360*GetMotorData(PIT_MOTOR).cycles) * GMPitchRamp.Calc(&GMPitchRamp);
	GMPPositionPID.Calc(&GMPPositionPID);
}

/** 
@brief Yaw���ƻ�
*/
void GMYawControlLoop(void){
	GMYPositionPID.Calc(&GMYPositionPID);
}

/** 
@brief ��̨������
*/
void GimbalMotorOutput(void){
	SetMotorCurrent(YAW_MOTOR,GMYPositionPID.output);
	SetMotorCurrent(PIT_MOTOR,GMPPositionPID.output);
}


/** 
@brief ���̿��Ƴ���
*/
void CMControlLoop(void){

}


/** 
@brief ������Ƶ�ѡ��(ͨ������ϵͳ��ȡ�ȼ������ݹ����Լ��趨)
*/
void ShootLimitSwitch(void){
	switch (JUDGE_GET_Hero_LevelData()){
		case 1:{
			HeatLimit = 90;
			ShootSpeed = 19;
			friction_wheel_max_duty= 1250;
			ShootFre = 2160;
		}break;
		case 2:{
			HeatLimit = 180;
			ShootSpeed = 22;
			friction_wheel_max_duty= 1400;
			ShootFre = 3240;
		}break;
		case 3:{
			HeatLimit = 360;
			ShootSpeed = 25;
			friction_wheel_max_duty= 1500;
			ShootFre =4000;
		}break;
		default:{
			HeatLimit = 90;
			ShootSpeed = 19;
			friction_wheel_max_duty= 1500;
			ShootFre =4000;
		}break;
	}
}

/** 
@brief ����Ĳ����ֿ���,�����ڿ���������������
*/
void BigBulletRammer_Control_Prc(void){
	BigRammerPID.ref=-36*10;
	BigRammerPID.fdb=GetMotorData(RAMMER_MOTOR).speed;
	BigRammerPID.Calc(&BigRammerPID);
	SetMotorCurrent(RAMMER_MOTOR,BigRammerPID.output);
	//SendMotorCurrent(RAMMER_MOTOR);
	

}
/** 
@brief �����Ħ���ֿ���,�����ڿ���������������
*/
void BigBulletFric_Control_Prc(void){
	
	static bool first_stop=false;///<��һ�ν���Ħ����ֹͣ��ʱ,��ȡ��ǰĦ����λ��
	static bool on_shooting=false;
	static bool bullet_stucked_after_boot_up=true;///<�տ���ʱ�����ǻ(��λ�������е�)Ϊ�˰�ȫ,�����ǰ����,���Ե�һ����������ӳٱȽϸ�
	static bool shoot_last;///<��һ���������
	static bool limit_switch;///<��λ�����Ƿ���
	static bool limit_switch_last;///<��λ������һ��״̬
	
	limit_switch=LIMIT_SWITCH;
	///<��ֱĦ���ֿ���
//	if(single_shoot_mode==true){///<����ģʽ
//		if(on_shooting==false){
////			if(shoot==false){
//			{
//				if(limit_switch==LIMIT_SWITCH_OFF){///<�����ٶȰѵ�������λ����
//					VFric_SpeedPID.ref=36*40;
//					first_stop=false;
//					bullet_stucked_after_boot_up=false;///<����δ����
//				}
//				else if(limit_switch_last==LIMIT_SWITCH_ON){///<����λ�ðѵ���ס
//					if(first_stop==false){
//						VFric_PositionPID.ref = GetMotorData(FRIC0_MOTOR).angle + GetMotorData(FRIC0_MOTOR).cycles*360;
//						if(bullet_stucked_after_boot_up==false)VFric_PositionPID.ref += 360*6;///<������δ�������ǰ����
//						first_stop=true;
//					}
//					else{
//						VFric_PositionPID.fdb=GetMotorData(FRIC0_MOTOR).angle + GetMotorData(FRIC0_MOTOR).cycles*360;
//						VFric_PositionPID.Calc(&VFric_PositionPID);
//						VFric_SpeedPID.ref=VFric_PositionPID.output;
//					}
//				}
//			}
//			if(shoot==true){///<��������
//				if(shoot_last==false)
//				on_shooting=true;
//			}
//		}
//	}
//	
//	else if(single_shoot_mode==false){///<����ģʽ
//		if(shoot==false){
//			if(limit_switch==LIMIT_SWITCH_OFF){///<�����ٶȰѵ�������λ����
//				VFric_SpeedPID.ref=36*40;
//				first_stop=false;
//				bullet_stucked_after_boot_up=false;///<����δ����
//			}
//			else if(limit_switch_last==LIMIT_SWITCH_ON){///<����λ�ðѵ���ס
//				if(first_stop==false){
//					VFric_PositionPID.ref=GetMotorData(FRIC0_MOTOR).angle + GetMotorData(FRIC0_MOTOR).cycles*360;
//					if(bullet_stucked_after_boot_up==false)VFric_PositionPID.ref += 360*6;///<������δ�������ǰ����
//					first_stop=true;
//				}
//				else{
//					VFric_PositionPID.fdb=GetMotorData(FRIC0_MOTOR).angle + GetMotorData(FRIC0_MOTOR).cycles*360;
//					VFric_PositionPID.Calc(&VFric_PositionPID);
//					VFric_SpeedPID.ref=VFric_PositionPID.output;
//				}
//			}
//		}
//		else{
//			VFric_SpeedPID.ref=36*60;
//		}
//	}
//	
//	if(on_shooting){///<�ѵ��Ƶ�Ħ����
//		VFric_SpeedPID.ref=36*60*2;
//		if(limit_switch==true)
//			if(limit_switch_last==false){
//				on_shooting=false;
//			}
//	}
//	VFric_SpeedPID.fdb=GetMotorData(FRIC0_MOTOR).speed;
//	VFric_SpeedPID.Calc(&VFric_SpeedPID);
//	SetMotorCurrent(FRIC0_MOTOR,VFric_SpeedPID.output);
	if(shoot){
		VFric_SpeedPID.ref=36*60*8;
	}
	else{
		VFric_SpeedPID.ref=0;
	}
	VFric_SpeedPID.fdb=GetMotorData(FRIC0_MOTOR).speed;
	VFric_SpeedPID.Calc(&VFric_SpeedPID);
	SetMotorCurrent(FRIC0_MOTOR,VFric_SpeedPID.output);
	
	///<ˮƽĦ���ֿ���
	if(big_fric){
		HFric1_SpeedPID.ref=-ShootSpeed*BULLET_SPEED_TO_MOTOR_SPEED;
		HFric2_SpeedPID.ref=ShootSpeed*BULLET_SPEED_TO_MOTOR_SPEED;
	}
	else {
		HFric1_SpeedPID.ref=0;
		HFric2_SpeedPID.ref=0;
	}
	HFric1_SpeedPID.fdb=GetMotorData(FRIC1_MOTOR).speed;
	HFric2_SpeedPID.fdb=GetMotorData(FRIC2_MOTOR).speed;
	HFric1_SpeedPID.Calc(&HFric1_SpeedPID);
	HFric2_SpeedPID.Calc(&HFric2_SpeedPID);
	SetMotorCurrent(FRIC1_MOTOR,HFric1_SpeedPID.output);
	SetMotorCurrent(FRIC2_MOTOR,HFric2_SpeedPID.output);
	///<���±���
	shoot_last=shoot;
	limit_switch_last=limit_switch;
}


/**
	@brief PID���ߵ���
	@para1 	x=1 ����Kp
			x=2 ����Ki
			x=3 ����Kd
	@para2	PLUS������Ӧ��������y/100
			MIN������Ӧ������Сy/100
	*/
void Monitor_Pitch_PID(void)
{
    printf("PKP:%3.2f,PKI:%1.2f,PKD:%3.2f\r\n",
    GMPPositionPID.kp,GMPPositionPID.ki,GMPPositionPID.kd);
}
void Monitor_Yaw_PID(void)
{
    printf("PKP:%3.2f,PKI:%1.2f,PKD:%3.2f\r\n",
    GMYPositionPID.kp,GMYPositionPID.ki,GMYPositionPID.kd);
}
void FRIC_PID()
{
#if(Debug_VFric==0)
	printf("PKP:%3.2f,PKI:%1.2f,PKD:%3.2f\r\n",
    HFric1_SpeedPID.kp,HFric1_SpeedPID.ki,HFric1_SpeedPID.kd);
#endif
#if(Debug_VFric==1)
	printf("PKP:%3.2f,PKI:%1.2f,PKD:%3.2f\r\n",
    VFric_SpeedPID.kp,VFric_SpeedPID.ki,VFric_SpeedPID.kd);
#endif
}
void GMP_PID_PLUS(int x,int y)
{
    switch(x){
    case 0x1:
        GMPPositionPID.kp += (float)y/100;
        break;
    case 0x2:
        GMPPositionPID.ki += (float)y/100;
        break;
    case 0x3:
        GMPPositionPID.kd += (float)y/100;
        break;
    default :
        ;
    }
    Monitor_Pitch_PID();
}
void GMP_PID_MIN(int x,int y)
{
    switch(x){
    case 0x1:
        GMPPositionPID.kp -= (float)y/100;
        break;
    case 0x2:
        GMPPositionPID.ki -= (float)y/100;
        break;
    case 0x3:
        GMPPositionPID.kd -= (float)y/100;
        break;
    default :
        ;
    }
    Monitor_Pitch_PID();
}
void GMY_PID_PLUS(int x,int y)
{
    switch(x){
    case 0x1:
        GMYPositionPID.kp += (float)y/100;
        break;
    case 0x2:
        GMYPositionPID.ki += (float)y/100;
        break;
    case 0x3:
        GMYPositionPID.kd += (float)y/100;
        break;
    default :
        ;
    }
    Monitor_Yaw_PID();
}
void GMY_PID_MIN(int x,int y)
{
    switch(x){
    case 0x1:
        GMYPositionPID.kp -= (float)y/100;
        break;
    case 0x2:
        GMYPositionPID.ki -= (float)y/100;
        break;
    case 0x3:
        GMYPositionPID.kd -= (float)y/100;
        break;
    default :
        ;
    }
    Monitor_Yaw_PID();
}
void FRIC_PID_PLUS(int x,int y)
{
#if(Debug_VFric==0)
    switch(x){
    case 0x1:
        HFric1_SpeedPID.kp += (float)y/100;
		HFric2_SpeedPID.kp += (float)y/100;
        break;
    case 0x2:
        HFric1_SpeedPID.ki += (float)y/100;
		HFric2_SpeedPID.kp += (float)y/100;
        break;
    case 0x3:
        HFric1_SpeedPID.kd += (float)y/100;
		HFric2_SpeedPID.kp += (float)y/100;
        break;
    default :
        ;
    }
#endif
#if(Debug_VFric==1)
    switch(x){
    case 0x1:
        VFric_SpeedPID.kp += (float)y/100;
        break;
    case 0x2:
        VFric_SpeedPID.ki += (float)y/100;
        break;
    case 0x3:
        VFric_SpeedPID.kd += (float)y/100;
        break;
    default :
        ;
    }
#endif
    FRIC_PID();
}
void FRIC_PID_MIN(int x,int y)
{
#if(Debug_VFric==0)
    switch(x){
    case 0x1:
        HFric1_SpeedPID.kp -= (float)y/100;
		HFric2_SpeedPID.kp -= (float)y/100;
        break;
    case 0x2:
        HFric1_SpeedPID.ki -= (float)y/100;
		HFric2_SpeedPID.kp -= (float)y/100;
        break;
    case 0x3:
        HFric1_SpeedPID.kd -= (float)y/100;
		HFric2_SpeedPID.kp -= (float)y/100;
        break;
    default :
        ;
    }
#endif	
#if(Debug_VFric==1)
    switch(x){
    case 0x1:
        VFric_SpeedPID.kp -= (float)y/100;
        break;
    case 0x2:
        VFric_SpeedPID.ki -= (float)y/100;
        break;
    case 0x3:
        VFric_SpeedPID.kd -= (float)y/100;
        break;
    default :
        ;
    }
#endif
    FRIC_PID();
}
