#include "control_task.h"
/*****************************************************************************
*  @file     control_task.c                                                  *
*  @brief    对设备进行控制                                                  *
*  ControlVariableInit()初始化相关参数,ControlPrc()在Control_Task中          *
*  以周期1ms运行                                                             *
*  实现选择控制模式,解算各设备PID,发射机构的控制,以及向command_task提供控制  *
*  接口                                                                      *
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

/// @brief PID初始化
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

/// @brief 斜坡函数初始化
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;///<云台启动斜坡函数
RampGen_t GMYawRamp   = RAMP_GEN_DAFAULT;
RampGen_t CMRamp      = RAMP_GEN_DAFAULT;

/// @brief	运行状态接口函数
static WorkState_e work_state_last = PREPARE_STATE;
static WorkState_e work_state     = PREPARE_STATE;
	void SetWorkState(WorkState_e state){if(work_state!=PREPARE_STATE)work_state=state;}
	WorkState_e GetWorkState(void){return work_state;}
/// @biref	启动大摩擦轮
static bool big_fric=false;
	void StartBigFric(void){big_fric=true;}
	void StopBigFric(void){big_fric=false;}
/// @biref	单发射击模式
static bool single_shoot_mode=false;
	void SetSingleShootMode(bool flag){single_shoot_mode=flag;}
	
/// @biref	射击命令(单发模式下需要置零后才能射击)
static bool shoot=false;
	void ShootCMD(bool flag){shoot=flag;}
	
/// @brief	拨盘反转
static int16_t volatile rammer_inversion = 0;
	void SetRammerInversionTime(int16_t t){rammer_inversion = t;}
	
/// @brief	摩擦轮启动成功标志，如果为假则拨盘不允许启动
static bool fric_accel_ok=false;
	void SetFricAccelOK(bool flag){fric_accel_ok=flag;}

	
/// @brief	摩擦轮的PWM占空比设定(snail电机专用)
static uint16_t friction_wheel_max_duty;
	uint16_t GetFricWheelMaxDuty(void){return friction_wheel_max_duty;}
	
/// @brief	摩擦轮启动计数(snail电机专用)
static uint16_t friction_counter;
	
/// @brief  枪口热量限制
static volatile uint16_t HeatLimit  = 90;
	
/// @brief  枪口射速限制
static volatile uint8_t  ShootSpeed = 16;
	
/// @brief  射频
static volatile int16_t ShootFre   = 1600;

	
/// @brief  控制变量初始化
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

/// @brief 控制主程序
void ControlPrc(void){
	GimbalControlModeSwitch();
	GMPitchControlLoop();
	GMYawControlLoop();
	GimbalMotorOutput();
  	CMControlLoop();
	BigBulletFric_Control_Prc();
	BigBulletRammer_Control_Prc();
}

/// @brief 云台不同模式控制
void GimbalControlModeSwitch(void){
	work_state_last=work_state;
	static float YawAngleBias = 0.0f;
	if(work_state!=work_state_last)
		WorkStateChange();
	switch(GetWorkState()){
		case PREPARE_STATE:{
			GMYPositionPID.ref = 0.0f;
			GMYPositionPID.fdb = -(GetMotorData(YAW_MOTOR).angle+360*GetMotorData(YAW_MOTOR).cycles)* GMYawRamp.Calc(&GMYawRamp);
			YawAngleBias = 0;//imu_yaw_angle;				///< @brief 获取陀螺仪初始位置
			if(xTaskGetTickCount() > 4000)	///< @brief 让云台进入陀螺仪控制
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
@brief 运行状态改变时运行
*/
void WorkStateChange(void){
	
}

/** 
@brief Pitch控制环
*/
void GMPitchControlLoop(void){
	GMPPositionPID.ref = GetGimbalTarget().pitch_angle_target;
	GMPPositionPID.fdb = (GetMotorData(PIT_MOTOR).angle+360*GetMotorData(PIT_MOTOR).cycles) * GMPitchRamp.Calc(&GMPitchRamp);
	GMPPositionPID.Calc(&GMPPositionPID);
}

/** 
@brief Yaw控制环
*/
void GMYawControlLoop(void){
	GMYPositionPID.Calc(&GMYPositionPID);
}

/** 
@brief 云台电机输出
*/
void GimbalMotorOutput(void){
	SetMotorCurrent(YAW_MOTOR,GMYPositionPID.output);
	SetMotorCurrent(PIT_MOTOR,GMPPositionPID.output);
}


/** 
@brief 底盘控制程序
*/
void CMControlLoop(void){

}


/** 
@brief 射击限制的选择(通过裁判系统读取等级，根据规则自己设定)
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
@brief 大弹丸的拨弹轮控制,必须在控制主程序内运行
*/
void BigBulletRammer_Control_Prc(void){
	BigRammerPID.ref=-36*10;
	BigRammerPID.fdb=GetMotorData(RAMMER_MOTOR).speed;
	BigRammerPID.Calc(&BigRammerPID);
	SetMotorCurrent(RAMMER_MOTOR,BigRammerPID.output);
	//SendMotorCurrent(RAMMER_MOTOR);
	

}
/** 
@brief 大弹丸的摩擦轮控制,必须在控制主程序内运行
*/
void BigBulletFric_Control_Prc(void){
	
	static bool first_stop=false;///<第一次进入摩擦轮停止环时,获取当前摩擦轮位置
	static bool on_shooting=false;
	static bool bullet_stucked_after_boot_up=true;///<刚开机时如果弹腔(限位开关上有弹)为了安全,不会加前推量,所以第一发大弹丸可能延迟比较高
	static bool shoot_last;///<上一次射击命令
	static bool limit_switch;///<限位开关是否按下
	static bool limit_switch_last;///<限位开关上一次状态
	
	limit_switch=LIMIT_SWITCH;
	///<垂直摩擦轮控制
//	if(single_shoot_mode==true){///<单发模式
//		if(on_shooting==false){
////			if(shoot==false){
//			{
//				if(limit_switch==LIMIT_SWITCH_OFF){///<控制速度把弹挤到限位开关
//					VFric_SpeedPID.ref=36*40;
//					first_stop=false;
//					bullet_stucked_after_boot_up=false;///<开机未卡弹
//				}
//				else if(limit_switch_last==LIMIT_SWITCH_ON){///<控制位置把弹卡住
//					if(first_stop==false){
//						VFric_PositionPID.ref = GetMotorData(FRIC0_MOTOR).angle + GetMotorData(FRIC0_MOTOR).cycles*360;
//						if(bullet_stucked_after_boot_up==false)VFric_PositionPID.ref += 360*6;///<若开机未卡弹则加前推量
//						first_stop=true;
//					}
//					else{
//						VFric_PositionPID.fdb=GetMotorData(FRIC0_MOTOR).angle + GetMotorData(FRIC0_MOTOR).cycles*360;
//						VFric_PositionPID.Calc(&VFric_PositionPID);
//						VFric_SpeedPID.ref=VFric_PositionPID.output;
//					}
//				}
//			}
//			if(shoot==true){///<开启发射
//				if(shoot_last==false)
//				on_shooting=true;
//			}
//		}
//	}
//	
//	else if(single_shoot_mode==false){///<连发模式
//		if(shoot==false){
//			if(limit_switch==LIMIT_SWITCH_OFF){///<控制速度把弹挤到限位开关
//				VFric_SpeedPID.ref=36*40;
//				first_stop=false;
//				bullet_stucked_after_boot_up=false;///<开机未卡弹
//			}
//			else if(limit_switch_last==LIMIT_SWITCH_ON){///<控制位置把弹卡住
//				if(first_stop==false){
//					VFric_PositionPID.ref=GetMotorData(FRIC0_MOTOR).angle + GetMotorData(FRIC0_MOTOR).cycles*360;
//					if(bullet_stucked_after_boot_up==false)VFric_PositionPID.ref += 360*6;///<若开机未卡弹则加前推量
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
//	if(on_shooting){///<把弹推到摩擦轮
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
	
	///<水平摩擦轮控制
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
	///<更新变量
	shoot_last=shoot;
	limit_switch_last=limit_switch;
}


/**
	@brief PID在线调参
	@para1 	x=1 参数Kp
			x=2 参数Ki
			x=3 参数Kd
	@para2	PLUS函数对应参数增加y/100
			MIN函数对应参数减小y/100
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
