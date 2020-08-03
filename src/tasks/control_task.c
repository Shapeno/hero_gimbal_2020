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
*  2020/04/04 | 1.0.2.1   | DengXY	       | 修改YAW控制逻辑，所有极性必须重 *
*			  | 	      | 		       | 新调试                    		 *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/
#include "sys.h"
#include "bsp.h"
#include "switch.h"
#include "mw_motor.h"

#include "ramp.h"
#include "pid_regulator.h"

#include "judge_task.h"
#include "command_task.h"
#include "can_task.h"
#include <math.h>

/// @brief PID初始化
PID_Regulator_t GMPPositionPID    = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;     
//PID_Regulator_t GMPSpeedPID       = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID    = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;	
//PID_Regulator_t GMYSpeedPID       = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;
PID_Regulator_t RAMMERSpeedPID    = RAMMER_SPEED_PID_DEFAULT;
PID_Regulator_t RAMMERPositionPID = RAMMER_POSITION_PID_DEFAULT;
PID_Regulator_t CMRotatePID       = CHASSIS_MOTOR_ROTATE_PID_DEFAULT;
PID_Regulator_t BigRammerPID   =  BIG_RAMMER_SPEED_PID_DEFAULT;
PID_Regulator_t VFric1_SpeedPID   = HFRICTION1_SPEED_PID_DEFAULT;
PID_Regulator_t VFric2_SpeedPID   = HFRICTION2_SPEED_PID_DEFAULT;
PID_Regulator_t HFric_SpeedPID   = VFRICTION0_SPEED_PID_DEFAULT;
PID_Regulator_t VFric_PositionPID   = VFRICTION0_POSITION_PID_DEFAULT;

/// @brief 斜坡函数初始化
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;///<云台启动斜坡函数
RampGen_t GMYawRamp   = RAMP_GEN_DAFAULT;
RampGen_t CMRamp      = RAMP_GEN_DAFAULT;

/// @brief	运行状态接口函数
static WorkState_e work_state_lastime = PREPARE_STATE;		//上个时间节点的运行状态
static WorkState_e work_state_last = PREPARE_STATE;			//上一个运行状态
static WorkState_e work_state     = PREPARE_STATE;			//当前运行状态
	void SetWorkState(WorkState_e state){if((work_state!=STARTUP_STATE)||work_state!=PREPARE_STATE)work_state=state;}
	WorkState_e GetWorkState(void){return work_state;}
/// @brief	准备模式获取码盘角度范围
static bool start_pre_rotate=false;
	void Start_Pre_Rotate(void){start_pre_rotate=true;}
/// @brief	yaw轴重映射数据
static float remap_pos=0;
static float remap_angle=540;
static int16_t remapped_yaw_cycle=0;
	int16_t Get_Rremap_Yaw_Cycle(void){return remapped_yaw_cycle;}
static float remapped_yaw_angle=0;///角度范围-180~180
	float Get_Rremap_Yaw_Angle(void){return remapped_yaw_angle;}
/// @brief	启动大摩擦轮
static bool big_fric=false;
	void StartBigFric(void){big_fric=true;}
	void StopBigFric(void){big_fric=false;}
/// @brief	单发射击模式
static bool single_shoot_mode=false;
	void SetSingleShootMode(bool flag){single_shoot_mode=flag;}
	
/// @brief	射击命令(单发模式下需要置零后才能射击)
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
static volatile uint8_t  ShootSpeed = 19;
	
/// @brief  射频
static volatile int16_t ShootFre   = 1600;

/// @brief  底盘速度控制量
static int16_t CM_left_right_target=0;
static int16_t CM_forward_back_target=0;
static int16_t CM_rotate_target=0;
	
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
	if(start_pre_rotate&&(Reach_Reset_Pos()==3))Remapping_Yaw_Angle();
	GimbalControlModeSwitch();
	GMPitchControlLoop();
	GMYawControlLoop();
	GimbalMotorOutput();
  	CMControlLoop();
//	BigBulletFric_Control_Prc();
//	BigBulletRammer_Control_Prc();
}

static float YawAngleBias = 0.0f;
static float YawIMUAngleBias = 0.0f;
static float YawTargetBias = 0.0f;
static float YawRemappedBias = 0.0f;
static float PitchAngleLast =0;
/// @brief 云台不同模式控制
static void GimbalControlModeSwitch(void){
	/*mode changed*/
	if(work_state!=work_state_lastime)
		WorkStateChange();
	
	
	switch(GetWorkState()){
	/*start up mode*/
		case STARTUP_STATE:{
			work_state_lastime=work_state;//更新上一次状态
			GMYPositionPID.ref = 0.0f;
			GMYPositionPID.fdb = 0.0f;//-(GetMotorData(YAW_MOTOR).angle+360*GetMotorData(YAW_MOTOR).cycles)* GMYawRamp.Calc(&GMYawRamp);
			if(xTaskGetTickCount() > STARTUP_TIME){	///< @brief 让云台进入陀螺仪控制
				work_state = PREPARE_STATE;
			}
		}break;
	/*prepare mode*/
		case PREPARE_STATE:{
			work_state_lastime=work_state;//更新上一次状态
			
			GMPPositionPID.ref = GetGimbalTarget().pitch_angle_target;
			GMPPositionPID.fdb = (GetMotorData(PIT_MOTOR,false).angle+360*GetMotorData(PIT_MOTOR,false).cycles);
			
			GMYPositionPID.ref=0;//(GetGimbalTarget().yaw_angle_target-YawTargetBias);
			GMYPositionPID.fdb = -(Get_IMU_data().yaw-YawIMUAngleBias);
			
			//get the Encoder range
			/////what if the switch is somethong wrong/////
			CM_forward_back_target=GetChassisSpeedTarget().forward_back_target;
			CM_left_right_target=GetChassisSpeedTarget().left_right_target;
			if(start_pre_rotate){
				CM_rotate_target=CHASSIS_ROTATE_SPEED;
				//>第一次复位位置
				static uint8_t resetstate=2;
				static float reset_pos_1=0;
				static float remap_pos_2=0;
				static float remap_pos_3=0;
				static int16_t Reset_Cycle_1=0;
				static int16_t Reset_Cycle_2=0;
				static int16_t Reset_Cycle_3=0;
//				if(resetstate==1&&Reach_Reset_Pos()==1)
//				{
//					reset_pos_1=GetMotorData(YAW_MOTOR,false).angle;
//					Reset_Cycle_1=GetMotorData(YAW_MOTOR,false).cycles;
//					printf("reset_pos_1:%d\t",(int)reset_pos_1);
//					printf("remap_pos_2:%d\t",(int)remap_pos_2);
//					printf("Reset_Cycle_1:%d\t",Reset_Cycle_1);
//					printf("Reset_Cycle_2:%d\r\n",Reset_Cycle_2);
//					resetstate++;
//				}
				//第二次复位
				if(resetstate==2&&Reach_Reset_Pos()==2)
				{
					remap_pos_2=GetMotorData(YAW_MOTOR,false).angle;			//>由于remap_pos=reset_pos_angle_2;所以省去reset_pos_angle_2变量
					Reset_Cycle_2=GetMotorData(YAW_MOTOR,false).cycles;	//>remap_cycle=reset_pos_cycle_2;所以省去reset_pos_cycle_2变量
//					remap_angle=remap_pos_2-reset_pos_1+360.0*(Reset_Cycle_2-Reset_Cycle_1);  //绝对值
//					remap_angle>0?(remap_angle=remap_angle):(remap_angle=-remap_angle);
					
//					printf("reset_pos_1:%d\t",(int)reset_pos_1);
//					printf("remap_pos_2:%d\t",(int)remap_pos_2);
//					printf("Reset_Cycle_1:%d\t",Reset_Cycle_1);
//					printf("Reset_Cycle_2:%d\r\n",Reset_Cycle_2);
					resetstate++;
				}
				else if(resetstate==3&&Reach_Reset_Pos()==3)
				{
					remap_pos_3=GetMotorData(YAW_MOTOR,false).angle;
					Reset_Cycle_3=GetMotorData(YAW_MOTOR,false).cycles;
					remap_pos=((float)((remap_pos_2+remap_pos_3)+360.0*(Reset_Cycle_3+Reset_Cycle_2))/2.0);
					work_state = FOLLOW_UP_STATE;
//					printf("remap_pos_3:%d\t",(int)remap_pos_3);
//					printf("Reset_Cycle_3:%d\t",Reset_Cycle_3);
//					printf("remap_pos:%d\t",(int)remap_pos);
//					printf("remap_angle:%d\t",(int)remap_angle);
				}
			}
			else{
				CM_rotate_target=GetChassisSpeedTarget().rotate_target;
			}
		}break;
	/*follow up mode*/
		case FOLLOW_UP_STATE:{
			static float last_yaw_remapped_yaw_angle;
			if(work_state_lastime==FREE_VIEW_STATE){
				last_yaw_remapped_yaw_angle=remapped_yaw_angle;
				GMYawRamp.SetScale(&GMYawRamp, fabs(last_yaw_remapped_yaw_angle)*5);//斜坡函数设定值,若误差为180度则回归时间900ms
				GMYawRamp.ResetCounter(&GMYawRamp);
				GMPitchRamp.SetScale(&GMPitchRamp, 500);//斜坡函数设定值,若误差为180度则回归时间900ms
				GMPitchRamp.ResetCounter(&GMPitchRamp);
				SetGimbalTarget_P(PitchAngleLast); 	//设定Pitch轴目标值为之前的位置
			}
			else if(work_state_lastime==CHASSIS_ROTATE_STATE){
				last_yaw_remapped_yaw_angle=remapped_yaw_angle;
				CMRamp.SetScale(&CMRamp, fabs(last_yaw_remapped_yaw_angle)*10);
				CMRamp.ResetCounter(&CMRamp);
			}
			work_state_lastime=work_state;//更新上一时间状态
			if(work_state_last==CHASSIS_ROTATE_STATE)
			{
				GMYPositionPID.ref = (GetGimbalTarget().yaw_angle_target-YawTargetBias);
				GMYPositionPID.fdb = -(Get_IMU_data().yaw-YawIMUAngleBias);
				//底盘回复原位
				CMRotatePID.ref=last_yaw_remapped_yaw_angle*(1-CMRamp.Calc(&CMRamp));
				CMRotatePID.fdb=remapped_yaw_angle; 
				CMRotatePID.Calc(&CMRotatePID);
				CM_forward_back_target=GetChassisSpeedTarget().forward_back_target;
				CM_left_right_target=GetChassisSpeedTarget().left_right_target;
				(CMRotatePID.output>0)?(CM_rotate_target=200):(CM_rotate_target=-200);
				//----------需要加入底盘复位机制-------------//
				if(YAW_SWITCH==POS_RESET)work_state_last=FOLLOW_UP_STATE;
			}
			else{
				GMYPositionPID.ref = last_yaw_remapped_yaw_angle*(1-GMYawRamp.Calc(&GMYawRamp));
				GMYPositionPID.fdb = remapped_yaw_angle;//+360.0*remapped_yaw_cycle-YawRemappedBias;
				CM_forward_back_target=GetChassisSpeedTarget().forward_back_target;
				CM_left_right_target=GetChassisSpeedTarget().left_right_target;
				CM_rotate_target=GetChassisSpeedTarget().rotate_target;
			}
			GMPPositionPID.ref = GetGimbalTarget().pitch_angle_target;
			GMPPositionPID.fdb = (GetMotorData(PIT_MOTOR,false).angle+360*GetMotorData(PIT_MOTOR,false).cycles);
			
			
		}break;
	/*free view mode*/
		case FREE_VIEW_STATE:{
			if(work_state_lastime!=FREE_VIEW_STATE){
				PitchAngleLast=GetMotorData(PIT_MOTOR,false).angle+360*GetMotorData(PIT_MOTOR,false).cycles;
			}
			work_state_lastime=work_state;//更新上一次状态
			GMPPositionPID.ref = GetGimbalTarget().pitch_angle_target;
			GMPPositionPID.fdb = (GetMotorData(PIT_MOTOR,false).angle+360*GetMotorData(PIT_MOTOR,false).cycles);
			GMYPositionPID.ref = (GetGimbalTarget().yaw_angle_target-YawTargetBias);
			GMYPositionPID.fdb = (remapped_yaw_angle+360*remapped_yaw_cycle-YawRemappedBias);
			CM_forward_back_target=GetChassisSpeedTarget().forward_back_target;
			CM_left_right_target=GetChassisSpeedTarget().left_right_target;
			CM_rotate_target=0;
		}break;
	/*chassis rotate mode*/
		case CHASSIS_ROTATE_STATE:{
			work_state_lastime=work_state;//更新上一次状态
			GMYPositionPID.ref = (GetGimbalTarget().yaw_angle_target-YawTargetBias);
			GMYPositionPID.fdb = -(Get_IMU_data().yaw-YawIMUAngleBias);
			CM_forward_back_target=GetChassisSpeedTarget().forward_back_target;
			CM_left_right_target=GetChassisSpeedTarget().left_right_target;
			CM_rotate_target=400;//CHASSIS_ROTATE_SPEED;
		}break;
		case STOP_STATE:{
		work_state_lastime=work_state;//更新上一次状态
		}break;
	}
}

/** 
@brief 运行状态改变时运行
*/
static void WorkStateChange(void){
	work_state_last=work_state_lastime;
	YawIMUAngleBias = Get_IMU_data().yaw;				///< @brief 获取陀螺仪初始位置
	YawAngleBias = 0;									///< @brief 获取yaw初始位置
	YawTargetBias = GetGimbalTarget().yaw_angle_target; ///< @brief 获取YAW目标值初始位置
	YawRemappedBias = (remapped_yaw_angle+360.0*remapped_yaw_cycle);
	
}

/** 
@brief Pitch控制环
*/
static void GMPitchControlLoop(void){
	if(GetWorkState()!=STARTUP_STATE){
		GMPPositionPID.ref = GetGimbalTarget().pitch_angle_target;
		GMPPositionPID.fdb = (GetMotorData(PIT_MOTOR,false).angle+360*GetMotorData(PIT_MOTOR,false).cycles);
	}
	GMPPositionPID.Calc(&GMPPositionPID);
}

/** 
@brief Yaw控制环
*/
static void GMYawControlLoop(void){
	if(YAW_SWITCH==POS_RESET)LED_C=LED_ON;
	else LED_C=LED_OFF;
//	if(xTaskGetTickCount()<1000){
//		YawTargetBias=GetMotorData(YAW_MOTOR,false).angle+360*GetMotorData(YAW_MOTOR,false).cycles;
////		YawTargetBias= Get_IMU_data().yaw;
//	}
//	else{
//	GMYPositionPID.ref = 0;//GetGimbalTarget().yaw_angle_target;
//	GMYPositionPID.fdb = GetMotorData(YAW_MOTOR,false).angle+360*GetMotorData(YAW_MOTOR,false).cycles-YawTargetBias;
////	GMYPositionPID.fdb = -(Get_IMU_data().yaw-YawAngleBias);
//	GMYPositionPID.Calc(&GMYPositionPID);
//	}
	GMYPositionPID.Calc(&GMYPositionPID);
}

/** 
@brief 云台电机输出
*/
static void GimbalMotorOutput(void){
	SetMotorCurrent(YAW_MOTOR,GMYPositionPID.output);
	SetMotorCurrent(PIT_MOTOR,GMPPositionPID.output * GMPitchRamp.Calc(&GMPitchRamp));
}


/** 
@brief 底盘控制程序
*/
static void CMControlLoop(void){
	static uint8_t tick_7ms=0;
	if(tick_7ms==7){
		tick_7ms=0;
		if(GetInputMode()==STOP)
			SendChassisSpeed(CAN2,0,0,0,0);
		else
		SendChassisSpeed(CAN2,1,CM_left_right_target,
		CM_forward_back_target,
		CM_rotate_target);
	}
	tick_7ms++;
}

/** 
@brief 重映射角度范围
*/
static void Remapping_Yaw_Angle(void){
	static float temp=0;
	//计算连续的总角度
	temp=GetMotorData(YAW_MOTOR,false).angle+360.0*GetMotorData(YAW_MOTOR,false).cycles-remap_pos;
	//角度映射到0~360范围，并修正圈数
	remapped_yaw_angle=((int32_t)(100*temp)%(int32_t)(100*remap_angle))*360.0f/100.0f/remap_angle;
	if(remapped_yaw_angle<0)remapped_yaw_angle=360.0+remapped_yaw_angle;//负数取余问题(不同语言有不同结果)
	remapped_yaw_cycle=(int16_t)(temp+remap_angle/2)/remap_angle;
	if((temp+remap_angle/2.0)<0)remapped_yaw_cycle--;
	//角度映射到-180~180°范围
	if(remapped_yaw_angle>180){
		remapped_yaw_angle-=360.0f;
	}
	else if(remapped_yaw_angle<-180){
		remapped_yaw_angle+=360.0f;
	}
}

/** 
@brief 射击限制的选择(通过裁判系统读取等级，根据规则自己设定)
*/
static void ShootLimitSwitch(void){
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
	BigRammerPID.fdb=GetMotorData(RAMMER_MOTOR,false).speed;
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
	static bool photo_switch;///<枪管光电开关是否被照射
	static bool photo_switch_last;///<枪管光电开关上一次状态
	
	photo_switch=GUN_SWITCH;
	if(photo_switch==1)LED_B=LED_OFF;
	else LED_B=LED_ON;
	///<小摩擦轮控制
	if(single_shoot_mode==true){///<单发模式
		if(on_shooting==false){
//			if(shoot==false){
			{
				if(photo_switch==BULLET_FILLING){///<控制速度把弹挤到限位开关
					HFric_SpeedPID.ref=36*40;
					first_stop=false;
					bullet_stucked_after_boot_up=false;///<开机未卡弹
				}
				else if(photo_switch_last==BULLET_IN_PLACE){///<控制位置把弹卡住
					if(first_stop==false){
						VFric_PositionPID.ref = GetMotorData(FRIC_MID_MOTOR,false).angle + GetMotorData(FRIC_MID_MOTOR,false).cycles*360;
						if(bullet_stucked_after_boot_up==false)VFric_PositionPID.ref += 0;//360*6;///<若开机未卡弹则加前推量
						first_stop=true;
					}
					else{
						VFric_PositionPID.fdb=GetMotorData(FRIC_MID_MOTOR,false).angle + GetMotorData(FRIC_MID_MOTOR,false).cycles*360;
						VFric_PositionPID.Calc(&VFric_PositionPID);
						HFric_SpeedPID.ref=VFric_PositionPID.output;
					}
				}
			}
			if(shoot==true){///<开启发射
				if(shoot_last==false)
				on_shooting=true;
			}
		}
	}
	
	else if(single_shoot_mode==false){///<连发模式
		if(shoot==false){
			if(photo_switch==BULLET_FILLING){///<控制速度把弹挤到限位开关
				HFric_SpeedPID.ref=36*40;
				first_stop=false;
				bullet_stucked_after_boot_up=false;///<开机未卡弹
			}
			else if(photo_switch_last==BULLET_IN_PLACE){///<控制位置把弹卡住
				if(first_stop==false){
					VFric_PositionPID.ref=GetMotorData(FRIC_MID_MOTOR,false).angle + GetMotorData(FRIC_MID_MOTOR,false).cycles*360;
					if(bullet_stucked_after_boot_up==false)VFric_PositionPID.ref += 360*6;///<若开机未卡弹则加前推量
					first_stop=true;
				}
				else{
					VFric_PositionPID.fdb=GetMotorData(FRIC_MID_MOTOR,false).angle + GetMotorData(FRIC_MID_MOTOR,false).cycles*360;
					VFric_PositionPID.Calc(&VFric_PositionPID);
					HFric_SpeedPID.ref=VFric_PositionPID.output;
				}
			}
		}
		else{
			HFric_SpeedPID.ref=36*60;
		}
	}
	
	if(on_shooting){///<把弹推到摩擦轮
		HFric_SpeedPID.ref=36*60*2;
		if(photo_switch==true)
			if(photo_switch_last==false){
				on_shooting=false;
			}
	}
	HFric_SpeedPID.fdb=GetMotorData(FRIC_MID_MOTOR,false).speed;
	HFric_SpeedPID.Calc(&HFric_SpeedPID);
	SetMotorCurrent(FRIC_MID_MOTOR,HFric_SpeedPID.output);
	
	///<大摩擦轮控制
	if(big_fric){
		VFric1_SpeedPID.ref=ShootSpeed*BULLET_SPEED_TO_MOTOR_SPEED;
		VFric2_SpeedPID.ref=-ShootSpeed*BULLET_SPEED_TO_MOTOR_SPEED;
	}
	else {
		VFric1_SpeedPID.ref=0;
		VFric2_SpeedPID.ref=0;
	}
	VFric1_SpeedPID.fdb=GetMotorData(FRIC_UP_MOTOR,false).speed;
	VFric2_SpeedPID.fdb=GetMotorData(FRIC_DOWN_MOTOR,false).speed;
	VFric1_SpeedPID.Calc(&VFric1_SpeedPID);
	VFric2_SpeedPID.Calc(&VFric2_SpeedPID);
	SetMotorCurrent(FRIC_UP_MOTOR,VFric1_SpeedPID.output);
	SetMotorCurrent(FRIC_DOWN_MOTOR,VFric2_SpeedPID.output);
	///<更新变量
	shoot_last=shoot;
	photo_switch_last=photo_switch;
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
    printf("PKP:%3.2f,PKI:%1.2f,PKD:%3.2f",
    GMPPositionPID.kp,GMPPositionPID.ki,GMPPositionPID.kd);
}
void Monitor_Yaw_PID(void)
{
    printf("PKP:%3.2f,PKI:%1.2f,PKD:%3.2f",
    GMYPositionPID.kp,GMYPositionPID.ki,GMYPositionPID.kd);
}
void FRIC_PID()
{
	printf("PKP:%3.2f,PKI:%1.2f,PKD:%3.2f",
    VFric1_SpeedPID.kp,VFric1_SpeedPID.ki,VFric1_SpeedPID.kd);
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
    switch(x){
    case 0x1:
        VFric1_SpeedPID.kp += (float)y/100;
		VFric2_SpeedPID.kp += (float)y/100;
        break;
    case 0x2:
        VFric1_SpeedPID.ki += (float)y/100;
		VFric2_SpeedPID.kp += (float)y/100;
        break;
    case 0x3:
        VFric1_SpeedPID.kd += (float)y/100;
		VFric2_SpeedPID.kp += (float)y/100;
        break;
    default :
        ;
    }
    FRIC_PID();
}
void FRIC_PID_MIN(int x,int y)
{
    switch(x){
    case 0x1:
        VFric1_SpeedPID.kp -= (float)y/100;
		VFric2_SpeedPID.kp -= (float)y/100;
        break;
    case 0x2:
        VFric1_SpeedPID.ki -= (float)y/100;
		VFric2_SpeedPID.kp -= (float)y/100;
        break;
    case 0x3:
        VFric1_SpeedPID.kd -= (float)y/100;
		VFric2_SpeedPID.kp -= (float)y/100;
        break;
    default :
        ;
    }
    FRIC_PID();
}
