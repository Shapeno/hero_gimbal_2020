#include "monitor_task.h"

#include "dbus.h"
#include "sys.h"
#include "timer.h"
#include <string.h>
#include "control_task.h"
#include "visualscope.h"
#include "pid_regulator.h"

#include "can.h"
//保存任务运行时间信息
char RunTimeInfo[400];
//FreeRTOS时间统计所用的节拍计数器
volatile unsigned long long FreeRTOSRunTimeTicks;
	void FreeRTOSRunTimeTicks_Add(void){FreeRTOSRunTimeTicks++;}///<中断处理函数调用
extern PID_Regulator_t HFric1_SpeedPID;
extern PID_Regulator_t HFric2_SpeedPID;

void MonitorPrc(void){
//	printf("%d \r\n",(int)GetMotorData(PIT_MOTOR).angle+100);
#if Monitor_Task_Time//打印到上位机
	ShowRunTimeStats();
#endif
#if Monitor_Task_Stack//打印到上位机
	ShowTaskList();
#endif
	//VisualScope(UART8,20000+19*BULLET_SPEED_TO_MOTOR_SPEED,20000+GetMotorData(FRIC1_MOTOR).speed,20000+GetMotorData(FRIC1_MOTOR).torque,2000+HFric1_SpeedPID.output);//2000+HFric1_SpeedPID.output,2000+HFric2_SpeedPID.output);
}

//------------------------------------------------------------
//用于实现系统任务运行时间比例监控的函数
//禁用该功能请配置FreeRTOSConfig.h中configGENERATE_RUN_TIME_STATS为0
//------------------------------------------------------------


/** 
	@brief 用于打印任务运行时间比例
*/
void ShowRunTimeStats(void){
	memset(RunTimeInfo,0,400);              //信息缓冲区清零
	vTaskGetRunTimeStats(RunTimeInfo);      //获取任务运行时间信息
	printf("任务名\t\t\t运行计数\t使用率\r\n");
	printf("%s\r\n",RunTimeInfo);
}

/** 
	@brief 用于打印任务运行状态列表
*/
void ShowTaskList(void){
	memset(RunTimeInfo,0,400);              //信息缓冲区清零
	vTaskList(RunTimeInfo);      //获取任务运行时间信息
	printf("任务名\t\t\t状态\t优先级\t剩余栈\t任务序号\r\n");
	printf("%s\r\n",RunTimeInfo);
}

/// @brief 初始化TIM6使其为FreeRTOS的时间统计提供时基(系统调用)
void ConfigureTimeForRunTimeStats(void)
{
    FreeRTOSRunTimeTicks=0;
    TIM6_Init();   //初始化TIM6
}

/// @brief 获取定时器运行时间(系统调用)
unsigned long long getFreeRTOSRunTimeTicks(void)
{
	return FreeRTOSRunTimeTicks;
}
