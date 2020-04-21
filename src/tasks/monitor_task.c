#include "monitor_task.h"

#include "dbus.h"
#include "sys.h"
#include <string.h>
#include "control_task.h"
#include "visualscope.h"
#include "pid_regulator.h"
#include "usbd_usr.h"
#include "led.h"

#include "can.h"
//保存任务运行时间信息
char RunTimeInfo[400];
//监控函数
static bool IfRemoteOffline(void);
static bool IfUSBConnected(void);

void MonitorPrc(void){
	//监测USB连接状态
	IfUSBConnected();
	//用于监控遥控器是否离线
	IfRemoteOffline();
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

/** 
	@brief 用于判断遥控器是否离线
	当大于2ms没有消息则视为离线
*/
static bool IfRemoteOffline(void){
	static portTickType currentTime;
	currentTime=xTaskGetTickCount();
	if(currentTime-GetRemoteDataTime()>300){
		Sys_Warning();
		return 1;
	}
	else{
		Sys_Warning_Clean();
		return 0;
	}
}
/** 
	@brief 用于显示USB的连接状态，USB连接成功LED_A亮
*/
static bool IfUSBConnected(void){
	static u8 usbstatus=0;
	if(usbstatus!=Get_USBConnectState()){//USB连接状态发生了改变.
		usbstatus=Get_USBConnectState();//记录新的状态
		if(usbstatus==CONFIGURED){//提示USB连接成功		
			LED_A=LED_ON;//DS1亮		
		}else{//提示USB断开
			LED_A=LED_OFF;//DS1灭
		}
	}
}