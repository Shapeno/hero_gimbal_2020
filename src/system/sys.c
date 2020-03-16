#include "sys.h"  
#include "bsp.h"

extern void xPortSysTickHandler( void );	//该函数在port.h

/// @brief 系统警告和错误
bool sys_warning=false;
	bool GetSysWarning(void){return sys_warning;}
	void Sys_Warning(void){sys_warning=true;}
	void Sys_Warning_Clean(void){sys_warning=false;}
bool sys_error=false;
	bool GetSysError(void){return sys_error;}
	void Sys_Error(void){sys_error=true;}
	void Sys_Error_Clean(void){sys_error=false;}

/** 
	@brief 用于判断遥控器是否离线
	当大于2ms没有消息则视为离线
*/
bool IfRemoteOffline(portTickType currentTime){
	if(currentTime-GetRemoteDataTime()>300)return 1;
	else return 0;
}
/// @brief systick中断服务函数,freeRTOS用到
void SysTick_Handler(void)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//如果OS已经在跑了,并且不是在中断里面(中断里面不能任务调度)	    
	{
		xPortSysTickHandler();
	}
}

/// @brief 堆栈溢出后红灯持续亮  蜂鸣器持续鸣叫
void vApplicationStackOverflowHook( TaskHandle_t xTask,signed char *pcTaskName ){
	LED_Red=LED_ON;
	LED_Green=LED_OFF;
	Sing_bad_case();
}
