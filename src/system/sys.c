#include "sys.h"  
#include "bsp.h"
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
//——————————————————————————————————————————RTOS——————————————————————————————————————————//

/*-------------------------------------RTOS实现函数-------------------------------------*/
extern void xPortSysTickHandler( void );	//该函数在port.h
/// @brief systick中断服务函数,freeRTOS用到
void SysTick_Handler(void)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//如果OS已经在跑了,并且不是在中断里面(中断里面不能任务调度)	    
	{
		xPortSysTickHandler();
	}
}
/*--------------------------------RTOS任务监控实现函数-----------------------------------*/
/// @brief FreeRTOS任务时间统计所用的节拍计数器
volatile unsigned long long FreeRTOSRunTimeTicks;
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
/// @brief TIM6中断服务函数
void TIM6_DAC_IRQHandler(void)  {///<为了方便移植，讲中断函数放在这里，用于监控RTOS进程占用时间
	if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET){
		FreeRTOSRunTimeTicks++;
	}
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
}
/*--------------------------------RTOS回调函数-----------------------------------*/
/// @brief 堆栈溢出回调函数：红灯持续亮，蜂鸣器持续鸣叫
void vApplicationStackOverflowHook( TaskHandle_t xTask,signed char *pcTaskName ){
	LED_Red=LED_ON;
	LED_Green=LED_OFF;
	Sing_bad_case();
}
//——————————————————————————————————————————RTOS——————————————————————————————————————————//
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

/*-------------------------------------系统报警函数-------------------------------------*/
/// @brief 系统警告和错误
bool sys_warning=false;
	bool GetSysWarning(void){return sys_warning;}
	void Sys_Warning(void){sys_warning=true;}
	void Sys_Warning_Clean(void){sys_warning=false;}
bool sys_error=false;
	bool GetSysError(void){return sys_error;}
	void Sys_Error(void){sys_error=true;}
	void Sys_Error_Clean(void){sys_error=false;}



