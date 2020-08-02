#include "sys.h"  
#include "bsp.h"
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
//������������������������������������������������������������������������������������RTOS������������������������������������������������������������������������������������//

/*-------------------------------------RTOSʵ�ֺ���-------------------------------------*/
extern void xPortSysTickHandler( void );	//�ú�����port.h
/// @brief systick�жϷ�����,freeRTOS�õ�
void SysTick_Handler(void)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//���OS�Ѿ�������,���Ҳ������ж�����(�ж����治���������)	    
	{
		xPortSysTickHandler();
	}
}
/*--------------------------------RTOS������ʵ�ֺ���-----------------------------------*/
/// @brief FreeRTOS����ʱ��ͳ�����õĽ��ļ�����
volatile unsigned long long FreeRTOSRunTimeTicks;
/// @brief ��ʼ��TIM6ʹ��ΪFreeRTOS��ʱ��ͳ���ṩʱ��(ϵͳ����)
void ConfigureTimeForRunTimeStats(void)
{
    FreeRTOSRunTimeTicks=0;
    TIM6_Init();   //��ʼ��TIM6
}

/// @brief ��ȡ��ʱ������ʱ��(ϵͳ����)
unsigned long long getFreeRTOSRunTimeTicks(void)
{
	return FreeRTOSRunTimeTicks;
}
/// @brief TIM6�жϷ�����
void TIM6_DAC_IRQHandler(void)  {///<Ϊ�˷�����ֲ�����жϺ�������������ڼ��RTOS����ռ��ʱ��
	if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET){
		FreeRTOSRunTimeTicks++;
	}
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
}
/*--------------------------------RTOS�ص�����-----------------------------------*/
/// @brief ��ջ����ص���������Ƴ���������������������
void vApplicationStackOverflowHook( TaskHandle_t xTask,signed char *pcTaskName ){
	LED_Red=LED_ON;
	LED_Green=LED_OFF;
	Sing_bad_case();
}
//������������������������������������������������������������������������������������RTOS������������������������������������������������������������������������������������//
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

/*-------------------------------------ϵͳ��������-------------------------------------*/
/// @brief ϵͳ����ʹ���
bool sys_warning=false;
	bool GetSysWarning(void){return sys_warning;}
	void Sys_Warning(void){sys_warning=true;}
	void Sys_Warning_Clean(void){sys_warning=false;}
bool sys_error=false;
	bool GetSysError(void){return sys_error;}
	void Sys_Error(void){sys_error=true;}
	void Sys_Error_Clean(void){sys_error=false;}



