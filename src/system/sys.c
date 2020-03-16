#include "sys.h"  
#include "bsp.h"

extern void xPortSysTickHandler( void );	//�ú�����port.h

/// @brief ϵͳ����ʹ���
bool sys_warning=false;
	bool GetSysWarning(void){return sys_warning;}
	void Sys_Warning(void){sys_warning=true;}
	void Sys_Warning_Clean(void){sys_warning=false;}
bool sys_error=false;
	bool GetSysError(void){return sys_error;}
	void Sys_Error(void){sys_error=true;}
	void Sys_Error_Clean(void){sys_error=false;}

/** 
	@brief �����ж�ң�����Ƿ�����
	������2msû����Ϣ����Ϊ����
*/
bool IfRemoteOffline(portTickType currentTime){
	if(currentTime-GetRemoteDataTime()>300)return 1;
	else return 0;
}
/// @brief systick�жϷ�����,freeRTOS�õ�
void SysTick_Handler(void)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//���OS�Ѿ�������,���Ҳ������ж�����(�ж����治���������)	    
	{
		xPortSysTickHandler();
	}
}

/// @brief ��ջ������Ƴ�����  ��������������
void vApplicationStackOverflowHook( TaskHandle_t xTask,signed char *pcTaskName ){
	LED_Red=LED_ON;
	LED_Green=LED_OFF;
	Sing_bad_case();
}
