#include "monitor_task.h"

#include "dbus.h"
#include "sys.h"
#include "timer.h"
#include <string.h>
#include "control_task.h"
#include "visualscope.h"
#include "pid_regulator.h"

#include "can.h"
//������������ʱ����Ϣ
char RunTimeInfo[400];
//FreeRTOSʱ��ͳ�����õĽ��ļ�����
volatile unsigned long long FreeRTOSRunTimeTicks;
	void FreeRTOSRunTimeTicks_Add(void){FreeRTOSRunTimeTicks++;}///<�жϴ���������
extern PID_Regulator_t HFric1_SpeedPID;
extern PID_Regulator_t HFric2_SpeedPID;

void MonitorPrc(void){
//	printf("%d \r\n",(int)GetMotorData(PIT_MOTOR).angle+100);
#if Monitor_Task_Time//��ӡ����λ��
	ShowRunTimeStats();
#endif
#if Monitor_Task_Stack//��ӡ����λ��
	ShowTaskList();
#endif
	//VisualScope(UART8,20000+19*BULLET_SPEED_TO_MOTOR_SPEED,20000+GetMotorData(FRIC1_MOTOR).speed,20000+GetMotorData(FRIC1_MOTOR).torque,2000+HFric1_SpeedPID.output);//2000+HFric1_SpeedPID.output,2000+HFric2_SpeedPID.output);
}

//------------------------------------------------------------
//����ʵ��ϵͳ��������ʱ�������صĺ���
//���øù���������FreeRTOSConfig.h��configGENERATE_RUN_TIME_STATSΪ0
//------------------------------------------------------------


/** 
	@brief ���ڴ�ӡ��������ʱ�����
*/
void ShowRunTimeStats(void){
	memset(RunTimeInfo,0,400);              //��Ϣ����������
	vTaskGetRunTimeStats(RunTimeInfo);      //��ȡ��������ʱ����Ϣ
	printf("������\t\t\t���м���\tʹ����\r\n");
	printf("%s\r\n",RunTimeInfo);
}

/** 
	@brief ���ڴ�ӡ��������״̬�б�
*/
void ShowTaskList(void){
	memset(RunTimeInfo,0,400);              //��Ϣ����������
	vTaskList(RunTimeInfo);      //��ȡ��������ʱ����Ϣ
	printf("������\t\t\t״̬\t���ȼ�\tʣ��ջ\t�������\r\n");
	printf("%s\r\n",RunTimeInfo);
}

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
