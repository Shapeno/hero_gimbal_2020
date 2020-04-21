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
//������������ʱ����Ϣ
char RunTimeInfo[400];
//��غ���
static bool IfRemoteOffline(void);
static bool IfUSBConnected(void);

void MonitorPrc(void){
	//���USB����״̬
	IfUSBConnected();
	//���ڼ��ң�����Ƿ�����
	IfRemoteOffline();
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

/** 
	@brief �����ж�ң�����Ƿ�����
	������2msû����Ϣ����Ϊ����
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
	@brief ������ʾUSB������״̬��USB���ӳɹ�LED_A��
*/
static bool IfUSBConnected(void){
	static u8 usbstatus=0;
	if(usbstatus!=Get_USBConnectState()){//USB����״̬�����˸ı�.
		usbstatus=Get_USBConnectState();//��¼�µ�״̬
		if(usbstatus==CONFIGURED){//��ʾUSB���ӳɹ�		
			LED_A=LED_ON;//DS1��		
		}else{//��ʾUSB�Ͽ�
			LED_A=LED_OFF;//DS1��
		}
	}
}