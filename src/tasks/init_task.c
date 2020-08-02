#include "init_task.h"

#include "bsp.h"
#include "sys.h"

#include "judge_task.h"
#include "command_task.h"
#include "control_task.h"
#include "can_task.h"
#include "monitor_task.h"
#include "OLED_task.h"
#include "usmart.h"
//#include "timer.h"
//#include "usart7.h"
//#include "iwdg.h"

//------------------------------------------------------------
//�����б�
//------------------------------------------------------------

//��������
#define CMD_TASK_PRIO	2			//�������ȼ�
#define CMD_STK_SIZE	128		//�����ջ��С
#define CMD_TASK_PERIOD 7			//������������ms
TaskHandle_t CMD_Task_Handler;		//������
void CMD_Task(void*p_arg);			//������
//CAN����
#define CAN_TASK_PRIO	2			//�������ȼ�
#define CAN_STK_SIZE	128			//�����ջ��С
#define CAN_TASK_PERIOD 1			//������������ms
TaskHandle_t CAN_Task_Handler;		//������
void CAN_Task(void*p_arg);			//������
//����������
#define IMU_TASK_PRIO	2			//�������ȼ�
#define IMU_STK_SIZE	128			//�����ջ��С
#define IMU_TASK_PERIOD 1			//������������ms
TaskHandle_t IMU_Task_Handler;		//������
void IMU_Task(void*p_arg);			//������
//��������
#define CONTROL_TASK_PRIO	2			//�������ȼ�
#define CONTROL_STK_SIZE	256			//�����ջ��С
#define CONTROL_TASK_PERIOD 1			//������������ms
TaskHandle_t Control_Task_Handler;		//������
void Control_Task(void*p_arg);			//������
//����ϵͳ��Ϣ��������
#define JUDGE_TASK_PRIO	2			//�������ȼ�
#define JUDGE_STK_SIZE	128			//�����ջ��С
#define JUDGE_TASK_PERIOD 10		//������������ms
TaskHandle_t Judge_Task_Handler;		//������
void Judge_Task(void*p_arg);			//������
//����������
#define MONITOR_TASK_PRIO	2			//�������ȼ�
#define MONITOR_STK_SIZE	128			//�����ջ��С
#define MONITOR_TASK_PERIOD 1			//������������ms
TaskHandle_t Monitor_Task_Handler;		//������
void Monitor_Task(void*p_arg);			//������
//OLED����
#define OLED_TASK_PRIO	2			//�������ȼ�
#define OLED_STK_SIZE	512			//�����ջ��С
#define OLED_TASK_PERIOD 5			//������������ms
TaskHandle_t OLED_Task_Handler;		//������
void OLED_Task(void*p_arg);			//������
//LED��ʾ�����Ƿ�������
#define RUNNING_TASK_PRIO	2			//�������ȼ�
#define RUNNING_STK_SIZE	50			//�����ջ��С
#define RUNNING_TASK_PERIOD 500			//������������ms
TaskHandle_t Running_Task_Handler;		//������
void Running_Task(void*p_arg);			//������
//------------------------------------------------------------
//����
//------------------------------------------------------------

/**	
*@brief	�����ʼ������
*/
void Task_Init(void){
	taskENTER_CRITICAL();//�����ٽ���
	
	xTaskCreate((TaskFunction_t	)CMD_Task,
				(const char*	)"CMD_Task",
				(uint16_t		)CMD_STK_SIZE,
				(void*			)NULL,
				(UBaseType_t	)CMD_TASK_PRIO,
				(TaskHandle_t*	)&CMD_Task_Handler);
				
	xTaskCreate((TaskFunction_t	)IMU_Task,
				(const char*	)"IMU_Task",
				(uint16_t		)IMU_STK_SIZE,
				(void*			)NULL,
				(UBaseType_t	)IMU_TASK_PRIO,
				(TaskHandle_t*	)&IMU_Task_Handler);
				
	xTaskCreate((TaskFunction_t	)Control_Task,
				(const char*	)"Control_Task",
				(uint16_t		)CONTROL_STK_SIZE,
				(void*			)NULL,
				(UBaseType_t	)CONTROL_TASK_PRIO,
				(TaskHandle_t*	)&Control_Task_Handler);
				
	xTaskCreate((TaskFunction_t	)CAN_Task,
				(const char*	)"CAN_Task",
				(uint16_t		)CAN_STK_SIZE,
				(void*			)NULL,
				(UBaseType_t	)CAN_TASK_PRIO,
				(TaskHandle_t*	)&CAN_Task_Handler);
				
	xTaskCreate((TaskFunction_t	)Monitor_Task,
				(const char*	)"Monitor_Task",
				(uint16_t		)MONITOR_STK_SIZE,
				(void*			)NULL,
				(UBaseType_t	)MONITOR_TASK_PRIO,
				(TaskHandle_t*	)&Monitor_Task_Handler);
				
	xTaskCreate((TaskFunction_t	)OLED_Task,
				(const char*	)"OLED_Task",
				(uint16_t		)OLED_STK_SIZE,
				(void*			)NULL,
				(UBaseType_t	)OLED_TASK_PRIO,
				(TaskHandle_t*	)&OLED_Task_Handler);
				
	xTaskCreate((TaskFunction_t	)Running_Task,
				(const char*	)"Running_Task",
				(uint16_t		)RUNNING_STK_SIZE,
				(void*			)NULL,
				(UBaseType_t	)RUNNING_TASK_PRIO,
				(TaskHandle_t*	)&Running_Task_Handler);
				
	taskEXIT_CRITICAL();//�˳��ٽ���
}

/**	
*@brief	��������
*/
void CMD_Task(void*p_arg){
	portTickType currentTime;
	while(1){
		currentTime = xTaskGetTickCount();	//��ȡ��ǰϵͳʱ��
		
		RcCmdPrc();
		
		vTaskDelayUntil(&currentTime, CMD_TASK_PERIOD/portTICK_RATE_MS);
	}
}

///**	
//*@brief	CANͨ�Ŵ�������
//*/
void CAN_Task(void*p_arg){
	portTickType currentTime;
	while(1){
		currentTime = xTaskGetTickCount();	//��ȡ��ǰϵͳʱ��
		
		CanMsgPrc();
		
		vTaskDelayUntil(&currentTime, CAN_TASK_PERIOD/portTICK_RATE_MS);
	}
}

/**	
*@brief	����������
*/
void IMU_Task(void*p_arg){
	while(1){	
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update(); 
		delay_ms(5);		//��ȡ��ǰϵͳʱ��
	}
}

/**	
*@brief	��������
*/
void Control_Task(void*p_arg){
	portTickType currentTime;
	while(1){
		currentTime = xTaskGetTickCount();	//��ȡ��ǰϵͳʱ��
		
		ControlPrc();
		
		vTaskDelayUntil(&currentTime, CONTROL_TASK_PERIOD/portTICK_RATE_MS);
	}
}

/**	
*@brief	����ϵͳ����
*/
void Judge_Task(void*p_arg){
	portTickType currentTime;
	while(1){
		currentTime = xTaskGetTickCount();	//��ȡ��ǰϵͳʱ��
		
		vTaskDelayUntil(&currentTime, JUDGE_TASK_PERIOD/portTICK_RATE_MS);
	}
}

/**	
*@brief	�����ϵͳ�������
*/
void Monitor_Task(void*p_arg){
//	portTickType currentTime;
	while(1){
//		currentTime = xTaskGetTickCount();	//��ȡ��ǰϵͳʱ��
	
		MonitorPrc();
		
//vTaskDelayUntil(&currentTime, MONITOR_TASK_PERIOD/portTICK_RATE_MS);
		delay_ms(MONITOR_TASK_PERIOD/portTICK_RATE_MS);
	}
}

void OLED_Task(void*p_arg){
	portTickType currentTime;
	while(1){
		currentTime = xTaskGetTickCount();	//��ȡ��ǰϵͳʱ��
		
		OLED_Prc();
		
		vTaskDelayUntil(&currentTime, OLED_TASK_PERIOD/portTICK_RATE_MS);
	}
}

/**	
*@brief	LED��ʾ�����Ƿ�������,ͬʱ����ϵͳ����ͱ���(��Ҫ�޸�)
*/
void Running_Task(void*p_arg){
	portTickType currentTime;
	static bool beep_signal=false;
	while(1){
		currentTime = xTaskGetTickCount();	//��ȡ��ǰϵͳʱ��
		
		if(GetSysWarning()){
			if(beep_signal==false){
				beep_signal=true;
				Sing(La6M);
			}
			else if(beep_signal==true){
				beep_signal=false;
				Sing(Silent);
			}
			LED_Red=~LED_Red;
			LED_Green=~LED_Green;
		}
		else if(GetSysError()){
			Sing(La6M);
			LED_Red=LED_ON;
			LED_Green=~LED_Green;
		}
		else{
			Sing(Silent);
			LED_Red=LED_OFF;
			LED_Green=~LED_Green;
		}
		vTaskDelayUntil(&currentTime, RUNNING_TASK_PERIOD/portTICK_RATE_MS);
	}
}

