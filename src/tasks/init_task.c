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
//任务列表
//------------------------------------------------------------

//命令任务
#define CMD_TASK_PRIO	2			//任务优先级
#define CMD_STK_SIZE	128		//任务堆栈大小
#define CMD_TASK_PERIOD 7			//任务运行周期ms
TaskHandle_t CMD_Task_Handler;		//任务句柄
void CMD_Task(void*p_arg);			//任务函数
//CAN任务
#define CAN_TASK_PRIO	2			//任务优先级
#define CAN_STK_SIZE	128			//任务堆栈大小
#define CAN_TASK_PERIOD 1			//任务运行周期ms
TaskHandle_t CAN_Task_Handler;		//任务句柄
void CAN_Task(void*p_arg);			//任务函数
//陀螺仪任务
#define IMU_TASK_PRIO	2			//任务优先级
#define IMU_STK_SIZE	128			//任务堆栈大小
#define IMU_TASK_PERIOD 1			//任务运行周期ms
TaskHandle_t IMU_Task_Handler;		//任务句柄
void IMU_Task(void*p_arg);			//任务函数
//控制任务
#define CONTROL_TASK_PRIO	2			//任务优先级
#define CONTROL_STK_SIZE	256			//任务堆栈大小
#define CONTROL_TASK_PERIOD 1			//任务运行周期ms
TaskHandle_t Control_Task_Handler;		//任务句柄
void Control_Task(void*p_arg);			//任务函数
//裁判系统消息更新任务
#define JUDGE_TASK_PRIO	2			//任务优先级
#define JUDGE_STK_SIZE	128			//任务堆栈大小
#define JUDGE_TASK_PERIOD 10		//任务运行周期ms
TaskHandle_t Judge_Task_Handler;		//任务句柄
void Judge_Task(void*p_arg);			//任务函数
//程序监控任务
#define MONITOR_TASK_PRIO	2			//任务优先级
#define MONITOR_STK_SIZE	128			//任务堆栈大小
#define MONITOR_TASK_PERIOD 1			//任务运行周期ms
TaskHandle_t Monitor_Task_Handler;		//任务句柄
void Monitor_Task(void*p_arg);			//任务函数
//OLED任务
#define OLED_TASK_PRIO	2			//任务优先级
#define OLED_STK_SIZE	512			//任务堆栈大小
#define OLED_TASK_PERIOD 5			//任务运行周期ms
TaskHandle_t OLED_Task_Handler;		//任务句柄
void OLED_Task(void*p_arg);			//任务函数
//LED显示任务是否在运行
#define RUNNING_TASK_PRIO	2			//任务优先级
#define RUNNING_STK_SIZE	50			//任务堆栈大小
#define RUNNING_TASK_PERIOD 500			//任务运行周期ms
TaskHandle_t Running_Task_Handler;		//任务句柄
void Running_Task(void*p_arg);			//任务函数
//------------------------------------------------------------
//任务
//------------------------------------------------------------

/**	
*@brief	任务初始化函数
*/
void Task_Init(void){
	taskENTER_CRITICAL();//进入临界区
	
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
				
	taskEXIT_CRITICAL();//退出临界区
}

/**	
*@brief	命令任务
*/
void CMD_Task(void*p_arg){
	portTickType currentTime;
	while(1){
		currentTime = xTaskGetTickCount();	//获取当前系统时间
		
		RcCmdPrc();
		
		vTaskDelayUntil(&currentTime, CMD_TASK_PERIOD/portTICK_RATE_MS);
	}
}

///**	
//*@brief	CAN通信处理任务
//*/
void CAN_Task(void*p_arg){
	portTickType currentTime;
	while(1){
		currentTime = xTaskGetTickCount();	//获取当前系统时间
		
		CanMsgPrc();
		
		vTaskDelayUntil(&currentTime, CAN_TASK_PERIOD/portTICK_RATE_MS);
	}
}

/**	
*@brief	陀螺仪任务
*/
void IMU_Task(void*p_arg){
	while(1){	
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update(); 
		delay_ms(5);		//获取当前系统时间
	}
}

/**	
*@brief	控制任务
*/
void Control_Task(void*p_arg){
	portTickType currentTime;
	while(1){
		currentTime = xTaskGetTickCount();	//获取当前系统时间
		
		ControlPrc();
		
		vTaskDelayUntil(&currentTime, CONTROL_TASK_PERIOD/portTICK_RATE_MS);
	}
}

/**	
*@brief	裁判系统任务
*/
void Judge_Task(void*p_arg){
	portTickType currentTime;
	while(1){
		currentTime = xTaskGetTickCount();	//获取当前系统时间
		
		vTaskDelayUntil(&currentTime, JUDGE_TASK_PERIOD/portTICK_RATE_MS);
	}
}

/**	
*@brief	程序和系统监控任务
*/
void Monitor_Task(void*p_arg){
//	portTickType currentTime;
	while(1){
//		currentTime = xTaskGetTickCount();	//获取当前系统时间
	
		MonitorPrc();
		
//vTaskDelayUntil(&currentTime, MONITOR_TASK_PERIOD/portTICK_RATE_MS);
		delay_ms(MONITOR_TASK_PERIOD/portTICK_RATE_MS);
	}
}

void OLED_Task(void*p_arg){
	portTickType currentTime;
	while(1){
		currentTime = xTaskGetTickCount();	//获取当前系统时间
		
		OLED_Prc();
		
		vTaskDelayUntil(&currentTime, OLED_TASK_PERIOD/portTICK_RATE_MS);
	}
}

/**	
*@brief	LED显示任务是否在运行,同时负责系统警告和报错(不要修改)
*/
void Running_Task(void*p_arg){
	portTickType currentTime;
	static bool beep_signal=false;
	while(1){
		currentTime = xTaskGetTickCount();	//获取当前系统时间
		
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

