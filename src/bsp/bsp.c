#include "bsp.h"

#include "judge_task.h"
#include "command_task.h"
#include "control_task.h"
#include "can_task.h"
#include "monitor_task.h"
#include "usmart.h"
#include "OLED_task.h"
#include "oled.h"
#include "adc.h"
#include "main.h"

//------------------------------------------------------------
//板级支持
//------------------------------------------------------------
/**	
*@brief	板级支持初始化函数
*/
void BSP_Init(void)
{
	BSP_Pre_Init();
	//陀螺仪与裁判系统
	SPI5_Init();   //控制陀螺仪
	mpu_device_init();
	init_quaternion();	
	
	Judge_Init();
	//基础硬件初始化
	Dbus_Init();
	KEY_Init();
	LED_Init();
	Power_Init();
	Laser_Init();
//	BEEP_Init();
	GUN_Switch_Init();
	YAW_Switch_Init();
	
	CAN_Device_Init();
	ControlVariableInit();
	CommandVariableInit();
	
	//USART6_Init(115200);  //与minipc通讯
	//IWDG_Init();
	
	SPI1_Init();
	oled_init();
	OLED_Button_ADC_init();
	menu_init();
	
#if Debug_PID_Online
	usmart_dev.init(SystemCoreClock/1000000);//串口在线调参，使用了timer4
#endif
}
void BSP_Pre_Init(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init(168);
	usbd_cdc_vcp_Init();
	#ifdef USE_USART3_TO_REPORT
	USART3_Init(115200);  //用于调试
	#endif
	#ifdef USE_UART7_TO_REPORT
	UART7_Init(115200);
	#endif
	#ifdef USE_UART8_TO_REPORT
	UART8_Init(115200);
	#endif
}
