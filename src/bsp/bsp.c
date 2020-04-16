#include "bsp.h"

#include "judge_task.h"
#include "command_task.h"
#include "control_task.h"
#include "can_task.h"
#include "monitor_task.h"
#include "OLED_task.h"
#include "usmart.h"
#include "oled.h"
#include "adc.h"
#include "main.h"
#include "usbd_cdc_vcp.h"
//#include "usbd_usr.h"
//------------------------------------------------------------
//�弶֧��
//------------------------------------------------------------
/**	
*@brief	�弶֧�ֳ�ʼ������
*/
void BSP_Init(void)
{
	BSP_Pre_Init();
	//�����������ϵͳ
	SPI5_Init();   //����������


//	imu_init();
	mpu_device_init();
	init_quaternion();	
	
	Judge_Init();
	//����Ӳ����ʼ��
	Dbus_Init();
	KEY_Init();
	LED_Init();
	Power_Init();
	Laser_Init();
	BEEP_Init();
	GUN_Switch_Init();
	
	CAN_Device_Init();
	ControlVariableInit();
	CommandVariableInit();
	
	//USART6_Init(115200);  //��minipcͨѶ
	//IWDG_Init();
#if Debug_PID_Online
	usmart_dev.init(SystemCoreClock/1000000);//�������ߵ��Σ�ʹ����timer4
#endif
	SPI1_Init();
	oled_init();
	OLED_Button_ADC_init();
	menu_init();
}
void BSP_Pre_Init(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init(168);
	#ifdef USE_USART3_TO_REPORT
	USART3_Init(115200);  //���ڵ���
	#endif
	#ifdef USE_UART7_TO_REPORT
	UART7_Init(115200);
	#endif
	#ifdef USE_UART8_TO_REPORT
	UART8_Init(115200);
	#endif
	usbd_cdc_vcp_Init();
}
