#ifndef _MAIN_H_
#define _MAIN_H_

/* Choose from USART3 UART7 UART8 USB to print to upper computer*/
#define USE_UART8_TO_REPORT		//只能选其中一个，例如USE_USART3_TO_REPORT

/* Specify the prepare time(ms) */
#define PREPARE_TIME_MS     2000

/* Choose to use PID inline debug */
#define Debug_PID_Online	1
#define Debug_VFric			1

/* Choose to monitor the system tasks */
#define Monitor_Task_Stack	0
#define Monitor_Task_Time	0

/* Choose to monitor the output of IMU */
#define Monitor_IMU_Angle   0
#define Monitor_IMU_Accel   0
#define Monitor_MPU_Accel   0

/* Choose to monitor the output of remoter */
#define Monitor_Remoter     0

/* Change the encoder offset of pitch/yaw motor */
#define Pit_Encoder_Offset  1450
#define Yaw_Encoder_Offset  7260


#endif 
