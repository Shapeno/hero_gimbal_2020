#ifndef _USART6_H_
#define _USART6_H_

#include "sys.h"

typedef enum {
    START_UPPER_MONITOR_CTR = 0x00,
    REQUEST_PITCH = 0x01,
    GIMBAL_MOVEBY = 0x02,
    GIMBAL_STOP = 0x03,
    SYN = 0x09,
    ACK = 0x10,
    ACK_STATUS = 0x11,
    NO_CMD = 0xff
}minipc_cmdid_e;

typedef struct{
	uint8_t startFriction;
	uint8_t cmdid;
	float d1;
	float d2;
}UpperMonitor_Ctr_t;

void UpperMonitorDataProcess(uint8_t *pData);
UpperMonitor_Ctr_t GetUpperMonitorCmd(void);
void USART6_Print(uint8_t* ch, int len);
void USART6_sendChar(uint8_t ch);
void USART6_Init(u32 bound);

#endif
