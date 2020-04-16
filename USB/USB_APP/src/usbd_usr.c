#include "usbd_usr.h"
#include "usb_dcd_int.h"
#include <stdio.h> 
#include "usbd_cdc_vcp.h"
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK STM32������
//USBD-USR ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/1/21
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//*******************************************************************************
//�޸���Ϣ
//��
////////////////////////////////////////////////////////////////////////////////// 	
static void     USBD_USR_DeviceReset (uint8_t speed);
static void     USBD_USR_DeviceConfigured (void);
static void     USBD_USR_DeviceSuspended(void);
static void     USBD_USR_DeviceResumed(void);

static void     USBD_USR_DeviceConnected(void);
static void     USBD_USR_DeviceDisconnected(void); 
//��ʾUSB����״̬
//0,û������;
//1,�Ѿ�����;
static vu8 bDeviceState=0;		//Ĭ��û������  
u8 Get_USBConnectState(void)
{
	return bDeviceState;
	
}

extern USB_OTG_CORE_HANDLE  USB_OTG_dev;


//USB OTG �жϷ�����
//��������USB�ж�
void OTG_FS_IRQHandler(void)
{
  	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}  
//ָ��DEVICE_PROP�ṹ��
//USB Device �û��ص�����. 
USBD_Usr_cb_TypeDef USR_cb =
{
  USBD_USR_Init,
  USBD_USR_DeviceReset,
  USBD_USR_DeviceConfigured,
  USBD_USR_DeviceSuspended,
  USBD_USR_DeviceResumed,
  USBD_USR_DeviceConnected,
  USBD_USR_DeviceDisconnected,    
};
//USB Device �û��Զ����ʼ������
void USBD_USR_Init(void)
{
	////printf("USBD_USR_Init\r\n");
} 
//USB Device ��λ
//speed:USB�ٶ�,0,����;1,ȫ��;����,����.
void USBD_USR_DeviceReset (uint8_t speed)
{
	switch (speed)
	{
		case USB_OTG_SPEED_HIGH:
			////printf("USB Device Library v1.1.0  [HS]\r\n");
			break; 
		case USB_OTG_SPEED_FULL: 
			//printf("USB Device Library v1.1.0  [FS]\r\n");
			break;
		default:
			//printf("USB Device Library v1.1.0  [??]\r\n"); 
			break;
	}
}
//USB Device ���óɹ�
void USBD_USR_DeviceConfigured (void)
{
    bDeviceState=CONFIGURED;
	//printf("MSC Interface started.\r\n"); 
} 
//USB Device����
void USBD_USR_DeviceSuspended(void)
{
    bDeviceState=SUSPENDED;
	//printf("Device In suspend mode.\r\n");
} 
//USB Device�ָ�
void USBD_USR_DeviceResumed(void)
{ bDeviceState = ATTACHED; 
	//printf("Device Resumed\r\n");
}
//USB Device���ӳɹ�
void USBD_USR_DeviceConnected (void)
{
	bDeviceState=1;
	//printf("USB Device Connected.\r\n");
}
//USB Deviceδ����
void USBD_USR_DeviceDisconnected (void)
{
	bDeviceState=UNCONNECTED;
	//printf("USB Device Disconnected.\r\n");
} 







