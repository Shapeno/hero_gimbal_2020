#include "usbd_usr.h"
#include "usb_dcd_int.h"
#include <stdio.h> 
#include "usbd_cdc_vcp.h"
#include "usbd_desc.h"
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//USBD-USR 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/21
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//*******************************************************************************
//修改信息
//无
////////////////////////////////////////////////////////////////////////////////// 	
extern uint8_t USBD_DeviceDesc   [USB_SIZ_DEVICE_DESC];

static void     USBD_USR_DeviceReset (uint8_t speed);
static void     USBD_USR_DeviceConfigured (void);
static void     USBD_USR_DeviceSuspended(void);
static void     USBD_USR_DeviceResumed(void);

static void     USBD_USR_DeviceConnected(void);
static void     USBD_USR_DeviceDisconnected(void); 
//表示USB连接状态
//0,没有连接;
//1,已经连接;
static vu8 bDeviceState=0;		//默认没有连接  
u8 Get_USBConnectState(void)
{
	return bDeviceState;
}

extern USB_OTG_CORE_HANDLE  USB_OTG_dev;


//USB OTG 中断服务函数
//处理所有USB中断
void OTG_FS_IRQHandler(void)
{
  	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}  
//指向DEVICE_PROP结构体
//USB Device 用户回调函数. 
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
//USB Device 用户自定义初始化函数
void USBD_USR_Init(void)
{
	usb_printf(1,"Init\r\n");
	////printf("USBD_USR_Init\r\n");
} 
//USB Device 复位
//speed:USB速度,0,高速;1,全速;其他,错误.
void USBD_USR_DeviceReset (uint8_t speed)
{
	usb_printf(1,"Reset\r\n");
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
//USB Device 配置成功
void USBD_USR_DeviceConfigured (void)
{
	usb_printf(1,"Configured\r\n");
    bDeviceState=CONFIGURED;
	//printf("MSC Interface started.\r\n"); 
} 
//USB Device挂起
void USBD_USR_DeviceSuspended(void)
{
#ifdef DUAL_COM
	USBD_DeviceDesc[4]=0xEF;                      /*bDeviceClass*/
	USBD_DeviceDesc[5]=0x02;                       /*bDeviceSubClass*/
	USBD_DeviceDesc[6]=0x01;                       /*bDeviceProtocol*/
#else
	USBD_DeviceDesc[4]=0x00;                      /*bDeviceClass*/
	USBD_DeviceDesc[5]=0x00;                       /*bDeviceSubClass*/
	USBD_DeviceDesc[6]=0x00;                       /*bDeviceProtocol*/
#endif
	usb_printf(1,"Suspended\r\n");
    bDeviceState=SUSPENDED;
	//printf("Device In suspend mode.\r\n");
} 
//USB Device恢复
void USBD_USR_DeviceResumed(void)
{ 	
	usb_printf(1,"Resumed\r\n");
	bDeviceState = ATTACHED; 
	//printf("Device Resumed\r\n");
}
//USB Device连接成功
void USBD_USR_DeviceConnected (void)
{
	usb_printf(1,"Connected\r\n");
	bDeviceState=1;
	//printf("USB Device Connected.\r\n");
}
//USB Device未连接
void USBD_USR_DeviceDisconnected (void)
{
	usb_printf(1,"Disconnected\r\n");
	bDeviceState=UNCONNECTED;
	//printf("USB Device Disconnected.\r\n");
} 








