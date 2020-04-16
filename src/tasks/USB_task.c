#include "USB_task.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"
#include "led.h"

#include "usmart.h"
u8 t;
u8 len;	
u16 times=0;//延时计数
u8 usbstatus=0;//连接状态

void USB_Prc(void){
	usmart_Prc();
	if(usbstatus!=Get_USBConnectState()){//USB连接状态发生了改变.
		usbstatus=Get_USBConnectState();//记录新的状态
		if(usbstatus==CONFIGURED){//提示USB连接成功		
			LED_A=LED_ON;//DS1亮		
		}else{//提示USB断开
			LED_A=LED_OFF;//DS1灭
		}
	}
}
