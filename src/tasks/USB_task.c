#include "USB_task.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"
#include "led.h"

#include "usmart.h"
u8 t;
u8 len;	
u16 times=0;//��ʱ����
u8 usbstatus=0;//����״̬

void USB_Prc(void){
	usmart_Prc();
	if(usbstatus!=Get_USBConnectState()){//USB����״̬�����˸ı�.
		usbstatus=Get_USBConnectState();//��¼�µ�״̬
		if(usbstatus==CONFIGURED){//��ʾUSB���ӳɹ�		
			LED_A=LED_ON;//DS1��		
		}else{//��ʾUSB�Ͽ�
			LED_A=LED_OFF;//DS1��
		}
	}
}
