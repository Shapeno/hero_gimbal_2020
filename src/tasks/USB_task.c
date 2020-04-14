#include "USB_task.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"
#include "led.h"

u8 t;
u8 len;	
u16 times=0;//延时计数
u8 usbstatus=0;//连接状态

void USB_Prc(void){
	if(usbstatus!=getusbConnectState())//USB连接状态发生了改变.
		{
			usbstatus=getusbConnectState();//记录新的状态
			if(usbstatus==CONFIGURED)//提示USB连接成功
			{
				LED_A=LED_ON;//DS1亮
			}else//提示USB断开
			{
				LED_A=LED_OFF;//DS1灭
			}
		}
		usb_printf("%f\r\n",3.1415926535897932354626);
		
//		if(USB_USART_RX_STA&0x8000)
//		{
//			len=USB_USART_RX_STA&0x3FFF;//得到此次接收到的数据长度
//			usb_printf("\r\n您发送的消息为:%d\r\n\r\n",len);
////			for(t=0;t<len;t++)
////			{
////				VCP_DataTx(USB_USART_RX_BUF[t]);//以字节方式,发送给USB 
////			}
//			usb_printf("%s",USB_USART_RX_BUF);
//			usb_printf("\r\n\r\n");//插入换行
//			USB_USART_RX_STA=0;
//		}
}