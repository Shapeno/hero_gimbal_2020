#include "USB_task.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"
#include "led.h"

u8 t;
u8 len;	
u16 times=0;//��ʱ����
u8 usbstatus=0;//����״̬

void USB_Prc(void){
	if(usbstatus!=getusbConnectState())//USB����״̬�����˸ı�.
		{
			usbstatus=getusbConnectState();//��¼�µ�״̬
			if(usbstatus==CONFIGURED)//��ʾUSB���ӳɹ�
			{
				LED_A=LED_ON;//DS1��
			}else//��ʾUSB�Ͽ�
			{
				LED_A=LED_OFF;//DS1��
			}
		}
		usb_printf("%f\r\n",3.1415926535897932354626);
		
//		if(USB_USART_RX_STA&0x8000)
//		{
//			len=USB_USART_RX_STA&0x3FFF;//�õ��˴ν��յ������ݳ���
//			usb_printf("\r\n�����͵���ϢΪ:%d\r\n\r\n",len);
////			for(t=0;t<len;t++)
////			{
////				VCP_DataTx(USB_USART_RX_BUF[t]);//���ֽڷ�ʽ,���͸�USB 
////			}
//			usb_printf("%s",USB_USART_RX_BUF);
//			usb_printf("\r\n\r\n");//���뻻��
//			USB_USART_RX_STA=0;
//		}
}