/**
  ******************************************************************************
  * @file    usbd_cdc_vcp.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   Generic media access Layer.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED 
#pragma     data_alignment = 4 
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

#include "usbd_cdc_vcp.h"
#include "usb_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

#include "string.h"	
#include "stdarg.h"		 
#include "stdio.h"	


#ifdef USE_USB_TO_REPORT		//Ҫʹ������main.h�����޸�
#pragma import(__use_no_semihosting)
struct __FILE {
    int handle;
};
void _ttywrch(int ch) 
{ 
ch = ch; 
} 
FILE __stdout;
void _sys_exit(int x) {
    x = x;
}

int fputc(int ch, FILE *f) {
	VCP_DataTx((u8)ch,0);
    return ch;
}
#endif

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

LINE_CODING linecoding[2] =
  {
    500000, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };
  
//#define USB_RX_LEN	 	(2 * 1024)	//USB�������ݻ�����
//uint8_t usb_rx_buf[USB_RX_LEN];
//uint16_t usb_rx_ptr_in = 0;
//uint16_t usb_rx_ptr_out = 0;

/* These are external variables imported from CDC core to be used for IN 
   transfer management. */
extern uint8_t  APP_Rx_Buffer []; /* Write CDC received data in this buffer.
                                     These data will be sent over USB IN endpoint
                                     in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in1;    /* Increment this pointer or roll it back to
                                     start address when writing received data
                                     in the buffer APP_Rx_Buffer. */
#ifdef DUAL_COM
extern uint8_t  APP_Rx2_Buffer []; /* Write CDC received data in this buffer.
                                     These data will be sent over USB IN endpoint
                                     in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in2;    /* Increment this pointer or roll it back to
                                     start address when writing received data
                                     in the buffer APP_Rx_Buffer. */
#endif
//�����ƴ���1�������ݵķ���,������USB���⴮�ڽ��յ�������.
u8 USB_USART_RX_BUF[2][USB_USART_REC_LEN]; 	//���ջ���,���USART_REC_LEN���ֽ�.
uint32_t USB_USART_RX_LEN[2]={0};
u16 USB_USART_RX_STA[2];   					//����״̬���	
void usbrecieveData(u8* buf,uint32_t* len,uint8_t Index){
	buf=USB_USART_RX_BUF[Index];
	*len=USB_USART_RX_LEN[Index];
}


/* Private function prototypes -----------------------------------------------*/
static uint16_t VCP_Init     (uint8_t Index);
static uint16_t VCP_DeInit   (uint8_t Index);
static uint16_t VCP_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len,uint8_t Index);
uint16_t VCP_DataTx   (uint8_t data,uint8_t Index);
static uint16_t VCP_DataRx   (uint8_t* buf, uint32_t Len,uint8_t Index);

CDC_IF_Prop_TypeDef VCP_fops[2] = 
{
  VCP_Init,
  VCP_DeInit,
  VCP_Ctrl,
  VCP_DataTx,
  VCP_DataRx
};

static uint16_t VCP_Init(uint8_t Index)
{
  return USBD_OK;
}

static uint16_t VCP_DeInit(uint8_t Index)
{
  return USBD_OK;
}


/**
  * @brief  VCP_Ctrl
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len,uint8_t Index)
{ 
  switch (Cmd)
  {
  case SEND_ENCAPSULATED_COMMAND:
    /* Not  needed for this driver */
    break;

  case GET_ENCAPSULATED_RESPONSE:
    /* Not  needed for this driver */
    break;

  case SET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case GET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case CLEAR_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case SET_LINE_CODING:
    linecoding[Index].bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8) | (Buf[2] << 16) | (Buf[3] << 24));
    linecoding[Index].format = Buf[4];
    linecoding[Index].paritytype = Buf[5];
    linecoding[Index].datatype = Buf[6];
    break;

  case GET_LINE_CODING:
    Buf[0] = (uint8_t)(linecoding[Index].bitrate);
    Buf[1] = (uint8_t)(linecoding[Index].bitrate >> 8);
    Buf[2] = (uint8_t)(linecoding[Index].bitrate >> 16);
    Buf[3] = (uint8_t)(linecoding[Index].bitrate >> 24);
    Buf[4] = linecoding[Index].format;
    Buf[5] = linecoding[Index].paritytype;
    Buf[6] = linecoding[Index].datatype; 
    break;

  case SET_CONTROL_LINE_STATE:
    /* Not  needed for this driver */
    break;

  case SEND_BREAK:
    /* Not  needed for this driver */
    break;    
    
  default:
    break;
  }

  return USBD_OK;
}

/**
  * @brief  VCP_DataTx
  *         CDC received data to be send over USB IN endpoint are managed in 
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
uint16_t VCP_DataTx (uint8_t data,uint8_t Index)
{
	if(Index==0){
	  if (linecoding[0].datatype == 7)
	  {
		APP_Rx_Buffer[APP_Rx_ptr_in1] = data & 0x7F;
	  }
	  else if (linecoding[0].datatype == 8)
	  {
		APP_Rx_Buffer[APP_Rx_ptr_in1] = data;
	  }
		
	  APP_Rx_ptr_in1++;
	  
	  /* To avoid buffer overflow */
	  if(APP_Rx_ptr_in1 == APP_RX_DATA_SIZE)
	  {
		APP_Rx_ptr_in1 = 0;
	  }  
	}
#ifdef DUAL_COM
	if(Index==1){
	  if (linecoding[Index].datatype == 7)
	  {
		APP_Rx2_Buffer[APP_Rx_ptr_in2] = data & 0x7F;
	  }
	  else if (linecoding[Index].datatype == 8)
	  {
		APP_Rx2_Buffer[APP_Rx_ptr_in2] = data;
	  }
		
	  APP_Rx_ptr_in2++;
	  
	  /* To avoid buffer overflow */
	  if(APP_Rx_ptr_in2 == APP_RX_DATA_SIZE)
	  {
		APP_Rx_ptr_in2 = 0;
	  }  
	}
#endif
  return USBD_OK;
}

/**
  * @brief  VCP_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
static uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len,uint8_t Index)
{
	memset(USB_USART_RX_BUF[Index],0,USB_USART_REC_LEN);
	memcpy(USB_USART_RX_BUF[Index],Buf,Len);
	
	USB_USART_RX_LEN[Index]=Len;
	USB_USART_RX_STA[Index]|=0x8000;
	USB_USART_RX_STA[Index]|=0x4000;
	USB_USART_RX_STA[Index]|=((USB_USART_RX_LEN[Index]-2)&0X3FFF);
  return USBD_OK;
}

void usbd_cdc_vcp_Init(void)
{
	// ��ʼ��usb
	USBD_Init(&USB_OTG_dev,
		USB_OTG_FS_CORE_ID,
		&USR_desc,
		&USBD_CDC_cb,
		&USR_cb);
}

u8  USART_PRINTF_Buffer[200];	//usb_printf���ͻ�����

//usb���⴮��,printf ����
//ȷ��һ�η������ݲ���USB_USART_REC_LEN�ֽ�
void usb_printf(uint8_t Index,char* fmt,...)  
{  
	u16 i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART_PRINTF_Buffer,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART_PRINTF_Buffer);//�˴η������ݵĳ���
	for(j=0;j<i;j++)//ѭ����������
	{
		VCP_DataTx(USART_PRINTF_Buffer[j],Index); 
	}
} 

void usbsendData(uint8_t* buf, uint32_t len,uint8_t Index)
{
	for(uint32_t i=0; i<len; i++)//ѭ����������
	{
		VCP_DataTx(buf[i],Index); 
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
