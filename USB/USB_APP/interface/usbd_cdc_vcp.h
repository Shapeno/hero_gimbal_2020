/**
  ******************************************************************************
  * @file    usbd_cdc_vcp.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   Header for usbd_cdc_vcp.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_VCP_H
#define __USBD_CDC_VCP_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdbool.h>

#include "usbd_cdc_core.h"
#include "usbd_conf.h"

#include "main.h"	
/* 用于适应串口调参-----------------------------------------------------------*/
#ifdef USE_USB_TO_REPORT
#define USART_RX_BUF USB_USART_RX_BUF[0]
#define USART_RX_STA USB_USART_RX_STA[0]
#endif

/* 接收和传输缓冲区-----------------------------------------------------------*/
#define USB_USART_REC_LEN	 	1024		//USB串口接收缓冲区最大字节数
extern u8 USB_USART_RX_BUF[2][USB_USART_REC_LEN+1]; 	//接收缓冲,最大USART_REC_LEN个字节.
extern uint32_t USB_USART_RX_LEN[2];
extern u16 USB_USART_RX_STA[2];   					//接收状态标记	
/* Exported typef ------------------------------------------------------------*/
/* The following structures groups all needed parameters to be configured for the 
   ComPort. These parameters can modified on the fly by the host through CDC class
   command class requests. */
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}LINE_CODING;

void usbd_cdc_vcp_Init(void);
void usbsendData(uint8_t* buf, uint32_t len,uint8_t Index);
void usbrecieveData(uint8_t* buf,uint32_t* len,uint8_t Index);
uint16_t VCP_DataTx   (uint8_t data,uint8_t Index);
void usb_printf(uint8_t Index,char* fmt,...); 

#endif /* __USBD_CDC_VCP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
