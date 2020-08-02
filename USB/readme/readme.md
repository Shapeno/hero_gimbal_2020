# USB虚拟串口说明

邓孝云 2020/5/10

## 文件添加

![添加文件](.\files.jpg)

![添加文件](.\heads.jpg)

## 预定义添加

![PreDef](.\PreDef.jpg)

说明：添加DUAL_COM之后编译双串口，不添加DUAL_COM编译单串口。

## 使用说明

主要功能函数在<usbd_cdc_vcp.c>文件中:
void usbd_cdc_vcp_Init(void)：初始化USB，在所有串口收发函数之前调用。

![bsp_pre_init](.\bsp_pre_init.jpg)

void usbsendData(uint8_t* buf, uint32_t len,uint8_t Index)：发送字符串数据，Index代表使用的哪一个串口，len代表字符串长度。

void usbrecieveData(uint8_t* buf,uint32_t* len,uint8_t Index)：接收buffer里面的数据，也可以直接使用变量USB_USART_RX_BUF[Index]。

uint16_t VCP_DataTx   (uint8_t data,uint8_t Index)：用于发送单个数据。

void usb_printf(uint8_t Index,char* fmt,...)：usb的格式化输出，也可以在main.h里面定义USE_USB_TO_REPORT重定向printf函数的输出到usb。