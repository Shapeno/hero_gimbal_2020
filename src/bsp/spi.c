#include "spi.h"
#include "imu.h"

void SPI5_Init(void){	
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef   SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5,ENABLE);

	MPU6500_NSS_High();

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOF, GPIO_PinSource7, GPIO_AF_SPI5);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource8, GPIO_AF_SPI5);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_SPI5);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5,ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5,DISABLE);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI5, &SPI_InitStructure);

	SPI_Cmd(SPI5, ENABLE);	
	SPI_ReadWriteByte(0xff);

	MPU6500_NSS_High();
}

void SPI_SetSpeed(u8 SPI_BaudRatePrescaler){
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI5->CR1&=0XFFC7;
	SPI5->CR1|=SPI_BaudRatePrescaler;
	SPI_Cmd(SPI5,ENABLE);
}

u8 SPI_ReadWriteByte(u8 TxData){
	while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET){}
	SPI_I2S_SendData(SPI5, TxData);
	while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_RXNE) == RESET){}
	return SPI_I2S_ReceiveData(SPI5);	    
}
