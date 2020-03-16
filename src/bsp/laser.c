#include "laser.h"
#include "stm32f4xx.h" 

/// @brief 激光器配置
void Laser_Configuration(){
	GPIO_InitTypeDef gpioInitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	gpioInitStruct.GPIO_Pin = GPIO_Pin_13;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOG, &gpioInitStruct);
	GPIO_ResetBits(GPIOG, GPIO_Pin_13);
}
/// @brief 激光器初始化
void Laser_Init(void){
  Laser_Configuration();
	LASER_ON();
}
