#include "power.h"
#include "stm32f4xx.h" 

/// @brief A板供电配置
void Power_ctrl_Configuration(){
	GPIO_InitTypeDef gpioInitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	gpioInitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	gpioInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOH, &gpioInitStruct);
}

/// @brief A板供电初始化
void Power_Init(void){
	Power_ctrl_Configuration();
	Power_ON();
}

