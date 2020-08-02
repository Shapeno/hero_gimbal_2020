#include "switch.h"
#include "delay.h"
#include "stm32f4xx.h" 

#include "led.h"

//ǹ����λ����
	
void GUN_Switch_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
}

//yaw���翪��
void YAW_Switch_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
}
/**
 * @brief ��ȡyaw�ᵽ�︴λλ�õĴ���
 * 
 * @return uint8_t ��ʼΪ0�����2��
 */
uint8_t Reach_Reset_Pos(void){
	static uint8_t reach=1;
	static uint32_t time=0;
	if(reach<2){
		if(YAW_SWITCH==POS_RESET){
			if(xTaskGetTickCount()>time){
				reach++;
				time=xTaskGetTickCount()+1000;
			}
		}
	}
	if(reach==2){
		if(YAW_SWITCH==NO_POS_RESET)
			reach=3;
	}
	return reach;
}

