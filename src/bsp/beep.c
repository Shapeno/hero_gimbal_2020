#include "beep.h"
#include "delay.h"

/// @brief存储的音调数据
const uint16_t tone_tab[] = {
  3822,  3405, 3033, 2863, 2551, 2272, 2024,
  1911,  1702, 1526, 1431, 1275, 1136, 1012,
   955,   851,  758,  715,  637,  568,  506,
};
/// @brief存储的启动旋律
const Sound_tone_e Mavic_Startup_music[Startup_Success_music_len] = {
  So5L, La6L, Mi3M, Silent
};
/// @brief 启动旋律
void Sing_Startup_music(void){
	int index = 0;
	while(index < Startup_Success_music_len){
    Sing(Mavic_Startup_music[index++]);
	  delay_ms(200);
	}
}
/// @brief 报错旋律
void Sing_bad_case(void){
	while(1){
	  Sing(La6M);
		delay_ms(1000);
		Sing(Silent);
		delay_ms(1000);
	}
}

/// @brief miniPC连接旋律
void Sing_miniPC_online(void){
    int index = 5;
    while (index >= 0) {
        Sing(Mavic_Startup_music[index--]);
        delay_ms(200);
    }
    Sing(Silent);
}
/// @brief 蜂鸣器初始化
void BEEP_Init(void){
	BEEP_Configuration();
	//Sing_Startup_music();
}
/// @brief 蜂鸣器配置
void BEEP_Configuration(void){
  GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource6,GPIO_AF_TIM12);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOH,&GPIO_InitStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM12,ENABLE);
	TIM_Cmd(TIM12, ENABLE);
}
/// @brief 发声函数
void Sing(Sound_tone_e tone){
  if(Silent == tone)
    BEEP_CH = 0;
  else{
    BEEP_ARR = tone_tab[tone];
    BEEP_CH = tone_tab[tone] / 2;
  }
}
