#include "timer.h"
#include "monitor_task.h"

//static int cnt = 0;
//uint8_t msk = 0;

/** 
@brief 初始化TIM3用于IMU
*/
void TIM3_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, DISABLE);

    TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    /* TIM3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_Cmd(TIM3, ENABLE);
}
/**
@brief 用于监控任务运行时间占比
*/
void TIM6_Init(void){
	TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef nvic;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0x02;
	nvic.NVIC_IRQChannelSubPriority = 0x00;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	tim.TIM_Prescaler = 84-1;    //定时器时钟为72M，分频系数为72-1,定时器3频率为72M/72=1M
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	tim.TIM_Period = 100-1;    //自动重装载为100-1，那么定时器周期就是100us
	TIM_TimeBaseInit(TIM6,&tim);
	TIM_Cmd(TIM6, ENABLE);
	TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}
void TIM6_DAC_IRQHandler(void)  {
	if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET){
		FreeRTOSRunTimeTicks_Add();
	}
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
}


//void TIM4_Init(void){
//  TIM_TimeBaseInitTypeDef tim;
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);    
//  tim.TIM_Period = 0xFFFFFFFF;     
//  tim.TIM_Prescaler = 168-1; 
//  tim.TIM_ClockDivision = TIM_CKD_DIV1;	
//  tim.TIM_CounterMode = TIM_CounterMode_Up;  
//  TIM_ARRPreloadConfig(TIM4, ENABLE);
//  TIM_TimeBaseInit(TIM4, &tim);
//  TIM_ARRPreloadConfig(TIM4, ENABLE);	
//  TIM_PrescalerConfig(TIM4, 0, TIM_PSCReloadMode_Update);
//  TIM_UpdateDisableConfig(TIM4, ENABLE);
//  TIM_Cmd(TIM4,ENABLE);
//}


//void TIM2_Init(void){
//  TIM_TimeBaseInitTypeDef tim;
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
//  tim.TIM_Period = 0xFFFFFFFF;
//  tim.TIM_Prescaler = 84 - 1; 
//  tim.TIM_ClockDivision = TIM_CKD_DIV1;	
//  tim.TIM_CounterMode = TIM_CounterMode_Up;  
//  TIM_ARRPreloadConfig(TIM2, ENABLE);	
//  TIM_TimeBaseInit(TIM2, &tim);
//  TIM_Cmd(TIM2,ENABLE);
//}

//void TIM8_Init(void){
//	TIM_TimeBaseInitTypeDef tim;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
//  tim.TIM_Period = 0xFFFFFFFF;
//  tim.TIM_Prescaler = 168-1;
//  tim.TIM_ClockDivision = TIM_CKD_DIV1;
//  tim.TIM_CounterMode = TIM_CounterMode_Up;  
//	TIM_ARRPreloadConfig(TIM8, ENABLE);
//  TIM_TimeBaseInit(TIM8, &tim);
//  TIM_ARRPreloadConfig(TIM8, ENABLE);
//	TIM_PrescalerConfig(TIM8, 0, TIM_PSCReloadMode_Update);
//	TIM_UpdateDisableConfig(TIM8, ENABLE);
//	TIM_Cmd(TIM8,ENABLE);
//}

//uint32_t Get_Time_Micros(void){
//  return (TIM2->CNT)/1000;
//}

//void TIM6_Start(void){
//  TIM_Cmd(TIM6, ENABLE);	 
//  TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
//  TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
//}

//void TIM2_IRQHandler(void){
//  if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET){
//  	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
//    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//  }
//}

//void TIM6_DAC_IRQHandler(void)  {
//  if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET){
//    TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
//    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
//		static int8_t n=0;
//		static float sss;
//    Control_Task();
//		if(cnt++ > 67){
//			cnt=0;
//			if(PHOTO_SWITCH == PHOTO_SWITCH_ON)		msk |= 0x38;
//			else{
//				if(Get_Flag(Ver_Fric_rotate)) {
//					if(n++<5) msk |= 0x38;
//				  else      msk &= 0x07;
//					if(n==10) n=0;
//				}
//				else msk &= 0x07;
//			}
//			if(*Autoaim*)   msk |= 0x01;
//			else msk &= 0x3E;
//			if(Get_Flag(AB_shoot_command))	{msk |= 0x02;
//			sss = 11111.111111;}
//			else {sss = 0.0000000001;
//				msk &= 0x3D;
//			}
//			}
//			packCustomData(sss,0,0,msk);
//		}
// }
