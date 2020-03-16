#ifndef _POWER_H_
#define _POWER_H_

#define Power_ON()   GPIO_SetBits(GPIOH, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5)
#define Power_OFF()  GPIO_ResetBits(GPIOH, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5)

void Power_ctrl_Configuration(void);
void Power_Init(void);

#endif
