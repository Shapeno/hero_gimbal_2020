#ifndef _SWITCH_H_
#define _SWITCH_H_

#include "sys.h"

//枪口限位开关
#define BULLET_FILLING  1		//子弹到位
#define  BULLET_IN_PLACE 0		//子弹没到位
#define GUN_SWITCH PFin(0)
//yaw轴光电开关
#define POS_RESET  1		//yaw复位
#define  NO_POS_RESET 0		//yaw未到达复位
#define YAW_SWITCH PFin(1)

void GUN_Switch_Init(void);
void YAW_Switch_Init(void);
uint8_t Reach_Reset_Pos(void);

#endif

