#ifndef _SWITCH_H_
#define _SWITCH_H_

#include "sys.h"

//ǹ����λ����
#define BULLET_FILLING  1		//�ӵ���λ
#define  BULLET_IN_PLACE 0		//�ӵ�û��λ
#define GUN_SWITCH PFin(0)
//yaw���翪��
#define POS_RESET  1		//yaw��λ
#define  NO_POS_RESET 0		//yawδ���︴λ
#define YAW_SWITCH PFin(1)

void GUN_Switch_Init(void);
void YAW_Switch_Init(void);
uint8_t Reach_Reset_Pos(void);

#endif

