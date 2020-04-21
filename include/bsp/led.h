#ifndef _LED_H_
#define _LED_H_

#include "sys.h"

#define LED_ON  0
#define LED_OFF 1

#define LED_H PGout(1)
#define LED_G PGout(2)
#define LED_F PGout(3)
#define LED_E PGout(4)
#define LED_D PGout(5)
#define LED_C PGout(6)
#define LED_B PGout(7)		//���Թ�
#define LED_A PGout(8)		//USB
#define LED_Red   PEout(11)	//���ڱ���
#define LED_Green PFout(14) //������ʾ�����Ƿ�������

void LED_Configuration(void);
void LED_Init(void);

#endif
