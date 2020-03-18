#ifndef _GUN_H_
#define _GUN_H_

#include "sys.h"

#define LIMIT_SWITCH_ON  1
#define LIMIT_SWITCH_OFF 0
				
#define LIMIT_SWITCH PFin(0)

void GUN_Switch_Init(void);

#endif

