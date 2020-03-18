#ifndef _MONITOR_TASK_H
#define _MONITOR_TASK_H

#include "sys.h"

void MonitorPrc(void);
void FreeRTOSRunTimeTicks_Add(void);
void ShowRunTimeStats(void);
void ShowTaskList(void);
void ConfigureTimeForRunTimeStats(void);
unsigned long long getFreeRTOSRunTimeTicks(void);


#endif
