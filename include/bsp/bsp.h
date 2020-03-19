#ifndef _BSP_H_
#define _BSP_H_
/*****************************************************************************
*  @file     bsp.c                                                       	 *
*  @brief    �弶֧��ͷ�ļ�		                                             *
*  �����ˣ�                                                                  *
*	(����)������LED���������������Դ��CANͨ�š���ʱ����������				 *
*	(����)ң������������                                                     *
*  ͷ�ļ�                                                                    *
*                                                                            *
*  @author   DENG		                                                     *
*  @version  1.0.1.1		                                                 *
*  @date     19/9/6				                                             *
*                                                                            *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2019/09/11 | 1.0.0.1   | DengXY	       | Create file                     *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/
#include "key.h"
#include "led.h"
#include "beep.h"
#include "power.h"
#include "can.h"
#include "timer.h"
#include "bsp_imu.h"	//#include "imu.h"
#include "spi.h"
#include "gun.h"

#include "usart2.h"
#include "usart3.h"
#include "usart6.h"
#include "uart7.h"
#include "uart8.h"

#include "dbus.h"
#include "laser.h"

void BSP_Pre_Init(void);
void BSP_Init(void);

#endif

