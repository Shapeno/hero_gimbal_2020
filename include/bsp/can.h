#ifndef _CAN1_H_
#define _CAN1_H_
/*****************************************************************************
*  @file     can.h                                                       	 *
*  @brief    用于对CAN通信与底盘数据获取                                     *
*  包含了CAN底盘通信的相关枚举和结构体的定义								 *
*  			                                                                 *
*                                                                            *
*  @author   DENG		                                                     *
*  @version  1.0.0.1		                                                 *
*  @date     19/9/8				                                             *
*                                                                            *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2019/09/8  | 1.0.0.1   | DengXY	       | Create file                     *
*----------------------------------------------------------------------------*
*  2019/09/11 | 1.0.1.1   | DengXY	       | Function test                   *
*----------------------------------------------------------------------------*
*  2020/08/3  | 1.0.1.0   | DengXY	       | separate motor funtions         *
*----------------------------------------------------------------------------*
*****************************************************************************/
#include "stm32f4xx.h"
#include "sys.h"

#define Chassis_ID	0x402
#define Gimbal_ID	0x401

//------------------------------------------------------------
//枚举类型与数据结构体定义
//------------------------------------------------------------

/**
 * @brief 接收底盘信息的命令码
 * 
 */
typedef enum
{
	MoveData = 0x00,
}Gimbal_Command_ID_e;
/**
 * @brief 
 * 
 */
typedef enum
{
	GunData 	= 	0x01,
	RoboStateData =	0x02,
}Chassis_Command_ID_e;
/**
 * @brief 裁判系统的枪口数据
 * 
 */
typedef __packed struct
{
    uint8_t 	bulletFreq;
    float  	bulletSpeed;
    uint16_t   shooterHeat;
}gun_data_t;
/**
 * @brief 裁判系统机器人状态数据
 * 
 */
typedef __packed struct
{
    uint8_t 	robot_id;
    uint8_t 	robot_level;
    uint16_t 	gun_cooling_rate;
    uint16_t	gun_cooling_limit;
    uint8_t 	gun_speed_limit;
}robot_status_t;

//初始化函数
void CAN1_Init(void);
void CAN2_Init(void);
//用户函数
void SendChassisSpeed(CAN_TypeDef *CANx, uint8_t mode, int16_t Vx, int16_t Vy, int16_t W);
gun_data_t Get_Gun_Data(void);
robot_status_t Get_Robot_Status(void);

#endif 
