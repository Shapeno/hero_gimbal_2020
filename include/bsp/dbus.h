/*****************************************************************************
*  @file     dbus.h                                                       	 *
*  @brief    用于对遥控器信息解包                                            *
*  包含了遥控器数据结构体定义，遥控器各通道参数说明，用户函数的声明			 *
*  		                                                                     *
*  @author   DENG		                                                     *
*  @version  1.0.1.1		                                                 *
*  @date     19/9/6				                                             *
*                                                                            *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2019/09/6  | 1.0.0.1   | DengXY	       | Create file                     *
*----------------------------------------------------------------------------*
*  2019/09/11 | 1.0.1.1   | DengXY	       | Function test                   *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/
#ifndef _DEBUS_H_
#define _DEBUS_H_

#include "sys.h"

//------------------------------------------------------------
//常量定义
//------------------------------------------------------------
///	@brief接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
#define DBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u
///	@brief 遥控器通道中间偏移值
#define REMOTE_CONTROLLER_OFFSET      (1024u) 

//------------------------------------------------------------
//数据结构体定义
//------------------------------------------------------------

/**
@brief	遥控器switch开关通道
	在某位置则为1，不在该位置则为0
*/
typedef __packed struct{
	uint8_t up:1;
	uint8_t mid:1;
	uint8_t down:1;
}Sw_Data_t;
/**
@brief	遥控器通道
	ch[0]~ch[3]遥控器拨杆(已经减去中间偏移量)：
		最大值 660 右/上
		中间值 0
		最小值 -660  左/下
	ch[4]遥控器左上角拨轮(已经减去中间偏移量)：
		最大值 660 下
		中间值 0
		最小值 -660 上
	s1~s2:
		上 1
		中 3
		下 2
*/
typedef __packed struct{
	int16_t ch[5];
	Sw_Data_t s1;		///<左
	Sw_Data_t s2;		///<右
}Remote_Data_t;

/**
@brief	鼠标通道
	x,y,z:
		最大值 32767 右 下
		最小值‐32768 左 上
		静止值 0	
	press_l，press_r:
		按下	1
		没按下	0
*/
typedef __packed struct{
	int16_t x;				///<鼠标x轴
	int16_t y;				///<鼠标y轴
	int16_t z;				///<鼠标z轴实际用不了
	uint8_t press_l;
	uint8_t press_r;
}Mouse_Data_t;

/**
@brief	键盘通道
		按下	1
		没按下	0
*/
typedef	__packed struct{
	uint16_t W:1;
	uint16_t S:1;
	uint16_t A:1;
	uint16_t D:1;
	uint16_t SHIFT:1;
	uint16_t CONTRL:1;
	uint16_t Q:1;
	uint16_t E:1;
	uint16_t R:1;
	uint16_t F:1;
	uint16_t G:1;
	uint16_t Z:1;
	uint16_t X:1;
	uint16_t C:1;
	uint16_t V:1;
	uint16_t B:1;
}Key_Data_t;

/**
@brief	控制信息
*/
typedef __packed struct{
	Remote_Data_t rc;
	Mouse_Data_t  mouse;
	Key_Data_t    key;
}RC_Data_t;

//------------------------------------------------------------
//用户函数
//------------------------------------------------------------
void Dbus_Init(void);
RC_Data_t GetRcData(void);
RC_Data_t GetLastRcData(void);
portTickType GetRemoteDataTime(void);


#endif
