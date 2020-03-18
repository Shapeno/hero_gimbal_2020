#ifndef _JUDGE_H
#define _JUDGE_H

#include "sys.h"
#include "stdio.h"
#include "delay.h"
#include "data_fifo.h"
#include "stm32f4xx_conf.h"
#include "string.h"

#define PROTOCAL_FRAME_MAX_SIZE 100

typedef __packed struct {
    uint8_t 	SOF;			//数据帧起始字节，固定值为0xA5;
    uint16_t    DataLength;		//数据帧内Data长度;
    uint8_t     Seq;			//包序号;
    uint8_t     CRC8;			//帧头CRC8;
}frame_header_t;

typedef enum            //解包步骤
{
    STEP_HEADER_SOF = 0,
    STEP_LENGTH_LOW = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16 = 5,
} unpack_step_e;

typedef struct          //整包
{
    fifo_s_t       *data_fifo;
    frame_header_t *p_header;
    uint16_t       data_len;
    uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
    unpack_step_e  unpack_step;
    uint16_t       index;
} unpack_data_t;

typedef enum
{
    GAME_STATE_ID       = 0x0001,   //比赛状态
    GAME_FRUIT_ID       = 0x0002,   //比赛结果
    ROBOT_SURVIVE_ID    = 0x0003,   //机器人存活数据
    FIELD_EVENT_ID      = 0x0101,   //场地事件
    SUPPLY_ACTION_ID    = 0x0102,   //补给站动作标识
    REQUEST_BULLET_ID   = 0x0103,   //请求补弹
    ROBOT_STATE_ID      = 0x0201,   //机器人状态
    REALTIME_POWER_ID   = 0x0202,   //实时功率热量
    ROBOT_POSITION_ID   = 0x0203,   //机器人位置
    ROBOT_BUFF_ID       = 0x0204,   //机器人BUFF
    DRONE_POWER_ID      = 0x0205,   //空中机器人能量
    ROBOT_HURT_ID       = 0x0206,   //伤害数据
    ROBOT_SHOOT_ID      = 0x0207,   //射击数据
    ROBOT_INTERACTIVE_ID= 0x0301    //机器人间交互，发送触发
} judge_data_id_e;

//比赛状态
typedef __packed struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time; 
} ext_game_state_t;

//比赛结果
typedef __packed struct { 
uint8_t winner; 
} ext_game_result_t;

//机器人存活数据
typedef __packed struct { 
uint16_t robot_legion; 
} ext_game_robot_survivors_t;

//场地事件
typedef __packed struct { 
uint32_t event_type; 
} ext_event_data_t;

//补给站动作
typedef __packed struct { 
    uint8_t supply_projectile_id; 
    uint8_t supply_robot_id; 
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num; 
} ext_supply_projectile_action_t;

//请求补弹
typedef __packed struct { 
    uint8_t supply_projectile_id; 
    uint8_t supply_robot_id;
    uint8_t supply_num; 
} ext_supply_projectile_booking_t;

//机器人状态
typedef __packed struct { 
    uint8_t robot_id; 
    uint8_t robot_level; 
    uint16_t remain_HP; 
    uint16_t max_HP; 
    uint16_t shooter_heat0_cooling_rate; 
    uint16_t shooter_heat0_cooling_limit; 
    uint16_t shooter_heat1_cooling_rate; 
    uint16_t shooter_heat1_cooling_limit; 
    uint8_t mains_power_gimbal_output : 1; 
    uint8_t mains_power_chassis_output : 1; 
    uint8_t mains_power_shooter_output : 1; 
} ext_game_robot_state_t;

//实时功率热量
typedef __packed struct { 
    uint16_t chassis_volt; 
    uint16_t chassis_current; 
    float chassis_power; 
    uint16_t chassis_power_buffer; 
    uint16_t shooter_heat0; 
    uint16_t shooter_heat1; 
} ext_power_heat_data_t;

//机器人位置
typedef __packed struct {
    float x; 
    float y; 
    float z; 
    float yaw; 
} ext_game_robot_pos_t;

//机器人增益
typedef __packed struct { 
    uint8_t power_rune_buff; 
}ext_buff_musk_t;

//空中机器人能量状态
typedef __packed struct { 
    uint8_t energy_point; 
    uint8_t attack_time; 
} aerial_robot_energy_t;

//伤害数据
typedef __packed struct { 
    uint8_t armor_id : 4; 
    uint8_t hurt_type : 4; 
} ext_robot_hurt_t;

//射击数据
typedef __packed struct { 
    uint8_t bullet_type; 
    uint8_t bullet_freq; 
    float bullet_speed; 
} ext_shoot_data_t;

//机器人间交互，包头
typedef __packed struct { 
    uint16_t data_cmd_id; 
    uint16_t send_ID; 
    uint16_t receiver_ID; 
}ext_student_interactive_header_data_t;

//客户端自定义数据
typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t masks;
} client_custom_data_t;

//数据段
/*
typedef __packed struct
{
    uint8_t data[]
} robot_interactive_data_t
*/
typedef struct
{
    ext_game_state_t                gameState;
    ext_game_result_t               gameResult;
    ext_game_robot_survivors_t      robotSurvive;
    ext_event_data_t                fieldEvent;
    ext_supply_projectile_action_t  supplyAction;
    ext_game_robot_state_t          robotState;
    ext_power_heat_data_t           powerHeat;
    ext_game_robot_pos_t            robotPosition;
    ext_buff_musk_t                 robotBUFF;
    aerial_robot_energy_t           droneEnergy;
    ext_robot_hurt_t                robotHurt;
    ext_shoot_data_t                robotShoot;
} receive_judge_t;

typedef __packed struct{
    ext_student_interactive_header_data_t header;
    client_custom_data_t data;
}client_custom_data_pack_t;

typedef struct
{
    fifo_s_t           *data_fifo;
    uint16_t           buff_size;
    uint8_t            *buff[2];
    uint16_t           read_index;
    uint16_t           write_index;
} uart_dma_rxdata_t;


float storage2float(uint8_t *storage);
void Judge_Init(void);

uint16_t JUDGE_GET_Main_ShootHeatData(void);
uint16_t JUDGE_GET_Affiliated_ShootHeatData(void);
uint8_t JUDGE_GET_Hero_LevelData(void);
float JUDGE_GET_Chassis_Power(void);
uint16_t JUDGE_GET_Chassis_Power_Buffer(void);
uint16_t JUDGE_GET_MHeat_Limit_Data(void);
void packCustomData(float d1,float d2,float d3,uint8_t msk);

#define COMPUTER_FRAME_BUFLEN 5

#define HEADER_LEN 5
#define CMD_LEN 2
#define CRC_LEN 2

void data_pack_handle(uint8_t *p_data, uint16_t len);
void TIM3_Configuration(void);

void Set_Judge_Send_Data(uint8_t i, float data);
void Set_Judge_Send_Flag(uint8_t i, uint8_t data);
void Judge_unpack(void);
uint32_t send_packed_fifo_data(void);
void packCustomData(float d1,float d2,float d3,uint8_t msk);
extern receive_judge_t judge_recv_mesg;

#endif
