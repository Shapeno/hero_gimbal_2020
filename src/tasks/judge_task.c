#include "judge_task.h"
#include "command_task.h"
#include "control_task.h"
#include "usart2.h"
#include <math.h>
#include "main.h"

#define JUDGE_FIFO_BUFLEN 800

bool judge_data_ready=false;
	bool IfJudgeDataReady(void){return judge_data_ready;}
bool judge_tx_data_ready=false;
	bool IfJudgeTxDataReady(void){return judge_tx_data_ready;}

uint8_t judge_dma_rxbuff[2][400];
unsigned char switch_buffer[20];
uint8_t data[13];
uint8_t computer_tx_buf[100];
static uart_dma_rxdata_t judge_rx_obj;
static fifo_s_t  judge_rxdata_fifo;
static fifo_s_t  judge_txdata_fifo;
static uint8_t   judge_rxdata_buf[JUDGE_FIFO_BUFLEN];
static uint8_t   judge_txdata_buf[100];
static unpack_data_t judge_unpack_obj;
receive_judge_t judge_recv_mesg;
static client_custom_data_pack_t clientData;

const uint8_t CRC8_INIT = 0xff;
const uint8_t CRC8_TAB[256] = {
  0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
  0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
  0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
  0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
  0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
  0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
  0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
  0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
  0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
  0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
  0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
  0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
  0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
  0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
  0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
  0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

uint16_t       CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/*
** Descriptions: CRC8 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint8_t get_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength, uint8_t ucCRC8)
{
    uint8_t ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = get_crc8_check_sum(pchMessage, dwLength - 1, CRC8_INIT);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength)
{
    uint8_t ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return;
    ucCRC = get_crc8_check_sum((uint8_t*)pchMessage, dwLength - 1, CRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t get_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
        return 0xFFFF;
    while (dwLength--) {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = get_crc16_check_sum(pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return;
    }
    wCRC = get_crc16_check_sum((uint8_t*)pchMessage, dwLength - 2, CRC_INIT);
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}

/*******************接口**************************************/

uint16_t JUDGE_GET_Affiliated_ShootHeatData(void)
{
    return judge_recv_mesg.powerHeat.shooter_heat0;
}

uint16_t JUDGE_GET_Main_ShootHeatData(void)
{
    return judge_recv_mesg.powerHeat.shooter_heat1;
}

uint16_t JUDGE_GET_MHeat_Limit_Data(void)
{
    return judge_recv_mesg.robotState.shooter_heat1_cooling_limit;
}

uint8_t JUDGE_GET_Hero_LevelData(void)
{
    return judge_recv_mesg.robotState.robot_level;
}

float JUDGE_GET_Chassis_Power(void){
    return judge_recv_mesg.powerHeat.chassis_power;
}

uint16_t JUDGE_GET_Chassis_Power_Buffer(void){
		return judge_recv_mesg.powerHeat.chassis_power_buffer;
}

//typedef __packed struct {            0x0103
//    uint8_t supply_projectile_id;    1   2
//    uint8_t supply_robot_id;         1   11
//    uint8_t supply_num;              50
//} ext_supply_projectile_booking_t;



/**********************************************************/


/*
void Set_Judge_Send_Data(uint8_t i, float Tmp_Data)
{
    switch (i){
    case 1:
        Judge_Send_Data.data1 = Tmp_Data;break;
    case 2:
        Judge_Send_Data.data2 = Tmp_Data;break;
    case 3:
        Judge_Send_Data.data3 = Tmp_Data;break;
    default:
        break;
    }
    return;
}

void Set_Judge_Send_Flag(uint8_t i, uint8_t Tmp_Flag)
{
    if (i > 5)return;
    if (Tmp_Flag == 0)
        Judge_Send_Data.mask &= ~(0x01 << i);
    else
        Judge_Send_Data.mask |= (0x01 << i);
}

*/

void dma_buffer_to_unpack_buffer(uart_dma_rxdata_t *dma_obj) {
    int16_t  tmp_len;
    uint8_t  current_memory_id;
    uint16_t remain_data_counter;
    uint8_t  *pdata = dma_obj->buff[0];

    current_memory_id = DMA_GetCurrentMemoryTarget(DMA1_Stream5);
    remain_data_counter = DMA_GetCurrDataCounter(DMA1_Stream5);

    //printf("%d",current_memory_id);
    if (current_memory_id)
        dma_obj->write_index = dma_obj->buff_size * 2 - remain_data_counter;
    else
        dma_obj->write_index = dma_obj->buff_size - remain_data_counter;
    if (dma_obj->write_index < dma_obj->read_index) {
        tmp_len = dma_obj->buff_size * 2 - dma_obj->read_index;

        if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len)) {
            //FIFO overflow
        }
        dma_obj->read_index = 0;

        //DMA当前缓存区
        tmp_len = dma_obj->write_index;
        if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len)) {
            //overflow
        }
        dma_obj->read_index = dma_obj->write_index;
    }
    else
    {
        tmp_len = dma_obj->write_index - dma_obj->read_index;
        if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len)) {
            ;//fifo_overflow
        }
        dma_obj->read_index = (dma_obj->write_index) % (dma_obj->buff_size * 2);
    }
    judge_data_ready=true;
}

void judgement_data_handler(uint8_t *p_frame)
{
    frame_header_t *p_header = (frame_header_t*)p_frame;
    memcpy(p_header, p_frame, HEADER_LEN);

    uint16_t data_length = p_header->DataLength;
    uint16_t cmd_id = *(uint16_t *)(p_frame + HEADER_LEN);
    uint8_t *data_addr = p_frame + HEADER_LEN + CMD_LEN;
    //printf("%d,%d\r\n",cmd_id,data_length);
    
    switch (cmd_id) {
    case GAME_STATE_ID:
        memcpy(&judge_recv_mesg.gameState, data_addr, data_length);
        break;

    case GAME_FRUIT_ID:
        memcpy(&judge_recv_mesg.gameResult, data_addr, data_length);
        break;

    case ROBOT_SURVIVE_ID:
        memcpy(&judge_recv_mesg.robotSurvive, data_addr, data_length);
        break;

    case FIELD_EVENT_ID:
        memcpy(&judge_recv_mesg.fieldEvent, data_addr, data_length);
        break;

    case SUPPLY_ACTION_ID:
        memcpy(&judge_recv_mesg.supplyAction, data_addr, data_length);
        break;

    case ROBOT_STATE_ID:
        memcpy(&judge_recv_mesg.robotState, data_addr, data_length);
        break;

    case REALTIME_POWER_ID:
        memcpy(&judge_recv_mesg.powerHeat, data_addr, data_length);
        break;
    
    case ROBOT_POSITION_ID:
        memcpy(&judge_recv_mesg.robotPosition, data_addr, data_length);
        break;
    
    case ROBOT_BUFF_ID:
        memcpy(&judge_recv_mesg.robotBUFF, data_addr, data_length);
        break;
    
    case DRONE_POWER_ID:
        memcpy(&judge_recv_mesg.droneEnergy, data_addr, data_length);
        break;
    
    case ROBOT_HURT_ID:
        memcpy(&judge_recv_mesg.robotHurt, data_addr, data_length);
        break;
    
    case ROBOT_SHOOT_ID:
        memcpy(&judge_recv_mesg.robotShoot, data_addr, data_length);
        break;
    /***************自定义协议*****************
    case ROBOT_INTERACTIVE_ID:
        memcpy(&judge_recv_mesg.robot_pos_data, data_addr, data_length);
        break;
    *****************************************/
    }

}

void unpack_fifo_data(unpack_data_t *p_obj) {
    uint8_t byte = 0;
    while (fifo_used_count(p_obj->data_fifo)) {
        byte = fifo_s_get(p_obj->data_fifo);
        switch (p_obj->unpack_step) {
        case STEP_HEADER_SOF:
        {
            if (byte == 0xA5)
            {
                p_obj->unpack_step = STEP_LENGTH_LOW;
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            else
            {
                p_obj->index = 0;
            }
        }break;

        case STEP_LENGTH_LOW:
        {
            p_obj->data_len = byte;
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_LENGTH_HIGH;
        }break;

        case STEP_LENGTH_HIGH:
        {
            p_obj->data_len |= (byte << 8);
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
            {
                p_obj->unpack_step = STEP_FRAME_SEQ;
            }
            else
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
        }break;

        case STEP_FRAME_SEQ:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_HEADER_CRC8;
        }break;

        case STEP_HEADER_CRC8:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->index == HEADER_LEN)
            {
                if (verify_crc8_check_sum(p_obj->protocol_packet, HEADER_LEN))
                {
                    p_obj->unpack_step = STEP_DATA_CRC16;
                }
                else
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
        }break;

        case STEP_DATA_CRC16:
        {
            if (p_obj->index < (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            if (p_obj->index >= (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;

                if (verify_crc16_check_sum(p_obj->protocol_packet, HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
                {
                    //处理数据
                    judgement_data_handler(p_obj->protocol_packet);
                }
            }
        }break;

        default:
        {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
        }break;
        }
    }
}

uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
    frame_header_t *p_header = (frame_header_t*)tx_buf;

    p_header->SOF = sof;
    p_header->DataLength = len;
    p_header->Seq = 0;


    memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
    append_crc8_check_sum(tx_buf, HEADER_LEN);
    memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);
    append_crc16_check_sum(tx_buf, frame_length);

    return tx_buf;
}

//打包发包
void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof)
{
    uint8_t tx_buf[PROTOCAL_FRAME_MAX_SIZE];
  
    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
  
    protocol_packet_pack(cmd_id, p_data, len, sof, tx_buf);
  
    fifo_s_puts(&judge_txdata_fifo, tx_buf, frame_length);

    judge_tx_data_ready=true;
}

void packCustomData(float d1,float d2,float d3,uint8_t msk){
    clientData.header.data_cmd_id = 0xD180;
    clientData.header.send_ID = judge_recv_mesg.robotState.robot_id;
    if(judge_recv_mesg.robotState.robot_id < 9) 
        clientData.header.receiver_ID = (uint16_t)judge_recv_mesg.robotState.robot_id + 0x0100;
    else
        clientData.header.receiver_ID = (uint16_t)judge_recv_mesg.robotState.robot_id + 0x0106;
    clientData.data.data1 = d1;
    clientData.data.data2 = d2;
    clientData.data.data3 = d3;
    clientData.data.masks = msk;
    data_packet_pack(0x0301,(uint8_t*)&clientData,19,0xA5);
}

uint32_t send_packed_fifo_data(void)
{
    uint32_t fifo_count = fifo_used_count(&judge_txdata_fifo);

    for(int i = 0;i < fifo_count; i++){
        uint8_t charData = fifo_s_get(&judge_txdata_fifo);
        USART2_SendChar(charData);
    }

    return fifo_count;
}

void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {//空闲中断
        //USART_cmd(USART2,DISABLE);
        USART_ReceiveData(USART2); //清中断
        dma_buffer_to_unpack_buffer(&judge_rx_obj);
        //USART_cmd(USART2,ENABLE);
    }
}

void DMA1_Stream5_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5)) {
        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);

        //DMA 已满（当前缓存？）
        
    }
}

void JudgeSystemDMA_Init(void) {
    DMA_InitTypeDef		DMA_InitStructure;
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

    DMA_DeInit(DMA1_Stream5);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)judge_dma_rxbuff[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = 400; //缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);

    //DMA双缓冲
    DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)&judge_dma_rxbuff[1], DMA_Memory_0);//DMA_Memory_0
    DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);

    DMA_Cmd(DMA1_Stream5, ENABLE);
}

void Judge_Init(void)
{
    USART2_Init();
    JudgeSystemDMA_Init();
    
    fifo_s_init(&judge_rxdata_fifo, judge_rxdata_buf, JUDGE_FIFO_BUFLEN);
    fifo_s_init(&judge_txdata_fifo, judge_txdata_buf, 100);
    judge_rx_obj.buff[0] = judge_dma_rxbuff[0];
    judge_rx_obj.buff[1] = judge_dma_rxbuff[1];
    judge_rx_obj.buff_size = 400;
    judge_rx_obj.data_fifo = &judge_rxdata_fifo;
    judge_rx_obj.read_index = 0;
    judge_rx_obj.write_index = 0;

    judge_unpack_obj.data_fifo = &judge_rxdata_fifo;
    judge_unpack_obj.p_header = (frame_header_t *)judge_unpack_obj.protocol_packet;
    judge_unpack_obj.index = 0;
    judge_unpack_obj.data_len = 0;
    judge_unpack_obj.unpack_step = STEP_HEADER_SOF;
}
void Judge_unpack(void) {
    unpack_fifo_data(&judge_unpack_obj);
}

