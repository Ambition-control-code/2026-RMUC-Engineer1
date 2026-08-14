#ifndef __SEND_MYDATA_H__
#define __SEND_MYDATA_H__

#include "stm32f4xx.h"                  // Device header

#define FRAME_HEADER_LENGTH 5 		// 帧头数据长度
#define CMD_ID_LENGTH 2       		// 命令码ID数据长度
#define DATA_LENGTH 30        		// 数据段长度
#define FRAME_TAIL_LENGTH 2   		// 帧尾数据长度
#define HAL_MAX_DELAY      0xFFFFFFFFU

#define DATA_FRAME_LENGTH (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH) // 整个数据帧的长度

#define CONTROLLER_CMD_ID 0x0302  // 自定义控制器命令码
#define BUY_AMMO_CMD_ID 0x0306 		//一键买弹的命令码

#define BUY_AMMO_DATA_LENGTH 8		//一键买弹的数据长度
#define DATA_FRAME_LENGTH_2 (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + BUY_AMMO_DATA_LENGTH + FRAME_TAIL_LENGTH) // 整个数据帧的长度

#define MAX_retry_count 6					//DMA串口发送失败后最大重复尝试再次发送的次数

#define BigPitch_setangle 62.0f
#define SmollPitch_setangle -86.5f
#define EndPitch_setangle -25.35f

//==================================================================
typedef __packed struct
{
    __packed struct
    {
        uint8_t sof;              // 起始字节，固定值为0xA5
        uint16_t data_length;     // 数据帧中data的长度
        uint8_t seq;              // 包序号
        uint8_t crc8;             // 帧头CRC8校验
    } frame_header;               // 帧头
    __packed uint16_t cmd_id;     // 命令码
    __packed uint8_t data[30];    // 自定义控制器的数据帧
    __packed uint16_t frame_tail; // 帧尾CRC16校验
}Controller_t;                    // 自定义控制器数据包，这是裁判系统要的所有数据，包括帧头帧尾校验等
extern Controller_t tx_data;

typedef	union
{
	struct
	{
		float Smoll_Pitch_data;      	// 自控小臂Pitch轴数据
		float control_yaw;       			// 自控C板yaw轴数据
		float Big_Pitch_data;        	// 自控大疆3508屁股控制大Pitch数据
		float Roll_3508_data;   			// 自控大疆3508屁股控制roll数据
		float endPitch_data;					// 自控末端Pitch轴3508屁股的数据
		float endRoll_data;						// 自控末端Roll轴3508屁股的数据
		uint16_t Claw_flag;						// 自控夹取的微动标志位
		uint8_t Left_flag:1;					// 自控左面快捷存储切换的标志位
		uint8_t Right_flag:1;					// 自控右面快捷存储切换的标志位
		uint8_t  data1:6;							// 自控预留数据位1
		uint16_t Quick_storage_flag;	// 自控快捷存储能量单元的微动标志位
		uint8_t  data2;								// 自控预留数据位2
	}ALL_data;
	uint8_t data[30];								//总的数据存储数组
}controller_union_data;       		//这里是共用体数据，其中都是起作用的作用数据
extern controller_union_data Tx_custom_data;

//======================自控数据包=============================================================

//=============================================================================================
typedef __packed struct
{
    __packed struct
    {
        uint8_t sof;              // 起始字节，固定值为0xA5
        uint16_t data_length;     // 数据帧中data的长度
        uint8_t seq;              // 包序号
        uint8_t crc8;             // 帧头CRC8校验
    } frame_header;               // 帧头
    __packed uint16_t cmd_id;     // 命令码
    __packed uint8_t data[8];     // 自定义控制器的数据帧
    __packed uint16_t frame_tail; // 帧尾CRC16校验
}Buy_Ammo_t;                      // 一键买弹数据包，这是裁判系统要的所有数据，包括帧头帧尾校验等
extern Buy_Ammo_t tx_data2;

typedef struct
{
	struct
	{
		uint16_t key_value;
		uint16_t x_position : 12;
		uint16_t mouse_left : 4;
		uint16_t y_position : 12;
		uint16_t mouse_right : 4;
		uint16_t reserved;
	}USE_DATA;
	uint8_t Send_data[8];
}custom_client_data_t;
extern custom_client_data_t custom_client_data;
//============================一键买弹数据包====================================================


void MY_UART1_DMA_TX_Init(void);
void Custos_Control_Serial(void);
void Buy_Ammo_Serial(void);

#endif


