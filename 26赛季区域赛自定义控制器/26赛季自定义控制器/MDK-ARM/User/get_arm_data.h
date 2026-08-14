#ifndef __GET_ARM_DATA_H__
#define __GET_ARM_DATA_H__

#include "stm32f4xx.h"                  // Device header

typedef union
{
	struct
	{
		float M1_Yaw;
		float M2_Big_Pitch;
		float M3_Smoll_Pitch;
		float M4_Big_Roll;
		float M5_End_Pitch;
		float M6_End_Roll;
		uint8_t yaw_joint1_flag:1;
		uint8_t Big_pitch_flag:1;
		uint8_t Smoll_pitch_flag:1;
		uint8_t Big_roll_flag:1;
		uint8_t end_pitch_flag:1;
		uint8_t	end_roll_flag:1;
		uint8_t data1:1;
		uint8_t	data2:1;
		uint8_t data3;
		uint8_t data4;
		uint8_t data5;
		uint8_t data6;
		uint8_t data7;
	}Arm_data_t;
	uint8_t RX_Arm_data[30];
}Arm_union_data_t;

void Back_data_control(void);

#endif


