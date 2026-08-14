#ifndef __GET_REMOTE_DATA_H__
#define __GET_REMOTE_DATA_H__

#include "stm32f4xx.h"                  // Device header

typedef struct{
	float VX;
	float VY;
	int16_t turn_speed;
	uint8_t  Stop;
	uint8_t Chassis_mode;
	uint8_t Quick_storage_sate;
	uint8_t shoutui;
	uint8_t ASR_flag;
	uint8_t v;
	uint8_t F;
	uint8_t C;
}Instruction_C_Data;
extern Instruction_C_Data C_Date;

typedef struct{
	uint8_t yaw_t;
	uint8_t Big_pitch_t;
	uint8_t Smoll_pitch_t;
	uint8_t Big_roll_t;
	uint8_t End_pitch_t;
	uint8_t End_roll_t;
	uint8_t claw_t;
	uint8_t data0:1;
	uint8_t data1:1;
	uint8_t data2:1;
	uint8_t data3:1;
	uint8_t data4:1;
	uint8_t data5:1;
	uint8_t data6:1;
	uint8_t data7:1;
}ui_t_data_t;
extern ui_t_data_t ui_t_data;

typedef struct{
	float turn_data;
}Turn_C_Data_t;
extern Turn_C_Data_t Turn_C_Date;

void Get_Control_data(Instruction_C_Data *c_data);
void Get_ui_data(ui_t_data_t *c_data);
void Get_Turn_data(Turn_C_Data_t *turn_c_data);

#endif


