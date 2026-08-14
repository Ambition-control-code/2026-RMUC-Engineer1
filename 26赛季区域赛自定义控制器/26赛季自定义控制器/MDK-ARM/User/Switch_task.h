#ifndef __SWITCH_TASK_H__
#define __SWITCH_TASK_H__

#include "stm32f4xx.h"                  // Device header

typedef struct 
{
	uint16_t Quick_storage_flag;		// 一键存储的微动开关
	uint16_t Claw_flag;				// 控制夹爪的微动开关
	uint8_t LEFT_Quikely;
	uint8_t RIGHT_Quikely;
	uint8_t MID_flag;
}Switch_Flag_t;
extern Switch_Flag_t Switch_flag;

typedef struct 
{
	uint8_t ONE;
	uint8_t TWO;
	uint8_t THREE;
}Difficulty_level_t;
extern Difficulty_level_t Difficulty_level;

void Switchi_task(Switch_Flag_t* Switch_flag_t);



#endif


