#include "Switch_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "Key.h"

extern osThreadId_t IMUHandle;

Switch_Flag_t Switch_flag = {0};
Difficulty_level_t Difficulty_level = {0};

void Switchi_task(Switch_Flag_t* Switch_flag_t)
{
	Switch_flag_t->Claw_flag = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
	Switch_flag_t->LEFT_Quikely = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6);
	Switch_flag_t->MID_flag = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14);
	Switch_flag_t->RIGHT_Quikely = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
}


