#ifndef __POWER_LIMIT_H__
#define __POWER_LIMIT_H__

#include "stm32f4xx.h"                  // Device header

#define chassis_max_power 120.0f			  //工程底盘功率峰值为120W
#define track_limit 36.0f								//最多给履带电机36W

void Power_Control(float *output,float *output_without_limit,float *Motor_re_speed);
void power_limit(float pow_max, float *Motor_re_speed, float *output_without_limit, float *output);

#endif


