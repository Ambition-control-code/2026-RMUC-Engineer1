#ifndef __MY_MATH_H__
#define __MY_MATH_H__

#include "stm32f4xx.h"                  // Device header

/* Row-major index macro */
#define M00(m) (m[0])		//第零行第零列
#define M01(m) (m[1])		//第零行第一列
#define M02(m) (m[2])		//第零行第二列
#define M10(m) (m[3])		//第一行第零列
#define M11(m) (m[4])		//第一行第一列
#define M12(m) (m[5])		//第一行第二列
#define M20(m) (m[6])		//第二行第零列
#define M21(m) (m[7])		//第二行第一列
#define M22(m) (m[8])		//第二行第二列

#define SLOPE_TURN_PI 3.14159265358979323846f	

float MY_ABS(float a);
float Pos_or_Neg(float x);
float InvSqrt(float x);

void slope_turn_build_chassis_C(float yaw_rad, float pitch_rad, float roll_rad, float out_3x3[9]);
void slope_turn_build_yaw(float yaw_rad, float out_3x3[9]);
void slope_turn_build_pitch(float pitch_rad, float out_3x3[9]);
void slope_turn_build_roll(float roll_rad, float out_3x3[9]);
float slope_turn_wrap_pi1(float a);

#endif


