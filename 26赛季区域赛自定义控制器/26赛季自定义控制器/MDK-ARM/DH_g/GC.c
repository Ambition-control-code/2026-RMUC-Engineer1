#include "GC.h"
#include "math.h"
#include "arm_math.h"

GravComp_t GravComp_L = {0};
GravComp_t GravComp_R = {0};
GravComp_t GravComp_Roll = {0};

/***重力补偿参数初始化***/
void GravComp_Init_L(GravComp_t *gc)
{
	//初始化机械相关参数
	gc->m1 = 0.5405f;
	gc->m2 = 0.2248f;
	gc->L1 = 0.112f;
	gc->d1 = 0.1045f;
	gc->d2 = 0.04397f;
	
	gc->mid_angle1 = 150.0f;
	gc->mid_angle2 = 45.0f;
	
	//初始化补偿系数的相关参数
	gc->gain1 = 1.15f;
	gc->gain2 = 3.0f;
	gc->Kd1 = 0.12f;
	gc->Kd2 = 0.1f;
	
	gc->speed_dead = 0.1f;
	
	//预计算相关参数
	gc->A = (gc->m1 * gc->d1 + gc->m2 * gc->L1) * g;
	gc->B =  gc->m2 * gc->d2 * g;
}

void GravComp_Init_R(GravComp_t *gc)
{
	//初始化机械相关参数
	gc->m1 = 0.272f;
	gc->m2 = 0.0f;
	gc->L1 = 0.18615f;
	gc->d1 = 0.272f;
	gc->d2 = 0.0f;
	
	gc->mid_angle1 = -116.5f;
	gc->mid_angle2 = 0.0f;
	
	//初始化补偿系数的相关参数
	gc->gain1 = 0.38f;
	gc->gain2 = 1.0f;
	gc->Kd1 = 0.04f;
	gc->Kd2 = 0.0f;
	
	gc->speed_dead = 0.1f;
	
	//预计算相关参数
	gc->A = (gc->m1 * gc->d1 + gc->m2 * gc->L1) * g;
	gc->B =  gc->m2 * gc->d2 * g;
}

void GravComp_Init_Roll(GravComp_t *gc)
{
	//初始化机械相关参数
	gc->m1 = 0.0f;
	gc->m2 = 0.0f;
	gc->L1 = 0.0f;
	gc->d1 = 0.0f;
	gc->d2 = 0.0f;
	
	gc->mid_angle1 = 0.0f;
	gc->mid_angle2 = 0.0f;
	
	//初始化补偿系数的相关参数
	gc->gain1 = 1.0f;
	gc->gain2 = 1.0f;
	gc->Kd1 = 0.02f;
	gc->Kd2 = 0.02f;
	
	gc->speed_dead = 0.8f;
	
	//预计算相关参数
	gc->A = (gc->m1 * gc->d1 + gc->m2 * gc->L1) * g;
	gc->B =  gc->m2 * gc->d2 * g;
}

//====================================================


float cos1,cos12,theta12,damping_tau1,damping_tau2;
/***重力补偿计算函数***/
void GravComp_Calc(GravComp_t *gc,float theta1,float theta2,float omega1,float omega2)
{
	theta1 = theta1 - gc->mid_angle1;
	theta2 = theta2 - gc->mid_angle2;
	
	omega1 = omega1 * RPM_to_FACTOR;
	omega2 = omega2 * RPM_to_FACTOR;
	
	theta12 = theta1 + theta2;
	
  cos1  = arm_cos_f32(theta1  * DEG2RAD);
  cos12 = arm_cos_f32(theta12 * DEG2RAD);

	// 计算阻尼力矩 (方向与速度相反)
  damping_tau1 = (+(gc->Kd1 * omega1));
  damping_tau2 = (+(gc->Kd2 * omega2));
	
  gc->tau1 = (gc->A * cos1 + gc->B * cos12) * gc->gain1;
  gc->tau2 = (gc->B * cos12) * gc->gain2;
	
	if(fabs(omega1) > gc->speed_dead)	gc->tau1 -= damping_tau1;
	if(fabs(omega2) > gc->speed_dead)	gc->tau2 -= damping_tau2;
	
	gc->Current1 = gc->tau1 * Tau_to_Current;
	gc->Current2 = gc->tau2 * Tau_to_Current;
	
	if(gc->Current1 >  16000)	gc->Current1 =  16000;
	if(gc->Current1 < -16000)	gc->Current1 = -16000;
	
	if(gc->Current2 >  16000)	gc->Current2 =  16000;
	if(gc->Current2 < -16000)	gc->Current2 = -16000;
}


