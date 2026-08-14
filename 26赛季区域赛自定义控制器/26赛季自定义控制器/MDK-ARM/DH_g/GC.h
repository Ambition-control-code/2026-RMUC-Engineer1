#ifndef __GC_H__
#define __GC_H__

#include "stm32f4xx.h"                  // Device header

#define g 9.80665f

#define Tau_to_Current 7382.2613f
#define RPM_to_FACTOR  0.10472f

#define DEG2RAD (3.1415926f / 180.0f)		//角度转化弧度用的	Π/180

typedef struct
{
	/* 机械参数 */
	float m1;    // 连杆1质量 [kg]
	float d1;    // 连杆1质心到关节1距离 [m]
	float L1;    // 连杆1长度 [m]
	
	float m2;    // 连杆2质量 [kg]
	float d2;    // 连杆2质心到关节2距离 [m]
	
	float mid_angle1;		//机械中值
	float mid_angle2;		//机械中值
	
	/* 预计算系数 */
	float A;     // (m1*d1 + m2*L1) * g
	float B;     // m2 * d2 * g
	
	float gain1;	//关节一施加增益微调
	float gain2;	//关节二施加增益微调
	
	float Kd1;		//一关节阻尼力
	float Kd2;		//二关节阻尼力
	
	/* 输出力矩 [N·m] */
	float tau1;  // 关节1重力补偿力矩
	float tau2;  // 关节2重力补偿力矩
	
	float Current1;	//关节1输出电流
	float Current2;	//关节2输出电流
	
	float speed_dead;
}GravComp_t;
extern GravComp_t GravComp_L;
extern GravComp_t GravComp_R;
extern GravComp_t GravComp_Roll;

void GravComp_Init_L(GravComp_t *gc);
void GravComp_Init_R(GravComp_t *gc);
void GravComp_Init_Roll(GravComp_t *gc);
void GravComp_Calc(GravComp_t *gc,float theta1,float theta2,float omega1,float omega2);

#endif


