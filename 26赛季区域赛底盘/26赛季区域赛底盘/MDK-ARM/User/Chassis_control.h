#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__

#include "stm32f4xx.h"                  // Device header

#define ALL_PID_INCR 6

#define Front_R 3 										//右前轮
#define Front_L 2 										//左前轮
#define Behind_L 1										//左后轮
#define Behind_R 0										//右后轮

typedef struct
{
  float  chassis_motor_set[4];
	float  A,B,C,D;

	float  vx;        									//前后
	float  vy;        									//左右
	float  wz;        									//旋转(角速度rad/s)
	
	float  vx_set;    									//车体三轴速度的设定
	float  vy_set;
  float  wz_set;	
	float  dr_set; 
	
	float  yaw_target;
  float  follow_angle_set;
	float  follow_angle_set_last;
	float  wz_current;
	float  angle_L;
	int    circle_count;
	float  remainder;
}CHASSIS_Task;
extern CHASSIS_Task Chassis_Task;

typedef enum
{
	Normol_mode = 0,
	Rise_mode = 1,
	Pass_mode = 2,
}Chassis_mode_t;

void Chassis_Mecanum_control(void);
void New_Leg_Control(void);

#endif


