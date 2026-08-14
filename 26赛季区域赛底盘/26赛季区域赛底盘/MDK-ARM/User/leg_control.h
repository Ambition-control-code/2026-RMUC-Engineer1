#ifndef __LEG_CONTROL_H__
#define __LEG_CONTROL_H__

#include "stm32f4xx.h"                  // Device header

// 四维弹簧阻尼系统
typedef struct 
{
	// 弹簧参数 (Nm/rad)
	float stiffness_front_back;    // 前后方向刚度
	
	float left_leg_target_angle;   // 关节目标角度 (rad) -> 默认腿应该弯曲多少度
	float right_leg_target_angle;  // 关节目标角度 (rad) -> 默认腿应该弯曲多少度
	
	// 阻尼参数 (Nm·s/rad)
	float damping_front_back;      // 前后方向阻尼
	
	float balance_pitch_kp;       // 姿态P增益 (Nm/rad) -> 只有车身歪了才用力
  float balance_pitch_kd;       // 姿态D增益 (Nm·s/rad) -> 抑制晃动
  float balance_roll_kp;        // 横滚P增益
  float balance_roll_kd;        // 横滚D增益
	
	float acc_gain;								 // 加速度增益
	float length;									 // 关节到质心的距离（m）
	float mass;										 // 相关质量（kg）
	
	// 参考平衡位置
	float pitch_ref;              // 俯仰平衡角度
	float roll_ref;               // 横滚平衡角度
	
	// 约束限制
	float max_torque;
	float min_torque;
	float max_angle;              // 最大允许倾斜角
	
	float pitch_error;
	float gyro_y_error;
	float spring_torque_pitch;
	float damping_torque_pitch;
	
	float roll_error;
	float gyro_x_error;
	float spring_torque_roll;
	float damping_torque_roll;
	
	float gravity_comp_gain;
	
	float Left_all_angle;
	float Right_all_angle;
	
	float Left_kp;
	float Right_kp;
	float Left_kd;
	float Right_kd;
	float Left_torque;
	float Right_torque;
} SpringDamperSystem_t;
extern SpringDamperSystem_t SpringDamperSystem;

typedef struct 
{
	float p_des;
	float v_des;
	float kp;
	float kd;
	float t_ff;
	float filtered_error;
	float FDN_Tff;
}DmMotorCmd_t;
extern DmMotorCmd_t MIT_L_leg;
extern DmMotorCmd_t MIT_R_leg;



void SpringDamper_Init(SpringDamperSystem_t* sys);
void SpringDamper_Update(SpringDamperSystem_t* sys);
void Pos_control_leg(SpringDamperSystem_t* sys);
void Suspension_With_Limits_Update(float target_pos, float current_pos, float current_vel, DmMotorCmd_t* output);
void Target_angle_integral(SpringDamperSystem_t* sys);

#endif


