#include "leg_control.h"
#include "imu.h"
#include "Motor_control.h"
#include "math.h"
#include "filter.h"

//==========================调参指南===============================
//// 先关闭加速度补偿
//sys->acc_gain = 0.0f;

//// 调整弹簧刚度（过大会振荡，过小会软）
//sys->stiffness_front_back = 10.0f;  // 从10开始
//sys->stiffness_left_right = 8.0f;   // 左右可以稍小

//// 调整阻尼（消除振荡）
//sys->damping_front_back = 2.0f;
//sys->damping_left_right = 1.5f;

//// 从小的增益开始
//sys->acc_gain = 0.3f;

//// 根据机器人实际参数调整
//sys->mass = 2.0f;       // 估计的摆动质量
//sys->length = 0.12f;    // 质心到髋关节的水平距离

//// 慢慢增加acc_gain，观察加速/减速时身体的稳定性
//===================================================================

SpringDamperSystem_t SpringDamperSystem = {0};

static inline float constrain(float value, float min_val, float max_val) 
{
	if (value < min_val) return min_val;
	if (value > max_val) return max_val;
	return value;
}

void SpringDamper_Init(SpringDamperSystem_t* sys) 
{
	// 默认参数（需要根据实际调整）
	sys->stiffness_front_back = 0.4f;    // 前后刚度 (Nm/rad)0.6
	sys->damping_front_back = 0.4f;      // 前后阻尼 (Nm·s/rad)0.6
	
	sys->balance_pitch_kp = 6.0f;//
	sys->balance_pitch_kd = 0.0f;//0.01
	sys->balance_roll_kp = 0.0f;
	sys->balance_roll_kd = 0.0f;
	
	// 加速度补偿参数
  sys->mass = 0.0f;                    // 相关质量 (kg) - 需要根据实际调整
  sys->length = 0.415f;                 // 质心到关节距离 (m) - 需要测量
  sys->acc_gain = 0.0f;                // 加速度增益
	
	sys->pitch_ref = 2.2f;               // 水平平衡位置（弧度）
	sys->roll_ref = 0.0f;                // 水平平衡位置（弧度）
	
	sys->left_leg_target_angle = 10.0f;
	sys->right_leg_target_angle = (-10.0f);
	
	sys->gravity_comp_gain = 0.0f;
//	sys->gravity_comp_gain = 27.0f;
	
	sys->max_torque =  2.0f;            	 // 最大扭矩限制
	sys->min_torque = -2.0f;
	
	sys->max_angle = 10.0f;              // 最大倾斜角 (~17度)
}

//=============================================================================================================
//float current_T = 0.0f;
//float balance_pitch_torque = 0.0f;
//float wcis = 0.001f;
//void SpringDamper_Update(SpringDamperSystem_t* sys) 
//{
//	current_T = ((DM_L_leg.tor - DM_R_leg.tor)/2.0f);
//	
//	// 左腿悬挂力 = Kp * (目标腿长 - 当前腿长) - Kd * 腿伸缩速度
//	float left_leg_speed_error = DM_L_leg.vel - 0.0f;
//	if(fabs(left_leg_speed_error) <= 0.3f)	left_leg_speed_error = 0.0f;
//	float left_leg_angle_error = sys->left_leg_target_angle - DM_L_leg.engle;
//	if(fabs(left_leg_angle_error) <= 0.1f)	left_leg_angle_error = 0.0f;
//	
//  float L_suspension_torque = sys->stiffness_front_back * left_leg_angle_error 
//																												- sys->damping_front_back * left_leg_speed_error;

//  // 右腿悬挂力
//	float right_leg_speed_error = DM_R_leg.vel - 0.0f;
//	if(fabs(right_leg_speed_error) <= 0.3)	right_leg_speed_error = 0.0f;
//	float right_leg_angle_error = sys->right_leg_target_angle - DM_R_leg.engle;
//	if(fabs(right_leg_angle_error) <= 0.1f)	right_leg_angle_error = 0.0f;
//	
//  float R_suspension_torque = sys->stiffness_front_back * right_leg_angle_error
//																												- sys->damping_front_back * right_leg_speed_error;
//	
//	// 重力补偿
//	float gravity_torque = sys->mass * 9.8f * sys->length * sys->gravity_comp_gain;
//	
//  L_suspension_torque += gravity_torque;
//  R_suspension_torque += gravity_torque;
//	
//	// 俯仰平衡力 (Pitch Balance)
//  sys->pitch_error = imu_angle.pitch - sys->pitch_ref;
////	float pitch_error_t = WindowFilter(&WindowFilter_pitch,pitch_error);
//  if(fabs(sys->pitch_error) < 1.0f) sys->pitch_error = 0.0f;
//	
////	sys->gyro_y_error = (imu_angle.gyro_y - 0.0f);
////	if(fabs(sys->gyro_y_error) <= 0.5f)	sys->gyro_y_error = 0.0f;
////  float balance_pitch_torque = -sys->balance_pitch_kp * pitch_error + sys->balance_pitch_kd * sys->gyro_y_error;
//	float balance_pitch_torque_t = -sys->balance_pitch_kp * sys->pitch_error;
//	balance_pitch_torque += balance_pitch_torque_t * wcis;
//	if(balance_pitch_torque > 30.0f)	balance_pitch_torque = 30.0f;
//	if(balance_pitch_torque < -30.0f)	balance_pitch_torque = -30.0f;
//	
//	
//  // 横滚平衡力 (Roll Balance)
//  float roll_error = imu_angle.roll - sys->roll_ref;
//  if(fabs(roll_error) < 1.0f) roll_error = 0.0f;
//	sys->gyro_x_error = (imu_angle.gyro_x - 0.0f);
//	if(fabs(sys->gyro_x_error) <= 0.5f)	sys->gyro_x_error = 0.0f;
//	
//	float balance_roll_torque = sys->balance_roll_kp * roll_error - sys->balance_roll_kd * sys->gyro_x_error;
//	
//	// 基础输出 = 各自的悬挂力
//  sys->Left_all_torque  = L_suspension_torque - balance_pitch_torque + balance_roll_torque;
//  sys->Right_all_torque = R_suspension_torque + balance_pitch_torque + balance_roll_torque;
//	
//	sys->Left_all_torque = constrain(sys->Left_all_torque ,sys->min_torque,sys->max_torque);
//	sys->Right_all_torque= constrain(sys->Right_all_torque,sys->min_torque,sys->max_torque);
//}


//void Pos_control_leg(SpringDamperSystem_t* sys)
//{
//	sys->pitch_error = imu_angle.pitch - sys->pitch_ref;
//  if(fabs(sys->pitch_error) < 0.05f) sys->pitch_error = 0.0f;
//	
//	if(fabs(sys->pitch_error) > 6.0f) sys->balance_pitch_kp = 10.0f;
//	else	sys->balance_pitch_kp = 5.0f;
//	
//	float balance_pitch_torque_t = -sys->balance_pitch_kp * sys->pitch_error;
//	
//	if(fabs(sys->Left_all_torque) > 40.0f && fabs(sys->Right_all_torque) > 40.0f)	sys->pitch_ref = 3.3f;
////	else if(fabs(sys->Left_all_torque) > 55.0f && fabs(sys->Right_all_torque) > 55.0f)	sys->pitch_ref = 0.5f;
//	else	sys->pitch_ref = 2.8f;
//	
//	sys->Left_all_torque = (sys->Left_all_torque - (balance_pitch_torque_t * wcis));
//	
//	if(sys->Left_all_torque < 1.5f)	sys->Left_all_torque = 1.5f;
//	if(sys->Left_all_torque > 65.0f)	sys->Left_all_torque = 65.0f;
//	
//	sys->Right_all_torque= (sys->Right_all_torque + (balance_pitch_torque_t * wcis));
//	
//	if(sys->Right_all_torque > (-1.5f))	sys->Right_all_torque = (-1.5f);
//	if(sys->Right_all_torque < (-65.0f))	sys->Right_all_torque = (-65.0f);
//}

//============================以上为遗弃的腿部控制代码，下面的为最新的===========================



//============================以上为遗弃的腿部控制代码，下面的为最新的===========================
#define ANGLE_MIN_LIMIT 0.0f             // 最小安全角度 (单位
#define ANGLE_MAX_LIMIT 10.0f            // 最大安全角度 (单位：度)
#define BUFFER_ZONE_SIZE 2.0f            // 缓冲区大小 (单位：度)，进入缓冲区后开始增加刚度
#define TAU_FF_MIN -2.5f                 // 前馈扭矩最小值
#define TAU_FF_MAX 2.5f                  // 前馈扭矩最大值
#define ERROR_LPF_ALPHA 0.1f             // 误差低通滤波系数
//===========================================================================================

float BUMP_MAX_KP = 500.0f;               // 碰撞时的最大刚度
float BUMP_MAX_KD = 3.0f;                 // 碰撞时的最大阻尼
float K_ADAPT_I = 0.003f;                 // 自适应前馈积分增益
float SUSPENSION_KP = 500.0f;             // 基础刚度
float SUSPENSION_KD = 2.0f;               // 基础阻尼
float KD_COMPRESSION = 0.0f;              // 收缩时的阻尼
float KD_REBOUND = 0.0f;                  // 伸展时的阻尼

/**
 * @brief 主动悬挂核心计算 (建议运行频率 500Hz)
 * @param current_pos 达妙反馈当前位置 (度)
 * @param current_vel 达妙反馈当前速度 (rad/s) -> 新增，用于感知冲击
 * @param dt_seconds 周期 (如 0.002)
 * @param output MIT指令输出
 */
DmMotorCmd_t MIT_L_leg = {0};
DmMotorCmd_t MIT_R_leg = {0};
float penetration = 0.0f;
void Suspension_With_Limits_Update(float target_pos, float current_pos, float current_vel, DmMotorCmd_t* output)
{
    // ---------------------------------------------------------
    // 第一部分：自适应前馈计算 (含悬空保护)
    // ---------------------------------------------------------
    float raw_error = target_pos - current_pos;
    output->filtered_error = (ERROR_LPF_ALPHA * raw_error) + ((1.0f - ERROR_LPF_ALPHA) * output->filtered_error);
    
    // 【关键修改】：如果当前角度已经进入了"飞坡伸直区"或"重压收缩区"，停止积分！
    // 只有在正常行驶区间内，才允许自适应调整重心，防止落地瞬间趴窝。
    if(fabs(current_pos) > (ANGLE_MIN_LIMIT + BUFFER_ZONE_SIZE) && 
       fabs(current_pos) < (ANGLE_MAX_LIMIT - BUFFER_ZONE_SIZE))
    {
        output->t_ff += K_ADAPT_I * output->filtered_error;
        output->t_ff = constrain(output->t_ff, TAU_FF_MIN, TAU_FF_MAX);
    }
		
    // ---------------------------------------------------------
    // 第二部分：虚拟软限位 (防撞机械死点)
    // ---------------------------------------------------------
    output->kp = SUSPENSION_KP;
    output->kd = SUSPENSION_KD;
    output->p_des = target_pos; // 默认目标位置为输入的理想位置，后续可能被调整以防撞

//	if (current_vel < 0.0f) output->kd = KD_COMPRESSION; // 正在收缩：阻尼变软，放行
//	else	output->kd = KD_REBOUND;     // 正在伸展：阻尼变硬，拉住

    // ---------------------------------------------------------
    // 第三部分：装填输出
    // ---------------------------------------------------------
    output->v_des = 0.0f;            // 期望速度为0，保持位置
}



#define Rise_leg_I 0.0012f
void Target_angle_integral(SpringDamperSystem_t* sys)
{
  // 这个函数用来根据当前的误差调整目标角度，形成一个简单的积分效果，帮助机器人更快地找到平衡点
  sys->pitch_ref = 2.8f;
	sys->Right_kp = 500.0f;
	sys->Left_kp = 500.0f;
	sys->Right_kd = 3.0f;
	sys->Left_kd = 3.0f;
	sys->Right_torque = -2.0f;
	sys->Left_torque = 2.0f;

  sys->pitch_error = imu_angle.pitch - sys->pitch_ref;
  if(fabs(sys->pitch_error) < 0.05f) sys->pitch_error = 0.0f;
	
	if(fabs(sys->pitch_error) > 6.0f) sys->balance_pitch_kp = 8.0f;
	else	sys->balance_pitch_kp = 5.0f;
	
	float balance_pitch_torque_t = -sys->balance_pitch_kp * sys->pitch_error;
	
	sys->Left_all_angle = (sys->Left_all_angle - (balance_pitch_torque_t * Rise_leg_I));
	
	if(sys->Left_all_angle < 1.5f)	sys->Left_all_angle = 1.5f;
	if(sys->Left_all_angle > 30.0f)	sys->Left_all_angle = 30.0f;
	
	sys->Right_all_angle= (sys->Right_all_angle + (balance_pitch_torque_t * Rise_leg_I));
	
	if(sys->Right_all_angle > (-3.5f))	sys->Right_all_angle = (-3.5f);
	if(sys->Right_all_angle < (-31.5f))	sys->Right_all_angle = (-31.5f);
}


