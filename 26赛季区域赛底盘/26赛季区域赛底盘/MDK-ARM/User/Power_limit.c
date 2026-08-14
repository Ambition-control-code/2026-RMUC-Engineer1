#include "Power_limit.h"
#include "MY_math.h"
#include "arm_math.h"
#include "PID.h"
#include "Get_Referee_Data.h"

/***@role: 底盘功率控制函数***/
void Power_Control(float *output,float *output_without_limit,float *Motor_re_speed)
{
	float CT=  1.99688994e-6f;          // 力矩电流系数(20/16384)*(0.3)*(187/3591)/9.55
	float k1 = 2.23e-07;             		// k1	速度损耗系数
	float k2 = 4.153e-07;            		// k2	力矩损耗系数
	float constant = 1.181f;						// 某种系数，可能是摩擦引起的，是静止状态下的电流值，是恒定的损耗
	float initial_give_power[6] = {0};			//PID计算的初始功率
	float scaled_give_power[6] = {0};
	float track_total_power = 0.0f;					//履带电机的总功率
	float initial_total_power = 0.0f;  		  //总功率
	float remaining_power = 0.0f;						//底盘能用的能量
	float chassis_total_power = 0.0f;				//底盘电机的总功率
	
	for(uint8_t i = 0;i < 6;i ++)//计算每个轮子的初始功率
	{
		initial_give_power[i]=
			  output_without_limit[i]*CT*Motor_re_speed[i]+								//机械功率（力矩 * 转速）
		    k1*Motor_re_speed[i]*Motor_re_speed[i]+											//与转速平方相关的损耗
		    k2*output_without_limit[i]*output_without_limit[i]+constant;//与力矩平方相关的损耗【constant恒定损耗】
		
		if(initial_give_power[i] < 0)	continue;													//如果功率为负（可能是制动回馈），跳过不计入总功率
		
		initial_total_power += initial_give_power[i];       						//累加各电机功率得到总功率
	}
	
	 //处理履带电机功率(下标4和5)
  for(uint8_t i = 4; i < 6; i++)
  {
		if(initial_give_power[i] > 0) track_total_power += initial_give_power[i];
  }
	
	if(track_total_power > track_limit) 
  {
		// 如果轨道电机超过36W，进行比例缩放
		float track_scale = track_limit / track_total_power;
		scaled_give_power[4] = initial_give_power[4] * track_scale;
		scaled_give_power[5] = initial_give_power[5] * track_scale;
		track_total_power = track_limit; // 此时轨道实际占用就是36W
  }
	else
	{
		scaled_give_power[4] = initial_give_power[4];
		scaled_give_power[5] = initial_give_power[5];
	}
	
	// 底盘能用的功率 = 总限额 - 轨道实际消耗的功率
  remaining_power = chassis_max_power - track_total_power;
	
	for(uint8_t i = 0; i < 4; i++)
	{
		if(initial_give_power[i] > 0) chassis_total_power += initial_give_power[i];
	}
	
	if(chassis_total_power > remaining_power) 											//是否大于最大功率
	{
		float power_scale = remaining_power / chassis_total_power;					//对功率进行比例缩放
		
		for(uint8_t i = 0; i < 4; i++)
    {
			scaled_give_power[i] = initial_give_power[i] * power_scale;
    }
	}
	else
	{
		for(uint8_t i = 0; i < 4; i++)
    {
			scaled_give_power[i] = initial_give_power[i];
    }
	}
//		for(uint8_t i = 0; i < 4; i++)																	//按电机分配功率
//		{
//			scaled_give_power[i] = initial_give_power[i] * power_scale; 	//得到缩放功率
//			if(scaled_give_power[i] < 0)	continue;												//跳过负功率电机
//			
//			//解二次方程求输出力矩	构建并求解二次方程 ka*x2 + kb*x + kc = 0，其中x为输出力矩
//			ka = k2;
//			kb = CT * Motor_re_speed[i];
//			kc = k1 * Motor_re_speed[i] * Motor_re_speed[i] - scaled_give_power[i] + constant;
//			inside = kb * kb - 4 * ka * kc;

//			if(inside < 0)	continue;																		//如果判别式小于0，无实数解，保持原输出
//			else if(output_without_limit[i] > 0) 												//根据原始力矩方向选择计算公式
//			{
//				arm_sqrt_f32(inside,&cmd);
//				//cmd = InvSqrt(inside);	//上面的可以试试，不好用的话用这个
//				temp = (-kb + cmd) / (2 * ka);
//				output[i] = (temp>16384?16384:temp);
//			}
//			else
//			{
//				arm_sqrt_f32(inside,&cmd);
//				//cmd = InvSqrt(inside);
//				temp = (-kb - cmd) / (2 * ka);
//				output[i] = (temp<-16384?-16384:temp);
//			}
//		}
//		return;
//	}
//	
//	//若总功率未超过最大功率，则将原始速度环PID的值发给底盘
//	for(uint8_t i=0;i<4;i++)	output[i] = output_without_limit[i];

	for(uint8_t i = 0; i < 6; i++)
	{
		// 如果初始功率小于0（制动），或者根本没超限，则直接输出原始PID值
		// 但为了严谨，只要总功率超了，我们统一通过方程计算最安全
		if(initial_give_power[i] <= 0 || scaled_give_power[i] == initial_give_power[i])
		{
			output[i] = output_without_limit[i];
			continue;
		}

		float ka = k2;
		float kb = CT * Motor_re_speed[i];
		float kc = k1 * Motor_re_speed[i] * Motor_re_speed[i] - scaled_give_power[i] + constant;
		float inside = kb * kb - 4 * ka * kc;

		if(inside < 0)//理论上不会进入这里，这个就和二次方程无实数解一样的情况
		{
			// 如果解不出方程，就用最简单的线性缩放（前面算出的比例）来兜底
			float fallback_scale = (i < 4) ? (remaining_power / chassis_total_power) : (track_limit / track_total_power);
			output[i] = output_without_limit[i] * fallback_scale; 
			continue;
		}

		float cmd;
		arm_sqrt_f32(inside,&cmd);
        
		float temp;
		if(output_without_limit[i] > 0) 
		{
			temp = (-kb + cmd) / (2 * ka);
			output[i] = (temp > 16384 ? 16384 : temp);
		}
		else 
		{
			temp = (-kb - cmd) / (2 * ka);
			output[i] = (temp < -16384 ? -16384 : temp);
		}
	}
}


//#define rpm_to_rad (2.0f * PI / 60.0f)
//#define i_to_real (16384.0f / 20.0f)
//float power_limit_k;
//void power_limit(float pow_max,float *Motor_re_speed,float *output_without_limit,float *output)
//{
//	float velo[6] = {0};
//	float cmd[6] = {0};
//	
//    float k0 = 0.66419934f, k1 = 0.00644428f, k2 = 0.00014239f,
//          k3 = 0.01764443f, k4 = 0.16501439f, k5 = 3.09672e-05f;
//		
//    //pow_max+=-PID_Cal(&Buffer_power,robot.classis_power_buffer,20);
//		
//		for(uint8_t i = 0;i < 4;i ++)
//		{
//			velo[i] = Motor_re_speed[i] * rpm_to_rad;
//		}
//		
//		for(uint8_t i = 0;i < 4;i ++)
//		{
//			cmd[i] = output_without_limit[i] / i_to_real;
//		}

//    float sum_cmd = cmd[0] + cmd[1] + cmd[2] + cmd[3];
//    float sum_velo = velo[0] + velo[1] + velo[2] + velo[3];
//    float sum_cmd2 = cmd[0] * cmd[0] + cmd[1] * cmd[1] + cmd[2] * cmd[2] + cmd[3] * cmd[3];
//    float sum_velo2 = velo[0] * velo[0] + velo[1] * velo[1] + velo[2] * velo[2] + velo[3] * velo[3];
//    float sum_velo_cmd = velo[0] * cmd[0] + velo[1] * cmd[1] + velo[2] * cmd[2] + velo[3] * cmd[3];

//    float a = k4 * sum_cmd2;
//    float b = k1 * sum_cmd + k3 * sum_velo_cmd;
//    float c = k0 + k2 * sum_velo + k5 * sum_velo2 - pow_max;

//    float mid_temp = b * b - 4.0f * a * c;
//		if(mid_temp < 0.0f)	mid_temp = 0.0f;

//    float k = 1.0f;
//    if(fabsf(a) > 1e-6f)	k = (-b + sqrtf(mid_temp)) / (2.0f * a);

//    if(k > 1.0f)	k = 1.0f;
//    if(k < 0.0f)	k = 0.0f;
//		
//    power_limit_k = k;

//    for(int i=0;i<4;i++)
//		{
//			output[i] = power_limit_k * output[i];
//		}
//}

#define rpm_to_rad (2.0f * PI / 60.0f) 
#define i_to_real (16384.0f / 20.0f)

void power_limit(float pow_max, float *Motor_re_speed, float *output_without_limit, float *output)
{
    // 功率模型参数 (第一套代码中的系数)
    float k0 = 0.66419934f, k1 = 0.00644428f, k2 = 0.00014239f,
          k3 = 0.01764443f, k4 = 0.16501439f, k5 = 3.09672e-05f;
    
    float velo[4] = {0};
    float cmd[4] = {0};
    float k_chassis = 1.0f;

//    pow_max += (-PID_Cal(&pid_Buffer_power,Referee_System_Info.power_heat_data.buffer_energy,0));

    // 1. 单位转换
    for(uint8_t i = 0; i < 4; i++) {
        velo[i] = Motor_re_speed[i] * rpm_to_rad;
        cmd[i] = output_without_limit[i] / i_to_real;
    }

    // ==========================================
    // 3. 处理底盘电机 (索引 0, 1, 2, 3)
    // ==========================================

    float sum_cmd_ch = cmd[0] + cmd[1] + cmd[2] + cmd[3];
    float sum_velo_ch = velo[0] + velo[1] + velo[2] + velo[3];
    float sum_cmd2_ch = (cmd[0] * cmd[0]) + (cmd[1] * cmd[1]) + (cmd[2] * cmd[2]) + (cmd[3] * cmd[3]);
    float sum_velo2_ch = (velo[0] * velo[0]) + (velo[1] * velo[1]) + (velo[2] * velo[2]) + (velo[3] * velo[3]);
    float sum_velo_cmd_ch = (velo[0] * cmd[0]) + (velo[1] * cmd[1]) + (velo[2] * cmd[2]) + (velo[3] * cmd[3]);

    float a_ch = k4 * sum_cmd2_ch;
    float b_ch = k1 * sum_cmd_ch + k3 * sum_velo_cmd_ch;
    float c_ch = k0 + k2 * sum_velo_ch + k5 * sum_velo2_ch - pow_max;

    float mid_temp_ch = b_ch * b_ch - 4.0f * a_ch * c_ch;
    if(mid_temp_ch < 0) mid_temp_ch = 0;

    if(fabsf(a_ch) > 1e-6f)
         k_chassis = (-b_ch + sqrtf(mid_temp_ch)) / (2.0f * a_ch);

    if(k_chassis > 1.0f) k_chassis = 1.0f;
    if(k_chassis < 0.0f) k_chassis = 0.0f;

//		float c_ch = 4.0f * k0 + k2 * sum_velo_ch + k5 * sum_velo2_ch - pow_max;

//    // 只有当底盘预测功率超过剩余功率时，才进行限制
//    float predict_p_ch = a_ch + b_ch + (4.0f * k0 + k2 * sum_velo_ch + k5 * sum_velo2_ch);
//    if(predict_p_ch > pow_max && a_ch > 1e-6f)
//		{
//			float mid_temp_ch = b_ch * b_ch - 4.0f * a_ch * c_ch;
//      k_chassis = (-b_ch + sqrtf(fmaxf(0, mid_temp_ch))) / (2.0f * a_ch);
//    }
//    k_chassis = fminf(1.0f, fmaxf(0.0f, k_chassis));

    // ==========================================
    // 4. 赋值输出
    // ==========================================
    // 底盘部分
    for(int i = 0; i < 4; i++) {
        output[i] = output_without_limit[i] * k_chassis;
    }
}
