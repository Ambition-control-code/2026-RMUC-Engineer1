#include "get_arm_data.h"
#include "Get_Referee_Data.h"
#include "string.h"
#include "PID.h"
#include "Motor_control.h"
#include "DH_g.h"

/***@role:	获取机器人机械臂角度各种数据的函数***/
Arm_union_data_t Arm_union_data = {0};
void Back_data_control(void)
{
	float Big_Compensate_I = 0.0f,Smoll_Compensate_I = 0.0f;
	
	memcpy(Arm_union_data.RX_Arm_data,Referee_System_Info.Arm_data.data,30);
	
	//gravity_compensation(Big_Pitch_back.ecd_angle,Smoll_Pitch_back.ecd_angle,&Big_Compensate_I,&Smoll_Compensate_I);
	
	if(Arm_union_data.Arm_data_t.Big_pitch_flag == 1)				//大Pitch轴关节超限控制
	{
		float BigPitch_angle = 0.0f,BigPitch_speed = 0.0f;
		
		BigPitch_angle = PID_Cal(&BigPitch_angle_pid,Big_Pitch_back.ecd_angle,Arm_union_data.Arm_data_t.M2_Big_Pitch);
		BigPitch_speed = PID_Cal(&BigPitch_speed_pid,Big_Pitch_back.re_speed,BigPitch_angle);
		
		Motor_control(&hcan1,0x1FF,BigPitch_speed,6);
	}
	else	Motor_control(&hcan1,0x1FF,Big_Compensate_I,6);		//使用重力补偿的电流值
	
	if(Arm_union_data.Arm_data_t.Smoll_pitch_flag == 1)			//小Pitch轴关节超限控制
	{
		float SmollPitch_angle = 0.0f,SmollPitch_speed = 0.0f;
		
		SmollPitch_angle = PID_Cal(&BigPitch_angle_pid,Smoll_Pitch_back.ecd_angle,Arm_union_data.Arm_data_t.M3_Smoll_Pitch);
		SmollPitch_speed = PID_Cal(&BigPitch_speed_pid,Smoll_Pitch_back.re_speed,SmollPitch_angle);
		
		Motor_control(&hcan1,0x1FF,SmollPitch_speed,5);
	}
	else	Motor_control(&hcan1,0x1FF,Smoll_Compensate_I,5);	//使用重力补偿的电流值
	
	if(Arm_union_data.Arm_data_t.Big_roll_flag == 1)				//Roll轴关节超限控制
	{
		float BigRoll_angle = 0.0f,BigRoll_speed = 0.0f;
		
		BigRoll_angle = PID_Cal(&BigPitch_angle_pid,Big_Roll_back.ecd_angle,Arm_union_data.Arm_data_t.M4_Big_Roll);
		BigRoll_speed = PID_Cal(&BigPitch_speed_pid,Big_Roll_back.re_speed,BigRoll_angle);
		
		Motor_control(&hcan2,0x200,BigRoll_speed,1);
	}
	else	Motor_control(&hcan2,0x200,0.0f,1);								//没有重力补偿，释放关节
	
	if(Arm_union_data.Arm_data_t.end_pitch_flag == 1)				//末端Pitch轴关节超限控制
	{
		float endPitch_angle = 0.0f,endPitch_speed = 0.0f;
		
		endPitch_angle = PID_Cal(&BigPitch_angle_pid,End_Pitch_back.ecd_angle,Arm_union_data.Arm_data_t.M5_End_Pitch);
		endPitch_speed = PID_Cal(&BigPitch_speed_pid,End_Pitch_back.re_speed,endPitch_angle);
		
		Motor_control(&hcan2,0x200,endPitch_speed,2);
	}
	else	Motor_control(&hcan2,0x200,0.0f,2);									//没有重力补偿，释放关节
	
	if(Arm_union_data.Arm_data_t.end_roll_flag == 1)					//末端Roll轴关节超限控制
	{
		float endRoll_angle = 0.0f,endRoll_speed = 0.0f;
		
		endRoll_angle = PID_Cal(&BigPitch_angle_pid,End_Roll_back.ecd_angle,Arm_union_data.Arm_data_t.M6_End_Roll);
		endRoll_speed = PID_Cal(&BigPitch_speed_pid,End_Roll_back.re_speed,endRoll_angle);
		
		Motor_control(&hcan2,0x200,endRoll_speed,3);
	}
	else	Motor_control(&hcan2,0x200,0.0f,3);									//没有重力补偿，释放关节
	
	Motor_enable(&hcan1,0x1FF,6);
	Motor_enable(&hcan2,0x200,6);
}


