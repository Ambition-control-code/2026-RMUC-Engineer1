#include "Chassis_control.h"
#include "Get_remote_data.h"
#include "PID.h"
#include "Motor_control.h"
#include "Power_limit.h"
#include "can.h"
#include "imu.h"
#include "math.h"
#include "G_Compensation.h"
#include "cmsis_os.h"
#include "leg_control.h"
#include "can_bsp.h"
#include "gpio.h"
#include "filter.h"

extern uint32_t life_state;

/***@role： 将目标的运动方式通过运动学解算到每一个电机上***/
CHASSIS_Task Chassis_Task = {0};
//slope_turn_result_t slope_result;
float Real_OUT[4] = {0};
float yaw_erro = 0.0f;
void Chassis_Mecanum_control(void)
{
	float L_Front = 0.0f,L_Behind = 0.0f,R_Front = 0.0f,R_Behind = 0.0f;
	float output_without_limit[4] = {0};												    //四个电机速度环PID输出值的存储数组
	
	float Motor_re_Speed[4] = {0};															    //底盘四个电机真实转速的存储数组
//	float Real_OUT[4] = {0};																		    //功率控制后真实发给电机的值
	
	Chassis_Task.vx_set = (-C_Date.VX);													    //此处使用负号是因为执行机构C板传来的数据拨杆向前是负数
	Chassis_Task.vy_set = ( C_Date.VY);													    //此处使用负号是因为执行机构C板传来的数据拨杆向前是负数
	Chassis_Task.wz = -PID_Cal(&pid_yaw_Angle,(imu_angle.yaw_real - yaw_erro),Turn_C_Date.turn_data);//执行机构传来的机体Yaw轴的设定值
	
	Chassis_Task.A = (( Chassis_Task.vx_set) + Chassis_Task.vy_set + Chassis_Task.wz);//  2-/B-----\A-1
	Chassis_Task.B = (( Chassis_Task.vx_set) - Chassis_Task.vy_set + Chassis_Task.wz);//  
	Chassis_Task.C = ((-Chassis_Task.vx_set) + Chassis_Task.vy_set + Chassis_Task.wz);//   
	Chassis_Task.D = ((-Chassis_Task.vx_set) - Chassis_Task.vy_set + Chassis_Task.wz);//  3-\C-----/D-4
	
	if(C_Date.Stop != 0 || life_state >= 500)	//底盘掉电停止判断
	{
		yaw_erro = imu_angle.yaw_real;
		
		R_Front  = PID_Cal(&R_Front_pid ,R_Front_back.re_speed ,0.0f);
		L_Front  = PID_Cal(&L_Front_pid ,L_Front_back.re_speed ,0.0f);
		R_Behind = PID_Cal(&R_Behind_pid,R_Behind_back.re_speed,0.0f);
		L_Behind = PID_Cal(&L_Behind_pid,L_Behind_back.re_speed,0.0f);
	}
	else
	{
		R_Front  = PID_Cal(&R_Front_pid ,R_Front_back.re_speed ,Chassis_Task.A);
		L_Front  = PID_Cal(&L_Front_pid ,L_Front_back.re_speed ,Chassis_Task.B);
		R_Behind = PID_Cal(&R_Behind_pid,R_Behind_back.re_speed,Chassis_Task.C);
		L_Behind = PID_Cal(&L_Behind_pid,L_Behind_back.re_speed,Chassis_Task.D);
	}
	
	output_without_limit[0] = R_Front ;
	output_without_limit[1] = L_Front ;
	output_without_limit[2] = L_Behind;
	output_without_limit[3] = R_Behind;
	
	Motor_re_Speed[0] = R_Front_back.re_speed ;
	Motor_re_Speed[1] = L_Front_back.re_speed ;
	Motor_re_Speed[2] = L_Behind_back.re_speed;
	Motor_re_Speed[3] = R_Behind_back.re_speed;
	
//	Power_Control(Real_OUT,output_without_limit,Motor_re_Speed);			//过功率控制参数需要在这里面调
	power_limit(115.0f,Motor_re_Speed,output_without_limit,Real_OUT);
	
//	调试的轮子PID参数的时候不过功控用的四行代码，正常比赛注释就好，调参数用这个
//	Motor_control(&hcan1,0x200,output_without_limit[0],1);
//	Motor_control(&hcan1,0x200,output_without_limit[1],2);
//	Motor_control(&hcan1,0x200,output_without_limit[2],3);
//	Motor_control(&hcan1,0x200,output_without_limit[3],4);

//	比赛使用正式的参数，过了功率控制，调参数的时候把这个注释掉
	Motor_control(&hcan1,0x200,Real_OUT[0],1);
	Motor_control(&hcan1,0x200,Real_OUT[1],2);
	Motor_control(&hcan1,0x200,Real_OUT[2],3);
	Motor_control(&hcan1,0x200,Real_OUT[3],4);
	
	Motor_enable(&hcan1,0x200,8);
}



/***
 *@role:	新的腿部控制函数，之前的那个太危险了，调试的时候注意安全
 *3.17更新：新的腿部控制函数，之前的那个太危险了，调试的时候注意安全
 ***/
Chassis_mode_t Chassis_mode;
uint8_t EHP_flag = 0;
extern float normal_angle;
float L_Normol_set,R_Normol_set;

void New_Leg_Control(void)
{
	if(fabs(imu_angle.pitch) >= 75.0f || fabs(imu_angle.roll) >= 75.0f)	Chassis_mode = Normol_mode;
	
	//这里是新的腿部控制函数，之前的那个太危险了，调试的时候注意安全
	if(C_Date.Chassis_mode == 1)	
		Chassis_mode = Rise_mode;
	else if(C_Date.Chassis_mode == 2)	
		Chassis_mode = Pass_mode;
	else	
		Chassis_mode = Normol_mode;
//	else	Chassis_mode = Rise_mode;
	
	switch(Chassis_mode)
	{
		case Normol_mode:
		{
			if(EHP_flag != 1)
			{
//				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
//				osDelay(1);
//				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);				
				SpringDamperSystem.Left_all_angle = 0.0f;
				SpringDamperSystem.Right_all_angle = 0.0f;
				SpringDamperSystem.pitch_error = 0.0f;
				
				//时刻采样用于低通滤波，用于缓冲，防止目标值阶跃
				LowpassFilter_L_Normol.Last_Output = DM_L_leg.engle;
				LowpassFilter_R_Normol.Last_Output = DM_R_leg.engle;
				
				EHP_flag = 1;
			}
			
			if(C_Date.F)
			{
				//防止设定值阶跃
				L_Normol_set = Lowpass_Filter(&LowpassFilter_L_Normol,12.0f,0.01f,0.3f);
				R_Normol_set = Lowpass_Filter(&LowpassFilter_R_Normol,-13.5f,0.01f,0.3f);
			
				Suspension_With_Limits_Update(L_Normol_set,DM_L_leg.engle,DM_L_leg.vel,&MIT_L_leg);
				Suspension_With_Limits_Update(R_Normol_set,DM_R_leg.engle,DM_R_leg.vel,&MIT_R_leg);
			
				DM_MIT_Control(&hcan2,0X04,MIT_L_leg.p_des,0.0f,MIT_L_leg.kp,MIT_L_leg.kd,(MIT_L_leg.t_ff + MIT_L_leg.FDN_Tff+1.0f));
				osDelay(1);
				DM_MIT_Control(&hcan2,0x03,MIT_R_leg.p_des,0.0f,MIT_R_leg.kp,MIT_R_leg.kd,(MIT_R_leg.t_ff + MIT_R_leg.FDN_Tff-1.0f));
			}
			else if(C_Date.C)
			{
				//防止设定值阶跃
				L_Normol_set = Lowpass_Filter(&LowpassFilter_L_Normol,18.0f,0.01f,0.3f);
				R_Normol_set = Lowpass_Filter(&LowpassFilter_R_Normol,-19.5f,0.01f,0.3f);
			
				Suspension_With_Limits_Update(L_Normol_set,DM_L_leg.engle,DM_L_leg.vel,&MIT_L_leg);
				Suspension_With_Limits_Update(R_Normol_set,DM_R_leg.engle,DM_R_leg.vel,&MIT_R_leg);
			
				DM_MIT_Control(&hcan2,0X04,MIT_L_leg.p_des,0.0f,MIT_L_leg.kp,MIT_L_leg.kd,(MIT_L_leg.t_ff + MIT_L_leg.FDN_Tff+1.0f));
				osDelay(1);
				DM_MIT_Control(&hcan2,0x03,MIT_R_leg.p_des,0.0f,MIT_R_leg.kp,MIT_R_leg.kd,(MIT_R_leg.t_ff + MIT_R_leg.FDN_Tff-1.0f));
			}
			else
			{
				//防止设定值阶跃
				L_Normol_set = Lowpass_Filter(&LowpassFilter_L_Normol,normal_angle,0.01f,0.3f);
				R_Normol_set = Lowpass_Filter(&LowpassFilter_R_Normol,(-normal_angle-1.5f),0.01f,0.3f);
				
				Suspension_With_Limits_Update(L_Normol_set,DM_L_leg.engle,DM_L_leg.vel,&MIT_L_leg);
				Suspension_With_Limits_Update(R_Normol_set,DM_R_leg.engle,DM_R_leg.vel,&MIT_R_leg);
			
				DM_MIT_Control(&hcan2,0X04,MIT_L_leg.p_des,0.0f,MIT_L_leg.kp,MIT_L_leg.kd,MIT_L_leg.t_ff + MIT_L_leg.FDN_Tff);
				osDelay(1);
				DM_MIT_Control(&hcan2,0x03,MIT_R_leg.p_des,0.0f,MIT_R_leg.kp,MIT_R_leg.kd,MIT_R_leg.t_ff + MIT_R_leg.FDN_Tff);
			}
		}	break;
		case Rise_mode:
		{
			if(fabs(DM_L_leg.engle) > 20.0f && fabs(DM_R_leg.engle) > 22.0f && EHP_flag != 0)
			{
//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_13,GPIO_PIN_SET);
//				osDelay(1);
//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);				
				MIT_L_leg.t_ff = 0.0f;
				MIT_R_leg.t_ff = 0.0f;
				EHP_flag = 0;
				normal_angle = 3.0f;
			}

//			if(C_Date.shoutui == 1)
//			{
//				SpringDamperSystem.Right_all_angle = 0.0f;
//				SpringDamperSystem.Left_all_angle = 0.0f;
//				SpringDamperSystem.pitch_error = 0.0f;
//				
//				//防止设定值阶跃
//				float L_Normol_set = Lowpass_Filter(&LowpassFilter_L_Normol,3.5f,0.02f,1.0f);
//				float R_Normol_set = Lowpass_Filter(&LowpassFilter_R_Normol,(-4.5f),0.02f,1.0f);
//				
//				//限幅
//				if(L_Normol_set < 1.5f)	L_Normol_set = 1.5f;
//				if(L_Normol_set > 65.0f)	L_Normol_set = 65.0f;
//				if(R_Normol_set > (-3.5f))	R_Normol_set = (-3.5f);
//				if(R_Normol_set < (-67.0f))	R_Normol_set = (-67.0f);
//				
//				//控制电机
//				DM_MIT_Control(&hcan2,0X04,L_Normol_set,0.0f,MIT_L_leg.kp,MIT_L_leg.kd,2.0f);
//				osDelay(1);
//				DM_MIT_Control(&hcan2,0x03,R_Normol_set,0.0f,MIT_R_leg.kp,MIT_R_leg.kd,(-2.0f));
//			}
//			else
//			{
				//积分计算，进行自稳
				Target_angle_integral(&SpringDamperSystem);
				
				//使能电机
				DM_MIT_Control(&hcan2,0X04,SpringDamperSystem.Left_all_angle,0.0f,
																	SpringDamperSystem.Left_kp,SpringDamperSystem.Left_kd,SpringDamperSystem.Left_torque);
				osDelay(1);
				DM_MIT_Control(&hcan2,0x03,SpringDamperSystem.Right_all_angle,0.0f,
																	SpringDamperSystem.Right_kp,SpringDamperSystem.Right_kd,SpringDamperSystem.Right_torque);
				
				//时刻采样用于低通滤波，用于缓冲，防止目标值阶跃
				LowpassFilter_L_Normol.Last_Output = DM_L_leg.engle;
				LowpassFilter_R_Normol.Last_Output = DM_R_leg.engle;
//			}
		}	break;
		case Pass_mode:
		{
			if(EHP_flag != 2)
			{
//				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
//				osDelay(1);
//				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);				
				SpringDamperSystem.Left_all_angle = 0.0f;
				SpringDamperSystem.Right_all_angle = 0.0f;
				SpringDamperSystem.pitch_error = 0.0f;
				
				//时刻采样用于低通滤波，用于缓冲，防止目标值阶跃
				LowpassFilter_L_Normol.Last_Output = DM_L_leg.engle;
				LowpassFilter_R_Normol.Last_Output = DM_R_leg.engle;
				
				EHP_flag = 2;
			}
			
			if(C_Date.F)
			{
				//防止设定值阶跃
				L_Normol_set = Lowpass_Filter(&LowpassFilter_L_Normol,8.0f,0.01f,0.3f);
				R_Normol_set = Lowpass_Filter(&LowpassFilter_R_Normol,-9.5f,0.01f,0.3f);
			
				Suspension_With_Limits_Update(L_Normol_set,DM_L_leg.engle,DM_L_leg.vel,&MIT_L_leg);
				Suspension_With_Limits_Update(R_Normol_set,DM_R_leg.engle,DM_R_leg.vel,&MIT_R_leg);
			
				DM_MIT_Control(&hcan2,0X04,MIT_L_leg.p_des,0.0f,MIT_L_leg.kp,MIT_L_leg.kd,(MIT_L_leg.t_ff + MIT_L_leg.FDN_Tff+1.0f));
				osDelay(1);
				DM_MIT_Control(&hcan2,0x03,MIT_R_leg.p_des,0.0f,MIT_R_leg.kp,MIT_R_leg.kd,(MIT_R_leg.t_ff + MIT_R_leg.FDN_Tff-1.0f));
			}
			else if(C_Date.C)
			{
				//防止设定值阶跃
				L_Normol_set = Lowpass_Filter(&LowpassFilter_L_Normol,15.0f,0.01f,0.3f);
				R_Normol_set = Lowpass_Filter(&LowpassFilter_R_Normol,-16.5f,0.01f,0.3f);
			
				Suspension_With_Limits_Update(L_Normol_set,DM_L_leg.engle,DM_L_leg.vel,&MIT_L_leg);
				Suspension_With_Limits_Update(R_Normol_set,DM_R_leg.engle,DM_R_leg.vel,&MIT_R_leg);
			
				DM_MIT_Control(&hcan2,0X04,MIT_L_leg.p_des,0.0f,MIT_L_leg.kp,MIT_L_leg.kd,(MIT_L_leg.t_ff + MIT_L_leg.FDN_Tff+1.0f));
				osDelay(1);
				DM_MIT_Control(&hcan2,0x03,MIT_R_leg.p_des,0.0f,MIT_R_leg.kp,MIT_R_leg.kd,(MIT_R_leg.t_ff + MIT_R_leg.FDN_Tff-1.0f));
			}
			else
			{
				//防止设定值阶跃
				L_Normol_set = Lowpass_Filter(&LowpassFilter_L_Normol,normal_angle,0.01f,0.3f);
				R_Normol_set = Lowpass_Filter(&LowpassFilter_R_Normol,(-normal_angle-1.5f),0.01f,0.3f);
			
				Suspension_With_Limits_Update(L_Normol_set,DM_L_leg.engle,DM_L_leg.vel,&MIT_L_leg);
				Suspension_With_Limits_Update(R_Normol_set,DM_R_leg.engle,DM_R_leg.vel,&MIT_R_leg);
			
				DM_MIT_Control(&hcan2,0X04,MIT_L_leg.p_des,0.0f,MIT_L_leg.kp,MIT_L_leg.kd,(MIT_L_leg.t_ff + MIT_L_leg.FDN_Tff+1.0f));
				osDelay(1);
				DM_MIT_Control(&hcan2,0x03,MIT_R_leg.p_des,0.0f,MIT_R_leg.kp,MIT_R_leg.kd,(MIT_R_leg.t_ff + MIT_R_leg.FDN_Tff-1.0f));
			}
		}	break;
	}
}
