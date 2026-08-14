//这是工程执行机构的代码文件
#include "math.h"
#include "actuator.h"
#include "PID.h"
#include "IMU.h"
#include "remote.h"
#include "arm_math.h"
#include "arm_control.h"

ARM_Theta_t ARM_Theta = {0};

/***@role: 两个Pitch关节的解算，会求出根部Pitch和二级Pitch轴到达目标位置所需要的角度***/
//void Think_Target_Pos(ARM_Theta_t* arm_theta_t,double End_X,double End_Y,double set_yaw,double set_roll)
//{
//	double d = (1/AHRS_invSqrt((End_X * End_X) + (End_Y * End_Y)));	     // 计算到目标点的距离
//	double min_reach = (fabs(Big_Arm_Long - Smoll_Arm_Long));
//	double max_reach = (Big_Arm_Long + Smoll_Arm_Long);
//	
//	arm_theta_t -> Last_theta1 = arm_theta_t -> theta1;
//	arm_theta_t -> Last_theta2 = arm_theta_t -> theta2;
//	
//	if(d <= min_reach && d >= max_reach)
//	{
//		arm_theta_t -> theta1 = arm_theta_t -> Last_theta1;
//		arm_theta_t -> theta2 = arm_theta_t -> Last_theta2;
//		return;																												    // 检查目标点是否可达
//	}
//	
//	//使用余弦定理计算小臂与大臂的夹角
//	double cos_theta2 = (((End_X * End_X) + (End_Y * End_Y) -(Big_Arm_Long * Big_Arm_Long)
//											 -(Smoll_Arm_Long * Smoll_Arm_Long))/(Big_Arm_Long * Smoll_Arm_Long * (-2)));
//	
//	// 处理数值误差，确保cos_theta2在有效范围内
//  if (cos_theta2 >  1.0) cos_theta2 =  1.0;
//  if (cos_theta2 < -1.0) cos_theta2 = -1.0;
//	
//	// 小臂与大臂的夹角（肘部向上配置）
//  double theta2 = AHRS_acosf(cos_theta2);													    // 小臂与大臂的夹角
//	
//	// 步骤4: 计算大臂与水平面的夹角
//  double alpha = AHRS_atan2f(End_Y, End_X);   										    // 基座到目标点的角度
//  double beta  = ((Smoll_Arm_Long * Smoll_Arm_Long)-(Big_Arm_Long*Big_Arm_Long)-(End_X * End_X)-(End_Y * End_Y))
//								 /((-2) * Big_Arm_Long * (1/AHRS_invSqrt((End_X * End_X) + (End_Y * End_Y))));
//	beta = AHRS_acosf(beta);
//	double theta1 = alpha + beta;																		    // 大臂与水平面的夹角
//	
//	// 存储结果
//	arm_theta_t -> theta1 = ((theta1 * 180.0f)/PI);											// 大臂转动的目标值
//	arm_theta_t -> theta2 = ((theta2 * 180.0f)/PI);									  	// 小臂转动的目标值
//	arm_theta_t -> Set_YAW  = set_yaw;
//	arm_theta_t -> Set_ROLL = set_roll;
//}
//>>>>>>>>>>>>>>>>>>>>>>上面是没有用上的解算，有点小问题，而且只解了两个关节，提升不大，暂且搁置<<<<<<<<<<<<<<<<<<

//==========================================================================================================
/***@role: 大小臂解算执行任务***/
float Smoll_Pitch = 0.0f,Big_Pitch = 0.0f;		//末端的初始坐标值，可以修改，具体根据实车情况看
float Set_Arm_yaw = 0.0f,Set_Arm_roll = 0.0f;
float Set_end_Pit = 0.0f,Set_end_roll = 0.0f;
void Arm_Task(void)
{
//===================================================================================================//
	float remote_Ly,remote_Ry,remote_LX,remote_RX;
	
	// 处理摇杆输入(乘以系数并取反)					
	remote_RX = (remote.R_x + New_VT03_data.R_rocker_x) * remote_K;       //右摇杆X轴
	remote_Ry = (remote.R_y + New_VT03_data.R_rocker_y) * remote_K;       //右摇杆Y轴
	remote_LX = (remote.L_x + New_VT03_data.L_rocker_x) * remote_K;       //左摇杆X轴
	remote_Ly = (remote.L_y + New_VT03_data.L_rocker_y) * remote_K;       //左摇杆Y轴

	switch(Chassis_or_Arm_or_End)
	{
		case 2:
		{
			/*下面四行代码需要根据实际情况进行修改！！！！！切记！切记！*/
			// 合成运动矢量
			Big_Pitch    -= (remote_Ly / 50000.0f);                       						//机械臂解算二维平面的Y轴
			Smoll_Pitch  -= (remote_Ry / 50000.0f);                       						//机械臂解算二维平面的X轴
			Set_Arm_yaw  += (remote_LX / 50000.0f);																		//大臂YAW轴左右转向
			Set_Arm_roll += (remote_RX / 50000.0f);																		//小臂的Roll轴左右转向
			//------这两行其实是有问题的需要后面进行编写代码的时候修改，目前只使用了两个变量
			//*上面四行代码需要根据实际情况进行修改！！！！！切记！切记！*//
		
			if(Smoll_Pitch  <  (Big_Pitch_back.engle-50.0f))	Smoll_Pitch = (Big_Pitch_back.engle-50.0f);
			if(Smoll_Pitch  >  25.0f)	Smoll_Pitch =  25.0f;
			if(Set_Arm_yaw  >  59.5f )	Set_Arm_yaw =  59.5f;
			if(Set_Arm_yaw  < -45.76f )	Set_Arm_yaw = -45.76f;
			if(Set_Arm_roll >  230.0f)	Set_Arm_roll =  230.0f;
			if(Set_Arm_roll < -230.0f)	Set_Arm_roll = -230.0f;
			if(Big_Pitch    >  0.0f  )	Big_Pitch =  0.0f;
			if(Big_Pitch    < -68.0f)	Big_Pitch = -68.0f;
		}	break;
		case 1:
		{
			Set_end_Pit  += (remote_Ry / 50000.0f);
			Set_end_roll += ((remote_RX / 50000.0f) * 37.5f);
			Set_Arm_yaw  += (remote_LX / 50000.0f);																		//大臂YAW轴左右转向
			Set_Arm_roll += (remote_Ly / 50000.0f);																		//小臂的Roll轴左右转向
			
//			ARM_Theta.end_pitch = Set_end_Pit ;																								//末端Pitch轴设定
//			ARM_Theta.end_roll  = Set_end_roll;																								//末端roll轴设定
		
			if(Set_end_Pit  >  100.0f)	Set_end_Pit  =  100.0f;
			if(Set_end_Pit  < -100.0f)	Set_end_Pit  = -100.0f;
			if(Set_Arm_yaw  >  59.5f )	Set_Arm_yaw  =  59.5f;
			if(Set_Arm_yaw  < -45.76f )	Set_Arm_yaw  = -45.76f;
			if(Set_Arm_roll >  230.0f)	Set_Arm_roll =  230.0f;
			if(Set_Arm_roll < -230.0f)	Set_Arm_roll = -230.0f;
		}	break;
		default:	break;
	}
}

//过载保护
void OverLoad_detection(void)
{
	if(fabs(Big_Pitch_back.tor)>6.0f&&fabs(Big_Pitch_back.vel)<1.0f)	
		Big_Pitch = Big_Pitch_back.engle;
	if(fabs(Smoll_Pitch_back.tor)>9.5f&&fabs(Smoll_Pitch_back.vel)<1.0f)	
		Smoll_Pitch = Smoll_Pitch_back.engle;
	if(fabs(DM_YAW_back.tor)>6.0f&&fabs(DM_YAW_back.vel)<1.0f)	
		Set_Arm_yaw = DM_YAW_back.engle;
	if(fabs(DM_ROLL_back.tor)>11.5f&&fabs(DM_ROLL_back.vel)<1.0f)	
		Set_Arm_roll = DM_ROLL_back.engle;
	if(fabs(DM_endRoll_back.tor)>13.0f&&fabs(DM_endRoll_back.vel)<1.0f)	
		Set_end_roll = DM_endRoll_back.engle;
	if(fabs(DM_endPitch_back.tor)>3.0f&&fabs(DM_endPitch_back.vel)<1.0f)	
		Set_end_Pit = DM_endPitch_back.engle;
}
//==============================================================================


/***
 *@role: 机械臂断电后再上电的初始化函数
 *@param:	初始化的时候先给电机传入电机使能时读到的数据的原因是为了让电机缓启动,
 *				启动的一瞬间电流过大,电机加速度很快尤是10010。
 *@param:	先传入初始值再发零点初始化状态，对机械结构友善，也更安全。
***/
uint8_t DM_Init_count = 0;
extern float End_Roll_angle_Init;
float Claw_mid_angle = 0.0f;
extern uint8_t change_claw_state;

void ARM_Init(void)
{
	if(DM_Init_count < 9)//注意跟随DM_Init_count的要求值修改比值数
	{
		switch(DM_Init_count)
		{
			case 0:	//缓启动
			{
				DM_Pos_Speed_Control(&hcan2,0x04,DM_YAW_back.engle,0.5f);
				osDelay(10);

				DM_Pos_Speed_Control(&hcan2,0x01,Big_Pitch_back.engle,0.5f);
				osDelay(10);

				DM_Pos_Speed_Control(&hcan2,0x02,Smoll_Pitch_back.engle,0.5f);
				osDelay(10);

				DM_Pos_Speed_Control(&hcan2,0x03,DM_ROLL_back.engle,0.5f);
				osDelay(10);
				
				DM_Pos_Speed_Control(&hcan2,0x05,DM_endPitch_back.engle,0.5f);
				osDelay(10);
				
				DM_Pos_Speed_Control(&hcan2,0x06,DM_endRoll_back.engle,0.5f);
				osDelay(10);
				
				DM_Pos_Speed_Control(&hcan2,0x07,DM_Claw_back.engle,0.5f);
				osDelay(10);

				DM_Init_count ++;
			}	break;
			case 1:
			{
				if(fabs(Big_Pitch_back.engle) <= 1.0f)	DM_Init_count ++;
				else	DM_Pos_Speed_Control(&hcan2,0x01,0.0f,1.0f);
			}	break;
			case 2:
			{
				if(fabs(DM_YAW_back.engle) <= 1.0f)	    DM_Init_count ++;
				else	DM_Pos_Speed_Control(&hcan2,0x04,0.0f,2.0f);
			}	break;
			case 3:
			{
				if(fabs(Smoll_Pitch_back.engle) <= 1.0f)	DM_Init_count ++;
				else	DM_Pos_Speed_Control(&hcan2,0x02,0.0f,0.5f);
			}	break;
			case 4:
			{
				if(fabs(DM_ROLL_back.engle) <= 1.0f)	DM_Init_count ++;
				else	DM_Pos_Speed_Control(&hcan2,0x03,0.0f,5.0f);
			}	break;
			case 5:
			{
				if(fabs(DM_endPitch_back.raw_pos) <= 1.0f)	DM_Init_count ++;
				else	DM_Pos_Speed_Control(&hcan2,0x05,0.0f,5.0f);
			}	break;
			case 6:
			{
				memcpy(custom_data.RX_custom_data,Referee_System_Info.custom_robot_data.data,30);
				End_Roll_angle_Init = custom_data.Custom_Data.endRoll_data;
				DM_Init_count ++;
			}	break;
			case 7:
			{
				if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14))
				{
					DM_Pos_Speed_Control(&hcan2,0x07,DM_Claw_back.engle,0.0f);
					Claw_mid_angle = DM_Claw_back.engle+2000.0f;
					DM_Init_count ++;
					change_claw_state = 0;
				}
				else if(fabs(DM_Claw_back.tor) > 2.27f)
				{
					DM_Pos_Speed_Control(&hcan2,0x07,DM_Claw_back.engle,0.0f);
					Claw_mid_angle = DM_Claw_back.engle + 4000.0f;
					DM_Init_count ++;
					change_claw_state = 0;
				}
				else
				{
					Claw_mid_angle = DM_Claw_back.engle - 300.0f;
					DM_Pos_Speed_Control(&hcan2,0x07,Claw_mid_angle,5.0f);
				}
			}	break;
		}
		osDelay(10);
	}
}
//=================================================================================================

// 一键存取矿变量
uint8_t TFR_count0 = 0;
uint8_t count1 = 0;
uint8_t L_access_flag = 0,R_access_flag = 0;
extern uint8_t Left_flag,Right_flag;
extern float Claw_Speed;
float target_Big_Pitch = 0.0f,target_Smoll_Pitch = 0.0f,target_End_Pitch = 0.0f,target_yaw = 0.0f;

void TFR_conrrol_L(void)
{
	switch(TFR_count0)
  {
		//===================================初始化归位===========================================
		case 0:	//大Roll归位
				DM_Pos_Speed_Control(&hcan2,0x03,0.0f,10.0f);
				TFR_count0 = 1;
		break;	
		case 1:	//yaw轴归位
				if(fabs(DM_YAW_back.engle - 0.0f)<1.0f&&fabs(DM_ROLL_back.engle)<1.0f)	TFR_count0 = 2;
				else	
				{
					DM_Pos_Speed_Control(&hcan2,0x04,0.0f,1.0f);
					TFR_count0 = 0;
				}
		break;
		case 2:
					if(L_access_flag)	//判断是否存有矿物
					{
						target_Big_Pitch = -0.29f;
						target_Smoll_Pitch = 19.62f;
						target_End_Pitch = 29.34f;
						
						if((fabs(DM_endPitch_back.engle-29.34f)<1.0f)&&(fabs(Smoll_Pitch_back.engle-19.62f)<1.0f)&&
							 (fabs(Big_Pitch_back.engle+0.29f)<1.0f))
							target_yaw = -45.76f;
						if((fabs(DM_endPitch_back.engle-29.34f)<1.0f)&&(fabs(Smoll_Pitch_back.engle-19.62f)<1.0f)&&
							 (fabs(Big_Pitch_back.engle+0.29f)<1.0f)&&(fabs(DM_YAW_back.engle+45.76f)<1.0f))	
							TFR_count0 = 7;
						else	TFR_count0 = 3;
					}
					else
					{
						target_Big_Pitch = -8.86f;
						target_Smoll_Pitch = -6.2f;
						target_End_Pitch = 15.62f;
						target_yaw = -45.76f;
						
						if((fabs(DM_endPitch_back.engle-15.62f)<1.0f)&&(fabs(Smoll_Pitch_back.engle+6.2f)<1.0f)&&
							 (fabs(Big_Pitch_back.engle+8.86f)<1.0f)&&(fabs(DM_YAW_back.engle+45.76f)<1.0f))	
							TFR_count0 = 9;
						else	TFR_count0 = 3;
					}
		break;	
		case 3: 
				DM_Pos_Speed_Control(&hcan2,0x05,target_End_Pitch,20.0f);
				TFR_count0 = 4;
		break;	
		case 4: 
				DM_Pos_Speed_Control(&hcan2,0x02,target_Smoll_Pitch,0.7f);
				TFR_count0 = 5;
		break;	
		case 5: 
				DM_Pos_Speed_Control(&hcan2,0x01,target_Big_Pitch,1.0f);
				TFR_count0 = 6;
		break;
		case 6:
				DM_Pos_Speed_Control(&hcan2,0x04,target_yaw,1.0f);
				TFR_count0 = 2;
		break;
		//=================================初始化归位=================================================
		
		//================================取下来的过程，先前后上======================================
		case 7:
				//向前平移
				//开始向前平移
				Claw_Speed = Claw_mid_angle + 55000.0f;
				DM_Pos_Speed_Control(&hcan2,0x07,Claw_Speed,30.0f);
				change_claw_state = 1;
			
				if(fabs(DM_Claw_back.engle - Claw_Speed) < 5.0f)
				{
					target_Big_Pitch = -17.93f;
					target_Smoll_Pitch = 7.75;
					target_End_Pitch = 26.39f;
					
					TFR_count0 = 11;
					count1 = 0;
				}
		break;
		case 8:
				Claw_Speed = Claw_mid_angle;
				DM_Pos_Speed_Control(&hcan2,0x07,Claw_Speed,30.0f);
				change_claw_state = 0;
		
				if(fabs(DM_Claw_back.engle - Claw_mid_angle) < 5.0f)
				{
					//向上平移
//					robot_deg.d1 = Big_Pitch_back.engle;
//					robot_deg.d2 = Smoll_Pitch_back.engle;
//					robot_deg.d3 = DM_endPitch_back.engle;
//					// 1. 计算逆解 (只有在处理第一个电机前计算，保证三轴同步)
//					moveUpward_Degree(&robot_deg, 0.12f);
					
					target_Big_Pitch = -11.25f;
					target_Smoll_Pitch = -6.11f;
					target_End_Pitch = 23.62f;
					
					TFR_count0 = 11;
					count1 = 1;
				}
		break;
		//================================取下来的过程，先前后上======================================

		//============================放进去的过程，先下后后==========================================
		case 9:
				//向下平移
//				robot_deg.d1 = Big_Pitch_back.engle;
//				robot_deg.d2 = Smoll_Pitch_back.engle;
//				robot_deg.d3 = DM_endPitch_back.engle;
//				// 1. 计算逆解
//				moveUpward_Degree(&robot_deg, -0.12f);
				
				target_Big_Pitch = -17.93f;
				target_Smoll_Pitch = 7.75;
				target_End_Pitch = 26.39f;
				
				TFR_count0 = 11;
				count1 = 0;
		break;
		case 10:
				Claw_Speed = Claw_mid_angle + 55000.0f;
				DM_Pos_Speed_Control(&hcan2,0x07,Claw_Speed,30.0f);
				change_claw_state = 1;
		
				if(fabs(DM_Claw_back.engle - Claw_Speed) < 5.0f)
				{
					//向后平移
					//开始向后平移
					target_Big_Pitch = -0.29f;
					target_Smoll_Pitch = 19.62f;
					target_End_Pitch = 29.34f;
					
					TFR_count0 = 11;
					count1 = 1;
				}
		break;
		//================================放进去的过程，先下后后======================================

		//================================控制电机就位===============================================
		case 11:
				// 2. 发送第一个电机	
				DM_Pos_Speed_Control(&hcan2, 0x01, target_Big_Pitch, 1.0f);
				TFR_count0 = 12;
		break;
		case 12:
				// 发送第二个电机
				DM_Pos_Speed_Control(&hcan2, 0x02, target_Smoll_Pitch, 0.7f);
				TFR_count0 = 13;
		break;
		case 13:
				// 发送第三个电机
				DM_Pos_Speed_Control(&hcan2, 0x05, target_End_Pitch, 20.0f);

				// 判断循环是否结束
				if(count1)
				{
					if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 1.0f && 
							fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 1.0f && 
								fabs(DM_endPitch_back.engle - target_End_Pitch) < 1.0f)
									TFR_count0 = 30; // 如果已经到位，直接跳转到结束的步骤
					else	TFR_count0 = 11;
				}
				else
				{
					if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 1.0f && 
							fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 1.0f && 
								fabs(DM_endPitch_back.engle - target_End_Pitch) < 1.0f &&
															L_access_flag==0)
					{
						L_access_flag = 1;
						TFR_count0 = 10; // 如果已经到位，直接跳转到下一个移动的步骤
					}
											
					else if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 1.0f && 
								fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 1.0f && 
									fabs(DM_endPitch_back.engle - target_End_Pitch) < 1.0f &&
																L_access_flag)
					{
						L_access_flag = 0;
						TFR_count0 = 8; // 如果已经到位，直接跳转到下一个移动的步骤
					}
												
					else	TFR_count0 = 11;
				}
		break;
		//================================控制电机就位===============================================

		//============================最终校准与退出===============================================
		case 30:
				//最终校准与退出
				if(fabs(DM_YAW_back.engle) < 1.0f)
				{
					count1 = 0;
					TFR_count0 = 0;		// 重置状态机
					Left_flag = 0;
					Right_flag = 0;
					target_Big_Pitch = 0.0f;
					target_Smoll_Pitch = 0.0f;
					target_End_Pitch = 0.0f;
					target_yaw = 0.0f;
				}
				else	DM_Pos_Speed_Control(&hcan2,0x04,0.0f,1.5f);
		break;
		//============================最终校准与退出===============================================

		default:	break;
	}
}

void TFR_conrrol_R(void)
{
	switch(TFR_count0)
  {
		//===================================初始化归位===========================================
		case 0:	//大Roll归位 
				DM_Pos_Speed_Control(&hcan2,0x03,0.0f,20.0f);
				TFR_count0 = 1;
		break;	
		case 1:	//yaw轴归位
				if(fabs(DM_YAW_back.engle - 0.0f)<1.0f&&fabs(DM_ROLL_back.engle)<1.0f)	TFR_count0 = 2;
				else	
				{
					DM_Pos_Speed_Control(&hcan2,0x04,0.0f,1.0f);
					TFR_count0 = 0;
				}
		break;
		case 2:
					if(R_access_flag)	//判断是否存有矿物
					{
						target_Big_Pitch = -2.79f;
						target_Smoll_Pitch = 17.07f;
						target_End_Pitch = 15.88f;
						
						if((fabs(DM_endPitch_back.engle-15.88f)<1.0f)&&(fabs(Smoll_Pitch_back.engle-17.07f)<1.0f)&&
							 (fabs(Big_Pitch_back.engle+2.79f)<1.0f))
							target_yaw = 57.89f;
						if((fabs(DM_endPitch_back.engle-15.88f)<1.0f)&&(fabs(Smoll_Pitch_back.engle-17.07f)<1.0f)&&
							 (fabs(Big_Pitch_back.engle+2.79f)<1.0f)&&(fabs(DM_YAW_back.engle-57.89f)<1.0f))	
							TFR_count0 = 7;
						else	TFR_count0 = 3;
					}
					else
					{
						target_Big_Pitch = -12.81f;
						target_Smoll_Pitch = -13.41f;
						target_End_Pitch = 33.6f;
						target_yaw = 57.89f;
						
						if((fabs(DM_endPitch_back.engle-33.6f)<1.0f)&&(fabs(Smoll_Pitch_back.engle+13.41f)<1.0f)&&
							 (fabs(Big_Pitch_back.engle+12.81f)<1.0f)&&(fabs(DM_YAW_back.engle-57.89f)<1.0f))	
							TFR_count0 = 9;
						else	TFR_count0 = 3;
					}
		break;	
		case 3: 
				DM_Pos_Speed_Control(&hcan2,0x05,target_End_Pitch,20.0f);
				TFR_count0 = 4;
		break;	
		case 4: 
				DM_Pos_Speed_Control(&hcan2,0x02,target_Smoll_Pitch,0.7f);
				TFR_count0 = 5;
		break;	
		case 5: 
				DM_Pos_Speed_Control(&hcan2,0x01,target_Big_Pitch,1.0f);
				TFR_count0 = 6;
		break;
		case 6:
				DM_Pos_Speed_Control(&hcan2,0x04,target_yaw,1.0f);
				TFR_count0 = 2;
		break;
		//=================================初始化归位=================================================
		
		//================================取下来的过程，先前后上======================================
		case 7:
				//向前平移
				//开始向后平移
				Claw_Speed = Claw_mid_angle + 55000.0f;
				change_claw_state = 1;
				DM_Pos_Speed_Control(&hcan2,0x07,Claw_Speed,30.0f);
		
				if(fabs(DM_Claw_back.engle - Claw_Speed) < 5.0f)
				{
					target_Big_Pitch = -20.99f;
					target_Smoll_Pitch = 2.55f;
					target_End_Pitch = 38.61f;
					
					TFR_count0 = 11;
					count1 = 0;
				}
		break;
		case 8:
				Claw_Speed = Claw_mid_angle;
				DM_Pos_Speed_Control(&hcan2,0x07,Claw_Speed,30.0f);
				change_claw_state = 0;
		
				if(fabs(DM_Claw_back.engle - Claw_Speed) < 5.0f)
				{
					Claw_Speed = Claw_mid_angle + 55000.0f;
					//向上平移
//					robot_deg.d1 = Big_Pitch_back.engle;
//					robot_deg.d2 = Smoll_Pitch_back.engle;
//					robot_deg.d3 = DM_endPitch_back.engle;
//					// 1. 计算逆解 (只有在处理第一个电机前计算，保证三轴同步)
//					moveUpward_Degree(&robot_deg, 0.12f);
					
					target_Big_Pitch = -10.81f;
					target_Smoll_Pitch = -13.41f;
					target_End_Pitch = 33.6f;
					
					TFR_count0 = 11;
					count1 = 1;
				}
		break;
		//================================取下来的过程，先前后上======================================

		//============================放进去的过程，先下后后==========================================
		case 9:
				//向下平移
//				robot_deg.d1 = Big_Pitch_back.engle;
//				robot_deg.d2 = Smoll_Pitch_back.engle;
//				robot_deg.d3 = DM_endPitch_back.engle;
//				// 1. 计算逆解
//				moveUpward_Degree(&robot_deg, -0.12f);
				
				target_Big_Pitch = -17.99f;
				target_Smoll_Pitch = 2.55f;
				target_End_Pitch = 38.61f;
				
				TFR_count0 = 11;
				count1 = 0;
		break;
		case 10:
				Claw_Speed = Claw_mid_angle + 55000.0f;
				DM_Pos_Speed_Control(&hcan2,0x07,Claw_Speed,30.0f);
				change_claw_state = 1;
		
				if(fabs(DM_Claw_back.engle - Claw_Speed) < 5.0f)
				{
					//向后平移
					//开始向后平移
					target_Big_Pitch = -2.79f;
					target_Smoll_Pitch = 17.07f;
					target_End_Pitch = 15.88f;
					
					TFR_count0 = 11;
					count1 = 1;
				}
		break;
		//================================放进去的过程，先下后后======================================

		//================================控制电机就位===============================================
		case 11:
				// 2. 发送第一个电机	
				DM_Pos_Speed_Control(&hcan2, 0x01, target_Big_Pitch, 1.0f);
				TFR_count0 = 12;
		break;
		case 12:
				// 发送第二个电机
				DM_Pos_Speed_Control(&hcan2, 0x02, target_Smoll_Pitch, 0.7f);
				TFR_count0 = 13;
		break;
		case 13:
				// 发送第三个电机
				DM_Pos_Speed_Control(&hcan2, 0x05, target_End_Pitch, 20.0f);

				// 判断循环是否结束
				if(count1)
				{
					if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 1.0f && 
							fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 1.0f && 
								fabs(DM_endPitch_back.engle - target_End_Pitch) < 1.0f)
									TFR_count0 = 30; // 如果已经到位，直接跳转到结束的步骤
					else	TFR_count0 = 11;
				}
				else
				{
					if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 1.0f && 
							fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 1.0f && 
								fabs(DM_endPitch_back.engle - target_End_Pitch) < 1.0f &&
														R_access_flag==0)
					{
							R_access_flag = 1;
							TFR_count0 = 10; // 如果已经到位，直接跳转到下一个移动的步骤
					}
											
					else if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 1.0f && 
								fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 1.0f && 
									fabs(DM_endPitch_back.engle - target_End_Pitch) < 1.0f &&
														R_access_flag)
					{
							R_access_flag = 0;
							TFR_count0 = 8; // 如果已经到位，直接跳转到下一个移动的步骤
					}
												
					else	TFR_count0 = 11;
				}
		break;
		//================================控制电机就位===============================================

		//============================最终校准与退出===============================================
		case 30:
				//最终校准与退出
				if(fabs(DM_YAW_back.engle) < 0.5f)
				{
					TFR_count0 = 0;		// 重置状态机
					Left_flag = 0;
					Right_flag = 0;
					target_Big_Pitch = 0.0f;
					target_Smoll_Pitch = 0.0f;
					target_End_Pitch = 0.0f;
					target_yaw = 0.0f;
				}
				else	DM_Pos_Speed_Control(&hcan2,0x04,0.0f,1.5f);
		break;
		//============================最终校准与退出===============================================

		default:	break;
	}
}
//====================================以下为键鼠遥控的控制说明==============================================================
// 【1】当左侧拨杆为最下端时为底盘控制，此时就是正常的WASD控制底盘的前后左右，鼠标的X轴控制左右转
//	2、对于遥控的话左拨杆控制Yaw轴转向，右拨杆控制底盘前后左右
// 【3】当左侧拨杆为中间时为机械臂控制（是大臂小臂的两个Pitch，Yaw和小臂的Roll轴），没有末端和底盘的控制
//	4、对于遥控器左侧拨杆的Y控制机械臂末端在二维上的X，右侧拨杆的Y控制机械臂末端在二维上的Y
//	5、遥控左侧拨杆的X控制机械臂的YAW，右侧拨杆的X控制机械臂小臂上的Roll轴
//	6、此时的键鼠控制逻辑是使用QWERASDF六个键位控制，解释如下
//	7、QR控制大臂YAW轴转，AF控制小臂Roll轴转，WS控制二维的Y，ED控制二维的X（鼠标并没有用上）
// 【8】当遥控器左侧拨杆为最上端时，此时控制的是大臂的YAW，小臂的Roll，末端的Pitch和末端的Roll（这一挡位本意就是用于控制末端）
//	9、使用了遥控器左右两个拨杆的X，左侧拨杆是机械臂的YAW,右侧拨杆是机械臂小臂的Roll，主要还是键鼠控制，使用了QWEASDZXCV（其中ZXCV还没写）
//	10、WS对应末端的Pitch，QE对应机械臂的YAW,AD对应机械臂的roll，鼠标左右键对应末端的roll轴
// 【11】当遥控器右面的拨杆为最上面时，强制锁定自定义控制器，此时不使用自定义控制器，所有的控制都使用键鼠和遥控，用来应对场上发生的意外情况
// 【12】当遥控器右面的拨杆为中间或者最下面时（还没有写好），放开自定义控制权，但是自定义控制器发送端还有控制权限，只不过不是！！！强锁了！！！
//	13、目前想要用侧边滚轮对应末端执行器的夹取和松开，向上拨为夹住，再向下拨动为松开（后面写）
//


