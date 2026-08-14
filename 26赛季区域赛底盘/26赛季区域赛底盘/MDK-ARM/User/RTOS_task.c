/***@role: 用于存放RTOS的任务，防止在Cubemx中误删掉之后导致任务丢失***/
#include "ALL.h"
extern uint32_t life_state;


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/***@role: 主要的底盘控制任务***/
void SendMyControl_Data_Task(void const * argument)
{
  /* USER CODE BEGIN SendMyControl_Data_Task */
  /* Infinite loop */
  for(;;)
  {
		Chassis_Mecanum_control();
		osDelay(2);
  }
  /* USER CODE END SendMyControl_Data_Task */
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/***@roel： 陀螺仪任务***/
void IMU_Task(void const * argument)
{
  /* USER CODE BEGIN IMU_Task */
  /* Infinite loop */
  for(;;)
  {
		life_state ++;
		IMU_task();
		if(offset_OK_flag==0)offset_OK_flag = cali_gyro_hook();
    osDelay(1);
  }
  /* USER CODE END IMU_Task */
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/***
 *@role:  登岛任务
 *左4右3腿
 *正电流右腿收左腿蹬
 *负电流右腿蹬左腿收
***/
uint32_t L_leg = 0,R_leg = 0;
float normal_angle = 3.5f;
void Lifting_task(void const * argument)
{
  /* USER CODE BEGIN Lifting_task */
	SpringDamper_Init(&SpringDamperSystem);
	
	MIT_L_leg.FDN_Tff = 3.5f;
	MIT_R_leg.FDN_Tff = -3.5f;
	uint8_t hip_Init = 0;
  /* Infinite loop */
  for(;;)
  {
		switch(hip_Init)
		{
			case 0:
			{
				if(DM_R_leg.state == 1)	hip_Init ++;
				else	DM_Motor_Mode_enable(&hcan2,0x03,DM_MIT_MODE);
			}	break;
			case 1:
			{
				if(DM_L_leg.state == 1)	hip_Init++;
				else	DM_Motor_Mode_enable(&hcan2,0x04,DM_MIT_MODE);
			}	break;
			case 2:
			{
				DM_MIT_Control(&hcan2,0x04,0.0f,0.0f,0.0f,0.0f,0.0f);
				hip_Init ++;
			}	break;
			case 3:
			{
				DM_MIT_Control(&hcan2,0x03,0.0f,0.0f,0.0f,0.0f,0.0f);
				
				if(fabs(DM_R_leg.engle)>40.0f&&fabs(DM_L_leg.engle)>40.0f)
					normal_angle = 60.0f;
				
				hip_Init ++;
			}	break;
			case 4:
			{
				New_Leg_Control();
				
				if(DM_R_leg.state > 1)
				{
					DM_Motor_clean_erro(&hcan2,0x03,DM_MIT_MODE);
					osDelay(1);
					DM_Motor_Mode_enable(&hcan2,0x03,DM_MIT_MODE);
				}
				if(DM_L_leg.state > 1)
				{
					DM_Motor_clean_erro(&hcan2,0x04,DM_MIT_MODE);
					osDelay(1);
					DM_Motor_Mode_enable(&hcan2,0x04,DM_MIT_MODE);
				}
				
				L_leg ++;
				R_leg ++;
				if(L_leg > 1000 || R_leg > 1000)
				{
					ui_g_Ungroup2_R_track_state->number = 0;
					ui_g_Ungroup2_R_track_state->color = 0;		
					ui_g_Ungroup2_L_track_state->number = 0;
					ui_g_Ungroup2_L_track_state->color = 0;	
					
					hip_Init = 0;
					SpringDamper_Init(&SpringDamperSystem);
				}
				else
				{
					ui_g_Ungroup2_R_track_state->number = 1;
					ui_g_Ungroup2_R_track_state->color = 2;
					ui_g_Ungroup2_L_track_state->number = 1;
					ui_g_Ungroup2_L_track_state->color = 2;
				}
			}	break;
		}
    osDelay(1);
  }
  /* USER CODE END Lifting_task */
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/***@role：	调试执行任务，后期不用可以直接删掉或者挂起，避免对CPU资源的占用***/
void Debug_task(void const * argument)
{
  /* USER CODE BEGIN Debug_task */
  uint8_t dubug_flag = 1;	//任务是否被挂起的标志位，0为运行，非零挂起任务节省CPU资源
  /* Infinite loop */
  for(;;)
  {
		if(dubug_flag == 0)
		{
			VOFA_Print(4,3000.0f,L_track_back.re_speed,(-3000.0f),R_track_back.re_speed);
			osDelay(1);
		}
		else	vTaskSuspend(NULL);
  }
  /* USER CODE END Debug_task */
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/***@role:	裁判系统UI的绘制任务***/
uint32_t R_Front_flag = 0,L_Front_flag = 0,R_Behind_flag = 0,L_Behind_flag = 0;
extern int ui_self_id;

void UI_task(void const * argument)
{
  /* USER CODE BEGIN UI_task */
	ui_self_id = Referee_System_Info.robot_status.robot_id;
	ui_init_g();
  /* Infinite loop */
  for(;;)
  {
		ui_self_id = Referee_System_Info.robot_status.robot_id;
		
		if(C_Date.v)	ui_init_g();
		
		R_Front_flag ++;
		L_Front_flag ++;
		R_Behind_flag ++;
		L_Behind_flag ++;
		
		if(R_Behind_flag > 500){
			ui_g_Ungroup2_R_behind_state->number = 0;
			ui_g_Ungroup2_R_behind_state->color = 0;
			
			ui_g_Ungroup1_R_behind_wheel->color = 4;
		}
		else{
			ui_g_Ungroup2_R_behind_state->number = 1;
			ui_g_Ungroup2_R_behind_state->color = 2;
			
			ui_g_Ungroup1_R_behind_wheel->color = 2;
		}
		if(L_Behind_flag > 500){
			ui_g_Ungroup2_L_behind_state->number = 0;
			ui_g_Ungroup2_L_behind_state->color = 0;
			
			ui_g_Ungroup1_L_behind_wheel->color = 4;
		}
		else{
			ui_g_Ungroup2_L_behind_state->number = 1;
			ui_g_Ungroup2_L_behind_state->color = 2;
			
			ui_g_Ungroup1_L_behind_wheel->color = 2;
		}
		if(R_Front_flag > 500){
			ui_g_Ungroup2_R_front_state->number = 0;
			ui_g_Ungroup2_R_front_state->color = 0;
			
			ui_g_Ungroup1_R_front_wheel->color = 4;
		}
		else{
			ui_g_Ungroup2_R_front_state->number = 1;
			ui_g_Ungroup2_R_front_state->color = 2;
			
			ui_g_Ungroup1_R_front_wheel->color = 2;
		}
		if(L_Front_flag > 500){
			ui_g_Ungroup2_L_front_state->number = 0;
			ui_g_Ungroup2_L_front_state->color = 0;
			
			ui_g_Ungroup1_L_front_wheel->color = 4;
		}
		else{
			ui_g_Ungroup2_L_front_state->number = 1;
			ui_g_Ungroup2_L_front_state->color = 2;
			
			ui_g_Ungroup1_L_front_wheel->color = 2;
		}
		
		ui_g_Ungroup4_big_pitch_state->number = ui_t_data.data1;
		ui_g_Ungroup4_yaw_state->number = ui_t_data.data0;
		ui_g_Ungroup4_smoll_pitch_state->number = ui_t_data.data2;
		ui_g_Ungroup4_big_roll_state->number = ui_t_data.data3;
		ui_g_Ungroup4_end_pitch_state->number = ui_t_data.data4;
		ui_g_Ungroup4_end_roll_state->number = ui_t_data.data5;
		ui_g_Ungroup4_claw_state->number = ui_t_data.data6;
		
		ui_g_Ungroup3_Big_Pitch->color = ui_t_data.Big_pitch_t;
		ui_g_Ungroup4_big_pitch_state->color = ui_t_data.Big_pitch_t;
		ui_g_Ungroup3_yaw->color = ui_t_data.yaw_t;
		ui_g_Ungroup4_yaw_state->color = ui_t_data.yaw_t;
		ui_g_Ungroup3_smoll_pitch->color = ui_t_data.Smoll_pitch_t;
		ui_g_Ungroup4_smoll_pitch_state->color = ui_t_data.Smoll_pitch_t;
		ui_g_Ungroup3_big_roll->color = ui_t_data.Big_roll_t;
		ui_g_Ungroup4_big_roll_state->color = ui_t_data.Big_roll_t;
		ui_g_Ungroup3_end_pitch->color = ui_t_data.End_pitch_t;
		ui_g_Ungroup4_end_pitch_state->color = ui_t_data.End_pitch_t;
		ui_g_Ungroup3_end_roll->color = ui_t_data.End_roll_t;
		ui_g_Ungroup4_end_roll_state->color = ui_t_data.End_roll_t;
		
		ui_update_g();
		
    osDelay(10);
  }
  /* USER CODE END UI_task */
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


