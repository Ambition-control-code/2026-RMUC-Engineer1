/***@role: Cubemx的任务函数文件，防止意外删除任务***/
#include "ALL.h"

//1：60，2：0，3：0，4：40

/***@role:	挂起和恢复任务的句柄,队列的句柄***/
extern osThreadId dJi_motor_armHandle;
extern osThreadId mainTaskHandle;
extern osThreadId deal_IMU_dataHandle;
extern osThreadId Data_HandleHandle;

extern osMessageQueueId_t Claw_flagHandle;
extern osMessageQueueId_t MYor_remoteHandle;
//=============================================


/***
 *@role:	错误判断标志位
 *@remember:	if(err_flag == 1)	代码进入了错误判断，且是非常严重的错误，竟然会数据紊乱
***/
uint8_t err_flag = 0;
extern uint32_t life_state_new;
extern uint32_t life_state_old;
uint8_t xxx = 0;
uint8_t Left_flag = 0,Right_flag = 0;
extern uint8_t claw_flag;

extern uint8_t TFR_count0;
extern uint8_t count1;
extern uint8_t L_access_flag,R_access_flag;
extern uint8_t Left_flag,Right_flag;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/***@role: 达妙机械臂执行任务***/
uint8_t enable_count = 0;
uint8_t ARM_Init_flag = 0;

void DM_Arm_Task(void const * argument)
{
  /* USER CODE BEGIN SendMyControl_Data_Task */
	uint8_t task_number0 = 0;
	uint8_t task_number1 = 0;
	uint8_t MyControl_flag = 0;
  /* Infinite loop */
	for(;;)
  {
//===============================达妙电机的使能==================================================
		if(ARM_Init_flag == 0)	//达妙电机未使能标志位判断，如果使能成功是1.
		{
			enable_count = 0;
			ARM_Init_flag = 0;
			
			if(DM_YAW_back.state      == 1)		enable_count ++;
			else	DM_Motor_Mode_enable(&hcan2,0x04,DM_POS_MODE);
			
			if(Big_Pitch_back.state   == 1)		enable_count ++;
			else	DM_Motor_Mode_enable(&hcan2,0x01,DM_POS_MODE);
			
			if(Smoll_Pitch_back.state == 1)		enable_count ++;
			else	DM_Motor_Mode_enable(&hcan2,0x02,DM_POS_MODE);
			
			if(DM_ROLL_back.state     == 1)		enable_count ++;
			else	DM_Motor_Mode_enable(&hcan2,0x03,DM_POS_MODE);
			
			if(DM_endPitch_back.state == 1)		enable_count ++;
			else	DM_Motor_Mode_enable(&hcan2,0x05,DM_POS_MODE);
			
			if(DM_endRoll_back.state  == 1)		enable_count ++;
			else	DM_Motor_Mode_enable(&hcan2,0x06,DM_POS_MODE);
			
			if(DM_Claw_back.state     == 1)		enable_count ++;
			else	DM_Motor_Mode_enable(&hcan2,0x07,DM_POS_MODE);
			
			if(enable_count == 7)		ARM_Init_flag = 1;
		}
//===============================电机初始化，将关节初始化转到初始角度=============================
		else if(ARM_Init_flag == 1)	//全部使能后才能进入
		{
			ARM_Init();
			if(DM_Init_count == 8)	ARM_Init_flag = 2;
		}
//===============================开始正式的电机运行任务===========================================
		else if(ARM_Init_flag == 2)
		{
			uint8_t remote_MyControl_flag_t;
			osMessageQueueGet(MYor_remoteHandle,&remote_MyControl_flag_t,0,0);
			
			MyControl_flag = remote_MyControl_flag_t;
			switch(MyControl_flag)
			{
				case 0:
				{
					switch(task_number0)
					{
						case 0:	DM10010L_Control_task();
										task_number0 = 1;			break;
						case 1:	DM4340_Control_task();
										task_number0 = 2;			break;
						case 2: DM4340_endPitch_Control();
										task_number0 = 3;			break;
						case 3: DM4310_claw_Control();
										task_number0 = 4;			break;
						case 4: DM8009_xPitch_Control_task();
										task_number0 = 5;			break;
						case 5: DM10010L_xYaw_Control_task();
										task_number0 = 6;			break;
						case 6: DM4310_endRoll_Control();
										task_number0 = 0;			break;
					}
				}	break;
				case 1:
				case 2:
				{
					Arm_Task();
					
					switch(Chassis_or_Arm_or_End)
					{
						case 1:	//末端达妙电机的遥控控制
						{
							switch(task_number1)
							{
								case 0:	DM_Pos_Speed_Control(&hcan2,0x05,Set_end_Pit,5.0f);
												task_number1 = 1;			break;
								case 1:	DM_Pos_Speed_Control(&hcan2,0x04,Set_Arm_yaw,1.0f);
												task_number1 = 2;			break;
								case 2:	DM4310_claw_Control();  
												task_number1 = 3;			break;
								case 3:	DM_Pos_Speed_Control(&hcan2,0x06,Set_end_roll,1.0f);
												task_number1 = 4;			break;
								case 4:	DM_Pos_Speed_Control(&hcan2,0x03,Set_Arm_roll,5.0);
												task_number1 = 0;			break;
							}
						}	break;
						case 2://机械臂达妙电机的遥控控制
						{
							switch(task_number1)
							{
								case 0: DM_Pos_Speed_Control(&hcan2,0x04,Set_Arm_yaw,0.3f);
												task_number1 = 1;			break;
								case 1:	DM_Pos_Speed_Control(&hcan2,0x02,Smoll_Pitch,1.0f);
												task_number1 = 2;			break;
								case 2:	DM4310_claw_Control();
												task_number1 = 3;			break;
								case 3:	DM_Pos_Speed_Control(&hcan2,0x01,Big_Pitch,1.0f);
												task_number1 = 4;			break;
								case 4:	DM_Pos_Speed_Control(&hcan2,0x03,Set_Arm_roll,5.0);
												task_number1 = 0;			break;
							}
						}	break;
						default:	
							
						break;//其余情况跳出
					}
				}	break;
				case 5:
				{
					switch(task_number1)
					{
						case 0:	DM_Pos_Speed_Control(&hcan2,0x05,42.81f,25.0f);
										task_number1 = 1;			break;
						case 1:	DM_Pos_Speed_Control(&hcan2,0x04,0.0f,1.0f);
										task_number1 = 2;			break;
						case 2:	DM4310_claw_Control();  
										task_number1 = 3;			break;
						case 3:	DM_Pos_Speed_Control(&hcan2,0x06,DM_endRoll_back.engle,1.0f);
										task_number1 = 4;			break;
						case 4:	DM_Pos_Speed_Control(&hcan2,0x03,0.0f,25.0);
										task_number1 = 5;			break;
						case 5:	DM_Pos_Speed_Control(&hcan2,0x02,19.75f,1.5f);
										task_number1 = 6;			break;
						case 6:	DM_Pos_Speed_Control(&hcan2,0x01,0.0f,1.5f);
										task_number1 = 0;			break;
							}
				}	break;
				case 6:
				{
					if(custom_data.Custom_Data.Mid_flag&&Left_flag)
					{
						count1 = 0;
						TFR_count0 = 0;		// 重置状态机
						Left_flag = 0;
						L_access_flag = 0;
					}
					else if(custom_data.Custom_Data.Mid_flag&&Right_flag)
					{
						count1 = 0;
						TFR_count0 = 0;		// 重置状态机
						Right_flag = 0;
						R_access_flag = 0;
					}
					else if(Left_flag)	TFR_conrrol_L();
					else if(Right_flag)	TFR_conrrol_R();
				}	break;
				default:	break;
			}
			
//			OverLoad_detection();//过载保护
			Clean_motor_erro();//清楚错误标志
		}
		else	err_flag = 1;
		
		osDelay(1);
	}
  /* USER CODE END SendMyControl_Data_Task */
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/***@roel：	陀螺仪任务***/
void IMU_Task(void const * argument)
{
  /* USER CODE BEGIN IMU_Task */
  /* Infinite loop */
  for(;;)
  {
		IMU_task();
		if(offset_OK_flag==0)	offset_OK_flag = cali_gyro_hook();
		life_state_new ++;			//生命状态值自增，判断图传是否连接的标志
		life_state_old ++;			//生命状态值自增，白色遥控器是否连接的标志
    osDelay(1);
  }
  /* USER CODE END IMU_Task */
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/***@role:	数据传输任务***/
float Structure_Yaw_set = 0.0f;
float Yaw_turn_speed = 0.0f;
extern uint32_t Chassis_life;
void Data_Solve_task(void const * argument)
{
  /* USER CODE BEGIN Data_to_chassis_task */
	uint8_t Wait_time = 0;
  /* Infinite loop */
  for(;;)
  {
		float Q = 0.0f,E = 0.0f;
		
		Flag_Inspect_task();//标志位更改任务
		
		if(Button_Check_Event(custom_data.Custom_Data.Claw_flag,&Key_Claw_flag))	claw_flag = !claw_flag;
		if(New_VT03_data.mode_sw!=0 && remote_MyControl_flag)
		{
			if(Button_Check_Event(VT13_use.use_data.fn_1,&Key_fn1_flag))	Left_flag = !Left_flag;
			if(Button_Check_Event(VT13_use.use_data.fn_2,&Key_fn2_flag))	Right_flag = !Right_flag;
		}
		else
		{
			if(Button_Check_Event(custom_data.Custom_Data.Left_flag,&Key_Left_flag))	Left_flag = !Left_flag;
			if(Button_Check_Event(custom_data.Custom_Data.Right_flag,&Key_Right_flag))	Right_flag = !Right_flag;
		}
		
		switch(Wait_time)
		{
			case 0:			//控制底盘绝大多数数据
				Remote_To_Chassis();//向底盘发送遥控器数据任务
				Wait_time = 1;
			break;
			case 1:			//控制UI数据
				Collision_detection(&Joint_collision_flag);
				Wait_time = 2;
			break;
			case 2:			//控制转向
				New_Remote_fifo();                           // 调用按键加速处理函数
			
				Q = (0.0025f * deal_Q);                //左转
				E = (0.0025f * deal_E);                //右转
	
				if(Chassis_or_Arm_or_End != 0 || Chassis_life > 500)	
				{
					Structure_Yaw_set = 0.0f;
					Yaw_turn_speed = 0.0f;
				}
				else
				{
					Yaw_turn_speed = (New_VT03_data.L_rocker_x / 500.0f) + E - Q;
					if(Big_Pitch_back.engle < -60.0f || New_VT03_data.shift)	Yaw_turn_speed = Yaw_turn_speed * 0.35f;
					Structure_Yaw_set -= Yaw_turn_speed;
					Turn_Date_sent(Structure_Yaw_set);
				}
				Wait_time = 0;
		}
    osDelay(1);
  }
  /* USER CODE END Data_to_chassis_task */
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=++++++
/***@role：	控制舵机云台的任务***/
void Send_angle_task(void const * argument)
{
  /* USER CODE BEGIN Send_angle_task */
	float Picture_yaw = 45.0f,Picture_pit = 145.0f;
	uint8_t yaw_change_flag = 0;
	uint8_t last_R = 0;
  /* Infinite loop */
  for(;;)
  {
		if(New_VT03_data.New_R!=last_R)
		{
			if(yaw_change_flag){
				Picture_yaw = 45.0f;
				yaw_change_flag = 0;
			}
			else{
				Picture_yaw = 230.0f;
				yaw_change_flag = 1;
			}
			last_R = New_VT03_data.New_R;
		}
		
		Picture_yaw -= VT13_use.use_data.mouse_x;
		Picture_pit -= VT13_use.use_data.mouse_y;
		
		// 限幅保护，防止代码异常导致舵机内部齿轮扫齿，留了1度的预留
		if(Picture_yaw < 1.0f) Picture_yaw = 1.0f;
		if(Picture_yaw > 250.0f) Picture_yaw = 250.0f;
	
		if(Picture_pit < 32.0f) Picture_pit = 32.0f;
		if(Picture_pit > 205.0f) Picture_pit = 205.0f;
		
		Set_Servo_Angle(Picture_yaw,Picture_pit);
		
    osDelay(10);
  }
  /* USER CODE END Send_angle_task */
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/***@role:	debug调试函数，需要的时候开，不需要的时候挂起任务就行***/
void debug_task(void const * argument)
{
  /* USER CODE BEGIN debug_task */
	uint8_t dubug_flag = 1;	//非零情况下挂起该函数，零的情况用于调试
  /* Infinite loop */
  for(;;)
  {
		if(dubug_flag == 0)	VOFA_Print(8,Set_Arm_yaw,DM_YAW_back.engle,Big_Pitch,Big_Pitch_back.engle,
			Smoll_Pitch,Smoll_Pitch_back.engle,Set_end_roll,DM_endRoll_back);
		else	vTaskSuspend(NULL);
		osDelay(1);
  }
  /* USER CODE END debug_task */
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//清楚错误标志位
uint32_t yaw = 0,B_pitch = 0,S_pitch = 0,B_roll = 0,End_pitch = 0,End_roll = 0,Claw_state = 0;

void Clean_motor_erro(void)
{
	//清楚错误标志位的现象太难复刻，不好试，注释掉，大不了就是被打死重新上电一下
//	if(DM_endPitch_back.state > 1)
//	{
//		DM_Motor_clean_erro(&hcan2,0x05,DM_POS_MODE);
//		osDelay(5);
//		DM_Motor_Mode_enable(&hcan2,0x05,DM_POS_MODE);
//		osDelay(5);
//		
//		if(DM_endPitch_back.state < 1)
//		{
//			enable_count = 0;
//			ARM_Init_flag = 0;
//		}
//		if(DM_endPitch_back.state == 1)	ARM_Init_flag = 1;
//	}
//	if(DM_YAW_back.state > 1)
//	{
//		DM_Motor_clean_erro(&hcan2,0x04,DM_POS_MODE);
//		osDelay(5);
//		DM_Motor_Mode_enable(&hcan2,0x04,DM_POS_MODE);
//		osDelay(5);
//		
//		if(DM_YAW_back.state < 1)
//		{
//			enable_count = 0;
//			ARM_Init_flag = 0;
//		}
//		if(DM_YAW_back.state == 1)	ARM_Init_flag = 1;
//	}
//	if(Big_Pitch_back.state > 1)
//	{
//		DM_Motor_clean_erro(&hcan2,0x01,DM_POS_MODE);
//		osDelay(5);
//		DM_Motor_Mode_enable(&hcan2,0x01,DM_POS_MODE);
//		osDelay(5);
//		
//		if(Big_Pitch_back.state < 1)	
//		{
//			enable_count = 0;
//			ARM_Init_flag = 0;
//		}
//		if(Big_Pitch_back.state == 1)	ARM_Init_flag = 1;
//	}
//	if(Smoll_Pitch_back.state > 1)
//	{
//		DM_Motor_clean_erro(&hcan2,0x02,DM_POS_MODE);
//		osDelay(5);
//		DM_Motor_Mode_enable(&hcan2,0x02,DM_POS_MODE);
//		osDelay(5);
//		
//		if(Smoll_Pitch_back.state < 1)
//		{
//			enable_count = 0;
//			ARM_Init_flag = 0;
//		}
//		if(Smoll_Pitch_back.state == 1)	ARM_Init_flag = 1;
//	}
//	if(DM_ROLL_back.state > 1)
//	{
//		DM_Motor_clean_erro(&hcan2,0x03,DM_POS_MODE);
//		osDelay(5);
//		DM_Motor_Mode_enable(&hcan2,0x03,DM_POS_MODE);
//		osDelay(5);
//			
//		if(DM_ROLL_back.state < 1)	
//		{
//			enable_count = 0;
//			ARM_Init_flag = 0;
//		}
//		if(DM_ROLL_back.state == 1)	ARM_Init_flag = 1;
//	}
//	if(DM_endRoll_back.state > 1)
//	{
//		DM_Motor_clean_erro(&hcan2,0x06,DM_POS_MODE);
//		osDelay(5);
//		DM_Motor_Mode_enable(&hcan2,0x06,DM_POS_MODE);
//		osDelay(5);
//		
//		if(DM_endRoll_back.state < 1)	
//		{
//			enable_count = 0;
//			ARM_Init_flag = 0;
//		}
//		if(DM_endRoll_back.state == 1)	ARM_Init_flag = 1;
//	}
//	if(DM_Claw_back.state > 1)
//	{
//		DM_Motor_clean_erro(&hcan2,0x07,DM_POS_MODE);
//		osDelay(5);
//		DM_Motor_Mode_enable(&hcan2,0x07,DM_POS_MODE);
//		osDelay(5);
//		
//		if(DM_Claw_back.state < 1)
//		{
//			enable_count = 0;
//			ARM_Init_flag = 0;
//		}
//		if(DM_Claw_back.state == 1)	ARM_Init_flag = 1;
//	}
//	
	//=================检测电机是否断联=================================//
	if(remote_MyControl_flag!=6)
	{
		if(Chassis_or_Arm_or_End == 2)
		{
			yaw ++;
			B_pitch ++;
			S_pitch ++;
			B_roll ++;
			Claw_state ++;
		}
		if(Chassis_or_Arm_or_End == 1)
		{
			End_pitch ++;
			End_roll ++;
			Claw_state ++;
			yaw ++;
			B_roll ++;
		}
		if(remote_MyControl_flag == 0)
		{
			End_pitch ++;
			End_roll ++;
			Claw_state ++;
			yaw ++;
			B_roll ++;
			B_pitch ++;
			S_pitch ++;
		}
	}
	
	if(yaw>1000||B_pitch>1000||S_pitch>1000||B_roll>1000||End_pitch>1000||End_roll>1000||Claw_state>1000)
	{
		enable_count = 0;
		ARM_Init_flag = 0;
		DM_Init_count = 0;
		yaw = 0;
		B_pitch = 0;
		S_pitch = 0;
		End_pitch = 0;
		B_roll = 0;
		End_roll = 0;
		
		if(yaw>1000)	Joint_collision_flag.data0 = 0;
		if(B_pitch>1000)	Joint_collision_flag.data1 = 0;
		if(S_pitch>1000)	Joint_collision_flag.data2 = 0;
		if(B_roll>1000)	Joint_collision_flag.data3 = 0;
		if(End_pitch>1000)	Joint_collision_flag.data4 = 0;
		if(End_roll>1000)	Joint_collision_flag.data5 = 0;
		if(Claw_state>1000)	Joint_collision_flag.data6 = 0;
	}
	//=====================================================================//
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


