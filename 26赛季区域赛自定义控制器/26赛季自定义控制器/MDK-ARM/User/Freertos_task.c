#include "ALL.h"

extern osMessageQueueId_t taskbegainHandle;
extern osThreadId_t Init_taskHandle;
extern osThreadId_t SendMyControl_DHandle;
extern osThreadId_t back_controlHandle;

uint8_t maintask_begain = 0;
float Yaw_angle1 = 0.0f,Roll_angle1 = 0.0f,End_Pitch = 0.0f;
float endRoll_angle1 = 0.0f;


/***@role: 自定义控制器数据发送任务函数***/
void SendMyControl_Data_Task(void const * argument)
{
  /* USER CODE BEGIN SendMyControl_Data_Task */
  /* Infinite loop */
  for(;;)
  {
		if(maintask_begain == 0)	vTaskSuspend(NULL);
    else
		{
			maintask_begain = 2;
			Custos_Control_Serial();
		}
		osDelay(34);
  }
  /* USER CODE END SendMyControl_Data_Task */
}
//===================================================



/***@role： 陀螺仪任务执行函数***/
void IMU_Task(void const * argument)
{
  /* USER CODE BEGIN IMU_Task */
  /* Infinite loop */
  for(;;)
  {
		IMU_task();
		if(offset_OK_flag==0)offset_OK_flag = cali_gyro_hook();
		osDelay(1);
  }
  /* USER CODE END IMU_Task */
}
//=========================================================



/***@role： 自定义控制器的按钮获取任务函数***/
//      右            左            中
uint8_t PE13_flag = 0,PC6_flag = 0,PE14_flag = 0;
void SwitchControl_task(void const * argument)
{
  /* USER CODE BEGIN SwitchControl_task */
  /* Infinite loop */
  for(;;)
  {
		if(maintask_begain == 0)
		{
			PE13_flag = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
			PC6_flag = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6);
			PE14_flag = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14);
		}
		else
		{
			maintask_begain = 2;
			Switchi_task(&Switch_flag);
		}
		
    osDelay(1);
  }
  /* USER CODE END SwitchControl_task */
}
//==============================================================



/***@role:	获取场上机械臂关节角度信息的任务函数***/
extern uint8_t Referee_flag;
extern Arm_union_data_t Arm_union_data;
void get_arm_task(void const * argument)
{
  /* USER CODE BEGIN get_arm_task */
  /* Infinite loop */
  for(;;)
  {
		if(Referee_flag == 1)
		{
			Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[0]);     	//处理缓冲区0的数据
			Referee_flag = 0;
		}
		else
		{
			Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[1]);			//处理缓冲区1的数据
			Referee_flag = 0;
		}
    osDelay(1);
  }
  /* USER CODE END get_arm_task */
}
//==================================================================================================


 
/***@role:	对于超限阻尼和重力补偿的任务函数***/
void back_control_task(void const * argument)
{
  /* USER CODE BEGIN back_control_task */
	GravComp_Init_L(&GravComp_L);
	GravComp_Init_R(&GravComp_R);
	GravComp_Init_Roll(&GravComp_Roll);
  /* Infinite loop */
  for(;;)
  {
		vTaskSuspend(NULL);
		if(maintask_begain == 0)	vTaskSuspend(NULL);
		else
		{
			GravComp_Calc(&GravComp_L,Big_Pitch_back.ecd_angle,Smoll_Pitch_back.ecd_angle,Big_Pitch_back.re_speed,Smoll_Pitch_back.re_speed);
			GravComp_Calc(&GravComp_R,End_Pitch_back.ecd_angle,0.0f,End_Pitch_back.re_speed,0.0f);
			GravComp_Calc(&GravComp_Roll,0.0f,0.0f,Big_Roll_back.re_speed,End_Roll_back.re_speed);
		
			Motor_control(&hcan1,0x1FE,GravComp_L.Current1,1);
			Motor_control(&hcan1,0x1FE,GravComp_L.Current2,3);
		
			Motor_control(&hcan1,0x1FE,GravComp_R.Current1,2);
		
			Motor_control(&hcan1,0x200,0,2);
			Motor_control(&hcan1,0x200,GravComp_Roll.Current1,2);
			Motor_control(&hcan1,0x200,GravComp_Roll.Current2,3);
		
//			Motor_enable(&hcan1,0x1FE,6);
//			Motor_enable(&hcan1,0x200,6);
		}

    osDelay(1);
  }
  /* USER CODE END back_control_task */
}
//==================================================



/***@role:	初始化各个关节任务***/
uint32_t wait_num1 = 0,wait_num2 = 0;
void Init_motor_Task(void *argument)
{
  /* USER CODE BEGIN Init_motor_Task */
  /* Infinite loop */
  for(;;)
  {
		float Bigm1 = 0.0f,Bigm2 = 0.0f,Smollm1 = 0.0f,Smollm2 = 0.0f,Endm1 = 0.0f,Endm2 = 0.0f;
		
		Bigm1 = PID_Cal(&BigPitch_angle_pid,Big_Pitch_back.ecd_angle,BigPitch_setangle);
		Bigm2 = PID_Cal(&BigPitch_speed_pid,Big_Pitch_back.re_speed,Bigm1);
		Motor_control(&hcan1,0x1FE,Bigm2,1);
		
		Smollm1 = PID_Cal(&SmollPitch_angle_pid,Smoll_Pitch_back.ecd_angle,SmollPitch_setangle);
		Smollm2 = PID_Cal(&SmollPitch_speed_pid,Smoll_Pitch_back.re_speed,Smollm1);
		Motor_control(&hcan1,0x1FE,Smollm2,3);
		
		Endm1 = PID_Cal(&endPitch_angle_pid,End_Pitch_back.ecd_angle,EndPitch_setangle);
		Endm2 = PID_Cal(&endPitch_speed_pid,End_Pitch_back.re_speed,Endm1);
		Motor_control(&hcan1,0x1FE,Endm2,2);
		
//		Motor_enable(&hcan1,0x1FE,6);
		
		if(PE13_flag == 1 && PC6_flag == 1 && PE14_flag == 1)	wait_num1 ++;
		else if(wait_num1 <= 1000)	wait_num1 = 0;
		if(PE13_flag == 0 && PC6_flag == 0 && PE14_flag == 0 && wait_num1 > 1000)
		{
			if((wait_num2 ++) > 150)
			{
//				Yaw_angle1 = Yaw_back.ecd_angle;
				Yaw_angle1 = Yaw_back.ecd_angle;
				Roll_angle1 = Big_Roll_back.ecd_angle;
				endRoll_angle1 = End_Roll_back.ecd_angle;
				End_Pitch = End_Pitch_back.ecd_angle;
				
				if(maintask_begain == 0)	maintask_begain = 1;
				
				vTaskResume(SendMyControl_DHandle);
//				vTaskResume(back_controlHandle);
				
				vTaskSuspend(NULL);
			}
		}
		else wait_num2 = 0;
    osDelay(2);
  }
  /* USER CODE END Init_motor_Task */
}
//=====================================================


