#include "Send_MyData.h"
#include "ALL.h"

extern float Yaw_angle1,Roll_angle1,End_Pitch;
extern float endRoll_angle1;

Controller_t tx_data; // 自定义控制器发送的数据
controller_union_data Tx_custom_data;

/***@role: UART1的DMA发送的重新初始化函数，用于DMA初次初始化出问题重新初始化串口时使用***/
extern DMA_HandleTypeDef hdma_usart1_tx;
void MY_UART1_DMA_TX_Init(void)
{
	__HAL_DMA_DISABLE(&hdma_usart1_tx);																											// 确保DMA处于禁用状态进行配置
	while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)	{__HAL_DMA_DISABLE(&hdma_usart1_tx);}
	
	__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF6 | DMA_HISR_HTIF6 | DMA_HISR_TEIF6);// 清除所有DMA标志
	hdma_usart1_tx.Instance->PAR = (uint32_t)&(USART6->DR);  					 											// 发送数据寄存		// 配置DMA参数
	__HAL_DMA_SET_COUNTER(&hdma_usart1_tx, 39u);                       											// 设置传输长度（39个字节）
  __HAL_DMA_ENABLE_IT(&hdma_usart1_tx, DMA_IT_TC);																				// 启用DMA传输完成中断（可选，用于通知发送完成）
  SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);																					// 启用UART的DMA发送请求
}

/**
 * @brief 数据拼接函数，将帧头、命令码、数据段、帧尾头拼接成一个数组
 * @param data 数据段的数组指针
 * @param data_lenth 数据段长度
 */
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
	static uint8_t seq = 0;
	// 帧头数据
	tx_data.frame_header.sof = 0xA5;                              			// 数据帧起始字节，固定值为 0xA5
	tx_data.frame_header.data_length = data_lenth;                			// 数据帧中数据段的长度
	tx_data.frame_header.seq = seq++;                             			// 包序号
	append_CRC8_check_sum((uint8_t *)(&tx_data.frame_header), 5); 			// 添加帧头 CRC8 校验位
	// 命令码ID
	tx_data.cmd_id = CONTROLLER_CMD_ID;																	// 图传链路自定义控制器命令码
	// 数据段
	memcpy(tx_data.data, data, data_lenth);															// 有用的数据搬运
	// 帧尾CRC16，整包校验
	append_CRC16_check_sum((uint8_t *)(&tx_data), DATA_FRAME_LENGTH);		// CRC16帧尾校验
}

/**
 * @brief 数据拼接函数，将帧头、命令码、数据段、帧尾头拼接成一个数组
 * @param data 数据段的数组指针
 * @param data_lenth 数据段长度
 */
Buy_Ammo_t tx_data2;
static void Data_Concatenation_2(uint8_t *data, uint16_t data_lenth)
{
	static uint8_t seq = 0;
	// 帧头数据
	tx_data2.frame_header.sof = 0xA5;                              			// 数据帧起始字节，固定值为 0xA5
	tx_data2.frame_header.data_length = data_lenth;                			// 数据帧中数据段的长度
	tx_data2.frame_header.seq = seq++;                             			// 包序号
	append_CRC8_check_sum((uint8_t *)(&tx_data2.frame_header), 5); 			// 添加帧头 CRC8 校验位
	// 命令码ID
	tx_data2.cmd_id = BUY_AMMO_CMD_ID;																	// 命令码
	// 数据段
	memcpy(tx_data2.data, data, data_lenth);														// 有用的数据搬运
	// 帧尾CRC16，整包校验
	append_CRC16_check_sum((uint8_t *)(&tx_data2), DATA_FRAME_LENGTH_2);	// CRC16帧尾校验
}

/**
 * @brief  执行将数据发出的任务
**/
uint8_t retry_count = 0;
void Custos_Control_Serial(void)
{
	float Target_end_roll = (((End_Roll_back.ecd_angle-endRoll_angle1)+(End_Pitch_back.ecd_angle-End_Pitch))*0.5f);
	float Target_end_pitch = (((End_Roll_back.ecd_angle-endRoll_angle1)-(End_Pitch_back.ecd_angle-End_Pitch))*0.5f);
	
	Tx_custom_data.ALL_data.control_yaw = ((Yaw_back.ecd_angle - Yaw_angle1) * 0.5f);	// 自控C板yaw轴数据
//	Tx_custom_data.ALL_data.control_yaw = -(imu_angle.yaw_real - Yaw_angle1);				// 自控C板yaw轴数据
	// 自控大疆3508屁股控制大Pitch数据
	Tx_custom_data.ALL_data.Big_Pitch_data = (-Big_Pitch_back.ecd_angle + BigPitch_setangle);
	// 自控大疆3508屁股控制小Pitch数据
	Tx_custom_data.ALL_data.Smoll_Pitch_data = (Smoll_Pitch_back.ecd_angle - SmollPitch_setangle);
	// 自控大疆3508屁股控制大Roll数据
	Tx_custom_data.ALL_data.Roll_3508_data = ((Big_Roll_back.ecd_angle - Roll_angle1) * 0.5f); 										
	// 自控大疆3508屁股控制末端Pitch数据
//	Tx_custom_data.ALL_data.endPitch_data= (End_Pitch_back.ecd_angle - EndPitch_setangle);
	Tx_custom_data.ALL_data.endPitch_data= (End_Pitch_back.ecd_angle - End_Pitch);
//	Tx_custom_data.ALL_data.endPitch_data= (-Target_end_pitch);
	// 自控大疆3508屁股控制末端Roll数据
	Tx_custom_data.ALL_data.endRoll_data = ((End_Roll_back.ecd_angle - endRoll_angle1) * 30.0f);
//	Tx_custom_data.ALL_data.endRoll_data = (-Target_end_roll);
	// 自控夹爪控制标志位
	Tx_custom_data.ALL_data.Claw_flag  = Switch_flag.Claw_flag;
	// 自控一键存储标志位
	Tx_custom_data.ALL_data.Quick_storage_flag = Switch_flag.Quick_storage_flag;
	//自控
	Tx_custom_data.ALL_data.Left_flag = Switch_flag.LEFT_Quikely;
	Tx_custom_data.ALL_data.Right_flag = Switch_flag.RIGHT_Quikely;

	Tx_custom_data.ALL_data.data1 = Switch_flag.MID_flag;										// 预留数据位1
	Tx_custom_data.ALL_data.data2 = 0;										// 预留数据位2

	Data_Concatenation(Tx_custom_data.data,DATA_LENGTH);  //将以上有用的数据整合进裁判系统认可的数据包中
	
//=========以下三行是早期使用的单纯的串口发送数据，但是发现同时打开陀螺仪任务后数据效果并不好，于是改进使用UART6的DMA串口发送===========//
//	UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
//	HAL_UART_Transmit(&huart6, (uint8_t *)(&tx_data), sizeof(tx_data), 50);
//	portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
//===============================================================================================================//
	
	if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX) 
	{ //DMA串口发送函数
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_data), sizeof(tx_data));
//		VOFA_Print(4,Tx_custom_data.ALL_data.Smoll_Pitch_data,Tx_custom_data.ALL_data.Big_Pitch_data,
//		Tx_custom_data.ALL_data.endRoll_data,Tx_custom_data.ALL_data.control_yaw);
		retry_count = 0;
		aRGB_led_show(0x7F123456);
	}
	else if((HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) && retry_count < MAX_retry_count)
	{ //如果未进入上面的判断中，证明串口处于繁忙的状态，没有修改回就绪态，证明出现了BUG此时需要重新发送三次，看是否是偶然一次不能用了
		retry_count ++;
	}
	else if((HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) && retry_count == MAX_retry_count)
	{ 
		//重新发送三次依然无用之后会进入这里重启串口，重新进行DMA的初始化
		UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();		//任务临界区保护		开启保护
		HAL_UART_AbortTransmit(&huart1);																					//中止DMA发送传输
		HAL_UART_DeInit(&huart1);																									//失能UART1
		HAL_UART_Init(&huart1);																										//使能UART1
		MY_UART1_DMA_TX_Init();																										//UART1的DMA初始化（这里是再次初始化）
		portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);             		//任务临界区保护		关闭保护
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_data), sizeof(tx_data));		//DMA串口开始发送
		retry_count = 0;																													//重复发送计数器置零
	}
}



uint8_t Buy_Ammo_count = 8;
custom_client_data_t custom_client_data = {0};
void Buy_Ammo_Serial(void)
{
	switch(Buy_Ammo_count)
	{
		case 0:
		{
			custom_client_data.USE_DATA.key_value = 79;
			custom_client_data.USE_DATA.mouse_left = 0;
			custom_client_data.USE_DATA.mouse_right = 0;
			custom_client_data.USE_DATA.x_position = 0;
			custom_client_data.USE_DATA.y_position = 0;
			
			Data_Concatenation_2(custom_client_data.Send_data,BUY_AMMO_DATA_LENGTH);
			
			if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX) 
					HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_data2), sizeof(tx_data2));
			
			Buy_Ammo_count = 1;
		}	break;
		case 1:
		{
			//这里往下是对应客户端的鼠标坐标值，记得修改
			if(Difficulty_level.ONE == 1 && Difficulty_level.TWO == 0 && Difficulty_level.THREE == 0)	//一级难度坐标系
			{
				custom_client_data.USE_DATA.key_value = 0;
				custom_client_data.USE_DATA.x_position = 111;
				custom_client_data.USE_DATA.y_position = 111;
				
				Data_Concatenation_2(custom_client_data.Send_data,BUY_AMMO_DATA_LENGTH);
			
				if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX) 
						HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_data2), sizeof(tx_data2));
			
				Buy_Ammo_count = 2;
			}
			else if(Difficulty_level.ONE == 0 && Difficulty_level.TWO == 1 && Difficulty_level.THREE == 0)//二级难度坐标系
			{
				custom_client_data.USE_DATA.key_value = 0;
				custom_client_data.USE_DATA.x_position = 222;
				custom_client_data.USE_DATA.y_position = 222;
				
				Data_Concatenation_2(custom_client_data.Send_data,BUY_AMMO_DATA_LENGTH);
			
				if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX) 
						HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_data2), sizeof(tx_data2));
			
				Buy_Ammo_count = 2;
			}
			else if(Difficulty_level.ONE == 0 && Difficulty_level.TWO == 0 && Difficulty_level.THREE == 1)	//三级难度坐标系
			{
				custom_client_data.USE_DATA.key_value = 0;
				custom_client_data.USE_DATA.x_position = 333;
				custom_client_data.USE_DATA.y_position = 333;
				
				Data_Concatenation_2(custom_client_data.Send_data,BUY_AMMO_DATA_LENGTH);
			
				if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX) 
						HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_data2), sizeof(tx_data2));
			
				Buy_Ammo_count = 2;
			}
			else	Buy_Ammo_count = 8;
		}	break;
		case 2:
		{
			custom_client_data.USE_DATA.mouse_left = 1;
			
			Data_Concatenation_2(custom_client_data.Send_data,BUY_AMMO_DATA_LENGTH);
			
			if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX) 
					HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_data2), sizeof(tx_data2));
			
			Buy_Ammo_count = 3;
		}	break;
		case 3:
		{
			custom_client_data.USE_DATA.mouse_left = 0;
			
			Data_Concatenation_2(custom_client_data.Send_data,BUY_AMMO_DATA_LENGTH);
			
			if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX) 
					HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_data2), sizeof(tx_data2));
			
			Buy_Ammo_count = 4;
		}	break;
		case 4:
		{
			//确认的坐标值，记得修改
			custom_client_data.USE_DATA.x_position = 555;
			custom_client_data.USE_DATA.y_position = 555;

			Data_Concatenation_2(custom_client_data.Send_data,BUY_AMMO_DATA_LENGTH);

			if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX) 
					HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_data2), sizeof(tx_data2));
			
			Buy_Ammo_count = 5;
		}	break;
		case 5:
		{
			custom_client_data.USE_DATA.mouse_left = 1;

			Data_Concatenation_2(custom_client_data.Send_data,BUY_AMMO_DATA_LENGTH);

			if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX) 
					HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_data2), sizeof(tx_data2));
			
			Buy_Ammo_count = 6;
		}	break;
		case 6:
		{
			custom_client_data.USE_DATA.mouse_left = 0;

			Data_Concatenation_2(custom_client_data.Send_data,BUY_AMMO_DATA_LENGTH);

			if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX) 
					HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_data2), sizeof(tx_data2));
			
			Buy_Ammo_count = 7;
		}	break;
		case 7:
		{
			custom_client_data.USE_DATA.key_value = 79;
			custom_client_data.USE_DATA.mouse_left = 0;
			custom_client_data.USE_DATA.mouse_right = 0;
			custom_client_data.USE_DATA.x_position = 0;
			custom_client_data.USE_DATA.y_position = 0;

			Data_Concatenation_2(custom_client_data.Send_data,BUY_AMMO_DATA_LENGTH);

			if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX) 
					HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_data2), sizeof(tx_data2));
			
			Buy_Ammo_count = 8;
		}	break;
		case 8:
		{
			Buy_Ammo_count = 0;//购买完成，等待下一次购买
			
			Difficulty_level.ONE = 0;
			Difficulty_level.TWO = 0;
			Difficulty_level.THREE = 0;
			
			vTaskSuspend(NULL);
		}	break;
	}
}



/***@role: DMA中断回调函数，清除繁忙状态的***/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance) 
  {
    // 确保状态被正确重置
    if(huart->gState == HAL_UART_STATE_BUSY_TX)
		{
			huart->gState = HAL_UART_STATE_READY;
    }
  }
}


