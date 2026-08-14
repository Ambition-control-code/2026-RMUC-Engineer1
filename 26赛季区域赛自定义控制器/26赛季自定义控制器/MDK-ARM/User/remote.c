#include "remote.h"

/*定义的变量***begain***/
u8 RXBuff[DMA_RX_BUF_SIZE];		//DMA接收缓冲区
u8 sendchassic[5];
u8 dbus_recieve_1[36] = {0},dbus_recieve_2[36] = {0};//双缓冲的DBUS接收数据（双缓冲接收机制）

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

Remote_all remote;
/*定义的变量***end*****/

void MY_UART3_DMA_Init(void)
{
	/***@important： 以下是DBUS接收重要的初始化工作，必不可少！！！*/
	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);                     //启用DMA接收请求
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                       //开启空闲中断
	__HAL_DMA_DISABLE(&hdma_usart3_rx);                                //失能DMA,解除保护
	while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)                   //循环检查，避免失能失败
	{
			__HAL_DMA_DISABLE(&hdma_usart3_rx);
	}
	__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx, DMA_LISR_TCIF1);             //清除DMA标志
	hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);          //配置DMA地址寄存器（核心配置）
	//以下两行是双缓存区，当缓冲区1满时自动切换到缓冲区2，同时CPU可处理缓冲区1的数据，避免数据覆盖，实现连续接收
	hdma_usart3_rx.Instance->M0AR = (uint32_t)(dbus_recieve_1);
	hdma_usart3_rx.Instance->M1AR = (uint32_t)(dbus_recieve_2);
	__HAL_DMA_SET_COUNTER(&hdma_usart3_rx, 36u);                       //设置传输长度（36个字节）
	SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);                //启用双缓冲模式
	__HAL_DMA_ENABLE(&hdma_usart3_rx);                                 //启用DMA
}

u8 if_change(u8 flag,u8 last_flag)//这里可能是消抖作用，从25赛季三号步兵扒下来的
{
if(flag!=last_flag) {return 1;}
else {return 0;}
}

/********************************/
/***@role：DBUS通讯协议**********/
/********************************/
void deal_dbus(u8 *RXBuff)
{
	if(((RXBuff[0]| (RXBuff[1] << 8)) & 0x07ff))
	{
		//右摇杆X通道 (通道0)
		remote.R_x = ((RXBuff[0]| (RXBuff[1] << 8)) & 0x07ff)-1024; //转换为-1024~1023
		// 同时将原始数据转发给底盘
		sendchassic[0]=RXBuff[0];
		sendchassic[1]=RXBuff[1];
	}
	if((((RXBuff[1] >> 3) | (RXBuff[2] << 5)) & 0x07ff))
	{
		//右摇杆Y通道 (通道1)
		remote.R_y = (((RXBuff[1] >> 3) | (RXBuff[2] << 5)) & 0x07ff)-1024;
		// 同时将原始数据转发给底盘
		sendchassic[2]=RXBuff[2];
	} //!< Channel 1
	if((((RXBuff[2] >> 6) | (RXBuff[3] << 2) | (RXBuff[4] << 10))& 0x07ff))
	{
		//左摇杆X通道（通道2）
		remote.L_x = (((RXBuff[2] >> 6) | (RXBuff[3] << 2) | (RXBuff[4] << 10))& 0x07ff)-1024;
	} //!< Channel 2
	if((((RXBuff[4] >> 1) | (RXBuff[5] << 7)) & 0x07ff))
	{
		//左摇杆Y通道（通道3）
		remote.L_y = (((RXBuff[4] >> 1) | (RXBuff[5] << 7)) & 0x07ff)-1024;
	} //!< Channel 3
	remote.sl = (((RXBuff[5] >> 4)& 0x000C) >> 2);    //!< Switch left// 左侧开关 (2位状态)
	remote.sr = (((RXBuff[5] >> 4)& 0x0003));         //!< Switch left // 右侧开关 (2位状态)
	int16_t temp;
	temp = 1024 - (RXBuff[16]|(RXBuff[17] << 8));     //遥控器侧面的拨轮
	//处理边界值
	if(temp>3000) {remote.trigger=65535-temp;}
	else {remote.trigger=temp;}
	//鼠标值解析
	remote.mouse_x = RXBuff[6] | (RXBuff[7]  << 8);   //!< Mouse X axis
	remote.mouse_y = RXBuff[8] | (RXBuff[9]  << 8);   //!< Mouse Y axis
	remote.mouse_z = RXBuff[10]| (RXBuff[11] << 8);   //!< Mouse Z axis//滚轮
	remote.mouse_l = RXBuff[12];                      //!< Mouse Left Is Press ?
	remote.mouse_r = RXBuff[13];                      //!< Mouse Right Is Press ?
	//键盘按键解析（位映射）
	remote.key = RXBuff[14] | (RXBuff[15] << 8);      //!< KeyBoard value
	remote.W = (remote.key&0x01)>>0;
	remote.S = (remote.key&0x02)>>1;
	remote.A = (remote.key&0x04)>>2;
	remote.D = (remote.key&0x08)>>3;
	remote.Shift = (remote.key&0x10)>>4;
	remote.Ctrl = (remote.key&0x20) >>5;
	remote.Q = (remote.key&0x40)  >>6 ;
	remote.E = (remote.key&0x80)  >>7 ;
	remote.R = (remote.key&0x100) >>8 ;
	remote.F = (remote.key&0x200) >>9 ;
	remote.G = (remote.key&0x400) >>10;
	remote.Z = (remote.key&0x800) >>11;
	remote.X = (remote.key&0x1000)>>12;
	remote.C = (remote.key&0x2000)>>13;
	remote.V = (remote.key&0x4000)>>14;
	remote.B = (remote.key&0x8000)>>15;
	//遥控器值抛出异常（数据有效性检验）
	remote.R_x = remote.R_x >  660 ? 0 : remote.R_x;   // 摇杆死区处理（超出±660置零）
	remote.R_x = remote.R_x < -660 ? 0 : remote.R_x;
	remote.R_y = remote.R_y >  660 ? 0 : remote.R_y;
	remote.R_y = remote.R_y < -660 ? 0 : remote.R_y;
	remote.L_x = remote.L_x >  660 ? 0 : remote.L_x;
	remote.L_x = remote.L_x < -660 ? 0 : remote.L_x;
	remote.L_y = remote.L_y >  660 ? 0 : remote.L_y;
	remote.L_y = remote.L_y < -660 ? 0 : remote.L_y;
	
	remote.mouse_x = remote.mouse_x >  1024 ? 0 : remote.mouse_x;
	remote.mouse_x = remote.mouse_x < -1024 ? 0 : remote.mouse_x;
	remote.mouse_y = remote.mouse_y >  1024 ? 0 : remote.mouse_y;
	remote.mouse_y = remote.mouse_y < -1024 ? 0 : remote.mouse_y;
	
	remote.sl_change_flag=if_change(remote.sl,remote.last_sl);
	remote.sr_change_flag=if_change(remote.sr,remote.last_sr);	
	remote.mr_change_flag=if_change(remote.mouse_r,remote.last_mouse_r);		
	remote.ml_change_flag=if_change(remote.mouse_l,remote.last_mouse_l);		
	
	remote.last_sl=remote.sl;
	remote.last_sr=remote.sr;
	remote.last_mouse_r=remote.mouse_r;
	remote.last_mouse_l=remote.mouse_l;
}

unsigned short AccStep[ACC_STEP_NUM] =      // 加速曲线数组
{
	0,3,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,6,6,6,6,6,6,7,7,7,7,7,8,8,8,8,9,9,9,9,10,10,10,
	11,11,11,12,12,12,13,13,13,14,14,15,15,16,16,16,17,17,18,18,19,20,20,21,21,22,23,23,
	24,25,25,26,27,28,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,46,47,48,49,51,
	52,54,55,57,58,60,61,63,64,66,68,70,72,73,75,77,79,81,83,85,88,90,92,94,97,99,101,104,
	106,109,111,114,117,119,122,125,128,131,133,136,139,142,146,149,152,155,158,162,165,168,
  171,175,178,182,185,189,192,196,199,203,207,210,214,218,221,225,229,233,236,240,244,248,
	251,255,259,262,266,270,274,277,281,285,288,292,296,299,303,307,310,314,317,321,324,327,
	331,334,337,341,344,347,350,353,357,360,363,366,368,371,374,377,380,382,385,388,390,393,
	395,398,400,403,405,407,410,412,414,416,418,420,422,424,426,428,430,431,433,435,437,438,
	440,441,443,444,446,447,449,450,451,453,454,455,456,458,459,460,461,462,463,464,465,466,
	467,468,469,470,471,471,472,473,474,474,475,476,477,477,478,478,479,480,480,481,481,482,
	482,483,483,484,484,485,485,486,486,486,487,487,488,488,488,489,489,489,490,490,490,490,
	491,491,491,492,492,492,492,492,493,493,493,493,494,494,494,494,494,494,495,495,495,495,
	495,495,495,496,496,496,496,496,496,496,496,497,497
};

float deal_W, deal_A, deal_S, deal_D ;			// 处理后的输出值
int t_W, t_A, t_S, t_D;											// 时间计数器
/********************************************************************/
/***@role： 按键加速控制*********************************************/
/***@objective： 解决机械按键的瞬时响应问题，实现平滑的加速/减速效果*/
/********************************************************************/
void remote_fifo(void)
{
	// 按键状态检测
	if(remote.W!=0 )t_W++; else t_W--;//W键
	if(remote.A!=0) t_A++; else t_A--;//A键
	if(remote.S!=0) t_S++; else t_S--;//S键
	if(remote.D!=0) t_D++; else t_D--;//D键

	// 计数器限幅
	t_W = t_W > 334 ? 334 : t_W;  t_W = t_W < 0 ? 0 : t_W;	
	t_A = t_A > 334 ? 334 : t_A;  t_A = t_A < 0 ? 0 : t_A;	
	t_S = t_S > 334 ? 334 : t_S;  t_S = t_S < 0 ? 0 : t_S;
	t_D = t_D > 334 ? 334 : t_D;  t_D = t_D < 0 ? 0 : t_D;

	// 映射到加速曲线
	deal_W=AccStep[t_W];
	deal_A=AccStep[t_A];
	deal_S=AccStep[t_S];
	deal_D=AccStep[t_D];
}

float rc_X, rc_Y;   //最终输出的底盘运动矢量分量
uint8_t spin = 3;   //旋转模式标志 (1=底盘旋转，3=云台跟随)
uint8_t stop=0;     //急停标志 (1=停止)
uint8_t Shift=0;    //超电/高速模式标志
uint8_t other=0;    //其他功能标志

/***@role: 处理识别遥控（或者键鼠）信息传给底盘***/
void mecanum_Resolving(void)
{                                                    //其中一位是所有控制的标志位集合
	if(remote.sr==1){spin = 1;}                        // 底盘旋转模式 (spin+)
	if(remote.sr==2||remote.sr==3){spin = 3;}          //云台跟随模式（gimbal）
	if(remote.sl == 2){stop = 1;}                      //下电停止
	else {stop = 0;}
	//if(rc.Shift ==1||rc.mouse_r==1||rc.sr==2)//超电       //调试
	//超电/高速模式 (鼠标右键触发)
	if(remote.mouse_r==1){Shift = 1;}          //超电      //调试     //高速模式
	else {Shift = 0;}
	if(remote.R==1){other = 1;}                //其他功能 (R键触发)
	else {other = 0;}
	float remote_x, remote_y, X, Y, aa;        //云台的坐标系
	float W = 0,A = 0,S = 0,D = 0;
	remote_fifo();                             // 调用按键加速处理函数
	// 将加速处理后的按键值乘以系数
	W = (int)(keypad_K * deal_W);              //前进
	A = (int)(keypad_K * deal_A);              //左移
	S = (int)(keypad_K * deal_S);              //后退
	D = (int)(keypad_K * deal_D);              //右移
	// 处理摇杆输入(乘以系数并取反)
	remote_x = -remote.R_x * remote_K;         //右摇杆X轴
	remote_y = -remote.R_y * remote_K;         //右摇杆Y轴
  /*下面四行代码需要根据实际情况进行修改！！！！！切记！切记！*/
	// 合成运动矢量 (键盘WASD + 右摇杆)
//	Y = W - S + remote_y;
//	X = -A + D + remote_x;
	Y = -W + S + remote_y;                     //前后方向
	X = +A - D + remote_x;                     //左右方向
//*上面四行代码需要根据实际情况进行修改！！！！！切记！切记！*//
	
	// 计算合成矢量的模长
	aa = sqrt(X*X+Y*Y);
	// 对速度矢量进行限幅 (防止超出最大值)
	X = aa <limit ? X : X*(limit/aa);
	X = aa >-limit ? X : X*(limit/aa);         //处理负方向
	Y = aa <limit ? Y : Y*(limit/aa);
	Y = aa >-limit ? Y : Y*(limit/aa);
	
	// 更新底盘控制输出
  Chassis_Date_sent(X,Y,Shift,spin,stop,other);   //这里发给底盘
}

/***@role: 能高效处理高频、小数据包的通信需求，确保不丢失任何数据帧***/
/***@why?: 1、无接收间隙 2、零拷贝处理 3、高实时性 4、防止数据覆盖 ***/
//串口中断
void USART3_IRQHandler(void)
{
	if(USART3->SR & UART_FLAG_IDLE)            //接收到数据(检测到空闲中断)
  {
		static uint16_t this_time_rx_len = 0;
    __HAL_UART_CLEAR_PEFLAG(&huart3);        //清除空闲中断标志
    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
    {
			/* 使用内存缓冲区0的情况 */
			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart3_rx);
	
			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = 36 - hdma_usart3_rx.Instance->NDTR;
	
			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart3_rx.Instance->NDTR = 36;
	
			//set memory buffer 1
			//设定缓冲区1
			hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
					
			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart3_rx);
	
			if(this_time_rx_len == 18)
			{
				HAL_UART_Receive_DMA(&huart3, (uint8_t*)dbus_recieve_1, 18);                                               
				deal_dbus(dbus_recieve_1);
			}
			else
      {
				/**
				*@注意： 
				*UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
				*portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
				*以上两行代码是用于中断内的保护，可以完美执行这两行代码之间的代码
				
				//taskENTER_CRITICAL();	
				//taskEXIT_CRITICAL();
				*这两行代码是用于任务中的，可以完美执行这两行的代码，防止被中断打断，被更高优先级的任务打断
				***/
				
				UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
				//taskENTER_CRITICAL();	
				HAL_UART_DeInit(&huart3);
				HAL_UART_Init(&huart3);
				MY_UART3_DMA_Init();
				//taskEXIT_CRITICAL();
				portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
			}
    }
    else
    {
			/* 使用内存缓冲区1的情况 1 */
			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart3_rx);

      //get receive data length, length = set_data_length - remain_length
      //获取接收数据长度,长度 = 设定长度 - 剩余长度
      this_time_rx_len = 36 - hdma_usart3_rx.Instance->NDTR;

      //reset set_data_lenght
      //重新设定数据长度
      hdma_usart3_rx.Instance->NDTR = 36;

      //set memory buffer 0
      //设定缓冲区0
      DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
			
      //enable DMA
      //使能DMA
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      if(this_time_rx_len == 18)
      {
				//处理接收到的数据
				HAL_UART_Receive_DMA(&huart3, (uint8_t*)dbus_recieve_2, 18);                                               
				deal_dbus(dbus_recieve_2);
      }
			else
      {
				/**
				*@注意： 
				*UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
				*portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
				*以上两行代码是用于中断内的保护，可以完美执行这两行代码之间的代码
				
				//taskENTER_CRITICAL();	
				//taskEXIT_CRITICAL();
				*这两行代码是用于任务中的，可以完美执行这两行的代码，防止被中断打断，被更高优先级的任务打断
				***/
				
				UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
				//taskENTER_CRITICAL();	
				HAL_UART_DeInit(&huart3);
				HAL_UART_Init(&huart3);
				MY_UART3_DMA_Init();
				//taskEXIT_CRITICAL();
				portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
			}
    }
  }
}


