#include "Motor_control.h"
#include "ALL.h"
/******begain！！！！！！！！！！结构体重命名*********/
Motor_Feedback Big_Pitch_back = {0};			     //自控大臂Pitch轴3508屁股电机
Motor_Feedback Big_Roll_back = {0};			       //自控小臂Roll轴3508屁股电机
Motor_Feedback Smoll_Pitch_back = {0};				 //自控小Pitch轴3508电机反馈
Motor_Feedback Yaw_back = {0};				 				 //自控小Pitch轴3508电机反馈
Motor_Feedback End_Pitch_back = {0};					 //自控末端Pitch轴3508电机反馈
Motor_Feedback End_Roll_back = {0};						 //自控末端Roll轴3508电机反馈
/******end！！！！！！！！！！结构体重命名*********/

//uint8_t motor_Speed_data[8] = {0};//准备电机控制数据
/*****@role：      CAN通讯给电机发送指令的函数*****/
//void Motor_control(CAN_HandleTypeDef *hcan,uint16_t id,int16_t Speed,int esc_id)
//{
//	switch(esc_id)          //标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
//	{
//		case 0:								//初始化为零，清零所用。
//		{
//			for(uint8_t i = 0;i < 8;i ++)
//			{
//				motor_Speed_data[i] = 0;
//			}
//		}	break;
//		case 1:
//		case 5:motor_Speed_data[0] = Speed >> 8;motor_Speed_data[1] = Speed & 0xFF;break;
//		case 2:
//		case 6:motor_Speed_data[2] = Speed >> 8;motor_Speed_data[3] = Speed & 0xFF;break;
//		case 3:
//		case 7:motor_Speed_data[4] = Speed >> 8;motor_Speed_data[5] = Speed & 0xFF;break;
//		case 4:
//		case 8:motor_Speed_data[6] = Speed >> 8;motor_Speed_data[7] = Speed & 0xFF;break;
//	}
//	canx_bsp_send_data(hcan, id, motor_Speed_data, 8);
//}


/***
 *@role:      CAN通讯给电机发送指令的函数
 *@remeber:   达妙电机一拖四模式，大疆电机，本末电机都可以使用这个代码
 *@remeber:   达妙4310和4340找到了对应的一拖四固件，10010L没有找到
 *@remeber:   大疆电机6020的没有写，是因为工程不需要，这样分类写出来太多了，就写了这些，有需要可以接着写
 *@remeber:   这样写是防止电机多了之后数据变串，虽然比控制一部分写一个函数在别的任务中省事，
 *						但是在这个任务中明显需要写很多很多的东西去分辨，两种方法各有优劣吧。
***/
uint8_t dJi200_hcan1_data[8] = {0};        //准备大疆电机can1链路0x200的控制数据【3508】
uint8_t dJi200_hcan2_data[8] = {0};        //准备大疆电机can2链路0x200的控制数据【3508】
uint8_t dJi1FF_hcan1_data[8] = {0};        //准备大疆电机can1链路0x1FF的控制数据【3508】
uint8_t dJi1FF_hcan2_data[8] = {0};        //准备大疆电机can2链路0x1FF的控制数据【3508】
uint8_t dJi2FF_hcan1_data[8] = {0};        //准备大疆电机can2链路0x1FF的控制数据【6020】
uint8_t dJi2FF_hcan2_data[8] = {0};        //准备大疆电机can2链路0x1FF的控制数据【6020】
uint8_t dJi1FE_hcan1_data[8] = {0};        //准备大疆电机can1链路0x1FE的控制数据【6020】
uint8_t dJi1FE_hcan2_data[8] = {0};        //准备大疆电机can2链路0x1FE的控制数据【6020】
uint8_t dJi2FE_hcan1_data[8] = {0};        //准备大疆电机can1链路0x1FE的控制数据【6020】
uint8_t dJi2FE_hcan2_data[8] = {0};        //准备大疆电机can2链路0x1FE的控制数据【6020】
uint8_t BM032_hcan1_data[8]  = {0};        //准备本末电机can1链路0x032的控制数据
uint8_t BM032_hcan2_data[8]  = {0};        //准备本末电机can2链路0x032的控制数据
uint8_t BM033_hcan1_data[8]  = {0};        //准备本末电机can1链路0x033的控制数据
uint8_t BM033_hcan2_data[8]  = {0};        //准备本末电机can2链路0x033的控制数据
void Motor_control(CAN_HandleTypeDef *hcan,uint16_t Control_ID,int16_t Speed,int esc_id)
{
	if(hcan == &hcan1)
	{
		switch(Control_ID)
		{
			case 0x200:	
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 1:dJi200_hcan1_data[0] = Speed >> 8;dJi200_hcan1_data[1] = Speed & 0xFF;break;
					case 2:dJi200_hcan1_data[2] = Speed >> 8;dJi200_hcan1_data[3] = Speed & 0xFF;break;
					case 3:dJi200_hcan1_data[4] = Speed >> 8;dJi200_hcan1_data[5] = Speed & 0xFF;break;
					case 4:dJi200_hcan1_data[6] = Speed >> 8;dJi200_hcan1_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x1FF:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 5:dJi1FF_hcan1_data[0] = Speed >> 8;dJi1FF_hcan1_data[1] = Speed & 0xFF;break;
					case 6:dJi1FF_hcan1_data[2] = Speed >> 8;dJi1FF_hcan1_data[3] = Speed & 0xFF;break;
					case 7:dJi1FF_hcan1_data[4] = Speed >> 8;dJi1FF_hcan1_data[5] = Speed & 0xFF;break;
					case 8:dJi1FF_hcan1_data[6] = Speed >> 8;dJi1FF_hcan1_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x2FF:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 5:dJi2FF_hcan1_data[0] = Speed >> 8;dJi2FF_hcan1_data[1] = Speed & 0xFF;break;
					case 6:dJi2FF_hcan1_data[2] = Speed >> 8;dJi2FF_hcan1_data[3] = Speed & 0xFF;break;
					case 7:dJi2FF_hcan1_data[4] = Speed >> 8;dJi2FF_hcan1_data[5] = Speed & 0xFF;break;
					case 8:dJi2FF_hcan1_data[6] = Speed >> 8;dJi2FF_hcan1_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x032:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 1:BM032_hcan1_data[0] = Speed >> 8;BM032_hcan1_data[1] = Speed & 0xFF;break;
					case 2:BM032_hcan1_data[2] = Speed >> 8;BM032_hcan1_data[3] = Speed & 0xFF;break;
					case 3:BM032_hcan1_data[4] = Speed >> 8;BM032_hcan1_data[5] = Speed & 0xFF;break;
					case 4:BM032_hcan1_data[6] = Speed >> 8;BM032_hcan1_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x033:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 5:BM033_hcan1_data[0] = Speed >> 8;BM033_hcan1_data[1] = Speed & 0xFF;break;
					case 6:BM033_hcan1_data[2] = Speed >> 8;BM033_hcan1_data[3] = Speed & 0xFF;break;
					case 7:BM033_hcan1_data[4] = Speed >> 8;BM033_hcan1_data[5] = Speed & 0xFF;break;
					case 8:BM033_hcan1_data[6] = Speed >> 8;BM033_hcan1_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x1FE:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 1:dJi1FE_hcan1_data[0] = Speed >> 8;dJi1FE_hcan1_data[1] = Speed & 0xFF;break;
					case 2:dJi1FE_hcan1_data[2] = Speed >> 8;dJi1FE_hcan1_data[3] = Speed & 0xFF;break;
					case 3:dJi1FE_hcan1_data[4] = Speed >> 8;dJi1FE_hcan1_data[5] = Speed & 0xFF;break;
					case 4:dJi1FE_hcan1_data[6] = Speed >> 8;dJi1FE_hcan1_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x2FE:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 5:dJi2FE_hcan1_data[0] = Speed >> 8;dJi2FE_hcan1_data[1] = Speed & 0xFF;break;
					case 6:dJi2FE_hcan1_data[2] = Speed >> 8;dJi2FE_hcan1_data[3] = Speed & 0xFF;break;
					case 7:dJi2FE_hcan1_data[4] = Speed >> 8;dJi2FE_hcan1_data[5] = Speed & 0xFF;break;
				}
			}	break;
		}
	}
	else
	{
		switch(Control_ID)
		{
			case 0x200:	
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 1:dJi200_hcan2_data[0] = Speed >> 8;dJi200_hcan2_data[1] = Speed & 0xFF;break;
					case 2:dJi200_hcan2_data[2] = Speed >> 8;dJi200_hcan2_data[3] = Speed & 0xFF;break;
					case 3:dJi200_hcan2_data[4] = Speed >> 8;dJi200_hcan2_data[5] = Speed & 0xFF;break;
					case 4:dJi200_hcan2_data[6] = Speed >> 8;dJi200_hcan2_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x1FF:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 5:dJi1FF_hcan2_data[0] = Speed >> 8;dJi1FF_hcan2_data[1] = Speed & 0xFF;break;
					case 6:dJi1FF_hcan2_data[2] = Speed >> 8;dJi1FF_hcan2_data[3] = Speed & 0xFF;break;
					case 7:dJi1FF_hcan2_data[4] = Speed >> 8;dJi1FF_hcan2_data[5] = Speed & 0xFF;break;
					case 8:dJi1FF_hcan2_data[6] = Speed >> 8;dJi1FF_hcan2_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x2FF:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 5:dJi2FF_hcan2_data[0] = Speed >> 8;dJi2FF_hcan2_data[1] = Speed & 0xFF;break;
					case 6:dJi2FF_hcan2_data[2] = Speed >> 8;dJi2FF_hcan2_data[3] = Speed & 0xFF;break;
					case 7:dJi2FF_hcan2_data[4] = Speed >> 8;dJi2FF_hcan2_data[5] = Speed & 0xFF;break;
					case 8:dJi2FF_hcan2_data[6] = Speed >> 8;dJi2FF_hcan2_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x032:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 1:BM032_hcan2_data[0] = Speed >> 8;BM032_hcan2_data[1] = Speed & 0xFF;break;
					case 2:BM032_hcan2_data[2] = Speed >> 8;BM032_hcan2_data[3] = Speed & 0xFF;break;
					case 3:BM032_hcan2_data[4] = Speed >> 8;BM032_hcan2_data[5] = Speed & 0xFF;break;
					case 4:BM032_hcan2_data[6] = Speed >> 8;BM032_hcan2_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x033:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 5:BM033_hcan2_data[0] = Speed >> 8;BM033_hcan2_data[1] = Speed & 0xFF;break;
					case 6:BM033_hcan2_data[2] = Speed >> 8;BM033_hcan2_data[3] = Speed & 0xFF;break;
					case 7:BM033_hcan2_data[4] = Speed >> 8;BM033_hcan2_data[5] = Speed & 0xFF;break;
					case 8:BM033_hcan2_data[6] = Speed >> 8;BM033_hcan2_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x1FE:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 1:dJi1FE_hcan2_data[0] = Speed >> 8;dJi1FE_hcan2_data[1] = Speed & 0xFF;break;
					case 2:dJi1FE_hcan2_data[2] = Speed >> 8;dJi1FE_hcan2_data[3] = Speed & 0xFF;break;
					case 3:dJi1FE_hcan2_data[4] = Speed >> 8;dJi1FE_hcan2_data[5] = Speed & 0xFF;break;
					case 4:dJi1FE_hcan2_data[6] = Speed >> 8;dJi1FE_hcan2_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x2FE:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 5:dJi2FE_hcan2_data[0] = Speed >> 8;dJi2FE_hcan2_data[1] = Speed & 0xFF;break;
					case 6:dJi2FE_hcan2_data[2] = Speed >> 8;dJi2FE_hcan2_data[3] = Speed & 0xFF;break;
					case 7:dJi2FE_hcan2_data[4] = Speed >> 8;dJi2FE_hcan2_data[5] = Speed & 0xFF;break;
				}
			}	break;
		}
	}
}

/***
 *@role：    使能CAN通讯的函数
 *@remeber： 大疆电机3508的控制标识符ID号1~4是0x200，5~8是0x1FF
 *@remeber： 大疆6020的控制标识符电压模式ID号1~4是0x1FF，5~7是0x2FF，电流模式ID号是1~4是0x1FE，5~7是0x2FE
 *@remeber： 本末电机的控制标识符ID号1~4是0x032，5~8是0x033
***/
void Motor_enable(CAN_HandleTypeDef *hcan,uint16_t Control_ID,uint8_t lenth)
{
	if(hcan == &hcan1)
	{
		switch(Control_ID)
		{
			case 0x200:	canx_bsp_send_data(hcan, Control_ID, dJi200_hcan1_data, lenth);	break;
			case 0x1FF:	canx_bsp_send_data(hcan, Control_ID, dJi1FF_hcan1_data, lenth);	break;
			case 0x032: canx_bsp_send_data(hcan, Control_ID, BM032_hcan1_data , lenth);	break;
			case 0x033: canx_bsp_send_data(hcan, Control_ID, BM033_hcan1_data , lenth);	break;
			case 0x1FE:	canx_bsp_send_data(hcan, Control_ID, dJi1FE_hcan1_data, lenth);	break;
			case 0x2FF:	canx_bsp_send_data(hcan, Control_ID, dJi2FF_hcan1_data, lenth);	break;
			case 0x2FE:	canx_bsp_send_data(hcan, Control_ID, dJi2FE_hcan1_data, lenth);	break;
		}
	}
	else
	{
		switch(Control_ID)
		{
			case 0x200:	canx_bsp_send_data(hcan, Control_ID, dJi200_hcan2_data, lenth);	break;
			case 0x1FF:	canx_bsp_send_data(hcan, Control_ID, dJi1FF_hcan2_data, lenth);	break;
			case 0x032: canx_bsp_send_data(hcan, Control_ID, BM032_hcan2_data , lenth);	break;
			case 0x033: canx_bsp_send_data(hcan, Control_ID, BM033_hcan2_data , lenth);	break;
			case 0x1FE:	canx_bsp_send_data(hcan, Control_ID, dJi1FE_hcan2_data, lenth);	break;
			case 0x2FF:	canx_bsp_send_data(hcan, Control_ID, dJi2FF_hcan2_data, lenth);	break;
			case 0x2FE:	canx_bsp_send_data(hcan, Control_ID, dJi2FE_hcan2_data, lenth);	break;
		}
	}
}

uint8_t data[8] = {0};//数据缓存区
/*****@role：    CAN通讯接收3508电机的数据*****/
void get_Motor_Data(volatile Motor_Feedback *feedback)//volatile关键字确保编译器不对电机状态结构体做优化（因为会被中断修改）
{
	int32_t temp_sum = 0;                               //用于计算速度平均值的临时累加器
	feedback->last_raw_value = feedback->raw_value;     //保存上一次的编码器原始值（用于计算差值）
	feedback->raw_value = ((data[0]<<8) | data[1]);     //编码器的值(这是大疆的通讯协议)
	feedback->diff = feedback->raw_value - feedback->last_raw_value;//计算两次采样间的编码器变化量，范围：-8191 ~ +8191（14位编码器）
	if(feedback->diff < -4096)                          //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		feedback->round_cnt++;
		feedback->ecd_raw_rate = feedback->diff + 8192;
	}
	else if(feedback->diff>4096)
	{
		feedback->round_cnt--;
		feedback->ecd_raw_rate = feedback->diff - 8192;
	}		
	else	{feedback->ecd_raw_rate = feedback->diff;}
	//真实角度【0-360】
	feedback->Real_Angle = feedback->raw_value * 0.0439453125f;
	//计算得到连续的编码器输出值
	feedback->ecd_value = feedback->raw_value + feedback->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	feedback->ecd_angle = ((float)(feedback->raw_value - feedback->ecd_bias)*360/8192 + feedback->round_cnt * 360);
	feedback->rate_buf[feedback->buf_count++] = feedback->ecd_raw_rate;
	if(feedback->buf_count == RATE_BUF_SIZE)	{feedback->buf_count = 0;}
	
	//计算速度平均值
	for(int i = 0;i < RATE_BUF_SIZE; i++)	{temp_sum += feedback->rate_buf[i];}
	feedback->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);
	
	if(fabs((float)feedback->filter_rate_max) < fabs((float)feedback->filter_rate))
	{
		feedback->filter_rate_max = feedback->filter_rate;
	}
	feedback->current = (int16_t)(data[4] << 8 | data[5]);      //电流
	feedback->tempature = data[6];                              //温度
	feedback->re_speed= (int16_t)(data[2] << 8 | data[3]);      //速度用这个
}

void Chassis_Date_sent(int16_t x,int16_t y,int8_t accelerate,int8_t spin_mode,int8_t stop,int8_t other) //底盘发送
{
	uint8_t DATA_TX[8];
		
	DATA_TX[0] = (uint8_t)(x >> 8);
	DATA_TX[1] = (uint8_t)x;
	DATA_TX[2] = (uint8_t)(y >> 8);
	DATA_TX[3] = (uint8_t)y;
	
	DATA_TX[4] = (uint8_t)accelerate;
	DATA_TX[5] = (uint8_t)spin_mode;
	DATA_TX[6] = (uint8_t)stop;
	DATA_TX[7] = (uint8_t)other;
	
	canx_bsp_send_data(&hcan1,0x00a,DATA_TX,8);	//其中的ID是板子的ID值
	
}
