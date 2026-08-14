#include "Get_remote_data.h"
#include "Motor_control.h"
#include "can.h"
#include "can_bsp.h"

/***@role:	获取执行机构C板的遥控指令***/
Instruction_C_Data C_Date = {0};
void Get_Control_data(Instruction_C_Data *c_data)
{
	union
	{
		struct
		{
			int16_t x_t;
			int16_t y_t;
			int16_t turn_t;
			uint8_t Chassis_mode;
			uint8_t stop_t:1;
			uint8_t Quick_storage_sate:1;	//预留数据位
			uint8_t erro:1;
			uint8_t V:1;
			uint8_t SRL:1;	//自稳标志位
			uint8_t C:3;
		}Specific_data_t;
		uint8_t DATA_RX[8];
	}C_to_C_rx_data;
	
	for (uint8_t i = 0; i < 8; i++)
	{
		C_to_C_rx_data.DATA_RX[i] = RX_data[i];
	}

	if(C_to_C_rx_data.Specific_data_t.erro == 1)
	{
		c_data->Stop = 5;
		__disable_irq();
		NVIC_SystemReset();
	}
	
	c_data->VX = (C_to_C_rx_data.Specific_data_t.x_t / 2.5f);													//x方向的速度
	c_data->VY = (C_to_C_rx_data.Specific_data_t.y_t / 2.5f);													//y方向的速度
	c_data->turn_speed = (C_to_C_rx_data.Specific_data_t.turn_t);											//yaw的设定值
	c_data->Stop = C_to_C_rx_data.Specific_data_t.stop_t;															//一键底盘断电标志位
	c_data->Chassis_mode = C_to_C_rx_data.Specific_data_t.Chassis_mode;								//底盘模式标志位
	c_data->Quick_storage_sate = C_to_C_rx_data.Specific_data_t.Quick_storage_sate;		//一件存取能量单元标志位
	c_data->v = C_to_C_rx_data.Specific_data_t.V;
	c_data->F = C_to_C_rx_data.Specific_data_t.SRL;
	c_data->C = C_to_C_rx_data.Specific_data_t.C;
}

ui_t_data_t ui_t_data = {0};
void Get_ui_data(ui_t_data_t *c_data)
{
	union
	{
		struct
		{
			uint8_t yaw_t;
			uint8_t Big_pitch_t;
			uint8_t Smoll_pitch_t;
			uint8_t Big_roll_t;
			uint8_t End_pitch_t;
			uint8_t End_roll_t;
			uint8_t claw_t;
			uint8_t data0:1;
			uint8_t data1:1;
			uint8_t data2:1;
			uint8_t data3:1;
			uint8_t data4:1;
			uint8_t data5:1;
			uint8_t data6:1;
			uint8_t data7:1;
		}Specific_data_t;
		uint8_t DATA_RX[8];
	}ui_rx_data;
	
	for (uint8_t i = 0; i < 8; i++)
	{
		ui_rx_data.DATA_RX[i] = RX_data[i];
	}
	
	c_data->yaw_t = ui_rx_data.Specific_data_t.yaw_t;
	c_data->Big_pitch_t = ui_rx_data.Specific_data_t.Big_pitch_t;
	c_data->Smoll_pitch_t = ui_rx_data.Specific_data_t.Smoll_pitch_t;
	c_data->Big_roll_t = ui_rx_data.Specific_data_t.Big_roll_t;
	c_data->End_pitch_t = ui_rx_data.Specific_data_t.End_pitch_t;
	c_data->End_roll_t = ui_rx_data.Specific_data_t.End_roll_t;
	c_data->claw_t = ui_rx_data.Specific_data_t.claw_t;
	c_data->data0 = ui_rx_data.Specific_data_t.data0;
	c_data->data1 = ui_rx_data.Specific_data_t.data1;
	c_data->data2 = ui_rx_data.Specific_data_t.data2;
	c_data->data3 = ui_rx_data.Specific_data_t.data3;
	c_data->data4 = ui_rx_data.Specific_data_t.data4;
	c_data->data5 = ui_rx_data.Specific_data_t.data5;
	c_data->data6 = ui_rx_data.Specific_data_t.data6;
}

Turn_C_Data_t Turn_C_Date = {0};
void Get_Turn_data(Turn_C_Data_t *turn_c_data)
{
	union
	{
		struct
		{
			float turn_data;
		}Specific_data_t;
		uint8_t DATA_RX[4];
	}C_to_C_rx_data;
	
	for (uint8_t i = 0; i < 4; i++)
	{
		C_to_C_rx_data.DATA_RX[i] = RX_data[i];
	}
	
	turn_c_data->turn_data = (C_to_C_rx_data.Specific_data_t.turn_data * 0.5f);													//x方向的速度
}


