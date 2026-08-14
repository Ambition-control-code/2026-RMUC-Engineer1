#ifndef __REMOTE_H__
#define __REMOTE_H__

/*包含头文件****begain！！！*/
#include "ALL.h"
/*包含头文件****end！！！****/

/*宏定义********begain!*/
#define DMA_RX_BUF_SIZE 18		//DMA接收缓冲区大小
#define keypad_K 16.5f
#define remote_K 12.5
#define limit 8192
#define ACC_STEP_NUM 1000
/*宏定义********end!****/

/***@struct： 结构体****/
typedef struct
{
	int16_t R_x;                     //右手x		右1684 左364 中1024  1684-1024 = 1024 - 364 = 660
	int16_t R_y;	                   //右手y    上1684 下364 中1024
	int16_t L_x;                     //左手x
	int16_t L_y;                     //右手y
	uint8_t sl;		                   //拨动左  上1 中3 下2（左拨杆）
	uint8_t sr;                      //右拨杆
	uint8_t sl_change_flag;			     //拨动左  上1 中3 下2     //拨动值异常检测标志
	uint8_t sr_change_flag;				   //拨动值异常检测标志
	uint8_t mr_change_flag;					 //拨动值异常检测标志
	uint8_t ml_change_flag;					 //拨动值异常检测标志
	int16_t mouse_x;	               //鼠标x轴
	int16_t mouse_y;                 //鼠标y轴
	int16_t mouse_z;                 //鼠标滚轮
	uint8_t mouse_l;	               //鼠标左键
	uint8_t mouse_r;                 //鼠标右键
	uint16_t key;		                 //键盘
	u8 W;                            //W键
	u8 S;                            //S键
	u8 A;                            //A键
	u8 D;                            //D键
	u8 Shift;                        //Shift键
	u8 Ctrl;                         //Ctrl键
	u8 Q;                            //Q键
	u8 E;                            //E键
	u8 R;                            //R键
	u8 F;                            //F键
	u8 G;                            //G键
	u8 Z;                            //Z键
	u8 X;                            //X键
	u8 C;                            //C键
	u8 V;                            //V键
	u8 B;                            //B键
	uint8_t last_sr;
	uint8_t last_sl;
	uint8_t last_mouse_l;
	uint8_t last_mouse_r;
	uint8_t last_R;
	uint16_t trigger;                //遥控侧边拨轮
}Remote_all;
extern Remote_all remote;
/***********************/

/***@role： 别处定义的变量***/
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern u8 dbus_recieve_1[36],dbus_recieve_2[36];
/****************************/

/***@role： 函数的声明*******/
void MY_UART3_DMA_Init(void);
u8 if_change(u8 flag,u8 last_flag);
void deal_dbus(u8 *RXBuff);
void remote_fifo(void);
void mecanum_Resolving(void);
void USART3_IRQHandler(void);
/****************************/

#endif


