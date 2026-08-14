#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#define RATE_BUF_SIZE 6

/*包含头文件****begain！！！*/
#include "ALL.h"
/*包含头文件******end！！！**/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

extern int8_t motor_id;
extern uint8_t motor_Speed_data[8];
extern uint8_t data[8];
//extern uint16_t rec_id;
extern uint16_t rec_id;
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/******begain！！！*********结构体*************/

typedef struct{
	int32_t raw_value;   					 //编码器不经处理的原始值
	int32_t last_raw_value;				 //上一次的编码器原始值
	int32_t ecd_value;             //经过处理后连续的编码器值
	int32_t diff;							     //两次编码器之间的差值
	int32_t temp_count;            //计数用
	uint8_t buf_count;						 //滤波更新buf用
	int32_t ecd_bias;						   //初始编码器值	
	int32_t ecd_raw_rate;					 //通过编码器计算得到的速度原始值
	int32_t rate_buf[6];	         //buf，for filter（滤波用的）
	int32_t round_cnt;						 //圈数
	int32_t filter_rate;					 //速度（计算出来的，不要用，有误差，实时性不好）
	int32_t filter_rate_max;			 //最大速度
	float ecd_angle;						   //角度
	int16_t current;               //电流
	int16_t re_speed;              //电调速度
	int8_t tempature;	             //温度
	float Real_Angle;							 //真实角度
}Motor_Feedback;
extern Motor_Feedback Big_Pitch_back;
extern Motor_Feedback Big_Roll_back;
extern Motor_Feedback Smoll_Pitch_back;
extern Motor_Feedback End_Pitch_back;
extern Motor_Feedback End_Roll_back;
extern Motor_Feedback Yaw_back;
/******end！！！***********结构体***********************/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/***********函数声明****begain！！！******/

void Motor_control(CAN_HandleTypeDef *hcan,uint16_t Control_ID,int16_t Speed,int esc_id);
void Motor_enable(CAN_HandleTypeDef *hcan,uint16_t Control_ID,uint8_t lenth);
void get_Motor_Data(volatile Motor_Feedback *feedback);
void Chassis_Date_sent(int16_t x,int16_t y,int8_t accelerate,int8_t spin_mode,int8_t stop,int8_t other);

/***********函数声明****end！！！*********/






#endif




