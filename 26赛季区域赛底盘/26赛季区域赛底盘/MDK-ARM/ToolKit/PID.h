#ifndef __PID_H__
#define __PID_H__

/******begain！！！*********结构体*************/
typedef struct
{
	float kp;              //比例项
	float ki;              //积分项
	float kd;              //微分项
	float kf;							 //前馈项
	
	float pout;            //比例项输出值
	float iout;            //积分项输出值
	float dout;            //微分项输出值
	float fout;						 //前馈项输出值
	
	float poutmax;         // 比例项输出最大值（限幅）
	float ioutmax;         // 积分项输出最大值（限幅）
	float doutmax;         // 微分项输出最大值（限幅）
	float foutmax;				 // 前馈项输出最大值（限幅）
	float outmax;          //PID计算输出最大值（限幅）
	
	float set;             //目标值
	float set_last;				 //上次的目标
	float real;            //真实值
	float dead;						 //死区
	float out;             //PID输出值
	
	float err;						 //误差
	float err_last;			 	 //上次误差
	float integral;				 //对误差进行积分（累加）
	float i_threshold;		 //积分分离阈值
	
	float error[3];		//上次，上上次，和当前误差存储的数组，用于增量式PID
}PID_T;
extern PID_T L_Front_pid;
extern PID_T R_Front_pid;
extern PID_T L_Behind_pid;
extern PID_T R_Behind_pid;

extern PID_T pid_yaw_Angle;

extern PID_T pid_L_track;
extern PID_T pid_R_track;

typedef struct
{
	float kp;              //比例项
	float kd;              //微分项
	
	float pout;            //比例项输出值
	float dout;            //微分项输出值
	float fout;						 //前馈项输出值
	
	float outmax;          //PID计算输出最大值（限幅）
	float outmin;					 //PID计算输出最小值（限幅）
	
	float set;             //目标值
	float real;            //真实值
	float dead;						 //死区
	float out;             //PID输出值
	
	float err;						 //误差
}PD_T;
extern PD_T L_PD;
extern PD_T R_PD;
/******end！！！***********结构体**************/

/***********函数声明****begain！！！******/
void PID_Init(PID_T *pid,
	            float kp,float ki,
							float kd,float kf,
							float dead,float i_threshold,
	            float poutmax,float ioutmax,
							float doutmax,float foutmax,
	            float outmax);
float PID_Cal(PID_T *pid, float get, float set);
float PID_Incremental(PID_T* Pid,float set ,float real);
void PID_Clear(PID_T* PID);
float Slope_Buffer(float Set_Angle,float real_Angle,float Turn_Speed);
void PD_Init(PD_T *pd,float PD_kp,float PD_kd,float PD_dead,float fout,float outmax,float outmin);
float PD_Cal(PD_T *pd,float set_pos,float get_pos,float set_vel,float get_vel);
/***********函数声明****end！！！*********/

#endif


