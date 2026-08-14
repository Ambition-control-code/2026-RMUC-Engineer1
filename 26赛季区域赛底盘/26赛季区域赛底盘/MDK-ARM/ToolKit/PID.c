#include "PID.h"
#include "math.h"

/***@role: 结构体再命名***/
PID_T L_Front_pid = {0};          //左前轮速度环PID
PID_T R_Front_pid = {0};          //右前轮速度环PID
PID_T L_Behind_pid = {0};         //左后轮速度环PID
PID_T R_Behind_pid = {0};         //右后轮速度环PID

PID_T pid_yaw_Angle = {0};        //Yaw轴PID角度环

PID_T pid_L_track = {0};
PID_T pid_R_track = {0};
/*************************/

/*****@role： PID的初始化函数*****/
void PID_Init(PID_T *pid,
	            float kp,float ki,												 //比例项，积分项
							float kd,float kf,       									 //微分项，前馈项
							float dead,float i_threshold,							 //死区，积分分离阈值
	            float poutmax,float ioutmax,							 //比例项输出最大值，积分项输出最大值
							float doutmax,float foutmax,							 //微分项输出最大值，前馈项输出最大值
	            float outmax)                              //PID计算输出最大值（限幅）
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->kf = kf;
	pid->dead = dead;
	pid->i_threshold = i_threshold;
	
	pid->poutmax = poutmax;
	pid->ioutmax = ioutmax;
	pid->doutmax = doutmax;
	pid->foutmax = foutmax;
	pid->outmax = outmax;
	
	pid->err = 0;
	pid->integral = 0;
	pid->err_last = 0;
	pid->set_last = 0;
}

/***@role：   常规PID的计算函数***/
float PID_Cal(PID_T *pid, float get, float set)  											// get为实际值  set为目标值
{
	pid->set  = set;																										//设定值
	pid->real = get;																										//实际值
	pid->err = pid->set - pid->real;                                    //计算当前误差 = 设定值 - 实际值
	
	if(fabs(pid->err) < pid->dead)	pid -> err = 0;											//死区限制，当误差在死区以内时则认为没有误差
	
	//====================================比例项=============================================================
	pid->pout = pid->kp * pid->err;                                     //比例项输出 = Kp * 当前误差
	/*通过三元运算符实现双限幅，防止输出超限*/
	pid->pout = pid->pout <  pid->poutmax ? pid->pout :  pid->poutmax;  //上限幅
	pid->pout = pid->pout > -pid->poutmax ? pid->pout : -pid->poutmax;  //下限幅
	
	//================================积分项(积分分离)========================================================
	if(fabs(pid->err) < pid->i_threshold)
	{
		pid->integral += pid->err;                                         //对误差进行积分（累加）
		
		pid->iout = pid->ki * pid->integral;                               //积分项输出 = Ki * 误差积分
		/*通过三元运算符实现双限幅，防止积分饱和*/
		pid->iout = pid->iout <  pid->ioutmax ? pid->iout :  pid->ioutmax; //上限幅
		pid->iout = pid->iout > -pid->ioutmax ? pid->iout : -pid->ioutmax; //下限幅
	}
	else
	{
		pid->integral = 0;																								 // 误差大于阈值:远离目标值,清零积分(或保持不变)
		pid->iout = 0;
	}
	
	//====================================微分项=============================================================
	pid->dout = pid->kd * (pid->err - pid->err_last);                    //微分项输出=Kd*(当前误差-上次误差)
	/*通过三元运算符实现双限幅，抑制微分突变*/
	pid->dout = pid->dout <  pid->doutmax ? pid->dout :  pid->doutmax;   //上限幅
	pid->dout = pid->dout > -pid->doutmax ? pid->dout : -pid->doutmax;   //下限幅
	
	//====================================前馈项=============================================================
	pid->fout = pid->kf * (pid->set - pid->set_last);										 //前馈项输出=kf*(当前设定-上次设定)
	/*通过三元运算符实现双限幅，抑制前馈突变*/
	pid->fout = pid->fout <  pid->foutmax ? pid->fout :  pid->foutmax;   //上限幅
	pid->fout = pid->fout > -pid->foutmax ? pid->fout : -pid->foutmax;   //下限幅
	
	//====================================输出项=============================================================
	/*合成输出与总限幅*/
	pid->out = pid->pout + pid->iout + pid->dout + pid->fout;            //总输出 = P + I + D + F
	/*对总输出进行限幅，确保输出在安全范围内*/
	pid->out = pid->out <  pid->outmax ? pid->out :  pid->outmax;        //上限幅      
	pid->out = pid->out > -pid->outmax ? pid->out : -pid->outmax;        //下限幅
	
	//==================================更新历史量============================================================
	pid->err_last = pid->err;                                            //保存上一次误差值
	pid->set_last = pid->set;																						 //保存上一次设定值
	
	//==================================返回输出值============================================================
	return pid->out;                                                     //返回PID计算值
}

/***@role：	增量式PID的计算函数****/
float  PID_Incremental(PID_T* Pid,float set ,float real)
{
	Pid->set = set;
	Pid->real = real;
	Pid->error[0] = Pid->set - Pid->real;
	
	Pid->pout = Pid->kp * ( Pid->error[0] - Pid->error[1] );
	Pid->pout >  Pid->poutmax ? Pid->pout =  Pid->poutmax : 1;
	Pid->pout < -Pid->poutmax ? Pid->pout = -Pid->poutmax : 1;
	
	Pid->iout =	Pid->ki * Pid->error[0];
	Pid->iout >  Pid->ioutmax ? Pid->iout =  Pid->ioutmax : 1;
	Pid->iout < -Pid->ioutmax ? Pid->iout = -Pid->ioutmax : 1;

	Pid->dout =	Pid->kd * ( Pid->error[0] - 2 * Pid->error[1] + Pid->error[2] );
	Pid->dout >  Pid->doutmax ? Pid->dout =  Pid->doutmax : 1;
	Pid->dout < -Pid->doutmax ? Pid->dout = -Pid->doutmax : 1;

	Pid->error[2] = Pid->error[1];
	Pid->error[1] = Pid->error[0];
	
	Pid->out += Pid->pout + Pid->iout + Pid->dout;
	Pid->out >  Pid->outmax ? Pid->out =  Pid->outmax : 1;
	Pid->out < -Pid->outmax ? Pid->out = -Pid->outmax : 1;

	return Pid->out;	
}

/***@role:	PID清楚计算器函数***/
void PID_Clear(PID_T* PID)
{
	PID->err = 0;
	PID->integral = 0;
	PID->err_last = 0;
	PID->set_last = 0;
}

/***@role: 斜坡缓冲函数,本人自己写的逻辑,使用的if可能不优雅，也有别的实现方式，好使就行***/
float Slope_Buffer(float Set_Angle,float real_Angle,float Turn_Speed)
{
	float Change_Angle = Set_Angle;
	float Ramp_step = ((Set_Angle - real_Angle) / Turn_Speed);
	
	if(fabs(Ramp_step) < 10.0f)	
	{
		if((Set_Angle - real_Angle) > 0.0f)	Ramp_step = ( 10.0f);
		if((Set_Angle - real_Angle) < 0.0f)	Ramp_step = (-10.0f);
	}
	if(fabs(real_Angle - Set_Angle) > 10.0f)
	{
		if(Set_Angle > real_Angle)
		{
			Change_Angle = (real_Angle + Ramp_step);
			if(Change_Angle > Set_Angle)
			{
				Change_Angle = Set_Angle;
			}
		}
		else if(Set_Angle < real_Angle)
		{
			Change_Angle = (real_Angle + Ramp_step);
			if(Change_Angle < Set_Angle)
			{
				Change_Angle = Set_Angle;
			}
		}
	}
	else	Change_Angle = Set_Angle;
	return Change_Angle;
}

/***@role：	PD阻抗控制器***/
PD_T L_PD = {0};
PD_T R_PD = {0};

void PD_Init(PD_T *pd,float PD_kp,float PD_kd,float PD_dead,float fout,float outmax,float outmin)
{
	pd->kp = PD_kp;
	pd->kd = PD_kd;
	pd->dead = PD_dead;
	pd->fout = fout;
	pd->outmax = outmax;
	pd->outmin = outmin;
	
	pd->err = 0.0f;
	pd->pout = 0.0f;
	pd->dout = 0.0f;
	pd->out = 0.0f;
}

float PD_Cal(PD_T *pd,float set_pos,float get_pos,float set_vel,float get_vel)
{
  pd->set = set_pos;
  pd->real = get_pos;
  pd->err = pd->set - pd->real;
	
  if(fabsf(pd->err) < pd->dead)	pd->err = 0;

  pd->pout = pd->kp * pd->err;
  pd->dout = pd->kd * (set_vel - get_vel);

  pd->out = pd->pout + pd->dout + pd->fout;
	
  if(pd->out > pd->outmax)	pd->out = pd->outmax;
	if(pd->out < -pd->outmax) pd->out = -pd->outmax;
	
	return pd->out;
}


