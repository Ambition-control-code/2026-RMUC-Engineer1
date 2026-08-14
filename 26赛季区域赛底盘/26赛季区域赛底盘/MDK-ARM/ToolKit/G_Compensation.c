#include "G_Compensation.h"
#include "MY_math.h"
#include "math.h"

/***@role:	底盘斜坡重力补偿算法***/
void Ramp_G_Compensation(slope_turn_result_t* res,float yaw_c_rad ,float pitch_c_rad,
                                                  float roll_c_rad,float flat_eps)
{
	//结构体数据初始化
	res->alpha_s = 0.0f;
	res->beta_s  = 0.0f;
	res->delta   = 0.0f;
	res->valid   = 0;
	
	//构建旋转矩阵
	float C[9];
	slope_turn_build_chassis_C(yaw_c_rad, pitch_c_rad, roll_c_rad, C);
	
	/* c_z is 3rd column of C 构建Z轴方向向量 */
	const float czx = M02(C);
	const float czy = M12(C);
	const float czz = M22(C);
	
	//计算斜坡倾角
	const float h = sqrtf(czx * czx + czy * czy);
	/* beta_S = atan( -sqrt(c_zx^2+c_zy^2) / c_zz ) */
	res->beta_s = atan2f(-h, czz);
	
	/* alpha_S = atan2(c_zy, c_zx) is meaningful only when not near-flat */
	//计算斜坡方位角
	if (h > flat_eps)
	{
		res->alpha_s = atan2f(czy, czx);
		res->valid = 1;
	}
	else
	{
		res->alpha_s = 0.0f;
		res->valid = 0;
	}
	
	//计算相对方位角
	res->delta = slope_turn_wrap_pi1(yaw_c_rad - res->alpha_s);
}


