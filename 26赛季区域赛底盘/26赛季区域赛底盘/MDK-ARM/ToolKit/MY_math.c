#include "MY_math.h"
#include "arm_math.h"

/***@role:	求绝对值的函数***/
float MY_ABS(float a)
{
	if(a<0) a=-a;
	return a;
}

/***@role:	求正负的函数***/
float Pos_or_Neg(float x)
{
	if(x>0)	
		return 1;
	else if(x<0)
		return -1;
	else
		return 0;
}

/***@role: 快速开根号***/
float InvSqrt(float x)
{
 if(x==0){return 0;}
 float xhalf = 0.5f*x;
 int32_t i = *(int32_t*)&x;        // get bits for floating VALUE
 i = 0x5f375a86- (i>>1);           // gives initial guess y0
 x = *(float*)&i;                  // convert bits BACK to float
 x = x*(1.5f-xhalf*x*x);           // Newton step, repeating increases accuracy
 x = 1 / x;
 return x;
}

//===================以下为斜坡重力补偿的3X3旋转矩阵数学算法函数============================================================

/***@role:	将欧拉角（偏航yaw、俯仰pitch、滚转roll）组合成一个3×3的旋转矩阵***/
void slope_turn_build_chassis_C(float yaw_rad, float pitch_rad, float roll_rad, float out_3x3[9])
{
	float Y[9], P[9], R[9];
	float YP[9];
	
	slope_turn_build_yaw(yaw_rad, Y);
	slope_turn_build_pitch(pitch_rad, P);
	slope_turn_build_roll(roll_rad, R);
	
	arm_matrix_instance_f32 mY, mP, mR, mYP, mC;
	
	arm_mat_init_f32(&mY,  3, 3, Y);
	arm_mat_init_f32(&mP,  3, 3, P);
	arm_mat_init_f32(&mR,  3, 3, R);
	arm_mat_init_f32(&mYP, 3, 3, YP);
	arm_mat_init_f32(&mC,  3, 3, out_3x3);
	
	/* YP = Y * P */
	(void)arm_mat_mult_f32(&mY, &mP, &mYP);
	
	/* C = (Y * P) * R */
	(void)arm_mat_mult_f32(&mYP, &mR, &mC);
}

/***@role:	yaw轴旋转矩阵***/
void slope_turn_build_yaw(float yaw_rad, float out_3x3[9])
{
    const float c = cosf(yaw_rad);
    const float s = sinf(yaw_rad);

    /* Rz(yaw) */
    M00(out_3x3) =  c;   M01(out_3x3) = -s;   M02(out_3x3) = 0.0f;
    M10(out_3x3) =  s;   M11(out_3x3) =  c;   M12(out_3x3) = 0.0f;
    M20(out_3x3) = 0.0f; M21(out_3x3) = 0.0f; M22(out_3x3) = 1.0f;
}

/***@role:	pitch轴旋转矩阵***/
void slope_turn_build_pitch(float pitch_rad, float out_3x3[9])
{
    const float c = cosf(pitch_rad);
    const float s = sinf(pitch_rad);

    /* Ry(pitch) */
    M00(out_3x3) =  c;    M01(out_3x3) = 0.0f; M02(out_3x3) =  s;
    M10(out_3x3) = 0.0f; M11(out_3x3) = 1.0f; M12(out_3x3) = 0.0f;
    M20(out_3x3) = -s;   M21(out_3x3) = 0.0f; M22(out_3x3) =  c;
}

/***@role:	roll轴旋转矩阵***/
void slope_turn_build_roll(float roll_rad, float out_3x3[9])
{
    const float c = cosf(roll_rad);
    const float s = sinf(roll_rad);

    /* Rx(roll) */
    M00(out_3x3) = 1.0f; M01(out_3x3) = 0.0f; M02(out_3x3) = 0.0f;
    M10(out_3x3) = 0.0f; M11(out_3x3) =  c;   M12(out_3x3) = -s;
    M20(out_3x3) = 0.0f; M21(out_3x3) =  s;   M22(out_3x3) =  c;
}

float slope_turn_wrap_pi1(float a)
{
    while (a >  SLOPE_TURN_PI) a -= 2.0f * SLOPE_TURN_PI;
    while (a < -SLOPE_TURN_PI) a += 2.0f * SLOPE_TURN_PI;
    return a;
}
//=============================================================================================================================
