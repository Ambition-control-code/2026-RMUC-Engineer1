//#include "DH_g.h"
//#include "arm_math.h"

//void gravity_compensation(float theta1_deg, float theta2_deg, float* current1, float* current2)
//{
//	float theta1 = (theta1_deg + ENCODER1_TO_PHYSICS_OFFSET_DEG) * DEG2RAD;
//	float theta2 = (theta2_deg + ENCODER2_TO_PHYSICS_OFFSET_DEG) * DEG2RAD;
//	
//	float cos_theta1, cos_theta1_theta2;
//	cos_theta1 = arm_cos_f32(theta1);
//	cos_theta1_theta2 = arm_cos_f32(theta1 + theta2);
//	
//	float tau1 = m1 * g * (L1/2.0f) * cos_theta1 + m2 * g * (L1 * cos_theta1 + (L2/2.0f) * cos_theta1_theta2);
//	float tau2 = m2 * g * (L2/2.0f) * cos_theta1_theta2;
//	
//	*current1 = tau1 / Kt;
//	*current2 = tau2 / Kt;
//}


