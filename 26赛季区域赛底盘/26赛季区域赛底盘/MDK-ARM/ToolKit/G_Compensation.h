#ifndef __G_COMPENSATION_H__
#define __G_COMPENSATION_H__

#include "stm32f4xx.h"                  // Device header

typedef struct
{
    float alpha_s;   /**< inferred slope yaw (rad) 斜坡倾斜方向的水平方位角 */
    float beta_s;    /**< inferred slope pitch/grade (rad) 斜坡的陡峭程度 */
    float delta;     /**< chassis relative turn angle (rad) 车辆前进方向与斜坡倾斜方向的夹角 */
    uint8_t valid;   /**< 1 if inference is reliable, 0 if near-flat singular 平坦度阈值，防止奇异点 */
} slope_turn_result_t;

void Ramp_G_Compensation(slope_turn_result_t* res,float yaw_c_rad ,float pitch_c_rad,
                                                  float roll_c_rad,float flat_eps);

#endif


