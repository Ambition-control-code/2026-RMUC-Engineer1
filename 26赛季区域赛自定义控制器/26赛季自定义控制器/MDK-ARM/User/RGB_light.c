#include "RGB_light.h"
#include "main.h"
extern TIM_HandleTypeDef htim5;
/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
/**
  * @brief          显示RGB
  * @param[in]      aRGB:0xaaRRGGBB,'aa' 是透明度,'RR'是红色,'GG'是绿色,'BB'是蓝色
  * @retval         none
  */
void aRGB_led_show(uint32_t aRGB)
{
		static uint8_t state = 0;
    static uint16_t red = 0,green = 0,blue = 0;
		static uint32_t now_time = 0,last_time = 0;
	
		now_time ++;
		
		if((now_time - last_time)>=5)
		{
			state ++;
			last_time = now_time;
		}
		
		switch(state)
		{
			case 0: // 鲜艳红色
            red   = 255;
            green = 0;
            blue  = 0;
            break;
     case 1: // 鲜艳绿色
            red   = 0;
            green = 255;
            blue  = 0;
            break;
		 case 2: // 鲜艳蓝色
            red   = 0;
            green = 0;
            blue  = 255;
            break;
		 case 3:
						state = 0;
						break;
		}
		
		uint8_t alpha = 255;

    red   = (uint16_t)(red * alpha);
    green = (uint16_t)(green * alpha);
    blue  = (uint16_t)(blue * alpha);

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}


