#include "ALL.h"

extern TIM_HandleTypeDef htim5;

void ALL_Init(void)
{
	can_bsp_init();																			// 其中包含开启CAN1通讯和CAN通讯过滤器的初始化
	uint8_t JiDao_WanSui;																// -----! ! ! 极 道 万 岁 ! ! !----- //
	JiDao_WanSui = BMI088_init();	                      // -----! ! ! 极 道 万 岁 ! ! !----- //
	while(JiDao_WanSui)	{JiDao_WanSui = BMI088_init();}	// -----! ! ! 极 道 万 岁 ! ! !----- //
	Read_Offset();																			// 从flash中读取零偏值
	//MY_UART3_DMA_Init(); 																// DMA,UART3,遥控器初始化（没有他无法接收遥控器数据）
	
	HAL_TIM_Base_Start(&htim5);
  //start pwm channel
  //开启PWM通道
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	
	BSP_USART_Init();
	
	PID_Init(&BigPitch_angle_pid,4.0f,0.0f,0.0f,0.0f,0.5f,5.0f ,12000.0f,1000.0f,12000.0f,1000.0f,12000.0f);
	PID_Init(&BigPitch_speed_pid,90.0f,0.0f,0.0f,0.0f,0.5f,5.0f,14000.0f,1000.0f,12000.0f,1000.0f,14000.0f);
	
	PID_Init(&SmollPitch_angle_pid,2.5f,0.0f,0.0f,0.0f,0.5f,5.0f ,12000.0f,1000.0f,12000.0f,1000.0f,12000.0f);
	PID_Init(&SmollPitch_speed_pid,70.0f,0.0f,0.0f,0.0f,0.5f,5.0f,14000.0f,1000.0f,12000.0f,1000.0f,14000.0f);
	
	PID_Init(&endPitch_angle_pid,4.0f,0.0f,0.0f,0.0f,0.5f,5.0f ,12000.0f,1000.0f,12000.0f,1000.0f,12000.0f);
	PID_Init(&endPitch_speed_pid,70.0f,0.0f,0.0f,0.0f,0.5f,5.0f,14000.0f,1000.0f,12000.0f,1000.0f,14000.0f);
}

//=================================代码始于二四级工程电控王健州==========================================//

									/* * * * * * * * * * * * * * * * * * * * * * * * * * */
									/*              代 码 无 BUG ! ! !               	   */
									/*																									 */
									/*        ┌─┐       ┌─┐ + +												 	 */
									/*     ┌──┘ ┴───────┘ ┴──┐++											 	 */
									/*     │                 │													 */
									/*     │       ───       │++ + + +							代   */
									/*     ███████───███████ │+											码   */
									/*     │                 │+											无   */
									/*     │      ─┴─        │											B  	 */
									/*     │                 │											U	   */
									/*     └───┐         ┌───┘											G	   */
									/*         │         │													!	   */
									/*         │         │   + +										!	   */
									/*         │         │													!	   */
									/*         │         └──────────────┐							   */
									/*         │                        │							 	 */
									/*         │                        ├─┐						 	 */
									/*         │                        ┌─┘						 	 */
									/*         │                        │							 	 */
									/*         └─┐  ┐  ┌───────┬──┐  ┌──┘  + + + +		 	 */
									/*           │ ─┤ ─┤       │ ─┤ ─┤								 	 */
									/*           └──┴──┘       └──┴──┘  + + + +				 	 */
									/*                																 	 */
									/*             代 码 无 BUG !	 ! !									 */
									/* * * * * * * * * * * * * * * * * * * * * * * * * * */

//==============================代码始于二四级工程电控王健州============================================//


