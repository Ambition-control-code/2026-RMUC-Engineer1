#ifndef __ALL_H__
#define __ALL_H__

//*一部分define*//
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GRYO_Pin GPIO_PIN_5
#define INT1_GRYO_GPIO_Port GPIOC
#define INT1_GRYO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
//*一部分define*//

//*所有的include头文件*//
#include "stm32f4xx.h"                  // Device header
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"  // 包含标准整数类型定义
#include "math.h"
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "can_bsp.h"
#include "can_driver.h"
#include "Motor_control.h"
#include "PID.h"
#include "stdarg.h"
#include "struct_typedef.h"
#include "BMI088reg.h"
#include "MY_AHRS.h"
#include "AHRS_middle.h"
#include "BMI088Middleware.h"
#include "BMI088driver.h"
#include "remote.h"
#include "IMU.h"
#include "flash.h"
#include "CRC.h"
#include "filter.h"
#include "Send_MyData.h"
#include "Switch_task.h"
#include "Freertos_task.h"
#include "Get_Referee_Data.h"
#include "get_arm_data.h"
#include "bits_change.h"
#include "uart_bsp.h"
#include "GC.h"
#include "Key.h"
#include "RGB_light.h"
#include "vofa.h"
//*所有的include头文件*//

void ALL_Init(void);


#endif


