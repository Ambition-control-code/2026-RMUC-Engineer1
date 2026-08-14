#ifndef __FILTER_H__
#define __FILTER_H__

#include "stm32f4xx.h"                  // Device header

//滑动窗口滤波器
typedef struct
{
    uint8_t len;   			//窗口宽度
    uint8_t index; 			//当前指针
    float buf[50];			//最大窗口长度50
}WindowFilter_Parameter;

float WindowFilter(float inPut , WindowFilter_Parameter *Window_Struct);

#endif


