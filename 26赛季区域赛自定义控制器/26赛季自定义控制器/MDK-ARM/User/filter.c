//该文件用于存储各种各样的滤波算法。
#include "filter.h"

/***@role: 滑动窗口滤波函数
 ***@advantage： 稳定，易于理解，简单好用
 ***@disadvantage： 使响应变慢，对内存消耗也大（如果是小内存的MCU，C板这些应该不至于），初始化也慢
***/
float WindowFilter(float inPut , WindowFilter_Parameter *Window_Struct)
{
	float outPut = 0;
  
  Window_Struct -> buf[Window_Struct -> index ++] = inPut;
  
  if(Window_Struct -> index >= Window_Struct -> len)	{Window_Struct -> index = 0;}
      
  for(uint8_t i = 0;i < Window_Struct -> len;i ++)
  {
		outPut += Window_Struct -> buf[i];
  }
  
  outPut /= Window_Struct -> len;
  
  return outPut;
}


