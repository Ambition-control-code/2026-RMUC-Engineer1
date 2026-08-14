#ifndef __KEY_H__
#define __KEY_H__

#include "stm32f4xx.h"               // Device header

typedef struct {
    uint8_t last_state;     // 上次物理状态
    uint32_t press_tick;    // 按下时刻的时间戳
} ButtonState_t;

extern ButtonState_t Key_Pin9;
extern ButtonState_t Key_Pin13;
extern ButtonState_t Key_Pin14;
extern ButtonState_t Key_Pin11;

uint8_t Button_Check_Event(uint8_t current_state, ButtonState_t *btn);

#endif

