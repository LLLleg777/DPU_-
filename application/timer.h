#ifndef _TIMER_H
#define _TIMER_H
#include "system.h"

void delay_us(uint16_t nus);
void delay_ms(uint16_t nms);
void TIM2_int(u8 mode);
void TIM6_int(u8 mode);
void TIM3_int(u8 mode);

extern int TIM2_mode, TIM6_mode, TIM7_mode, TIM3_mode;
extern int t1, t_1, t2, t_2, t_Accel, Accelt, t_text, textt;
extern int Time_chang_zero;


#endif


