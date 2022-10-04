#ifndef __PROTECT_H
#define __PROTECT_H	 
#include "system.h"

void TIME_Change(void);
void Remove_Flag(void);
u8 Project_Temperature(void);
u8 Project_NUM(void);
void Data_Processing(u8 lcd_flag);
void JY_YAW_Handle(void);
void NF24G_Init(void);
void LCD_NAME(u8 LCD_flag);
void LCD_Handle(u8 LCD_flag);
void Bound_Reset(void);
extern float YAW_Init;
extern float YAW_Real_angle;
extern float YAW_angle;
#endif



