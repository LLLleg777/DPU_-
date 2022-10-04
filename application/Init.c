#include "Init.h"
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"
#include "app_can.h"
#include "app_usart.h"
#include "lcd.h"
#include "pid.h"
#include "DR16.h"
#include "move.h"
#include "key.h"
#include "timer.h"
#include "math.h"
#include "leg_task.h"
#include "24l01.h"
#include "remote.h"
#include "protect.h"


void ALL_Init(u8 LCD_flag)
{
	LCD_Init();           //初始化LCD FSMC接口和显示驱动
	NF24G_Init();         //2.4g初始化
//   remote_control_init();//DJi遥控器初始化
	can_filter_init();    //can滤波器初始化
	motor_pid_set();      //DJi pid初始化
	LCD_NAME(LCD_flag);
BEED=1;HAL_Delay(1000);BEED=0;					       //	LED为0亮，1灭;BEED為1叫
//	HAL_UART_Receive_IT(&huart5,(u8*)&rec_data5,1);// 开启串口接收陀螺仪数据
//    HAL_UART_Receive_IT(&huart1,(u8*)&rec_data1,1);//开启串口接收通信数据
	HAL_Delay (1000);
//	YAW_Init = angle.Yaw ;        //对YAW归零
	HAL_TIM_Base_Start_IT(&htim2);//开启定时器2中断
	HAL_TIM_Base_Start_IT(&htim3);//开启定时器3中断
	HAL_TIM_Base_Start_IT(&htim6);//开启定时器6中断
	HAL_TIM_Base_Start_IT(&htim4);//开启定时器4中断
	Convert_rigid_Coordinate_Init();
}



