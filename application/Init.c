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
	LCD_Init();           //��ʼ��LCD FSMC�ӿں���ʾ����
	NF24G_Init();         //2.4g��ʼ��
//   remote_control_init();//DJiң������ʼ��
	can_filter_init();    //can�˲�����ʼ��
	motor_pid_set();      //DJi pid��ʼ��
	LCD_NAME(LCD_flag);
BEED=1;HAL_Delay(1000);BEED=0;					       //	LEDΪ0����1��;BEED��1��
//	HAL_UART_Receive_IT(&huart5,(u8*)&rec_data5,1);// �������ڽ�������������
//    HAL_UART_Receive_IT(&huart1,(u8*)&rec_data1,1);//�������ڽ���ͨ������
	HAL_Delay (1000);
//	YAW_Init = angle.Yaw ;        //��YAW����
	HAL_TIM_Base_Start_IT(&htim2);//������ʱ��2�ж�
	HAL_TIM_Base_Start_IT(&htim3);//������ʱ��3�ж�
	HAL_TIM_Base_Start_IT(&htim6);//������ʱ��6�ж�
	HAL_TIM_Base_Start_IT(&htim4);//������ʱ��4�ж�
	Convert_rigid_Coordinate_Init();
}



