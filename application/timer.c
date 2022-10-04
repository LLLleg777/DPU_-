#include "timer.h"
#include "tim.h"
#include "Init.h"




/*****微秒延时***/
#define DLY_TIM_Handle  (&htim14)                     //微秒延时句柄

//实际测量每50us大约差0.24us
void delay_us(uint16_t nus)
{
    __HAL_TIM_SET_COUNTER(DLY_TIM_Handle, 0);
    __HAL_TIM_ENABLE(DLY_TIM_Handle);

    while (__HAL_TIM_GET_COUNTER(DLY_TIM_Handle) < nus)
    {
    }

    __HAL_TIM_DISABLE(DLY_TIM_Handle);
}
void delay_ms(uint16_t nms)
{
    uint32_t i;

    for(i = 0; i < nms; i++)
        delay_us(1000);
}
/*************微秒延时结束***********/

/*************初始化**********/
int TIM2_mode=1,TIM3_mode=1,TIM6_mode= 1;                //定时器模式
int t1, t_1, t2, t_2, t_Accel, Accelt, t_text;//时间计算
int Time_chang_zero=0;

/*************初始化结束**********/

/****************定时器回调函数*********/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        TIM2_int(TIM2_mode);

    }

    if(htim->Instance == TIM3)
    {
        TIM3_int(TIM3_mode);

    }
    if(htim->Instance == TIM6)
    {
        TIM6_int(TIM6_mode);

    }
}
/**
* @name 	    TIM2_int
* @brief  		定时器2输入初始化
* @param     mode：1：开启所有 else：none
* @param
* @param
* @retval
*/
void TIM2_int(u8 mode)
{			
   if(mode == 1)			
    {	
			
			TIME[0]=TIME[0]+Speed;if(TIME[0]>TS)TIME[0]=0;else if(TIME[0]<0)TIME[0]=0;
			TIME[1]=TIME[1]+Speed;if(TIME[1]>TS)TIME[1]=0;else if(TIME[1]<0)TIME[1]=0;
			TIME[2]=TIME[2]+Speed;if(TIME[2]>TS)TIME[2]=0;else if(TIME[2]<0)TIME[2]=0;
			TIME[3]=TIME[3]+Speed;if(TIME[3]>TS)TIME[3]=0;else if(TIME[3]<0)TIME[3]=0;

			m3508set[0].num = (int16_t)PID_motoloop(&Motor_PID[0], &m3508[0],m3508set[0].setspeed,m3508set[0].setpos);  
			m3508set[1].num = (int16_t)PID_motoloop(&Motor_PID[1], &m3508[1],m3508set[1].setspeed,m3508set[1].setpos);
			m3508set[2].num = (int16_t)PID_motoloop(&Motor_PID[2], &m3508[2],m3508set[2].setspeed,m3508set[2].setpos);  
			m3508set[3].num = (int16_t)PID_motoloop(&Motor_PID[3], &m3508[3],m3508set[3].setspeed,m3508set[3].setpos);	
			
			can1_cmd1(m3508set[0].num,m3508set[1].num,m3508set[2].num,m3508set[3].num);
		}
		else
		{

		}
}

void TIM3_int(u8 mode)
{
    if(mode == 1)			
    {	
			m2006set[0].num = (int16_t)PID_motoloop(&Motor_PID[4], &m2006[0],m2006set[0].setspeed,m2006set[0].setpos);  
			m2006set[1].num = (int16_t)PID_motoloop(&Motor_PID[5], &m2006[1],m2006set[1].setspeed,m2006set[1].setpos);
			m2006set[2].num = (int16_t)PID_motoloop(&Motor_PID[6], &m2006[2],m2006set[2].setspeed,m2006set[2].setpos);  
			m2006set[3].num = (int16_t)PID_motoloop(&Motor_PID[7], &m2006[3],m2006set[3].setspeed,m2006set[3].setpos);
			
			can2_cmd1(m2006set[0].num,m2006set[1].num,m2006set[2].num,m2006set[3].num);				
        
    }
    else
    {

    }
}

void TIM6_int(u8 mode)
{
    if(mode == 1)
    { 
        //时间计算
        if(t_1 == 1)
            t1++;
        else
            t1 = 0;

        if(t_2 == 1)
            t2++;
        else
            t2 = 0;

        if(t_Accel == 1)
            Accelt++;
        else
            Accelt = 9999;
		}
		else 
		{
		}
			
}

