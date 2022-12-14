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
#include "main.h"
float YAW_angle;
float YAW_Init;
float YAW_Real_angle;

/*
* @name 	    TIME_Change
* @brief  		
* @param    	trot :arr占空比一般取0.5  
* @param 			Walk :arr占空比一般取0.25 
* @param      Bound:arr占空比一般取0.1 
* @retval
*/
void TIME_Change(void)
{
	static u8 Reset_TIME_flag[4]={0};
	static float Reset_TIME[4]={0};
	  if(flag==STOP||flag==BOUND||flag==BOUND_PLANE||flag==TROT_STOP)//stop/retset time =0
		{
			if(Reset_TIME_flag[0]==0)
			{
//				Speed=0;
				TIME[0]=0;
				TIME[1]=0;
				TIME[2]=0;
				TIME[3]=0;

				Reset_TIME_flag[3]=0;
				Reset_TIME_flag[1]=0;
				Reset_TIME_flag[2]=0;
				Reset_TIME_flag[0]=1;		
			}
		}
		else if(flag==TROT||flag==LEFT||flag==BEHIND||flag==RIGHT||flag==TROT_WARP)//trot
		{
			Reset_TIME[0]=0.0f;Reset_TIME[1]=0.50f;Reset_TIME[2]=0.50f;Reset_TIME[3]=0.0f;
			if(Reset_TIME_flag[1]==0)
			{
				TIME[0]=TIME[0]+Reset_TIME[0];
				TIME[1]=TIME[1]+Reset_TIME[1];
				TIME[2]=TIME[2]+Reset_TIME[2];
				TIME[3]=TIME[3]+Reset_TIME[3];
				
				Reset_TIME_flag[0]=0;
				Reset_TIME_flag[2]=0;
				Reset_TIME_flag[3]=0;
				Reset_TIME_flag[1]=1;
			}
		}		

}



/*
* @name 	    Project_NUM
* @brief  		
* @param    	trot :arr占空比一般取0.5  
* @param 			Walk :arr占空比一般取0.25 
* @param      Bound:arr占空比一般取0.1 
* @retval
*/
u8 Project_NUM(void)
{
	static u8 num_flag=0;
	static u8 flag_num=0;
	static u8 speed_flag=0;
if(m2006set[0].num>15000||m2006set[1].num>15000||m2006set[2].num>15000||m2006set[3].num>15000||m3508set[0].num>15000||m3508set[1].num>15000||m3508set[2].num>15000||m3508set[3].num>15000)
num_flag=1;
else num_flag=0;

if(abs(m3508[0].speed)<2||abs(m3508[1].speed)<2||abs(m3508[2].speed)<2||abs(m3508[3].speed)<2||abs(m2006[0].speed)<2||abs(m2006[1].speed)<2||abs(m2006[2].speed)<2||abs(m2006[3].speed)<2)
speed_flag =1;
else speed_flag=0;

if(speed_flag==1&&num_flag==1)
{
	m3508set[0].num=0;m3508set[1].num=0;m3508set[2].num=0;m3508set[3].num=0;
	m2006set[0].num=0;m2006set[1].num=0;m2006set[2].num=0;m2006set[3].num=0;
	m3508set[0].setspeed=0;m3508set[1].setspeed=0;m3508set[2].setspeed=0;m3508set[3].setspeed=0;
	m2006set[0].setspeed=0;m2006set[1].setspeed=0;m2006set[2].setspeed=0;m2006set[3].setspeed=0;
	LED1 =1;		
	flag_num=1;
}
else flag_num=0;
 return  flag_num;
}

/*
* @name 	    Project_Temperature
* @brief  		
* @param    	trot :arr占空比一般取0.5  
* @param 			Walk :arr占空比一般取0.25 
* @param      Bound:arr占空比一般取0.1 
* @retval
*/
u8 Project_Temperature(void)
{
	static u8 Temperature_flag=0;
	static u8 flag_temperature=0;
	
 if(m2006[0].temperate>85||m2006[1].temperate>80||m2006[2].temperate>80||m2006[3].temperate>80||m3508[0].temperate>80||m3508[1].temperate>80||m3508[2].temperate>80||m3508[3].temperate>80)
 {
	m3508set[0].setspeed=0;
	m3508set[1].setspeed=0;
	m3508set[3].setspeed=0;
	m3508set[2].setspeed=0;
	m2006set[0].setspeed=0;
	m2006set[1].setspeed=0;
	m2006set[2].setspeed=0;
	m2006set[3].setspeed=0;
	m3508set[0].num=0;m3508set[1].num=0;m3508set[2].num=0;m3508set[3].num=0;
	m2006set[0].num=0;m2006set[1].num=0;m2006set[2].num=0;m2006set[3].num=0;
	 
	Temperature_flag=1;
	flag_temperature=1;
 }
 else if(flag_temperature==0)
 Temperature_flag=0;
	
 return  Temperature_flag;
}
/*
* @name 	    Remove_Flag
* @brief  		遥控器改变模式
* @param    	control.s[0]为WASD的键返回值  
* @param 			control.s[1]
* @param       
* @retval
*/
void Remove_Flag(void)
{
	if(control.s[0]==0&&control.s[1]==0)      																					                  								 flag=STOP;
	else if(control.s[0]==1&&control.s[1]!=1&&control.s[1]!=2)                                                              flag=TROT;
	else if(control.s[0]==2&&control.s[1]==0)                                                                              flag=LEFT;
  else if(control.s[0]==3&&control.s[1]==0)                                                                              flag=BEHIND;
	else if(control.s[0]==4&&control.s[1]==0)                                                                              flag=RIGHT;
	else if(control.s[1]==1)                                                                              flag=BOUND;
  else if(control.s[1]==2)                                                                            	flag=BOUND_PLANE;
//  else if(control.s[5]==1&&control.s[0]==1&&control.s[1]==0&&control.s[4]==2)                                            flag=TROT_WARP ; 
//	else if(control.s[0]==1&&control.s[1]==0&&control.s[4]==1)                                            flag=TROT_STOP;

}
void Bound_Reset(void)
{
	if(flag!=BOUND)
	Bound_Flag_Point[0]=1;
	if(flag!=BOUND_PLANE)
	Bound_Flag_Point[1]=1;
}
/*
* @name 	    JY_YAW_Handle
* @brief  		遥控器改变模式
* @param    	  
* @param 			
* @param       
* @retval
*/
void JY_YAW_Handle(void)
{
	YAW_Real_angle=angle.Yaw-YAW_Init ;
	if(YAW_Real_angle<-180)YAW_Real_angle=YAW_Real_angle+360;
	if(YAW_Real_angle>180)YAW_Real_angle=YAW_Real_angle-360;
}



/*
* @name 	    void Data_Processing(u8 lcd_flag)
* @brief  		数据处理
* @param    	 lcd_flag=0是为坐标点
* @param 			 lcd_flag=1是为电机角度反馈
* @param       
* @retval
*/
void Data_Processing(u8 lcd_flag)
{			
	LCD_Handle(lcd_flag);      //LCD反馈
	Receive_control();         //接受遙控器	
	JY_YAW_Handle();           //YAW陀螺仪角度校准
	TIME_Change();             //时间改变标志
	Bound_Reset();             //起跳复位
	Remove_Flag();             //遥控器控制模式
}

/*
* @name 	    NF24G_Init
* @brief  		2.4G遥控器初始化
* @param    	  
* @param 			
* @param       
* @retval
*/
void NF24G_Init(void)
{
		NRF24L01_Init();
		while(NRF24L01_Check())
		{	
		LED0 =0;			
		}
		LED0 =1;			
		NRF24L01_RX_Mode();//配置为接收模式
}

void LCD_NAME(u8 LCD_flag)
{
	 
	if(LCD_flag==0)
	{
		LCD_DisplayString_color(0, 0, 16,  "Ax", BLACK, GREEN);
		LCD_DisplayString_color(0, 20, 16, "Bx", BLACK, GREEN);
		LCD_DisplayString_color(0, 40, 16, "Ex", BLACK, GREEN);
		LCD_DisplayString_color(60, 0, 16, "Az", BLACK, GREEN);
		LCD_DisplayString_color(60, 20, 16,"Bz", BLACK, GREEN);
		LCD_DisplayString_color(60, 40, 16,"Ez", BLACK, GREEN);

		LCD_Draw_Line(0, 77, 80, 77, RED);LCD_Draw_Line(81, 77, 160, 77, GREEN);LCD_Draw_Line(161, 77, 240, 77, BLUE);
		
		LCD_DisplayString_color(0, 80, 16, "A2x", BLACK, GREEN);
		LCD_DisplayString_color(0, 100, 16,"B2x", BLACK, GREEN);
		LCD_DisplayString_color(0, 120, 16,"E2x", BLACK, GREEN);
		LCD_DisplayString_color(60, 80, 16, "A2z", BLACK, GREEN);
		LCD_DisplayString_color(60, 100, 16, "B2Z", BLACK, GREEN);
		LCD_DisplayString_color(60, 120, 16,"E2Z", BLACK, GREEN);

		LCD_Draw_Line(0, 137, 80, 137, RED);LCD_Draw_Line(81, 137, 160, 137, GREEN);LCD_Draw_Line(161, 137, 240, 137, BLUE);		
		LCD_DisplayString_color(0, 140, 16, "A3x", BLACK, GREEN);
		LCD_DisplayString_color(0, 160, 16, "B3x", BLACK, GREEN);
		LCD_DisplayString_color(0, 180, 16, "E3x", BLACK, GREEN);
		LCD_DisplayString_color(60, 140, 16,"A3z", BLACK, GREEN);
		LCD_DisplayString_color(60, 160, 16,"B3Z", BLACK, GREEN);
		LCD_DisplayString_color(60, 180, 16,"E3Z", BLACK, GREEN);	
		
		LCD_Draw_Line(0, 197, 80, 197, RED);LCD_Draw_Line(81, 197, 160, 197, GREEN);LCD_Draw_Line(161, 197, 240, 197, BLUE);
		LCD_DisplayString_color(0, 200, 16, "A4x", BLACK, GREEN);
		LCD_DisplayString_color(0, 220, 16, "B4x", BLACK, GREEN);
		LCD_DisplayString_color(0, 240, 16, "E4x", BLACK, GREEN);
		LCD_DisplayString_color(75, 200, 16,"A4z", BLACK, GREEN);
		LCD_DisplayString_color(75, 220, 16,"B4Z", BLACK, GREEN);
		LCD_DisplayString_color(75, 240, 16,"E4Z", BLACK, GREEN);	

		LCD_Draw_Line(0, 257, 80, 257, RED);LCD_Draw_Line(81, 257, 160, 257, GREEN);LCD_Draw_Line(161, 257, 240, 257, BLUE);
		
		LCD_DisplayString_color(0, 260, 16, "s0", BLACK, GREEN);
		LCD_DisplayString_color(0, 280, 16, "s1", BLACK, GREEN);
		LCD_DisplayString_color(75, 260, 16,"s2", BLACK, GREEN);
		LCD_DisplayString_color(75, 280, 16,"s3", BLACK, GREEN);
	}
	else if(LCD_flag!=0)
	{
		LCD_DisplayString_color(0, 0, 16,  "20st", BLACK, GREEN);
		LCD_DisplayString_color(0, 20, 16, "21st", BLACK, GREEN);
		LCD_DisplayString_color(0, 40, 16, "22st", BLACK, GREEN);
		LCD_DisplayString_color(0, 60, 16, "23st", BLACK, GREEN);
		
		LCD_DisplayString_color(100, 0, 16, "20re", BLACK, GREEN);
		LCD_DisplayString_color(100, 20, 16, "21re", BLACK, GREEN);
		LCD_DisplayString_color(100, 40, 16, "22re", BLACK, GREEN);
		LCD_DisplayString_color(100, 60, 16, "23re", BLACK, GREEN);

		LCD_Draw_Line(0, 77, 80, 77, RED);LCD_Draw_Line(81, 77, 160, 77, GREEN);LCD_Draw_Line(161, 77, 240, 77, BLUE);
		LCD_DisplayString_color(0, 80, 16, "30st", BLACK, GREEN);
		LCD_DisplayString_color(0, 100, 16,"31st", BLACK, GREEN);
		LCD_DisplayString_color(0, 120, 16,"32st", BLACK, GREEN);
		LCD_DisplayString_color(0, 140, 16,"33st", BLACK, GREEN);
		
		LCD_DisplayString_color(100, 80, 16, "30re", BLACK, GREEN);
		LCD_DisplayString_color(100, 100, 16,"31re", BLACK, GREEN);
		LCD_DisplayString_color(100, 120, 16,"32re", BLACK, GREEN);
		LCD_DisplayString_color(100, 140, 16,"33re", BLACK, GREEN);

		LCD_Draw_Line(0, 137, 80, 137, RED);LCD_Draw_Line(81, 137, 160, 137, GREEN);LCD_Draw_Line(161, 137, 240, 137, BLUE);		

		LCD_DisplayString_color(0, 180, 16, "flag", BLACK, GREEN);
		LCD_DisplayString_color(0, 200, 16,"TIME", BLACK, GREEN);
		LCD_DisplayString_color(0, 220, 16,"bdfp", BLACK, GREEN);


		LCD_Draw_Line(0, 197, 80, 197, RED);LCD_Draw_Line(81, 197, 160, 197, GREEN);LCD_Draw_Line(161, 197, 240, 197, BLUE);


		LCD_Draw_Line(0, 257, 80, 257, RED);LCD_Draw_Line(81, 257, 160, 257, GREEN);LCD_Draw_Line(161, 257, 240, 257, BLUE);
		
		LCD_DisplayString_color(0, 260, 16, "s0", BLACK, GREEN);
		LCD_DisplayString_color(0, 280, 16, "s1", BLACK, GREEN);
		LCD_DisplayString_color(75, 260, 16,"s2", BLACK, GREEN);
		LCD_DisplayString_color(75, 280, 16,"s3", BLACK, GREEN);
		LCD_DisplayString_color(150, 260, 16,"s4", BLACK, GREEN);
		LCD_DisplayString_color(150, 280, 16,"s5", BLACK, GREEN);

	}
}
void LCD_Handle(u8 LCD_flag)
{
	if(LCD_flag==0)
	{
	LCD_Num(30,  0,link[0].point_A.x , 3, 16);
	LCD_Num(30, 20,link[0].point_B.x , 3, 16);
	LCD_Num(30, 40,link[0].point_E.x , 3, 16);
	LCD_Num(90,  0,link[0].point_A.z , 3, 16);
	LCD_Num(90, 20,link[0].point_B.z , 3, 16);
	LCD_Num(90, 40,link[0].point_E.z , 3, 16);
		
	LCD_Num(30, 80, link[1].point_A.x, 3, 16);
	LCD_Num(30, 100,link[1].point_B.x, 3, 16);
	LCD_Num(30, 120,link[1].point_E.x, 3, 16);
	LCD_Num(90, 80, link[1].point_A.z, 3, 16);
	LCD_Num(90, 100,link[1].point_B.z, 3, 16);
	LCD_Num(90, 120,link[1].point_E.z, 3, 16);	
		
	LCD_Num(30, 140,link[2].point_A.x, 3, 16);
	LCD_Num(30, 160,link[2].point_B.x, 3, 16);
	LCD_Num(30, 180,link[2].point_E.x, 3, 16);
	LCD_Num(90, 140,link[2].point_A.z, 3, 16);
	LCD_Num(90, 160,link[2].point_B.z, 3, 16);
	LCD_Num(90, 180,link[2].point_E.z, 3, 16);

	LCD_Num(30, 200,link[3].point_A.x, 3, 16);
	LCD_Num(30, 220,link[3].point_B.x, 3, 16);
	LCD_Num(30, 240,link[3].point_E.x, 3, 16);
	LCD_Num(90, 200,link[3].point_A.z, 3, 16);
	LCD_Num(90, 220,link[3].point_B.z, 3, 16);
	LCD_Num(90, 240,link[3].point_E.z, 3, 16);

	LCD_Num(50, 260,control.s[0] , 5, 16);
	LCD_Num(50, 280,control.s[1] , 5, 16);
	LCD_Num(150, 260,control.s[2], 5, 16);
	LCD_Num(150, 280,control.s[3], 5, 16);
	}
	if(LCD_flag!=0)
	{
		LCD_Num(50,  0,m2006set[0].setpos , 6, 16);
		LCD_Num(50, 20,m2006set[1].setpos , 6, 16);
		LCD_Num(50, 40,m2006set[2].setpos , 6, 16);
		LCD_Num(50, 60,m2006set[3].setpos , 6, 16);
		
		LCD_Num(150, 0,m2006[0].real_angle, 6, 16);
		LCD_Num(150,20,m2006[1].real_angle, 6, 16);
		LCD_Num(150,40,m2006[2].real_angle, 6, 16);
		LCD_Num(150,60,m2006[3].real_angle, 6, 16);	
	
		LCD_Num(50,  80,m3508set[0].setpos , 6, 16);
		LCD_Num(50, 100,m3508set[1].setpos , 6, 16);
		LCD_Num(50, 120,m3508set[2].setpos , 6, 16);
		LCD_Num(50, 140,m3508set[3].setpos , 6, 16);
		
		LCD_Num(150, 80,m3508[0].real_angle, 6, 16);
		LCD_Num(150,100,m3508[1].real_angle, 6, 16);
		LCD_Num(150,120,m3508[2].real_angle, 6, 16);
		LCD_Num(150,140,YAW_Real_angle, 6, 16);	

		LCD_Num(50, 180,flag , 3, 16);	
		LCD_Num(50, 200,TIME[0]*1000 , 5, 16);		
		
		LCD_Num(30, 260,control.s[0], 3, 16);
		LCD_Num(30, 280,control.s[1], 3, 16);
		LCD_Num(100, 260,control.s[2], 3, 16);
		LCD_Num(100, 280,control.s[3], 3, 16);
		LCD_Num(170, 260,control.s[4], 3, 16);
		LCD_Num(170, 280,control.s[5], 3, 16);			
	}		

}


