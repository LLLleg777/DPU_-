#include "touch.h"
#include "ft5426.h"
#include "xpt2046.h"
#include "lcd.h"
#include "timer.h"
#include "app_can.h"
#include "move.h"
#include "pid.h"







///*********************************************************************************
//*********************�������� STM32F407Ӧ�ÿ�����(�����)*************************
//**********************************************************************************
//* �ļ�����: touch.c                                                              *
//* �ļ����������������Գ���                                                       *
//* �������ڣ�2018.08.30                                                           *
//* ��    ����V1.0                                                                 *
//* ��    �ߣ�Clever                                                               *
//* ˵    ����                                                                     *
//**********************************************************************************
//*********************************************************************************/
////��������ʼ��
//void Touch_Init(void)
//{
//    if(lcd_id==0x9341)
//    {
//        XPT2046_Init();
//    }
//    else if(lcd_id==0x1963)
//    {
//        FT5426_Init();
//    }
//}

////�����Ļ
//void Clear_Screen(void)
//{
//	if(ball_t>=6)
//  {
//		LCD_Clear(BLACK);//����
//	}
//    BRUSH_COLOR=BLUE;//��������Ϊ��ɫ
//    LCD_DisplayString(lcd_width-40,lcd_height-18,16,"Clear");//��ʾ��������
//    BRUSH_COLOR=RED;//���û�����ɫ
// 
//	
//}

///****************************************************************************
//* ��    ��: u8 Draw_Point(u16 x,u16 y,u16 color)
//* ��    �ܣ���һ����(4*4�ĵ�)
//* ��ڲ�����x,y:����
//            color:��ɫ
//* ���ز���: ��
//* ˵    ����
//****************************************************************************/
//void Draw_Point(u16 x,u16 y,u16 color)
//{
//    u8 i=0;

//    BRUSH_COLOR=color;
//    for(i=0; i<4; i++)
//    {
//        LCD_DrawPoint(x,y+i);
//        LCD_DrawPoint(x+1,y+i);
//        LCD_DrawPoint(x+2,y+i);
//        LCD_DrawPoint(x+3,y+i);
//    }
//}

////���败�������Ժ���

//void R_Touch_test(void)
//{
//   

////    XPT2046_Scan(0);

////    if(Xdown<lcd_width&&Ydown<lcd_height)
////    {
////			if(Xdown>(lcd_width-40)&&Ydown>lcd_height-18)
////        Clear_Screen();  //�����Ļ
////			
////			 touch_PIN();

////        
//////        else if(Xdown<125&&Ydown<80)
//////        {

//////            LCD_Fill_onecolor(0,0,125,80,YELLOW);
//////            Draw_Point(Xdown,Ydown,RED);		//��ͼ

//////        }
//   // }
//	while(1)
//	{


//	}
//}


//���ݴ��������Ժ���
//void C_Touch_test(void)
//{
//    //u8 i=0;

//    while(1)
//    {
//        FT5426_Scan();

//        if(x[0]>(lcd_width-40)&&y[0]>lcd_height-24)
//        {
//            Clear_Screen();//����

//        }
//    }
//}


//	




