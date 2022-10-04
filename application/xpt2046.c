#include "xpt2046.h" 
#include "lcd.h"
#include "stdlib.h"
#include "timer.h"
 


/*********************************************************************************
*********************�������� STM32F407Ӧ�ÿ�����(�����)*************************
**********************************************************************************
* �ļ�����: xpt2046.c                                                            *
* �ļ�������������������������                                                   *
* �������ڣ�2018.08.30                                                           *
* ��    ����V1.0                                                                 *
* ��    �ߣ�Clever                                                               *
* ˵    ����                                                                     * 
**********************************************************************************
*********************************************************************************/	 
	
//u16 Xdown=0; 		 
//u16 Ydown=0;	     //�����������¾ͷ��صĵ�����ֵ
//u16 Xup=0;
//u16 Yup=0; 			   //������������֮��̧�𷵻صĵ�����ֵ 

///**************Ĭ��Ϊ��������***************/
//u8  CMD_RDX=0XD0;
//u8  CMD_RDY=0X90;

//float xFactor=0.06671114;	 //����У׼���� 
//float yFactor=0.09117551;
//short xOffset=-11;       //xOffset��yOffset ���Ը���ʵ����ͷ�ĵ������ʵ���������׼
//short yOffset=-18;
///*******************************************/

///**************����Ϊ��ͨIOģ��SPIͨ��XPT2046***************/
///****************************************************************************
//* ��    ��: void SPI_Write_Byte(u8 num) 
//* ��    �ܣ�SPIд����,������ICд��1byte���� 
//* ��ڲ�����num:Ҫд�������
//* ���ز�������
//* ˵    ����       
//****************************************************************************/
//void SPI_Write_Byte(u8 num)    
//{  
//	u8 count=0;   
//	for(count=0;count<8;count++)  
//	{ 	  
//		if(num&0x80)TDIN=1;  
//		else TDIN=0;   
//		num<<=1;    
//		TCLK=0; 
//		delay_us(1);
//		TCLK=1;		 //��������Ч	        
//	}		 			    
//} 	

///****************************************************************************
//* ��    ��: u16 SPI_Read_AD(u8 CMD)
//* ��    �ܣ�SPI������ ,�Ӵ�����IC��ȡadcֵ
//* ��ڲ�����CMD:ָ��
//* ���ز���: ����������	   
//* ˵    ����       
//****************************************************************************/  
//u16 SPI_Read_AD(u8 CMD)	  
//{ 	 
//	u8 count=0; 	  
//	u16 Num=0; 
//	TCLK=0;		//������ʱ�� 	 
//	TDIN=0; 	//����������
//	TCS=0; 		//ѡ�д�����IC 2046
//	SPI_Write_Byte(CMD);//����������
//	delay_us(6); 
//	TCLK=0; 	     	    
//	delay_us(1);    	   
//	TCLK=1;		 
//	delay_us(1);    
//	TCLK=0; 	     	    
//	for(count=0;count<16;count++)//����16λ����,ֻ�и�12λ��Ч 
//	{ 				  
//		Num<<=1; 	 
//		TCLK=0;	 //�½�����Ч  	    	   
//		delay_us(1);    
// 		TCLK=1;
// 		if(DOUT)Num++; 		 
//	}  	
//	Num>>=4;   	//��12λ����Ч
//	TCS=1;		  //�ͷ�Ƭѡ	 
//	return(Num);   
//}
///**************����Ϊ��ͨIOģ��SPIͨ��XPT2046***************/


///****************************************************************************
//* ��    ��: u16 RTouch_Read_XorY(u8 xy)
//* ��    �ܣ���ȡһ������ֵ(x����y)
//* ��ڲ�����xoy:ָ�CMD_RDX/CMD_RDY
//* ���ز���: ����������	   
//* ˵    ����������ȡ5������,����Щ������������, Ȼ��ȥ����ͺ���1����,ȡƽ��ֵ       
//****************************************************************************/ 
//u16 RTouch_Read_XorY(u8 xoy)
//{
//	u16 i, j;
//	u16 buf[5];
//	u16 sum=0;
//	u16 temp;
//	
//	for(i=0;i<5;i++)buf[i] = SPI_Read_AD(xoy);		 		    
//	for(i=0;i<5-1; i++)  //����
//	{
//		for(j=i+1;j<5;j++)
//		{
//			if(buf[i]>buf[j])//���� 
//			{
//				temp=buf[i];
//				buf[i]=buf[j];
//				buf[j]=temp;
//			}
//		}
//	}	  
//	sum=0;
//	for(i=1;i<5-1;i++)sum+=buf[i];
//	temp=sum/(5-2*1);
//	return temp;   
//} 

///****************************************************************************
//* ��    ��: u8 RTouch_Read_XY(u16 *x,u16 *y)
//* ��    �ܣ���ȡx,y����
//* ��ڲ�����x,y:��ȡ��������ֵ
//* ���ز���: ����ֵ:0,ʧ��;1,�ɹ�   
//* ˵    ������Сֵ��������50.       
//****************************************************************************/ 
//u8 RTouch_Read_XY(u16 *x,u16 *y)
//{
//	u16 xtemp,ytemp;			 	 		  
//	xtemp=RTouch_Read_XorY(CMD_RDX);
//	ytemp=RTouch_Read_XorY(CMD_RDY);	  												   
//	if(xtemp<50||ytemp<50)return 0;//����ʧ��
//	*x=xtemp;
//	*y=ytemp;
//	return 1;//�����ɹ�
//}

///****************************************************************************
//* ��    ��: u8 RTouch_Read_XY2(u16 *x,u16 *y)
//* ��    �ܣ�����2�ζ�ȡ������IC
//* ��ڲ�����x,y:��ȡ��������ֵ
//* ���ز���: ����ֵ:0,ʧ��;1,�ɹ�   
//* ˵    ����        
//****************************************************************************/
//u8 RTouch_Read_XY2(u16 *x,u16 *y) 
//{
//	u16 x1,y1;
// 	u16 x2,y2;
// 	u8 flag;    
//    flag=RTouch_Read_XY(&x1,&y1);   
//    if(flag==0)return(0);
//    flag=RTouch_Read_XY(&x2,&y2);	   
//    if(flag==0)return(0);   
//    if(((x2<=x1&&x1<x2+50)||(x1<=x2&&x2<x1+50))  //ǰ�����β�����+-50��
//    &&((y2<=y1&&y1<y2+50)||(y1<=y2&&y2<y1+50)))
//    {
//        *x=(x1+x2)/2;
//        *y=(y1+y2)/2;
//        return 1;
//    }else return 0;	  
//}  


//u16 x;
//u16 y;
//u8 time;
///****************************************************************************
//* ��    ��: void XPT2046_Scan(u8 tp)
//* ��    �ܣ���������ɨ��	
//* ��ڲ�����type:0,��Ļ����;1,�������� 
//* ���ز���: ��  
//* ˵    ����   ������#������     
//****************************************************************************/
//void XPT2046_Scan(u8 type)
//{			   
//		Xup=0xffff;
//    Yup=0xffff;	 
//	if(PEN==0)//�а�������
//	{
//		if(type)RTouch_Read_XY2(&x,&y);//��ȡ��������
//		else if(RTouch_Read_XY2(&x,&y))//��ȡ��Ļ����
//		{
//	 		x=xFactor*x+xOffset;     //�����ת��Ϊ��Ļ����
//			y=yFactor*y+yOffset;  
//	 	} 	
//		Xdown=x;
//		Ydown=y;
//		
//		time++;		   
//	}else  //��̧��
//	{    
//		 if(time>2)
//		 {
//			 		Xup=x;
//		      Yup=y;	 
//		 }
//		 time=0;
//		 Xdown=0xffff;
//     Ydown=0xffff;	 
//	}
//}	

//int one=0,two=0,three=0,four=0,five=0,six=0,eleven=0,twelve=0,thirteen=0,fourteen=0,fifteen=0,sixteen=0,seventeen=0;
// int XPT2046_PEN(void)
//{
//	
//	if(PEN==0)//�а�������return 
//	{
////		 if(Xdown<25&&Ydown<30&&one==0) 
////		 {
////			one=1;
////		 return 1;
////		 }
////		else if(Xdown>25&&Xdown<50&&Ydown<30&&two==0)
////		{
////			two=1;
////		return 2;
////		}
////		else if(Xdown>50&&Xdown<75&&Ydown<30&&three==0)
////		{three=1;
////			return 3;
////		}
////		else if(Xdown>75&&Xdown<100&&Ydown<30&&four==0)
////		{four=1;
////			return 4;
////		}
////		else if(Xdown>100&&Xdown<125&&Ydown<30&&five==0)
////		{five=1;
////			return 5;
////		}
////		else if(Xdown>125&&Xdown<150&&Ydown<30&&six==0)
////		{six=1;
////			return 6;	
////		}	
//		 if(Xdown<40&&Ydown<100&&Ydown>60&&eleven==0)
//		{
//			eleven=1;
//			return 11;
//		}
//		else if(Xdown<110&&Xdown>70&&Ydown<100&&Ydown>60&&twelve==0)
//		{
//			twelve=1;
//			return 12;
//		}
//		else if(Xdown<180&&Xdown>140&&Ydown<100&&Ydown>60&&thirteen==0)
//		{
//			thirteen=1;
//			return 13;
//		}
//		else if(Xdown<140&&Xdown>100&&Ydown<160&&Ydown>120&&fourteen==0)
//		{	
//			fourteen=1;
//			return 14;
//		}
//		else if(Xdown<70&&Xdown>30&&Ydown<160&&Ydown>120&&fifteen==0)
//		{	
//			fifteen=1;
//			return 15;
//		}	
//		else if(Xdown>70&&Xdown<110&&Ydown<220&&Ydown>180&&sixteen==0)
//		{	
//			sixteen=1;
//			return 16;	
//		}		
//		else if(Xdown>140&&Xdown<180&&Ydown<220&&Ydown>180&&seventeen==0)
//		{	
//			seventeen=1;
//			return 17;	
//		}			
//	}
//	
//	else 
//		
//		return 0;
//	
//	return 0;
//	
//}





////xpt2046��ʼ��  		    
//void XPT2046_Init(void)
//{
//   GPIO_InitTypeDef  GPIO_InitStructure;	
//    __HAL_RCC_GPIOA_CLK_ENABLE();           //����GPIOAʱ��  
// 		__HAL_RCC_GPIOB_CLK_ENABLE();           //����GPIOBʱ��
//	  __HAL_RCC_GPIOF_CLK_ENABLE();           //����GPIOFʱ��
//	
//    GPIO_InitStructure.Pin=GPIO_PIN_2; //A5
//    GPIO_InitStructure.Mode=GPIO_MODE_INPUT;  //����ģʽ
//    GPIO_InitStructure.Pull=GPIO_PULLUP;          //����
//    GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;     //����
//    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
//	
//	  GPIO_InitStructure.Pin=GPIO_PIN_11;
//    HAL_GPIO_Init(GPIOF,&GPIO_InitStructure);	
//	
//	  GPIO_InitStructure.Pin=GPIO_PIN_0|GPIO_PIN_1; //A5
//    GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_PP;  //���ģʽ
//    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
//	
//	  GPIO_InitStructure.Pin=GPIO_PIN_5; //A5
//    HAL_GPIO_Init(GPIOA,&GPIO_InitStructure);	
//   
//		if(dir_flag)  //���������ת X Y
//		{
//			CMD_RDX=0X90;
//			CMD_RDY=0XD0;	
//      xFactor=-0.09195402;	//����У׼���� 
//      yFactor=0.06736275;
//      xOffset=348;
//      yOffset=-19;			
//		}
//		else				    
//		{
//			CMD_RDX=0XD0;
//			CMD_RDY=0X90;
//      xFactor=0.06671114;	  //����У׼���� 
//      yFactor=0.09117551;
//      xOffset=-11;  
//      yOffset=-18;				
//		}										 
//}

