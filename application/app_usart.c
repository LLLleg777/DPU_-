#include "main.h"
#include "usart.h"
#include "Init.h"
u8 rec_data1;
u8 rec_data5;
gyro angle;
separation hab;
/**************************/
/****************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {	
//				revice_Data(rec_data1);
        HAL_UART_Receive_IT(&huart1, (u8 *)&rec_data1, 1);//再次开启串口接收数据
    }
    if(huart->Instance == UART5)
    {
        Judge_Data(rec_data5);//处理数据
      HAL_UART_Receive_IT(&huart5, (u8 *)&rec_data5, 1);//再次开启串口接收数据
    }
}
void revice_Data(u8 ucData)
{
	static u8 ucRxBuffer1[250];
	static u8 ucRxCnt1 = 0;	
	ucRxBuffer1[ucRxCnt1++]=ucData;	//将收到的数据存入缓冲区中
 
	if (ucRxBuffer1[0]!=0x4A) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt1=0;
		return;	
	}

	if (ucRxCnt1<8) {return;}//数据不满8个，则返回
	else
	{ 
		switch(ucRxBuffer1[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x44:
			{
				hab.F = ucRxBuffer1[2];
				hab.R = ucRxBuffer1[3];	
				hab.H = ucRxBuffer1[4]<<8|ucRxBuffer1[5];						
				break;
			}
			default:break;	
		}
		
		ucRxCnt1=0;//清空缓存区
	}
}

void Judge_Data(u8 ucData)
{
    static u8 ucRxBuffer[250];
    static u8 ucRxCnt = 0;
    ucRxBuffer[ucRxCnt++] = ucData;	//将收到的数据存入缓冲区中
    if (ucRxBuffer[0] != 0x55) //数据头不对，则重新开始寻找0x55数据头
    {
        ucRxCnt = 0;
        return;
    }
    if (ucRxCnt<11)
   {
        return;   //数据不满11个，则返回
    }
    else
    {
        switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
        {
        //memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
        case 0x52://角速度
        {
            angle.Rollspeed= (short)(ucRxBuffer[3]<<8|ucRxBuffer[2])*2000/32768;
            angle.Pitchspeed=(short)(ucRxBuffer[5]<<8|ucRxBuffer[4])*2000/32768;
            angle.Yawspeed=  (short)(ucRxBuffer[7]<<8|ucRxBuffer[6])*2000/32768;
        }
        break;
        case 0x53://角度
        {
            angle.Roll= (short)(ucRxBuffer[3]<<8|ucRxBuffer[2])*180/32768.0;
            angle.Pitch=(short)(ucRxBuffer[5]<<8|ucRxBuffer[4])*180/32768.0;
            angle.Yaw=  (short)(ucRxBuffer[7]<<8|ucRxBuffer[6])*180/32768.0;
			// 			if(angle.Yaw>0){angle.Yaw =angle.Yaw -180;}
        }
        break;
	      case 0x51:
						angle.Rollplusspeed = (short)(ucRxBuffer [3]<<8| ucRxBuffer [2])/32768.0*180;  
						angle.Pitchplusspeed = (short)(ucRxBuffer [5]<<8| ucRxBuffer [4])/32768.0*180;
						angle.Yawplusspeed = (short)(ucRxBuffer [7]<<8| ucRxBuffer [6])/32768.0*180;
        default:
            break;
        }
        ucRxCnt=0;//清空缓存区
    }
}



//串口1发送一个字符
void uart1SendChar(u8 ch)
{
    while((USART1->SR & 0x40) == 0);

    USART1->DR = (u8) ch;
}
/****************************************************************************
* 名    称: void uart1SendChars(u8 *str, u16 strlen)
* 功    能：串口1发送一字符串
* 入口参数：*str：发送的字符串
            strlen：字符串长度
* 返回参数：无
* 说    明：
****************************************************************************/
void uart1SendChars(u8 *str, u16 strlen)
{
    u16 k = 0 ;

    do
    {
        uart1SendChar(*(str + k));    //循环发送,直到发送完毕
        k++;
    }
    while (k < strlen);
}

void uart1_Display(short num, int ch)
{
    int i;
    u8  tempbuf[20] = {0};
    short data[10] = {0};


    if(num < 0)
    {
        num = -num;					    //转为正数
        tempbuf[0] = '-';
    }
    else
    {
        tempbuf[0] = '+';
    }

    for(i = 1; i <= ch ; i++)
    {
        data[i] = pow(10, ch  - i);
        tempbuf[i] = (num / data[i]) % 10 + '0';
    }
    uart1SendChars(tempbuf, ch + 5);
}





/****************************************************************************
* 名    称:void Judge_wifi(u8 ucData)  
* 功    能：wifi处理数据
* 返回参数：无
* 说    明：
****************************************************************************/


void Judge_wifi(u8 ucData)  
{
   static u8 rec_data;
   static u8 receive_str[250];
   static u8 uart_byte_count = 0;
				rec_data =ucData;         //(USART1->DR) 读取接收到的数据
        if(rec_data=='S')		  	                         //如果是S，表示是命令信息的起始位
				{
					uart_byte_count=0x01; 
				}

			else if(rec_data=='E')		                         //如果E，表示是命令信息传送的结束位
				{
					if(strcmp("f",(char *)receive_str)==0){}	   //前进
					else if(strcmp("b",(char *)receive_str)==0){}	   //后退
					else if(strcmp("r",(char *)receive_str)==0){}  //右转
//					else if(strcmp("l",(char *)receive_str)==0){rspeed=500;lspeed=1000;}  //左转
//					else if(strcmp("tr",(char *)receive_str)==0){rspeed=750;lspeed=-750;}   //原地右转
//					else if(strcmp("tl",(char *)receive_str)==0){rspeed=-750;lspeed=750;} //原地左转
//					else if(strcmp("inb",(char *)receive_str)==0){setpos=700;}	   //抬高
//					else if(strcmp("outb",(char *)receive_str)==0){setpos=3500;}	   //抬低
//					else if(strcmp("bup",(char *)receive_str)==0){setpos+=100;}	   //缓慢上抬
//					else if(strcmp("bdown",(char *)receive_str)==0){setpos-=100;}	   //缓慢下抬
//					else if(strcmp("up",(char *)receive_str)==0){rspeed+=500;lspeed+=500;}//加速
//					else if(strcmp("down",(char *)receive_str)==0){rspeed-=500;lspeed-=500;}//减速
//					else if(strcmp("stop",(char *)receive_str)==0){rspeed=0;lspeed=0;}//停止							
				
						
					for(uart_byte_count=0;uart_byte_count<32;uart_byte_count++)receive_str[uart_byte_count]=0x00;
					uart_byte_count=0;    
				}				  
			else if((uart_byte_count>0)&&(uart_byte_count<=250))
				{
				   receive_str[uart_byte_count-1]=rec_data;
				   uart_byte_count++;
				}                		 
}




