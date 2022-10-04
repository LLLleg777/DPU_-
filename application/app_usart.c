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
        HAL_UART_Receive_IT(&huart1, (u8 *)&rec_data1, 1);//�ٴο������ڽ�������
    }
    if(huart->Instance == UART5)
    {
        Judge_Data(rec_data5);//��������
      HAL_UART_Receive_IT(&huart5, (u8 *)&rec_data5, 1);//�ٴο������ڽ�������
    }
}
void revice_Data(u8 ucData)
{
	static u8 ucRxBuffer1[250];
	static u8 ucRxCnt1 = 0;	
	ucRxBuffer1[ucRxCnt1++]=ucData;	//���յ������ݴ��뻺������
 
	if (ucRxBuffer1[0]!=0x4A) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt1=0;
		return;	
	}

	if (ucRxCnt1<8) {return;}//���ݲ���8�����򷵻�
	else
	{ 
		switch(ucRxBuffer1[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
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
		
		ucRxCnt1=0;//��ջ�����
	}
}

void Judge_Data(u8 ucData)
{
    static u8 ucRxBuffer[250];
    static u8 ucRxCnt = 0;
    ucRxBuffer[ucRxCnt++] = ucData;	//���յ������ݴ��뻺������
    if (ucRxBuffer[0] != 0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
    {
        ucRxCnt = 0;
        return;
    }
    if (ucRxCnt<11)
   {
        return;   //���ݲ���11�����򷵻�
    }
    else
    {
        switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
        {
        //memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
        case 0x52://���ٶ�
        {
            angle.Rollspeed= (short)(ucRxBuffer[3]<<8|ucRxBuffer[2])*2000/32768;
            angle.Pitchspeed=(short)(ucRxBuffer[5]<<8|ucRxBuffer[4])*2000/32768;
            angle.Yawspeed=  (short)(ucRxBuffer[7]<<8|ucRxBuffer[6])*2000/32768;
        }
        break;
        case 0x53://�Ƕ�
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
        ucRxCnt=0;//��ջ�����
    }
}



//����1����һ���ַ�
void uart1SendChar(u8 ch)
{
    while((USART1->SR & 0x40) == 0);

    USART1->DR = (u8) ch;
}
/****************************************************************************
* ��    ��: void uart1SendChars(u8 *str, u16 strlen)
* ��    �ܣ�����1����һ�ַ���
* ��ڲ�����*str�����͵��ַ���
            strlen���ַ�������
* ���ز�������
* ˵    ����
****************************************************************************/
void uart1SendChars(u8 *str, u16 strlen)
{
    u16 k = 0 ;

    do
    {
        uart1SendChar(*(str + k));    //ѭ������,ֱ���������
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
        num = -num;					    //תΪ����
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
* ��    ��:void Judge_wifi(u8 ucData)  
* ��    �ܣ�wifi��������
* ���ز�������
* ˵    ����
****************************************************************************/


void Judge_wifi(u8 ucData)  
{
   static u8 rec_data;
   static u8 receive_str[250];
   static u8 uart_byte_count = 0;
				rec_data =ucData;         //(USART1->DR) ��ȡ���յ�������
        if(rec_data=='S')		  	                         //�����S����ʾ��������Ϣ����ʼλ
				{
					uart_byte_count=0x01; 
				}

			else if(rec_data=='E')		                         //���E����ʾ��������Ϣ���͵Ľ���λ
				{
					if(strcmp("f",(char *)receive_str)==0){}	   //ǰ��
					else if(strcmp("b",(char *)receive_str)==0){}	   //����
					else if(strcmp("r",(char *)receive_str)==0){}  //��ת
//					else if(strcmp("l",(char *)receive_str)==0){rspeed=500;lspeed=1000;}  //��ת
//					else if(strcmp("tr",(char *)receive_str)==0){rspeed=750;lspeed=-750;}   //ԭ����ת
//					else if(strcmp("tl",(char *)receive_str)==0){rspeed=-750;lspeed=750;} //ԭ����ת
//					else if(strcmp("inb",(char *)receive_str)==0){setpos=700;}	   //̧��
//					else if(strcmp("outb",(char *)receive_str)==0){setpos=3500;}	   //̧��
//					else if(strcmp("bup",(char *)receive_str)==0){setpos+=100;}	   //������̧
//					else if(strcmp("bdown",(char *)receive_str)==0){setpos-=100;}	   //������̧
//					else if(strcmp("up",(char *)receive_str)==0){rspeed+=500;lspeed+=500;}//����
//					else if(strcmp("down",(char *)receive_str)==0){rspeed-=500;lspeed-=500;}//����
//					else if(strcmp("stop",(char *)receive_str)==0){rspeed=0;lspeed=0;}//ֹͣ							
				
						
					for(uart_byte_count=0;uart_byte_count<32;uart_byte_count++)receive_str[uart_byte_count]=0x00;
					uart_byte_count=0;    
				}				  
			else if((uart_byte_count>0)&&(uart_byte_count<=250))
				{
				   receive_str[uart_byte_count-1]=rec_data;
				   uart_byte_count++;
				}                		 
}




