#include "main.h"
#include "Init.h"


Remote_typedef control;

u8 nrf24l01_receive[nrf24l01_REC_NUM];     //接收缓存数组,最大USART_REC_LEN个字节 
u8 nrf24l01_count=0;

/**
* @name 	    Receive_control
* @brief  		2.4g接收
* @param  		
* @retval	
*/
void Receive_control(void)
{
	if(NRF24L01_RxPacket(nrf24l01_receive)==0)//2.4g接收
		{
			switch(nrf24l01_count)
			{
				case 0:
					if(nrf24l01_receive[0]==0x11)
						nrf24l01_count++;
					else
						nrf24l01_count=0;
				break;
					
				case 1:
					if(nrf24l01_receive[7]==0x18)
					{
						control.s[0]=nrf24l01_receive[1];
						control.s[1]=nrf24l01_receive[2];
						control.s[2]=nrf24l01_receive[3];
						control.s[3]=nrf24l01_receive[4];						
						control.s[4]=nrf24l01_receive[5];
						control.s[5]=nrf24l01_receive[6];						
//						a[2]=(int16_t)nrf24l01_receive[3]<<8|(int16_t)nrf24l01_receive[4];
//						a[3]=(int16_t)nrf24l01_receive[5]<<8|(int16_t)nrf24l01_receive[6];
					}
					else
						nrf24l01_count=0;
				break;
					
				default:
				 nrf24l01_count=0;
			  break;
			}
		}

}
