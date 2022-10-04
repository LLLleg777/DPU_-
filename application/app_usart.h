#ifndef __APP_USART_H
#define __APP_USART_H
#include "usart.h"
#include "system.h"
typedef struct
{
    float Roll;
    float Pitch;
    float Yaw;
    float Rollspeed;
    float Pitchspeed;
    float Yawspeed;
    float Rollplusspeed;
    float Pitchplusspeed;
    float Yawplusspeed;
} gyro;

typedef struct
{
	u8 R;
	u8 F;
	u16 H;
   
}  separation;

/********************************/


#define USART1_REC_NUM  			100  	//定义最大接收字节数 200
extern u8 uart_byte_count;          //uart_byte_count要小于USART_REC_LEN
extern u8 receive_str[USART1_REC_NUM];  


 

/*****************************/

void Judge_Data(u8 ucData);
void uart1_Display(short num,int ch);
void uart1SendChar(u8 ch);
void uart1SendChars(u8 *str, u16 strlen);
void Judge_wifi(u8 ucData);
void revice_Data(u8 ucData);

/**********************extern*********************************************/
extern u8 rec_data1;
extern u8 rec_data5;

extern gyro angle;
extern separation hab;

#endif


