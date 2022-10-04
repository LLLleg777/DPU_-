/**
  ******************************************************************************
  * @file		 pid.h
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#ifndef _PID_H
#define _PID_H
#include "system.h"
#include "app_can.h"
#define PI 3.1415926535f

#define Kp1 -0.8
#define Ki1 0
#define Kd1 -0.1

#define Kp2 3.0
#define Ki2 0
#define Kd2 1.0
 
#define Kp3 -2.5
#define Ki3 0
#define Kd3 -0.1

#define Kp4 -1
#define Ki4 0
#define Kd4 -0.05
typedef struct
{
    u8 mode;
    //�ٶȻ����ڻ�����
    fp32 in_Kp;  
    fp32 in_Ki;
    fp32 in_Kd;

    fp32 in_max_out;  //������
    fp32 in_max_iout; //���������

    fp32 in_set;     //�ڻ�����
    fp32 in_get;     //�ڻ�����

    fp32 in_out;     //�ڻ������
    fp32 in_Pout;    //���������
    fp32 in_Iout;    //���������
    fp32 in_Dout;    //΢�������
    fp32 in_Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 in_err[3]; //����� 0���� 1��һ�� 2���ϴ�
	  //λ�û����ڻ�
	  fp32 out_Kp;
    fp32 out_Ki;
    fp32 out_Kd;

    fp32 out_max_out;  //������
    fp32 out_max_dout; //���������
    fp32 deadband;	//ֻ���⻷���������� �������ڲ�����
		
    fp32 out_set;     //����λ��
    fp32 out_get;     //λ������

    fp32 out_out;     //�⻷�����
    fp32 out_Pout;    //���������
    fp32 out_Iout;    //���������
    fp32 out_Dout;    //΢�������
    fp32 out_Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 out_err[3]; //����� 0���� 1��һ�� 2���ϴ�

} PID_typedef;


typedef struct
{
    u8 mode;
    //�ٶȻ����ڻ�����
    fp32 in_Kp;
    fp32 in_Ki;
    fp32 in_Kd;

    fp32 in_max_out;  //������
    fp32 in_max_iout; //���������

    fp32 in_set;     //�ڻ�����
    fp32 in_get;     //�ڻ�����
    fp32 in_out;     //�ڻ������
    fp32 in_Pout;    //���������
    fp32 in_Iout;    //���������
    fp32 in_Dout;    //΢�������
    fp32 in_err[2]; //����� 0���� 1��һ��
    fp32 Dbuf;     //��������ȥ�ϴ����
    fp32 deadband;    //����

} PID_5065typedef;

typedef struct
{
	float SetLine;  //�����Ƕ�
	float ActualLine;//ʵ�ʽǶ�
	float SetYaw;  //�����Ƕ�
	float ActualYaw;//ʵ�ʽǶ�
	float SetAngleSpeed;  //�������ٶ�
	float ActualAngleSpeed; //ʵ�ʽ��ٶ�

	float SetPos;  //����λ��
	float ActualPos;//ʵ��λ��
	float SetSpeed;  //�����ٶ�
	float ActualSpeed; //ʵ���ٶ�
	float err;       //���
	float err_last;  //�ϴ����
  float err_last2;//���ϴ����
  float Pout;
	float Kp;
	float Ki;
	float Kd;
  float Iout;
  float Dout;	
	float output;//�����
	float deadband;	
}pid;
/************************************/
void PID_motoinit(PID_typedef *pid, u8 mode, fp32 PID[6], fp32 max_out[2], fp32 max_idout[2], fp32 deadband);
fp32 PID_motoloop(PID_typedef *pid, motor_measure_t*ptr, fp32 setspeed, fp32 setpos);
float PID_turnloop_Z(float angle_speed,float real_angle,float turn_angle,float deadhead);
float PID_turn_angle_Z(float real_angle_speed,float turn_angle_speed);
float PID_turnloop_X(float real_angle,float turn_angle);
float PID_turn_angle_X(float real_angle_speed,float turn_angle_speed);
/*****************************extern***********************************/
extern PID_typedef Motor_PID[16];			//��ʼ����8�����

#endif



