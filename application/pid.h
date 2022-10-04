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
    //速度环即内环参数
    fp32 in_Kp;  
    fp32 in_Ki;
    fp32 in_Kd;

    fp32 in_max_out;  //最大输出
    fp32 in_max_iout; //最大积分输出

    fp32 in_set;     //内环期望
    fp32 in_get;     //内环输入

    fp32 in_out;     //内环总输出
    fp32 in_Pout;    //比例项输出
    fp32 in_Iout;    //积分项输出
    fp32 in_Dout;    //微分项输出
    fp32 in_Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 in_err[3]; //误差项 0最新 1上一次 2上上次
	  //位置环即内环
	  fp32 out_Kp;
    fp32 out_Ki;
    fp32 out_Kd;

    fp32 out_max_out;  //最大输出
    fp32 out_max_dout; //最大积分输出
    fp32 deadband;	//只有外环设置了死区 即死区内不计算
		
    fp32 out_set;     //期望位置
    fp32 out_get;     //位置输入

    fp32 out_out;     //外环总输出
    fp32 out_Pout;    //比例项输出
    fp32 out_Iout;    //积分项输出
    fp32 out_Dout;    //微分项输出
    fp32 out_Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 out_err[3]; //误差项 0最新 1上一次 2上上次

} PID_typedef;


typedef struct
{
    u8 mode;
    //速度环即内环参数
    fp32 in_Kp;
    fp32 in_Ki;
    fp32 in_Kd;

    fp32 in_max_out;  //最大输出
    fp32 in_max_iout; //最大积分输出

    fp32 in_set;     //内环期望
    fp32 in_get;     //内环输入
    fp32 in_out;     //内环总输出
    fp32 in_Pout;    //比例项输出
    fp32 in_Iout;    //积分项输出
    fp32 in_Dout;    //微分项输出
    fp32 in_err[2]; //误差项 0最新 1上一次
    fp32 Dbuf;     //本次误差减去上次误差
    fp32 deadband;    //死区

} PID_5065typedef;

typedef struct
{
	float SetLine;  //期望角度
	float ActualLine;//实际角度
	float SetYaw;  //期望角度
	float ActualYaw;//实际角度
	float SetAngleSpeed;  //期望角速度
	float ActualAngleSpeed; //实际角速度

	float SetPos;  //期望位置
	float ActualPos;//实际位置
	float SetSpeed;  //期望速度
	float ActualSpeed; //实际速度
	float err;       //误差
	float err_last;  //上次误差
  float err_last2;//上上次误差
  float Pout;
	float Kp;
	float Ki;
	float Kd;
  float Iout;
  float Dout;	
	float output;//总输出
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
extern PID_typedef Motor_PID[16];			//初始化了8个电机

#endif



