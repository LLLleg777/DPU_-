/**
  ******************************************************************************
  * @file    pid.c
  * @author  王正浩
  * @version V1.0.0
  * @date    2020/12/12
  * @brief   对每一个pid结构体都要先进行函数的连接，再进行初始化
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Init.h"

PID_typedef Motor_PID[16] = {0};	//初始化16个大疆电机
pid loop[20]={0};
/**********限幅函数**********/
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_Speed:速度环PID  PID_Position：位置环PID
  * @param[in]      PID: 0: in_kp, 1: in_ki, 2:in_kd 3: out_kp, 4: out_ki, 5:out_kd
  * @param[in]      max_out: 0: 内环pid最大输出 1：外环pid最大输出
  * @param[in]      max_idout: 0：内环pid最大积分输出 1：外环最大微分输出
	* @param[in]      deadband: 外环死区
  * @retval         none
  */

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      PID: 0: in_kp, 1: in_ki, 2:in_kd
	* @param[in]      max_out:   0:总最大输出   1:积分最大输出
  * @retval         none
  */
void PID_motoinit(PID_typedef *pid, u8 mode,  fp32 PID[6], fp32 max_out[2], fp32 max_idout[2], fp32 deadband)
{
    //内环初始化
    pid->mode = mode;
    pid->in_Kp = PID[0];
    pid->in_Ki = PID[1];
    pid->in_Kd = PID[2];
    pid->in_max_out = max_out[0];
    pid->in_max_iout = max_idout[0];
    pid->in_Dbuf[0] = pid->in_Dbuf[1] = pid->in_Dbuf[2] = 0.0f;
    pid->in_err[0] = pid->in_err[1] = pid->in_err[2] = pid->in_Pout = pid->in_Iout = pid->in_Dout = pid->in_out = 0.0f;
    //外环初始化
    pid->out_Kp = PID[3];
    pid->out_Ki = PID[4];
    pid->out_Kd = PID[5];
    pid->out_max_out = max_out[1];
    pid->out_max_dout = max_idout[1];
    pid->deadband = deadband;
    pid->out_Dbuf[0] = pid->out_Dbuf[1] = pid->out_Dbuf[2] = 0.0f;
    pid->out_err[0] = pid->out_err[1] = pid->out_err[2] = pid->out_Pout = pid->out_Iout = pid->out_Dout = pid->out_out = 0.0f;
}

/**
  * @funNm  PID_motoloop
  * @brief  PID闭环
  * @param	speed:期望速度（是尾转转速，要换算成rad/s，之前用的都是rad/min）
  * @param	Pos:期望位置（一定要是角度值，如要转两圈，值就是720）	  * @param	deadband:死区（只有用位置环才需要死区，该死区指的是度数，如死区为2，即在2度误差内不做调整）
  * @retval 串级PID计算后的值
*/
fp32 PID_motoloop(PID_typedef *pid, motor_measure_t*ptr, fp32 setspeed, fp32 setpos)
{
    if(pid->mode == 1)
    {
        pid->out_max_out = setspeed;//将外环总输出限制设置为期望速度，因为外环计算出的就是速度值
        pid->out_err[2] = pid->out_err[1];//增量式PID要用到上上次的误差值，位置式PID不用，此参数是无用的；
        pid->out_err[1] = pid->out_err[0];//外环上一次误差
        pid->out_set = setpos;          //期望位置
        pid->out_get = ptr->real_angle; //实际位置（要将编码器值8192换算成360°，方便计算）
        pid->out_err[0] = pid->out_set - pid->out_get;

        if(fabs(pid->out_err[0]) < pid->deadband) //死区内，不做计算
            return 0;

        pid->out_Pout = pid->out_Kp * pid->out_err[0];                   //Pout
        LimitMax(pid->out_Pout, pid->out_max_out);
        pid->out_Iout += pid->out_Ki * pid->out_err[0];        //Iout
        pid->out_Dout = pid->out_Kd * (pid->out_err[0] - pid->out_err[1]); //Dout
        LimitMax(pid->out_Dout, pid->out_max_dout);
        pid->out_out = pid->out_Pout + pid->out_Iout + pid->out_Dout;
        LimitMax(pid->out_out, pid->out_max_out);
        
        pid->in_err[2] = pid->in_err[1];                                    //内环的上上次误差
        pid->in_err[1] = pid->in_err[0];                                    //内环上一次误差
        pid->in_set = pid->out_out;                                      //外环的输出值 = 内环的输入值 = 期望速度
        pid->in_get = ptr->speed / 60.0f;                /*有改动*/       //实际速度(注意这里将转速值处理成了rad/s,之前是rad/min)
        pid->in_err[0] = pid->in_set - pid->in_get;                         //速度误差
        pid->in_Pout =  pid->in_Kp * pid->in_err[0];
        pid->in_Iout += pid->in_Ki * pid->in_err[0];
        pid->in_Dout =  pid->in_Kd * (pid->in_err[0] - pid->in_err[1]);
        LimitMax(pid->in_Iout, pid->in_max_iout);
        pid->in_out = pid->in_Pout + pid->in_Iout + pid->in_Dout;
        LimitMax(pid->in_out, pid->in_max_out);
    }
    else
    {
        pid->in_err[2] = pid->in_err[1];                                  //内环的上上次误差
        pid->in_err[1] = pid->in_err[0];                                  //内环上一次误差
        pid->in_set = setspeed ;                                    //外环的输出值 = 内环的输入值 = 期望速度
        pid->in_get = ptr->speed / 60.0f;                /*有改动*/       //实际速度(注意这里将转速值处理成了rad/s,之前是rad/min)
        pid->in_err[0] = pid->in_set - pid->in_get;                         //速度误差
        pid->in_Pout =  pid->in_Kp * pid->in_err[0];
        pid->in_Iout += pid->in_Ki * pid->in_err[0];
        pid->in_Dout =  pid->in_Kd * (pid->in_err[0] - pid->in_err[1]);
        LimitMax(pid->in_Iout, pid->in_max_iout);
        pid->in_out = pid->in_Pout + pid->in_Iout + pid->in_Dout;
        LimitMax(pid->in_out, pid->in_max_out);
    }
    return pid->in_out;
}

//
/**
  * @funNm  PID_turnloop
  * @brief  
  * @param
  * @param
  * @param
  * @retval 转向环输出
*/
float PID_turnloop_Z(float angle_speed,float real_angle,float turn_angle,float deadhead)
{
    //转向环外环-角度 	
		loop[1].err_last=loop[1].err;
    if(fabs(loop[1].err)>180)
    loop[1].err-=360;
    if(fabs(loop[1].err)<-180)
    loop[1].err+=360;
    loop[1].err_last=loop[1].err;
    loop[1].err=turn_angle-real_angle;
		
		if(fabs(loop[1].err)<=deadhead)
			return 0;
		else
		{
    loop[1].Pout=Kp2*loop[1].err;
		loop[1].Dout=Kd2*(loop[1].err-loop[1].err_last) ;
    loop[1].output=loop[1].Pout+loop[1].Dout;
		
		loop[1].output=loop[1].output>angle_speed?angle_speed:(loop[1].output<(-angle_speed)?(-angle_speed):loop[1].output);

			return loop[1].output;
    }
}
