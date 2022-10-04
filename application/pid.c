/**
  ******************************************************************************
  * @file    pid.c
  * @author  ������
  * @version V1.0.0
  * @date    2020/12/12
  * @brief   ��ÿһ��pid�ṹ�嶼Ҫ�Ƚ��к��������ӣ��ٽ��г�ʼ��
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Init.h"

PID_typedef Motor_PID[16] = {0};	//��ʼ��16���󽮵��
pid loop[20]={0};
/**********�޷�����**********/
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
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_Speed:�ٶȻ�PID  PID_Position��λ�û�PID
  * @param[in]      PID: 0: in_kp, 1: in_ki, 2:in_kd 3: out_kp, 4: out_ki, 5:out_kd
  * @param[in]      max_out: 0: �ڻ�pid������ 1���⻷pid������
  * @param[in]      max_idout: 0���ڻ�pid��������� 1���⻷���΢�����
	* @param[in]      deadband: �⻷����
  * @retval         none
  */

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      PID: 0: in_kp, 1: in_ki, 2:in_kd
	* @param[in]      max_out:   0:��������   1:����������
  * @retval         none
  */
void PID_motoinit(PID_typedef *pid, u8 mode,  fp32 PID[6], fp32 max_out[2], fp32 max_idout[2], fp32 deadband)
{
    //�ڻ���ʼ��
    pid->mode = mode;
    pid->in_Kp = PID[0];
    pid->in_Ki = PID[1];
    pid->in_Kd = PID[2];
    pid->in_max_out = max_out[0];
    pid->in_max_iout = max_idout[0];
    pid->in_Dbuf[0] = pid->in_Dbuf[1] = pid->in_Dbuf[2] = 0.0f;
    pid->in_err[0] = pid->in_err[1] = pid->in_err[2] = pid->in_Pout = pid->in_Iout = pid->in_Dout = pid->in_out = 0.0f;
    //�⻷��ʼ��
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
  * @brief  PID�ջ�
  * @param	speed:�����ٶȣ���βתת�٣�Ҫ�����rad/s��֮ǰ�õĶ���rad/min��
  * @param	Pos:����λ�ã�һ��Ҫ�ǽǶ�ֵ����Ҫת��Ȧ��ֵ����720��	  * @param	deadband:������ֻ����λ�û�����Ҫ������������ָ���Ƕ�����������Ϊ2������2������ڲ���������
  * @retval ����PID������ֵ
*/
fp32 PID_motoloop(PID_typedef *pid, motor_measure_t*ptr, fp32 setspeed, fp32 setpos)
{
    if(pid->mode == 1)
    {
        pid->out_max_out = setspeed;//���⻷�������������Ϊ�����ٶȣ���Ϊ�⻷������ľ����ٶ�ֵ
        pid->out_err[2] = pid->out_err[1];//����ʽPIDҪ�õ����ϴε����ֵ��λ��ʽPID���ã��˲��������õģ�
        pid->out_err[1] = pid->out_err[0];//�⻷��һ�����
        pid->out_set = setpos;          //����λ��
        pid->out_get = ptr->real_angle; //ʵ��λ�ã�Ҫ��������ֵ8192�����360�㣬������㣩
        pid->out_err[0] = pid->out_set - pid->out_get;

        if(fabs(pid->out_err[0]) < pid->deadband) //�����ڣ���������
            return 0;

        pid->out_Pout = pid->out_Kp * pid->out_err[0];                   //Pout
        LimitMax(pid->out_Pout, pid->out_max_out);
        pid->out_Iout += pid->out_Ki * pid->out_err[0];        //Iout
        pid->out_Dout = pid->out_Kd * (pid->out_err[0] - pid->out_err[1]); //Dout
        LimitMax(pid->out_Dout, pid->out_max_dout);
        pid->out_out = pid->out_Pout + pid->out_Iout + pid->out_Dout;
        LimitMax(pid->out_out, pid->out_max_out);
        
        pid->in_err[2] = pid->in_err[1];                                    //�ڻ������ϴ����
        pid->in_err[1] = pid->in_err[0];                                    //�ڻ���һ�����
        pid->in_set = pid->out_out;                                      //�⻷�����ֵ = �ڻ�������ֵ = �����ٶ�
        pid->in_get = ptr->speed / 60.0f;                /*�иĶ�*/       //ʵ���ٶ�(ע�����ｫת��ֵ�������rad/s,֮ǰ��rad/min)
        pid->in_err[0] = pid->in_set - pid->in_get;                         //�ٶ����
        pid->in_Pout =  pid->in_Kp * pid->in_err[0];
        pid->in_Iout += pid->in_Ki * pid->in_err[0];
        pid->in_Dout =  pid->in_Kd * (pid->in_err[0] - pid->in_err[1]);
        LimitMax(pid->in_Iout, pid->in_max_iout);
        pid->in_out = pid->in_Pout + pid->in_Iout + pid->in_Dout;
        LimitMax(pid->in_out, pid->in_max_out);
    }
    else
    {
        pid->in_err[2] = pid->in_err[1];                                  //�ڻ������ϴ����
        pid->in_err[1] = pid->in_err[0];                                  //�ڻ���һ�����
        pid->in_set = setspeed ;                                    //�⻷�����ֵ = �ڻ�������ֵ = �����ٶ�
        pid->in_get = ptr->speed / 60.0f;                /*�иĶ�*/       //ʵ���ٶ�(ע�����ｫת��ֵ�������rad/s,֮ǰ��rad/min)
        pid->in_err[0] = pid->in_set - pid->in_get;                         //�ٶ����
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
  * @retval ת�����
*/
float PID_turnloop_Z(float angle_speed,float real_angle,float turn_angle,float deadhead)
{
    //ת���⻷-�Ƕ� 	
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
