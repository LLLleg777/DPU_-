#ifndef __MOVE_H
#define __MOVE_H
#include "system.h"
#define RESET_ANGLE   32
#define RESET_ANGLE2  36

typedef struct
{
    float num;//电流
    float setspeed;//速度
    float setpos;//位置
} PID_motor;

typedef struct
{
    int adjust;
    int anglespeed;
    int angle;
} PID_angle;


/******************声明函数*******************************/
void motor_pid_set(void);
fp32 AccelS(int acc_t, fp32 speed);//S型加速曲线

void BOUND_Inclined_Plane_flat_ground(float Height_body_start,float Height_body_end);
void BOUND_Plane_flat_ground(float Height_body_start,float Height_body_end);
void Stop_flat_ground(float Height_body);
void trot_flat_ground(float step,float length_hight,float Height_body,float speed);
void trot_behind_flat_ground(float step,float length_hight,float Height_body,float speed);
void trot_flat_Left_ground(float step,float length_hight,float Height_body,float speed);
void trot_flat_Right_ground(float step,float length_hight,float Height_body,float speed);
void MOTOR_PID_CHANGE(u8 Mid);
void M3508_trot_postion(float arr,float Ts);
void Trot_start_flat_ground(float Height_body,float length_hight,float step);
void Trot_start_behind_ground(float Height_body,float length_hight,float step);
void trot_Warping_plate(float step,float length_hight,float Height_body,float speed);//跷跷板
void M3508_stop_postion(float setspeed,float arr,float Ts);
/*********************************extern************************************/
extern PID_motor m3508set[8];
extern PID_motor m2006set[8];
extern u8 Bound_Flag_Point[2];





#endif
