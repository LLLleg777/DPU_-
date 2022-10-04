#include "main.h"
#include "Init.h"

PID_motor m3508set[8] = {0};
PID_motor m2006set[8] = {0};
u8 Bound_Flag_Point[2]={1,1};
/**
* @name 	   motor_pid_set
* @brief  	 ���pid��ʼ��
* @param     DJ���ڻ�pid��1000��6��0  �⻷0.1��0��0.006
* @param     mode��1ʹ���⻷
* @param     deadband��10
*/
void motor_pid_set(void)
{
    static u8 i[2];
    static float PID[6] = {1000.0f, 6.0f, 0.0f, 0.1f, 0.0f, 0.006f};
    static float max_out[2] = {16000.0f, 400.0f};
    static float max_idout[2] = {2000.0f, 100.0f};

    for(i[0] = 0; i[0] < 4; i[0]++)
    {
        PID_motoinit(&Motor_PID[i[0]], 1, PID, max_out, max_idout, 0);//3508_1
        PID_motoinit(&Motor_PID[i[0] + 4], 1, PID, max_out, max_idout, 0); //3508_2
        PID_motoinit(&Motor_PID[i[0] + 8], 1, PID, max_out, max_idout, 0); //2006_1
        PID_motoinit(&Motor_PID[i[0] + 12], 1, PID, max_out, max_idout, 0); //2006_2
    }

    for(i[1] = 0; i[1] < 4; i[1]++)
    {
        m3508set[i[1]].setpos = 0;
        m3508set[i[1]].setspeed = 0;
        m3508set[i[1] + 4].setpos = 0;
        m3508set[i[1] + 4].setspeed = 0;

        m2006set[i[1]].setpos = 0;
        m2006set[i[1]].setspeed = 0;
        m2006set[i[1] + 4].setpos = 0;
        m2006set[i[1] + 4].setspeed = 0;
    }
}

/**
* @name 	    MOTOR_PID_CHANGE
* @brief
* @param    	MidΪ0Ϊ����
							MidΪ1����
							Mid Ϊ2 �ٶȻ���P
* @param
* @retval
*/
void MOTOR_PID_CHANGE(u8 Mid)
{
    static float OUT_KP[2]= {0.1,0.03};
    static float NUM_max[2]= {16000,8000};
    static float IN_KP[2]= {1000,1400};
    if(Mid==0)
    {
        Motor_PID[0].out_Kp=OUT_KP[0];
        Motor_PID[1].out_Kp=OUT_KP[0];
        Motor_PID[2].out_Kp=OUT_KP[0];
        Motor_PID[3].out_Kp=OUT_KP[0];
        Motor_PID[4].out_Kp=OUT_KP[0];
        Motor_PID[5].out_Kp=OUT_KP[0];
        Motor_PID[6].out_Kp=OUT_KP[0];
        Motor_PID[7].out_Kp=OUT_KP[0];
        Motor_PID[0].in_Kp =IN_KP[0];
        Motor_PID[1].in_Kp =IN_KP[0];
        Motor_PID[2].in_Kp =IN_KP[0];
        Motor_PID[3].in_Kp =IN_KP[0];
        Motor_PID[4].in_Kp =IN_KP[0];
        Motor_PID[5].in_Kp =IN_KP[0];
        Motor_PID[6].in_Kp =IN_KP[0];
        Motor_PID[7].in_Kp =IN_KP[0];
        Motor_PID[0].in_max_out=NUM_max[0];
        Motor_PID[1].in_max_out=NUM_max[0];
        Motor_PID[2].in_max_out=NUM_max[0];
        Motor_PID[3].in_max_out=NUM_max[0];
        Motor_PID[4].in_max_out=NUM_max[0];
        Motor_PID[5].in_max_out=NUM_max[0];
        Motor_PID[6].in_max_out=NUM_max[0];
        Motor_PID[7].in_max_out=NUM_max[0];
    }
    else if(Mid==1)
    {
        Motor_PID[0].out_Kp=OUT_KP[1];
        Motor_PID[1].out_Kp=OUT_KP[1];
        Motor_PID[2].out_Kp=OUT_KP[1];
        Motor_PID[3].out_Kp=OUT_KP[1];
        Motor_PID[4].out_Kp=OUT_KP[1];
        Motor_PID[5].out_Kp=OUT_KP[1];
        Motor_PID[6].out_Kp=OUT_KP[1];
        Motor_PID[7].out_Kp=OUT_KP[1];
        Motor_PID[0].in_Kp =IN_KP[0];
        Motor_PID[1].in_Kp =IN_KP[0];
        Motor_PID[2].in_Kp =IN_KP[0];
        Motor_PID[3].in_Kp =IN_KP[0];
        Motor_PID[4].in_Kp =IN_KP[0];
        Motor_PID[5].in_Kp =IN_KP[0];
        Motor_PID[6].in_Kp =IN_KP[0];
        Motor_PID[7].in_Kp =IN_KP[0];
        Motor_PID[0].in_max_out=NUM_max[1];
        Motor_PID[1].in_max_out=NUM_max[1];
        Motor_PID[2].in_max_out=NUM_max[1];
        Motor_PID[3].in_max_out=NUM_max[1];
        Motor_PID[4].in_max_out=NUM_max[1];
        Motor_PID[5].in_max_out=NUM_max[1];
        Motor_PID[6].in_max_out=NUM_max[1];
        Motor_PID[7].in_max_out=NUM_max[1];
    }
    else if(Mid==2)
    {
        Motor_PID[0].out_Kp=OUT_KP[0];
        Motor_PID[1].out_Kp=OUT_KP[0];
        Motor_PID[2].out_Kp=OUT_KP[0];
        Motor_PID[3].out_Kp=OUT_KP[0];
        Motor_PID[4].out_Kp=OUT_KP[0];
        Motor_PID[5].out_Kp=OUT_KP[0];
        Motor_PID[6].out_Kp=OUT_KP[0];
        Motor_PID[7].out_Kp=OUT_KP[0];
        Motor_PID[0].in_Kp =IN_KP[1];
        Motor_PID[1].in_Kp =IN_KP[1];
        Motor_PID[2].in_Kp =IN_KP[1];
        Motor_PID[3].in_Kp =IN_KP[1];
        Motor_PID[4].in_Kp =IN_KP[1];
        Motor_PID[5].in_Kp =IN_KP[1];
        Motor_PID[6].in_Kp =IN_KP[1];
        Motor_PID[7].in_Kp =IN_KP[1];
        Motor_PID[0].in_max_out=NUM_max[0];
        Motor_PID[1].in_max_out=NUM_max[0];
        Motor_PID[2].in_max_out=NUM_max[0];
        Motor_PID[3].in_max_out=NUM_max[0];
        Motor_PID[4].in_max_out=NUM_max[0];
        Motor_PID[5].in_max_out=NUM_max[0];
        Motor_PID[6].in_max_out=NUM_max[0];
        Motor_PID[7].in_max_out=NUM_max[0];
    }

}
/**
* @name 	    Stop_flat_ground(float Height_body)
* @brief      stop_flag_speed=0��ʼ�ٶȽ�С��
* @param    	stop_flag_speed=1�ٶȽϴ������
* @param
* @retval    control.s[3]==2 Ϊƽ�ز�̬�µ�λ��  control.s[3]==3Ϊ��б���µ�λ�� control.s[3]==1Ϊ��б�µ�λ��
*/
void Stop_flat_ground(float Height_body)
{	
//		static float step=170;
 //   static float Slope_Err[2]= {60,60}; //100 240
 //   static float Slope_step[2]= {170,170}; //100,170
    Speed=0.00f;
    /*************�Ը�������E�������ʼ��**************/
    if(control.s[3]==2&&(control.s[5]==2||control.s[5]==3||control.s[5]==1)&&control.s[4]==2)//ƽ�ع���
    {    MOTOR_PID_CHANGE(0);
				
        Xstart[0]=0;
        Xstart[1]=0;
        Xstart[2]=0;
        Xstart[3]=0;
        Xend[0]=Xstart[0];
        Xend[1]=Xstart[1];
        Xend[2]=Xstart[2];
        Xend[3]=Xstart[3];

        Hight[0]=0;
        Hight[3]=Hight[2]=Hight[1]=Hight[0];

        Zstart[0]=-Height_body;  //can2 id 1,2  ��1
        Zstart[1]=-Height_body;  //can2 id 3,4  ��2
        Zstart[2]=-Height_body+20;  //can1 id 1,2  ��3
        Zstart[3]=-Height_body+20;  //can1 id 3,4  ��4
        M3508_stop_postion(10,0.5,TS);
    }
    else if(control.s[3]==3)//�������
    {
        MOTOR_PID_CHANGE(0);
        Xstart[0]=0;
        Xstart[1]=0;
        Xstart[2]=0;
        Xstart[3]=0;

        Xend[0]=Xstart[0];
        Xend[1]=Xstart[1];
        Xend[2]=Xstart[2];
        Xend[3]=Xstart[3];

        Hight[0]=0;
        Hight[3]=Hight[2]=Hight[1]=Hight[0];

        Zstart[0]=-Height_body+50;  //can2 id 1,2  ��1
        Zstart[1]=-Height_body+50;  //can2 id 3,4  ��2
        Zstart[2]=-Height_body;  //can1 id 1,2  ��3
        Zstart[3]=-Height_body;  //can1 id 3,4  ��4

        M3508_stop_postion(15,0.5,TS);
    }
    else if(control.s[3]==1)//�������
    {
        MOTOR_PID_CHANGE(0);
        Xstart[0]=0;
        Xstart[1]=0;
        Xstart[2]=0;
        Xstart[3]=0;

        Xend[0]=Xstart[0];
        Xend[1]=Xstart[1];
        Xend[2]=Xstart[2];
        Xend[3]=Xstart[3];

        Hight[0]=0;
        Hight[3]=Hight[2]=Hight[1]=Hight[0];

        Zstart[0]=-Height_body;  //can2 id 1,2  ��1
        Zstart[1]=-Height_body;  //can2 id 3,4  ��2
        Zstart[2]=-Height_body+50;  //can1 id 1,2  ��3
        Zstart[3]=-Height_body+50;  //can1 id 3,4  ��4

        M3508_stop_postion(15,0.5,TS);
    }
//    else if(control.s[3]==2&&control.s[5]==3)//���ΰ����λ��
//    {
//        MOTOR_PID_CHANGE(0);

//        Xstart[0]=-(Slope_step[0]+Slope_Err[0])/2;
//        Xstart[1]=(Slope_step[0]-Slope_Err[0])/2;
//        Xstart[2]=(Slope_step[1]-Slope_Err[1])/2;
//        Xstart[3]=-(Slope_step[1]+Slope_Err[1])/2;

//        Xend[0]=Xstart[0] ; //+stepΪǰ��   ��1
//        Xend[1]=Xstart[1] ;                     //��2
//        Xend[2]=Xstart[2] ;       //��3
//        Xend[3]=Xstart[3] ;                     //��4
//        Hight[0]=0;
//        Hight[3]=Hight[2]=Hight[1]=Hight[0];


//        Zstart[0]=-Height_body+15;     //can2 id 1,2 ��1
//        Zstart[1]=-Height_body+15;     //can2 id 3,4 ��2
//        Zstart[2]=-Height_body+20;  //can1 id 1,2 ��3
//        Zstart[3]=-Height_body+20;  //can1 id 3,4 ��4
//   
//        M3508_stop_postion(20,0.5,TS);
//    }
}



void M3508_stop_postion(float setspeed,float arr,float Ts)
{
    static float M3508_Angle[16]= {0} ;
    static float M2006_Angle[16]= {0} ;

    /******************����E,������********************************/
    Leg_cycloid(0,&link[0],arr,Ts,Hight[0],Zstart[0],Xstart[0],Xend[0]);
    Leg_cycloid(1,&link[1],arr,Ts,Hight[1],Zstart[1],Xstart[1],Xend[1]);

    Leg_cycloid(2,&link[2],arr,Ts,Hight[2],Zstart[2],Xstart[2],Xend[2]);
    Leg_cycloid(3,&link[3],arr,Ts,Hight[3],Zstart[3],Xstart[3],Xend[3]);


    Inverse_leg(&leg[0],&link[0]);
    Inverse_leg(&leg[1],&link[1]);	 //E3,E4���˶�ѧ���
    Inverse_leg(&leg[2],&link[2]);
    Inverse_leg(&leg[3],&link[3]);	 //E3,E4���˶�ѧ���

    M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //�� can2 ID1,2 ��1
    M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

    M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //�� can2 ID3,4 ��3
    M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

    M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //ǰ can1 ID1,2 ��2
    M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

    M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //ǰ can1 ID3,4 ��4
    M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

    m2006set[0].setspeed=setspeed;
    m2006set[1].setspeed=setspeed;
    m2006set[2].setspeed=setspeed;
    m2006set[3].setspeed=setspeed;

    m3508set[0].setspeed=setspeed;
    m3508set[1].setspeed=setspeed;
    m3508set[2].setspeed=setspeed;
    m3508set[3].setspeed=setspeed;

    m2006set[0].setpos = M2006_Angle[0];
    m2006set[1].setpos = M2006_Angle[1];
    m2006set[2].setpos = M2006_Angle[2];
    m2006set[3].setpos = M2006_Angle[3];

    m3508set[0].setpos = M3508_Angle[0];
    m3508set[1].setpos = M3508_Angle[1];
    m3508set[2].setpos = M3508_Angle[2];
    m3508set[3].setpos = M3508_Angle[3];
}
/**
* @name 	    trot_flat_ground
* @brief  		�Խǲ�̬
* @param    	speed �����ٶ�
* @param      T[0]=T[3]=0 T[2]=T[1]=0.5  ˳��λ �����ǰ->��ǰ�к�
* @retval    control.s[3]==2 Ϊƽ�ز�̬�µ�λ��  control.s[3]==3Ϊ��б���µ�λ�� control.s[3]==1Ϊ��б�µ�λ��
*/

void trot_flat_ground(float step,float length_hight,float Height_body,float speed)
{
    MOTOR_PID_CHANGE(0);
    static float Deviation_Err=5;//8  ƽ�����
    static float Deviation_Err_s5=0;//8  ���̰����
    static float Slope_Err[2]= {60,60}; //60 60  ���ΰ��ͺ�λ��
    static float Slope_step[2]= {140,140}; //170,170 ���ΰ岽��
    /*************�Ը�������E�������ʼ��**************/

    if(control.s[5]==3)//���ΰ�ǰ���켣
    {
        Speed=0.0035;
			if(control.s[1]==4)
			{
				Xstart[0]=-(Slope_Err[0]+Slope_step[0]+38)/2.0f;
				Xstart[1]=-(Slope_Err[0]+Slope_step[0]-30)/2.0f;
        Xstart[2]=-(Slope_Err[1]+Slope_step[1]+38)/2.0f;
        Xstart[3]=-(Slope_Err[1]+Slope_step[1]-30)/2.0f;

        Xend[0]=Xstart[0] + Slope_step[0] +38 ; //+stepΪǰ��   ��1
        Xend[1]=Xstart[1] + Slope_step[0] -30  ;                     //��2
        Xend[2]=Xstart[2] + Slope_step[1] +38 ;       //��3
        Xend[3]=Xstart[3] + Slope_step[1] -30  ;                     //��4

			}
			else if(control.s[1]==3)
			{
				Xstart[0]=-(Slope_Err[0]+Slope_step[0]-38)/2.0f;
				Xstart[1]=-(Slope_Err[0]+Slope_step[0]+48)/2.0f;
        Xstart[2]=-(Slope_Err[1]+Slope_step[1]-38)/2.0f;
        Xstart[3]=-(Slope_Err[1]+Slope_step[1]+48)/2.0f;

        Xend[0]=Xstart[0] + Slope_step[0] -38 ; //+stepΪǰ��   ��1
        Xend[1]=Xstart[1] + Slope_step[0] +48  ;                     //��2
        Xend[2]=Xstart[2] + Slope_step[1] -38 ;       //��3
        Xend[3]=Xstart[3] + Slope_step[1] +48  ;                     //��4

			}
			else
			{
				Xstart[0]=-(Slope_Err[0]+Slope_step[0]-Deviation_Err_s5)/2.0f;
				Xstart[1]=-(Slope_Err[0]+Slope_step[0])/2.0f;
        Xstart[2]=-(Slope_Err[1]+Slope_step[1]-Deviation_Err_s5)/2.0f;
        Xstart[3]=-(Slope_Err[1]+Slope_step[1])/2.0f;

        Xend[0]=Xstart[0] + Slope_step[0] -Deviation_Err_s5 ; //+stepΪǰ��   ��1
        Xend[1]=Xstart[1] + Slope_step[0]-0  ;                     //��2
        Xend[2]=Xstart[2] + Slope_step[1] -Deviation_Err_s5 ;       //��3
        Xend[3]=Xstart[3] + Slope_step[1]-0  ;                     //��4
			}
        Hight[0]=length_hight-20;
        Hight[1]=length_hight-20;
        Hight[2]=length_hight-20;
        Hight[3]=length_hight-20;

        Zstart[0]=-Height_body+15;     //can2 id 1,2 ��1
        Zstart[1]=-Height_body+15;     //can2 id 3,4 ��2
        Zstart[2]=-Height_body+20;  //can1 id 1,2 ��3
        Zstart[3]=-Height_body+20;  //can1 id 3,4 ��4
    }

     if(control.s[3]==2&&(control.s[5]==2||control.s[5]==1)&&control.s[4]==2)//���������켣
    {
        Speed=speed;
			if(control.s[1]==4)
			{
        Xstart[2]=Xstart[0]=-(130-Deviation_Err)/2.0f;
        Xstart[3]=Xstart[1]=-100/2.0f;
        Xend[0]=Xstart[0] + 130 -Deviation_Err ; //+stepΪǰ��   ��1
        Xend[1]=Xstart[1] + 100  ;                     //��2
        Xend[2]=Xstart[2] + 130 -Deviation_Err ;       //��3
        Xend[3]=Xstart[3] + 100  ; 
				}
			else if(control.s[1]==3)
			{
        Xstart[2]=Xstart[0]=-(100-Deviation_Err)/2.0f;
        Xstart[3]=Xstart[1]=-130/2.0f;
        Xend[0]=Xstart[0] + 100 -Deviation_Err ; //+stepΪǰ��   ��1
        Xend[1]=Xstart[1] + 130  ;                     //��2
        Xend[2]=Xstart[2] + 100 -Deviation_Err ;       //��3
        Xend[3]=Xstart[3] + 130  ; 
			}
			else 
			{
        Xstart[2]=Xstart[0]=-(100-Deviation_Err)/2.0f;
        Xstart[3]=Xstart[1]=-100/2.0f;
        Xend[0]=Xstart[0] + 100 -Deviation_Err ; //+stepΪǰ��   ��1
        Xend[1]=Xstart[1] + 100  ;                     //��2
        Xend[2]=Xstart[2] + 100 -Deviation_Err ;       //��3
        Xend[3]=Xstart[3] + 100  ;                     //��4
			}
        Hight[0]=length_hight+0;
        Hight[1]=length_hight+0;
        Hight[2]=length_hight+0;
        Hight[3]=length_hight+0;

        Zstart[0]=-Height_body;     //can2 id 1,2 ��1
        Zstart[1]=-Height_body;     //can2 id 3,4 ��2
        Zstart[2]=-Height_body+20;  //can1 id 1,2 ��3
        Zstart[3]=-Height_body+20;  //can1 id 3,4 ��4
    }
    else if(control.s[3]==3&&control.s[5]==2)//���µ����켣 ����Ϊ50
    {
        Speed=speed;
//			if(control.s[1]==4)
//			{
//			
//        Xstart[0]=-(85)/2.0f;
//        Xstart[1]=-50/2.0f;
//        Xstart[2]=-(85+50)/2.0f;
//        Xstart[3]=-(50+50)/2.0f;

//        Xend[0]=Xstart[0] + 85  ; //+stepΪǰ��   ��1
//        Xend[1]=Xstart[1] + 50  ;                     //��2
//        Xend[2]=Xstart[2] + 85  ;       //��3
//        Xend[3]=Xstart[3] + 50  ;                     //��4

//			}
//			else if(control.s[1]==3)
//			{
//			
//        Xstart[0]=-(55)/2.0f;
//        Xstart[1]=-80/2.0f;
//        Xstart[2]=-(55+50)/2.0f;
//        Xstart[3]=-(80+50)/2.0f;

//        Xend[0]=Xstart[0] + 55  ; //+stepΪǰ��   ��1
//        Xend[1]=Xstart[1] + 80  ;                     //��2
//        Xend[2]=Xstart[2] + 55  ;       //��3
//        Xend[3]=Xstart[3] + 80  ;                     //��4

//			}
			
			
        Xstart[0]=-(30)/2.0f;
        Xstart[1]=-30/2.0f;
        Xstart[2]=-(0+30)/2.0f;
        Xstart[3]=-(0+30)/2.0f;

        Xend[0]=Xstart[0] + 30  ; //+stepΪǰ��   ��1
        Xend[1]=Xstart[1] + 30  ;                     //��2
        Xend[2]=Xstart[2] + 30  ;       //��3
        Xend[3]=Xstart[3] + 30  ;                     //��4
			
        Hight[0]=length_hight-30;
        Hight[1]=length_hight-30;
        Hight[2]=length_hight-10;
        Hight[3]=length_hight-10;

        Zstart[0]=-Height_body+40;     //can2 id 1,2 ��1
        Zstart[1]=-Height_body+40;     //can2 id 3,4 ��2
        Zstart[2]=-Height_body-10;  //can1 id 1,2 ��3
        Zstart[3]=-Height_body-10;  //can1 id 3,4 ��4
    }
    else if(control.s[3]==1&&control.s[5]==2)
    {
        Speed=0.0032;

        Xstart[0]=-(75-Deviation_Err)/2.0f;
        Xstart[1]=-75/2.0f;
        Xstart[2]=-(75-Deviation_Err)/2.0f;
        Xstart[3]=-(75)/2.0f;

        Xend[0]=Xstart[0] + 75 -Deviation_Err ; //+stepΪǰ��   ��1
        Xend[1]=Xstart[1] + 75  ;                     //��2
        Xend[2]=Xstart[2] + 75 -Deviation_Err ;       //��3
        Xend[3]=Xstart[3] + 75  ;                     //��4
                //��4

        Hight[0]=length_hight-0;
        Hight[1]=length_hight-0;
        Hight[2]=length_hight-30;
        Hight[3]=length_hight-30;

        Zstart[0]=-Height_body-10;     //can2 id 1,2 ��1
        Zstart[1]=-Height_body-10;     //can2 id 3,4 ��2
        Zstart[2]=-Height_body+40;  //can1 id 1,2 ��3
        Zstart[3]=-Height_body+40;  //can1 id 3,4 ��4
    }

    M3508_trot_postion(0.5,TS);
}

/**
* @name 	    trot_flat_ground
* @brief  		�Խǲ�̬
* @param    	speed �����ٶ�
* @param      T[0]=T[3]=0 T[2]=T[1]=0.5  ˳��λ �����ǰ->��ǰ�к�
* @retval    control.s[3]==2 Ϊƽ�ز�̬�µ�λ��  control.s[3]==3Ϊ��б���µ�λ�� control.s[3]==1Ϊ��б�µ�λ��
*/
void trot_behind_flat_ground(float step,float length_hight,float Height_body,float speed)
{
    MOTOR_PID_CHANGE(0);

    Speed=speed;
    if(control.s[3]==2)//ƽ������
    {
        /*************�Ը�������E�������ʼ��**************/
        Xstart[2]=Xstart[0]=step/2.0f;
        Xstart[3]=Xstart[1]=step/2.0f;

        Xend[0]=Xstart[0] - step ; //-step Ϊ����
        Xend[1]=Xstart[1] - step ;
        Xend[2]=Xstart[2] - step ;
        Xend[3]=Xstart[3] - step ;

        Hight[3]=length_hight;
        Hight[2]=length_hight;
        Hight[1]=length_hight;
        Hight[0]=length_hight;


        Zstart[0]=-Height_body+15;  //can2 id 1,2 ��1
        Zstart[1]=-Height_body+15;  //can2 id 3,4 ��2
        Zstart[2]=-Height_body;  //can1 id 1,2 ��3
        Zstart[3]=-Height_body;  //can1 id 3,4 ��4
    }
    if(control.s[3]==3)//����
    {
				Xstart[0]=(50)/2.0f;
        Xstart[1]=50/2.0f;
        Xstart[2]=(50)/2.0f;
        Xstart[3]=(50)/2.0f;
        /*************�Ը�������E�������ʼ��**************/
//        Xstart[0]=step/2.0f;
//        Xstart[1]=step/2.0f;
//        Xstart[2]=(step)/2.0f;
//        Xstart[3]=(step)/2.0f;

        Xend[0]=Xstart[0] - 50 ; //-step Ϊ����
        Xend[1]=Xstart[1] - 50 ;
        Xend[2]=Xstart[2] - 50 ;
        Xend[3]=Xstart[3] - 50 ;

        Hight[0]=length_hight-40;
        Hight[1]=length_hight-40;
        Hight[2]=length_hight-40;
        Hight[3]=length_hight-40;

        Zstart[0]=-Height_body+40;  //can2 id 1,2 ��1
        Zstart[1]=-Height_body+40;  //can2 id 3,4 ��2
        Zstart[2]=-Height_body-10;  //can1 id 1,2 ��3
        Zstart[3]=-Height_body-10;  //can1 id 3,4 ��4
    }
    M3508_trot_postion(0.5,TS);
}
/**
* @name 	    trot_flat_Right_ground
* @brief  		�Խǲ�̬��ת
* @param    	speed �����ٶ�
* @param      T[0]=T[3]=0 T[2]=T[1]=0.5  ˳��λ �����ǰ->��ǰ�к�
* @retval
*/
void trot_flat_Right_ground(float step,float length_hight,float Height_body,float speed)
{
 //   static float Deviation_Err=30;//8
    MOTOR_PID_CHANGE(0);
    /*************�Ը�������E�������ʼ��**************/
    static float Deviation_Err_s5=50;//8
    static float Slope_Err[2]= {60,60}; //100 240
    static float Slope_step[2]= {150,150}; //100,170
    /*************�Ը�������E�������ʼ��**************/
//    if(control.s[5]==1)
//    {
//        Speed=0.0025;

//				Xstart[0]=-(Slope_Err[0]+Slope_step[0])/2.0f;
//				Xstart[1]=-(Slope_Err[0]+Slope_step[0]-Deviation_Err_s5)/2.0f;
//        Xstart[2]=-(Slope_Err[1]+Slope_step[1])/2.0f;
//        Xstart[3]=-(Slope_Err[1]+Slope_step[1]-Deviation_Err_s5)/2.0f;

//        Xend[0]=Xstart[0] + Slope_step[0]  ; //+stepΪǰ��   ��1
//        Xend[1]=Xstart[1] + Slope_step[0] -Deviation_Err_s5 ;                     //��2
//        Xend[2]=Xstart[2] + Slope_step[1]  ;       //��3
//        Xend[3]=Xstart[3] + Slope_step[1] -Deviation_Err_s5 ;                     //��4

//        Hight[0]=length_hight-0;
//        Hight[1]=length_hight-0;
//        Hight[2]=length_hight-20;
//        Hight[3]=length_hight-20;

//        Zstart[0]=-Height_body+25;     //can2 id 1,2 ��1
//        Zstart[1]=-Height_body+25;     //can2 id 3,4 ��2
//        Zstart[2]=-Height_body+20;  //can1 id 1,2 ��3
//        Zstart[3]=-Height_body+20;  //can1 id 3,4 ��4
//    }
    if(control.s[3]==2&&control.s[5]==2)
    {
        Speed=speed;
        Xstart[0]=-step/2.0f;
        Xstart[1]=step/2.0f;
        Xstart[2]=-step/2.0f;
        Xstart[3]=step/2.0f;

        Xend[0]=Xstart[0] + step ;
        Xend[1]=Xstart[1] - step ;
        Xend[2]=Xstart[2] + step ;
        Xend[3]=Xstart[3] - step ;


        Hight[3]=length_hight;
        Hight[2]=length_hight;
        Hight[1]=length_hight;
        Hight[0]=length_hight;

        Zstart[0]=-Height_body;  //can2 id 1,2
        Zstart[1]=-Height_body;  //can2 id 3,4
        Zstart[2]=-Height_body;  //can1 id 1,2
        Zstart[3]=-Height_body;  //can1 id 3,4
    }
    if(control.s[3]==3&&control.s[5]==2)
    {
        Speed=speed;
				Xstart[0]=(30)/2.0f;
        Xstart[1]=30/2.0f;
        Xstart[2]=-(30)/2.0f;
        Xstart[3]=-(30)/2.0f;

        Xend[0]=Xstart[0] + 30 ;
        Xend[1]=Xstart[1] - 30 ;
        Xend[2]=Xstart[2] + 30 ;
        Xend[3]=Xstart[3] - 30 ;

        Hight[3]=length_hight-40;
        Hight[2]=length_hight-40;
        Hight[1]=length_hight-40;
        Hight[0]=length_hight-40;

        Zstart[0]=-Height_body+50;  //can2 id 1,2
        Zstart[1]=-Height_body+50;  //can2 id 3,4
        Zstart[2]=-Height_body;  //can1 id 1,2
        Zstart[3]=-Height_body;  //can1 id 3,4
    }
    if(control.s[3]==1&&control.s[5]==2)
    {
        Speed=speed;
				Xstart[0]=(20)/2.0f;
        Xstart[1]=20/2.0f;
        Xstart[2]=-(20)/2.0f;
        Xstart[3]=-(20)/2.0f;

        Xend[0]=Xstart[0] + step ;
        Xend[1]=Xstart[1] - step ;
        Xend[2]=Xstart[2] + step ;
        Xend[3]=Xstart[3] - step ;

        Hight[3]=length_hight-40;
        Hight[2]=length_hight-40;
        Hight[1]=length_hight-40;
        Hight[0]=length_hight-40;

        Zstart[0]=-Height_body;  //can2 id 1,2
        Zstart[1]=-Height_body;  //can2 id 3,4
        Zstart[2]=-Height_body+50;  //can1 id 1,2
        Zstart[3]=-Height_body+50;  //can1 id 3,4
    }
    M3508_trot_postion(0.5,TS);

}
/**
* @name 	    trot_flat_Left_ground
* @brief  		�Խǲ�̬��ת
* @param    	speed �����ٶ�
* @param      T[0]=T[3]=0 T[2]=T[1]=0.5  ˳��λ �����ǰ->��ǰ�к�
* @retval
*/
void trot_flat_Left_ground(float step,float length_hight,float Height_body,float speed)
{
//    static float Deviation_Err=-30;//8
//    static float Slope_Err[2]= {100,240}; //100 240
//    static float Slope_step[2]= {170,120}; //100,170
    MOTOR_PID_CHANGE(0);
    /*************�Ը�������E�������ʼ��**************/
    static float Deviation_Err_s5=50;//8
    static float Slope_Err[2]= {60,60}; //100 240
    static float Slope_step[2]= {150,150}; //100,170
    /*************�Ը�������E�������ʼ��**************/
//    if(control.s[5]==1)
//    {
//        Speed=0.0025;

//				Xstart[0]=-(Slope_Err[0]+Slope_step[0]-Deviation_Err_s5)/2.0f;
//				Xstart[1]=-(Slope_Err[0]+Slope_step[0])/2.0f;
//        Xstart[2]=-(Slope_Err[1]+Slope_step[1]-Deviation_Err_s5)/2.0f;
//        Xstart[3]=-(Slope_Err[1]+Slope_step[1])/2.0f;

//        Xend[0]=Xstart[0] + Slope_step[0] -Deviation_Err_s5 ; //+stepΪǰ��   ��1
//        Xend[1]=Xstart[1] + Slope_step[0]  ;                     //��2
//        Xend[2]=Xstart[2] + Slope_step[1] -Deviation_Err_s5 ;       //��3
//        Xend[3]=Xstart[3] + Slope_step[1]  ;                     //��4

//        Hight[0]=length_hight-0;
//        Hight[1]=length_hight-0;
//        Hight[2]=length_hight-20;
//        Hight[3]=length_hight-20;

//        Zstart[0]=-Height_body+25;     //can2 id 1,2 ��1
//        Zstart[1]=-Height_body+25;     //can2 id 3,4 ��2
//        Zstart[2]=-Height_body+20;  //can1 id 1,2 ��3
//        Zstart[3]=-Height_body+20;  //can1 id 3,4 ��4
//    }

    if(control.s[3]==2&&control.s[5]==2)
    {
			Speed =speed;
        /*************�Ը�������E�������ʼ��**************/
        Xstart[0]=step/2.0f;
        Xstart[1]=-step/2.0f;
        Xstart[2]=step/2.0f;
        Xstart[3]=-step/2.0f;

        Xend[0]=Xstart[0] - step ;
        Xend[1]=Xstart[1] + step ;
        Xend[2]=Xstart[2] - step ;
        Xend[3]=Xstart[3] + step ;
        Zstart[0]=-Height_body;  //can2 id 1,2
        Zstart[1]=-Height_body;  //can2 id 3,4
        Zstart[2]=-Height_body;  //can1 id 1,2
        Zstart[3]=-Height_body;  //can1 id 3,4
        Hight[3]=Hight[2]=Hight[1]=Hight[0]=length_hight;

    }
    if(control.s[3]==3&&control.s[5]==2)
    {
			Speed =speed;
        /*************�Ը�������E�������ʼ��**************/
				Xstart[0]=(30)/2.0f;
        Xstart[1]=-30/2.0f;
        Xstart[2]=(50)/2.0f;
        Xstart[3]=-(50)/2.0f;

        Xend[0]=Xstart[0] - 30 ;
        Xend[1]=Xstart[1] + 30 ;
        Xend[2]=Xstart[2] - 50 ;
        Xend[3]=Xstart[3] + 50 ;
        Zstart[0]=-Height_body+50;  //can2 id 1,2
        Zstart[1]=-Height_body+50;  //can2 id 3,4
        Zstart[2]=-Height_body;  //can1 id 1,2
        Zstart[3]=-Height_body;  //can1 id 3,4

        Hight[0]=length_hight-40;
        Hight[1]=length_hight-40;
        Hight[2]=length_hight-40;
        Hight[3]=length_hight-40;
    }
    if(control.s[3]==1&&control.s[5]==2)
    {
			Speed =speed;
        /*************�Ը�������E�������ʼ��**************/
				Xstart[0]=(30)/2.0f;
        Xstart[1]=30/2.0f;
        Xstart[2]=-(30)/2.0f;
        Xstart[3]=-(30)/2.0f;

        Xend[0]=Xstart[0] - step ;
        Xend[1]=Xstart[1] + step ;
        Xend[2]=Xstart[2] - step ;
        Xend[3]=Xstart[3] + step ;
        Zstart[0]=-Height_body;  //can2 id 1,2
        Zstart[1]=-Height_body;  //can2 id 3,4
        Zstart[2]=-Height_body+50;  //can1 id 1,2
        Zstart[3]=-Height_body+50;  //can1 id 3,4

        Hight[0]=length_hight-40;
        Hight[1]=length_hight-40;
        Hight[2]=length_hight-40;
        Hight[3]=length_hight-40;
    }
    M3508_trot_postion(0.5,TS);
}

/*
* @name 	    M3508_postion
* @brief  		�Խǲ�̬���λ��
* @param    	trot :arrռ�ձ�һ��ȡ0.5
* @param 			Walk :arrռ�ձ�һ��ȡ0.25
* @param      Bound:arrռ�ձ�һ��ȡ0.1
* @retval
*/
void M3508_trot_postion(float arr,float Ts)
{
    static float M3508_Angle[16]= {0} ;
    static float M2006_Angle[16]= {0} ;
    /******************����E,������********************************/
    Leg_cycloid(0,&link[0],arr,Ts,Hight[0],Zstart[0],Xstart[0],Xend[0]);
    Leg_cycloid(1,&link[1],arr,Ts,Hight[1],Zstart[1],Xstart[1],Xend[1]);
    Leg_cycloid(2,&link[2],arr,Ts,Hight[2],Zstart[2],Xstart[2],Xend[2]);
    Leg_cycloid(3,&link[3],arr,Ts,Hight[3],Zstart[3],Xstart[3],Xend[3]);

    Inverse_leg(&leg[0],&link[0]);
    Inverse_leg(&leg[1],&link[1]);	 //E3,E4���˶�ѧ���
    Inverse_leg(&leg[2],&link[2]);
    Inverse_leg(&leg[3],&link[3]);	 //E3,E4���˶�ѧ���

    M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //�� can2 ID1,2 ��1
    M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

    M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //�� can2 ID3,4 ��3
    M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

    M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //ǰ can1 ID1,2 ��2
    M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

    M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //ǰ can1 ID3,4 ��4
    M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

    if(link[0].point_E.x==Xend[0]||link[0].point_E.x==Xstart[0])
    {
        m2006set[0].setspeed  = 0;
        m2006set[1].setspeed  = 0;
        LED0=0;
    }
    else
    {
        m2006set[0].setspeed  = 150;
        m2006set[1].setspeed  = 150;
        LED0=1;
    }
    if(link[1].point_E.x==Xend[1]||link[1].point_E.x==Xstart[1])
    {
        m2006set[2].setspeed  = 0;
        m2006set[3].setspeed  = 0;
        LED0=0;

    }
    else
    {
        m2006set[2].setspeed  = 150;
        m2006set[3].setspeed  = 150;
        LED0=1;
    }
    if(link[2].point_E.x==Xend[2]||link[2].point_E.x==Xstart[2])
    {
        m3508set[0].setspeed = 0;
        m3508set[1].setspeed = 0;
        LED0=0;

    }
    else
    {
        m3508set[0].setspeed = 150;
        m3508set[1].setspeed = 150;
        LED0=1;
    }
    if(link[3].point_E.x==Xend[3]||link[3].point_E.x==Xstart[3])
    {
        m3508set[2].setspeed = 0;
        m3508set[3].setspeed = 0;
        LED0=0;
    }
    else
    {
        m3508set[2].setspeed = 150;
        m3508set[3].setspeed = 150;
        LED0=1;
    }

    m2006set[0].setpos = M2006_Angle[0];
    m2006set[1].setpos = M2006_Angle[1];
    m2006set[2].setpos = M2006_Angle[2];
    m2006set[3].setpos = M2006_Angle[3];

    m3508set[0].setpos = M3508_Angle[0];
    m3508set[1].setpos = M3508_Angle[1];
    m3508set[2].setpos = M3508_Angle[2];
    m3508set[3].setpos = M3508_Angle[3];
}


/**
* @name 	    BOUND_Inclined_Plane_flat_ground
* @brief  		б������Ծ
* @param    	speed �����ٶ�
* @param      T[0]=T[3]=0 T[2]=T[1]=0.5  ˳��λ �����ǰ->��ǰ�к�
* @retval
*/
void BOUND_Inclined_Plane_flat_ground(float Height_body_start,float Height_body_end)
{
    static float M2006_Angle[4]= {0};
    static float M3508_Angle[4]= {0};
    static float point_Speed[6]= {0,15,150,150,150,20};
    static u8 bound_flag_point2=1;
    LCD_Num(50, 240,bound_flag_point2,1, 16);
    /*******************��һ����***********************/
    if(Bound_Flag_Point[0]==1)
    {
        MOTOR_PID_CHANGE(1);
        Leg_Point(0,&link[0],-Height_body_start,-50);
        Leg_Point(1,&link[1],-Height_body_start,-50);
        Leg_Point(2,&link[2],-Height_body_start,-85);
        Leg_Point(3,&link[3],-Height_body_start,-85);//85

        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //�� can2 ID1,2 ��1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //�� can2 ID3,4 ��3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //ǰ can1 ID1,2 ��2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //ǰ can1 ID3,4 ��4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[1];
        m2006set[1].setspeed  = point_Speed[1];
        m2006set[2].setspeed  = point_Speed[1];
        m2006set[3].setspeed  = point_Speed[1];
        m3508set[0].setspeed  = point_Speed[1];
        m3508set[1].setspeed  = point_Speed[1];
        m3508set[2].setspeed  = point_Speed[1];
        m3508set[3].setspeed  = point_Speed[1];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];

        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if(control.s[0]==1)Bound_Flag_Point[0]=2;

    }
		else if(control.s[2]==3)
		{bound_flag_point2=1;}
    /**********************�ڶ�����*********************************/
    else if(Bound_Flag_Point[0]==2&&(control.s[2]==2||control.s[2]==1))
    {
        MOTOR_PID_CHANGE(0);//�ٶ�P�Ӵ�
        if(control.s[2]==3)bound_flag_point2=1;

        Leg_Point(0,&link[0],-Height_body_end,-140);
        Leg_Point(1,&link[1],-Height_body_end,-140);//˫Ŀ��
        Leg_Point(2,&link[2],-Height_body_end,-140);
        Leg_Point(3,&link[3],-Height_body_end,-140);

        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //�� can2 ID1,2 ��1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //�� can2 ID3,4 ��3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //ǰ can1 ID1,2 ��2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //ǰ can1 ID3,4 ��4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[2];
        m2006set[1].setspeed  = point_Speed[2];
        m2006set[2].setspeed  = point_Speed[2];
        m2006set[3].setspeed  = point_Speed[2];
        m3508set[0].setspeed  = point_Speed[2];
        m3508set[1].setspeed  = point_Speed[2];
        m3508set[2].setspeed  = point_Speed[2];
        m3508set[3].setspeed  = point_Speed[2];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m2006set[0].setpos-m2006[0].real_angle)<15)&&(fabs(m2006set[1].setpos-m2006[1].real_angle)<15)&&(control.s[2]==2||control.s[2]==1))Bound_Flag_Point[0]=3;
    }
    else if(Bound_Flag_Point[0]==3&&(control.s[2]==2||control.s[2]==1))
    {
        /*****************3��************************/
        MOTOR_PID_CHANGE(0);
        if(control.s[2]==3)bound_flag_point2=1;


        Leg_Point(0,&link[0],-Height_body_start,0);
        Leg_Point(1,&link[1],-Height_body_start,0);
        Leg_Point(2,&link[2],-Height_body_start,0);
        Leg_Point(3,&link[3],-Height_body_start,0);
        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //�� can2 ID1,2 ��1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //�� can2 ID3,4 ��3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //ǰ can1 ID1,2 ��2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //ǰ can1 ID3,4 ��4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[3];
        m2006set[1].setspeed  = point_Speed[3];
        m2006set[2].setspeed  = point_Speed[3];
        m2006set[3].setspeed  = point_Speed[3];
        m3508set[0].setspeed  = point_Speed[3];
        m3508set[1].setspeed  = point_Speed[3];
        m3508set[2].setspeed  = point_Speed[3];
        m3508set[3].setspeed  = point_Speed[3];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m3508set[1].setpos-m3508[1].real_angle)<20)&&(fabs(m3508set[0].setpos-m3508[0].real_angle)<20)&&(control.s[2]==2||control.s[2]==1))Bound_Flag_Point[0]=4;

    }
    /**********************��4����*********************************/
    else if(Bound_Flag_Point[0]==4&&(control.s[2]==2||control.s[2]==1))
    {
        MOTOR_PID_CHANGE(1);
        if(control.s[2]==3)bound_flag_point2=1;

        Leg_Point(0,&link[0],-Height_body_start-50,120);//-60,120
        Leg_Point(1,&link[1],-Height_body_start-50,120);
        Leg_Point(2,&link[2],-Height_body_start-50,120);
        Leg_Point(3,&link[3],-Height_body_start-50,120);
        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //�� can2 ID1,2 ��1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //�� can2 ID3,4 ��3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //ǰ can1 ID1,2 ��2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //ǰ can1 ID3,4 ��4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[4];
        m2006set[1].setspeed  = point_Speed[4];
        m2006set[2].setspeed  = point_Speed[4];
        m2006set[3].setspeed  = point_Speed[4];
        m3508set[0].setspeed  = point_Speed[4];
        m3508set[1].setspeed  = point_Speed[4];
        m3508set[2].setspeed  = point_Speed[4];
        m3508set[3].setspeed  = point_Speed[4];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m2006set[1].setpos-m2006[1].real_angle)<100)&&(fabs(m2006set[0].setpos-m2006[0].real_angle)<100)&&(control.s[2]==2||control.s[2]==1))
        {
            Bound_Flag_Point[0]=5;
        }
    }
    else if(Bound_Flag_Point[0]==5&&(control.s[2]==2||control.s[2]==1))
    {
        MOTOR_PID_CHANGE(1);
        if(control.s[2]==3)
            bound_flag_point2=1;

        Leg_Point(0,&link[0],-Height_body_start-50,-20);
        Leg_Point(1,&link[1],-Height_body_start-50,-20);
        Leg_Point(2,&link[2],-Height_body_start-50,-20);
        Leg_Point(3,&link[3],-Height_body_start-50,-20);
        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //�� can2 ID1,2 ��1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;
        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //�� can2 ID3,4 ��2
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //ǰ can1 ID1,2 ��3
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //ǰ can1 ID3,4 ��4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[5];
        m2006set[1].setspeed  = point_Speed[5];
        m2006set[2].setspeed  = point_Speed[5];
        m2006set[3].setspeed  = point_Speed[5];
        m3508set[0].setspeed  = point_Speed[5];
        m3508set[1].setspeed  = point_Speed[5];
        m3508set[2].setspeed  = point_Speed[5];
        m3508set[3].setspeed  = point_Speed[5];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m2006set[1].setpos-m2006[1].real_angle)<50)&&(fabs(m2006set[0].setpos-m2006[0].real_angle)<50)&&(control.s[2]==2||control.s[2]==1)) {
            Bound_Flag_Point[0]=6;
        }
    }
		else if(Bound_Flag_Point[0]==6)
		{if(control.s[2]==3)Bound_Flag_Point[0]=1;}

}


/**
* @name 	    BOUND_Inclined_Plane_flat_ground
* @brief  		ƽ��
* @param    	point_Speed ÿ���������ٶ�
* @param
* @retval
*/
void BOUND_Plane_flat_ground(float Height_body_start,float Height_body_end)
{
    static float M2006_Angle[4]= {0};
    static float M3508_Angle[4]= {0};
    static float point_Speed[6]= {0,15,150,150,150,20};
    static u8 bound_flag_point2=1;
    LCD_Num(50, 220,bound_flag_point2,1, 16);

    /*******************��һ����***********************/
    if(Bound_Flag_Point[1]==1)
    {
        MOTOR_PID_CHANGE(0);
        Leg_Point(0,&link[0],-Height_body_start,-50);
        Leg_Point(1,&link[1],-Height_body_start,-50);
        Leg_Point(2,&link[2],-Height_body_start,-50);
        Leg_Point(3,&link[3],-Height_body_start,-50);

        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //�� can2 ID1,2 ��1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //�� can2 ID3,4 ��3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //ǰ can1 ID1,2 ��2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //ǰ can1 ID3,4 ��4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[1];
        m2006set[1].setspeed  = point_Speed[1];
        m2006set[2].setspeed  = point_Speed[1];
        m2006set[3].setspeed  = point_Speed[1];
        m3508set[0].setspeed  = point_Speed[1];
        m3508set[1].setspeed  = point_Speed[1];
        m3508set[2].setspeed  = point_Speed[1];
        m3508set[3].setspeed  = point_Speed[1];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];

        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if(control.s[0]==1)Bound_Flag_Point[1]=2;

    }
		else if(control.s[2]==3)
		{bound_flag_point2=1;}
    /**********************�ڶ�����*********************************/
    else if(Bound_Flag_Point[1]==2&&(control.s[2]==2||control.s[2]==1))
    {
        MOTOR_PID_CHANGE(0);//�ٶ�P�Ӵ�
        if(control.s[2]==3)bound_flag_point2=1;

        Leg_Point(0,&link[0],-Height_body_end,-140);
        Leg_Point(1,&link[1],-Height_body_end,-140);
        Leg_Point(2,&link[2],-Height_body_end,-140);
        Leg_Point(3,&link[3],-Height_body_end,-140);

        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //�� can2 ID1,2 ��1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //�� can2 ID3,4 ��3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //ǰ can1 ID1,2 ��2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //ǰ can1 ID3,4 ��4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[2];
        m2006set[1].setspeed  = point_Speed[2];
        m2006set[2].setspeed  = point_Speed[2];
        m2006set[3].setspeed  = point_Speed[2];
        m3508set[0].setspeed  = point_Speed[2];
        m3508set[1].setspeed  = point_Speed[2];
        m3508set[2].setspeed  = point_Speed[2];
        m3508set[3].setspeed  = point_Speed[2];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m2006set[0].setpos-m2006[0].real_angle)<10)&&(fabs(m2006set[1].setpos-m2006[1].real_angle)<10)&&(control.s[2]==2||control.s[2]==1))Bound_Flag_Point[1]=3;
    }
    else if(Bound_Flag_Point[1]==3&&(control.s[2]==2||control.s[2]==1))
    {
        /*****************3��************************/
        MOTOR_PID_CHANGE(0);
        if(control.s[2]==3)bound_flag_point2=1;


        Leg_Point(0,&link[0],-Height_body_start,0);
        Leg_Point(1,&link[1],-Height_body_start,0);
        Leg_Point(2,&link[2],-Height_body_start,0);
        Leg_Point(3,&link[3],-Height_body_start,0);
        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //�� can2 ID1,2 ��1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //�� can2 ID3,4 ��3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //ǰ can1 ID1,2 ��2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //ǰ can1 ID3,4 ��4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[3];
        m2006set[1].setspeed  = point_Speed[3];
        m2006set[2].setspeed  = point_Speed[3];
        m2006set[3].setspeed  = point_Speed[3];
        m3508set[0].setspeed  = point_Speed[3];
        m3508set[1].setspeed  = point_Speed[3];
        m3508set[2].setspeed  = point_Speed[3];
        m3508set[3].setspeed  = point_Speed[3];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m3508set[1].setpos-m3508[1].real_angle)<20)&&(fabs(m3508set[0].setpos-m3508[0].real_angle)<20)&&(control.s[2]==2||control.s[2]==1))Bound_Flag_Point[1]=4;

    }
    /**********************��4����*********************************/
    else if(Bound_Flag_Point[1]==4&&(control.s[2]==2||control.s[2]==1))
    {
        MOTOR_PID_CHANGE(1);
        if(control.s[2]==3)bound_flag_point2=1;

        Leg_Point(0,&link[0],-Height_body_start-60,120);
        Leg_Point(1,&link[1],-Height_body_start-60,120);
        Leg_Point(2,&link[2],-Height_body_start-60,120);
        Leg_Point(3,&link[3],-Height_body_start-60,120);
        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //�� can2 ID1,2 ��1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //�� can2 ID3,4 ��3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //ǰ can1 ID1,2 ��2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //ǰ can1 ID3,4 ��4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[4];
        m2006set[1].setspeed  = point_Speed[4];
        m2006set[2].setspeed  = point_Speed[4];
        m2006set[3].setspeed  = point_Speed[4];
        m3508set[0].setspeed  = point_Speed[4];
        m3508set[1].setspeed  = point_Speed[4];
        m3508set[2].setspeed  = point_Speed[4];
        m3508set[3].setspeed  = point_Speed[4];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m2006set[1].setpos-m2006[1].real_angle)<20)&&(fabs(m2006set[0].setpos-m2006[0].real_angle)<20)&&(control.s[2]==2||control.s[2]==1)) {
            Bound_Flag_Point[1]=5;
        }
    }
    else if(Bound_Flag_Point[1]==5&&(control.s[2]==2||control.s[2]==1))
    {
        MOTOR_PID_CHANGE(1);
        if(bound_flag_point2==5&&control.s[2]==3)bound_flag_point2=1;

        Leg_Point(0,&link[0],-Height_body_start-60,-30);
        Leg_Point(1,&link[1],-Height_body_start-60,-30);
        Leg_Point(2,&link[2],-Height_body_start-60,-30);
        Leg_Point(3,&link[3],-Height_body_start-60,-30);
        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //�� can2 ID1,2 ��1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;
        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //�� can2 ID3,4 ��2
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //ǰ can1 ID1,2 ��3
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //ǰ can1 ID3,4 ��4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[5];
        m2006set[1].setspeed  = point_Speed[5];
        m2006set[2].setspeed  = point_Speed[5];
        m2006set[3].setspeed  = point_Speed[5];
        m3508set[0].setspeed  = point_Speed[5];
        m3508set[1].setspeed  = point_Speed[5];
        m3508set[2].setspeed  = point_Speed[5];
        m3508set[3].setspeed  = point_Speed[5];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m2006set[1].setpos-m2006[1].real_angle)<50)&&(fabs(m2006set[0].setpos-m2006[0].real_angle)<50)&&(control.s[2]==2||control.s[2]==1)) {
            Bound_Flag_Point[1]=6;
        }
    }
    else if(Bound_Flag_Point[1]==6&&(control.s[2]==2||control.s[2]==1))
    

{if(control.s[2]==3)Bound_Flag_Point[1]=1;}

}



