#ifndef __LEG_TASK_H
#define __LEG_TASK_H	 
#include "system.h"

#define pi 3.1415926585f
#define R_TO_D   (57.295780f)//Radian to Degree



#define Medium_length  380
#define link_AB        60
#define link_BD        130
#define link_AC        130
#define link_DE        220
#define link_CE        220
#define link_EF        0

#define link_AB2       60
#define link_BD2       130
#define link_AC2       130
#define link_DE2       230
#define link_CE2       230
#define link_EF2       0

#define M3508Reduction_Ratio 19

#define STOP   0
#define TROT   1
#define LEFT   2
#define BEHIND 3
#define RIGHT  4
#define BOUND  5
#define BOUND_PLANE 6
#define TROT_STOP   7
#define TROT_BEHIND  8
#define TROT_WARP    9

//#define LEFT   5
//#define BEHIND 6
//#define START  7
//#define CLIMB  8


typedef struct
{
 float x;
 float y;
 float z;
}
Point_typedef;

/*
 * Link ����
 */
typedef struct 
{
  Point_typedef point_A;

  Point_typedef point_B;	
	/*������������*/
  Point_typedef point_D;
	/*�����������*/
  Point_typedef point_E;
  /*ʵ����˹켣*/
  Point_typedef point_F;
	
	Point_typedef point_A2;

	Point_typedef point_B2;	
		/*������������*/
	Point_typedef point_D2;
	/*�����������*/
	Point_typedef point_E2;
	/*ʵ����˹켣*/
	Point_typedef point_F2;
	
	Point_typedef point_A3;
	
	/*��������*/
	Point_typedef point_B3;	
	/*������������*/
	Point_typedef point_D3;
	/*�����������*/
	Point_typedef point_E3;
	/*ʵ����˹켣*/
	Point_typedef point_F3;
	
	Point_typedef point_A4;

	Point_typedef point_B4;	
		/*������������*/
	Point_typedef point_D4;
	/*�����������*/
	Point_typedef point_E4;
	/*ʵ����˹켣*/
	Point_typedef point_F4;
	
}Link_typedef;

extern Link_typedef link[4];

typedef struct
{
float x1;
float y1;	
float angle1;
float angle2;
float real_angle[4];
float shank;
float shank1;
float fail;
float fail_angle1;
float Auxiliary_angle[4];


}
LEG_typedef;
extern LEG_typedef leg[4];

typedef struct
{
float hight;//
float cycle;//����
float arr;//ռ�ձ�
float X_start;
float X_end;
float Y_start;
float Y_end;	
float Z_start;
float Z_end;
float sigma;//�н�
float t;//��λʱ��
}
Footend_typedef;
extern Footend_typedef foot[4];


/****************************/
/**********/
extern u8 flag;
extern float TIME[4];
extern float Xstart[4];
extern float Xend[4];
extern float Hight[4];
extern float Zstart[4];
extern float Speed;
extern float TS;
/**********/
void Convert_rigid_Coordinate_Init(void);
void Coordinate_Ceneration(Link_typedef*link);
void Inverse_leg(LEG_typedef*leg,Link_typedef*link);
void Inverse_leg2(LEG_typedef*leg,Link_typedef*link);
void Leg_cycloid(u8 i,Link_typedef*link,float arr,float Ts,float hight,float Zstart,float Xstart,float Xend);
void Leg_Point(u8 i,Link_typedef*link,float Z_pos,float X_pos);

/****************************/

#endif
