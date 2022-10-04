#include "main.h"
#include "Init.h"


LEG_typedef     leg[4]= {0};
Link_typedef    link[4]={0};
Footend_typedef foot[4]={0};
/*
  * @funNm  *坐标起始
  * @brief  t8为左后退，t9为右后退，t10为左前退，t11为右前退，
*/
/*****坐标起始量********/
u8 flag=0;
float TS=1.0f;
float TIME[4]  ={0};
float Zstart[4]={0};
float Xstart[4]={0};
float Xend[4]  ={0};
float Hight[4] ={0};
float Speed=0.00;
/*
  * @funNm  World_Coordinate_Init(Link_typedef*link)
  * @brief  坐标初始化
*/
void Convert_rigid_Coordinate_Init(void)
{
	Coordinate_Ceneration(&link[0]);
	Coordinate_Ceneration(&link[1]);
	Coordinate_Ceneration(&link[2]);
	Coordinate_Ceneration(&link[3]);
}
/*
  * @funNm  Coordinate_Ceneration(Link_typedef*link)
  * @brief  生成AB身体坐标
*/
void Coordinate_Ceneration(Link_typedef*link)
{	
  link->point_A.x  =-link_AB/2;
	link->point_B.x  =link_AB/2;
  link->point_A.z =0;
	link->point_B.z =0;
}


/*
  * @funNm  Leg_cycloid(Footend_typedef*foot)
  * @brief  足端的轨迹方程(摆线方程)
*/
void Leg_cycloid(u8 i,Link_typedef*link,float arr,float Ts,float hight,float Zstart,float Xstart,float Xend)
{
	static float sigma;

	
switch(i)
	{
		case 0 : 
							if(TIME[0]<arr*Ts)
							{	
								sigma = 2*pi*TIME[0]/(arr*Ts);
								link->point_E.x = (Xend -Xstart)*(sigma-sin(sigma))/(2*pi) + Xstart;
								link->point_E.z = hight*(1-cos(sigma))/2+Zstart;										
							}
							else if(TIME[0]>=arr*Ts)
							{
								sigma = 2*pi*(TIME[0]-arr*Ts)/((Ts-arr)*Ts);
								link->point_E.x = -(Xend -Xstart)*(sigma-sin(sigma))/(2*pi) + Xend;
								link->point_E.z = Zstart;								
							}
							break;
		case 1 :
							if(TIME[1]<arr*Ts)
							{	
								sigma = 2*pi*TIME[1]/(arr*Ts);
								link->point_E.x = (Xend -Xstart)*(sigma-sin(sigma))/(2*pi) + Xstart;
								link->point_E.z = hight*(1-cos(sigma))/2+Zstart;										
							}
							else if(TIME[1]>=arr*Ts)
							{
								sigma = 2*pi*(TIME[1]-arr*Ts)/((Ts-arr)*Ts);
								link->point_E.x = -(Xend -Xstart)*(sigma-sin(sigma))/(2*pi) + Xend;
								link->point_E.z = Zstart;								
							}	
							break;		
    case 2 :				
							if(TIME[2]<arr*Ts)
							{	
								sigma = 2*pi*TIME[2]/(arr*Ts);
								link->point_E.x = (Xend -Xstart)*(sigma-sin(sigma))/(2*pi) + Xstart;
								link->point_E.z = hight*(1-cos(sigma))/2+Zstart;										
							}
							else if(TIME[2]>=arr*Ts)
							{
								sigma = 2*pi*(TIME[2]-arr*Ts)/((Ts-arr)*Ts);
								link->point_E.x = -(Xend -Xstart)*(sigma-sin(sigma))/(2*pi) + Xend;
								link->point_E.z = Zstart;								
							}
							break;		
		case 3:
							if(TIME[3]<arr*Ts)
							{	
								sigma = 2*pi*TIME[3]/(arr*Ts);
								link->point_E.x = (Xend -Xstart)*(sigma-sin(sigma))/(2*pi) + Xstart;
								link->point_E.z = hight*(1-cos(sigma))/2+Zstart;										
							}
							else if(TIME[3]>=arr*Ts)
							{
								sigma = 2*pi*(TIME[3]-arr*Ts)/((Ts-arr)*Ts);
								link->point_E.x = -(Xend -Xstart)*(sigma-sin(sigma))/(2*pi) + Xend;
								link->point_E.z = Zstart;								
							}	
							break;
	}
}


/*
  * @funNm  Leg_cycloid(Footend_typedef*foot)
  * @brief  足端的轨迹方程(摆线方程)
*/
void Leg_Point(u8 i,Link_typedef*link,float Z_pos,float X_pos)
{
switch(i)
	{
		case 0 : 					
							link->point_E.x =X_pos;
							link->point_E.z =Z_pos;											
							break;
		case 1 :
							link->point_E.x =X_pos;
							link->point_E.z =Z_pos;												
							break;		
    case 2 :				
							link->point_E.x =X_pos;
							link->point_E.z =Z_pos;											
							break;		
		case 3:
							link->point_E.x =X_pos;
							link->point_E.z =Z_pos;					
							break;
	}
}
/*
  * @funNm  Leg_cycloid(Footend_typedef*foot)
  * @brief  足端的轨迹方程(摆线方程)
*/
void Leg_cycloid_YAW(u8 i,Footend_typedef*foot,Link_typedef*link,float arr,float Ts,float hight,float Zstart,float Xstart,float Xend,float yaw_path)
{
	static float sigma;
	foot->arr = arr;
	foot->t = Ts;
	foot->hight=hight;
	foot->X_end=Xend;
	foot->X_start=Xstart;
	foot->Z_start=Zstart;
	
switch(i)
	{
		case 0 : 
							if(TIME[0]<foot->arr*foot->t)
							{	
								sigma = 2*pi*TIME[0]/(foot->arr*foot->t);
								link->point_E.x = (foot->X_end - foot->X_start)*(sigma-sin(sigma))/(2*pi) + foot->X_start;
								link->point_E.z = foot->hight*(1-cos(sigma))/2+foot->Z_start;								
							}
								
							else if(TIME[0]>=foot->arr*foot->t)
							{
								sigma = 2*pi*(TIME[0]-foot->arr*foot->t)/((Ts-foot->arr)*foot->t);
								link->point_E.x = -(foot->X_end - foot->X_start)*(sigma-sin(sigma))/(2*pi) + foot->X_end ;
								link->point_E.z = foot->Z_start;
							}
link->point_E3.x = cos(yaw_path+atan((link->point_E.z-Zstart)/(link->point_E.x-Xstart)))*(sqrt((pow(link->point_E.x,2.f)-pow(Xstart,2.f))+(pow(link->point_E.x,2.f)-pow(Zstart,2.f))));
link->point_E3.z = sin(yaw_path+atan((link->point_E.z-Zstart)/(link->point_E.x-Xstart)))*(sqrt((pow(link->point_E.x,2.f)-pow(Xstart,2.f))+(pow(link->point_E.x,2.f)-pow(Zstart,2.f))));									
							
							break;
		case 1 :
							if(TIME[1]<foot->arr*foot->t)
							{	
								sigma = 2*pi*TIME[1]/(foot->arr*foot->t);
								link->point_E2.x = (foot->X_end - foot->X_start)*(sigma-sin(sigma))/(2*pi) + foot->X_start;
								link->point_E2.z = foot->hight*(1-cos(sigma))/2+foot->Z_start;		
							}
							else if(TIME[1]>=foot->arr*foot->t)
							{
								sigma = 2*pi*(TIME[1]-foot->arr*foot->t)/((Ts-foot->arr)*foot->t);
								link->point_E2.x = -(foot->X_end - foot->X_start)*(sigma-sin(sigma))/(2*pi) + foot->X_end;
								link->point_E2.z = foot->Z_start;			
							}		
link->point_E4.x = cos(yaw_path+atan((link->point_E2.z-Zstart)/(link->point_E2.x-Xstart)))*(sqrt((pow(link->point_E2.x,2.f)-pow(Xstart,2.f))+(pow(link->point_E2.x,2.f)-pow(Zstart,2.f))));
link->point_E4.z = sin(yaw_path+atan((link->point_E2.z-Zstart)/(link->point_E2.x-Xstart)))*(sqrt((pow(link->point_E2.x,2.f)-pow(Xstart,2.f))+(pow(link->point_E2.x,2.f)-pow(Zstart,2.f))));							
							break;		
    case 2 :				
							if(TIME[2]<foot->arr*foot->t)
							{	
								sigma = 2*pi*TIME[2]/(foot->arr*foot->t);
								link->point_E.x = (foot->X_end - foot->X_start)*(sigma-sin(sigma))/(2*pi) + foot->X_start;
								link->point_E.z = foot->hight*(1-cos(sigma))/2+foot->Z_start;				
							}
							else if(TIME[2]>=foot->arr*foot->t)
							{
								sigma = 2*pi*(TIME[2]-foot->arr*foot->t)/((Ts-foot->arr)*foot->t);
								link->point_E.x = -(foot->X_end - foot->X_start)*(sigma-sin(sigma))/(2*pi) + foot->X_end ;
								link->point_E.z = foot->Z_start;			
							}	
link->point_E3.x = cos(yaw_path+atan((link->point_E.z-Zstart)/(link->point_E.x-Xstart)))*(sqrt((pow(link->point_E.x,2.f)-pow(Xstart,2.f))+(pow(link->point_E.x,2.f)-pow(Zstart,2.f))));
link->point_E3.z = sin(yaw_path+atan((link->point_E.z-Zstart)/(link->point_E.x-Xstart)))*(sqrt((pow(link->point_E.x,2.f)-pow(Xstart,2.f))+(pow(link->point_E.x,2.f)-pow(Zstart,2.f))));									
							
							break;		
		case 3:
							if(TIME[3]<foot->arr*foot->t)
							{	
								sigma = 2*pi*TIME[3]/(foot->arr*foot->t);
								link->point_E2.x = (foot->X_end - foot->X_start)*(sigma-sin(sigma))/(2*pi) + foot->X_start;
								link->point_E2.z = foot->hight*(1-cos(sigma))/2+foot->Z_start;				
							}
							else if(TIME[3]>=foot->arr*foot->t)
							{
								sigma = 2*pi*(TIME[3]-foot->arr*foot->t)/((Ts-foot->arr)*foot->t);
								link->point_E2.x = -(foot->X_end - foot->X_start)*(sigma-sin(sigma))/(2*pi) + foot->X_end;
								link->point_E2.z = foot->Z_start;			
							}
link->point_E4.x = cos(yaw_path+atan((link->point_E2.z-Zstart)/(link->point_E2.x-Xstart)))*(sqrt((pow(link->point_E2.x,2.f)-pow(Xstart,2.f))+(pow(link->point_E2.x,2.f)-pow(Zstart,2.f))));
link->point_E4.z = sin(yaw_path+atan((link->point_E2.z-Zstart)/(link->point_E2.x-Xstart)))*(sqrt((pow(link->point_E2.x,2.f)-pow(Xstart,2.f))+(pow(link->point_E2.x,2.f)-pow(Zstart,2.f))));							
							
							break;
	}
}

/*
  *@funNm  Inverse_leg(LEG_typedef*leg,Link_typedef*link)
  *@brief  运动学逆解
*/
void Inverse_leg(LEG_typedef*leg,Link_typedef*link)
{	
	static float link_BE,link_AE,crank_angle[4];
/*临时辅助线BE AE*/	
	link_BE = sqrt(pow(link->point_E.x - link->point_B.x,2.f) + pow(link->point_E.z,2.f));
	link_AE = sqrt(pow(link->point_E.x - link->point_A.x,2.f) + pow(link->point_E.z,2.f));
/*角DBX+的解*/	
	leg->fail_angle1 = atanf(fabs(link->point_E.z / (link->point_B.x-link->point_E.x)));
	leg->fail        = acosf((pow(link_BE,2) + pow(link_BD,2.f) - pow(link_DE,2.f)) / (2*link_BE*link_BD));
	
	if(link->point_E.x<=link->point_B.x)       crank_angle[0] = pi- leg->fail_angle1 - leg->fail;	
	else if	(link->point_E.x>link->point_B.x)  crank_angle[0] = leg->fail_angle1-leg->fail;

	leg->real_angle[0] = crank_angle[0];	
	
	leg->Auxiliary_angle[0] = acosf((powf(link_AE,2.f) + powf(link_AC,2.f)- powf(link_CE,2.f)) /(2*link_AC*link_AE));
	leg->Auxiliary_angle[1] = atanf(fabs(link->point_E.z/(link->point_E.x-link->point_A.x)));	

/*角CAX-的解*/
	if(link->point_E.x<link->point_A.x)        crank_angle[1] = leg->Auxiliary_angle[1] - leg->Auxiliary_angle[0];			
	else if(link->point_E.x>=link->point_A.x)  crank_angle[1] = pi - leg->Auxiliary_angle[0] - leg->Auxiliary_angle[1];		

	leg->real_angle[1]=crank_angle[1];		
/*弧度化角度*/	
	leg->real_angle[0]=leg->real_angle[0]*180/pi;
	leg->real_angle[1]=leg->real_angle[1]*180/pi;

/*	
	leg->real_angle[0]=(int)leg->real_angle[0];
	leg->real_angle[1]=(int)leg->real_angle[1];
	leg->real_angle[2]=(int)leg->real_angle[2];
	leg->real_angle[3]=(int)leg->real_angle[3];
*/
}



/*
  *@funNm  Inverse_leg(LEG_typedef*leg,Link_typedef*link)
  *@brief  运动学逆解
*/
void Inverse_leg2(LEG_typedef*leg,Link_typedef*link)
{	
	static float link_BE,link_AE,link_BE2,link_AE2,crank_angle[4];
/*临时辅助线BE AE*/	
	link_BE = sqrt(pow(link->point_E.x - link->point_B.x,2.f) + pow(link->point_E.z,2.f));
	link_AE=  sqrt(pow(link->point_E.x - link->point_A.x,2.f) + pow(link->point_E.z,2.f));
	link_BE2 = sqrt(pow(link->point_E2.x - link->point_B2.x,2.f) + pow(link->point_E2.z,2.f));
	link_AE2 = sqrt(pow(link->point_E2.x - link->point_A2.x,2.f) + pow(link->point_E2.z,2.f));
/*角DBX+的解*/	
	leg->fail_angle1 = atanf(fabs(link->point_E.z / (link->point_B.x-link->point_E.x)));
	leg->fail        = acosf((pow(link_BE,2) + pow(link_BD,2.f) - pow(link_DE,2.f)) / (2*link_BE*link_BD));
	leg->shank       = acosf((pow(link_BE2,2.f) + pow(link_BD2,2.f) - pow(link_DE2,2.f)) / (2*link_BE2*link_BD2));
	leg->shank1      = atanf( fabs(link->point_E2.z / (link->point_B2.x -link->point_E2.x)));	
	
	
	if(link->point_E.x<=link->point_B.x)       crank_angle[0] = pi- leg->fail_angle1 - leg->fail;	
	else if	(link->point_E.x>link->point_B.x)  crank_angle[0] = leg->fail_angle1-leg->fail;

	if(link->point_E2.x <= link->point_B2.x )   crank_angle[2] = pi - leg->shank - leg->shank1;
	else if(link->point_E2.x>link->point_B2.x)  crank_angle[2] = leg->shank1 - leg->shank;

	leg->real_angle[0] = crank_angle[0];	
	leg->real_angle[2] = crank_angle[2];
	
	leg->Auxiliary_angle[0] = acosf((powf(link_AE,2.f) + powf(link_AC,2.f)- powf(link_CE,2.f)) /(2*link_AC*link_AE));
	leg->Auxiliary_angle[1] = atanf(fabs(link->point_E.z/(link->point_E.x-link->point_A.x)));	
	leg->Auxiliary_angle[2] = acosf((powf(link_AE2,2.f) + powf(link_AC2,2.f)- powf(link_CE2,2.f)) / (2*link_AC2*link_AE2));
	leg->Auxiliary_angle[3] = atanf(fabs(link->point_E2.z /(link->point_E2.x-link->point_A2.x)));	
/*角CAX-的解*/
	if(link->point_E.x<link->point_A.x)        crank_angle[1] = leg->Auxiliary_angle[1] - leg->Auxiliary_angle[0];			
	else if(link->point_E.x>=link->point_A.x)  crank_angle[1] = pi - leg->Auxiliary_angle[0] - leg->Auxiliary_angle[1];		
	
	if(link->point_E2.x >=link->point_A2.x )      crank_angle[3]= pi- leg->Auxiliary_angle[3] - leg->Auxiliary_angle[2];
	else if(link->point_E2.x < link->point_A2.x ) crank_angle[3]= leg->Auxiliary_angle[3] - leg->Auxiliary_angle[2];
	
	leg->real_angle[1]=crank_angle[1];
	leg->real_angle[3]=crank_angle[3];		
		
/*弧度化角度*/	
	leg->real_angle[0]=leg->real_angle[0]*180/pi;
	leg->real_angle[1]=leg->real_angle[1]*180/pi;
	leg->real_angle[2]=leg->real_angle[2]*180/pi;
	leg->real_angle[3]=leg->real_angle[3]*180/pi;
/*	
	leg->real_angle[0]=(int)leg->real_angle[0];
	leg->real_angle[1]=(int)leg->real_angle[1];
	leg->real_angle[2]=(int)leg->real_angle[2];
	leg->real_angle[3]=(int)leg->real_angle[3];
*/
}

	
