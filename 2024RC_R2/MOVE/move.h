#ifndef __MOVE_H
#define __MOVE_H

#include "main.h"

typedef struct ROBOT_TARGET_VELOCITY
{
	float Vx;
	float Vy;
	float W;
	float Vx_RPM;
	float Vy_RPM;
	float W_RPM;
}ROBOT_TARGET_VELOCITY;

typedef struct
{
	float X;
	float Y;
	float Yaw;
	float V_x;
	float V_y;
	float V_w;
}PATH_TYPEDEF;

// 机器人的真实位置
typedef struct ROBOT_REAL_POS
{
	float POS_X;
	float POS_Y;     
	float POS_YAW;
	int robot_location;
}ROBOT_REAL_POS;

void Move_Init(void);
void Angle_Limit(float *angle);
unsigned long long factorial(int n);
void Yaw_Adjust(float target_angle);
void LockupPoint(float POS_X, float POS_Y, float POS_YAW);
void PD_Controller(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos);
int Bezier_PathPlan(float t_real, float t_target, int num, float *X, float *Y, float *Yaw);
int ThreeB_PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw);


extern ROBOT_REAL_POS ROBOT_REAL_POS_DATA;
#endif
