#include "move.h"

PID_Data YAW_PID;
PID_Data POINT_PID_X;
PID_Data POINT_PID_Y;


// 计算阶乘的函数
unsigned long long factorial(int n) 
{
    if (n <= 1) return 1;
    return n * factorial(n - 1);
}


/**
* @brief  Move_Init初始化
* @note		移动相关PID初始化
* @param
* @retval 
*/
void Move_Init(void)
{
    PID_Parameter_Init(&POINT_PID_X, 0.025, 0.001, 0.33, 0.3, -1, 0, 0);
	PID_Parameter_Init(&POINT_PID_Y, 0.025, 0.001, 0.33, 0.3, -1, 0, 0);
	PID_Parameter_Init(&YAW_PID, 2.0, 0.001, 0.01, 1.5, -1, 0, 0);
}


void Angle_Limit(float *angle)
{
    static uint8_t recursiveTimes = 0;

    recursiveTimes++;

    if(recursiveTimes<100)
    {
        if(*angle > 180.0f)
        {
            *angle -= 360.0f;
            Angle_Limit(angle);
        }
        else if(*angle < -180.0f)
        {
            *angle += 180.0f;
            Angle_Limit(angle);
        }
        else{}
    }
    recursiveTimes--;
}


/**
* @brief  YawAdjust偏航角控制
* @note		将偏航角控制在目标角度
* @param  Target_angle:要限制的值
* @retval 
*/
void Yaw_Adjust(float target_angle)
{
    //计算误差
    float error;
    if(ROBOT_REAL_POS_DATA.POS_YAW*target_angle > 0)
    {
        error = target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
    }
    else
    {
        if(ABS(ROBOT_REAL_POS_DATA.POS_YAW) + ABS(target_angle) <= 180)
            error = target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
        else
            Angle_Limit(&error);
    }

    //PID输出角速度
    PID_Position_Calculate_by_error(&YAW_PID,error);
    ROBOT_CHASSI.WORLD.World_W = YAW_PID.Output;
}


/**
* @brief  LockupPoint锁定车
* @note		将车锁定在某一点上
* @param  POS_X:要限制的X值，POS_Y:要限制的Y值，POS_YAW:要限制的偏航角
* @retval 
*/
void LockupPoint(float POS_X, float POS_Y, float POS_YAW)
{
	//Yaw_Adjust(POS_YAW);
	PID_Position_Calculate(&POINT_PID_X, POS_X,ROBOT_REAL_POS_DATA.POS_X);
	PID_Position_Calculate(&POINT_PID_Y, POS_Y,ROBOT_REAL_POS_DATA.POS_Y);
	
	ROBOT_CHASSI.WORLD.World_X = POINT_PID_X.Output;
	ROBOT_CHASSI.WORLD.World_Y = POINT_PID_Y.Output;
}


float kp_x = 0.008;
float kd_x = 0;	//0.00011
float kp_y = 0.008;
float kd_y = 0;	//0.00011
float kp_yaw = 0.01;
float kd_yaw = 0;
float error_X;float error_Y;	
float error_Yaw;							// 偏航角偏差
float now_yaw;								// 当前弧度制偏航角
float u_output;								// 本体坐标x方向速度输出
float v_output;								// 本体坐标y方向速度输出
float w_ouput;								// 角速度输出
/**
* @brief  PDController跟踪器
* @note		跟踪规划好的路径
* @param  target_point:单位时间要跟踪的点（需先规划好速度），robot_now_pos:机器人当前世界坐标下的位置
* @retval 
*/
void PD_Controller(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos)
{
    Yaw_Adjust(target_point.Yaw);

    //计算误差
    error_X = target_point.X - robot_now_pos.POS_X;
    error_Y = target_point.Y - robot_now_pos.POS_Y;

    PID_Position_Calculate_by_error(&POINT_PID_X,error_X);
    PID_Position_Calculate_by_error(&POINT_PID_Y,error_Y);

    ROBOT_CHASSI.WORLD.World_X = POINT_PID_X.Output;
    ROBOT_CHASSI.WORLD.World_Y = POINT_PID_Y.Output;
}


int first_time_flag = 1;
float Hz;
float last_X;float last_Y;float last_Yaw;
PATH_TYPEDEF now_path_point;
/**
* @brief  PathPlan规划+跟踪
* @note		贝塞尔曲线规划，误差直接赋值，到达终点返回1，否则返回0
* @param  t_real:真实经过的时间，t_target:目标总时间，num:控制点数目+1，X、Y:控制点数组
* @retval 
*/
int Bezier_PathPlan(float t_real, float t_target, int num, float *X, float *Y, float *Yaw)
{
    float t;
    float x=0,y=0;
    t = t_real/t_target;   

    for (int i = 0; i < num; ++i)
    {
        float b = (float)(factorial(num - 1)/(factorial(i)*factorial(num-1-i)))*pow(1-t,num-1-i)*pow(t,i);
        x += b*X[i];
        y += b*Y[i];
    }
    
    if(first_time_flag)
	{
		now_path_point.V_x = 0;
		now_path_point.V_y = 0;
		now_path_point.V_w = 0;
		first_time_flag = 0;
		Hz = 1 / t_real;
	}
	else
	{
		now_path_point.V_x = (now_path_point.X - last_X) * Hz;
		now_path_point.V_y = (now_path_point.Y - last_Y) * Hz;
		now_path_point.V_w = (now_path_point.Yaw - last_Yaw) * Hz;
	}

    PD_Controller(now_path_point,ROBOT_REAL_POS_DATA);

    //保留本次值
    last_X = now_path_point.X;
    last_Y = now_path_point.Y;
    last_Yaw = now_path_point.Yaw;

    //到达终点
    if(t_real > t_target)
	{
		ROBOT_CHASSI.WORLD.World_X = 0;
		ROBOT_CHASSI.WORLD.World_Y = 0;
		first_time_flag = 1;
		return 1;
	} 
	return 0;
}


int k;
float t;
float f1s;float f2s;float f3s;float f4s;
/**
* @brief  PathPlan规划+跟踪
* @note		三次B样条规划，误差直接赋值，到达终点返回1，否则返回0
* @param  t_real:真实经过的时间，t_target:目标总时间，num:控制点数目+1，X、Y:控制点数组
* @retval 
*/
int ThreeB_PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw)
{
    k=(int)(t_real*num / t_target);     //第k段
    t=t_real - k*t_target / num;        //第k段时间
    t=t*num / t_target;

    //位置样条函数
    f1s = (1 - t) * (1 - t) * (1 - t) / 6;
	f2s = (3 * t * t * t - 6 * t * t + 4) / 6;
	f3s = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6;
	f4s = (t * t * t) / 6;

    // 计算目标跟踪点
    now_path_point.X = X[k]*f1s + X[k+1]*f2s + X[k+2]*f3s + X[k+3]*f3s;
    now_path_point.Y = Y[k]*f1s + Y[k+1]*f2s + Y[k+2]*f3s + Y[k+3]*f4s;
    now_path_point.Yaw = Yaw[k]*f1s + Yaw[k+1]*f2s + Yaw[k+2]*f3s + Yaw[k+3]*f4s;

    if(first_time_flag)
    {
        now_path_point.V_x = 0;
		now_path_point.V_y = 0;
		now_path_point.V_w = 0;
		first_time_flag = 0;
		Hz = 1 / t_real;
    }
    else
    {
        now_path_point.V_x = (now_path_point.X - last_X) * Hz;
		now_path_point.V_y = (now_path_point.Y - last_Y) * Hz;
		now_path_point.V_w = (now_path_point.Yaw - last_Yaw) * Hz;
    }

    //PD跟踪器
    PD_Controller(now_path_point,ROBOT_REAL_POS_DATA);

    //判断是否到达终点
    if(t_real > t_target)
    {
        ROBOT_CHASSI.WORLD.World_X = 0;
        ROBOT_CHASSI.WORLD.World_Y = 0;
        first_time_flag = 1;
        return 1;
    }

    return 0;
}
