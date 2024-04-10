#include <pid.h>
/*
PID  (积分分离，死区，限幅)
*/

//pwm+=Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]


// 对变量进行范围限制
float PID_Abs_Limit(float a, float ABS_MAX)
{
    if(a > ABS_MAX)
        a = ABS_MAX;
		
    if(a < -ABS_MAX)
        a = -ABS_MAX;
		return a;
}


//增量式PID(速度环)
void PID_Incremental_PID_Calculation(PID_Data *PID, float Expect, float Encoder_Count)	
{
	PID->Error = Expect - Encoder_Count;
	PID->D_Error = PID->Error - PID->Last_Error;
	
	//计算输出值
	PID->Output += PID->K_P * (PID->D_Error) + PID_Abs_Limit(PID->K_I*PID->Error,PID->I_Limit) + PID->K_D * (PID->Error +  PID->Earlier_Error - 2*PID->Last_Error);
	
	//输出限制
	if(PID->Output > PID->Out_MAX )  PID->Output = PID->Out_MAX;
	if(PID->Output < - PID->Out_MAX )  PID->Output = -PID->Out_MAX;
	
	PID->Earlier_Error = PID->Last_Error;
	PID->Last_Error = PID->Error;

	//死区
	if(ABS(PID->Error) < PID->Dead_Size)
	{
		PID->Output = 0;
	}
}


//增量式PID(速度环)
void PID_Incremental_PID_Calculation_by_error(PID_Data *PID, float error)	
{
	PID->Error = error;
	PID->D_Error = error - PID->Last_Error;
	PID->Sum_Error = error;
	
	//计算输出值
	PID->Output += PID->K_P * (PID->D_Error) + PID_Abs_Limit(PID->K_I*PID->Error,PID->I_Limit) + PID->K_D * (PID->Error +  PID->Earlier_Error - 2*PID->Last_Error);
	
	//输出限制
	if(PID->Output > PID->Out_MAX )  PID->Output = PID->Out_MAX;
	if(PID->Output < - PID->Out_MAX )  PID->Output = -PID->Out_MAX;
	
	PID->Earlier_Error = PID->Last_Error;
	PID->Last_Error = PID->Error;

	//死区
	if(ABS(PID->Error) < PID->Dead_Size)
	{
		PID->Output = 0;
	}
}


//位置环PID，有死区
void PID_Position_Calculate(PID_Data *PID, float Next_Point, float Current_Point)
{
	if(PID->first_flag == 1)
	{
		PID->Last_Error = Next_Point - Current_Point;
		PID->Earlier_Error = Next_Point - Current_Point;
		PID->first_flag = 0;
	}

	PID->Error = Next_Point - Current_Point;
	PID->Sum_Error += PID->Error;
	PID->D_Error = PID->Error - PID->Last_Error;

	PID->Output =  PID->K_P * PID->Error + PID_Abs_Limit(PID->K_I * PID->Sum_Error, PID->K_I ) + PID->K_D * PID->D_Error;  

	if (PID->Output > PID->Out_MAX) PID->Output = PID->Out_MAX;
	if (PID->Output < PID->Out_MIN) PID->Output = PID->Out_MIN;

	PID->Last_Error = PID->Error;

	if(ABS(PID->Error) < PID->Dead_Size) PID->Output = 0;
}


//位置环PID，有死区
void PID_Position_Calculate_by_error(PID_Data *PID, float error)
{
	if(PID->first_flag == 1)
	{
		PID->Last_Error = error;
		PID->Earlier_Error = error;
		PID->first_flag = 0;
	}

	PID->Error = error;
	PID->Sum_Error += PID->Error;
	PID->D_Error = PID->Error - PID->Last_Error;

	PID->Output =  PID->K_P * PID->Error + PID_Abs_Limit(PID->K_I * PID->Sum_Error, PID->K_I ) + PID->K_D * PID->D_Error;  

	if (PID->Output > PID->Out_MAX) PID->Output = PID->Out_MAX;
	if (PID->Output < PID->Out_MIN) PID->Output = PID->Out_MIN;

	PID->Last_Error = PID->Error;

	if(ABS(PID->Error) < PID->Dead_Size) PID->Output = 0;
}


void PID_Parameter_Init(PID_Data *PID, float Pi, float Ki, float Di, float Out_MAX, float Dead_Size, float I_Separate, float I_Limit)
{
	PID->K_P = Pi;
	PID->K_I = Ki;
	PID->K_D = Di;
	PID->Dead_Size = Dead_Size;
	PID->Out_MAX = Out_MAX;
	PID->Out_MIN = -Out_MAX;
	PID->I_Limit = I_Limit;
	PID->I_Separate = I_Separate;
	PID->Error = PID->Last_Error = PID->Earlier_Error = 0;
	PID->Output = 0;
}

