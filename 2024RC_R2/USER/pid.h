#ifndef  __PID_H
#define  __PID_H

#define  ABS(x)      ((x)>0? (x):(-(x)))
#include "main.h"

typedef struct PID_Data
{
	float Error ;
	float Last_Error ;
	float Earlier_Error ;
	float Error_Max;
	float Sum_Error;
	float D_Error;
	float Dead_Size;
	float K_P ;
	float K_I ;
	float K_D ;
	float I_Separate ;
	float I_Limit ;
	float Out_MAX ;
	float Out_MIN ; 
	float Output ;
	uint8_t first_flag;
}PID_Data;


extern PID_Data MOTOR_PID_RPM[8];
extern PID_Data MOTOR_PID_POS[8];	//位置pid信息


void PID_Position_Calculate_by_error(PID_Data *PID, float error);
void PID_Position_Calculate(PID_Data * PID,float expect,float Encoder_Count);
void PID_Parameter_Init(PID_Data * PID,float Pi,float Ki,float Di,float Out_MAX,float Dead_Size,float I_Limit ,float I_Separate );
void PID_Incremental_PID_Calculation(PID_Data *PID, float Expect, float Encoder_Count);
void PID_Incremental_PID_Calculation_by_error(PID_Data *PID, float error);

#endif



