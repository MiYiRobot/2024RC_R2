#include "main.h"
#include "chassis.h"

#define FOUR_OMNI_CHASSIS  1
#define THREE_OMNI_CHASSIS 0
#define MECANUM_CHASSIS 0

ROBOT_CHASSIS ROBOT_CHASSI={0};

/**
 * @brief 地盘解算，四全向轮
 * @param NULL
 * @return NULL
*/
void Robot_Wheels_RPM_calculate(void)
{
	float COS,SIN;
	COS = cos (ROBOT_CHASSI.Angle * PI /180);
	SIN = sin (ROBOT_CHASSI.Angle * PI /180);
	
	//世界坐标系转换
	//转化为转速
    if(ROBOT_CHASSI.World_Move_Flag == 1)
    {
		ROBOT_CHASSI.SPEED.Robot_VX  = (ROBOT_CHASSI.WORLD.World_X * COS - ROBOT_CHASSI.WORLD.World_Y * SIN);
		ROBOT_CHASSI.SPEED.Robot_VY  = (ROBOT_CHASSI.WORLD.World_X * SIN + ROBOT_CHASSI.WORLD.World_Y * COS);
    }
#if FOUR_OMNI_CHASSIS 

	ROBOT_CHASSI.Motor_Target_RPM[0] = (-ROBOT_CHASSI.SPEED.Robot_VY*COS45 + ROBOT_CHASSI.SPEED.Robot_VX*COS45 + ROBOT_CHASSI.WORLD.World_W*CHASSIS_R) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[1] = (-ROBOT_CHASSI.SPEED.Robot_VY*COS45 - ROBOT_CHASSI.SPEED.Robot_VX*COS45 + ROBOT_CHASSI.WORLD.World_W*CHASSIS_R) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[2] = ( ROBOT_CHASSI.SPEED.Robot_VY*COS45 - ROBOT_CHASSI.SPEED.Robot_VX*COS45 + ROBOT_CHASSI.WORLD.World_W*CHASSIS_R) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[3] = ( ROBOT_CHASSI.SPEED.Robot_VY*COS45 + ROBOT_CHASSI.SPEED.Robot_VX*COS45 + ROBOT_CHASSI.WORLD.World_W*CHASSIS_R) * MS_transition_RM;

	for (int i = 0; i < 4; i++)
	{
		Speed_Control(&MOTO_REAL_INFO[i],ROBOT_CHASSI.Motor_Target_RPM[i]);
	}
#endif

#if THREE_OMNI_CHASSIS 

	ROBOT_CHASSI.Motor_Target_RPM[0] = ( ROBOT_CHASSI.SPEED.Robot_VY*COS30 - ROBOT_CHASSI.SPEED.Robot_VX*SIN30 + ROBOT_CHASSI.SPEED.World_W*MS_transition_RM) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[1] = (-ROBOT_CHASSI.SPEED.Robot_VY*SIN30 + ROBOT_CHASSI.SPEED.Robot_VX*COS30 + ROBOT_CHASSI.SPEED.World_W*MS_transition_RM) * MS_transition_RM; 
	ROBOT_CHASSI.Motor_Target_RPM[2] = ( ROBOT_CHASSI.SPEED.Robot_VX + ROBOT_CHASSI.SPEED.World_W*MS_transition_RM) * MS_transition_RM;

	for (int i = 0; i < 3; i++)
	{
		Speed_Control(&MOTO_REAL_INFO[i],ROBOT_CHASSI.Motor_Target_RPM[i]);
	}

#endif

#if MECANUM_CHASSIS

	ROBOT_CHASSI.Motor_Target_RPM[0] = (-ROBOT_CHASSI.SPEED.Robot_VX + ROBOT_CHASSI.SPEED.Robot_VY + (Mecanum_Rx + Mecanum_Ry)*ROBOT_CHASSI.SPEED.World_W) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[1] = ( ROBOT_CHASSI.SPEED.Robot_VX + ROBOT_CHASSI.SPEED.Robot_VY - (Mecanum_Rx + Mecanum_Ry)*ROBOT_CHASSI.SPEED.World_W) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[2] = (-ROBOT_CHASSI.SPEED.Robot_VX + ROBOT_CHASSI.SPEED.Robot_VY - (Mecanum_Rx + Mecanum_Ry)*ROBOT_CHASSI.SPEED.World_W) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[3] = ( ROBOT_CHASSI.SPEED.Robot_VX + ROBOT_CHASSI.SPEED.Robot_VY + (Mecanum_Rx + Mecanum_Ry)*ROBOT_CHASSI.SPEED.World_W) * MS_transition_RM;

	for (int i = 0; i < 4; i++)
	{
		Speed_Control(&MOTO_REAL_INFO[i],ROBOT_CHASSI.Motor_Target_RPM[i]);
	}
#endif
}


