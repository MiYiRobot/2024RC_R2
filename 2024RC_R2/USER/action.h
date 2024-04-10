#ifndef __ACTION_H
#define __ACTION_H

#include "main.h"

typedef struct ACTION_GL_POS
{
	float ANGLE_Z;
	float ANGLE_X;
	float ANGLE_Y;
	float POS_X;
	float POS_Y;
	float W_Z;

	float LAST_POS_X;
	float LAST_POS_Y;

	float DELTA_POS_X;
	float DELTA_POS_Y;
	
	float REAL_X;
	float REAL_Y;
	
	
} ACTION_GL_POS;



void action_data_analyse(void);
void Update_Action_gl_position(float value[6]);

#endif
