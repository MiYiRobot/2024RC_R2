#ifndef __RM_MOTOR_H
#define __RM_MOTOR_H

#include "main.h"



void M3508_Send_Currents(void);
void get_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8]);
void RM_MOTOR_Angle_Integral(MOTOR_REAL_INFO* RM_MOTOR);

#endif
