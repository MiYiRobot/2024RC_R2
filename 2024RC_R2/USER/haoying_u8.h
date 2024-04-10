#ifndef __HAOYING_H
#define __HAOYING_H

#include "main.h"

// typedef struct HAOYING_DRIVE{
//     uint8_t PWM_ID;
//     uint16_t Throttle_value;
// }HAOYING_DRIVE;

typedef uint16_t PWM_PIN;
#define U8_PWM_PIN6 6
#define U8_PWM_PIN8 8
#define N5065_PWM_PIN9 9

void HaoYing_U8_Init(void);
void u8_speed_set(int16_t pulse_value,PWM_PIN PIN);

#endif
