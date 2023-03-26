#ifndef __STEER_H_
#define __STEER_H_

#include "zf_pwm.h"

/****�궨��****/
#define STEER_PIN PWMA_CH2N_P13
#define MIDDLE_ANGLE 4000

/****��������****/
void Steer_Init(void);
void Steer_Spin(float target_angle);

#endif
