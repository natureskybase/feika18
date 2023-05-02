#ifndef __STEER_H_
#define __STEER_H_

#include "zf_pwm.h"

/****�궨��****/
#define STEER_PIN PWMA_CH2N_P13
#define MIDDLE_ANGLE 610    //530-670

extern float angle_limit;

/****��������****/
void Steer_Init(void);
void Steer_Spin(float target_angle);
void Steer_Spin_limit(float target_angle,float angle_limit);

#endif
