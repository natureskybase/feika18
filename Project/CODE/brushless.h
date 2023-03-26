#ifndef _BRUSHLESS_H_
#define _BRUSHLESS_H_


#include "zf_pwm.h"


#define BRUSHLESS_L PWMA_CH1P_P10
#define BRUSHLESS_R PWMA_CH4P_P16

void Brushless_Init(void);
void Set_Brushless_Output(uint16 value);











#endif
