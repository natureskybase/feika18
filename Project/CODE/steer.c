#include "steer.h"

/**************************************************************************
函数功能：舵机pwm初始化
频    率：50Hz
初始中值：MIDDLE_ANGLE
**************************************************************************/
void Steer_Init(void)
{
	pwm_init(STEER_PIN, 50, MIDDLE_ANGLE);
}

/**************************************************************************
函数功能：舵机旋转
输入参数：舵机相对中值旋转的角度（顺时针为正）
单    位：°
**************************************************************************/
void Steer_Spin(float target_angle)
{
	uint32 duty;
	
	/****限幅****/
	if(target_angle > 100) target_angle =  100;
	if(target_angle <-100) target_angle = -100;
	if(target_angle>=-100&&target_angle<=100)
	{
	if(target_angle>=0)
	{
		duty = MIDDLE_ANGLE +0.006*target_angle*target_angle;
//	duty = MIDDLE_ANGLE + target_angle*0.4;
	}
	
	if(target_angle<0)
	{
		duty = MIDDLE_ANGLE +(-0.007)*target_angle*target_angle;
//	duty = MIDDLE_ANGLE + target_angle*0.3;
	}
	}
	pwm_duty(STEER_PIN,duty);
}



