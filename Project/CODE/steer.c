#include "steer.h"

float angle_limit;
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
输入参数：舵机相对中值旋转的角度（负数为左转,正数为右转）
单    位：°
**************************************************************************/
void Steer_Spin(float target_angle)
{
	uint32 duty;
	
	/****限幅****/
	if(target_angle > 28) target_angle =  28;
	if(target_angle <-28) target_angle = -28;
	if(target_angle >= -28&&target_angle <= 28)
	{
	if(target_angle>=0)
	{
//		duty = MIDDLE_ANGLE +0.102*target_angle*target_angle;
	duty = MIDDLE_ANGLE + target_angle*2;
	}
	
	if(target_angle<0)
	{
//		duty = MIDDLE_ANGLE +(-0.102)*target_angle*target_angle;
	duty = MIDDLE_ANGLE + target_angle*2.5;
	}
	}
	pwm_duty(STEER_PIN,duty);
}

/**************************************************************************
函数功能：舵机限幅
输入参数：舵机相对中值旋转的角度（负数为左转,正数为右转）
单    位：°
**************************************************************************/
void Steer_Spin_limit(float target_angle,float angle_limit)
{
	if(target_angle > angle_limit)
		target_angle = angle_limit;
	if(target_angle < -angle_limit)
		target_angle = -angle_limit;
	
	Steer_Spin(target_angle);
	
}


