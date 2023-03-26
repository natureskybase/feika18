#include "motor.h"

/****编码器读数****/
int16 pulse_left  = 0; 
int16 pulse_right = 0;

/**************************************************************************
函数功能：电机pwm初始化，编码器初始化
频    率：10KHz
PWM_DUTY_MAX：50000
**************************************************************************/
void Motor_Init(void)
{
	/****左两路PWM****/
	pwm_init(PWM_L1, 10000, 0);
	pwm_init(PWM_L2, 10000, 0);
	/****右两路PWM****/
	pwm_init(PWM_R1, 10000, 0);
	pwm_init(PWM_R2, 10000, 0);
	/****两路编码器****/
	ctimer_count_init(CTIM_L);	//左 DIR P35
	ctimer_count_init(CTIM_R);	//右 DIR P53
}

/**************************************************************************
函数功能：控制电机PWM占空比
输入参数：左PWM占空比，右PWM占空比
频    率：10KHz
PWM_DUTY_MAX：50000
**************************************************************************/
void Motor_Control(int32 left_duty,int32 right_duty)
{
	/****PWM限幅****/
	if(left_duty  >  DUTY_LIMIT)	left_duty  =  DUTY_LIMIT;
	if(left_duty  < -DUTY_LIMIT)	left_duty  = -DUTY_LIMIT;
	if(right_duty >  DUTY_LIMIT)	right_duty =  DUTY_LIMIT;
	if(right_duty < -DUTY_LIMIT)	right_duty = -DUTY_LIMIT;
	/****左两路PWM控制****/
	if(left_duty >= 0)	
	{									  	
		pwm_duty(PWM_L1,0);
		pwm_duty(PWM_L2,left_duty);
	}
	else
	{
		pwm_duty(PWM_L1,-left_duty);
		pwm_duty(PWM_L2,0);
	}
	/****右两路PWM控制****/
	if(right_duty >= 0)
	{
		pwm_duty(PWM_R1,0);
		pwm_duty(PWM_R2,right_duty);
	}
	else
	{
		pwm_duty(PWM_R1,-right_duty);
		pwm_duty(PWM_R2,0); 
	}
}

/**************************************************************************
函数功能：关闭电机
**************************************************************************/
void Motor_Shut(void)
{
	pwm_duty(PWM_L1,0);
	pwm_duty(PWM_L2,0);
	pwm_duty(PWM_R1,0);
	pwm_duty(PWM_R2,0);
}

/**************************************************************************
函数功能：读取编码器读数并赋值给变量pulse_left、pulse_right
**************************************************************************/
void Encoder_Read(void)
{
	if(DIR_L == 1)	
		pulse_left  = ctimer_count_read(CTIM_L);
	else
		pulse_left  = ctimer_count_read(CTIM_L) * -1;

	if(DIR_R == 1)																									//暂时除以二，等编码器换了改
		pulse_right = ctimer_count_read(CTIM_R);
	else
		pulse_right = ctimer_count_read(CTIM_R) * -1;
	pulse_left = pulse_left*2;
}

/**************************************************************************
函数功能：编码器数值清空（不是清空变量）
**************************************************************************/
void Encoder_Clear(void)
{
	ctimer_count_clean(CTIM_L);
	ctimer_count_clean(CTIM_R);
}