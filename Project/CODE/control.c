#include "control.h"
#include "math.h"
#include "pid.h"


akeman_t akeman_left ;		//结构体变量，记录脉冲，实际速度，目标速度
akeman_t akeman_right;
uint8 pid_flag=0;					//电机闭环开启标志
float order_angle=0;			//目标角度
float order_speed=0;			//目标速度

/**************************************************************************
函数功能：舵机角度和电机速度的协同控制
输入参数：小车基准速度（单位：mm/s），舵机角度（单位：°）
**************************************************************************/
void Akeman_Control(float basic_speed,float target_angle)
{
	float akeman_diff;	//左右轮相对basic_speed的差速
	
	/****读取编码器数值****/
	Encoder_Read();
	Encoder_Clear();
	
	/****计算两电机实际转速****/
	//编码器是1024线，4倍频，齿数为30
	//车轮直径56mm，与编码器对应齿数为68
	akeman_left.current_speed  = pulse_left *0.20603922;
	akeman_right.current_speed = pulse_right*0.20603922;
	
	/****计算两电机目标转速****/
	akeman_diff = basic_speed*(14.2*tan(target_angle*3.14/180.0))/20.0;	//相对差速
	akeman_left.target_speed  = basic_speed - akeman_diff;
	akeman_right.target_speed = basic_speed + akeman_diff;
	
	/****PID开启时控制电机和舵机****/
	if(pid_flag)
	{
		/****电机闭环****/
		Motor_PID_Control(akeman_left.current_speed ,akeman_left.target_speed ,akeman_right.current_speed ,akeman_right.target_speed);
	
		/****舵机转角****/
		Steer_Spin(target_angle);
	}
}

/**************************************************************************
函数功能：电机PID闭环控制
输入参数：左右轮的实际与目标转速
**************************************************************************/
void Motor_PID_Control(float current_l,float target_l,float current_r,float target_r)
{
	static float pwm_l=0;
	static float pwm_r=0;	
	
//	static float pwm_l=0,error_l=0,last_error_l=0,prev_error_l=0;
//	static float pwm_r=0,error_r=0,last_error_r=0,prev_error_r=0;
//	float kp_l=akeman_left.kp ,ki_l=akeman_left.ki ,kd_l=akeman_left.kd ;
//	float kp_r=akeman_right.kp,ki_r=akeman_right.ki,kd_r=akeman_right.kd;
//	
//	/****计算本次偏差****/
//	error_l = target_l - current_l;
//	error_r = target_r - current_r;
//	
//	/****若实际速度为0，则不加积分项****/
//	if(current_l==0) ki_l=0;
//	if(current_r==0) ki_r=0;
//	
//	/****增量式PID控制器****/
//	pwm_l += (int32)(kp_l*(error_l-last_error_l) + ki_l*error_l + kd_l*(error_l-2*last_error_l+prev_error_l))*100;
//	pwm_r += (int32)(kp_r*(error_r-last_error_r) + ki_r*error_r + kd_r*(error_r-2*last_error_r+prev_error_r))*100;
//	
//	/****限幅****/
//	if(pwm_l >  PWM_DUTY_MAX) pwm_l =  PWM_DUTY_MAX;
//	if(pwm_l < -PWM_DUTY_MAX) pwm_l = -PWM_DUTY_MAX;
//	if(pwm_r >  PWM_DUTY_MAX) pwm_r =  PWM_DUTY_MAX;
//	if(pwm_r < -PWM_DUTY_MAX) pwm_r = -PWM_DUTY_MAX;
//	
//	/****保存前两次偏差****/
//	prev_error_l = last_error_l;
//	prev_error_r = last_error_r;
//	last_error_l = error_l;
//	last_error_r = error_r;

	pwm_l=pid_calc(&pid_left_,current_l,target_l);
	pwm_r=pid_calc(&pid_right_,current_r,target_r);	
	Motor_Control((int32)pwm_l,(int32)pwm_r);
}

/**************************************************************************
函数功能：根据adc读值修正角度
输入参数：PD控制参数
**************************************************************************/
float Correct_Angle(float kp,float kd)
{
	float adc_err,target_angle;
	static float adc_err_last = 0;
	adc_err = (adc1-adc4)/(float)(adc1+adc4);
	target_angle = kp*adc_err + kd*(adc_err-adc_err_last);
	adc_err_last = adc_err;
	return target_angle;
}

/**************************************************************************
函数功能：开启电机闭环
**************************************************************************/
void PID_on(void)
{
	pid_flag = 1;
}

/**************************************************************************
函数功能：关闭电机闭环
**************************************************************************/
void PID_off(void)
{
	pid_flag = 0;
}
