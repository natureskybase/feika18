#include "control.h"
#include "math.h"
#include "pid.h"


akeman_t akeman_left ;		//�ṹ���������¼���壬ʵ���ٶȣ�Ŀ���ٶ�
akeman_t akeman_right;
uint8 pid_flag=0;					//����ջ�������־
float order_angle=0;			//Ŀ��Ƕ�
float order_speed=0;			//Ŀ���ٶ�

/**************************************************************************
�������ܣ�����ǶȺ͵���ٶȵ�Эͬ����
���������С����׼�ٶȣ���λ��mm/s��������Ƕȣ���λ���㣩
**************************************************************************/
void Akeman_Control(float basic_speed,float target_angle)
{
	float akeman_diff;	//���������basic_speed�Ĳ���
	
	/****��ȡ��������ֵ****/
	Encoder_Read();
	Encoder_Clear();
	
	/****���������ʵ��ת��****/
	//��������1024�ߣ�4��Ƶ������Ϊ30
	//����ֱ��56mm�����������Ӧ����Ϊ68
	akeman_left.current_speed  = pulse_left *0.20603922;
	akeman_right.current_speed = pulse_right*0.20603922;
	
	/****���������Ŀ��ת��****/
	akeman_diff = basic_speed*(14.2*tan(target_angle*3.14/180.0))/20.0;	//��Բ���
	akeman_left.target_speed  = basic_speed - akeman_diff;
	akeman_right.target_speed = basic_speed + akeman_diff;
	
	/****PID����ʱ���Ƶ���Ͷ��****/
	if(pid_flag)
	{
		/****����ջ�****/
		Motor_PID_Control(akeman_left.current_speed ,akeman_left.target_speed ,akeman_right.current_speed ,akeman_right.target_speed);
	
		/****���ת��****/
		Steer_Spin(target_angle);
	}
}

/**************************************************************************
�������ܣ����PID�ջ�����
��������������ֵ�ʵ����Ŀ��ת��
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
//	/****���㱾��ƫ��****/
//	error_l = target_l - current_l;
//	error_r = target_r - current_r;
//	
//	/****��ʵ���ٶ�Ϊ0���򲻼ӻ�����****/
//	if(current_l==0) ki_l=0;
//	if(current_r==0) ki_r=0;
//	
//	/****����ʽPID������****/
//	pwm_l += (int32)(kp_l*(error_l-last_error_l) + ki_l*error_l + kd_l*(error_l-2*last_error_l+prev_error_l))*100;
//	pwm_r += (int32)(kp_r*(error_r-last_error_r) + ki_r*error_r + kd_r*(error_r-2*last_error_r+prev_error_r))*100;
//	
//	/****�޷�****/
//	if(pwm_l >  PWM_DUTY_MAX) pwm_l =  PWM_DUTY_MAX;
//	if(pwm_l < -PWM_DUTY_MAX) pwm_l = -PWM_DUTY_MAX;
//	if(pwm_r >  PWM_DUTY_MAX) pwm_r =  PWM_DUTY_MAX;
//	if(pwm_r < -PWM_DUTY_MAX) pwm_r = -PWM_DUTY_MAX;
//	
//	/****����ǰ����ƫ��****/
//	prev_error_l = last_error_l;
//	prev_error_r = last_error_r;
//	last_error_l = error_l;
//	last_error_r = error_r;

	pwm_l=pid_calc(&pid_left_,current_l,target_l);
	pwm_r=pid_calc(&pid_right_,current_r,target_r);	
	Motor_Control((int32)pwm_l,(int32)pwm_r);
}

/**************************************************************************
�������ܣ�����adc��ֵ�����Ƕ�
���������PD���Ʋ���
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
�������ܣ���������ջ�
**************************************************************************/
void PID_on(void)
{
	pid_flag = 1;
}

/**************************************************************************
�������ܣ��رյ���ջ�
**************************************************************************/
void PID_off(void)
{
	pid_flag = 0;
}
