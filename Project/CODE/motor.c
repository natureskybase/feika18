#include "motor.h"

/****����������****/
int16 pulse_left  = 0; 
int16 pulse_right = 0;

/**************************************************************************
�������ܣ����pwm��ʼ������������ʼ��
Ƶ    �ʣ�10KHz
PWM_DUTY_MAX��50000
**************************************************************************/
void Motor_Init(void)
{
	/****����·PWM****/
	pwm_init(PWM_L1, 10000, 0);
	pwm_init(PWM_L2, 10000, 0);
	/****����·PWM****/
	pwm_init(PWM_R1, 10000, 0);
	pwm_init(PWM_R2, 10000, 0);
	/****��·������****/
	ctimer_count_init(CTIM_L);	//�� DIR P35
	ctimer_count_init(CTIM_R);	//�� DIR P53
}

/**************************************************************************
�������ܣ����Ƶ��PWMռ�ձ�
�����������PWMռ�ձȣ���PWMռ�ձ�
Ƶ    �ʣ�10KHz
PWM_DUTY_MAX��50000
**************************************************************************/
void Motor_Control(int32 left_duty,int32 right_duty)
{
	/****PWM�޷�****/
	if(left_duty  >  DUTY_LIMIT)	left_duty  =  DUTY_LIMIT;
	if(left_duty  < -DUTY_LIMIT)	left_duty  = -DUTY_LIMIT;
	if(right_duty >  DUTY_LIMIT)	right_duty =  DUTY_LIMIT;
	if(right_duty < -DUTY_LIMIT)	right_duty = -DUTY_LIMIT;
	/****����·PWM����****/
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
	/****����·PWM����****/
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
�������ܣ��رյ��
**************************************************************************/
void Motor_Shut(void)
{
	pwm_duty(PWM_L1,0);
	pwm_duty(PWM_L2,0);
	pwm_duty(PWM_R1,0);
	pwm_duty(PWM_R2,0);
}

/**************************************************************************
�������ܣ���ȡ��������������ֵ������pulse_left��pulse_right
**************************************************************************/
void Encoder_Read(void)
{
	if(DIR_L == 1)	
		pulse_left  = ctimer_count_read(CTIM_L);
	else
		pulse_left  = ctimer_count_read(CTIM_L) * -1;

	if(DIR_R == 1)																									//��ʱ���Զ����ȱ��������˸�
		pulse_right = ctimer_count_read(CTIM_R);
	else
		pulse_right = ctimer_count_read(CTIM_R) * -1;
	pulse_left = pulse_left*2;
}

/**************************************************************************
�������ܣ���������ֵ��գ�������ձ�����
**************************************************************************/
void Encoder_Clear(void)
{
	ctimer_count_clean(CTIM_L);
	ctimer_count_clean(CTIM_R);
}