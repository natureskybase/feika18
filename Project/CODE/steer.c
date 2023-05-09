#include "steer.h"

float angle_limit;
/**************************************************************************
�������ܣ����pwm��ʼ��
Ƶ    �ʣ�50Hz
��ʼ��ֵ��MIDDLE_ANGLE
**************************************************************************/
void Steer_Init(void)
{
	pwm_init(STEER_PIN, 50, MIDDLE_ANGLE);
}

/**************************************************************************
�������ܣ������ת
�����������������ֵ��ת�ĽǶȣ�����Ϊ��ת,����Ϊ��ת��
��    λ����
**************************************************************************/
void Steer_Spin(float target_angle)
{
	uint32 duty;
	
	/****�޷�****/
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
�������ܣ�����޷�
�����������������ֵ��ת�ĽǶȣ�����Ϊ��ת,����Ϊ��ת��
��    λ����
**************************************************************************/
void Steer_Spin_limit(float target_angle,float angle_limit)
{
	if(target_angle > angle_limit)
		target_angle = angle_limit;
	if(target_angle < -angle_limit)
		target_angle = -angle_limit;
	
	Steer_Spin(target_angle);
	
}


