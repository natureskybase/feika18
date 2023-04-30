#include "steer.h"

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
�����������������ֵ��ת�ĽǶȣ�˳ʱ��Ϊ����
��    λ����
**************************************************************************/
void Steer_Spin(float target_angle)
{
	uint32 duty;
	
	/****�޷�****/
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



