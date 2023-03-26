#include "brushless.h"







// ��ˢ�����ʼ��  �źŵĸߵ�ƽʱ�䷶Χ��1-2ms��1msʱ�����ת��2msʱ�����ת��ͨ�����ڸߵ�ƽ��ʱ�����ı���ˢ�����ת�١�
// ������ˢ���ת��   ��1ms - 2ms��/20ms * 10000��10000��PWM����ռ�ձ�ʱ���ֵ��
// ��ˢ���ת�� 0%   Ϊ 500
// ��ˢ���ת�� 100% Ϊ 1000
void Brushless_Init(void)
{
	// pwm��ʼ��Ƶ��Ϊ50hz ��ˢ����0%ռ�ձ�
	pwm_init(BRUSHLESS_L, 50, 500);	//��ˢ����0%ռ�ձ�					
	pwm_init(BRUSHLESS_R, 50, 500);	//��ˢ����0%ռ�ձ�   									

}
//���PWM������ˢ
//valueֵΪת�ٰٷֱȣ�0~100
void Set_Brushless_Output(uint16 value)
{
	if(value<=100)
	{
		pwm_duty(BRUSHLESS_L, (value*5+500)); // ��ˢ���ת�� value% 
		pwm_duty(BRUSHLESS_R, (value*5+500)); // ��ˢ���ת�� value%  		
	}else //value �������
	{
		pwm_duty(BRUSHLESS_L, (0*5+500));  // ��ˢ���ת�� 0% 
		pwm_duty(BRUSHLESS_R, (0*5+500));  // ��ˢ���ת�� 0% 			
	}
	
}

















































