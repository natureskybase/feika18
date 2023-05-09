#include "timer.h"

/**************************************************************************
�������ܣ���ʱ���жϳ�ʼ��
���������ѭ�����ڣ���λ��ms
ʹ��ʱ�ӣ�TIM_4��TIM_3
ע���ж���ִ�г����� isr.c �� TM4_Isr() ��
**************************************************************************/
void Timer_Init(uint16 time_ms)
{
	uint16 temp;
	temp = (uint16)65536 - (uint16)(sys_clk / (12 * (1000 / time_ms)));
	
	T3L = temp; 	
	T3H = temp >> 8;
	T4L = temp; 	
	T4H = temp >> 8;
	
	T4T3M |= 0x88; // ������ʱ��3�Ͷ�ʱ��4
	IE2 |= 0x40; // ʹ�ܶ�ʱ��4�ж�
}

void Timer4_to_Timer3(void)
{
	IE2 &= ~0x40;
	IE2 |= 0x20;
}

void Timer3_to_Timer4(void)
{
	IE2 &= ~0x20;
	IE2 |= 0x40;
}