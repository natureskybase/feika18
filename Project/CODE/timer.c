#include "timer.h"

/**************************************************************************
函数功能：定时器中断初始化
输入参数：循环周期，单位：ms
使用时钟：TIM_4和TIM_3
注：中断内执行程序在 isr.c 的 TM4_Isr() 中
**************************************************************************/
void Timer_Init(uint16 time_ms)
{
	uint16 temp;
	temp = (uint16)65536 - (uint16)(sys_clk / (12 * (1000 / time_ms)));
	
	T3L = temp; 	
	T3H = temp >> 8;
	T4L = temp; 	
	T4H = temp >> 8;
	
	T4T3M |= 0x88; // 启动定时器3和定时器4
	IE2 |= 0x40; // 使能定时器4中断
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