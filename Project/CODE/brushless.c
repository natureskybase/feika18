#include "brushless.h"







// 无刷电机初始化  信号的高电平时间范围是1-2ms，1ms时电机不转，2ms时电机满转。通过调节高电平的时间来改变无刷电机的转速。
// 计算无刷电调转速   （1ms - 2ms）/20ms * 10000（10000是PWM的满占空比时候的值）
// 无刷电调转速 0%   为 500
// 无刷电调转速 100% 为 1000
void Brushless_Init(void)
{
	// pwm初始化频率为50hz 无刷设置0%占空比
	pwm_init(BRUSHLESS_L, 50, 500);	//无刷设置0%占空比					
	pwm_init(BRUSHLESS_R, 50, 500);	//无刷设置0%占空比   									

}
//输出PWM驱动无刷
//value值为转速百分比：0~100
void Set_Brushless_Output(uint16 value)
{
	if(value<=100)
	{
		pwm_duty(BRUSHLESS_L, (value*5+500)); // 无刷电调转速 value% 
		pwm_duty(BRUSHLESS_R, (value*5+500)); // 无刷电调转速 value%  		
	}else //value 输入错误
	{
		pwm_duty(BRUSHLESS_L, (0*5+500));  // 无刷电调转速 0% 
		pwm_duty(BRUSHLESS_R, (0*5+500));  // 无刷电调转速 0% 			
	}
	
}

















































