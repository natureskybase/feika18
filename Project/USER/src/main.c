/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"

/*
 * 系统频率，可查看board.h中的 FOSC 宏定义修改。
 * board.h文件中FOSC的值设置为0,则程序自动设置系统频率为33.1776MHZ
 * 在board_init中,已经将P54引脚设置为复位
 * 如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
 */


void main()
{
	int16 adc_err_read;
	int16 order_angle_read;
	board_init();

	// 初始化寄存器,勿删除此句代码。	
	// 此处编写用户代码(例如：外设初始化代码等)
	PID_struct_init(&pid_left_,POSITION_PID,10000,5000,30,0.5,22); //pid_left_PID初始化
	PID_struct_init(&pid_right_,POSITION_PID,10000,5000,28,0.4,11);	//pid_right_PID初始化
	My_Adc_Init(); //adc初始化
	Motor_Init(); //电机初始化
	Steer_Init(); //舵机初始化
	Brushless_Init(); //无刷初始化
	OLED_Init();    //OLED初始化
	LED=0;	//默认关闭LED
	BUZZER=0;	//默认关闭buzzer
	
//	Timer_Init();// 启动定时器3和定时器4,使能定时器4中断
	pit_timer_ms(TIM_4, 5);
	
    while(1)
	{
	
		Steer_Spin_limit(order_angle,angle_limit);
		Motor_Control(order_speed,order_speed);
	
		
		//上位机数值显示//
		adc_err_read =(int16)(adc_err * 100);
		order_angle_read =(int16)(order_angle);
		send_five_data(0xF1, adc1, adc2, adc3, adc4, adc_err_read);
		send_five_data(0xF2, Dir_judge_flag, order_angle_read, 0, 0, 0);
		
		
		delay_ms(5);
  }
}







