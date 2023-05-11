/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"

/*
 * ϵͳƵ�ʣ��ɲ鿴board.h�е� FOSC �궨���޸ġ�
 * board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�����ϵͳƵ��Ϊ33.1776MHZ
 * ��board_init��,�Ѿ���P54��������Ϊ��λ
 * �����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����
 */


void main()
{
	int16 adc_err_read;
	int16 order_angle_read;
	board_init();

	// ��ʼ���Ĵ���,��ɾ���˾���롣	
	// �˴���д�û�����(���磺�����ʼ�������)
	PID_struct_init(&pid_left_,POSITION_PID,10000,5000,30,0.5,22); //pid_left_PID��ʼ��
	PID_struct_init(&pid_right_,POSITION_PID,10000,5000,28,0.4,11);	//pid_right_PID��ʼ��
	My_Adc_Init(); //adc��ʼ��
	Motor_Init(); //�����ʼ��
	Steer_Init(); //�����ʼ��
	Brushless_Init(); //��ˢ��ʼ��
	OLED_Init();    //OLED��ʼ��
	LED=0;	//Ĭ�Ϲر�LED
	BUZZER=0;	//Ĭ�Ϲر�buzzer
	
//	Timer_Init();// ������ʱ��3�Ͷ�ʱ��4,ʹ�ܶ�ʱ��4�ж�
	pit_timer_ms(TIM_4, 5);
	
    while(1)
	{
	
		Steer_Spin_limit(order_angle,angle_limit);
		Motor_Control(order_speed,order_speed);
	
		
		//��λ����ֵ��ʾ//
		adc_err_read =(int16)(adc_err * 100);
		order_angle_read =(int16)(order_angle);
		send_five_data(0xF1, adc1, adc2, adc3, adc4, adc_err_read);
		send_five_data(0xF2, Dir_judge_flag, order_angle_read, 0, 0, 0);
		
		
		delay_ms(5);
  }
}







