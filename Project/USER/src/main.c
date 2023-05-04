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
	Timer4_Init(5);
	LED=0;	//Ĭ�Ϲر�LED
	BUZZER=0;	//Ĭ�Ϲر�buzzer
	
//	pwm_init(PWMB_CH4_P23,10000,0);
//	uart_init(UART_2,UART2_RX_P10,UART2_TX_P11,38400, TIM_2);
    while(1)
	{
		
		
		printf("Dir_judge_flag = %d  ADC_1=%d  ADC_2=%d  ADC_3=%d  ADC_4=%d  adc_err=%f\r\n",Dir_judge_flag,adc1,adc2,adc3,adc4,adc_err);
//		printf("adc[1]=%f  adc[2]=%f  adc[3]=%f  adc[4]=%f err=%f \r\n",adc_err_array[1],adc_err_array[2],adc_err_array[3],adc_err_array[4],adc_err);
//		printf("%f %f\r\n",Correct_Angle(130,1200,0),adc_err);
//		printf("Dir_judge_flag = %d \r\n",Dir_judge_flag);
//		printf("%f %f\r\n",akeman_left.current_speed,akeman_right.current_speed);
		
		adc_err_read =(int16)(adc_err*100);
		send_five_data(0xF1,adc1,adc2,adc3,adc4,adc_err_read);
		delay_ms(10);
  }
}

