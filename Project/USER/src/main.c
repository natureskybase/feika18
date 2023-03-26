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




#define BUZZER P50	// �ߵ�ƽ����
#define LED P07     // �ߵ�ƽ����



/*
 * ϵͳƵ�ʣ��ɲ鿴board.h�е� FOSC �궨���޸ġ�
 * board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�����ϵͳƵ��Ϊ33.1776MHZ
 * ��board_init��,�Ѿ���P54��������Ϊ��λ
 * �����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����
 */


void main()
{
	board_init();			// ��ʼ���Ĵ���,��ɾ���˾���롣	
	// �˴���д�û�����(���磺�����ʼ�������)
	PID_struct_init(&pid_left_,POSITION_PID,10000,5000,0,0,0); //pid_left_PID��ʼ��
	PID_struct_init(&pid_right_,POSITION_PID,10000,5000,0,0,0);	//pid_right_PID��ʼ��
	My_Adc_Init(); //adc��ʼ��
	Motor_Init(); //�����ʼ��
	Steer_Init(); //�����ʼ��
	Brushless_Init(); //��ˢ��ʼ��
	OLED_Init();    //OLED��ʼ��
	Timer4_Init(10);
	
    while(1)
	{
		 // �˴���д��Ҫѭ��ִ�еĴ���
		
  }
}

