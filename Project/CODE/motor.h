#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_pwm.h"
#include "zf_tim.h"

/****�궨��****/
#define DUTY_LIMIT 30000		//�޷� 80%
#define PWM_L1 PWMB_CH4_P23
#define PWM_L2 PWMB_CH3_P22
#define PWM_R2 PWMB_CH2_P21
#define PWM_R1 PWMB_CH1_P20
#define CTIM_L CTIM0_P34
#define CTIM_R CTIM3_P04
#define DIR_L	P35
#define DIR_R P53

/****��չ����****/
extern int16 pulse_left;
extern int16 pulse_right;

/****��������****/
void Motor_Init(void);																	//���pwm��ʼ������������ʼ��
void Motor_Control(int32 left_duty,int32 right_duty);		//���Ƶ��PWMռ�ձ�,��Χ0-10000
void Motor_Shut(void);																	//�رյ��
void Encoder_Read(void);																//��ȡ����������
void Encoder_Clear(void);																//��������ֵ���

#endif