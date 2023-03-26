#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_pwm.h"
#include "zf_tim.h"

/****宏定义****/
#define DUTY_LIMIT 30000		//限幅 80%
#define PWM_L1 PWMB_CH4_P23
#define PWM_L2 PWMB_CH3_P22
#define PWM_R2 PWMB_CH2_P21
#define PWM_R1 PWMB_CH1_P20
#define CTIM_L CTIM0_P34
#define CTIM_R CTIM3_P04
#define DIR_L	P35
#define DIR_R P53

/****扩展变量****/
extern int16 pulse_left;
extern int16 pulse_right;

/****函数声明****/
void Motor_Init(void);																	//电机pwm初始化，编码器初始化
void Motor_Control(int32 left_duty,int32 right_duty);		//控制电机PWM占空比,范围0-10000
void Motor_Shut(void);																	//关闭电机
void Encoder_Read(void);																//读取编码器读数
void Encoder_Clear(void);																//编码器数值清空

#endif