#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "headfile.h"

/****定义结构体****/
typedef struct{
	float current_speed;		//根据脉冲数量换算成的实际速度
	float target_speed ;		//目标速度
	float kp;								//电机PID参数，参数设置统一放到main函数开头
	float ki;
	float kd;
} akeman_t;

/****扩展变量****/
extern akeman_t akeman_left;
extern akeman_t akeman_right;
extern uint8 pid_flag;
extern float order_angle;
extern float order_speed;
extern float adc_err;
extern float adc_err_array[];
extern int16 window_flag;
extern int16 Roundabout_flag;
extern float ADC_error_a; 

/****函数声明****/
void Akeman_Control(float basic_speed,float target_angle);
void Motor_PID_Control(float current_l,float target_l,float current_r,float target_r);
float ADC_error_processing(float A,float B,float compensation);
float ADC_error_weight_filtering(void);
float ADC_error_window_filtering(void);
float ADC_error_acceleration(void);
void Roundabout_detection(void);
float Correct_Angle(float kp,float kd,float ki);
void PID_on(void);
void PID_off(void);

#endif