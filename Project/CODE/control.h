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
extern int16 Roundabout_flag_L;
extern int16 Roundabout_flag_R;
extern int16 Roundabout_count;
extern float ADC_error_a;
extern int16 Dir_judge_flag;

/****函数声明****/
void Akeman_Control(float basic_speed,float target_angle);
void Motor_PID_Control(float current_l,float target_l,float current_r,float target_r);
float ADC_error_processing(float A,float B,float compensation);
float ADC_error_weight_filtering(void);
float ADC_error_window_filtering(void);
float ADC_error_acceleration(void);
int16 Direct_judge(void);
void lost_line_judge(void);
void lostline_deal(void);
float Correct_Angle(float kp,float kd,float ki);
float Dev_Err_Window_Filter_2(void);
float ADC_Err_Trendency(void);
int16 Direct_judge_Accele(void);
void PID_on(void);
void PID_off(void);

#endif