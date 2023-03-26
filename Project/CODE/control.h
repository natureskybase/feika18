#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "headfile.h"

/****����ṹ��****/
typedef struct{
	float current_speed;		//����������������ɵ�ʵ���ٶ�
	float target_speed ;		//Ŀ���ٶ�
	float kp;								//���PID��������������ͳһ�ŵ�main������ͷ
	float ki;
	float kd;
} akeman_t;

/****��չ����****/
extern akeman_t akeman_left;
extern akeman_t akeman_right;
extern uint8 pid_flag;
extern float order_angle;
extern float order_speed;

/****��������****/
void Akeman_Control(float basic_speed,float target_angle);
void Motor_PID_Control(float current_l,float target_l,float current_r,float target_r);
float Correct_Angle(float kp,float kd);
void PID_on(void);
void PID_off(void);

#endif