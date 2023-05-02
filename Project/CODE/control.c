#include "control.h"
#include "math.h"
#include "pid.h"


akeman_t akeman_left ;		//结构体变量，记录脉冲，实际速度，目标速度
akeman_t akeman_right;
uint8 pid_flag=0;					//电机闭环开启标志
float order_angle=0;			//目标角度
float order_speed=0;			//目标速度
float adc_err=0;         //电感误差
float adc_err_array[5];  //窗口电感误差
int16 window_flag=1;   	 //窗口标志位
int16 Roundabout_flag=0; //环岛检测标志位
float ADC_error_a=0;     //电感误差加速度(ms)
int16 lostline_flag;     //丢线标志
int16 lostline_dir;      //左,右丢线标志
int16 Dir_judge_flag;    //位置检测标志位

/**************************************************************************
函数功能：舵机角度和电机速度的协同控制
输入参数：小车基准速度（单位：mm/s），舵机角度（单位：°）
**************************************************************************/
void Akeman_Control(float basic_speed,float target_angle)
{
	float akeman_diff;	//左右轮相对basic_speed的差速
	
	/****读取编码器数值****/
	Encoder_Read();
	Encoder_Clear();
	
	/****计算两电机实际转速****/
	//编码器是1024线，4倍频，齿数为30
	//车轮直径56mm，与编码器对应齿数为68
	akeman_left.current_speed  = pulse_left *0.20603922;
	akeman_right.current_speed = pulse_right*0.20603922;
	
	/****计算两电机目标转速****/
	akeman_diff = basic_speed*(14.2*tan(target_angle*3.14/180.0))/20.0;	//相对差速
	akeman_left.target_speed  = basic_speed - akeman_diff;
	akeman_right.target_speed = basic_speed + akeman_diff;
	
	/****PID开启时控制电机和舵机****/
	if(pid_flag)
	{
		/****电机闭环****/
		Motor_PID_Control(akeman_left.current_speed ,akeman_left.target_speed ,akeman_right.current_speed ,akeman_right.target_speed);
		
		/****舵机转角****/
		Steer_Spin(target_angle);
	}
}

/**************************************************************************
函数功能：电机PID闭环控制
输入参数：左右轮的实际与目标转速
**************************************************************************/
void Motor_PID_Control(float current_l,float target_l,float current_r,float target_r)
{
	static float pwm_l=0;
	static float pwm_r=0;	
	
	
//	static float pwm_l=0,error_l=0,last_error_l=0,prev_error_l=0;
//	static float pwm_r=0,error_r=0,last_error_r=0,prev_error_r=0;
//	float kp_l=akeman_left.kp ,ki_l=akeman_left.ki ,kd_l=akeman_left.kd ;
//	float kp_r=akeman_right.kp,ki_r=akeman_right.ki,kd_r=akeman_right.kd;
//	
//	/****计算本次偏差****/
//	error_l = target_l - current_l;
//	error_r = target_r - current_r;
//	
//	/****若实际速度为0，则不加积分项****/
//	if(current_l==0) ki_l=0;
//	if(current_r==0) ki_r=0;
//	
//	/****增量式PID控制器****/
//	pwm_l += (int32)(kp_l*(error_l-last_error_l) + ki_l*error_l + kd_l*(error_l-2*last_error_l+prev_error_l))*100;
//	pwm_r += (int32)(kp_r*(error_r-last_error_r) + ki_r*error_r + kd_r*(error_r-2*last_error_r+prev_error_r))*100;
//	
//	/****限幅****/
//	if(pwm_l >  PWM_DUTY_MAX) pwm_l =  PWM_DUTY_MAX;
//	if(pwm_l < -PWM_DUTY_MAX) pwm_l = -PWM_DUTY_MAX;
//	if(pwm_r >  PWM_DUTY_MAX) pwm_r =  PWM_DUTY_MAX;
//	if(pwm_r < -PWM_DUTY_MAX) pwm_r = -PWM_DUTY_MAX;
//	
//	/****保存前两次偏差****/
//	prev_error_l = last_error_l;
//	prev_error_r = last_error_r;
//	last_error_l = error_l;
//	last_error_r = error_r;

	pwm_l=pid_calc(&pid_left_,current_l,target_l);
	pwm_r=pid_calc(&pid_right_,current_r,target_r);	
	Motor_Control((int32)pwm_l,(int32)pwm_r);
}
/**************************************************************************
函数功能：电感差比和归一化运算
输入参数：A,B为差比和权重
**************************************************************************/
float ADC_error_processing(float A,float B,float compensation)
{
//	考虑电感值误差处理
//	adc_err = (1/1.0*adc1-1/1.0*adc4)/(1/1.0*adc1+1/1.0*adc4);
//	adc_err = (adc4 - adc1)/10;
	adc_err = (A*(adc1-adc4)/(float)(adc1+adc4))+(B*(adc2-adc3)/(float)(adc2+adc3))+compensation/100;
	return adc_err;
}


/**************************************************************************
函数功能：一阶低通滤波
输入参数：无
**************************************************************************/
float ADC_error_weight_filtering(void)
{
	static float adc_err_last = 0;
	adc_err = (0.8)*adc_err+(0.2)*adc_err_last;
	adc_err_last = adc_err;
	
	return adc_err;
}

/**************************************************************************
函数功能：窗口滤波
输入参数：窗口数组
**************************************************************************/
float ADC_error_window_filtering(void)
{
	adc_err_array[window_flag]=adc_err;
	window_flag ++;
	if(window_flag==5)
		window_flag=0;
	if(adc_err_array[0]!=0)
		adc_err=(adc_err_array[1]+adc_err_array[2]+adc_err_array[3]+adc_err_array[4])/4;
	return adc_err;
}

/**************************************************************************
函数功能：ADC_err采集值加速度 //大概是没用的
输入参数：无
**************************************************************************/
float ADC_error_acceleration(void)
{
	ADC_error_a=(adc_err_array[3]+adc_err_array[4]-adc_err_array[1]-adc_err_array[2])/4*100;
	return ADC_error_a;
}
/**************************************************************************
函数功能：轨道状态检测
输入参数：无
**************************************************************************/
int16 Direct_judge(void)
{
	 static int16 res = 0;  // 小车当运行位置:0表示直道 
	
	//  阈值判断方向 //
	if(res == 0 && res != 2 && adc_err >= 0.29)
	res = 1;        // 左转弯
	
	else if(res == 0 && res != 1 && adc_err <= -0.29)
	res = 2;        // 右转弯
	
	else if(adc_err >= -0.29 && adc_err <= 0.29)
	res = 0;				// 直道(暂时未考虑环岛的影响)
	
	// 过渡状态的判断 //
	if(adc_err >= 0.6 && adc4 <= 500)
	res = 8;        //左转弯过渡状态
	
	else if(adc_err <= -0.6 && adc1 <= 500)
	res = 9;        //右转弯过渡状态
	
	return res;
}

/**************************************************************************
函数功能：丢线判断
输入参数：无
**************************************************************************/
void lost_line_judge(void)
{
	int8 i,L_count=0,R_count=0;// 左右计数
	lostline_dir = 0; 	
	if(lostline_flag==0)//丢线标志为0时进入
	{
		if(adc1 < 2000 && adc2 < 2000 && adc3 < 2000 && adc4 < 2000) //进入丢线条件
			lostline_flag=1;//丢线标志置1

		for(i=0;i<5;i++)//奇数
		{
			if(adc_err_array[i]<0)
			 R_count++;
			if(adc_err_array[i]>0)
			 L_count++;
		}
		if(L_count>R_count)
			lostline_dir=1;//左丢线标志
		if(L_count<R_count) 
			lostline_dir=2;//右丢线标志

	}
}
/**************************************************************************
函数功能：丢线处理
输入参数：无
**************************************************************************/
void lostline_deal(void)
{

	if(lostline_flag==1)
		{
			if(lostline_dir==1)//左丢线
				order_angle = -100;
			
			if(lostline_dir==2)//右丢线
				order_angle = 100;

			if(adc1 > 2000 && adc2 > 2000 && adc3 > 2000 && adc4 > 2000)
				lostline_flag=0;
		}
}

/**************************************************************************
函数功能：根据adc读值修正角度(舵机传统pid算法)
输入参数：PID控制参数
**************************************************************************/
float Correct_Angle(float kp,float kd,float ki)
{
	float target_angle;
	float target_angle_last;
	static float adc_err_last = 0;
	
//	pd控制方案
// 	target_angle = kp*adc_err + kd*(adc_err-adc_err_last);
//	kp+=adc_err*adc_err*0.1;//加入动态变化
	
//	pid控制方案
	target_angle = kp*adc_err + +kd*(adc_err-adc_err_last)+(ki+=ki*adc_err);
	target_angle=0.8*target_angle+0.2*target_angle_last;
	target_angle_last=target_angle;
	
	adc_err_last = adc_err;
	
	return target_angle;
}

/**************************************************************************
函数功能：开启电机闭环
**************************************************************************/
void PID_on(void)
{
	pid_flag = 1;
}

/**************************************************************************
函数功能：关闭电机闭环
**************************************************************************/
void PID_off(void)
{
	pid_flag = 0;
}
