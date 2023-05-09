#include "control.h"
#include "math.h"
#include "pid.h"

akeman_t akeman_left ;		// 结构体变量，记录脉冲，实际速度，目标速度
akeman_t akeman_right;
uint8 pid_flag=0;					// 电机闭环开启标志


float order_angle=0;			// 目标角度
float order_speed=0;			// 目标速度
float ADC_error_a=0;     // 电感误差加速度(ms)

float adc_err=0;         // 电感误差
float adc_err_array[5];  // 窗口电感误差
int16 window_flag=1;   	 // 窗口标志位

int16 Roundabout_flag_L=0; // 左环岛检测标志位
int16 Roundabout_flag_R=0; // 右环岛检测标志位
int16 Roundabout_count=0;  // 环岛打死计时


int16 lostline_flag=0;     // 丢线标志
int16 lostline_dir=0;      // 左,右丢线标志
int16 lostline_count;      // 丢线打死计数

int16 Dir_judge_flag=0;    // 位置检测标志位


enum Car_State			// 用于表示小车当前状态
{
	Straight = 0,
	Turn_Left,
	Turn_Right,
	Round,		// 环岛
	Slope,		// 斜坡
};

int countADC = 0;
float Adc_Five_Del_1 = 0, Adc_Five_Del_2 = 0, Adc_Five_Del_3 = 0, Adc_Five_Del_4 = 0;
float Adc_Five_Mean_1_Las = 0, Adc_Five_Mean_2_Las = 0, Adc_Five_Mean_3_Las = 0, Adc_Five_Mean_4_Las = 0;
float Dev_errs[8] = {0, 0, 0, 0, 0, 0, 0, 0};		// 偏心率误差判断
int16 dev_flag = 0;   	 // 窗口标志位

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
	adc_err = (A*(adc1-adc4)/(float)(adc1+adc4))+(B*(adc2-adc3)/(float)(adc2+adc3))+compensation;
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
函数功能：窗口滤波2
输入参数：窗口数组
**************************************************************************/
float Dev_Err_Window_Filter_2(void)
{
	float sum = 0;
	int i =0;
	
	Dev_errs[dev_flag] = adc_err;
	dev_flag ++;
	
	if(dev_flag==8)
		window_flag=0;
	
	
	if(Dev_errs[0]==0)
	{
		for(i = 0; i< 8; i++)
		{
			Dev_errs[i] = adc_err;
		}
	}
	
	for(i = 0; i< 8; i++)
	{
		sum += Dev_errs[i];
	}
	
	adc_err = sum / 8;
	
	
	return adc_err;
}

/**************************************************************************
函数功能：ADC_Err的变化趋势

函数操作：
					判定ADC_Err目前处于什么变化趋势
					求相关系数


输入参数：无
**************************************************************************/
float ADC_Err_Trendency(void)
{
	float Up1 = 0, Down1 = 0, DownX1 = 0, DownY1 = 0;
	float Up2 = 0, Down2 = 0, DownX2 = 0, DownY2 = 0;
	float R1, R2;
	int i = 0;
	
	// 先求出 X，Y的平均值
	float X_Mean = 4.5, Y_Mean_1 = 0, Y_Mean_2 = 0;

	for(;i < 8; i++)
	{
		Y_Mean_1 += Dev_errs[i];
	}
	Y_Mean_1 /= 8;

	
	// 回归系数R：求分子
	for(i = 0;i < 8; i++)
	{	
		// (Xi - X_Mean) * (Yi - Y_Mean)
		Up1 += ((i + 1) - 4.5) * ( Dev_errs[i] - Y_Mean_1 );
	}
	
	// 回归系数R：求分母
	for(i = 0;i < 8; i++)
	{	
		//  (Xi - X_Mean) * (Xi - X_Mean)
		//  (Yi - Y_Mean) * (Yi - Y_Mean)
		DownX1 += ((i + 1) - 4.5) * ((i + 1) - 4.5);
		DownY1 += ( Dev_errs[i] - Y_Mean_1 ) * ( Dev_errs[i] - Y_Mean_1 );
	}
	DownX1 = sqrt(DownX1);
	DownY1 = sqrt(DownY1);
	Down1 = DownX1 * DownY1;
	
	R1 = Up1 / Down1;
	
	/*					求二次型的回归系数			*/
	
	
	for(i=0 ;i < 8; i++)
	{
		Y_Mean_2 += sqrt(Dev_errs[i]);
	}
	Y_Mean_2 /= 8;

	
	// 回归系数R：求分子
	for(i = 0;i < 8; i++)
	{	
		// (Xi - X_Mean) * (Yi - Y_Mean)
		Up2 += ((i + 1) - 4.5) * ( sqrt(Dev_errs[i]) - Y_Mean_2 );
	}
	
	// 回归系数R：求分母
	for(i = 0;i < 8; i++)
	{	
		//  (Xi - X_Mean) * (Xi - X_Mean)
		//  (Yi - Y_Mean) * (Yi - Y_Mean)
		DownX2 += ((i + 1) - 4.5) * ((i + 1) - 4.5);
		DownY2 += ( sqrt(Dev_errs[i]) - Y_Mean_2 ) * ( sqrt(Dev_errs[i]) - Y_Mean_2 );
	}
	
	DownX2 = sqrt(DownX2);
	DownY2 = sqrt(DownY2);
	Down2 = DownX2 * DownY2;
	
	R2 = Up2 / Down2;
	
	if(fabs(R1) > fabs(R2))		return 0; // 线性
	else if (R2 < 0)					return 1; // 左二次性
	else 											return 2; // 右二次性
}



/**************************************************************************
函数功能：轨道状态检测
输入参数：无
**************************************************************************/
int16 Direct_judge(void)
{
	 static int16 res = 0;  // 小车当运行位置:0表示直道 
	
	//  阈值判断方向 //
	if(adc_err >= -0.29 && adc_err <= 0.29)
	res = 0;				// 直道
	
	if(res == 0 && res != 2 && adc_err >= 0.29)
	res = 1;        // 左转弯
	
	if(res == 0 && res != 1 && adc_err <= -0.29)
	res = 2;        // 右转弯

	
	// 环岛检测 //
	if(adc1>3900 && adc2 >3900)
	{
		if(adc3 < 3900 && adc4 < 3900)
	  res = 3;    // 左环岛入环
	}
	
	if(adc3>3900 && adc4 >3900)
	{
		if(adc1 < 3900 && adc2 < 3900)
		res = 4;    // 右环岛入环
	}
	
	// 过渡状态的判断 //
	if(adc_err >= 0.6 && adc4 <= 500)
	res = 8;        //左转弯过渡状态
	
	else if(adc_err <= -0.6 && adc1 <= 500)
	res = 9;        //右转弯过渡状态
	
	return res;
}

/**************************************************************************
函数功能：环岛处理函数
输入参数：无
**************************************************************************/
void Roundabout_deal(void)
{
	delay_ms(100);//打角延时
	Roundabout_count = 200; //时间计算为Roundabout_count * 0.01 (s)
	if(Roundabout_flag_L == 1)
	{
		while(Roundabout_count-- > 0)
		{
			Steer_Spin(-28);
			Motor_Control(2550,2550);
			delay_ms(10);
		}
		Roundabout_flag_L = 0;
	}
		
	if(Roundabout_flag_R == 1)
	{
		while(Roundabout_count-- > 0)
		{
			Steer_Spin(28);
			Motor_Control(2550,2550);
			delay_ms(10);
		}
		Roundabout_flag_R = 0;
	}
}
/**************************************************************************
函数功能：轨道状态检测(Version 2)
输入参数：无
**************************************************************************/
int16 Direct_judge_Accele(void)
{
	 static enum Car_State state = Straight;  // 小车状态：默认直行
	
	/*		直线的判定：
									偏心率一般会很小
									若有偏移现象，一般是由车体不正造成， 偏心率将线性变化
	*/
	
	if( fabs(adc_err) < 0.2 && ADC_Err_Trendency() == 0)//if( fabs(adc_err) < 0.2 && ADC_Err_Trendency(1.2) == 0)
	{
		state = Straight;
	}
	
	/*		环岛的判定：
									偏心率一般显著比直线大
									所有电感都在增大
	*/
	if( fabs(adc_err) > 0.2 && Adc_Five_Del_1 > 0 && Adc_Five_Del_2 > 0 && Adc_Five_Del_3 > 0 && Adc_Five_Del_4 > 0)
	{
		state = Round;
	}
	
	
	/*		进入转弯的判定：
									车当前仍在直行
									偏心率一般会比直线明显偏大
									偏心率将二次变化（x方 + y方 = R方）
	*/
	
	if( adc_err < -0.2 && ADC_Err_Trendency() == 1 && state == Straight)
	{
		state = Turn_Left;
	}
	if( adc_err > 0.2 && ADC_Err_Trendency() == 2 && state == Straight)
	{
		state = Turn_Right;
	}
	
	
	/*		离开转弯的判定：
									车当前仍在弯道
									偏心率将二次变化（x方 + y方 = R方）
									偏心率变化方向与当前转弯方向相反
	*/
	
	if( fabs(adc_err) < 0.2 && ADC_Err_Trendency() == 2 && state == Turn_Left)
	{
		state = Straight;
	}
	if( fabs(adc_err) < 0.2 && ADC_Err_Trendency() == 1 && state == Turn_Right)
	{
		state = Straight;
	}
	
	
	return state;
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
		if(adc1 < 3000 && adc2 < 3000 && adc3 < 3000 && adc4 < 3000) //进入丢线条件
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
	lostline_count = 200; //时间计算为Roundabout_count * 0.01 (s)
	if(lostline_flag==1)
		{
			if(lostline_dir==1)//左丢线
			{
				while(Roundabout_count-- > 0)
				{
				Steer_Spin(-28);
				Motor_Control(2550,2550);
				delay_ms(10);
				}
			}
		
			if(lostline_dir==2)//右丢线
			{
				while(Roundabout_count-- > 0)
				{
				Steer_Spin(-28);
				Motor_Control(2550,2550);
				delay_ms(10);
				}
			}

			if(adc1 > 1000 && adc2 > 1000 && adc3 > 1000 && adc4 > 1000)
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
	static float integral_err = 0;
	
//	pd控制方案
// 	target_angle = kp*adc_err + kd*(adc_err-adc_err_last);
//	kp+=adc_err*adc_err*0.1;//加入动态变化
	
//	pid控制方案
	target_angle = kp*adc_err + kd*(adc_err-adc_err_last)+(integral_err+=ki*adc_err);
	target_angle=0.8*target_angle+0.2*target_angle_last;
	target_angle_last=target_angle;
	
	adc_err_last = adc_err;
	
	return target_angle;
}
/*********************************************************************

函数功能： 根据 ADC值 修正 舵机 角度
输入参数： 容忍度
					方向误差：Kp Ki Kd
					位置误差：Kp Ki Kd
					传统PID：Kp Kd
返回值： 舵机的最终调整角度
******************************************************************/
float Dev_Tolerant_Correc_Ang(
		float Tolerance,
		float DKp,
		float DKi,
		float Dkd,
		float PKp,
		float PKi,
		float PKd,
		float kp,
		float kd)
{
	
	static float Itg_dev_err = 0.0;//积分调节，其实可要可不要，设 0 即可
	static float Itg_posi_err = 0.0;
		
	static float dev_last;//静态声明 前次偏心率
	static float dev_last2;//静态声明 再前次偏心率
	//可以不用，但是不能不写
	
	float dev, Delta_dev, Delta_dev_las; //声明 偏心率,偏心率变化量，前一次偏心率变化量
	
	float Direc_angle, Posi_angle;//声明 方向角度，位置角度
	
	
	dev = adc_err;  //计算偏心率（应该在 -1 ~ 1之间）
	
	//如果偏心率已经大于 忍耐度 了，直接使用 传统位置 pid
	if(fabs(dev) >= Tolerance)
	{
		return Correct_Angle(kp,kd, 0);
	}
	
	//能跑到这里，肯定 是没有 大于 忍耐度 的

	
	Delta_dev = dev - dev_last;// 计算偏心率变化量
	Delta_dev_las = dev_last - dev_last2;
	
	//对方向PID目标：使 偏心率变化量(Delta_dev) 为零，则 车 走直线 或者 以恒定 曲率半径 过弯
	//由于 Delta_dev 的调节目标是 0 ，直接用本身当Error就行
	
	Itg_dev_err += Delta_dev;// 累计 积分误差
	Direc_angle = DKp * Delta_dev + DKi * Itg_dev_err + Dkd * (Delta_dev - Delta_dev_las);// 计算PID
	
	
	//对位置PID目标：使 偏心率（dev） 为零，即让 走直线 但是 线不在中心 的车 《!缓慢!》 回到中心
	//参数必须远小于方向PID，不然就变成普通位置PID了
	
	Itg_posi_err += dev;// 累计 位置误差
	Posi_angle = PKp * dev + PKi * Itg_posi_err + PKd * Delta_dev;// 计算PID
	
	dev_last2 = dev_last;
	dev_last = dev;
	
	return Direc_angle + Posi_angle; // 叠加返回
	
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
