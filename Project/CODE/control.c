#include "control.h"
#include "math.h"
#include "pid.h"

akeman_t akeman_left ;		// �ṹ���������¼���壬ʵ���ٶȣ�Ŀ���ٶ�
akeman_t akeman_right;
uint8 pid_flag=0;					// ����ջ�������־


float order_angle=0;			// Ŀ��Ƕ�
float order_speed=0;			// Ŀ���ٶ�
float ADC_error_a=0;     // ��������ٶ�(ms)

float adc_err=0;         // ������
float adc_err_array[5];  // ���ڵ�����
int16 window_flag=1;   	 // ���ڱ�־λ

int16 Roundabout_flag_L=0; // �󻷵�����־λ
int16 Roundabout_flag_R=0; // �һ�������־λ
int16 Roundabout_count=0;  // ����������ʱ


int16 lostline_flag=0;     // ���߱�־
int16 lostline_dir=0;      // ��,�Ҷ��߱�־
int16 lostline_count;      // ���ߴ�������

int16 Dir_judge_flag=0;    // λ�ü���־λ


enum Car_State			// ���ڱ�ʾС����ǰ״̬
{
	Straight = 0,
	Turn_Left,
	Turn_Right,
	Round,		// ����
	Slope,		// б��
};

int countADC = 0;
float Adc_Five_Del_1 = 0, Adc_Five_Del_2 = 0, Adc_Five_Del_3 = 0, Adc_Five_Del_4 = 0;
float Adc_Five_Mean_1_Las = 0, Adc_Five_Mean_2_Las = 0, Adc_Five_Mean_3_Las = 0, Adc_Five_Mean_4_Las = 0;
float Dev_errs[8] = {0, 0, 0, 0, 0, 0, 0, 0};		// ƫ��������ж�
int16 dev_flag = 0;   	 // ���ڱ�־λ

/**************************************************************************
�������ܣ�����ǶȺ͵���ٶȵ�Эͬ����
���������С����׼�ٶȣ���λ��mm/s��������Ƕȣ���λ���㣩
**************************************************************************/
void Akeman_Control(float basic_speed,float target_angle)
{
	float akeman_diff;	//���������basic_speed�Ĳ���
	
	/****��ȡ��������ֵ****/
	Encoder_Read();
	Encoder_Clear();
	
	/****���������ʵ��ת��****/
	//��������1024�ߣ�4��Ƶ������Ϊ30
	//����ֱ��56mm�����������Ӧ����Ϊ68
	akeman_left.current_speed  = pulse_left *0.20603922;
	akeman_right.current_speed = pulse_right*0.20603922;
	
	/****���������Ŀ��ת��****/
	akeman_diff = basic_speed*(14.2*tan(target_angle*3.14/180.0))/20.0;	//��Բ���
	akeman_left.target_speed  = basic_speed - akeman_diff;
	akeman_right.target_speed = basic_speed + akeman_diff;
	
	/****PID����ʱ���Ƶ���Ͷ��****/
	if(pid_flag)
	{
		/****����ջ�****/
		Motor_PID_Control(akeman_left.current_speed ,akeman_left.target_speed ,akeman_right.current_speed ,akeman_right.target_speed);
		
		/****���ת��****/
		Steer_Spin(target_angle);
	}
}


/**************************************************************************
�������ܣ����PID�ջ�����
��������������ֵ�ʵ����Ŀ��ת��
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
//	/****���㱾��ƫ��****/
//	error_l = target_l - current_l;
//	error_r = target_r - current_r;
//	
//	/****��ʵ���ٶ�Ϊ0���򲻼ӻ�����****/
//	if(current_l==0) ki_l=0;
//	if(current_r==0) ki_r=0;
//	
//	/****����ʽPID������****/
//	pwm_l += (int32)(kp_l*(error_l-last_error_l) + ki_l*error_l + kd_l*(error_l-2*last_error_l+prev_error_l))*100;
//	pwm_r += (int32)(kp_r*(error_r-last_error_r) + ki_r*error_r + kd_r*(error_r-2*last_error_r+prev_error_r))*100;
//	
//	/****�޷�****/
//	if(pwm_l >  PWM_DUTY_MAX) pwm_l =  PWM_DUTY_MAX;
//	if(pwm_l < -PWM_DUTY_MAX) pwm_l = -PWM_DUTY_MAX;
//	if(pwm_r >  PWM_DUTY_MAX) pwm_r =  PWM_DUTY_MAX;
//	if(pwm_r < -PWM_DUTY_MAX) pwm_r = -PWM_DUTY_MAX;
//	
//	/****����ǰ����ƫ��****/
//	prev_error_l = last_error_l;
//	prev_error_r = last_error_r;
//	last_error_l = error_l;
//	last_error_r = error_r;

	pwm_l=pid_calc(&pid_left_,current_l,target_l);
	pwm_r=pid_calc(&pid_right_,current_r,target_r);	
	Motor_Control((int32)pwm_l,(int32)pwm_r);
}


/**************************************************************************
�������ܣ���в�Ⱥ͹�һ������
���������A,BΪ��Ⱥ�Ȩ��
**************************************************************************/
float ADC_error_processing(float A,float B,float compensation)
{
//	���ǵ��ֵ����
//	adc_err = (1/1.0*adc1-1/1.0*adc4)/(1/1.0*adc1+1/1.0*adc4);
//	adc_err = (adc4 - adc1)/10;
	adc_err = (A*(adc1-adc4)/(float)(adc1+adc4))+(B*(adc2-adc3)/(float)(adc2+adc3))+compensation;
	return adc_err;
}


/**************************************************************************
�������ܣ�һ�׵�ͨ�˲�
�����������
**************************************************************************/
float ADC_error_weight_filtering(void)
{
	static float adc_err_last = 0;
	adc_err = (0.8)*adc_err+(0.2)*adc_err_last;
	adc_err_last = adc_err;
	
	return adc_err;
}


/**************************************************************************
�������ܣ������˲�
�����������������
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
�������ܣ������˲�2
�����������������
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
�������ܣ�ADC_Err�ı仯����

����������
					�ж�ADC_ErrĿǰ����ʲô�仯����
					�����ϵ��


�����������
**************************************************************************/
float ADC_Err_Trendency(void)
{
	float Up1 = 0, Down1 = 0, DownX1 = 0, DownY1 = 0;
	float Up2 = 0, Down2 = 0, DownX2 = 0, DownY2 = 0;
	float R1, R2;
	int i = 0;
	
	// ����� X��Y��ƽ��ֵ
	float X_Mean = 4.5, Y_Mean_1 = 0, Y_Mean_2 = 0;

	for(;i < 8; i++)
	{
		Y_Mean_1 += Dev_errs[i];
	}
	Y_Mean_1 /= 8;

	
	// �ع�ϵ��R�������
	for(i = 0;i < 8; i++)
	{	
		// (Xi - X_Mean) * (Yi - Y_Mean)
		Up1 += ((i + 1) - 4.5) * ( Dev_errs[i] - Y_Mean_1 );
	}
	
	// �ع�ϵ��R�����ĸ
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
	
	/*					������͵Ļع�ϵ��			*/
	
	
	for(i=0 ;i < 8; i++)
	{
		Y_Mean_2 += sqrt(Dev_errs[i]);
	}
	Y_Mean_2 /= 8;

	
	// �ع�ϵ��R�������
	for(i = 0;i < 8; i++)
	{	
		// (Xi - X_Mean) * (Yi - Y_Mean)
		Up2 += ((i + 1) - 4.5) * ( sqrt(Dev_errs[i]) - Y_Mean_2 );
	}
	
	// �ع�ϵ��R�����ĸ
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
	
	if(fabs(R1) > fabs(R2))		return 0; // ����
	else if (R2 < 0)					return 1; // �������
	else 											return 2; // �Ҷ�����
}



/**************************************************************************
�������ܣ����״̬���
�����������
**************************************************************************/
int16 Direct_judge(void)
{
	 static int16 res = 0;  // С��������λ��:0��ʾֱ�� 
	
	//  ��ֵ�жϷ��� //
	if(adc_err >= -0.29 && adc_err <= 0.29)
	res = 0;				// ֱ��
	
	if(res == 0 && res != 2 && adc_err >= 0.29)
	res = 1;        // ��ת��
	
	if(res == 0 && res != 1 && adc_err <= -0.29)
	res = 2;        // ��ת��

	
	// ������� //
	if(adc1>3900 && adc2 >3900)
	{
		if(adc3 < 3900 && adc4 < 3900)
	  res = 3;    // �󻷵��뻷
	}
	
	if(adc3>3900 && adc4 >3900)
	{
		if(adc1 < 3900 && adc2 < 3900)
		res = 4;    // �һ����뻷
	}
	
	// ����״̬���ж� //
	if(adc_err >= 0.6 && adc4 <= 500)
	res = 8;        //��ת�����״̬
	
	else if(adc_err <= -0.6 && adc1 <= 500)
	res = 9;        //��ת�����״̬
	
	return res;
}

/**************************************************************************
�������ܣ�����������
�����������
**************************************************************************/
void Roundabout_deal(void)
{
	delay_ms(100);//�����ʱ
	Roundabout_count = 200; //ʱ�����ΪRoundabout_count * 0.01 (s)
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
�������ܣ����״̬���(Version 2)
�����������
**************************************************************************/
int16 Direct_judge_Accele(void)
{
	 static enum Car_State state = Straight;  // С��״̬��Ĭ��ֱ��
	
	/*		ֱ�ߵ��ж���
									ƫ����һ����С
									����ƫ������һ�����ɳ��岻����ɣ� ƫ���ʽ����Ա仯
	*/
	
	if( fabs(adc_err) < 0.2 && ADC_Err_Trendency() == 0)//if( fabs(adc_err) < 0.2 && ADC_Err_Trendency(1.2) == 0)
	{
		state = Straight;
	}
	
	/*		�������ж���
									ƫ����һ��������ֱ�ߴ�
									���е�ж�������
	*/
	if( fabs(adc_err) > 0.2 && Adc_Five_Del_1 > 0 && Adc_Five_Del_2 > 0 && Adc_Five_Del_3 > 0 && Adc_Five_Del_4 > 0)
	{
		state = Round;
	}
	
	
	/*		����ת����ж���
									����ǰ����ֱ��
									ƫ����һ����ֱ������ƫ��
									ƫ���ʽ����α仯��x�� + y�� = R����
	*/
	
	if( adc_err < -0.2 && ADC_Err_Trendency() == 1 && state == Straight)
	{
		state = Turn_Left;
	}
	if( adc_err > 0.2 && ADC_Err_Trendency() == 2 && state == Straight)
	{
		state = Turn_Right;
	}
	
	
	/*		�뿪ת����ж���
									����ǰ�������
									ƫ���ʽ����α仯��x�� + y�� = R����
									ƫ���ʱ仯�����뵱ǰת�䷽���෴
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
�������ܣ������ж�
�����������
**************************************************************************/
void lost_line_judge(void)
{
	int8 i,L_count=0,R_count=0;// ���Ҽ���
	lostline_dir = 0; 	
	if(lostline_flag==0)//���߱�־Ϊ0ʱ����
	{
		if(adc1 < 3000 && adc2 < 3000 && adc3 < 3000 && adc4 < 3000) //���붪������
			lostline_flag=1;//���߱�־��1

		for(i=0;i<5;i++)//����
		{
			if(adc_err_array[i]<0)
			 R_count++;
			if(adc_err_array[i]>0)
			 L_count++;
		}
		if(L_count>R_count)
			lostline_dir=1;//���߱�־
		if(L_count<R_count) 
			lostline_dir=2;//�Ҷ��߱�־

	}
}


/**************************************************************************
�������ܣ����ߴ���
�����������
**************************************************************************/
void lostline_deal(void)
{
	lostline_count = 200; //ʱ�����ΪRoundabout_count * 0.01 (s)
	if(lostline_flag==1)
		{
			if(lostline_dir==1)//����
			{
				while(Roundabout_count-- > 0)
				{
				Steer_Spin(-28);
				Motor_Control(2550,2550);
				delay_ms(10);
				}
			}
		
			if(lostline_dir==2)//�Ҷ���
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
�������ܣ�����adc��ֵ�����Ƕ�(�����ͳpid�㷨)
���������PID���Ʋ���
**************************************************************************/
float Correct_Angle(float kp,float kd,float ki)
{
	float target_angle;
	float target_angle_last;
	static float adc_err_last = 0;
	static float integral_err = 0;
	
//	pd���Ʒ���
// 	target_angle = kp*adc_err + kd*(adc_err-adc_err_last);
//	kp+=adc_err*adc_err*0.1;//���붯̬�仯
	
//	pid���Ʒ���
	target_angle = kp*adc_err + kd*(adc_err-adc_err_last)+(integral_err+=ki*adc_err);
	target_angle=0.8*target_angle+0.2*target_angle_last;
	target_angle_last=target_angle;
	
	adc_err_last = adc_err;
	
	return target_angle;
}
/*********************************************************************

�������ܣ� ���� ADCֵ ���� ��� �Ƕ�
��������� ���̶�
					������Kp Ki Kd
					λ����Kp Ki Kd
					��ͳPID��Kp Kd
����ֵ�� ��������յ����Ƕ�
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
	
	static float Itg_dev_err = 0.0;//���ֵ��ڣ���ʵ��Ҫ�ɲ�Ҫ���� 0 ����
	static float Itg_posi_err = 0.0;
		
	static float dev_last;//��̬���� ǰ��ƫ����
	static float dev_last2;//��̬���� ��ǰ��ƫ����
	//���Բ��ã����ǲ��ܲ�д
	
	float dev, Delta_dev, Delta_dev_las; //���� ƫ����,ƫ���ʱ仯����ǰһ��ƫ���ʱ仯��
	
	float Direc_angle, Posi_angle;//���� ����Ƕȣ�λ�ýǶ�
	
	
	dev = adc_err;  //����ƫ���ʣ�Ӧ���� -1 ~ 1֮�䣩
	
	//���ƫ�����Ѿ����� ���Ͷ� �ˣ�ֱ��ʹ�� ��ͳλ�� pid
	if(fabs(dev) >= Tolerance)
	{
		return Correct_Angle(kp,kd, 0);
	}
	
	//���ܵ�����϶� ��û�� ���� ���Ͷ� ��

	
	Delta_dev = dev - dev_last;// ����ƫ���ʱ仯��
	Delta_dev_las = dev_last - dev_last2;
	
	//�Է���PIDĿ�꣺ʹ ƫ���ʱ仯��(Delta_dev) Ϊ�㣬�� �� ��ֱ�� ���� �Ժ㶨 ���ʰ뾶 ����
	//���� Delta_dev �ĵ���Ŀ���� 0 ��ֱ���ñ���Error����
	
	Itg_dev_err += Delta_dev;// �ۼ� �������
	Direc_angle = DKp * Delta_dev + DKi * Itg_dev_err + Dkd * (Delta_dev - Delta_dev_las);// ����PID
	
	
	//��λ��PIDĿ�꣺ʹ ƫ���ʣ�dev�� Ϊ�㣬���� ��ֱ�� ���� �߲������� �ĳ� ��!����!�� �ص�����
	//��������ԶС�ڷ���PID����Ȼ�ͱ����ͨλ��PID��
	
	Itg_posi_err += dev;// �ۼ� λ�����
	Posi_angle = PKp * dev + PKi * Itg_posi_err + PKd * Delta_dev;// ����PID
	
	dev_last2 = dev_last;
	dev_last = dev;
	
	return Direc_angle + Posi_angle; // ���ӷ���
	
}

/**************************************************************************
�������ܣ���������ջ�
**************************************************************************/
void PID_on(void)
{
	pid_flag = 1;
}


/**************************************************************************
�������ܣ��رյ���ջ�
**************************************************************************/
void PID_off(void)
{
	pid_flag = 0;
}
