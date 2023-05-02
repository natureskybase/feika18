#include "control.h"
#include "math.h"
#include "pid.h"


akeman_t akeman_left ;		//�ṹ���������¼���壬ʵ���ٶȣ�Ŀ���ٶ�
akeman_t akeman_right;
uint8 pid_flag=0;					//����ջ�������־
float order_angle=0;			//Ŀ��Ƕ�
float order_speed=0;			//Ŀ���ٶ�
float adc_err=0;         //������
float adc_err_array[5];  //���ڵ�����
int16 window_flag=1;   	 //���ڱ�־λ
int16 Roundabout_flag=0; //��������־λ
float ADC_error_a=0;     //��������ٶ�(ms)
int16 lostline_flag;     //���߱�־
int16 lostline_dir;      //��,�Ҷ��߱�־
int16 Dir_judge_flag;    //λ�ü���־λ

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
	adc_err = (A*(adc1-adc4)/(float)(adc1+adc4))+(B*(adc2-adc3)/(float)(adc2+adc3))+compensation/100;
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
�������ܣ�ADC_err�ɼ�ֵ���ٶ� //�����û�õ�
�����������
**************************************************************************/
float ADC_error_acceleration(void)
{
	ADC_error_a=(adc_err_array[3]+adc_err_array[4]-adc_err_array[1]-adc_err_array[2])/4*100;
	return ADC_error_a;
}
/**************************************************************************
�������ܣ����״̬���
�����������
**************************************************************************/
int16 Direct_judge(void)
{
	 static int16 res = 0;  // С��������λ��:0��ʾֱ�� 
	
	//  ��ֵ�жϷ��� //
	if(res == 0 && res != 2 && adc_err >= 0.29)
	res = 1;        // ��ת��
	
	else if(res == 0 && res != 1 && adc_err <= -0.29)
	res = 2;        // ��ת��
	
	else if(adc_err >= -0.29 && adc_err <= 0.29)
	res = 0;				// ֱ��(��ʱδ���ǻ�����Ӱ��)
	
	// ����״̬���ж� //
	if(adc_err >= 0.6 && adc4 <= 500)
	res = 8;        //��ת�����״̬
	
	else if(adc_err <= -0.6 && adc1 <= 500)
	res = 9;        //��ת�����״̬
	
	return res;
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
		if(adc1 < 2000 && adc2 < 2000 && adc3 < 2000 && adc4 < 2000) //���붪������
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

	if(lostline_flag==1)
		{
			if(lostline_dir==1)//����
				order_angle = -100;
			
			if(lostline_dir==2)//�Ҷ���
				order_angle = 100;

			if(adc1 > 2000 && adc2 > 2000 && adc3 > 2000 && adc4 > 2000)
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
	
//	pd���Ʒ���
// 	target_angle = kp*adc_err + kd*(adc_err-adc_err_last);
//	kp+=adc_err*adc_err*0.1;//���붯̬�仯
	
//	pid���Ʒ���
	target_angle = kp*adc_err + +kd*(adc_err-adc_err_last)+(ki+=ki*adc_err);
	target_angle=0.8*target_angle+0.2*target_angle_last;
	target_angle_last=target_angle;
	
	adc_err_last = adc_err;
	
	return target_angle;
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
