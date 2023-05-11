#include "adc.h"

/****ADC��ȡֵ****/
int16 adc1 = 0; //��ֵ1800
int16 adc2 = 0; //��ֵ2800
int16 adc3 = 0; //��ֵ2800
int16 adc4 = 0; //��ֵ1800
int16 adc5 = 0; 
int16 adc6 = 0;
int16 adc7 = 0; 
int16 adc8 = 0;

//�� 1 Ϊ�±꿪ʼ//
int16 adc1_array[11] = {0,0,0,0,0,0,0,0,0,0,0}; //��ֵ1800
int16 adc2_array[11] = {0,0,0,0,0,0,0,0,0,0,0}; //��ֵ2800
int16 adc3_array[11] = {0,0,0,0,0,0,0,0,0,0,0}; //��ֵ2800
int16 adc4_array[11] = {0,0,0,0,0,0,0,0,0,0,0}; //��ֵ1800
int16 adc5_array[11] = {0,0,0,0,0,0,0,0,0,0,0};
int16 adc6_array[11] = {0,0,0,0,0,0,0,0,0,0,0};
int16 adc7_array[11] = {0,0,0,0,0,0,0,0,0,0,0}; 
int16 adc8_array[11] = {0,0,0,0,0,0,0,0,0,0,0};
/**************************************************************************
�������ܣ�adc��ʼ��
**************************************************************************/
void My_Adc_Init(void)
{
	adc_init(ADC_1, ADC_SYSclk_DIV_2);	//ADCʱ��Ƶ�ʣ�SYSclk/2
	adc_init(ADC_2, ADC_SYSclk_DIV_2);	//ADCʱ��Ƶ�ʣ�SYSclk/2
	adc_init(ADC_3, ADC_SYSclk_DIV_2);	//ADCʱ��Ƶ�ʣ�SYSclk/2
	adc_init(ADC_4, ADC_SYSclk_DIV_2);	//ADCʱ��Ƶ�ʣ�SYSclk/2
//���õ��	��ʼ��
	adc_init(S_ADC1, ADC_SYSclk_DIV_2);	//ADCʱ��Ƶ�ʣ�SYSclk/2
	adc_init(S_ADC5, ADC_SYSclk_DIV_2);	//ADCʱ��Ƶ�ʣ�SYSclk/2
	adc_init(S_ADC7, ADC_SYSclk_DIV_2);	//ADCʱ��Ƶ�ʣ�SYSclk/2
	adc_init(S_ADC11, ADC_SYSclk_DIV_2);	//ADCʱ��Ƶ�ʣ�SYSclk/2
}

/**************************************************************************
�������ܣ�adc��ֵ�˲�
���������adcͨ����ʱ��Ƶ�ʣ���ȡ����
**************************************************************************/
uint16 adc_mean_filter(ADCN_enum adcn,ADC_SPEED_enum speed, uint8 count)
{
	uint8 i;
	uint32 sum;

	sum = 0;
	for(i=0; i<count; i++)
	{
			sum += adc_once(adcn, speed);
	}

	sum = sum/count;
	return (uint16)sum;
}

/**************************************************************************
�������ܣ�adc��ȡ
**************************************************************************/
void Read_Adc_Value(void)
{
	uint8 adc_read_count = 10;//adcx_array[0]��Ϊ�ڱ�
	
//	const uint16 adc1_last,adc2_last,adc3_last,adc4_last;
	adc1 = adc_mean_filter(ADC_1,ADC_12BIT,5);		//��Ӧ�����ϵ�IN1
	adc2 = adc_mean_filter(ADC_2,ADC_12BIT,5);		//��Ӧ�����ϵ�IN2
	adc3 = adc_mean_filter(ADC_3,ADC_12BIT,5);		//��Ӧ�����ϵ�IN3
	adc4 = adc_mean_filter(ADC_4,ADC_12BIT,5);		//��Ӧ�����ϵ�IN4
//���õ��		
//	adc5 = adc_mean_filter(S_ADC1,ADC_12BIT,5);   //��Ӧ�����ϵ�IN5
//	adc6 = adc_mean_filter(S_ADC5,ADC_12BIT,5);		//��Ӧ�����ϵ�IN6
//	adc7 = adc_mean_filter(S_ADC7,ADC_12BIT,5);		//��Ӧ�����ϵ�IN7
//	adc8 = adc_mean_filter(S_ADC11,ADC_12BIT,5);	//��Ӧ�����ϵ�IN8
	
	adc1_array[0] = adc1;
	adc2_array[0] = adc2;
	adc3_array[0] = adc3;
	adc4_array[0] = adc4;
//	adc5_array[0] = adc5;
//	adc6_array[0] = adc6;
//	adc7_array[0] = adc7;
//	adc8_array[0] = adc8;
	for(adc_read_count; adc_read_count > 0; adc_read_count--)
	{
		adc1_array[adc_read_count] = adc1_array[adc_read_count-1];
		adc2_array[adc_read_count] = adc2_array[adc_read_count-1];
		adc3_array[adc_read_count] = adc3_array[adc_read_count-1];
		adc4_array[adc_read_count] = adc4_array[adc_read_count-1];
//		adc5_array[adc_read_count] = adc5_array[adc_read_count-1];
//		adc6_array[adc_read_count] = adc6_array[adc_read_count-1];
//		adc7_array[adc_read_count] = adc7_array[adc_read_count-1];
//		adc8_array[adc_read_count] = adc8_array[adc_read_count-1];
	}
	
	
}


