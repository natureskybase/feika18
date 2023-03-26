#include "adc.h"

/****ADC读取值****/
uint16 adc1 = 0; 
uint16 adc2 = 0;
uint16 adc3 = 0; 
uint16 adc4 = 0;
uint16 adc5 = 0; 
uint16 adc6 = 0;
uint16 adc7 = 0; 
uint16 adc8 = 0;


/**************************************************************************
函数功能：adc初始化
**************************************************************************/
void My_Adc_Init(void)
{
	adc_init(ADC_1, ADC_SYSclk_DIV_2);	//ADC时钟频率：SYSclk/2
	adc_init(ADC_2, ADC_SYSclk_DIV_2);	//ADC时钟频率：SYSclk/2
	adc_init(ADC_3, ADC_SYSclk_DIV_2);	//ADC时钟频率：SYSclk/2
	adc_init(ADC_4, ADC_SYSclk_DIV_2);	//ADC时钟频率：SYSclk/2
//备用电感	初始化
	adc_init(S_ADC1, ADC_SYSclk_DIV_2);	//ADC时钟频率：SYSclk/2
	adc_init(S_ADC5, ADC_SYSclk_DIV_2);	//ADC时钟频率：SYSclk/2
	adc_init(S_ADC7, ADC_SYSclk_DIV_2);	//ADC时钟频率：SYSclk/2
	adc_init(S_ADC11, ADC_SYSclk_DIV_2);	//ADC时钟频率：SYSclk/2

}

/**************************************************************************
函数功能：adc均值滤波
输入参数：adc通道，时钟频率，读取次数
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
函数功能：adc读取
**************************************************************************/
void Read_Adc_Value(void)
{
//	const uint16 adc1_last,adc2_last,adc3_last,adc4_last;
	adc1 = adc_mean_filter(ADC_1,ADC_12BIT,5);		//对应板子上的IN1
	adc2 = adc_mean_filter(ADC_2,ADC_12BIT,5);		//对应板子上的IN2
	adc3 = adc_mean_filter(ADC_3,ADC_12BIT,5);		//对应板子上的IN3
	adc4 = adc_mean_filter(ADC_4,ADC_12BIT,5);		//对应板子上的IN4
//备用电感		
	adc5 = adc_mean_filter(S_ADC1,ADC_12BIT,5);   //对应板子上的IN5
	adc6 = adc_mean_filter(S_ADC5,ADC_12BIT,5);		//对应板子上的IN6
	adc7 = adc_mean_filter(S_ADC7,ADC_12BIT,5);		//对应板子上的IN7
	adc8 = adc_mean_filter(S_ADC11,ADC_12BIT,5);	//对应板子上的IN8
}
