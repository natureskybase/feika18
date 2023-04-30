#ifndef __ADC_H_
#define __ADC_H_

#include "zf_adc.h"

/****宏定义****/
#define ADC_1 ADC_P00
#define ADC_2 ADC_P01
#define ADC_3 ADC_P05
#define ADC_4 ADC_P06

//备用电感
#define S_ADC1 ADC_P11
#define S_ADC5 ADC_P15
#define S_ADC7 ADC_P17
#define S_ADC11 ADC_P03

/****扩展变量****/
extern int16 adc1; 
extern int16 adc2;
extern int16 adc3; 
extern int16 adc4;
extern int16 adc5; 
extern int16 adc6;
extern int16 adc7; 
extern int16 adc8;


/****函数声明****/
void My_Adc_Init(void);
void Read_Adc_Value(void);
uint16 adc_mean_filter(ADCN_enum adcn,ADC_SPEED_enum speed, uint8 count);


#endif