#include "host.h"



//uint8 inf[20] = {0xAA, 0x05, 0xAF, 0xF1, 0x08};
//void send_four_data(uint8 frame, int16 num1, int16 num2, int16 num3, int16 num4)
//{
//	uint8 t;
//	uint8 sum = 0;
//	inf[3] = frame;

//	inf[5] = num1 >> 8;
//	inf[7] = num2 >> 8;
//	inf[9] = num3 >> 8;
//	inf[11] = num4 >> 8;
////	memcpy(inf + 6, &num1, 1);
////	memcpy(inf + 8, &num2, 1);
////	memcpy(inf + 10, &num3, 1);
////	memcpy(inf + 12, &num4, 1);
//	inf[6]=num1&0x00FF;
//	inf[8]=num2&0x00FF;
//	inf[10]=num3&0x00FF;
//	inf[12]=num4&0x00FF;
//	for (t = 0; t < 13; t++)
//	{
//		sum += inf[t];
//		uart_putchar(T_UART, inf[t]);

//	}
//	uart_putchar(T_UART, sum); //����У���

//}


void send_four_data(int16 num1, int16 num2 ,int16 num3 ,int16 num4) //�����ĸ�16λ����
{
	uint8 i;
	uint8 sumcheck = 0;			//��У��
	uint8 addcheck = 0;			//����У��
	uint8 data_buf[20];  	//���ݻ�����
	
	//֡ͷ//
	data_buf[0] = 0xAA;
	data_buf[1] = 0xFF;
	data_buf[2] = 0xF1;
	data_buf[3] = 0x08;//2+2+2+2
	
	//����//
	data_buf[4] = num1& 0xFF;
	data_buf[6] = num2 & 0xFF;
	data_buf[8] = num3 & 0xFF;
	data_buf[10] = num4 & 0xFF;
	
	data_buf[5] = num1  >> 8;
	data_buf[7] = num2 >> 8;
	data_buf[9] = num3 >> 8;
	data_buf[11] = num4 >> 8;
	
	//У��λ//
	for( i=0; i < (data_buf[3]+4); i++)
	{
		sumcheck += data_buf[i]; //��֡ͷ��ʼ����ÿһ�ֽڽ�����ͣ�ֱ��DATA������
		addcheck += sumcheck;		 //ÿһ�ֽڵ���Ͳ���������һ��sumcheck���ۼ�
	}
	data_buf[12] = sumcheck;
	data_buf[13] = addcheck;
	
	//���ݷ�������λ��//
	for (i = 0;i < 14; i++)
	uart_putchar(UART_1, data_buf[i]);
	
}
	



