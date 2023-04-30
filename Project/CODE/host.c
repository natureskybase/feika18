#include "host.h"



uint8 inf[20] = {0xAA, 0x05, 0xAF, 0xF1, 0x08};
void send_four_data(uint8 frame, int16 num1, int16 num2, int16 num3, int16 num4)
{
	uint8 t;
	uint8 sum = 0;
	inf[3] = frame;

	inf[5] = num1 >> 8;
	inf[7] = num2 >> 8;
	inf[9] = num3 >> 8;
	inf[11] = num4 >> 8;
//	memcpy(inf + 6, &num1, 1);
//	memcpy(inf + 8, &num2, 1);
//	memcpy(inf + 10, &num3, 1);
//	memcpy(inf + 12, &num4, 1);
	inf[6]=num1&0x00FF;
	inf[8]=num2&0x00FF;
	inf[10]=num3&0x00FF;
	inf[12]=num4&0x00FF;
	for (t = 0; t < 13; t++)
	{
		sum += inf[t];
		uart_putchar(T_UART, inf[t]);

	}
	uart_putchar(T_UART, sum); //发送校验和

}