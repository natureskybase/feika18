#include "key.h"

/**************************************************************************
�������ܣ�����ɨ��
���������mode��1 ֧��������0 ��֧������
*ע�����ȼ� KEY1 > KEY2 > KEY3 > KEY4
**************************************************************************/
uint8 Key_Scan(uint8 mode) 
{
    static uint8 key_up = 1;
    if(mode)key_up=1;  //֧������
    if(key_up&&(KEY1==0||KEY2==0||KEY3==0||KEY4==0))
    {
        delay_ms(10);//ȥ����
        key_up=0;
        if(KEY1==0)return 1;
        else if(KEY2==0)return 2;
        else if(KEY3==0)return 3;
        else if(KEY4==0)return 4;
    }else if(KEY1==1&&KEY2==1&&KEY3==1&&KEY4==1)key_up=1;
    return 0;//�ް�������
}