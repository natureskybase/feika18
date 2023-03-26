#include "key.h"

/**************************************************************************
函数功能：按键扫描
输入参数：mode：1 支持连按；0 不支持连按
*注：优先级 KEY1 > KEY2 > KEY3 > KEY4
**************************************************************************/
uint8 Key_Scan(uint8 mode) 
{
    static uint8 key_up = 1;
    if(mode)key_up=1;  //支持连按
    if(key_up&&(KEY1==0||KEY2==0||KEY3==0||KEY4==0))
    {
        delay_ms(10);//去抖动
        key_up=0;
        if(KEY1==0)return 1;
        else if(KEY2==0)return 2;
        else if(KEY3==0)return 3;
        else if(KEY4==0)return 4;
    }else if(KEY1==1&&KEY2==1&&KEY3==1&&KEY4==1)key_up=1;
    return 0;//无按键按下
}