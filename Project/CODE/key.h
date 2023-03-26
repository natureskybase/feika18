#ifndef __KEY_H_
#define __KEY_H_

#include "zf_delay.h"

#define KEY1 P70
#define KEY2 P71
#define KEY3 P72
#define KEY4 P73

#define KEY1_PRES 1
#define KEY2_PRES 2
#define KEY3_PRES 3
#define KEY4_PRES 4

uint8 Key_Scan(uint8 mode); //按键扫描 模式1 支持连按 模式0 不支持连按 返回值为按下的按键（只能按住一个）

#endif