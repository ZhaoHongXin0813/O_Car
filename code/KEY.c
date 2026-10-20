/*
 * KEY.c
 *
 *  Created on: 2023年3月9日
 *      Author: 清羽
 */
#include "zf_common_headfile.h"

bool Key_Status[5];         //按键状态
bool Key_Status_Last[5];    //上次按键状态

Key Key_Value = none;       //按键值
void Key_init(void)
{
    gpio_init(_UP, GPI, 1, GPI_PULL_UP);
    gpio_init(_DOWN, GPI, 1, GPI_PULL_UP);
    gpio_init(_LEFT, GPI, 1, GPI_PULL_UP);
    gpio_init(_RIGHT, GPI, 1, GPI_PULL_UP);
    gpio_init(_MID, GPI, 1, GPI_PULL_UP);
}


void key_scan(void)
{
    Key_Status_Last[0] = Key_Status[0];
    Key_Status_Last[1] = Key_Status[1];
    Key_Status_Last[2] = Key_Status[2];
    Key_Status_Last[3] = Key_Status[3];
    Key_Status_Last[4] = Key_Status[4];

    Key_Status[0] = gpio_get_level(_UP);
    Key_Status[1] = gpio_get_level(_DOWN);
    Key_Status[2] = gpio_get_level(_LEFT);
    Key_Status[3] = gpio_get_level(_RIGHT);
    Key_Status[4] = gpio_get_level(_MID);

    //检测到按键按下之后  并放开置位标志位

    if(Key_Status[0] && !Key_Status_Last[0])        Key_Value = UP;
    else if(Key_Status[1] && !Key_Status_Last[1])   Key_Value = DOWN;
    else if(Key_Status[2] && !Key_Status_Last[2])   Key_Value = LEFT;
    else if(Key_Status[3] && !Key_Status_Last[3])   Key_Value = RIGHT;
    else if(Key_Status[4] && !Key_Status_Last[4])   Key_Value = MID;
    else    Key_Value = none;
}
