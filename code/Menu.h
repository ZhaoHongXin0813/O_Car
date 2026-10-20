/*
 * Menu.h
 *
 *  Created on: 2023Äê7ÔÂ27ÈÕ
 *      Author: qingyu
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_
#include "zf_common_headfile.h"
extern uint16 ADC_Filter[3];

enum
{
    ROW_0 = 0,
    ROW_1,
    ROW_2,
    ROW_3,
    ROW_4,
    ROW_5,
}ROW;


void Show_Parmeter(void);
void Show_Sign(void);



#endif /* CODE_MENU_H_ */
