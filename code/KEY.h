#ifndef KEY_H_
#define KEY_H_
/*
 * KEY.h
 *
 *  Created on: 2023Äê3ÔÂ9ÈÕ
 *      Author: qingyu
 */

#include "zf_common_headfile.h"

#define _UP  P22_3
#define _DOWN  P22_2
#define _LEFT  P22_0
#define _RIGHT  P23_1
#define _MID  P22_1

typedef enum
{
    None = 0,
    UP,
    DOWN,
    LEFT,
    RIGHT,
    MID,
}Key;

extern Key Key_Value;

void key_scan(void);
void Key_init(void);


#endif /* CODE_KEY_H_ */
