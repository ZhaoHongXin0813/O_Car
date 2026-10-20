/*
 * Siyuansu.h
 *
 *  Created on: 2023年3月15日
 *      Author: qingyu
 */

#ifndef CODE_SIYUANSU_H_
#define CODE_SIYUANSU_H_

#include "zf_common_typedef.h"
#include "zf_common_headfile.h"

#include <math.h>


#define delta_T     0.001f  // 采样周期1ms 即频率1KHZ
#define pi          3.1415926f

typedef struct{
    int16 Xdata;
    int16 Ydata;
      int16 Zdata;
}gyro_param_t;


typedef struct{
    float q0;
    float q1;
    float q2;
    float q3;
}quater_param_t;

typedef struct{
    float pitch;
    float roll;
    float yaw;

}euler_param_t;

typedef struct{
    float acc_x;
    float acc_y;
      float acc_z;
      float gyro_x;
    float gyro_y;
      float gyro_z;

}icm_param_t;

extern icm_param_t icm_data;
extern euler_param_t eulerAngle;
extern float q0,q1,q3,q4;
void gyroOffsetInit(void) ;
void icmGetValues(void);
void icmAHRSupdate(icm_param_t* icm);
float myRsqrt(float num);
void shuchu(void);

#endif /* CODE_SIYUANSU_H_ */
