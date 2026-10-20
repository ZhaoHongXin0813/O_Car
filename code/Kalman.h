/*
 * Kalman.h
 *
 *  Created on: 2023年3月9日
 *      Author: qingyu
 */

#ifndef CODE_KALMAN_H_
#define CODE_KALMAN_H_

#include "zf_common_headfile.h"

extern int16 aacx,aacy,aacz;      //加速度传感器原始数据  angular acceleration
extern int16 gyrox,gyroy,gyroz;       //陀螺仪原始数据  gyroscope
extern float Gyro_x;               //X轴陀螺仪数据暂存
extern float Gyro_y;               //Y轴陀螺仪数据暂存
extern float Gyro_z;               //Z轴陀螺仪数据暂存
extern float Angle_x_temp;
extern float Angle_y_temp;
extern float Angle_z_temp;         //由加速度计算的y倾斜角度
extern float Angle_X_Final;        //X最终倾斜角度
extern float Angle_Y_Final;        //Y最终倾斜角度

extern float Q_bias;


void Angle_Calcu(void);
float Kalman_Filter_x(float Accel,float Gyro);
float Kalman_Filter_y(float Accel,float Gyro);
float Complementary_Filter_y(float angle_m, float gyro_m);
float Complementary_Filter_x(void);
#endif /* CODE_KALMAN_H_ */
