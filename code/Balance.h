/*
 * Balance.h
 *
 *  Created on: 2023年3月11日
 *      Author: qingyu
 */

#ifndef CODE_BALANCE_H_
#define CODE_BALANCE_H_
#include "zf_common_headfile.h"
extern float Dynamic_Zero_k;       //动态零点系数



void Pid_init_DL(void);     //动量轮电机PID初始化
void Pid_init_go(void);     //行进轮电机PID初始化

float Dl_Angle_v(float Gyro , float Target_gyro);       //动量轮角速度环
float Dl_Angle(float Angle, float Target_Angle );       //动量轮角度环
float Dl_speed(int16 Encoder);                          //动量轮速度环

float Go_Angle_v(float Gyro , float Target_gyro) ;      //行进轮角速度环
float Go_Angle(float Angle, float Target_Angle );       //行进轮角度环
float Go_speed(int16 Encoder,int16 Target_Go_Speed);    //行进轮速度环

//float Go_Angle(float Angle, float Target_Angle, float gyro);

int Turn(int gyro_Z,int offset);
int Turn_Angle(float Yaw,float Gyro_z,float Target_Yaw);
float Low_Battery(void);                                //低电压报警
int Turn_Angle_v(float Gyro_z,float Target_Gyro);
float Dynamic_Zero(float Encoder ,float err, float k);
float Dynamic_acc(float Encoder ,float err, float k);
float Cut_Gyro_Drift(void);
#endif /* CODE_BALANCE_H_ */
