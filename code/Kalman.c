/*
 * Kalman.c
 *
 *  Created on: 2023年3月9日
 *      Author: 清羽
 */
#include "Kalman.h"
#include "zf_device_imu660ra.h"

//卡尔曼解算法库

int16 aacx,aacy,aacz;       //加速度传感器原始数据
int16 gyrox,gyroy,gyroz;    //陀螺仪原始数据
int16 temperature;          //陀螺仪温度数据
float Accel_x;              //X轴加速度值暂存
float Accel_y;              //Y轴加速度值暂存
float Accel_z;              //Z轴加速度值暂存
float Gyro_x;               //X轴陀螺仪数据暂存
float Gyro_y;               //Y轴陀螺仪数据暂存
float Gyro_z;               //Z轴陀螺仪数据暂存
float Angle_x_temp;         //由加速度计算的x倾斜角度
float Angle_y_temp;         //由加速度计算的y倾斜角度
float Angle_z_temp;         //由加速度计算的y倾斜角度
float Angle_X_Final;        //X最终倾斜角度
float Angle_Y_Final;        //Y最终倾斜角度
float bias;
float Gyro_Z_Drift = 0;     //陀螺仪Z轴零飘值

float dt=0.001;       //每5ms进行一次滤波
float Kalman_Filter_x(float Accel,float Gyro)
{
    static float angle,angle_dot;
    float Q_angle=0.001; // 过程噪声的协方差
    float Q_gyro=0.003; //0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
    float R_angle=0.5;      // 测量噪声的协方差 既测量偏差
    char  C_0 = 1;
    static float Q_bias, Angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] ={0,0,0,0};
    static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
    angle+=(Gyro - Q_bias) * dt; //先验估计
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1]=-PP[1][1];
    Pdot[2]=-PP[1][1];
    Pdot[3]=Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - angle;  //zk-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    angle   += K_0 * Angle_err;  //后验估计
    Q_bias  += K_1 * Angle_err;  //后验估计
    angle_dot   = Gyro - Q_bias;     //输出值(后验估计)的微分=角速度
    return angle;
}
float Kalman_Filter_y(float Accel,float Gyro)
{
    static float angle_dot;
    static float angle;
    float Q_angle=0.001; // 过程噪声的协方差
    float Q_gyro=0.003; //0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
    float R_angle=0.0005;      // 测量噪声的协方差 既测量偏差
    char  C_0 = 1;
    static float Q_bias, Angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] ={0,0,0,0};
    static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
    angle+=(Gyro - Q_bias) * dt; //先验估计
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
    Pdot[1]=-PP[1][1];
    Pdot[2]=-PP[1][1];
    Pdot[3]=Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;
    Angle_err = Accel - angle;  //zk-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    angle   += K_0 * Angle_err;    //后验估计
    Q_bias  += K_1 * Angle_err;  //后验估计
    angle_dot   = Gyro - Q_bias;    //输出值(后验估计)的微分=角速度
    return angle;
}
float Complementary_Filter_y(float angle_m, float gyro_m)
{
     static float angle;
     float K1 =0.02;
     angle = K1 * angle_m+ (1-K1) * (angle + (gyro_m-0.2) * dt);
     return angle;
}
void Angle_Calcu(void)
{
    Gyro_x=imu660ra_gyro_x;
    Gyro_y=imu660ra_gyro_y;
    Gyro_z=imu660ra_gyro_z;
    Accel_x=imu660ra_acc_x;
    Accel_y=imu660ra_acc_y;
    Accel_z=imu660ra_acc_z;
    if(Gyro_x>32768)  Gyro_x-=65536;                 //数据类型转换  也可通过short强制类型转换
    if(Gyro_y>32768)  Gyro_y-=65536;                 //数据类型转换  也可通过short强制类型转换
    if(Gyro_z>32768)  Gyro_z-=65536;                 //数据类型转换
    if(Accel_x>32768) Accel_x-=65536;                //数据类型转换
    if(Accel_y>32768) Accel_y-=65536;                //数据类型转换
    if(Accel_z>32768) Accel_z-=65536;                //数据类型转换
    Accel_x=imu660ra_acc_transition(imu660ra_acc_x);
    Accel_y=imu660ra_acc_transition(imu660ra_acc_y);
    Accel_z=imu660ra_acc_transition(imu660ra_acc_z);
    Gyro_x=imu660ra_gyro_transition(imu660ra_gyro_x);
    Gyro_y=imu660ra_gyro_transition(imu660ra_gyro_y);
    Gyro_z=imu660ra_gyro_transition(imu660ra_gyro_z - Gyro_Z_Drift);
    Angle_x_temp=atan2(Accel_y,Accel_z)*180/PI;     //计算倾角，转换单位为度
    Angle_y_temp=atan2(Accel_x,Accel_z)*180/PI;     //计算倾角，转换单位为度
    Angle_z_temp+=Gyro_z/16.4*0.015;
    if(Angle_z_temp> 360.0)  Angle_z_temp =0;
    if(Angle_z_temp<-360.0)  Angle_z_temp =0;
    Angle_X_Final=Kalman_Filter_x(Angle_x_temp,Gyro_x);//卡尔曼滤波
    Angle_Y_Final=Kalman_Filter_y(Angle_y_temp,-Gyro_y);
    Gyro_x = Xianfu(Gyro_x,-120,120);
//    Angle_z_temp=Kalman_Filter_x(Accel_z,Gyro_z);
//    Angle_X_Final=Complementary_Filter_y(Angle_x_temp,Gyro_x);
}

/**************************************************************************
Function: First order complementary filtering
Input   : acceleration、angular velocity
Output  : none
函数功能：一阶互补滤波
入口参数：加速度获取的角度、角速度
返回  值：x轴角速度
**************************************************************************/
float Complementary_Filter_x(void)
{
     static float angle;
     float K1 =0.1;
     //1.原始数据读取
     float accx,accy,accz;//三方向角加速度值
     Accel_x = imu660ra_acc_x;//x轴加速度值暂存
     Accel_y = imu660ra_acc_y;//y轴加速度值暂存
     Accel_z = imu660ra_acc_z;//z轴加速度值暂存
     Gyro_x  = imu660ra_gyro_x;//x轴陀螺仪值暂存
     Gyro_y  = imu660ra_gyro_y;//y轴陀螺仪值暂存
     Gyro_z  = imu660ra_gyro_z;//z轴陀螺仪值暂存

     //2.角加速度原始值处理过程
     //加速度传感器配置寄存器0X1C内写入0x01,设置范围为±2g。换算关系：2^16/4 = 16384LSB/g
     if(Accel_x<32764) accx=Accel_x/16384;//计算x轴加速度
     else              accx=1-(Accel_x-49152)/16384;
     if(Accel_y<32764) accy=Accel_y/16384;//计算y轴加速度
     else              accy=1-(Accel_y-49152)/16384;
     if(Accel_z<32764) accz=Accel_z/16384;//计算z轴加速度
     else              accz=(Accel_z-49152)/16384;
     //加速度反正切公式计算三个轴和水平面坐标系之间的夹角
     Angle_x_temp=(atan(accy/accz))*180/3.14;
     Angle_y_temp=(atan(accx/accz))*180/3.14;
     //判断计算后角度的正负号
     if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
     if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
     if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
     if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;

     //3.角速度原始值处理过程
     //陀螺仪配置寄存器0X1B内写入0x18，设置范围为±2000deg/s。换算关系：2^16/4000=16.4LSB/(°/S)
     ////计算角速度
     if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);
     if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
     if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);
     if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
     if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);
     if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;
     angle = K1 * Angle_x_temp+ (1-K1) * (angle + (Gyro_x-0.46)* dt);
     return angle;
}

