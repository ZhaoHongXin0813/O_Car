/*
 * Balance.c
 *
 *  Created on: 2023年3月11日
 *      Author: qingyu
 */
#include "Balance.h"
//动量轮左右平衡PID
PID Pid_Dl_Angle_v;                          //角速度环PID
PID Pid_Dl_Angle;                            //角度环PID
PID Pid_Dl_Speed;                            //动量轮速度环PID

//行进轮前后平衡PID
PID Pid_Go_Angle_v;                       //角速度环
PID Pid_Go_Angle;                         //角度环
PID Pid_Go_Speed;                         //速度环

//转向环PID
PID Pid_Turn;                            //转向环PID
PID Pid_Turn_Angle_v;                            //转向环PID
PID Pid_Turn_Speed;                              //转向环速度PID

int16 Dl_speed_L=2000;                       //动量轮左边速度
int16 Dl_speed_R=0;                       //动量轮右边速度


int16 PWM_Dl_L=0;                           //动量轮左边PWM输出值
int16 PWM_Dl_R=0;                           //动量轮右边PWM输出值

int16 Turn_Kp=-15,Turn_Kd =2, Turn_Ki =0.5;          //转向环-45        20

float Dynamic_Zero_k;       //动态零点系数
float Dynamic_Acc_k;        //动态加速度

float Mid_Angle_Dl = 0;        //动量轮机械零点
float Mid_Angle_Go = 0;        //行进轮机械零点

extern int16 start;

extern int16 Encoder_L,Encoder_R,Encoder,Encoder_Go;   //动量轮及行进轮编码器值
void Pid_init_DL(void)                 //动量轮PID参数初始化
{
    Pid_Dl_Angle_v.kp=350;       //350             //角速度环PID参数
    Pid_Dl_Angle_v.ki=3.8;       //3.5
    Pid_Dl_Angle_v.kd=0;       //0
    Pid_Dl_Angle_v.imax=2000;
    Pid_Dl_Angle_v.out_limit=9000;

    Pid_Dl_Angle.kp=-1.5;          //-1.7           //角度环PID参数
    Pid_Dl_Angle.ki=0;          //0
    Pid_Dl_Angle.kd=0.1;          //-0.1
    Pid_Dl_Angle.imax=300;
    Pid_Dl_Angle.out_limit=5000;

    Pid_Dl_Speed.kp=-0.15;        //-0.15                 //速度环PID参数
    Pid_Dl_Speed.ki=-0.001;     //0.001
    Pid_Dl_Speed.kd=0;
    Pid_Dl_Speed.imax=40;
    Pid_Dl_Speed.out_limit=1000;
//    Pid_Dl_Angle_v.kp=300;       //350             //角速度环PID参数
//    Pid_Dl_Angle_v.ki=2.1;       //4.3  //2.5
//    Pid_Dl_Angle_v.kd=0;       //0
//    Pid_Dl_Angle_v.imax=3000;
//    Pid_Dl_Angle_v.out_limit=9000;
//
//    Pid_Dl_Angle.kp=-2.0;          //-1.7       //-2.0        //角度环PID参数
//    Pid_Dl_Angle.ki=0;          //0
//    Pid_Dl_Angle.kd=-1.0;          //-1.0       //-1.0
//    Pid_Dl_Angle.imax=300;
//    Pid_Dl_Angle.out_limit=5000;
//
//    Pid_Dl_Speed.kp=-0.08;        //-0.15       //-0.15          //速度环PID参数
//    Pid_Dl_Speed.ki=-0.00055;     //0.001        //-0.0007
//    Pid_Dl_Speed.kd=0;
//    Pid_Dl_Speed.imax=20;
//    Pid_Dl_Speed.out_limit=1000;

}
void Pid_init_go(void)
{
    Pid_Go_Angle_v.kp=16;           //16
    Pid_Go_Angle_v.ki=0.65  ;          //0.6
    Pid_Go_Angle_v.kd=0.5;          //0.5
    Pid_Go_Angle_v.imax=4000;
    Pid_Go_Angle_v.out_limit=3200;

    Pid_Go_Angle.kp=2;              //2
    Pid_Go_Angle.ki=0;
    Pid_Go_Angle.kd=10.0;           //10
    Pid_Go_Angle.imax=100;
    Pid_Go_Angle.out_limit=1000;

    Pid_Go_Speed.kp=-0.032;          //-0.03
    Pid_Go_Speed.ki=-0.001;        //-0.0009
    Pid_Go_Speed.kd=-0;
    Pid_Go_Speed.imax=15;
    Pid_Go_Speed.out_limit=40;

      Pid_Turn.kp=50;    //100.5
      Pid_Turn.ki=0.0001;
      Pid_Turn.kd=0;        //15
      Pid_Turn.out_limit = 4000;

      Pid_Turn_Angle_v.kp=200;           //150
      Pid_Turn_Angle_v.ki=5;           //5
      Pid_Turn_Angle_v.kd=0;
      Pid_Turn_Angle_v.out_limit = 8000;
      Pid_Turn_Angle_v.imax =2000;

      Pid_Turn_Speed.kp=-0.1;           //-0.3
      Pid_Turn_Speed.ki=0;           //0
      Pid_Turn_Speed.kd=0;
      Pid_Turn_Speed.out_limit = 1000;
      Pid_Turn_Speed.imax =200;

      Dynamic_Acc_k = 0.2;
 }

//分割线---------------------------------------动量轮控制------------------------------------------分割线//
float Dl_Angle_v(float Gyro , float Target_gyro)               //角速度环
{
    float error;
    error = Target_gyro - Gyro;
    if(start == 0)      error=0;
    Pid_Dl_Angle_v.integrator += error;     //累计误差
    Pid_Dl_Angle_v.integrator = Xianfu(Pid_Dl_Angle_v.integrator, -Pid_Dl_Angle_v.imax, Pid_Dl_Angle_v.imax);   //误差限幅
    Pid_Dl_Angle_v.out_p = Pid_Dl_Angle_v.kp * error;   //P
    Pid_Dl_Angle_v.out_i = Pid_Dl_Angle_v.ki * Pid_Dl_Angle_v.integrator;//I
    Pid_Dl_Angle_v.out_d = Pid_Dl_Angle_v.kd *(error - Pid_Dl_Angle_v.last_error); //D
    Pid_Dl_Angle_v.out = Pid_Dl_Angle_v.out_p + Pid_Dl_Angle_v.out_i + Pid_Dl_Angle_v.out_d; //OUT
    Pid_Dl_Angle_v.out = Xianfu(Pid_Dl_Angle_v.out,-Pid_Dl_Angle_v.out_limit, Pid_Dl_Angle_v.out_limit);    //输出限幅
    Pid_Dl_Angle_v.last_error = error;
    return Pid_Dl_Angle_v.out;

}
float Dl_Angle(float Angle, float Target_Angle )               //角度环
{
    float error;
    error = Target_Angle-1.5 - Dynamic_Zero(Encoder_Go,image_para.offset,Dynamic_Zero_k)- Angle;
//    error = Target_Angle  - Angle;      //-3.5
    if(start == 0)      error=0;
    Pid_Dl_Angle.integrator += error;     //累计误差
    Pid_Dl_Angle.integrator = Xianfu(Pid_Dl_Angle.integrator, -Pid_Dl_Angle.imax, Pid_Dl_Angle.imax);   //误差限幅
    Pid_Dl_Angle.out_p = Pid_Dl_Angle.kp * error;   //P
    Pid_Dl_Angle.out_i = Pid_Dl_Angle.ki * Pid_Dl_Angle.integrator;//I
    Pid_Dl_Angle.out_d = Pid_Dl_Angle.kd *(error - Pid_Dl_Angle.last_error); //D
    Pid_Dl_Angle.out = Pid_Dl_Angle.out_p + Pid_Dl_Angle.out_i + Pid_Dl_Angle.out_d; //OUT
    Pid_Dl_Angle.out = Xianfu(Pid_Dl_Angle.out,-Pid_Dl_Angle.out_limit, Pid_Dl_Angle.out_limit);    //输出限幅
    Pid_Dl_Angle.last_error = error;
    return Pid_Dl_Angle.out;
}
float Dl_speed(int16 Encoder)                                  //动量轮速度环
{
    float error,Encoder_Least;
    Encoder_Least = (float)Encoder;                                                   //速度滤波
    Encoder *= 0.7;                                                              //一阶低通滤波器
    Encoder += Encoder_Least*0.3;
    error = 0 - Encoder;
    if(start == 0)      error=0;
    Pid_Dl_Speed.integrator += error;     //累计误差
    Pid_Dl_Speed.integrator = Xianfu(Pid_Dl_Speed.integrator, -Pid_Dl_Speed.imax, Pid_Dl_Speed.imax);   //误差限幅
    Pid_Dl_Speed.out_p = Pid_Dl_Speed.kp * error;   //P
    Pid_Dl_Speed.out_i = Pid_Dl_Speed.ki * Pid_Dl_Speed.integrator;//I
    Pid_Dl_Speed.out_d = Pid_Dl_Speed.kd *(error - Pid_Dl_Speed.last_error); //D
    Pid_Dl_Speed.out = Pid_Dl_Speed.out_p + Pid_Dl_Speed.out_i + Pid_Dl_Speed.out_d; //OUT
    Pid_Dl_Speed.out = Xianfu(Pid_Dl_Speed.out,-Pid_Dl_Speed.out_limit, Pid_Dl_Speed.out_limit);    //输出限幅
    Pid_Dl_Speed.last_error = error;
    return Pid_Dl_Speed.out;
}

//分割线---------------------------------------动量轮控制------------------------------------------分割线//


//分割线---------------------------------------行进轮控制------------------------------------------分割线//

float Go_Angle_v(float Gyro , float Target_gyro)               //角速度环
{
    float error;
    error = Target_gyro - Gyro;
    if(start == 0)      error=0;
    Pid_Go_Angle_v.integrator += error;     //累计误差
    Pid_Go_Angle_v.integrator = Xianfu(Pid_Go_Angle_v.integrator, -Pid_Go_Angle_v.imax, Pid_Go_Angle_v.imax);   //误差限幅
    Pid_Go_Angle_v.out_p = Pid_Go_Angle_v.kp * error;   //P
    Pid_Go_Angle_v.out_i = Pid_Go_Angle_v.ki * Pid_Go_Angle_v.integrator;//I
    Pid_Go_Angle_v.out_d = Pid_Go_Angle_v.kd *(error - Pid_Go_Angle_v.last_error); //D
    Pid_Go_Angle_v.out = Pid_Go_Angle_v.out_p + Pid_Go_Angle_v.out_i + Pid_Go_Angle_v.out_d; //OUT
    Pid_Go_Angle_v.out = Xianfu(Pid_Go_Angle_v.out,-Pid_Go_Angle_v.out_limit, Pid_Go_Angle_v.out_limit);    //输出限幅
    Pid_Go_Angle_v.last_error = error;

    return Pid_Go_Angle_v.out;

}
float Go_Angle(float Angle, float Target_Angle)               //角度环
{
    float error;
    error = Target_Angle - Angle;
    if(start == 0)      error=0;
    Pid_Go_Angle.integrator += error;     //累计误差
    Pid_Go_Angle.integrator = Xianfu(Pid_Go_Angle.integrator, -Pid_Go_Angle.imax, Pid_Go_Angle.imax);   //误差限幅
    Pid_Go_Angle.out_p = Pid_Go_Angle.kp * error;   //P
    Pid_Go_Angle.out_i = Pid_Go_Angle.ki * Pid_Go_Angle.integrator;//I
    Pid_Go_Angle.out_d = Pid_Go_Angle.kd *(error - Pid_Go_Angle.last_error); //D
//    Pid_Go_Angle.out_d = Pid_Go_Angle.kd * gyro ; //D
    Pid_Go_Angle.out = Pid_Go_Angle.out_p + Pid_Go_Angle.out_i + Pid_Go_Angle.out_d; //OUT
    Pid_Go_Angle.out = Xianfu(Pid_Go_Angle.out,-Pid_Go_Angle.out_limit, Pid_Go_Angle.out_limit);    //输出限幅
    Pid_Go_Angle.last_error = error;
    return Pid_Go_Angle.out;
}

/**************************************************************************
C电机速度PI控制,速度正反馈环
**************************************************************************/
float Go_speed(int16 Encoder,int16 Target_Go_Speed)                                  //动量轮速度环
{
    float error,Encoder_Least;
    Encoder_Least = (float)Encoder;                                                   //速度滤波
    Encoder *= 0.7;                                                              //一阶低通滤波器
    Encoder += Encoder_Least*0.3;
    error = Target_Go_Speed - Encoder;
    if(start == 0)      error=0;
    Pid_Go_Speed.integrator += error;     //累计误差
    Pid_Go_Speed.integrator = Xianfu(Pid_Go_Speed.integrator, -Pid_Go_Speed.imax, Pid_Go_Speed.imax);   //误差限幅
    Pid_Go_Speed.out_p = Pid_Go_Speed.kp * error;   //P
    Pid_Go_Speed.out_i = Pid_Go_Speed.ki * Pid_Go_Speed.integrator;//I
    Pid_Go_Speed.out_d = Pid_Go_Speed.kd *(error - Pid_Go_Speed.last_error); //D
    Pid_Go_Speed.out = Pid_Go_Speed.out_p + Pid_Go_Speed.out_i + Pid_Go_Speed.out_d; //OUT
    Pid_Go_Speed.out = Xianfu(Pid_Go_Speed.out,-Pid_Go_Speed.out_limit, Pid_Go_Speed.out_limit);    //输出限幅
    Pid_Go_Speed.last_error = error;
    return Pid_Go_Speed.out;
}

//单P转向环
int Turn(int gyro_Z,int offset)
{
    int PWM_out,Integrator=0;
    Integrator += offset;
    PWM_out= Turn_Kp*offset + Turn_Ki*Integrator + Turn_Kd*gyro_Z;          //15
    if(PWM_out > 0) PWM_out++;
    if(PWM_out < 0) PWM_out--;
    PWM_out=Xianfu(PWM_out,-5000,5000);
    return PWM_out;
}
//单角度环转向环
int Turn_Angle(float Yaw,float Gyro_z,float Target_Yaw)
{
    float error;
    error = Yaw-Target_Yaw;
    if(start == 0)      error=0;
    Pid_Turn.integrator += error;     //累计误差
    Pid_Turn.integrator = Xianfu(Pid_Turn.integrator, -Pid_Turn.imax, Pid_Turn.imax);   //误差限幅

    Pid_Turn.out = Pid_Turn.kp*error + Pid_Turn.ki*Pid_Turn.integrator + Pid_Turn.kd*Gyro_z; //OUT
    Pid_Turn.out = Xianfu(Pid_Turn.out,-Pid_Turn.out_limit, Pid_Turn.out_limit);    //输出限幅
    Pid_Turn.last_error = error;
    return (int)Pid_Turn.out;
}
//角速度环转向环
int Turn_Angle_v(float Gyro_z,float Target_Gyro)
{
    float error;
    error = Gyro_z-Target_Gyro;
    if(start == 0)      error=0;
    Pid_Turn_Angle_v.integrator += error;     //累计误差
    Pid_Turn_Angle_v.integrator = Xianfu(Pid_Turn_Angle_v.integrator, -Pid_Turn_Angle_v.imax, Pid_Turn_Angle_v.imax);   //误差限幅
    Pid_Turn_Angle_v.out = Pid_Turn_Angle_v.kp*error + Pid_Turn_Angle_v.ki*Pid_Turn_Angle_v.integrator + Pid_Turn_Angle_v.kd*(error - Pid_Turn_Angle_v.last_error); //OUT
    Pid_Turn_Angle_v.out = Xianfu(Pid_Turn_Angle_v.out,-Pid_Turn_Angle_v.out_limit, Pid_Turn_Angle_v.out_limit);    //输出限幅
    Pid_Turn_Angle_v.last_error = error;
    return (int)Pid_Turn_Angle_v.out;
}
//速度环转向环
int Turn_Speed(float Encoder_Dl)
{
    float error;
    error = Encoder_Dl;
    Pid_Turn_Speed.integrator += error;     //累计误差
    Pid_Turn_Speed.integrator = Xianfu(Pid_Turn_Speed.integrator, -Pid_Turn_Speed.imax, Pid_Turn_Speed.imax);   //误差限幅
    Pid_Turn_Speed.out = Pid_Turn_Speed.kp*error + Pid_Turn_Speed.ki*Pid_Turn_Speed.integrator + Pid_Turn_Speed.kd*(error - Pid_Turn_Speed.last_error); //OUT
    Pid_Turn_Speed.out = Xianfu(Pid_Turn_Speed.out,-Pid_Turn_Speed.out_limit, Pid_Turn_Speed.out_limit);    //输出限幅
    Pid_Turn_Speed.last_error = error;
    return (int)Pid_Turn_Speed.out;
}
//动态零点
float Dynamic_Zero(float Encoder ,float err, float k)
{
    float Angle;
    Encoder = Encoder/2800;
    Angle = k * Encoder* Encoder*  err * 8.3;
    Angle = Angle > 6 ?  6 :  Angle;
    Angle = Angle <-6 ? -6 :  Angle;
    return Angle;
}
//动态加速度
float Dynamic_acc(float Encoder ,float err, float k)
{
    float acc;
    Encoder = Encoder/2800;
    acc = k * Encoder* Encoder*  err * 16.3;
    acc = abs_f(acc);
    if(acc > 30)
        acc = 30;
    else if(acc < 5)
        acc = 5;
    return acc;
}
//低电压检测
float Low_Battery(void)
{
    float voltage;
    voltage= 37.7 * (adc_convert (ADC0_CH8_A8)) / 4096;
    if(voltage>18 && voltage<22)    gpio_set_level(P21_4, GPIO_HIGH);
    else  gpio_set_level(P21_4, GPIO_LOW);
    return voltage;
}
//Z轴去除零飘
float Cut_Gyro_Drift(void)
{
    // 计算陀螺仪Z轴零飘校准值
    float Gyro_Offset = 0;
    // 采集一定数量的陀螺仪数据
    for (int i = 0; i < 100; i++)
    {
        imu660ra_get_gyro();  // 获取 IMU660RA 的角速度测量数值
        // 累加陀螺仪Z轴数据
        Gyro_Offset += imu660ra_gyro_z;
    }
    Gyro_Offset /= 100;
    return Gyro_Offset;
}

