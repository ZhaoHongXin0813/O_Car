/*
 * Pid.h
 *
 *  Created on: 2023年3月10日
 *      Author: qingyu
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

typedef struct
{

        float                kp;         //P
        float                ki;         //I
        float                kd;         //D
        float                imax;       //积分限幅

        float                out_p;  //KP输出
        float                out_i;  //KI输出
        float                out_d;  //KD输出
        float                out;    //pid输出
        float                out_limit; //  输出限幅

        float                integrator; //< 积分值
        float                last_error; //< 上次误差
        float                last_derivative;//< 上次误差与上上次误差之差
        unsigned long        last_t;     //< 上次时间
}PID;


//void PidInit(PID * pid);
float Xianfu(float amt, float low, float high);
float PidLocCtrl(PID * pid, float error);
float PidIncCtrl(PID * pid, float error);
int Xianfu_int(int amt, int low, int high);


#endif /* CODE_PID_H_ */
