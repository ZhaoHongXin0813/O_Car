/*
 * Pid.c
 *
 *  Created on: 2023年3月10日
 *      Author: qingyu
 */
#include "Pid.h"

//void PidInit(PID * pid)
//{
//    pid->kp        = 0;
//    pid->ki        = 0;
//    pid->kd        = 0;
//    pid->imax      = 0;
//    pid->out_p     = 0;
//    pid->out_i     = 0;
//    pid->out_d     = 0;
//    pid->out       = 0;
//    pid->integrator= 0;
//    pid->last_error= 0;
//    pid->last_derivative   = 0;
//    pid->last_t    = 0;
//}
/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：限幅函数
 *  参数说明：
  * @param    amt   ： 参数
  * @param    low   ： 最低值
  * @param    high  ： 最高值
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float Xianfu(float amt, float low, float high)
{
    if(amt<=low)    return low;
    else if(amt>=high)  return high;
    else
        return amt;
}
int Xianfu_int(int amt, int low, int high)
{
    if(amt<=low)    return low;
    else if(amt>=high)  return high;
    else
        return amt;
}



float PidLocCtrl(PID * pid, float error)
{
    /* 累积误差 */
    pid->integrator += error;

    /* 误差限幅 */
    pid->integrator=Xianfu(pid->integrator, -pid->imax, pid->imax);


    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}

float PidIncCtrl(PID * pid, float error)
{

    pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error;
    pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}




