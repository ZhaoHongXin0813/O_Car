#include "Brushless.h"
/*
 * Brushless.h
 *
 *  Created on: 2023年3月9日
 *      Author: 清羽
 */
void Brushless_Init(void)
{
        pwm_init (LEFT_FLYWHEEL_PWM, 17000, LEFT_FLYWHEEL_DUTY_LEVEL == 0 ? PWM_DUTY_MAX : 0);                  // 初始化左侧飞轮PWM信号
        gpio_init(LEFT_FLYWHEEL_DIR, GPO, LEFT_FLYWHEEL_CLOCKWISE, GPO_PUSH_PULL);                              // 初始化左侧飞轮DIR信号
        gpio_init(LEFT_FLYWHEEL_BRAKE, GPO, 1, GPO_PUSH_PULL);                                                  // 初始化左侧飞轮刹车信号
        encoder_quad_init(LEFT_FLYWHEEL_ENCODER_INDEX, LEFT_FLYWHEEL_ENCODER_CH1, LEFT_FLYWHEEL_ENCODER_CH2);   // 初始化左侧飞轮编码器接口

        pwm_init (RIGHT_FLYWHEEL_PWM, 17000, RIGHT_FLYWHEEL_DUTY_LEVEL == 0 ? PWM_DUTY_MAX : 0);                // 初始化右侧飞轮PWM信号
        gpio_init(RIGHT_FLYWHEEL_DIR, GPO, RIGHT_FLYWHEEL_CLOCKWISE, GPO_PUSH_PULL);                            // 初始化右侧飞轮DIR信号
        gpio_init(RIGHT_FLYWHEEL_BRAKE, GPO, 1, GPO_PUSH_PULL);                                                 // 初始化右侧飞轮刹车信号
        encoder_quad_init(RIGHT_FLYWHEEL_ENCODER_INDEX, RIGHT_FLYWHEEL_ENCODER_CH1, RIGHT_FLYWHEEL_ENCODER_CH2);// 初始化右侧飞轮编码器接口

}
void Motor_Init(void)
{
    pwm_init(ATOM0_CH0_P21_2, 12500, 0);                                                 // PWM 通道 L1 初始化频率 17KHz 占空比初始为 0
    pwm_init(ATOM0_CH1_P21_3, 12500, 0);
    encoder_quad_init(TIM4_ENCOEDER,TIM4_ENCOEDER_CH1_P02_8,TIM4_ENCOEDER_CH2_P00_9);       //行进轮编码器初始化
}

void Set_Go_Pwm(int16 PWM)
{
    if(PWM>=3500)      PWM=3500;
    else if (PWM < (-3500))    PWM=-3500;

    if(PWM>=0)
    {
        pwm_set_duty(ATOM0_CH0_P21_2,PWM);                  // 计算占空比
        pwm_set_duty(ATOM0_CH1_P21_3, 0);
    }
    else
    {
        pwm_set_duty(ATOM0_CH0_P21_2,0);                  // 计算占空比
        pwm_set_duty(ATOM0_CH1_P21_3,-PWM);
    }
}
void Set_Dl_Pwm(int16 Pwm_Left,int16 Pwm_Right)                       //设置动量轮PWM以及方向
{
    if(Pwm_Left>0)
    {
        gpio_set_level(LEFT_FLYWHEEL_DIR, 0);
    }
    else
    {
        gpio_set_level(LEFT_FLYWHEEL_DIR, 1);
    }

    if(Pwm_Right>0)
    {
        gpio_set_level(RIGHT_FLYWHEEL_DIR, 1);
    }
    else
    {
        gpio_set_level(RIGHT_FLYWHEEL_DIR, 0);
    }

    pwm_set_duty(LEFT_FLYWHEEL_PWM,PWM_DUTY_MAX-(abs(Pwm_Left)));  // 输出占空比 占空比必须为正值 因此此处使用绝对值

    pwm_set_duty(RIGHT_FLYWHEEL_PWM,PWM_DUTY_MAX-(abs(Pwm_Right)));  // 输出占空比 占空比必须为正值 因此此处使用绝对值
}

