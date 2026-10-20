#ifndef BRUSHLESS_H_
#define BRUSHLESS_H_

#include "zf_common_headfile.h"

#define LEFT_FLYWHEEL_PWM               (ATOM0_CH7_P02_7        )     // 左侧飞轮PWM控制引脚
#define LEFT_FLYWHEEL_DUTY_LEVEL        (0                      )     // 左侧飞轮占空比有效电平
#define LEFT_FLYWHEEL_DIR               (P02_6                  )     // 左侧飞轮方向控制引脚
#define LEFT_FLYWHEEL_CLOCKWISE         (1                      )     // 左侧飞轮顺时针旋转方向
#define LEFT_FLYWHEEL_BRAKE             (P11_3                  )     // 左侧飞轮刹车控制引脚
#define LEFT_FLYWHEEL_ENCODER_INDEX     (TIM5_ENCOEDER          )     // 左侧飞轮编码器编号
#define LEFT_FLYWHEEL_ENCODER_CH1       (TIM5_ENCOEDER_CH1_P10_3)     // 左侧飞轮编码器通道1
#define LEFT_FLYWHEEL_ENCODER_CH2       (TIM5_ENCOEDER_CH2_P10_1)     // 左侧飞轮编码器通道2

#define RIGHT_FLYWHEEL_PWM              (ATOM0_CH5_P02_5        )     // 右侧飞轮PWM控制引脚
#define RIGHT_FLYWHEEL_DUTY_LEVEL       (0                      )     // 右侧飞轮占空比有效电平
#define RIGHT_FLYWHEEL_DIR              (P02_4                  )     // 右侧飞轮方向控制引脚
#define RIGHT_FLYWHEEL_CLOCKWISE        (0                      )     // 右侧飞轮顺时针旋转方向
#define RIGHT_FLYWHEEL_BRAKE            (P11_2                  )     // 右侧飞轮刹车控制引脚
#define RIGHT_FLYWHEEL_ENCODER_INDEX    (TIM2_ENCOEDER          )     // 右侧飞轮编码器编号
#define RIGHT_FLYWHEEL_ENCODER_CH1      (TIM2_ENCOEDER_CH1_P33_7)     // 右侧飞轮编码器通道1
#define RIGHT_FLYWHEEL_ENCODER_CH2      (TIM2_ENCOEDER_CH2_P33_6)     // 右侧飞轮编码器通道2


void Brushless_Init(void);      //无刷电机初始化
void Motor_Init(void);          //行进轮电机初始化
void Set_Go_Pwm(int16 PWM);     //设置行进轮电机PWM
void Set_Dl_Pwm(int16 Pwm_Left,int16 Pwm_Right);

#endif
