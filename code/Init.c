/*
 * Init.c
 *
 *  Created on: 2023年6月10日
 *      Author: qingyu
 */
#include "Init.h"

extern FiterStruct Filter_parm;
extern float Gyro_Z_Drift;     //陀螺仪Z轴零飘值
void ADC_Init(void)
{
    adc_init(ADC0_CH0_A0,ADC_12BIT);//初始化ADC
    adc_init(ADC0_CH1_A1,ADC_12BIT);
    adc_init(ADC0_CH2_A2,ADC_12BIT);
    adc_init(ADC0_CH8_A8,ADC_12BIT);
}
void Beep_Init(void)
{
    gpio_init(P21_4, GPO, GPIO_LOW, GPO_PUSH_PULL);
}
void Ips114_Init(void)
{
    ips114_init();                  //屏幕初始化
    ips114_set_dir(IPS114_CROSSWISE_180);
    ips114_set_color(RGB565_BLUE, RGB565_WHITE);
}
void All_Init(void)
{
    MTL_PWM_Init();
    imu660ra_init();                //陀螺仪初始化
    Gyro_Z_Drift = Cut_Gyro_Drift();
//    Gyro_Z_Drift = 0;
    mt9v03x_init();                 //初始化摄像头
    image_process_init();           //初始化图像数组
    Brushless_Init();               //无刷电机初始化
    Motor_Init();                   //行进轮电机初始化
    uart_init(UART_2,115200,UART2_TX_P10_5,UART2_RX_P10_6);
    Key_init();                     //按键初始化
    uart_rx_interrupt(UART_2, 1);
    dl1a_init();
    Ips114_Init();
    Pid_init_DL();                  //动量轮PID参数初始化
    Pid_init_go();                  //行进轮PID参数初始化

    ADC_Init();
    Init_Filter(M_FT, M_FC,&Filter_parm);
    Beep_Init();
}


