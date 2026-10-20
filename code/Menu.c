/*
 * Menu.c
 *
 *  Created on: 2023年7月27日
 *      Author: qingyu
 */
#include "Menu.h"

int16 Sign_Num = 0;
int8 Row = 1;
bool Show_Img_Flag = 0;
uint8 Led_Flag=0;
extern bool start;                  //开始标志
extern int16 Go_speed_Target,Go_speed_i;
extern int16 Angle_Dl_v_out;         //行进轮三环输出量
extern int16 Dl_Turn_Pwm;
extern int16 Turn_Speed_out;
extern uint32 MTL_PWM;
void Show_Parmeter(void)
{
    if(Show_Img_Flag)
    {
        ips114_displayimage03x((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);   // 显示原始图像
        ips114_show_int(0,50,image_para.imgtype,2);
        ips114_show_int(30,50,img_mode,2);
        if(img_mode != 5)
            ips114_show_float(50,50,image_para.offset,2,1);
        else
            ips114_show_float(50,50,Adc_Err,2,1);

        ips114_show_float(100,50,s3_encode,4,1);
         ips114_show_int(0,   80, ADC_Filter[0],5);
         ips114_show_int(50,  80, ADC_Filter[1],5);
         ips114_show_int(100, 80, ADC_Filter[2],5);
         ips114_show_uint(0, 100, dl1a_distance_mm,4);
         ips114_show_float(190, 0, Speed_K, 2, 2);
         ips114_show_int(190, 20, Speed_Acc,2);
         ips114_show_float(190, 40, Angle_z_temp, 3, 1);
         ips114_show_int(60, 100 , Adc_Err ,3);
         ips114_show_float(100, 100,OPpen_Encode,4,1);

         ips114_show_int(190, 60, Angle_Dl_v_out,5);
         ips114_show_int(190, 80, Dl_Turn_Pwm,5);
         ips114_show_int(190, 100, Turn_Speed_out,5);

    }
    else
    {
        ips114_show_string(8, ROW_0, "Show_IMG");
        ips114_show_string(8, ROW_1*16, "START:");    ips114_show_int(100, ROW_1*16, start, 1);
        ips114_show_string(8, ROW_2*16, "Speed_K:");  ips114_show_float(100, ROW_2*16, Speed_K, 2, 2);
        ips114_show_string(8, ROW_3*16, "Speed_Acc:");ips114_show_int(100, ROW_3*16, Speed_Acc, 2);
        ips114_show_string(8, ROW_4*16, "LED_SHOW:");ips114_show_int(100, ROW_4*16, Led_Flag, 1);

//        ips114_show_int(8, ROW_4*16 ,Go_speed_Target,4);
//        ips114_show_int(8, ROW_5*16 ,MTL_PWM,4);

        ips114_show_char(0,Sign_Num,'>');
        ips114_show_int(150, 100 , Row ,3);
    }


    Show_Sign();
}
void Show_Sign(void)
{
    switch(Key_Value)
    {
        case UP: {
                    Row --;
                    ips114_clear();
                }break;
        case DOWN:
                {
                    Row ++;
                    ips114_clear();
                }break;
        case MID:
                {
                    if(Row == 0 && Show_Img_Flag == 0)    {ips114_clear(); Show_Img_Flag = 1;}
                    else                                  {ips114_clear(); Show_Img_Flag = 0;}

                    if(Row == 1 && start == 0)    {ips114_clear(); start = 1;}
                    else                          {ips114_clear(); start = 0;}
                }break;
        case LEFT:
                {
                    if(Row == 2)                  {ips114_clear(); Speed_K -= 0.05;}
                    if(Row == 3)                  {ips114_clear(); Speed_Acc -= 1;}
                    if(Row == 4)                  {ips114_clear(); Led_Flag -= 1;}
                }break;
        case RIGHT:
                {
                    if(Row == 2)                  {ips114_clear(); Speed_K += 0.05;}
                    if(Row == 3)                  {ips114_clear(); Speed_Acc += 1;}
                    if(Row == 4)                  {ips114_clear(); Led_Flag += 1;}
                }break;
    }

    Row = Row > 7 ? 0:Row;
    Row = Row < 0 ? 7:Row;
    Sign_Num = 16*Row;


}


