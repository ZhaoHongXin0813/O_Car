/*
 * ADC.c
 *
 *  Created on: 2023年6月10日
 *      Author: qingyu
 */
#include "ADC.h"


FiterStruct Filter_parm;
uint16 ADC_MAX=3500;
uint16 ADC_MIN=140;
/**********************************************************************************************************
*函 数 名: Init_Filter
*功能说明: 二阶IIR低通滤波器系数计算，该滤波器由二阶模拟低通滤波电路数字化推导而来
*形    参: 滤波器运行频率 截止频率 滤波器结构体指针
*返 回 值: 无
**********************************************************************************************************/

void Init_Filter(float deltaT, float Fcut, FiterStruct *FilterPara)
{
    float a = 1 / (2 * M_PI * Fcut * deltaT);
    FilterPara->b0 = 1 / (a*a + 3*a + 1);
    FilterPara->a1 = (2*a*a + 3*a) / (a*a + 3*a + 1);
    FilterPara->a2 = (a*a) / (a*a + 3*a + 1);
}
/**********************************************************************************************************
*函 数 名: IIR_Filter
*功能说明: 二阶IIR低通滤波器过程实现
*形    参: 滤波器结构体指针 原始数据
*返 回 值: 经过滤波的数据
**********************************************************************************************************/
void IIR_Filter(FiterStruct FilterPara, uint16 ADC_Filter_befor[3], uint16 ADC_Filter_after[3])
{
    uint16 i;
    static  uint16 preout[3];
    static  uint16 lastout[3];

    for(i=0; i<3; i++)
    {
        ADC_Filter_after[i] = ADC_Filter_befor[i] * FilterPara.b0 + lastout[i] * FilterPara.a1
                                                    - preout[i] * FilterPara.a2;

        preout[i] = lastout[i];
        lastout[i] = ADC_Filter_after[i];
    }
}
/**********************************************************************************************************
*函 数 名: ADC_To_One
*功能说明: 电感值归一化
*形    参: 滤波后的电感值数组
*返 回 值: 无
**********************************************************************************************************/
void ADC_To_One(uint16 ADC[3])
{
    uint8 i;
    for(i=0;i<3;i++)
    {
        ADC[i] = (ADC[i]*100)/(ADC_MAX - ADC_MIN);
    }
}






