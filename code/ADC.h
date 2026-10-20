/*
 * ADC.h
 *
 *  Created on: 2023Äê6ÔÂ10ÈÕ
 *      Author: qingyu
 */

#ifndef CODE_ADC_H_
#define CODE_ADC_H_

#include "zf_common_headfile.h"

#define M_PI        3.141592653f
#define M_FT        0.001f
#define M_FC        100.0f

typedef struct
{
    float b0;
    float a1;
    float a2;
}FiterStruct;

void Init_Filter(float deltaT, float Fcut, FiterStruct *FilterPara);
void IIR_Filter(FiterStruct FilterPara, uint16 ADC_Filter_befor[3], uint16 ADC_Filter_after[3]);
void ADC_To_One(uint16 ADC[3]);
#endif /* CODE_ADC_H_ */
