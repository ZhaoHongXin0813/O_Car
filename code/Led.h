/*
 * Led.h
 *
 *  Created on: 2023年8月1日
 *      Author: qingyu
 */

#ifndef CODE_LED_H_
#define CODE_LED_H_
#include "zf_common_headfile.h"

#define MTL_PIN     ATOM2_CH7_P20_7
#define DS1         P20_8
#define ST1         P20_9
#define SH1         P33_9

#define DS2         P33_11
#define ST2         P33_12
#define SH2         P33_13

void MTL_PWM_Init(void);
void Set_MTL_PWM(void);
uint8 Reverse_Bin(uint8 a);      //翻转二进制值 如0101 0101 翻转后为 1010 1010
//void TLD_74HC595_WriteByte(uint16 Byte);
//void L_74HC595_WriteByte(uint16 Byte);
void TLD_74HC595_WriteByte(uint8 Byte_H , uint8 Byte_L);
void L_74HC595_WriteByte(uint8 Byte_H , uint8 Byte_L);
void Led_Matrix(uint8 Column, uint8 Data);
void Show_Infineon(void);
void Show_Zhuo_Da(void);
void Show_Chu_Ku(void);
void Show_Ru_Ku(void);
void Welcome_(void);
void Led_Show_Num(uint8 Number);
void Led_Matrix_Mo(uint8 Column_1, uint8 Data_1,uint8 Column_2, uint8 Data_2);

#endif /* CODE_LED_H_ */
