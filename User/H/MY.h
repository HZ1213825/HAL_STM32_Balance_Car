#ifndef __MY_H
#define __MY_H
#include "main.h"
#define MY_L 1
#define MY_R 0
void Init(void);
void Set_Motor_PWM(uint8_t R_L, int16_t PWM_Val);
float Get_Speed(uint8_t R_L);
void PWM_Amplitude_Limit(int16_t *Val);
#endif
