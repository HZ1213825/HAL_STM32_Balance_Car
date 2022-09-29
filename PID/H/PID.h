#ifndef _PID_H_
#define _PID_H_
#include "main.h"
extern int Speed_Integral, Speed_Err_Last;
int PID_Upright(float Med, float Angle, float gyro);
int PID_Speed(int16_t expect, int16_t speed);
int PID_Steering(int16_t expect, float angle, float gyro);
#endif
