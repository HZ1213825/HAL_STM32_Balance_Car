#include "PID.h"
// float PID_Upright_Kp = 0; //直立环Kp
// float PID_Upright_Kd = 0; //直立环Kd

// float PID_Speed_Kp = 0; //速度环Kp
// float PID_Speed_Ki = 0; //速度环Ki

float PID_Steering_Kp = 0; //转向环Kp
float PID_Steering_Kd = 5; //转向环Kd

float PID_Upright_Kp = 300; //直立环Kp
float PID_Upright_Kd = 1.5; //直立环Kd

float PID_Speed_Kp = 200;       //速度环Kp
float PID_Speed_Ki = 200 / 200; //速度环Ki

// float PID_Steering_Kp = 0; //转向环Kp
// float PID_Steering_Kd = 5; //转向环Kd
int Speed_Integral, Speed_Err_Last;
/**
 * @brief 直立环(位置式PD)
 * @param expect:期望角度
 * @param angle:真实角度
 * @param gyro:真实角速度
 * @return PWM
 * @author HZ12138
 * @date 2022-08-16 23:17:39
 */
int PID_Upright(float expect, float angle, float gyro)
{
    int PWM_out;
    PWM_out = PID_Upright_Kp * (angle - expect) + PID_Upright_Kd * gyro;
    return PWM_out;
}
/**
 * @brief 速度环(位置式PI)
 * @param expect:期望速度
 * @param speed:当前速度
 * @return PWM
 * @author HZ12138
 * @date 2022-08-21 13:06:59
 */
int PID_Speed(int16_t expect, int16_t speed)
{
    int PWM_out, Speed_Err;
    float a = 0.7;

    Speed_Err = speed - expect;                           //误差计算
    Speed_Err = (1 - a) * Speed_Err + a * Speed_Err_Last; //低通滤波
    Speed_Err_Last = Speed_Err;                           //更新上次误差

    Speed_Integral += Speed_Err; //积分计算

    if (Speed_Integral > 10000)
        Speed_Integral = 0;
    if (Speed_Integral < -10000)
        Speed_Integral = 0; //积分限幅

    PWM_out = PID_Speed_Kp * Speed_Err + PID_Speed_Ki * Speed_Integral; //计算输出
    return PWM_out;
}
/**
 * @brief 转向环(位置式PD)
 * @param expect:期望转向的角度
 * @param angle:真实的角度(一般是yaw)
 * @param gyro:真实角加速度(一般是gyro_z)
 * @return PWM
 * @author HZ12138
 * @date 2022-08-21 13:07:02
 */
int PID_Steering(int16_t expect, float angle, float gyro)
{
    int PWM_out;
    float a = 0.7;
    static float gyro_Last;
    gyro = (1 - a) * gyro + a * gyro_Last; //低通滤波
    PWM_out = PID_Steering_Kp * (angle - expect) + PID_Steering_Kd * gyro;
    return PWM_out;
}
