#include "MY.h"
/**
 * @brief PWM限幅
 * @param Val:PWM的指针
 * @return 无
 * @author HZ12138
 * @date 2022-08-14 23:20:49
 */
void PWM_Amplitude_Limit(int16_t *Val)
{
    int a = 4500;
    if (*Val < -a)
        *Val = -a;
    if (*Val > a)
        *Val = a;
}
/**
 * @brief 设置控制电机的PWM
 * @param R_L:左右
 * @param PWM_Val:PWM值(正值正转,负值反转)
 * @return 无
 * @author HZ12138
 * @date 2022-08-14 23:15:36
 */
void Set_Motor_PWM(uint8_t R_L, int16_t PWM_Val)
{
    PWM_Amplitude_Limit(&PWM_Val);
    if (R_L == MY_L)
    {
        if (PWM_Val > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_Val);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -PWM_Val);
        }
    }
    else if (R_L == MY_R)
    {
        if (PWM_Val > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_Val);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -PWM_Val);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        }
    }
}
/**
 * @brief 初始化(PWM,MPU6050)
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-08-14 23:16:47
 */
void Init(void)
{
    MPU_Init();     // mpu6050初始化
    mpu_dmp_init(); // DMP初始化

    //将电机设为低电平
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}
/**
 * @brief 获取转速
 * @param R_L:左右
 * @return 转速
 * @author HZ12138
 * @date 2022-08-14 23:17:08
 */
float Get_Speed(uint8_t R_L)
{
    int16_t zj;
    float Speed;
    if (R_L == MY_L)
    {
        zj = -__HAL_TIM_GetCounter(&htim2);
        __HAL_TIM_SetCounter(&htim2, 0);
    }
    else if (R_L == MY_R)
    {
        zj = __HAL_TIM_GetCounter(&htim3);
        __HAL_TIM_SetCounter(&htim3, 0);
    }
    // Speed = (float)zj / (4 * 15 * 34) * 200 * 60;
    Speed = zj;
    return Speed;
}
