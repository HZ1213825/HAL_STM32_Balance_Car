/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PWM_Lim 500
#define Z_Static_Deviation (-16)
#define Mechanical_Median (-3.5)

uint8_t init_OK = 0;
float pitch, roll, yaw;
short gyro_x, gyro_y, gyro_z;
int16_t Speed = 0;
uint16_t T_1 = 0;
int16_t T_2 = 0;
int16_t T_3 = 0;

float Speed_L, Speed_R;
int16_t PWM_Upright = 0,
        PWM_Speed = -4,
        PWM_Steering = 0,
        PID_PWM_R = 0,
        PID_PWM_L = 0;

void Pick_Up_Test()
{
  if (init_OK == 1)
  {
    if (PWM_Upright + PWM_Speed > 3000)
      T_2++;
    else
    {
      if (pitch > 40 || pitch < -40)
        T_2++;
      else
        T_2 = 0;
    }

    if (T_2 > 100)
    {
      init_OK = 0;
      Set_Motor_PWM(MY_R, 0);
      Set_Motor_PWM(MY_L, 0);
    }
  }
  if (init_OK == 0)
  {
    if (pitch < 5 + Mechanical_Median && pitch > -5 + Mechanical_Median && gyro_y < 5 && gyro_y > -5)
    {
      T_3++;
    }

    if (T_3 > 400)
    {
      __set_FAULTMASK(1); //关中断
      NVIC_SystemReset(); //复位
    }
  }
}
void PID()
{

  T_1++;
  if (T_1 == 72)
  { //每5ms计算一次PID
    T_1 = 0;
    // printf("pitch:%f,roll:%f\r\n", pitch, roll);
    if (init_OK == 1)
    {

      Speed_L = Get_Speed(MY_L);
      Speed_R = Get_Speed(MY_R);
      Speed = Speed_L + Speed_R; //获取转速

      PWM_Upright = PID_Upright(Mechanical_Median, pitch, gyro_y);       //直立环
      PWM_Speed = PID_Speed(0, Speed);                                   //速度环
      PWM_Steering = PID_Steering(10, yaw, gyro_z + Z_Static_Deviation); //转向环

      //计算并级PID
      PID_PWM_R = PWM_Upright + PWM_Speed - PWM_Steering;
      PID_PWM_L = PWM_Upright + PWM_Speed + PWM_Steering;

      // PWM低占空比低时不给电机(避免堵转异响)
      if (PID_PWM_L > PWM_Lim || PID_PWM_L < -PWM_Lim)
        Set_Motor_PWM(MY_L, PID_PWM_L);
      if (PID_PWM_R > PWM_Lim || PID_PWM_R < -PWM_Lim)
        Set_Motor_PWM(MY_R, PID_PWM_R);
    }
    Pick_Up_Test();
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim == &htim1)
  {
    PID();
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Init();
  init_OK = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //开指示灯
    MPU_Get_Gyroscope(&gyro_x, &gyro_y, &gyro_z);       //读取角加速度
    mpu_dmp_get_data(&pitch, &roll, &yaw);              //读取DMP解算的欧拉角

    printf("%f\r\n", pitch);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
