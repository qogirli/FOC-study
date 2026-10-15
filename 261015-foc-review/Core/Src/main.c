/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TWO_PI                 6.2831853f
#define SQRT_3                 1.7320508f
#define SIN_60                 0.8660254f // sqrt(3) / 2
#define COS_60                 0.5f
#define PWM_HZ                 20000.0f   // PWM 周期中断频率，每个周期算一次
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 开环设定：你改这三个值就能改变转动。 */
float frequency = 1.0f;        // 电频率，Hz（电角度每秒转几圈）
float voltage   = 1.0f;        // 电压矢量大小，V（大小决定力矩）
float Udc       = 18.0f;       // 母线电压，V

volatile float theta = 0.0f;   // 电压矢量的方向（电角度），rad
volatile float Ualpha = 0.0f;  // 电压矢量在 alpha 轴的分量，V
volatile float Ubeta  = 0.0f;  // 电压矢量在 beta 轴的分量，V

/* SVPWM 的中间结果，放全局只为了调试时能看。 */
volatile uint8_t sector;       // 电压矢量落在第几个扇区（1~6）
volatile float Tx, Ty, T0;     // 两个相邻有效矢量和零矢量占一个 PWM 周期的比例
volatile float dutyA, dutyB, dutyC; // 三相上桥臂占空比，0~1

volatile uint8_t motor_request = 0; // 按键切换：0 停止，1 运行
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* SVPWM：输入电压矢量 (Ualpha, Ubeta)，输出三相占空比，写进三个 CCR。 */
static void SVPWM_Update(float ualpha, float ubeta)
{
    float U1, U2, U3;
    float k = SQRT_3 / Udc;
    uint8_t N = 0;

    /* 第一步：把矢量投影到三个轴上，看正负判断扇区。 */
    U1 = ubeta;
    U2 =  SIN_60 * ualpha - COS_60 * ubeta;
    U3 = -SIN_60 * ualpha - COS_60 * ubeta;

    if (U1 > 0.0f) N += 1;
    if (U2 > 0.0f) N += 2;
    if (U3 > 0.0f) N += 4;

    /* 第二步：按扇区算两个有效矢量的作用时间 Tx、Ty。 */
    switch (N)
    {
        case 3: sector = 1; Tx =  k * U2; Ty =  k * U1; break;
        case 1: sector = 2; Tx = -k * U2; Ty = -k * U3; break;
        case 5: sector = 3; Tx =  k * U1; Ty =  k * U3; break;
        case 4: sector = 4; Tx = -k * U1; Ty = -k * U2; break;
        case 6: sector = 5; Tx =  k * U3; Ty =  k * U2; break;
        case 2: sector = 6; Tx = -k * U3; Ty = -k * U1; break;
        default: sector = 0; Tx = 0.0f; Ty = 0.0f; break;
    }
    T0 = 1.0f - Tx - Ty; // 剩下的时间给零矢量

    /* 第三步：零矢量对半分给 000 和 111，再把 Ty、Tx 累加上去，得到三相占空比。 */
    switch (sector)
    {
        case 1: dutyC = T0 / 2.0f; dutyB = dutyC + Ty; dutyA = dutyB + Tx; break;
        case 2: dutyC = T0 / 2.0f; dutyA = dutyC + Ty; dutyB = dutyA + Tx; break;
        case 3: dutyA = T0 / 2.0f; dutyC = dutyA + Ty; dutyB = dutyC + Tx; break;
        case 4: dutyA = T0 / 2.0f; dutyB = dutyA + Ty; dutyC = dutyB + Tx; break;
        case 5: dutyB = T0 / 2.0f; dutyA = dutyB + Ty; dutyC = dutyA + Tx; break;
        case 6: dutyB = T0 / 2.0f; dutyC = dutyB + Ty; dutyA = dutyC + Tx; break;
        default: dutyA = dutyB = dutyC = 0.0f; break;
    }

    /* 第四步：占空比乘上计数周期，就是 CCR。 */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,
            (uint32_t)(dutyA * __HAL_TIM_GET_AUTORELOAD(&htim8)));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,
            (uint32_t)(dutyB * __HAL_TIM_GET_AUTORELOAD(&htim8)));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,
            (uint32_t)(dutyC * __HAL_TIM_GET_AUTORELOAD(&htim8)));
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
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  /* 上电先关驱动（MOTOR_EN 低电平）。 */
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);

  /* 启动三相 PWM（上桥臂和下桥臂），此时驱动还是关的。 */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

  /* 第 4 通道只用来产生每个周期一次的中断，FOC 计算都在中断里做。 */
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* KEY1：松开后再按下，切换启停。 */
    static uint8_t key_armed = 0;

    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET)
    {
        key_armed = 1;
    }
    else if (key_armed == 1)
    {
        key_armed = 0;
        motor_request = !motor_request;
    }

    HAL_Delay(20); // 按键轮询间隔
  }
  /* USER CODE END 3 */
}

/* 系统时钟168MHz。 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* HSE经PLL产生系统时钟。 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* 配置系统和总线时钟。 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 每个 PWM 周期（50 us）进来一次。 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t pwm_prepared = 0;

    if ((htim->Instance != TIM8) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_4))
    {
        return;
    }

    if (motor_request == 0)
    {
        /* 停止：关驱动，角度回零。 */
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
        pwm_prepared = 0;
        theta = 0.0f;
        return;
    }

    if (pwm_prepared == 0)
    {
        /* 第一个周期先算好 CCR，驱动仍然关着，避免一上来输出乱波形。 */
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
        theta = 0.0f;
        pwm_prepared = 1;
    }
    else
    {
        /* CCR 已经装载，开驱动，并让电压矢量转一小步。 */
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
        theta += TWO_PI * frequency / PWM_HZ;
        if (theta >= TWO_PI) theta -= TWO_PI;
    }

    /* 矢量 = 大小 × (cos 方向, sin 方向)，交给 SVPWM。 */
    Ualpha = voltage * cosf(theta);
    Ubeta  = voltage * sinf(theta);
    SVPWM_Update(Ualpha, Ubeta);
}

/* USER CODE END 4 */

/* 出错时关闭驱动并停在这里。 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/* 参数断言失败回调。 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
