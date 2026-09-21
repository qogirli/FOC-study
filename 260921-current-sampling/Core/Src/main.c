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
#define SIN_60                 0.8660254f // sqrt(3) / 2
#define COS_60                 0.5f
#define SQRT_3                 1.7320508f // 根号 3
#define CURRENT_SAMPLING_ONLY  1  // 零电流采样阶段：禁止启动功率级

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile float theta = 0.0f;  // 电角度，单位：弧度
float frequency = 1.0f;       // 电频率，1 Hz = 电角度每秒转一圈
float voltage = 1.0f;         // 电压矢量幅值，单位 V
volatile float Ualpha = 0.0f;
volatile float Ubeta  = 0.0f;
float U1, U2, U3;
uint8_t N;
volatile uint8_t sector;
float Udc = 18.0f;          // 母线电压，单位 V
float k;
volatile float Tx, Ty, T0;  // 占整个 PWM 周期的时间比例
volatile float dutyA, dutyB, dutyC; // 三相上桥臂占空比
volatile uint8_t motor_request = 0; // while 写请求，中断执行：0 停止，1 运行
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  /* 默认关闭功率级，等SVPWM参数准备完成后再启动。 */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
#if !CURRENT_SAMPLING_ONLY
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 4198);
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);

HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);

HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

//	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if !CURRENT_SAMPLING_ONLY
    static uint8_t key_armed = 0;

    /* KEY1：PE0，松开为高，按下为低；硬件已消抖。 */
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET)
    {
        key_armed = 1;
    }
    else if ((HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) &&
             (key_armed == 1))
    {
        key_armed = 0;
        motor_request = !motor_request;
    }
#endif

    HAL_Delay(20); // 按键轮询间隔，不阻止 TIM8 中断
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
#if CURRENT_SAMPLING_ONLY
    (void)htim;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    return;
#else
    if ((htim->Instance == TIM8) &&
        (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4))
    {
        static uint8_t pwm_prepared = 0;

        if (motor_request == 0)
        {
            /* 关闭功率级，定时器仍然运行。 */
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
            pwm_prepared = 0;
            theta = 0.0f;
            return;
        }

        if (pwm_prepared == 0)
        {
            /* 第一次先保持关闭，用零角度计算下面的三路 CCR。 */
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
            theta = 0.0f;
            pwm_prepared = 1;
        }
        else
        {
            /* 下一次 CH4 中断，上一组 CCR 已经过更新事件装载。 */
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
            theta += TWO_PI * frequency / 20000.0f;
        }

			if (theta >= TWO_PI)
			{
				theta -= TWO_PI;
			}

			Ualpha = voltage * cosf(theta);
			Ubeta  = voltage * sinf(theta);

			U1 = Ubeta;
			U2 =  SIN_60 * Ualpha - COS_60 * Ubeta;
			U3 = -SIN_60 * Ualpha - COS_60 * Ubeta;

			N = 0;
			if (U1 > 0.0f) N += 1;
			if (U2 > 0.0f) N += 2;
			if (U3 > 0.0f) N += 4;

			k = SQRT_3 / Udc;

			switch (N)
			{
					case 3:
							sector = 1;
							Tx = k * U2;
							Ty = k * U1;
							break;

					case 1:
							sector = 2;
							Tx = -k * U2;
							Ty = -k * U3;
							break;

					case 5:
							sector = 3;
							Tx = k * U1;
							Ty = k * U3;
							break;

					case 4:
							sector = 4;
							Tx = -k * U1;
							Ty = -k * U2;
							break;

					case 6:
							sector = 5;
							Tx = k * U3;
							Ty = k * U2;
							break;

					case 2:
							sector = 6;
							Tx = -k * U3;
							Ty = -k * U1;
							break;

					default:
							sector = 0;
							Tx = 0.0f;
							Ty = 0.0f;
							break;
			}
			T0 = 1.0f - Tx - Ty;

			/* 零矢量时间均分给 000 和 111，再累加有效矢量时间。 */
			switch (sector)
			{
					case 1:
							dutyC = T0 / 2.0f;
							dutyB = dutyC + Ty;
							dutyA = dutyB + Tx;
							break;

					case 2:
							dutyC = T0 / 2.0f;
							dutyA = dutyC + Ty;
							dutyB = dutyA + Tx;
							break;

					case 3:
							dutyA = T0 / 2.0f;
							dutyC = dutyA + Ty;
							dutyB = dutyC + Tx;
							break;

					case 4:
							dutyA = T0 / 2.0f;
							dutyB = dutyA + Ty;
							dutyC = dutyB + Tx;
							break;

					case 5:
							dutyB = T0 / 2.0f;
							dutyA = dutyB + Ty;
							dutyC = dutyA + Tx;
							break;

					case 6:
							dutyB = T0 / 2.0f;
							dutyC = dutyB + Ty;
							dutyA = dutyC + Tx;
							break;

					default:
							dutyA = 0.0f;
							dutyB = 0.0f;
							dutyC = 0.0f;
							break;
			}

			/* 占空比换算成比较值，功率级由上面的启停逻辑控制。 */
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,
					(uint32_t)(dutyA * __HAL_TIM_GET_AUTORELOAD(&htim8)));
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,
					(uint32_t)(dutyB * __HAL_TIM_GET_AUTORELOAD(&htim8)));
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,
					(uint32_t)(dutyC * __HAL_TIM_GET_AUTORELOAD(&htim8)));
    }
#endif
}
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

#ifdef  USE_FULL_ASSERT
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
