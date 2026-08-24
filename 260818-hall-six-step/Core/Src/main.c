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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
volatile uint8_t motor_step = 0xFF;
volatile	uint32_t old_time = 0;
volatile uint16_t speed = 0;
volatile uint8_t printf_flag =0;
volatile uint16_t speed_print =0;
volatile uint16_t target_rpm = 200;
volatile uint16_t pwm_value = 420;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Motor_SetStep(uint8_t step)
{

  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);

  switch (step)
  {
    case 2: //A+ B- 
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwm_value);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 65535); //直接拉到最满，确保恒定导通
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
      break;

    case 6: //B- C+
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwm_value);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 65535); //直接拉到最满，确保恒定导通
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
      break;

    case 4: //A- C+
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwm_value);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 65535); //直接拉到最满，确保恒定导通
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
      break;
     
    case 5: //A- B+
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm_value);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 65535); //直接拉到最满，确保恒定导通
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
      break;
    
    case 1: //C- B+
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm_value);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 65535); //直接拉到最满，确保恒定导通
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
      break;
      
    case 3: //C- a+
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwm_value);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 65535); //直接拉到最满，确保恒定导通
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
      break;
    
    default:
            /* 非法状态保持三相关闭 */
      break;
  }
}

static uint8_t HALL_Read(void)
{
    uint8_t hall_state = 0;

    hall_state |= HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) << 2;
    hall_state |= HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11) << 1;
    hall_state |= HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12);

    return hall_state;
}
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
  /* USER CODE BEGIN 2 */
	HAL_TIMEx_HallSensor_Start_IT(&htim5);

HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

/*
 * CubeMX的HAL_TIM_PWM_ConfigChannel会开启CCR预装载。
 * 当前模仿502，让CCR修改立即生效。
 */
__HAL_TIM_DISABLE_OCxPRELOAD(&htim8, TIM_CHANNEL_1);
__HAL_TIM_DISABLE_OCxPRELOAD(&htim8, TIM_CHANNEL_2);
__HAL_TIM_DISABLE_OCxPRELOAD(&htim8, TIM_CHANNEL_3);

/* 先配置第一拍：A+、B- */


/*
 * 第一次测试时先注释这一句，只观察六路逻辑波形。
 * 波形正确并接好限流电源后，再取消注释。
 */
HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
HAL_Delay(10);  // 你当前缺的就是这个

motor_step = HALL_Read();
Motor_SetStep(motor_step);
//HAL_Delay(100);

//Motor_SetStep(2);
//HAL_Delay(100);

//Motor_SetStep(3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t control_time = 0;
  static int32_t speed_integral = 0;

  while (1)
  {
        uint32_t now = HAL_GetTick();

        if ((uint32_t)(now - control_time) >= 30U)
        {
            int32_t speed_error;
            int32_t pwm_output;

            control_time = now;

            speed_error = (int32_t)target_rpm - (int32_t)speed_print;

            speed_integral += speed_error;

            if (speed_integral < 0)
            {
                speed_integral = 0;
            }
            else if (speed_integral > 42000)
            {
                speed_integral = 42000;
            }

            /* PI控制：Kp = 1，Ki = 1/20 */
            pwm_output = speed_error + speed_integral / 20;

            if (pwm_output < 0)
            {
                pwm_output = 0;
            }
            else if (pwm_output > 840)
            {
                pwm_output = 840;
            }

            pwm_value = (uint16_t)pwm_output;
        }

		      if (printf_flag == 1)
        {
            printf_flag=0;
            printf("rpm=%u pwm_value=%d\r\n", (unsigned int)speed_print,pwm_value);

        }

//		uint32_t time = __HAL_TIM_GET_COUNTER(&htim5);
//		
//		// HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
//		// HAL_Delay(1000);
////    Motor_SetStep(motor_step);

////    motor_step++;

////    if (motor_step > 6)
////    {
////        motor_step = 1;
////    }
//		
//		volatile uint8_t i = HALL_Read();

//		if (i != motor_step)
//		{
//				motor_step = i;
//				Motor_SetStep(i);
//				uint32_t timediff = time - old_time; 
//				if ((old_time != 0) && (timediff != 0))
//				{
//						speed += 60000000U / (timediff * 24U);
//				}
//				old_time = time;
//				if(i==1)
//				{
//					speed = speed/6;
//					printf("rpm=%u\r\n",(unsigned int)speed);
//					speed = 0;
//					
//				}
//		}
		
//		volatile uint16_t speed = 0;
//		speed = 
    /* 每20ms换一拍 */
//    HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        uint32_t time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (time != 0)
        {
            speed += 60000000U / (time * 24U);
        }

        motor_step = HALL_Read();
        Motor_SetStep(motor_step);

        if (motor_step == 1)
        {
					printf_flag = 1;

					speed_print = speed;
					speed_print = speed_print / 6;
					speed =0;
//            speed = speed / 6;
//            printf("rpm=%u\r\n", (unsigned int)speed);
//            speed = 0;
        }
    }
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
