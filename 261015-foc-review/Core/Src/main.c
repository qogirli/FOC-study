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
#include <stdio.h>
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

/* 电流采样：ADC 读到的是电压，要换算成电流。 */
#define ADC_VREF_V             3.3f       // ADC 参考电压，V
#define ADC_COUNTS             4096.0f    // 12 位 ADC，满量程 4096
#define SHUNT_OHM              0.02f      // 采样电阻，欧姆
#define AMP_GAIN               (5100.0f / (1200.0f + 68.0f)) // 运放放大倍数
#define CURRENT_SIGN           (-1.0f)    // 实测方向与预期相反，所以取反
#define CAL_SKIP_SAMPLES       2000U      // 上电后先丢掉前 2000 个（0.1 s），等电路稳定
#define CAL_AVG_SAMPLES        256U       // 再平均 256 个，得到零电流时的读数
#define PRINT_PERIOD_MS        200U       // 串口打印间隔
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

/* 电流采样结果。 */
volatile uint16_t raw_u = 0, raw_v = 0;   // ADC 原始读数（0~4095）
float offset_u = 0.0f, offset_v = 0.0f;   // 零电流时的读数
volatile uint8_t offset_ready = 0;        // 零偏校准完成标志
volatile float current_u = 0.0f;          // U 相电流，A
volatile float current_v = 0.0f;          // V 相电流，A
volatile float current_w = 0.0f;          // W 相电流，A（没有测，由 U、V 算出）

/* 打印用的累加：每个打印周期取平均。 */
static float sum_u = 0.0f, sum_v = 0.0f;
static uint32_t sum_n = 0;

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
/* ADC 读数减零偏，再换算成电流：读数 → 电压 → 除以放大倍数 → 除以采样电阻。 */
static float ADC_ToCurrent(uint16_t raw, float offset)
{
    float volts = ((float)raw - offset) * ADC_VREF_V / ADC_COUNTS;
    return CURRENT_SIGN * volts / AMP_GAIN / SHUNT_OHM;
}

/* 上电后在驱动关闭、电流为零时，平均若干次读数，得到零电流对应的读数。 */
static void Offset_Calibrate(uint16_t u, uint16_t v)
{
    static uint32_t n = 0;
    static float acc_u = 0.0f, acc_v = 0.0f;

    n++;
    if (n <= CAL_SKIP_SAMPLES) return; // 前面的先丢掉

    acc_u += (float)u;
    acc_v += (float)v;
    if (n == CAL_SKIP_SAMPLES + CAL_AVG_SAMPLES)
    {
        offset_u = acc_u / CAL_AVG_SAMPLES;
        offset_v = acc_v / CAL_AVG_SAMPLES;
        offset_ready = 1;
    }
}

/* 每秒 5 次从串口打印窗口内的平均电流。 */
static void Print_Currents(void)
{
    static uint32_t last_ms = 0;
    char line[96];
    float iu, iv;
    uint32_t n;
    int len;

    if ((uint32_t)(HAL_GetTick() - last_ms) < PRINT_PERIOD_MS) return;
    last_ms = HAL_GetTick();

    /* 暂停中断，取出累加值并清零。 */
    __disable_irq();
    n = sum_n;
    iu = sum_u;
    iv = sum_v;
    sum_n = 0;
    sum_u = sum_v = 0.0f;
    __enable_irq();

    if (n != 0U)
    {
        iu /= (float)n;
        iv /= (float)n;
    }
    len = snprintf(line, sizeof(line),
        "cal=%u raw_u=%u raw_v=%u Iu_mA=%ld Iv_mA=%ld\r\n",
        (unsigned)offset_ready, (unsigned)raw_u, (unsigned)raw_v,
        (long)(iu * 1000.0f), (long)(iv * 1000.0f));
    HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)len, 5);
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  /* 上电先关驱动（MOTOR_EN 低电平）。校准零偏时电机必须没有电流。 */
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);

  /* ADC 开始等待触发：每次 TIM8 通道 4 到点，U 相和 V 相同时采一次。 */
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc2);

  /* 启动三相 PWM（上桥臂和下桥臂），此时驱动还是关的。 */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

  /* 第 4 通道有两个用处：触发 ADC 采样，和每个周期产生一次中断。 */
  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_CC4);
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

    if (offset_ready == 0)
    {
        /* 零偏没校准完，不允许启动。 */
        motor_request = 0;
        key_armed = 0;
    }
    else if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET)
    {
        key_armed = 1;
    }
    else if (key_armed == 1)
    {
        key_armed = 0;
        motor_request = !motor_request;
    }

    Print_Currents();
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

/* U、V 两路 ADC 各转换完成一次就进来一次，两路都到了才处理。 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static uint8_t got_u = 0, got_v = 0;

    if (hadc->Instance == ADC1)
    {
        raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        got_u = 1;
    }
    if (hadc->Instance == ADC2)
    {
        raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        got_v = 1;
    }
    if ((got_u == 0) || (got_v == 0)) return;
    got_u = got_v = 0;

    if (offset_ready == 0)
    {
        Offset_Calibrate(raw_u, raw_v);
        return;
    }

    current_u = ADC_ToCurrent(raw_u, offset_u);
    current_v = ADC_ToCurrent(raw_v, offset_v);
    current_w = -current_u - current_v; // 三相电流之和为零

    sum_u += current_u;
    sum_v += current_v;
    sum_n++;
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
