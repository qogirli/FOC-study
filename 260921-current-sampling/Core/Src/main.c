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
#define SIN_60                 0.8660254f // sqrt(3) / 2
#define COS_60                 0.5f
#define SQRT_3                 1.7320508f // 根号 3
#define PWM_UPDATE_HZ          20000.0f // PWM更新频率，Hz
#define HALL_TIMER_HZ          1000000.0f // 霍尔计时频率，1计数=1us
#define HALL_EDGES_PER_CYCLE   6.0f       // 每电角度圈6个区间
#define HALL_SECTOR_RAD        (TWO_PI / HALL_EDGES_PER_CYCLE)
#define HALL_PHASE_SHIFT_DEG   120.0f // 硬石655参考偏移，实际接线待验证
#define HALL_PHASE_SHIFT_RAD   (HALL_PHASE_SHIFT_DEG * TWO_PI / 360.0f)
#define HALL_STOP_TIMEOUT_MS   1000U     // 无边沿超时，ms
#define MONITOR_PERIOD_MS      100U    // 打印间隔，ms
#define MONITOR_TX_TIMEOUT_MS  5U      // 串口发送超时，ms
#define ADC_SETTLE_SAMPLES     2000U // 丢弃上电后的前100ms采样
#define ADC_OFFSET_SAMPLES     256U  // 零点平均次数
#define ADC_COUNTS_PER_RANGE   4096.0f
#define ADC_VREF_V             3.3f   // 标称ADC参考电压，V
#define CURRENT_SHUNT_OHM      0.02f  // 2024版原理图 R12/R18/R24
#define CURRENT_AMP_GAIN       (5100.0f / (1200.0f + 68.0f))
#define CURRENT_PI_KP          0.5f // 计算练习值，V/A；尚未调参
#define CURRENT_PI_KI          5.0f // V/(A*s)
#define CURRENT_PI_DT          (1.0f / PWM_UPDATE_HZ) // 采样间隔，s
#define CURRENT_PI_LIMIT_V     1.0f // 每轴电压指令限幅，V

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile float theta = 0.0f;  // 开环电角度，rad
float frequency = 1.0f;       // 开环电频率，Hz
float voltage = 1.0f;         // 电压矢量幅值，V
volatile float Ualpha = 0.0f;
volatile float Ubeta  = 0.0f;
float U1, U2, U3;
uint8_t N;
volatile uint8_t sector;
float Udc = 18.0f;          // 母线电压，单位 V
float k;
volatile float Tx, Ty, T0;  // 占整个 PWM 周期的时间比例
volatile float dutyA, dutyB, dutyC; // 三相上桥臂占空比
volatile uint8_t motor_request = 0; // 启停请求：0停止，1运行
volatile uint16_t adc_u_raw = 0;
volatile uint16_t adc_v_raw = 0;
volatile uint32_t adc_u_count = 0;
volatile uint32_t adc_v_count = 0;
volatile float adc_u_offset = 0.0f; // 零电流ADC读数
volatile float adc_v_offset = 0.0f;
volatile uint8_t adc_u_offset_ready = 0; // 零点平均完成标志
volatile uint8_t adc_v_offset_ready = 0;
volatile float current_u = 0.0f; // 相电流，A；正方向待验证
volatile float current_v = 0.0f;
volatile float current_w = 0.0f; // 由U、V重建
volatile float current_alpha = 0.0f;
volatile float current_beta = 0.0f;
volatile float current_d = 0.0f; // dq电流，A；仅观察
volatile float current_q = 0.0f;
volatile float park_angle = 0.0f; // 霍尔角度加零点偏移，rad
volatile uint8_t park_angle_valid = 0; // 本次dq计算是否有有效角度
volatile float Id_ref = 0.0f, Iq_ref = 0.2f; // 目标电流，A；仅用于计算练习
volatile float Ud = 0.0f, Uq = 0.0f;         // PI电压指令，V；尚未接入PWM
volatile uint8_t current_pi_valid = 0;
volatile float Ualpha_cl = 0.0f, Ubeta_cl = 0.0f; // 逆Park结果，V；仅观察，不进PWM
static float integral_d = 0.0f, integral_q = 0.0f; // 各轴积分电压，V
volatile uint32_t clarke_count = 0; // 电流变换次数
volatile uint8_t hall_state = 0;       // H3 H2 H1：PH12 PH11 PH10
volatile int8_t hall_direction = 0;    // +1正转，-1反转，0未知
volatile uint32_t hall_period_us = 0; // 上一区间用时，us
volatile uint32_t hall_edge_count = 0;
volatile float hall_electrical_hz = 0.0f; // 有符号电频率；超时清零
volatile float hall_angle = 0.0f;       // 霍尔相对电角度，rad；零点未对齐磁极
volatile uint8_t hall_angle_valid = 0;  // 插值有效标志
static volatile float hall_edge_angle = 0.0f; // 最近边沿角度，rad
static volatile uint32_t hall_last_edge_ms = 0; // 最近捕获时刻，ms
/* 串口窗口统计，每次打印后清空。 */
static volatile uint32_t monitor_samples = 0;
static volatile float monitor_u_sum = 0.0f, monitor_v_sum = 0.0f;
static volatile float monitor_u_min = 0.0f, monitor_u_max = 0.0f;
static volatile float monitor_v_min = 0.0f, monitor_v_max = 0.0f;
static volatile uint32_t monitor_dq_samples = 0;
static volatile float monitor_d_sum = 0.0f, monitor_q_sum = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ADC减零点，再除以放大倍数和采样电阻，得到电流。 */
static float ADC_ToCurrent(uint16_t raw, float offset)
{
    float voltage_delta;

    voltage_delta = ((float)raw - offset) * ADC_VREF_V / ADC_COUNTS_PER_RANGE;
    return voltage_delta / CURRENT_AMP_GAIN / CURRENT_SHUNT_OHM;
}

/* 等幅值Clarke：三相变静止两轴，alpha与U相重合。 */
static void Clarke_Update(float iu, float iv)
{
    current_w = -iu - iv;
    current_alpha = iu;
    current_beta = (iu + 2.0f * iv) / SQRT_3;
    clarke_count++;
}

/* Park：静止两轴变旋转两轴，angle决定坐标轴方向。 */
static void Park_Update(float alpha, float beta, float angle)
{
    float sin_angle = sinf(angle);
    float cos_angle = cosf(angle);

    current_d =  alpha * cos_angle + beta * sin_angle;
    current_q = -alpha * sin_angle + beta * cos_angle;
}

/* 逆Park：Ud/Uq变静止两轴电压，是Park_Update的逆变换，angle必须与Park相同。
 * 结果存入Ualpha_cl/Ubeta_cl，仅观察，不写入Ualpha/Ubeta，不影响PWM。 */
static void InversePark_Update(float ud, float uq, float angle)
{
    float sin_angle = sinf(angle);
    float cos_angle = cosf(angle);

    Ualpha_cl = ud * cos_angle - uq * sin_angle;
    Ubeta_cl  = ud * sin_angle + uq * cos_angle;
}

/* 电压指令限制在正负1V内。 */
static float Current_PI_Limit(float value)
{
    if (value > CURRENT_PI_LIMIT_V) return CURRENT_PI_LIMIT_V;
    if (value < -CURRENT_PI_LIMIT_V) return -CURRENT_PI_LIMIT_V;
    return value;
}

/* 每对电流采样后计算一次；只观察Ud、Uq，不控制PWM。 */
static void Current_PI_Update(void)
{
    float error_d, error_q;

    current_pi_valid = (motor_request != 0U) && (park_angle_valid != 0U);
    if (current_pi_valid == 0U)
    {
        integral_d = integral_q = 0.0f; // 停止或角度无效时清积分
        Ud = Uq = 0.0f;
        return;
    }

    error_d = Id_ref - current_d;
    error_q = Iq_ref - current_q;
    integral_d += CURRENT_PI_KI * error_d * CURRENT_PI_DT;
    integral_q += CURRENT_PI_KI * error_q * CURRENT_PI_DT;
    integral_d = Current_PI_Limit(integral_d);
    integral_q = Current_PI_Limit(integral_q);
    Ud = Current_PI_Limit(CURRENT_PI_KP * error_d + integral_d);
    Uq = Current_PI_Limit(CURRENT_PI_KP * error_q + integral_q);
}

/* 角度 = 最近边沿位置 + 估算角速度 × 已过时间，仅供观察。
 * TIM5、TIM8保持相同抢占优先级，避免读到更新了一半的霍尔数据。
 */
static void Hall_AngleUpdate(void)
{
    uint32_t elapsed_us; // 距最近边沿已过的时间，us
    float angle;         // 本次估算角度，rad

    /* TIM5在边沿清CNT；读取这一区间已过的时间。 */
    elapsed_us = __HAL_TIM_GET_COUNTER(&htim5);
    /* 新边沿还没处理完，等起点更新后再算。 */
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_CC1) != RESET)
    {
        hall_angle_valid = 0;
        return;
    }
    /* 1秒没边沿：清除旧速度，暂停角度推算。 */
    if ((uint32_t)(HAL_GetTick() - hall_last_edge_ms) >= HALL_STOP_TIMEOUT_MS)
    {
        hall_electrical_hz = 0.0f;
        hall_direction = 0;
        hall_period_us = 0;
        hall_angle_valid = 0;
        return; // 保留最后角度，不再按旧速度转下去
    }
    /* 没有有效速度，暂时无法推算。 */
    if ((hall_period_us == 0U) || (hall_electrical_hz == 0.0f))
    {
        hall_angle_valid = 0;
        return;
    }

    /* 已过时间达到上一段用时，却没有新边沿：标记估算无效。 */
    hall_angle_valid = elapsed_us < hall_period_us;
    /* 限制计算时间，最多推算一个60度区间。 */
    if (elapsed_us > hall_period_us)
        elapsed_us = hall_period_us;
    /* 频率换成rad/s，时间换成秒；带符号速度决定加减方向。 */
    angle = hall_edge_angle + TWO_PI * hall_electrical_hz *
                              ((float)elapsed_us / HALL_TIMER_HZ);
    /* 角度回绕到0~2π。 */
    if (angle >= TWO_PI) angle -= TWO_PI;
    if (angle < 0.0f) angle += TWO_PI;
    hall_angle = angle;
}

/* 主循环每100ms打印一次统计数据。 */
static void Monitor_Print(void)
{
    static uint32_t last_ms = 0;
    char line[448];
    uint32_t now = HAL_GetTick(), primask, period, edges, pairs, dq_pairs;
    uint8_t request, enabled, calibrated, state, angle_valid, park_valid, pi_valid;
    int8_t direction;
    float iu, iv, u_min, u_max, v_min, v_max, electrical_hz, angle, id, iq, p_angle, ud, uq;
    int length;

    if ((uint32_t)(now - last_ms) < MONITOR_PERIOD_MS)
        return;
    last_ms = now;

    /* 关中断取快照、清统计；恢复中断后再打印。 */
    primask = __get_PRIMASK();
    __disable_irq();
    request = motor_request;
    enabled = (MOTOR_EN_GPIO_Port->ODR & MOTOR_EN_Pin) != 0U;
    calibrated = (adc_u_offset_ready != 0U) && (adc_v_offset_ready != 0U);
    state = hall_state;
    direction = hall_direction;
    period = hall_period_us;
    edges = hall_edge_count;
    electrical_hz = hall_electrical_hz;
    angle = hall_angle;
    angle_valid = hall_angle_valid;
    p_angle = park_angle;
    park_valid = park_angle_valid;
    pi_valid = current_pi_valid;
    ud = Ud;
    uq = Uq;
    dq_pairs = monitor_dq_samples;
    id = monitor_d_sum;
    iq = monitor_q_sum;
    pairs = monitor_samples;
    iu = monitor_u_sum;
    iv = monitor_v_sum;
    u_min = monitor_u_min;
    u_max = monitor_u_max;
    v_min = monitor_v_min;
    v_max = monitor_v_max;
    monitor_samples = 0;
    monitor_u_sum = 0.0f;
    monitor_v_sum = 0.0f;
    monitor_dq_samples = 0;
    monitor_d_sum = monitor_q_sum = 0.0f;
    __set_PRIMASK(primask);

    if (pairs != 0U)
    {
        iu /= (float)pairs;
        iv /= (float)pairs;
    }
    else
    {
        /* pairs=0表示没有有效采样，打印零只是占位。 */
        iu = iv = u_min = u_max = v_min = v_max = 0.0f;
    }

    if (dq_pairs != 0U)
    {
        id /= (float)dq_pairs;
        iq /= (float)dq_pairs;
    }
    else
    {
        id = iq = 0.0f; // 无有效dq采样时打印占位值
    }

    /* 电流用mA、角度用千分之一度；dq_pairs只统计角度有效的采样。 */
    length = snprintf(line, sizeof(line),
        "req=%u en=%u cal=%u hall=%u dir=%d dt_us=%lu edges=%lu fe_mHz=%ld "
        "Iu_avg_mA=%ld Iu_min=%ld Iu_max=%ld "
        "Iv_avg_mA=%ld Iv_min=%ld Iv_max=%ld pairs=%lu ha_mdeg=%ld ha_ok=%u "
        "pa_mdeg=%ld pa_ok=%u Id_avg_mA=%ld Iq_avg_mA=%ld dq_pairs=%lu "
        "Ud_mV=%ld Uq_mV=%ld pi_ok=%u\r\n",
        (unsigned)request, (unsigned)enabled, (unsigned)calibrated,
        (unsigned)state, (int)direction, (unsigned long)period,
        (unsigned long)edges, (long)(electrical_hz * 1000.0f),
        (long)(iu * 1000.0f), (long)(u_min * 1000.0f), (long)(u_max * 1000.0f),
        (long)(iv * 1000.0f), (long)(v_min * 1000.0f), (long)(v_max * 1000.0f),
        (unsigned long)pairs, (long)(angle * (360000.0f / TWO_PI)),
        (unsigned)angle_valid, (long)(p_angle * (360000.0f / TWO_PI)),
        (unsigned)park_valid, (long)(id * 1000.0f), (long)(iq * 1000.0f),
        (unsigned long)dq_pairs, (long)(ud * 1000.0f), (long)(uq * 1000.0f),
        (unsigned)pi_valid);

    if ((length > 0) && (length < (int)sizeof(line)))
    {
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)line,
                                (uint16_t)length, MONITOR_TX_TIMEOUT_MS);
    }
}

/* 编码器原始计数，每100ms打印一次。电机不上电，用手转轴观察。 */
static void Encoder_Print(void)
{
    static uint32_t last_ms = 0, last_count = 0;
    char line[96];
    uint32_t now = HAL_GetTick(), count;
    int32_t delta;
    int length;

    if ((uint32_t)(now - last_ms) < MONITOR_PERIOD_MS)
        return;
    last_ms = now;

    count = __HAL_TIM_GET_COUNTER(&htim2);
    delta = (int32_t)(count - last_count); // 正转或反转由符号区分
    last_count = count;

    length = snprintf(line, sizeof(line), "enc_cnt=%lu enc_delta=%ld\r\n",
                      (unsigned long)count, (long)delta);
    if ((length > 0) && (length < (int)sizeof(line)))
    {
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)line,
                                (uint16_t)length, MONITOR_TX_TIMEOUT_MS);
    }
}

/* USER CODE END 0 */

/* 初始化外设，主循环处理按键和打印。 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* 初始化HAL、系统时钟和外设。 */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  /* 上电关闭驱动。 */
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
  /* 启动霍尔捕获。 */
  if (HAL_TIMEx_HallSensor_Start_IT(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  /* 启动编码器计数，只观察，不参与控制。 */
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
  /* ADC等待CH4触发。 */
  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /* 启动三相PWM，驱动仍关闭。 */
  if ((HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) ||
      (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK) ||
      (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3) != HAL_OK))
  {
    Error_Handler();
  }
  /* CH4触发ADC和PWM回调，CCR4由CubeMX配置。 */
  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_CC4);
  if (HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint8_t key_armed = 0;

    /* 校准完成前禁止启动。 */
    if ((adc_u_offset_ready == 0) || (adc_v_offset_ready == 0))
    {
        motor_request = 0;
        key_armed = 0;
    }
    /* KEY1松开后再按下，切换启停。 */
    else if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET)
    {
        key_armed = 1;
    }
    else if ((HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) &&
             (key_armed == 1))
    {
        key_armed = 0;
        motor_request = !motor_request;
    }

    Monitor_Print();
    Encoder_Print();
    HAL_Delay(20); // 按键轮询间隔，ms
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
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* 正转5→1→3→2→6→4→5；表中保存各状态的下一状态。 */
    static const uint8_t next_state[8] = {0, 3, 6, 2, 5, 1, 4, 0};
    /* 状态5/1/3/2/6/4对应区间0/1/2/3/4/5；编号×60度为下边界，零点未对齐磁极。 */
    static const uint8_t sector_index[8] = {0, 1, 3, 2, 5, 0, 4, 0};
    static uint8_t previous_state = 0;    // 上一次霍尔状态
    static int8_t previous_direction = 0; // 上一次方向
    uint32_t captured_us;                // 刚走完一个区间的用时，us

    /* 只处理TIM5通道1的霍尔捕获。 */
    if ((htim->Instance != TIM5) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1))
    {
        return;
    }

    /* CCR1保存刚结束的区间用时；CNT已开始为下一区间计时。 */
    captured_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* 超时后重新捕获，先丢弃旧的方向和状态。 */
    if ((uint32_t)(HAL_GetTick() - hall_last_edge_ms) >= HALL_STOP_TIMEOUT_MS)
    {
        previous_state = 0;
        previous_direction = 0;
    }
    hall_last_edge_ms = HAL_GetTick();
    hall_angle_valid = 0;
    hall_state = (uint8_t)((GPIOH->IDR >> 10U) & 0x07U); // 取PH12/11/10三位，拼成H3H2H1状态
    hall_edge_count++;
    hall_direction = 0;
    hall_period_us = 0;
    hall_electrical_hz = 0.0f;

    if ((hall_state == 0U) || (hall_state == 7U))
    {
        previous_state = 0; // 0和7为无效霍尔状态
        previous_direction = 0;
        return;
    }

    /* 用前后状态判断方向；重复或跳区间时，方向保持0。 */
    if (previous_state != 0U)
    {
        if (next_state[previous_state] == hall_state)
            hall_direction = 1;
        else if (next_state[hall_state] == previous_state)
            hall_direction = -1;

        /* 相邻状态跳变才采用本次区间用时。 */
        if (hall_direction != 0)
            hall_period_us = captured_us;

        /* 连续两次方向一致、用时非零，才估算速度。 */
        if ((hall_direction != 0) &&
            (hall_direction == previous_direction) && (captured_us != 0U))
        {
            /* 六个区间是一圈电角度，方向决定频率正负。 */
            hall_electrical_hz = (float)hall_direction * HALL_TIMER_HZ /
                                (HALL_EDGES_PER_CYCLE * (float)captured_us);
        }
    }

    if (hall_direction != 0)
    {
        /* 新边沿校正角度起点。 */
        hall_edge_angle = (float)sector_index[hall_state] * HALL_SECTOR_RAD;
        /* 正转从区间下边界进入，反转从上边界进入。 */
        if (hall_direction < 0) hall_edge_angle += HALL_SECTOR_RAD;
        if (hall_edge_angle >= TWO_PI) hall_edge_angle -= TWO_PI;
        hall_angle = hall_edge_angle;
        hall_angle_valid = hall_electrical_hz != 0.0f; // 有速度依据才允许插值
    }
    previous_direction = hall_direction;
    previous_state = hall_state;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static uint32_t u_samples = 0, v_samples = 0;
    static uint32_t u_sum = 0, v_sum = 0;

    /* 驱动关闭时校准零点，完成后不再更新offset。 */
    if (hadc->Instance == ADC1)
    {
        adc_u_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        adc_u_count++;
        if (adc_u_offset_ready == 0)
        {
            u_samples++;
            if (u_samples > ADC_SETTLE_SAMPLES)
            {
                u_sum += adc_u_raw;
            }
            if (u_samples == ADC_SETTLE_SAMPLES + ADC_OFFSET_SAMPLES)
            {
                adc_u_offset = (float)u_sum / ADC_OFFSET_SAMPLES;
                adc_u_offset_ready = 1;
            }
        }
    }
    else if (hadc->Instance == ADC2)
    {
        adc_v_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        adc_v_count++;
        if (adc_v_offset_ready == 0)
        {
            v_samples++;
            if (v_samples > ADC_SETTLE_SAMPLES)
            {
                v_sum += adc_v_raw;
            }
            if (v_samples == ADC_SETTLE_SAMPLES + ADC_OFFSET_SAMPLES)
            {
                adc_v_offset = (float)v_sum / ADC_OFFSET_SAMPLES;
                adc_v_offset_ready = 1;
            }
        }
    }
    else
    {
        return;
    }

    /* 两路计数相等时计算，避免混用两轮数据；漏采或调试中断后需重启采样。 */
    if ((adc_u_offset_ready != 0) && (adc_v_offset_ready != 0) &&
        (adc_u_count == adc_v_count))
    {
        current_u = ADC_ToCurrent(adc_u_raw, adc_u_offset);
        current_v = ADC_ToCurrent(adc_v_raw, adc_v_offset);
        Clarke_Update(current_u, current_v);
        Hall_AngleUpdate();
        park_angle_valid = hall_angle_valid;
        if (park_angle_valid != 0U)
        {
            /* 霍尔相对角度加参考偏移，作为Park的旋转角度。 */
            park_angle = hall_angle + HALL_PHASE_SHIFT_RAD;
            if (park_angle >= TWO_PI) park_angle -= TWO_PI;
            if (park_angle < 0.0f) park_angle += TWO_PI;
            Park_Update(current_alpha, current_beta, park_angle);
            monitor_d_sum += current_d;
            monitor_q_sum += current_q;
            monitor_dq_samples++;
        } // 角度无效时保留上次dq值，不计入平均
        Current_PI_Update();
        if (current_pi_valid != 0U)
        {
            InversePark_Update(Ud, Uq, park_angle); // 只观察，不控制PWM
        }
        else
        {
            Ualpha_cl = Ubeta_cl = 0.0f;
        }

        /* 累加打印统计。 */
        if (monitor_samples == 0U)
        {
            monitor_u_min = monitor_u_max = current_u;
            monitor_v_min = monitor_v_max = current_v;
        }
        else
        {
            if (current_u < monitor_u_min) monitor_u_min = current_u;
            if (current_u > monitor_u_max) monitor_u_max = current_u;
            if (current_v < monitor_v_min) monitor_v_min = current_v;
            if (current_v > monitor_v_max) monitor_v_max = current_v;
        }
        monitor_u_sum += current_u;
        monitor_v_sum += current_v;
        monitor_samples++;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM8) &&
        (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4))
    {
        static uint8_t pwm_prepared = 0;

        Hall_AngleUpdate(); // 独立观察霍尔角度

        if ((motor_request == 0) ||
            (adc_u_offset_ready == 0) || (adc_v_offset_ready == 0))
        {
            /* 停止或未校准：关闭驱动。 */
            HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
            pwm_prepared = 0;
            theta = 0.0f;
            return;
        }

        if (pwm_prepared == 0)
        {
            /* 首次先计算CCR，保持驱动关闭。 */
            HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
            theta = 0.0f;
            pwm_prepared = 1;
        }
        else
        {
            /* CCR装载后使能驱动，并推进开环角度。 */
            HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
            theta += TWO_PI * frequency / PWM_UPDATE_HZ;
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

			/* 分配零矢量和有效矢量时间，得到三相占空比。 */
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

			/* 占空比乘ARR，写入三相CCR。 */
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,
					(uint32_t)(dutyA * __HAL_TIM_GET_AUTORELOAD(&htim8)));
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,
					(uint32_t)(dutyB * __HAL_TIM_GET_AUTORELOAD(&htim8)));
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,
					(uint32_t)(dutyC * __HAL_TIM_GET_AUTORELOAD(&htim8)));
    }
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
