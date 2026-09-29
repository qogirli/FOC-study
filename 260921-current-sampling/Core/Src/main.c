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
#define PWM_UPDATE_HZ          20000.0f // TIM8 CH4 每个 PWM 周期计算一次
#define HALL_TIMER_HZ          1000000.0f // TIM5 每计数一次为 1 us
#define HALL_EDGES_PER_CYCLE   6.0f       // 一圈电角度有 6 个霍尔区间
#define HALL_SECTOR_RAD        (TWO_PI / HALL_EDGES_PER_CYCLE)
#define HALL_STOP_TIMEOUT_MS   1000U     // 学习阶段：1秒没有边沿，测速失效
#define MONITOR_PERIOD_MS      100U    // 串口每秒打印 10 行
#define MONITOR_TX_TIMEOUT_MS  5U      // 打印超时就放弃本行，不影响启停逻辑
#define ADC_SETTLE_SAMPLES     2000U // 20 kHz 下先丢弃约 100 ms 数据
#define ADC_OFFSET_SAMPLES     256U  // 每相零电流偏置的平均次数
#define ADC_COUNTS_PER_RANGE   4096.0f
#define ADC_VREF_V             3.3f   // 标称值，后续按 MCU 实际 VREF+ 校正
#define CURRENT_SHUNT_OHM      0.02f  // 2024版原理图 R12/R18/R24
#define CURRENT_AMP_GAIN       (5100.0f / (1200.0f + 68.0f))

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
volatile uint16_t adc_u_raw = 0;
volatile uint16_t adc_v_raw = 0;
volatile uint32_t adc_u_count = 0;
volatile uint32_t adc_v_count = 0;
volatile float adc_u_offset = 0.0f; // 单位为 ADC 码值，还不是安培
volatile float adc_v_offset = 0.0f;
volatile uint8_t adc_u_offset_ready = 0; // 仅表示平均完成，不代表硬件校验通过
volatile uint8_t adc_v_offset_ready = 0;
volatile float current_u = 0.0f; // A；正号表示采样电压高于零点，暂不用于闭环
volatile float current_v = 0.0f; // 仅在对应 offset_ready=1 后有效
volatile float current_w = 0.0f; // 由 U、V 重建，不是第三路实测值
volatile float current_alpha = 0.0f;
volatile float current_beta = 0.0f;
volatile float current_d = 0.0f; // Park 输出，单位 A；本阶段仅观察
volatile float current_q = 0.0f;
volatile float park_angle = 0.0f; // 测试坐标角度，单位 rad；尚未接入转子电角度
volatile uint32_t clarke_count = 0; // 每完成一对电流的 Clarke 计算加 1
volatile uint8_t hall_state = 0;       // H3 H2 H1：PH12 PH11 PH10
volatile int8_t hall_direction = 0;    // 最近一次边沿：+1 正向，-1 反向，0 未确定
volatile uint32_t hall_period_us = 0; // 两次捕获的间隔；TIM5 每计数一次为 1 us
volatile uint32_t hall_edge_count = 0;
volatile float hall_electrical_hz = 0.0f; // 有符号电频率；超时清零
volatile float hall_angle = 0.0f;       // 霍尔相对电角度，rad；零点未对齐磁极
volatile uint8_t hall_angle_valid = 0;  // 仅表示插值有依据，不代表可直接用于FOC
static volatile float hall_edge_angle = 0.0f;
static volatile uint32_t hall_last_edge_ms = 0;
/* 仅供串口观察：每次打印取走并清空一个统计窗口。 */
static volatile uint32_t monitor_samples = 0;
static volatile float monitor_u_sum = 0.0f, monitor_v_sum = 0.0f;
static volatile float monitor_u_min = 0.0f, monitor_u_max = 0.0f;
static volatile float monitor_v_min = 0.0f, monitor_v_max = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float ADC_ToCurrent(uint16_t raw, float offset)
{
    float voltage_delta;

    voltage_delta = ((float)raw - offset) * ADC_VREF_V / ADC_COUNTS_PER_RANGE;
    return voltage_delta / CURRENT_AMP_GAIN / CURRENT_SHUNT_OHM;
}

/* 等幅值 Clarke，alpha 轴与 U 相重合；本阶段仅观察，不用于闭环。
 * 沿用 ADC_ToCurrent 的正号，实际电机相电流方向仍需后续验证。
 */
static void Clarke_Update(float iu, float iv)
{
    current_w = -iu - iv;
    current_alpha = iu;
    current_beta = (iu + 2.0f * iv) / SQRT_3;
    clarke_count++;
}

/* 将静止 alpha/beta 坐标投影到角度为 angle 的 d/q 坐标。
 * 用于真实 FOC 时，angle 必须换成与采样时刻对应的转子电角度。
 */
static void Park_Update(float alpha, float beta, float angle)
{
    float sin_angle = sinf(angle);
    float cos_angle = cosf(angle);

    current_d =  alpha * cos_angle + beta * sin_angle;
    current_q = -alpha * sin_angle + beta * cos_angle;
}

/* 仅观察：TIM5在霍尔边沿自动清CNT，CNT就是距最近边沿的微秒数。
 * TIM5和TIM8当前同为抢占优先级0，不会互相打断；不要单独改变其优先级。
 */
static void Hall_AngleUpdate(void)
{
    uint32_t elapsed_us;
    float angle;

    elapsed_us = __HAL_TIM_GET_COUNTER(&htim5);
    /* 硬件已捕获但霍尔回调尚未处理时，不能混用旧边沿角度和新CNT。 */
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_CC1) != RESET)
        return;
    if ((uint32_t)(HAL_GetTick() - hall_last_edge_ms) >= HALL_STOP_TIMEOUT_MS)
    {
        hall_electrical_hz = 0.0f;
        hall_direction = 0;
        hall_period_us = 0;
        hall_angle_valid = 0;
        return; // 保留最后角度，不再按旧速度转下去
    }
    if ((hall_period_us == 0U) || (hall_electrical_hz == 0.0f))
    {
        hall_angle_valid = 0;
        return;
    }

    hall_angle_valid = elapsed_us < hall_period_us;
    /* 新边沿迟迟不来时，最多推算一个60度区间，不凭空跨过多个区间。 */
    if (elapsed_us > hall_period_us)
        elapsed_us = hall_period_us;
    angle = hall_edge_angle + TWO_PI * hall_electrical_hz *
                              ((float)elapsed_us / HALL_TIMER_HZ);
    if (angle >= TWO_PI) angle -= TWO_PI;
    if (angle < 0.0f) angle += TWO_PI;
    hall_angle = angle;
}

/* 只在主循环调用。沿用 USART1 的 921600 波特率，不使用中断内 printf。 */
static void Monitor_Print(void)
{
    static uint32_t last_ms = 0;
    char line[320];
    uint32_t now = HAL_GetTick(), primask, period, edges, pairs;
    uint8_t request, enabled, calibrated, state, angle_valid;
    int8_t direction;
    float iu, iv, u_min, u_max, v_min, v_max, electrical_hz, angle;
    int length;

    if ((uint32_t)(now - last_ms) < MONITOR_PERIOD_MS)
        return;
    last_ms = now;

    /* 仅复制数据时短暂屏蔽中断，避免同一行混入更新前后的值。
     * 格式化和串口发送时中断正常运行。
     */
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
    __set_PRIMASK(primask);

    if (pairs != 0U)
    {
        iu /= (float)pairs;
        iv /= (float)pairs;
    }
    else
    {
        /* 未校准或没有新采样时，pairs=0；零值不表示实测电流为零。 */
        iu = iv = u_min = u_max = v_min = v_max = 0.0f;
    }

    /* avg/min/max 均为窗口统计值，单位 mA；pairs 为参与统计的电流对数。
     * ha_mdeg为相对电角度的千分之一度；ha_ok=0时不应相信角度插值。
     */
    length = snprintf(line, sizeof(line),
        "req=%u en=%u cal=%u hall=%u dir=%d dt_us=%lu edges=%lu fe_mHz=%ld "
        "Iu_avg_mA=%ld Iu_min=%ld Iu_max=%ld "
        "Iv_avg_mA=%ld Iv_min=%ld Iv_max=%ld pairs=%lu ha_mdeg=%ld ha_ok=%u\r\n",
        (unsigned)request, (unsigned)enabled, (unsigned)calibrated,
        (unsigned)state, (int)direction, (unsigned long)period,
        (unsigned long)edges, (long)(electrical_hz * 1000.0f),
        (long)(iu * 1000.0f), (long)(u_min * 1000.0f), (long)(u_max * 1000.0f),
        (long)(iv * 1000.0f), (long)(v_min * 1000.0f), (long)(v_max * 1000.0f),
        (unsigned long)pairs, (long)(angle * (360000.0f / TWO_PI)),
        (unsigned)angle_valid);

    if ((length > 0) && (length < (int)sizeof(line)))
    {
        /* 主循环短时轮询发送；PWM/ADC/霍尔中断仍可抢占。超时不重试。 */
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)line,
                                (uint16_t)length, MONITOR_TX_TIMEOUT_MS);
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
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  /* 默认关闭功率级，等SVPWM参数准备完成后再启动。 */
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
  /* 霍尔只观察，不改变 park_angle，不参与开环输出。 */
  if (HAL_TIMEx_HallSensor_Start_IT(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  /* 先让 ADC 等待 CH4 触发；校准完成前 MOTOR_EN 始终为低。 */
  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /* 启动三相定时器输出，但驱动仍关闭；按键后才允许电机通电。 */
  if ((HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) ||
      (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK) ||
      (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3) != HAL_OK))
  {
    Error_Handler();
  }
  /* CCR4 沿用 CubeMX 的 4198：硬件触发 ADC，同时进入 PWM 回调。 */
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

    /* 校准期间不接受启动，也不记住提前按下的按键。 */
    if ((adc_u_offset_ready == 0) || (adc_v_offset_ready == 0))
    {
        motor_request = 0;
        key_armed = 0;
    }
    /* KEY1：PE0。校准后先松开，再按下，才切换启停。 */
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
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* 教程正向顺序：5 -> 1 -> 3 -> 2 -> 6 -> 4 -> 5。
     * 下标是当前霍尔状态，表内值是正向的下一个状态。
     */
    static const uint8_t next_state[8] = {0, 3, 6, 2, 5, 1, 4, 0};
    /* 学习坐标：状态5占0~60度，1占60~120度，依次类推；不是磁极零点。 */
    static const uint8_t sector_index[8] = {0, 1, 3, 2, 5, 0, 4, 0};
    static uint8_t previous_state = 0;
    static int8_t previous_direction = 0;
    uint32_t captured_us;

    if ((htim->Instance != TIM5) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1))
    {
        return;
    }

    captured_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    if ((uint32_t)(HAL_GetTick() - hall_last_edge_ms) >= HALL_STOP_TIMEOUT_MS)
    {
        previous_state = 0;
        previous_direction = 0;
    }
    hall_last_edge_ms = HAL_GetTick();
    hall_angle_valid = 0;
    hall_state = (uint8_t)((GPIOH->IDR >> 10U) & 0x07U);
    hall_edge_count++;
    hall_direction = 0;
    hall_period_us = 0;
    hall_electrical_hz = 0.0f;

    if ((hall_state == 0U) || (hall_state == 7U))
    {
        previous_state = 0; // 120度霍尔的无效状态，不参与方向判断
        previous_direction = 0;
        return;
    }

    if (previous_state != 0U)
    {
        if (next_state[previous_state] == hall_state)
            hall_direction = 1;
        else if (next_state[hall_state] == previous_state)
            hall_direction = -1;

        /* 复位模式下 CCR1 已是间隔，不再减上一次 CCR1。
         * 第一次捕获只记录状态；重复或跳变状态不作为有效间隔。
         */
        if (hall_direction != 0)
            hall_period_us = captured_us;

        /* 连续两次方向一致，才用刚走完的 60 度计算速度。
         * 启动、换向和异常跳变后的首个区间不用于测速；排除除零。
         */
        if ((hall_direction != 0) &&
            (hall_direction == previous_direction) && (captured_us != 0U))
        {
            hall_electrical_hz = (float)hall_direction * HALL_TIMER_HZ /
                                (HALL_EDGES_PER_CYCLE * (float)captured_us);
        }
    }

    if (hall_direction != 0)
    {
        hall_edge_angle = (float)sector_index[hall_state] * HALL_SECTOR_RAD;
        /* 反转从区间上边界进入，不能照搬正转的下边界角度。 */
        if (hall_direction < 0) hall_edge_angle += HALL_SECTOR_RAD;
        if (hall_edge_angle >= TWO_PI) hall_edge_angle -= TWO_PI;
        hall_angle = hall_edge_angle;
        hall_angle_valid = hall_electrical_hz != 0.0f;
    }
    previous_direction = hall_direction;
    previous_state = hall_state;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static uint32_t u_samples = 0, v_samples = 0;
    static uint32_t u_sum = 0, v_sum = 0;

    /* 上电先关闭驱动，电机无电流、采样电路供电稳定时建立零点。
     * 校准完成后冻结 offset；电机运行时继续采样，不重新平均零点。
     */
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

    /* 两路同由 TIM8 CH4 触发：正常无漏采时，计数相等表示本轮都已更新。
     * 不在第一路刚更新时计算，避免混用上一轮的另一路数据。
     * 这是学习阶段配对；长时间停机调试/漏采后须重启采样，不能据此检测所有漏采。
     */
    if ((adc_u_offset_ready != 0) && (adc_v_offset_ready != 0) &&
        (adc_u_count == adc_v_count))
    {
        current_u = ADC_ToCurrent(adc_u_raw, adc_u_offset);
        current_v = ADC_ToCurrent(adc_v_raw, adc_v_offset);
        Clarke_Update(current_u, current_v);
        /* 顺序执行两个变换；测试角度为 0 时，d=alpha、q=beta。 */
        Park_Update(current_alpha, current_beta, park_angle);

        /* 只累加监控统计；Clarke/Park 仍使用本次瞬时电流。 */
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

        Hall_AngleUpdate(); // 即使驱动关闭也观察霍尔，不修改theta或park_angle

        if ((motor_request == 0) ||
            (adc_u_offset_ready == 0) || (adc_v_offset_ready == 0))
        {
            /* 未校准或收到停止请求：关闭驱动，ADC 和霍尔继续运行。 */
            HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
            pwm_prepared = 0;
            theta = 0.0f;
            return;
        }

        if (pwm_prepared == 0)
        {
            /* 第一次先保持关闭，用零角度计算下面的三路 CCR。 */
            HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
            theta = 0.0f;
            pwm_prepared = 1;
        }
        else
        {
            /* 下一次 CH4 中断，上一组 CCR 已经过更新事件装载。 */
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
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
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
