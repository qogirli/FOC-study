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
/* ========================= 阅读地图（从上往下读这个文件）=========================
 * CubeMX 生成的框架（文件头、main 里的 Init、SystemClock_Config、Error_Handler）可以跳过，
 * 只读 USER CODE 之间的内容。用 Ctrl+F 搜函数名。
 *
 * 【主线】面试要能讲清楚的：
 *   1. 变量和设定            搜 PV：电流给定、母线电压、运行状态
 *   2. SVPWM_Update          电压矢量 (Ualpha, Ubeta) → 三相占空比 → 三个 CCR
 *   3. Encoder_ElectricalAngle  编码器计数 → 转子电角度
 *   4. ADC_ToCurrent、Offset_Calibrate   ADC 读数 → 电流，上电零偏校准
 *   5. Clarke_Update、Park_Update        三相电流 → alpha、beta → d、q
 *   6. Current_PI_Update、InversePark_Update   电流环：PI 算 Ud、Uq，逆 Park 变回 alpha、beta
 *   7. HAL_ADCEx_InjectedConvCpltCallback  每个 PWM 周期采一次电流 → Clarke → Park → PI
 *   8. HAL_TIM_PWM_PulseFinishedCallback   每个 PWM 周期：逆 Park → SVPWM → 写 CCR
 *   9. Encoder_Align         对零（找编码器零点和转子磁极的偏差）
 *
 * 【支线】可以不细读，面试很少问：
 *   - Print_Currents        串口打印，调试用
 *   - key_sw 和主循环里的按键判断   启停，调试用
 * ============================================================================== */
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

/* 电流采样：ADC 读到的是电压，要换算成电流。 */
#define ADC_VREF_V             3.3f       // ADC 参考电压，V
#define ADC_COUNTS             4096.0f    // 12 位 ADC，满量程 4096
#define SHUNT_OHM              0.02f      // 采样电阻，欧姆
#define AMP_GAIN               (5100.0f / (1200.0f + 68.0f)) // 运放放大倍数
#define CURRENT_SIGN           (-1.0f)    // 实测方向与预期相反，所以取反
#define CAL_SKIP_SAMPLES       2000U      // 上电后先丢掉前 2000 个（0.1 s），等电路稳定
#define CAL_AVG_SAMPLES        256U       // 再平均 256 个，得到零电流时的读数
#define PRINT_PERIOD_MS        200U       // 串口打印间隔

/* 电流环（PI 控制器）。每个 PWM 周期算一次。 */
#define PWM_HZ                 20000.0f
#define PI_DT                  (1.0f / PWM_HZ) // 两次计算间隔，s
#define CURRENT_PI_KP          0.5f       // 比例系数，V/A
#define CURRENT_PI_KI          250.0f     // 积分系数，V/(A·s)
#define CURRENT_PI_LIMIT_V     1.0f       // d、q 轴电压各自的上限，V（限幅，保护电机）

/* 编码器和电机：电角度 = 机械角度 × 极对数。 */
#define ENC_COUNTS_PER_REV     4000       // 1000 线 × 4 倍频，机械转一圈的计数
#define POLE_PAIRS             4          // 电机极对数
#define ENC_COUNTS_PER_ELEC    (ENC_COUNTS_PER_REV / POLE_PAIRS) // 电角度转一圈的计数（1000）

/* 对零：电压矢量固定在 0 度，转子会被吸过来，此时转子 d 轴 = 0 度。 */
#define ALIGN_VOLTAGE_V        0.6f       // 对零电压，V
#define ALIGN_KICK_MS          600U       // 先向两侧各推 90 度的时间
#define ALIGN_SETTLE_MS        800U       // 回到 0 度后等转子停稳的时间
#define ALIGN_MAX_SPREAD       120        // 两次停点相差超过这个计数就认为对零失败
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 设定：你改这几个值就能改变转动（Keil 里加进 Watch 窗口直接改）。 */
float Id_ref    = 0.0f;        // d 轴电流给定，A（0 = 不沿磁极方向用力）
float Iq_ref    = 0.6f;        // q 轴电流给定，A（决定力矩，正数正转）
float Udc       = 12.0f;       // 母线电压，V（要和实际供电一致，现在用 12 V）

/* 运行状态：按键切换。 */
#define MODE_OFF               0U // 驱动关闭
#define MODE_ALIGN             1U // 对零：电压矢量固定在 align_theta
#define MODE_RUN               2U // 运转：电流环控制 Id、Iq
volatile uint8_t drive_mode = MODE_OFF;
volatile uint8_t key_sw = 0;   // 调试时在 Watch 窗口里写 1，等于按一下 KEY1

volatile float align_theta = 0.0f;  // 对零时电压矢量的固定方向，rad
uint32_t enc_zero = 0;              // 转子 d 轴对应的编码器计数
volatile uint8_t enc_zero_valid = 0;// 对零成功标志

volatile float theta = 0.0f;   // 当前用的电角度，rad（对零时是矢量方向，运转时是转子电角度）
volatile float Ualpha = 0.0f;  // 电压矢量在 alpha 轴的分量，V
volatile float Ubeta  = 0.0f;  // 电压矢量在 beta 轴的分量，V

/* SVPWM 的中间结果，放全局只为了调试时能看。 */
volatile uint8_t sector;       // 电压矢量落在第几个扇区（1~6）
volatile float Tx, Ty, T0;     // 两个相邻有效矢量和零矢量占一个 PWM 周期的比例
volatile float dutyA, dutyB, dutyC; // 三相上桥臂占空比，0~1


/* 电流采样结果。 */
volatile uint16_t raw_u = 0, raw_v = 0;   // ADC 原始读数（0~4095）
float offset_u = 0.0f, offset_v = 0.0f;   // 零电流时的读数
volatile uint8_t offset_ready = 0;        // 零偏校准完成标志
volatile float current_u = 0.0f;          // U 相电流，A
volatile float current_v = 0.0f;          // V 相电流，A
volatile float current_w = 0.0f;          // W 相电流，A（没有测，由 U、V 算出）
volatile float current_alpha = 0.0f;      // Clarke 变换后：静止两轴电流，A
volatile float current_beta  = 0.0f;
volatile float current_d = 0.0f;          // Park 变换后：旋转两轴电流，A
volatile float current_q = 0.0f;

/* 电流环输出。 */
volatile float Ud = 0.0f;                 // d 轴电压指令，V
volatile float Uq = 0.0f;                 // q 轴电压指令，V
static float integral_d = 0.0f;           // PI 的积分项，V
static float integral_q = 0.0f;

/* 打印用的累加：每个打印周期取平均。 */
static float sum_d = 0.0f, sum_q = 0.0f, sum_ud = 0.0f, sum_uq = 0.0f;
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
/* 转子电角度：编码器计数减去 d 轴零点，对 1000 取余（电角度转一圈 1000 个计数），换成弧度。 */
static float Encoder_ElectricalAngle(void)
{
    int32_t rel = (int32_t)(__HAL_TIM_GET_COUNTER(&htim2) - enc_zero);

    rel %= ENC_COUNTS_PER_ELEC;
    if (rel < 0) rel += ENC_COUNTS_PER_ELEC;
    return TWO_PI * (float)rel / (float)ENC_COUNTS_PER_ELEC;
}

/* 对零（主循环里调用，会等几秒）：
 * 电压矢量固定在 0 度，转子被吸到 d 轴对齐的位置，记下此时的计数。
 * 只推一次，静摩擦大时转子可能停在偏一点的位置，甚至 180 度反着停。
 * 所以先把矢量转到 +90 度再回 0 度，记一次停点；再转到 -90 度回 0 度，再记一次，
 * 两个停点一边偏多、一边偏少，取平均就是 d 轴零点。 */
static void Encoder_Align(void)
{
    uint32_t stop[2];
    int32_t spread;
    int i;

    enc_zero_valid = 0;
    drive_mode = MODE_ALIGN;
    for (i = 0; i < 2; i++)
    {
        align_theta = (i == 0) ? (TWO_PI / 4.0f) : (-TWO_PI / 4.0f);
        HAL_Delay(ALIGN_KICK_MS);
        align_theta = 0.0f;
        HAL_Delay(ALIGN_SETTLE_MS);
        stop[i] = __HAL_TIM_GET_COUNTER(&htim2);
    }

    spread = (int32_t)(stop[1] - stop[0]);
    if ((spread > ALIGN_MAX_SPREAD) || (spread < -ALIGN_MAX_SPREAD))
    {
        drive_mode = MODE_OFF; // 两次停点差太多，说明没对好，不运转
        return;
    }
    enc_zero = stop[0] + (uint32_t)(spread / 2);
    enc_zero_valid = 1;
}

/* ADC 读数减零偏，再换算成电流：读数 → 电压 → 除以放大倍数 → 除以采样电阻。 */
static float ADC_ToCurrent(uint16_t raw, float offset)
{
    float volts = ((float)raw - offset) * ADC_VREF_V / ADC_COUNTS;
    return CURRENT_SIGN * volts / AMP_GAIN / SHUNT_OHM;
}

/* Clarke：三相电流（只用 U、V，W 由 U+V+W=0 得到）变成静止两轴 alpha、beta。
 * alpha 轴和 U 相重合。 */
static void Clarke_Update(float iu, float iv)
{
    current_alpha = iu;
    current_beta  = (iu + 2.0f * iv) / SQRT_3;
}

/* Park：静止两轴变成跟着转子转的两轴 d、q。angle 是 d 轴当前的方向。
 * 这里用的是转子电角度（编码器给出，对零后 0 度就是 d 轴方向）。 */
static void Park_Update(float alpha, float beta, float angle)
{
    float sin_angle = sinf(angle);
    float cos_angle = cosf(angle);

    current_d =  alpha * cos_angle + beta * sin_angle;
    current_q = -alpha * sin_angle + beta * cos_angle;
}

/* 把 value 限制在 ±limit 之内。 */
static float Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/* 电流环：Id、Iq 各一个 PI。输入给定和测量的差，输出要加的电压 Ud、Uq。
 * 输出 = Kp × 误差 + 积分项；积分项每次加 Ki × 误差 × 时间，并且限幅，防止积分越积越大。
 * 不在运转状态时清零，下次启动从零开始。 */
static void Current_PI_Update(void)
{
    float error_d, error_q;

    if (drive_mode != MODE_RUN)
    {
        integral_d = integral_q = 0.0f;
        Ud = Uq = 0.0f;
        return;
    }

    error_d = Id_ref - current_d;
    error_q = Iq_ref - current_q;

    integral_d = Clamp(integral_d + CURRENT_PI_KI * error_d * PI_DT, CURRENT_PI_LIMIT_V);
    integral_q = Clamp(integral_q + CURRENT_PI_KI * error_q * PI_DT, CURRENT_PI_LIMIT_V);

    Ud = Clamp(CURRENT_PI_KP * error_d + integral_d, CURRENT_PI_LIMIT_V);
    Uq = Clamp(CURRENT_PI_KP * error_q + integral_q, CURRENT_PI_LIMIT_V);
}

/* 逆 Park：d、q 轴电压（跟着转子转）变回静止的 alpha、beta 电压。
 * 和 Park_Update 互为逆运算，angle 必须用同一个转子电角度。 */
static void InversePark_Update(float ud, float uq, float angle)
{
    float sin_angle = sinf(angle);
    float cos_angle = cosf(angle);

    Ualpha = ud * cos_angle - uq * sin_angle;
    Ubeta  = ud * sin_angle + uq * cos_angle;
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

/* 每秒 5 次从串口打印：状态、编码器计数、转子电角度、窗口内平均的 Id、Iq、Ud、Uq。 */
static void Print_Currents(void)
{
    static uint32_t last_ms = 0;
    char line[160];
    float id, iq, ud, uq;
    uint32_t n;
    int len;

    if ((uint32_t)(HAL_GetTick() - last_ms) < PRINT_PERIOD_MS) return;
    last_ms = HAL_GetTick();

    /* 暂停中断，取出累加值并清零。 */
    __disable_irq();
    n = sum_n;
    id = sum_d;
    iq = sum_q;
    ud = sum_ud;
    uq = sum_uq;
    sum_n = 0;
    sum_d = sum_q = sum_ud = sum_uq = 0.0f;
    __enable_irq();

    if (n != 0U)
    {
        id /= (float)n;
        iq /= (float)n;
        ud /= (float)n;
        uq /= (float)n;
    }
    len = snprintf(line, sizeof(line),
        "mode=%u zero_ok=%u cnt=%lu ang_deg=%ld Id_mA=%ld Iq_mA=%ld Ud_mV=%ld Uq_mV=%ld\r\n",
        (unsigned)drive_mode, (unsigned)enc_zero_valid,
        (unsigned long)__HAL_TIM_GET_COUNTER(&htim2),
        (long)(Encoder_ElectricalAngle() * 360.0f / TWO_PI),
        (long)(id * 1000.0f), (long)(iq * 1000.0f),
        (long)(ud * 1000.0f), (long)(uq * 1000.0f));
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* 上电先关驱动（MOTOR_EN 低电平）。校准零偏时电机必须没有电流。 */
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);

  /* 编码器开始计数（A、B 相接 PA15、PB3，TIM2 硬件自动数）。 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

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
    /* KEY1：松开后再按下算一次按键。停止时按：先对零再运转；运转时按：停止。 */
    static uint8_t key_armed = 0;
    uint8_t pressed = 0;

    if (offset_ready == 0)
    {
        /* 零偏没校准完，不允许启动。 */
        drive_mode = MODE_OFF;
        key_armed = 0;
        key_sw = 0;
    }
    else
    {
        if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET)
        {
            key_armed = 1;
        }
        else if (key_armed == 1)
        {
            key_armed = 0;
            pressed = 1;
        }
        if (key_sw != 0U)
        {
            key_sw = 0;
            pressed = 1;
        }

        if (pressed != 0U)
        {
            if (drive_mode == MODE_OFF)
            {
                Encoder_Align();
                if (enc_zero_valid != 0U) drive_mode = MODE_RUN;
            }
            else
            {
                drive_mode = MODE_OFF;
            }
        }
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
    uint8_t mode = drive_mode;

    if ((htim->Instance != TIM8) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_4))
    {
        return;
    }

    if (mode == MODE_OFF)
    {
        /* 停止：关驱动。 */
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
        pwm_prepared = 0;
        return;
    }

    if (mode == MODE_ALIGN)
    {
        /* 对零：矢量固定在 align_theta，电压小一点。 */
        theta = align_theta;
        Ualpha = ALIGN_VOLTAGE_V * cosf(theta);
        Ubeta  = ALIGN_VOLTAGE_V * sinf(theta);
    }
    else
    {
        /* 运转：电流环给出 Ud、Uq，用转子电角度逆 Park 变成 alpha、beta 电压。 */
        theta = Encoder_ElectricalAngle();
        InversePark_Update(Ud, Uq, theta);
    }

    if (pwm_prepared == 0)
    {
        /* 第一个周期先算好 CCR，驱动仍然关着，避免一上来输出乱波形。 */
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
        pwm_prepared = 1;
    }
    else
    {
        /* CCR 已经装载，开驱动。 */
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
    }

    /* 电压矢量 (Ualpha, Ubeta) 交给 SVPWM，算出三相占空比写进 CCR。 */
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

    Clarke_Update(current_u, current_v);
    Park_Update(current_alpha, current_beta, Encoder_ElectricalAngle());

    Current_PI_Update();

    sum_d += current_d;
    sum_q += current_q;
    sum_ud += Ud;
    sum_uq += Uq;
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
