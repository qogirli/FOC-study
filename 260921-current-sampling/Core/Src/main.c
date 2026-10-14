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
#include <string.h>
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
/* 编码器对零和电角度。1000线4倍频=4000计数/圈；极对数4由日志算得（一个电周期1000计数）。 */
#define ENCODER_COUNTS_PER_REV  4000
#define MOTOR_POLE_PAIRS        4
#define ENCODER_COUNTS_PER_ELEC (ENCODER_COUNTS_PER_REV / MOTOR_POLE_PAIRS)
#define HALF_PI                 1.5707963f
#define AUX_NONE                0U // 辅助模式：无
#define AUX_ALIGN               1U // 对零：电压矢量固定在0度
#define AUX_ENC_SPIN            2U // 编码器换相：电压矢量超前转子90度
#define ALIGN_VOLTAGE_V         1.0f  // 对零电压，V；与开环相同，不提高
#define ALIGN_HOLD_MS           2000U // 对零保持时间，ms
#define ALIGN_CHECK_BEFORE_MS   300U  // 结束前这么久读一次计数，判断转子已静止
#define ALIGN_MAX_DRIFT_COUNTS  5     // 静止判据：两次计数相差不超过这个值
#define ENC_SPIN_VOLTAGE_V      1.0f  // 编码器换相电压（只有Uq），V
#define ENC_SPIN_MAX_MS         5000U // 编码器换相最长时间，ms
#define ENC_OVERSPEED_COUNTS    12    // 每个PWM周期（50us）计数变化超过这个值就停，约3600rpm
#define AUX_CURRENT             3U // 电流环：Id、Iq跟踪给定，用编码器角度
#define CURRENT_SIGN            (-1.0f) // 实测：电压矢量0度时U相读数为负，所以取反
#define CURRENT_REF_STEP_A      0.1f // 串口命令每次改变的电流给定，A
#define ID_REF_MAX_A            1.0f // d轴电流给定上限，A
#define IQ_REF_MAX_A            1.0f // q轴电流给定上限，A；对零用过1.95A，仍在1V电压限幅内；0.6A克服不了静摩擦
#define AUX_SPEED               4U // 速度环：编码器测速，PI输出Iq给定
#define AUX_POSITION            5U // 位置环：位置误差给速度环，速度环再给Iq
#define AUX_USES_CURRENT_LOOP() ((aux_mode == AUX_CURRENT) || (aux_mode == AUX_SPEED) || (aux_mode == AUX_POSITION))
#define SPEED_LOOP_DIV          20U // 每20个PWM周期（1ms）算一次测速和速度环
#define SPEED_WINDOW_STEPS      5U // 测速窗口：5次1ms，即5ms，分辨率3rpm
#define SPEED_RPM_PER_COUNT     (60.0f * 1000.0f / ((float)SPEED_WINDOW_STEPS * (float)ENCODER_COUNTS_PER_REV)) // 窗口内每个计数对应的rpm
#define SPEED_PI_DT             0.001f // 速度环周期，s
#define SPEED_PI_KP             0.003f // A/rpm；起点值，上板后调
#define SPEED_PI_KI             0.1f   // A/(rpm*s)；积分要能克服约0.45A的静摩擦
#define SPEED_REF_STEP_RPM      20.0f  // 串口命令每次改变的速度给定，rpm
#define SPEED_REF_MAX_RPM       120.0f // 速度给定上限，rpm；1V时空载约150rpm
#define POSITION_STEP_COUNTS    1000   // 串口命令每次改变的位置给定，计数，约90度机械角
#define POSITION_KP             0.3f   // rpm/计数
#define POSITION_SPEED_MAX_RPM  60.0f  // 位置环输出的速度上限，rpm
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
#define CURRENT_PI_KP          0.5f // V/A；起点按每相0.5欧、电感约1mH估，尚未实测调参
#define CURRENT_PI_KI          250.0f // V/(A*s)；PI零点约500rad/s
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
static volatile float monitor_d_sq = 0.0f, monitor_q_sq = 0.0f; // dq电流平方和，算窗口标准差
/* 编码器零点和辅助模式：0无，1对零，2编码器换相。 */
static volatile uint32_t enc_zero = 0;
static volatile uint8_t enc_zero_valid = 0;
static volatile uint8_t aux_mode = AUX_NONE;
static volatile uint8_t drive_fault = 0; // 1=编码器换相超速，已停机
/* 速度环、位置环：编码器测速，PI输出Iq给定。 */
static volatile float speed_rpm = 0.0f;     // 编码器测速（10ms窗口），rpm
static volatile float speed_ref_rpm = 0.0f; // 速度给定，rpm
static volatile uint32_t position_ref = 0;  // 位置给定，编码器计数
static volatile int32_t position_error = 0; // 位置误差，计数
static float speed_integral_a = 0.0f;       // 速度环积分，A
static volatile float speed_kp = SPEED_PI_KP, speed_ki = SPEED_PI_KI, position_kp = POSITION_KP; // 串口可调
static uint8_t speed_started = 0;           // 测速历史是否已建立
static volatile uint32_t speed_samples = 0; // 打印窗口内的测速次数
static volatile float speed_sum = 0.0f, speed_sq = 0.0f, speed_min = 0.0f, speed_max = 0.0f;
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
    return CURRENT_SIGN * voltage_delta / CURRENT_AMP_GAIN / CURRENT_SHUNT_OHM;
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
    float error_d, error_q, vector_square, vector_scale;

    current_pi_valid = ((motor_request != 0U) || AUX_USES_CURRENT_LOOP()) && (park_angle_valid != 0U);
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
    /* 合成电压矢量也限制在1V内，保持比例。 */
    vector_square = Ud * Ud + Uq * Uq;
    if (vector_square > (CURRENT_PI_LIMIT_V * CURRENT_PI_LIMIT_V))
    {
        vector_scale = CURRENT_PI_LIMIT_V / sqrtf(vector_square);
        Ud *= vector_scale;
        Uq *= vector_scale;
    }
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
    float iu, iv, u_min, u_max, v_min, v_max, electrical_hz, angle, id, iq, p_angle, ud, uq, id_sq, iq_sq, id_sd, iq_sd;
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
    id_sq = monitor_d_sq;
    iq_sq = monitor_q_sq;
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
    monitor_d_sq = monitor_q_sq = 0.0f;
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
        id_sq /= (float)dq_pairs;
        iq_sq /= (float)dq_pairs;
        id_sd = sqrtf(fmaxf(id_sq - id * id, 0.0f)); // 窗口内标准差，含测量噪声
        iq_sd = sqrtf(fmaxf(iq_sq - iq * iq, 0.0f));
    }
    else
    {
        id = iq = id_sd = iq_sd = 0.0f; // 无有效dq采样时打印占位值
    }

    /* 电流用mA、角度用千分之一度；dq_pairs只统计角度有效的采样。 */
    length = snprintf(line, sizeof(line),
        "req=%u en=%u cal=%u hall=%u dir=%d dt_us=%lu edges=%lu fe_mHz=%ld "
        "Iu_avg_mA=%ld Iu_min=%ld Iu_max=%ld "
        "Iv_avg_mA=%ld Iv_min=%ld Iv_max=%ld pairs=%lu ha_mdeg=%ld ha_ok=%u "
        "pa_mdeg=%ld pa_ok=%u Id_avg_mA=%ld Iq_avg_mA=%ld dq_pairs=%lu "
        "Ud_mV=%ld Uq_mV=%ld pi_ok=%u Id_sd_mA=%ld Iq_sd_mA=%ld\r\n",
        (unsigned)request, (unsigned)enabled, (unsigned)calibrated,
        (unsigned)state, (int)direction, (unsigned long)period,
        (unsigned long)edges, (long)(electrical_hz * 1000.0f),
        (long)(iu * 1000.0f), (long)(u_min * 1000.0f), (long)(u_max * 1000.0f),
        (long)(iv * 1000.0f), (long)(v_min * 1000.0f), (long)(v_max * 1000.0f),
        (unsigned long)pairs, (long)(angle * (360000.0f / TWO_PI)),
        (unsigned)angle_valid, (long)(p_angle * (360000.0f / TWO_PI)),
        (unsigned)park_valid, (long)(id * 1000.0f), (long)(iq * 1000.0f),
        (unsigned long)dq_pairs, (long)(ud * 1000.0f), (long)(uq * 1000.0f),
        (unsigned)pi_valid, (long)(id_sd * 1000.0f), (long)(iq_sd * 1000.0f));

    if ((length > 0) && (length < (int)sizeof(line)))
    {
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)line,
                                (uint16_t)length, MONITOR_TX_TIMEOUT_MS);
    }
}

/* 编码器电角度，0到2pi：相对对零时的计数，每ENCODER_COUNTS_PER_ELEC个计数一圈。 */
static float Encoder_ElectricalAngle(void)
{
    int32_t relative = (int32_t)(__HAL_TIM_GET_COUNTER(&htim2) - enc_zero);
    int32_t within = relative % (int32_t)ENCODER_COUNTS_PER_ELEC;

    if (within < 0) within += (int32_t)ENCODER_COUNTS_PER_ELEC;
    return TWO_PI * (float)within / (float)ENCODER_COUNTS_PER_ELEC;
}

/* 串口发一行文字，超时很短，不影响控制。 */
static void Print_Text(const char *text)
{
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text,
                            (uint16_t)strlen(text), MONITOR_TX_TIMEOUT_MS);
}

/* 读串口命令，每次一个字符：a对零，v编码器换相，x全部停止，p检查串口。 */
static uint8_t Serial_ReadCommand(void)
{
    uint8_t command = 0;

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
    {
        (void)huart1.Instance->SR; // 清溢出标志：先读SR再读DR
        (void)huart1.Instance->DR;
    }
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
        command = (uint8_t)(huart1.Instance->DR & 0xFFU);
    }
    return command;
}

/* 对称限幅。 */
static float Clamp_Symmetric(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/* 每个PWM周期调用一次；每1ms（20个周期）测一次速，速度环和位置环也在这里算。
 * 测速：当前计数减10ms前的计数。位置环：位置误差给速度给定；速度环：PI输出Iq给定，Id给定为0。 */
static void Speed_Loop_Update(void)
{
    static uint8_t divider = 0, history_index = 0;
    static uint32_t history[SPEED_WINDOW_STEPS];
    uint32_t count, i;
    float error, speed_target;

    divider++;
    if (divider < SPEED_LOOP_DIV)
        return;
    divider = 0;

    count = __HAL_TIM_GET_COUNTER(&htim2);
    if (speed_started == 0U)
    {
        for (i = 0; i < SPEED_WINDOW_STEPS; i++) history[i] = count; // 刚进入：历史填成当前位置，速度为0
        history_index = 0;
        speed_rpm = 0.0f;
        speed_integral_a = 0.0f;
        speed_started = 1;
        return;
    }
    speed_rpm = (float)(int32_t)(count - history[history_index]) * SPEED_RPM_PER_COUNT;
    history[history_index] = count;
    history_index++;
    if (history_index >= SPEED_WINDOW_STEPS) history_index = 0;

    /* 打印窗口统计：平均、平方和、最小、最大。 */
    if (speed_samples == 0U)
    {
        speed_min = speed_max = speed_rpm;
    }
    else
    {
        if (speed_rpm < speed_min) speed_min = speed_rpm;
        if (speed_rpm > speed_max) speed_max = speed_rpm;
    }
    speed_sum += speed_rpm;
    speed_sq += speed_rpm * speed_rpm;
    speed_samples++;

    if (aux_mode == AUX_ENC_SPIN)
        return; // 编码器换相只测速，不闭环

    if (aux_mode == AUX_POSITION)
    {
        position_error = (int32_t)(position_ref - count);
        speed_target = Clamp_Symmetric(position_kp * (float)position_error, POSITION_SPEED_MAX_RPM);
        speed_ref_rpm = speed_target;
    }
    error = speed_ref_rpm - speed_rpm;
    speed_integral_a = Clamp_Symmetric(speed_integral_a + speed_ki * error * SPEED_PI_DT, IQ_REF_MAX_A);
    Id_ref = 0.0f;
    Iq_ref = Clamp_Symmetric(speed_kp * error + speed_integral_a, IQ_REF_MAX_A);
}

/* 编码器计数、电角度、转速统计，每100ms打印一次。转速统计是上一个100ms内每1ms测速的结果。 */
static void Encoder_Print(void)
{
    static uint32_t last_ms = 0, last_count = 0;
    char line[256];
    uint32_t now = HAL_GetTick(), count, primask, samples;
    int32_t delta, pos_err;
    float mean = 0.0f, sd = 0.0f, low, high, ref, sum, sq;
    int length;

    if ((uint32_t)(now - last_ms) < MONITOR_PERIOD_MS)
        return;
    last_ms = now;

    count = __HAL_TIM_GET_COUNTER(&htim2);
    delta = (int32_t)(count - last_count); // 正转或反转由符号区分
    last_count = count;

    primask = __get_PRIMASK();
    __disable_irq();
    samples = speed_samples;
    sum = speed_sum;
    sq = speed_sq;
    low = speed_min;
    high = speed_max;
    ref = speed_ref_rpm;
    pos_err = position_error;
    speed_samples = 0;
    speed_sum = speed_sq = 0.0f;
    __set_PRIMASK(primask);

    if (samples != 0U)
    {
        mean = sum / (float)samples;
        sd = sqrtf(fmaxf(sq / (float)samples - mean * mean, 0.0f));
    }
    else
    {
        low = high = 0.0f; // 没有测速时打印占位值
    }

    length = snprintf(line, sizeof(line),
        "enc_cnt=%lu enc_delta=%ld enc_ok=%u enc_e_mdeg=%ld aux=%u ref_d=%ld ref_q=%ld "
        "spd_x10=%ld spd_sd_x10=%ld spd_min_x10=%ld spd_max_x10=%ld spd_ref_x10=%ld pos_err=%ld\r\n",
        (unsigned long)count, (long)delta, (unsigned)enc_zero_valid,
        (long)(Encoder_ElectricalAngle() * (360000.0f / TWO_PI)),
        (unsigned)aux_mode, (long)(Id_ref * 1000.0f), (long)(Iq_ref * 1000.0f),
        (long)(mean * 10.0f), (long)(sd * 10.0f), (long)(low * 10.0f),
        (long)(high * 10.0f), (long)(ref * 10.0f), (long)pos_err);
    if ((length > 0) && (length < (int)sizeof(line)))
    {
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)line,
                                (uint16_t)length, MONITOR_TX_TIMEOUT_MS);
    }
}

/* 对零和编码器换相的启停与计时。KEY2或'a'对零，KEY3或'v'编码器换相，KEY1或'x'停止。 */
static void Drive_AuxUpdate(void)
{
    static uint8_t key2_armed = 0, key3_armed = 0, checked = 0;
    static uint32_t position_home = 0; // 进入位置环时的计数，'0'命令回到这里
    static uint32_t start_ms = 0, check_count = 0;
    char line[96];
    uint32_t now = HAL_GetTick(), count;
    int32_t drift;
    uint8_t command = Serial_ReadCommand();
    uint8_t align_event = (command == 'a') ? 1U : 0U;
    uint8_t spin_event = (command == 'v') ? 1U : 0U;
    uint8_t current_event = (command == 'c') ? 1U : 0U;
    uint8_t speed_event = (command == 's') ? 1U : 0U;
    uint8_t position_event = (command == 'g') ? 1U : 0U;
    uint8_t calibrated = ((adc_u_offset_ready != 0U) && (adc_v_offset_ready != 0U)) ? 1U : 0U;
    int length;

    /* 按键松开后再按下才算一次。 */
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET)
    {
        key2_armed = 1;
    }
    else if (key2_armed == 1)
    {
        key2_armed = 0;
        align_event = 1;
    }
    if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_SET)
    {
        key3_armed = 1;
    }
    else if (key3_armed == 1)
    {
        key3_armed = 0;
        spin_event = 1;
    }

    /* 电流环运行中：u/j改Iq给定，i/k改Id给定，0清零，每次0.1A。 */
    if (aux_mode == AUX_CURRENT)
    {
        if (command == 'u') Iq_ref += CURRENT_REF_STEP_A;
        else if (command == 'j') Iq_ref -= CURRENT_REF_STEP_A;
        else if (command == 'i') Id_ref += CURRENT_REF_STEP_A;
        else if (command == 'k') Id_ref -= CURRENT_REF_STEP_A;
        else if (command == '0') Id_ref = Iq_ref = 0.0f;
        if (Iq_ref > IQ_REF_MAX_A) Iq_ref = IQ_REF_MAX_A;
        if (Iq_ref < -IQ_REF_MAX_A) Iq_ref = -IQ_REF_MAX_A;
        if (Id_ref > ID_REF_MAX_A) Id_ref = ID_REF_MAX_A;
        if (Id_ref < -ID_REF_MAX_A) Id_ref = -ID_REF_MAX_A;
    }

    /* 速度环运行中：u/j改速度给定，0清零，每次20rpm。 */
    if (aux_mode == AUX_SPEED)
    {
        if (command == 'u') speed_ref_rpm += SPEED_REF_STEP_RPM;
        else if (command == 'j') speed_ref_rpm -= SPEED_REF_STEP_RPM;
        else if (command == '0') speed_ref_rpm = 0.0f;
        speed_ref_rpm = Clamp_Symmetric(speed_ref_rpm, SPEED_REF_MAX_RPM);
    }
    /* 位置环运行中：u/j改位置给定，每次1000计数，0回到起点。 */
    if (aux_mode == AUX_POSITION)
    {
        if (command == 'u') position_ref += (uint32_t)POSITION_STEP_COUNTS;
        else if (command == 'j') position_ref -= (uint32_t)POSITION_STEP_COUNTS;
        else if (command == '0') position_ref = position_home;
    }

    /* 速度环、位置环运行中调参：e/d速度环Kp，r/f速度环Ki，t/y位置环Kp，每次乘或除1.5。 */
    if ((aux_mode == AUX_SPEED) || (aux_mode == AUX_POSITION))
    {
        uint8_t gain_changed = 1U;

        if (command == 'e') speed_kp *= 1.5f;
        else if (command == 'd') speed_kp /= 1.5f;
        else if (command == 'r') speed_ki *= 1.5f;
        else if (command == 'f') speed_ki /= 1.5f;
        else if (command == 't') position_kp *= 1.5f;
        else if (command == 'y') position_kp /= 1.5f;
        else gain_changed = 0U;
        if (gain_changed != 0U)
        {
            length = snprintf(line, sizeof(line), "gain speed_kp_x1e4=%ld speed_ki_x1e3=%ld pos_kp_x1e3=%ld\r\n",
                              (long)(speed_kp * 10000.0f), (long)(speed_ki * 1000.0f), (long)(position_kp * 1000.0f));
            if ((length > 0) && (length < (int)sizeof(line)))
            {
                Print_Text(line);
            }
        }
    }

    if (command == 'p')
    {
        Print_Text("pong\r\n");
    }
    if (drive_fault != 0U)
    {
        drive_fault = 0;
        Print_Text("fault: overspeed, stopped\r\n");
    }

    if (command == 'x')
    {
        aux_mode = AUX_NONE;
        motor_request = 0;
        Print_Text("stop\r\n");
    }
    else if ((align_event != 0U) || (spin_event != 0U) || (current_event != 0U) ||
             (speed_event != 0U) || (position_event != 0U))
    {
        if (aux_mode != AUX_NONE)
        {
            aux_mode = AUX_NONE; // 再按一次就停
            Print_Text("stop\r\n");
        }
        else if ((motor_request != 0U) || (calibrated == 0U))
        {
            Print_Text("refused: running or not calibrated\r\n");
        }
        else if (align_event != 0U)
        {
            enc_zero_valid = 0;
            checked = 0;
            start_ms = now;
            aux_mode = AUX_ALIGN;
            Print_Text("align start\r\n");
        }
        else if (enc_zero_valid == 0U)
        {
            Print_Text("refused: align first\r\n");
        }
        else if (spin_event != 0U)
        {
            start_ms = now;
            aux_mode = AUX_ENC_SPIN;
            Print_Text("spin start\r\n");
        }
        else if (speed_event != 0U)
        {
            Id_ref = 0.0f;
            Iq_ref = 0.0f;
            speed_ref_rpm = 0.0f;
            start_ms = now;
            aux_mode = AUX_SPEED;
            Print_Text("speed start\r\n");
        }
        else if (position_event != 0U)
        {
            Id_ref = 0.0f;
            Iq_ref = 0.0f;
            speed_ref_rpm = 0.0f;
            position_home = __HAL_TIM_GET_COUNTER(&htim2);
            position_ref = position_home;
            start_ms = now;
            aux_mode = AUX_POSITION;
            Print_Text("position start\r\n");
        }
        else if (current_event != 0U)
        {
            Id_ref = 0.0f; // 电流环从零给定开始，再用串口命令调整
            Iq_ref = 0.0f;
            start_ms = now;
            aux_mode = AUX_CURRENT;
            Print_Text("current start\r\n");
        }
    }

    if (aux_mode == AUX_ALIGN)
    {
        if ((checked == 0U) &&
            ((uint32_t)(now - start_ms) >= (ALIGN_HOLD_MS - ALIGN_CHECK_BEFORE_MS)))
        {
            check_count = __HAL_TIM_GET_COUNTER(&htim2);
            checked = 1;
        }
        if ((uint32_t)(now - start_ms) >= ALIGN_HOLD_MS)
        {
            count = __HAL_TIM_GET_COUNTER(&htim2);
            drift = (int32_t)(count - check_count); // 最后一段时间转子是否还在动
            if ((drift <= ALIGN_MAX_DRIFT_COUNTS) && (drift >= -ALIGN_MAX_DRIFT_COUNTS))
            {
                enc_zero = count;
                enc_zero_valid = 1;
            }
            else
            {
                enc_zero_valid = 0;
            }
            aux_mode = AUX_NONE;
            length = snprintf(line, sizeof(line), "align ok=%u zero=%lu drift=%ld\r\n",
                              (unsigned)enc_zero_valid, (unsigned long)count, (long)drift);
            if ((length > 0) && (length < (int)sizeof(line)))
            {
                Print_Text(line);
            }
        }
    }
    else if ((aux_mode == AUX_ENC_SPIN) || AUX_USES_CURRENT_LOOP())
    {
        if ((uint32_t)(now - start_ms) >= ENC_SPIN_MAX_MS)
        {
            aux_mode = AUX_NONE;
            Print_Text("spin end\r\n");
        }
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
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
        aux_mode = AUX_NONE;
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
        if (aux_mode != AUX_NONE)
        {
            aux_mode = AUX_NONE; // 对零或编码器换相中，KEY1只负责停止
        }
        else
        {
            motor_request = !motor_request;
        }
    }

    Drive_AuxUpdate();
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
        if (enc_zero_valid != 0U)
        {
            park_angle = Encoder_ElectricalAngle(); // 已对零：用编码器电角度
            park_angle_valid = 1;
        }
        else
        {
            park_angle_valid = hall_angle_valid;
            if (park_angle_valid != 0U)
            {
                /* 霍尔相对角度加参考偏移，作为Park的旋转角度。 */
                park_angle = hall_angle + HALL_PHASE_SHIFT_RAD;
                if (park_angle >= TWO_PI) park_angle -= TWO_PI;
                if (park_angle < 0.0f) park_angle += TWO_PI;
            }
        }
        if (park_angle_valid != 0U)
        {
            Park_Update(current_alpha, current_beta, park_angle);
            monitor_d_sum += current_d;
            monitor_q_sum += current_q;
            monitor_d_sq += current_d * current_d;
            monitor_q_sq += current_q * current_q;
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
        static uint32_t spin_last_count = 0;
        static uint8_t spin_started = 0;
        float angle_now; // 电流环用的编码器电角度，rad
        float vmag; // 本周期电压矢量幅值，V

        Hall_AngleUpdate(); // 独立观察霍尔角度

        /* 编码器换相超速保护：相邻两个PWM周期计数变化太大就停机。 */
        if ((aux_mode == AUX_ENC_SPIN) || AUX_USES_CURRENT_LOOP())
        {
            uint32_t count = __HAL_TIM_GET_COUNTER(&htim2);
            int32_t step = (int32_t)(count - spin_last_count);

            if ((spin_started != 0U) &&
                ((step > ENC_OVERSPEED_COUNTS) || (step < -ENC_OVERSPEED_COUNTS)))
            {
                aux_mode = AUX_NONE;
                drive_fault = 1;
            }
            spin_last_count = count;
            spin_started = 1;
        }
        else
        {
            spin_started = 0;
        }

        /* 编码器测速；速度环、位置环在其中每1ms算一次，输出Iq给定。 */
        if ((aux_mode == AUX_ENC_SPIN) || (aux_mode == AUX_SPEED) || (aux_mode == AUX_POSITION))
        {
            Speed_Loop_Update();
        }
        else
        {
            speed_started = 0;
        }

        if (((motor_request == 0) && (aux_mode == AUX_NONE)) ||
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
            if (aux_mode == AUX_NONE)
            {
                theta += TWO_PI * frequency / PWM_UPDATE_HZ;
            }
        }

			if (theta >= TWO_PI)
			{
				theta -= TWO_PI;
			}

			vmag = voltage;
			if (aux_mode == AUX_ALIGN)
			{
			    theta = 0.0f; // 电压矢量固定在0度，转子被拉到d轴
			    vmag = ALIGN_VOLTAGE_V;
			}
			else if (aux_mode == AUX_ENC_SPIN)
			{
			    theta = Encoder_ElectricalAngle() + HALF_PI; // 电压矢量超前转子90度，只有Uq
			    if (theta >= TWO_PI) theta -= TWO_PI;
			    vmag = ENC_SPIN_VOLTAGE_V;
			}
			else if (AUX_USES_CURRENT_LOOP())
			{
				angle_now = Encoder_ElectricalAngle(); // 逆Park用同一个编码器角度
			}

			if (AUX_USES_CURRENT_LOOP())
			{
				/* 逆Park：PI输出的Ud、Uq变成静止两轴电压。 */
				Ualpha = Ud * cosf(angle_now) - Uq * sinf(angle_now);
				Ubeta  = Ud * sinf(angle_now) + Uq * cosf(angle_now);
			}
			else
			{
				Ualpha = vmag * cosf(theta);
				Ubeta  = vmag * sinf(theta);
			}

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
