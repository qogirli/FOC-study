/* Temporary diagnostic only. Never starts on reset; debugger requests a test.
 * Captures untouched raw paired results; no printing in ISR.
 * A run request uses existing 1 V / 1 Hz settings and stops within 3.5 seconds.
 */
#define DIAG_N 2048U
typedef struct {
    uint32_t cycles, seq;
    uint16_t u, v, cnt, flags, ccr1, ccr2, ccr3, reserved;
} DiagPoint;
volatile DiagPoint diag_data[DIAG_N];
volatile uint32_t diag_enabled = 1;
volatile uint32_t diag_request = 0, diag_done = 0, diag_count = 0;
volatile uint32_t diag_running = 0, diag_start_ms = 0;
volatile uint32_t diag_capture = 0;

static void Diag_Sample(void)
{
    uint32_t n = diag_count;
    if (!diag_capture || n >= DIAG_N) return;
    diag_data[n].cycles = DWT->CYCCNT;
    diag_data[n].seq = adc_u_count;
    diag_data[n].u = adc_u_raw;
    diag_data[n].v = adc_v_raw;
    diag_data[n].cnt = TIM8->CNT;
    diag_data[n].flags = (uint16_t)((TIM8->CR1 & TIM_CR1_DIR) |
                          ((GPIOH->ODR & GPIO_PIN_9) ? 1U : 0U));
    diag_data[n].ccr1 = TIM8->CCR1;
    diag_data[n].ccr2 = TIM8->CCR2;
    diag_data[n].ccr3 = TIM8->CCR3;
    diag_data[n].reserved = (uint16_t)(adc_u_count - adc_v_count);
    diag_count = n + 1U;
    if (diag_count == DIAG_N) {
        diag_capture = 0;
        motor_request = 0;
        GPIOH->BSRR = GPIO_BSRR_BR9;
        diag_running = 0;
        diag_done = 1;
    }
}

static void Diag_Poll(void)
{
    static uint32_t waiting = 0;
    uint32_t req;
    if (!adc_u_offset_ready || !adc_v_offset_ready) return;
    if (diag_request && !waiting && !diag_capture) {
        req = diag_request;
        diag_request = 0;
        motor_request = 0;
        GPIOH->BSRR = GPIO_BSRR_BR9;
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        diag_count = 0;
        diag_done = 0;
        diag_start_ms = HAL_GetTick();
        if (req == 2U) {
            diag_running = 1;
            motor_request = 1;
            waiting = 1;
        } else {
            diag_capture = 1;
        }
    }
    if (waiting && (uint32_t)(HAL_GetTick() - diag_start_ms) >= 2000U) {
        waiting = 0;
        diag_capture = 1;
    }
}
