/* Standstill ADC diagnostic: motor-control tasks and IRQs intentionally absent. */
#include "stm32f4xx_hal.h"
void SysTick_Handler(void) { HAL_IncTick(); HAL_SYSTICK_IRQHandler(); }
void HardFault_Handler(void)
{
  GPIOH->BSRR = (uint32_t)GPIO_PIN_9 << 16;
  while (1) {}
}
