/* Isolated 652 test harness. Never included by the learning project. */
#include "mc_api.h"
#include "mc_config.h"

#define DIAG_START_KEY 0x652A18u
#define DIAG_MAX_MS 5000u
#define DIAG_ROWS 700u
typedef struct {
  uint32_t ms, state, faults, hall_edges;
  int16_t rpm, iq, id, iu, iv, bus_v, temp_c, reliable;
} DiagRow;
volatile uint32_t diag_request, diag_ready, diag_active, diag_done, diag_reason;
volatile uint32_t diag_count, diag_started, diag_hall_edges, diag_hall_invalid;
volatile uint32_t diag_preflight[5][3]; /* mean, min, max: U,V,W,bus,temp */
volatile DiagRow diag_rows[DIAG_ROWS];
static uint32_t diag_last_ms, diag_last_log, diag_previous_hall;

static void Diag_GPIO(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOH_CLK_ENABLE();
  GPIOH->BSRR = GPIO_PIN_9 << 16;
  g.Pin=GPIO_PIN_9; g.Mode=GPIO_MODE_OUTPUT_PP; g.Pull=GPIO_PULLDOWN;
  g.Speed=GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOH,&g);
  g.Pin=GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12; g.Mode=GPIO_MODE_INPUT;
  g.Pull=GPIO_NOPULL; HAL_GPIO_Init(GPIOH,&g);
}

/* Runs before MC initialization. Restore ADC configuration immediately after. */
static void Diag_Preflight(void)
{
  const uint32_t channels[5]={6,8,9,10,13};
  uint32_t c,n,x,sum,lo,hi,deadline;
  ADC1->CR1=0; ADC1->CR2=ADC_CR2_ADON;
  ADC1->SQR1=0; ADC1->SQR2=0;
  ADC1->SMPR1=0x24924924; ADC1->SMPR2=0x24924924; /* 84 cycles */
  HAL_Delay(5);
  for(c=0;c<5;c++) {
    sum=0;lo=4095;hi=0;ADC1->SQR3=channels[c];
    for(n=0;n<256;n++) {
      ADC1->SR=0;ADC1->CR2|=ADC_CR2_SWSTART;deadline=HAL_GetTick();
      while(!(ADC1->SR&ADC_SR_EOC)) {
        if(HAL_GetTick()-deadline>10) {diag_reason=10;return;}
      }
      x=ADC1->DR;sum+=x;if(x<lo)lo=x;if(x>hi)hi=x;
    }
    diag_preflight[c][0]=sum/256;diag_preflight[c][1]=lo;diag_preflight[c][2]=hi;
  }
  ADC1->CR2=0;
}

static void Diag_Stop(uint32_t reason)
{
  GPIOH->BSRR=GPIO_PIN_9<<16;
  diag_active=0;diag_done=1;diag_reason=reason;
  MC_StopMotor1();
}

/* SysTick hook: independent of main's telemetry or host connection. */
void Diag_Tick(void)
{
  uint32_t t,h,s,f;
  Curr_Components iq,ab;
  volatile DiagRow *r;
  if(!diag_ready)return;
  t=HAL_GetTick();if(t==diag_last_ms)return;diag_last_ms=t;
  h=(GPIOH->IDR>>10)&7;
  if(h!=diag_previous_hall) {
    if(h&&h!=7)diag_hall_edges++;else diag_hall_invalid++;
    diag_previous_hall=h;
  }
  s=MC_GetSTMStateMotor1();f=MC_GetCurrentFaultsMotor1();
  if(diag_active) {
    if(f||s==FAULT_NOW||s==FAULT_OVER)Diag_Stop(2);
    else if(t-diag_started>=DIAG_MAX_MS)Diag_Stop(1);
  } else GPIOH->BSRR=GPIO_PIN_9<<16;
  if(diag_started && t-diag_last_log>=10 && diag_count<DIAG_ROWS) {
    diag_last_log=t;r=&diag_rows[diag_count];
    iq=MC_GetIqdMotor1();ab=MC_GetIabMotor1();
    r->ms=t-diag_started;r->state=s;
    r->faults=f|((uint32_t)MC_GetOccurredFaultsMotor1()<<16);
    r->hall_edges=diag_hall_edges;
    r->rpm=MC_GetMecSpeedAverageMotor1()*6;
    r->iq=iq.qI_Component1;r->id=iq.qI_Component2;
    r->iu=ab.qI_Component1;r->iv=ab.qI_Component2;
    r->bus_v=VBS_GetAvBusVoltage_V(&RealBusVoltageSensorParamsM1._Super);
    r->temp_c=NTC_GetAvTemp_C(&TempSensorParamsM1);
    r->reliable=MC_GetSpeedSensorReliabilityMotor1();
    diag_count++;
  }
}

static void Diag_Main(void)
{
  uint32_t i,v;
  if(diag_active)IWDG->KR=0xAAAA;
  /* No retries, no automatic acknowledgement of any fault. */
  if(diag_request!=DIAG_START_KEY||diag_started||diag_done)return;
  diag_request=0;
  v=VBS_GetAvBusVoltage_V(&RealBusVoltageSensorParamsM1._Super);
  if(diag_reason||HAL_GetTick()<1000||MC_GetSTMStateMotor1()!=IDLE||
     MC_GetCurrentFaultsMotor1()||MC_GetOccurredFaultsMotor1()||v<15||v>20) {
    diag_done=1;diag_reason=3;return;
  }
  for(i=0;i<3;i++)if(diag_preflight[i][0]<1700||diag_preflight[i][0]>2400) {
    diag_done=1;diag_reason=4;return;
  }
  /* Approximately 1 second watchdog; resetting returns immediately to PH9 off. */
  IWDG->KR=0x5555;IWDG->PR=4;IWDG->RLR=500;
  IWDG->KR=0xAAAA;IWDG->KR=0xCCCC;
  diag_started=HAL_GetTick();diag_last_log=diag_started;
  diag_hall_edges=0;diag_hall_invalid=0;
  diag_active=1;
  MC_ProgramSpeedRampMotor1(1000/6,0);
  GPIOH->BSRR=GPIO_PIN_9;
  if(!MC_StartMotor1())Diag_Stop(5);
}
