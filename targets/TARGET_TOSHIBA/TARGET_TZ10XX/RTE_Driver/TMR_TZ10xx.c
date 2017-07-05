/**
 * @file TMR_TZ10xx.c
 * @brief TZ10xx TMR/ADVTMR driver
 * @version V0.0
 * @date $Date:: 2016-02-26 14:55:11 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */
#include "TZ10xx.h"
#include "TMR_TZ10xx.h"
#include "PMU_TZ10xx.h"

#define tmr_reg_ch(reg) (*(&reg + rsc->ch_offset))

#define TZ1000_TMR_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 41)   /* driver version */

typedef struct {
  bool init;
  ARM_POWER_STATE power_state;
  TMR_SignalEvent_t cb_event;
  uint32_t timer_size;
  uint32_t event_mask;
} TMR_INFO;

typedef struct {
  ptrdiff_t ch_offset;
  IRQn_Type irq;
  TMR_INFO *info;
} const TMR_RESOURCES;

static TMR_INFO TMR0_Info;
static TMR_INFO TMR1_Info;

static TMR_RESOURCES TMR0_Resources = {
  0,
  TMR0_IRQn,
  &TMR0_Info
};

static TMR_RESOURCES TMR1_Resources = {
  (&tmr->TIMER1LOAD - &tmr->TIMER0LOAD),
  TMR1_IRQn,
  &TMR1_Info
};

static TMR_STATUS TMR_PowerControl(TMR_RESOURCES *rsc, ARM_POWER_STATE state);

/**
 * @brief Check whether a value is valid for TIMERnLOAD or TIMERnBGLOAD
 * @param[in] value a value to set to TIMERnLOAD or TIMERnBGLOAD
 * @param[in] timer_size bit width of timer
 * @return 0=invalid, 1=valid
 */
static uint32_t IsCounterValid(uint32_t value, uint32_t timer_size)
{
  if (value == 0) {
    return 0;
  } else if((timer_size < 32) && (value > ((1u << timer_size) - 1))) {
    return 0;
  } else {
    return 1;
  }
}

/**
  @brief Get driver version.
  @return \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION TMRX_GetVersion(void)
{
  static const ARM_DRIVER_VERSION driver_version = {
    TZ1000_TMR_API_VERSION,
    TZ1000_TMR_DRV_VERSION
  };
  return driver_version;
}

/**
  @brief Get driver capabilities.
  @return \ref TMR_CAPABILITIES
*/
static TMR_CAPABILITIES TMRX_GetCapabilities(void)
{
  static const TMR_CAPABILITIES caps = {
    0, /* capture */
    0, /* compare */
    0, /* tff */
    0, /* event_counter */
  };
  return caps;
}

/**
  @brief Initialize TMR Driver Interface.
  @param[in]   cb_event    Pointer to \ref TMR_SignalEvent
  @param[in]   event_mask  Events that are reported through callbacks
  @return      execution status \ref TMR_STATUS
*/
static TMR_STATUS TMR_Initialize(TMR_RESOURCES *rsc, TMR_SignalEvent_t cb_event, uint32_t event_mask)
{
  if (rsc->info->init) {
    return TMR_ERROR;
  }
  
  rsc->info->cb_event = cb_event;
  rsc->info->event_mask = event_mask;
  if ((event_mask & (1u << TMR_EVENT_BASE_COUNTER)) != 0) {
    NVIC_ClearPendingIRQ(rsc->irq);
    NVIC_EnableIRQ(rsc->irq);
  }
  rsc->info->init = true;
  rsc->info->timer_size = 32;
  TMR_PowerControl(rsc, ARM_POWER_LOW);
  return TMR_OK;
}

/**
  @brief De-initialize TMR Driver Interface.
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS TMR_Uninitialize(TMR_RESOURCES *rsc)
{
  NVIC_ClearPendingIRQ(rsc->irq);
  NVIC_DisableIRQ(rsc->irq);
  TMR_PowerControl(rsc, ARM_POWER_OFF);
  rsc->info->init = false;
  return TMR_OK;
}

/**
  @brief Control TMR Driver Interface Power.
  @param[in] state Power state
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS TMR_PowerControl(TMR_RESOURCES *rsc, ARM_POWER_STATE state)
{
  if (!rsc->info->init) {
    return TMR_ERROR;
  }
  
  if (state == rsc->info->power_state) {
    return TMR_OK;
  }
  
  if ((uint32_t)state > ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  
  if (state != ARM_POWER_OFF) {
    /* OFF -> (LOW or FULL) */
    Driver_PMU.EnableModule(PMU_MODULE_TMR, 1);
  }
  
  rsc->info->power_state = state;
  
  if (state == ARM_POWER_OFF) {
    /* (LOW or FULL) -> OFF */
    /* stop TMR clock if all channels are off */
    if ((TMR0_Info.power_state == ARM_POWER_OFF)
        && (TMR1_Info.power_state == ARM_POWER_OFF)) {
      Driver_PMU.EnableModule(PMU_MODULE_TMR, 0);
    }
  }
  
  return TMR_OK;
}

/**
  @brief Configure TMR
  @param[in] couter_bits
  @param[in] count_mode
  @param[in] divisor
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS TMR_Configure(
  TMR_RESOURCES *rsc,
  uint32_t counter_bits,
  TMR_COUNT_MODE count_mode,
  uint32_t divisor)
{
  uint32_t one_shot_count;
  uint32_t timer_size;
  uint32_t clock_divid;
  uint32_t periodic_mode;
  
  if (!rsc->info->init) {
    return TMR_ERROR;
  }
  if (rsc->info->power_state == ARM_POWER_OFF) {
    return TMR_ERROR;
  }
  if ((tmr_reg_ch(tmr->TIMER0CONTROL) & (1u << 7)) != 0) {
    /* Timer is running */
    return TMR_ERROR;
  }
  
  switch (counter_bits) {
  case 16:
    timer_size = 0;
    break;
  case 32:
    timer_size = 1;
    break;
  default:
    return TMR_ERROR;
  }
  
  switch (count_mode) {
  case TMR_COUNT_MODE_ONE_SHOT:
    one_shot_count = 1;
    periodic_mode = 0;
    break;
  case TMR_COUNT_MODE_PERIODIC:
    one_shot_count = 0;
    periodic_mode = 1;
    break;
  case TMR_COUNT_MODE_FREE_RUN:
    one_shot_count = 0;
    periodic_mode = 0;
    break;
  default:
    return TMR_ERROR;
  }
  
  switch (divisor) {
  case 1:
    clock_divid = 0;
    break;
  case 16:
    clock_divid = 1;
    break;
  case 256:
    clock_divid = 2;
    break;
  default:
    return TMR_ERROR;
  }  
  
  tmr_reg_ch(tmr->TIMER0CONTROL) = one_shot_count
                                 | (timer_size << 1)
                                 | (clock_divid << 2)
                                 | (1u << 5) /* interrupt_enable */
                                 | (periodic_mode << 6);
  rsc->info->timer_size = counter_bits;
  return TMR_OK;
}

/**
  @brief Start the timer
  @param[in] initial_value initial counter value
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS TMR_Start(TMR_RESOURCES *rsc, uint32_t initial_value)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  if (!IsCounterValid(initial_value, rsc->info->timer_size)) {
    return TMR_ERROR;
  }
  tmr_reg_ch(tmr->TIMER0LOAD) = initial_value;
  tmr_reg_ch(tmr->TIMER0CONTROL) |= 1u << 7; /* enable = 1 */
  return TMR_OK;
}

/**
  @brief Stop the timer
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS TMR_Stop(TMR_RESOURCES *rsc)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  tmr_reg_ch(tmr->TIMER0CONTROL) &= ~(1u << 7); /* enable = 0 */
  return TMR_OK;
}

/**
  @brief Indicate whether the timer is running
  @return true if the timer is running, otherwise false
*/
static bool TMR_IsRunning(TMR_RESOURCES *rsc)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return false;
  }
  return (tmr_reg_ch(tmr->TIMER0CONTROL) >> 7) & 1u; /* TIMERnCONTROL.enable */
}

/**
  @brief Set value of timer immediately
  @param[in] value new value
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS TMR_SetValue(TMR_RESOURCES *rsc, uint32_t value)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  if (!IsCounterValid(value, rsc->info->timer_size)) {
    return TMR_ERROR;
  }
  tmr_reg_ch(tmr->TIMER0LOAD) = value;
  return TMR_OK;
}

/**
  @brief Set reload value of timer
  @param[in] value new value
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS TMR_SetReloadValue(TMR_RESOURCES *rsc, uint32_t reload_value)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  if (!IsCounterValid(reload_value, rsc->info->timer_size)) {
    return TMR_ERROR;
  }
  tmr_reg_ch(tmr->TIMER0BGLOAD) = reload_value;
  return TMR_OK;
}

/**
  @brief Get current value of timer
  @return current value of timer
*/
static uint32_t TMR_GetValue(TMR_RESOURCES *rsc)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return 0xffffffff;
  }
  return tmr_reg_ch(tmr->TIMER0VALUE);
}

/**
 * @brief not available in TMR driver
 */
static TMR_STATUS TMRX_ConfigureCapture(
  TMR_SIGNAL signal, TMR_CAPTURE_TRIGGER trigger, bool reload, bool sync)
{
  return TMR_ERROR;
}

/**
 * @brief not available in TMR driver
 */
static TMR_STATUS TMRX_EnableCapture(bool enable)
{
  return TMR_ERROR;
}

/**
 * @brief not available in TMR driver
 */
static uint32_t TMRX_GetCapturedValue(void)
{
  return 0xffffffffu;
}

/**
 * @brief not available in TMR driver
 */
static TMR_STATUS TMRX_EnableCompare(bool enable)
{
  return TMR_ERROR;
}

/**
 * @brief not available in TMR driver
 */
static TMR_STATUS TMRX_SetCompareValue(uint32_t value, bool load_at_terminal)
{
  return TMR_ERROR;
}

/**
 * @brief not available in TMR driver
 */
static TMR_STATUS TMRX_ConfigureTFF(TMR_TFF_MODE mode, bool init_high, bool invert)
{
  return TMR_ERROR;
}
  
/**
 * @brief not available in TMR driver
 */
static TMR_STATUS TMRX_EnableTFF(bool enable)
{
  return TMR_ERROR;
}

/**
 * @brief not available in TMR driver
 */
static TMR_STATUS TMRX_ConfigureEventCounter(
  TMR_EVENT_EN_MODE event_en_mode, bool ev_sync, bool ev_delay, TMR_SIGNAL evcntsrc, TMR_SIGNAL evcnten)
{
  return TMR_ERROR;
}

/**
 * @brief not available in TMR driver
 */
static TMR_STATUS TMRX_UnconfigureEventCounter(void)
{
  return TMR_ERROR;
}

/**
 * @brief not available in TMR driver
 */
static TMR_STATUS TMRX_EnableEventCounter(bool enable)
{
  return TMR_ERROR;
}

/**
 * @brief TMR base counter interrupt handler
 */
static void TMR_IRQHandler(TMR_RESOURCES *rsc)
{
  tmr_reg_ch(tmr->TIMER0INTCLR) = 1;
  if ((tmr_reg_ch(tmr->TIMER0CONTROL) & 1u) != 0) {
    TMR_Stop(rsc);
  }
  if (rsc->info->cb_event != 0) {
    rsc->info->cb_event(TMR_EVENT_BASE_COUNTER);
  }
}

static TMR_STATUS TMR0_Initialize(TMR_SignalEvent_t cb_event, uint32_t event_mask)
{
  return TMR_Initialize(&TMR0_Resources, cb_event, event_mask);
}

static TMR_STATUS TMR0_Uninitialize(void)
{
  return TMR_Uninitialize(&TMR0_Resources);
}

static TMR_STATUS TMR0_PowerControl(ARM_POWER_STATE state)
{
  return TMR_PowerControl(&TMR0_Resources, state);
}

static TMR_STATUS TMR0_Configure(
  uint32_t counter_bits,
  TMR_COUNT_MODE count_mode,
  uint32_t divisor)
{
  return TMR_Configure(&TMR0_Resources, counter_bits, count_mode, divisor);
}

static TMR_STATUS TMR0_Start(uint32_t initial_value)
{
  return TMR_Start(&TMR0_Resources, initial_value);
}

static TMR_STATUS TMR0_Stop(void)
{
  return TMR_Stop(&TMR0_Resources);
}

static bool TMR0_IsRunning(void)
{
  return TMR_IsRunning(&TMR0_Resources);
}

static TMR_STATUS TMR0_SetValue(uint32_t value)
{
  return TMR_SetValue(&TMR0_Resources, value);
}

static TMR_STATUS TMR0_SetReloadValue(uint32_t reload_value)
{
  return TMR_SetReloadValue(&TMR0_Resources, reload_value);
}

static uint32_t TMR0_GetValue(void)
{
  return TMR_GetValue(&TMR0_Resources);
}

void TMR0_IRQHandler(void)
{
  TMR_IRQHandler(&TMR0_Resources);
}

TZ10XX_DRIVER_TMR Driver_TMR0 = {
  TMRX_GetVersion,
  TMRX_GetCapabilities,
  TMR0_Initialize,
  TMR0_Uninitialize,
  TMR0_PowerControl,
  TMR0_Configure,
  TMR0_Start,
  TMR0_Stop,
  TMR0_IsRunning,
  TMR0_SetValue,
  TMR0_SetReloadValue,
  TMR0_GetValue,
  TMRX_ConfigureCapture,
  TMRX_EnableCapture,
  TMRX_GetCapturedValue,
  TMRX_EnableCompare,
  TMRX_SetCompareValue,
  TMRX_ConfigureTFF,
  TMRX_EnableTFF,
  TMRX_ConfigureEventCounter,
  TMRX_UnconfigureEventCounter,
  TMRX_EnableEventCounter
};

static TMR_STATUS TMR1_Initialize(TMR_SignalEvent_t cb_event, uint32_t event_mask)
{
  return TMR_Initialize(&TMR1_Resources, cb_event, event_mask);
}

static TMR_STATUS TMR1_Uninitialize(void)
{
  return TMR_Uninitialize(&TMR1_Resources);
}

static TMR_STATUS TMR1_PowerControl(ARM_POWER_STATE state)
{
  return TMR_PowerControl(&TMR1_Resources, state);
}

static TMR_STATUS TMR1_Configure(
  uint32_t counter_bits,
  TMR_COUNT_MODE count_mode,
  uint32_t divisor)
{
  return TMR_Configure(&TMR1_Resources, counter_bits, count_mode, divisor);
}

static TMR_STATUS TMR1_Start(uint32_t initial_value)
{
  return TMR_Start(&TMR1_Resources, initial_value);
}

static TMR_STATUS TMR1_Stop(void)
{
  return TMR_Stop(&TMR1_Resources);
}

static bool TMR1_IsRunning(void)
{
  return TMR_IsRunning(&TMR1_Resources);
}

static TMR_STATUS TMR1_SetValue(uint32_t value)
{
  return TMR_SetValue(&TMR1_Resources, value);
}

static TMR_STATUS TMR1_SetReloadValue(uint32_t reload_value)
{
  return TMR_SetReloadValue(&TMR1_Resources, reload_value);
}

static uint32_t TMR1_GetValue(void)
{
  return TMR_GetValue(&TMR1_Resources);
}

void TMR1_IRQHandler(void)
{
  TMR_IRQHandler(&TMR1_Resources);
}

TZ10XX_DRIVER_TMR Driver_TMR1 = {
  TMRX_GetVersion,
  TMRX_GetCapabilities,
  TMR1_Initialize,
  TMR1_Uninitialize,
  TMR1_PowerControl,
  TMR1_Configure,
  TMR1_Start,
  TMR1_Stop,
  TMR1_IsRunning,
  TMR1_SetValue,
  TMR1_SetReloadValue,
  TMR1_GetValue,
  TMRX_ConfigureCapture,
  TMRX_EnableCapture,
  TMRX_GetCapturedValue,
  TMRX_EnableCompare,
  TMRX_SetCompareValue,
  TMRX_ConfigureTFF,
  TMRX_EnableTFF,
  TMRX_ConfigureEventCounter,
  TMRX_UnconfigureEventCounter,
  TMRX_EnableEventCounter
};
