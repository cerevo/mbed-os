/**
 * @file ADVTMR_TZ10xx.c
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

#define advtmr_reg_ch(reg) (*(&reg + rsc->ch_offset))

#define TZ1000_ADVTMR_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 41)   /* driver version */

typedef struct {
  bool init;
  ARM_POWER_STATE power_state;
  TMR_SignalEvent_t cb_event;
  uint32_t timer_size;
  uint32_t event_mask;
  uint32_t capture_value;
} ADVTMR_INFO;

typedef struct {
  uint32_t channel;
  ptrdiff_t ch_offset;
  IRQn_Type irq;
  IRQn_Type irq_cap;
  IRQn_Type irq_cmp;
  ADVTMR_INFO *info;
} const ADVTMR_RESOURCES;

static ADVTMR_INFO ADVTMR0_Info;
static ADVTMR_INFO ADVTMR1_Info;
static ADVTMR_INFO ADVTMR2_Info;
static ADVTMR_INFO ADVTMR3_Info;

static ADVTMR_RESOURCES ADVTMR0_Resources = {
  0,
  0,
  ADVTMR0_IRQn,
  ADVTMR0_CAP_IRQn,
  ADVTMR0_CMP_IRQn,
  &ADVTMR0_Info
};

static ADVTMR_RESOURCES ADVTMR1_Resources = {
  1,
  (&advtmr->T1LOAD - &advtmr->T0LOAD),
  ADVTMR1_IRQn,
  ADVTMR1_CAP_IRQn,
  ADVTMR1_CMP_IRQn,
  &ADVTMR1_Info
};

static ADVTMR_RESOURCES ADVTMR2_Resources = {
  2,
  (&advtmr->T2LOAD - &advtmr->T0LOAD),
  ADVTMR2_IRQn,
  ADVTMR2_CAP_IRQn,
  ADVTMR2_CMP_IRQn,
  &ADVTMR2_Info
};

static ADVTMR_RESOURCES ADVTMR3_Resources = {
  3,
  (&advtmr->T3LOAD - &advtmr->T0LOAD),
  ADVTMR3_IRQn,
  ADVTMR3_CAP_IRQn,
  ADVTMR3_CMP_IRQn,
  &ADVTMR3_Info
};

static TMR_STATUS ADVTMR_PowerControl(ADVTMR_RESOURCES *rsc, ARM_POWER_STATE state);

/**
 * @brief Check whether a value is valid for TIMERnLOAD or TIMERnBGLOAD
 * @param[in] value a value to set to TIMERnLOAD or TIMERnBGLOAD
 * @param[in] timer_size bit width of timer
 * @return 0=invalid, 1=valid
 */
static uint32_t IsCounterValid(uint32_t value, uint32_t timer_size)
{
  if ((value == 0) || (value > ((1u << timer_size) - 1))) {
    return 0;
  } else {
    return 1;
  }
}

/**
  @brief Get driver version.
  @return \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION ADVTMRX_GetVersion(void)
{
  static const ARM_DRIVER_VERSION driver_version = {
    TZ1000_TMR_API_VERSION,
    TZ1000_ADVTMR_DRV_VERSION
  };
  return driver_version;
}

/**
  @brief Get driver capabilities.
  @return \ref TMR_CAPABILITIES
*/
static TMR_CAPABILITIES ADVTMRX_GetCapabilities(void)
{
  static const TMR_CAPABILITIES caps = {
    1, /* capture */
    1, /* compare */
    1, /* tff */
    1, /* event_counter */
  };
  return caps;
}

/**
  @brief Initialize ADVTMR Driver Interface.
  @param[in]   cb_event    Pointer to \ref TMR_SignalEvent
  @param[in]   event_mask  Events that are reported through callbacks
  @return      execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_Initialize(ADVTMR_RESOURCES *rsc, TMR_SignalEvent_t cb_event, uint32_t event_mask)
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
  
  /*
   * ADVTMR driver handles capture events even if callback isn't needed.
   * The driver acquires capture counter value in interrupt handler.
   */
  NVIC_ClearPendingIRQ(rsc->irq_cap);
  NVIC_EnableIRQ(rsc->irq_cap);
  
  if ((event_mask & (1u << TMR_EVENT_COMPARE)) != 0) {
    NVIC_ClearPendingIRQ(rsc->irq_cmp);
    NVIC_EnableIRQ(rsc->irq_cmp);
  }
  rsc->info->init = true;
  rsc->info->timer_size = 16;
  rsc->info->capture_value = 0;
  ADVTMR_PowerControl(rsc, ARM_POWER_LOW);
  return TMR_OK;
}

/**
  @brief De-initialize ADVTMR Driver Interface.
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_Uninitialize(ADVTMR_RESOURCES *rsc)
{
  NVIC_ClearPendingIRQ(rsc->irq);
  NVIC_DisableIRQ(rsc->irq);
  NVIC_ClearPendingIRQ(rsc->irq_cap);
  NVIC_DisableIRQ(rsc->irq_cap);
  NVIC_ClearPendingIRQ(rsc->irq_cmp);
  NVIC_DisableIRQ(rsc->irq_cmp);
  ADVTMR_PowerControl(rsc, ARM_POWER_OFF);
  
  rsc->info->init = false;
  return TMR_OK;
}

/**
  @brief Control ADVTMR Driver Interface Power.
  @param[in] state Power state
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_PowerControl(ADVTMR_RESOURCES *rsc, ARM_POWER_STATE state)
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
    Driver_PMU.EnableModule(PMU_MODULE_ADVTMR, 1);
    Driver_PMU.EnableModule(PMU_MODULE_EVM, 1);
  }
  
  rsc->info->power_state = state;
  
  if (state == ARM_POWER_OFF) {
    /* (LOW or FULL) -> OFF */
    /* stop TMR clock if all channels are off */
    if ((ADVTMR0_Info.power_state == ARM_POWER_OFF)
        && (ADVTMR1_Info.power_state == ARM_POWER_OFF)
        && (ADVTMR2_Info.power_state == ARM_POWER_OFF)
        && (ADVTMR3_Info.power_state == ARM_POWER_OFF)) {
      Driver_PMU.EnableModule(PMU_MODULE_ADVTMR, 0);
      Driver_PMU.EnableModule(PMU_MODULE_EVM, 0);
    }
  }
  
  return TMR_OK;
}

/**
  @brief Configure ADVTMR
  @param[in] couter_bits
  @param[in] count_mode
  @param[in] divisor
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_Configure(
  ADVTMR_RESOURCES *rsc,
  uint32_t counter_bits,
  TMR_COUNT_MODE count_mode,
  uint32_t divisor)
{
  uint32_t one_shot_count;
  uint32_t timer_size;
  uint32_t clock_divid=0;
  uint32_t periodic_mode;
  static uint8_t  divid_tab[8]={0,1,2,3,5,7,9,10};
  
  if (!rsc->info->init) {
    return TMR_ERROR;
  }
  if (rsc->info->power_state == ARM_POWER_OFF) {
    return TMR_ERROR;
  }
  if ((advtmr_reg_ch(advtmr->T0CONTROL) & (1u << 7)) != 0) {
    /* Timer is running */
    return TMR_ERROR;
  }
  
  switch (counter_bits) {
  case 8:
    timer_size = 0;
    break;
  case 16:
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
  
  while(clock_divid< 8) {
    if((1u<<divid_tab[clock_divid]) == divisor) break;
    clock_divid++;
  }
  if(clock_divid > 7) {
    return TMR_ERROR;
  }
  
  advtmr_reg_ch(advtmr->T0CONTROL) = clock_divid
                                 | (timer_size << 3)
                                 | (one_shot_count << 4)
                                 | (periodic_mode << 5)
                                 | (1u << 6); /* interrupt_enable */
  rsc->info->timer_size = counter_bits;
  return TMR_OK;
}

/**
  @brief Start the timer
  @param[in] initial_value initial counter value
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_Start(ADVTMR_RESOURCES *rsc, uint32_t initial_value)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  if (!IsCounterValid(initial_value, rsc->info->timer_size)) {
    return TMR_ERROR;
  }
  advtmr_reg_ch(advtmr->T0LOAD) = initial_value;
  advtmr_reg_ch(advtmr->T0CONTROL) |= 1u << 7; /* enable = 1 */
  return TMR_OK;
}

/**
  @brief Stop the timer
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_Stop(ADVTMR_RESOURCES *rsc)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  advtmr_reg_ch(advtmr->T0CONTROL) &= ~(1u << 7); /* enable = 0 */
  return TMR_OK;
}

/**
  @brief Indicate whether the timer is running
  @return true if the timer is running, otherwise false
*/
static bool ADVTMR_IsRunning(ADVTMR_RESOURCES *rsc)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return false;
  }
  return (advtmr_reg_ch(advtmr->T0CONTROL) >> 7) & 1u; /* TnCONTROL.enable */
}

/**
  @brief Set value of timer immediately
  @param[in] value new value
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_SetValue(ADVTMR_RESOURCES *rsc, uint32_t value)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  if (!IsCounterValid(value, rsc->info->timer_size)) {
    return TMR_ERROR;
  }
  advtmr_reg_ch(advtmr->T0LOAD) = value;
  return TMR_OK;
}

/**
  @brief Set reload value of timer
  @param[in] value new value
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_SetReloadValue(ADVTMR_RESOURCES *rsc, uint32_t reload_value)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  if (!IsCounterValid(reload_value, rsc->info->timer_size)) {
    return TMR_ERROR;
  }
  advtmr_reg_ch(advtmr->T0BGLOAD) = reload_value;
  return TMR_OK;
}

/**
  @brief Get current value of timer
  @return current value of timer
*/
static uint32_t ADVTMR_GetValue(ADVTMR_RESOURCES *rsc)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return 0xffffffff;
  }
  return advtmr_reg_ch(advtmr->T0VALUE);
}

/**
  @brief Configure capture function
  @param[in] signal capture input signal
  @param[in] trigger capture condition (level/edge)
  @param[in] reload whether reload counter when capture event occurs
  @param[in] sync capture input synchronisation
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_ConfigureCapture(
  ADVTMR_RESOURCES *rsc, TMR_SIGNAL signal, TMR_CAPTURE_TRIGGER trigger, bool reload, bool sync)
{
  uint32_t capsel;
  
  if (rsc->info->power_state == ARM_POWER_OFF) {
    return TMR_ERROR;
  }
  
  if ((uint32_t)signal <= TMR_SIGNAL_TFF3) {
    if ((uint32_t)signal == TMR_SIGNAL_TFF0 + rsc->channel) {
      return TMR_ERROR;
    } else {
      capsel = (uint32_t)signal;
    }
  } else {
    uint32_t evm_event;
    volatile uint32_t *evm_reg = &evm->TRIGGER_CH0 + rsc->channel;

    if ((TMR_SIGNAL_INT_GPIO_24 > signal) || (TMR_SIGNAL_INT_RTC_INTERVAL < signal)) {
      return TMR_ERROR;
    }
    evm_event = signal - TMR_SIGNAL_INT_GPIO_24;

    evm->EVM_EN = 1;
    *evm_reg = 0;
    *evm_reg = (evm_event << 8);
    *evm_reg = (evm_event << 8) | 1u;
    capsel = 4 + rsc->channel;
  }
  
  if ((uint32_t)trigger > TMR_CAPTURE_TRIGGER_EDGE_BOTH) {
    return TMR_ERROR;
  }
  
  advtmr_reg_ch(advtmr->T0CAPRELOAD) = (reload != false);
  advtmr_reg_ch(advtmr->T0CAPCTRL) = trigger;
  advtmr_reg_ch(advtmr->T0CAPSYNC) = (sync != false);
  advtmr_reg_ch(advtmr->T0CAPSEL) = capsel;
  
  return TMR_OK;
}

/**
  @brief Enable or disable capture
  @param[in] enable true:enable false:disable
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_EnableCapture(ADVTMR_RESOURCES *rsc, bool enable)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  advtmr_reg_ch(advtmr->T0CAPINTEN) = (enable != false);
  advtmr_reg_ch(advtmr->T0CAPEN) = (enable != false);
  
  
  return TMR_OK;
}

/**
  @brief Get last captured timer value
  @return captured timer value
*/
static uint32_t ADVTMR_GetCapturedValue(ADVTMR_RESOURCES *rsc)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return 0xffffffffu;
  }
  
  return rsc->info->capture_value;
}

/**
  @brief Enable or disable compare
  @param[in] enable true:enable false:disable
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_EnableCompare(ADVTMR_RESOURCES *rsc, bool enable)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  
  advtmr_reg_ch(advtmr->T0CMPEN) = (enable != false);
  advtmr_reg_ch(advtmr->T0CMPINTEN) = (enable != false);
  
  return TMR_OK;
}

/**
  @brief Set compare value
  @param[in] value new compare value
  @param[in] load_at_terminal true:update the value when the timer counter becomes 0 false:update immediately
  @return execution status \ref TMR_STATUS
*/

static TMR_STATUS ADVTMR_SetCompareValue(ADVTMR_RESOURCES *rsc, uint32_t value, bool load_at_terminal)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  if (value > ((1u << rsc->info->timer_size) - 1)) {
    return TMR_ERROR;
  }
 
  if (load_at_terminal) {
    advtmr_reg_ch(advtmr->T0BGCOMPARE) = value;
  } else {
    advtmr_reg_ch(advtmr->T0COMPARE) = value;
  }
  
  return TMR_OK;
}

/**
  @brief Configure TFF output
  @param[in] mode output mode
  @param[in] init_high initial value true:high false:low 
  @param[in] invert output inversion
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_ConfigureTFF(ADVTMR_RESOURCES *rsc, TMR_TFF_MODE mode, bool init_high, bool invert)
{
  uint32_t ctrl;
  uint32_t zero;
  
  if (rsc->info->power_state == ARM_POWER_OFF) {
    return TMR_ERROR;
  }

  if (TMR_TFF_MODE_CMP_TERM_TOGGLE < mode) {
      return TMR_ERROR;
    }
  if(TMR_TFF_MODE_CMP_TERM_TOGGLE == mode) {
    ctrl = 3;
    zero = 1;
  } else {
    ctrl = (uint32_t) mode;
    zero =0 ;
  }
  
  advtmr_reg_ch(advtmr->T0TFFCTRL) = ctrl;
  advtmr_reg_ch(advtmr->T0TFFZERO) = zero;
  advtmr_reg_ch(advtmr->T0TFFINIT) = (init_high != 0);
  advtmr_reg_ch(advtmr->T0TFFINV) = (invert != 0);
  return TMR_OK;
}
  
/**
  @brief Enable or disable TFF output
  @param[in] enable true:enable false:disable
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_EnableTFF(ADVTMR_RESOURCES *rsc, bool enable)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  advtmr_reg_ch(advtmr->T0IOSEL) = (enable != 0);
  advtmr_reg_ch(advtmr->T0TFFEN) = (enable != 0);
  return TMR_OK;
}

/**
  @brief Configure event counter
  @param[in] event_en_mode
  @param[in] ev_sync
  @param[in] ev_delay
  @param[in] evcntsrc
  @param[in] evcnten
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_ConfigureEventCounter(ADVTMR_RESOURCES *rsc,
  TMR_EVENT_EN_MODE event_en_mode, bool ev_sync, bool ev_delay, TMR_SIGNAL evcntsrc, TMR_SIGNAL evcnten)
{
  uint32_t event_mode;
  uint32_t evcntsrc_sel;
  uint32_t evcnten_sel;
  uint32_t event_select = 0;
  
  if (rsc->info->power_state == ARM_POWER_OFF) {
    return TMR_ERROR;
  }
  
  /* Event_Select */
  if ((uint32_t)event_en_mode > TMR_EVENT_EN_MODE_EXT_EDGE) {
    return TMR_ERROR;
  }
  if ((uint32_t)event_en_mode > 0) {
    event_select = (uint32_t)event_en_mode + 1;
  }
  
  /* Event_Mode, EVCNTSRC_sel */
  if ((uint32_t)evcntsrc == ((uint32_t)TMR_SIGNAL_TFF0 + rsc->channel)) {
    return TMR_ERROR;
  } else if (evcntsrc == TMR_SIGNAL_PRESCALER) {
    event_mode = 2;
    evcntsrc_sel = 0;
  } else if (evcntsrc == TMR_SIGNAL_CASCADE) {
    event_mode = 3;
    evcntsrc_sel = 4 + rsc->channel;
  } else if ((uint32_t)evcntsrc <= TMR_SIGNAL_TFF3) {
    event_mode = 3;
    evcntsrc_sel = (uint32_t)evcntsrc;
  } else {
    return TMR_ERROR;
  }
  
  /* EVCNTEN_sel */
  if ((uint32_t)evcnten <= TMR_SIGNAL_TFF3) {
    evcnten_sel = (uint32_t)evcnten;
  } else {
    return TMR_ERROR;
  }
  
  advtmr_reg_ch(advtmr->T0CONTROL) =
    (advtmr_reg_ch(advtmr->T0CONTROL) & ~(3u << 8)) | (event_mode << 8);
  advtmr_reg_ch(advtmr->T0EVCONTROL) =
    (event_select << 4) | ((ev_sync != false) << 3) | ((ev_delay != false) << 1);
  advtmr_reg_ch(advtmr->T0EVSEL) = (evcntsrc_sel << 4) | evcnten_sel;
  
  return TMR_OK;
}

/**
  @brief Unconfigure event counter
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_UnconfigureEventCounter(ADVTMR_RESOURCES *rsc)
{
  if (rsc->info->power_state == ARM_POWER_OFF) {
    return TMR_ERROR;
  }
  /* clear Event_Mode */
  advtmr_reg_ch(advtmr->T0CONTROL) &= ~(3u << 8);
  return TMR_OK;
}  

/**
  @brief Enable or disable event counter
  @param[in] enable true:enable false:disable
  @return execution status \ref TMR_STATUS
*/
static TMR_STATUS ADVTMR_EnableEventCounter(ADVTMR_RESOURCES *rsc, bool enable)
{
  if (rsc->info->power_state != ARM_POWER_FULL) {
    return TMR_ERROR;
  }
  
  if (enable) {
    advtmr_reg_ch(advtmr->T0EVCONTROL) |= 1u;
  } else {
    advtmr_reg_ch(advtmr->T0EVCONTROL) &= ~1u;
  }
  return TMR_OK;
}

/**
 * @brief ADVTMR base counter interrupt handler
 */
static void ADVTMR_IRQHandler(ADVTMR_RESOURCES *rsc)
{
  if (advtmr_reg_ch(advtmr->T0MIS) != 0) {
    advtmr_reg_ch(advtmr->T0INTCLR) = 1;
    if ((advtmr_reg_ch(advtmr->T0CONTROL) & (1u << 4)) != 0) {
      ADVTMR_Stop(rsc);
    }
    if (rsc->info->cb_event != 0) {
      rsc->info->cb_event(TMR_EVENT_BASE_COUNTER);
    }
  }
}

/**
 * @brief ADVTMR capture interrupt handler
 */
static void ADVTMR_CAP_IRQHandler(ADVTMR_RESOURCES *rsc)
{
  if (advtmr_reg_ch(advtmr->T0CAPMIS) != 0) {
    rsc->info->capture_value = advtmr_reg_ch(advtmr->T0CAPTURE);
    advtmr_reg_ch(advtmr->T0CAPINTCLR) = 1;
    if (((rsc->info->event_mask & (1u << TMR_EVENT_CAPTURE)) != 0)
        && (rsc->info->cb_event != 0)) {
      rsc->info->cb_event(TMR_EVENT_CAPTURE);
    }
  }
}

/**
 * @brief ADVTMR compare interrupt handler
 */
static void ADVTMR_CMP_IRQHandler(ADVTMR_RESOURCES *rsc)
{
  if (advtmr_reg_ch(advtmr->T0CMPMIS) != 0) {
    advtmr_reg_ch(advtmr->T0CMPINTCLR) = 1;
    if (rsc->info->cb_event != 0) {
      rsc->info->cb_event(TMR_EVENT_COMPARE);
    }
  }
}

static TMR_STATUS ADVTMR0_Initialize(TMR_SignalEvent_t cb_event, uint32_t event_mask)
{
  return ADVTMR_Initialize(&ADVTMR0_Resources, cb_event, event_mask);
}

static TMR_STATUS ADVTMR0_Uninitialize(void)
{
  return ADVTMR_Uninitialize(&ADVTMR0_Resources);
}

static TMR_STATUS ADVTMR0_PowerControl(ARM_POWER_STATE state)
{
  return ADVTMR_PowerControl(&ADVTMR0_Resources, state);
}

static TMR_STATUS ADVTMR0_Configure(
  uint32_t counter_bits,
  TMR_COUNT_MODE count_mode,
  uint32_t divisor)
{
  return ADVTMR_Configure(&ADVTMR0_Resources, counter_bits, count_mode, divisor);
}

static TMR_STATUS ADVTMR0_Start(uint32_t initial_value)
{
  return ADVTMR_Start(&ADVTMR0_Resources, initial_value);
}

static TMR_STATUS ADVTMR0_Stop(void)
{
  return ADVTMR_Stop(&ADVTMR0_Resources);
}

static bool ADVTMR0_IsRunning(void)
{
  return ADVTMR_IsRunning(&ADVTMR0_Resources);
}

static TMR_STATUS ADVTMR0_SetValue(uint32_t value)
{
  return ADVTMR_SetValue(&ADVTMR0_Resources, value);
}

static TMR_STATUS ADVTMR0_SetReloadValue(uint32_t reload_value)
{
  return ADVTMR_SetReloadValue(&ADVTMR0_Resources, reload_value);
}

static uint32_t ADVTMR0_GetValue(void)
{
  return ADVTMR_GetValue(&ADVTMR0_Resources);
}

static TMR_STATUS ADVTMR0_ConfigureCapture(
  TMR_SIGNAL signal, TMR_CAPTURE_TRIGGER trigger, bool reload, bool sync)
{
  return ADVTMR_ConfigureCapture(&ADVTMR0_Resources, signal, trigger, reload, sync);
}

static TMR_STATUS ADVTMR0_EnableCapture(bool enable)
{
  return ADVTMR_EnableCapture(&ADVTMR0_Resources, enable);
}

static uint32_t ADVTMR0_GetCapturedValue(void)
{
  return ADVTMR_GetCapturedValue(&ADVTMR0_Resources);
}

static TMR_STATUS ADVTMR0_EnableCompare(bool enable)
{
  return ADVTMR_EnableCompare(&ADVTMR0_Resources, enable);
}

static TMR_STATUS ADVTMR0_SetCompareValue(uint32_t value, bool load_at_terminal) {
  return ADVTMR_SetCompareValue(&ADVTMR0_Resources, value, load_at_terminal);
}

static TMR_STATUS ADVTMR0_ConfigureTFF(TMR_TFF_MODE mode, bool init_high, bool invert)
{
  return ADVTMR_ConfigureTFF(&ADVTMR0_Resources, mode, init_high, invert);
}
  
static TMR_STATUS ADVTMR0_EnableTFF(bool enable)
{
  return ADVTMR_EnableTFF(&ADVTMR0_Resources, enable);
}  

static TMR_STATUS ADVTMR0_ConfigureEventCounter(TMR_EVENT_EN_MODE event_en_mode,
  bool ev_sync, bool ev_delay, TMR_SIGNAL evcntsrc, TMR_SIGNAL evcnten)
{
  return ADVTMR_ConfigureEventCounter(&ADVTMR0_Resources, event_en_mode, ev_sync, ev_delay, evcntsrc, evcnten);
}

static TMR_STATUS ADVTMR0_UnconfigureEventCounter(void)
{
  return ADVTMR_UnconfigureEventCounter(&ADVTMR0_Resources);
}

static TMR_STATUS ADVTMR0_EnableEventCounter(bool enable)
{
  return ADVTMR_EnableEventCounter(&ADVTMR0_Resources, enable);
}  

void ADVTMR0_IRQHandler(void)
{
  ADVTMR_IRQHandler(&ADVTMR0_Resources);
}

void ADVTMR0_CAP_IRQHandler(void)
{
  ADVTMR_CAP_IRQHandler(&ADVTMR0_Resources);
}

void ADVTMR0_CMP_IRQHandler(void)
{
  ADVTMR_CMP_IRQHandler(&ADVTMR0_Resources);
}

TZ10XX_DRIVER_TMR Driver_ADVTMR0 = {
  ADVTMRX_GetVersion,
  ADVTMRX_GetCapabilities,
  ADVTMR0_Initialize,
  ADVTMR0_Uninitialize,
  ADVTMR0_PowerControl,
  ADVTMR0_Configure,
  ADVTMR0_Start,
  ADVTMR0_Stop,
  ADVTMR0_IsRunning,
  ADVTMR0_SetValue,
  ADVTMR0_SetReloadValue,
  ADVTMR0_GetValue,
  ADVTMR0_ConfigureCapture,
  ADVTMR0_EnableCapture,
  ADVTMR0_GetCapturedValue,
  ADVTMR0_EnableCompare,
  ADVTMR0_SetCompareValue,
  ADVTMR0_ConfigureTFF,
  ADVTMR0_EnableTFF,
  ADVTMR0_ConfigureEventCounter,
  ADVTMR0_UnconfigureEventCounter,
  ADVTMR0_EnableEventCounter,
};

static TMR_STATUS ADVTMR1_Initialize(TMR_SignalEvent_t cb_event, uint32_t event_mask)
{
  return ADVTMR_Initialize(&ADVTMR1_Resources, cb_event, event_mask);
}

static TMR_STATUS ADVTMR1_Uninitialize(void)
{
  return ADVTMR_Uninitialize(&ADVTMR1_Resources);
}

static TMR_STATUS ADVTMR1_PowerControl(ARM_POWER_STATE state)
{
  return ADVTMR_PowerControl(&ADVTMR1_Resources, state);
}

static TMR_STATUS ADVTMR1_Configure(
  uint32_t counter_bits,
  TMR_COUNT_MODE count_mode,
  uint32_t divisor)
{
  return ADVTMR_Configure(&ADVTMR1_Resources, counter_bits, count_mode, divisor);
}

static TMR_STATUS ADVTMR1_Start(uint32_t initial_value)
{
  return ADVTMR_Start(&ADVTMR1_Resources, initial_value);
}

static TMR_STATUS ADVTMR1_Stop(void)
{
  return ADVTMR_Stop(&ADVTMR1_Resources);
}

static bool ADVTMR1_IsRunning(void)
{
  return ADVTMR_IsRunning(&ADVTMR1_Resources);
}

static TMR_STATUS ADVTMR1_SetValue(uint32_t value)
{
  return ADVTMR_SetValue(&ADVTMR1_Resources, value);
}

static TMR_STATUS ADVTMR1_SetReloadValue(uint32_t reload_value)
{
  return ADVTMR_SetReloadValue(&ADVTMR1_Resources, reload_value);
}

static uint32_t ADVTMR1_GetValue(void)
{
  return ADVTMR_GetValue(&ADVTMR1_Resources);
}

static TMR_STATUS ADVTMR1_ConfigureCapture(
  TMR_SIGNAL signal, TMR_CAPTURE_TRIGGER trigger, bool reload, bool sync)
{
  return ADVTMR_ConfigureCapture(&ADVTMR1_Resources, signal, trigger, reload, sync);
}

static TMR_STATUS ADVTMR1_EnableCapture(bool enable)
{
  return ADVTMR_EnableCapture(&ADVTMR1_Resources, enable);
}

static uint32_t ADVTMR1_GetCapturedValue(void)
{
  return ADVTMR_GetCapturedValue(&ADVTMR1_Resources);
}

static TMR_STATUS ADVTMR1_EnableCompare(bool enable)
{
  return ADVTMR_EnableCompare(&ADVTMR1_Resources, enable);
}

static TMR_STATUS ADVTMR1_SetCompareValue(uint32_t value, bool load_at_terminal) {
  return ADVTMR_SetCompareValue(&ADVTMR1_Resources, value, load_at_terminal);
}

static TMR_STATUS ADVTMR1_ConfigureTFF(TMR_TFF_MODE mode, bool init_high, bool invert)
{
  return ADVTMR_ConfigureTFF(&ADVTMR1_Resources, mode, init_high, invert);
}
  
static TMR_STATUS ADVTMR1_EnableTFF(bool enable)
{
  return ADVTMR_EnableTFF(&ADVTMR1_Resources, enable);
}  

static TMR_STATUS ADVTMR1_ConfigureEventCounter(TMR_EVENT_EN_MODE event_en_mode,
  bool ev_sync, bool ev_delay, TMR_SIGNAL evcntsrc, TMR_SIGNAL evcnten)
{
  return ADVTMR_ConfigureEventCounter(&ADVTMR1_Resources, event_en_mode, ev_sync, ev_delay, evcntsrc, evcnten);
}

static TMR_STATUS ADVTMR1_UnconfigureEventCounter(void)
{
  return ADVTMR_UnconfigureEventCounter(&ADVTMR1_Resources);
}

static TMR_STATUS ADVTMR1_EnableEventCounter(bool enable)
{
  return ADVTMR_EnableEventCounter(&ADVTMR1_Resources, enable);
}  

void ADVTMR1_IRQHandler(void)
{
  ADVTMR_IRQHandler(&ADVTMR1_Resources);
}

void ADVTMR1_CAP_IRQHandler(void)
{
  ADVTMR_CAP_IRQHandler(&ADVTMR1_Resources);
}

void ADVTMR1_CMP_IRQHandler(void)
{
  ADVTMR_CMP_IRQHandler(&ADVTMR1_Resources);
}

TZ10XX_DRIVER_TMR Driver_ADVTMR1 = {
  ADVTMRX_GetVersion,
  ADVTMRX_GetCapabilities,
  ADVTMR1_Initialize,
  ADVTMR1_Uninitialize,
  ADVTMR1_PowerControl,
  ADVTMR1_Configure,
  ADVTMR1_Start,
  ADVTMR1_Stop,
  ADVTMR1_IsRunning,
  ADVTMR1_SetValue,
  ADVTMR1_SetReloadValue,
  ADVTMR1_GetValue,
  ADVTMR1_ConfigureCapture,
  ADVTMR1_EnableCapture,
  ADVTMR1_GetCapturedValue,
  ADVTMR1_EnableCompare,
  ADVTMR1_SetCompareValue,
  ADVTMR1_ConfigureTFF,
  ADVTMR1_EnableTFF,
  ADVTMR1_ConfigureEventCounter,
  ADVTMR1_UnconfigureEventCounter,
  ADVTMR1_EnableEventCounter,
};

static TMR_STATUS ADVTMR2_Initialize(TMR_SignalEvent_t cb_event, uint32_t event_mask)
{
  return ADVTMR_Initialize(&ADVTMR2_Resources, cb_event, event_mask);
}

static TMR_STATUS ADVTMR2_Uninitialize(void)
{
  return ADVTMR_Uninitialize(&ADVTMR2_Resources);
}

static TMR_STATUS ADVTMR2_PowerControl(ARM_POWER_STATE state)
{
  return ADVTMR_PowerControl(&ADVTMR2_Resources, state);
}

static TMR_STATUS ADVTMR2_Configure(
  uint32_t counter_bits,
  TMR_COUNT_MODE count_mode,
  uint32_t divisor)
{
  return ADVTMR_Configure(&ADVTMR2_Resources, counter_bits, count_mode, divisor);
}

static TMR_STATUS ADVTMR2_Start(uint32_t initial_value)
{
  return ADVTMR_Start(&ADVTMR2_Resources, initial_value);
}

static TMR_STATUS ADVTMR2_Stop(void)
{
  return ADVTMR_Stop(&ADVTMR2_Resources);
}

static bool ADVTMR2_IsRunning(void)
{
  return ADVTMR_IsRunning(&ADVTMR2_Resources);
}

static TMR_STATUS ADVTMR2_SetValue(uint32_t value)
{
  return ADVTMR_SetValue(&ADVTMR2_Resources, value);
}

static TMR_STATUS ADVTMR2_SetReloadValue(uint32_t reload_value)
{
  return ADVTMR_SetReloadValue(&ADVTMR2_Resources, reload_value);
}

static uint32_t ADVTMR2_GetValue(void)
{
  return ADVTMR_GetValue(&ADVTMR2_Resources);
}

static TMR_STATUS ADVTMR2_ConfigureCapture(
  TMR_SIGNAL signal, TMR_CAPTURE_TRIGGER trigger, bool reload, bool sync)
{
  return ADVTMR_ConfigureCapture(&ADVTMR2_Resources, signal, trigger, reload, sync);
}

static TMR_STATUS ADVTMR2_EnableCapture(bool enable)
{
  return ADVTMR_EnableCapture(&ADVTMR2_Resources, enable);
}

static uint32_t ADVTMR2_GetCapturedValue(void)
{
  return ADVTMR_GetCapturedValue(&ADVTMR2_Resources);
}

static TMR_STATUS ADVTMR2_EnableCompare(bool enable)
{
  return ADVTMR_EnableCompare(&ADVTMR2_Resources, enable);
}

static TMR_STATUS ADVTMR2_SetCompareValue(uint32_t value, bool load_at_terminal) {
  return ADVTMR_SetCompareValue(&ADVTMR2_Resources, value, load_at_terminal);
}

static TMR_STATUS ADVTMR2_ConfigureTFF(TMR_TFF_MODE mode, bool init_high, bool invert)
{
  return ADVTMR_ConfigureTFF(&ADVTMR2_Resources, mode, init_high, invert);
}
  
static TMR_STATUS ADVTMR2_EnableTFF(bool enable)
{
  return ADVTMR_EnableTFF(&ADVTMR2_Resources, enable);
}  

static TMR_STATUS ADVTMR2_ConfigureEventCounter(TMR_EVENT_EN_MODE event_en_mode,
  bool ev_sync, bool ev_delay, TMR_SIGNAL evcntsrc, TMR_SIGNAL evcnten)
{
  return ADVTMR_ConfigureEventCounter(&ADVTMR2_Resources, event_en_mode, ev_sync, ev_delay, evcntsrc, evcnten);
}

static TMR_STATUS ADVTMR2_UnconfigureEventCounter(void)
{
  return ADVTMR_UnconfigureEventCounter(&ADVTMR2_Resources);
}

static TMR_STATUS ADVTMR2_EnableEventCounter(bool enable)
{
  return ADVTMR_EnableEventCounter(&ADVTMR2_Resources, enable);
}  

void ADVTMR2_IRQHandler(void)
{
  ADVTMR_IRQHandler(&ADVTMR2_Resources);
}

void ADVTMR2_CAP_IRQHandler(void)
{
  ADVTMR_CAP_IRQHandler(&ADVTMR2_Resources);
}

void ADVTMR2_CMP_IRQHandler(void)
{
  ADVTMR_CMP_IRQHandler(&ADVTMR2_Resources);
}

TZ10XX_DRIVER_TMR Driver_ADVTMR2 = {
  ADVTMRX_GetVersion,
  ADVTMRX_GetCapabilities,
  ADVTMR2_Initialize,
  ADVTMR2_Uninitialize,
  ADVTMR2_PowerControl,
  ADVTMR2_Configure,
  ADVTMR2_Start,
  ADVTMR2_Stop,
  ADVTMR2_IsRunning,
  ADVTMR2_SetValue,
  ADVTMR2_SetReloadValue,
  ADVTMR2_GetValue,
  ADVTMR2_ConfigureCapture,
  ADVTMR2_EnableCapture,
  ADVTMR2_GetCapturedValue,
  ADVTMR2_EnableCompare,
  ADVTMR2_SetCompareValue,
  ADVTMR2_ConfigureTFF,
  ADVTMR2_EnableTFF,
  ADVTMR2_ConfigureEventCounter,
  ADVTMR2_UnconfigureEventCounter,
  ADVTMR2_EnableEventCounter,
};

static TMR_STATUS ADVTMR3_Initialize(TMR_SignalEvent_t cb_event, uint32_t event_mask)
{
  return ADVTMR_Initialize(&ADVTMR3_Resources, cb_event, event_mask);
}

static TMR_STATUS ADVTMR3_Uninitialize(void)
{
  return ADVTMR_Uninitialize(&ADVTMR3_Resources);
}

static TMR_STATUS ADVTMR3_PowerControl(ARM_POWER_STATE state)
{
  return ADVTMR_PowerControl(&ADVTMR3_Resources, state);
}

static TMR_STATUS ADVTMR3_Configure(
  uint32_t counter_bits,
  TMR_COUNT_MODE count_mode,
  uint32_t divisor)
{
  return ADVTMR_Configure(&ADVTMR3_Resources, counter_bits, count_mode, divisor);
}

static TMR_STATUS ADVTMR3_Start(uint32_t initial_value)
{
  return ADVTMR_Start(&ADVTMR3_Resources, initial_value);
}

static TMR_STATUS ADVTMR3_Stop(void)
{
  return ADVTMR_Stop(&ADVTMR3_Resources);
}

static bool ADVTMR3_IsRunning(void)
{
  return ADVTMR_IsRunning(&ADVTMR3_Resources);
}

static TMR_STATUS ADVTMR3_SetValue(uint32_t value)
{
  return ADVTMR_SetValue(&ADVTMR3_Resources, value);
}

static TMR_STATUS ADVTMR3_SetReloadValue(uint32_t reload_value)
{
  return ADVTMR_SetReloadValue(&ADVTMR3_Resources, reload_value);
}

static uint32_t ADVTMR3_GetValue(void)
{
  return ADVTMR_GetValue(&ADVTMR3_Resources);
}

static TMR_STATUS ADVTMR3_ConfigureCapture(
  TMR_SIGNAL signal, TMR_CAPTURE_TRIGGER trigger, bool reload, bool sync)
{
  return ADVTMR_ConfigureCapture(&ADVTMR3_Resources, signal, trigger, reload, sync);
}

static TMR_STATUS ADVTMR3_EnableCapture(bool enable)
{
  return ADVTMR_EnableCapture(&ADVTMR3_Resources, enable);
}

static uint32_t ADVTMR3_GetCapturedValue(void)
{
  return ADVTMR_GetCapturedValue(&ADVTMR3_Resources);
}

static TMR_STATUS ADVTMR3_EnableCompare(bool enable)
{
  return ADVTMR_EnableCompare(&ADVTMR3_Resources, enable);
}

static TMR_STATUS ADVTMR3_SetCompareValue(uint32_t value, bool load_at_terminal) {
  return ADVTMR_SetCompareValue(&ADVTMR3_Resources, value, load_at_terminal);
}

static TMR_STATUS ADVTMR3_ConfigureTFF(TMR_TFF_MODE mode, bool init_high, bool invert)
{
  return ADVTMR_ConfigureTFF(&ADVTMR3_Resources, mode, init_high, invert);
}
  
static TMR_STATUS ADVTMR3_EnableTFF(bool enable)
{
  return ADVTMR_EnableTFF(&ADVTMR3_Resources, enable);
}  

static TMR_STATUS ADVTMR3_ConfigureEventCounter(TMR_EVENT_EN_MODE event_en_mode,
  bool ev_sync, bool ev_delay, TMR_SIGNAL evcntsrc, TMR_SIGNAL evcnten)
{
  return ADVTMR_ConfigureEventCounter(&ADVTMR3_Resources, event_en_mode, ev_sync, ev_delay, evcntsrc, evcnten);
}

static TMR_STATUS ADVTMR3_UnconfigureEventCounter(void)
{
  return ADVTMR_UnconfigureEventCounter(&ADVTMR3_Resources);
}

static TMR_STATUS ADVTMR3_EnableEventCounter(bool enable)
{
  return ADVTMR_EnableEventCounter(&ADVTMR3_Resources, enable);
}  

void ADVTMR3_IRQHandler(void)
{
  ADVTMR_IRQHandler(&ADVTMR3_Resources);
}

void ADVTMR3_CAP_IRQHandler(void)
{
  ADVTMR_CAP_IRQHandler(&ADVTMR3_Resources);
}

void ADVTMR3_CMP_IRQHandler(void)
{
  ADVTMR_CMP_IRQHandler(&ADVTMR3_Resources);
}

TZ10XX_DRIVER_TMR Driver_ADVTMR3 = {
  ADVTMRX_GetVersion,
  ADVTMRX_GetCapabilities,
  ADVTMR3_Initialize,
  ADVTMR3_Uninitialize,
  ADVTMR3_PowerControl,
  ADVTMR3_Configure,
  ADVTMR3_Start,
  ADVTMR3_Stop,
  ADVTMR3_IsRunning,
  ADVTMR3_SetValue,
  ADVTMR3_SetReloadValue,
  ADVTMR3_GetValue,
  ADVTMR3_ConfigureCapture,
  ADVTMR3_EnableCapture,
  ADVTMR3_GetCapturedValue,
  ADVTMR3_EnableCompare,
  ADVTMR3_SetCompareValue,
  ADVTMR3_ConfigureTFF,
  ADVTMR3_EnableTFF,
  ADVTMR3_ConfigureEventCounter,
  ADVTMR3_UnconfigureEventCounter,
  ADVTMR3_EnableEventCounter,
};
