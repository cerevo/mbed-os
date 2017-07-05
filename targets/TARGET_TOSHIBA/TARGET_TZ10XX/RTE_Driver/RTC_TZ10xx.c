/**
 * @file RTC_TZ10xx.c
 * @brief TZ10xx RTC driver
 * @date $Date:: 2015-04-16 09:45:43 +0900 #$
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
#include "RTC_TZ10xx.h"
#include "PMU_TZ10xx.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

#if RTE_RTC

#define RTC_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 29)   /* driver version */

#define RTC_ITRPT_MASK 0x10000000

typedef struct {
  bool init;
  struct {
    uint32_t intvalen;
    uint32_t pren;
    uint32_t alen;
  } rtc_ctrl;
  RTC_SignalEvent_t callback_intval;
  RTC_SignalEvent_t callback_alarm;
  RTC_SignalEvent_t callback_period;
  uint32_t prevented_irq;
} RTC_INFO;

typedef struct {
  rtclv_Type *dev;
  IRQn_Type irq; /* IRQ number */
  RTC_INFO *info;
} const RTC_RESOURCES;

static RTC_INFO RTC0_Info;

static RTC_RESOURCES RTC0_Resources = {
  rtclv,
  RTC_IRQn,
  &RTC0_Info
};

/**
 * @brief Wait a while by busy loop
 * @param[in] usec Wait time in microseconds
 */
static void usleep(uint32_t usec)
{
  /* It assumes 8 cycles per loop */
  int32_t t;
  for (t = (SystemCoreClock / 1000000) * usec; t > 0; t -= 8) {
    __NOP();
    __NOP();
    __NOP();
    __NOP();
  }
}

/* bcd -> decimal */
static uint32_t bcd_decode(uint32_t bcd)
{
  return bcd - (bcd >> 4) * 6;
}

/* decimal -> bcd */
static uint32_t bcd_encode(uint32_t dec)
{
  return dec + (dec / 10) * 6;
}

static void set_disable_irq(void)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;

	if(rsc->info->prevented_irq & RTC_ITRPT_MASK) return;
	
	rsc->info->prevented_irq =  __get_PRIMASK();
  __disable_irq();
}

static void set_enable_irq(void)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;

	if(!rsc->info->prevented_irq) {
		rsc->info->prevented_irq = 1;
    __enable_irq();
	}
}
	
static void poling_rtc_ctrl_wip(void)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
	volatile uint32_t tmp;
	 do {
		 tmp = rsc->dev->RTC_CTRL_WIP;
	 } while(tmp);
}
	
static void set_intclr(volatile uint32_t *intclr)
{
	set_disable_irq();
	poling_rtc_ctrl_wip();
  *intclr = 1;
	*((uint32_t *)0x4280027c) = 0;  // rtl_ctrl_bit31  bit band align
	set_enable_irq();
}

static void set_rtc_ctrl(uint32_t rst)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  
	set_disable_irq();
	poling_rtc_ctrl_wip();
  rsc->dev->RTC_CTRL = rst
                      | 0xa0u /* TMEN | HR24_12 */
                      | (rsc->info->rtc_ctrl.intvalen << 9)
                      | (rsc->info->rtc_ctrl.pren << 3)
                      | (rsc->info->rtc_ctrl.alen << 1);
	set_enable_irq();
  if (rst != 0) {
    usleep(400); /* 12clk@32kHz + margin */
    while ((rsc->dev->RTC_CTRL & rst) != 0) {
      /* DO NOTHING */
    }
  } else {
    usleep(150); /* 4clk@32kHz + margin */
  }
}

static void null_callback(RTC_EVENT e)
{
  /* DO NOTHING */
}

/**
 @brief Get driver version
 @return @ref ARM_DRV_VERSION
 */
static ARM_DRIVER_VERSION RTC_GetVersion(void)
{
  static const ARM_DRIVER_VERSION driver_version = {
    RTC_API_VERSION,
    RTC_DRV_VERSION
  };
  return driver_version;
}

/**
 @brief Initialize RTC interface
 @return execution status @ref RTC_STATUS
 */
static RTC_STATUS RTC_Initialize(void)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  
  if (rsc->info->init) {
    return RTC_ERROR;
  }
  
  rsc->info->callback_intval = null_callback;
  rsc->info->callback_alarm = null_callback;
  rsc->info->callback_period = null_callback;
  
  /* Negate reset */
  Driver_PMU.EnableModule(PMU_MODULE_RTCLV, 1);
  
  /* Initialize RTC_CTRL register */
  rsc->info->rtc_ctrl.intvalen = 0;
  rsc->info->rtc_ctrl.pren = 0;
  rsc->info->rtc_ctrl.alen = 0;
  usleep(400); /* 12clk@32kHz + margin */
  set_rtc_ctrl(0);
  
  /* Clear interrupt flags */
  set_intclr(&(rsc->dev->RTC_INTVAL_INTCLR));
  set_intclr(&(rsc->dev->RTC_PERIOD_INTCLR));
  set_intclr(&(rsc->dev->RTC_ALARM_INTCLR));
  while (rsc->dev->RTC_MIS != 0) {
    /* DO NOTHING */
  }
  
  /* Enable RTC IRQ */
  NVIC_ClearPendingIRQ(rsc->irq);
  NVIC_EnableIRQ(rsc->irq);
  
	rsc->info->prevented_irq = 1;

  rsc->info->init = true;
 
  return RTC_OK;
}

/**
 @brief De-initialize RTC interface
 @return execution status @ref RTC_STATUS
 */
static RTC_STATUS RTC_Uninitialize()
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  
  if (!rsc->info->init) {
    return RTC_ERROR;
  }
  
  /* Assert reset */
  Driver_PMU.EnableModule(PMU_MODULE_RTCLV, 0);
  
  /* Disable RTC IRQ */
  NVIC_ClearPendingIRQ(rsc->irq);
  NVIC_DisableIRQ(rsc->irq);
  
  rsc->info->init = false;
  
  return RTC_OK;
}

/**
 @brief Control RTC interface power.
 @param[in] state power state
 @return execution status @ref RTC_STATUS
 @note This API does nothing since RTC has no power state.
 */
static RTC_STATUS RTC_PowerControl(ARM_POWER_STATE state)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  
  if (!rsc->info->init) {
    return RTC_ERROR;
  }
  
  return RTC_OK;
}

/**
 @brief Set date and time to RTC
 @param[in] time date and time to set
 @return execution status @ref RTC_STATUS
 */
static RTC_STATUS RTC_SetTime(const RTC_TIME *time)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  const uint8_t max_mday[] = {0, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  
  if (!rsc->info->init) {
    return RTC_ERROR;
  }
  
  if ((time->year > 99)
      || (time->mon == 0) || (time->mon > 12)
      || (time->mday == 0) || (time->mday > max_mday[time->mon])
      || (time->wday > 6)
      || (time->hour > 23)
      || (time->min > 59)
      || (time->sec > 59)) {
    return RTC_ERROR;
  }
  
  /* wait until previous TMRST operation finish */
  while (rsc->dev->RTC_CTRL_b.TMRST == 1) {
    /* DO NOTHING */
  }
  
  rsc->dev->RTC_TIMESET1 = bcd_encode(time->sec)
                          | (bcd_encode(time->min) << 8);
  rsc->dev->RTC_TIMESET2 = bcd_encode(time->hour)
                          | (bcd_encode(time->mday) << 8);
  rsc->dev->RTC_TIMESET3 = bcd_encode(time->mon)
                          | (time->wday << 5)
                          | (bcd_encode(time->year) << 8);
  rsc->dev->RTC_TIMESET4 = time->year & 3;
  
  set_rtc_ctrl(1u << 6); /* TMRST */
 	
  return RTC_OK;
}

/**
 @brief Get current date and time from RTC
 @param[in] time
 @return execution status @ref RTC_STATUS
 */
static RTC_STATUS RTC_GetTime(RTC_TIME *time)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  uint32_t t1[2] = {0, 0};
  uint32_t t2[2] = {0, 0};
  uint32_t t3[2] = {0, 0};
  int i = 0;
  
  if (!rsc->info->init) {
    return RTC_ERROR;
  }
  
  /* wait until load & adjust finish */
  while ((rsc->dev->RTC_CTRL_b.TMRST == 1) || (rsc->dev->RTC_CTRL_b.ADJUST == 1)) {
    /* DO NOTHING */
  }
  
  /* read CURRENTn registers twice and compare them. */
  do {
    i = 1 - i;
    t1[i] = rsc->dev->RTC_CURRENT1;
    t2[i] = rsc->dev->RTC_CURRENT2;
    t3[i] = rsc->dev->RTC_CURRENT3;
  } while ((t1[0] != t1[1]) || (t2[0] != t2[1]) || (t3[0] != t3[1]));
  
  time->sec  = bcd_decode((t1[0]     ) & 0x7f);
  time->min  = bcd_decode((t1[0] >> 8) & 0x7f);
  time->hour = bcd_decode((t2[0]     ) & 0x3f);
  time->mday = bcd_decode((t2[0] >> 8) & 0x3f);
  time->mon  = bcd_decode((t3[0]     ) & 0x1f);
  time->year = bcd_decode((t3[0] >> 8) & 0xff);
  time->wday = bcd_decode((t3[0] >> 5) & 0x07);
  
  return RTC_OK;
}

/**
 @brief Set alarm event
 @param[in] time alarm date and time
 @param[in] callback a function when alarm event occurs
 @return execution status @ref RTC_STATUS
 */
static RTC_STATUS RTC_SetAlarm(
  const RTC_TIME *time,
  RTC_SignalEvent_t cb_event)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  
  if (!rsc->info->init) {
    return RTC_ERROR;
  }
  
  if ((time->mday == 0) || (time->mday > 31)
      || (time->wday > 6)
      || (time->hour > 23)
      || (time->min > 59)) {
    return RTC_ERROR;
  }
      
  if (cb_event == NULL) {
    rsc->info->callback_alarm = null_callback;
  } else {
    rsc->info->callback_alarm = cb_event;
  }
  
  /* wait until previous ALRST operation finish */
  while (rsc->dev->RTC_CTRL_b.ALRST == 1) {
    /* DO NOTHING */
  }
  
  rsc->dev->RTC_ALARM1 = bcd_encode(time->min) << 8;
  rsc->dev->RTC_ALARM2 = bcd_encode(time->hour)
                        | (bcd_encode(time->mday) << 8);
  rsc->dev->RTC_ALARM3 = time->wday << 5;
  
  rsc->info->rtc_ctrl.alen = 1;
  set_rtc_ctrl(1u << 0); /* ALRST */
  set_intclr(&(rsc->dev->RTC_ALARM_INTCLR));
	while (rsc->dev->RTC_MIS_b.ALARM_INTMASKST != 0) {
    /* DO NOTHING */
  }
  rsc->dev->RTC_INTMASK_b.ALARM_INTMASK = 1;
  
  return RTC_OK;
}

/**
 @brief Cancel alarm event
 @return execution status @ref RTC_STATUS
 */
static RTC_STATUS RTC_ClearAlarm(void)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  
  if (!rsc->info->init) {
    return RTC_ERROR;
  }
  
  rsc->info->rtc_ctrl.alen = 0;
  set_rtc_ctrl(0);
  rsc->dev->RTC_INTMASK_b.ALARM_INTMASK = 0;
  set_intclr(&(rsc->dev->RTC_ALARM_INTCLR));
	while (rsc->dev->RTC_MIS_b.ALARM_INTMASKST != 0) {
    /* DO NOTHING */
  }
  rsc->info->callback_alarm = null_callback;
  
  return RTC_OK;
}

/**
 @brief Set periodic event
 @param[in] mode periodic mode
 @param[in] callback a function when periodic event occurs
 @return execution status @ref RTC_STATUS
 */
static RTC_STATUS RTC_SetPeriodicInterrupt(
  RTC_PERIOD mode,
  RTC_SignalEvent_t cb_event)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  
  if (!rsc->info->init) {
    return RTC_ERROR;
  }
  
  if (!((mode >= RTC_PERIOD_EVERY_SECOND)
     && (mode <= RTC_PERIOD_EVERY_MONTH)) &&
      !((mode >= RTC_PERIOD_EVERY_1_2_SECOND)
     && (mode <= RTC_PERIOD_EVERY_1_256_SECOND))) {
    return RTC_ERROR;
  }
  
  if (cb_event == NULL) {
    rsc->info->callback_period = null_callback;
  } else {
    rsc->info->callback_period = cb_event;
  }
  
  /* wait until previous PRRST operation finish */
  while (rsc->dev->RTC_CTRL_b.PRRST == 1) {
    /* DO NOTHING */
  }
  
  rsc->dev->RTC_PERIODIC = (uint32_t)mode;
  rsc->info->rtc_ctrl.pren = 1;
  set_rtc_ctrl(1u << 2); /* PRRST */
  set_intclr(&(rsc->dev->RTC_PERIOD_INTCLR));
  while (rsc->dev->RTC_MIS_b.PERIOD_INTMASKST != 0) {
    /* DO NOTHING */
  }
  rsc->dev->RTC_INTMASK_b.PERIOD_INTMASK = 1;
  
  return RTC_OK;
}

/**
 @brief Cancel periodic event
 @return execution status @ref RTC_STATUS
 */
static RTC_STATUS RTC_ClearPeriodicInterrupt(void)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  
  if (!rsc->info->init) {
    return RTC_ERROR;
  }
  
  rsc->info->rtc_ctrl.pren = 0;
  set_rtc_ctrl(0);
  rsc->dev->RTC_INTMASK_b.PERIOD_INTMASK = 0;
  set_intclr(&(rsc->dev->RTC_PERIOD_INTCLR));
  while (rsc->dev->RTC_MIS_b.PERIOD_INTMASKST != 0) {
    /* DO NOTHING */
  }
  rsc->info->callback_period = null_callback;
  
  return RTC_OK;
}

/**
 @brief Set interval event
 @param[in] ticks_at_2048hz event interval
 @param[in] callback a function when interval event occurs
 @return execution status @ref RTC_STATUS
 */
static RTC_STATUS RTC_SetIntervalInterrupt(
  uint32_t ticks_at_2048hz,
  RTC_SignalEvent_t cb_event)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  
  if (!rsc->info->init) {
    return RTC_ERROR;
  }
  
  if ((ticks_at_2048hz < 2) || (ticks_at_2048hz > 65536)) {
    return RTC_ERROR;
  }
  
  if (cb_event == NULL) {
    rsc->info->callback_intval = null_callback;
  } else {
    rsc->info->callback_intval = cb_event;
  }
  
  /* wait until previous INTVALRST operation finish */
  while (rsc->dev->RTC_CTRL_b.INTVALRST == 1) {
    /* DO NOTHING */
  }
  
  rsc->dev->RTC_INTVAL_SET = ticks_at_2048hz - 1u;
  rsc->info->rtc_ctrl.intvalen = 0;
  set_rtc_ctrl(1u << 8); /* INTVALRST */
  set_intclr(&(rsc->dev->RTC_INTVAL_INTCLR));
	while (rsc->dev->RTC_MIS_b.INTVAL_INTMASKST != 0) {
    /* DO NOTHING */
  }
  rsc->info->rtc_ctrl.intvalen = 1;
  set_rtc_ctrl(0);
	rsc->dev->RTC_INTMASK_b.INTVAL_INTMASK = 1;
  
  return RTC_OK;
}

/**
 @brief Cancel interval event
 @return execution status @ref RTC_STATUS
 */
static RTC_STATUS RTC_ClearIntervalInterrupt(void)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  
  if (!rsc->info->init) {
    return RTC_ERROR;
  }
  
  rsc->info->rtc_ctrl.intvalen = 0;
  set_rtc_ctrl(0);
  rsc->dev->RTC_INTMASK_b.INTVAL_INTMASK = 0;
  set_intclr(&(rsc->dev->RTC_INTVAL_INTCLR));
  while (rsc->dev->RTC_MIS_b.INTVAL_INTMASKST != 0) {
    /* DO NOTHING */
  }
  rsc->info->callback_intval = null_callback;
  
  return RTC_OK;
}

/**
 @brief Get interval counter
 @return value of counter
 */
static uint32_t RTC_GetIntervalCounter(void)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  
  if (!rsc->info->init) {
    return 0;
  }
  
   /* wait until load finish */
  while (rsc->dev->RTC_CTRL_b.INTVALRST == 1) {
    /* DO NOTHING */
  }
 
  return rsc->dev->RTC_INTVAL_TMR_b.COUNT_VAL;
}

void RTC_IRQHandler(void)
{
  RTC_RESOURCES *rsc = &RTC0_Resources;
  uint32_t mis = rsc->dev->RTC_MIS;

	rsc->info->prevented_irq |= RTC_ITRPT_MASK;
  
 /* interval interrupt */
  if ((mis & (1u << 2)) != 0) {
    set_intclr(&(rsc->dev->RTC_INTVAL_INTCLR));
    rsc->info->callback_intval(RTC_EVENT_INTERVAL);
  }
  /* periodic interrupt */
  if ((mis & (1u << 1)) != 0) {
    set_intclr(&(rsc->dev->RTC_PERIOD_INTCLR));
    rsc->info->callback_period(RTC_EVENT_PERIODIC);
  }
  /* alarm interrupt */
  if ((mis & (1u << 0)) != 0) {
    RTC_SignalEvent_t callback = rsc->info->callback_alarm;
    RTC_ClearAlarm();
   (*callback)(RTC_EVENT_ALARM);
  }
 
	rsc->info->prevented_irq &= ~(RTC_ITRPT_MASK);
}

TZ10XX_DRIVER_RTC Driver_RTC = {
  RTC_GetVersion,
  RTC_Initialize,
  RTC_Uninitialize,
  RTC_PowerControl,
  RTC_SetTime,
  RTC_GetTime,
  RTC_SetAlarm,
  RTC_ClearAlarm,
  RTC_SetPeriodicInterrupt,
  RTC_ClearPeriodicInterrupt,
  RTC_SetIntervalInterrupt,
  RTC_ClearIntervalInterrupt,
  RTC_GetIntervalCounter
};


#endif /* RTE_RTC */
