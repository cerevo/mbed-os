/* mbed Microcontroller Library
 * Copyright 2017, Cerevo Inc. 
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "device.h"
#if DEVICE_LOWPOWERTIMER
#include "mbed_error.h"
#include "TMR_TZ10xx.h"
#include "RTC_TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "rtc_api.h"
#include "lp_ticker_api.h"

/* 
 * When adding a new event, if the new time fits within ADVTMR 16bit counter,
 * its interrupt is set only, otherwise RTC seconds alarm is used, and the
 * leftover is for ADVTMR timer.
 */

#define ADVTMR_MAX 0xFFFE
#define RTC_TICK_PR_SEC 2048
#define RTC_INTERVAL_MAX_SEC 32
#define RTC_INTERVAL_MAX_TICKS 65536
#define US_PER_SECOND 1000000U
#define ADVTMR_NUM_DIV 6            /* from 4->1024 */

#define USEC_TO_ADVTMR_TICKS(us, cpuFreq, advtmrDiv) (uint64_t)((uint64_t)us * (cpuFreq / advtmrDiv) / 1000000U)
#define ADVTMR_MAX_US_DIV(cpuFreq, advtmrDiv)      (uint64_t)(((uint64_t)US_PER_SECOND * ADVTMR_MAX * advtmrDiv)/ cpuFreq)
/* RTC Driver interface */
extern TZ10XX_DRIVER_RTC Driver_RTC;
/* RTC Driver interface */
extern TZ10XX_DRIVER_TMR Driver_ADVTMR0;
/* PMU Driver interface */
extern TZ10XX_DRIVER_PMU Driver_PMU;
/* Accumulate lp ticker in seconds */
static int acc_lp_sec = 0;
static int lp_ticker_inited = 0;

static void rtc_interval_callback(RTC_EVENT e)
{
    acc_lp_sec += RTC_INTERVAL_MAX_SEC;
}

static void advtmr0_callback(TMR_EVENT e)
{
    lp_ticker_irq_handler();
}

/** Initialize LowPower ticker use RTC for counter
 *
 *  @param  NONE.
 *
 *  @returns
 *    NONE.
 */
void lp_ticker_init(void)
{
    if (lp_ticker_inited) {
        return;
    }
    rtc_init();
    Driver_RTC.SetIntervalInterrupt(RTC_INTERVAL_MAX_TICKS, rtc_interval_callback);
    Driver_ADVTMR0.Initialize(advtmr0_callback, (1u << TMR_EVENT_BASE_COUNTER));
    lp_ticker_inited = 1;
}

/** Read number ticks of LowPower ticker.
 *
 *  @param  NONE.
 *
 *  @returns
 *    Number ticks.
 */
uint32_t lp_ticker_read(void)
{
    uint32_t microseconds = 0;
    uint32_t lp_ticks = 0;

    if (!lp_ticker_inited) {
        lp_ticker_init();
    }

    lp_ticks = RTC_INTERVAL_MAX_TICKS - rtclv->RTC_INTVAL_TMR_b.COUNT_VAL;
    microseconds = ((acc_lp_sec * US_PER_SECOND) + (((uint64_t)lp_ticks * US_PER_SECOND) / RTC_TICK_PR_SEC));
    return microseconds;
}

/** Set LowPower ticker interrupt.
 *
 *  @param  timestamp       Time for interrupt occur.
 *  @note   Interrupt time depends of ADV Timer 16 solution.
 *  @returns
 *    Number ticks.
 */
void lp_ticker_set_interrupt(timestamp_t timestamp)
{
    uint32_t now, ticks_us, ticks;
    uint32_t pmu_clk, divisor;
    static const uint16_t advtmr_div[ADVTMR_NUM_DIV] = {4, 8, 32, 128, 512, 1024};

    now = lp_ticker_read();
    ticks_us = timestamp > now ? timestamp - now : (uint32_t)((uint64_t)timestamp + 0xFFFFFFFFu - now);
    pmu_clk = Driver_PMU.GetFrequency(PMU_CD_PPIER0);
    divisor = 0;

    for (uint8_t i=0; i < (ADVTMR_NUM_DIV -1); i++) {
        uint32_t t1 = ADVTMR_MAX_US_DIV(pmu_clk, advtmr_div[i]);
        uint32_t t2 = ADVTMR_MAX_US_DIV(pmu_clk, advtmr_div[i+1]);

        if(((t1 < ticks_us) &&(ticks_us <= t2)) || ((advtmr_div[i] == 4) && (ticks_us < t1))) {
            divisor = advtmr_div[i+1];
        }
    }

    if (divisor != 0) {
        if(Driver_ADVTMR0.IsRunning()) {
            Driver_ADVTMR0.Stop();
        }
        ticks = USEC_TO_ADVTMR_TICKS(ticks_us, pmu_clk, divisor);
        /* Start Timer */
        Driver_ADVTMR0.PowerControl(ARM_POWER_FULL);
        Driver_ADVTMR0.Configure(16, TMR_COUNT_MODE_ONE_SHOT, divisor);
        Driver_ADVTMR0.Start(ticks);
    } else {
        /* Out of range of ADV Timer */
        error("Out of range\n");
    }
}

/** Disable LowPower ticker interrupt.
 *
 *  @param  NONE.
 *
 *  @returns NONE.
 */
void lp_ticker_disable_interrupt(void)
{
    Driver_ADVTMR0.Stop();
}

/** Clear LowPower ticker interrupt.
 *
 *  @param  NONE.
 *
 *  @returns NONE.
 */
void lp_ticker_clear_interrupt(void)
{
    advtmr->T0INTCLR = 1u;
}
#endif /* DEVICE_LOWPOWERTIMER */
