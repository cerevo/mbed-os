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
#include <stddef.h>
#include "us_ticker_api.h"
#include "PeripheralNames.h"
#include "TMR_TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "pmu.h"

#define MAX_COUNTER 0xFFFFFFFF
#define US_PER_SECOND 1000000

/* TMR Driver interface */
extern TZ10XX_DRIVER_TMR Driver_TMR0;
extern TZ10XX_DRIVER_TMR Driver_TMR1;

/* PMU Driver interface */
extern TZ10XX_DRIVER_PMU Driver_PMU;

static uint32_t ticks_per_us = 0;
static uint32_t max_us_ticker = 0; /* maximum us ticker of TMR0/TMR1 */

/* Accumulate us ticker */
static int acc_us_ticker = 0;

/* Is ticker initialized yet */
static int us_ticker_inited = 0;

/** Static function callback for tmr0
* @remarks
* 
* @param e  tmr event.
* 
* @returns
*   NONE
*/
static void callback_tmr0(TMR_EVENT e)
{
    acc_us_ticker += max_us_ticker;
}

/** Static function callback for tmr1
* @remarks
* 
* @param e  tmr event.
* 
* @returns
*   NONE
*/
static void callback_tmr1(TMR_EVENT e)
{
    /* Ticker interrupt handle */
    us_ticker_irq_handler();
}

/** Initialize the Timer peripheral module.
 *  @remarks
 *    Use CMSIS Timer Driver:
 *      - TMR0_Initialize
 *      - TMR1_Initialize
 *      - TMR0_PowerControl
 *      - TMR1_PowerControl
 *      - TMR0_Configure
 *      - TMR1_Configure
 *      - TMR0_Start
 *
 *  @param NONE
 *
 *  @returns
 *    NONE.
 */
void us_ticker_init(void)
{
    /* Check ticker initialize */
    if (us_ticker_inited) {
        return;
    }
    PmuInit();
    /* Determine number of ticker per microseconds */
    ticks_per_us = Driver_PMU.GetFrequency(PMU_CD_PPIER0) / US_PER_SECOND;

    /* Determine maximum us ticker of TMR0/TMR1 */
    max_us_ticker = MAX_COUNTER/ticks_per_us;

    /* Initialize TMR0, TMR1 */
    /* Counter of TMR0 and TMR1 count down to Zero */
    Driver_TMR0.Initialize(callback_tmr0, (1u << TMR_EVENT_BASE_COUNTER));
    Driver_TMR1.Initialize(callback_tmr1, (1u << TMR_EVENT_BASE_COUNTER));

    /* Set power full */
    Driver_TMR0.PowerControl(ARM_POWER_FULL);
    Driver_TMR1.PowerControl(ARM_POWER_FULL);

    /* Configure TMR0, TMR1: 32 bit counter */
    /* Whenever counter of TMR0 reaches Zero, load TMR0 reload max value (0xFFFF_FFFF) to the counter */
    /* Whenever counter of TMR1 reaches Zero, load TMR1 reload value to the counter */
    Driver_TMR0.Configure(32, TMR_COUNT_MODE_FREE_RUN, 1);
    Driver_TMR1.Configure(32, TMR_COUNT_MODE_PERIODIC, 1);

    /* TMR0 start from max counter value */
    Driver_TMR0.Start(MAX_COUNTER - 1);

    /* Ticker already initialize */
    us_ticker_inited = 1;
}

/** Get current tick of Ticker.
 *  @remarks
 *    Use CMSIS Timer Driver:
 *      - TMR0_GetValue
 *
 *  @param NONE
 *
 *  @returns
 *    Current tick.
 */
uint32_t us_ticker_read()
{
    uint32_t ret_val = 0;
    uint32_t us_ticker = 0;

    if (!us_ticker_inited) {
        us_ticker_init();
    }

    /* Get us ticker from TMR0 */
    us_ticker = (MAX_COUNTER - Driver_TMR0.GetValue())/ticks_per_us;

    ret_val = acc_us_ticker + us_ticker ;

    return ret_val;
}

/** Set interrupt for Ticker.
 *  @remarks
 *    Use CMSIS Timer Driver:
 *      - TMR1_IsRunning()
 *      - TMR1_SetValue()
 *      - TMR1_SetReloadValue()
 *      - TMR1_Start()
 *
 *  @param  timestamp           The time set to raise an event
 *
 *  @returns
 *    NONE.
 */
void us_ticker_set_interrupt(timestamp_t timestamp)
{
    uint32_t timer_value = 0;
    int delta = 0;
    if (!us_ticker_inited) {
        us_ticker_init();
    }

    delta = (int)(timestamp - us_ticker_read());
    if (delta <= 0) {
        /* Ticker interrupt handle */
        us_ticker_irq_handler();
        return;
    }

    timer_value = ((uint32_t)delta) * ticks_per_us;

    /* Enable interrupt of TMR1 */
    tmr->TIMER1CONTROL = (tmr->TIMER1CONTROL) | (1u << 5);

    if (!Driver_TMR1.IsRunning()) {
        /* Set current counter to timer_value
         * And set Reload value to timer_value
         * And start timer
         */
        Driver_TMR1.Start(timer_value);
    } else {
        /* Set current counter to timer_value
         * And set Reload value to timer_value
         */
        Driver_TMR1.SetValue(timer_value);
    }
}

/** Disable interrupt of Ticker.
 *  @remarks
 *    Use CMSIS-CORE:
 *      - NVIC_ClearPendingIRQ
 *      - NVIC_DisableIRQ
 *
 *  @param NONE
 *
 *  @returns
 *    NONE.
 */
void us_ticker_disable_interrupt(void)
{
    /* Disable interrupt of TMR1
     * Bit 5 value
     * 1: Interrupt enable 0: Interrupt disable.
     */
    tmr->TIMER1CONTROL = (tmr->TIMER1CONTROL) & (~(1u << 5));
}

/** Clear interrupt of Ticker.
 *
 *  @param NONE
 *
 *  @returns
 *    NONE.
 */
void us_ticker_clear_interrupt(void)
{
    /* Clear interrupt of TMR1 */
    tmr->TIMER1INTCLR = 1;
}
