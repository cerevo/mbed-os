/* mbed Microcontroller Library
 * Copyright 2017, Cerevo Inc. 
 * All rights reserved.
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

#include "rtc_api.h"
#include "btle_tz1em.h"
#include "RTC_TZ10xx.h"

extern TZ10XX_DRIVER_RTC Driver_RTC;

static TZ1EM_DEF(BTLE);
static bool btle_tz1em_initialized = false;

static void periodic_callback(RTC_EVENT e)
{
    /* do nothing */
}

/** Configure TZ1EM and initialize RTC periodic interrupt for BLE stack
 *
 *  @returns
 *    BTLE_TZ1EM_OK.      Success
 *    BTLE_TZ1EM_ERROR.   Error
 */
int btle_tz1em_init( void )
{
    tz1emRequirement_t gp;
    RTC_STATUS status_rtc;
    if(!btle_tz1em_initialized) {
        btle_tz1em_initialized = true;
        /* Initialize tz1em */
        if( TZ1EM_STATUS_OK != tz1emInitializeEntry(TZ1EM(BTLE))) {
            return BTLE_TZ1EM_ERROR;
        }
        gp.pcd = TZ1EM_PCDG_BLE_TZBT;       /* BLE will be used */
        gp.mode = TZ1EM_OP_BLE_DOZE_RET;    /* CPU mode must be higher than Retention */
        gp.sunshine_vf = TZ1EM_VF_UN;       /* Frequency is not specified */
        gp.wakeup = NULL;
        gp.permissible_time_lag_us = TZ1EM_PTL_SAFE_ASYNCHRONOUS_IO;
#ifdef TARGET_BLUENINJA_CDP_TZ01B
        gp.trigger[0].event = TZ1EM_WE_OFF; gp.trigger[0].factor = TZ1EM_WF_G6;
#else
        gp.trigger[0].event = TZ1EM_WE_OFF; gp.trigger[0].factor = TZ1EM_WF_UN;
#endif
        gp.trigger[1].event = TZ1EM_WE_OFF; gp.trigger[1].factor = TZ1EM_WF_RTC;  /* wakeup trigger is RTC */
        gp.trigger[2].event = TZ1EM_WE_OFF; gp.trigger[2].factor = TZ1EM_WF_UN;
        if( TZ1EM_STATUS_OK != tz1emConfigureEntry(TZ1EM(BTLE), &gp) ){
            return BTLE_TZ1EM_ERROR;
        }

        /* Initialize RTC for periodic wakeup */
        rtc_init();
        status_rtc = Driver_RTC.SetPeriodicInterrupt(RTC_PERIOD_EVERY_SECOND, periodic_callback);
        if(status_rtc != RTC_OK) {
            return BTLE_TZ1EM_ERROR;
        }
    }
    return BTLE_TZ1EM_OK;
}

/** Put TZ1EM that CPU enter sleep mode
 *
 *  @returns
 *    BTLE_TZ1EM_OK.      Success
 *    BTLE_TZ1EM_ERROR.   Error
 */
int btle_tz1em_go_to_sleep( void )
{
    tz1emStatus_t tz1em_status;
#ifdef __MBED_CMSIS_RTOS_CM
    tz1em_status = tz1emGoIntoTheShade(TZ1EM(BTLE), false);     /* CPU not sleep, should to in idle task */
#else
    tz1em_status = tz1emGoIntoTheShade(TZ1EM(BTLE), true);    /* CPU enters Sleep mode */
#endif
    if((tz1em_status != TZ1EM_STATUS_HAS_SLEPT) && 
        (tz1em_status != TZ1EM_STATUS_OK)) {
        return BTLE_TZ1EM_ERROR;
    }

    return BTLE_TZ1EM_OK;
}

/** Put TZ1EM enter the Sunshine state
 *
 *  @returns
 *    BTLE_TZ1EM_OK.      Success
 *    BTLE_TZ1EM_ERROR.   Error
 */
int btle_tz1em_resume( void )
{
    tz1emStatus_t tz1em_status;
    tz1em_status = tz1emParticipateIn(TZ1EM(BTLE));   /* TZ1EM management to enter the Sunshine state */
    if((tz1em_status != TZ1EM_STATUS_HAS_SLEPT) && 
        (tz1em_status != TZ1EM_STATUS_OK)) {
        return BTLE_TZ1EM_ERROR;
    }

    return BTLE_TZ1EM_OK;
}
