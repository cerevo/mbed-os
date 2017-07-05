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

#include "mbed_assert.h"
#include "device.h"
#include "RTC_TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "pmu.h"

#if DEVICE_RTC

#include "rtc_api.h"

/* RTC Driver interface */
extern TZ10XX_DRIVER_RTC Driver_RTC;

/* PMU Driver interface */
extern TZ10XX_DRIVER_PMU Driver_PMU;

/*Initial flag  0: Disabled, 1: Enabled*/
static int flag = 0;

static int diff_year = 0;

static RTC_TIME rtc_time_now = {0, 0, 0, 0, 0, 0, 0};

/** Initialize the rtc peripheral module.
 *  @remarks
 *    + Use CMSIS RTC Driver:
 *      - RTC_Initialize
 *
 *  @param NONE
 *
 *  @returns
 *    NONE.
 */
void rtc_init(void)
{
    if (!flag) {
        PmuInit();
        Driver_PMU.StartClockSource(PMU_CLOCK_SOURCE_OSC32K);
        Driver_PMU.SelectClockSource(PMU_CSM_RTC, PMU_CLOCK_SOURCE_OSC32K);
        Driver_RTC.Initialize();
        flag = 1;
    }
}

/** De-Initialize RTC peripheral.
 *  @remarks
 *    + Use CMSIS RTC Driver:
 *      - RTC_Uninitialize
 *
 *  @param NONE
 *
 *  @returns
 *    NONE.
 */
void rtc_free(void)
{
    if (flag) {
        Driver_RTC.Uninitialize();
        flag = 0;
    }
}


/** Check the RTC is enabled or not.
 *
 *  @param NONE
 *
 *  @returns
 *    Status of RTC enabling.
 */
int rtc_isenabled(void)
{
    return flag;
}

/** Get time of RTC, then convert to UNIX timestamp.
 *  @remarks
 *    + Use CMSIS RTC Driver:
 *      - RTC_GetTime
 *
 *  @param NONE
 *
 *  @returns
 *    UNIX timestamp.
 */
time_t rtc_read(void)
{
    /* Setup a tm structure based on the RTC */
    struct tm timeinfo;
    Driver_RTC.GetTime(&rtc_time_now);
    timeinfo.tm_sec = rtc_time_now.sec;
    timeinfo.tm_min = rtc_time_now.min;
    timeinfo.tm_hour = rtc_time_now.hour;
    timeinfo.tm_mday = rtc_time_now.mday;
    timeinfo.tm_mon = rtc_time_now.mon;
    timeinfo.tm_wday = rtc_time_now.wday;
    timeinfo.tm_year = rtc_time_now.year + diff_year;

    /* Convert to timestamp */
    time_t t = mktime(&timeinfo);
    return t;
}

/** Set RTC time in UNIX timestamp.
 *  @remarks
 *    + Use CMSIS RTC Driver:
 *      - RTC_SetTime
 *
 *  @param t UNIX timestamp
 *
 *  @returns
 *    NONE.
 */
void rtc_write(time_t t)
{
    /* Convert the time in to a tm */
    struct tm *timeinfo = localtime(&t);

    /* Set the RTC time*/
    /* second (0..59) */
    /* minute (0..59) */
    /* hour (0..23) */
    /* day of month (1..31) */
    /* month (1..12) */
    /* year (0..99) */
    /* day of week (0..6); 0 is Sunday */
    /* save diff year */
    diff_year = timeinfo->tm_year - (timeinfo->tm_year%100);

    rtc_time_now.sec = (uint8_t)(timeinfo->tm_sec);
    rtc_time_now.min = (uint8_t)(timeinfo->tm_min);
    rtc_time_now.hour = (uint8_t)(timeinfo->tm_hour);
    rtc_time_now.mday = (uint8_t)(timeinfo->tm_mday);
    /* tm.month is from 0 to 11 */
    rtc_time_now.mon = (uint8_t)( ((timeinfo->tm_mon)) + 1 );
    rtc_time_now.wday = (uint8_t)(timeinfo->tm_wday);
    rtc_time_now.year = (uint8_t)(timeinfo->tm_year - diff_year);

    /* Set time for RTC */
    const RTC_TIME * now = &rtc_time_now;
    Driver_RTC.SetTime(now);
}

#endif /* DEVICE_RTC */
