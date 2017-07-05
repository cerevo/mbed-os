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
#include "sleep_api.h"
#include "cmsis.h"
#include "mbed_interface.h"
#include "PMU_TZ10xx.h"
#include "pmu.h"

extern TZ10XX_DRIVER_PMU Driver_PMU;

/** Set to sleep mode. CPU is stopped by WFI.
 *  @remarks
 *    Use CMSIS PMU Driver:
 *      - PMU_SetPowerMode
 *
 *  @param NONE
 *
 *  @returns
 *    NONE.
 */
void hal_sleep(void)
{
    /* CPU is stopped by WFI.
     * Fastest wake up time
     */
    PmuInit();
    Driver_PMU.SetPowerMode(PMU_POWER_MODE_SLEEP0);
}

/** Set to deep sleep mode. CPU/MPIER/SDMAC/AESA/RNG/SRAMC/SPIC are stopped.
 *  @remarks
 *    Use CMSIS PMU Driver:
 *      - PMU_SetPowerMode
 *
 *  @param NONE
 *
 *  @returns
 *    NONE.
 */
void hal_deepsleep(void)
{
    /* CPU/MPIER/SDMAC/AESA/RNG/SRAMC/SPIC are stopped
     */
    PmuInit();
    Driver_PMU.SetPowerMode(PMU_POWER_MODE_SLEEP1);
}

