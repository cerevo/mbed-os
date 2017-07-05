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
#include "gpio_api.h"
#include "PMU_TZ10xx.h"
#include "pmu.h"

#include "RTE_Device.h"
#if (RTE_BLE)
#include "twic_interface.h"
#endif
#include "mbed_error.h"

extern TZ10XX_DRIVER_PMU Driver_PMU;

static uint32_t pmu_init = 0;

/** Initialize the PMU.
 *
 *  @param      NONE.
 *
 *  @note       If a module use PMU RTE Driver, then it must call PmuInit() in initialization code
 *
 *  @returns
 *    NONE.
 */
void PmuInit()
{
#if defined ( RTE_BLE )
    if (!pmu_init) {
        if (TZ1EM_STATUS_OK != tz1emInitializeSystem()) {
            error("tz1em Initialize System fail \r\n");
        }
        /* Initializes the HW resources */
        if ( TWIC_STATUS_OK != twicIfLeIoInitialize() ) {
            error("twic initializes HW fail \r\n");
        }
        
        /* Setup Wakeup mechanism for GPIO6, Switch 2 */
        Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_6, 0);
        Driver_PMU.ConfigureWakeup(PMU_WAKEUP_FACTOR_GPIO_6, PMU_WAKEUP_EVENT_EDGE_BOTH);
        Driver_PMU.EnableWakeup(PMU_WAKEUP_FACTOR_GPIO_6, true);
        /* BrownOut interrupt */
        Driver_PMU.SetBrownOutMode(PMU_BROWN_OUT_NOTIFY);
        Driver_PMU.ConfigureWakeup(PMU_WAKEUP_FACTOR_BROWNOUT, PMU_WAKEUP_EVENT_EDGE_POS);
        Driver_PMU.EnableWakeup(PMU_WAKEUP_FACTOR_BROWNOUT, true);
        
        pmu_init =1;
    }
#else
    if (!pmu_init) {
        Driver_PMU.Initialize(NULL);
        Driver_PMU.SetPrescaler(PMU_CD_PPIER0, 4);
        Driver_PMU.SetPrescaler(PMU_CD_PPIER1, 4);
        Driver_PMU.SetPrescaler(PMU_CD_PPIER2, 4);
        Driver_PMU.StartClockSource(PMU_CLOCK_SOURCE_PLL);
        Driver_PMU.SelectClockSource(PMU_CSM_MAIN, PMU_CLOCK_SOURCE_PLL);
        Driver_PMU.SelectClockSource(PMU_CSM_CPUST, PMU_CLOCK_SOURCE_OSC12M);

        /* Set up Wakeup mechanism for sleep() */
        Driver_PMU.SetCpuLowFrequencyOnSleep(false);
        /* Setup Wakeup mechanism for GPIO6, Switch 2 */
        Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_6, 0);
        Driver_PMU.ConfigureWakeup(PMU_WAKEUP_FACTOR_GPIO_6, PMU_WAKEUP_EVENT_EDGE_BOTH);
        Driver_PMU.EnableWakeup(PMU_WAKEUP_FACTOR_GPIO_6, true);
        /* BrownOut interrupt */
        Driver_PMU.SetBrownOutMode(PMU_BROWN_OUT_NOTIFY);
        Driver_PMU.ConfigureWakeup(PMU_WAKEUP_FACTOR_BROWNOUT, PMU_WAKEUP_EVENT_EDGE_POS);
        Driver_PMU.EnableWakeup(PMU_WAKEUP_FACTOR_BROWNOUT, true);

        Driver_PMU.RetainOutputBuffer(PMU_PD_AON_PM, 0);
        Driver_PMU.RetainOutputBuffer(PMU_PD_AON_PP1, 0);

        pmu_init = 1;
    }
#endif
}
