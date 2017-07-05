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
#include "analogin_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "PeripheralNames.h"
#include "PeripheralPins.h"
#include "ADCC12_TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "pmu.h"

extern TZ10XX_DRIVER_ADCC12 Driver_ADCC12;
extern TZ10XX_DRIVER_PMU Driver_PMU;

static volatile uint32_t g_ConvFlg = 0;

void ADC12_Single_Callback(ADCC12_EVENT event)
{
    if(event & ADCC12_EVENT_CONV_END) {
        g_ConvFlg= 1;
    }
}

/** Initialize analog INPUT object for pin 
 *
 *  @param  obj          The analogin object to initialize.
 *  @param  pin          The pin to use as analog input pin.
 *
 *  @returns 
 *    NONE.
 */
void analogin_init(analogin_t *obj, PinName pin)
{
    ADCC12_STATUS status = ADCC12_OK;
    uint32_t event_mask = ADCC12_EVENT_CONV_END;

    /* Get ADC channel and assign it to analogin object */
    obj->adc = (ADC12Name)pinmap_peripheral(pin, PinMap_ADC12);
    MBED_ASSERT(obj->adc != (ADC12Name)NC);

    /* Init PMU Module */
    PmuInit();
    Driver_PMU.SelectClockSource(PMU_CSM_ADCC12, PMU_CLOCK_SOURCE_SIOSC4M);
    Driver_PMU.SetPrescaler(PMU_CD_ADCC12, 1);

    /* Initialize analog input */
    status = Driver_ADCC12.Initialize(ADC12_Single_Callback, event_mask);

    status = Driver_ADCC12.PowerControl(ARM_POWER_FULL);
    MBED_ASSERT(ADCC12_OK == status);
}

/** Read the ADC conversion data, 
 *  Represented as an unsigned short in the range [0x0,0xFFFF]
 *
 *  @param  obj         The analogin object to be read data.
 *
 *  @returns 
 *    The analogin conversion data in unsigned number format.
 */
uint16_t analogin_read_u16(analogin_t *obj)
{
    ADCC12_STATUS status = ADCC12_OK;
    /* Converted data in single mode */
    static uint16_t          data = 0;

    /* Set mode */
    status = Driver_ADCC12.SetScanMode(ADCC12_SCAN_MODE_SINGLE, (ADCC12_CHANNEL)obj->adc);
    MBED_ASSERT(ADCC12_OK == status);

    status = Driver_ADCC12.SetDataFormat((ADCC12_CHANNEL)obj->adc, ADCC12_UNSIGNED);
    MBED_ASSERT(ADCC12_OK == status);

    status = Driver_ADCC12.SetOffset((ADCC12_CHANNEL)obj->adc, 0x00);
    MBED_ASSERT(ADCC12_OK == status);

    status = Driver_ADCC12.SetComparison(ADCC12_CMP_DATA_0, 0x00, ADCC12_CMP_NO_COMPARISON, (ADCC12_CHANNEL)obj->adc);
    MBED_ASSERT(ADCC12_OK == status);

    /* Launch ADC */
    status = Driver_ADCC12.Start();
    MBED_ASSERT(ADCC12_OK == status);

    while (g_ConvFlg != 1) {
        ;
    }
    g_ConvFlg = 0;

    status = Driver_ADCC12.ReadData((ADCC12_CHANNEL)obj->adc, &data);
    MBED_ASSERT(ADCC12_OK == status);

    /* Stop ADC */
    status = Driver_ADCC12.Stop();

    return data;
}

/** Read the ADC conversion data,
 *  Represented as percentage in the range [0.0, 1.0]
 *
 *  @param      obj         The analogin object to be read data.
 *
 *  @returns 
 *    The analogin conversion data in percentage format.
 */
float analogin_read(analogin_t *obj)
{
    uint16_t value = analogin_read_u16(obj);
    return (float)(value * (1.0f / (float)0xFFFF));
}
