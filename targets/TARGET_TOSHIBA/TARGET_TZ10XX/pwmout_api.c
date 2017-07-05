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
#include "pwmout_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "TMR_TZ10xx.h"
#include "PeripheralPins.h"
#include "PMU_TZ10xx.h"
#include "pmu.h"

#define COUNTER_BITS 16 /* Advandce timer support counter as 8 and 16bits*/
#define MAXIMUM_COUNT_VALUE 65535 /* 2^16 - 1*/

/* TMR Driver interface */
extern TZ10XX_DRIVER_TMR Driver_ADVTMR0;
extern TZ10XX_DRIVER_TMR Driver_ADVTMR1;
extern TZ10XX_DRIVER_TMR Driver_ADVTMR2;
extern TZ10XX_DRIVER_TMR Driver_ADVTMR3;

/** Initialize the PWM driver.
 *  @remarks
 *    + Configures the pins used by PWM, sets a default period (Default to 20ms: standard for servos,
 *    and fine for e.g. brightness control), duty-cycle = 0
 *    + Used CMSIS ADVTMR (Advance Timer) Driver:
 *      - ADVTMR_Initialize
 *      - ADVTMR_PowerControl
 *      - ADVTMR_Configure
 *      - ADVTMR_SetCompareValue
 *      - ADVTMR_ConfigureTFF
 *      - ADVTMR_EnableCapture
 *      - ADVTMR_EnableTFF
 *      - ADVTMR_Start
 *
 *  @param  obj         The PWM object to initialize.
 *  @param  pin         The pin to use for PWM output.
 *
 *  @returns
 *    NONE.
 */
void pwmout_init(pwmout_t* obj, PinName pin)
{
    TMR_STATUS status = TMR_OK;

    /* Init PMU Module */
    PmuInit();

    /* determine the channel */
    PWMName pwm = (PWMName)pinmap_peripheral(pin, PinMap_PWM);
    MBED_ASSERT(pwm != (PWMName)NC);

    switch (pwm) {
        case PWM_0:
            obj->pwm = &Driver_ADVTMR0;
            break;
        case PWM_1:
            obj->pwm = &Driver_ADVTMR1;
            break;
        case PWM_2:
            obj->pwm = &Driver_ADVTMR2;
            break;
        case PWM_3:
            obj->pwm = &Driver_ADVTMR3;
            break;
        default:
            obj->pwm = NULL;
            break;
    }

    MBED_ASSERT(obj->pwm != NULL);

    /* Set pin function as PWM */
    pinmap_pinout(pin, PinMap_PWM);

    /* ADVTMR: PWM enabled, period = 20ms, duty cycle = 0 */
    /* Division rate */
    obj->divisor = 4;

    status = obj->pwm->Initialize(NULL, 0);
    MBED_ASSERT(TMR_OK == status);
    status = obj->pwm->PowerControl(ARM_POWER_FULL);
    MBED_ASSERT(TMR_OK == status);
    /* Timer must be stop before configure. */
    status = obj->pwm->Stop();
    MBED_ASSERT(TMR_OK == status);
    status = obj->pwm->Configure(COUNTER_BITS, TMR_COUNT_MODE_PERIODIC, obj->divisor);
    MBED_ASSERT(TMR_OK == status);

    /* Preload = 59999 (20ms = 1/50s) */
    obj->preload_value = (Driver_PMU.GetFrequency(PMU_CD_PPIER0) / obj->divisor / 50 - 1);
    obj->duty_cycle = 0.0f;
    obj->compare_value = (int)(obj->preload_value * obj->duty_cycle);

    status = obj->pwm->SetCompareValue(obj->compare_value, false);
    MBED_ASSERT(TMR_OK == status);
    status = obj->pwm->ConfigureTFF(TMR_TFF_MODE_CMP_TERM_TOGGLE, false, false);
    MBED_ASSERT(TMR_OK == status);
    status = obj->pwm->EnableCompare(true);
    MBED_ASSERT(TMR_OK == status);
    status = obj->pwm->EnableTFF(true);
    MBED_ASSERT(TMR_OK == status);
    status = obj->pwm->Start(obj->preload_value);
    MBED_ASSERT(TMR_OK == status);
}

/** De-initialize the PWM driver.
 *  @remarks
 *    + Return the pins owned by the PWM object to their reset state.
 *    + Timer driver end procedure is done.
 *    + The end procedure of the internal variables
 *    and the interrupt controller setting are done.
 *    + Transit to the Un-initialize state.
 *    + Used CMSIS  ADVTMR Driver:
 *      - ADVTMR_Uninitialize
 *
 *  @param  obj         The PWM object to de-initialize.
 *
 *  @returns
 *    NONE.
 */
void pwmout_free(pwmout_t* obj)
{
    TMR_STATUS status = TMR_OK;
    MBED_ASSERT(obj->pwm != NULL);
    status = obj->pwm->Uninitialize();
    obj->pwm = NULL;
    obj->divisor = 0;
    obj->preload_value = 0;
    obj->compare_value = 0;
    obj->duty_cycle = 0.0f;

    MBED_ASSERT(TMR_OK == status);
}

/** Set the output duty-cycle, specified as a percentage.
 *  @remarks
 *     Used CMSIS ADVTMR Driver:
 *      - ADVTMR_SetCompareValue()
 *
 *  @param  obj         The PWM object to write.
 *  @param  value       The percent of duty-cycle.
 *
 *  @returns
 *    NONE.
 */
void pwmout_write(pwmout_t* obj, float value)
{
    TMR_STATUS status = TMR_OK;
    MBED_ASSERT(obj->pwm != NULL);

    /* duty cycle is specified as percentage(%) */
    obj->duty_cycle = value;
    obj->compare_value = (int)(obj->preload_value * obj->duty_cycle);

    status = obj->pwm->SetCompareValue(obj->compare_value, false);
    MBED_ASSERT(TMR_OK == status);
}

/** Read the current output duty-cycle setting, measured as a percentage (float).
 *
 *  @param  obj         The PWM object to read.
 *
 *  @returns
 *    The current output duty-cycle setting, measured as a percentage (float).
 */
float pwmout_read(pwmout_t* obj)
{
    MBED_ASSERT(obj->pwm != NULL);
    return obj->duty_cycle;
}

/** Set the PWM output period, specified in seconds (float).
 *  @remarks
 *    + This set the period in second but keeping the duty cycle the same.
 *    + Use pwmout_period_us
 *
 *  @param  obj         The PWM object to use for configures.
 *  @param  seconds     The period value to set in seconds.
 *
 *  @returns
 *    NONE.
 */
void pwmout_period(pwmout_t* obj, float seconds)
{
    pwmout_period_us(obj, (int)(seconds * 1000000.0f));
}

/** Set the PWM period, specified in milliseconds (int).
 *  @remarks
 *    + This set the period in millisecond but keeping the duty cycle the same.
 *    + Use pwmout_period_us
 *
 *  @param  obj         The PWM object to use for configures.
 *  @param  ms          The period value to set in milliseconds.
 *
 *  @returns
 *    NONE.
 */
void pwmout_period_ms(pwmout_t* obj, int ms)
{
    pwmout_period_us(obj, ms * 1000);
}

/** Set the PWM period, specified in microseconds (int).
 *  @remarks
 *    + This set the period in micro-second but keeping the duty cycle the same.
 *    + Used CMSIS ADVTMR Driver:
 *      - ADVTMR_Stop
 *      - ADVTMR_Configure
 *      - ADVTMR_SetCompareValue
 *      - ADVTMR_Start
 *
 *  @param  obj         The PWM object to use for configures.
 *  @param  us          The period value to set in microseconds.
 *
 *  @returns
 *    NONE.
 */
void pwmout_period_us(pwmout_t* obj, int us)
{
    TMR_STATUS status = TMR_OK;
    MBED_ASSERT(obj->pwm != NULL);

    float seconds = (float)(us / 1000000.0f);
    static const uint32_t prescale_tbl[] = {
        1, 2, 4, 8, 32, 128, 512, 1024
    };

    /* Select highest timer resolution */
    for (int i = 0; i < 8; ++i) {
        obj->preload_value = (int)((Driver_PMU.GetFrequency(PMU_CD_PPIER0) / prescale_tbl[i]) * seconds);
        if (obj->preload_value <= MAXIMUM_COUNT_VALUE) {
            obj->divisor = prescale_tbl[i];
            break;
        }
    }

    /* Timer must be stop before re-configure. */
    status = obj->pwm->Stop();
    MBED_ASSERT(TMR_OK == status);
    status = obj->pwm->Configure(COUNTER_BITS, TMR_COUNT_MODE_PERIODIC, obj->divisor);
    MBED_ASSERT(TMR_OK == status);
    /* Keeping the duty cycle the same. */
    obj->compare_value = (int)(obj->preload_value * obj->duty_cycle);
    /* Apply new values */
    status = obj->pwm->SetCompareValue(obj->compare_value, false);
    MBED_ASSERT(TMR_OK == status);
    status = obj->pwm->Start(obj->preload_value);
    MBED_ASSERT(TMR_OK == status);
}

/** Set the PWM pulsewidth, specified in seconds (float).
 *  @remarks
 *    + This set the pulsewidth in second but keeping the period the same.
 *    + Use pwmout_pulsewidth_us
 *
 *  @param  obj         The PWM object to use for configure.
 *  @param  seconds     The pulse width value to set in seconds.
 *
 *  @returns
 *    NONE.
 */
void pwmout_pulsewidth(pwmout_t* obj, float seconds)
{
    pwmout_pulsewidth_us(obj, (int)(seconds * 1000000.0f));
}

/** Set the PWM pulsewidth, specified in milliseconds (int).
 *  @remarks
 *    + This set the pulsewidth in milliseconds but keeping the period the same.
 *    + Use pwmout_pulsewidth_us
 *
 *  @param  obj         The PWM object to use for configure.
 *  @param  ms          The pulse width value to set in milliseconds.
 *
 *  @returns
 *    NONE.
 */
void pwmout_pulsewidth_ms(pwmout_t* obj, int ms)
{
    pwmout_pulsewidth_us(obj, ms * 1000);
}

/** Set the PWM pulsewidth, specified in microseconds (int).
 *  @remarks
 *    + This set the pulsewidth in micro-second but keeping the period the same.
 *    + Use CMSIS ADVTMR Driver:
 *      - ADVTMR_SetCompareValue
 *
 *  @param  obj         The PWM object to use for configures.
 *  @param  us          The pulse width value to set in microseconds.
 *
 *  @returns
 *    NONE.
 */
void pwmout_pulsewidth_us(pwmout_t* obj, int us)
{
    TMR_STATUS status = TMR_OK;
    MBED_ASSERT(obj->pwm != NULL);

    float seconds = (float)(us / 1000000.0f);
    /* Keeping the period the same. */
    obj->compare_value = (int)((Driver_PMU.GetFrequency(PMU_CD_PPIER0) / obj->divisor) * seconds);
    MBED_ASSERT(obj->compare_value <= obj->preload_value);
    /* Apply new values */
    status = obj->pwm->SetCompareValue(obj->compare_value, false);
    MBED_ASSERT(TMR_OK == status);
}
