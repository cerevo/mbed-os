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
#include "gpio_api.h"
#include "pinmap.h"
#include "mbed_error.h"
#include "PeripheralPins.h"
#include "PMU_TZ10xx.h"
#include "GPIO_TZ10xx.h"
#include "pmu.h"

extern TZ10XX_DRIVER_GPIO Driver_GPIO;
extern TZ10XX_DRIVER_PMU Driver_PMU;

uint32_t init_gpio = 0;

/** Get gpio control registers pointer
 *
 *  @param  pin_id          The id of pin used for GPIO.
 *
 *  @returns
 *    GPIO information.
 */
gpio_Type *gpio_get_pointer(uint32_t pin_id)
{
    uint32_t group = pin_id >> 3;

    /* Get pin group */
    switch (group) {
        case 0:
            return gpio0;
        case 1:
            return gpio1;
        case 2:
            return gpio2;
        case 3:
            return gpio3;
        default:
            return NULL;
    }
}

/** Get pin mask, set pin as GPIO
 *
 *  @param  pin         The pin to be get mask and set as GPIO pin.
 *
 *  @returns
 *    Mask of the pin (position of pin in port).
 */
uint32_t gpio_set(PinName pin)
{
    /* Check that pin is valid */
    MBED_ASSERT(pin != (PinName)NC);

    /* Check that pin belong to GPIO module */
    GPIOName gpio = (GPIOName) pinmap_peripheral(pin, PinMap_GPIO);
    MBED_ASSERT(gpio != (GPIOName)NC);

    /* Set pin function as GPIO */
    pinmap_pinout(pin, PinMap_GPIO);

    /* Return pin mask */
    return (1 << (gpio & 0x07));
}

/** Initialize gpio pin
 *
 *  @param  obj         The gpio object to initialize.
 *  @param  pin         The pin to use as GPIO function.
 *
 *  @returns
 *    NONE.
 */
void gpio_init(gpio_t *obj, PinName pin)
{
    /* Get pin mask, set pin as GPIO */
    uint32_t mask = gpio_set(pin);

    /* Get pin id */
    GPIOName gpio = (GPIOName) pinmap_peripheral(pin, PinMap_GPIO);

    /* Get gpio control registers pointer */
    gpio_Type *gpio_p = gpio_get_pointer((uint32_t)gpio);
    MBED_ASSERT(gpio_p != NULL);

    /* Fill GPIO object structure for future use */
    obj->pin      = pin;
    obj->pin_id   = (uint32_t) gpio;
    obj->mask     = mask;
    obj->reg_dir  = &gpio_p->GPIODIR;
    obj->reg_ie   = &gpio_p->GPIOIE;
    obj->reg_ic   = &gpio_p->GPIOIC;

    /* Initialize drivers */
    if (!init_gpio) {
        PmuInit();
        Driver_GPIO.Initialize();
        Driver_GPIO.PowerControl(ARM_POWER_FULL);
        init_gpio = 1;
    }
}

/** Set mode for pin of gpio object
 *
 *  @param  obj         The gpio object to be set pin mode.
 *  @param  mode        The mode to be set.
 *
 *  @returns
 *    NONE.
 */
void gpio_mode(gpio_t *obj, PinMode mode)
{
    pin_mode(obj->pin, mode);
}

/** Set direction for pin of gpio object
 *
 *  @param  obj         The gpio object to be set pin direction.
 *  @param  direction   The direction to be set.
 *
 *  @returns
 *    NONE.
 */
void gpio_dir(gpio_t *obj, PinDirection direction)
{
    /* Disable interrupt before setting direction */
    *obj->reg_ie &= ~obj->mask;

    /* Set direction */
    switch (direction) {
        case PIN_INPUT :
            /* Enable input buffer */
            pin_mode(obj->pin, InputActive);
            /* Set INPUT direction */
            *obj->reg_dir &= ~obj->mask;
            break;
        case PIN_OUTPUT:
            /* Set OUTPUT direction */
            *obj->reg_dir |=  obj->mask;
            break;
        default:
            error("Invalid direction\n");
            break;
    }

    /* Clear interrupt */
    *obj->reg_ic |=  obj->mask;
}

/** Check that gpio object is connected
 *
 *  @param  obj         The gpio object to be checked.
 *
 *  @returns
 *    Appropriate int value with current status.
 */
int gpio_is_connected(const gpio_t *obj)
{
    /* Check that pin is valid */
    if (obj->pin_id <= (uint32_t)GPIO_31) {
        return 1;
    }
    return 0;
}

/** Write data to pin of gpio object
 *
 *  @param  obj         The gpio object to be written data.
 *  @param  value       Data to be written to gpio pin.
 *
 *  @returns
 *    NONE.
 */
void gpio_write(gpio_t *obj, int value)
{
    GPIO_STATUS gpio_ret = GPIO_OK;
    if (value) {
        gpio_ret = Driver_GPIO.WritePin((uint32_t)obj->pin_id, 1);
    } else {
        gpio_ret = Driver_GPIO.WritePin((uint32_t)obj->pin_id, 0);
    }
    /* Assert that write to GPIO pin successfully */
    MBED_ASSERT(gpio_ret == GPIO_OK);
}

/** Read data from pin of gpio object
 *
 *  @param  obj         The gpio object to be read data.
 *
 *  @returns
 *    Result of reading:
 *          -1          : Fail to read gpio data
 *          0, 1        : Data read from gpio pin
 */
int gpio_read(gpio_t *obj)
{
    GPIO_STATUS gpio_ret = GPIO_ERROR;
    uint32_t data = 0;

    gpio_ret = Driver_GPIO.ReadPin((uint32_t)obj->pin_id, &data);
    if (gpio_ret == GPIO_OK) {
        return data;
    } else {
        error("Read gpio pin fail\n");
        return -1;
    }
}
