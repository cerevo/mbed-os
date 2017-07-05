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

#include "gpio_irq_api.h"
#include "gpio_api.h"
#include "gpio_object.h"
#include "cmsis.h"
#include "pinmap.h"
#include "mbed_error.h"
#include "mbed_assert.h"
#include "PeripheralPins.h"
#include "PMU_TZ10xx.h"
#include "GPIO_TZ10xx.h"
#include "pmu.h"

#define CHANNEL_NUM	    32
#define EDGE_NONE    (0)
#define EDGE_RISE    (1)
#define EDGE_FALL    (2)
#define EDGE_BOTH    (3)

extern TZ10XX_DRIVER_GPIO Driver_GPIO;
extern uint32_t init_gpio;

static uint32_t channel_ids[CHANNEL_NUM] = {0};
static gpio_irq_handler hal_irq_handler[CHANNEL_NUM] = {NULL};

/** GPIO interrupt handler
 *
 *  @param  pin         A terminal number for which the event occurred is passed. Any value within
 *  the range of [0,31] is passed
 *
 *  @returns
 *    NONE
 */
static void pin_irq_handler(uint32_t pin)
{
    uint32_t val;
    Driver_GPIO.ReadPin(pin, &val);

    switch (val) {
        /* rising edge detection */
        case 1:
            hal_irq_handler[pin](channel_ids[pin], IRQ_RISE);
            break;
        /* falling edge detection */
        case 0:
            hal_irq_handler[pin](channel_ids[pin], IRQ_FALL);
            break;
        default:
            break;
    }
}

/** Initialize gpio interrupt object for the pin
 *
 *  @param  obj         The gpio interrupt object to initialize.
 *  @param  pin         The pin to use as GPIO IRQ.
 *  @param  handler     Callback function of GPIO IRQ.
 *  @param  id          The parameter pass to callback handler.
 *
 *  @returns
 *    The result of initialization:
 *          0           : success
 *          -1          : fail
 */
int gpio_irq_init(gpio_irq_t *obj, PinName pin, gpio_irq_handler handler, uint32_t id)
{
    GPIO_STATUS gpio_ret = GPIO_ERROR;

    /* Set pin as GPIO */
    gpio_set(pin);

    /* Initialize drivers */
    if (!init_gpio) {
        PmuInit();
        Driver_GPIO.Initialize();
        Driver_GPIO.PowerControl(ARM_POWER_FULL);
        init_gpio = 1;
    }

    /* Get pin ID */
    GPIOName pin_id = (GPIOName) pinmap_peripheral(pin, PinMap_GPIO);

    /* Get gpio control registers pointer */
    gpio_Type *gpio = gpio_get_pointer((uint32_t)pin_id);
    MBED_ASSERT(gpio != NULL);

    /* Store gpio_irq object for further use */
    obj->pin_id  = (uint32_t) pin_id;
    obj->event   = EDGE_NONE;

    /* Store for IRQ handler */
    channel_ids[pin_id] = id;
    hal_irq_handler[pin_id] = handler;

    /* Configure pin IRQ, pin direction must be configured as Input Hi-z */
    gpio_ret = Driver_GPIO.Configure(pin_id, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, pin_irq_handler);
    if (GPIO_ERROR == gpio_ret) {
        return -1;
    } else {
        return 0;
    }
}

/** Deinitialize gpio interrupt object
 *
 *  @param  obj         The gpio interrupt object to be free.
 *
 *  @returns
 *    NONE.
 */
void gpio_irq_free(gpio_irq_t *obj)
{
    /* Clear pending interrupt */
    NVIC_ClearPendingIRQ((IRQn_Type)(obj->pin_id));

    /* Reset global store for gpio_irq object */
    channel_ids[obj->pin_id] = 0;
    hal_irq_handler[obj->pin_id] = NULL;

    /* Reset gpio_irq object's event; */
    obj->event   = EDGE_NONE;
}

/** Enable/disable gpio interrupt event
 * and set event for interrupt detection
 *
 *  @param  obj         The gpio interrupt object to be set.
 *  @param  event       Detection signal to be set for gpio interrupt object.
 *  @param  enable      Enable/Disable event.
 *
 *  @returns
 *    NONE.
 */
void gpio_irq_set(gpio_irq_t *obj, gpio_irq_event event, uint32_t enable)
{
    GPIO_STATUS gpio_ret = GPIO_ERROR;
    uint32_t gpio_event = 0xff;

    if (enable) {
        if (event == IRQ_RISE) {
            /* If falling edge or both edge has been set to the object,
             * set object's event to both edge.
             * Otherwise, set object's event to rising edge.
             */
            if ((obj->event == EDGE_FALL) || (obj->event == EDGE_BOTH)) {
                obj->event = EDGE_BOTH; /* BOTH Edge */
            } else { /* NONE or RISE */
                obj->event = EDGE_RISE;
            }
        }
        if (event == IRQ_FALL) {
            /* If rising edge or both edge has been set to the object,
             * set object's event to both edge.
             * Otherwise, set object's event to falling edge.
             */
            if ((obj->event == EDGE_RISE) || (obj->event == EDGE_BOTH)) {
                obj->event = EDGE_BOTH;
            } else { /* NONE or FALL */
                obj->event = EDGE_FALL;
            }
        }
    } else { /* Disable */
        if (event == IRQ_RISE) {
            /* If falling edge or both edge has been set to the object,
             * disable rising edge by setting object's event to falling.
             * Otherwise, disable rising edge by setting object's event to none edge.
             */
            if ((obj->event == EDGE_FALL) || (obj->event == EDGE_BOTH)) {
                obj->event = EDGE_FALL;
            } else { /* NONE or RISE */
                obj->event = EDGE_NONE;
            }
        }
        if (event == IRQ_FALL) {
            /* If rising edge or both edge has been set to the object,
             * disable falling edge by setting object's event to rising.
             * Otherwise, disable falling edge by setting object's event to none edge.
             */
            if ((obj->event == EDGE_RISE) || (obj->event == EDGE_BOTH)) {
                obj->event = EDGE_RISE;
            } else { /* NONE or FALL */
                obj->event = EDGE_NONE;
            }
        }
    }

    /* Convert mbed event to gpio cmcis event */
    switch(obj->event) {
        case EDGE_RISE:
            gpio_event = GPIO_EVENT_EDGE_POS;
            break;
        case EDGE_FALL:
            gpio_event = GPIO_EVENT_EDGE_NEG;
            break;
        case EDGE_BOTH:
            gpio_event = GPIO_EVENT_EDGE_BOTH;
            break;
        case EDGE_NONE:
            gpio_event = GPIO_EVENT_DISABLE;
            break;
        default:
            error("Event error: Event not supported.\n");
            return;
    }

    /* Configure pin IRQ, pin direction must be configured as Input Hi-z */
    gpio_ret = Driver_GPIO.Configure(obj->pin_id, GPIO_DIRECTION_INPUT_HI_Z, (GPIO_EVENT)gpio_event, pin_irq_handler);
    MBED_ASSERT(GPIO_OK == gpio_ret);
}

/** Enable interrupt for gpio interrupt object
 *
 *  @param  obj         The gpio interrupt object to be enabled interrupt.
 *
 *  @returns
 *    NONE.
 */
void gpio_irq_enable(gpio_irq_t *obj)
{
    NVIC_EnableIRQ((IRQn_Type)(obj->pin_id));
}

/** Disable interrupt for gpio interrupt object
 *
 *  @param  obj         The gpio interrupt object to be disabled interrupt.
 *
 *  @returns
 *    NONE.
 */
void gpio_irq_disable(gpio_irq_t *obj)
{
    NVIC_DisableIRQ((IRQn_Type)(obj->pin_id));
}
