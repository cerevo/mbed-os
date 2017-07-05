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
#include "port_api.h"
#include "pinmap.h"
#include "gpio_api.h"
#include "mbed_error.h"
#include "mbed_assert.h"
#include "GPIO_TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "pmu.h"
#include "PeripheralPins.h"

extern TZ10XX_DRIVER_GPIO Driver_GPIO;
extern TZ10XX_DRIVER_PMU Driver_PMU;
extern uint32_t init_gpio;

/** Get gpio control registers pointer.
 *
 *  @param  port        The port to get pin name.
 *
 *  @returns
 *    gpio_Type         GPIO type that define in gpio_object.h.
 */
gpio_Type *port_get_pointer(PortName port)
{
    switch (port) {
        case Gpio_Port0:
            return gpio0;
        case Gpio_Port1:
            return gpio1;
        case Gpio_Port2:
            return gpio2;
        case Gpio_Port3:
            return gpio3;
        default:
            return NULL;
    }
}

/** Receive PinName of pin at position n in the port.
 *  @remarks
 *    There are one more mbed pins map to one pin of the port.
 *    This API only return one mbeb pin for the pin at position pin_n in the port.
 *
 *  @param  port        The port to get pin name.
 *  @param  pin_n       Position of pin in the port.
 *
 *  @returns
 *    Pin name.
 */
PinName port_pin(PortName port, int pin_n)
{
    /* Check that port is valid */
    MBED_ASSERT(port != (PortName)NC);

    static const uint32_t port_pin_tbl[] = {
        TABLE_PORTPIN_ROW(RTE_GPIO_0_ID, PA0, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_1_ID, PA1, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_2_ID, PA2, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_3_ID, PA3, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_4_ID, PA4, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_5_ID, PA5, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_6_ID, PA6, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_7_ID, PA7, NC),

        TABLE_PORTPIN_ROW(RTE_GPIO_8_ID, PA8, PG0),
        TABLE_PORTPIN_ROW(RTE_GPIO_9_ID, PA9, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_10_ID, PA10, PD10),
        TABLE_PORTPIN_ROW(RTE_GPIO_11_ID, PA11, PD11),
        TABLE_PORTPIN_ROW(RTE_GPIO_12_ID, PA12, PD4),
        TABLE_PORTPIN_ROW(RTE_GPIO_13_ID, PA13, PD5),
        TABLE_PORTPIN_ROW(RTE_GPIO_14_ID, PA14, PC2),
        TABLE_PORTPIN_ROW(RTE_GPIO_15_ID, PA15, PC3),

        TABLE_PORTPIN_ROW(RTE_GPIO_16_ID, PC0, PE0),
        TABLE_PORTPIN_ROW(RTE_GPIO_17_ID, PC1, PE1),
        TABLE_PORTPIN_ROW(RTE_GPIO_18_ID, PD8, PE2),
        TABLE_PORTPIN_ROW(RTE_GPIO_19_ID, PD9, PE3),
        TABLE_PORTPIN_ROW(RTE_GPIO_20_ID, PD0, PE4),
        TABLE_PORTPIN_ROW(RTE_GPIO_21_ID, PD1, PE5),
        TABLE_PORTPIN_ROW(RTE_GPIO_22_ID, PD6, PE6),
        TABLE_PORTPIN_ROW(RTE_GPIO_23_ID, PD7, PE7),

        TABLE_PORTPIN_ROW(RTE_GPIO_24_ID, PB8, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_25_ID, PB9, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_26_ID, PB10, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_27_ID, PB11, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_28_ID, PB12, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_29_ID, PB13, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_30_ID, PB14, NC),
        TABLE_PORTPIN_ROW(RTE_GPIO_31_ID, PB15, NC)
    };

    return (PinName) port_pin_tbl[(port*8) + pin_n];
}

/** Initialize port object for masked pins with specified direction.
 *  @remarks
 *    Wrapper from CMSIS Drivers:
 *      - GPIO_Initialize
 *      - GPIO_PowerControl
 *      - PMU_Initialize
 *      - PMU_SelectClockSource
 *      - PMU_SetPrescaler
 *
 *  @param  obj         The port object to initialize.
 *  @param  port        The port to be initialized.
 *  @param  mask        The masked pins in port to be initialized.
 *  @param  dir         Direction of port to be initialized.
 *
 *  @returns
 *    NONE.
 */
void port_init(port_t *obj, PortName port, int mask, PinDirection dir)
{
    uint32_t i;
    PinName pin;

    /* Initialize drivers */
    if (!init_gpio) {
        PmuInit();
        Driver_GPIO.Initialize();
        Driver_GPIO.PowerControl(ARM_POWER_FULL);
        init_gpio = 1;
    }

    /* Get port control registers */
    gpio_Type *gpio = port_get_pointer(port);

    /* Check that gpio registers existed */
    MBED_ASSERT(gpio != NULL);

    /* Store port object for further use */
    obj->mask    = mask & 0xff;
    obj->port    = port;
    obj->reg_dir = &gpio->GPIODIR;
    obj->reg_ie  = &gpio->GPIOIE;
    obj->reg_ic  = &gpio->GPIOIC;

    /* Set port pins as GPIO */
    for (i=0; i<8; i++) {
        if (obj->mask & (1<<i)) {
            /* Get PinName of one pin in the port */
            pin = port_pin(obj->port, i);
            /* Set pin function as GPIO pin */
            gpio_set(pin);
        }
    }

    /* Set port direction */
    port_dir(obj, dir);
}

/** Set mode for pin of port object.
 *
 *  @param  obj         The port object to be set port mode.
 *  @param  mode        The mode to be set.
 *
 *  @returns
 *    NONE.
 */
void port_mode(port_t *obj, PinMode mode)
{
    uint32_t i;
    PinName pin;
    /* The mode is set per pin: reuse pinmap logic */
    for (i=0; i<8; i++) {
        if (obj->mask & (1<<i)) {
            pin = port_pin(obj->port, i);
            pin_mode(pin, mode);
        }
    }
}

/** Set direction for pin of port object.
 *  @remarks
 *    Wrapper from CMSIS PMU Driver:
 *      - PMU_StandbyInputBuffer
 *
 *  @param  obj         The port object to be set pin direction.
 *  @param  dir         The direction to be set.
 *
 *  @returns
 *    NONE.
 */
void port_dir(port_t *obj, PinDirection dir)
{
    /* Disable interrupt before setting direction*/
    *obj->reg_ie &= ~obj->mask;

    /* The direction is set per pin: reuse gpio logic */
    switch (dir) {
        case PIN_INPUT :
            /* Enable input buffer
             * If not input buffer is standby, then pin is always read as Zero*/
            for (uint32_t i=0; i<8; i++) {
                if (obj->mask & (1<<i)) {
                    PinName pin = port_pin(obj->port, i);
                    pin_mode(pin, InputActive);
                }
            }
            /* Set INPUT direction*/
            *obj->reg_dir &= ~obj->mask;
            break;
        case PIN_OUTPUT:
            /* Set OUTPUT direction*/
            *obj->reg_dir |=  obj->mask;
            break;
        default:
            error("Invalid direction\n");
            break;
    }

    /* Clear interrupt*/
    *obj->reg_ic |=  obj->mask;
}

/** Write data to masked pins of the port.
 *  @remarks
 *    Wrapper from CMSIS GPIO Driver:
 *      - GPIO_Write
 *
 *  @param  obj         The port object to be written data.
 *  @param  value       Data to be written to port pins.
 *
 *  @returns
 *    NONE.
 */
void port_write(port_t *obj, int value)
{
    uint32_t mask  = obj->mask << (obj->port*8);
    uint32_t val   = value << (obj->port*8);

    GPIO_STATUS gpio_ret = Driver_GPIO.Write(mask, (uint32_t) val);
    /* Assert that write to GPIO pin successfully*/
    MBED_ASSERT(gpio_ret == GPIO_OK);
}

/** Read data from masked pins of port.
 *  @remarks
 *    Wrapper from CMSIS PMU Driver:
 *      - GPIO_Read()
 *
 *  @param  obj         The port object to be read data.
 *
 *  @returns
 *    Data read from port pins.
 *      -1                      : Fail to read port data
 *      unsigned integer value  : Data read from port
 */
int port_read(port_t *obj)
{
    uint32_t data;

    GPIO_STATUS gpio_ret = Driver_GPIO.Read(&data);
    if (gpio_ret == GPIO_OK) {
        switch (obj->port) {
            case Gpio_Port0:
                data &= 0x000000ff;
                break;
            case Gpio_Port1:
                data &= 0x0000ff00;
                break;
            case Gpio_Port2:
                data &= 0x00ff0000;
                break;
            case Gpio_Port3:
                data &= 0xff000000;
                break;
            default:
                error("Read port fail\n");
                return -1;
        }
        return data >> (obj->port*8);
    } else {
        error("Read port fail\n");
        return -1;
    }
}
