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
#ifndef MBED_OBJECTS_H
#define MBED_OBJECTS_H

#include "cmsis.h"
#include "PortNames.h"
#include "PeripheralNames.h"
#include "PinNames.h"
#include "gpio_object.h"
#if defined(YOTTA_CFG_MBED_OS)
#include "target_config.h"
#endif

#include "SPI_TZ10xx.h"
#include "Driver_UART.h"
#include "Driver_I2C.h"
#include "TMR_TZ10xx.h"

#ifdef __cplusplus
extern "C" {
#endif

struct gpio_irq_s {
    uint32_t pin_id;
    uint32_t event;
};

struct port_s {
    PortName port;
    uint32_t mask;

    __IO uint32_t *reg_dir;
    __IO uint32_t *reg_ie;
    __IO uint32_t *reg_ic;
};

struct pwmout_s {
    TZ10XX_DRIVER_TMR *pwm;
    uint32_t divisor;
    uint32_t preload_value;
    uint32_t compare_value;
    float duty_cycle; /* Unit of duty cycle is percentage (%). */
};

/*
* uart_Type         : in TZ10xx.h
* index             : UART0, UART1 are available for free external connection
*/
struct serial_s {
    ARM_DRIVER_UART  *uart;
    uint32_t index;
    uint32_t interrupt_mask;
    uint32_t baudrate;
    ARM_UART_PARITY parity;
    ARM_UART_STOP_BITS stop_bits;
    ARM_UART_FLOW_CONTROL flow_control;
    uint8_t data_bits;
};

struct analogin_s {
    ADC12Name adc;
};

struct dac_s {
    DACName dac;
};

/*
* ARM_DRIVER_I2C    : in Driver_I2C.h
* index             : I2C0, I2C1 are available for free external connection
*/
struct i2c_s {
    ARM_DRIVER_I2C *i2c;
    int index;
};

/*
* TZ10XX_DRIVER_SPI : in SPI_TZ10xx.h
*/
struct spi_s {
    TZ10XX_DRIVER_SPI *driver;
    SPIName module;
};


#ifdef __cplusplus
}
#endif

#endif
