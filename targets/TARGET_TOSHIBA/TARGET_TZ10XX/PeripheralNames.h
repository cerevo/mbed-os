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
#ifndef MBED_PERIPHERALNAMES_H
#define MBED_PERIPHERALNAMES_H

#include "cmsis.h"
#include "PinNames.h"
#include "RTE_Device.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GPIO_0 = 0,
    GPIO_1,
    GPIO_2,
    GPIO_3,
    GPIO_4,
    GPIO_5,
    GPIO_6,
    GPIO_7,
    GPIO_8,
    GPIO_9,
    GPIO_10,
    GPIO_11,
    GPIO_12,
    GPIO_13,
    GPIO_14,
    GPIO_15,
    GPIO_16,
    GPIO_17,
    GPIO_18,
    GPIO_19,
    GPIO_20,
    GPIO_21,
    GPIO_22,
    GPIO_23,
    GPIO_24,
    GPIO_25,
    GPIO_26,
    GPIO_27,
    GPIO_28,
    GPIO_29,
    GPIO_30,
    GPIO_31,
    INVALID_GPIO = (int)NC
} GPIOName;

typedef enum {
    UART_0 = 0,
    UART_1,
#if (!RTE_BLE)
    UART_2,
#endif
    INVALID_UART = (int)NC
} UARTName;

typedef enum {
    ADC12_0 = (1<<0),
    ADC12_1 = (1<<1),
    ADC12_2 = (1<<2),
    ADC12_3 = (1<<3),

    INVALID_ADC12 = (int)NC
} ADC12Name;

typedef enum {
    ADC24_0 = (1<<0),
    ADC24_1 = (1<<1),
    ADC24_2 = (1<<2),
    INVALID_ADC24 = (int)NC
} ADC24Name;

typedef enum {
    DAC_0 = 0
} DACName;

typedef enum {
    SPIM_0 = 0,
    SPIM_1,
    SPIM_2,
    SPIM_3,
    SPIC,
    INVALID_SPI = (int)NC
} SPIName;

typedef enum {
    I2C_0 = 0,
    I2C_1,
    I2C_2,
    INVALID_I2C = (int)NC
} I2CName;

typedef enum {
    PWM_0 = 0,
    PWM_1,
    PWM_2,
    PWM_3,
    INVALID_PWM = (int)NC
} PWMName;

#define MBED_ANALOGIN0    A0
#define MBED_ANALOGIN1    A1
#define MBED_ANALOGIN2    A2
#define MBED_ANALOGIN3    A3
#define MBED_ANALOGIN4    A4
#define MBED_ANALOGIN5    A5

#define MBED_PWMOUT0      PA8
#define MBED_PWMOUT1      PA9
#define MBED_PWMOUT2      PE2
#define MBED_PWMOUT3      NC

#define STDIO_UART_TX     USBTX
#define STDIO_UART_RX     USBRX
#define STDIO_UART        UART_1

#ifdef __cplusplus
}
#endif

#endif
