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
#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PIN_INPUT,
    PIN_OUTPUT
} PinDirection;

typedef enum {
    /* TZ1001 Pin Names
     * First 4 bits represent pin number, the remaining
     * bits represent port number (A = 0, B = 1, ...)
     */

    /* Mulifunction IO pins */
    PA0 = 0 << 4,   /* MCU_GPIO_0 */
    PA1,            /* MCU_GPIO_1 */
    PA2,            /* MCU_GPIO_2 */
    PA3,            /* MCU_GPIO_3 */
    PA4,            /* MCU_GPIO_4 */
    PA5,            /* MCU_GPIO_5 */
    PA6,            /* MCU_GPIO_6 */
    PA7,            /* MCU_GPIO_7 */
    PA8,            /* MCU_GPIO_8 */
    PA9,            /* MCU_GPIO_9 */
    PA10,           /* MCU_GPIO_10 */
    PA11,           /* MCU_GPIO_11 */
    PA12,           /* MCU_GPIO_12 */
    PA13,           /* MCU_GPIO_13 */
    PA14,           /* MCU_GPIO_14 */
    PA15,           /* MCU_GPIO_15 */

    PB0 = 1 << 4, PB1, PB2, PB3, PB4, PB5, PB6, PB7, /* Reserve pins */
    PB8,            /* MCU_GPIO_24 */
    PB9,            /* MCU_GPIO_25 */
    PB10,           /* MCU_GPIO_26 */
    PB11,           /* MCU_GPIO_27 */
    PB12,           /* MCU_GPIO_28 */
    PB13,           /* MCU_GPIO_29 */
    PB14,           /* MCU_GPIO_30 */
    PB15,           /* MCU_GPIO_31 */

    PC0 = 2 << 4,   /* MCU_I2C0_DATA */
    PC1,            /* MCU_I2C0_CLK */
    PC2,            /* MCU_I2C1_DATA */
    PC3,            /* MCU_I2C1_CLK */
    PC4,            /* MCU_I2C2_DATA */
    PC5,            /* MCU_I2C2_CLK */
    PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15, /* Reserve pins */

    PD0 = 3 << 4,   /* MCU_UA0_RXD */
    PD1,            /* MCU_UA0_TXD */
    PD2, PD3,       /* Reserve pins */
    PD4,            /* MCU_UA1_RXD */ 
    PD5,            /* MCU_UA1_TXD */
    PD6,            /* MCU_UA1_RTS_N */
    PD7,            /* MCU_UA1_CTS_N */
    PD8,            /* MCU_UA2_RXD */
    PD9,            /* MCU_UA2_TXD */
    PD10,           /* MCU_UA2_RTS_N */
    PD11,           /* MCU_UA2_CTS_N */
    PD12, PD13, PD14, PD15, /* Reserve pins */

    PE0 = 4 << 4,   /* MCU_SPIM0_CS_N */
    PE1,            /* MCU_SPIM0_CLK */
    PE2,            /* MCU_SPIM0_MOSI */
    PE3,            /* MCU_SPIM0_MISO */
    PE4,            /* MCU_SPIM1_CS_N */
    PE5,            /* MCU_SPIM1_CLK */
    PE6,            /* MCU_SPIM1_MOSI */
    PE7,            /* MCU_SPIM1_MISO */
    PE8,            /* MCU_SPIM2_CS_N */
    PE9,            /* MCU_SPIM2_CLK */
    PE10,           /* MCU_SPIM2_MOSI */
    PE11,           /* MCU_SPIM2_MISO */
    PE12,           /* MCU_SPIM3_CS_N */
    PE13,           /* MCU_SPIM3_CLK */
    PE14,           /* MCU_SPIM3_MOSI */
    PE15,           /* MCU_SPIM3_MISO */

    PF0 = 5 << 4,   /* MCU_SPIC_CS_N */
    PF1,            /* MCU_SPIC_CLK */
    PF2,            /* MCU_SPIC_MOSI */
    PF3,            /* MCU_SPIC_MISO */
    PF4,            /* MCU_SPIC_IO2 */
    PF5,            /* MCU_SPIC_IO3 */
    PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15, /* Reserve pins */

    PG0 = 6 << 4,   /* MCU_ADC24_SYNC */
    PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15, /* Reserve pins */

    /* Non Mulifunction IO pins */
    PH0 = 7 << 4,   /* MCU_ADC_AIN0 */
    PH1,            /* MCU_ADC_AIN1 */
    PH2,            /* MCU_ADC_AIN2 */
    PH3,            /* MCU_ADC_AIN3 */
    PH4,            /* MCU_ADC_AIN4 */
    PH5,            /* MCU_ADC_AIN5 */
    PH6, PH7, PH8, PH9, PH10, PH11, PH12, PH13, PH14, PH15, /* Reserve pins */

    /* LED */
    LED1 = PA10,    /* Red */
    LED2 = PA11,    /* Green */
    LED3 = LED1,    /* Red */
    LED4 = LED2,    /* Green */

    /* mbed DIP Pin Names */
    USBTX = PD5, /* MCU_UA1_TXD */
    USBRX = PD4, /* MCU_UA1_RXD */
    BUTTON = PA6,

    /* Arch Pro Pin Names (for mbed test MBED_BUS) */
    D0 = PA7,    /* MCU_GPIO_7 */
    D1 = PA8,    /* MCU_GPIO_8 */
    D2 = PA9,    /* MCU_GPIO_9 */
    D3 = PE4,    /* MCU_GPIO_20 */
    D4 = PE5,    /* MCU_GPIO_21 */
    D5 = PE6,    /* MCU_GPIO_22 */
    D6 = PE7,    /* MCU_GPIO_23 */
    D7 = PE3,    /* MCU_GPIO_19 */
    D8 = PE2,    /* MCU_GPIO_18 */
    D9 = PE1,    /* MCU_GPIO_17 */
    D10 = PE0,   /* MCU_GPIO_16 */

    A0 = PH0,    /* MCU_ADC_AIN0 */
    A1 = PH1,    /* MCU_ADC_AIN1 */
    A2 = PH2,    /* MCU_ADC_AIN2 */
    A3 = PH3,    /* MCU_ADC_AIN3 */
    A4 = PH4,    /* MCU_ADC_AIN4 */
    A5 = PH5,    /* MCU_ADC_AIN5 */

    I2C_SCL = PC1,    /* I2C_0_SCL */
    I2C_SDA = PC0,    /* I2C_0_SDA */

    /* BlueNinja breakout board pin names */
    p2  = PA7,        /* MCU_GPIO_7 */
    p3  = PA8,        /* MCU_GPIO_8/PWM0 */
    p4  = PA9,        /* MCU_GPIO_9/PWM1 */
    p5  = PD0,        /* MCU_UA0_RXD/GPIO_20 */
    p6  = PD1,        /* MCU_UA0_TXD/GPIO_21 */
    p7  = PE4,        /* MCU_SPIM1_CS_N/GPIO_20/UA0_RXD */
    p8  = PE5,        /* MCU_SPIM1_CLK/GPIO_21/UA0_TXD */
    p9  = PE6,        /* MCU_SPIM1_MOSI/GPIO_22 */
    p10 = PE7,        /* MCU_SPIM1_MISO/GPIO_23 */
    p14 = PA1,        /* MCU_GPIO_1 */
    p16 = PA0,        /* MCU_GPIO_0 */
    p22 = PH0,        /* MCU_ADC_AIN0 */
    p23 = PH1,        /* MCU_ADC_AIN1 */
    p25 = PH2,        /* MCU_ADC_AIN2 */
    p26 = PH3,        /* MCU_ADC_AIN3 */
    p28 = PH4,        /* MCU_ADC_AIN4 */
    p29 = PH5,        /* MCU_ADC_AIN5 */
    p32 = PC0,        /* MCU_I2C0_DATA/CAPTURE2/GPIO_16 */
    p33 = PC1,        /* MCU_I2C0_CLK/CAPTURE3/GPIO_17 */
    p36 = PE0,        /* MCU_SPIM0_CS_N/GPIO_16/PWM0 */
    p37 = PE1,        /* MCU_SPIM0_CLK/GPIO_17/PWM1 */
    p38 = PE2,        /* MCU_SPIM0_MOSI/GPIO_18/PWM2 */
    p39 = PE3,        /* MCU_SPIM0_MISO/GPIO_19 */

    /* Not connected */
    NC = (int)0xFFFFFFFF
} PinName;

typedef enum {
    PullDown,
    PullNone,
    PullUp,

    Cap2mA,
    Cap4mA,
    Cap5mA,
    Cap7mA,

    InputStandby,
    InputActive,

    PullDefault = PullNone
} PinMode;

#define PINGROUP(pin) ((pin)&0xf0)
#define PINNO(pin) ((pin)&0x0f)

/* bit 31..16: PinName, bit 15..0: PinId */
#define PIN_ID_ROW(PinName, PinId) (int)(((PinName) << 16)| ((PinId) << 0))
#define PINNAME(X)    ( ((X) & 0xffff0000) >> 16)
#define PINID(X)      ((X) & 0xffff)

#ifdef __cplusplus
}
#endif

#endif
