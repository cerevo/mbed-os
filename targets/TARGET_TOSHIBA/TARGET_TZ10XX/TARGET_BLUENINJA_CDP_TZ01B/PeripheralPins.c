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

#include "PeripheralPins.h"
#include "RTE_Device.h"

/* =====
   Note: Commented lines are alternative possibilities which are not used per default.
         If you change them, you will have also to modify the corresponding xxx_api.c file
         for pwmout, analogin, analogout, ...
  =====
*/

/* GPIO */
/* GPIO */
const PinMap PinMap_GPIO[] = {
    {PA0,  GPIO_0, 1}, /* MCU_GPIO_0:  GPIO_0 */
    {PA1,  GPIO_1, 1}, /* MCU_GPIO_1:  GPIO_1 */
    {PA2,  GPIO_2, 1}, /* MCU_GPIO_2:  GPIO_2 */
    {PA3,  GPIO_3, 1}, /* MCU_GPIO_3:  GPIO_3 */
    {PA4,  GPIO_4, 1}, /* MCU_GPIO_4:  GPIO_4 */
    {PA5,  GPIO_5, 1}, /* MCU_GPIO_5:  GPIO_5 */
    {PA6,  GPIO_6, 1}, /* MCU_GPIO_6:  GPIO_6 */
    {PA7,  GPIO_7, 1}, /* MCU_GPIO_7:  GPIO_7 */
    {PA8,  GPIO_8, 1}, /* MCU_GPIO_8:  GPIO_8 */
    {PA9,  GPIO_9, 1}, /* MCU_GPIO_9:  GPIO_9 */
    {PA10, GPIO_10, 1}, /* MCU_GPIO_10: GPIO_10 */
    {PA11, GPIO_11, 1}, /* MCU_GPIO_11: GPIO_11 */
    {PA12, GPIO_12, 1}, /* MCU_GPIO_12: GPIO_12 */
    {PA13, GPIO_13, 1}, /* MCU_GPIO_13: GPIO_13 */
    {PA14, GPIO_14, 1}, /* MCU_GPIO_14: GPIO_14 */
    {PA15, GPIO_15, 1}, /* MCU_GPIO_15: GPIO_15 */

    {PE0,  GPIO_16, 2}, /* MCU_SPIM0_CS_N: GPIO_16 */
    {PE1,  GPIO_17, 2}, /* MCU_SPIM0_CLK:  GPIO_17 */
    {PE2,  GPIO_18, 2}, /* MCU_SPIM0_MOSI: GPIO_18 */
    {PE3,  GPIO_19, 2}, /* MCU_SPIM0_MISO: GPIO_19 */
    {PE4,  GPIO_20, 2}, /* MCU_SPIM1_CS_N: GPIO_20 */
    {PE5,  GPIO_21, 2}, /* MCU_SPIM1_CLK:  GPIO_21 */
    {PE6,  GPIO_22, 2}, /* MCU_SPIM1_MOSI: GPIO_22 */
    {PE7,  GPIO_23, 2}, /* MCU_SPIM1_MISO: GPIO_23 */

    {PB8,  GPIO_24, 1}, /* MCU_GPIO_24: GPIO_24 */
    {PB9,  GPIO_25, 1}, /* MCU_GPIO_25: GPIO_25 */
    {PB10, GPIO_26, 1}, /* MCU_GPIO_26: GPIO_26 */
    {PB11, GPIO_27, 1}, /* MCU_GPIO_27: GPIO_27 */
    {PB12, GPIO_28, 1}, /* MCU_GPIO_28: GPIO_28 */
    {PB13, GPIO_29, 1}, /* MCU_GPIO_29: GPIO_29 */
    {PB14, GPIO_30, 1}, /* MCU_GPIO_30: GPIO_30 */
    {PB15, GPIO_31, 1}, /* MCU_GPIO_31: GPIO_31 */

    {PG0,  GPIO_8, 3}, /* MCU_ADC24_SYNC: GPIO_8 */
    {PD10, GPIO_10, 3}, /* MCU_UA2_RTS_N:  GPIO_10 */
    {PD11, GPIO_11, 3}, /* MCU_UA2_CTS_N:  GPIO_11 */
    {PD4,  GPIO_12, 3}, /* MCU_UA1_RXD:    GPIO_12 */
    {PD5,  GPIO_13, 3}, /* MCU_UA1_TXD:    GPIO_13 */
    {PC2,  GPIO_14, 3}, /* MCU_I2C1_DATA:  GPIO_14 */
    {PC3,  GPIO_15, 3}, /* MCU_I2C1_CLK:   GPIO_15 */

    {PC0,  GPIO_16, 3}, /* MCU_I2C0_DATA:  GPIO_16 */
    {PC1,  GPIO_17, 3}, /* MCU_I2C0_CLK:   GPIO_17 */
    {PD8,  GPIO_18, 3}, /* MCU_UA2_RXD:    GPIO_18 */
    {PD9,  GPIO_19, 3}, /* MCU_UA2_TXD:    GPIO_19 */
    {PD0,  GPIO_20, 3}, /* MCU_UA0_RXD:    GPIO_20 */
    {PD1,  GPIO_21, 3}, /* MCU_UA0_TXD:    GPIO_21 */
    {PD6,  GPIO_22, 3}, /* MCU_UA1_RTS_N:  GPIO_22 */
    {PD7,  GPIO_23, 3}, /* MCU_UA1_CTS_N:  GPIO_23 */

    {NC,   NC,    0}
};

/* ADC */
const PinMap PinMap_ADC12[] = {
    {PH0,  ADC12_0, 0}, /* MCU_ADC_AIN0: Non multifunction */
    {PH1,  ADC12_1, 0}, /* MCU_ADC_AIN1: Non multifunction */
    {PH2,  ADC12_2, 0}, /* MCU_ADC_AIN2: Non multifunction */
    {PH3,  ADC12_3, 0}, /* MCU_ADC_AIN3: Non multifunction */

    {NC,   NC,    0}
};

const PinMap PinMap_ADC24[] = {
    {PH0,  ADC24_0, 0}, /* MCU_ADC_AIN0: Non multifunction */
    {PH1,  ADC24_0, 0}, /* MCU_ADC_AIN1: Non multifunction */
    {PH2,  ADC24_1, 0}, /* MCU_ADC_AIN3: Non multifunction */
    {PH3,  ADC24_1, 0}, /* MCU_ADC_AIN4: Non multifunction */
    {PH4,  ADC24_2, 0}, /* MCU_ADC_AIN5: Non multifunction */
    {PH5,  ADC24_2, 0}, /* MCU_ADC_AIN6: Non multifunction */

    {PG0,  ADC24_0, 1},  /* MCU_ADC24_SYNC: ADC24_SYNC */
    {PG0,  ADC24_1, 1},  /* MCU_ADC24_SYNC: ADC24_SYNC */
    {PG0,  ADC24_2, 1},  /* MCU_ADC24_SYNC: ADC24_SYNC */

    {NC,   NC,    0}
};

/* DAC */
const PinMap PinMap_DAC[] = {
    {NC,   NC,    0}
};

/* I2C */
const PinMap PinMap_I2C_SDA[] = {
    {PC0,  I2C_0, 1}, /* MCU_I2C0_DATA: I2C0_DATA */
    {PC2,  I2C_1, 1}, /* MCU_I2C1_DATA: I2C1_DATA */
    {PC4,  I2C_2, 1}, /* MCU_I2C2_DATA: I2C2_DATA */

    {PD5,  I2C_0, 2}, /* MCU_UA1_TXD:   I2C0_DATA */
    {PD6,  I2C_2, 2}, /* MCU_UA1_RTS_N: I2C2_DATA */

    {NC,    NC,    0}
};

const PinMap PinMap_I2C_SCL[] = {
    {PC1,  I2C_0, 1}, /* MCU_I2C0_CLK: I2C0_CLK */
    {PC3,  I2C_1, 1}, /* MCU_I2C1_CLK: I2C1_CLK */
    {PC5,  I2C_2, 1}, /* MCU_I2C2_CLK: I2C2_CLK -> same mask as PC4 when set function */

    {PD4,  I2C_0, 2}, /* MCU_UA1_RXD:   I2C0_CLK */
    {PD7,  I2C_2, 2}, /* MCU_UA1_CTS_N: I2C2_CLK */

    {NC,    NC,    0}
};

/* PWM */
const PinMap PinMap_PWM[] = {
    {PA8,   PWM_0, 2}, /* MCU_GPIO_8:  PWM0 */
    {PA9,   PWM_1, 2}, /* MCU_GPIO_9:  PWM1 */
    {PA10,  PWM_2, 2}, /* MCU_GPIO_10: PWM2 */
    {PA11,  PWM_3, 2}, /* MCU_GPIO_11: PWM3 */

    {PE0,  PWM_0, 3},  /* MCU_SPIM0_CS_N: PWM0 */
    {PE1,  PWM_1, 3},  /* MCU_SPIM0_CLK:  PWM1 */
    {PE2,  PWM_2, 3},  /* MCU_SPIM0_MOSI: PWM2 */
    {PB11, PWM_3, 3},  /* MCU_GPIO_27:    PWM3 */

    {NC,    NC,    0}
};

/* SERIAL */
const PinMap PinMap_UART_TX[] = {
    {PD1, UART_0, 1}, /* MCU_UA0_TXD: UA0_TXD */
    {PD5, UART_1, 1}, /* MCU_UA1_TXD: UA1_TXD */
#if (!RTE_BLE)
    {PD9, UART_2, 1}, /* MCU_UA2_TXD: UA2_TXD */
#endif
    {PE5, UART_0, 3}, /* MCU_SPIM1_CLK: UA0_TXD */

    {NC,    NC,    0}
};

const PinMap PinMap_UART_RX[] = {
    {PD0, UART_0, 1}, /* MCU_UA0_RXD: UA0_RXD */
    {PD4, UART_1, 1}, /* MCU_UA1_RXD: UA1_RXD */
#if (!RTE_BLE)
    {PD8, UART_2, 1}, /* MCU_UA2_RXD: UA2_RXD */
#endif
    {PE4, UART_0, 3}, /* MCU_SPIM1_CS_N: UA0_RXD */
    {NC,    NC,    0}
};

#if (DEVICE_SERIAL_FC)
/* RTS/CTS PinMap for flow control */
const PinMap PinMap_UART_CTS[] = {
    {PC3, UART_0, 2}, /* MCU_I2C1_CLK: UA0_CTS_N */
    {PD7, UART_1, 1}, /* MCU_UA1_CTS_N: UA1_CTS_N */
#if (!RTE_BLE)
    {PD11, UART_2, 1},/* MCU_UA2_CTS_N: UA2_CTS_N */
#endif
    {PA15, UART_0, 3},/* MCU_GPIO_15: UA0_CTS_N */
    {NC    , NC   , 0}
};
const PinMap PinMap_UART_RTS[] = {
    {PC2, UART_0, 2}, /* MCU_I2C1_DATA: UA0_RTS_N */
    {PD6, UART_1, 1}, /* MCU_UA1_RTS_N: UA1_RTS_N */
#if (!RTE_BLE)
    {PD10, UART_2, 1},/* MCU_UA2_RTS_N: UA2_RTS_N */
#endif
    {PA14, UART_0, 3},/* MCU_GPIO_14: MCU_GPIO_14 */
    {NC    , NC   , 0}
};
#endif

/* SPI */
const PinMap PinMap_SPI_MOSI[] = {
    {PE2,  SPIM_0, 1}, /* MCU_SPIM0_MOSI: SPIM0_MOSI */
    {PE6,  SPIM_1, 1}, /* MCU_SPIM1_MOSI: SPIM1_MOSI */
    {PE10, SPIM_2, 1}, /* MCU_SPIM2_MOSI: SPIM2_MOSI -> same mask as PE8 when set function */
    {PE14, SPIM_3, 1}, /* MCU_SPIM3_MOSI: SPIM3_MOSI -> same mask as PE12 when set function */
    {PA14, SPIM_3, 2}, /* MCU_GPIO_14: SPIM3_MOSI */

    {NC,    NC,    0}
};

const PinMap PinMap_SPI_MISO[] = {
    {PE3,  SPIM_0, 1}, /* MCU_SPIM0_MISO: SPIM0_MISO */
    {PE7,  SPIM_1, 1}, /* MCU_SPIM1_MISO: SPIM1_MISO */
    {PE11, SPIM_2, 1}, /* MCU_SPIM2_MISO: SPIM2_MISO -> same mask as PE8 when set function */
    {PE15, SPIM_3, 1}, /* MCU_SPIM3_MISO: SPIM3_MISO -> same mask as PE12 when set function */
    {PA15, SPIM_3, 2}, /* MCU_GPIO_15: SPIM3_MISO */
    {NC,    NC,    0}
};

const PinMap PinMap_SPI_SCLK[] = {
    {PE1,  SPIM_0, 1}, /* MCU_SPIM0_CLK: SPIM0_CLK */
    {PE5,  SPIM_1, 1}, /* MCU_SPIM1_CLK: SPIM1_CLK */
    {PE9,  SPIM_2, 1}, /* MCU_SPIM2_CLK: SPIM2_CLK -> same mask as PE8 when set function */
    {PE13, SPIM_3, 1}, /* MCU_SPIM3_CLK: SPIM3_CLK -> same mask as PE12 when set function */
    {PA13, SPIM_3, 2}, /* MCU_GPIO_13: SPIM3_CLK */

    {NC,    NC,    0}
};

const PinMap PinMap_SPI_SCSN[] = {
    {PE0,  SPIM_0, 1}, /* MCU_SPIM0_CS_N: SPIM0_CS_N */
    {PE4,  SPIM_1, 1}, /* MCU_SPIM1_CS_N: SPIM1_CS_N */
    {PE8,  SPIM_2, 1}, /* MCU_SPIM2_CS_N: SPIM2_CS_N */
    {PE12, SPIM_3, 1}, /* MCU_SPIM3_CS_N: SPIM3_CS_N */
    {PA12, SPIM_3, 2}, /* MCU_GPIO_12: SPIM3_CS_N */
    {NC,    NC,    0}
};

const PinMap PinMap_SPI_SPIC[] = {
    {PF0,  SPIC, 0}, /* MCU_SPIC_CS_N: SPIC_CS_N */
    {PF1,  SPIC, 0}, /* MCU_SPIC_CLK:  SPIC_CLK   -> same mask as PF0 when set function */
    {PF2,  SPIC, 0}, /* MCU_SPIC_MOSI: SPIC_MOSI  -> same mask as PF0 when set function */
    {PF3,  SPIC, 0}, /* MCU_SPIC_MISO: SPIC_MISO  -> same mask as PF0 when set function */
    {PF4,  SPIC, 0}, /* MCU_SPIC_IO2:  SPIC_IO2    -> same mask as PF0 when set function */
    {PF5,  SPIC, 0}, /* MCU_SPIC_IO3:  SPIC_IO3    -> same mask as PF0 when set function */

    {NC,   NC,   0}
};

