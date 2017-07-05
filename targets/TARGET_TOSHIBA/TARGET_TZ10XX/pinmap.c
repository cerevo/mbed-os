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
#include "pinmap.h"
#include "gpio_api.h"
#include "mbed_error.h"
#include "mbed_assert.h"
#include "PMU_TZ10xx.h"

/** Get pin ID.
 *  @remarks
 *    For pin ID, refer to Chapter ...
 *
 *  @param  pin             The pin to be get ID.
 *
 *  @returns
 *    The ID of pin.
 */
PMU_IO_PIN get_pin_id(PinName pin)
{
    int i = 0;

    /* Check that pin is valid */
    MBED_ASSERT(pin != NC);

    static const uint32_t pin_id_tbl[] = {
        PIN_ID_ROW(PA0,   PMU_IO_PIN_GPIO_0),
        PIN_ID_ROW(PA1,   PMU_IO_PIN_GPIO_1),
        PIN_ID_ROW(PA2,   PMU_IO_PIN_GPIO_2),
        PIN_ID_ROW(PA3,   PMU_IO_PIN_GPIO_3),
        PIN_ID_ROW(PA4,   PMU_IO_PIN_GPIO_4),
        PIN_ID_ROW(PA5,   PMU_IO_PIN_GPIO_5),
        PIN_ID_ROW(PA6,   PMU_IO_PIN_GPIO_6),
        PIN_ID_ROW(PA7,   PMU_IO_PIN_GPIO_7),
        PIN_ID_ROW(PA8,   PMU_IO_PIN_GPIO_8),
        PIN_ID_ROW(PA9,   PMU_IO_PIN_GPIO_9),
        PIN_ID_ROW(PA10,  PMU_IO_PIN_GPIO_10),
        PIN_ID_ROW(PA11,  PMU_IO_PIN_GPIO_11),
        PIN_ID_ROW(PA12,  PMU_IO_PIN_GPIO_12),
        PIN_ID_ROW(PA13,  PMU_IO_PIN_GPIO_13),
        PIN_ID_ROW(PA14,  PMU_IO_PIN_GPIO_14),
        PIN_ID_ROW(PA15,  PMU_IO_PIN_GPIO_15),

        PIN_ID_ROW(PB8,   PMU_IO_PIN_GPIO_24),
        PIN_ID_ROW(PB9,   PMU_IO_PIN_GPIO_25),
        PIN_ID_ROW(PB10,  PMU_IO_PIN_GPIO_26),
        PIN_ID_ROW(PB11,  PMU_IO_PIN_GPIO_27),
        PIN_ID_ROW(PB12,  PMU_IO_PIN_GPIO_28),
        PIN_ID_ROW(PB13,  PMU_IO_PIN_GPIO_29),
        PIN_ID_ROW(PB14,  PMU_IO_PIN_GPIO_30),
        PIN_ID_ROW(PB15,  PMU_IO_PIN_GPIO_31),

        PIN_ID_ROW(PC0,   PMU_IO_PIN_I2C0_DATA),
        PIN_ID_ROW(PC1,   PMU_IO_PIN_I2C0_CLK),
        PIN_ID_ROW(PC2,   PMU_IO_PIN_I2C1_DATA),
        PIN_ID_ROW(PC3,   PMU_IO_PIN_I2C1_CLK),
        PIN_ID_ROW(PC4,   PMU_IO_PIN_I2C2_DATA),
        PIN_ID_ROW(PC5,   PMU_IO_PIN_I2C2_CLK),

        PIN_ID_ROW(PD0,  PMU_IO_PIN_UA0_RXD),
        PIN_ID_ROW(PD1,  PMU_IO_PIN_UA0_TXD),
        PIN_ID_ROW(PD4,  PMU_IO_PIN_UA1_RXD),
        PIN_ID_ROW(PD5,  PMU_IO_PIN_UA1_TXD),
        PIN_ID_ROW(PD6,  PMU_IO_PIN_UA1_RTS_N),
        PIN_ID_ROW(PD7,  PMU_IO_PIN_UA1_CTS_N),
        PIN_ID_ROW(PD8,  PMU_IO_PIN_UA2_RXD),
        PIN_ID_ROW(PD9,  PMU_IO_PIN_UA2_TXD),
        PIN_ID_ROW(PD10, PMU_IO_PIN_UA2_RTS_N),
        PIN_ID_ROW(PD11, PMU_IO_PIN_UA2_CTS_N),

        PIN_ID_ROW(PE0,   PMU_IO_PIN_SPIM0_CS_N),
        PIN_ID_ROW(PE1,   PMU_IO_PIN_SPIM0_CLK),
        PIN_ID_ROW(PE2,   PMU_IO_PIN_SPIM0_MOSI),
        PIN_ID_ROW(PE3,   PMU_IO_PIN_SPIM0_MISO),
        PIN_ID_ROW(PE4,   PMU_IO_PIN_SPIM1_CS_N),
        PIN_ID_ROW(PE5,   PMU_IO_PIN_SPIM1_CLK),
        PIN_ID_ROW(PE6,   PMU_IO_PIN_SPIM1_MOSI),
        PIN_ID_ROW(PE7,   PMU_IO_PIN_SPIM1_MISO),
        PIN_ID_ROW(PE8,   PMU_IO_PIN_SPIM2_CS_N),
        PIN_ID_ROW(PE9,   PMU_IO_PIN_SPIM2_CLK),
        PIN_ID_ROW(PE10,  PMU_IO_PIN_SPIM2_MOSI),
        PIN_ID_ROW(PE11,  PMU_IO_PIN_SPIM2_MISO),
        PIN_ID_ROW(PE12,  PMU_IO_PIN_SPIM3_CS_N),
        PIN_ID_ROW(PE13,  PMU_IO_PIN_SPIM3_CLK),
        PIN_ID_ROW(PE14,  PMU_IO_PIN_SPIM3_MOSI),
        PIN_ID_ROW(PE15,  PMU_IO_PIN_SPIM3_MISO),

        PIN_ID_ROW(PF0,   PMU_IO_PIN_SPIC_CS_N),
        PIN_ID_ROW(PF1,   PMU_IO_PIN_SPIC_CLK),
        PIN_ID_ROW(PF2,   PMU_IO_PIN_SPIC_MOSI),
        PIN_ID_ROW(PF3,   PMU_IO_PIN_SPIC_MISO),
        PIN_ID_ROW(PF4,   PMU_IO_PIN_SPIC_IO2),
        PIN_ID_ROW(PF5,   PMU_IO_PIN_SPIC_IO3),

        PIN_ID_ROW(PG0,   PMU_IO_PIN_ADC24_SYNC),

        (uint32_t) NC

    };

    /* Return the ID coressponding the pin */
    for (i = 0; pin_id_tbl[i] != (uint32_t)NC; i++) {
        if (PINNAME(pin_id_tbl[i]) == pin) {
            return (PMU_IO_PIN) PINID(pin_id_tbl[i]);
        }
    }

    return PMU_IO_PIN_INVALID;
}

/** Configure pin function.
 *  @remarks
 *    For pin function, refer to Chapter 6.1.
 *    Setting Multiple function I/O.
 *
 *  @param  pin             The pin to be set function.
 *  @param  function        Function to be set to pin.
 *
 *  @returns
 *    NONE.
 */
void pin_function(PinName pin, int function)
{
    __IO uint32_t *reg = NULL;
    uint32_t t         = 0;

    /* Check that pin is valid */
    MBED_ASSERT(pin != (PinName)NC);

    static const struct {
        uint8_t reg; /* gconf->FMODE_CFG_* */
        uint8_t bit;
    } tbl[] = {
        {0, 0}, /* GPIO_0 */
        {0, 2}, /* GPIO_1 */
        {0, 4}, /* GPIO_2 */
        {0, 6}, /* GPIO_3 */
        {0, 8}, /* GPIO_4 */
        {0, 10}, /* GPIO_5 */
        {0, 12}, /* GPIO_6 */
        {0, 14}, /* GPIO_7 */
        {0, 16}, /* GPIO_8 */
        {0, 18}, /* GPIO_9 */
        {0, 20}, /* GPIO_10 */
        {0, 22}, /* GPIO_11 */
        {0, 24}, /* GPIO_12 */
        {0, 26}, /* GPIO_13 */
        {0, 28}, /* GPIO_14 */
        {0, 30}, /* GPIO_15 */
        {1, 16}, /* GPIO_24 */
        {1, 18}, /* GPIO_25 */
        {1, 20}, /* GPIO_26 */
        {1, 22}, /* GPIO_27 */
        {1, 24}, /* GPIO_28 */
        {1, 26}, /* GPIO_29 */
        {1, 28}, /* GPIO_30 */
        {1, 30}, /* GPIO_31 */
        {2,  0}, /* I2C0_DATA */
        {2,  2}, /* I2C0_CLK */
        {2,  4}, /* I2C1_DATA */
        {2,  6}, /* I2C1_CLK */
        {2,  8}, /* I2C2_DATA */
        {2, 10}, /* I2C2_CLK */
        {3,  0}, /* UA0_RXD */
        {3,  2}, /* UA0_TXD */
        {3,  8}, /* UA1_RXD */
        {3, 10}, /* UA1_TXD */
        {3, 12}, /* UA1_RTS_N */
        {3, 14}, /* UA1_CTS_N */
        {3, 16}, /* UA2_RXD */
        {3, 18}, /* UA2_TXD */
        {3, 20}, /* UA2_RTS_N */
        {3, 22}, /* UA2_CTS_N */
        {4,  0}, /* SPIM0_CS_N */
        {4,  2}, /* SPIM0_CLK */
        {4,  4}, /* SPIM0_MOSI */
        {4,  6}, /* SPIM0_MISO */
        {4,  8}, /* SPIM1_CS_N */
        {4, 10}, /* SPIM1_CLK */
        {4, 12}, /* SPIM1_MOSI */
        {4, 14}, /* SPIM1_MISO */
        {4, 16}, /* SPIM2_CS_N */
        {4, 16}, /* SPIM2_CLK */
        {4, 16}, /* SPIM2_MOSI */
        {4, 16}, /* SPIM2_MISO */
        {4, 24}, /* SPIM3_CS_N */
        {4, 24}, /* SPIM3_CLK */
        {4, 24}, /* SPIM3_MOSI */
        {4, 24}, /* SPIM3_MISO */
        {5,  0}, /* SPIC_CS_N */
        {5,  0}, /* SPIC_CLK */
        {5,  0}, /* SPIC_MOSI */
        {5,  0}, /* SPIC_MISO */
        {5,  0}, /* SPIC_IO2 */
        {5,  0}, /* SPIC_IO3 */
        {6,  0}, /* ADC24_SYNC */
    };

    /* Get pin id */
    PMU_IO_PIN pin_id = get_pin_id(pin);
    MBED_ASSERT(pin_id != PMU_IO_PIN_INVALID);

    /* set pin function */
    reg = &gconf->FMODE_CFG0 + tbl[pin_id].reg;
    t = *reg;
    t &= ~(3u << tbl[pin_id].bit);
    t |= function << tbl[pin_id].bit;
    *reg = t;
}

/** Set mode for pin of port object to pull-up/pull-down.
 *
 *  @param  pin             The pin to be set mode.
 *  @param  mode            The mode to be set to pin.
 *
 *  @returns
 *    NONE.
 */
void pin_mode(PinName pin, PinMode mode)
{
    uint32_t set_pull  = 0;
    uint32_t set_capa  = 0;
    uint32_t cur_pull  = 0;
    uint32_t cur_capa  = 0;

    __IO uint32_t *reg = NULL;
    __IO uint32_t *reg1 = NULL;
    uint32_t t         = 0;

    /* Check that pin is valid */
    MBED_ASSERT(pin != (PinName)NC);

    static const struct {
        uint8_t reg; /* gconf->IO_CFG_* */
        uint8_t bit;
    } tbl[] = {
        {0, 99}, /* GPIO_0 */
        {0,  4}, /* GPIO_1 */
        {0,  8}, /* GPIO_2 */
        {0, 12}, /* GPIO_3 */
        {0, 16}, /* GPIO_4 */
        {0, 20}, /* GPIO_5 */
        {0, 24}, /* GPIO_6 */
        {0, 28}, /* GPIO_7 */
        {1,  0}, /* GPIO_8 */
        {1,  4}, /* GPIO_9 */
        {1,  8}, /* GPIO_10 */
        {1, 12}, /* GPIO_11 */
        {1, 16}, /* GPIO_12 */
        {1, 20}, /* GPIO_13 */
        {1, 24}, /* GPIO_14 */
        {1, 28}, /* GPIO_15 */
        {3,  0}, /* GPIO_24 */
        {3,  4}, /* GPIO_25 */
        {3,  8}, /* GPIO_26 */
        {3, 12}, /* GPIO_27 */
        {3, 16}, /* GPIO_28 */
        {3, 20}, /* GPIO_29 */
        {3, 24}, /* GPIO_30 */
        {3, 28}, /* GPIO_31 */
        {4,  0}, /* I2C0_DATA */
        {4,  4}, /* I2C0_CLK */
        {4,  8}, /* I2C1_DATA */
        {4, 12}, /* I2C1_CLK */
        {4, 16}, /* I2C2_DATA */
        {4, 20}, /* I2C2_CLK */
        {5,  0}, /* UA0_RXD */
        {5,  4}, /* UA0_TXD */
        {5, 16}, /* UA1_RXD */
        {5, 20}, /* UA1_TXD */
        {5, 24}, /* UA1_RTS_N */
        {5, 28}, /* UA1_CTS_N */
        {6,  0}, /* UA2_RXD */
        {6,  4}, /* UA2_TXD */
        {6,  8}, /* UA2_RTS_N */
        {6, 12}, /* UA2_CTS_N */
        {7,  0}, /* SPIM0_CS_N */
        {7,  4}, /* SPIM0_CLK */
        {7,  8}, /* SPIM0_MOSI */
        {7, 12}, /* SPIM0_MISO */
        {7, 16}, /* SPIM1_CS_N */
        {7, 20}, /* SPIM1_CLK */
        {7, 24}, /* SPIM1_MOSI */
        {7, 28}, /* SPIM1_MISO */
        {8,  0}, /* SPIM2_CS_N */
        {8,  4}, /* SPIM2_CLK */
        {8,  8}, /* SPIM2_MOSI */
        {8, 12}, /* SPIM2_MISO */
        {8, 16}, /* SPIM3_CS_N */
        {8, 20}, /* SPIM3_CLK */
        {8, 24}, /* SPIM3_MOSI */
        {8, 28}, /* SPIM3_MISO */
        {9,  0}, /* SPIC_CS_N */
        {9,  4}, /* SPIC_CLK */
        {9,  8}, /* SPIC_MOSI */
        {9, 12}, /* SPIC_MISO */
        {9, 16}, /* SPIC_IO2 */
        {9, 20}, /* SPIC_IO3 */
        {10, 0}, /* ADC24_SYNC */
    };

    static const struct {
        uint8_t reg; /* CTRL_IO_AON_* */
        uint8_t bit;
    } tbl1[] = {
        {0, 99}, /* GPIO_0 */
        {3,  1}, /* GPIO_1 */
        {3,  2}, /* GPIO_2 */
        {3,  3}, /* GPIO_3 */
        {3,  4}, /* GPIO_4 */
        {3,  5}, /* GPIO_5 */
        {3,  6}, /* GPIO_6 */
        {3,  7}, /* GPIO_7 */
        {3,  8}, /* GPIO_8 */
        {3,  9}, /* GPIO_9 */
        {3, 10}, /* GPIO_10 */
        {3, 11}, /* GPIO_11 */
        {3, 12}, /* GPIO_12 */
        {3, 13}, /* GPIO_13 */
        {3, 14}, /* GPIO_14 */
        {3, 15}, /* GPIO_15 */
        {3, 24}, /* GPIO_24 */
        {3, 25}, /* GPIO_25 */
        {3, 26}, /* GPIO_26 */
        {3, 27}, /* GPIO_27 */
        {3, 28}, /* GPIO_28 */
        {3, 29}, /* GPIO_29 */
        {3, 30}, /* GPIO_30 */
        {3, 31}, /* GPIO_31 */
        {6, 12}, /* I2C0_DATA */
        {6, 13}, /* I2C0_CLK */
        {6, 14}, /* I2C1_DATA */
        {6, 15}, /* I2C1_CLK */
        {2,  0}, /* I2C2_DATA */
        {2,  0}, /* I2C2_CLK */
        {6,  0}, /* UA0_RXD */
        {6,  0}, /* UA0_TXD */
        {6,  8}, /* UA1_RXD */
        {6,  9}, /* UA1_TXD */
        {6, 10}, /* UA1_RTS_N */
        {6, 11}, /* UA1_CTS_N */
        {2,  4}, /* UA2_RXD */
        {2,  4}, /* UA2_TXD */
        {2,  5}, /* UA2_RTS_N */
        {2,  6}, /* UA2_CTS_N */
        {6, 16}, /* SPIM0_CS_N */
        {6, 17}, /* SPIM0_CLK */
        {6, 18}, /* SPIM0_MOSI */
        {6, 19}, /* SPIM0_MISO */
        {6, 20}, /* SPIM1_CS_N */
        {6, 21}, /* SPIM1_CLK */
        {6, 22}, /* SPIM1_MOSI */
        {6, 23}, /* SPIM1_MISO */
        {2,  1}, /* SPIM2_CS_N */
        {2,  1}, /* SPIM2_CLK */
        {2,  1}, /* SPIM2_MOSI */
        {2,  1}, /* SPIM2_MISO */
        {2,  2}, /* SPIM3_CS_N */
        {2,  2}, /* SPIM3_CLK */
        {2,  2}, /* SPIM3_MOSI */
        {2,  2}, /* SPIM3_MISO */
        {0, 99}, /* SPIC_CS_N */
        {0, 99}, /* SPIC_CLK */
        {0, 99}, /* SPIC_MOSI */
        {0, 99}, /* SPIC_MISO */
        {0, 99}, /* SPIC_IO2 */
        {0, 99}, /* SPIC_IO3 */
        {2,  8}, /* ADC24_SYNC */
        {0, 99}, /* DBG */
    };

    /* Get pin id*/
    PMU_IO_PIN pin_id = get_pin_id(pin);
    MBED_ASSERT(pin_id != PMU_IO_PIN_INVALID);
    MBED_ASSERT(tbl[pin_id].bit != 99 || tbl1[pin_id].bit == 99); /* Do not set mode for GPIO0 */


    /* Get curent pull, capa*/
    reg = &gconf->IO_CFG0 + tbl[pin_id].reg;
    t = *reg;
    cur_capa = (t >> (tbl[pin_id].bit + 2)) & 0x03;
    cur_pull = (t >> tbl[pin_id].bit) & 0x03;


    /* Convert mbed pinmode to gpio cmcis pin mode */
    switch (mode) {
        case PullDown:
            set_capa = cur_capa;
            set_pull = PMU_IO_RESISTOR_PULLDOWN;
            break;
        case PullNone:
            set_capa = cur_capa;
            set_pull = PMU_IO_RESISTOR_NONE;
            break;
        case PullUp:
            set_capa = cur_capa;
            set_pull = PMU_IO_RESISTOR_PULLUP;
            break;
        case Cap2mA:
            set_capa = PMU_DRIVE_CAPABILITY_2MA;
            set_pull = cur_pull;
            break;
        case Cap4mA:
            set_capa = PMU_DRIVE_CAPABILITY_4MA;
            set_pull = cur_pull;
            break;
        case Cap5mA:
            set_capa = PMU_DRIVE_CAPABILITY_5MA;
            set_pull = cur_pull;
            break;
        case Cap7mA:
            set_capa = PMU_DRIVE_CAPABILITY_7MA;
            set_pull = cur_pull;
            break;
        case InputStandby:
            reg1 = &pmulv->CTRL_IO_AON_0 + tbl1[pin_id].reg;
            *reg1 &= ~(1u << tbl1[pin_id].bit);
            return;
        case InputActive:
            reg1 = &pmulv->CTRL_IO_AON_0 + tbl1[pin_id].reg;
            *reg1 |= 1u << tbl1[pin_id].bit;
            return;
        default:
            error("Invalid mode\r\n");
            return;
    }

    /* Set pin mode*/
    t = *reg;
    t &= ~(15u << tbl[pin_id].bit);
    t |= (((uint32_t)set_capa << 2) + (uint32_t)set_pull) << tbl[pin_id].bit;
    *reg = t;
}
