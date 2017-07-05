/**
 * @file twic_led.h
 * @brief a source file for TZ10xx TWiC for Bluetooth 4.0/4.1 Smart
 * @version V1.2.0
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef _TWIC_LED_H_
#define _TWIC_LED_H_

#include "tz1sm_config.h"


/*
 * @brief
 * Configuration LED I/O
 *
 */
uint8_t twicLedInit(void);

/*
 * @brief
 * Finalize LED
 *
 */
uint8_t twicLedFinalize(void);

/*
 * @brief
 * Set LED status
 * @return status (true:LED ON, false:LED OFF)
 *
 */
bool twicReadLedStatus(const uint8_t num);

/*
 * @brief
 * Set led
 *
 */
void twicSetLed(const uint8_t num, const bool out);

#endif /* _TWIC_LED_H_ */
