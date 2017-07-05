/**
 * @file twic_button.h
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

#ifndef _TWIC_BUTTON_H_
#define _TWIC_BUTTON_H_

#include "tz1sm_config.h"


#define TWIC_BUTTON_PUSH  (0x1000)
#define TWIC_BUTTON_NOP   (0x2000)


/*
 * @brief
 * Configuration Button I/O
 *
 */
uint8_t twicButtonInit(void);

/*
 * @brief
 * Get button status
 * @return    status (TWIC_BUTTON_NOP:no pushed,
 *                    TWIC_BUTTON_PUSH:pushed)
 *
 */
uint16_t twicButton(void);

/*
 * @brief
 * Void button
 *
 */
void twicButtonVoid(void);

/*
 * @brief
 * Enable button
 *
 */
void twicButtonEnable(void);

/*
 * @brief
 * Get pin level
 *
 */
bool twicButtonLevel(void);

#endif /* _TWIC_BUTTON_H_ */
