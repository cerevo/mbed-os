/**************************************************************************//**
 * @file     system_TZ10xx.h
 * @brief    CMSIS Device System Header File for
 *           TZ1000 Series
 * @date     $Date:: 2014-07-11 09:54:51 +0900 #$
 *
 * @note
 *
 ******************************************************************************/
/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */


#ifndef SYSTEM_TZ10XX_H
#define SYSTEM_TZ10XX_H

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit (void);

/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_TZ10XX_H */
