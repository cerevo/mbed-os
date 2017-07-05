/**
 * @file system_TZ10xx.c
 * @brief CMSIS Device System Source File for TZ10xx Device Series
 * @date $Date:: 2014-02-17 17:44:42 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#include "TZ10xx.h"

#define __SYSTEM_CLOCK 12000000

#if defined( USE_TZ10XX_BOOT_FLASH ) && defined( __ICCARM__ )
#define TZ10xx_STARTUP  @ ".startup"
#else
#define TZ10xx_STARTUP
#endif

uint32_t SystemCoreClock = __SYSTEM_CLOCK;/*!< System Clock Frequency (Core Clock)*/


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)   TZ10xx_STARTUP         /* Get Core Clock Frequency      */
{
  /* TODO: obtain frequency from PMU */
  SystemCoreClock = __SYSTEM_CLOCK;
}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 * @note   This function is called before startup routine.
 *         Do not call functions or read/write SRAM in this function
           because SRAM is not initialized yet.
 */
void SystemInit (void)  TZ10xx_STARTUP
{
  #if (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                   (3UL << 11*2)  );               /* set CP11 Full Access */
  #endif

#ifdef UNALIGNED_SUPPORT_DISABLE
  SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif
}
