/**
 * @file twic_led.c
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

#include "twic_interface.h"
#include "twic_led.h"

#if defined(TWIC_GPIO__CMSIS_GPIO)

#include "TZ10xx.h"
#include "GPIO_TZ10xx.h"


/* debug */
#define TWIC_LED_INFO(...)
#define TWIC_LED_ERROR(...)

/* GPIO Driver Control Block */
extern TZ10XX_DRIVER_GPIO Driver_GPIO;

/*
 * @brief
 * Configuration LED I/O
 *
 * If the "TWIC_LED_GPIO_LED[1-5]" is defined in the tz1sm_config.h, this
 * API can be used for the operation of LED. */
uint8_t twicLedInit(void)
{
  uint8_t ret;
  twicStatus_t status = twicIfleIoStatus();

  if (status != TWIC_STATUS_OK)
    return 1;
  
  ret = tz1smHalPmuLedInit();
  if ((ret))
    return ret;

  return tz1smHalGpioLedInit();
}

/*
 * @brief
 * Finalize LED
 *
 */
uint8_t twicLedFinalize(void)
{
  uint8_t ret;

  ret = tz1smHalPmuLedFinalize();
  if ((ret))
    return ret;

  return tz1smHalGpioLedFinalize();
}

/*
 * @brief
 * Read LED status
 * @return status (true:The level of the GPIO teminal is Hi,
 *                 false:The level of the GPIO teminal is Lo)
 *
 * Read the GPIO terminal level of a LED.
 * This API reads the level of the designated GPIO terminal.
 */
bool twicReadLedStatus(const uint8_t num)
{
  bool val;
  
  tz1smHalGpioRead(num, &val);

  return val;
}

/*
 * @brief
 * Set LED
 *
 * Turn on or off a LED with the GPIO number.
 * This API sets the parameter "out" to the designated GPIO terminal.
 */
void twicSetLed(const uint8_t num, const bool out)
{
  tz1smHalGpioWrite(num, out);

  return;
}

#else

/*
 * @brief
 * Configuration LED I/O
 *
 */
uint8_t twicLedInit(void) { return 0; }

/*
 * @brief
 * Set LED status
 * @return status (true:LED ON, false:LED OFF)
 *
 */
bool twicReadLedStatus(const uint8_t num) { return false; }

/*
 * @brief
 * Set LED
 *
 */
void twicSetLed(const uint8_t num, const bool out) { return; };

#endif
