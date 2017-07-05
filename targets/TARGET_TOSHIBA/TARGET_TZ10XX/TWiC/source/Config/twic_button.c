/**
 * @file twic_button.c
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
#include "twic_button.h"

#if defined(TWIC_GPIO__CMSIS_GPIO) && defined(TWIC_BUTTON_GPIO_NO)

#include "TZ10xx.h"
#include "GPIO_TZ10xx.h"


#if (TWIC_BUTTON_GPIO_NO > 31)
#error up to 31
#endif

/* debug */
#define TWIC_BUTTON_INFO(...)
#define TWIC_BUTTON_ERROR(...)

/* GPIO Driver Control Block */
extern TZ10XX_DRIVER_GPIO Driver_GPIO;

/* */
static bool gpio_button_enable = false;
static bool gpio_button_event = false;

/* callback handler */
static void gpio_button_handler(uint32_t pin);


/*
 * @brief
 * Configuration Button I/O
 *
 * If the "TWIC_BUTTON_GPIO_NO" is defined in the tz1sm_config.h, this
 * API can be used for handling the event of a button press. */
uint8_t twicButtonInit(void)
{
  uint8_t ret;
  twicStatus_t status = twicIfleIoStatus();

  if (status != TWIC_STATUS_OK)
    return 1;

  ret = tz1smHalPmuButtonInit();
  if ((ret))
    return ret;

  return tz1smHalGpioButtonInit(gpio_button_handler);
}


/*
 * @brief
 * Get button status
 * @return    status (TWIC_BUTTON_NOP:no pushed,
 *                    TWIC_BUTTON_PUSH:pushed)
 *
 * This API gets the past record of the button press. */
uint16_t twicButton(void)
{
  uint16_t sts = TWIC_BUTTON_NOP;
  
  if (gpio_button_event == true)
    sts = TWIC_BUTTON_PUSH;
  gpio_button_event = false;
    
  return sts;
}

/* This API cleans up the past record of the button press and disables
 * the system to hold the past record of the button. */
void twicButtonVoid(void)
{
  gpio_button_enable = false;
  gpio_button_event = false;

  return;
}

/* This API enable the system to hold the past record of the button. */
void twicButtonEnable(void)
{
  gpio_button_enable = true;
  gpio_button_event = false;

  return;
}

/* @return    bool (true: The button is being pressed.
                    false: The button is not being pressed.
 * This API gets the realtime level of the GPIO which is assigned to
 * the button. */
bool twicButtonLevel(void)
{
  bool val;
  
  tz1smHalGpioRead(TWIC_BUTTON_GPIO_NO, &val);
  
  return val;
}

static void gpio_button_handler(uint32_t pin)
{
  if (true == gpio_button_enable)
    gpio_button_event = true;

  return;
}


/*
 * @brief
 * Finalizes the Button.
 *
 * This API disables and frees the resource for the button.
 */
uint8_t twicButtonFinalize(void)
{
  uint8_t ret;

  ret = tz1smHalPmuButtonFinalize();
  if ((ret))
    return ret;

  return tz1smHalGpioButtonFinalize();
}

#else

uint8_t twicButtonInit(void) {};

uint16_t twicButton(void) { return TWIC_BUTTON_NOP; }

void twicButtonVoid(void) {};

void twicButtonEnable(void) { return; }

bool twicButtonLevel(void) {};

#endif
