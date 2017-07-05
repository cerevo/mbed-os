/**
 * @file GPIO_TZ10xx.h
 * @brief a header file for TZ10xx GPIO driver
 * @date $Date:: 2014-07-11 09:54:51 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef GPIO_TZ10XX_H
#define GPIO_TZ10XX_H

#include "Driver_Common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* API version */
#define GPIO_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 00)

/**
 * @brief status of executed operation
 */
typedef enum _GPIO_STATUS {
  GPIO_OK    = 0,     /**< Operation succeeded */
  GPIO_ERROR = 1,  /**< Unspecified error */
} GPIO_STATUS;

/**
 * @brief pin direction
 */
typedef enum _GPIO_DIRECTION {
  GPIO_DIRECTION_INPUT_HI_Z,             /**< without resistor */
  GPIO_DIRECTION_INPUT_PULL_UP,      /**< with pull up resistor */
  GPIO_DIRECTION_INPUT_PULL_DOWN, /**< with pull down resistor */
  GPIO_DIRECTION_OUTPUT_2MA,         /**< 2mA buffer drivability */
  GPIO_DIRECTION_OUTPUT_4MA,         /**< 4mA buffer drivability */
  GPIO_DIRECTION_OUTPUT_5MA,         /**< 5mA buffer drivability */
  GPIO_DIRECTION_OUTPUT_7MA,         /**< 7mA buffer drivability */
} GPIO_DIRECTION;

/**
 * @brief event detection
 */
typedef enum _GPIO_EVENT {
  GPIO_EVENT_DISABLE,                      /**< event disable */
  GPIO_EVENT_LEVEL_HIGH,                 /**< event level high */
  GPIO_EVENT_LEVEL_LOW,                  /**< event level low */    
  GPIO_EVENT_EDGE_POS,                   /**< event edge positive */
  GPIO_EVENT_EDGE_NEG,                   /**< event edge negative */
  GPIO_EVENT_EDGE_BOTH,                 /**< event edge both */
} GPIO_EVENT;

/**
 * @brief pin capabilities
 */
typedef struct _GPIO_CAPABILITIES {
  uint32_t connected : 1;      /**< GPIO connected to pad */
  uint32_t resistor     : 1;      /**< has pull up/down resistor */
  uint32_t output       : 1;      /**< output drivability */
  uint32_t wake_up     : 1;      /**< wake up from sleep2/wait/rtc/stop */
  uint32_t event        : 1;      /**< event handling */
  uint32_t reserved    :27;     /**< reserved */
} GPIO_CAPABILITIES;

/**
 * @brief signal event
 */
typedef void (*GPIO_SignalEvent_t)(uint32_t pin);

/**
 * @brief driver function pointers
 */
typedef struct _TZ10XX_DRIVER_GPIO {
  ARM_DRIVER_VERSION    (*GetVersion)   (void);
  GPIO_CAPABILITIES (*GetCapabilities)(uint32_t pin);
  GPIO_STATUS        (*Initialize)          (void);
  GPIO_STATUS        (*Uninitialize)       (void);
  GPIO_STATUS        (*PowerControl)  (ARM_POWER_STATE state);
  GPIO_STATUS        (*Configure)        (uint32_t               pin,
																												GPIO_DIRECTION    dir,
                                                        GPIO_EVENT          event,
                                                        GPIO_SignalEvent_t cb_event);
  GPIO_STATUS        (*ReadPin)          (uint32_t pin, 
																												uint32_t *val);
  GPIO_STATUS        (*Read)               (uint32_t *val);
  GPIO_STATUS        (*WritePin)          (uint32_t pin, 
	                                                       uint32_t val);
  GPIO_STATUS        (*Write)              (uint32_t mask,
                                                        uint32_t val);
} const TZ10XX_DRIVER_GPIO;


	#ifdef __cplusplus
}
#endif

#endif /* GPIO_TZ10XX_H */
