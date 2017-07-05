/**
 * @file WDT_TZ10xx.h
 * @brief a header file for TZ10xx WDT driver
 * @date $Date:: 2014-07-22 15:32:30 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef WDT_TZ10XX_H
#define WDT_TZ10XX_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "RTE_Device.h"
#include "Driver_Common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Enumerations */

/* WDT Event */
typedef enum _WDT_EVENT {
  WDT_EVENT_TIMEOUT,             /* WDT Timeout occured */
} WDT_EVENT;

/* Return Status */
typedef enum _WDT_STATUS {
    WDT_OK,
    WDT_ERROR,
    WDT_CALLBACK_ERROR,
    WDT_ARGUMENT_ERROR,
} WDT_STATUS;

/* WDT Capabilities */
typedef struct _WDT_CAPABILITIES {
  uint32_t settimeout     : 1;      /* Set timeout support  */
  uint32_t resetenable    : 1;      /* System Reset signal support */
  uint32_t keepaliveping  : 1;      /* Keep Alive support  */
  uint32_t reserved       :29;      /* reserved */
} WDT_CAPABILITIES;

typedef void (*WDT_SignalEvent_t)(WDT_EVENT event);

typedef struct _TZ10XX_DRIVER_WDT{
    ARM_DRIVER_VERSION      (*GetVersion)(void);
    WDT_CAPABILITIES (*GetCapabilities)(void);    
    WDT_STATUS      (*Initialize)(WDT_SignalEvent_t cb_event);
    WDT_STATUS      (*Uninitialize)(void);
    WDT_STATUS      (*PowerControl)(ARM_POWER_STATE state);
    WDT_STATUS      (*SetTimeoutInterval)(uint32_t timeout);
    WDT_STATUS      (*GetTimeoutInterval)(uint32_t *timeout);
    WDT_STATUS      (*GetTimeLeft)(uint32_t *timeleft);
    WDT_STATUS      (*StartWDT)(void);
    WDT_STATUS      (*StopWDT)(void);
    WDT_STATUS      (*KeepAlive)(void);
} const TZ10XX_DRIVER_WDT;

extern TZ10XX_DRIVER_WDT Driver_WDT;


#ifdef __cplusplus
}
#endif

#endif /* WDT_TZ10XX_H */
