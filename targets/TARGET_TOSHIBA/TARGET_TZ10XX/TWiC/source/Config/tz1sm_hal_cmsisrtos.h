/**
 * @file tz1sm_hal_cmsisrtos.h
 * @brief a header file for TZ10xx TWiC for Bluetooth 4.0/4.1 Smart
 * @version V1.2.0
 * @date $LastChangedDate$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */


#ifndef _TZ1SM_HAL_CMSISRTOS_H_
#define _TZ1SM_HAL_CMSISRTOS_H_

#include "cmsis_os.h"

#define tz1smHalTimer_t osTimerDef_t
#define TZ1SM_HAL_TIMER osTimer
#define TZ1SM_HAL_TIMER_INIT osTimerDef
#define tz1smHalTimerId osTimerId
#define tz1smHalTimerCreate osTimerCreate
#define tz1smHalTimerDelete osTimerDelete
#define tz1smHalTimerStart osTimerStart
#define tz1smHalTimerStop osTimerStop
#define tz1smHalMutexId_t osMutexId_t
#define tz1smHalMutexDef_t osMutexDef_t
#define TZ1SM_HAL_MUTEX_INIT osMutexDef
#define TZ1SM_HAL_MUTEX osMutex
#define tz1smHalMutexCreate osMutexCreate
#define tz1smHalMutexWait osMutexWait
#define tz1smHalMutexRelease osMutexRelease
#define tz1smHalMutexDelete osMutexDelete

typedef enum {
  TZ1SM_HAL_STATUS_OK = osOK,
  TZ1SM_HAL_STATUS_EVENT_SIGNAL = osEventSignal,
  TZ1SM_HAL_STATUS_EVENT_MESSAGE = osEventMessage,
  TZ1SM_HAL_STATUS_EVENT_MAIL = osEventMail,
  TZ1SM_HAL_STATUS_EVENT_TIMEOUT = osEventTimeout,
  TZ1SM_HAL_STATUS_ERROR_PARAMETER = osErrorParameter,
  TZ1SM_HAL_STATUS_ERROR_RESOURCE = osErrorResource,
  TZ1SM_HAL_STATUS_ERROR_TIMEOUT_RESOURCE = osErrorTimeoutResource,
  TZ1SM_HAL_STATUS_ERROR_ISR = osErrorISR,
  TZ1SM_HAL_STATUS_ERROR_ISR_RECURSIVE = osErrorISRRecursive,
  TZ1SM_HAL_STATUS_ERROR_PRIORITY = osErrorPriority,
  TZ1SM_HAL_STATUS_ERROR_NOMEMORY = osErrorNoMemory,
  TZ1SM_HAL_STATUS_ERROR_VALUE = osErrorValue,
  TZ1SM_HAL_STATUS_ERROR_OS = osErrorOS,
  TZ1SM_HAL_STATUS_ERROR_CTS,
  TZ1SM_HAL_STATUS_ERROR_DRIVER,
  TZ1SM_HAL_STATUS_SOURCE_STOPPED,
  TZ1SM_HAL_STATUS_SOURCE_PREPARING,
  TZ1SM_HAL_STATUS_SOURCE_RUNNING,
  TZ1SM_HAL_STATUS_IGNORE,
  TZ1SM_HAL_STATUS_RESERVED = os_status_reserved,
} tz1smHalStatus_t;

typedef enum {
  TZ1SM_HAL_TIMER_ONCE = osTimerOnce,
  TZ1SM_HAL_TIMER_PERIODIC = osTimerPeriodic,
} tz1smHalTimerBehavior_t;

void tz1smHalOsYeild(void);

/*
 * @brief
 * Enters at a critical session or comes out.
 */
#define TZ1SM_HAL_INTR_STATUS_DEF volatile uint32_t tz1sm_hal_intr_status = 0
#define TZ1SM_HAL_INTR_STATUS tz1sm_hal_intr_status
#define TZ1SM_HAL_IRQ_DISABLE_SAVE() {              \
    tz1sm_hal_intr_status = __get_PRIMASK();        \
    if (!tz1sm_hal_intr_status) __disable_irq(); }
#define TZ1SM_HAL_IRQ_ENABLE_RESTORE() {          \
    if (!tz1sm_hal_intr_status) __enable_irq(); }
#define TZ1SM_HAL_IRQ_GET_IPSR() __get_IPSR()

#endif /* _TZ1SM_HAL_CMSISRTOS_H_ */
