/**
 * @file TMR_TZ10xx.h
 * @brief a header file for TZ10xx TMR driver
 * @version V0.0
 * @date $Date:: 2014-07-17 14:53:00 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef TMR_TZ10XX_H
#define TMR_TZ10XX_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "Driver_Common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* API version */
#define TZ1000_TMR_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)

/**
 * @brief status of executed operation
 */
typedef enum _TMR_STATUS {
  TMR_OK,    /**< Operation succeeded */
  TMR_ERROR, /**< Unspecified error */
} TMR_STATUS;

typedef enum _TMR_COUNT_MODE {
  TMR_COUNT_MODE_ONE_SHOT, /**< one shot mode */
  TMR_COUNT_MODE_PERIODIC, /**< periodic mode */
  TMR_COUNT_MODE_FREE_RUN  /**< free run mode */
} TMR_COUNT_MODE;

typedef enum _TMR_EVENT {
  TMR_EVENT_BASE_COUNTER, /**< down counter reached zero */
  TMR_EVENT_CAPTURE,      /**< capture event */
  TMR_EVENT_COMPARE,      /**< compare event */
} TMR_EVENT;

typedef enum _TMR_CAPTURE_TRIGGER {
  TMR_CAPTURE_TRIGGER_PULSE,     /**< pulse */
  TMR_CAPTURE_TRIGGER_EDGE_POS,  /**< rising edge */
  TMR_CAPTURE_TRIGGER_EDGE_NEG,  /**< falling edge */
  TMR_CAPTURE_TRIGGER_EDGE_BOTH, /**< rising or falling edge */
} TMR_CAPTURE_TRIGGER;

typedef enum _TMR_TFF_MODE {
  TMR_TFF_MODE_CMP_ZERO,        /**< set TFF to 0 at compare event */
  TMR_TFF_MODE_CMP_ONE,         /**< set TFF to 1 at compare event  */
  TMR_TFF_MODE_CMP_TOGGLE,      /**< toggle TFF at compare event  */
  TMR_TFF_MODE_PULSE,           /**< output compare signal to TFF */
  TMR_TFF_MODE_CMP_TERM_TOGGLE, /**< toggle TFF at compare or terminal event */
} TMR_TFF_MODE;

typedef enum _TMR_SIGNAL {
  TMR_SIGNAL_CAPTURE0,         /**< CAPTURE0 */
  TMR_SIGNAL_CAPTURE1,         /**< CAPTURE1 */
  TMR_SIGNAL_CAPTURE2,         /**< CAPTURE2 */
  TMR_SIGNAL_CAPTURE3,         /**< CAPTURE3 */
  TMR_SIGNAL_TFF0,             /**< T0TFFOUT */
  TMR_SIGNAL_TFF1,             /**< T1TFFOUT */
  TMR_SIGNAL_TFF2,             /**< T2TFFOUT */
  TMR_SIGNAL_TFF3,             /**< T3TFFOUT */
  TMR_SIGNAL_CASCADE,          /**< TnTIMINT (cascade) */
  TMR_SIGNAL_INT_GPIO_24,      /**< GPIO_24 interrupt */
  TMR_SIGNAL_INT_GPIO_25,      /**< GPIO_25 interrupt */
  TMR_SIGNAL_INT_GPIO_26,      /**< GPIO_26 interrupt */
  TMR_SIGNAL_INT_GPIO_27,      /**< GPIO_27 interrupt */
  TMR_SIGNAL_INT_GPIO_29,      /**< GPIO_29 interrupt */
  TMR_SIGNAL_INT_GPIO_30,      /**< GPIO_30 interrupt */
  TMR_SIGNAL_INT_GPIO_31,      /**< GPIO_31 interrupt */
  TMR_SIGNAL_INT_RTC_ALARM,    /**< RTC alarm interrupt */
  TMR_SIGNAL_INT_RTC_PERIODIC, /**< RTC periodic interrupt */
  TMR_SIGNAL_INT_RTC_INTERVAL, /**< RTC interval interrupt */
  TMR_SIGNAL_PRESCALER,        /**< prescaler */
} TMR_SIGNAL;

typedef enum _TMR_EVENT_EN_MODE {
  TMR_EVENT_EN_MODE_EXT_DISABLE, /**< EnableEvent() */
  TMR_EVENT_EN_MODE_EXT_LEVEL,   /**< EnableEvent() && external signal (level) */
  TMR_EVENT_EN_MODE_EXT_EDGE,    /**< EnableEvent() && external signal (pos-edge) */
} TMR_EVENT_EN_MODE;

typedef struct _TMR_CAPABILITIES {
  uint32_t capture       : 1;
  uint32_t compare       : 1;
  uint32_t tff           : 1;
  uint32_t event_counter : 1;
} TMR_CAPABILITIES;


typedef void (*TMR_SignalEvent_t)(TMR_EVENT);

typedef struct _TZ10XX_DRIVER_TMR {
  ARM_DRIVER_VERSION (*GetVersion)(void);
  TMR_CAPABILITIES (*GetCapabilities)(void);
  TMR_STATUS (*Initialize)(TMR_SignalEvent_t cb_event, uint32_t event_mask);
  TMR_STATUS (*Uninitialize)(void);
  TMR_STATUS (*PowerControl)(ARM_POWER_STATE state);
  TMR_STATUS (*Configure)
      (uint32_t counter_bits, TMR_COUNT_MODE count_mode, uint32_t divisor);
  TMR_STATUS (*Start)(uint32_t initial_value);
  TMR_STATUS (*Stop)(void);
  bool (*IsRunning)(void);
  TMR_STATUS (*SetValue)(uint32_t value);
  TMR_STATUS (*SetReloadValue)(uint32_t reload_value);
  uint32_t (*GetValue)(void);
  /* Functions below are supported by ADVTMR only */
  TMR_STATUS (*ConfigureCapture)
      (TMR_SIGNAL signal, TMR_CAPTURE_TRIGGER trigger, bool reload, bool sync);
  TMR_STATUS (*EnableCapture)(bool enable);
  uint32_t (*GetCapturedValue)(void);
  TMR_STATUS (*EnableCompare)(bool enable);
  TMR_STATUS (*SetCompareValue)(uint32_t value, bool load_at_terminal);
  TMR_STATUS (*ConfigureTFF)
      (TMR_TFF_MODE mode, bool init_high, bool invert);
  TMR_STATUS (*EnableTFF)(bool enable);
  TMR_STATUS (*ConfigureEventCounter)
      (TMR_EVENT_EN_MODE event_en_mode, bool ev_sync, bool ev_delay,
       TMR_SIGNAL evcntsrc, TMR_SIGNAL evcnten);
  TMR_STATUS (*UnconfigureEventCounter)(void);
  TMR_STATUS (*EnableEventCounter)(bool enable);
} const TZ10XX_DRIVER_TMR;
  
#ifdef __cplusplus
}
#endif

#endif /* TMR_TZ10XX_H */
