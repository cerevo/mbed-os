/**
 * @file RTC_TZ10xx.h
 * @brief a header file for TZ10xx RTC driver
 * @version V0.0
 * @date $Date:: 2014-07-17 13:38:01 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef RTC_TZ10XX_H
#define RTC_TZ10XX_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "Driver_Common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* API version */
#define RTC_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)

/**
 * @brief status of executed operation
 */
typedef enum _RTC_STATUS {
  RTC_OK    = 0, /**< Operation succeeded */
  RTC_ERROR = 1, /**< Unspecified error */
} RTC_STATUS;

/**
 * @brief RTC driver event
 */
typedef enum _RTC_EVENT {
  RTC_EVENT_ALARM,
  RTC_EVENT_PERIODIC,
  RTC_EVENT_INTERVAL,
} RTC_EVENT;


/**
 * @brief type of periodic event
 */
typedef enum _RTC_PERIOD {
  RTC_PERIOD_DISABLE            = 0,  /**< disabled */
  RTC_PERIOD_EVERY_SECOND       = 1,  /**< **-**-** **:**:** */
  RTC_PERIOD_EVERY_MINUTE       = 2,  /**< **-**-** **:**:00 */
  RTC_PERIOD_EVERY_HOUR         = 3,  /**< **-**-** **:00:00 */
  RTC_PERIOD_EVERY_DAY          = 4,  /**< **-**-** 00:00:00 */
  RTC_PERIOD_EVERY_MONTH        = 5,  /**< **-**-01 00:00:00 */
  RTC_PERIOD_EVERY_1_2_SECOND   = 8,  /**< every 1/2 second */
  RTC_PERIOD_EVERY_1_4_SECOND   = 9,  /**< every 1/4 second */
  RTC_PERIOD_EVERY_1_8_SECOND   = 10, /**< every 1/8 second */
  RTC_PERIOD_EVERY_1_16_SECOND  = 11, /**< every 1/16 second */
  RTC_PERIOD_EVERY_1_32_SECOND  = 12, /**< every 1/32 second */
  RTC_PERIOD_EVERY_1_64_SECOND  = 13, /**< every 1/64 second */
  RTC_PERIOD_EVERY_1_128_SECOND = 14, /**< every 1/128 second */
  RTC_PERIOD_EVERY_1_256_SECOND = 15  /**< every 1/256 second */
} RTC_PERIOD;

/**
 * @brief date and time
 */
typedef struct _RTC_TIME {
  uint8_t sec;  /**< second (0..59) */
  uint8_t min;  /**< minute (0..59) */
  uint8_t hour; /**< hour (0..23) */
  uint8_t mday; /**< day of month (1..31) */
  uint8_t mon;  /**< month (1..12) */
  uint8_t year; /**< year (0..99) */
  uint8_t wday; /**< day of week (0..6); 0 is Sunday */
} RTC_TIME;

typedef void (*RTC_SignalEvent_t)(RTC_EVENT);

/**
 * @brief  Access structure of the RTC Driver.
 */
typedef struct _TZ10XX_DRIVER_RTC {
  ARM_DRIVER_VERSION (*GetVersion)(void);
  RTC_STATUS (*Initialize)(void);
  RTC_STATUS (*Uninitialize)(void);
  RTC_STATUS (*PowerControl)(ARM_POWER_STATE state);
  RTC_STATUS (*SetTime)(const RTC_TIME *time);
  RTC_STATUS (*GetTime)(RTC_TIME *time);
  RTC_STATUS (*SetAlarm)(
    const RTC_TIME *time,
    RTC_SignalEvent_t cb_event);
  RTC_STATUS (*ClearAlarm)(void);
  RTC_STATUS (*SetPeriodicInterrupt)(
    RTC_PERIOD mode,
    RTC_SignalEvent_t cb_event);
  RTC_STATUS (*ClearPeriodicInterrupt)(void);
  RTC_STATUS (*SetIntervalInterrupt)(
    uint32_t ticks_at_2048hz,
    RTC_SignalEvent_t cb_event);
  RTC_STATUS (*ClearIntervalInterrupt)(void);
  uint32_t (*GetIntervalCounter)(void);
} const TZ10XX_DRIVER_RTC;

#ifdef __cplusplus
}
#endif

#endif /* RTC_TZ10XX_H */
