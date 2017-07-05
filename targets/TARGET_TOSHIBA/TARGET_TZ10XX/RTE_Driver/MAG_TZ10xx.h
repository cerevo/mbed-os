/**
 * @file MAG_TZ10xx.h
 * @brief a header file for TZ10xx Magnetometer driver
 * @date $Date:: 2014-12-19 11:39:40 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */
#ifndef MAG_TZ10XX_H
#define MAG_TZ10XX_H

#include "RTE_Components.h"
#include "RTE_Device.h"
#include "Driver_I2C.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef RTE_MAGNETOMETER_TZ1011
#define MAG_ENABLE_INTERRUPT
#define MAG_ENABLE_NSF
#define MAG_ENABLE_TEMPERATURE
#define MAG_ENABLE_16BIT
#endif
  
typedef enum _MAG_STATUS {
  MAG_OK,
  MAG_ERROR,
  MAG_ERROR_I2C_BUS,
  MAG_ERROR_MODE,
  MAG_ERROR_NO_DATA,
  MAG_ERROR_TEST,
  MAG_ERROR_GPIO,
} MAG_STATUS;

typedef enum _MAG_EVENT {
  MAG_EVENT_DATA_READY      = 1 << 0,
} MAG_EVENT;

typedef enum _MAG_MODE {
  MAG_MODE_STANDBY          = 1,
  MAG_MODE_SINGLE           = 2,
  MAG_MODE_CONTINUOUS_10HZ  = 4,
  MAG_MODE_CONTINUOUS_20HZ  = 8,
  MAG_MODE_CONTINUOUS_50HZ  = 12,
  MAG_MODE_CONTINUOUS_100HZ = 16,
} MAG_MODE;

typedef enum _MAG_ATTR {
  MAG_ATTR_NORMAL   = 0,
  MAG_ATTR_OVERRUN  = 1,
  MAG_ATTR_OVERFLOW = 2,
} MAG_ATTR;

typedef enum _MAG_NSF {  /* Enum for NSF(Noise Suppression Filter) */
  MAG_NSF_DISABLE   = 0, /* Output magnetic data is not filtered. (Default)*/
  MAG_NSF_LOW       = 1, /* Output magnetic data is filtered mildly. */
  MAG_NSF_MIDDLE    = 2, /* Output magnetic data is filtered moderately. */
  MAG_NSF_HIGH      = 3, /* Output magnetic data is filtered strongly. */
} MAG_NSF;

typedef void (*MAG_SignalEvent_t)(MAG_EVENT event);

typedef struct _MAG_CAPABILITIES {
  uint32_t reserved : 32;
} MAG_CAPABILITIES;

typedef struct _TZ10XX_DRIVER_MAG {
  ARM_DRIVER_VERSION  (*GetVersion)          (void);
  MAG_CAPABILITIES (*GetCapabilities)     (void);
  MAG_STATUS       (*Initialize)          (MAG_SignalEvent_t cb_event);
  MAG_STATUS       (*Uninitialize)        (void);
  MAG_STATUS       (*PowerControl)        (ARM_POWER_STATE state);
  MAG_STATUS       (*BusSpeed)            (ARM_I2C_BUS_SPEED speed);
  MAG_STATUS       (*SensorModeControl)   (MAG_MODE mode);
  MAG_STATUS       (*LoadAdjustmentValues)(void);
  MAG_STATUS       (*DataAvailable)       (void);
  MAG_STATUS       (*ReadData)            (int16_t *density, uint32_t *attr);
  MAG_STATUS       (*RunSelfTest)         (void);
#ifdef MAG_ENABLE_NSF
  MAG_STATUS       (*SetNoiseSuppressionFilter) (MAG_NSF nsf);
#endif
#ifdef MAG_ENABLE_TEMPERATURE
  MAG_STATUS       (*SetTempMeasurement)     (bool enable);
#endif
#ifdef MAG_ENABLE_INTERRUPT
  MAG_STATUS       (*SetDataReadyInterrupt)  (bool enable);
#endif
} const TZ10XX_DRIVER_MAG;

#ifdef __cplusplus
}
#endif

#endif /* MAG_TZ10XX_H */
