/**
 * @file RNG_TZ10xx.h
 * @brief a header file for TZ10xx RNG driver
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

#ifndef RNG_TZ10XX_H
#define RNG_TZ10XX_H

#include "Driver_Common.h"

#define ARM_DRV_VERSION_MAJOR_MINOR(major,minor) (((major) << 8) | (minor))

#ifdef __cplusplus
extern "C" {
#endif

/* API version */
#define RNG_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)

/**
 * @brief status of executed operation
 */
typedef enum _RNG_STATUS {
  RNG_OK    = 0,     /**< Operation succeeded */
  RNG_ERROR = 1,  /**< Unspecified error */
} RNG_STATUS;

/**
 * @brief capabilities
 */
typedef struct _RNG_CAPABILITIES {
	uint32_t reserved : 32;
} RNG_CAPABILITIES;

/**
 * @brief driver function pointers
 */
typedef struct __TZ10XX_DRIVER_RNG {
  ARM_DRIVER_VERSION    (*GetVersion)      (void);
  RNG_CAPABILITIES (*GetCapabilities)(void);
  RNG_STATUS        (*Initialize)      (void);
  RNG_STATUS        (*Uninitialize)    (void);
  RNG_STATUS        (*PowerControl)    (ARM_POWER_STATE state);
  RNG_STATUS        (*Read)   (uint32_t *rn);
   } const TZ10XX_DRIVER_RNG;

#ifdef __cplusplus
}
#endif

#endif /* RNG_TZ10XX_H */
