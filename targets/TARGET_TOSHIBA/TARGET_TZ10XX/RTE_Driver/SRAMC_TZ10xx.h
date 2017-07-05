/**
 * @file SRAMC_TZ10xx.h
 * @brief a header file for TZ10xx SRAMC driver
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

#ifndef SRAMC_TZ10XX_H
#define SRAMC_TZ10XX_H

//#include <stddef.h>
//#include <stdint.h>
//#include <stdbool.h>

#include "Driver_Common.h"

#define ARM_DRIVER_VERSION_MAJOR_MINOR(major,minor) (((major) << 8) | (minor))


#ifdef __cplusplus
extern "C" {
#endif

/* API version */
#define SRAMC_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 00)

/**
 * @brief status of executed operation
 */
typedef enum _SRAMC_STATUS {
  SRAMC_OK    = 0,     /**< Operation succeeded */
  SRAMC_ERROR = 1,  /**< Unspecified error */
} SRAMC_STATUS;

/**
 * @brief  status of transfer
 */
typedef enum _SRAMC_EVENT {
	SRAMC_EVENT_TRANSFER     = 1, /* done transfer data. */
	SRAMC_EVENT_ERROR        = 16, /* recept error response. */
} SRAMC_EVENT;

/**
 * @brief capabilities
 */
typedef struct _SRAMC_CAPABILITIES {
	uint32_t reserved : 32;
} SRAMC_CAPABILITIES;

/**
 * @brief signal event
 */
typedef void (*SRAMC_SignalEvent_t)(SRAMC_EVENT event);

/**
 * @brief driver function pointers
 */
typedef struct __TZ10XX_DRIVER_SRAMC {
  ARM_DRIVER_VERSION    (*GetVersion)      (void);
  SRAMC_CAPABILITIES (*GetCapabilities)(void);
  SRAMC_STATUS        (*Initialize)      (SRAMC_SignalEvent_t cb_event);
  SRAMC_STATUS        (*Uninitialize)    (void);
  SRAMC_STATUS        (*PowerControl)    (ARM_POWER_STATE state);
  SRAMC_STATUS        (*StartTransfer)   (uint8_t *dst, const uint8_t *src,
																														uint32_t size);
  SRAMC_STATUS        (*AbortTransfer)    (void);
   } const TZ10XX_DRIVER_SRAMC;

	 #ifdef __cplusplus
}
#endif

#endif /* SRAMC_TZ10XX_H */
