/**
 * @file ADCC_TZ10xx_common.h
 * @brief a common header file for TZ10xx ADCC driver
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef ADCC12_TZ10XX_COMMON_H
#define ADCC12_TZ10XX_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _ADCC_VALIDATION {
    ADCC_DISABLE = 0,
    ADCC_ENABLE = 1
} ADCC_VALIDATION;

typedef enum __ADCC_WORKSTATE {
    ADCC_BUSY = 0,
    ADCC_DONE = 1
} ADCC_WORKSTATE;

#ifdef __cplusplus
}
#endif

#endif /* ADCC12_TZ10XX_COMMON_H */

