/**
 * @file usb_misc.h
 * @brief Common include files and macros
 * @version V0.0
 * @date $Date:: 2014-07-08 19:48:23 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef USB_MISC_H
#define USB_MISC_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#ifndef elemof
#define elemof(array)  (sizeof(array)/sizeof(array[0]))
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef bit
#define bit(n) ((unsigned int)1<<(n))
#endif

#ifndef bool_t
typedef bool	bool_t;
#endif

#endif /* USB_MISC_H */
