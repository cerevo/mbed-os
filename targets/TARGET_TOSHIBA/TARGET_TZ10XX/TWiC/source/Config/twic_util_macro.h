/**
 * @file twic_util_macro.h
 * @brief a source file for TZ10xx TWiC for Bluetooth 4.0/4.1 Smart
 * @version V1.0.1.FS (Free Sample - The information in this code is
 * subject to change without notice and should not be construed as a
 * commitment by TOSHIBA CORPORATION SEMICONDUCTOR & STORAGE PRODUCTS
 * COMPANY.
 * @note TZ1EM provides the automatic low energy consumption. The low
 * energy consumption of interal BLE Processor and UART2 is managed by
 * the part of TWIC BLE CE (BLE Controller Extension). Please refer to
 * the twicIfLeCe API group. TZ1EM combines the HOST Core low energy
 * consumption with the BLE CE and the other peripheral modules.
 */

/*
 * COPYRIGHT (C) 2015-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef _TWIC_UTIL_MACRO_H_
#define _TWIC_UTIL_MACRO_H_

#define TWIC_UTIL_UUID16(a,b) 0x ## b, a
#define TWIC_UTIL_UUID16_U64SET(a,b) a ## b, 0

#define TWIC_UTIL_UUID128(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p) 0x ## p, 0x ## o, 0x ## n, 0x ## m, 0x ## l, 0x ## k, 0x ## j, 0x ## i, 0x ## h, 0x ## g, 0x ## f, 0x ## e, 0x ## d, 0x ## c, 0x ## b
#define TWIC_UTIL_UUID128_U64SET(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p) a ## b ## c ## d ## e ## f ## g ## h, 0x ## i ## j ## k ## l ## m ## n ## o ## p

#endif /* _TWIC_UTIL_MACRO_H_ */
