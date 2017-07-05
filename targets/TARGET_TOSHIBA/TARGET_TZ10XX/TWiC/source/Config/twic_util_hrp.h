/**
 * @file twic_util_hrp.h
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

#ifndef __TWIC_UTIL_HRP_H__
#define __TWIC_UTIL_HRP_H__

extern void twicUtHrpSetupService(void);
extern void twicUtHrpStopNotification(void);
extern void twicUtHrpDescriptorChange(uint8_t, const twicAttValue_t *const);
extern uint8_t twicUtHrpReadOut(twicConnIface_t * const cif, uint8_t eidx,
                                uint16_t num);
extern uint8_t twicUtHrpWrittenIn(uint8_t, const twicAttValue_t *const);
extern void twicUtHrpTimerSetup(void);
extern void twicUtHrpTimerCleanup(void);
extern void twicUtHrpCongestionCheck(void);
extern void twicUtHrpRun(twicConnIface_t * const cif);

#endif /* __TWIC_UTIL_HRP_H__ */
