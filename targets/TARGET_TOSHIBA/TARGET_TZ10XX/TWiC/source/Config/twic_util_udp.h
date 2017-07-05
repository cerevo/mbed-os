/**
 * @file twic_util_udp.h
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

#ifndef __TWIC_UTIL_UDP_H__
#define __TWIC_UTIL_UDP_H__

extern void twicUtUdpSetupService(void);
extern void twicUtUdpStopNotification(void);
extern void twicUtUdpStopIndication(void);
extern void twicUtUdpDescriptorChange(uint8_t eidx,
                                      const twicAttValue_t *const arg);
extern uint8_t twicUtUdpReadOut(twicConnIface_t * const iface, uint8_t eidx,
                                uint16_t num);
extern uint8_t twicUtUdpWrittenIn(twicConnIface_t * const iface, uint8_t eidx,
                                  const twicAttValue_t *const arg);
extern void twicUtUdpTimerSetup(void);
extern void twicUtUdpTimerCleanup(void);
extern void twicUtUdpCongestionCheck(void);
extern void twicUtUdpRun(twicConnIface_t * const cif);
extern void twicUtUdpIndicationConfirmation(const uint8_t status);

#endif /* __TWIC_UTIL_UDP_H__ */
