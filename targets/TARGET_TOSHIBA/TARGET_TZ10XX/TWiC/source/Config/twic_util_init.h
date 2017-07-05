/**
 * @file twic_util_init.h
 * @brief a source file for TZ10xx TWiC for Bluetooth 4.0 Smart
 * @version V1.0.0.FS (Free Sample - The information in this code is
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

#ifndef _TWIC_UTIL_INIT_H_
#define _TWIC_UTIL_INIT_H_

extern twicStatus_t twicUtLeCeInit1(twicConnIface_t *cif, uint8_t *aret,
                                    uint8_t *wait_ms, twicTzbtBr_t br,
                                    uint16_t pu);
extern twicStatus_t twicUtLeCeInit2(twicConnIface_t *cif, uint8_t *aret,
                                    uint8_t *wait_ms, uint64_t *bd_addr,
                                    bool low_power);
extern twicStatus_t twicUtLeCeInit3(twicConnIface_t *cif, uint8_t *aret,
                                    bool client);
extern twicStatus_t twicUtLeCeInit0(twicConnIface_t *cif, uint8_t *aret,
                                    uint16_t ext);
#endif /* _TWIC_UTIL_INIT_H_ */
