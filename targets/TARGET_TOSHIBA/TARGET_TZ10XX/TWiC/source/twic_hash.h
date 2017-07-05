/**
 * @file twic_hash.h
 * @brief a header file for TZ10xx TWiC for Bluetooth 4.0/4.1 Smart
 * @version V1.2.0
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */


void twicHashCreate(twicEntry_t *(* const entry_hash)[15],
                    const uint16_t entry_hash_size);

twicEntry_t *twicHashFind1stHandle(const uint16_t handle);
twicEntry_t *twicHashFind2ndHandle(const uint16_t handle);
twicEntry_t *twicHashFindInterface(const uint8_t interface);
void twicHashInsertInterface(twicEntry_t * const entity,
                             const uint8_t interface);
void twicHashRemoveInterface(const uint8_t interface);
void twicHashCleanupInterface(const uint8_t interface);
void twicHashInsert1stHandle(twicEntry_t * const entity,
                             const uint16_t handle);
void twicHashRemove1stHandle(const uint16_t handle);
void twicHashInsert2ndHandle(twicEntry_t * const entity,
                             const uint16_t handle);
uint8_t twicHashIsEmpty(void);
