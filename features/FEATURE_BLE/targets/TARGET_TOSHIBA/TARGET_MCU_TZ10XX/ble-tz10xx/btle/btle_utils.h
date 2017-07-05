/* mbed Microcontroller Library
 * Copyright 2017, Cerevo Inc. 
 * All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _BTLE_UTILS_H_
#define _BTLE_UTILS_H_

#include "ble/blecommon.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "RNG_TZ10xx.h"
#include "PMU_TZ10xx.h"

#define BDADDR_LEN      6

#define BTLE_SETDWORD_BE(p, x) do { \
    (p)[0] = (uint8_t)((x) >> 56);     \
    (p)[1] = (uint8_t)((x) >> 48);     \
    (p)[2] = (uint8_t)((x) >> 40);     \
    (p)[3] = (uint8_t)((x) >> 32);     \
    (p)[4] = (uint8_t)((x) >> 24);     \
    (p)[5] = (uint8_t)((x) >> 16);     \
    (p)[6] = (uint8_t)((x) >>  8);     \
    (p)[7] = (uint8_t)((x)      );     \
    } while(0)

#define BTLE_CONVERT_BLE_TO_TWIC_PASSKEY(ble_passkey             \
                                            , twic_passkey)         \
                do {                                                \
                    uint32_t _passkey;                              \
                    _passkey = (ble_passkey[5] - '0')+              \
                               (ble_passkey[4] - '0') * 10 +        \
                               (ble_passkey[3] - '0') *100 +        \
                               (ble_passkey[2] - '0') * 1000 +      \
                               (ble_passkey[1] - '0') * 10000 +     \
                               (ble_passkey[0] - '0') * 100000;     \
                    twic_passkey[0] = (_passkey >> 0) & 0xff;       \
                    twic_passkey[1] = (_passkey >> 8) & 0xff;       \
                    twic_passkey[2] = (_passkey >> 16) & 0xff;      \
                    twic_passkey[3] = (_passkey >> 24) & 0xff;      \
                } while(0)

#define BTLE_CONVERT_TWIC_TO_BLE_PASSKEY(ble_passkey                         \
                                            , twic_passkey)                     \
                do {                                                            \
                    uint32_t (_passkey);                                        \
                    (_passkey) = (((twic_passkey)[3] << 24) |                   \
                               ((twic_passkey)[2] << 16) |                      \
                               ((twic_passkey)[1] << 8) |                       \
                               (twic_passkey)[0]);                              \
                    (ble_passkey)[5] = ((_passkey % 10) + '0');                 \
                    (ble_passkey)[4] = (((_passkey / 10) % 10) + '0');          \
                    (ble_passkey)[3] = (((_passkey / 100) % 10) + '0');         \
                    (ble_passkey)[2] = (((_passkey / 1000) % 10) + '0');        \
                    (ble_passkey)[1] = (((_passkey / 10000) % 10) + '0');       \
                    (ble_passkey)[0] = (((_passkey / 100000) % 10) + '0');      \
                } while(0)

void btle_error_handler(int error_code, uint32_t line_num, const uint8_t * p_file_name);

#ifdef DEBUG
#define BTLE_ERROR_HANDLER(ERR_CODE)                         \
    do                                                      \
    {                                                       \
        btle_error_handler((ERR_CODE), __LINE__, (uint8_t*) __FILE__);  \
    } while (0)
#else
#define BTLE_ERROR_HANDLER(ERR_CODE)                         \
    do                                                      \
    {                                                       \
        btle_error_handler((ERR_CODE), 0, 0);  \
    } while (0)
#endif

/** Convert mbed UUID to TWiC UUID
 *
 *  @param uuid_msb             The TWiC Least Significant Octet in the TWiC UUID.
 *  @param uuid_lsb             The TWiC Most Significant Octet in the TWiC UUID.
 *  @param base_uuid            mbed base UUID.
 *
 *  @returns
 *      NONE
 */
extern void btle_utils_copy_uuid_128(uint64_t *uuid_msb, uint64_t *uuid_lsb, const uint8_t *base_uuid);

/** Compare stored paired address in bonding infomation with new address connected
 *
 *  @param bd_a             Stored BD_ADDR.
 *  @param bd_b             Connected BD_ADDR.
 *
 *  @returns
 *    True.      Two address is the same
 *    false      Stored address is different with new connected address
 */
extern bool btle_utils_compare_bd_adddress(const uint8_t *bd_a, const uint8_t *bd_b);

/** Utils API for generate random TWiC Passkey
 *
 *  @param twic_passkey       TWiC Passkey is generated.
 *
 *  @returns
 *    0.        Success
 *    1.        Error
 */
extern int btle_utils_generate_random_twic_passkey(uint8_t *twic_passkey);

#ifdef __cplusplus
}
#endif

#endif /* ifndef _BTLE_UTILS_H_ */
