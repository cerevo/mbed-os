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
#include "mbed.h"
#include "btle_utils.h"
#include "debug.h"

extern TZ10XX_DRIVER_RNG Driver_RNG;

/** Convert mbed UUID to TWiC UUID
 *
 *  @param uuid_msb             The TWiC Least Significant Octet in the TWiC UUID.
 *  @param uuid_lsb             The TWiC Most Significant Octet in the TWiC UUID.
 *  @param base_uuid            mbed base UUID.
 *
 *  @returns
 *      NONE
 */
void btle_utils_copy_uuid_128(uint64_t *uuid_msb, uint64_t *uuid_lsb, const uint8_t *base_uuid)
{
    memcpy(uuid_lsb, base_uuid, sizeof(uint64_t));
    memcpy(uuid_msb, base_uuid + sizeof(uint64_t), sizeof(uint64_t));
}

/** Compare stored paired address in bonding infomation with new address connected
 *
 *  @param bd_a             Stored BD_ADDR.
 *  @param bd_b             Connected BD_ADDR.
 *
 *  @returns
 *    True.      Two address is the same
 *    False      Stored address is different with new connected address
 */
bool btle_utils_compare_bd_adddress(const uint8_t* bd_a, const uint8_t* bd_b)
{
    bool ret = true;
    for(unsigned i=0; i<BDADDR_LEN; i++) {
        if(bd_a[i] != bd_b[i]) {
            ret = false;
            break;
        }
    }

    return ret;
}

/** Utils API for generate random TWiC Passkey
 *
 *  @param twic_passkey       TWiC Passkey is generated.
 *
 *  @returns
 *    0.        Success
 *    1.        Error
 */
int btle_utils_generate_random_twic_passkey(uint8_t* twic_passkey)
{
    uint32_t randval;
    RNG_STATUS status = RNG_OK;

    Driver_PMU.SetPowerDomainState(PMU_PD_ENCRYPT, PMU_PD_MODE_ON);
    status = Driver_RNG.Initialize();
    if(status != RNG_OK) {
        return 1;
    }

    status = Driver_RNG.PowerControl(ARM_POWER_FULL);
    if(status != RNG_OK) {
        return 1;
    }

    status = Driver_RNG.Read(&randval);
    if(status != RNG_OK) {
        return 1;
    }

    status = Driver_RNG.Uninitialize();
    if(status != RNG_OK)  {
        return 1;
    }
    /* get 6 least significant decimal digits (000,000 to 999,999) */
    randval = randval % 100000;

    twic_passkey[0] = (randval >> 0) & 0xff;
    twic_passkey[1] = (randval >> 8) & 0xff;
    twic_passkey[2] = (randval >> 16) & 0xff;
    twic_passkey[3] = (randval >> 24) & 0xff;

    return 0;
}
