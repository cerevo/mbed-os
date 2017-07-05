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

#ifndef _BLE_TZ10XX_BTLE_BTLE_TZ1EM_H_
#define _BLE_TZ10XX_BTLE_BTLE_TZ1EM_H_

#include "twic_interface.h"

#define BTLE_TZ1EM_OK       (0)
#define BTLE_TZ1EM_ERROR    (-1)

/** Configure TZ1EM for BLE stack
 *
 *  @returns
 *    BTLE_TZ1EM_OK.      Success
 *    BTLE_TZ1EM_ERROR.   Error
 */
extern int btle_tz1em_init( void );


/** Put TZ1EM that CPU enter sleep mode
 *
 *  @returns
 *    BTLE_TZ1EM_OK.      Success
 *    BTLE_TZ1EM_ERROR.   Error
 */
extern int btle_tz1em_go_to_sleep( void );

/** Put TZ1EM enter the Sunshine state
 *
 *  @returns
 *    BTLE_TZ1EM_OK.      Success
 *    BTLE_TZ1EM_ERROR.   Error
 */
extern int btle_tz1em_resume( void );

#endif /* _BLE_TZ10XX_BTLE_BTLE_TZ1EM_H_ */
