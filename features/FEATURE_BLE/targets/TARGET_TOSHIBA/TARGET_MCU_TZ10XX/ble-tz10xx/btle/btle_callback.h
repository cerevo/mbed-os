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
#ifndef _BTLE_CALLBACK_H_
#define _BTLE_CALLBACK_H_

#include "tz1SecurityManager.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "twic_interface.h"
/*
 * TWiC functions callback structure
 */
extern const twicIfLeCb_t twic_common_callback;
extern const twicIfLeServerCb_t twic_server_callback;
extern const twicIfLeClientCb_t twic_client_callback;
extern const twicIfLeSmpRCb_t twic_smp_responder_callback;
extern const twicIfLeSmpICb_t twic_smp_initiator_callback;

/** Initialize global variable use in BTLE callback functions.
 *
 *  @param 
 *      NONE.
 *
 *  @returns
 *      NONE.
 */
extern void btle_callback_on_init_variable(void);

/** Call mbed display passkey functions callback
 *
 *  @param conn_handle              Connection handle.
 *  @param passkey                  TWiC passkey pointer.
 *
 *  @returns
 *    NONE.
 */
extern void btle_callback_display_passkey_event(uint16_t conn_handle, const uint8_t *passkey);

#ifdef __cplusplus
}
#endif

#endif /* _BTLE_CALLBACK_H_ */
