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
#ifndef _BTLE_CONFIG_H_
#define _BTLE_CONFIG_H_

#include "tz1em.h"

/* 1152: 0xF, 1536: 0xD, 2304: 0x7, 3072: 0x5, 4608: 0x4, 9216: 0x2 */
#define BTLE_UART_BAUDRATE 0x7u
/* <o> punctuation of UART2(us) <3-8192> */
#define BTLE_UART2_PUNCTUATION                      (3u)
/* The Factor wake up */
#define BTLE_TZ1EM_WF_GPIO    TZ1EM_WF_G2
/* Define power mode for BTLE on connection */
#define BTLE_POWER_LOW_SPEED                        (1u)
#define BTLE_POWER_NORMAL_SPEED                     (2u)
#define BTLE_POWER_HIGH_SPEED                       (3u)
#define BTLE_POWER_MODE       BTLE_POWER_HIGH_SPEED
/* BTLE Max retry execute TWiC API */
#define BTLE_MAX_RETRY                              (3u)

#define BTLE_ENTRY_SIZE_MAX                         (40u)
#define BTLE_ATT_MTU_DEFAULT                        (23u)
#define BTLE_INNER_EVENT_BUF_NUM                    (16u)

#define BTLE_DEFAULT_GA_CHAR_DEVICE_NAME            0x43, 0x44, 0x50, 0x30, 0x31
#define BTLE_DEFAULT_GA_CHAR_APPEARANCE             0x00u, 0x00u
#define BTLE_DEFAULT_GATT_CHAR_SERVICE_CHANGED      0x00u, 0x00u, 0x00u, 0x00u

#define BTLE_OOB_DATA_FLAG                          (1)
#define BTLE_MAX_ENCODE_KEY_SIZE                    (16)
#define BTLE_INITIATOR_DISTRIBUTES_ENCKEY           (1)
#define BTLE_INITIATOR_DISTRIBUTES_SIGN             (1)
#define BTLE_RESPONDER_DISTRIBUTES_ENCKEY           (1)
#define BTLE_RESPONDER_DISTRIBUTES_SIGN             (1)

#define BTLE_MIN_CE_LENGTH (0x20)
/* <o> maximum CE length (n * 0.625ms) <0x0-0xffff> */
#define BTLE_MAX_CE_LENGTH (0x30)

/**< Acceptance wait count for timeout */
#define BTLE_TIMEOUT_COUNT                    (6000)

#define BTLE_DEVICE_NAME_MAX_LENGTH             (10)

#define BTLE_SECURITY_REQUEST_MAX_COUNT         (2)

#endif /* ifndef _BTLE_CONFIG_H_ */
