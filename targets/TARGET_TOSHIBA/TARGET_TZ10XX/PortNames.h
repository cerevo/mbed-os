/* mbed Microcontroller Library
 * Copyright 2017, Cerevo Inc. 
 *
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
#ifndef MBED_PORTNAMES_H
#define MBED_PORTNAMES_H

#include "PinNames.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    Gpio_Port0   = 0,
    Gpio_Port1   = 1,
    Gpio_Port2   = 2,
    Gpio_Port3   = 3,
    INVALID_Port = (int)NC
} PortName;

#define TABLE_PORTPIN_ROW(index, data1, data2) \
(((index) == 1) ? (data1) : (((index) == 2) ? (data2) : NC))

#ifdef __cplusplus
}
#endif
#endif
