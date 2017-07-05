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
#ifndef MBED_GPIO_OBJECT_H
#define MBED_GPIO_OBJECT_H

#include "mbed_assert.h"
#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t pin_id;
    PinName pin;
    uint32_t mask;

    __IO uint32_t *reg_dir;
    __IO uint32_t *reg_ie;
    __IO uint32_t *reg_ic;
} gpio_t;

gpio_Type *gpio_get_pointer(uint32_t pin_id);

#ifdef __cplusplus
}
#endif

#endif
