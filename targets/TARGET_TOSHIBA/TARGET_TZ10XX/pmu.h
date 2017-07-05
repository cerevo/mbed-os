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
#ifndef MBED_PMU_H
#define MBED_PMU_H

#include "mbed_assert.h"
#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the PMU .
 *
 *  @param      NONE.
 *
 *  @note       If a module use PMU RTE Driver, then it must call PmuInit() in initialization code
 *
 *  @returns
 *    NONE.
 */
void PmuInit(void);

#ifdef __cplusplus
}
#endif

#endif
