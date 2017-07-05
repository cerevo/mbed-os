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

#ifdef __cplusplus
extern "C" {
#endif

#include <rt_misc.h>
#include <stdint.h>
#include <stddef.h>

#if defined(__ARMCC_VERSION)
extern "C" void _ttywrch(int ch)
{
}

extern "C" void _sys_exit(int return_code)
{
    while(1);
}

extern "C" void *__dso_handle = (void *)NULL;

#endif

#ifdef __cplusplus
}
#endif
