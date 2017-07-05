/* mbed Microcontroller Library
 * Copyright (c) 2017 ARM Limited
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

#ifndef MBED_MBED_RTX_H
#define MBED_MBED_RTX_H

#if defined(TARGET_BLUENINJA_CDP_TZ01B)
#include <stdint.h>

#if defined(__CC_ARM)
extern uint32_t __heap_base[];
extern uint32_t Heap_Size[];
extern uint32_t Stack_Mem[];
extern uint32_t Stack_Size[];
#define HEAP_START              ((unsigned char *)__heap_base)
#define HEAP_SIZE               ((uint32_t)Heap_Size)
#define ISR_STACK_START         ((unsigned char*)Stack_Mem)
#define ISR_STACK_SIZE          ((uint32_t)Stack_Size)
#elif defined(__GNUC__)
extern uint32_t __heap_base[];
extern uint32_t __heap_limit[];
extern uint32_t __stack_limit[];
extern uint32_t __initial_sp[];
#define HEAP_START              ((unsigned char *)__heap_base)
#define HEAP_SIZE               ((uint32_t)((uint32_t)__heap_limit - (uint32_t)__heap_base))
#define ISR_STACK_START         ((unsigned char*)__stack_limit)
#define ISR_STACK_SIZE          ((uint32_t)((uint32_t)__initial_sp - (uint32_t)__stack_limit))
#endif

#ifndef OS_TASKCNT
#define OS_TASKCNT              14
#endif

#ifndef OS_MAINSTKSIZE
#define OS_MAINSTKSIZE          128
#endif

#ifndef OS_CLOCK
#define OS_CLOCK                48000000
#endif

#endif

#endif  /* MBED_MBED_RTX_H */
