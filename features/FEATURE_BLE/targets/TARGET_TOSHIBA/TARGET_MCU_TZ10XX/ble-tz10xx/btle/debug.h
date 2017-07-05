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

#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
/* Exported macro ------------------------------------------------------------*/
/* NOTE: IF ENABLE DEBUG, PLEASE COMMENT DEFINE twicLog, twicTrace, twicPrintf in tz1sm_config.h */
#undef DEBUG
#ifdef DEBUG
#include <stdio.h>

#define _LOGD(fmt, ...) printf("%s: "fmt"%s", __func__, __VA_ARGS__)
#define _LOGT()        printf("%s:%d \r\n", __func__, __LINE__)
#define _LOGP(...)    printf(__VA_ARGS__)

#define LOGD(...) _LOGD(__VA_ARGS__, "")
#define twicLog(...) _LOGD(__VA_ARGS__, "")
#define twicTrace()     _LOGT()
#define twicPrintf(...) _LOGP(__VA_ARGS__)

#else

#define LOGD(...)
#define twicLog(...)
#define twicTrace()
#define twicPrintf(...)

#endif

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H */
