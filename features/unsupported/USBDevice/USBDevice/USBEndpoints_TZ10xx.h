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

#define NUMBER_OF_LOGICAL_ENDPOINTS (3)
#define NUMBER_OF_PHYSICAL_ENDPOINTS (NUMBER_OF_LOGICAL_ENDPOINTS * 2)
#define NUMBER_OF_ENDPOINTS          (NUMBER_OF_PHYSICAL_ENDPOINTS + 2)  /* Includes EP0 */
/*      Endpoint    No.     Type(s)       MaxPacket */
/*      ----------------    ------------  --------- */
#define EP0OUT      (0)  /* Control       64        */
#define EP0IN       (1)  /* Control       64        */
#define EP1IN       (3)  /* Int/Bulk      64/64     */
#define EP2OUT      (4)  /* Int/Bulk      64/64     */
#define EP1OUT      (2)  /* Int/Bulk      64/64     */
#define EP2IN       (5)  /* Int/Bulk      64/64     */
#define EP3OUT      (6)  /* Int/Bulk      64/64     */
#define EP3IN       (7)  /* Int/Bulk      64/64     */

#define USBD_EP_TO_ADDR(ep)  (((ep)>>1) | (((ep) & 1   ) ? 0x80 : 0x00))
#define USBD_ADDR_TO_EP(ep)  (((ep)<<1) | (((ep) & 0x80) ? 0x01 : 0x00))

/* Maximum Packet sizes */
#define MAX_PACKET_SIZE_EP0 (64)
#define MAX_PACKET_SIZE_EP1 (64) /* Int/Bulk */
#define MAX_PACKET_SIZE_EP2 (64) /* Int/Bulk */
#define MAX_PACKET_SIZE_EP3 (64) /* Int/Bulk */

/* Generic endpoints - intended to be portable accross devices */
/* and be suitable for simple USB devices. */

/* Bulk endpoint */
#define EPBULK_OUT  (EP1OUT)
#define EPBULK_IN   (EP1IN)
#define EPBULK_OUT_callback         EP1_OUT_callback
#define EPBULK_IN_callback          EP1_IN_callback
/* Interrupt endpoint */
#define EPINT_OUT   (EP2OUT)
#define EPINT_IN    (EP2IN)
#define EPINT_OUT_callback          EP2_OUT_callback
#define EPINT_IN_callback           EP2_IN_callback
/* TZ10xx do not support Isochronous endpoint */
/* The definition to bypass (USBAudio) compilation of USBLib */
#define EPISO_OUT   (-1)
#define EPISO_IN    (-1)
#define EPISO_OUT_callback      EP3_OUT_callback
#define EPISO_IN_callback       EP3_IN_callback

#define MAX_PACKET_SIZE_EPBULK  (MAX_PACKET_SIZE_EP1)
#define MAX_PACKET_SIZE_EPINT   (MAX_PACKET_SIZE_EP2)
#define MAX_PACKET_SIZE_EPISO   (0)
