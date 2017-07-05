/* ----------------------------------------------------------------------
 * Copyright (C) 2013 ARM Limited. All rights reserved.
 *  
 * $Date:        12. Sept 2013
 * $Revision:    V1.10
 *  
 * Project:      USB Driver common definitions
 * -------------------------------------------------------------------- */

/* History:
 *  Version 1.10
 *    Namespace prefix ARM_ added
 *  Version 1.01
 *    Added PID Types
 *  Version 1.00
 *    Initial release
 */ 

#ifndef __DRIVER_USB_H
#define __DRIVER_USB_H

#include "Driver_Common.h"

/* USB Role */
#define ARM_USB_ROLE_NONE                0
#define ARM_USB_ROLE_HOST                1
#define ARM_USB_ROLE_DEVICE              2

/* USB Pins */
#define ARM_USB_PINS_DATA               (3 << 0) ///< USB D+/D- pin
#define ARM_USB_PIN_DP                  (1 << 0) ///< USB D+ pin
#define ARM_USB_PIN_DM                  (1 << 1) ///< USB D- pin
#define ARM_USB_PIN_VBUS                (1 << 2) ///< USB VBUS pin
#define ARM_USB_PIN_OC                  (1 << 3) ///< USB OverCurrent pin
#define ARM_USB_PIN_ID                  (1 << 4) ///< USB ID pin            

/* USB Speed */
#define ARM_USB_SPEED_LOW                0
#define ARM_USB_SPEED_FULL               1
#define ARM_USB_SPEED_HIGH               2

/* USB PID Types */
#define ARM_USB_PID_OUT                  1
#define ARM_USB_PID_IN                   9
#define ARM_USB_PID_SOF                  5
#define ARM_USB_PID_SETUP                13
#define ARM_USB_PID_DATA0                3
#define ARM_USB_PID_DATA1                11
#define ARM_USB_PID_DATA2                7
#define ARM_USB_PID_MDATA                15
#define ARM_USB_PID_ACK                  2
#define ARM_USB_PID_NAK                  10
#define ARM_USB_PID_STALL                14
#define ARM_USB_PID_NYET                 6
#define ARM_USB_PID_PRE                  12
#define ARM_USB_PID_ERR                  12
#define ARM_USB_PID_SPLIT                8
#define ARM_USB_PID_PING                 4
#define ARM_USB_PID_RESERVED             0

/* USB Endpoint Address (bEndpointAddress) */
#define ARM_USB_ENDPOINT_NUMBER_MASK     0x0F
#define ARM_USB_ENDPOINT_DIRECTION_MASK  0x80

/**
\brief  USB Endpoint Type
*/
typedef enum _ARM_USB_ENDPOINT_TYPE {
  ARM_USB_ENDPOINT_CONTROL,              ///< Control Endpoint
  ARM_USB_ENDPOINT_ISOCHRONOUS,          ///< Isochronous Endpoint
  ARM_USB_ENDPOINT_BULK,                 ///< Bulk Endpoint
  ARM_USB_ENDPOINT_INTERRUPT,            ///< Interrupt Endpoint
} ARM_USB_ENDPOINT_TYPE;

/*  Endpoint Maximum Packet Size (wMaxPacketSize) */
#define ARM_USB_ENDPOINT_MAX_PACKET_SIZE_MASK           0x07FF
#define ARM_USB_ENDPOINT_MICROFRAME_TRANSACTIONS_MASK   0x1800
#define ARM_USB_ENDPOINT_MICROFRAME_TRANSACTIONS_1      0x0000
#define ARM_USB_ENDPOINT_MICROFRAME_TRANSACTIONS_2      0x0800
#define ARM_USB_ENDPOINT_MICROFRAME_TRANSACTIONS_3      0x1000


#endif /* __DRIVER_USB_H */
