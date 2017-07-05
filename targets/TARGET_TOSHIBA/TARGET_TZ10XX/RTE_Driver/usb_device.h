/**
 * @file usb_device.h
 * @brief USB Device Driver Define
 * @version V0.0
 * @date $Date:: 2014-10-03 16:48:52 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef __USB_DEVICE_H__
#define __USB_DEVICE_H__

#include <stdint.h>

/* USB event flag bit definitions */
#define USB_EVENT_ENUMDONE 1U
#define USB_EVENT_REQDONE 2U
#define USB_EVENT_REQUEST 4U 
#define USB_EVENT_DISCONNECT 32U
#define USB_EVENT_EP(n,d)  (1 << (16 + (d) + (n * 2)))

typedef union usb_event {
	/** raw register data */
	uint32_t d32;
	/** register bits */
	struct {
		uint32_t enumdone      :1; // Completion of linkup after the USB Reset.
		uint32_t requestdone   :1; // EP0 SETUP transfer done
		uint32_t request       :1; // EP0 SETUP packet received.
		uint32_t               :1; 
		uint32_t               :1; 
		uint32_t sesenddet     :1; // VBus OFF
		uint32_t reset         :1; // USB Reset
		uint32_t wkupintr      :1; // USB Resume
		uint32_t sessreqintr   :1; // VBus ON
		uint32_t erlysuspend   :1;
		uint32_t usbsuspend    :1; // USB Suspend
		uint32_t requestabort  :1; // EP0 SETUP packet received during middle of EP0 transfer.
		uint32_t afterxferdone :1; // after xferdone
		uint32_t reserved13_15 :3;
		
		uint32_t ep0out        :1;
		uint32_t ep0in         :1;
		uint32_t ep1out        :1;
		uint32_t ep1in         :1;
		uint32_t ep2out        :1;
		uint32_t ep2in         :1;
		uint32_t ep3out        :1;
		uint32_t ep3in         :1;
		uint32_t reserved21_31 :8;
	} b;
} usb_event_t;

/**
 * States of EP0.
 */
typedef enum EP0_State {
	EP0_DISCONNECT,		/* no host */
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_IN_STATUS_PHASE,
	EP0_OUT_STATUS_PHASE,
	EP0_STALL,
} EP0_State_t;

/**
 * States of Endpoint
 */
#define USB_EP_STOPPED 0x80

typedef enum {
	USB_EP_CLOSED   = 0,
	USB_EP_RUNNING  = 1,
	USB_EP_ABORTING = 2,
	USB_EP_ABORTED  = USB_EP_ABORTING | USB_EP_STOPPED,
	USB_EP_STALLING = 3,
	USB_EP_HALTED   = USB_EP_STALLING | USB_EP_STOPPED,
	USB_EP_STALL_PENDING= 4,
	USB_EP_TIMEOUT= 5,
} USB_ep_state_t;

/* USB status */
typedef enum {
	USB_OK = 0,         /* Success */
	USB_EINVAL,         /* Parameter error, etc.. */
	USB_EIO,            /* Transfer error */
	USB_ENXIO,          /* Address error */
	USB_EBUSY,          /* Busy */
	USB_ETIMEOUT,       /* Timeout */
	USB_EABORT,         /* Transfer abort */
	USB_ENOSYS,         /* Non support */
} USB_err_t; 

/* USB Link speed */
typedef enum {
	USB_SPEED_HIGH = 0,
	USB_SPEED_FULL,
	USB_SPEED_LOW,
} USB_speed_t; 

/* USB Device state */
typedef enum {
	USB_STATE_DETACH = 0,
	USB_STATE_ATTACH,
	USB_STATE_POWERED,
	USB_STATE_DEFAULT,
	USB_STATE_ADDRESSED,
	USB_STATE_CONFIGURED,
} USB_state_t; 

#define USB_STATE_SUSPEND_BIT 0x80
#define USB_STATE_MASK        0x7F 
#define USB_STATE_GETSTATE(s) ((USB_state_t)((s) & USB_STATE_MASK))

/* USB Transfer management */
typedef struct USB_transfer {
	void*      pvBuffer;   /* Transfer data buffer */
	uint16_t   usLen;      /* Transfer request size */
	uint16_t   usActLen;   /* Transferred size */
	uint16_t   usTrfSz;    /* Application-programmed initial transfer size */
	USB_err_t  iStatus;    /* Transfer status */
} USB_xfer_t; 

#endif
