/**
 * @file usb_desc.h
 * @brief USB Descriptor
 * @version V0.0
 * @date $Date:: 2014-10-28 11:55:42 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef __USB_DESC_H__
#define __USB_DESC_H__

typedef uint8_t     USB_byte_t; 
typedef uint8_t     USB_word_t[2]; 
typedef uint8_t     USB_dword_t[4]; 


#define USB_GETW(word)          (((uint16_t)((word)[0])) |      \
                                 ((uint16_t)((word)[1]) << 8))
#define USB_SETW(word,value)    ((word)[0] = (uint8_t)(((value) >> 0) & 0xff), \
                                 (word)[1] = (uint8_t)(((value) >> 8) & 0xff))
#define USB_GETDW(word)         (((uint32_t)((word)[0]))       | \
                                 ((uint32_t)((word)[1]) <<  8) | \
                                 ((uint32_t)((word)[2]) << 16) | \
                                 ((uint32_t)((word)[3]) << 24))
#define USB_SETDW(word,value)   ((word)[0] = (uint8_t)(((value) >>  0) & 0xff), \
                                 (word)[1] = (uint8_t)(((value) >>  8) & 0xff), \
                                 (word)[2] = (uint8_t)(((value) >> 16) & 0xff), \
                                 (word)[3] = (uint8_t)(((value) >> 24) & 0xff))

typedef struct USB_deviceDesc {
	USB_byte_t		bLength; 
	USB_byte_t		bDescriptorType; 
	USB_word_t		bcdUSB; 
	USB_byte_t		bDeviceClass; 
#define USB_CLASS_INTERFACE         (0x00)
#define USB_CLASS_VENDOR            (0xff)
	USB_byte_t		bDeviceSubClass; 
#define USB_SUBCLASS_INTERFACE      (0x00)
	USB_byte_t		bDeviceProtocol; 
#define USB_PROTOCOL_INTERFACE      (0x00)
	USB_byte_t		bMaxPacketSize0; 
	USB_word_t		idVendor; 
	USB_word_t		idProduct; 
	USB_word_t		bcdDevice; 
	USB_byte_t		iManufacturer; 
	USB_byte_t		iProduct; 
	USB_byte_t		iSerialNumber; 
	USB_byte_t		bNumConfigurations; 
} USB_ddesc_t; 
#define USB_DDESC_SIZE          (18)

typedef struct USB_deviceQualifierDesc {
	USB_byte_t		bLength; 
	USB_byte_t		bDescriptorType; 
	USB_word_t		bcdUSB; 
	USB_byte_t		bDeviceClass; 
	USB_byte_t		bDeviceSubClass; 
	USB_byte_t		bDeviceProtocol; 
	USB_byte_t		bMaxPacketSize0; 
	USB_byte_t		bNumConfigurations; 
	USB_byte_t		bReserved; 
} USB_qdesc_t; 
#define USB_QDESC_SIZE          (10)

typedef struct USB_configurationDesc {
	USB_byte_t		bLength; 
	USB_byte_t		bDescriptorType; 
	USB_word_t		wTotalLength; 
	USB_byte_t		bNumInterfaces; 
	USB_byte_t		bConfigurationValue; 
	USB_byte_t		iConfiguration; 
	USB_byte_t		bmAttributes; 
#define USB_CA_SELF_POWERED    (0x40)
#define USB_CA_REMOTE_WAKEUP   (0x20)
	USB_byte_t		bMaxPower; 
} USB_cdesc_t; 
#define USB_CDESC_SIZE          (9)

typedef struct USB_interfaceAssociationDesc {
	USB_byte_t		bLength; 
	USB_byte_t		bDescriptorType; 
	USB_byte_t		bFirstInterface; 
	USB_byte_t		bInterfaceCount; 
	USB_byte_t		bFunctionClass; 
	USB_byte_t		bFunctionSubClass; 
	USB_byte_t		bFunctionProtocol; 
	USB_byte_t		iFunction; 
} USB_iadesc_t; 
#define USB_IADESC_SIZE          (8)

typedef struct USB_interfaceDesc {
	USB_byte_t		bLength; 
	USB_byte_t		bDescriptorType; 
	USB_byte_t		bInterfaceNumber; 
	USB_byte_t		bAlternateSetting; 
	USB_byte_t		bNumEndpoints; 
	USB_byte_t		bInterfaceClass; 
	USB_byte_t		bInterfaceSubClass; 
	USB_byte_t		bInterfaceProtocol; 
	USB_byte_t		iInterface; 
} USB_idesc_t; 
#define USB_IDESC_SIZE          (9)

typedef struct USB_endpointDesc {
	USB_byte_t		bLength; 
	USB_byte_t		bDescriptorType; 
	USB_byte_t		bEndpointAddress; 
#define USB_EP_GETDIR(adr)      ((adr) & 0x80)
#define  USB_EP_DIR_IN          (0x80)
#define  USB_EP_DIR_OUT         (0x00)
#define USB_EP_GETNUMBER(adr)   (((uint8_t)adr) & 0x0f)
	USB_byte_t		bmAttributes; 
#define USB_EP_GETTYPE(atr)     (((uint8_t)atr) & 0x03)
#define  USB_EP_CTRL            (0x00)
#define  USB_EP_ISOC            (0x01)
#define  USB_EP_BULK            (0x02)
#define  USB_EP_INTR            (0x03)
#define  USB_EP_IS_PERIODIC(type) (((uint8_t)type) & 0x1)
	USB_word_t		wMaxPacketSize; 
	USB_byte_t		bInterval; 
} USB_edesc_t; 
#define USB_EDESC_SIZE          (7)
#define USB_ep_addr(n,d)   (uint8_t)((USB_EP_GETNUMBER(n)) | (((uint8_t)d) ? USB_EP_DIR_IN : USB_EP_DIR_OUT))

typedef struct USB_stringDesc {
	USB_byte_t		bLength; 
	USB_byte_t		bDescriptorType; 
	USB_word_t		bString; 
} USB_sdesc_t; 

/* Device request */
typedef struct USB_deviceRequest {
    USB_byte_t  bmRequestType; 
    USB_byte_t  bRequest; 
    USB_word_t  wValue; 
    USB_word_t  wIndex; 
    USB_word_t  wLength; 
} USB_devReq_t; 
#define USB_DEVREQ_SIZE         (8)

/* bmRequestType */
#define USB_DR_GETRECIP(bm)     ((bm) & 0x1f)
#define  USB_DR_RECIP_DEVICE    (0x00)
#define  USB_DR_RECIP_INTERFACE (0x01)
#define  USB_DR_RECIP_ENDPOINT  (0x02)
#define  USB_DR_RECIP_OTHER     (0x03)
#define USB_DR_GETTYPE(bm)      ((bm) & 0x60)
#define  USB_DR_TYPE_STANDARD   (0x00)
#define  USB_DR_TYPE_CLASS      (0x20)
#define  USB_DR_TYPE_VENDOR     (0x40)
#define USB_DR_GETDIR(bm)       ((bm) & 0x80)
#define  USB_DR_DIR_H2D         (0x00)    /* Host to Device */
#define  USB_DR_DIR_D2H         (0x80)    /* Device to Host */

/* bRequest (9.4) */
#define USB_DR_GET_STATUS       (0x00)
#define USB_DR_CLEAR_FEATURE    (0x01)
#define USB_DR_SET_FEATURE      (0x03)
#define USB_DR_SET_ADDRESS      (0x05)
#define USB_DR_GET_DESC         (0x06)
#define USB_DR_SET_DESC         (0x07)
#define USB_DR_GET_CONFIG       (0x08)
#define USB_DR_SET_CONFIG       (0x09)
#define USB_DR_GET_INTERFACE    (0x0a)
#define USB_DR_SET_INTERFACE    (0x0b)
#define USB_DR_SYNCH_FRAME      (0x0c)

#define USB_DESC_DEVICE             (0x01)
#define USB_DESC_CONFIG             (0x02)
#define USB_DESC_STRING             (0x03)
#define USB_DESC_INTERFACE          (0x04)
#define USB_DESC_ENDPOINT           (0x05)
#define USB_DESC_DEVICE_QUALIFIER   (0x06)
#define USB_DESC_OTHER_SPEED        (0x07)
#define USB_DESC_INTERFACE_POWER    (0x08)
#define USB_DESC_OTG                (0x09)
#define USB_DESC_DEBUG              (0x0A)
#define USB_DESC_INTERFACE_ASSOCIATION (0x0B)

/* Feature selectors for Device, Endpoint */
#define USB_FD_REMOTE_WAKEUP    (0x01)
#define USB_FD_TEST_MODE        (0x02)
#define USB_FE_ENDPOINT_HALT    (0x00)

/* GetStatus Information for Device, Interface, Endpoint */
#define USB_SD_SELF_POWERED     (0x0001)
#define USB_SD_REMOTE_WAKEUP    (0x0002)
#define USB_SI_RESERVED         (0x0000)
#define USB_SE_HALT             (0x0001)

/* LANG-ID */
#define USB_LANGID_EN_US        (0x0409)

/* class codes */
#define USB_CLASS_MISC (0xEF)

#endif /* __USB_DESC_H__ */
