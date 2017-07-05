/**
 * @file USBD_TZ10xx.h
 * @brief a header file for TZ10xx USBD driver
 * @version V0.0
 * @date $Date:: 2014-10-14 14:07:40 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */
#ifndef USBD_TZ10XX_H
#define USBD_TZ10XX_H

#include <Driver_USBD.h>
#include <RTE_Device.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _USBD_TZ10xx_ERROR {    // callback   ,  param content,   description
	USBD_TZ10xx_ERROR_NONE,
	USBD_TZ10xx_ERROR_EP_STSPHSERCVD,// EPEvent(-1),  uint8_t ep_addr  The host initiated premature status stage at the control write transfer.
	USBD_TZ10xx_ERROR_EP_SETUP_LOST, // EPEvent(-1),  uint8_t ep_addr  SETUP packet may have been lost.
	USBD_TZ10xx_ERROR_EP_AHB_ERROR,  // EPEvent(-1),  uint8_t ep_addr  ahberr interrupt status occured.
	USBD_TZ10xx_ERROR_EP_TIMEOUT,    // EPEvent(-1),  uint8_t ep_addr  timeout interrupt status occured.
	USBD_TZ10xx_ERROR_EP_MISMATCH,   // EPEvent(-1),  uint8_t ep_addr  intknepmis interrupt status occured.
	USBD_TZ10xx_ERROR_MISMATCH,      // EPEvent(-1),  none             epmismatch interrupt status occured.
	USBD_TZ10xx_ERROR_CORE_HANG,     // DevEvent(-1), uint32_t locate  The status register wait timeout.
} USBD_TZ10xx_ERROR;

///< Control fields
#define USBD_TZ10xx_CONTROL_CODE_Pos      0
#define USBD_TZ10xx_CONTROL_CODE_Mask     0x000000FFUL
#define USBD_TZ10xx_CONTROL_EP_ADDR_Pos   8
#define USBD_TZ10xx_CONTROL_EP_ADDR_Mask  0x0000FF00UL

///< Control code field
#define USBD_TZ10xx_CONTROL_ENABLE_EVENT (0x01UL << USBD_TZ10xx_CONTROL_CODE_Pos)
#define USBD_TZ10xx_CONTROL_DISABLE_EVENT (0x02UL << USBD_TZ10xx_CONTROL_CODE_Pos)

///< Control ep_addr field
#define USBD_TZ10xx_CONTROL_EP_ADDR(ep_addr) (((ep_addr) & 0xFF) << USBD_TZ10xx_CONTROL_EP_ADDR_Pos)

///< Kind of event
typedef enum _USBD_TZ10xx_EVENT {    // callback    ,  description
	USBD_TZ10xx_EVENT_SOF ,          // DevEvent(-2),  SOF Received
} USBD_TZ10xx_EVENT;

typedef struct _TZ10XX_DRIVER_USBD {
	// Driver_USBD.h/typedef struct _ARM_DRIVER_USBD {
	ARM_DRIVER_VERSION       (*GetVersion)         (void);                                               ///< Pointer to \ref ARM_USBD_GetVersion : Get driver version.
	ARM_USBD_CAPABILITIES (*GetCapabilities)    (void);                                               ///< Pointer to \ref ARM_USBD_GetCapabilities : Get driver capabilities.
	ARM_USBD_STATUS       (*Initialize)         (ARM_USBD_SignalDeviceEvent_t   cb_device_event,                     
	                                             ARM_USBD_SignalEndpointEvent_t cb_endpoint_event);   ///< Pointer to \ref ARM_USBD_Initialize : Initialize USB Device Interface. 
	ARM_USBD_STATUS       (*Uninitialize)       (void);                                               ///< Pointer to \ref ARM_USBD_Uninitialize : De-initialize USB Device Interface.
	ARM_USBD_STATUS       (*PowerControl)       (ARM_POWER_STATE state);                              ///< Pointer to \ref ARM_USBD_PowerControl : Control USB Device Interface Power.
	ARM_USBD_STATUS       (*DeviceConnect)      (void);                                               ///< Pointer to \ref ARM_USBD_DeviceConnect : Connect USB Device.
	ARM_USBD_STATUS       (*DeviceDisconnect)   (void);                                               ///< Pointer to \ref ARM_USBD_DeviceDisconnect : Disconnect USB Device.
	ARM_USBD_STATE        (*DeviceGetState)     (void);                                               ///< Pointer to \ref ARM_USBD_DeviceGetState : Get current USB Device State.
	ARM_USBD_STATUS       (*DeviceRemoteWakeup) (void);                                               ///< Pointer to \ref ARM_USBD_DeviceRemoteWakeup : Trigger USB Remote Wakeup.
	ARM_USBD_STATUS       (*DeviceSetAddress)   (uint8_t dev_addr, ARM_USBD_SET_ADDRESS_STAGE stage); ///< Pointer to \ref ARM_USBD_DeviceSetAddress : Set USB Device Address.
	ARM_USBD_STATUS       (*DeviceConfigure)    (bool configure);                                     ///< Pointer to \ref ARM_USBD_DeviceConfigure : Configure/unconfigure USB Device.
	ARM_USBD_STATUS       (*EndpointConfigure)  (uint8_t ep_addr,
	                                             ARM_USB_ENDPOINT_TYPE ep_type,
	                                             uint16_t ep_max_packet_size);                        ///< Pointer to \ref ARM_USBD_EndpointConfigure : Configure USB Endpoint.
	ARM_USBD_STATUS       (*EndpointUnconfigure)(uint8_t ep_addr);                                    ///< Pointer to \ref ARM_USBD_EndpointUnconfigure : Unconfigure USB Endpoint.
	ARM_USBD_STATUS       (*EndpointStall)      (uint8_t ep_addr, bool stall);                        ///< Pointer to \ref ARM_USBD_EndpointStall : Set/Clear Stall for USB Endpoint.
	ARM_USBD_STATUS       (*EndpointReadStart)  (uint8_t ep_addr,       uint8_t *buf, uint32_t len);  ///< Pointer to \ref ARM_USBD_EndpointReadStart : Start USB Endpoint Read operation.
	int32_t               (*EndpointRead)       (uint8_t ep_addr,       uint8_t *buf, uint32_t len);  ///< Pointer to \ref ARM_USBD_EndpointRead : Read data from USB Endpoint.
	int32_t               (*EndpointWrite)      (uint8_t ep_addr, const uint8_t *buf, uint32_t len);  ///< Pointer to \ref ARM_USBD_EndpointWrite : Write data to USB Endpoint.
	ARM_USBD_STATUS       (*EndpointAbort)      (uint8_t ep_addr);                                    ///< Pointer to \ref ARM_USBD_EndpointAbort : Abort current USB Endpoint transfer.
	uint16_t              (*GetFrameNumber)     (void);                                               ///< Pointer to \ref ARM_USBD_GetFrameNumber : Get current USB Frame Number.
	// } Driver_USBD.h/typedef struct _ARM_DRIVER_USBD
	ARM_USBD_STATUS       (*EndpointWriteStart) (uint8_t ep_addr, const uint8_t *buf, uint32_t len);  ///< Start USB Endpoint Write operation.
	uint32_t              (*EndpointTransferGetResult) (uint8_t ep_addr);                             ///< Get result of USB Endpoint transfer.
	ARM_USBD_STATUS       (*ReadSetupPacket)  (uint8_t *setup);                                       ///< Read setup packet received over Control Endpoint.
	ARM_USBD_STATUS       (*EndpointSetDPID)    (uint8_t ep_addr, uint8_t toggle);                    ///< Set Endpoint DATAn PID.
	ARM_USBD_STATUS       (*DeviceSetTestMode)  (uint8_t mode);                                       ///< Set Device Test mode.
	USBD_TZ10xx_ERROR     (*GetError)   (uint32_t *param);                                            ///< Get Error status
	ARM_USBD_STATUS       (*Control)   (uint32_t  control, uint32_t arg);                             ///< Control USB Interface. 
} const TZ10XX_DRIVER_USBD;

#if RTE_USB2FS
extern TZ10XX_DRIVER_USBD Driver_USBD0;
#endif

#ifdef __cplusplus
}
#endif

#endif /* USBD_TZ10XX_H */
