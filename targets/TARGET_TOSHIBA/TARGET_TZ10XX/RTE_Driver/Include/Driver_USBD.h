/* ----------------------------------------------------------------------
 * Copyright (C) 2013 ARM Limited. All rights reserved.
 *  
 * $Date:        12. Sept 2013
 * $Revision:    V1.10
 *  
 * Project:      USB Device Driver for Middleware
 * -------------------------------------------------------------------- */

/* History:
 *  Version 1.10
 *    Namespace prefix ARM_ added
 *  Version 1.00
 *    Initial release
 */ 

#ifndef __DRIVER_USBD_H
#define __DRIVER_USBD_H

#include "Driver_USB.h"

#define ARM_USBD_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,10)    /* API version */


/**
\brief  USB Device State
*/
typedef struct _ARM_USBD_STATE {
  uint8_t powered     : 1;              ///< USB Device powered flag
  uint8_t connected   : 1;              ///< USB Device connected flag
  uint8_t active      : 1;              ///< USB Device active lag
  uint8_t speed       : 2;              ///< USB Device speed setting 
} ARM_USBD_STATE;


/**
\brief  USB Device SET_ADDRESS Stage
*/
typedef enum _ARM_USBD_SET_ADDRESS_STAGE {
  ARM_USBD_SET_ADDRESS_SETUP,
  ARM_USBD_SET_ADDRESS_STATUS
} ARM_USBD_SET_ADDRESS_STAGE;


/**
\brief  USB Device Status of executed operation
*/
typedef enum _ARM_USBD_STATUS {
  ARM_USBD_OK       =  0,               ///< USBD Status: Operation succeeded
  ARM_USBD_ERROR    =  1                ///< USBD Status: Unspecified error
} ARM_USBD_STATUS;


/**
\brief  USB Device Event
*/
typedef enum _ARM_USBD_EVENT {
  ARM_USBD_EVENT_POWER_ON,              ///< USB Device Power On
  ARM_USBD_EVENT_POWER_OFF,             ///< USB Device Power Off
  ARM_USBD_EVENT_CONNECT,               ///< USB Device connected
  ARM_USBD_EVENT_DISCONNECT,            ///< USB Device disconnected
  ARM_USBD_EVENT_RESET,                 ///< USB Reset occurred
  ARM_USBD_EVENT_HIGH_SPEED,            ///< USB switch to High Speed occurred
  ARM_USBD_EVENT_SUSPEND,               ///< USB Suspend occurred
  ARM_USBD_EVENT_RESUME,                ///< USB Resume occurred
  ARM_USBD_EVENT_REMOTE_WAKEUP          ///< USB Remote wakeup
} ARM_USBD_EVENT;

/**
\brief  USB Endpoint Event
*/
typedef enum _ARM_USBD_EP_EVENT {
  ARM_USBD_EP_EVENT_SETUP,              ///< SETUP Packet
  ARM_USBD_EP_EVENT_OUT,                ///< OUT Packet
  ARM_USBD_EP_EVENT_IN                  ///< IN Packet
} ARM_USBD_EP_EVENT;


#ifndef __DOXYGEN_MW__                  // exclude from middleware documentation

// Function documentation
/**
  \fn          ARM_DRV_VERSION ARM_USBD_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
/**
  \fn          ARM_USBD_CAPABILITIES ARM_USBD_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_USBD_CAPABILITIES
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_Initialize (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                                    ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
  \brief       Initialize USB Device Interface.
  \param[in]   cb_device_event    Pointer to \ref ARM_USBD_SignalDeviceEvent
  \param[in]   cb_endpoint_event  Pointer to \ref ARM_USBD_SignalEndpointEvent
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_Uninitialize (void)
  \brief       De-initialize USB Device Interface.
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_PowerControl (ARM_POWER_STATE state)
  \brief       Control USB Device Interface Power.
  \param[in]   state  Power state
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_DeviceConnect (void)
  \brief       Connect USB Device.
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_DeviceDisconnect (void)
  \brief       Disconnect USB Device.
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          ARM_USBD_STATE ARM_USBD_DeviceGetState (void)
  \brief       Get current USB Device State.
  \return      execution status \ref ARM_USBD_STATE
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_DeviceRemoteWakeup (void)
  \brief       Trigger USB Remote Wakeup.
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_DeviceSetAddress (uint8_t dev_addr,
                                                          ARM_USBD_SET_ADDRESS_STAGE stage)
  \brief       Set USB Device Address.
  \param[in]   dev_addr  Device Address
  \param[in]   stage     Stage in which the function is called
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_DeviceConfigure (bool configure)
  \brief       Configure/unconfigure USB Device.
  \param[in]   configure  Operation
                - \b false Unconfigure
                - \b true  Configure
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_EndpointConfigure (uint8_t ep_addr,
                                                           ARM_USB_ENDPOINT_TYPE ep_type,
                                                           uint16_t ep_max_packet_size)
  \brief       Configure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   ep_type  Endpoint Type
  \param[in]   ep_max_packet_size Endpoint Maximum Packet Size
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_EndpointUnconfigure (uint8_t ep_addr)
  \brief       Unconfigure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_EndpointStall (uint8_t ep_addr, bool stall)
  \brief       Set/Clear Stall for USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   stall  Operation
                - \b false Clear
                - \b true Set
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_EndpointReadStart (uint8_t ep_addr, uint8_t *buf, uint32_t len)
  \brief       Start USB Endpoint Read operation.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[out]  buf  Buffer for data read from Endpoint
  \param[in]   len  Buffer length
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          int32_t ARM_USBD_EndpointRead (uint8_t ep_addr, uint8_t *buf, uint32_t len)
  \brief       Read data from USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[out]  buf  Buffer for data read from Endpoint
  \param[in]   len  Buffer length
  \return      number of data bytes read or execution status
                 - value >= 0: number of data bytes read
                 - value < 0: error occurred, -value is execution status as defined with \ref ARM_USBD_STATUS 
*/
/**
  \fn          int32_t ARM_USBD_EndpointWrite (uint8_t ep_addr, const uint8_t *buf, uint32_t len)
  \brief       Write data to USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   buf  Buffer with data to write to Endpoint
  \param[in]   len  Buffer length
  \return      number of data bytes written or execution status
                 - value >= 0: number of data bytes written
                 - value < 0: error occurred, -value is execution status as defined with \ref ARM_USBD_STATUS 
*/
/**
  \fn          ARM_USBD_STATUS ARM_USBD_EndpointAbort (uint8_t ep_addr)
  \brief       Abort current USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      execution status \ref ARM_USBD_STATUS
*/
/**
  \fn          uint16_t ARM_USBD_GetFrameNumber (void)
  \brief       Get current USB Frame Number.
  \return      Frame Number
*/

/**
  \fn          void ARM_USBD_SignalDeviceEvent (ARM_USBD_EVENT event)
  \brief       Signal USB Device Event.
  \param[in]   event \ref ARM_USBD_EVENT
  \return      none
*/
/**
  \fn          void ARM_USBD_SignalEndpointEvent (uint8_t ep_addr, ARM_USBD_EP_EVENT ep_event)
  \brief       Signal USB Endpoint Event.
               Occurs when Endpoint Read/Write completes.
  \param[in]   ep_addr  Endpoint Address
                ep_addr.0..3: Address
                ep_addr.7:    Direction
  \param[in]   ep_event \ref ARM_USBD_EP_EVENT
  \return      none
*/

typedef void (*ARM_USBD_SignalDeviceEvent_t)   (ARM_USBD_EVENT event);                           ///< Pointer to \ref ARM_USBD_SignalDeviceEvent : Signal USB Device Event.
typedef void (*ARM_USBD_SignalEndpointEvent_t) (uint8_t ep_addr, ARM_USBD_EP_EVENT ep_event);    ///< Pointer to \ref ARM_USBD_SignalEndpointEvent : Signal USB Endpoint Event.

/**
\brief  USB Device Driver Capabilities.
*/
typedef struct _ARM_USBD_CAPABILITIES {
  uint32_t event_power_on      :  1;    ///< Signal Power On event
  uint32_t event_power_off     :  1;    ///< Signal Power Off event
  uint32_t event_connect       :  1;    ///< Signal Connect event
  uint32_t event_disconnect    :  1;    ///< Signal Disconnect event
  uint32_t event_reset         :  1;    ///< Signal Reset event
  uint32_t event_high_speed    :  1;    ///< Signal switch to High-speed event
  uint32_t event_suspend       :  1;    ///< Signal Suspend event
  uint32_t event_resume        :  1;    ///< Signal Resume event
  uint32_t event_remote_wakeup :  1;    ///< Signal Remote Wakeup event
  uint32_t reserved            : 23;    ///< reserved for future use
} ARM_USBD_CAPABILITIES;


/**
\brief  Access structure of the USB Device Driver.
*/
typedef struct _ARM_DRIVER_USBD {
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
} const ARM_DRIVER_USBD;

#endif /* __DOXYGEN_MW__ */

#endif /* __DRIVER_USBD_H */
