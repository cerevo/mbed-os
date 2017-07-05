/* ----------------------------------------------------------------------
 * Copyright (C) 2013 ARM Limited. All rights reserved.
 *  
 * $Date:        12. Sept 2013
 * $Revision:    V1.10
 *  
 * Project:      I2C Driver (Master) for Middleware
 * -------------------------------------------------------------------- */

/* History:
 *  Version 1.10
 *    Namespace prefix ARM_ added
 *  Version 1.00
 *    Initial release
 */ 

#ifndef __DRIVER_I2C_H
#define __DRIVER_I2C_H

#define ARM_I2C_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,10)    /* API version */

#include "Driver_Common.h"


/**
\brief I2C Status of executed operation
*/
typedef enum _ARM_I2C_STATUS {
  ARM_I2C_OK                =  0,       ///< I2C Status: Operation succeeded
  ARM_I2C_ERROR             =  1,       ///< I2C Status: Unspecified error
  ARM_I2C_ERROR_UNSUPPORTED =  2,       ///< I2C Status: Operation not supported
  ARM_I2C_ERROR_NO_SLAVE    =  3,       ///< I2C Status: Slave not responding
  ARM_I2C_ERROR_BUS_BUSY    =  4        ///< I2C Status: Communication ongoing on the bus
} ARM_I2C_STATUS;

/**
\brief I2C Bus Speed
*/
typedef enum _ARM_I2C_BUS_SPEED {
  ARM_I2C_BUS_SPEED_LOW,                ///< I2C Bus: Low Speed      ( 10kHz)
  ARM_I2C_BUS_SPEED_STANDARD,           ///< I2C Bus: Standard Speed (100kHz)
  ARM_I2C_BUS_SPEED_FAST,               ///< I2C Bus: Fast Speed     (400kHz)
  ARM_I2C_BUS_SPEED_FAST_PLUS,          ///< I2C Bus: Fast+ Speed    (  1MHz)
  ARM_I2C_BUS_SPEED_HIGH                ///< I2C Bus: High Speed     (3.4MHz)
} ARM_I2C_BUS_SPEED;

#define ARM_I2C_ADDRESS_10BIT   0x80000000  /* 10-bit Address flag */

/**
\brief I2C Event
*/
typedef enum _ARM_I2C_EVENT {
  ARM_I2C_EVENT_RESERVED                ///< reserved for future use
} ARM_I2C_EVENT;



// Function documentation
/**
  \fn          ARM_DRV_VERSION ARM_I2C_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
/**
  \fn          ARM_I2C_CAPABILITIES ARM_I2C_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_I2C_CAPABILITIES
*/
/**
  \fn          ARM_I2C_STATUS ARM_I2C_Initialize (ARM_I2C_SignalEvent_t cb_event)
  \brief       Initialize I2C Interface.
  \param[in]   cb_event  Pointer to \ref ARM_I2C_SignalEvent
  \return      execution status \ref ARM_I2C_STATUS
*/
/**
  \fn          ARM_I2C_STATUS ARM_I2C_Uninitialize (void)
  \brief       De-initialize ARM_I2C Interface.
  \return      execution status \ref ARM_I2C_STATUS
*/
/**
  \fn          ARM_I2C_STATUS ARM_I2C_PowerControl (ARM_POWER_STATE state)
  \brief       Control I2C Interface Power.
  \param[in]   state  Power state
  \return      execution status \ref ARM_I2C_STATUS
*/
/**
  \fn          ARM_I2C_STATUS ARM_I2C_BusSpeed (ARM_I2C_BUS_SPEED speed)
  \brief       Set bus speed for I2C transfers.
  \param[in]   speed  Requested bus speed
  \return      execution status \ref ARM_I2C_STATUS
*/
/**
  \fn          ARM_I2C_STATUS ARM_I2C_BusClear (void)
  \brief       Send nine clock pulses to clear the bus.
  \return      execution status \ref ARM_I2C_STATUS
*/
/**
  \fn          int32_t ARM_I2C_SendData (uint32_t addr, const uint8_t *data, uint32_t size, bool xfer_pending)
  \brief       Send data to I2C Slave.
  \param[in]   addr         Slave address (7-bit or 10-bit)
  \param[in]   data         Pointer to buffer with data to send to I2C slave
  \param[in]   size         Data buffer size in bytes
  \param[in]   xfer_pending Transfer operation is pending - Stop condition will not be generated
  \return      number of data bytes sent and acknowledged or execution status
                 - value >= 0: number of data bytes sent and acknowledged
                 - value < 0: error occurred, -value is execution status as defined with \ref ARM_I2C_STATUS 
*/
/**
  \fn          int32_t ARM_I2C_ReceiveData (uint32_t addr, uint8_t *data, uint32_t size, bool xfer_pending)
  \brief       Receive data from I2C Slave.
  \param[in]   addr  Slave address (7-bit or 10-bit)
  \param[out]  data  Pointer to buffer for data received from I2C slave
  \param[in]   size  Data buffer size in bytes
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \return      number of data bytes received or execution status
                 - value >= 0: number of data bytes received
                 - value < 0: error occurred, -value is execution status as defined with \ref ARM_I2C_STATUS 
*/
/**
  \fn          ARM_I2C_STATUS ARM_I2C_AbortTransfer (void)
  \brief       Abort current I2C transfer.
  \return      execution status \ref ARM_I2C_STATUS
*/
/**
  \fn          void ARM_I2C_SignalEvent (ARM_I2C_EVENT event)
  \brief       Signal I2C Event.
  \param[in]   event \ref ARM_I2C_EVENT
*/

typedef void (*ARM_I2C_SignalEvent_t) (ARM_I2C_EVENT event);  ///< Pointer to \ref ARM_I2C_SignalEvent : Signal I2C Event.

/**
\brief  I2C Driver Capabilities.
*/
typedef struct _ARM_I2C_CAPABILITIES {
  uint32_t reserved : 32;               ///< reserved for future use
} ARM_I2C_CAPABILITIES;


/**
\brief  Access structure of the I2C Driver.
*/
typedef struct _ARM_DRIVER_I2C {
  ARM_DRIVER_VERSION      (*GetVersion)     (void);                                                                 ///< Pointer to \ref ARM_I2C_GetVersion : Get driver version.
  ARM_I2C_CAPABILITIES (*GetCapabilities)(void);                                                                 ///< Pointer to \ref ARM_I2C_GetCapabilities : Get driver capabilities.
  ARM_I2C_STATUS       (*Initialize)     (ARM_I2C_SignalEvent_t cb_event);                                       ///< Pointer to \ref ARM_I2C_Initialize : Initialize I2C Interface.
  ARM_I2C_STATUS       (*Uninitialize)   (void);                                                                 ///< Pointer to \ref ARM_I2C_Uninitialize : De-initialize I2C Interface.
  ARM_I2C_STATUS       (*PowerControl)   (ARM_POWER_STATE state);                                                ///< Pointer to \ref ARM_I2C_PowerControl : Control I2C Interface Power.
  ARM_I2C_STATUS       (*BusSpeed)       (ARM_I2C_BUS_SPEED speed);                                              ///< Pointer to \ref ARM_I2C_BusSpeed : Set bus speed for I2C transfers.
  ARM_I2C_STATUS       (*BusClear)       (void);                                                                 ///< Pointer to \ref ARM_I2C_BusClear : Send nine clock pulses to clear the bus.
  int32_t              (*SendData)       (uint32_t addr, const uint8_t *data, uint32_t size, bool xfer_pending); ///< Pointer to \ref ARM_I2C_SendData : Send data to I2C Slave.
  int32_t              (*ReceiveData)    (uint32_t addr,       uint8_t *data, uint32_t size, bool xfer_pending); ///< Pointer to \ref ARM_I2C_ReceiveData : Receive data from I2C Slave.
  ARM_I2C_STATUS       (*AbortTransfer)  (void);                                                                 ///< Pointer to \ref ARM_I2C_AbortTransfer : Abort current I2C transfer.
} const ARM_DRIVER_I2C;

#endif /* __DRIVER_I2C_H */
