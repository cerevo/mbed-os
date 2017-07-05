/* ----------------------------------------------------------------------
 * Copyright (C) 2013 ARM Limited. All rights reserved.
 *  
 * $Date:        12. Sept 2013
 * $Revision:    V1.10
 *  
 * Project:      SPI Driver (Master) for Middleware
 * -------------------------------------------------------------------- */

/* History:
 *  Version 1.10
 *    Namespace prefix ARM_ added
 *  Version 1.01
 *    Added "send_done_event" to Capabilities
 *  Version 1.00
 *    Initial release
 */ 

#ifndef __DRIVER_SPI_H
#define __DRIVER_SPI_H

#include "Driver_Common.h"

#define ARM_SPI_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,10)   /* API version */


/**
\brief SPI Status of executed operation
*/
typedef enum _ARM_SPI_STATUS {
  ARM_SPI_OK                =  0,       ///< SPI Status: Operation succeeded
  ARM_SPI_ERROR             =  1,       ///< SPI Status: Unspecified error
  ARM_SPI_ERROR_UNSUPPORTED =  2        ///< SPI Status: Operation not supported
} ARM_SPI_STATUS;

/**
\brief SPI Frame Format
*/
typedef enum _ARM_SPI_FRAME_FORMAT {
  ARM_SPI_CPOL0_CPHA0,                  ///< Clock Polarity 0, Clock Phase 0
  ARM_SPI_CPOL0_CPHA1,                  ///< Clock Polarity 0, Clock Phase 1
  ARM_SPI_CPOL1_CPHA0,                  ///< Clock Polarity 1, Clock Phase 0
  ARM_SPI_CPOL1_CPHA1                   ///< Clock Polarity 1, Clock Phase 1
} ARM_SPI_FRAME_FORMAT;

/**
\brief SPI Bit Order
*/
typedef enum _ARM_SPI_BIT_ORDER {
  ARM_SPI_MSB_LSB,                      ///< SPI Bit order from MSB to LSB
  ARM_SPI_LSB_MSB                       ///< SPI Bit order from LSB to MSB
} ARM_SPI_BIT_ORDER;

/**
\brief SPI Slave Select Signal
*/
typedef enum _ARM_SPI_SS_SIGNAL {
  ARM_SPI_SS_INACTIVE,                  ///< SPI Slave Select Signal Inactive
  ARM_SPI_SS_ACTIVE                     ///< SPI Slave Select Signal Active
} ARM_SPI_SS_SIGNAL;

/**
\brief SPI Event
*/
typedef enum _ARM_SPI_EVENT {
  ARM_SPI_EVENT_SEND_DONE,              ///< send data completed
} ARM_SPI_EVENT;


// Function documentation
/**
  \fn          ARM_DRV_VERSION ARM_SPI_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/

/**
  \fn          ARM_SPI_CAPABILITIES ARM_SPI_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_SPI_CAPABILITIES
*/
/**
  \fn          ARM_SPI_STATUS ARM_SPI_Initialize (ARM_SPI_SignalEvent_t cb_event)
  \brief       Initialize SPI Interface.
  \param[in]   cb_event  Pointer to \ref ARM_SPI_SignalEvent
  \return      execution status \ref ARM_SPI_STATUS
*/
/**
  \fn          ARM_SPI_STATUS ARM_SPI_Uninitialize (void)
  \brief       De-initialize SPI Interface.
  \return      execution status \ref ARM_SPI_STATUS
*/
/**
  \fn          ARM_SPI_STATUS ARM_SPI_PowerControl (ARM_POWER_STATE state)
  \brief       Control SPI Interface Power.
  \param[in]   state  Power state
  \return      execution status \ref ARM_SPI_STATUS
*/
/**
  \fn          ARM_SPI_STATUS ARM_SPI_Configure (ARM_SPI_FRAME_FORMAT frame_format,
                                                 ARM_SPI_BIT_ORDER    bit_order)
  \brief       Configure SPI Interface.
  \param[in]   frame_format  \ref ARM_SPI_FRAME_FORMAT
  \param[in]   bit_order     \ref ARM_SPI_BIT_ORDER
  \return      execution status \ref ARM_SPI_STATUS
*/
/**
  \fn          uint32_t ARM_SPI_BusSpeed (uint32_t bps)
  \brief       Set bus speed for SPI transfers.
  \param[in]   bps    Requested speed in bits/s
  \return      Configured bus speed in bits/s
*/
/**
  \fn          ARM_SPI_STATUS ARM_SPI_SlaveSelect (ARM_SPI_SS_SIGNAL ss)
  \brief       Control Slave Select (SS) signal.
  \param[in]   ss     Slave Select status \ref ARM_SPI_SS_SIGNAL
  \return      execution status \ref ARM_SPI_STATUS
*/
/**
  \fn          uint8_t ARM_SPI_TransferByte (uint8_t out)
  \brief       Send and receive one byte via SPI Interface.
  \param[in]   out  Byte to be sent to the SPI data output
  \return      Byte received from the SPI data input
*/
/**
  \fn          ARM_SPI_STATUS ARM_SPI_SendData (const uint8_t *buf, uint32_t len)
  \brief       Send data block to SPI Slave.
  \param[in]   buf  Pointer to the buffer containing data to send
  \param[in]   len  Data buffer length in bytes
  \return      execution status \ref ARM_SPI_STATUS
*/
/**
  \fn          ARM_SPI_STATUS ARM_SPI_ReceiveData (uint8_t *buf, uint32_t len, uint8_t out)
  \brief       Receive data block from SPI Slave.
  \param[out]  buf  Pointer to the buffer for receiving data
  \param[in]   len  Data buffer length in bytes
  \param[in]   out  Byte to be sent repeatedly during receive
  \return      execution status \ref ARM_SPI_STATUS
*/
/**
  \fn          ARM_SPI_STATUS ARM_SPI_AbortTransfer (void)
  \brief       Abort current SPI transfer.
  \return      execution status \ref ARM_SPI_STATUS
*/
/**
  \fn          void ARM_SPI_SignalEvent (ARM_SPI_EVENT event)
  \brief       Signal SPI Event.
  \param[in]   event \ref ARM_SPI_EVENT
  \return      none
*/

typedef void (*ARM_SPI_SignalEvent_t) (ARM_SPI_EVENT event);  ///< Pointer to \ref ARM_SPI_SignalEvent : Signal SPI Event.

/**
\brief  SPI Driver Capabilities.
*/
typedef struct _ARM_SPI_CAPABILITIES {
  uint32_t send_done_event :  1;        ///< Signal send data completed: \ref ARM_SPI_EVENT_SEND_DONE
  uint32_t reserved        : 31;        ///< reserved for future use
} ARM_SPI_CAPABILITIES;


/**
\brief  Access structure of the SPI Driver.
*/
typedef struct _ARM_DRIVER_SPI {
  ARM_DRIVER_VERSION      (*GetVersion)     (void);                              ///< Pointer to \ref ARM_SPI_GetVersion : Get driver version.
  ARM_SPI_CAPABILITIES (*GetCapabilities)(void);                              ///< Pointer to \ref ARM_SPI_GetCapabilities : Get driver capabilities.
  ARM_SPI_STATUS       (*Initialize)     (ARM_SPI_SignalEvent_t cb_event);    ///< Pointer to \ref ARM_SPI_Initialize : Initialize SPI Interface.
  ARM_SPI_STATUS       (*Uninitialize)   (void);                              ///< Pointer to \ref ARM_SPI_Uninitialize : De-initialize SPI Interface.
  ARM_SPI_STATUS       (*PowerControl)   (ARM_POWER_STATE state);             ///< Pointer to \ref ARM_SPI_PowerControl : Control SPI Interface Power.
  ARM_SPI_STATUS       (*Configure)      (ARM_SPI_FRAME_FORMAT frame_format,      
                                          ARM_SPI_BIT_ORDER    bit_order);    ///< Pointer to \ref ARM_SPI_Configure : Configure SPI Interface.
  uint32_t             (*BusSpeed)       (uint32_t bps);                      ///< Pointer to \ref ARM_SPI_BusSpeed : Set bus speed for SPI transfers.
  ARM_SPI_STATUS       (*SlaveSelect)    (ARM_SPI_SS_SIGNAL ss);              ///< Pointer to \ref ARM_SPI_SlaveSelect : Control Slave Select signal.
  uint8_t              (*TransferByte)   (uint8_t out);                       ///< Pointer to \ref ARM_SPI_TransferByte : Send and receive one byte via SPI Interface.
  ARM_SPI_STATUS       (*SendData)       (const uint8_t *buf, uint32_t len);  ///< Pointer to \ref ARM_SPI_SendData : Send data block to SPI Slave.
  ARM_SPI_STATUS       (*ReceiveData)    (      uint8_t *buf, uint32_t len,   
                                                uint8_t  out);                ///< Pointer to \ref ARM_SPI_ReceiveData : Receive data block from SPI Slave.
  ARM_SPI_STATUS       (*AbortTransfer)  (void);                              ///< Pointer to \ref ARM_SPI_AbortTransfer : Abort current SPI transfer.
} const ARM_DRIVER_SPI;

#endif /* __DRIVER_SPI_H */
