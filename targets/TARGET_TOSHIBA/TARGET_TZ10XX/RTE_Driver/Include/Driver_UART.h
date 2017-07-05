/* ----------------------------------------------------------------------
 * Copyright (C) 2013 ARM Limited. All rights reserved.
 *  
 * $Date:        12. Sept 2013
 * $Revision:    V1.10
 *  
 * Project:      UART Driver for Middleware
 * -------------------------------------------------------------------- */

/* History:
 *  Version 1.10
 *    Namespace prefix ARM_ added
 *  Version 1.01
 *    Added events:
 *      ARM_UART_EVENT_TX_EMPTY,     ARM_UART_EVENT_RX_TIMEOUT
 *      ARM_UART_EVENT_TX_THRESHOLD, ARM_UART_EVENT_RX_THRESHOLD
 *    Added functions: SetTxThreshold, SetRxThreshold
 *    Added "rx_timeout_event" to capabilities
 *  Version 1.00
 *    Initial release
 */ 

#ifndef __DRIVER_UART_H
#define __DRIVER_UART_H

#include "Driver_Common.h"

#define ARM_UART_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,10)  /* API version */


/**
\brief  UART Parity
*/
typedef enum _ARM_UART_PARITY {
  ARM_UART_PARITY_NONE,                 ///< No Parity
  ARM_UART_PARITY_ODD,                  ///< Odd Parity
  ARM_UART_PARITY_EVEN,                 ///< Even Parity
  ARM_UART_PARITY_MARK,                 ///< Mark (1) Parity
  ARM_UART_PARITY_SPACE                 ///< Space (0) Parity
} ARM_UART_PARITY;

/**
\brief  UART Stop Bits
*/
typedef enum _ARM_UART_STOP_BITS {
  ARM_UART_STOP_BITS_1,                 ///< 1 Stop bit
  ARM_UART_STOP_BITS_1_5,               ///< 1.5 Stop bits
  ARM_UART_STOP_BITS_2                  ///< 2 Stop bits
} ARM_UART_STOP_BITS;

/**
\brief  UART Flow Control
*/
typedef enum _ARM_UART_FLOW_CONTROL {
  ARM_UART_FLOW_CONTROL_NONE,           ///< No Flow Control
  ARM_UART_FLOW_CONTROL_RTS_CTS,        ///< RTS/CTS Flow Control
  ARM_UART_FLOW_CONTROL_XON_XOFF        ///< XON/XOFF Flow Control
} ARM_UART_FLOW_CONTROL;

/**
\brief  UART Communication Error
*/
typedef struct _ARM_UART_COM_ERROR {
  uint8_t rx_overrun       : 1;         ///< Receive buffer overrun
  uint8_t rx_break         : 1;         ///< Break detected on receive
  uint8_t rx_parity_error  : 1;         ///< Parity error detected on receive
  uint8_t rx_framing_error : 1;         ///< Framing error detected on receive
} ARM_UART_COM_ERROR;

/**
\brief  UART Modem Control
*/
typedef enum _ARM_UART_MODEM_CONTROL {
  ARM_UART_RTS_CLEAR,                   ///< Deactivate RTS
  ARM_UART_RTS_SET,                     ///< Activate RTS
  ARM_UART_DTR_CLEAR,                   ///< Deactivate DTR
  ARM_UART_DTR_SET                      ///< Activate DTR
} ARM_UART_MODEM_CONTROL;

/**
\brief  UART Modem Status
*/
typedef struct _ARM_UART_MODEM_STATUS {
  uint8_t cts : 1;                      ///< CTS state (1 - Active, 0 - Inactive)
  uint8_t dsr : 1;                      ///< DSR state (1 - Active, 0 - Inactive)
  uint8_t dcd : 1;                      ///< DCD state (1 - Active, 0 - Inactive)
  uint8_t ri  : 1;                      ///< RI  state (1 - Active, 0 - Inactive)
} ARM_UART_MODEM_STATUS;

/**
\brief  UART Status of executed operation
*/
typedef enum _ARM_UART_STATUS {
  ARM_UART_OK                 =  0,     ///< Operation succeeded
  ARM_UART_ERROR              =  1,     ///< Unspecified error
  ARM_UART_ERROR_BAUDRATE     =  2,     ///< Specified baudrate not supported
  ARM_UART_ERROR_DATA_BITS    =  3,     ///< Specified number of Data bits not supported
  ARM_UART_ERROR_PARITY       =  4,     ///< Specified Parity not supported
  ARM_UART_ERROR_STOP_BITS    =  5,     ///< Specified Stop bits not supported
  ARM_UART_ERROR_FLOW_CONTROL =  6      ///< Specified Flow Control not supported
} ARM_UART_STATUS;


/**
\brief  UART Event
*/
typedef enum _ARM_UART_EVENT {
  ARM_UART_EVENT_TX_DONE          =  0, ///< Transmit completed
  ARM_UART_EVENT_TX_EMPTY         =  1, ///< Transmit buffer empty
  ARM_UART_EVENT_RX_NOT_EMPTY     =  2, ///< Receive buffer not empty
  ARM_UART_EVENT_RX_OVERRUN       =  3, ///< Receive buffer overrun detected
  ARM_UART_EVENT_RX_THRESHOLD     =  4, ///< Receive buffer threshold reached
  ARM_UART_EVENT_TX_THRESHOLD     =  5, ///< Transmit buffer threshold reached
  ARM_UART_EVENT_RX_TIMEOUT       =  6, ///< Receive character timeout
  ARM_UART_EVENT_RX_BREAK         =  7, ///< Break detected on receive
  ARM_UART_EVENT_RX_PARITY_ERROR  =  8, ///< Parity error detected on receive
  ARM_UART_EVENT_RX_FRAMING_ERROR =  9, ///< Framing error detected on receive
  ARM_UART_EVENT_CTS              = 10, ///< CTS state changed
  ARM_UART_EVENT_DSR              = 11, ///< DSR state changed
  ARM_UART_EVENT_DCD              = 12, ///< DCD state changed
  ARM_UART_EVENT_RI               = 13  ///< RI  state changed
} ARM_UART_EVENT;


// Function documentation
/**
  \fn          ARM_DRV_VERSION ARM_UART_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
/**
  \fn          ARM_UART_CAPABILITIES ARM_UART_GetCapabilities (void)
  \brief       Get driver capabilities
  \return      \ref ARM_UART_CAPABILITIES
*/
/**
  \fn          ARM_UART_STATUS ARM_UART_Initialize (ARM_UART_SignalEvent_t cb_event,
                                                    uint32_t               event_mask)
  \brief       Initialize UART Interface.
  \param[in]   cb_event    Pointer to \ref ARM_UART_SignalEvent
  \param[in]   event_mask  Events that are reported through callbacks
  \return      execution status \ref ARM_UART_STATUS
*/
/**
  \fn          ARM_UART_STATUS ARM_UART_Uninitialize (void)
  \brief       De-initialize UART Interface.
  \return      execution status \ref ARM_UART_STATUS
*/
/**
  \fn          ARM_UART_STATUS ARM_UART_PowerControl (ARM_POWER_STATE state)
  \brief       Control  UART Interface Power.
  \param[in]   state    Power state
  \return      execution status \ref ARM_UART_STATUS
*/
/**
  \fn          ARM_UART_STATUS ARM_UART_Configure (uint32_t              baudrate,
                                                   uint8_t               data_bits,
                                                   ARM_UART_PARITY       parity,
                                                   ARM_UART_STOP_BITS    stop_bits,
                                                   ARM_UART_FLOW_CONTROL flow_control)
  \brief       Configure UART Interface.
  \param[in]   baudrate      Requested baudrate in bits/s
  \param[in]   data_bits     Number of data bits
  \param[in]   parity        \ref ARM_UART_PARITY
  \param[in]   stop_bits     \ref ARM_UART_STOP_BITS
  \param[in]   flow_control  \ref ARM_UART_FLOW_CONTROL
  \return      execution status \ref ARM_UART_STATUS
*/
/**
  \fn          int32_t ARM_UART_WriteData (const uint8_t *data, uint32_t size)
  \brief       Write data to UART transmitter.
  \param[in]   data  Pointer to buffer with data to write to UART transmitter
  \param[in]   size  Data buffer size in bytes
  \return      number of data bytes written or execution status
                 - value >= 0: number of data bytes written
                 - value < 0: error occurred, -value is execution status as defined with \ref ARM_UART_STATUS 
*/
/**
  \fn          int32_t ARM_UART_ReadData (uint8_t *data, uint32_t size)
  \brief       Read data from UART receiver.
  \param[out]  data  Pointer to buffer for data read from UART receiver
  \param[in]   size  Data buffer size in bytes
  \return      number of data bytes read or execution status
                 - value >= 0: number of data bytes read
                 - value < 0: error occurred, -value is execution status as defined with \ref ARM_UART_STATUS 
*/
/**
  \fn          int32_t ARM_UART_DataAvailable (void)
  \brief       Check available data in UART receiver.
  \return      number of data bytes in receive buffer or execution status
                 - value >= 0: number of data bytes in receive buffer
                 - value < 0: error occurred, -value is execution status as defined with \ref ARM_UART_STATUS 
*/
/**
  \fn          bool ARM_UART_TxDone (void)
  \brief       Check if UART transmission is completed.
  \return      transmitter completion status
                 - \b true transmission done
                 - \b false transmission in progress
*/
/**
  \fn          ARM_UART_STATUS ARM_UART_SetTxThreshold (uint32_t level)
  \brief       Set transmit threshold for UART_TX_THRESHOLD event.
  \param[in]   level  Number of character in transmit buffer
  \return      execution status \ref ARM_UART_STATUS
*/
/**
  \fn          ARM_UART_STATUS ARM_UART_SetRxThreshold (uint32_t level)
  \brief       Set receive threshold for UART_RX_THRESHOLD event.
  \param[in]   level  Number of character in receive buffer
  \return      execution status \ref ARM_UART_STATUS
*/
/**
  \fn          ARM_UART_STATUS ARM_UART_FlushTxBuffer (void)
  \brief       Flush UART transmit buffer.
  \return      execution status \ref ARM_UART_STATUS
*/
/**
  \fn          ARM_UART_STATUS ARM_UART_FlushRxBuffer (void)
  \brief       Flush UART receive buffer.
  \return      execution status \ref ARM_UART_STATUS
*/
/**
  \fn          ARM_UART_COM_ERROR ARM_UART_GetComError (void)
  \brief       Get UART Communication errors.
  \return      \ref ARM_UART_COM_ERROR
*/
/**
  \fn          ARM_UART_STATUS ARM_UART_SetComBreak (void)
  \brief       Suspend transmission and put UART Transmission line in a break state.
  \return      execution status \ref ARM_UART_STATUS
*/
/**
  \fn          ARM_UART_STATUS ARM_UART_ClearComBreak (void)
  \brief       Resume transmission and put UART Transmission line in a non-break state.
  \return      execution status \ref ARM_UART_STATUS
*/
/**
  \fn          ARM_UART_STATUS ARM_UART_SetModemControl (ARM_UART_MODEM_CONTROL control)
  \brief       Set UART Modem Control line state.
  \param[in]   control  \ref ARM_UART_MODEM_CONTROL
  \return      execution status \ref ARM_UART_STATUS
*/
/**
  \fn          ARM_UART_MODEM_STATUS ARM_UART_GetModemStatus (void)
  \brief       Get UART Modem Status lines state.
  \return      modem status \ref ARM_UART_MODEM_STATUS
*/
/**
  \fn          void ARM_UART_SignalEvent (ARM_UART_EVENT event)
  \brief       Signal UART Event.
  \param[in]   event  \ref ARM_UART_EVENT
  \return      none
*/

typedef void (*ARM_UART_SignalEvent_t) (ARM_UART_EVENT event);  ///< Pointer to \ref ARM_UART_SignalEvent : Signal UART Event.

/**
\brief  UART Device Driver Capabilities.
*/
typedef struct _ARM_UART_CAPABILITIES {
  uint32_t tx_buffer_size;              ///< Transmit Buffer Size
  uint32_t rx_buffer_size;              ///< Receive Buffer Size
  uint32_t rx_timeout_event :  1;       ///< Signal receive character timeout event: \ref ARM_UART_EVENT_RX_TIMEOUT
  uint32_t cts              :  1;       ///< CTS Line: 0=not available, 1=available
  uint32_t cts_event        :  1;       ///< Signal CTS change event: \ref ARM_UART_EVENT_CTS
  uint32_t dsr              :  1;       ///< DSR Line: 0=not available, 1=available
  uint32_t dsr_event        :  1;       ///< Signal DSR change event: \ref ARM_UART_EVENT_DSR
  uint32_t dcd              :  1;       ///< DCD Line: 0=not available, 1=available
  uint32_t dcd_event        :  1;       ///< Signal DCD change event: \ref ARM_UART_EVENT_DCD
  uint32_t ri               :  1;       ///< RI Line: 0=not available, 1=available
  uint32_t ri_event         :  1;       ///< Signal RI change event: \ref ARM_UART_EVENT_RI
  uint32_t reserved         : 23;       ///< reserved for future use
} ARM_UART_CAPABILITIES;


/**
\brief  Access structure of the UART Driver.
*/
typedef struct _ARM_DRIVER_UART {
  ARM_DRIVER_VERSION       (*GetVersion)      (void);                               ///< Pointer to \ref ARM_UART_GetVersion : Get driver version.
  ARM_UART_CAPABILITIES (*GetCapabilities) (void);                               ///< Pointer to \ref ARM_UART_GetCapabilities : Get driver capabilities.
  ARM_UART_STATUS       (*Initialize)      (ARM_UART_SignalEvent_t cb_event,        
                                            uint32_t               event_mask);  ///< Pointer to \ref ARM_UART_Initialize : Initialize UART Interface.
  ARM_UART_STATUS       (*Uninitialize)    (void);                               ///< Pointer to \ref ARM_UART_Uninitialize : De-initialize UART Interface.
  ARM_UART_STATUS       (*PowerControl)    (ARM_POWER_STATE state);              ///< Pointer to \ref ARM_UART_PowerControl : Control UART Interface Power.
  ARM_UART_STATUS       (*Configure)       (uint32_t              baudrate,            
                                            uint8_t               data_bits,
                                            ARM_UART_PARITY       parity,
                                            ARM_UART_STOP_BITS    stop_bits,
                                            ARM_UART_FLOW_CONTROL flow_control); ///< Pointer to \ref ARM_UART_Configure : Configure UART Interface.
  int32_t               (*WriteData)       (const uint8_t *data, uint32_t size); ///< Pointer to \ref ARM_UART_WriteData : Write data to UART transmitter.
  int32_t               (*ReadData)        (      uint8_t *data, uint32_t size); ///< Pointer to \ref ARM_UART_ReadData : Read data from UART receiver.
  int32_t               (*DataAvailable)   (void);                               ///< Pointer to \ref ARM_UART_DataAvailable : Check available data in UART receiver.
  bool                  (*TxDone)          (void);                               ///< Pointer to \ref ARM_UART_TxDone : Check if UART transmission is completed.
  ARM_UART_STATUS       (*SetTxThreshold)  (uint32_t level);                     ///< Pointer to \ref ARM_UART_SetTxThreshold : Set transmit threshold.
  ARM_UART_STATUS       (*SetRxThreshold)  (uint32_t level);                     ///< Pointer to \ref ARM_UART_SetRxThreshold : Set receive threshold.
  ARM_UART_STATUS       (*FlushTxBuffer)   (void);                               ///< Pointer to \ref ARM_UART_FlushTxBuffer : Flush UART transmit buffer.
  ARM_UART_STATUS       (*FlushRxBuffer)   (void);                               ///< Pointer to \ref ARM_UART_FlushRxBuffer : Flush UART receive buffer.
  ARM_UART_COM_ERROR    (*GetComError)     (void);                               ///< Pointer to \ref ARM_UART_GetComError : Get UART Communication errors.
  ARM_UART_STATUS       (*SetComBreak)     (void);                               ///< Pointer to \ref ARM_UART_SetComBreak : Suspend transmission and put UART Transmission line in a break state.
  ARM_UART_STATUS       (*ClearComBreak)   (void);                               ///< Pointer to \ref ARM_UART_ClearComBreak : Resume transmission and put UART Transmission line in a non-break state.
  ARM_UART_STATUS       (*SetModemControl) (ARM_UART_MODEM_CONTROL control);     ///< Pointer to \ref ARM_UART_SetModemControl : Set UART Modem Control line state.
  ARM_UART_MODEM_STATUS (*GetModemStatus)  (void);                               ///< Pointer to \ref ARM_UART_GetModemStatus : Get UART Modem Status lines state.
} const ARM_DRIVER_UART;

#endif /* __DRIVER_UART_H */
