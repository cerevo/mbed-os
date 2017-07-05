/**
 * @file twic_interface_cb.h
 * @brief a header file for TZ10xx TWiC for Bluetooth 4.0/4.1 Smart
 * @version V1.2.0
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef _TWIC_INTERFACE_CB_H_
#define _TWIC_INTERFACE_CB_H_

/*
 * LIST OF ERROR CODES [1]
 *
 * Error Code Name
 * 0x01       Unknown HCI Command
 * 0x02       Unknown Connection Identifier
 * 0x03       Hardware Failure
 * 0x04       Page Timeout
 * 0x05       Authentication Failure
 * 0x06       PIN or Key Missing
 * 0x07       Memory Capacity Exceeded
 * 0x08       Connection Timeout
 * 0x09       Connection Limit Exceeded
 * 0x0A       Synchronous Connection Limit To A Device Exceeded
 * 0x0B       ACL Connection Already Exists
 * 0x0C       Command Disallowed
 * 0x0D       Connection Rejected due to Limited Resources
 * 0x0E       Connection Rejected Due To Security Reasons
 * 0x0F       Connection Rejected due to Unacceptable BD_ADDR
 * 0x10       Connection Accept Timeout Exceeded
 * 0x11       Unsupported Feature or Parameter Value
 * 0x12       Invalid HCI Command Parameters
 * 0x13       Remote User Terminated Connection
 * 0x14       Remote Device Terminated Connection due to Low Resources
 * 0x15       Remote Device Terminated Connection due to Power Off
 * 0x16       Connection Terminated By Local Host
 * 0x17       Repeated Attempts
 * 0x18       Pairing Not Allowed
 * 0x19       Unknown LMP PDU
 * 0x1A       Unsupported Remote Feature / Unsupported LMP Feature
 * 0x1B       SCO Offset Rejected
 * 0x1C       SCO Interval Rejected
 * 0x1D       SCO Air Mode Rejected
 * 0x1E       Invalid LMP Parameters
 * 0x1F       Unspecified Error
 * 0x20       Unsupported LMP Parameter Value
 * 0x21       Role Change Not Allowed
 * 0x22       LMP Response Timeout / LL Response Timeout
 * 0x23       LMP Error Transaction Collision
 * 0x24       LMP PDU Not Allowed
 * 0x25       Encryption Mode Not Acceptable
 * 0x26       Link Key cannot be Changed
 * 0x27       Requested QoS Not Supported
 * 0x28       Instant Passed
 * 0x29       Pairing With Unit Key Not Supported
 * 0x2A       Different Transaction Collision
 * 0x2B       Reserved
 * 0x2C       QoS Unacceptable Parameter
 * 0x2D       QoS Rejected
 * 0x2E       Channel Classification Not Supported
 * 0x2F       Insufficient Security
 * 0x30       Parameter Out Of Mandatory Range
 * 0x31       Reserved
 * 0x32       Role Switch Pending
 * 0x33       Reserved
 * 0x34       Reserved Slot Violation
 * 0x35       Role Switch Failed
 * 0x36       Extended Inquiry Response Too Large
 * 0x37       Secure Simple Pairing Not Supported By Host.
 * 0x38       Host Busy - Pairing
 * 0x39       Connection Rejected due to No Suitable Channel Found
 * 0x3A       Controller Busy
 * 0x3B       Unacceptable Connection Interval
 * 0x3C       Directed Advertising Timeout
 * 0x3D       Connection Terminated due to MIC Failure
 * 0x3E       Connection Failed to be Established
 * 0x3F       MAC Connection Failed
 */

/* LIST OF ERROR CODES [2]
 *
 * Error Code Name
 * 0x01       Invalid Handle
 * 0x02       Read Not permitted
 * 0x03       Write Not permitted
 * 0x04       Invalid Packet Data Unit
 * 0x05       Insufficient Authentication
 * 0x06       Request Not supported
 * 0x07       Invalid Offset
 * 0x08       Insufficient Authorization
 * 0x09       Prepare Queue Full
 * 0x0A       Look Up attribute Not Found
 * 0x0B       Attribute Not Long
 * 0x0C       Insufficient Encryption Key Size
 * 0x0D       Invalid Attribute Value Length
 * 0x0E       Unlikely Error
 * 0x0F       Insufficient Encryption
 * 0x10       Unsupported Group Type
 * 0x11       Insufficient Resources
 * 0xFE       Application Error
 * 0xA1       GATT Not Initialized
 * 0xA3       Invalid Connection_Handle
 * 0xA4       Disconnect Error
 * 0xA5       Remote Data Length Error (Data from remote greater then MTU Size)
 * 0xA6       Invalid Parameter Length
 * 0xA8       Invalid Request (Error in Opcode)
 * 0xA9       Initialization Already Done
 * 0xAA       Timeout Occurred
 * 0xAB       Invalid Accept Request (Error In Parameter Length)
 * 0xAC       Execute Write Cancelled by Remote Client
 * 0xB0       Database Memory Insufficient(Memory Not Available)
 * 0xB1       Exceeds Maximum Characteristic Elements.
              (No more char elements can be accommodated in database
              under the specified service or Characteristic declaration)
 * 0xB2       Post Message Failure
 * 0xB3       Buffer Full Error
              (Overflow of Internal Queued Buffers to Process Request)
 */

/* LIST OF ERROR CODES [3]
 *
 * Error Code Name
 * 0x00       Success
 * 0x01       Abnormalities in parameter
 * 0x02       LE Device already Initialized
 * 0x03       LE Device Not Initialized
 * 0x04       LE MNG Command in Progress
 * 0x05       NO LE ACL Link
 * 0x06       LE Device Role Slave
 * 0x07       Memory Capacity Exceeded
 * 0x08       LE Maximum No. of devices already Connected
 * 0x10       GATT Request in Progress
 * 0x11       GATT Error: Link is Not Authenticated
 * 0x12       GATT Error: Link is encrypted
 * 0x13       GATT Error: Notification not enabled
 * 0x14       GATT Error: Indication not enabled
 * 0x15       GATT Error: MTU Request in progress
 * 0x16       Device Already Connected
 * 0xA6       GATT Error: Invalid Parameter Length
 */

/* LIST OF ERROR CODES [4]
 *
 * Error Code Name
 * 0x01       Passkey Entry Failed
 * 0x02       OOB Not Available
 * 0x03       Authentication Requirements Not Met
 * 0x04       Confirm Value Failed
 * 0x05       Pairing Not Supported
 * 0x06       Encryption Key Size insufficient
 * 0x07       Command Not Supported
 * 0x08       Unspecified Reason
 * 0x09       Repeated Attempts
 * 0x0A       Invalid Parameters
 * 0xC0       SMP Time Out
 * 0xC1       SMP timeout has happened previously and no further SMP commands
 *            shall be sent over the L2CAP Security Manager Channel till
 *            the reconnection happens.
 * 0x81       Device Not Initialized
 * 0x82       Request in Progress
 * 0x84       Disconnect error
 * 0x86       Invalid parameter Length Error
 * 0x89       Initialization Already Done
 * 0x92       SMP Command In Progress
 * 0x93       CSRK Not Available
 * 0x94       Data Signing Error
 * 0x95       Signed Data Verification Error
 * 0xC2       No Response From Remote
 * 0xC3       No Response From Host
 * 0xC4       Host Rejected Pin Key
 */

/*
 * Call Backs
 *
 */

typedef TZ1K_PACKED_HDR struct {
  uint8_t status;               /* LIST OF ERROR CODES [1] */
  uint8_t role;                 /* 0x00, 0x01: Connection is master,
                                 * Connection is slave */
  uint8_t peer_address_type;    /* 0x00,0x01: Public Device
                                 * Address, Random Device Address */
  twicBdaddr_t peer;            /* Public Device Address or Random Device
                                 * Address of the device to be connected. */
  uint16_t conn_interval;       /* N = 0x0006 to 0x0C80 = Time is N * 1.25
                                 * msec. Time Range: 7.5 msec to 4000
                                 * msec. */
  uint16_t slave_latency;        /* 0x0000 to 0x03E8 = Slave latency for the
                                 * connection in number of connection
                                 * events. */
  uint16_t supervision_timeout; /* Range: 0x0006 to 0x000A Time = N *
                                   10 msec Time Range: 100 msec to 32 msec. */
  uint8_t master_clock_accuracy;
} TZ1K_PACKED_FTR twicConnectionComplete_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t status;               /* LIST OF ERROR CODES [1] */
  uint16_t conn_interval;       /* Range: 0x0006 to 0x0C80 Time = N * 1.25
                                   msec Time Range: 7.5 msec to 4000 msec. */
  uint16_t conn_latency;        /* Range: 0x0006 to 0x0C80 Time = N * 1.25
                                   msec Time Range: 7.5 msec to 4000 msec. */
  uint16_t supervision_timeout; /* Range: 0x0006 to 0x000A Time = N *
                                   10 msec Time Range: 100 msec to 32 msec. */
} TZ1K_PACKED_FTR twicConnectionUpdate_t;

#if defined(TWIC_BLE_HWIP_V41)
typedef TZ1K_PACKED_HDR struct {
  uint8_t status;               /* LIST OF ERROR CODES [1] */
  uint16_t conn_interval_min;   /* Minimum value of the connection interval.
                                   Range: 0x0006 to 0x0C80
                                   Time = N * 1.25 ms
                                   Time Range: 7.5 msec to 4 seconds. */
  uint16_t conn_interval_max;   /* Maximum value of the connection interval.
                                   Range: 0x0006 to 0x0C80
                                   Time = N * 1.25 ms
                                   Time Range: 7.5 msec to 4 seconds. */
  uint16_t conn_latency;        /* Maximum allowed slave latency for the
                                   connection specified as the
                                   number of connection events.
                                   Range: 0x0000 to 0x01F3 (499). */
  uint16_t supervision_timeout; /* Supervision timeout for the connection.
                                   Range: 0x000A to 0x0C80
                                   Time = N * 10 ms
                                   Time Range: 100 ms to 32 seconds. */
} TZ1K_PACKED_FTR twicConnectionParameter_t;
#endif

typedef TZ1K_PACKED_HDR struct {
  uint8_t status; /* Result code. */
  uint8_t reason; /* The reason for terminating the connection. */
  /* Please refer to the "LIST OF ERROR CODES [1]" for the both result
   * code and reason. */
} TZ1K_PACKED_FTR twicDisconnection_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t status; /* 0x00, 0xAA, 0x0E : Success, Remote Response Time
                   * Out, Unlikely Error */
  uint16_t negotiated_mtu_size;
} TZ1K_PACKED_FTR twicMtuExchangeResult_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t value[TWIC_CONTROLLER_TXD_MAX];
} TZ1K_PACKED_FTR twicAttValue_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t eidxs[TWIC_CONTROLLER_TXD_MAX];
} TZ1K_PACKED_FTR twicAttEid_t;

typedef struct {
  void (*mtu_exchange_demand)(
    const void * const connif, const uint16_t client_rx_mtu_size);
  void (*mtu_exchange_result)(
    const void * const connif, const twicMtuExchangeResult_t *const resp);
  void (*notification_sent)(const void * const connif);
  void (*indication_confirmation)(
    const void * const connif, const uint8_t status);
  void (*queued_writes_complete)(
    const void * const connif, const uint8_t status);
  /* descriptor */
  void (*char_desp_writein_demand)(
    const void * const connif, const twicAttValue_t *const resp,
    const uint8_t eidx);
  void (*char_desp_readout_demand)(
    const void * const connif, const uint8_t eidx);
  void (*long_char_desp_readout_demand)(
    const void * const connif, const uint16_t offset, const uint8_t eidx);
  void (*long_char_desp_prepare_writein_demand)(
    const void * const connif, const uint16_t offset, const uint8_t eidx);
  void (*char_desp_exec_writein_demand)(
    const void * const connif, const twicAttValue_t *const resp,
    const uint8_t eidx);
  /* characteristics */
  void (*char_val_readout_demand)(
    const void * const connif, const uint8_t eidx);
  void (*char_val_multi_readout_demand)(
    const void * const connif, const twicAttEid_t *const resp);
  void (*char_val_writein_post)(
    const void * const connif, const twicAttValue_t *const resp,
    const uint8_t eidx);
  void (*char_val_writein_demand)(
    const void * const connif, const twicAttValue_t *const resp,
    const uint8_t eidx);
  void (*long_char_val_readout_demand)(
    const void * const connif, const uint16_t offset, const uint8_t eidx);
  void (*long_char_val_prepare_writein_demand)(
    const void * const connif, const uint16_t offset, const uint8_t eidx);  
  void (*char_val_exec_writein_demand)(
    const void * const connif, const twicAttValue_t *const resp,
    const uint8_t eidx);
} twicIfLeServerCb_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t number_of_char_values;
  TZ1K_PACKED_HDR struct {
    uint8_t status;
    uint16_t handle;
  } TZ1K_PACKED_FTR char_value[TWIC_RESP_D24E_TOTAL_HANDLES];
} TZ1K_PACKED_FTR twicReliableWriteinResp_t;

/* For the status, please refer to "LIST OF ERROR CODES [2]" */
typedef struct {
  /* connection */
  void (*mtu_exchange_result)(
    const void * const connif, const twicMtuExchangeResult_t *const resp);
  /* twicIfLeGattClientDiscoverPrimaryService */
  void (*primary_service)(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint16_t end_group_handle, const uint64_t uuid_lsb,
    const uint64_t uuid_msb, const uint8_t uuid_len);
  /* twicIfLeGattClientDiscoverPrimaryServiceByServiceUuid */
  void (*primary_service_by_uuid)(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint16_t end_group_handle);
  /* twicIfLeGattClientFindIncludedService */
  void (*included_service)(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint16_t included_service_attribute_handle,
    const uint16_t included_service_end_group_handle,
    const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
  /* twicIfLeGattClientDiscoverAllCharacteristics */
  void (*all_char_of_service)(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint8_t char_properties, const uint16_t char_value_handle,
    const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
  /* twicIfLeGattClientDiscoverCharacteristicsByUuid */
  void (*char_by_uuid)(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint8_t char_properties, const uint16_t char_value_handle,
    const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
  /* twicIfLeGattClientDiscoverAllDescriptors */
  void (*all_char_descriptors)(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
  /* twicIfLeGattClientReadCharacteristicValue */
  void (*char_value_readout)(
    const void * const connif, const uint8_t status,
    const twicAttValue_t *const resp);
  /* twicIfLeGattClientReadUsingCharacteristicUuid */
  void (*char_value_readout_using_uuid)(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t char_value_handle,
    const twicAttValue_t *const resp);
  /* twicIfLeGattClientWriteCharacteristicValue */
  void (*char_value_writein_response)(
    const void * const connif, const uint8_t status);
  /* twicIfLeGattClientReadCharacteristicDescriptor */
  void (*char_desp_readout)(
    const void * const connif, const uint8_t status,
    const twicAttValue_t *const resp);
  /* twicIfLeGattClientWriteCharacteristicDescriptor */
  void (*char_desp_writein_response)(
    const void * const connif, const uint8_t status);
  /* Notification received */
  void (*notification_received)(
    const void * const connif, const uint8_t status,
    const uint16_t char_value_handle, const twicAttValue_t *const resp);
  /* Indication recieved. Needs to call
   * twicIfLeGattClientIndicationConfirmationResponse */
  void (*indication_received)(
    const void * const connif, const uint8_t status,
    const uint16_t char_value_handle, const twicAttValue_t *const resp);
  /* twicIfLeGattClientWriteWithoutResponse or
   * twicIfLeGattClientSignedWriteWithoutResponse */
  void (*char_vale_writein_started)(
    const void * const connif, const uint8_t status);
  /* twicIfLeGattClientReadMultipleCharValues */
  void (*multiple_char_values_readout)(
    const void * const connif, const uint8_t status,
    const twicAttValue_t *const resp);
  /* twicIfLeGattClientReliableWrite */
  void (*reliable_writein_confirmation)(
    const uint8_t status, const twicReliableWriteinResp_t *const resp);
  /* twicIfLeGattClientReadLongCharacteristicValue */
  void (*long_char_value_readout)(
    const void * const connif, const uint8_t status, const bool next,
    const twicAttValue_t *const resp);
  /* twicIfLeGattClientReadLongCharacteristicDescriptor */
  void (*long_char_desp_readout)(
    const void * const connif, const uint8_t status, const bool next,
    const twicAttValue_t *const resp);
  /* twicIfLeGattClientWriteLongCharacteristicValue */
  void (*long_char_value_writein_response)(
    const void * const connif, const uint8_t status);
  /* twicIfLeGattClientWriteLongCharacteristicDescriptor */
  void (*long_char_desp_writein_response)(
    const void * const connif, const uint8_t status);
} twicIfLeClientCb_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t io_capability;
  bool oob_data_present;
  bool auth_req_bonding;
  bool auth_req_mitm_protection;
  uint8_t max_enc_key_size;
  bool init_key_dist_enckey;
  bool init_key_dist_idkey;
  bool init_key_dist_sign;
  bool resp_key_dist_enckey;
  bool resp_key_dist_idkey;
  bool resp_key_dist_sign;
} TZ1K_PACKED_FTR twicPairingFeature_t;

typedef struct {
  /* It is invoked when Slave responds to Master with its
   * Input Output capabilities and Authentication information.
   * The resp must be ignored when status is not success(0). */
  void (*pairing_response)(
    const void * const connif, const uint8_t status,
    const twicPairingFeature_t *const resp);
  /* Slave Device requests for pairing.  If the Application accepts
     the Slave Security Request, the Master will initiate the pairing
     process using the API twicIfLeSmMasPairingRequest. If the
     Application rejects the Slave Security Request, the Master will
     issue the API twicIfLeSmMasSecurityReject. */
  void (*security_request)(
    const void * const connif, const bool bonded_device,
    const bool auth_req_bonding, const bool auth_req_mitm_protection);
  void (*pairing_failed)(
    const void * const connif, const twicSmReasonCode_t reason);
  void (*stk_generation_method)(
    const void * const connif, const uint8_t status,
    const twicStkGenMethod_t method);
  void (*input_passkey)(const void * const connif);
  void (*display_passkey)(const void * const connif);
  void (*stk_generated)(const void * const connif, const twicStk_t *const stk);
  void (*encryption_info)(
    const void * const connif, const twicLtk_t *const remote_ltk);
  void (*master_identification)(
    const void * const connif, const twicEdiv_t *const remote_ediv,
    const twicRand_t *const remote_rand);
  void (*encryption_info_sent)(
    const void * const connif, const twicLtk_t *const local_ltk);
  void (*master_identification_sent)(
    const void * const connif, const twicEdiv_t *const local_ediv,
    const twicRand_t *const local_rand);
  void (*encryption_change)(
    const void * const connif, const twicSmReasonCode_t reason,
    const uint8_t key_type, const bool encryption_enable,
    const uint8_t encryption_key_size);
  void (*encryption_key_refresh_complete)(
    const void * const connif, const twicSmReasonCode_t reason,
    const uint8_t key_type, const uint8_t encryption_key_size);
  void (*pairing_complete)(
    const void * const connif, const uint8_t status,
    const twicAuthInfo_t bits);
  void (*identity_information)(
    const void * const connif, const twicIrk_t *const remote_irk);
  void (*identity_address_information)(
    const void * const connif, const bool remote_address_type_random,
    const twicBdaddr_t *const remote_identity);
  void (*signing_information)(
    const void * const connif, const twicCsrk_t *const remote_csrk);
  void (*identity_information_sent)(
    const void * const connif, const twicIrk_t *const local_irk);
  void (*identity_address_information_sent)(
    const void * const connif, const bool local_address_type_random,
    const twicBdaddr_t *const local_identity);
  void (*signing_information_sent)(
    const void * const connif, const twicCsrk_t *const local_csrk);
  void (*oob_information)(const void * const connif);
  void (*store_bonding_information)(
    const void * const connif, const bool remote_address_type_random,
    const twicBdaddr_t *const remote_identity, const bool erase);
  void (*inquiry_bonding_information)(
    const void * const connif, const bool address_type_random,
    const twicBdaddr_t *const remote_identity);
  void (*bonding_state)(
    const void * const connif, const twicSmReasonCode_t reason);
} twicIfLeSmpICb_t;

typedef struct {
  /* twicIfLeSmSlvSecurityRequest */
  void (*pairing_demand)(
    const void * const connif, const twicPairingFeature_t *const resp);
  /* When pairing_demand is received, invokes twicIfLeSmSlvPairingConfirm */
  void (*pairing_acceptance_sent)(
    const void * const connif, const uint8_t status);
  void (*pairing_failed)(
    const void * const connif, const twicSmReasonCode_t reason);
  void (*stk_generation_method)(
    const void * const connif, const uint8_t status,
    const twicStkGenMethod_t method);
  /* This event is generated by the Slave Device to the application to
   * request to enter the PassKey used in the "PassKey Entry" Pairing
   * method. This event is generated as part of pairing process. In
   * response to this event, the application will generate the key write
   * request "twicIfLeSmSlvKbPasskeyEntryReply" to input the Passkey. */
  void (*input_passkey)(const void * const connif);
  /* This event is generated by the Slave Device to its application to
   * input the passkey displayed on the LE Device. This PassKey is
   * used in the "PassKey Entry" Pairing method. This event is
   * generated as part of pairing process. In response to this event,
   * the application will generate the key write request
   * "twicIfLeSmSlvDpPasskeyEntryReply" to input the Displayed Key. */
  void (*display_passkey)(const void * const connif);
  void (*stk_generated)(const void * const connif, const twicStk_t *const stk);
  void (*encryption_info)(
    const void * const connif, const twicLtk_t *const remote_ltk);
  void (*master_identification)(
    const void * const connif, const twicEdiv_t *const remote_ediv,
    const twicRand_t *const remote_rand);
  void (*encryption_info_sent)(
    const void * const connif, const twicLtk_t *const local_ltk);
  void (*master_identification_sent)(
    const void * const connif, const twicEdiv_t *const local_ediv,
    const twicRand_t *const local_rand);
  void (*encryption_change)(
    const void * const connif, const twicSmReasonCode_t reason,
    const uint8_t key_type, const bool encryption_enable,
    const uint8_t encryption_key_size);
  void (*encryption_key_refresh_complete)(
    const void * const connif, twicSmReasonCode_t reason,
    const uint8_t key_type, const uint8_t encryption_key_size);
  void (*pairing_complete)(
    const void * const connif, const uint8_t status,
    const twicAuthInfo_t bits);
/* It has been sent the Long Term Key Request Reply to the Master
 * Device on receiving the LE Long Term Key Request Event. */
  void (*stk_session_request_reply_event)(
    const void * const connif, const uint8_t status,
    const twicStk_t *const local_stk);
/* It has been sent the Long Term Key Request Reply to the Master
 * Device on receiving the LE Long Term Key Request Event. */
  void (*ltk_session_request_reply_event)(
    const void * const connif, const uint8_t status,
    const twicLtk_t *const local_ltk);
  void (*identity_information)(
    const void * const connif, const twicIrk_t *const remote_irk);
  void (*identity_address_information)(
    const void * const connif, const bool remote_address_type_random,
    const twicBdaddr_t *const remote_identity);
  void (*signing_information)(
    const void * const connif, const twicCsrk_t *const remote_csrk);
  void (*identity_information_sent)(
    const void * const connif, const twicIrk_t *const local_irk);
  void (*identity_address_information_sent)(
    const void * const connif, const bool local_address_type_random,
    const twicBdaddr_t *const local_identity);
  void (*signing_information_sent)(
    const void * const connif, const twicCsrk_t *const local_csrk);
  void (*oob_information)(const void * const connif);
  void (*store_bonding_information)(
    const void * const connif, const bool remote_address_type_random,
    const twicBdaddr_t *const remote_identity, const bool erase);
  void (*inquiry_bonding_information)(
    const void * const connif, const bool address_type_random,
    const twicBdaddr_t *const remote_identity);
  void (*bonding_state)(
    const void * const connif, const twicSmReasonCode_t reason);
} twicIfLeSmpRCb_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t status; /* LIST OF ERROR CODES [1] */
  uint8_t le_features[8]; /* CoreV4.0 [Vol 6] Part B, Section 4.6. */
} TZ1K_PACKED_FTR twicRemoteUsedFeatures_t;

/* BLUETOOTH SPECIFICATION Version 4.0 [Vol 2] */
typedef TZ1K_PACKED_HDR struct {
  uint8_t status; /* LIST OF ERROR CODES [1] */
  uint8_t version; /* 7.7.12 Read Remote Version Information Complete Event. */
  uint16_t manufacturer_name;
  uint16_t sub_version;
} TZ1K_PACKED_FTR twicRemoteVersion_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t num_reports; /* i = {0x01,...,0x19 (max)} */
  TZ1K_PACKED_HDR struct {
    uint8_t adv_event_type;
    bool address_type_random;
    twicBdaddr_t bd;
    uint8_t rssi;
    uint8_t length;
    uint8_t data[31];
  } TZ1K_PACKED_FTR reports[TWIC_NUM_REPORTS_ROM5];
} TZ1K_PACKED_FTR twicAdvReport_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t status; /* LIST OF ERROR CODES [1] */  
  twicBdaddr_t bd;
  twicIrk_t irk;
} TZ1K_PACKED_FTR twicPrivacy_t;

typedef struct {
  void (*connection_complete)(const void * const connif, const bool central,
                              const twicConnectionComplete_t *const resp);
  void (*connection_update)(const void * const connif, const bool central,
                            const twicConnectionUpdate_t *const resp);
#if defined(TWIC_BLE_HWIP_V41) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
  /* BLUETOOTH SPECIFICATION Version 4.1 [Vol 6]
     Figure 6.17: Slave-initiated Connection Parameters Request procedure
     slave requests change in LE connection parameters,
     master's Host accepts.
     Host Controller Interface Functional Specification.
     7.8.31 LE Remote Connection Parameter Request Reply Command.
     7.8.32 LE Remote Connection Parameter Request Negative Reply Command.
  */
  void (*connection_parameter)(const void * const connif, const bool central,
                               const twicConnectionParameter_t *const request);
#endif  
  void (*disconnection)(const void * const connif, const bool central,
                        const twicDisconnection_t *const resp);
  void (*isr_downstream_request)(void);
  void (*l2cap_connection_update)(const void * const connif,
                                  const bool central, const uint8_t status);
  void (*reject_l2cap_connection_update)(const void * const connif,
                                         const bool central,
                                         const uint8_t status);
  void (*remote_used_features)(const void * const connif,
                               const twicRemoteUsedFeatures_t *const resp);
  void (*remote_version)(const void * const connif,
                         const twicRemoteVersion_t *const resp);
  void (*resolvable_privacy)(const void * const connif,
                             const twicPrivacy_t *const resp);
  void (*resolved_privacy)(const void * const connif,
                           const twicPrivacy_t *const resp);
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
  /* BLUETOOTH SPECIFICATION Version 4.1 [Vol 2]
     Host Controller Interface Functional Specification.
     7.8.22 LE Encrypt Command */
  void (*encrypt)(const void * const connif,
                  const uint8_t *const encrypted_data);
#endif  
  void (*read_result_rssi)(const void * const connif, const uint8_t rssi);
  void (*read_result_tx_power)(const void * const connif,
                               const uint8_t tx_power);
  void (*isr_wakeup_request)(void);
  void (*adv_report)(const void * const connif,
                     const twicAdvReport_t *const resp);
} twicIfLeCb_t;


#endif /* _TWIC_INTERFACE_CB_H_ */
