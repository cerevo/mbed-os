/* mbed Microcontroller Library
 * Copyright 2017, Cerevo Inc. 
 * All rights reserved.
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
#ifndef _BTLE_LOCAL_H_
#define _BTLE_LOCAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "twic_interface.h"

/* Local definition */
#define BTLE_INITIATOR               (0)                   /* Initiator */
#define BTLE_RESPONDER               (1)                   /* Responder */
#define BTLE_RETRY                   (1000)             /* internal retry message */
/* Common ble callback status, some time status of ble to response need be other value */
#define BTLE_CALLBACK_STATUS_OK         (0x00)             /* Callback Status Is Accepted */
#define BTLE_CALLBACK_STATUS_REJECT     (0x01)             /* Callback Status Is Not Accepted */
/* General GAP and GATT UUID */
#define BTLE_GAP_SERVICE_UUID                               (0x1800)
#define BTLE_GAP_CHAR_DEVICE_NAME_UUID                      (0x2A00)
#define BTLE_GAP_CHAR_APPEARANCE_UUID                       (0x2A01)
#define BTLE_GAP_CHAR_PERIPHERAL_PREF_CONN_PARAM_UUID       (0x2A04)
#define BTLE_CHAR_CLIENT_CONFIG_DESC_UUID                   (0x2902)
#define BTLE_GATT_SERVICE_UUID                              (0x1801)
#define BTLE_GATT_CHAR_SERVICE_CHANGE_UUID                  (0x2A05)

typedef struct _BTLE_ConnectionInfo {
    uint64_t peer_addr;
    uint16_t handle;
} BTLE_ConnectionInfo;
/**
 * @brief Enumeration for recoginize callback
 */
typedef enum _BTLECallbackId_t {
    /* common callbacks */
    BTLE_CB_ISR_DOWNSTREAM_REQUEST,
    BTLE_CB_ISR_WAKEUP_REQUEST,
    BTLE_CB_CONNECTION_COMPLETE,
    BTLE_CB_CONNECTION_UPDATE,
    BLELIB_SM_CB_CONNECTION_PARAMETER,
    BTLE_CB_DISCONNECT,
    BTLE_CB_ADVERTISE_REPORT,

    /* server callbacks */
    BTLE_CB_MTU_EXCHANGE_DEMAND,
    BTLE_CB_MTU_EXCHANGE_RESULT,
    BTLE_CB_NOTIFICATION_SENT,
    BTLE_CB_INDICATION_CONFIRM,
    BTLE_CB_QUEUED_WRITE_COMPLETE,
    BTLE_CB_READ_RESULT_RESSI,
    BTLE_CB_READ_RESULT_TX_POWER,
    BTLE_CB_CHAR_DESP_WRITEIN_DEMAND,
    BTLE_CB_CHAR_DESP_READOUT_DEMAND,
    BTLE_CB_LONG_CHAR_DESP_READOUT_DEMAND,
    BTLE_CB_LONG_CHAR_DESP_PREPARE_WRITEIN_DEMAND,
    BTLE_CB_CHAR_DESP_EXEC_WRITEIN_DEMAND,
    BTLE_CB_CHAR_VAL_READOUT_DEMAND,
    BTLE_CB_CHAR_VAL_MULTI_READOUT_DEMAND,
    BTLE_CB_CHAR_VAL_WRITEIN_POST,
    BTLE_CB_CHAR_VAL_WRITEIN_DEMAND,
    BTLE_CB_LONG_CHAR_VAL_READOUT_DEMAND,
    BTLE_CB_LONG_CHAR_VAL_PREPARE_WRITEIN_DEMAND,
    BTLE_CB_CHAR_VAL_EXEC_WRITEIN_DEMAND,

    /* client callbacks */
    BTLE_CB_DISCOVER_PRIMARY_SERVICE,
    BTLE_CB_DISCOVER_PRIMARY_SERVICE_BY_UUID,
    BTLE_CB_DISCOVER_INCLUDED_SERVICE,
    BTLE_CB_DISCOVER_CHAR,
    BTLE_CB_DISCOVER_CHAR_BY_UUID,
    BTLE_CB_DISCOVER_DESP,
    BTLE_CB_CHAR_VAL_READOUT,
    BTLE_CB_CHAR_VAL_WRITEIN_RESPONSE,
    BTLE_CB_CHAR_DESP_READOUT,
    BTLE_CB_CHAR_DESP_WRITEIN_RESPONSE,
    BTLE_CB_NOTIFICATION_RECEIVED,
    BTLE_CB_INDICATION_RECEIVED,
    BTLE_CB_CHAR_VAL_WRITEIN_STARTED,
    BTLE_CB_CHAR_VAL_MULTI_READOUT,
    BTLE_CB_RELIABLE_WRITEIN_CONFIRM,
    BTLE_CB_LONG_CHAR_VAL_READOUT,
    BTLE_CB_LONG_CHAR_DESP_READOUT,
    BTLE_CB_LONG_CHAR_VAL_WRITEIN_RESPONSE,
    BTLE_CB_LONG_CHAR_DESP_WRITEIN_RESPONSE,

    /* SMP callback */
    BTLE_CB_PAIRING_DEMAND,
    BTLE_CB_DISPLAY_PASSKEY_DEMAND,
    BTLE_CB_KEYBOARD_PASSKEY_DEMAND,
    BTLE_CB_INQUIRY_BONDING_INFORMATION,
    BTLE_CB_PAIRING_RESPONSE,
    BTLE_CB_SECURITY_REQUEST,
    BTLE_CB_INITIATOR_RESPONDER,
    BTLE_CB_PAIRING_COMPLETED,
    BTLE_CB_ENCRYPTION_INFORMATION,
    BTLE_CB_BONDING_INFO_STORED
} BTLECallbackId_t;

/**
 * @brief Enumeration for BTLE state connection
 */
typedef enum _BTLE_State_Connect_t {
    BTLE_STATE_OFFLINE,                    /* Disconnected */
    BTLE_STATE_ONLINE                      /* Connecting */
} BTLE_State_Connect_t;
/**
 * @brief Enumeration for BTLE connection role
 */
typedef enum _BTLE_Gap_Role_t {
    BTLE_DEVICE_IS_CENTRAL,                    /* GAP role is central device */
    BTLE_DEVICE_IS_PERIPHERAL                  /* GAP role is central device */
} BTLE_Gap_Role_t;
/**
 * @brief Security information structure for initialization
 */
typedef TZ1K_PACKED_HDR struct _BTLESecurityInformation_t {
    twicPairingFeature_t securityParams;
    twicPasskeyEntry_t passKey;    /* Passkey */
} TZ1K_PACKED_FTR BTLESecurityInformation_t;
/**
 * @brief Keyring structure
 */
typedef TZ1K_PACKED_HDR struct _BTLEKeyRing_t {
    twicEdiv_t ediv;                    /* Encrypted diversifier */
    twicRand_t rand;                    /* Random number */
    twicLtk_t ltk;                      /* Long term key */
    twicIrk_t irk;                      /* Identity resolving key */
    twicCsrk_t csrk;                    /* Connection signature resolving key */
} TZ1K_PACKED_FTR BTLEKeyRing_t;

/**
 * @brief Bonding information structure
 */
typedef TZ1K_PACKED_HDR struct _BTLEBondingInformation_t {
    BTLEKeyRing_t remoteKey;                /* Remote key ring */
    BTLEKeyRing_t localKey;                 /* Local key ring */
    uint8_t identityAddr[6];
    uint8_t isRandomAddr;
    uint8_t keySize;                        /* Encryption key size */
    uint8_t isKeyStored;                    /* bonding information is stored */
} TZ1K_PACKED_FTR BTLEBondingInformation_t;

////////////////////////////////////////////////////////////////////////////////

typedef TZ1K_PACKED_HDR struct _BTLEEventParingDemand_t {
    uint8_t iocap;                                      /* IO capability of initiator */
    uint8_t bond:                       1;              /* Bonding is requested */
    uint8_t mitm:                       1;              /* MITM protection is requested */
    uint8_t initator_responder:         1;              /* Initator or Responder */
    uint8_t status;
} TZ1K_PACKED_FTR BTLEEventParingDemand_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventRequestSecurityDemand_t {
    bool bonded_device:                       1;
    bool bond:                       1;
    bool mitm:                       1;
    uint8_t initator_responder:         1;              /* Initator or Responder */
} TZ1K_PACKED_FTR BTLEEventRequestSecurityDemand_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventDisplayPasskey_t {
    uint8_t initator_responder:         1;
} TZ1K_PACKED_FTR BTLEEventDisplayPasskey_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventKeyboardPasskey_t {
    uint8_t initator_responder:         1;
} TZ1K_PACKED_FTR BTLEEventKeyboardPasskey_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventEncryptionChanged_t {
    uint8_t link_status;
    uint8_t key_type;
    uint8_t key_size;
} TZ1K_PACKED_FTR BTLEEventEncryptionChanged_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventInquiryBonding_t {
    uint8_t status;
    uint8_t initator_responder:         1;
    BTLEBondingInformation_t *info_bonding;
} TZ1K_PACKED_FTR BTLEEventInquiryBonding_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventParingCompleted_t {
    twicSmReasonCode_t status;
    twicAuthInfo_t auth_info;
} TZ1K_PACKED_FTR BTLEEventParingCompleted_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventGap_t {
    uint16_t conn_handle;
    union {
        twicConnectionComplete_t connected;
        twicDisconnection_t disconnected;
        twicConnectionUpdate_t conn_params_update;
#if defined(TWIC_BLE_HWIP_V41)
        twicConnectionParameter_t conn_params;
#endif
        twicAdvReport_t adv_report;
        BTLEEventParingDemand_t paring_params;
        BTLEEventDisplayPasskey_t passkey_display;
        BTLEEventKeyboardPasskey_t passkey_keyboard;
        BTLEEventEncryptionChanged_t enc_changed;
        BTLEEventInquiryBonding_t inquiry_bond_info;
        BTLEEventParingCompleted_t paring_status;
        BTLEEventRequestSecurityDemand_t security_request;
    } params;
} TZ1K_PACKED_FTR BTLEEventGap_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventGattsWrite_t {
    uint8_t handle;
    uint16_t offset;
    uint16_t len;
    uint8_t value[1];
} TZ1K_PACKED_FTR TZ1K_ALIGNED((4)) BTLEEventGattsWrite_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventGattsRead_t {
    uint8_t handle;
    uint16_t offset;
} TZ1K_PACKED_FTR BTLEEventGattsRead_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventMtuExchange_t {
    uint16_t mtu_size;
} TZ1K_PACKED_FTR BTLEEventMtuExchange_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventHVX_t {
    unsigned count;
} TZ1K_PACKED_FTR BTLEEventHVX_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventGatts_t {
    uint16_t conn_handle;
    union {
        BTLEEventGattsWrite_t write;
        BTLEEventGattsRead_t read;
        BTLEEventMtuExchange_t mtu_exchange;
        BTLEEventHVX_t hvx_count;
        twicMtuExchangeResult_t mtu_exchange_result;
    } params;
} TZ1K_PACKED_FTR BTLEEventGatts_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventGattcPriServiceDiscoverResp_t {
    bool next;
    uint16_t attribute_handle;
    uint16_t end_group_handle;
    uint64_t uuid_lsb;
    uint64_t uuid_msb;
    uint8_t uuid_len;
} TZ1K_PACKED_FTR BTLEEventGattcPriServiceDiscoverResp_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventGattcPriServiceDiscoverByUuidResp_t {
    bool next;
    uint16_t attribute_handle;
    uint16_t end_group_handle;
} TZ1K_PACKED_FTR BTLEEventGattcPriServiceDiscoverByUuidResp_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventGattcCharsDiscoverResp_t {
    bool next;
    uint8_t char_properties;
    uint16_t attribute_handle;
    uint16_t char_value_handle;
    uint64_t uuid_lsb;
    uint64_t uuid_msb;
    uint8_t uuid_len;
} TZ1K_PACKED_FTR BTLEEventGattcCharsDiscoverResp_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventGattcReadResp_t {
    uint16_t offset;
    uint16_t len;
    uint8_t value[1];
} TZ1K_PACKED_FTR TZ1K_ALIGNED((4)) BTLEEventGattcReadResp_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventGattcHVX_t {
    uint16_t handle;
    uint16_t len;
    uint8_t value[1];
} TZ1K_PACKED_FTR TZ1K_ALIGNED((4)) BTLEEventGattcHVX_t;

typedef TZ1K_PACKED_HDR struct _BTLEEventGattc_t {
    uint8_t status;
    uint16_t conn_handle;
    uint16_t error_handle;
    union {
        BTLEEventGattcPriServiceDiscoverResp_t pri_serv_disc_resp;
        BTLEEventGattcPriServiceDiscoverByUuidResp_t pri_serv_by_uuid_disc_resp;
        BTLEEventGattcCharsDiscoverResp_t char_disc_resp;
        BTLEEventGattcReadResp_t read_resp;
        BTLEEventGattcHVX_t hvx_resp;
    } prams;
} TZ1K_PACKED_FTR BTLEEventGattc_t;

/**
 * @brief Data structure for update internal value
 */
typedef struct _BTLEEntryValue_t {
    uint8_t value[BTLE_ATT_MTU_DEFAULT];
    uint8_t length;
    uint8_t id;
} BTLEEntryValue_t;

typedef TZ1K_PACKED_HDR struct _BTLEEvent_t {
    BTLECallbackId_t event_id;
    uint16_t event_len;

    union {
        BTLEEventGap_t gaps_evt;
        BTLEEventGatts_t gatts_evt;
        BTLEEventGattc_t gattc_evt;
    } evt;
} TZ1K_PACKED_FTR BTLEEvent_t;

#ifdef __cplusplus
}
#endif

#endif /* _BTLE_LOCAL_H_ */

