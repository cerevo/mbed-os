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
#ifndef __BTLE_H__
#define __BTLE_H__

#include "btle_config.h"
#include "btle_local.h"

#include "tz1Gap.h"
#include "tz1GattServer.h"
#include "ble/SecurityManager.h"

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <inttypes.h>
#include "twic_interface.h"

#define MAX_UINT32_T          (0xFFFFFFFF)
/* Characteristic property */
#define BTLE_PROPERTY_BROADCAST                     TWIC_PROPERTY_B             /* broadcast */
#define BTLE_PROPERTY_READ                          TWIC_PROPERTY_R             /* read */
#define BTLE_PROPERTY_WRITE_WITHOUT_RESPONSE        TWIC_PROPERTY_WWR           /* write without response */
#define BTLE_PROPERTY_WRITE                         TWIC_PROPERTY_W             /* write */
#define BTLE_PROPERTY_NOTIFY                        TWIC_PROPERTY_N             /* notification */
#define BTLE_PROPERTY_INDICATE                      TWIC_PROPERTY_I             /* indication */
#define BTLE_PROPERTY_SIGNED_WRITE                  TWIC_PROPERTY_A             /* signed writes */
#define BTLE_PROPERTY_EXTENDED                      TWIC_PROPERTY_E             /* extended property */

/* Attribute permission bit assign*/
#define BTLE_PERMISSION_READ                        TWIC_PERMISSION_R           /* read */
#define BTLE_PERMISSION_WRITE                       TWIC_PERMISSION_W           /* write */
#define BTLE_PERMISSION_MITM                        TWIC_PERMISSION_M0          /* MITM enabled/disabled (R or R/W) */
#define BTLE_PERMISSION_AUTHENTICATION              TWIC_PERMISSION_A0          /* Authentication required (R or R/W) */
#define BTLE_PERMISSION_ENCRYPTION                  TWIC_PERMISSION_E0          /* Encryption requied (R or R/W) */
#define BTLE_PERMISSION_MITM_WRITE                  TWIC_PERMISSION_M1          /* MITM enabled/disabled (W) */
#define BTLE_PERMISSION_AUTHENTICATION_WRITE        TWIC_PERMISSION_A1          /* Authentication required (W) */
#define BTLE_PERMISSION_ENCRYPTION_WRITE            TWIC_PERMISSION_E1          /* Encryption required (W) */
#define BTLE_PERMISSION_KEY7                        TWIC_PERMISSION_K7          /* key size 7 */
#define BTLE_PERMISSION_KEY8                        TWIC_PERMISSION_K8          /* key size 8 */
#define BTLE_PERMISSION_KEY9                        TWIC_PERMISSION_K9          /* key size 9 */
#define BTLE_PERMISSION_KEY10                       TWIC_PERMISSION_K10         /* key size 10 */
#define BTLE_PERMISSION_KEY11                       TWIC_PERMISSION_K11         /* key size 11 */
#define BTLE_PERMISSION_KEY12                       TWIC_PERMISSION_K12         /* key size 12 */
#define BTLE_PERMISSION_KEY13                       TWIC_PERMISSION_K13         /* key size 13 */
#define BTLE_PERMISSION_KEY14                       TWIC_PERMISSION_K14         /* key size 14 */
#define BTLE_PERMISSION_KEY15                       TWIC_PERMISSION_K15         /* key size 15 */
#define BTLE_PERMISSION_KEY16                       TWIC_PERMISSION_K16         /* key size 16 */
/**
 * Enumeration for BTLE APIs return value
 */
#define BTLE_OK                                     (0)
#define BTLE_EVENT_BUF_EMPTY                        (1)
#define BTLE_ERROR                                  (-1)
#define BTLE_ERROR_PARAMETER                        (-2)
#define BTLE_ERROR_CANNOT_EXECUTE                   (-3)
#define BTLE_ERROR_BUSY                             (-4)
#define BTLE_ERROR_CONNECTION_UPDATE                (-5)
#define BTLE_ERROR_LOWPOWER                         (-6)
#define BTLE_ERROR_DISCONNECT                       (-7)
#define BTLE_ERROR_MTU_EXCHANGE_RESPONSE            (-8)
#define BTLE_ERROR_MTU_EXCHANGE_REQUEST             (-100)
#define BTLE_ERROR_CHAR_MUTIL_READ_RESPONSE         (-101)
#define BTLE_ERROR_CHAR_READ_RESPONSE               (-102)
#define BTLE_ERROR_CHAR_WRITEIN_RESPONSE            (-103)
#define BTLE_ERROR_DESP_READ_RESPONSE               (-104)
#define BTLE_ERROR_DESP_WRITEIN_RESPONSE            (-105)
#define BTLE_ERROR_INDICATION_RESPONSE              (-106)
#define BTLE_ERROR_PAIRING_CONFIRM                  (-200)
#define BTLE_ERROR_REQUEST_SECURITY                 (-201)
#define BTLE_ERROR_DISPLAY_PASSKEY_REPLY            (-202)
#define BTLE_ERROR_KEYBOARD_PASSKEY_REPLY           (-203)
#define BTLE_ERROR_BONDING_INQUIRY_REPLY            (-204)
#define BTLE_ERROR_REQUEST_PAIRING                  (-205)
#define BTLE_ERROR_START_ENCRYPTION                 (-300)
#define BTLE_ERROR_SECURITY_REQUEST_RESPONSE        (-301)
/* GA Characteristics value length */
#define BTLE_DEVICE_APPEARANCE_VALUE_LENGTH         (2)
#define BTLE_DEVICE_PREF_CONN_PARAM_VALUE_LENGTH    (8)
/* GATT Service Change Characteristic value length */
#define BTLE_DEVICE_SERVICE_CHANGE_VALUE_LENGTH     (4)
/* Connection parameters */
#define BTLE_MIN_CONNECTION_INTERVAL                (0x0006)
#define BTLE_MAX_CONNECTION_INTERVAL                (0x0C80)
#define BTLE_MAX_CONNECTION_SLAVE_LATENCY           (0x01F3)
#define BTLE_MIN_CONNECTION_SUPERVISION_TIMEOUT     (0x000A)
#define BTLE_MAX_CONNECTION_SUPERVISION_TIMEOUT     (0x0C80)
/* Advertising parameters */
#define BTLE_ADVERTISING_LENGTH_MAX                 (31)
#define BTLE_MIN_ADVERTISING_INTERVAL               (0x0020)
#define BTLE_MAX_ADVERTISING_INTERVAL               (0x4000)
/* Scan parameters */
#define BTLE_SCAN_INTERVAL_MIN                      (0x0004)
#define BTLE_SCAN_INTERVAL_MAX                      (0x4000)
#define BTLE_SCAN_WINDOW_MIN                        (0x0004)
#define BTLE_SCAN_WINDOW_MAX                        (0x4000)

/*
 * Enumeration for UUID length.
 */
typedef enum _BTLE_UUIDType {
    BTLE_UUID_8     = TWIC_UUID8,   /* 8bit */
    BTLE_UUID_16    = TWIC_UUID16,  /* 16bit */
    BTLE_UUID_32    = TWIC_UUID32,  /* 32bit */
    BTLE_UUID_128   = TWIC_UUID128  /* 128bit */
} BTLE_UUIDType;

/*
 * Enumeration for Address type.
 */
typedef enum _BTLE_AddressType {
    BTLE_PUBLIC_ADDRESS = TWIC_ADDR_TYPE_PUBLIC_DEVICE_ADDRESS,
    BTLE_RANDOM_STATIC_ADDRESS = TWIC_ADDR_TYPE_RANDOM_DEVICE_ADDRESS,
    BTLE_RANDOM_PRIVATE_RESOLVABLE_ADDRESS = (TWIC_ADDR_TYPE_RANDOM_DEVICE_ADDRESS + 1),
    BTLE_RANDOM_PRIVATE_NON_RESOLVABLE_ADDRESS = (TWIC_ADDR_TYPE_RANDOM_DEVICE_ADDRESS + 2),
} BTLE_AddressType;

/*
 * Enumeration for Advertising type.
 */
typedef enum _BTLE_AdvertisingType {
    BTLE_ADV_TYPE_IND = TWIC_ADV_TYPE_IND,
    BTLE_ADV_TYPE_DIRECT_IND = TWIC_ADV_TYPE_DIRECT_IND,
    BTLE_ADV_TYPE_SCAN_IND = TWIC_ADV_TYPE_SCAN_IND,
    BTLE_ADV_TYPE_NONCONN_IND = TWIC_ADV_TYPE_NONCONN_IND,
} BTLE_AdvertisingType;

/*
 * Enumeration for Advertising Policy.
 */
typedef enum _BTLE_AdvertisingFilterPolicy {
    BTLE_ADV_FILTER_ANY_ANY = TWIC_ADV_FILTER_ANY_ANY,
    BTLE_ADV_FILTER_WHITE_ANY = TWIC_ADV_FILTER_WHITE_ANY,
    BTLE_ADV_FILTER_ANY_WHITE = TWIC_ADV_FILTER_ANY_WHITE,
    BTLE_ADV_FILTER_WHITE_WHITE = TWIC_ADV_FILTER_WHITE_WHITE,
} BTLE_AdvertisingFilterPolicy;

/*
 * Enumeration for Security I/O Capability.
 */
typedef enum _BTLE_IoCapability {
    BTLE_IO_CAPS_DISPLAY_ONLY = TWIC_SMP_IO_CAPABILITY_DISPLAY_ONLY,
    BTLE_IO_CAPS_DISPLAY_YES_NO = TWIC_SMP_IO_CAPABILITY_DISPLAY_YES_NO,
    BTLE_IO_CAPS_KEYBOARD_ONLY = TWIC_SMP_IO_CAPABILITY_KEYBOARD_ONLY,
    BTLE_IO_CAPS_NONE = TWIC_SMP_IO_CAPABILITY_NOINPUT_NO_OUTPUT,
    BTLE_IO_CAPS_KEYBOARD_DISPLAY = TWIC_SMP_IO_CAPABILITY_KEYBOARD_DISPLAY,
} BTLE_IoCapability;

/*
 * BTLE UUID structure
 */
typedef struct _BTLE_UUID {
    uint64_t uuid_lsb;                          /* LSB of UUID */
    uint64_t uuid_msb;                          /* MSB of UUID */
    BTLE_UUIDType uuid_type;                    /* Length of UUID */
} BTLE_UUID;

/*
 * GATT Address structure
 */
typedef struct _BTLE_Address {
    uint64_t value;
    BTLE_AddressType type;
} BTLE_Address;

/*
 * GATT Connection Parameter structure
 */
typedef struct _BTLE_ConnectionParameter {
    uint16_t interval_min;
    uint16_t interval_max;
    uint16_t slave_latency;
    uint16_t supervison_timeout;
} BTLE_ConnectionParameter;

/*
 * GATT Advertising Data structure
 */
typedef struct _BTLE_AdvertisingData {
    uint8_t advertising_data_len;
    uint8_t advertising_data[BTLE_ADVERTISING_LENGTH_MAX];
    uint8_t scan_resp_data_len;
    uint8_t scan_resp_data[BTLE_ADVERTISING_LENGTH_MAX];
} BTLE_AdvertisingData;

/*
 * GATT Advertising Parameter structure
 */
typedef struct _BTLE_AdvertisingParameter {
    uint16_t min_interval;
    uint16_t max_interval;
    uint16_t time_to_stop;
    BTLE_AdvertisingType advertising_type;
    BTLE_AddressType direct_address_type;
    BTLE_AdvertisingFilterPolicy advertising_filter_policy;
    uint64_t direct_address;
} BTLE_AdvertisingParameter;

/*
 * GATT Scan Parameter structure
 */
typedef struct _BTLE_ScanParameter {
    uint16_t interval;
    uint16_t window;
    bool active_scanning;
    bool use_whitelist;
    bool filter_duplicates;
} BTLE_ScanParameter;

/* APIs for ble device hardware initialize and finalize */
/** Initialize BLE Controller extension, GATT service and register Generic Access service (0x1800)
 *  Generic Attribute service (0x1801)
 *
 *  @param          NONE
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_init(void);

/** Finalize BLE Controller extension
 *
 *  @param          NONE
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_finalize(void);
/* APIs for BLE security manager */

/** Enable the BLE Security Manager. 
 *
 *  @param enableBonding    Allow for bonding.
 *  @param requireMITM      Require protection for man-in-the-middle attacks.
 *  @param iocaps           The I/O capabilities of this peripheral.
 *  @param passkey          To specify a static passkey.
 *
 *  @returns
 *    BLE_ERROR_NONE.   Success
 *    other.            Error
 */
extern ble_error_t btle_security_init(bool enableBonding,
                                      bool requireMITM,
                                      SecurityManager::SecurityIOCapabilities_t iocaps,
                                      const SecurityManager::Passkey_t passkey);

/** Get security manager is enable or disable
 *
 *  @param          NONE
 *
 *  @returns
 *    true.     Enable
 *    false.    Disable
 */
extern bool btle_security_has_initialized(void);

/** Get the security status of a connection.
 *
 *  @param connectionHandle     Handle to identify the connection.
 *  @param securityStatusP      Security status.
 *
 *  @returns
 *    BLE_ERROR_NONE.   Success
 *    other.            Error
 */
extern ble_error_t btle_security_get_link(Gap::Handle_t connectionHandle,
        SecurityManager::LinkSecurityStatus_t *securityStatusP);

/** Set the security mode on a connection.
 *
 *  @param connectionHandle     Handle to identify the connection..
 *  @param securityMode         Requested security mode.
 *
 *  @returns
 *    BLE_ERROR_NONE.   Success
 *    other.            Error
 */
extern ble_error_t btle_security_set_link(Gap::Handle_t connectionHandle,
        SecurityManager::SecurityMode_t securityMode);

/* APIs BLE for GATT server*/
/** Begin Service Creation
 *
 *  @param serviceHandle        The unique number to handle the sevice.
 *  @param sevice_uuid          The service uuid 
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gatts_begin_service_creation(uint8_t *serviceHandle,
                                     BTLE_UUID sevice_uuid);

/** End the Service Creation
 *
 *  @param serviceHandle        The unique number to handle the sevice.
 *  @param sevice_uuid          The service uuid 
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gatts_end_service_creation(uint8_t serviceHandle,
                                       BTLE_UUID sevice_uuid);

/** Add characteristic declaration.
 *
 *  @param requiredSecurity     mbed security mode for this characteristic.
 *  @param serviceHandle        The unique number to handle the sevice contain the characteristic.
 *  @param charHandle           The unique number to handle the characteristic.
 *  @param char_uuid            The BTLE_UUID of this characteristic.
 *  @param readAuthorization    The read authorization is enabled or not.
 *  @param writeAuthorization   The write authorization is enabled or not.
 *  @param properties           The 8-bit field containing the characteristic's properties.
 *  @param ptr_value            The memory holding the initial value.
 *  @param length               The length in bytes of this characteristic's value.
 *  @param maxLength            The max length in bytes of this characteristic's value.
 *  @param has_variable_len     Whether the attribute's value length changes over time.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gatts_add_characteristics(SecurityManager::SecurityMode_t requiredSecurity,
                                    uint8_t serviceHandle,
                                    uint8_t *charHandle,
                                    BTLE_UUID char_uuid,
                                    bool readAuthorization,
                                    bool writeAuthorization,
                                    uint8_t properties,
                                    const uint8_t *ptr_value,
                                    uint16_t length,
                                    uint16_t maxLength,
                                    bool has_variable_len);

/** Add descriptor declaration.
 *
 *  @param charHandle           The unique number to handle the characteristic contain the descriptor.
 *  @param desc_uuid            The BTLE_UUID of this descriptor.
 *  @param permission           The 8-bit field containing the characteristic's properties.
 *  @param ptr_value            The memory holding the initial value.
 *  @param length               The length in bytes of this descriptor's value.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gatts_add_descriptor(uint8_t charHandle,
                               uint8_t *descHandle,
                               BTLE_UUID desc_uuid,
                               uint8_t permission,
                               const uint8_t *ptr_value,
                               uint16_t length);

/** Update the value of a characteristic/descriptor on the local GATT server.
 *
 *  @param eidx                 Handle for the value attribute of the characteristic.
 *  @param buffer               A pointer to a buffer holding the new value.
 *  @param len                  Size of the new value (in bytes).
 *  @param localOnly            Should this update be kept on the local
 *                              GATT server regardless of the state of the
 *                              notify/indicate flag in the CCCD for this
 *                              Characteristic? If set to true, no notification
 *                              or indication is generated.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gatts_update_value(uint8_t eidx, const uint8_t buffer[], uint16_t len, bool localOnly);

/** Determine the updates-enabled status (notification or indication) for a characteristic's CCCD.
 *
 *  @param eidx                 Handle for the value attribute of the characteristic.
 *
 *  @returns
 *    true.     Enable
 *    false.    Disable
 */
extern bool btle_gatts_characteristic_enable_update(uint8_t eidx);

/* APIs for BLE GAP */
/** Set the BTLE MAC address and type.
 *
 *  @param addr                 The BTLE_Address struct address to be set.
 *
 *  @note                       Only support RANDOM STATIC ADDRESS
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_set_address(BTLE_Address addr);

/** Get the BTLE MAC address and type.
 *
 *  @param          NONE
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern BTLE_Address btle_gap_get_address(void);

/** Set the device name characteristic in the GAP service.
 *
 *  @param device_name          The new value for the device-name.
 *  @param length               Data to be written to gpio pin.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_set_device_name(const uint8_t *device_name, uint8_t *length);

/** Get the value of the device name characteristic in the GAP service.
 *
 *  @param device_name          Pointer to an empty buffer to store current device name
 *  @param length               Length of the buffer pointed to by deviceName
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_get_device_name(uint8_t *device_name, uint8_t *length);

/** Set the GAP peripheral preferred connection parameters.
 *
 *  @param params               Connection param will be set
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_set_perferred_conn_params(const BTLE_ConnectionParameter *params);

/** Get the GAP peripheral preferred connection parameters.
 *
 *  @param params               Pointer to store current connection parameters
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_get_perferred_conn_params(BTLE_ConnectionParameter *params);

/** Set the appearance characteristic in the GAP service.
 *
 *  @param appearance           The new value for the device-appearance.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_set_device_appearance(const uint16_t appearance);

/** Get the appearance characteristic in the GAP service.
 *
 *  @param appearance           The current device-appearance value.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_get_device_appearance(uint16_t *appearance);

/** Set the GAP advertisingan and scan response data
 *
 *  @param adv_data_len          The advertising data length.
 *  @param adv_data              The advertising data to be set.
 *  @param scan_resp_data_len    The scan response data length.
 *  @param adv_data              The scan response data to be set.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_set_adv_data(const uint8_t adv_data_len,
                                       const uint8_t *const adv_data,
                                       const uint8_t scan_resp_data_len,
                                       const uint8_t *const scan_resp_data);

/** Start advertising.
 *
 *  @param prams                Advertising data will be apply
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_start_advertising(const BTLE_AdvertisingParameter *prams);

/** Stop advertising.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_stop_advertising(void);

/** Create a connection
 *
 *  @param conn_addr                 Address struct of device will be connect.
 *  @param conn_params               Connection parameters will be applied after connected.
 *  @param scan_params               Scan parameters will be used.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_create_connection(BTLE_Address conn_addr,
                                  BTLE_ConnectionParameter *conn_params,
                                  BTLE_ScanParameter *scan_params);

/** Terminate an existing GAP connection.
 *  @param conn_handle              Handle of the connection will be disconnected. 
 *  @param reason                   Reason disconnect. 
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_disconnect(uint16_t conn_handle, uint8_t reason);

/** Update GAP connection
 *
 *  @param conn_handle              Handle of the connection will be updated. 
 *  @param params                   The connection parameters will be updated.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_connection_update(uint16_t conn_handle,const BTLE_ConnectionParameter *params);

/** Start scanning the BLE to set a connection.
 *
 *  @param params                 Scan parameters will be used.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_start_scan(BTLE_ScanParameter *params);

/** Stop the currently scanning.
 *
 *  @param          NONE
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_stop_scan(void);

/** Set tx power register.
 *
 *  @param txPower                  The value of TX power
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_set_tx_power(int8_t txPower);

/* APIs for BLE GATT Client */
/**  Discover all the primary services on a server.
 *
 *  @param startHandle              The Starting Handle shall be set to 0x0001.
 *  @param endHandle                The Ending Handle shall be set to 0xFFFF.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gattc_discover_primary_service(uint16_t startHandle,
        uint16_t endHandle);

/** Discover a specific primary service on a server when only the Service UUID is known.
 *
 *  @param startHandle              The Starting Handle.
 *  @param endHandle                The Ending Handle.
 *  @param service_uuid             The UUID structure of service.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gattc_discover_primary_service_by_uuid(uint16_t startHandle,
        uint16_t endHandle,
        BTLE_UUID service_uuid);

/** Find all the characteristic declarations within a service definition on a server 
 *  when only the service handle range is known.
 *
 *  @param startHandle              The Starting Handle
 *  @param endHandle                The Ending Handle
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gattc_discover_all_characteristics(uint16_t startHandle,
        uint16_t endHandle);

/** Find all the characteristic declarations within a service definition on a server 
 *  when only the UUID service is known.
 *
 *  @param startHandle              The Starting Handle.
 *  @param endHandle                The Ending Handle.
 *  @param uuid_lsb                 The part of the Least Significant Octet in the UUID.
 *  @param uuid_msb                 The part of the Most Significant Octet in the UUID.
 *  @param uuid_len                 The bytes of UUID.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gattc_discover_characteristics_by_uuid(uint16_t startHandle,
        uint16_t endHandle,
        uint64_t uuid_lsb,
        uint64_t uuid_msb,
        uint8_t uuid_len);

/** Write a Characteristic Value to a server when the client knows the Characteristic Value Handle
 *
 *  @param handle                   The Attribute Handle parameter shall be set to the this parameter.
 *  @param length                   The length of the new characteristic.
 *  @param value                    The Attribute Value parameter shall be set to the new characteristic.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gattc_write_characteristic(uint16_t handle,
        uint8_t length,
        const uint8_t * value);

/** Write a Characteristic Value to a server without response when the client knows the Characteristic Value Handle
 *
 *  @param handle                   The Attribute Handle parameter shall be set to the this parameter.
 *  @param length                   The length of the new characteristic.
 *  @param value                    The Attribute Value parameter shall be set to the new characteristic.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gattc_write_characteristic_without_response(uint16_t handle,
                                       uint8_t length,
                                       const uint8_t *value);

/** Write a Characteristic Descriptor Value to a server when the client knows the Characteristic Descriptor Handle.
 *
 *  @param handle                   The Attribute Handle parameter shall be set to this parameter.
 *  @param length                   The length of the new characteristic descriptor value.
 *  @param value                    This parameter shall be set to the new characteristic descriptor value.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gattc_write_characteristic_descriptor(uint16_t handle,
        uint8_t length,
        const uint8_t * value);

/** Read a Characteristic Value from a server when the client knows the Characteristic Value Handle.
 *
 *  @param handle                   Characteristic Value Handle of the Characteristic Value to be read.
 *  @param offset                   The Offset parameter shall be the offset within the Characteristic Value to be read.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gattc_read_characteristic(uint16_t handle, uint16_t offset);

/* APIs for scheduling and process BLE events */
/** Execute the BLE events.
 *
 *  @param          NONE
 *
 *  @returns
 *    NONE.
 */
extern void btle_internal_events_execute(void);

/** Add the event to scheduler to process later.
 *
 *  @param          NONE
 *
 *  @returns
 *    NONE.
 */
extern void btle_scheduler_events_execute(void);

/** BTLE callback function.
 *
 *  @param          NONE
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *    NONE.
 */
extern void btle_done_callback(BTLEEvent_t *event);

#ifdef __cplusplus
}
#endif

#endif /* ifndef _BTLE_H_ */
