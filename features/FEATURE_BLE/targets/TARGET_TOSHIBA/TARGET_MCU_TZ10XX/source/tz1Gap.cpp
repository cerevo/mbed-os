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
#include "mbed.h"

#include "tz1Gap.h"
#include "tz1GattServer.h"

#include "debug.h"
#include "btle.h"
#include "btle_utils.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "twic_service.h"
#ifdef __cplusplus
}
#endif

/** Create and return the tz1Gap instance
 *
 * @param            NONE
 *
 * @return 
 *  tz1Gap instance
 */
tz1Gap &tz1Gap::getInstance(void)
{
    static tz1Gap m_instance;
    return m_instance;
}

/** Sets the advertising parameters and payload for the device
 * @note device name and appearance need to set the same in advertising data 
 * by setDeviceName and setAppearance functions
 *
 * @param advData           The primary advertising data payload
 * @param scanResponse      The optional Scan Response payload if the advertising type is set
 *
 * @return 
 *    BLE_ERROR_NONE.       The advertising data was set successfully.
 *    other                 An appropriate error.
 */
ble_error_t tz1Gap::setAdvertisingData(const GapAdvertisingData &advData, const GapAdvertisingData &scanResponse)
{
    int ret;
    /* Make sure we don't exceed the advertising payload length */
    if (advData.getPayloadLen() > GAP_ADVERTISING_DATA_MAX_PAYLOAD) {
        return BLE_ERROR_BUFFER_OVERFLOW;
    }
    /* Make sure we have a payload! */
    if (advData.getPayloadLen() <= 0) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    } else {
        ret = btle_gap_set_adv_data(advData.getPayloadLen(), advData.getPayload(),
                                          scanResponse.getPayloadLen(), scanResponse.getPayload());
        if(ret != BTLE_OK) {
            return BLE_ERROR_UNSPECIFIED;
        }
        /* Restart advertising data */
        if(state.advertising == 1) {
            stopAdvertising();
            startAdvertising(_advParams);
        }
    }

    return BLE_ERROR_NONE;
};

/* Utility to stop advertising */
static void advertisingTimeout(void)
{
    Gap::GapState_t state;

    state = tz1Gap::getInstance().getState();
    if (state.advertising == 1) {
        tz1Gap::getInstance().stopAdvertising();
    }
}

/** Starts advertising 
 *
 * @note                            All services must be added before calling this function!
 *
 * @param  params                   Advertising parameters.
 *
 * @return  
 *      BLE_ERROR_NONE              The device started advertising successfully.
 *      other                       Error.
 */
ble_error_t tz1Gap::startAdvertising(const GapAdvertisingParams &params)
{
    int status;
    ble_error_t ret;
    BTLE_AdvertisingParameter adv_prams;

    /* Make sure we support the advertising type */
    if (params.getAdvertisingType() == GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED) {
        return BLE_ERROR_NOT_IMPLEMENTED;
    }

    /* Check interval range */
    if (params.getAdvertisingType() == GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED) {
        /* Min delay is slightly longer for unconnectable devices */
        if ((params.getIntervalInADVUnits() < GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MIN_NONCON) ||
                (params.getIntervalInADVUnits() > GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MAX)) {
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
        }
    } else {
        if ((params.getIntervalInADVUnits() < GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MIN) ||
                (params.getIntervalInADVUnits() > GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MAX)) {
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
        }
    }

    adv_prams.min_interval = params.getIntervalInADVUnits();
    adv_prams.max_interval = params.getIntervalInADVUnits();
    adv_prams.direct_address = 0x000000000000;
    adv_prams.direct_address_type = BTLE_PUBLIC_ADDRESS;
    /* Now, advertising only supported no filter policy */
    adv_prams.advertising_filter_policy = BTLE_ADV_FILTER_ANY_ANY;
    adv_prams.time_to_stop = params.getTimeout();

    switch(params.getAdvertisingType()) {
        case GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED:
            adv_prams.advertising_type = BTLE_ADV_TYPE_IND;
            break;
        case GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED:
            adv_prams.advertising_type = BTLE_ADV_TYPE_DIRECT_IND;
            break;
        case GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED:
            adv_prams.advertising_type = BTLE_ADV_TYPE_SCAN_IND;
            break;
        case GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED:
            adv_prams.advertising_type = BTLE_ADV_TYPE_NONCONN_IND;
            break;
        default:
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    status = btle_gap_start_advertising(&adv_prams);
    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }

    state.advertising = 1;

    if(params.getTimeout() != 0) {
        _advTimeout.attach(advertisingTimeout, params.getTimeout());
    }
    return ret;
}

/** Stops the BLE advertising data
 *
 * @param   NONE
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully stopped advertising procedure.
 *      other                       Error.
 */
ble_error_t tz1Gap::stopAdvertising(void)
{
    int status;
    ble_error_t ret;
    if(state.advertising == 0) {
        return BLE_ERROR_NONE;
    }

    status = btle_gap_stop_advertising();

    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }

    state.advertising = 0;
    return ret;
}

/* Utility to stop scanning */
static void scanningTimeout(void)
{
    if (tz1Gap::getInstance().getScaningStatus()) {
        tz1Gap::getInstance().stopScan();
    }
}

/** Start scanning procedure.
 *
 * @param scanningParams    Scan parameters
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully start scanning procedure.
 *      other                       Error.
 */
ble_error_t tz1Gap::startRadioScan(const GapScanningParams &scanningParams)
{
    int status;
    ble_error_t ret;
    BTLE_ScanParameter scan_params;

    scan_params.interval = scanningParams.getInterval();
    scan_params.window = scanningParams.getWindow();
    scan_params.active_scanning = scanningParams.getActiveScanning();
    scan_params.filter_duplicates = false;
    if (_scanningPolicyMode != Gap::SCAN_POLICY_IGNORE_WHITELIST) {
        scan_params.use_whitelist = true;
    } else  {
        scan_params.use_whitelist = false;
    }

    status = btle_gap_start_scan(&scan_params);
    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }

    _isScanning = true;
    if(scanningParams.getTimeout() != 0) {
        _scanTimeout.attach(scanningTimeout, scanningParams.getTimeout());
    }
    
    return ret;
}

/** Stop scanning, the current scanning parameters remain in effect.
 *
 * @param   NONE
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::stopScan()
{
    int status;
    ble_error_t ret;

    status = btle_gap_stop_scan();

    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }

    _isScanning = false;
    return ret;
}

/** Create a connection
 *
 * @param peerAddr              48-bit address, LSB format.
 * @param peerAddrType          Address type of the peer.
 * @param connectionParams      Connection parameters.
 * @param scanParams            Paramters to be used while scanning for the peer.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::connect(const Address_t           peerAddr,
                            Gap::AddressType_t        peerAddrType,
                            const ConnectionParams_t *connectionParams,
                            const GapScanningParams  *scanParams)
{
    int status;
    ble_error_t ret;
    BTLE_Address conn_addr;
    BTLE_ConnectionParameter conn_params;
    BTLE_ScanParameter scan_params;

    if(_isScanning) {
        stopScan();
    }

    if(state.advertising) {
        stopAdvertising();
    }

    for( unsigned i = 0; i < BLEProtocol::ADDR_LEN; ++i ) {
        conn_addr.value <<= 8;
        conn_addr.value += peerAddr[(BLEProtocol::ADDR_LEN - 1) - i];
    }

    switch(peerAddrType) {
        case BLEProtocol::AddressType::PUBLIC:
            conn_addr.type = BTLE_PUBLIC_ADDRESS;
            break;
        case BLEProtocol::AddressType::RANDOM_STATIC:
            conn_addr.type = BTLE_RANDOM_STATIC_ADDRESS;
            break;
        case BLEProtocol::AddressType::RANDOM_PRIVATE_RESOLVABLE:
            conn_addr.type = BTLE_RANDOM_PRIVATE_RESOLVABLE_ADDRESS;
            break;
        case BLEProtocol::AddressType::RANDOM_PRIVATE_NON_RESOLVABLE:
            conn_addr.type = BTLE_RANDOM_PRIVATE_NON_RESOLVABLE_ADDRESS;
            break;
        default:
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }
    if (connectionParams != NULL) {
        conn_params.interval_min = connectionParams->minConnectionInterval;
        conn_params.interval_max = connectionParams->maxConnectionInterval;
        conn_params.slave_latency = connectionParams->slaveLatency;
        conn_params.supervison_timeout = connectionParams->minConnectionInterval;
    } else {
        conn_params.interval_min = 50;
        conn_params.interval_max = 100;
        conn_params.slave_latency = 0;
        conn_params.supervison_timeout = 600;
    }

    if (scanParams != NULL) {
        scan_params.active_scanning      = scanParams->getActiveScanning();
        scan_params.interval    = scanParams->getInterval();
        scan_params.window      = scanParams->getWindow();
        scan_params.use_whitelist = false;
    } else {
        scan_params.active_scanning      = _scanningParams.getActiveScanning();
        scan_params.interval    = _scanningParams.getInterval();
        scan_params.window      = _scanningParams.getWindow();
        scan_params.use_whitelist = false;
    }

    status = btle_gap_create_connection(conn_addr, &conn_params, &scan_params);
    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }

    return ret;
}

/** This call initiates the disconnection procedure
 *
 * @param reason                The reason for disconnection; to be sent back to the peer.
 * @param connectionHandle      The handle of the connection to disconnect from.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::disconnect(Handle_t connectionHandle, DisconnectionReason_t reason)
{
    int status;

    status = btle_gap_disconnect(connectionHandle, reason);
    if(status != BTLE_OK) {
        return BLE_ERROR_UNSPECIFIED;
    }

    state.advertising = 0;
    state.connected   = 0;

    return BLE_ERROR_NONE;
}

/**
 * Disconnects if we are connected to a central device
 *
 * @param reason
 *              The reason for disconnection; to be sent back to the peer.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::disconnect(DisconnectionReason_t reason)
{
    return disconnect(_connectionHandle, reason);
}

/** Set current connection handle
 *
 * @param           NONE
 *
 * @return
 *      NONE
 */
void tz1Gap::setConnectionHandle(uint16_t con_handle)
{
    _connectionHandle = con_handle;
}

/** Get current connection handle
 *
 * @param           NONE
 *
 * @return  
 *      Value of current connection handle
 */
uint16_t tz1Gap::getConnectionHandle(void)
{
    return _connectionHandle;
}

/** Get Preferred Connection Paramterss
 *
 * @param params        The structure where the parameters will be stored. Memory
 *                      for this is owned by the caller.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::getPreferredConnectionParams(ConnectionParams_t *params)
{
    int status;
    BTLE_ConnectionParameter conn_params;

    if(params == NULL) {
        return BLE_ERROR_INVALID_PARAM;
    }
    
    status = btle_gap_get_perferred_conn_params(&conn_params);
    if(status != BTLE_OK) {
        return BLE_ERROR_UNSPECIFIED;
    }
    params->minConnectionInterval = conn_params.interval_min;
    params->maxConnectionInterval = conn_params.interval_max;
    params->slaveLatency = conn_params.slave_latency;
    params->connectionSupervisionTimeout = conn_params.supervison_timeout;
    return BLE_ERROR_NONE;
}

/** Set Preferred Connection Paramterss
 *
 * @param params        The structure containing the desired parameters.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::setPreferredConnectionParams(const ConnectionParams_t *params)
{
    int status;
    BTLE_ConnectionParameter conn_params;

    if(params == NULL) {
        return BLE_ERROR_INVALID_PARAM;
    }
    conn_params.interval_min = params->minConnectionInterval;
    conn_params.interval_max = params->maxConnectionInterval;
    conn_params.slave_latency = params->slaveLatency;
    conn_params.supervison_timeout = params->connectionSupervisionTimeout;

    status = btle_gap_set_perferred_conn_params(&conn_params);
    if(status != BTLE_OK) {
        return BLE_ERROR_UNSPECIFIED;
    }
    return BLE_ERROR_NONE;
}

/** Update Connection Paramters
 *
 * @param handle            Connection Handle.
 * @param newParams         Pointer to desired connection parameters. If NULL is provided on a peripheral role,
 *                          the parameters in the PPCP characteristic of the GAP service will be used instead.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::updateConnectionParams(Handle_t handle, const ConnectionParams_t *newParams)
{
    int status;
    ble_error_t ret;
    BTLE_ConnectionParameter new_params;

    if(newParams == NULL) {
        return BLE_ERROR_INVALID_PARAM;
    }
    new_params.interval_min = newParams->minConnectionInterval;
    new_params.interval_max = newParams->maxConnectionInterval;
    new_params.slave_latency = newParams->slaveLatency;
    new_params.supervison_timeout = newParams->connectionSupervisionTimeout;

    status = btle_gap_connection_update(handle, &new_params);
    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }

    return ret;
}

/** Sets the BLE device address
 *
 * @param type              The type of the BLE address to set.
 * @param address           The BLE address to set.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::setAddress(BLEProtocol::AddressType_t type, const BLEProtocol::AddressBytes_t address)
{
    int status;
    ble_error_t ret;
    BTLE_Address dev_addr;

    switch(type) {
        case BLEProtocol::AddressType::PUBLIC:
            dev_addr.type = BTLE_PUBLIC_ADDRESS;
            break;
        case BLEProtocol::AddressType::RANDOM_STATIC:
            dev_addr.type = BTLE_RANDOM_STATIC_ADDRESS;
            break;
        case BLEProtocol::AddressType::RANDOM_PRIVATE_RESOLVABLE:
            dev_addr.type = BTLE_RANDOM_PRIVATE_RESOLVABLE_ADDRESS;
            break;
        case BLEProtocol::AddressType::RANDOM_PRIVATE_NON_RESOLVABLE:
            dev_addr.type = BTLE_RANDOM_PRIVATE_NON_RESOLVABLE_ADDRESS;
            break;
        default:
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    for( unsigned i = 0; i < BLEProtocol::ADDR_LEN; ++i ) {
        dev_addr.value <<= 8;
        dev_addr.value += address[(BLEProtocol::ADDR_LEN - 1) - i];
    }

    status = btle_gap_set_address(dev_addr);
    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }
    return ret;
}

/** Gets the BLE device address.
 *
 * @param  typeP        The current BLE address type.
 * @param  address      The current BLE address.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::getAddress(BLEProtocol::AddressType_t *typeP, BLEProtocol::AddressBytes_t address)
{
    BTLE_Address dev_addr;

    if(typeP == NULL) {
        return BLE_ERROR_INVALID_PARAM;
    }
    dev_addr = btle_gap_get_address();

    for(unsigned i = 0; i < BLEProtocol::ADDR_LEN; ++i ) {
        address[i] = (dev_addr.value >> (8*i)) & 0xff;
    }

    switch(dev_addr.type) {
        case BTLE_PUBLIC_ADDRESS:
            *typeP = BLEProtocol::AddressType::PUBLIC;
            break;
        case BTLE_RANDOM_STATIC_ADDRESS:
            *typeP = BLEProtocol::AddressType::RANDOM_STATIC;
            break;
        case BTLE_RANDOM_PRIVATE_RESOLVABLE_ADDRESS:
            *typeP = BLEProtocol::AddressType::RANDOM_PRIVATE_RESOLVABLE;
            break;
        case BTLE_RANDOM_PRIVATE_NON_RESOLVABLE_ADDRESS:
            *typeP = BLEProtocol::AddressType::RANDOM_PRIVATE_NON_RESOLVABLE;
            break;
        default:
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    return BLE_ERROR_NONE;
}

/** Set the value of the device name.
 *
 * @param deviceName
 *       The new value for the device-name. This is a UTF-8 encoded, <b>NULL-terminated</b> string.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::setDeviceName(const uint8_t *deviceName)
{
    ble_error_t ret;
    int status;
    uint8_t nameLen = 0u;
    
    if(deviceName == NULL) {
        return BLE_ERROR_INVALID_PARAM;
    }
    nameLen = strlen((const char*)deviceName);

    status = btle_gap_set_device_name(deviceName, &nameLen);
    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }
    return ret;
}

/** Get the value of the device name.
 *
 * @param deviceName     Pointer to an empty buffer where the UTF-8 *non NULL-
 *                      terminated* string will be placed.
 * @param    lengthP    Length of the buffer pointed to by deviceName.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::getDeviceName(uint8_t *deviceName, unsigned *lengthP)
{
    ble_error_t ret;
    int status;

    if(deviceName == NULL || lengthP == NULL) {
        return BLE_ERROR_INVALID_PARAM;
    }

    status = btle_gap_get_device_name(deviceName, (uint8_t *)lengthP);
    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }
    return ret;
}

/** Set the appearance characteristic.
 *
 * @param appearance                The new value for the device-appearance.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::setAppearance(GapAdvertisingData::Appearance appearance)
{
    ble_error_t ret;
    int status;

    status = btle_gap_set_device_appearance(appearance);
    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }
    return ret;
}

/** Get the appearance characteristic.
 *
 * @param appearanceP           The current device-appearance value.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::getAppearance(GapAdvertisingData::Appearance *appearanceP)
{
    ble_error_t ret;
    int status;

    if(!appearanceP) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }
    status = btle_gap_get_device_appearance((uint16_t *)appearanceP);
    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }
    return ret;
}

/** Set the radio's transmit power.
 *
 * @note                Valid values of txPower are -20, -16, -12, -8, -4, 0
 * @param txPower       Radio's transmit power in dBm.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::setTxPower(int8_t txPower)
{
    ble_error_t ret;
    int status;

    status = btle_gap_set_tx_power(txPower);

    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }
    return ret;
}

/** Get Permitted TxPower Values
 *
 * @param valueArrayPP          Out parameter to receive the immutable array of Tx values.
 * @param countP                Out parameter to receive the array's size.
 *
 * @return  
 *      NONE
 */
void tz1Gap::getPermittedTxPowerValues(const int8_t **valueArrayPP, size_t *countP)
{
    if(*valueArrayPP == NULL || countP == NULL) {
        return;
    }
    /* 0x00: Reserved,   0x01: 0dBm,   0x02: -4dBm,   0x03: -8dBm,
             0x04: -12dBm,   0x05: -16dBm,   0x06: -20dBm */
    static const int8_t permittedTxValues[] = {
        -20, -16, -12, -8, -4, 0
    };

    *valueArrayPP = permittedTxValues;
    *countP = sizeof(permittedTxValues) / sizeof(int8_t);
}

/** Set the advertising policy filter mode that will be used in
 * the next call to startAdvertising().
 *
 * @param mode                      The new advertising policy filter mode.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::setAdvertisingPolicyMode(Gap::AdvertisingPolicyMode_t mode)
{
    _advertisingPolicyMode = mode;

    return BLE_ERROR_NONE;
}

/** Set the scanning policy filter mode that will be used in
 * the next call to startAdvertising().
 *
 * @param mode
 *              The new scan policy filter mode.
 *
 * @return  
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::setScanningPolicyMode(Gap::ScanningPolicyMode_t mode)
{
    _scanningPolicyMode = mode;

    return BLE_ERROR_NONE;
}

/** Get the current advertising policy filter mode
 *
 * param    NONE
 *
 * @return The set advertising policy filter mode.
 */
Gap::AdvertisingPolicyMode_t tz1Gap::getAdvertisingPolicyMode(void) const
{
    return _advertisingPolicyMode;
}

/** Get the current scanning policy filter mode.
 *
 * @param   NONE
 *
 * @return The set scan policy filter mode.
 *
 */
Gap::ScanningPolicyMode_t tz1Gap::getScanningPolicyMode(void) const
{
    return _scanningPolicyMode;
}

/** Clear tz1Gap's state and variable
 *
 * @param   NONE
 *
 * @return 
 *      BLE_ERROR_NONE              Successfully
 *      other                       Error.
 */
ble_error_t tz1Gap::reset(void)
{
    if (state.advertising == 1) {
        stopAdvertising();
    }
    if( _isScanning == true ) {
        stopScan();
    }
    if(_connectionHandle != BLE_CONN_HANDLE_INVALID) {
        disconnect(LOCAL_HOST_TERMINATED_CONNECTION);
    }
    /* Clear all state that is from the parent, including private members */
    Gap::reset();

    /* Clear derived class members */
    _connectionHandle = BLE_CONN_HANDLE_INVALID;

    /* Set the whitelist policy filter modes to IGNORE_WHITELIST */
    _advertisingPolicyMode = Gap::ADV_POLICY_IGNORE_WHITELIST;
    _scanningPolicyMode    = Gap::SCAN_POLICY_IGNORE_WHITELIST;

    _isScanning = false;
    return BLE_ERROR_NONE;
}
