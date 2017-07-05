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
#include "ble/UUID.h"
#include "tz1GattClient.h"
#include "debug.h"
#include "btle.h"
#include "btle_utils.h"

static uint8_t props_mask[] = {
    0x01,
    0x02,
    0x04,
    0x08,
    0x10,
    0x20,
    0x40,
    0x80
};

/** Create and return the tz1GattClient instance
 *
 * @param            NONE
 *
 * @return 
 *  tz1GattClient instance
 */
tz1GattClient &tz1GattClient::getInstance(void)
{
    static tz1GattClient m_instance;
    return m_instance;
}

/** Launch service discovery.
 *
 * @param       connectionHandle                Handle for the connection with the peer.
 * @param       sc                              This is the application callback for a matching service. Taken as NULL by default.
 * @param       cc                              This is the application callback for a matching characteristic. Taken as NULL by default.
 * @param       matchingServiceUUID             UUID-based filter for specifying a service in which the application is interested.
 * @param       matchingCharacteristicUUIDIn    UUID-based filter for specifying characteristic in which the application is interested.
 *                                              By default it is set as the wildcard UUID_UKNOWN to match against any characteristic.
 * @return
 *    BLE_ERROR_NONE                            If service discovery is launched successfully
 *    other                                     An appropriate error.
 */
ble_error_t
tz1GattClient::launchServiceDiscovery(Gap::Handle_t                               connectionHandle,
                                      ServiceDiscovery::ServiceCallback_t         sc,
                                      ServiceDiscovery::CharacteristicCallback_t  cc,
                                      const UUID                                 &matchingServiceUUIDIn,
                                      const UUID                                 &matchingCharacteristicUUIDIn)
{
    int status;
    ble_error_t ret;
    BTLE_UUID service_uuid;

    _connectionHandle = connectionHandle;
    serviceDiscoveryCallback = sc;
    characteristicDiscoveryCallback = cc;
    _matchingServiceUUID = matchingServiceUUIDIn;
    _matchingCharacteristicUUIDIn = matchingCharacteristicUUIDIn;

    /* reset services */
    _numServices = 0;
    _numChars = 0;
    _servIndex = 0;

    for(unsigned j = 0u; j < BLE_TOTAL_DISCOVERED_SERVICES; j++) {
        discoveredService[j].setup(BLE_UUID_UNKNOWN, GattAttribute::INVALID_HANDLE, GattAttribute::INVALID_HANDLE);
    }

    if(matchingServiceUUIDIn == BLE_UUID_UNKNOWN) {
        /* Wildcard: search for all services */
        status = btle_gattc_discover_primary_service(0x0001, 0xffff);
    } else {

        uint8_t type = matchingServiceUUIDIn.shortOrLong();

        if(type == UUID::UUID_TYPE_SHORT) {
            service_uuid.uuid_lsb = matchingServiceUUIDIn.getShortUUID();
            service_uuid.uuid_msb = 0;
            service_uuid.uuid_type = BTLE_UUID_16;
        } else if(type==UUID::UUID_TYPE_LONG) {
            btle_utils_copy_uuid_128(&service_uuid.uuid_lsb, &service_uuid.uuid_msb, matchingServiceUUIDIn.getBaseUUID());
            service_uuid.uuid_type = BTLE_UUID_128;
        }

        status = btle_gattc_discover_primary_service_by_uuid(0x0001,    /* start handle. search all handles */
                                                                0xffff, /* end handle. search all handles */
                                                                service_uuid);
    }
    _currentState = GATT_SERVICE_DISCOVERY;

    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }
    return ret;
}

/** Internal launch service discovery for services.
 *
 * @param   NONE
 *
 * @return
 *    BLE_ERROR_NONE        If service discovery is launched successfully; else an appropriate error.
 *    other                 An appropriate error.
 */
ble_error_t tz1GattClient::discoveryServiceCharsProcess()
{
    int status;
    ble_error_t ret;
    DiscoveredService *service = NULL;

    if(_currentState != GATT_SERVICE_DISCOVERY && _currentState != GATT_CHARS_DISCOVERY_COMPLETE) {
        return BLE_ERROR_INVALID_STATE;
    }

    service = &discoveredService[_servIndex];

    status = btle_gattc_discover_all_characteristics(service->getStartHandle(), service->getEndHandle());
    _servIndex++;

    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }
    return ret;
}

/** Initiate a GATT Client write procedure.
 *
 * @param cmd                       Command can be either a write-request (which generates a
 *                                  matching response from the peripheral), or a write-command
 *                                  (which doesn't require the connected peer to respond).
 * @param connHandle                Connection handle.
 * @param attributeHandle           Handle for the target attribtue on the remote GATT server.
 * @param length                    Length of the new value.
 * @param value                     New value being written.
 *
 * @return
 *    BLE_ERROR_NONE        If write procedure was successfully started.
 *    other                 An appropriate error.
 */
ble_error_t tz1GattClient::write(GattClient::WriteOp_t cmd, Gap::Handle_t connHandle, GattAttribute::Handle_t attributeHandle, size_t length, const uint8_t *value) const
{
    int charsDiscoveredIndex;
    int status;
    ble_error_t ret;
    tz1GattClient *gattc = const_cast<tz1GattClient*>(this);

    gattc->_currentState = GATT_WRITE_CHAR;
    gattc->writeCBParams.connHandle = connHandle;
    gattc->writeCBParams.writeOp = GattWriteCallbackParams::OP_WRITE_REQ;
    gattc->writeCBParams.handle = attributeHandle;
    gattc->writeCBParams.offset = 0;
    gattc->writeCBParams.len = length;
    gattc->writeCBParams.data = value;

    charsDiscoveredIndex = resolveValueHandleToCharIndex(attributeHandle);

    if (charsDiscoveredIndex != -1) {
        if(cmd == GattClient::GATT_OP_WRITE_REQ) {
            status = btle_gattc_write_characteristic(attributeHandle, length, value);
        } else {
            gattc->writeCBParams.writeOp = GattWriteCallbackParams::OP_WRITE_CMD;
            status = btle_gattc_write_characteristic_without_response(attributeHandle, length, value);
        }
    } else {
        status = btle_gattc_write_characteristic_descriptor(attributeHandle, length, value);
    }

    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }
    return ret;
}

/** Initiate a GATT Client read procedure by attribute-handle.
 *
 * @param   connHandle          Handle for the connection with the peer.
 * @param   attributeHandle     Handle of the attribute to read data from.
 * @param   offset              The offset from the start of the attribute value to be read.
 *
 * @return
 *    BLE_ERROR_NONE        If read procedure was successfully started.
 *    other                 An appropriate error.
 */
ble_error_t tz1GattClient::read(Gap::Handle_t connHandle, GattAttribute::Handle_t attributeHandle, uint16_t offset) const
{
    (void)offset;
    int status;
    ble_error_t ret;

    tz1GattClient *gattc = const_cast<tz1GattClient*>(this);
    gattc->_currentState = GATT_READ_CHAR;
    gattc->readCBParams.handle = attributeHandle;

    status = btle_gattc_read_characteristic(attributeHandle, offset);
    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }

    return ret;
}

/** Check if service-discovery is currently active.
 *
 * @param   NONE
 *
 * @return 
 *  True.       service-discovery is active
 *  False.       otherwise.
 */
bool tz1GattClient::isServiceDiscoveryActive() const
{
    if(_currentState == GATT_IDLE ||
            _currentState == GATT_DISCOVERY_TERMINATED ||
            _currentState == GATT_READ_CHAR ||
            _currentState == GATT_WRITE_CHAR ) {
        return false;
    }
    return true;
}

/** Terminate an ongoing service-discovery. This should result in an
 * invocation of the TerminationCallback if service-discovery is active.
 *
 * @param   NONE
 *
 * Return   NONE
 */
void tz1GattClient::terminateServiceDiscovery()
{
    _currentState = GATT_DISCOVERY_TERMINATED;

    if (terminationCallback) {
        terminationCallback(_connectionHandle);
    }
}

/** Call mbed service dicovered callback by btle_callback
 *
 *  @param connectionHandle             The connection handle.
 *  @param status                       Status of discovery primary service
 *  @param next                         Remaining service will be dicovered or not
 *  @param error_handle                 Error code.
 *  @param attribute_handle             The start handle of the discovered service
 *  @param end_group_handle             The end handle of the discovered service.
 *  @param uuid_lsb                     The part of the Least Significant Octet in the UUID
 *  @param uuid_msb                     The part of the Most Significant Octet in the UUID
 *  @param uuid_len                     The bytes of UUID
 *
 * @return 
 *      NONE
 */
void tz1GattClient::primaryServiceCB(Gap::Handle_t connectionHandle, uint8_t status, bool next,
                                     uint16_t error_handle, uint16_t attribute_handle,
                                     uint16_t end_group_handle, uint64_t uuid_lsb,
                                     uint64_t uuid_msb,   uint8_t uuid_len)
{
    (void)connectionHandle;
    ble_error_t ret = BLE_ERROR_NONE;
    UUID uuid;
    uint64_t uuid_msb_le;
    uint64_t uuid_lsb_le;
    UUID::LongUUIDBytes_t uuid_long;
    const UUID _uuid = UUID(uuid_lsb);

    if (status != 0 || error_handle !=0) {
        return;
    }

    if(uuid_len == TWIC_UUID16) {
        uuid = UUID(_uuid);
    } else {
        BTLE_SETDWORD_BE((uint8_t *)&uuid_msb_le, uuid_msb);
        BTLE_SETDWORD_BE((uint8_t *)&uuid_lsb_le, uuid_lsb);
        memcpy((uint8_t *)&uuid_long,(uint8_t *)&uuid_msb_le, sizeof(uint64_t));
        memcpy((uint8_t *)&uuid_long + sizeof(uint64_t), (uint8_t *)&uuid_lsb_le, sizeof(uint64_t));
        uuid.setupLong(uuid_long, UUID::MSB);
    }
    discoveredService[_numServices].setup(uuid, attribute_handle, end_group_handle);
    if(serviceDiscoveryCallback) {
        serviceDiscoveryCallback(&discoveredService[_numServices]);
    }
    _numServices++;

    if(!next) {
        ret = discoveryServiceCharsProcess();
    }

    if(ret != BLE_ERROR_NONE) {
        BTLE_ERROR_HANDLER(ret);
    }
}

/** Call mbed service dicovered callback by btle_callback with specific UUID
 *
 *  @param connectionHandle             The connection handle.
 *  @param status                       Status of discovery primary service
 *  @param next                         Remaining service will be dicovered or not
 *  @param error_handle                 Error code.
 *  @param attribute_handle             The start handle of the discovered service
 *  @param end_group_handle             The end handle of the discovered service.
 *
 * @return 
 *      NONE
 */
void tz1GattClient::primaryServiceByUuidCB(Gap::Handle_t connectionHandle, const uint8_t status,
        bool next, uint16_t error_handle, uint16_t attribute_handle,
        uint16_t end_group_handl)
{
    (void)connectionHandle;
    ble_error_t ret = BLE_ERROR_NONE;
    if (status != 0 || error_handle !=0) {
        return;
    }
    discoveredService[_numServices].setup(_matchingServiceUUID, attribute_handle, end_group_handl);
    if(serviceDiscoveryCallback) {
        serviceDiscoveryCallback(&discoveredService[_numServices]);
    }
    _numServices++;
    discoveryServiceCharsProcess();
    if(ret != BLE_ERROR_NONE) {
        BTLE_ERROR_HANDLER(ret);
    }
}

/** Call mbed characteristic dicovered callback by btle_callback
 *
 *  @param connectionHandle             The connection handle.
 *  @param status                       Status of discovery characteristic.
 *  @param next                         Remaining characteristic will be dicovered or not.
 *  @param error_handle                 Error code.
 *  @param attribute_handle             The attribute of the discovered characteristic.
 *  @param char_properties              Characteristic's properties.
 *  @param char_value_handle            The discovered characteristic index.
 *  @param uuid_lsb                     The part of the Least Significant Octet in the UUID.
 *  @param uuid_msb                     The part of the Most Significant Octet in the UUID.
 *  @param uuid_len                     The bytes of UUID.
 *
 * @return 
 *      NONE
 */
void tz1GattClient::serviceCharCB(Gap::Handle_t connectionHandle, uint8_t status, bool next,
                                  uint16_t error_handle, uint16_t attribute_handle,
                                  uint8_t char_properties, uint16_t char_value_handle,
                                  uint64_t uuid_lsb, uint64_t uuid_msb, uint8_t uuid_len)
{
    ble_error_t ret = BLE_ERROR_NONE;
    UUID uuid;
    uint64_t uuid_msb_le;
    uint64_t uuid_lsb_le;
    UUID::LongUUIDBytes_t uuid_long;
    const UUID _uuid = UUID(uuid_lsb);

    if (status != 0 || error_handle !=0) {
        return;
    }
    DiscoveredCharacteristic::Properties_t p;
    p._broadcast = (props_mask[0] & char_properties);
    p._read = (props_mask[1] & char_properties)>>1;
    p._writeWoResp = (props_mask[2] & char_properties)>>2;
    p._write = (props_mask[3] & char_properties)>>3;
    p._notify = (props_mask[4] & char_properties)>>4;
    p._indicate = (props_mask[5] & char_properties)>>5;
    p._authSignedWrite = (props_mask[6] & char_properties)>>6;

    charsDiscoveredhandle[_numChars] = char_value_handle;

    if(uuid_len == TWIC_UUID16) {
        uuid = UUID(_uuid);
    } else {
        BTLE_SETDWORD_BE((uint8_t *)&uuid_msb_le, uuid_msb);
        BTLE_SETDWORD_BE((uint8_t *)&uuid_lsb_le, uuid_lsb);
        memcpy((uint8_t *)&uuid_long[0],(uint8_t *)&uuid_msb_le, sizeof(uint64_t));
        memcpy((uint8_t *)&uuid_long[8], (uint8_t *)&uuid_lsb_le, sizeof(uint64_t));
        uuid.setupLong(uuid_long, UUID::MSB);
    }
    if(_matchingCharacteristicUUIDIn == BLE_UUID_UNKNOWN) {
        discoveredChar[_numChars].setup(this,
                                        connectionHandle,
                                        uuid,
                                        p,
                                        attribute_handle,
                                        char_value_handle);
        if(characteristicDiscoveryCallback) {
            characteristicDiscoveryCallback(&discoveredChar[_numChars]);
        }
        _numChars++;
    } else if(_matchingCharacteristicUUIDIn == uuid) {
        discoveredChar[_numChars].setup(this,
                                        connectionHandle,
                                        uuid,
                                        p,
                                        attribute_handle,
                                        char_value_handle);
        _servIndex = _numServices;

        if(characteristicDiscoveryCallback) {
            characteristicDiscoveryCallback(&discoveredChar[_numChars]);
        }
        _numChars++;
    }
    if(!next && (_servIndex >= _numServices)) {
        _currentState = GATT_CHARS_DISCOVERY_COMPLETE;
        terminateServiceDiscovery();
    } else if(!next) {
        discoveryServiceCharsProcess();
        if(ret != BLE_ERROR_NONE) {
            BTLE_ERROR_HANDLER(ret);
        }
    }
}

/** Call mbed characteristic read completed callback by btle_callback
 *
 *  @param connectionHandle             The connection handle.
 *  @param status                       Status of read characteristic.
 *  @param offset                       Characteristic's value offset.
 *  @param len                          Characteristic's value length.
 *  @param value                        Characteristic's value.
 *
 * @return 
 *      NONE
 */
void tz1GattClient::charReadCB(Gap::Handle_t connectionHandle, uint8_t status,
                               uint16_t offset, uint16_t len, uint8_t *value)
{
    readCBParams.connHandle = connectionHandle;
    readCBParams.offset = offset;
    readCBParams.len = len;
    readCBParams.data = value;
    tz1GattClient::getInstance().processReadResponse(&readCBParams);
}

/** Call mbed characteristic write completed callback by btle_callback
 *
 *  @param connectionHandle             The connection handle.
 *  @param status                       Status of write characteristic.
 *
 * @return 
 *      NONE
 */
void tz1GattClient::charWriteCB(Gap::Handle_t connectionHandle, uint8_t status)
{
    if(status != 0) {
        writeCBParams.connHandle = connectionHandle;
        writeCBParams.data = NULL;
        writeCBParams.len = 0;
    } else {
        writeCBParams.connHandle = connectionHandle;
    }
    tz1GattClient::getInstance().processWriteResponse(&writeCBParams);
}

/** Call mbed characteristic HVX event callback by btle_callback
 *
 *  @param connectionHandle             The connection handle.
 *  @param status                       Status of HVX event.
 *  @param type                         HVX type is notification or indication
 *  @param char_value_handle            The handle of characteristic was occured event.
 *  @param len                          Characteristic's value length.
 *  @param value                        Characteristic's value.
 */
void tz1GattClient::handleHVXEvent(Gap::Handle_t connectionHandle,const uint8_t status, HVXType_t type,
                                   const uint16_t char_value_handle, uint16_t len, uint8_t *value)
{
    GattHVXCallbackParams params;
    params.connHandle = connectionHandle;
    params.handle = char_value_handle;
    params.type = type;
    params.len = len;
    params.data = value;
    tz1GattClient::getInstance().processHVXEvent(&params);
}
