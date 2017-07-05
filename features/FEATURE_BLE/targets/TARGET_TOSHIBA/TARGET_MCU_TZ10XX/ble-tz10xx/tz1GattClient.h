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
#ifndef __TZ1_GATT_CLIENT_H__
#define __TZ1_GATT_CLIENT_H__

#include "ble/GattClient.h"
#include "ble/DiscoveredService.h"
#include "tz1DiscoveredCharacteristic.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "twic_interface.h"
#include "twic_service.h"

#ifdef __cplusplus
}
#endif
#define BLE_TOTAL_DISCOVERED_SERVICES   (12)
#define BLE_TOTAL_DISCOVERED_CHARS      (20)

class tz1GattClient : public GattClient
{
public:
    static tz1GattClient &getInstance();

    enum {
        GATT_IDLE,
        GATT_SERVICE_DISCOVERY,
        GATT_CHARS_DISCOVERY_COMPLETE,
        GATT_DISCOVERY_TERMINATED,
        GATT_READ_CHAR,
        GATT_WRITE_CHAR
    };

    /**
     * Launch service discovery. Once launched, service discovery will remain
     * active with callbacks being issued back into the application for matching
     * services/characteristics. isActive() can be used to determine status; and
     * a termination callback (if setup) will be invoked at the end. Service
     * discovery can be terminated prematurely if needed using terminate().
     *
     * @param  connectionHandle
     *           Handle for the connection with the peer.
     * @param  sc
     *           This is the application callback for matching service. Taken as
     *           NULL by default. Note: service discovery may still be active
     *           when this callback is issued; calling asynchronous BLE-stack
     *           APIs from within this application callback might cause the
     *           stack to abort service discovery. If this becomes an issue, it
     *           may be better to make local copy of the discoveredService and
     *           wait for service discovery to terminate before operating on the
     *           service.
     * @param  cc
     *           This is the application callback for matching characteristic.
     *           Taken as NULL by default. Note: service discovery may still be
     *           active when this callback is issued; calling asynchronous
     *           BLE-stack APIs from within this application callback might cause
     *           the stack to abort service discovery. If this becomes an issue,
     *           it may be better to make local copy of the discoveredCharacteristic
     *           and wait for service discovery to terminate before operating on the
     *           characteristic.
     * @param  matchingServiceUUID
     *           UUID based filter for specifying a service in which the application is
     *           interested. By default it is set as the wildcard UUID_UNKNOWN,
     *           in which case it matches all services. If characteristic-UUID
     *           filter (below) is set to the wildcard value, then a service
     *           callback will be invoked for the matching service (or for every
     *           service if the service filter is a wildcard).
     * @param  matchingCharacteristicUUIDIn
     *           UUID based filter for specifying characteristic in which the application
     *           is interested. By default it is set as the wildcard UUID_UKNOWN
     *           to match against any characteristic. If both service-UUID
     *           filter and characteristic-UUID filter are used with non- wildcard
     *           values, then only a single characteristic callback is
     *           invoked for the matching characteristic.
     *
     * @Note     Using wildcard values for both service-UUID and characteristic-
     *           UUID will result in complete service discovery--callbacks being
     *           called for every service and characteristic.
     *
     * @return
     *           BLE_ERROR_NONE if service discovery is launched successfully; else an appropriate error.
     */
    virtual ble_error_t launchServiceDiscovery(Gap::Handle_t                               connectionHandle,
            ServiceDiscovery::ServiceCallback_t         sc = NULL,
            ServiceDiscovery::CharacteristicCallback_t  cc = NULL,
            const UUID                                 &matchingServiceUUID = UUID::ShortUUIDBytes_t(BLE_UUID_UNKNOWN),
            const UUID                                 &matchingCharacteristicUUIDIn = UUID::ShortUUIDBytes_t(BLE_UUID_UNKNOWN));

    /** Set up a callback for when serviceDiscovery terminates.
    *
    * @param   callback         Event handler being registered.
    *
    * Return   NONE
    */
    virtual void onServiceDiscoveryTermination(ServiceDiscovery::TerminationCallback_t callback)
    {
        terminationCallback = callback;
    }


    /** Check if service-discovery is currently active.
    *
    * @param   NONE
    *
    * @return 
    *  True,       service-discovery is active
    *  False       otherwise.
    */
    virtual bool isServiceDiscoveryActive(void) const;


    /** Terminate an ongoing service-discovery. This should result in an
    * invocation of the TerminationCallback if service-discovery is active.
    *
    * @param   NONE
    *
    * Return   NONE
    */
    virtual void terminateServiceDiscovery(void);

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
    virtual ble_error_t read(Gap::Handle_t connHandle, GattAttribute::Handle_t attributeHandle, uint16_t offset) const;

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
    virtual ble_error_t write(GattClient::WriteOp_t cmd, Gap::Handle_t connHandle,
                              GattAttribute::Handle_t attributeHandle, size_t length,
                              const uint8_t *value) const;

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
    void primaryServiceCB(Gap::Handle_t connectionHandle, uint8_t status, bool next,
                          uint16_t error_handle, uint16_t attribute_handle,
                          uint16_t end_group_handle, uint64_t uuid_lsb,
                          uint64_t uuid_msb,   uint8_t uuid_len);

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
    void primaryServiceByUuidCB(Gap::Handle_t connectionHandle, const uint8_t status,
                                bool next, uint16_t error_handle, uint16_t attribute_handle,
                                uint16_t end_group_handl);

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
    void serviceCharCB(Gap::Handle_t connectionHandle, uint8_t status, bool next,
                       uint16_t error_handle, uint16_t attribute_handle,
                       uint8_t char_properties, uint16_t char_value_handle,
                       uint64_t uuid_lsb, uint64_t uuid_msb, uint8_t uuid_len);

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
    void charReadCB(Gap::Handle_t connectionHandle, uint8_t status,
                    uint16_t offset, uint16_t len, uint8_t *value);

    /** Call mbed characteristic write completed callback by btle_callback
    *
    *  @param connectionHandle             The connection handle.
    *  @param status                       Status of write characteristic.
    *
    * @return 
    *      NONE
    */
    void charWriteCB(Gap::Handle_t connectionHandle, uint8_t status);

    /** Call mbed characteristic HVX event callback by btle_callback
    *
    *  @param connectionHandle             The connection handle.
    *  @param status                       Status of HVX event.
    *  @param type                         HVX type is notification or indication
    *  @param char_value_handle            The handle of characteristic was occured event.
    *  @param len                          Characteristic's value length.
    *  @param value                        Characteristic's value.
    */
    void handleHVXEvent(Gap::Handle_t connectionHandle, const uint8_t status, HVXType_t type,
                        const uint16_t char_value_handle, uint16_t len, uint8_t *value);

    /** Internal launch service discovery for services.
    *
    * @param   NONE
    *
    * @return
    *    BLE_ERROR_NONE        If service discovery is launched successfully; else an appropriate error.
    *    other                 An appropriate error.
    */
    ble_error_t discoveryServiceCharsProcess();
private:

    ServiceDiscovery::ServiceCallback_t  serviceDiscoveryCallback;
    ServiceDiscovery::CharacteristicCallback_t characteristicDiscoveryCallback;
    ServiceDiscovery::TerminationCallback_t terminationCallback;

    int resolveValueHandleToCharIndex(GattAttribute::Handle_t valueHandle) const
    {
        unsigned charIndex;
        for (charIndex = 0; charIndex < _numChars; charIndex++) {
            if (charsDiscoveredhandle[charIndex] == valueHandle) {
                return charIndex;
            }
        }
        return -1;
    }

    Gap::Handle_t _connectionHandle;
    DiscoveredService discoveredService[BLE_TOTAL_DISCOVERED_SERVICES];
    tz1DiscoveredCharacteristic discoveredChar[BLE_TOTAL_DISCOVERED_CHARS];
    uint16_t charsDiscoveredhandle[BLE_TOTAL_DISCOVERED_CHARS];

    GattReadCallbackParams readCBParams;
    GattWriteCallbackParams writeCBParams;

    UUID _matchingServiceUUID;
    UUID _matchingCharacteristicUUIDIn;
    uint8_t _currentState;
    uint8_t _numServices, _servIndex;
    uint8_t _numChars;

    tz1GattClient() :
        _connectionHandle(0),
        discoveredService(),
        discoveredChar(),
        charsDiscoveredhandle(),
        readCBParams(),
        writeCBParams(),
        _matchingServiceUUID(BLE_UUID_UNKNOWN),
        _matchingCharacteristicUUIDIn(),
        _currentState(GATT_IDLE),
        _numServices(0),
        _servIndex(0),
        _numChars(0)
    {
        /* Empty */
    }

    ~tz1GattClient()
    {
        /* Empty */
    }
    tz1GattClient(tz1GattClient const &);
    void operator=(tz1GattClient const &);
};

#endif
