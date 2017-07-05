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
#ifndef __TZ1_GATT_SERVER_H__
#define __TZ1_GATT_SERVER_H__

#include <stddef.h>
#include "ble/Gap.h"
#include "ble/GattServer.h"

#define BLE_TOTAL_CHARACTERISTICS   20
#define BLE_TOTAL_DESCRIPTORS       8

class tz1GattServer : public GattServer
{
public:

    /** Create and return the tz1GattServer instance
    *
    * @param            NONE
    *
    * @return 
    *  tz1GattServer instance
    */
    static tz1GattServer &getInstance();

    /* Functions that must be implemented from GattServer */
    /** Adds a new service to the GATT table on the peripheral
    *
    * @param service                   The service to be added.
    *
    * @return  
    *  BLE_ERROR_NONE          Successfully
    *  other                   error
    */
    virtual ble_error_t addService(GattService &);

    /** Reads the value of a characteristic, based on the service and characteristic index fields
    *
    * @param attributeHandle               The handle of the GattCharacteristic to read from
    * @param buffer                        Buffer to hold the the characteristic's value
    * @param lengthP                       Length in bytes to be read.
    *
    * @return 
    *      BLE_ERROR_NOT_IMPLEMENTED    Requested a feature that isn't yet implemented or isn't supported by the target HW.
    */
    virtual ble_error_t read(GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP);

    /** Reads the value of a characteristic, based on the service and characteristic index fields
    *
    * @param connectionHandle              The confirmation handle
    * @param attributeHandle               The handle of the GattCharacteristic to read from
    * @param buffer                        Buffer to hold the the characteristic's value
    * @param lengthP                       Length in bytes to be read.
    *
    * @return 
    *      BLE_ERROR_NOT_IMPLEMENTED    Requested a feature that isn't yet implemented or isn't supported by the target HW.
    */
    virtual ble_error_t read(Gap::Handle_t connectionHandle, GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP);

    /** Updates the value of a characteristic, based on the service and characteristic index fields
    *
    * @param attributeHandle           Handle for the value attribute of the characteristic.
    * @param buffer                    Data to use when updating the characteristic's value
    * @param len                       Length of the new value (in bytes).
    * @param localOnly
    *
    * @return  
    *  BLE_ERROR_NONE          Successfully
    *  other                   error
    */
    virtual ble_error_t write(GattAttribute::Handle_t, const uint8_t[], uint16_t, bool localOnly = false);

    /** Updates the value of a characteristic, based on the service and characteristic index fields
    *
    * @param connectionHandle          The connection handle
    * @param attributeHandle           Handle for the value attribute of the characteristic.
    * @param buffer                    Data to use when updating the characteristic's value
    * @param len                       Length of the new value (in bytes).
    * @param localOnly
    *
    * @return  
    *  BLE_ERROR_NONE          Successfully
    *  other                   error
    */
    virtual ble_error_t write(Gap::Handle_t connectionHandle, GattAttribute::Handle_t, const uint8_t[], uint16_t, bool localOnly = false);

    /** Determine the updates-enabled status (notification or indication)
    *
    * @param characteristic            The characteristic.
    * @param enabledP                  Upon return, *enabledP is true if updates are enabled, else false.
    *
    * @return
    *  BLE_ERROR_NONE          Successfully
    *  other                   error
    */
    virtual ble_error_t areUpdatesEnabled(const GattCharacteristic &characteristic, bool *enabledP);

    /** Determine the updates-enabled status (notification or indication)
    *
    * @param connectionHandle          The connection handle.
    * @param characteristic            The characteristic.
    * @param enabledP                  Upon return, *enabledP is true if updates are enabled, else false.
    *
    * @return
    *      BLE_ERROR_NONE          Successfully
    *      other                   error
    */
    virtual ble_error_t areUpdatesEnabled(Gap::Handle_t connectionHandle, const GattCharacteristic &characteristic, bool *enabledP);

    /** Clear tz1GattServer's state and variable
    *
    * @param NONE
    *
    * @return
    *      BLE_ERROR_NONE          Successfully
    *      other                   error
    */
    virtual ble_error_t reset(void);

    /** Helper function that notifies the registered handler of an occurrence
    * of updates enabled, updates disabled and confirmation received events.
    *
    * @param type                      The type of event that occurred.
    * @param charHandle                The handle of the attribute that was modified.
    *
    * @return
    *      NONE
    */
    void tz1kHandleEvent(GattServerEvents::gattEvent_e type, uint16_t charHandle);

    /** Helper function that notifies all registered handlers of an occurrence
    * of a data written event
    *
    * @param params                    The data written parameters passed to the registered handlers.
    *
    * @return
    *      NONE
    */
    void tz1kDataWrittenEvent(const GattWriteCallbackParams *params);

    /** Helper function that notifies all registered handlers of an occurrence
    * of a data sent event.
    *
    * @param count                     The counter of event.
    * @return 
    *  NONE
    */
    void tz1SentDataEvent(unsigned count);

    /** Get Characteristic from handle
    *
    * @param attrHandle                The attribute handle
    *
    * @return  
    *          NULL                    The attribute handle does not match any characteristic
    *          p_char                  The pointer of characteristic.
    */
    GattCharacteristic* getCharacteristicFromHandle(uint16_t charHandle);
private:
    /** Resolve a value attribute to its owning characteristic.
     * @param  valueHandle the value handle to be resolved.
     * @return             characteristic index if a resolution is found, else -1.
     */
    int resolveValueHandleToCharIndex(GattAttribute::Handle_t valueHandle) const
    {
        unsigned charIndex;
        for (charIndex = 0; charIndex < characteristicCount; charIndex++) {
            if (characteristicHandle[charIndex] == valueHandle) {
                return charIndex;
            }
        }
        return -1;
    }
    uint8_t descriptorCount;
    uint8_t indicate_index;
    uint16_t characteristicHandle[BLE_TOTAL_CHARACTERISTICS];
    GattCharacteristic *p_characteristics[BLE_TOTAL_CHARACTERISTICS];

    tz1GattServer() : GattServer(),
        descriptorCount(0),
        indicate_index(0),
        characteristicHandle(),
        p_characteristics()
    {
        memset(p_characteristics, 0, sizeof(p_characteristics));
        memset(characteristicHandle, 0, sizeof(uint16_t));
    }
    ~tz1GattServer()
    {
        /* Empty */
    }
    tz1GattServer(tz1GattServer const &);
    void operator=(tz1GattServer const &);
};

#endif /* ifndef __tz1_GATT_SERVER_H__ */
