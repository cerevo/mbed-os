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
#include "ble/GattCharacteristic.h"

#include "debug.h"
#include "tz1GattServer.h"
#include "tz1Gap.h"

#include "btle.h"
#include "btle_utils.h"

/** Create and return the tz1GattServer instance
 *
 * @param            NONE
 *
 * @return 
 *  tz1GattServer instance
 */
tz1GattServer &tz1GattServer::getInstance(void)
{
    static tz1GattServer m_instance;
    return m_instance;
}

/** Adds a new service to the GATT table on the peripheral
 *
 * @param service                   The service to be added.
 *
 * @return  
 *  BLE_ERROR_NONE          Successfully
 *  other                   error
 */
ble_error_t tz1GattServer::addService(GattService &service)
{
    BTLE_UUID service_uuid;
    BTLE_UUID chara_uuid;
    BTLE_UUID desc_uuid;
    uint8_t uuid_type;
    uint8_t serviceHandle;
    uint8_t charHandle;
    uint8_t descHandle;
    int status;
    /* add service to TZ1K */
    uuid_type = service.getUUID().shortOrLong();
    if(uuid_type == UUID::UUID_TYPE_SHORT) {
        service_uuid.uuid_lsb = service.getUUID().getShortUUID();
        service_uuid.uuid_msb = 0;
        service_uuid.uuid_type = BTLE_UUID_16;
    } else {
        btle_utils_copy_uuid_128(&service_uuid.uuid_msb, &service_uuid.uuid_lsb, service.getUUID().getBaseUUID());
        service_uuid.uuid_type = BTLE_UUID_128;
    }

    status = btle_gatts_begin_service_creation(&serviceHandle, service_uuid);
    if(status != BTLE_OK) {
        return BLE_ERROR_INTERNAL_STACK_FAILURE;
    }
    service.setHandle(serviceHandle);

    for (uint8_t i = 0; i < service.getCharacteristicCount(); i++) {
        /* iterate to include all characteristics */
        if (characteristicCount >= BLE_TOTAL_CHARACTERISTICS) {
            return BLE_ERROR_NO_MEM;
        }
        GattCharacteristic *p_char = service.getCharacteristic(i);
        uuid_type = p_char->getValueAttribute().getUUID().shortOrLong();
        if(uuid_type == UUID::UUID_TYPE_SHORT) {
            chara_uuid.uuid_lsb = p_char->getValueAttribute().getUUID().getShortUUID();
            chara_uuid.uuid_msb = 0;
            chara_uuid.uuid_type = BTLE_UUID_16;
        } else {
            btle_utils_copy_uuid_128(&chara_uuid.uuid_msb, &chara_uuid.uuid_lsb, p_char->getValueAttribute().getUUID().getBaseUUID());
            chara_uuid.uuid_type = BTLE_UUID_128;
        }

        status = btle_gatts_add_characteristics( p_char->getRequiredSecurity(),
                                           serviceHandle,
                                           &charHandle,
                                           chara_uuid,
                                           p_char->isReadAuthorizationEnabled(),
                                           p_char->isWriteAuthorizationEnabled(),
                                           p_char->getProperties(),
                                           p_char->getValueAttribute().getValuePtr(),
                                           p_char->getValueAttribute().getLength(),
                                           p_char->getValueAttribute().getMaxLength(),
                                           p_char->getValueAttribute().hasVariableLength());
        if(status != BTLE_OK) {
            return BLE_ERROR_INTERNAL_STACK_FAILURE;
        }
        /* Update the characteristic handle */
        p_characteristics[characteristicCount] = p_char;
        p_char->getValueAttribute().setHandle(charHandle);
        characteristicHandle[characteristicCount] = charHandle;
        characteristicCount++;
        /* Add optional descriptors if any */
        for (uint8_t j = 0; j < p_char->getDescriptorCount(); j++) {
            GattAttribute *descriptor = p_char->getDescriptor(j);
            if (descriptorCount >= BLE_TOTAL_DESCRIPTORS) {
                return BLE_ERROR_NO_MEM;
            }
            if (descriptor == NULL) {
                return BLE_ERROR_INVALID_PARAM;
            }
            uuid_type = descriptor->getUUID().shortOrLong();
            if(uuid_type == UUID::UUID_TYPE_SHORT) {
                desc_uuid.uuid_lsb = descriptor->getUUID().getShortUUID();
                desc_uuid.uuid_msb = 0;
                desc_uuid.uuid_type = BTLE_UUID_16;
            } else {
                btle_utils_copy_uuid_128(&desc_uuid.uuid_msb, &desc_uuid.uuid_lsb, descriptor->getUUID().getBaseUUID());
                desc_uuid.uuid_type = BTLE_UUID_128;
            }
            status = btle_gatts_add_descriptor(charHandle, &descHandle, desc_uuid,
                                         BTLE_PERMISSION_READ | BTLE_PERMISSION_WRITE,
                                         descriptor->getValuePtr(), descriptor->getMaxLength());
            if(status != BTLE_OK) {
                return BLE_ERROR_INTERNAL_STACK_FAILURE;
            }
            descriptor->setHandle(descHandle);
            descriptorCount++;
        }
    }

    status = btle_gatts_end_service_creation(serviceHandle, service_uuid);
    if(status != BTLE_OK) {
        return BLE_ERROR_INTERNAL_STACK_FAILURE;
    }

    serviceCount++;
    return BLE_ERROR_NONE;
}

/** Reads the value of a characteristic, based on the service and characteristic index fields
 *
 * @param attributeHandle               The handle of the GattCharacteristic to read from
 * @param buffer                        Buffer to hold the the characteristic's value
 * @param lengthP                       Length in bytes to be read.
 *
 * @return 
 *      BLE_ERROR_NOT_IMPLEMENTED    Requested a feature that isn't yet.
 */
ble_error_t tz1GattServer::read(GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP)
{
    return BLE_ERROR_NOT_IMPLEMENTED;
}

/** Reads the value of a characteristic, based on the service and characteristic index fields
 *
 * @param connectionHandle              The confirmation handle
 * @param attributeHandle               The handle of the GattCharacteristic to read from
 * @param buffer                        Buffer to hold the the characteristic's value
 * @param lengthP                       Length in bytes to be read.
 *
 * @return 
 *      BLE_ERROR_NOT_IMPLEMENTED       Requested a feature that isn't yet implemented.
 */
ble_error_t tz1GattServer::read(Gap::Handle_t connectionHandle, GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP)
{
    (void)connectionHandle;
    (void)attributeHandle;
    (void)buffer;
    (void)lengthP;

    return BLE_ERROR_NOT_IMPLEMENTED;
}

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
ble_error_t tz1GattServer::write(GattAttribute::Handle_t attributeHandle, const uint8_t buffer[], uint16_t len, bool localOnly)
{
    uint16_t connectionHandle = tz1Gap::getInstance().getConnectionHandle();
    return write(connectionHandle, attributeHandle, buffer, len, localOnly);
}

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
ble_error_t tz1GattServer::write(Gap::Handle_t connectionHandle, GattAttribute::Handle_t attributeHandle, const uint8_t buffer[], uint16_t len, bool localOnly)
{
    int status;
    ble_error_t ret;

    if(connectionHandle == BLE_CONN_HANDLE_INVALID || buffer == NULL ) {
        return BLE_ERROR_INVALID_PARAM;
    }

    status = btle_gatts_update_value(attributeHandle, buffer, len, localOnly);
    if(status == BTLE_OK) {
        ret = BLE_ERROR_NONE;
    } else {
        ret = BLE_ERROR_UNSPECIFIED;
    }
    return ret;
}

/** Determine the updates-enabled status (notification or indication)
 *
 * @param characteristic            The characteristic.
 * @param enabledP                  Upon return, *enabledP is true if updates are enabled, else false.
 *
 * @return
 *  BLE_ERROR_NONE          Successfully
 *  other                   error
 */
ble_error_t tz1GattServer::areUpdatesEnabled(const GattCharacteristic &characteristic, bool *enabledP)
{
    uint16_t connectionHandle = tz1Gap::getInstance().getConnectionHandle();
    return areUpdatesEnabled(connectionHandle, characteristic, enabledP);
}

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
ble_error_t tz1GattServer::areUpdatesEnabled(Gap::Handle_t connectionHandle, const GattCharacteristic &characteristic, bool *enabledP)
{
    (void)connectionHandle;
    uint32_t charHandle = characteristic.getValueAttribute().getHandle();
    *enabledP = btle_gatts_characteristic_enable_update(charHandle);
    return BLE_ERROR_NONE;
}

/** Clear tz1GattServer's state and variable
 *
 * @param NONE
 *
 * @return
 *      BLE_ERROR_NONE          Successfully
 *      other                   error
 */
ble_error_t tz1GattServer::reset(void)
{
    /* Clear all state that is from the parent, including private members */
    GattServer::reset();

    /* Clear derived class members */
    memset(p_characteristics, 0, sizeof(p_characteristics));
    memset(characteristicHandle, 0, sizeof(uint16_t));
    descriptorCount = 0;
    indicate_index = 0;

    return BLE_ERROR_NONE;
}

/** Get Characteristic from handle
 *
 * @param attrHandle                The attribute handle
 *
 * @return  
 *          NULL                    The attribute handle does not match any characteristic
 *          p_char                  The pointer of characteristic.
 */
GattCharacteristic* tz1GattServer::getCharacteristicFromHandle(uint16_t attrHandle)
{
    GattCharacteristic *p_char = NULL;
    int characteristicIndex = resolveValueHandleToCharIndex(attrHandle);
    if(characteristicIndex == -1) {
        return NULL;
    }
    p_char = p_characteristics[characteristicIndex];
    return p_char;
}

/** Helper function that notifies the registered handler of an occurrence
 * of updates enabled, updates disabled and confirmation received events.
 *
 * @param type                      The type of event that occurred.
 * @param charHandle                The handle of the attribute that was modified.
 *
 * @return
 *      NONE
 */
void tz1GattServer::tz1kHandleEvent(GattServerEvents::gattEvent_e type, uint16_t charHandle)
{
    this->handleEvent(type, charHandle);
}

/** Helper function that notifies all registered handlers of an occurrence
 * of a data written event
 *
 * @param params                    The data written parameters passed to the registered handlers.
 *
 * @return
 *      NONE
 */
void tz1GattServer::tz1kDataWrittenEvent(const GattWriteCallbackParams *params)
{
    this->handleDataWrittenEvent(params);
}

/** Helper function that notifies all registered handlers of an occurrence
 * of a data sent event.
 *
 * @param count                     The counter of event.
 * @return 
 *  NONE
 */
void tz1GattServer::tz1SentDataEvent(unsigned count)
{
    this->handleDataSentEvent(count);
}
