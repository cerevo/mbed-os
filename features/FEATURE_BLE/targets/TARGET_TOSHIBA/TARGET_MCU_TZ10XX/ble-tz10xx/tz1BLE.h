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
#ifndef __TZ1_BLE_H__
#define __TZ1_BLE_H__

#include "ble/BLE.h"
#include "ble/BLEInstanceBase.h"

#include "tz1Gap.h"
#include "tz1GattServer.h"
#include "tz1GattClient.h"
#include "tz1SecurityManager.h"

#define VERSION "BLE 1.0.0 TWIC 1.41.0"

class tz1BLE : public BLEInstanceBase
{
public:
    tz1BLE(void);
    static tz1BLE& Instance(BLE::InstanceID_t instanceId);
    virtual ~tz1BLE(void);

    /** Initialize the BLE stack.
    *
    * @param  instanceID
    * @param  callback                          Callback function
    *   
    * @return   
    *  BLE_ERROR_NONE.                          If the initialization procedure was started successfully.
    *  BLE_ERROR_INITIALIZATION_INCOMPLETE.     If used on an instance before the corresponding transport is initialized.
    *  BLE_ERROR_UNSPECIFIED.                   TWiC middleware initialization failure
    */
    virtual ble_error_t init(BLE::InstanceID_t instanceID, FunctionPointerWithContext<BLE::InitializationCompleteCallbackContext *> callback);

    /** Check the BLE stack has initialized or not .
    *
    * @param            NONE
    *
    * @return  
    *  True                 Initialized
    *  False                Not initialized
    */
    virtual bool hasInitialized(void) const
    {
        return initialized;
    }

    /** Purge the BLE stack of GATT and GAP state.
    *
    *   @param    NONE
    *
    *   @returns     
    *       BLE_ERROR_NONE               Everything executed properly
    *       other                        Error
    */
    virtual ble_error_t shutdown(void);
    
    /** This call allows the application to get the BLE stack version information.
    *
    * @param       NONE
    *
    * @return      A pointer to a const string representing the version.
    */
    virtual const char *getVersion(void);

    /** Get tz1Gap object
    *
    * @param       NONE
    *
    * @return     
    *    A reference to tz1Gap.
    */
    virtual Gap &getGap()
    {
        return tz1Gap::getInstance();
    };

    /** Get tz1Gap object
    *
    * @param       NONE
    *
    * @return     
    *    A reference to tz1Gap.
    */
    virtual const Gap &getGap() const
    {
        return tz1Gap::getInstance();
    };

    /** Get tz1GattServer object
    *
    * @param       NONE
    *
    * @return     
    *    A reference to tz1GattServer.
    */
    virtual GattServer &getGattServer()
    {
        return tz1GattServer::getInstance();
    };

    /** Get tz1GattServer object
    *
    * @param       NONE
    *
    * @return     
    *    A reference to tz1GattServer.
    */
    virtual const GattServer &getGattServer() const
    {
        return tz1GattServer::getInstance();
    };

    /** Get tz1GattClient object
    *
    * @param       NONE
    *
    * @return     
    *    A reference to tz1GattClient.
    */
    virtual GattClient &getGattClient()
    {
        return tz1GattClient::getInstance();
    }

    /** Get tz1SecurityManager object
    *
    * @param       NONE
    *
    * @return     
    *    A reference to tz1SecurityManager.
    */
    virtual const tz1SecurityManager &getSecurityManager() const
    {
        return tz1SecurityManager::getInstance();
    }

    /** Get tz1SecurityManager object
    *
    * @param       NONE
    *
    * @return     
    *    A reference to tz1SecurityManager.
    */
    virtual tz1SecurityManager &getSecurityManager()
    {
        return tz1SecurityManager::getInstance();
    }
    
    /** Yield control to the BLE stack or to other tasks waiting for events.
    *
    *   @param    NONE
    *
    *   @returns     
    *       NONE
    */
    virtual void waitForEvent(void);
    
    /** Process ALL pending events living in the BLE stack 
    *
    *   @param    NONE
    *
    *   @returns     
    *       NONE
    */
    virtual void processEvents(void);
private:
    bool              initialized;
    BLE::InstanceID_t instanceID;
};

#endif  /* __TZ1_BLE_H__ */
