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
#include "tz1BLE.h"
#include "debug.h"
#include "btle_utils.h"
extern "C" {
#include "btle_tz1em.h"
}
/* The singleton which represents the TZ10XX transport for the BLE. */
static tz1BLE deviceInstance;

/*
 * BLE-API requires an implementation of the following function in order to
 * obtain its transport handle.
 */
BLEInstanceBase *
createBLEInstance(void)
{
    return (&deviceInstance);
}

/** Get the singleton of tz1BLE
 *
 * @param  instanceID       BLE Instance ID
 *
 * @return  
 *  tz1BLE&        The address of singleton tz1BLE
 *
 */
tz1BLE& tz1BLE::Instance(BLE::InstanceID_t instanceID)
{
    (void) instanceID;
    return deviceInstance;
}

tz1BLE::tz1BLE(void) :
    initialized(false),
    instanceID(BLE::DEFAULT_INSTANCE)
{
    /* Empty */
}

tz1BLE::~tz1BLE(void)
{
    /* Empty */
}

/** This call allows the application to get the BLE stack version information.
 *
 * @param       NONE
 *
 * @return      A pointer to a const string representing the version.
 *
 */
const char *tz1BLE::getVersion(void)
{
    static char version[] = VERSION;
    return version;
}

/** Initialize the BLE stack.
 *
 * @param  instanceID
 * @param  callback                             Callback function
 *
 * @return  
 *  BLE_ERROR_NONE.                              If the initialization procedure was started successfully.
 *  BLE_ERROR_INITIALIZATION_INCOMPLETE.         If used on an instance before the corresponding transport is initialized.
 *  BLE_ERROR_UNSPECIFIED.                       TWiC middleware initialization failure
 */
ble_error_t tz1BLE::init(BLE::InstanceID_t instanceID, FunctionPointerWithContext<BLE::InitializationCompleteCallbackContext *> callback)
{
    int status;

    if (initialized) {
        BLE::InitializationCompleteCallbackContext context = {
            BLE::Instance(instanceID),
            BLE_ERROR_ALREADY_INITIALIZED
        };
        callback.call(&context);
        return BLE_ERROR_ALREADY_INITIALIZED;
    }
    instanceID   = instanceID;
    /* initialize TZ1K power managerment */
    if(BTLE_TZ1EM_OK != btle_tz1em_init()) {
        return BLE_ERROR_UNSPECIFIED;
    }
    status = btle_init();
    if(status != BTLE_OK) {
        return BLE_ERROR_UNSPECIFIED;
    }

    initialized = true;
    BLE::InitializationCompleteCallbackContext context = {
        BLE::Instance(instanceID),
        BLE_ERROR_NONE
    };
    callback.call(&context);
    return  BLE_ERROR_NONE;
}

/** Purge the BLE stack of GATT and GAP state.
 *
 *   @param    NONE
 *
 *   @returns     
 *       BLE_ERROR_NONE               Everything executed properly
 *       other                        Error
 */
ble_error_t tz1BLE::shutdown(void)
{
    ble_error_t error;
    if (!initialized) {
        return BLE_ERROR_INITIALIZATION_INCOMPLETE;
    }
    btle_finalize();

    error = getGap().reset();
    if (error != BLE_ERROR_NONE) {
        return error;
    }

    error = getGattServer().reset();
    if (error != BLE_ERROR_NONE) {
        return error;
    }

    error = getGattClient().reset();
    if (error != BLE_ERROR_NONE) {
        return error;
    }

    error = getSecurityManager().reset();
    if (error != BLE_ERROR_NONE) {
        return error;
    }

    initialized = false;
    return  BLE_ERROR_NONE;
}

/** Yield control to the BLE stack or to other tasks waiting for events.
 *
 *   @param    NONE
 *
 *   @returns     
 *       NONE
 */
void tz1BLE::waitForEvent(void)
{
    /* Resume device after sleep */
    btle_tz1em_resume();
    /* process event */
    btle_internal_events_execute();
    /* Put device to sleep mode */
    btle_tz1em_go_to_sleep();
}

/** Process ALL pending events living in the BLE stack 
 *
 *   @param    NONE
 *
 *   @returns     
 *       NONE
 */
void tz1BLE::processEvents(void)
{
    btle_internal_events_execute();
}
