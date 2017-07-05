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
#include "tz1DiscoveredCharacteristic.h"
#include "tz1GattClient.h"

/** Setup handle, properties, attribule handle, connection handle of the discovered characteristic.
 *
 * @param gattcIn                   GattClient instance
 * @param connectionHandleIn        Connection handle
 * @param propsIn                   Properties of the discovered characteristic
 * @param declHandleIn              Value handle of the discovered characteristic's declaration attribute
 * @param valueHandleIn             Value handle of the discovered characteristic's value attribute.
 *
 * @return 
 *  NONE
 */
void tz1DiscoveredCharacteristic::setup(tz1GattClient *gattcIn,
                                        Gap::Handle_t     connectionHandleIn,
                                        DiscoveredCharacteristic::Properties_t    propsIn,
                                        GattAttribute::Handle_t  declHandleIn,
                                        GattAttribute::Handle_t  valueHandleIn)
{
    gattc       = gattcIn;
    connHandle  = connectionHandleIn;
    declHandle  = declHandleIn;
    valueHandle = valueHandleIn;

    props._broadcast       = propsIn.broadcast();
    props._read            = propsIn.read();
    props._writeWoResp     = propsIn.writeWoResp();
    props._write           = propsIn.write();
    props._notify          = propsIn.notify();
    props._indicate        = propsIn.indicate();
    props._authSignedWrite = propsIn.authSignedWrite();
}

/** Setup handle, properties, attribule handle, uuid, connection handle of the discovered characteristic.
 *
 * @param gattcIn                   GattClient instance
 * @param connectionHandleIn        Connection handle
 * @param uuidIn                    Discovered characteristic's UUID.
 * @param propsIn                   Properties of the discovered characteristic
 * @param declHandleIn              Value handle of the discovered characteristic's declaration attribute
 * @param valueHandleIn             Value handle of the discovered characteristic's value attribute.
 *
 * @return 
 *  NONE
 */
void tz1DiscoveredCharacteristic::setup(tz1GattClient         *gattcIn,
                                        Gap::Handle_t            connectionHandleIn,
                                        UUID   uuidIn,
                                        DiscoveredCharacteristic::Properties_t    propsIn,
                                        GattAttribute::Handle_t  declHandleIn,
                                        GattAttribute::Handle_t  valueHandleIn)
{
    gattc       = gattcIn;
    connHandle  = connectionHandleIn;
    uuid        = uuidIn;
    declHandle  = declHandleIn;
    valueHandle = valueHandleIn;

    props._broadcast       = propsIn.broadcast();
    props._read            = propsIn.read();
    props._writeWoResp     = propsIn.writeWoResp();
    props._write           = propsIn.write();
    props._notify          = propsIn.notify();
    props._indicate        = propsIn.indicate();
    props._authSignedWrite = propsIn.authSignedWrite();
}
