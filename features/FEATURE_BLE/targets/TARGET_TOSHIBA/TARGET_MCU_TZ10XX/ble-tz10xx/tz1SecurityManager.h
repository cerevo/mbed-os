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
#ifndef __TZ1_SECURITY_MANAGER_H__
#define __TZ1_SECURITY_MANAGER_H__

#include "mbed.h"

#include "btle.h"
#include "tz1Gap.h"
#include "ble/SecurityManager.h"

class tz1SecurityManager : public SecurityManager
{
public:
    static tz1SecurityManager &getInstance()
    {
        static tz1SecurityManager m_instance;
        return m_instance;
    }

    /* Functions that must be implemented from SecurityManager */
    /** Enable the BLE stack's Security Manager.
     *
     * @param enableBonding     Allow for bonding.
     * @param requireMITM       Require protection for man-in-the-middle attacks.
     * @param iocaps            To specify the I/O capabilities of this peripheral,
     *                          such as availability of a display or keyboard, to
     *                          support out-of-band exchanges of security data.
     * @param passkey           To specify a static passkey.
     *
     * @return 
     *      BLE_ERROR_NONE       Success.
     *      other                Error.
     */
    virtual ble_error_t init(bool                     enableBonding,
                             bool                     requireMITM,
                             SecurityIOCapabilities_t iocaps,
                             const Passkey_t          passkey)
    {
        return btle_security_init(enableBonding, requireMITM, iocaps, passkey);
    }

    /** Get the security status of a connection.
     *
     * @param connectionHandle             Handle to identify the connection.
     * @param securityStatusP              Security status.
     *
     * @return 
     *      BLE_ERROR_NONE       Success.
     *      other                Error.
     */
    virtual ble_error_t getLinkSecurity(Gap::Handle_t connectionHandle, LinkSecurityStatus_t *securityStatusP)
    {
        return btle_security_get_link(connectionHandle, securityStatusP);
    }

    /** Set the security mode on a connection.
     *
     * @param  connectionHandle   Handle to identify the connection.
     * @param  securityMode       Requested security mode.
     *
     * @return 
     *      BLE_ERROR_NONE       Success.
     *      other                Error.
     */
    virtual ble_error_t setLinkSecurity(Gap::Handle_t connectionHandle, SecurityMode_t securityMode)
    {
        return btle_security_set_link(connectionHandle, securityMode);
    }

    /** Delete all peer device context and all related bonding information from
     * the database within the security manager.
     *
     * @return 
     *      BLE_ERROR_NOT_IMPLEMENTED   Requested a feature that isn't yet implemented or isn't supported by the target HW.
     */
    virtual ble_error_t purgeAllBondingState(void)
    {
        return BLE_ERROR_NOT_IMPLEMENTED;
    }

    /** Returns a list of addresses from peers in the stacks bond table.
     *
     * @param   addresses           (on input) @ref Gap::Whitelist_t structure where at
     *                              most addresses.capacity addresses from bonded peers will
     *                              be stored.
     *                              (on output) A copy of the addresses from bonded peers.
     *
     * @return
     *      BLE_ERROR_NOT_IMPLEMENTED   Requested a feature that isn't yet implemented or isn't supported by the target HW.
     */
    virtual ble_error_t getAddressesFromBondTable(Gap::Whitelist_t &addresses) const
    {
        return BLE_ERROR_NOT_IMPLEMENTED;
    }

    /**
     * @brief  Clear tz1SecurityManager's state.
     *
     * @return
     *      BLE_ERROR_NONE       Success.
     *      other                Error.
     */
    virtual ble_error_t reset(void)
    {
        SecurityManager::reset();
        return BLE_ERROR_NONE;
    }

    /** Check the BLE Security Manager has initialized or not .
    *
    * @param            NONE
    *
    * @return  
    *  True                 Initialized
    *  False                Not initialized
    */
    bool hasInitialized(void) const
    {
        return btle_security_has_initialized();
    }

public:
    friend class tz1n;

    tz1SecurityManager()
    {
        /* empty */
    }

    ~tz1SecurityManager()
    {
        /* empty */
    }
private:
    tz1SecurityManager(const tz1SecurityManager &);
    const tz1SecurityManager& operator=(const tz1SecurityManager &);
    friend class tz1Gap;
};

#endif /* ifndef __TZ1_SECURITY_MANAGER_H__ */
