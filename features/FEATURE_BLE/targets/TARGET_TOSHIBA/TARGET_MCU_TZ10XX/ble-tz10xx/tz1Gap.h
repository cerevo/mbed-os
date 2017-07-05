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
#ifndef __TZ1_GAP_H__
#define __TZ1_GAP_H__

#include "mbed.h"
#include "ble/GapAdvertisingParams.h"
#include "ble/GapAdvertisingData.h"
#include "ble/Gap.h"
#include "ble/GapScanningParams.h"

#define BLE_CONN_HANDLE_INVALID             (0x0)
#define MIN_ADVERTISING_INTERVAL            (0x20)
#define MIN_ADVERTISING_NONCON_INTERVAL     (0xA0)
#define MAX_ADVERTISING_INTERVAL            (0x4000)

class tz1Gap : public Gap
{
public:

    /** Create and return the tz1Gap instance
    *
    * @param            NONE
    *
    * @return 
    *  tz1Gap instance
    */
    static tz1Gap &getInstance();

    /* Functions that must be implemented from Gap */
    /**
    * Sets the BLE device address
    *
    * @param type              The type of the BLE address to set.
    * @param address           The BLE address to set.
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully
    *      other                       Error.
    */
    virtual ble_error_t setAddress(BLEProtocol::AddressType_t type, const BLEProtocol::AddressBytes_t address);

    /** Gets the BLE device address.
    *
    * @param  typeP        The current BLE address type.
    * @param  address      The current BLE address.
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully
    *      other                       Error.
    */
    virtual ble_error_t getAddress(BLEProtocol::AddressType_t *typeP, BLEProtocol::AddressBytes_t address);

    /** Sets the advertising parameters and payload for the device
    * @note device name and appearance need to set the same in advertising data 
    * by setDeviceName and setAppearance functions
    *
    * @param advData           The primary advertising data payload
    * @param scanResponse      The optional Scan Response payload if the advertising type is set
    *
    * @return 
    *  BLE_ERROR_NONE.         The advertising data was set successfully.
    *    other                 An appropriate error.
    */
    virtual ble_error_t setAdvertisingData(const GapAdvertisingData &, const GapAdvertisingData &);

    /** Get the minimum advertising interval in milliseconds for connectable
     * undirected and connectable directed event types supported by the
     * underlying BLE stack.
     *
     * @return Minimum Advertising interval in milliseconds for connectable
     *         undirected and connectable directed event types.
     */
    virtual uint16_t    getMinAdvertisingInterval(void) const
    {
        return GapAdvertisingParams::ADVERTISEMENT_DURATION_UNITS_TO_MS(MIN_ADVERTISING_INTERVAL);
    }

    /** Get the minimum advertising interval in milliseconds for scannable
     * undirected and non-connectable undirected even types supported by the
     * underlying BLE stack.
     *
     * @return Minimum Advertising interval in milliseconds for scannable
     *         undirected and non-connectable undirected event types.
     */
    virtual uint16_t    getMinNonConnectableAdvertisingInterval(void) const
    {
        return GapAdvertisingParams::ADVERTISEMENT_DURATION_UNITS_TO_MS(MIN_ADVERTISING_NONCON_INTERVAL);
    }

    /**
     * Get the maximum advertising interval in milliseconds for all event types
     * supported by the underlying BLE stack.
     *
     * @return Maximum Advertising interval in milliseconds.
     */
    virtual uint16_t    getMaxAdvertisingInterval(void) const
    {
        return GapAdvertisingParams::ADVERTISEMENT_DURATION_UNITS_TO_MS(MAX_ADVERTISING_INTERVAL);
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
    virtual ble_error_t startAdvertising(const GapAdvertisingParams &);

    /** Stops the BLE HW and disconnects from any devices
    *
    * @param   NONE
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully stopped advertising procedure.
    *      other                       Error.
    */
    virtual ble_error_t stopAdvertising(void);

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
    virtual ble_error_t connect(const Address_t, Gap::AddressType_t peerAddrType, const ConnectionParams_t *connectionParams, const GapScanningParams *scanParams);

    /** This call initiates the disconnection procedure
    *
    * @param reason                The reason for disconnection; to be sent back to the peer.
    * @param connectionHandle      The handle of the connection to disconnect from.
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully
    *      other                       Error.
    */
    virtual ble_error_t disconnect(Handle_t connectionHandle, DisconnectionReason_t reason);

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
    virtual ble_error_t disconnect(DisconnectionReason_t reason);

    /** Set current connection handle
    *
    * @param           NONE
    *
    * @return
    *      NONE
    */
    void     setConnectionHandle(uint16_t con_handle);

    /** Get current connection handle
    *
    * @param           NONE
    *
    * @return  
    *      Value of current connection handle
    */
    uint16_t getConnectionHandle(void);

    /** Set the value of the device name.
    *
    * @param deviceName
    *       The new value for the device-name. This is a UTF-8 encoded, <b>NULL-terminated</b> string.
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully
    *      other                       Error.
    */
    virtual ble_error_t setDeviceName(const uint8_t *deviceName);

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
    virtual ble_error_t getDeviceName(uint8_t *deviceName, unsigned *lengthP);

    /** Set the appearance characteristic.
    *
    * @param appearance                The new value for the device-appearance.
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully
    *      other                       Error.
    */
    virtual ble_error_t setAppearance(GapAdvertisingData::Appearance appearance);

    /** Get the appearance characteristic.
    *
    * @param appearanceP           The current device-appearance value.
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully
    *      other                       Error.
    */
    virtual ble_error_t getAppearance(GapAdvertisingData::Appearance *appearanceP);

    /** Set the radio's transmit power.
    *
    * @note                Valid values of txPower are -20, -16, -12, -8, -4, 0
    * @param txPower       Radio's transmit power in dBm.
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully
    *      other                       Error.
    */
    virtual ble_error_t setTxPower(int8_t txPower);

    /** Get Permitted TxPower Values
    *
    * @param valueArrayPP          Out parameter to receive the immutable array of Tx values.
    * @param countP                Out parameter to receive the array's size.
    *
    * @return  
    *      NONE
    */
    virtual void        getPermittedTxPowerValues(const int8_t **valueArrayPP, size_t *countP);

    /** Get Preferred Connection Paramterss
    *
    * @param params        The structure where the parameters will be stored. Memory
    *                      for this is owned by the caller.
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully
    *      other                       Error.
    */
    virtual ble_error_t getPreferredConnectionParams(ConnectionParams_t *params);

    /** Set Preferred Connection Paramterss
    *
    * @param params        The structure containing the desired parameters.
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully
    *      other                       Error.
    */
    virtual ble_error_t setPreferredConnectionParams(const ConnectionParams_t *params);

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
    virtual ble_error_t updateConnectionParams(Handle_t handle, const ConnectionParams_t *params);

    /** Start scanning procedure.
    *
    * @param scanningParams    Scan parameters
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully start scanning procedure.
    *      other                       Error.
    */
    virtual ble_error_t startRadioScan(const GapScanningParams &scanningParams);

    /** Stop scanning, the current scanning parameters remain in effect.
    *
    * @param   NONE
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully
    *      other                       Error.
    */
    virtual ble_error_t stopScan(void);

    /** Set the advertising policy filter mode that will be used in
    * the next call to startAdvertising().
    *
    * @param mode
    *              The new advertising policy filter mode.
    *
    * @return  
    *      BLE_ERROR_NONE              Successfully
    *      other                       Error.
    */
    virtual ble_error_t setAdvertisingPolicyMode(AdvertisingPolicyMode_t mode);

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
    virtual ble_error_t setScanningPolicyMode(ScanningPolicyMode_t mode);

    /** Get the current advertising policy filter mode
    *
    * param    NONE
    *
    * @return The set advertising policy filter mode.
    */
    virtual Gap::AdvertisingPolicyMode_t getAdvertisingPolicyMode(void) const;

    /** Get the current scanning policy filter mode.
    *
    * @param   NONE
    *
    * @return The set scan policy filter mode.
    *
    */
    virtual Gap::ScanningPolicyMode_t getScanningPolicyMode(void) const;

    /** Clear tz1Gap's state and variable
    *
    * @param   NONE
    *
    * @return 
    *      BLE_ERROR_NONE              Successfully
    *      other                       Error.
    */
    virtual ble_error_t reset(void);

    /** Get BLE is scanning or not
    *
    * @param   NONE
    *
    * @return 
    *      True              Scanning is active
    *      False             Scanning is not active.
    */
    bool getScaningStatus(void)
    {
        return _isScanning;
    }
private:

    uint16_t _connectionHandle;
    bool _isScanning;

    Timeout _advTimeout;
    Timeout _scanTimeout;

    /* Policy modes set by the user. By default these are set to ignore the whitelist */
    Gap::AdvertisingPolicyMode_t _advertisingPolicyMode;
    Gap::ScanningPolicyMode_t    _scanningPolicyMode;

    tz1Gap(): _connectionHandle(BLE_CONN_HANDLE_INVALID),
        _isScanning(false),
        _advTimeout(),
        _advertisingPolicyMode(),
        _scanningPolicyMode()
    {
        /* empty */
    }
    ~tz1Gap()
    {
        /* empty */
    }
    tz1Gap(tz1Gap const &);
    void operator=(tz1Gap const &);
};
#endif /* ifndef __TZ1_GAP_H__ */
