/**
 * @file twic_interface.h
 * @brief a header file for TZ10xx TWiC for Bluetooth 4.0/4.1 Smart
 * @version V1.2.0
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef _TWIC_INTERFACE_H_
#define _TWIC_INTERFACE_H_

#include "tz1sm_config.h" /* system configuration */
#include "tz1ut_list.h"
#include "tz1sm_hal.h"
#include "twic_service.h"
#include "twic_hash.h"
#include "tz1em.h"

/*
 * TWiC INTERFACE Call Back and API INDEX.
 *
 * Pl. refer to "twic_interface_cb.h" for twicIfLeRegisterCallback()
 *
 */
#include "twic_interface_fid.h"
#include "twic_interface_cb.h"
#include "twic_interface_internal.h"

/*
 * TWiC INTERFACE API.
 *
 */

twicStatus_t twicIfLeIoInitialize(void);

twicStatus_t twicIfleIoStatus(void);

twicStatus_t twicIfLeIoFinalize(const bool stop_osc32K);

twicStatus_t twicIfGattInitialize(void);

twicStatus_t twicIfGattRegistration(uint8_t * const interface);

twicStatus_t twicIfGattCleanup(const uint8_t interface,
                               twicConnIface_t * const conn_iface);

/* Pl. refer to "twic_interface_cb.h" for twicIfLeRegisterCallback() */
twicStatus_t twicIfLeRegisterCallback(
  twicConnIface_t * const iface,
  const twicIfLeServerCb_t *server_callbacks,
  const twicIfLeClientCb_t *client_callbacks,
#if defined(WIC_CONFIG_SM_INITIATOR)
  const twicIfLeSmpICb_t *smp_i_callbacks,
#else
  const void *smp_i_callbacks,
#endif  
#if defined(WIC_CONFIG_SM_RESPONDER)
  const twicIfLeSmpRCb_t *smp_r_callbacks,
#else  
  const void *smp_r_callbacks,
#endif
  const twicIfLeCb_t *le_callbacks);

twicStatus_t twicIfIsDone(twicConnIface_t * const iface, uint8_t * const fidx);

twicStatus_t twicIfDbIsDone(twicConnIface_t * const iface, uint8_t * const fidx,
                            uint8_t * const eidx, uint8_t const specified_fidx,
                            uint8_t const specified_eidx);

TZ1K_INLINE twicStatus_t twicIfLeReadBdaddr(twicConnIface_t * const iface);

#if defined(TWIC_API_LEREADLOCALVERSIONINFORMATION)
TZ1K_INLINE twicStatus_t
twicIfLeReadLocalVersionInformation(twicConnIface_t * const iface);
#endif

TZ1K_INLINE twicStatus_t twicIfLeWriteBdaddr(twicConnIface_t * const iface,
                                             const uint64_t * const bdaddr);

TZ1K_INLINE twicStatus_t twicIfLeCeChangeToCm(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfLeInitializeDevice(twicConnIface_t * const iface);

#if defined(TWIC_CONFIG_ENABLE_SCAN)
TZ1K_INLINE twicStatus_t
twicIfLeSetScanEnable(twicConnIface_t * const iface, const uint16_t interval,
                      const uint16_t window, const bool active_scanning,
                      const bool own_bdaddr_is_random,
                      const bool whitelist_enable,
                      const bool filter_duplicates);

TZ1K_INLINE twicStatus_t twicIfLeSetScanDisable(twicConnIface_t * const iface);
#endif
#if defined(TWIC_API_LECLEAREWHITELIST)
TZ1K_INLINE twicStatus_t twicIfLeClearWhitelist(twicConnIface_t * const iface);
#endif
#if defined(TWIC_API_LEDELWHITELIST)
TZ1K_INLINE twicStatus_t
twicIfLeDelWhitelist(twicConnIface_t * const iface, const bool random,
                     const twicBdaddr_t * const bd);
#endif
#if defined(TWIC_API_LEADDWHITELIST)
TZ1K_INLINE twicStatus_t
twicIfLeAddWhitelist(twicConnIface_t * const iface, const bool random,
                     const twicBdaddr_t * const bd);
#endif
#if defined(TWIC_API_LEREADWHITELISTSIZE)
TZ1K_INLINE twicStatus_t
twicIfLeReadWhitelistSize(twicConnIface_t * const iface);
#endif
TZ1K_INLINE twicStatus_t
twicIfLeReadRemoteUsedFeatures(twicConnIface_t * const iface);
#if defined(TWIC_API_LEREADLOCALSUPPORTEDFEATURES)
TZ1K_INLINE twicStatus_t
twicIfLeReadLocalSupportedFeatures(twicConnIface_t * const iface);
#endif
TZ1K_INLINE twicStatus_t
twicIfLeReadRemoteUsedFeatures(twicConnIface_t * const iface);

#if defined(TWIC_API_LESETHOSTCHANNELCLASSIFICATION)
TZ1K_INLINE twicStatus_t
twicIfLeSetHostChannelClassification(twicConnIface_t * const iface,
                                     const twicChannelMap_t *const channel_map);
#endif
#if defined(TWIC_API_LEREADCHANNELMAP)
TZ1K_INLINE twicStatus_t twicIfLeReadChannelMap(twicConnIface_t * const iface);
#endif
#if defined(TWIC_API_LEREADSUPPORTEDSTATES)
TZ1K_INLINE twicStatus_t
twicIfLeReadSupportedStates(twicConnIface_t * const iface);
#endif
TZ1K_INLINE twicStatus_t twicIfLeGattClientStart(twicConnIface_t * const iface);
/* The master can update the connection parameters by sending the
   twicIfLeConnectionUpdate. The twicIfLeConnectionUpdate will ask to send
   LL_CONNECTION_UPDATE_REQ PDU. The slave shall not send this PDU, but the
   slave may request a change to the connection parameters using the L2CAP
   LE signaling channel. For this purpose, the slave device also can use the
   twicIfLeConnectionUpdate. */
TZ1K_INLINE twicStatus_t
twicIfLeConnectionUpdate(twicConnIface_t * const iface, uint16_t conn_int_min,
                         uint16_t conn_int_max, uint16_t slave_latency,
                         uint16_t supervison_timeout, uint16_t min_ce_length,
                         uint16_t max_ce_length);
#if defined(TWIC_BLE_HWIP_V41)
TZ1K_INLINE twicStatus_t
twicIfLeConnectionParameterRequestReply(twicConnIface_t * const iface,
                                        uint16_t conn_int_min,
                                        uint16_t conn_int_max,
                                        uint16_t slave_latency,
                                        uint16_t supervison_timeout);
TZ1K_INLINE twicStatus_t
twicIfLeConnectionParameterRequestNegativeReply(twicConnIface_t * const iface,
                                                const uint8_t reason);
#endif
/* do not care bdaddr->type */
TZ1K_INLINE twicStatus_t twicIfLeDisconnect(twicConnIface_t * const iface,
                                            const twicBdaddr_t * const bd);

TZ1K_INLINE twicStatus_t twicIfLeReadTxPowerLevel(twicConnIface_t * const iface,
                                                  const uint8_t type);

TZ1K_INLINE twicStatus_t twicIfLeReadRssi(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfLeReadRemoteVersion(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfLeCreateConnection(twicConnIface_t * const iface, uint16_t interval,
                         uint16_t window, bool use_white_list,
                         bool peer_bdaddr_is_public,
                         const twicBdaddr_t *const peer_bdaddr,
                         uint16_t conn_int_min, uint16_t conn_int_max,
                         uint16_t slave_latency, uint16_t supervison_timeout,
                         uint16_t min_ce_length, uint16_t max_ce_length,
                         bool own_bdaddr_is_public);

TZ1K_INLINE twicStatus_t
twicIfLeCreateConnectionCancel(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t twicIfLeGattServerStart(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfLeGattDbBeginServiceCreation(twicConnIface_t * const iface,
                                   const uint8_t service,
                                   const uint64_t uuid_lsb,
                                   const uint64_t uuid_msb,
                                   const uint8_t uuid_len);

TZ1K_INLINE twicStatus_t
twicIfLeGattDbBeginSecondaryServiceCreation(twicConnIface_t * const iface,
                                            const uint8_t primacy,
                                            const uint8_t service,
                                            const uint64_t uuid_lsb,
                                            const uint64_t uuid_msb,
                                            const uint8_t uuid_len);

TZ1K_INLINE twicStatus_t
twicIfLeGattDbIncludeServiceCreation(twicConnIface_t * const iface,
                                     const uint8_t service,
                                     const uint8_t inclusion,
                                     const uint64_t uuid_lsb,
                                     const uint64_t uuid_msb,
                                     const uint8_t uuid_len);

TZ1K_INLINE twicStatus_t
twicIfLeGattDbAddCharacteristics(twicConnIface_t * const iface,
                                 const uint8_t service, const uint8_t cha,
                                 const uint8_t properties,
                                 const uint64_t uuid_lsb,
                                 const uint64_t uuid_msb,
                                 const uint8_t uuid_len);

TZ1K_INLINE twicStatus_t
twicIfLeGattDbUpdCharacteristics(twicConnIface_t * const iface,
                                 const uint8_t cha, const uint16_t permissions,
                                 const uint16_t element_length,
                                 const uint8_t * const element_value,
                                 const uint64_t uuid_lsb,
                                 const uint64_t uuid_msb,
                                 const uint8_t uuid_len);

TZ1K_INLINE twicStatus_t
twicIfLeGattDbSetCharacteristics(twicConnIface_t * const iface,
                                 const uint8_t cha, const uint16_t permissions,
                                 const uint16_t element_length,
                                 const uint8_t * const element_value,
                                 const uint64_t uuid_lsb,
                                 const uint64_t uuid_msb,
                                 const uint8_t uuid_len);

TZ1K_INLINE twicStatus_t
twicIfLeGattDbSetCharacteristicsVl(twicConnIface_t * const iface,
                                   const uint8_t cha,
                                   const uint16_t permissions,
                                   const uint16_t max_element_length,
                                   const uint16_t element_length,
                                   const uint8_t * const element_value,
                                   const uint64_t uuid_lsb,
                                   const uint64_t uuid_msb,
                                   const uint8_t uuid_len);

TZ1K_INLINE twicStatus_t
twicIfLeGattDbSetDescriptor(twicConnIface_t * const iface,
                            const uint8_t entity, const uint8_t desc,
                            const uint16_t permissions,
                            const uint16_t element_length,
                            const uint8_t * const element_value,
                            const uint64_t uuid_lsb,
                            const uint64_t uuid_msb, const uint8_t uuid_len);

#if defined(TWIC_LEGATTDBSETDESCRIPTORVL)
TZ1K_INLINE twicStatus_t
twicIfLeGattDbSetDescriptorVl(twicConnIface_t * const iface,
                              const uint8_t entity, const uint8_t desc,
                              const uint16_t permissions,
                              const uint16_t max_element_length,
                              const uint16_t element_length,
                              const uint8_t * const element_value,
                              const uint64_t uuid_lsb, const uint64_t uuid_msb,
                              const uint8_t uuid_len);
#endif

TZ1K_INLINE twicStatus_t
twicIfLeGattDbEndServiceCreation(twicConnIface_t * const iface,
                                 const uint8_t service);
#if defined(TWIC_API_LEGATTDBSETPERMISSIONS)
TZ1K_INLINE twicStatus_t
twicIfLeGattDbSetPermissions(twicConnIface_t * const iface,
                             const uint8_t entity, const uint16_t permissions);
#endif
#if defined(TWIC_API_LEGATTDBGETPERMISSIONS)
TZ1K_INLINE twicStatus_t
twicIfLeGattDbGetPermissions(twicConnIface_t * const iface,
                             const uint8_t entity);
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICE)
TZ1K_INLINE twicStatus_t
twicIfLeGattClientDiscoverPrimaryService(twicConnIface_t * const iface,
                                         const uint16_t start_handle,
                                         const uint16_t end_handle);
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID)
TZ1K_INLINE twicStatus_t twicIfLeGattClientDiscoverPrimaryServiceByServiceUuid(
  twicConnIface_t * const iface, const uint16_t start_handle,
  const uint16_t end_handle, const uint64_t uuid_lsb,
  const uint64_t uuid_msb, const uint8_t uuid_len);
#endif
#if defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE)
TZ1K_INLINE twicStatus_t
twicIfLeGattClientFindIncludedService(twicConnIface_t * const iface,
                                      const uint16_t start_handle,
                                      const uint16_t end_handle);
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS)
TZ1K_INLINE twicStatus_t
twicIfLeGattClientDiscoverAllCharacteristics(twicConnIface_t * const iface,  
                                             const uint16_t start_handle,
                                             const uint16_t end_handle);
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)
TZ1K_INLINE twicStatus_t
twicIfLeGattClientDiscoverCharacteristicsByUuid(twicConnIface_t * const iface,
                                                const uint16_t start_handle,
                                                const uint16_t end_handle,
                                                const uint64_t uuid_lsb,
                                                const uint64_t uuid_msb,
                                                const uint8_t uuid_len);
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS)
TZ1K_INLINE twicStatus_t
twicIfLeGattClientDiscoverAllDescriptors(twicConnIface_t * const iface,
                                         const uint16_t start_handle,
                                         const uint16_t end_handle);
#endif
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE)
TZ1K_INLINE twicStatus_t
twicIfLeGattClientReadCharacteristicValue(twicConnIface_t * const iface,
                                          const uint16_t handle);
#endif
#if defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID)
TZ1K_INLINE twicStatus_t
twicIfLeGattClientReadUsingCharacteristicUuid(twicConnIface_t * const iface,
                                              const uint16_t start_handle,
                                              const uint16_t end_handle,
                                              const uint64_t uuid_lsb,
                                              const uint64_t uuid_msb,
                                              const uint8_t uuid_len);
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICVALUE)
TZ1K_INLINE twicStatus_t
twicIfLeGattClientWriteCharacteristicValue(twicConnIface_t * const iface,
                                           const uint16_t handle,
                                           const uint8_t length,
                                           const uint8_t * const value);
#endif
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR)
TZ1K_INLINE twicStatus_t
twicIfLeGattClientReadCharacteristicDescriptor(twicConnIface_t * const iface,
                                               const uint16_t handle);
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR)
TZ1K_INLINE twicStatus_t
twicIfLeGattClientWriteCharacteristicDescriptor(twicConnIface_t * const iface,
                                                const uint16_t handle,
                                                const uint8_t length,
                                                const uint8_t * const value);
#endif
#if defined(TWIC_API_GATTCLIENTINDICATIONCONFIRMATIONRESPONSE)
TZ1K_INLINE twicStatus_t
twicIfLeGattClientIndicationConfirmationResponse(twicConnIface_t * const iface);
#endif

/* twicIfLeDiscoverable and twicIfLeStopAdvertising are applicable to
 * the LE Set Advertise Enable, Advertising Data and Advertising
 * Parameters Command. */
TZ1K_INLINE twicStatus_t
twicIfLeDiscoverable(twicConnIface_t * const iface,
                     const uint16_t min_interval,
                     const uint16_t max_interval,
                     const uint8_t advertising_type,
                     const uint8_t own_address_type,
                     const uint8_t direct_address_type,
                     const uint64_t direct_address,
                     const uint8_t advertising_channel_map,
                     const uint8_t advertising_filter_policy,
                     const uint8_t advertising_data_length,
                     const uint8_t * const advertising_data,
                     const uint8_t scan_resp_data_length,
                     const uint8_t * const scan_resp_data);

TZ1K_INLINE twicStatus_t twicIfLeStopAdvertising(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfGattNotification(twicConnIface_t * const iface, const uint8_t cha,
                       const uint8_t length, const uint8_t * const data);

TZ1K_INLINE twicStatus_t
twicIfGattIndication(twicConnIface_t * const iface, const uint8_t cha,
                     const uint8_t length, const uint8_t * const data);

TZ1K_INLINE twicStatus_t
twicIfGattServerWriteCharacteristics(twicConnIface_t * const iface,
                                     const uint8_t cha, const uint16_t length,
                                     const uint8_t * const value);

twicStatus_t twicIfPeekEvent(void);

twicStatus_t twicIfDoEvents(void);

twicStatus_t twicIfGattDeregistration(const uint8_t interface);

twicStatus_t twicIfAcceptance(twicConnIface_t * const iface,
                              uint8_t * const result,
                              uint8_t (*const data)[23]);

twicStatus_t twicIfDbAcceptance(twicConnIface_t * const iface,
                                const uint8_t eidx, uint8_t * const result);

/* Server APP has to call twicIfGattServerExgMtuResponse
 * after the twicCbExgMtuDemand invocation. */
TZ1K_INLINE twicStatus_t
twicIfGattServerExgMtuResponse(twicConnIface_t * const iface,
                               const uint8_t status,
                               const uint16_t rx_mtu_size);

#if defined(TWIC_API_GATTCLIENTEXGMTU)
TZ1K_INLINE twicStatus_t
twicIfGattClientExgMtu(twicConnIface_t * const iface,
                       const uint16_t rx_mtu_size);
#endif

/* NOTE: The update of the characteristic descriptor and value must be
 * performed by the twicIfGattServerWriteCharacteristics API before
 * the each response API such as
 * twicIfGattServerCharDespWriteInResponse API is invoked. */

/* Server APP has to call twicIfGattServerCharDespWriteInResponse
 * after the twicCbCharDespWriteInDemand invocation. */
TZ1K_INLINE twicStatus_t
twicIfGattServerCharDespWriteInResponse(twicConnIface_t * const iface,
                                        const uint8_t desc,
                                        const uint8_t status);

/* Server APP has to call twicIfGattServerCharDespReadOutResponse
 * after the twicCbCharDespReadOutDemand invocation. */
TZ1K_INLINE twicStatus_t
twicIfGattServerCharDespReadOutResponse(twicConnIface_t * const iface,
                                        const uint8_t desc,
                                        const uint8_t status);

/* Server APP has to call twicIfGattServerCharValReadOutResponse
 * after the twicCbCharValReadOutDemand invocation. */
TZ1K_INLINE twicStatus_t
twicIfGattServerCharValReadOutResponse(twicConnIface_t * const iface,
                                       const uint8_t cha, const uint8_t status);

/* Server APP has to call twicIfGattServerCharValMultiReadOutResponse
 * after the twicCbCharValMultiReadOutDemand incocation. */
TZ1K_INLINE twicStatus_t
twicIfGattServerCharValMultiReadOutResponse(twicConnIface_t * const iface,
                                            const uint8_t status,
                                            const uint8_t err_eidx);

/* Server APP has to call twicIfGattServerCharValWriteInResponse
 * after the twicCbCharValWriteInDemand invocation. */
TZ1K_INLINE twicStatus_t
twicIfGattServerCharValWriteInResponse(twicConnIface_t * const iface,
                                       const uint8_t cha, const uint8_t status);

#if defined(TWIC_API_GATTSERVERLONGCHARVALREADOUTRESPONSE)
TZ1K_INLINE twicStatus_t
twicIfGattServerLongCharValReadOutResponse(twicConnIface_t * const iface,
                                           const uint8_t cha,
                                           const uint8_t status);
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARDESPREADOUTRESPONSE)
TZ1K_INLINE twicStatus_t
twicIfGattServerLongCharDespReadOutResponse(twicConnIface_t * const iface,
                                            const uint8_t desc,
                                            const uint8_t status);
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARVALPREPAREWRITEINRESPONSE)
TZ1K_INLINE twicStatus_t
twicIfGattServerLongCharValPrepareWriteInResponse(twicConnIface_t * const iface,
                                                  const uint8_t cha,
                                                  const uint8_t status);
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARDESPPREPAREWRITEINRESPONSE)
TZ1K_INLINE twicStatus_t twicIfGattServerLongCharDespPrepareWriteInResponse(
  twicConnIface_t * const iface, const uint8_t cha, const uint8_t status);
#endif
#if defined(TWIC_API_GATTSERVEREXECCHARVALWRITEINRESPONSE)
TZ1K_INLINE twicStatus_t
twicIfGattServerExecCharValWriteInResponse(twicConnIface_t * const iface,
                                           const uint8_t cha,
                                           const uint8_t status);
#endif
#if defined(TWIC_API_GATTSERVEREXECCHARDESPWRITEINRESPONSE)
TZ1K_INLINE twicStatus_t
twicIfGattServerExecCharDespWriteInResponse(twicConnIface_t * const iface,
                                            const uint8_t desp,
                                            const uint8_t status);
#endif
TZ1K_INLINE twicStatus_t
twicIfLeCeSetBaudrate(twicConnIface_t * const iface,
                      const twicTzbtBr_t br, const uint16_t pu);
  
TZ1K_INLINE twicStatus_t
twicIfLeCeHostDelay(twicConnIface_t * const iface, const uint16_t delay);


TZ1K_INLINE twicStatus_t
twicIfLeCePatchBase(twicConnIface_t * const iface, const uint8_t length,
                    const uint8_t * const code);

TZ1K_INLINE twicStatus_t
twicIfLeCePatchWrite(twicConnIface_t * const iface, const uint8_t length,
                     const uint8_t * const code);

TZ1K_INLINE twicStatus_t
twicIfLeCePatchControl(twicConnIface_t * const iface, const uint8_t reg,
                       const bool enable);

#if defined(TWIC_API_LECEREADFWVER)
TZ1K_INLINE twicStatus_t twicIfLeCeReadFwVer(twicConnIface_t * const iface);
#endif

#if defined(TWIC_API_LECEDBUSWRITE)
TZ1K_INLINE twicStatus_t
twicIfLeCeDbusWrite(twicConnIface_t * const iface, const uint8_t addr,
                    const uint16_t data);
#endif

#if defined(TWIC_API_LECEMEMORYWRITE)
TZ1K_INLINE twicStatus_t
twicIfLeCeMemoryWrite(twicConnIface_t * const iface, const uint32_t addr,
                      const uint16_t data);
#endif

#if defined(TWIC_API_LECEXOSCTRIMING)
TZ1K_INLINE twicStatus_t
twicIfLeCeXoscTriming(twicConnIface_t * const iface, const uint8_t data);
#endif

/* 0x00: Reserved,   0x01: 0dBm,   0x02: -4dBm,   0x03: -8dBm,
   0x04: -12dBm,   0x05: -16dBm,   0x06: -20dBm */
TZ1K_INLINE twicStatus_t
twicIfLeCeSetTxPower(twicConnIface_t * const iface, const uint8_t dbm_idx);
/* power */

#if defined(TWIC_LECE_LOWPOWER)
TZ1K_INLINE twicStatus_t
twicIfLeCeLowPowerPrimarySetup(twicConnIface_t * const iface,
                               const uint16_t osc_tolerance,
                               const uint16_t osc_jitter);

TZ1K_INLINE twicStatus_t
twicIfLeCeLowPowerClockSetup(twicConnIface_t * const iface,
                             twicStatus_t * const osc_state);

TZ1K_INLINE twicStatus_t
twicIfLeCeLowPowerMode(twicConnIface_t * const iface, const bool idle,
                       const bool discoverable, const bool conn_scan,
                       const bool shutdown);

TZ1K_INLINE twicStatus_t
twicIfLeCeLowPowerDiscoverableSetup(twicConnIface_t * const iface);

static TZ1K_INLINE twicStatus_t
twicIfLeCeLowPowerControlPinSetup(twicConnIface_t * const iface,
                                  const bool host_wake);

TZ1K_INLINE twicStatus_t
twicIfLeCeHkLowPowerInfo(twicCeLeLpInfo_t * const info);

/*
 * @brief If an API has been canceled since it returns in
 * TWIC_STATUS_WAITING_FOR_ACTIVATION, use this to make the system
 * settled to either the low power consumption or activation.
 */
TZ1K_INLINE twicStatus_t twicIfLeCeHkLowPowerTransition(const bool enable);
#endif

#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
static TZ1K_INLINE twicStatus_t twicIfLeCeInitializeTz1em(void);

static TZ1K_INLINE twicStatus_t
twicIfLeCeNcAdvConfigureTz1em(tz1emVf_t sunshine_vf, tz1emOm_t mode,
                              tz1emWe_t e1, tz1emWf_t f1,
                              tz1emWe_t e2, tz1emWf_t f2, tz1emCallback_t cb);
static TZ1K_INLINE twicStatus_t
twicIfLeCeCnAdvConfigureTz1em(tz1emVf_t sunshine_vf, tz1emOm_t mode,
                              tz1emWe_t e1, tz1emWf_t f1,
                              tz1emWe_t e2, tz1emWf_t f2, tz1emCallback_t cb);
static TZ1K_INLINE twicStatus_t
twicIfLeCeIdleConfigureTz1em(tz1emVf_t sunshine_vf, tz1emOm_t mode,
                             tz1emWe_t e1, tz1emWf_t f1,
                             tz1emWe_t e2, tz1emWf_t f2, tz1emCallback_t cb);
static TZ1K_INLINE twicStatus_t
twicIfLeCeConnConfigureTz1em(tz1emVf_t sunshine_vf, tz1emOm_t mode,
                             tz1emWe_t e1, tz1emWf_t f1,
                             tz1emWe_t e2, tz1emWf_t f2, tz1emCallback_t cb);

static TZ1K_INLINE void twicIfLeCeNcAdvWithdrawalFromTz1em(void);
static TZ1K_INLINE void twicIfLeCeCnAdvWithdrawalFromTz1em(void);
static TZ1K_INLINE void twicIfLeCeIdleWithdrawalFromTz1em(void);
static TZ1K_INLINE void twicIfLeCeConnWithdrawalFromTz1em(void);

static TZ1K_INLINE void twicIfLeCeNcAdvPermitTz1em(void);
static TZ1K_INLINE void twicIfLeCeCnAdvPermitTz1em(void);
static TZ1K_INLINE void twicIfLeCeIdlePermitTz1em(void);
static TZ1K_INLINE void twicIfLeCeConnPermitTz1em(void);

static TZ1K_INLINE void twicIfLeCeHkTz1emNcAdvInfo(tz1emInfo_t * const info);
static TZ1K_INLINE void twicIfLeCeHkTz1emCnAdvInfo(tz1emInfo_t * const info);
static TZ1K_INLINE void twicIfLeCeHkTz1emIdleInfo(tz1emInfo_t * const info);
static TZ1K_INLINE void twicIfLeCeHkTz1emConnInfo(tz1emInfo_t * const info);
#endif

TZ1K_INLINE twicStatus_t twicIfLeCeHkTryToUnlock(void);

TZ1K_INLINE twicStatus_t
twicIfLeCeHkTimeout(const uint16_t cycle, const uint8_t specified_fidx);

TZ1K_INLINE twicStatus_t
twicIfLeCeFlowControl(twicConnIface_t * const iface, const bool enable);

/* smp */
#if defined(TWIC_API_LELMGENRESOLVABLEBDADDR)
TZ1K_INLINE twicStatus_t
twicIfLeLmGenResolvableBdaddr(twicConnIface_t * const iface);
#endif
#if defined(TWIC_API_LESETRANDOMADDRESS)
TZ1K_INLINE twicStatus_t
twicIfLeSetRandomAddress(twicConnIface_t * const iface,
                         const twicBdaddr_t *const bd);
#endif
#if defined(TWIC_API_LELMRESOLVEBDADDR)
TZ1K_INLINE twicStatus_t
twicIfLeLmResolveBdaddr(twicConnIface_t * const iface,
                        const twicBdaddr_t *const bd, const uint8_t num_of_irk,
                        const twicIrk_t * irks);
#endif
#if defined(TWIC_API_LESMSETIRVALUE)
TZ1K_INLINE twicStatus_t
twicIfLeSmSetIrValue(twicConnIface_t * const iface, const twicIr_t * const ir);
#endif
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
TZ1K_INLINE twicStatus_t
twicIfLeSmEncrypt(twicConnIface_t * const iface, const uint8_t * const key,
                  const uint8_t * const plain_text_data);
#endif
#if defined(TWIC_API_LELESETADVERTISINGDATA) && defined(TWIC_BLE_HWIP_V41)
/* BLUETOOTH SPECIFICATION Version 4.1 [Vol 2].
   Host Controller Interface Functional Specification.
   7.8.7 LE Set Advertising Data Command. */
TZ1K_INLINE twicStatus_t
twicIfLeSetAdvertisingData(twicConnIface_t * const iface,
                           const uint8_t advertising_data_length,
                           const uint8_t * const advertising_data);
#endif
#if defined(TWIC_CONFIG_SM_INITIATOR)

/* The Auth_Req field is a bit field that indicates the requested
 * security properties for STK and GAP bonding information The
 * following figure defines the authentication requirements bit field.
 *
 * Please refer to "BLUETOOTH SPECIFICATION Version 4.0 [Vol 3]"
 * Security Manager Specification. AuthReq (1 octet). Figure 3.3:
 * Authentication Requirements Flags.
 *
 * The MITM field is true that is set to one if the device is
 * requesting MITM protection, otherwise it shall be set to false. A
 * device sets the MITM flag to one to request an Authenticated
 * security property for STK.
 */

/* pairing_response will happen */
TZ1K_INLINE twicStatus_t
twicIfLeSmMasPairingRequest(twicConnIface_t * const iface,
                            const twicSmpIoCapability_t io_capability,
                            const bool oob_data_present,
                            const bool auth_req_bonding,
                            const bool auth_req_mitm_protection,
                            const uint8_t max_enc_key_size,
                            const bool init_key_dist_enckey,
                            const bool init_key_dist_idkey,
                            const bool init_key_dist_sign,  
                            const bool resp_key_dist_enckey,
                            const bool resp_key_dist_idkey,
                            const bool resp_key_dist_sign);

TZ1K_INLINE twicStatus_t
twicIfLeSmMasSecurityReject(twicConnIface_t * const iface,
                            const twicSmReasonCode_t reason);
  
TZ1K_INLINE twicStatus_t
twicIfLeSmMasPairingFailed(twicConnIface_t * const iface,
                           const twicSmReasonCode_t reason);

TZ1K_INLINE twicStatus_t
twicIfLeSmMasKbPasskeyEntryReply(twicConnIface_t * const iface,
                                 const twicPasskeyEntry_t *const passkey);

TZ1K_INLINE twicStatus_t
twicIfLeSmMasKbPasskeyEntryNegativeReply(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfLeSmMasDpPasskeyEntryReply(twicConnIface_t * const iface,
                                 const twicPasskeyEntry_t *const passkey);

TZ1K_INLINE twicStatus_t
twicIfLeSmMasDpPasskeyEntryNegativeReply(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfLeSmMasStartEncryption(twicConnIface_t * const iface,
                             const twicEdiv_t *const ediv,
                             const twicRand_t *const rand,
                             const twicLtk_t *const ltk,
                             const uint8_t encryption_key_size);

TZ1K_INLINE twicStatus_t
twicIfLeSmMasOobTkReply(twicConnIface_t * const iface,
                        const twicOobTk_t *const tk);

TZ1K_INLINE twicStatus_t
twicIfLeSmMasOobTkNegativeReply(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfLeSmMasBondingInformationReply(twicConnIface_t * const iface,
                                     const twicEdiv_t * const r_ediv,
                                     const twicRand_t * const r_rand,
                                     const twicLtk_t * const r_ltk,
                                     const twicIrk_t * const r_irk,
                                     const twicCsrk_t * const r_csrk,
                                     const twicEdiv_t * const l_ediv,
                                     const twicRand_t * const l_rand,
                                     const twicLtk_t * const l_ltk,
                                     const twicIrk_t * const l_irk,
                                     const twicCsrk_t * const l_csrk,
                                     const uint8_t encryption_key_size);

TZ1K_INLINE twicStatus_t
twicIfLeSmMasBondingInformationNegativeReply(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfLeSmMasBondingState(twicConnIface_t * const iface,
                          const twicAuthInfo_t bits);
#endif

#if defined(TWIC_CONFIG_SM_RESPONDER)
TZ1K_INLINE twicStatus_t
twicIfLeSmSlvSecurityRequest(twicConnIface_t * const iface,
                             const bool auth_req_bonding,
                             const bool auth_req_mitm_protection);

TZ1K_INLINE twicStatus_t
twicIfLeSmSlvPairingConfirm(twicConnIface_t * const iface,
                            const twicSmpIoCapability_t io_capability,
                            const bool oob_data_present,
                            const bool auth_req_bonding,
                            const bool auth_req_mitm_protection,
                            const uint8_t max_enc_key_size,
                            const bool init_key_dist_enckey,
                            const bool init_key_dist_idkey,
                            const bool init_key_dist_sign,  
                            const bool resp_key_dist_enckey,
                            const bool resp_key_dist_idkey,
                            const bool resp_key_dist_sign);

TZ1K_INLINE twicStatus_t
twicIfLeSmSlvPairingFailed(twicConnIface_t * const iface,
                           const twicSmReasonCode_t reason);
  
TZ1K_INLINE twicStatus_t
twicIfLeSmSlvKbPasskeyEntryReply(twicConnIface_t * const iface,
                                 const twicPasskeyEntry_t *const passkey);

TZ1K_INLINE twicStatus_t
twicIfLeSmSlvKbPasskeyEntryNegativeReply(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfLeSmSlvDpPasskeyEntryReply(twicConnIface_t * const iface,
                                 const twicPasskeyEntry_t *const passkey);

TZ1K_INLINE twicStatus_t
twicIfLeSmSlvDpPasskeyEntryNegativeReply(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfLeSmSlvOobTkReply(twicConnIface_t * const iface,
                        const twicOobTk_t *const tk);

TZ1K_INLINE twicStatus_t
twicIfLeSmSlvOobTkNegativeReply(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfLeSmSlvBondingInformationReply(twicConnIface_t * const iface,
                                     const twicEdiv_t * const r_ediv,
                                     const twicRand_t * const r_rand,
                                     const twicLtk_t * const r_ltk,
                                     const twicIrk_t * const r_irk,
                                     const twicCsrk_t * const r_csrk,
                                     const twicEdiv_t * const l_ediv,
                                     const twicRand_t * const l_rand,
                                     const twicLtk_t * const l_ltk,
                                     const twicIrk_t * const l_irk,
                                     const twicCsrk_t * const l_csrk,
                                     const uint8_t encryption_key_size);

TZ1K_INLINE twicStatus_t
twicIfLeSmSlvBondingInformationNegativeReply(twicConnIface_t * const iface);

TZ1K_INLINE twicStatus_t
twicIfLeSmSlvBondingState(twicConnIface_t * const iface,
                          const twicAuthInfo_t bits);
#endif

#endif /* _TWIC_INTERFACE_H_ */
