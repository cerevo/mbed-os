/**
 * @file twic_interface_internal.h
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

#ifndef _TWIC_INTERFACE_INTERNAL_H_
#define _TWIC_INTERFACE_INTERNAL_H_

#include "tz1sm_config.h" /* system configuration */
#include "tz1ut_list.h"
#include "tz1sm_hal.h"
#include "twic_service.h"
#include "twic_hash.h"

/*
 * TWIC INTERFACE API INTERNAL
 *
 */

twicStatus_t _twicIfLeCePatch(
  twicEntry_t * const conn, const uint8_t fid, const uint8_t length,
  const uint8_t * const code, const uint8_t reg, const bool enable);

twicStatus_t _twicIfLeCeReadFwVer(twicEntry_t * const conn);

twicStatus_t _twicIfLeCeDbusRead(twicEntry_t * const conn, const uint8_t addr);

twicStatus_t _twicIfLeCeDbusWrite(
  twicEntry_t * const conn, const uint8_t addr, const uint16_t data);

twicStatus_t _twicIfLeCeMemoryRead(
  twicEntry_t * const conn, const uint32_t addr);

twicStatus_t _twicIfLeCeMemoryWrite(
  twicEntry_t * const conn, const uint32_t addr, const uint16_t data);

twicStatus_t _twicIfLeCeXoscTriming(
  twicEntry_t * const conn, const uint8_t data);

twicStatus_t _twicIfLeCeSetTxPower(twicEntry_t * const conn,
                                   const uint8_t dbm_idx);

twicStatus_t _twicIfLeCeFlowControl(
  twicEntry_t * const conn, const bool enable);

twicStatus_t _twicIfLeCeSetBaudrate(
  twicEntry_t * const conn, const twicTzbtBr_t br, const uint16_t pu);

twicStatus_t _twicIfLeCeHostDelay(
  twicEntry_t * const conn, const uint16_t delay);

twicStatus_t _twicIfLeCeLowPowerPrimarySetup(
  twicEntry_t * const conn, const uint16_t osc_tolerance,
  const uint16_t osc_jitter);

twicStatus_t _twicIfLeCeLowPowerClockSetup(
  twicEntry_t * const conn, twicStatus_t * const osc_state);

twicStatus_t _twicIfLeCeLowPowerMode(
  twicEntry_t * const conn, const bool idle, const bool discoverable,
  const bool conn_scan, const bool shutdown);

twicStatus_t _twicIfLeCeLowPowerDiscoverableSetup(twicEntry_t * const conn);

twicStatus_t _twicIfLeCeHkLowPowerInfo(twicCeLeLpInfo_t * const info);

twicStatus_t _twicIfLeCeHkLowPowerTransition(const bool enable);

#if defined(TZ1EM_POWER_PROFILE)
twicStatus_t _twicIfLeCeInitializeTz1em(void);

twicStatus_t _twicIfLeCeConfigureTz1em(
  const uint8_t type, tz1emRequirement_t * const gp);

twicStatus_t _twicIfLeCeWithdrawalFromTz1em(const uint8_t type);

twicStatus_t _twicIfLeCePermitTz1em(const uint8_t type);

void _twicIfLeCeHkTz1emInfo(const uint8_t type, tz1emInfo_t * const info);
#endif

twicStatus_t _twicIfLeCeHkTimeout(
  const bool cleanup, const uint16_t cycle, const uint8_t specified_fidx);

twicStatus_t _twicIfLeCeLowPowerControlPinSetup(
  twicEntry_t * const conn, const bool host_wake);

twicStatus_t _twicIfLeReadBdaddr(twicEntry_t * const conn);

#if defined(TWIC_API_LEREADLOCALVERSIONINFORMATION)
twicStatus_t _twicIfLeReadLocalVersionInformation(twicEntry_t * const conn);
#endif

twicStatus_t _twicIfLeWriteBdaddr(
  twicEntry_t * const conn, const uint64_t * const bd);

twicStatus_t _twicIfLeCeChangeToCm(twicEntry_t * const conn);

twicStatus_t _twicIfLeInitializeDevice(twicEntry_t * const conn);

twicStatus_t _twicIfLeGattClientStart(twicEntry_t * const conn);

twicStatus_t _twicIfLeConnectionUpdate(
  twicEntry_t * const conn, uint16_t conn_int_min,
  uint16_t conn_int_max, uint16_t slave_latency, uint16_t supervison_timeout,
  uint16_t min_ce_length, uint16_t max_ce_length);

#if defined(TWIC_BLE_HWIP_V41)
twicStatus_t _twicIfLeConnectionParameterRequestReply(
  twicEntry_t * const conn, uint16_t conn_int_min,
  uint16_t conn_int_max, uint16_t slave_latency, uint16_t supervison_timeout);
/* const uint8_t reason: LIST OF ERROR CODES [1] */
twicStatus_t _twicIfLeConnectionParameterRequestNegativeReply(
  twicEntry_t * const conn, const uint8_t reason);
#endif

twicStatus_t _twicIfLeDisconnect(
  twicEntry_t * const conn, const twicBdaddr_t * const bd);

twicStatus_t _twicIfLeGattServerStart(twicEntry_t * const conn);

twicStatus_t _twicIfLeReadTxPowerLevel(
  twicEntry_t * const conn, const uint8_t type);

twicStatus_t _twicIfLeReadRssi(twicEntry_t * const conn);

twicStatus_t _twicIfLeReadRemoteVersion(twicEntry_t * const conn);

twicStatus_t _twicIfLeCreateConnection(twicEntry_t * const conn,
                                       uint16_t interval,
                                       uint16_t window, bool use_white_list,
                                       bool peer_bdaddr_is_random,
                                       const twicBdaddr_t *const peer_bdaddr,
                                       uint16_t conn_int_min,
                                       uint16_t conn_int_max,
                                       uint16_t slave_latency,
                                       uint16_t supervison_timeout,
                                       uint16_t min_ce_length,
                                       uint16_t max_ce_length,
                                       bool own_bdaddr_is_random);

twicStatus_t _twicIfLeCreateConnectionCancel(twicEntry_t * const conn);
#if defined(TWIC_API_LELMGENRESOLVABLEBDADDR)
twicStatus_t _twicIfLeLmGenResolvableBdaddr(twicEntry_t * const conn);
#endif
#if defined(TWIC_API_LELMRESOLVEBDADDR)
twicStatus_t _twicIfLeLmResolveBdaddr(twicEntry_t *const conn,
                                      const twicBdaddr_t * const bd,
                                      const uint8_t num_of_irk,
                                      const twicIrk_t * const irks);
#endif
#if defined(TWIC_CONFIG_ENABLE_SCAN)
twicStatus_t _twicIfLeSetScanEnable(twicEntry_t *const conn,
                                    const uint16_t interval,
                                    const uint16_t window,
                                    const bool active_scanning,
                                    const bool own_bdaddr_is_random,
                                    const bool whitelist_enable,
                                    const bool scan_enable,
                                    const bool filter_duplicates);
twicStatus_t _twicIfLeSetScanDisable(twicEntry_t *const conn);
#endif
#if defined(TWIC_API_LEDELWHITELIST)
twicStatus_t _twicIfLeDelWhitelist(
  twicEntry_t *const conn, const bool random, const twicBdaddr_t *const bd);
#endif
#if defined(TWIC_API_LEADDWHITELIST)
twicStatus_t _twicIfLeAddWhitelist(
  twicEntry_t *const conn, const bool random, const twicBdaddr_t *const bd);
#endif
#if defined(TWIC_API_LECLEAREWHITELIST)
twicStatus_t _twicIfLeClearWhitelist(twicEntry_t *const conn);
#endif
#if defined(TWIC_API_LEREADWHITELISTSIZE)
twicStatus_t _twicIfLeReadWhitelistSize(twicEntry_t *const conn);
#endif
twicStatus_t _twicIfLeReadRemoteUsedFeatures(twicEntry_t *const conn);
#if defined(TWIC_API_LEREADLOCALSUPPORTEDFEATURES)
twicStatus_t _twicIfLeReadLocalSupportedFeatures(twicEntry_t *const conn);
#endif
#if defined(TWIC_API_LESETHOSTCHANNELCLASSIFICATION)
twicStatus_t _twicIfLeSetHostChannelClassification(
  twicEntry_t *const conn, const twicChannelMap_t *const channel_map);
#endif

#if defined(TWIC_API_LEREADCHANNELMAP)
twicStatus_t _twicIfLeReadChannelMap(twicEntry_t *const conn);
#endif
#if defined(TWIC_API_LEREADSUPPORTEDSTATES)
twicStatus_t _twicIfLeReadSupportedStates(twicEntry_t *const conn);
#endif
#if defined(TWIC_API_LESETRANDOMADDRESS)
twicStatus_t _twicIfLeSetRandomAddress(twicEntry_t *const conn,
                                       const twicBdaddr_t *const bd);
#endif
#if defined(TWIC_API_LESMSETIRVALUE)
twicStatus_t
_twicIfLeSmSetIrValue(twicEntry_t * const conn, const twicIr_t * const ir);
#endif
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
twicStatus_t
_twicIfLeSmEncrypt(twicEntry_t * const conn, const uint8_t * const key,
                   const uint8_t * const plain_text_data);
#endif
#if defined(TWIC_API_LELESETADVERTISINGDATA) && defined(TWIC_BLE_HWIP_V41)
twicStatus_t
_twicIfLeSetAdvertisingData(twicEntry_t * const conn,
                            const uint8_t advertising_data_length,
                            const uint8_t * const advertising_data);
#endif
#if defined(TWIC_CONFIG_SM_INITIATOR)
twicStatus_t _twicIfLeSmMasPairingRequest(
  twicEntry_t * const conn,
  const twicSmpIoCapability_t io_capability,
  const bool oob_data_present,
  const bool auth_req_bonding, const bool auth_req_mitm_protection,
  const uint8_t max_enc_key_size, const bool init_key_dist_enckey,
  const bool init_key_dist_idkey, const bool init_key_dist_sign,  
  const bool resp_key_dist_enckey, const bool resp_key_dist_idkey,
  const bool resp_key_dist_sign);

twicStatus_t _twicIfLeSmMasSecurityReject(
  twicEntry_t * const conn, const twicSmReasonCode_t reason);

twicStatus_t _twicIfLeSmMasPairingFailed(
  twicEntry_t * const conn, const twicSmReasonCode_t reason);

twicStatus_t _twicIfLeSmMasKbPasskeyReply(
  twicEntry_t * const conn, const twicPasskeyEntry_t *const passkey);

twicStatus_t _twicIfLeSmMasKbPasskeyNegativeReply(twicEntry_t * const conn);

twicStatus_t _twicIfLeSmMasDpPasskeyReply(
  twicEntry_t * const conn, const twicPasskeyEntry_t *const passkey);

twicStatus_t _twicIfLeSmMasDpPasskeyNegativeReply(twicEntry_t * const conn);

twicStatus_t _twicIfLeSmMasOobTkReply(
  twicEntry_t * const conn, const twicOobTk_t *const tk);

twicStatus_t _twicIfLeSmMasOobTkNegativeReply(twicEntry_t * const conn);

twicStatus_t _twicIfLeSmMasBondingInformationReply(
  twicEntry_t * const conn, const twicEdiv_t * const r_ediv,
  const twicRand_t * const r_rand, const twicLtk_t * const r_ltk,
  const twicIrk_t * const r_irk, const twicCsrk_t * const r_csrk,
  const twicEdiv_t * const l_ediv, const twicRand_t * const l_rand,
  const twicLtk_t * const l_ltk, const twicIrk_t * const l_irk,
  const twicCsrk_t * const l_csrk, const uint8_t encryption_key_size);

twicStatus_t
_twicIfLeSmMasBondingInformationNegativeReply(twicEntry_t * const conn);

twicStatus_t _twicIfLeSmMasBondingState(
  twicEntry_t * const conn, const twicAuthInfo_t bits);

twicStatus_t _twicIfLeSmMasStartEncryption(
  twicEntry_t * const conn, const twicEdiv_t *const ediv,
  const twicRand_t *const rand, const twicLtk_t *const ltk,
  const uint8_t encryption_key_size);
#endif /* TWIC_CONFIG_SM_INITIATOR */

#if defined(TWIC_CONFIG_SM_RESPONDER)
twicStatus_t _twicIfLeSmSlvPairingConfirm(
  twicEntry_t * const conn,
  const twicSmpIoCapability_t io_capability,
  const bool oob_data_present,
  const bool auth_req_bonding, const bool auth_req_mitm_protection,
  const uint8_t max_enc_key_size, const bool init_key_dist_enckey,
  const bool init_key_dist_idkey, const bool init_key_dist_sign,  
  const bool resp_key_dist_enckey, const bool resp_key_dist_idkey,
  const bool resp_key_dist_sign);

twicStatus_t _twicIfLeSmSlvPairingFailed(
  twicEntry_t * const conn, const twicSmReasonCode_t reason);

twicStatus_t _twicIfLeSmSlvSecurityRequest(
  twicEntry_t * const conn,
  const bool auth_req_bonding, const bool auth_req_mitm_protection);

twicStatus_t _twicIfLeSmSlvKbPasskeyReply(
  twicEntry_t * const conn, const twicPasskeyEntry_t *const passkey);

twicStatus_t _twicIfLeSmSlvKbPasskeyNegativeReply(twicEntry_t * const conn);

twicStatus_t _twicIfLeSmSlvDpPasskeyReply(
  twicEntry_t * const conn, const twicPasskeyEntry_t *const passkey);

twicStatus_t _twicIfLeSmSlvDpPasskeyNegativeReply(twicEntry_t * const conn);

twicStatus_t _twicIfLeSmSlvOobTkReply(
  twicEntry_t * const conn, const twicOobTk_t *const tk);

twicStatus_t _twicIfLeSmSlvOobTkNegativeReply(twicEntry_t * const conn);

twicStatus_t _twicIfLeSmSlvBondingInformationReply(
  twicEntry_t * const conn, const twicEdiv_t * const r_ediv,
  const twicRand_t * const r_rand, const twicLtk_t * const r_ltk,
  const twicIrk_t * const r_irk, const twicCsrk_t * const r_csrk,
  const twicEdiv_t * const l_ediv, const twicRand_t * const l_rand,
  const twicLtk_t * const l_ltk, const twicIrk_t * const l_irk,
  const twicCsrk_t * const l_csrk, const uint8_t encryption_key_size);

twicStatus_t
_twicIfLeSmSlvBondingInformationNegativeReply(twicEntry_t * const conn);

twicStatus_t _twicIfLeSmSlvBondingState(
  twicEntry_t * const conn, const twicAuthInfo_t bits);
#endif /* TWIC_CONFIG_SM_RESPONDER */

twicStatus_t _twicIfLeGattBeginServiceCreation(
  twicEntry_t * const entry,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);

twicStatus_t _twicIfLeGattBeginSecondaryServiceCreation(
  const twicEntry_t * const service, twicEntry_t * const entry,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);

twicStatus_t _twicIfLeGattIncludeServiceCreation(
  twicEntry_t * const service, twicEntry_t * const inclusion,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);

twicStatus_t _twicIfLeGattAddCharacteristics(
  const twicEntry_t * const service, twicEntry_t * const entry,
  const uint8_t properties, const uint64_t uuid_lsb,
  const uint64_t uuid_msb, const uint8_t uuid_len);

twicStatus_t _twicIfLeGattSetCharacteristics(twicEntry_t * const entity,
                                             const bool db_init,
                                             const uint16_t permissions,
                                             const uint16_t length,
                                             const uint8_t * const value,
                                             const uint64_t uuid_lsb,
                                             const uint64_t uuid_msb,
                                             const uint8_t uuid_len);
#if defined(TWIC_API_LEGATTDBSETCHARACTERISTICSVL) || \
  defined(TWIC_API_LEGATTDBUPDCHARACTERISTICSVL)
twicStatus_t _twicIfLeGattSetCharacteristicsVl(twicEntry_t * const entity,
                                               const bool db_init,
                                               const uint16_t permissions,
                                               const uint16_t max_length,
                                               const uint16_t length,
                                               const uint8_t * const value,
                                               const uint64_t uuid_lsb,
                                               const uint64_t uuid_msb,
                                               const uint8_t uuid_len);
#endif
twicStatus_t _twicIfLeGattSetDescriptor(const twicEntry_t * const entity,
                                        twicEntry_t * const entry,
                                        const uint16_t permissions,
                                        const uint16_t length,
                                        const uint8_t * const value,
                                        const uint64_t uuid_lsb,
                                        const uint64_t uuid_msb,
                                        const uint8_t uuid_len);
#if defined(TWIC_LEGATTDBSETDESCRIPTORVL)
twicStatus_t _twicIfLeGattSetDescriptorVl(const twicEntry_t * const entity,
                                          twicEntry_t * const entry,
                                          const uint16_t permissions,
                                          const uint16_t max_length,
                                          const uint16_t length,
                                          const uint8_t * const value,
                                          const uint64_t uuid_lsb,
                                          const uint64_t uuid_msb,
                                          const uint8_t uuid_len);
#endif
twicStatus_t _twicIfLeEndServiceCreation(twicEntry_t * const service);
#if defined(TWIC_API_LEGATTDBSETPERMISSIONS)
twicStatus_t _twicIfLeGattDbSetPermissions(twicEntry_t * const entity,
                                           const uint16_t permissions);
#endif
#if defined(TWIC_API_LEGATTDBGETPERMISSIONS)
twicStatus_t _twicIfLeGattDbGetPermissions(twicEntry_t * const entity);
#endif
#if defined(TWIC_API_GATTCLIENTEXGMTU)
twicStatus_t
_twicIfGattClientExgMtu(twicEntry_t * const conn, const uint16_t rx_mtu_size);
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICE) ||             \
  defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID) ||  \
  defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE) ||                  \
  defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS) ||           \
  defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)||         \
  defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS)
twicStatus_t _twicIfLeGattClientDiscovery(
  twicEntry_t * const conn, const uint8_t seq,
  const uint16_t start_handle, const uint16_t end_handle,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
#endif
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE) ||    \
  defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID) ||  \
  defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR)
twicStatus_t _twicIfLeGattClientRead(
  twicEntry_t * const conn, const uint8_t seq,
  const uint16_t start_handle, const uint16_t end_handle,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICVALUE) ||    \
  defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR) || \
  defined(TWIC_API_GATTCLIENTWRITEWITHOUTRESPONSE)
twicStatus_t _twicIfLeGattClientWrite(
  twicEntry_t * const conn, const uint8_t seq,
  const uint16_t handle, const uint8_t length, const uint8_t * const data);
#endif
#if defined(TWIC_API_GATTCLIENTSIGNEDWRITEWITHOUTRESPONSE)
twicStatus_t _twicIfLeGattClientSignedWrite(
  twicEntry_t * const conn,
  const uint16_t handle, const uint8_t length, const uint8_t * const data);
#endif
#if defined(TWIC_API_GATTCLIENTREADMULTIPLECHARVALUES)
twicStatus_t _twicIfLeGattClientReadMultipleCharValues(
  twicEntry_t * const conn,
  const uint8_t number, const uint16_t * const handles);
#endif
#if defined(TWIC_API_GATTCLIENTRELIABLEWRITE)
twicStatus_t _twicIfLeGattClientReliableWrite(
  twicEntry_t * const conn, const uint16_t total,
  const twicReliableWritein_t * const characteristics_value);
#endif
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE) ||  \
  defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR)
twicStatus_t _twicIfLeGattClientLongRead(
  twicEntry_t * const conn, const uint8_t seq,
  const uint16_t handle, const uint16_t offset);
#endif
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICVALUE) || \
  defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR)
twicStatus_t _twicIfLeGattClientLongWrite(
  twicEntry_t * const conn, const uint8_t seq,
  const uint16_t handle, const uint16_t offset,
  const uint16_t length, const uint8_t * const value);
#endif
#if defined(TWIC_API_GATTCLIENTINDICATIONCONFIRMATIONRESPONSE)
twicStatus_t _twicIfLeGattClientResponse(
  twicEntry_t * const conn, const uint8_t seq, const uint8_t status);
#endif

twicStatus_t _twicIfLeDiscoverable(
  twicEntry_t * const connection, const uint16_t min_interval,
  const uint16_t max_interval, const uint8_t advertising_type,
  const uint8_t own_address_type, const uint8_t direct_address_type,
  const uint64_t direct_address, const uint8_t advertising_channel_map,
  const uint8_t advertising_filter_policy,
  const uint8_t advertising_data_length,
  const uint8_t * const advertising_data,
  const uint8_t scan_resp_data_length, const uint8_t * const scan_resp_data);

twicStatus_t _twicIfLeStopAdvertising(twicEntry_t * const conn);

twicStatus_t _twicIfGattNotification(
  twicEntry_t * const conn, const twicEntry_t * const cha,
  const uint8_t length, const uint8_t * const data);

twicStatus_t _twicIfGattIndication(
  twicEntry_t * const conn, const twicEntry_t * const cha,
  const uint8_t length, const uint8_t * const data);

twicStatus_t _twicIfGattServerWriteElement(
  const uint8_t seq, twicEntry_t * const conn,
  const twicEntry_t * const entity, const uint16_t length,
  const uint8_t * const value);

/* Server APP has to call twicIfGattServerExgMtuResponse
 * after the twicCbExgMtuDemand invocation. */
twicStatus_t _twicIfGattServerExgMtuResponse(
  twicEntry_t * const conn, const uint8_t status, const uint16_t rx_mtu_size);

twicStatus_t _twicIfGattServerAttResponse(
  twicEntry_t * const conn, const uint8_t seq,
  const twicEntry_t * const entity, const uint8_t status);

/* Server APP has to call twicIfGattServerCharValMultiReadOutResponse
 * after the twicCbCharValMultiReadOutDemand incocation. */
twicStatus_t _twicIfGattServerCharValMultiReadOutResponse(
  twicEntry_t *const conn, const uint8_t status, const twicEntry_t *const cha);

#if defined(TWIC_INTERFACE_MACRO_FUNC)

#include "twic_interface_macro.h"

#else

/* @brief Place the Patch Base Program.
 * This API setups the Patch Swap Target Program.
 *
 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint8_t length
 * The argument 'code' length of the octets.
 * @param const uint8_t * const code
 * The program.
 *
 * @return TWIC_STATUS_OK Success
 * @return TWIC_STATUS_ERROR_RESOURCE Check the "tz1sm_hal.[ch]" porting.
 * @return TWIC_STATUS_OPERATION_NOT_PERMITTED This API cannot be
 * invoked at the context.
 * @return TWIC_STATUS_ERROR_PARAMETER	Argument Error.
 * @return TWIC_STATUS_ERROR_TZ1EM	Failure in the TZ1EM. */
static TZ1K_INLINE twicStatus_t
twicIfLeCePatchBase(twicConnIface_t * const iface, const uint8_t length,
                     const uint8_t * const code)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCePatch(&iface->conn,
                             TWIC_LECEPATCHBASE, length, code, 0, false);
  return __ret;
}

/* @brief Place the Patch Program.
 * This API setups the Patch Program.
 *
 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint8_t length
 * The argument 'code' length of the octets.
 * @param const uint8_t * const code
 * The program.
 *
 * @return TWIC_STATUS_OK Success
 * @return TWIC_STATUS_ERROR_RESOURCE Check the "tz1sm_hal.[ch]" porting.
 * @return TWIC_STATUS_OPERATION_NOT_PERMITTED This API cannot be
 * invoked at the context.
 * @return TWIC_STATUS_ERROR_PARAMETER	Argument Error.
 * @return TWIC_STATUS_ERROR_TZ1EM	Failure in the TZ1EM. */
static TZ1K_INLINE twicStatus_t
twicIfLeCePatchWrite(twicConnIface_t * const iface, const uint8_t length,
                     const uint8_t * const code)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCePatch(&iface->conn,
                             TWIC_LECEPATCHWRITE, length, code, 0, false);
  return __ret;
}

/* @brief Enable the Patch Program.
 * This API enables the Patch Program.
 *
 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const bool enable
 * true: Enable the Patch Program, false: Disable the Patch Program.
 *
 * @return TWIC_STATUS_OK Success
 * @return TWIC_STATUS_ERROR_RESOURCE Check the "tz1sm_hal.[ch]" porting.
 * @return TWIC_STATUS_OPERATION_NOT_PERMITTED This API cannot be
 * invoked at the context.
 * @return TWIC_STATUS_ERROR_PARAMETER	Argument Error.
 * @return TWIC_STATUS_ERROR_TZ1EM	Failure in the TZ1EM. */
static TZ1K_INLINE twicStatus_t
twicIfLeCePatchControl(twicConnIface_t * const iface, const uint8_t reg,
                       const bool enable)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCePatch(&iface->conn,
                             TWIC_LECEPATCHCONTROL, 0, NULL, reg, enable);
  return __ret;
}

#if defined(TWIC_API_LECEDBUSREAD)
static TZ1K_INLINE twicStatus_t
twicIfLeCeDbusRead(twicConnIface_t * const iface, const uint8_t addr)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req)) __ret = _twicIfLeCeDbusRead(&iface->conn, addr);
  return __ret;
}
#endif

#if defined(TWIC_API_LECEREADFWVER)
static TZ1K_INLINE twicStatus_t
twicIfLeCeReadFwVer(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req)) __ret = _twicIfLeCeReadFwVer(&iface->conn);
  return __ret;
}
#endif
  
#if defined(TWIC_API_LECEDBUSWRITE)
static TZ1K_INLINE twicStatus_t
twicIfLeCeDbusWrite(twicConnIface_t * const iface, const uint8_t addr,
                    const uint16_t data)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeDbusWrite(&iface->conn, addr, data);
  return __ret;
}
#endif

#if defined(TWIC_API_LECEMEMORYREAD)
static TZ1K_INLINE twicStatus_t
twicIfLeCeMemoryRead(twicConnIface_t * const iface, const uint32_t addr)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req)) __ret = _twicIfLeCeMemoryRead(&iface->conn, addr);
  return __ret;
}
#endif

#if defined(TWIC_API_LECEMEMORYWRITE)
static TZ1K_INLINE twicStatus_t
twicIfLeCeMemoryWrite(twicConnIface_t * const iface, const uint32_t addr,
                      const uint16_t data)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeMemoryWrite(&iface->conn, addr, data);
  return __ret;
}
#endif

#if defined(TWIC_API_LECEXOSCTRIMING)
static TZ1K_INLINE twicStatus_t
twicIfLeCeXoscTriming(twicConnIface_t * const iface, const uint8_t data)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeXoscTriming(&iface->conn, data);
  return __ret;
}
#endif

#if defined(TWIC_API_LECESETTXPOWER)
/* 0x00: Reserved,   0x01: 0dBm,   0x02: -4dBm,   0x03: -8dBm,
   0x04: -12dBm,   0x05: -16dBm,   0x06: -20dBm */
static TZ1K_INLINE twicStatus_t
twicIfLeCeSetTxPower(twicConnIface_t * const iface, const uint8_t dbm_idx)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeSetTxPower(&iface->conn, dbm_idx);
  return __ret;
}
#endif

static TZ1K_INLINE twicStatus_t
twicIfLeCeFlowControl(twicConnIface_t * const iface, const bool enable)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeFlowControl(&iface->conn, enable);
  return __ret;
}

/* Setup the baud rate and punctuation of the TZ1BT.
   The default baud rate is 115.2k can be changed before the
   controller's low power settings are configured.
   The maximum baud rate depends on the voltage mode of the MCU.
   The interval of 100ms is necessary for the subsequent call of any API. */
static TZ1K_INLINE twicStatus_t
twicIfLeCeSetBaudrate(twicConnIface_t * const iface, const twicTzbtBr_t br,
                      const uint16_t pu)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeSetBaudrate(&iface->conn, br, pu);
  return __ret;
}

/* Setup of the delay time when the controller gets the HOST up.
 * Assign 0x000A to the delay if TZ1EM is used. */
static TZ1K_INLINE twicStatus_t
twicIfLeCeHostDelay(twicConnIface_t * const iface, const uint16_t delay)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeHostDelay(&iface->conn, delay);
  return __ret;
}

#if defined(TWIC_LECE_LOWPOWER)
static TZ1K_INLINE twicStatus_t
twicIfLeCeLowPowerPrimarySetup(twicConnIface_t * const iface,
                               const uint16_t osc_tolerance,
                               const uint16_t osc_jitter)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeLowPowerPrimarySetup(
      &iface->conn, osc_tolerance, osc_jitter);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeCeLowPowerClockSetup(twicConnIface_t * const iface,
                             twicStatus_t * const osc_state)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeLowPowerClockSetup(&iface->conn, osc_state);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeCeLowPowerControlPinSetup(twicConnIface_t * const iface,
                                  const bool host_wake)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeLowPowerControlPinSetup(&iface->conn, host_wake);
  return __ret;
}

/* conn_scan must be disabled when the ROM5 is the scanner or GAP Central. */
static TZ1K_INLINE twicStatus_t twicIfLeCeLowPowerMode(
  twicConnIface_t * const iface, const bool idle, const bool discoverable,
  const bool conn_scan, const bool shutdown)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeLowPowerMode(
      &iface->conn, idle, discoverable, conn_scan, shutdown);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeCeLowPowerDiscoverableSetup(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeLowPowerDiscoverableSetup(&iface->conn);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeCeHkLowPowerInfo(twicCeLeLpInfo_t * const info)
{
  return _twicIfLeCeHkLowPowerInfo(info);
}

static TZ1K_INLINE twicStatus_t
twicIfLeCeHkLowPowerTransition(const bool enable)
{
  return _twicIfLeCeHkLowPowerTransition(enable);
}

#endif /* TWIC_LECE_LOWPOWER */

#if defined(TZ1EM_POWER_PROFILE)
static TZ1K_INLINE twicStatus_t twicIfLeCeInitializeTz1em(void)
{
  return _twicIfLeCeInitializeTz1em();
}

/* @brief Configure the requirements of the TZ1EM.
 * The designated period of this requirement is used for the term of
 * which either the Non-connectable Advertiser.
 *
 * If the "tz1emGoIntoTheShade" API is invoked, the HOST may be
 * inactivated when the BLE Controller allows the HOST to repose.
 *
 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param tz1emVf_t sunshine_vf
 * Assign TZ1EM the voltage mode for the discoverable mode during the
 * active state of HOST.
 * @param tz1emOm_t mode
 * Assign TZ1EM the power operation mode for the discoverable mode
 * during the in-active state of HOST. 'TZ1EM_OP_BLE_DOZE_WRET' can be
 * used if 'twicIfLeCeHostDelay' is used with 0x00A0.
 * @param tz1emWe_t e1,e2
 * Assign TZ1EM each event which activates the HOST for the
 * discoverable mode during the in-active state of HOST.
 * @param tz1emWf_t f1,f2
 * Assign TZ1EM the factors of each event.
 * @param tz1emCallback_t cb
 * 'cb' will be invoked when the HOST wakes up.
 *
 * @return TWIC_STATUS_OK Success
 * @return TWIC_STATUS_ERROR_RESOURCE Check the "tz1sm_hal.[ch]" porting.
 * @return TWIC_STATUS_OPERATION_NOT_PERMITTED This API cannot be
 * invoked at the context.
 * @return TWIC_STATUS_ERROR_PARAMETER	Argument Error.
 * @return TWIC_STATUS_ERROR_TZ1EM	Failure in the TZ1EM. */
static TZ1K_INLINE twicStatus_t
twicIfLeCeNcAdvConfigureTz1em(tz1emVf_t sunshine_vf, tz1emOm_t mode,
                              tz1emWe_t e1, tz1emWf_t f1,
                              tz1emWe_t e2, tz1emWf_t f2, tz1emCallback_t cb)
{
  tz1emRequirement_t condition;

  if (TZ1EM_VF_LO == sunshine_vf) return TWIC_STATUS_ERROR_PARAMETER;
  condition.sunshine_vf = sunshine_vf;
  condition.mode = mode;
  if (TZ1EM_WF_RTC == f1) e1 = TZ1EM_WE_EP;
  if (TZ1EM_WF_RTC == f2) e2 = TZ1EM_WE_EP;  
  condition.trigger[0].event = e1; condition.trigger[0].factor = f1;
  condition.trigger[1].event = e2; condition.trigger[1].factor = f2;
  condition.wakeup = cb;

  return _twicIfLeCeConfigureTz1em(TWIC_TZ1EM_NC_ADV, &condition);
}

/* @brief Configure the requirements of the TZ1EM.
 * The designated period of this requirement is used for the term of
 * which either the Connectable Advertiser.
 *
 * If the "tz1emGoIntoTheShade" API is invoked, the HOST may be
 * inactivated when the BLE Controller allows the HOST to repose.
 *
 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param tz1emVf_t sunshine_vf
 * Assign TZ1EM the voltage mode for the discoverable mode during the
 * active state of HOST.
 * @param tz1emOm_t mode
 * Assign TZ1EM the power operation mode for the discoverable mode
 * during the in-active state of HOST. 'TZ1EM_OP_BLE_DOZE_WRET' can be
 * used if 'twicIfLeCeHostDelay' is used with 0x00A0.
 * @param tz1emWe_t e1,e2
 * Assign TZ1EM each event which activates the HOST for the
 * discoverable mode during the in-active state of HOST.
 * @param tz1emWf_t f1,f2
 * Assign TZ1EM the factors of each event.
 * @param tz1emCallback_t cb
 * 'cb' will be invoked when the HOST wakes up.
 *
 * @return TWIC_STATUS_OK Success
 * @return TWIC_STATUS_ERROR_RESOURCE Check the "tz1sm_hal.[ch]" porting.
 * @return TWIC_STATUS_OPERATION_NOT_PERMITTED This API cannot be
 * invoked at the context.
 * @return TWIC_STATUS_ERROR_PARAMETER	Argument Error.
 * @return TWIC_STATUS_ERROR_TZ1EM	Failure in the TZ1EM. */
static TZ1K_INLINE twicStatus_t
twicIfLeCeCnAdvConfigureTz1em(tz1emVf_t sunshine_vf, tz1emOm_t mode,
                              tz1emWe_t e1, tz1emWf_t f1,
                              tz1emWe_t e2, tz1emWf_t f2, tz1emCallback_t cb)
{
  tz1emRequirement_t condition;
  
  if (TZ1EM_VF_LO == sunshine_vf) return TWIC_STATUS_ERROR_PARAMETER;
  condition.sunshine_vf = sunshine_vf;
  condition.mode = mode;
  if (TZ1EM_WF_RTC == f1) e1 = TZ1EM_WE_EP;
  if (TZ1EM_WF_RTC == f2) e2 = TZ1EM_WE_EP;  
  condition.trigger[0].event = e1; condition.trigger[0].factor = f1;
  condition.trigger[1].event = e2; condition.trigger[1].factor = f2;
  condition.wakeup = cb;
  
  return _twicIfLeCeConfigureTz1em(TWIC_TZ1EM_CN_ADV, &condition);
}

/* @brief Configure the requirements of the TZ1EM.
 * The designated period of this requirement is used for the term of
 * the Idle. The Idle is the state except the Scanner, Advertiser and
 * Connection.
 *
 * If the "tz1emGoIntoTheShade" API is invoked, the HOST may be
 * inactivated when the BLE Controller allows the HOST to repose.
 *
 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param tz1emVf_t sunshine_vf
 * Assign TZ1EM the voltage mode for the discoverable mode during the
 * active state of HOST.
 * @param tz1emOm_t mode
 * Assign TZ1EM the power operation mode for the discoverable mode
 * during the in-active state of HOST. 'TZ1EM_OP_BLE_DOZE_WRET' can be
 * used if 'twicIfLeCeHostDelay' is used with 0x00A0.
 * @param tz1emWe_t e1,e2
 * Assign TZ1EM each event which activates the HOST for the
 * discoverable mode during the in-active state of HOST.
 * @param tz1emWf_t f1,f2
 * Assign TZ1EM the factors of each event.
 * @param tz1emCallback_t cb
 * 'cb' will be invoked when the HOST wakes up.
 *
 * @return TWIC_STATUS_OK Success
 * @return TWIC_STATUS_ERROR_RESOURCE Check the "tz1sm_hal.[ch]" porting.
 * @return TWIC_STATUS_OPERATION_NOT_PERMITTED This API cannot be
 * invoked at the context.
 * @return TWIC_STATUS_ERROR_PARAMETER	Argument Error.
 * @return TWIC_STATUS_ERROR_TZ1EM	Failure in the TZ1EM. */
static TZ1K_INLINE twicStatus_t
twicIfLeCeIdleConfigureTz1em(tz1emVf_t sunshine_vf, tz1emOm_t mode,
                             tz1emWe_t e1, tz1emWf_t f1,
                             tz1emWe_t e2, tz1emWf_t f2, tz1emCallback_t cb)
{
  tz1emRequirement_t condition;
  
  if (TZ1EM_VF_LO == sunshine_vf) return TWIC_STATUS_ERROR_PARAMETER;
  condition.sunshine_vf = sunshine_vf;
  condition.mode = mode;
  if (TZ1EM_WF_RTC == f1) e1 = TZ1EM_WE_EP;
  if (TZ1EM_WF_RTC == f2) e2 = TZ1EM_WE_EP;  
  condition.trigger[0].event = e1; condition.trigger[0].factor = f1;
  condition.trigger[1].event = e2; condition.trigger[1].factor = f2;
  condition.wakeup = cb;
  
  return _twicIfLeCeConfigureTz1em(TWIC_TZ1EM_IDLE, &condition);
}

/* @brief Configure the requirements of the TZ1EM.
 * Bluetooth LE Controller extension.
 * Setup the HOST's Low Power Consumption requirements of the term
 * during the connection.
 * The designated period of this requirement is used for the term of
 * which either the Central or the Peripheral has a connection.
 *
 * If the "tz1emGoIntoTheShade" API is invoked, the HOST may be
 * inactivated when the BLE Controller allows the HOST to repose.
 *
 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param tz1emVf_t sunshine_vf
 * Assign TZ1EM the voltage mode for the discoverable mode during the
 * active state of HOST.
 * @param tz1emOm_t mode
 * Assign TZ1EM the power operation mode for the discoverable mode
 * during the in-active state of HOST. 'TZ1EM_OP_BLE_DOZE_WRET' can be
 * used if 'twicIfLeCeHostDelay' is used with 0x00A0.
 * @param tz1emWe_t e1,e2
 * Assign TZ1EM each event which activates the HOST for the
 * discoverable mode during the in-active state of HOST.
 * @param tz1emWf_t f1,f2
 * Assign TZ1EM the factors of each event.
 * @param tz1emCallback_t cb
 * 'cb' will be invoked when the HOST wakes up.
 *
 * @return TWIC_STATUS_OK Success
 * @return TWIC_STATUS_ERROR_RESOURCE Check the "tz1sm_hal.[ch]" porting.
 * @return TWIC_STATUS_OPERATION_NOT_PERMITTED This API cannot be
 * invoked at the context.
 * @return TWIC_STATUS_ERROR_PARAMETER	Argument Error.
 * @return TWIC_STATUS_ERROR_TZ1EM	Failure in the TZ1EM. */
static TZ1K_INLINE twicStatus_t
twicIfLeCeConnConfigureTz1em(tz1emVf_t sunshine_vf, tz1emOm_t mode,
                             tz1emWe_t e1, tz1emWf_t f1,
                             tz1emWe_t e2, tz1emWf_t f2, tz1emCallback_t cb)
{
  tz1emRequirement_t condition;
  
  if (TZ1EM_VF_LO == sunshine_vf) return TWIC_STATUS_ERROR_PARAMETER;
  condition.sunshine_vf = sunshine_vf;
  condition.mode = mode;
  if (TZ1EM_WF_RTC == f1) e1 = TZ1EM_WE_EP;
  if (TZ1EM_WF_RTC == f2) e2 = TZ1EM_WE_EP;  
  condition.trigger[0].event = e1; condition.trigger[0].factor = f1;
  condition.trigger[1].event = e2; condition.trigger[1].factor = f2;
  condition.wakeup = cb;
  
  return _twicIfLeCeConfigureTz1em(TWIC_TZ1EM_CONN, &condition);
}


static TZ1K_INLINE void twicIfLeCeNcAdvWithdrawalFromTz1em(void)
{
  _twicIfLeCeWithdrawalFromTz1em(TWIC_TZ1EM_NC_ADV);
}

static TZ1K_INLINE void twicIfLeCeCnAdvWithdrawalFromTz1em(void)
{
  _twicIfLeCeWithdrawalFromTz1em(TWIC_TZ1EM_CN_ADV);
}

static TZ1K_INLINE void twicIfLeCeIdleWithdrawalFromTz1em(void)
{
  _twicIfLeCeWithdrawalFromTz1em(TWIC_TZ1EM_IDLE);
}

static TZ1K_INLINE void twicIfLeCeConnWithdrawalFromTz1em(void)
{
  _twicIfLeCeWithdrawalFromTz1em(TWIC_TZ1EM_CONN);
}

static TZ1K_INLINE void twicIfLeCeNcAdvPermitTz1em(void)
{
  _twicIfLeCePermitTz1em(TWIC_TZ1EM_NC_ADV);
}

static TZ1K_INLINE void twicIfLeCeCnAdvPermitTz1em(void)
{
  _twicIfLeCePermitTz1em(TWIC_TZ1EM_CN_ADV);
}

static TZ1K_INLINE void twicIfLeCeIdlePermitTz1em(void)
{
  _twicIfLeCePermitTz1em(TWIC_TZ1EM_IDLE);
}

static TZ1K_INLINE void twicIfLeCeConnPermitTz1em(void)
{
  _twicIfLeCePermitTz1em(TWIC_TZ1EM_CONN);
}

static TZ1K_INLINE void twicIfLeCeHkTz1emNcAdvInfo(tz1emInfo_t * const info)
{
  _twicIfLeCeHkTz1emInfo(TWIC_TZ1EM_NC_ADV, info);
}

static TZ1K_INLINE void twicIfLeCeHkTz1emCnAdvInfo(tz1emInfo_t * const info)
{
  _twicIfLeCeHkTz1emInfo(TWIC_TZ1EM_CN_ADV, info);
}

static TZ1K_INLINE void twicIfLeCeHkTz1emIdleInfo(tz1emInfo_t * const info)
{
  _twicIfLeCeHkTz1emInfo(TWIC_TZ1EM_IDLE, info);
}

static TZ1K_INLINE void twicIfLeCeHkTz1emConnInfo(tz1emInfo_t * const info)
{
  _twicIfLeCeHkTz1emInfo(TWIC_TZ1EM_CONN, info);
}

#endif /* TZ1EM_POWER_PROFILE */

static TZ1K_INLINE twicStatus_t twicIfLeCeHkTryToUnlock(void)
{
  return _twicIfLeCeHkTimeout(true, 0, 0);
}

static TZ1K_INLINE twicStatus_t
twicIfLeCeHkTimeout(const uint16_t cycle, const uint8_t specified_fidx)
{
  return _twicIfLeCeHkTimeout(false, cycle, specified_fidx);
}

static TZ1K_INLINE twicStatus_t
twicIfLeReadBdaddr(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req)) __ret = _twicIfLeReadBdaddr(&iface->conn);
  return __ret;
}

#if defined(TWIC_API_LEREADLOCALVERSIONINFORMATION)
/* HCI_Version        : 1 byte
 * HCI_Revision       : 2 byte
 * LMP/PAL_Version    : 1 byte
 * Manufacturer_Name  : 2 byte
 * LMP/PAL_Subversion : 2 byte */
static TZ1K_INLINE twicStatus_t
twicIfLeReadLocalVersionInformation(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req)) __ret = _twicIfLeReadLocalVersionInformation(
    &iface->conn);
  return __ret;
}
#endif

static TZ1K_INLINE twicStatus_t
twicIfLeWriteBdaddr(twicConnIface_t * const iface,
                    const uint64_t * const bdaddr)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req)) __ret = _twicIfLeWriteBdaddr(&iface->conn, bdaddr);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeDisconnect(twicConnIface_t * const iface, const twicBdaddr_t * const bd)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req)) __ret = _twicIfLeDisconnect(&iface->conn, bd);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeConnectionUpdate(twicConnIface_t * const iface, uint16_t conn_int_min,
                         uint16_t conn_int_max, uint16_t slave_latency,
                         uint16_t supervison_timeout, uint16_t min_ce_length,
                         uint16_t max_ce_length)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeConnectionUpdate(&iface->conn, conn_int_min, conn_int_max,
                                      slave_latency, supervison_timeout,
                                      min_ce_length, max_ce_length);
  return __ret;
}

#if defined(TWIC_BLE_HWIP_V41)
static TZ1K_INLINE twicStatus_t
twicIfLeConnectionParameterRequestReply(twicConnIface_t * const iface,
                                        uint16_t conn_int_min,
                                        uint16_t conn_int_max,
                                        uint16_t slave_latency,
                                        uint16_t supervison_timeout)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeConnectionParameterRequestReply(
      &iface->conn, conn_int_min, conn_int_max, slave_latency,
      supervison_timeout);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeConnectionParameterRequestNegativeReply(twicConnIface_t * const iface,
                                                const uint8_t reason)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeConnectionParameterRequestNegativeReply(
      &iface->conn, reason);
  return __ret;
}
#endif

/* The interval of 30ms is necessary for the subsequent call of any API. */
static TZ1K_INLINE twicStatus_t
twicIfLeCeChangeToCm(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCeChangeToCm(&iface->conn);
  return __ret;
}

/* The interval between ChangeToCm and InitializeDevice needs 30ms. */
static TZ1K_INLINE twicStatus_t
twicIfLeInitializeDevice(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeInitializeDevice(&iface->conn);
  return __ret;
}
#if defined(TWIC_API_LEDELWHITELIST)
static TZ1K_INLINE twicStatus_t
twicIfLeDelWhitelist(twicConnIface_t * const iface, const bool random,
                     const twicBdaddr_t *const bd)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeDelWhitelist(&iface->conn, random, bd);
  return __ret;
}
#endif
#if defined(TWIC_CONFIG_ENABLE_SCAN)
/* whitelist_enable:false
   Accept all advertisement packets (default).
   whitelist_enable:true
   Ignore advertisement packets from devices not in the White List Only. */
static TZ1K_INLINE twicStatus_t
twicIfLeSetScanEnable(twicConnIface_t * const iface, const uint16_t interval,
                      const uint16_t window, const bool active_scanning,
                      const bool own_bdaddr_is_random,
                      const bool whitelist_enable, const bool filter_duplicates)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSetScanEnable(&iface->conn, interval, window,
                                   active_scanning, own_bdaddr_is_random,
                                   whitelist_enable, true, filter_duplicates);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeSetScanDisable(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSetScanDisable(&iface->conn);
  return __ret;
}
#endif /* TWIC_CONFIG_ENABLE_SCAN */
#if defined(TWIC_API_LECLEAREWHITELIST)
static TZ1K_INLINE twicStatus_t
twicIfLeClearWhitelist(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeClearWhitelist(&iface->conn);
  return __ret;
}
#endif
#if defined(TWIC_API_LEADDWHITELIST)
static TZ1K_INLINE twicStatus_t
twicIfLeAddWhitelist(twicConnIface_t * const iface, const bool random,
                     const twicBdaddr_t *const bd)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeAddWhitelist(&iface->conn, random, bd);
  return __ret;
}
#endif
#if defined(TWIC_API_LEREADWHITELISTSIZE)
static TZ1K_INLINE twicStatus_t
twicIfLeReadWhitelistSize(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeReadWhitelistSize(&iface->conn);
  return __ret;
}
#endif
static TZ1K_INLINE twicStatus_t
twicIfLeReadRemoteUsedFeatures(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeReadRemoteUsedFeatures(&iface->conn);
  return __ret;
}
#if defined(TWIC_API_LEREADLOCALSUPPORTEDFEATURES)
static TZ1K_INLINE twicStatus_t
twicIfLeReadLocalSupportedFeatures(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeReadLocalSupportedFeatures(&iface->conn);
  return __ret;
}
#endif
#if defined(TWIC_API_LESETHOSTCHANNELCLASSIFICATION)
static TZ1K_INLINE twicStatus_t
twicIfLeSetHostChannelClassification(twicConnIface_t * const iface,
                                     const twicChannelMap_t *const channel_map)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSetHostChannelClassification(&iface->conn, channel_map);
  return __ret;
}
#endif

#if defined(TWIC_API_LEREADCHANNELMAP)
static TZ1K_INLINE twicStatus_t
twicIfLeReadChannelMap(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeReadChannelMap(&iface->conn);
  return __ret;
}
#endif
#if defined(TWIC_API_LEREADSUPPORTEDSTATES)
static TZ1K_INLINE twicStatus_t
twicIfLeReadSupportedStates(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeReadSupportedStates(&iface->conn);
  return __ret;
}
#endif
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientStart(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientStart(&iface->conn);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeGattServerStart(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattServerStart(&iface->conn);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeReadTxPowerLevel(twicConnIface_t * const iface, const uint8_t type)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeReadTxPowerLevel(&iface->conn, type);
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeReadRssi(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeReadRssi(&iface->conn);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeReadRemoteVersion(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeReadRemoteVersion(&iface->conn);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeCreateConnection(twicConnIface_t * const iface, uint16_t interval,
                         uint16_t window, bool use_white_list,
                         bool peer_bdaddr_is_random,
                         const twicBdaddr_t *const peer_bdaddr,
                         uint16_t conn_int_min, uint16_t conn_int_max,
                         uint16_t slave_latency, uint16_t supervison_timeout,
                         uint16_t min_ce_length, uint16_t max_ce_length,
                         bool own_bdaddr_is_random)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCreateConnection(&iface->conn, interval, window,
                                      use_white_list, peer_bdaddr_is_random,
                                      peer_bdaddr, conn_int_min, conn_int_max,
                                      slave_latency, supervison_timeout,
                                      min_ce_length, max_ce_length,
                                      own_bdaddr_is_random);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeCreateConnectionCancel(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeCreateConnectionCancel(&iface->conn);
  return __ret;
}
#if defined(TWIC_API_LELMGENRESOLVABLEBDADDR)
static TZ1K_INLINE twicStatus_t
twicIfLeLmGenResolvableBdaddr(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeLmGenResolvableBdaddr(&iface->conn);
  return __ret;
}
#endif
#if defined(TWIC_API_LESETRANDOMADDRESS)
static TZ1K_INLINE twicStatus_t
twicIfLeSetRandomAddress(twicConnIface_t * const iface,
                         const twicBdaddr_t *const bd)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSetRandomAddress(&iface->conn, bd);
  return __ret;
}
#endif
#if defined(TWIC_API_LELMRESOLVEBDADDR)
static TZ1K_INLINE twicStatus_t
twicIfLeLmResolveBdaddr(twicConnIface_t * const iface,
                        const twicBdaddr_t *const bd, const uint8_t num_of_irk,
                        const twicIrk_t * irks)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeLmResolveBdaddr(&iface->conn, bd, num_of_irk, irks);
  return __ret;
}
#endif
#if defined(TWIC_API_LESMSETIRVALUE)
static TZ1K_INLINE twicStatus_t
twicIfLeSmSetIrValue(twicConnIface_t * const iface, const twicIr_t * const ir)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSetIrValue(&iface->conn, ir);
  
  return __ret;
}
#endif
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
static TZ1K_INLINE twicStatus_t
twicIfLeSmEncrypt(twicConnIface_t * const iface, const uint8_t * const key,
                  const uint8_t * const plain_text_data)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmEncrypt(&iface->conn, key, plain_text_data);
  
  return __ret;
}
#endif
#if defined(TWIC_API_LELESETADVERTISINGDATA) && defined(TWIC_BLE_HWIP_V41)
/* BLUETOOTH SPECIFICATION Version 4.1 [Vol 2].
   Host Controller Interface Functional Specification.
   7.8.7 LE Set Advertising Data Command. */
static TZ1K_INLINE twicStatus_t
twicIfLeSetAdvertisingData(twicConnIface_t * const iface,
                           const uint8_t advertising_data_length,
                           const uint8_t * const advertising_data)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSetAdvertisingData(&iface->conn,
                                        advertising_data_length,
                                        advertising_data);
  return __ret;
}
#endif
#if defined(TWIC_CONFIG_SM_INITIATOR)
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Security Manager Specification.
 * 3.5.1 Pairing Request.
 *
 * @brief
 * The initiator starts the Pairing Feature Exchange by invoking this
 * API to send a Pairing Request command to the responding device.
 *
 * io_capability : Table 3.3 defines the values which are used when
 * exchanging IO capabilities (see Section 2.3.2).
 *
 * oob_data_present : Table 3.4 defines the values which are used when
 * indicating whether OOB authentication data is available (see
 * Section 2.3.3).
 *
 * auth_req_bonding : This field that indicates the type of bonding
 * being requested by the responding device as defined in Table 3.5.
 *
 * auth_req_mitm_protection : 1-bit flag that is set to one if the
 * device is requesting MITM protection, otherwise it shall be set to
 * 0. A device sets the MITM flag to one to request an Authenticated
 * security property for STK.
 *
 * max_enc_key_size : This value defines the maximum encryption key
 * size in octets that the device can support. The maximum key size
 * shall be in the range 7 to 16 octets.
 *
 * The Initiator Key Distribution field defines which keys the
 * initiator shall distribute and use during the Transport Specific
 * Key Distribution phase (see Section 2.4.3). The Initiator Key
 * Distribution field format and usage are defined in Section 3.6.1.
 *
 * init_key_dist_enckey : EncKey is a 1-bit field that is set to one
 * to indicate that the device shall distribute LTK using the
 * Encryption Information command followed by EDIV and Rand using the
 * Master Identification command.
 *
 * init_key_dist_idkey : IdKey is a 1-bit field that is set to one to
 * indicate that the device shall distribute IRK using the Identity
 * Information command followed by its public device or static random
 * address using Identity Address Information.
 *
 * init_key_dist_sign : Sign is a 1-bit field that is set to one to
 * indicate that the device shall distribute CSRK using the Signing
 * Information command.
 *
 * The Responder Key Distribution field defines which keys the
 * responder shall distribute and use during the Transport Specific
 * Key Distribution phase (see Section 2.4.3). The Responder Key
 * Distribution field format and usage are defined in Section 3.6.1.
 *
 * resp_key_dist_enckey : EncKey is a 1-bit field that is set to one
 * to indicate that the device shall distribute LTK using the
 * Encryption Information command followed by EDIV and Rand using the
 * Master Identification command.
 *
 * resp_key_dist_idkey : IdKey is a 1-bit field that is set to one to
 * indicate that the device shall distribute IRK using the Identity
 * Information command followed by its public device or static random
 * address using Identity Address Information.
 *
 * resp_key_dist_sign : Sign is a 1-bit field that is set to one to
 * indicate that the device shall distribute CSRK using the Signing
 * Information command.
 */
static TZ1K_INLINE twicStatus_t
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
                            const bool resp_key_dist_sign)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasPairingRequest(
      &iface->conn, io_capability, oob_data_present,
      auth_req_bonding, auth_req_mitm_protection,
      max_enc_key_size,
      init_key_dist_enckey, init_key_dist_idkey, init_key_dist_sign,  
      resp_key_dist_enckey, resp_key_dist_idkey, resp_key_dist_sign);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeSmMasSecurityReject(
  twicConnIface_t * const iface, const twicSmReasonCode_t reason)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasSecurityReject(&iface->conn, reason);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeSmMasPairingFailed(
  twicConnIface_t * const iface, const twicSmReasonCode_t reason)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasPairingFailed(&iface->conn, reason);
  
  return __ret;
}

/* 000,000 to 999,999. ex) 123456 is 0x0001E240 = {0x40, 0xE2, 0x01, 0x00} */
static TZ1K_INLINE twicStatus_t
twicIfLeSmMasKbPasskeyEntryReply(twicConnIface_t * const iface,
                                 const twicPasskeyEntry_t *const passkey)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasKbPasskeyReply(&iface->conn, passkey);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeSmMasKbPasskeyEntryNegativeReply(
  twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasKbPasskeyNegativeReply(&iface->conn);
  
  return __ret;
}

/* void (*display_passkey)(const void * const connif) is invoked to
 * input the passkey displayed on the LE Device. This PassKey is used
 * in the "PassKey Entry" Pairing method. The display_passkey is
 * generated as part of pairing process. In response to the
 * display_passkey, the application will generate the key write
 * request "twicIfLeSmMasDpPasskeyEntryReply" to input the Displayed
 * Key. If the application needs to reject the pairing process, the
 * twicIfLeSmMasDpPasskeyEntryNegativeReply must be used. */
static TZ1K_INLINE twicStatus_t twicIfLeSmMasDpPasskeyEntryReply(
  twicConnIface_t * const iface, const twicPasskeyEntry_t *const passkey)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasDpPasskeyReply(&iface->conn, passkey);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeSmMasDpPasskeyEntryNegativeReply(
  twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasDpPasskeyNegativeReply(&iface->conn);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeSmMasOobTkReply(
  twicConnIface_t * const iface, const twicOobTk_t *const tk)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasOobTkReply(&iface->conn, tk);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeSmMasOobTkNegativeReply(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasOobTkNegativeReply(&iface->conn);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeSmMasBondingInformationReply(
  twicConnIface_t * const iface,
  const twicEdiv_t * const r_ediv, const twicRand_t * const r_rand,
  const twicLtk_t * const r_ltk, const twicIrk_t * const r_irk,
  const twicCsrk_t * const r_csrk, const twicEdiv_t * const l_ediv,
  const twicRand_t * const l_rand, const twicLtk_t * const l_ltk,
  const twicIrk_t * const l_irk, const twicCsrk_t * const l_csrk,
  const uint8_t encryption_key_size)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasBondingInformationReply(
      &iface->conn, r_ediv, r_rand, r_ltk, r_irk, r_csrk, l_ediv, l_rand,
      l_ltk, l_irk, l_csrk, encryption_key_size);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeSmMasBondingInformationNegativeReply(
  twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasBondingInformationNegativeReply(&iface->conn);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeSmMasBondingState(
  twicConnIface_t * const iface, const twicAuthInfo_t bits)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasBondingState(&iface->conn, bits);
  
  return __ret;
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Security Manager Specification.
 * 6.6 START ENCRYPTION.
 * 
 * @brief
 * This API is issued by an ppplication on the Master device to
 * encrypt the link using LTK. After the link is encrypted the
 * callback is invoked.
 *
 * void (*encryption_change)(
 *   const void * const connif, const twicSmReasonCode_t reason,
 *   const uint8_t key_type, const bool encryption_enable,
 *   const uint8_t encryption_key_size);
 */
static TZ1K_INLINE twicStatus_t
twicIfLeSmMasStartEncryption(twicConnIface_t * const iface,
                             const twicEdiv_t *const ediv,
                             const twicRand_t *const rand,
                             const twicLtk_t *const ltk,
                             const uint8_t encryption_key_size)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmMasStartEncryption(&iface->conn, ediv, rand, ltk,
                                          encryption_key_size);
  
  return __ret;
}

#endif /* TWIC_CONFIG_SM_INITIATOR */

#if defined(TWIC_CONFIG_SM_RESPONDER)
/* BLUETOOTH SPECIFICATION Version 4.0 [Vol 3] page 624 of 656.
 * Security Manager Specification.
 *
 * io_capability : Table 3.3 defines the values which are used when
 * exchanging IO capabilities (see Section 2.3.2).
 *
 * oob_data_present : Table 3.4 defines the values which are used when
 * indicating whether OOB authentication data is available (see
 * Section 2.3.3).
 *
 * auth_req_bonding : This field that indicates the type of bonding
 * being requested by the responding device as defined in Table 3.5.
 *
 * auth_req_mitm_protection : 1-bit flag that is set to one if the
 * device is requesting MITM protection, otherwise it shall be set to
 * 0. A device sets the MITM flag to one to request an Authenticated
 * security property for STK.
 *
 * max_enc_key_size : This value defines the maximum encryption key
 * size in octets that the device can support. The maximum key size
 * shall be in the range 7 to 16 octets.
 *
 * The Initiator Key Distribution field defines which keys the
 * initiator shall distribute and use during the Transport Specific
 * Key Distribution phase (see Section 2.4.3). The Initiator Key
 * Distribution field format and usage are defined in Section 3.6.1.
 *
 * init_key_dist_enckey : EncKey is a 1-bit field that is set to one
 * to indicate that the device shall distribute LTK using the
 * Encryption Information command followed by EDIV and Rand using the
 * Master Identification command.
 *
 * init_key_dist_idkey : IdKey is a 1-bit field that is set to one to
 * indicate that the device shall distribute IRK using the Identity
 * Information command followed by its public device or static random
 * address using Identity Address Information.
 *
 * init_key_dist_sign : Sign is a 1-bit field that is set to one to
 * indicate that the device shall distribute CSRK using the Signing
 * Information command.
 *
 * The Responder Key Distribution field defines which keys the
 * responder shall distribute and use during the Transport Specific
 * Key Distribution phase (see Section 2.4.3). The Responder Key
 * Distribution field format and usage are defined in Section 3.6.1.
 *
 * resp_key_dist_enckey : EncKey is a 1-bit field that is set to one
 * to indicate that the device shall distribute LTK using the
 * Encryption Information command followed by EDIV and Rand using the
 * Master Identification command.
 *
 * resp_key_dist_idkey : IdKey is a 1-bit field that is set to one to
 * indicate that the device shall distribute IRK using the Identity
 * Information command followed by its public device or static random
 * address using Identity Address Information.
 *
 * resp_key_dist_sign : Sign is a 1-bit field that is set to one to
 * indicate that the device shall distribute CSRK using the Signing
 * Information command.
 */
static TZ1K_INLINE twicStatus_t twicIfLeSmSlvPairingConfirm(
  twicConnIface_t * const iface,
  const twicSmpIoCapability_t io_capability,
  const bool oob_data_present,
  const bool auth_req_bonding, const bool auth_req_mitm_protection,
  const uint8_t max_enc_key_size, const bool init_key_dist_enckey,
  const bool init_key_dist_idkey, const bool init_key_dist_sign,  
  const bool resp_key_dist_enckey, const bool resp_key_dist_idkey,
  const bool resp_key_dist_sign)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSlvPairingConfirm(&iface->conn, io_capability,
                                         oob_data_present,
                                         auth_req_bonding,
                                         auth_req_mitm_protection,
                                         max_enc_key_size,
                                         init_key_dist_enckey,
                                         init_key_dist_idkey,
                                         init_key_dist_sign,
                                         resp_key_dist_enckey,
                                         resp_key_dist_idkey,
                                         resp_key_dist_sign);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeSmSlvPairingFailed(
  twicConnIface_t * const iface, const twicSmReasonCode_t reason)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSlvPairingFailed(&iface->conn, reason);
  
  return __ret;
}

/* BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Security Manager Specification.
 * 2.4.6 Slave Security Request.
 *
 * The slave device may request security by transmitting a Security
 * Request to the master. When a master device receives a Security
 * Request it may encrypt the link, initiate the pairing procedure, or
 * reject the request. */
   
static TZ1K_INLINE twicStatus_t twicIfLeSmSlvSecurityRequest(
  twicConnIface_t * const iface,
  const bool auth_req_bonding, const bool auth_req_mitm_protection)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSlvSecurityRequest(&iface->conn, auth_req_bonding,
                                          auth_req_mitm_protection);
  
  return __ret;
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Security Manager Specification.
 * 5.3.2.2 Phase 2: Short Term Key Generation - Passkey Entry.
 *
 * @brief
 * After Pairing Feature Exchange has completed, if the Passkey Entry
 * pairing method is selected, the "void (*input_passkey)(void)"
 * callback function may occur. This API is concatenated to the
 * callback to pass the entered key by the responder for the Passkey
 * Entry. */
static TZ1K_INLINE twicStatus_t
twicIfLeSmSlvKbPasskeyEntryReply(twicConnIface_t * const iface,
                                 const twicPasskeyEntry_t *const passkey)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSlvKbPasskeyReply(&iface->conn, passkey);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeSmSlvKbPasskeyEntryNegativeReply(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSlvKbPasskeyNegativeReply(&iface->conn);
  
  return __ret;
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Security Manager Specification.
 * 5.3.2.2 Phase 2: Short Term Key Generation - Passkey Entry.
 * 2.3.5.3 Passkey Entry.
 *
 * @brief
 * After Pairing Feature Exchange has completed, if the Passkey Entry
 * pairing method is selected, the " void (*display_passkey)(const
 * void * const connif)" callback function may occur. This API is
 * concatenated to the callback to pass the displayed key on the
 * responder for the Passkey Entry.  If the SMP responder does not
 * have the key, the "twicUtLeSmSlvKbPasskeyEntryNegativeReply" should
 * be invoked instead of this API. */
static TZ1K_INLINE twicStatus_t
twicIfLeSmSlvDpPasskeyEntryReply(twicConnIface_t * const iface,
                                 const twicPasskeyEntry_t *const passkey)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSlvDpPasskeyReply(&iface->conn, passkey);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeSmSlvDpPasskeyEntryNegativeReply(
  twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSlvDpPasskeyNegativeReply(&iface->conn);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeSmSlvOobTkReply(
  twicConnIface_t * const iface, const twicOobTk_t *const tk)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSlvOobTkReply(&iface->conn, tk);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t twicIfLeSmSlvOobTkNegativeReply(
  twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSlvOobTkNegativeReply(&iface->conn);
  
  return __ret;
}

/* 
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].  The Host of the
 * Central initiates the pairing process as defined in [Vol. 3], Part
 * C Section 2.1 with the Bonding_Flags set to Bonding as defined in
 * [Vol. 3], Part H Section 3.5.1. If the peer device is in the
 * bondable mode, the devices shall exchange and store the bonding
 * information in the security database.
 *
 * If the callback function is invoked, the SMP responder must invoke
 * this API to inform the bonding information.
 * 
 * void (*inquiry_bonding_information)(const bool address_type_random,
 *                                     const twicBdaddr_t *const identity)
 */
static TZ1K_INLINE twicStatus_t twicIfLeSmSlvBondingInformationReply(
  twicConnIface_t * const iface,
  const twicEdiv_t * const r_ediv, const twicRand_t * const r_rand,
  const twicLtk_t * const r_ltk, const twicIrk_t * const r_irk,
  const twicCsrk_t * const r_csrk, const twicEdiv_t * const l_ediv,
  const twicRand_t * const l_rand, const twicLtk_t * const l_ltk,
  const twicIrk_t * const l_irk, const twicCsrk_t * const l_csrk,
  const uint8_t encryption_key_size)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSlvBondingInformationReply(
      &iface->conn, r_ediv, r_rand, r_ltk, r_irk, r_csrk, l_ediv, l_rand,
      l_ltk, l_irk, l_csrk, encryption_key_size);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeSmSlvBondingInformationNegativeReply(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSlvBondingInformationNegativeReply(&iface->conn);
  
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeSmSlvBondingState(twicConnIface_t * const iface,
                          const twicAuthInfo_t bits)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeSmSlvBondingState(&iface->conn, bits);
  
  return __ret;
}

#endif /* TWIC_CONFIG_SM_RESPONDER */

static TZ1K_INLINE twicStatus_t
twicIfLeDiscoverable(twicConnIface_t * const iface,
                     const uint16_t min_interval, const uint16_t max_interval,
                     const uint8_t advertising_type,
                     const uint8_t own_address_type,
                     const uint8_t direct_address_type,
                     const uint64_t direct_address,
                     const uint8_t advertising_channel_map,
                     const uint8_t advertising_filter_policy,
                     const uint8_t advertising_data_length,
                     const uint8_t * const advertising_data,
                     const uint8_t scan_resp_data_length,
                     const uint8_t * const scan_resp_data)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeDiscoverable(&iface->conn, min_interval, max_interval,
                                  advertising_type, own_address_type,
                                  direct_address_type, direct_address,
                                  advertising_channel_map,
                                  advertising_filter_policy,
                                  advertising_data_length, advertising_data,
                                  scan_resp_data_length, scan_resp_data);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfLeStopAdvertising(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeStopAdvertising(&iface->conn);
  return __ret;
}

/* Reference materials.
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * 4.10 CHARACTERISTIC VALUE NOTIFICATION.
 *
 * @brief
 * This procedure is used to notify a client of the value of a
 * Characteristic Value from a server.
 * The 'le_callbacks->notification_sent' which is registered by the
 * 'twicIfLeRegisterCallback' API will be invoked from the
 * 'twicIfDoEvents' API when the Characteristic Value Notification has
 * been sent to the GATT Client. */
static TZ1K_INLINE twicStatus_t
twicIfGattNotification(twicConnIface_t * const iface, const uint8_t cha,
                       const uint8_t length, const uint8_t * const data)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattNotification(&iface->conn, TWIC_IF_TO_ENTRY(cha), length,
                                    data);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfGattIndication(twicConnIface_t * const iface, const uint8_t cha,
                     const uint8_t length, const uint8_t * const data)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattIndication(
      &iface->conn, TWIC_IF_TO_ENTRY(cha), length, data);
  return __ret;
}

/* MAX element length is MTU - 2 */
static TZ1K_INLINE twicStatus_t
twicIfGattServerWriteCharacteristics(twicConnIface_t * const iface,
                                     const uint8_t cha,
                                     const uint16_t length,
                                     const uint8_t * const value)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattServerWriteElement(TWIC_GATTSERVERWRITECHARACTERISTICS,
                                          &iface->conn, TWIC_IF_TO_ENTRY(cha),
                                          length, value);
  return __ret;
}

/* The Maximum ATT_MTU size supported by Firmware is 72 bytes (0x48) */
static TZ1K_INLINE twicStatus_t
twicIfGattServerExgMtuResponse(twicConnIface_t * const iface,
                               const uint8_t status, const uint16_t rx_mtu_size)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfGattServerExgMtuResponse(&iface->conn, status, rx_mtu_size);
  return __ret;
}

/* If authorization is required by the remote device to read this char
 * value from server and host application do not want to authorize the
 * corresponding remote device to read this data ,then the host shall
 * not update this value and must send the accept request with status
 * as 0x08 (Insufficient Authorization. Please refer to
 * twic_interface_cb.h "LIST OF ERROR CODES [2]") to GATT Server. The
 * same will be indicated to remote client in error response. Also if
 * host want to reject the event with some other Error Response it can
 * indicate the same in twicIfGatt Response. */

/* NOTE: The update of the characteristic descriptor and value must be
 * performed by the twicIfGattServerWriteCharacteristics API before
 * the each response API such as
 * twicIfGattServerCharDespWriteInResponse API is invoked. */

static TZ1K_INLINE twicStatus_t
twicIfGattServerCharDespWriteInResponse(twicConnIface_t * const iface,
                                        const uint8_t desc,
                                        const uint8_t status)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(desc)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattServerAttResponse(
      &iface->conn, TWIC_GATTSERVERCHARDESPWRITEINRESPONSE,
      TWIC_IF_TO_ENTRY(desc), status);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfGattServerCharDespReadOutResponse(twicConnIface_t * const iface,
                                        const uint8_t desc,
                                        const uint8_t status)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(desc)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattServerAttResponse(
      &iface->conn, TWIC_GATTSERVERCHARDESPREADOUTRESPONSE,
      TWIC_IF_TO_ENTRY(desc), status);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfGattServerCharValReadOutResponse(twicConnIface_t * const iface,
                                       const uint8_t cha, const uint8_t status)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattServerAttResponse(
      &iface->conn, TWIC_GATTSERVERCHARVALREADOUTRESPONSE,
      TWIC_IF_TO_ENTRY(cha), status);
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfGattServerCharValMultiReadOutResponse(twicConnIface_t * const iface,
                                            const uint8_t status,
                                            const uint8_t err_eidx)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req)) {
    if (err_eidx == 0)
      __ret = _twicIfGattServerCharValMultiReadOutResponse(&iface->conn, 0, 0);
    else if (TWIC_IF_TO_ENTRY(err_eidx)->ap == TWIC_ENTRY_HANDLE_DECLARATION)
      __ret = _twicIfGattServerCharValMultiReadOutResponse(
        &iface->conn, status, TWIC_IF_TO_ENTRY(err_eidx));
  }
  return __ret;
}

static TZ1K_INLINE twicStatus_t
twicIfGattServerCharValWriteInResponse(twicConnIface_t * const iface,
                                       const uint8_t cha, const uint8_t status)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattServerAttResponse(
      &iface->conn, TWIC_GATTSERVERCHARVALWRITEINRESPONSE,
      TWIC_IF_TO_ENTRY(cha), status);
  return __ret;
}

#if defined(TWIC_API_GATTSERVERLONGCHARVALREADOUTRESPONSE)
static TZ1K_INLINE twicStatus_t
twicIfGattServerLongCharValReadOutResponse(twicConnIface_t * const iface,
                                           const uint8_t cha,
                                           const uint8_t status)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattServerAttResponse(
      &iface->conn, TWIC_GATTSERVERLONGCHARVALREADOUTRESPONSE,
      TWIC_IF_TO_ENTRY(cha), status);
  return __ret;
}
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARDESPREADOUTRESPONSE)
static TZ1K_INLINE twicStatus_t
twicIfGattServerLongCharDespReadOutResponse(twicConnIface_t * const iface,
                                            const uint8_t desc,
                                            const uint8_t status)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(desc)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattServerAttResponse(
      &iface->conn, TWIC_GATTSERVERLONGCHARDESPREADOUTRESPONSE,
      TWIC_IF_TO_ENTRY(desc), status);
  return __ret;
}
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARVALPREPAREWRITEINRESPONSE)
static TZ1K_INLINE twicStatus_t
twicIfGattServerLongCharValPrepareWriteInResponse(twicConnIface_t * const iface,
                                                  const uint8_t cha,
                                                  const uint8_t status)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattServerAttResponse(
      &iface->conn, TWIC_GATTSERVERLONGCHARVALPREPAREWRITEINRESPONSE,
      TWIC_IF_TO_ENTRY(cha), status);
  return __ret;
}
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARDESPPREPAREWRITEINRESPONSE)
static TZ1K_INLINE twicStatus_t
twicIfGattServerLongCharDespPrepareWriteInResponse(
  twicConnIface_t * const iface, const uint8_t cha, const uint8_t status)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattServerAttResponse(
      &iface->conn, TWIC_GATTSERVERLONGCHARDESPPREPAREWRITEINRESPONSE,
      TWIC_IF_TO_ENTRY(cha), status);
  return __ret;
}
#endif
#if defined(TWIC_API_GATTSERVEREXECCHARVALWRITEINRESPONSE)
static TZ1K_INLINE twicStatus_t
twicIfGattServerExecCharValWriteInResponse(twicConnIface_t * const iface,
                                           const uint8_t cha,
                                           const uint8_t status)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattServerAttResponse(
      &iface->conn, TWIC_GATTSERVEREXECCHARVALWRITEINRESPONSE,
      TWIC_IF_TO_ENTRY(cha), status);
  return __ret;
}
#endif
#if defined(TWIC_API_GATTSERVEREXECCHARDESPWRITEINRESPONSE)
static TZ1K_INLINE twicStatus_t
twicIfGattServerExecCharDespWriteInResponse(twicConnIface_t * const iface,
                                            const uint8_t desp,
                                            const uint8_t status)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(desp)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfGattServerAttResponse(
      &iface->conn, TWIC_GATTSERVEREXECCHARDESPWRITEINRESPONSE,
      TWIC_IF_TO_ENTRY(desp), status);
  return __ret;
}
#endif
static TZ1K_INLINE twicStatus_t
twicIfLeGattDbBeginServiceCreation(twicConnIface_t * const iface,
                                   const uint8_t service,
                                   const uint64_t uuid_lsb,
                                   const uint64_t uuid_msb,
                                   const uint8_t uuid_len)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(TWIC_IF_TO_ENTRY(service)->req))
    __ret = _twicIfLeGattBeginServiceCreation(
      TWIC_IF_TO_ENTRY(service), uuid_lsb, uuid_msb, uuid_len);
  return __ret;
}

/* This API is used to add secondary service declaration to the server
 * database. The secondary service declaration in server database with
 * Attribute Type is 0x2801. */
static TZ1K_INLINE twicStatus_t
twicIfLeGattDbBeginSecondaryServiceCreation(twicConnIface_t * const iface,
                                            const uint8_t primacy,
                                            const uint8_t service,
                                            const uint64_t uuid_lsb,
                                            const uint64_t uuid_msb,
                                            const uint8_t uuid_len)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(primacy)->ap == TWIC_ENTRY_HANDLE_DEFINITION &&
      !(TWIC_IF_TO_ENTRY(service)->req))
    __ret = _twicIfLeGattBeginSecondaryServiceCreation(
      TWIC_IF_TO_ENTRY(primacy), TWIC_IF_TO_ENTRY(service),
      uuid_lsb, uuid_msb, uuid_len);
  return __ret;
}

/* This API is used to Add Include Service within the primary service
 * or secondary service. */
static TZ1K_INLINE twicStatus_t
twicIfLeGattDbIncludeServiceCreation(twicConnIface_t * const iface,
                                     const uint8_t service,
                                     const uint8_t inclusion,
                                     const uint64_t uuid_lsb,
                                     const uint64_t uuid_msb,
                                     const uint8_t uuid_len)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(service)->ap == TWIC_ENTRY_HANDLE_DEFINITION &&
      TWIC_IF_TO_ENTRY(inclusion)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(TWIC_IF_TO_ENTRY(service)->req))
    __ret = _twicIfLeGattIncludeServiceCreation(
      TWIC_IF_TO_ENTRY(service), TWIC_IF_TO_ENTRY(inclusion),
      uuid_lsb, uuid_msb, uuid_len);
  return __ret;
}

/* This API is used to add characteristic declaration for a particular
 * primary or secondary service. */
static TZ1K_INLINE twicStatus_t
twicIfLeGattDbAddCharacteristics(twicConnIface_t * const iface,
                                 const uint8_t service, const uint8_t cha,
                                 const uint8_t properties,
                                 const uint64_t uuid_lsb,
                                 const uint64_t uuid_msb,
                                 const uint8_t uuid_len)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(service)->ap == TWIC_ENTRY_HANDLE_DEFINITION &&
      !(TWIC_IF_TO_ENTRY(cha)->req))
    __ret = _twicIfLeGattAddCharacteristics(TWIC_IF_TO_ENTRY(service),
                                            TWIC_IF_TO_ENTRY(cha),
                                            properties, uuid_lsb, uuid_msb,
                                            uuid_len);
  return __ret;
}

/* This API is used to add and set Fixed Length Attribute Value like
 * Characteristic Value Declaration, Characteristic Descriptor
 * Declaration for the Characteristic defined. */
static TZ1K_INLINE twicStatus_t
twicIfLeGattDbSetCharacteristics(twicConnIface_t * const iface,
                                 const uint8_t cha, const uint16_t permissions,
                                 const uint16_t element_length,
                                 const uint8_t * const element_value,
                                 const uint64_t uuid_lsb,
                                 const uint64_t uuid_msb,
                                 const uint8_t uuid_len)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DEFINITION &&
      !(TWIC_IF_TO_ENTRY(cha)->req))
    __ret = _twicIfLeGattSetCharacteristics(TWIC_IF_TO_ENTRY(cha), true,
                                            permissions, element_length,
                                            element_value, uuid_lsb, uuid_msb,
                                            uuid_len);
  return __ret;
}

/* This API is used to update Fixed Length Attribute Value like
 * Characteristic Value Declaration, Characteristic Descriptor
 * Declaration for the Characteristic declared. */
static TZ1K_INLINE twicStatus_t
twicIfLeGattDbUpdCharacteristics(twicConnIface_t * const iface,
                                 const uint8_t cha, const uint16_t permissions,
                                 const uint16_t element_length,
                                 const uint8_t * const element_value,
                                 const uint64_t uuid_lsb,
                                 const uint64_t uuid_msb,
                                 const uint8_t uuid_len)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if ((TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DEFINITION ||
       TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DECLARATION) &&
      !(TWIC_IF_TO_ENTRY(cha)->req) && !(iface->conn.req))
    __ret = _twicIfLeGattSetCharacteristics(TWIC_IF_TO_ENTRY(cha), false,
                                            permissions, element_length,
                                            element_value, uuid_lsb, uuid_msb,
                                            uuid_len);
  return __ret;
}
#if defined(TWIC_API_LEGATTDBSETCHARACTERISTICSVL) || \
  defined(TWIC_API_LEGATTDBUPDCHARACTERISTICSVL)
static TZ1K_INLINE twicStatus_t
twicIfLeGattDbSetCharacteristicsVl(twicConnIface_t * const iface,
                                   const uint8_t cha,
                                   const uint16_t permissions,
                                   const uint16_t max_element_length,
                                   const uint16_t element_length,
                                   const uint8_t * const element_value,
                                   const uint64_t uuid_lsb,
                                   const uint64_t uuid_msb,
                                   const uint8_t uuid_len)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(cha)->ap == TWIC_ENTRY_HANDLE_DEFINITION &&
      !(TWIC_IF_TO_ENTRY(cha)->req))
    __ret = _twicIfLeGattSetCharacteristicsVl(TWIC_IF_TO_ENTRY(cha),
                                              true, permissions,
                                              max_element_length,
                                              element_length, element_value,
                                              uuid_lsb, uuid_msb, uuid_len);
  return __ret;
}
#endif
static TZ1K_INLINE twicStatus_t
twicIfLeGattDbSetDescriptor(twicConnIface_t * const iface,
                            const uint8_t entity, const uint8_t desc,
                            const uint16_t permissions,
                            const uint16_t element_length,
                            const uint8_t * const element_value,
                            const uint64_t uuid_lsb, const uint64_t uuid_msb,
                            const uint8_t uuid_len)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(entity)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(TWIC_IF_TO_ENTRY(desc)->req))
    __ret = _twicIfLeGattSetDescriptor(TWIC_IF_TO_ENTRY(entity),
                                       TWIC_IF_TO_ENTRY(desc),
                                       permissions, element_length,
                                       element_value,
                                       uuid_lsb, uuid_msb, uuid_len);
  return __ret;
}
#if defined(TWIC_LEGATTDBSETDESCRIPTORVL)
static TZ1K_INLINE twicStatus_t
twicIfLeGattDbSetDescriptorVl(twicConnIface_t * const iface,
                              const uint8_t entity, const uint8_t desc,
                              const uint16_t permissions,
                              const uint16_t max_element_length,
                              const uint16_t element_length,
                              const uint8_t * const element_value,
                              const uint64_t uuid_lsb, const uint64_t uuid_msb,
                              const uint8_t uuid_len)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(entity)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(TWIC_IF_TO_ENTRY(desc)->req))
    __ret = _twicIfLeGattSetDescriptorVl(TWIC_IF_TO_ENTRY(entity),
                                         TWIC_IF_TO_ENTRY(desc),
                                         permissions, max_element_length,
                                         element_length, element_value,
                                         uuid_lsb, uuid_msb, uuid_len);
  return __ret;
}
#endif
static TZ1K_INLINE twicStatus_t
twicIfLeGattDbEndServiceCreation(twicConnIface_t * const iface,
                                 const uint8_t service)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(service)->ap == TWIC_ENTRY_HANDLE_DEFINITION &&
      !(TWIC_IF_TO_ENTRY(service)->req))
    __ret = _twicIfLeEndServiceCreation(TWIC_IF_TO_ENTRY(service));
  return __ret;
}
#if defined(TWIC_API_LEGATTDBSETPERMISSIONS)
static TZ1K_INLINE twicStatus_t
twicIfLeGattDbSetPermissions(twicConnIface_t * const iface,
                             const uint8_t entity, const uint16_t permissions)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(entity)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfLeGattDbSetPermissions(TWIC_IF_TO_ENTRY(entity),
                                          permissions);
  return __ret;
}
#endif
#if defined(TWIC_API_LEGATTDBGETPERMISSIONS)
static TZ1K_INLINE twicStatus_t
twicIfLeGattDbGetPermissions(twicConnIface_t * const iface,
                             const uint8_t entity)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (TWIC_IF_TO_ENTRY(entity)->ap == TWIC_ENTRY_HANDLE_DECLARATION &&
      !(iface->conn.req))
    __ret = _twicIfLeGattDbGetPermissions(TWIC_IF_TO_ENTRY(entity));
  return __ret;
}
#endif
#if defined(TWIC_API_GATTCLIENTEXGMTU)
static TZ1K_INLINE twicStatus_t
twicIfGattClientExgMtu(twicConnIface_t * const iface,
                       const uint16_t rx_mtu_size)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfGattClientExgMtu(&iface->conn, rx_mtu_size);
  return __ret;
}
#endif

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT)
 * 4.4.1 Discover All Primary Services.
 *
 * @brief Discover All Primary Services.
 * This API is used by a client to discover all the primary services
 * on a server.
 *
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint16_t start_handle The Starting Handle shall be set to 0x0001.
 * @param uint16_t end_handle The Ending Handle shall be set to 0xFFFF.
 *
 * Expects the events which have a list of Attribute Handle, End Group
 * Handle, and Attribute Value tuples corresponding to the services
 * supported by the server. Each Attribute Value contained in the
 * response is the Service UUID of a service supported by the
 * server. The Attribute Handle is the handle for the service
 * declaration. The End Group Handle is the handle of the last
 * attribute within the service definition.
 *
 * No error occured:

 void (*primary_service)(
 const uint8_t status, const bool next, const uint16_t error_handle,
 const uint16_t attribute_handle, const uint16_t end_group_handle,
 const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
 
 * Error occured in the first Handle:
 * 
 * In this case the fetched data in the Event contains only the Error
 * code(Status) , Continue Flag (0x00) and Error Handle. In this
 * scenario parameter length shall be always 6 bytes with status other
 * than SUCCESS (0x00).  To find other char declaration the discovery
 * request must be re-issued with Start handle as (Error Handle +1).
 * When event is received with status as success and continue flag as
 * zero it indicates all services are discovered and Attribute Not
 * Found Error was received by Client Module.
 *
 * Error occurred after the second Handles:
 *
 * In this case the discovered data along with the Error code,
 * Continue flag, Number of Groups and Length of each Group shall be
 * sent across the event.  The host device shall parse this data and
 * issue further GATT request with the updated handle.  In this
 * scenario parameter length shall be always more than 6 bytes with
 * status other than SUCCESS (0x00).  In the below example, Host
 * issued Discover Char Declaration By UUID request with Start Handle
 * : 0x0001 and End Handle: 0xFFFF Host received proper handle-value
 * pair (Handle : 0x0411 and Value 0x0000) along with an error code
 * Read Not Permitted(0x02).  This means there is some Char
 * Declaration after the handle 0x0411 which resulted in error.  Hence
 * host has to issue next Discover Char Declaration By UUID request
 * with the start handle as 0x0412 and End Handle: 0xFFFF.  When
 * discovery request is re-issued from 0x412 the Handle that resulted
 * in Error shall be contained in event and parameter length shall be
 * 0x06 bytes as described in Case 2.  To find other char declaration
 * the discovery request must be re-issued with Start handle as (Error
 * Handle +1).  This shall be repeated until all the data are
 * discovered in the requested handle range.  When event is received
 * with status as success and continue flag as zero it indicates all
 * services are discovered and Attribute Not Found Error was received
 * by Client Module. */
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICE)
static TZ1K_INLINE twicStatus_t twicIfLeGattClientDiscoverPrimaryService(
  twicConnIface_t * const iface, const uint16_t start_handle,
  const uint16_t end_handle)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientDiscovery(
      &iface->conn, TWIC_GATTCLIENTDISCOVERPRIMARYSERVICE,
      start_handle, end_handle, 0, 0, 0);
  return __ret;
}
#endif

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT)
 * 4.4.2 Discover Primary Service by Service UUID.
 *
 * @brief Discover Primary Service by Service UUID.
 *
 * @param twicConnIface_t * const cif
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint16_t start_handle
 * The Starting Handle.
 * @param uint16_t end_handle
 * The Ending Handle.
 * @param const uint64_t uuid_lsb
 * The part of the Least Significant Octet in the UUID.
 * @param const uint64_t uuid_msb
 * The part of the Most Significant Octet in the UUID.
 * @param const uint8_t uuid_len
 * The bytes of UUID.
 * 
 * This API is used by a client to discover a specific primary service
 * on a server when only the Service UUID is known. The specific
 * primary service may exist multiple times on a server. The primary
 * service being discovered is identified by the service UUID.  The
 * Attribute Protocol Find By Type Value Request shall be used with
 * the Attribute Type parameter set to the UUID for the Primary
 * Service and the Attribute Value set to the 16-bit Bluetooth UUID or
 * 128-bit UUID for the specific primary service. The Starting Handle
 * shall be set to 0x0001 and the Ending Handle shall be set to
 * 0xFFFF.
 * 
 * Expects the events which have a list of Attribute Handle
 * ranges. The Attribute Handle range is the starting handle and the
 * ending handle of the service definition.
 * The 'primary_service_by_uuid' API is invoked when the service is found.
 
 void (*primary_service_by_uuid)(
 const bool status, const bool next,
 const uint16_t attribute_handle, const uint16_t end_group_handle) */
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientDiscoverPrimaryServiceByServiceUuid(
  twicConnIface_t * const iface,
  const uint16_t start_handle, const uint16_t end_handle,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientDiscovery(
      &iface->conn, TWIC_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID,
      start_handle, end_handle, uuid_lsb, uuid_msb, uuid_len);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT)
 * 4.5.1 Find Included Services.
 *
 * @brief Find Included Services.
 * This API is used by a client to find include service declarations
 * within a service definition on a server. The service specified is
 * identified by the service handle range.
 *
 * @param twicConnIface_t * const cif 
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param uint16_t start_handle
 * The Starting Handle shall be set to starting handle of the
 * specified service.
 * @param uint16_t end_handle
 * The Ending Handle shall be set to the ending handle of the
 * specified service.
 *
 * The Starting Handle shall be set to starting handle of the
 * specified service and the Ending Handle shall be set to the ending
 * handle of the specified service. It is permitted to end the
 * API early if a desired included service is found prior to
 * discovering all the included services of the specified service
 * supported on the server.
 * returns a set of Attribute Handle and Attribute Value pairs
 * corresponding to the included services in the service
 * definition. Each Attribute Value contained in the response is
 * composed of the Attribute Handle of the included service
 * declaration and the End Group Handle. If the service UUID is a
 * 16-bit Bluetooth UUID it is also returned in the response.

 void (*included_service)(
 const uint8_t status, const bool next, const uint16_t error_handle,
 const uint16_t attribute_handle,
 const uint16_t included_service_attribute_handle,
 const uint16_t included_service_end_group_handle,
 const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len); */
#if defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE)
static TZ1K_INLINE twicStatus_t twicIfLeGattClientFindIncludedService(
  twicConnIface_t * const iface, const uint16_t start_handle,
  const uint16_t end_handle)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientDiscovery(&iface->conn,
                                         TWIC_GATTCLIENTFINDINCLUDEDSERVICE,
                                         start_handle, end_handle, 0, 0, 0);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT)
 * 4.6.1 Discover All Characteristics of a Service.
 *
 * This API is used by a client to find all the characteristic
 * declarations within a service definition on a server when only the
 * service handle range is known. The service specified is identified
 * by the service handle range.
 *
 * Expects the events which have a list of Attribute Handle and
 * Attribute Value pairs corresponding to the characteristics in the
 * service definition. The Attribute Handle is the handle for the
 * characteristic declaration. The Attribute Value is the
 * Characteristic Properties, Characteristic Value Handle and
 * Characteristic UUID.

 void (*all_char_of_service)(
 const uint8_t status, const bool next, const uint16_t error_handle,
 const uint16_t attribute_handle, const uint8_t char_properties,
 const uint16_t char_value_handle,
 const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
*/
#if defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientDiscoverAllCharacteristics(twicConnIface_t * const iface,
                                             const uint16_t start_handle,
                                             const uint16_t end_handle)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientDiscovery(
      &iface->conn, TWIC_GATTCLIENTDISCOVERALLCHARACTERISTICS,
      start_handle, end_handle, 0, 0, 0);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT)
 * 4.6.2 Discover Characteristics by UUID.
 *
 * Expects the events which have a list of Attribute Handle and
 * Attribute Value pairs corresponding to the characteristics
 * contained in the handle range provided.  Each Attribute Value in
 * the list is the Attribute Value for the characteristic
 * declaration. The Attribute Value contains the characteristic
 * properties, Characteristic Value Handle and characteristic UUID.

 void (*char_by_uuid)(
 const uint8_t status, const bool next, const uint16_t error_handle,
 const uint16_t attribute_handle, const uint8_t char_properties,
 const uint16_t char_value_handle,
 const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
 */
#if defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)
static TZ1K_INLINE twicStatus_t twicIfLeGattClientDiscoverCharacteristicsByUuid(
  twicConnIface_t * const iface,
  const uint16_t start_handle, const uint16_t end_handle,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientDiscovery(
      &iface->conn, TWIC_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID,
      start_handle, end_handle, uuid_lsb, uuid_msb, uuid_len);
  return __ret;
}
#endif

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * 4.7.1 Discover All Characteristic Descriptors.
 *
 * This API is used by a client to find all the characteristic
 * descriptor's Attribute Handles and Attribute Types within a
 * characteristic definition when only the characteristic handle range
 * is known. The characteristic specified is identified by the
 * characteristic handle range.
 *
 * Expects the events which have a list of Attribute Handle and
 * Attribute Value pairs corresponding to the characteristic
 * descriptors in the characteristic definition. The Attribute Handle
 * is the handle for the characteristic descriptor declaration. The
 * Attribute Value is the Characteristic Descriptor UUID.

 void (*all_char_descriptors)(
 const uint8_t status, const bool next, const uint16_t error_handle,
 const uint16_t attribute_handle,
 const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len); */
#if defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientDiscoverAllDescriptors(twicConnIface_t * const iface,
                                         const uint16_t start_handle,
                                         const uint16_t end_handle)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientDiscovery(
      &iface->conn, TWIC_GATTCLIENTDISCOVERALLDESCRIPTORS,
      start_handle, end_handle, 0, 0, 0);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * 4.8.1 Read Characteristic Value.
 *
 * @brief Read Characteristic Value.
 * This API is used to read a Characteristic Value from a server when
 * the client knows the Characteristic Value Handle.  When this API
 * processing is completed characteristic value event
 * "char_value_readout" is generated.

 void (*char_value_readout)(
 const uint8_t status, const twicAttValue_t *const resp);

 *
 * @param twicConnIface_t * const iface 
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint16_t handle Characteristic Value Handle. */
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientReadCharacteristicValue(twicConnIface_t * const iface,
                                          const uint16_t handle)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientRead(
      &iface->conn, TWIC_GATTCLIENTREADCHARACTERISTICVALUE,
      handle, 0, 0, 0, 0);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * 4.8.2 Read Using Characteristic UUID.
 *
 * @brief Read Characteristic Value.
 * This API is used to read a Characteristic Value from a server when
 * the client does not know the Characteristic Value Handle and knows
 * only the characteristic UUID.

 void (*char_value_readout_using_uuid)(
 const uint8_t status, const bool next, const uint16_t error_handle,
 const uint16_t char_value_handle,
 const twicAttValue_t *const resp);

 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint16_t start_handle
 * Starting Handle. This is typically the handle range for the service
 * in which the characteristic belongs.
 * @param const uint16_t end_handle
 * Endiing Handle. This is typically the handle range for the service
 * in which the characteristic belongs.
 * @param const uint64_t uuid_lsb
 * The known characteristic UUID.
 * If UUID is 7905F431-B5CE-4E99-140F-4B1E122D00D0, this parameter
 * shall be set with argument 0x140F4B1E122D00D0.
 * @param const uint64_t uuid_msb
 * The known characteristic UUID.
 * If UUID is 7905F431-B5CE-4E99-140F-4B1E122D00D0, this parameter
 * shall be set with argument 0x7905F431B5CE4E99.
 * @param const uint8_t uuid_len
 * If UUID is 7905F431-B5CE-4E99-140F-4B1E122D00D0, this parameter
 * shall be set with argument 16.
 */
#if defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientReadUsingCharacteristicUuid(twicConnIface_t * const iface,
                                              const uint16_t start_handle,
                                              const uint16_t end_handle,
                                              const uint64_t uuid_lsb,
                                              const uint64_t uuid_msb,
                                              const uint8_t uuid_len)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientRead(
      &iface->conn, TWIC_GATTCLIENTREADUSINGCHARACTERISTICUUID,
      start_handle, end_handle, uuid_lsb, uuid_msb, uuid_len);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT)
 * 4.9.3 Write Characteristic Value.
 *
 * @brief Write Characteristic Value.
 * This API is used to write a Characteristic Value to a server when
 * the client knows the Characteristic Value Handle.  Characteristic
 * Attribute value to be written onto server. Even Zero bytes can be
 * written.
 *
 * When this API processing is completed characteristic write value
 * call-back API is invoked.

 void (*char_value_writein_response)(const uint8_t status);

 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint16_t handle
 * The Attribute Handle parameter shall be set to the this parameter.
 * @param const uint8_t length
 * The length of the new characteristic.
 * @param const uint8_t * const value
 * The Attribute Value parameter shall be set to the new characteristic.
 * @note This API only writes the first (ATT_MTU %G–%@ 3) octets of a
 * Characteristic Value. */
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICVALUE)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientWriteCharacteristicValue(twicConnIface_t * const iface,
                                           const uint16_t handle,
                                           const uint8_t length,
                                           const uint8_t * const value)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientWrite(
      &iface->conn, TWIC_GATTCLIENTWRITECHARACTERISTICVALUE,
      handle, length, value);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT)
 * 4.12.1 Read Characteristic Descriptors.
 *
 * @brief Read Characteristic Descriptors.
 * This API is used to read a characteristic descriptor (ex:
 * characteristic format, characteristic user descriptor) from a
 * server when the client knows the characteristic descriptor
 * declaration's Attribute handle.
 *  
 * When this API processing is completed characteristic write value
 * call-back API is invoked.

 void (*char_desp_readout)(const uint8_t status, const twicAttValue_t *const);

 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint16_t handle Characteristic descriptor handle.
 */   
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientReadCharacteristicDescriptor(twicConnIface_t * const iface,
                                               const uint16_t handle)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientRead(
      &iface->conn, TWIC_GATTCLIENTREADCHARACTERISTICDESCRIPTOR,
      handle, 0, 0, 0, 0);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Generic Attribute Profile (GATT)
 * 4.12.3 Write Characteristic Descriptors.
 *
 * @brief Write Characteristic Descriptors.
 * This API is used to write a Characteristic Descriptor Value to a
 * server when the client knows the Characteristic Descriptor Handle.
 * 
 * When this command processing is completed characteristic descriptor
 * call-back API is invoked.

 void (*char_desp_writein_response)(const * uint8_t status)

 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint16_t handle
 * The Attribute Handle parameter shall be set to this parameter.
 * @param const uint8_t length
 * The length of the new characteristic descriptor value.
 * @param const uint8_t * const value
 * This parameter shall be set to the new characteristic descriptor value.
 */
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientWriteCharacteristicDescriptor(twicConnIface_t * const iface,
                                                const uint16_t handle,
                                                const uint8_t length,
                                                const uint8_t * const value)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientWrite(
      &iface->conn, TWIC_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR,
      handle, length, value);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Attribute Protocol (ATT)
 * 3.4.7.3 Handle Value Confirmation.
 *
 * @brief Handle Value Confirmation.
 * The Characteristic Value Confirmation Accept API is sent by client
 * application to GATT client. This response is Confirmation to
 * Indication received from server.  The client must send this
 * response for every indication received from server.
 *
 * On receiving the Accept Request, the GATT Client sends the
 * confirmation to client device. Also it will send the Accept
 * Response Confirmation to Client Application.
 *
 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface. */
#if defined(TWIC_API_GATTCLIENTINDICATIONCONFIRMATIONRESPONSE) 
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientIndicationConfirmationResponse(twicConnIface_t * const iface)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientResponse(
      &iface->conn, TWIC_GATTCLIENTINDICATIONCONFIRMATIONRESPONSE, 0);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Attribute Protocol (ATT)
 * 4.9.1 Write Without Response.
 *
 * @brief Write Without Response.
 * This API is used to write a Characteristic Value to a server when
 * the client knows the Characteristic Value Handle and the client
 * does not need an acknowledgement that the write was successfully
 * performed. This API only writes the first (ATT_MTU %G–%@ 3)
 * octets of a Characteristic Value. This API cannot be used
 * to write a long characteristic. If the attribute value is longer
 * than (ATT_MTU%G–%@3) octets, then only the first (ATT_MTU%G–%@3) octets of
 * this attributes value shall be sent in a Write Command by the GATT
 * Server to Remote Device.
 *
 * When the processing of the Write Command is started, the call-back
 * API is invoked.

 void (*char_vale_writein_started)(const uint8_t status);

 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint16_t handle
 * The Attribute Handle parameter shall be set to the this parameter.
 * @param const uint8_t length
 * The length of the new characteristic.
 * @param const uint8_t * const value
 * The Attribute Value parameter shall be set to the new characteristic. */
#if defined(TWIC_API_GATTCLIENTWRITEWITHOUTRESPONSE)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientWriteWithoutResponse(twicConnIface_t * const iface,
                                       const uint16_t handle,
                                       const uint8_t length,
                                       const uint8_t * const value)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientWrite(
      &iface->conn, TWIC_GATTCLIENTWRITEWITHOUTRESPONSE,
      handle, length, value);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Attribute Protocol (ATT)
 * 4.9.2 Signed Write Without Response.
 *
 * @brief Signed Write Without Response.
 * This API is used to write a Characteristic Value to a server when
 * the client knows the Characteristic Value Handle and the client
 * does not need an acknowledgement that the write was successfully
 * performed.
 *
 * This API must be used if the Characteristic Properties
 * authenticated Signed Writes bit is enabled and the client and
 * server device share a bond. This API only writes the
 * first (ATT_MTU %G–%@ 15) octets of a Characteristic Value. This
 * API cannot be used to write a long characteristic.
 *
 * When the processing of the Write Command is started, the call-back
 * API is invoked.

 void (*char_vale_writein_started)(const uint8_t status);

 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint16_t handle
 * The Attribute Handle parameter shall be set to the this parameter.
 * @param const uint8_t length
 * The length of the new characteristic.
 * @param const uint8_t * const value
 * The Attribute Value parameter shall be set to the new characteristic. */
#if defined(TWIC_API_GATTCLIENTSIGNEDWRITEWITHOUTRESPONSE)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientSignedWriteWithoutResponse(twicConnIface_t * const iface,
                                             const uint16_t handle,
                                             const uint8_t length,
                                             const uint8_t * const value)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
  __ret = _twicIfLeGattClientSignedWrite(&iface->conn, handle, length, value);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Attribute Protocol (ATT)
 * 4.8.4 Read Multiple Characteristic Values.
 *
 * @brief Read Multiple Characteristic Values.
 * This API is used to read multiple Characteristic Values from a
 * server when the client knows the Characteristic Value Handles.
 * 
 * When this API processing is completed, the call-back API is
 * invoked.

 void (*multiple_char_values_readout)(const uint8_t status,
                                      const twicAttValue_t *const resp);

 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint8_t number The number of the Handles.
 * @param const uint16_t handles Characteristic Value Handles. */
#if defined(TWIC_API_GATTCLIENTREADMULTIPLECHARVALUES)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientReadMultipleCharValues(twicConnIface_t * const iface,
                                         const uint8_t number,
                                         const uint16_t * const handles)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientReadMultipleCharValues(
      &iface->conn, number, handles);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Attribute Protocol (ATT)
 * 4.9.5 Reliable Writes.
 *
 * @brief Reliable Writes.
 * This API is used to write a Characteristic Value to a server when
 * the client knows the Characteristic Value Handle, and assurance is
 * required that the correct Characteristic Value is going to be
 * written by transferring the Characteristic Value to be written in
 * both directions before the write is performed. This request can
 * also be used when multiple values must be written, in order, in a
 * single operation.
 *
 * When this API processing is completed, the API is invoked.

 void (*reliable_writein_confirmation)(const uint8_t status, const
 twicReliableWrite_t *const resp);

 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint16_t total
 * The number of the twicReliableWritein_t parameters.
 * @param const twicReliableWritein_t * const characteristics_value
 * - The Characteristic Value Handle to write the value.
 * - Offset for writing the characteristic value.
 * - Length of the characteristic value to be written.
 * - Characteristic Attribute value to be written. */
#if defined(TWIC_API_GATTCLIENTRELIABLEWRITE)
static TZ1K_INLINE twicStatus_t twicIfLeGattClientReliableWrite(
  twicConnIface_t * const iface, const uint16_t total,
  const twicReliableWritein_t * const characteristics_value)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientReliableWrite(
      &iface->conn, total, characteristics_value);
  return __ret;
}
#endif 
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Attribute Protocol (ATT)
 * 4.8.3 Read Long Characteristic Values.
 *
 * @brief Read Long Characteristic Values.
 * This API used to read a Long Characteristic Value from a server
 * when the client knows the Characteristic Value Handle.
 *
 * When this API processing is completed, the API is invoked.

 void (*long_char_value_readout)( const uint8_t status, const bool next,
 const twicAttValue_t *const resp);

 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint16_t handle
 * Characteristic Value Handle of the Characteristic Value to be read.
 * @param const uint16_t offset
 * The Offset parameter shall be the offset within the Characteristic
 * Value to be read. */
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientReadLongCharacteristicValue(twicConnIface_t * const iface,
                                              const uint16_t handle,
                                              const uint16_t offset)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientLongRead(
      &iface->conn, TWIC_GATTCLIENTREADLONGCHARACTERISTICVALUE,
      handle, offset);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Attribute Protocol (ATT)
 * 4.12.2 Read Long Characteristic Descriptors.
 *
 * @brief Read Long Characteristic Descriptors.
 * This API is used to read a Long Characteristic Descriptor from a
 * server when the client knows the Characteristic Descriptor Handle.
 *
 * When this API processing is completed, the API is invoked.

 void (*long_char_desp_readout)(const uint8_t status, const bool next,
 const twicAttValue_t *const resp);

 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint16_t handle
 * Characteristic Descriptor Handle of the Characteristic Descriptor
 * Value to be read.
 * @param const uint16_t offset
 * The Offset parameter shall be the offset within the Characteristic
 * Descriptor Value to be read. */
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientReadLongCharacteristicDescriptor(
  twicConnIface_t * const iface, const uint16_t handle, const uint16_t offset)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientLongRead(
      &iface->conn, TWIC_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR,
      handle, offset);
  return __ret;
}
#endif
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Attribute Protocol (ATT)
 * 4.9.4 Write Long Characteristic Values.
 *
 * @brief Write Long Characteristic Values.
 * This API is used to write a Long Characteristic Value to a server
 * when the client knows the Characteristic Value Handle.
 *
 * When this API processing is completed, the API is invoked.

 void (*long_char_value_writein_response)(const uint8_t status)

 * Client receives response from the server indicating whether the
 * writing of characteristic long value was success or resulted in
 * error.
 *
 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint16_t handle
 * Characteristic Value Handle of the Characteristic Value to be written.
 * @param const uint16_t offset
 * The Offset parameter shall be the offset within the Characteristic
 * Value to be written.
 * @param const uint16_t length
 * The length of the parameter Value.
 * @param const uint8_t * const value
 * The Value parameter shall be set to the part of the Attribute Value
 * that is being written. */
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICVALUE)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientWriteLongCharacteristicValue(
  twicConnIface_t * const iface, const uint16_t handle, const uint16_t offset,
  const uint16_t length, const uint8_t * const value)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientLongWrite(
      &iface->conn, TWIC_GATTCLIENTWRITELONGCHARACTERISTICVALUE,
      handle, offset, length, value);
  return __ret;
}
#endif 
/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 3].
 * Attribute Protocol (ATT)
 * 4.12.4 Write Long Characteristic Descriptors.
 *
 * @brief Write Long Characteristic Descriptors.
 * This API is used to write a Long Characteristic Descriptor to a
 * server when the client knows the Characteristic Descriptor Handle.
 *
 * When this API processing is completed, the API is invoked.

 void (*long_char_desp_writein_response)(const uint8_t status)

 * Client receives response from the server indicating whether the
 * writing of characteristic long descriptor was success or resulted
 * in error.
 *
 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const uint16_t handle
 * Characteristic Descriptor Handle of the Characteristic Descriptor
 * Value to be written.
 * @param const uint16_t offset
 * The Offset parameter shall be the offset within the Characteristic
 * Descriptor Value to be written.
 * @param const uint16_t length
 * The length of the parameter Value.
 * @param const uint8_t * const value
 * The Value parameter shall be set to the part of the Attribute Value
 * that is being written. */
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR)
static TZ1K_INLINE twicStatus_t
twicIfLeGattClientWriteLongCharacteristicDescriptor(
  twicConnIface_t * const iface, const uint16_t handle, const uint16_t offset,
  const uint16_t length, const uint8_t * const value)
{
  twicStatus_t __ret = TWIC_STATUS_UNDER_PROCESSING;
  if (!(iface->conn.req))
    __ret = _twicIfLeGattClientLongWrite(
      &iface->conn, TWIC_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR,
      handle, offset, length, value);
  return __ret;
}
#endif

#endif /* TWIC_INTERFACE_MACRO_FUNC */

#endif /* _TWIC_INTERFACE_INTERNAL_H_ */
