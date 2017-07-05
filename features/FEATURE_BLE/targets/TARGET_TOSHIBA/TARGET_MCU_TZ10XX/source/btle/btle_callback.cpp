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
#include <inttypes.h>

#include "mbed.h"
#include "debug.h"
#include "btle_callback.h"
#include "btle_config.h"
#include "btle_local.h"
#include "btle_utils.h"
#include "btle.h"

#include "ble/Gap.h"
#include "ble/GapEvents.h"
#include "tz1Gap.h"
#include "tz1GattServer.h"

/* Local variable for SMP callback */
static BTLEBondingInformation_t bondingInfo;
static BTLEEvent_t event;
static unsigned data_sent_count;                                /* Counter for notification Sent */
/* Callback of TWiC */
static void connectionCompleteCb(const void * const connif, const bool central, const twicConnectionComplete_t *const resp);
static void connectionUpdateCb(const void * const connif, const bool central, const twicConnectionUpdate_t *const resp);
#if defined(TWIC_BLE_HWIP_V41)
static void connectionParameterCb(const void * const connif, const bool central, const twicConnectionParameter_t *const request);
#endif
static void disconnectCb(const void * const connif, const bool central, const twicDisconnection_t *const resp);
static void mtuExchangeDemandCb(const void * const connif, const uint16_t client_rx_mtu_size);
static void mtuExchangeResultCb(const void * const connif, const twicMtuExchangeResult_t *const resp);
static void notificationSentCb(const void * const connif);
static void indicationConfirmationCb(const void * const connif, const uint8_t status);
static void queuedWritesCompleteCb(const void * const connif, const uint8_t status);
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT)
static void encryptCb(const void * const connif, const uint8_t *const encrypted_data);
#endif
static void readResultRssiCb(const void * const connif, const uint8_t rssi);
static void readResultTxPowerCb(const void * const connif, const uint8_t tx_power);
static void charDespWriteinDemandCb(const void * const connif, const twicAttValue_t *const resp, const uint8_t eidx);
static void charDespReadoutDemandCb(const void * const connif, const uint8_t eidx);
static void longCharDespReadoutDemandCb(const void * const connif, const uint16_t offset, const uint8_t eidx);
static void longCharDespPrepareWriteinDemandCb(const void * const connif, const uint16_t offset, const uint8_t eidx);
static void charDespExecWriteinDemandCb(const void * const connif, const twicAttValue_t *const resp, const uint8_t eidx);
static void charValReadoutDemandCb(const void * const connif, const uint8_t eidx);
static void charValMultiReadoutDemandCb(const void * const connif, const twicAttEid_t *const resp);
static void charValWriteinPostCb(const void * const connif, const twicAttValue_t *const resp, const uint8_t eidx);
static void charValWriteinDemandCb(const void * const connif, const twicAttValue_t *const resp, const uint8_t eidx);
static void longCharValReadoutDemandCb(const void * const connif, const uint16_t offset, const uint8_t eidx);
static void longCharValPrepareWriteinDemandCb(const void * const connif, const uint16_t offset, const uint8_t eidx);
static void charValExecWriteinDemandCb(const void * const connif, const twicAttValue_t *const resp, const uint8_t eidx);
static void isrDownstreamRequestCb(void);
static void isrWakeupRequestCb(void);
static void advertiseReposrtCb(const void* connif, const twicAdvReport_t *const retp);

static void primaryServiceCb(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint16_t end_group_handle, const uint64_t uuid_lsb,
    const uint64_t uuid_msb, const uint8_t uuid_len);
static void primaryServiceByUuidCb(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint16_t end_group_handle);
static void includeServiceCb(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint16_t included_service_attribute_handle,
    const uint16_t included_service_end_group_handle,
    const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
static void allCharOfServiceCb(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint8_t char_properties, const uint16_t char_value_handle,
    const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
static void allCharByUuidCb(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint8_t char_properties, const uint16_t char_value_handle,
    const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
static void allCharDespCb(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
static void charValueReadoutCb(
    const void * const connif, const uint8_t status,
    const twicAttValue_t *const resp);
static void charValueWriteinResponseCb(
    const void * const connif, const uint8_t status);
static void charDespReadoutCb(
    const void * const connif, const uint8_t status,
    const twicAttValue_t *const resp);
static void charDespWriteinResponseCb(
    const void * const connif, const uint8_t status);
static void notificationReceivedCb(
    const void * const connif, const uint8_t status,
    const uint16_t char_value_handle, const twicAttValue_t *const resp);
static void indicationReceivedCb(
    const void * const connif, const uint8_t status,
    const uint16_t char_value_handle, const twicAttValue_t *const resp);
static void charValueWriteinStartedCb(
    const void * const connif, const uint8_t status);
static void multiCharValueReadoutCb(
    const void * const connif, const uint8_t status,
    const twicAttValue_t *const resp);
static void reliableWriteinConfirmCb(
    const uint8_t status, const twicReliableWriteinResp_t *const resp);
static void longCharValueReadoutCb(
    const void * const connif, const uint8_t status, const bool next,
    const twicAttValue_t *const resp);
static void longCharDespReadoutCb(
    const void * const connif, const uint8_t status, const bool next,
    const twicAttValue_t *const resp);
static void longCharValueWriteinResponseCb(
    const void * const connif, const uint8_t status);
static void longCharDespWriteinResponseCb(
    const void * const connif, const uint8_t status);

static void smpPairingAcceptanceSentCb(const void *const connif, const uint8_t status);
static void smpPairingFailedCb(const void *const connif, const twicSmReasonCode_t reason);
static void smpStkGenerationMethodCb(const void *const connif, const uint8_t status, const twicStkGenMethod_t method);
static void smpStkGeneratedCb(const void *const connif, const twicStk_t *const stk);
static void smpEncryptionInfoCb(const void *const connif, const twicLtk_t *const ltk);
static void smpMasterIdentificationCb(const void *const connif, const twicEdiv_t *const ediv, const twicRand_t *const rand);
static void smpEncryptionInfoSentCb(const void *const connif, const twicLtk_t *const ltk);
static void smpMasterIdentificationSentCb(const void *const connif, const twicEdiv_t *const ediv, const twicRand_t *const rand);
static void smpEncryptionChangeCb(const void * const connif, const twicSmReasonCode_t reason,
                                  const uint8_t key_type, const bool encryption_enable, const uint8_t encryption_key_size);
static void smpEncryptionKeyRefreshCompleteCb(const void *const connif, twicSmReasonCode_t reason, const uint8_t key_type, const uint8_t encryption_key_size);
static void smpPairingCompleteEventCb(const void *const connif, const uint8_t status, const twicAuthInfo_t bits);
static void smpStkSessionRequestReplyEventCb(const void *const connif, const uint8_t status, const twicStk_t *const stk);
static void smpLtkSessionRequestReplyEventCb(const void *const connif, const uint8_t status, const twicLtk_t *const ltk);
static void smpIdentityInformationCb(const void *const connif, const twicIrk_t *const irk);
static void smpIdentityAddressInformationCb(const void *const connif, const bool address_type_random, const twicBdaddr_t *const identity);
static void smpSigningInformationCb(const void *const connif, const twicCsrk_t *const csrk);
static void smpIdentityInformationSentCb(const void *const connif, const twicIrk_t *const irk);
static void smpIdentityAddressInformationSentCb(const void *const connif, const bool address_type_random, const twicBdaddr_t *const identity);
static void smpSigningInformationSentCb(const void *const connif, const twicCsrk_t *const csrk);
static void smpOobInformationCb(const void *const conni);
static void smpStoreBondingInformationCb(const void *const connif, const bool address_type_random, const twicBdaddr_t *const identity, const bool erase);
static void smpBondingStateCb(const void *const connif, const twicSmReasonCode_t reason);

static void smprPairingDemandCb(const void *const connif, const twicPairingFeature_t *const resp);
static void smprInputPasskeyCb(const void *const connif);
static void smprDisplayPasskeyCb(const void *const connif);
static void smprInquiryBondingInformationCb(const void *const connif, const bool address_type_random, const twicBdaddr_t *const identity);

static void smpiPairingResponseCb(const void * const connif, const uint8_t status, const twicPairingFeature_t *const resp);
static void smpiSecurityRequestCb(const void * const connif, const bool bonded_device, const bool auth_req_bonding, const bool auth_req_mitm_protection);
static void smpiInputPasskeyCb(const void *const connif);
static void smpiDisplayPasskeyCb(const void *const connif);
static void smpiInquiryBondingInformationCb(const void *const connif, const bool address_type_random, const twicBdaddr_t *const identity);

const twicIfLeCb_t twic_common_callback = {
    connectionCompleteCb,
    connectionUpdateCb,
#if defined(TWIC_BLE_HWIP_V41)
    connectionParameterCb,
#endif
    disconnectCb,
    isrDownstreamRequestCb,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT)
    encryptCb,
#endif
    readResultRssiCb,
    readResultTxPowerCb,
    isrWakeupRequestCb,
    advertiseReposrtCb
};

const twicIfLeServerCb_t twic_server_callback = {
    mtuExchangeDemandCb,
    mtuExchangeResultCb,
    notificationSentCb,
    indicationConfirmationCb,
    queuedWritesCompleteCb,
    charDespWriteinDemandCb,
    charDespReadoutDemandCb,
    longCharDespReadoutDemandCb,
    longCharDespPrepareWriteinDemandCb,
    charDespExecWriteinDemandCb,
    charValReadoutDemandCb,
    charValMultiReadoutDemandCb,
    charValWriteinPostCb,
    charValWriteinDemandCb,
    longCharValReadoutDemandCb,
    longCharValPrepareWriteinDemandCb,
    charValExecWriteinDemandCb
};

const twicIfLeClientCb_t twic_client_callback = {
    mtuExchangeResultCb,
    primaryServiceCb,
    primaryServiceByUuidCb,
    includeServiceCb,
    allCharOfServiceCb,
    allCharByUuidCb,
    allCharDespCb,
    charValueReadoutCb,
    NULL, /* char_value_readout_using_uuid */
    charValueWriteinResponseCb,
    charDespReadoutCb,
    charDespWriteinResponseCb,
    notificationReceivedCb,
    indicationReceivedCb,
    charValueWriteinStartedCb,
    multiCharValueReadoutCb,
    reliableWriteinConfirmCb,
    longCharValueReadoutCb,
    longCharDespReadoutCb,
    longCharValueWriteinResponseCb,
    longCharDespWriteinResponseCb
};

/*
 * Initiator callback and responder callback are almost same.
 * If the callback use doneCallback functions,
 * the callback need to notify it is initiator callback or responder callback,
 * so the callback use doneCallback shall be separated.
 */
const twicIfLeSmpRCb_t twic_smp_responder_callback = {
    smprPairingDemandCb,                                /* Use doneCallback */
    smpPairingAcceptanceSentCb,
    smpPairingFailedCb,
    smpStkGenerationMethodCb,
    smprInputPasskeyCb,                                /* Use doneCallback */
    smprDisplayPasskeyCb,                                /* Use doneCallback */
    smpStkGeneratedCb,
    smpEncryptionInfoCb,
    smpMasterIdentificationCb,
    smpEncryptionInfoSentCb,
    smpMasterIdentificationSentCb,
    smpEncryptionChangeCb,
    smpEncryptionKeyRefreshCompleteCb,
    smpPairingCompleteEventCb,
    smpStkSessionRequestReplyEventCb,
    smpLtkSessionRequestReplyEventCb,
    smpIdentityInformationCb,
    smpIdentityAddressInformationCb,
    smpSigningInformationCb,
    smpIdentityInformationSentCb,
    smpIdentityAddressInformationSentCb,
    smpSigningInformationSentCb,
    smpOobInformationCb,
    smpStoreBondingInformationCb,
    smprInquiryBondingInformationCb,                    /* Use donCallback */
    smpBondingStateCb
};

const twicIfLeSmpICb_t twic_smp_initiator_callback = {
    smpiPairingResponseCb,                            /* Use doneCallback */
    smpiSecurityRequestCb,                            /* Use doneCallback */
    smpPairingFailedCb,
    smpStkGenerationMethodCb,
    smpiInputPasskeyCb,                                /* Use doneCallback */
    smpiDisplayPasskeyCb,                                /* Use doneCallback */
    smpStkGeneratedCb,
    smpEncryptionInfoCb,
    smpMasterIdentificationCb,
    smpEncryptionInfoSentCb,
    smpMasterIdentificationSentCb,
    smpEncryptionChangeCb,
    smpEncryptionKeyRefreshCompleteCb,
    smpPairingCompleteEventCb,
    smpIdentityInformationCb,
    smpIdentityAddressInformationCb,
    smpSigningInformationCb,
    smpIdentityInformationSentCb,
    smpIdentityAddressInformationSentCb,
    smpSigningInformationSentCb,
    smpOobInformationCb,
    smpStoreBondingInformationCb,
    smpiInquiryBondingInformationCb,                    /* Use doneCallback */
    smpBondingStateCb
};

/* Function definition */
static void connectionCompleteCb(const void * const connif, const bool central, const twicConnectionComplete_t *const resp)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_CONNECTION_COMPLETE;
    event.event_len = sizeof(twicConnectionComplete_t);
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    memcpy(&event.evt.gaps_evt.params, resp, sizeof(twicConnectionComplete_t));
    btle_done_callback(&event);
}

static void connectionUpdateCb(const void * const connif, const bool central, const twicConnectionUpdate_t *const resp)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_CONNECTION_UPDATE;
    event.event_len = sizeof(twicConnectionUpdate_t);
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    memcpy(&event.evt.gaps_evt.params, resp, sizeof(twicConnectionUpdate_t));
    btle_done_callback(&event);
}

#if defined(TWIC_BLE_HWIP_V41)
static void connectionParameterCb(const void * const connif, const bool central, const twicConnectionParameter_t *const request)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BLELIB_SM_CB_CONNECTION_PARAMETER;
    event.event_len = sizeof(twicConnectionParameter_t);
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    memcpy(&event.evt.gaps_evt.params, request, sizeof(twicConnectionParameter_t));
    btle_done_callback(&event);
}
#endif

static void disconnectCb(const void * const connif, const bool central, const twicDisconnection_t *const resp)
{
    data_sent_count = 0;
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_DISCONNECT;
    event.event_len = sizeof(twicDisconnection_t);
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    memcpy(&event.evt.gaps_evt.params, resp, sizeof(twicDisconnection_t));
    btle_done_callback(&event);
}

static void mtuExchangeDemandCb(const void * const connif, const uint16_t client_rx_mtu_size)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_MTU_EXCHANGE_DEMAND;
    event.evt.gatts_evt.conn_handle = connIface->conn.handle;
    event.evt.gatts_evt.params.mtu_exchange.mtu_size = client_rx_mtu_size;
    btle_done_callback(&event);
}

static void mtuExchangeResultCb(const void * const connif, const twicMtuExchangeResult_t *const resp)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_MTU_EXCHANGE_RESULT;
    event.evt.gatts_evt.conn_handle = connIface->conn.handle;
    memcpy(&event.evt.gatts_evt.params.mtu_exchange_result, resp, sizeof(twicMtuExchangeResult_t));
    btle_done_callback(&event);
}

static void notificationSentCb(const void * const connif)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    data_sent_count++;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_NOTIFICATION_SENT;
    event.evt.gatts_evt.conn_handle = connIface->conn.handle;
    event.evt.gatts_evt.params.hvx_count.count = data_sent_count;
    btle_done_callback(&event);
}

static void indicationConfirmationCb(const void * const connif, const uint8_t status)
{
    /* Empty */
}

static void queuedWritesCompleteCb(const void * const connif, const uint8_t status_code)
{
    /* Empty */
}

#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT)
static void encryptCb(const void * const connif, const uint8_t *const encrypted_data)
{
    /* Empty */
}
#endif

static void readResultRssiCb(const void * const connif, const uint8_t rssi)
{
    /* Empty */
}

static void readResultTxPowerCb(const void * const connif, const uint8_t tx_power)
{
    /* Empty */
}

static void constructWriteDataEvent(BTLEEvent_t *ble_event, const twicAttValue_t *const resp, const uint8_t eidx)
{
    ble_event->evt.gatts_evt.params.write.handle = eidx;
    ble_event->evt.gatts_evt.params.write.offset = 0;
    ble_event->evt.gatts_evt.params.write.len = resp->parameter_length;
    memcpy(ble_event->evt.gatts_evt.params.write.value, resp->value, resp->parameter_length);
}

static void charDespWriteinDemandCb(const void * const connif, const twicAttValue_t *const resp, const uint8_t eidx)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_CHAR_DESP_WRITEIN_DEMAND;
    event.evt.gatts_evt.conn_handle = connIface->conn.handle;
    constructWriteDataEvent(&event, resp, eidx);
    btle_done_callback(&event);
}

static void charDespReadoutDemandCb(const void * const connif, const uint8_t eidx)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_CHAR_DESP_READOUT_DEMAND;
    event.evt.gatts_evt.conn_handle = connIface->conn.handle;
    event.evt.gatts_evt.params.read.handle = eidx;
    event.evt.gatts_evt.params.read.offset = 0;
    btle_done_callback(&event);
}

static void longCharDespReadoutDemandCb(const void * const connif, const uint16_t offset, const uint8_t eidx)
{
    /* Empty */
}

static void longCharDespPrepareWriteinDemandCb(const void * const connif, const uint16_t offset, const uint8_t eidx)
{
    /* Empty */
}

static void charDespExecWriteinDemandCb(const void * const connif, const twicAttValue_t *const resp, const uint8_t eidx)
{
    /* Empty */
}

static void charValReadoutDemandCb(const void * const connif, const uint8_t eidx)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_CHAR_VAL_READOUT_DEMAND;
    event.evt.gatts_evt.conn_handle = connIface->conn.handle;
    event.evt.gatts_evt.params.read.handle = eidx;
    event.evt.gatts_evt.params.read.offset = 0;
    btle_done_callback(&event);
}

static void charValMultiReadoutDemandCb(const void * const connif, const twicAttEid_t *const resp)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_CHAR_VAL_MULTI_READOUT_DEMAND;
    event.evt.gatts_evt.conn_handle = connIface->conn.handle;
    btle_done_callback(&event);
}

static void charValWriteinPostCb(const void * const connif, const twicAttValue_t *const resp, const uint8_t eidx)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_CHAR_VAL_WRITEIN_POST;
    event.evt.gatts_evt.conn_handle = connIface->conn.handle;
    constructWriteDataEvent(&event, resp, eidx);
    btle_done_callback(&event);
}

static void charValWriteinDemandCb(const void * const connif, const twicAttValue_t *const resp, const uint8_t eidx)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_CHAR_VAL_WRITEIN_DEMAND;
    event.evt.gatts_evt.conn_handle = connIface->conn.handle;
    constructWriteDataEvent(&event, resp, eidx);
    btle_done_callback(&event);
}

static void longCharValReadoutDemandCb(const void * const connif, const uint16_t offset, const uint8_t eidx)
{
    /* Empty */
}

static void longCharValPrepareWriteinDemandCb(const void * const connif, const uint16_t offset, const uint8_t eidx)
{
    /* Empty */
}

static void charValExecWriteinDemandCb(const void * const connif, const twicAttValue_t *const resp, const uint8_t eidx)
{
    /* Empty */
}

static void isrDownstreamRequestCb(void)
{
    btle_scheduler_events_execute();
}

static void isrWakeupRequestCb(void)
{
    btle_scheduler_events_execute();
}

static void advertiseReposrtCb(const void * connif, const twicAdvReport_t *const resp)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;
    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_ADVERTISE_REPORT;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    memcpy(&event.evt.gaps_evt.params, resp, sizeof(twicAdvReport_t));
    btle_done_callback(&event);
}

static void primaryServiceCb(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint16_t end_group_handle, const uint64_t uuid_lsb,
    const uint64_t uuid_msb, const uint8_t uuid_len)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_DISCOVER_PRIMARY_SERVICE;
    event.evt.gattc_evt.conn_handle = connIface->conn.handle;
    event.evt.gattc_evt.status = status;
    event.evt.gattc_evt.error_handle = error_handle;
    event.evt.gattc_evt.prams.pri_serv_disc_resp.next = next;
    event.evt.gattc_evt.prams.pri_serv_disc_resp.attribute_handle = attribute_handle;
    event.evt.gattc_evt.prams.pri_serv_disc_resp.end_group_handle = end_group_handle;
    event.evt.gattc_evt.prams.pri_serv_disc_resp.uuid_lsb = uuid_lsb;
    event.evt.gattc_evt.prams.pri_serv_disc_resp.uuid_msb = uuid_msb;
    event.evt.gattc_evt.prams.pri_serv_disc_resp.uuid_len = uuid_len;
    btle_done_callback(&event);
}

static void primaryServiceByUuidCb(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint16_t end_group_handle)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_DISCOVER_PRIMARY_SERVICE_BY_UUID;
    event.evt.gattc_evt.conn_handle = connIface->conn.handle;
    event.evt.gattc_evt.status = status;
    event.evt.gattc_evt.error_handle = error_handle;
    event.evt.gattc_evt.prams.pri_serv_by_uuid_disc_resp.next = next;
    event.evt.gattc_evt.prams.pri_serv_by_uuid_disc_resp.attribute_handle = attribute_handle;
    event.evt.gattc_evt.prams.pri_serv_by_uuid_disc_resp.end_group_handle = end_group_handle;
    btle_done_callback(&event);
}

static void includeServiceCb(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint16_t included_service_attribute_handle,
    const uint16_t included_service_end_group_handle,
    const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len)
{
    /* Empty */
}

static void handleCharDiscoverEvent(BTLEEvent_t *ble_event, const uint8_t status, const bool next,
        const uint16_t error_handle, const uint16_t attribute_handle,
        const uint8_t char_properties, const uint16_t char_value_handle,
        const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len)
{
    ble_event->evt.gattc_evt.status = status;
    ble_event->evt.gattc_evt.error_handle = error_handle;
    ble_event->evt.gattc_evt.prams.char_disc_resp.next = next;
    ble_event->evt.gattc_evt.prams.char_disc_resp.attribute_handle = attribute_handle;
    ble_event->evt.gattc_evt.prams.char_disc_resp.char_properties = char_properties;
    ble_event->evt.gattc_evt.prams.char_disc_resp.char_value_handle = char_value_handle;
    ble_event->evt.gattc_evt.prams.char_disc_resp.uuid_lsb = uuid_lsb;
    ble_event->evt.gattc_evt.prams.char_disc_resp.uuid_msb = uuid_msb;
    ble_event->evt.gattc_evt.prams.char_disc_resp.uuid_len = uuid_len;
}

static void allCharOfServiceCb(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint8_t char_properties, const uint16_t char_value_handle,
    const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_DISCOVER_CHAR;
    event.evt.gattc_evt.conn_handle = connIface->conn.handle;
    handleCharDiscoverEvent(&event, status, next,
            error_handle, attribute_handle,
            char_properties, char_value_handle,
            uuid_lsb, uuid_msb, uuid_len);
    btle_done_callback(&event);
}

static void allCharByUuidCb(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint8_t char_properties, const uint16_t char_value_handle,
    const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_DISCOVER_CHAR_BY_UUID;
    event.evt.gattc_evt.conn_handle = connIface->conn.handle;
    handleCharDiscoverEvent(&event, status, next,
            error_handle, attribute_handle,
            char_properties, char_value_handle,
            uuid_lsb, uuid_msb, uuid_len);
    btle_done_callback(&event);
}

static void allCharDespCb(
    const void * const connif, const uint8_t status, const bool next,
    const uint16_t error_handle, const uint16_t attribute_handle,
    const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len)
{
    /* Empty */
}

static void charValueReadoutCb(
    const void * const connif, const uint8_t status,
    const twicAttValue_t *const resp)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_CHAR_VAL_READOUT;
    event.evt.gattc_evt.conn_handle = connIface->conn.handle;
    event.evt.gattc_evt.prams.read_resp.offset = 0;
    event.evt.gattc_evt.prams.read_resp.len = resp->parameter_length;
    memcpy(event.evt.gattc_evt.prams.read_resp.value, resp->value, resp->parameter_length);
    btle_done_callback(&event);
}

static void charValueWriteinResponseCb(
    const void * const connif, const uint8_t status)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_CHAR_VAL_WRITEIN_RESPONSE;
    event.evt.gattc_evt.conn_handle = connIface->conn.handle;
    event.evt.gattc_evt.status = status;
    btle_done_callback(&event);
}

static void charDespReadoutCb(
    const void * const connif, const uint8_t status,
    const twicAttValue_t *const resp)
{
    /* Empty */
}

static void charDespWriteinResponseCb(
    const void * const connif, const uint8_t status)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_CHAR_DESP_WRITEIN_RESPONSE;
    event.evt.gattc_evt.conn_handle = connIface->conn.handle;
    event.evt.gattc_evt.status = status;
    btle_done_callback(&event);
}

static void handleClientHVXEvent(BTLEEvent_t *ble_event, const uint8_t status,
        const uint16_t char_value_handle, const twicAttValue_t *const resp)
{
    ble_event->evt.gattc_evt.status = status;
    ble_event->evt.gattc_evt.prams.hvx_resp.handle = char_value_handle;
    ble_event->evt.gattc_evt.prams.hvx_resp.len = resp->parameter_length;
    memcpy(ble_event->evt.gattc_evt.prams.hvx_resp.value, resp->value, resp->parameter_length);
}

static void notificationReceivedCb(
    const void * const connif, const uint8_t status,
    const uint16_t char_value_handle, const twicAttValue_t *const resp)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_NOTIFICATION_RECEIVED;
    event.evt.gattc_evt.conn_handle = connIface->conn.handle;
    handleClientHVXEvent(&event, status, char_value_handle, resp);
    btle_done_callback(&event);
}

static void indicationReceivedCb(
    const void * const connif, const uint8_t status,
    const uint16_t char_value_handle, const twicAttValue_t *const resp)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_INDICATION_RECEIVED;
    event.evt.gattc_evt.conn_handle = connIface->conn.handle;
    handleClientHVXEvent(&event, status, char_value_handle, resp);
    btle_done_callback(&event);
}

static void charValueWriteinStartedCb(
    const void * const connif, const uint8_t status)
{
    /* Empty */
}

static void multiCharValueReadoutCb(
    const void * const connif, const uint8_t status,
    const twicAttValue_t *const resp)
{
    /* Empty */
}

static void reliableWriteinConfirmCb(
    const uint8_t status, const twicReliableWriteinResp_t *const resp)
{
    /* Empty */
}

static void longCharValueReadoutCb(
    const void * const connif, const uint8_t status, const bool next,
    const twicAttValue_t *const resp)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_CHAR_VAL_READOUT;
    event.evt.gattc_evt.conn_handle = connIface->conn.handle;
    /* Because this callback does not have offset factor so set 0 for offset value */
    event.evt.gattc_evt.prams.read_resp.offset = 0;
    event.evt.gattc_evt.prams.read_resp.len = resp->parameter_length;
    memcpy(event.evt.gattc_evt.prams.read_resp.value, resp->value, resp->parameter_length);
    btle_done_callback(&event);
}

static void longCharDespReadoutCb(
    const void * const connif, const uint8_t status, const bool next,
    const twicAttValue_t *const resp)
{
    /* Empty */
}

static void longCharValueWriteinResponseCb(
    const void * const connif, const uint8_t status)
{
    /* Empty */
}

static void longCharDespWriteinResponseCb(
    const void * const connif, const uint8_t status)
{
    /* Empty */
}
/* SMP common callback */
static void smpPairingAcceptanceSentCb(const void *const connif, const uint8_t status)
{
    /* Empty */
}

static void smpPairingFailedCb(const void *const connif, const twicSmReasonCode_t reason)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_PAIRING_COMPLETED;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.paring_status.status = reason;
    memset(&event.evt.gaps_evt.params.paring_status.auth_info, 0, sizeof(twicAuthInfo_t));

    btle_done_callback(&event);
}

static void smpStkGenerationMethodCb(const void *const connif, const uint8_t status, const twicStkGenMethod_t method)
{
    /* Empty */
}

static void smpStkGeneratedCb(const void *const connif, const twicStk_t *const stk)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_ENCRYPTION_INFORMATION;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.enc_changed.link_status = SecurityManager::ENCRYPTION_IN_PROGRESS;

    btle_done_callback(&event);
}

static void smpEncryptionInfoCb(const void *const connif, const twicLtk_t *const ltk)
{
    memcpy(&bondingInfo.remoteKey.ltk, ltk, sizeof(twicLtk_t));
}

static void smpMasterIdentificationCb(const void *const connif, const twicEdiv_t *const ediv, const twicRand_t *const rand)
{
    memcpy(&bondingInfo.remoteKey.ediv, ediv, sizeof(twicEdiv_t));
    memcpy(&bondingInfo.remoteKey.rand, rand, sizeof(twicRand_t));
}

static void smpEncryptionInfoSentCb(const void *const connif, const twicLtk_t *const ltk)
{
    memcpy(&bondingInfo.localKey.ltk, ltk, sizeof(twicLtk_t));
}

static void smpMasterIdentificationSentCb(const void *const connif, const twicEdiv_t *const ediv, const twicRand_t *const rand)
{
    memcpy(&bondingInfo.localKey.ediv, ediv, sizeof(twicEdiv_t));
    memcpy(&bondingInfo.localKey.rand, rand, sizeof(twicRand_t));
}

static void smpEncryptionChangeCb(const void * const connif, const twicSmReasonCode_t reason,
                                  const uint8_t key_type, const bool encryption_enable, const uint8_t encryption_key_size)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_ENCRYPTION_INFORMATION;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    if(encryption_enable) {
        event.evt.gaps_evt.params.enc_changed.link_status = SecurityManager::ENCRYPTED;
    } else {
        event.evt.gaps_evt.params.enc_changed.link_status = SecurityManager::NOT_ENCRYPTED;
    }
    event.evt.gaps_evt.params.enc_changed.key_type = key_type;
    event.evt.gaps_evt.params.enc_changed.key_size = encryption_key_size;

    btle_done_callback(&event);
}

static void smpEncryptionKeyRefreshCompleteCb(const void *const connif, twicSmReasonCode_t reason, const uint8_t key_type, const uint8_t encryption_key_size)
{
    /* Empty */
}

static void smpPairingCompleteEventCb(const void *const connif, const uint8_t status, const twicAuthInfo_t bits)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_PAIRING_COMPLETED;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.paring_status.status = (status == 0) ? TWIC_SMP_SUCCESS : TWIC_UNSPECIFIED_REASON;
    memcpy(&event.evt.gaps_evt.params.paring_status.auth_info, &bits, sizeof(twicAuthInfo_t));

    btle_done_callback(&event);
}

static void smpStkSessionRequestReplyEventCb(const void *const connif, const uint8_t status, const twicStk_t *const stk)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_ENCRYPTION_INFORMATION;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.enc_changed.link_status = SecurityManager::ENCRYPTION_IN_PROGRESS;

    btle_done_callback(&event);
}

static void smpLtkSessionRequestReplyEventCb(const void *const connif, const uint8_t status, const twicLtk_t *const ltk)
{
    memcpy(&bondingInfo.localKey.ltk, ltk, sizeof(twicLtk_t));
}

static void smpIdentityInformationCb(const void *const connif, const twicIrk_t *const irk)
{
    memcpy(&bondingInfo.remoteKey.irk, irk, sizeof(twicIrk_t));
}

static void smpIdentityAddressInformationCb(const void *const connif, const bool address_type_random, const twicBdaddr_t *const identity)
{
    /* Empty */
}

static void smpSigningInformationCb(const void *const connif, const twicCsrk_t *const csrk)
{
    memcpy(&bondingInfo.remoteKey.csrk, csrk, sizeof(twicCsrk_t));
}

static void smpIdentityInformationSentCb(const void *const connif, const twicIrk_t *const irk)
{
    memcpy(&bondingInfo.localKey.irk, irk, sizeof(twicIrk_t));
}

static void smpIdentityAddressInformationSentCb(const void *const connif, const bool address_type_random, const twicBdaddr_t *const identity)
{
    /* Empty */
}

static void smpSigningInformationSentCb(const void *const connif, const twicCsrk_t *const csrk)
{
    memcpy(&bondingInfo.localKey.csrk, csrk, sizeof(twicCsrk_t));
}

static void smpOobInformationCb(const void *const conni)
{
    /* Empty */
}

static void smpStoreBondingInformationCb(const void *const connif, const bool address_type_random, const twicBdaddr_t *const identity, const bool erase)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    if(erase) {
        bondingInfo.isKeyStored = false;
    } else {
        bondingInfo.isRandomAddr = address_type_random;
        memcpy(&bondingInfo.identityAddr[0], &identity->address[0], BLEProtocol::ADDR_LEN);
        bondingInfo.isKeyStored = true;
    }

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_BONDING_INFO_STORED;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;

    btle_done_callback(&event);
}

static void smpBondingStateCb(const void *const connif, const twicSmReasonCode_t reason)
{
    /* Empty */
}

/* SMP responder callback */
static void smprPairingDemandCb(const void *const connif, const twicPairingFeature_t *const resp)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_PAIRING_DEMAND;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.paring_params.initator_responder = BTLE_RESPONDER;
    event.evt.gaps_evt.params.paring_params.bond = resp->auth_req_bonding;
    event.evt.gaps_evt.params.paring_params.mitm = resp->auth_req_mitm_protection;
    event.evt.gaps_evt.params.paring_params.iocap = resp->io_capability;
    event.evt.gaps_evt.params.paring_params.status = BTLE_CALLBACK_STATUS_OK;
    btle_done_callback(&event);
}

static void smprInputPasskeyCb(const void *const connif)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_KEYBOARD_PASSKEY_DEMAND;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.passkey_keyboard.initator_responder = BTLE_RESPONDER;
    btle_done_callback(&event);
}

static void smprDisplayPasskeyCb(const void *const connif)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_DISPLAY_PASSKEY_DEMAND;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.passkey_display.initator_responder = BTLE_RESPONDER;
    btle_done_callback(&event);
}

static void smprInquiryBondingInformationCb(const void *const connif, const bool address_type_random, const twicBdaddr_t *const identity)
{
    bool address_match;
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    address_match =  btle_utils_compare_bd_adddress(bondingInfo.identityAddr, identity->address);

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_INQUIRY_BONDING_INFORMATION;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.inquiry_bond_info.initator_responder = BTLE_RESPONDER;
    event.evt.gaps_evt.params.inquiry_bond_info.status = BTLE_CALLBACK_STATUS_OK;

    if(!bondingInfo.isKeyStored ||
            bondingInfo.isRandomAddr != address_type_random ||
            !address_match) {
        event.evt.gaps_evt.params.inquiry_bond_info.status = BTLE_CALLBACK_STATUS_REJECT;
    }

    event.evt.gaps_evt.params.inquiry_bond_info.info_bonding = &bondingInfo;
    btle_done_callback(&event);
}

/* SMP initiator callbacks */
static void smpiPairingResponseCb(const void * const connif, const uint8_t status, const twicPairingFeature_t *const resp)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_PAIRING_RESPONSE;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.paring_params.initator_responder = BTLE_INITIATOR;
    event.evt.gaps_evt.params.paring_params.bond = resp->auth_req_bonding;
    event.evt.gaps_evt.params.paring_params.mitm = resp->auth_req_mitm_protection;
    event.evt.gaps_evt.params.paring_params.iocap = resp->io_capability;
    event.evt.gaps_evt.params.paring_params.status = status;

    btle_done_callback(&event);
}

static void smpiSecurityRequestCb(const void * const connif, const bool bonded_device, const bool auth_req_bonding, const bool auth_req_mitm_protection)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_SECURITY_REQUEST;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.security_request.initator_responder = BTLE_INITIATOR;
    event.evt.gaps_evt.params.security_request.bond = auth_req_bonding;
    event.evt.gaps_evt.params.security_request.mitm = auth_req_mitm_protection;
    event.evt.gaps_evt.params.security_request.bonded_device = bonded_device;

    btle_done_callback(&event);
}

static void smpiInputPasskeyCb(const void *const connif)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_KEYBOARD_PASSKEY_DEMAND;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.passkey_keyboard.initator_responder = BTLE_INITIATOR;
    btle_done_callback(&event);
}

static void smpiDisplayPasskeyCb(const void *const connif)
{
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_DISPLAY_PASSKEY_DEMAND;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.passkey_display.initator_responder = BTLE_INITIATOR;
    btle_done_callback(&event);
}

static void smpiInquiryBondingInformationCb(const void *const connif, const bool address_type_random, const twicBdaddr_t *const identity)
{
    bool address_match;
    twicConnIface_t * const connIface = (twicConnIface_t *)connif;

    address_match =  btle_utils_compare_bd_adddress(bondingInfo.identityAddr, identity->address);

    memset(&event, 0, sizeof(BTLEEvent_t));
    event.event_id = BTLE_CB_INQUIRY_BONDING_INFORMATION;
    event.evt.gaps_evt.conn_handle = connIface->conn.handle;
    event.evt.gaps_evt.params.inquiry_bond_info.initator_responder = BTLE_INITIATOR;
    event.evt.gaps_evt.params.inquiry_bond_info.status = BTLE_CALLBACK_STATUS_OK;
    if(!bondingInfo.isKeyStored ||
            bondingInfo.isRandomAddr != address_type_random ||
            !address_match) {
        event.evt.gaps_evt.params.inquiry_bond_info.status = BTLE_CALLBACK_STATUS_REJECT;
    }
    event.evt.gaps_evt.params.inquiry_bond_info.info_bonding = &bondingInfo;
    btle_done_callback(&event);
}

/** Initialize global variable use in BTLE callback functions.
 *
 *  @param 
 *      NONE.
 *
 *  @returns
 *      NONE.
 */
void btle_callback_on_init_variable(void)
{
    data_sent_count = 0;
}

/** Call mbed display passkey functions callback
 *
 *  @param conn_handle              Connection handle.
 *  @param passkey                  TWiC passkey pointer.
 *
 *  @returns
 *    NONE.
 */
void btle_callback_display_passkey_event(uint16_t conn_handle, const uint8_t *passkey)
{
    SecurityManager::Passkey_t ble_passkey;

    if(passkey != NULL) {
        BTLE_CONVERT_TWIC_TO_BLE_PASSKEY(ble_passkey, passkey);
    } else {
        memset(&ble_passkey, 0, sizeof(SecurityManager::Passkey_t));
    }

    tz1SecurityManager::getInstance().processPasskeyDisplayEvent(conn_handle, ble_passkey);
}
