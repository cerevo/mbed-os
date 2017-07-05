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
#include "mbed_error.h"
#include "btle.h"
#include "debug.h"
#include "btle_callback.h"
#include "btle_utils.h"
#include "tz1BLE.h"
#include "tz1GattClient.h"

volatile uint32_t btle_intr_status = 0;

#define BTLE_IRQ_DISABLE_SAVE() {             \
    btle_intr_status = __get_PRIMASK();       \
    if (!btle_intr_status) {                  \
        __disable_irq();                      \
    }                                         \
}

#define BTLE_IRQ_ENABLE_RESTORE() {          \
    if (!btle_intr_status) {                 \
        __enable_irq();                      \
    }                                        \
}

#define BTLE_IRQ_GET_IPSR() __get_IPSR()

/* Data for TWiC */
static uint8_t interface_id;                                    /* Interface id */
twicConnIface_t conn_iface;                                     /* Interface substance */
static twicEntry_t *entries[BTLE_ENTRY_SIZE_MAX];               /* Entry pointer list */
static twicEntry_t entry_array[BTLE_ENTRY_SIZE_MAX];            /* Entry substance */
static int entry_size;                                          /* Entry num */
static uint8_t register_entry_index;                            /* Index used for register entry */
static uint8_t btle_handle_events_pending;
/* Entry Index of GA Service */
uint8_t eidx_ga_device_name;
uint8_t eidx_ga_appearance;
uint8_t eidx_ga_pref_conn;
/* Default value of GA and GATT generic service */
static uint8_t btle_ga_appearance[] = {BTLE_DEFAULT_GA_CHAR_APPEARANCE};
static uint8_t btle_ga_device_name[BTLE_DEVICE_NAME_MAX_LENGTH];
static uint8_t btle_gatt_service_changed[] = {BTLE_DEFAULT_GATT_CHAR_SERVICE_CHANGED};
static uint8_t client_desc[] = {0x00u, 0x00u};
static uint8_t btle_ga_pref_conn[] = {0x28u, 0x00u, 0x64u, 0x00u, 0x04u, 0x00u, 0x90u, 0x01u};
static BTLE_Address device_addr;                                                /* Peripheral BDADDR */
static uint64_t command_flag = 0u;                   /* Command flag */
static BTLE_AdvertisingData advertising_data;
static BTLE_ConnectionInfo connected_device_info;
/*
 * Event data ring buffer
 */
typedef struct _BTLEEventDataRingBuffer_t {
    BTLEEvent_t buffer[BTLE_INNER_EVENT_BUF_NUM];
    volatile uint8_t tail;
    volatile uint8_t head;
} BTLEEventDataRingBuffer_t;

static BTLEEventDataRingBuffer_t event_buffer;
static BTLEEvent_t btle_event;
/* Gap role of device */
BTLE_Gap_Role_t gap_role;
/* Variables for each demand */
static BTLE_State_Connect_t btle_status_connect;           /* Connection status */

static uint32_t current_mtu;                                    /* Current MTU size */

/* Mask bit for enable notification or indication */
static uint64_t notify_indicate_mask;
/* Security manager variable */
static bool security_initialized;                                                    /* Flag security initialization >*/
static BTLESecurityInformation_t security_info;                                      /* Store role for security manager>*/
static SecurityManager::SecurityMode_t security_mode;
static SecurityManager::LinkSecurityStatus_t  security_link_status;
static twicAuthInfo_t auth_info;
static uint8_t has_passkey;

BTLEBondingInformation_t *bonding_inquiry_info = NULL;                                   /* Bonding infomation for bonding inquiry */

/* Command flag bit */
#define BTLE_FLAG_CONNECTED                             (1ULL << 1)        /* Connection is connected */
#define BTLE_FLAG_CONNECTION_DISCONNECTED               (1ULL << 2)        /* Connection is disconnected */
#define BTLE_FLAG_CONNECTION_UPDATE                     (1ULL << 3)        /* Update connection request */
#define BTLE_FLAG_ADV_REPORT                            (1ULL << 4)        /* ADV report */
#define BTLE_FLAG_EXG_MTU_RESP                          (1ULL << 5)        /* ExgMtuResponse */
#define BTLE_FLAG_EXG_MTU_RESULT                        (1ULL << 6)        /* ExgMtuResponse */
#define BTLE_FLAG_NOTIFICATION_SENT                     (1ULL << 7)        /* Notification sent */
#define BTLE_FLAG_CHAR_MULTI_READOUT_RESP               (1ULL << 8)        /* Characteristics multi read out response */
#define BTLE_FLAG_CHAR_READOUT_RESP                     (1ULL << 9)        /* Characteristics read out response */
#define BTLE_FLAG_CHAR_WRITEIN_RESP                     (1ULL << 10)       /* Characteristics write in response */
#define BTLE_FLAG_CHAR_WRITE_POST                       (1ULL << 11)       /* Characteristics write post */
#define BTLE_FLAG_DESP_READOUT_RESP                     (1ULL << 12)       /* Descriptor read out response */
#define BTLE_FLAG_DESP_WRITEIN_RESP                     (1ULL << 13)       /* Descriptor write in response */

#define BTLE_FLAG_DISCOVERED_PRIMARY_SERVICE            (1ULL << 14)       /* GattClient discovered primary service */
#define BTLE_FLAG_DISCOVERED_PRIMARY_SERVICE_BY_UUID    (1ULL << 15)       /* GattClient discovered primary service by UUID */
#define BTLE_FLAG_DISCOVERED_CHAR                       (1ULL << 16)       /* GattClient discovered characteristics */
#define BTLE_FLAG_CHAR_VAL_READOUT                      (1ULL << 17)       /* GattClient characteristic readout */
#define BTLE_FLAG_CHAR_VAL_WRITEIN_RESPONSE             (1ULL << 18)       /* GattClient characteristic write in response */
#define BTLE_FLAG_CHAR_DESP_WRITEIN_RESPONSE            (1ULL << 19)       /* GattClient descriptor write in response */
#define BTLE_FLAG_NOTIFICATION_RECEIVED                 (1ULL << 20)       /* GattClient notification is reveived */
#define BTLE_FLAG_INDICATION_RESP                       (1ULL << 21)       /* GattClient Indication confirmation response */

#define BTLE_FLAG_LOWPOWER                              (1ULL << 22)       /* Lowpower setting */
#define BTLE_FLAG_PAIRING_CONFIRM                       (1ULL << 23)       /* Pairing confirm */
#define BTLE_FLAG_REQUEST_SECURITY                      (1ULL << 24)       /* Request security */
#define BTLE_FLAG_DISPLAY_PASSKEY_REPLY                 (1ULL << 25)       /* Display passkey reply */
#define BTLE_FLAG_KEYBOARD_PASSKEY_REPLY                (1ULL << 26)       /* Input passkey reply */
#define BTLE_FLAG_BONDING_REPLY                         (1ULL << 27)       /* Reply for inquiry bonding information */
#define BTLE_FLAG_REQUEST_PAIRING                       (1ULL << 28)       /* Request pairing */
#define BTLE_FLAG_START_ENCRYPTION                      (1ULL << 29)       /* Start encryption */
#define BTLE_FLAG_SECURITY_REQ_RESP                     (1ULL << 30)       /* Response for security request */
#define BTLE_FLAG_PAIRING_COMPLETED                     (1ULL << 31)       /* Pairing completed */
#define BTLE_FLAG_ENCRYPTION_CHANGED                    (1ULL << 32)       /* The Link change status */
#define BTLE_FLAG_BONDING_INFO_STORED                   (1ULL << 33)       /* The bonding information is stored */
#define BTLE_FLAG_CONNECTION_PARAMETER                  (1ULL << 34)      /* Connection parameter reply */
#define BTLE_FLAG_MAX_BIT                               (35)              /* Maximum bit of command for internal event execute*/
#define BTLE_FLAG_UPDATE_VALUE                          (1ULL << BTLE_FLAG_MAX_BIT)        /* UpdateValue is called */

/* Utils functions use internal BTLE brigde */
/* Initialize the GATT and register callback functions */
static int BTLE_registerService( const twicIfLeServerCb_t *server_callbacks,
        const twicIfLeClientCb_t *client_callbacks,
        const twicIfLeSmpICb_t *smp_i_callbacks,
        const twicIfLeSmpRCb_t *smp_r_callbacks,
        const twicIfLeCb_t *le_callbacks);
/* TWiC DB initialize */
static int BTLE_dbInit(uint64_t baudrate,
                                const uint64_t *peripheral_bdaddr);
/* Initialize GA basic service 0x1800 */
static int BTLE_gaServiceInit();
/* Initialize GA basic service 0x1801 */
static int BTLE_gattServiceInit();
/* Function do and checks result of TWiC DB API */
static int BTLE_twicDbApiIsDone(const uint8_t fidx,
        const uint8_t eidx, twicStatus_t status);
/* Function do and check result of TWiC API */
static int BTLE_twicApiIsDone(const uint8_t fidx, twicStatus_t status);
/* Function initializes TZ1EM */
static void BTLE_configureTz1emOnInitialize(void);
/* Function Finalize TZ1EM */
static void BTLE_configureTz1emOnFinalize(void);
/* Function disable TZ1EM lowpower */
static void BTLE_configureTz1emOnLowPowerOff(void);
/* Function enable TZ1EM lowpower */
static void BTLE_configureTz1emOnLowPowerOn(void);
/* Function resets or initialize variable when device disconnected */
static void BTLE_initializeOnDisconnect(void);
/* Function adds internal Function Response into minar Scheduler */
static void BTLE_doEvent(BTLEEvent_t *event);
/* Function checks API done with Retry message */
static int BTLE_twicApiIsDoneWithRetry(const uint8_t fidx, twicStatus_t status);
/* Function gets response value when characteristic need write authorization */
static uint8_t BTLE_getServerRespWriteAuthorizationDemand(uint16_t conn_handle,
        GattCharacteristic *p_char,
        uint16_t offset,
        uint16_t len,
        uint8_t *value);
/* Function gets response value when characteristic need read authorization */
static uint8_t BTLE_getServerRespReadAuthorizationDemand(uint16_t conn_handle,
        GattCharacteristic *p_char);
/* Function gets TWiC Security Manager code */
static twicSmReasonCode_t BTLE_getSmReasonCode(uint8_t status);
/* Function gets mbed BLE Security mode from twic authorization information */
static void BTLE_getSecurityModeFromAuthorizationParams(twicAuthInfo_t *bits, SecurityManager::SecurityMode_t *sm);
/* Function enqueues BTLE callback event to buffer */
static TZ1K_INLINE int BTLE_enqueueEventData(BTLEEvent_t *event);
/* Function dequeues BTLE callback event to buffer */
static TZ1K_INLINE int BTLE_dequeueEventData(BTLEEvent_t *event);
/* Function checks BTLE callback event buffer is avaiable or not */
static TZ1K_INLINE int BTLE_bufferEventIsAvaiable(void);
/* Function checks BTLE callback event buffer is full or not */
static TZ1K_INLINE int BTLE_bufferEventIsFull(void);
/* Function checks status of internal Buffer, that uses for preventing
 internal buffer overflow */
static bool BTLE_internIsBusy(void);
/**********************************************************
************ Implement BTLE public functions **************
***********************************************************
*/

/* APIs for tz1BLE device */
/** Initialize BLE Controller extension, GATT service and register Generic Access service (0x1800)
 *  Generic Attribute service (0x1801)
 *
 *  @param          NONE
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_init()
{
    int status;
    uint8_t dev_name[] = {BTLE_DEFAULT_GA_CHAR_DEVICE_NAME};
    /* Initialize BTLE variable */
    current_mtu = BTLE_ATT_MTU_DEFAULT;
    security_initialized = false;
    interface_id = 0;
    entry_size = BTLE_ENTRY_SIZE_MAX;
    register_entry_index = 1;
    btle_handle_events_pending = 0;
    bonding_inquiry_info = NULL;
    eidx_ga_device_name = 0;
    eidx_ga_appearance = 0;
    eidx_ga_pref_conn = 0;
    /* set default this device address */
    device_addr.value = 0xE2D920561478;
    /* set device address is ramdom static type */
    device_addr.type = BTLE_RANDOM_STATIC_ADDRESS;

    memset(btle_ga_device_name, 0, BTLE_DEVICE_NAME_MAX_LENGTH);
    /* set device name default */
    memcpy(btle_ga_device_name, dev_name, sizeof(dev_name));
    memset(&conn_iface, 0, sizeof(twicConnIface_t));
    memset(entries, 0, sizeof(entries));
    memset(entry_array, 0, sizeof(entry_array));
    memset(&notify_indicate_mask, 0, sizeof(uint64_t));
    memset(&auth_info, 0, sizeof(twicAuthInfo_t));
    memset(&event_buffer, 0, sizeof(BTLEEventDataRingBuffer_t));
    memset(&advertising_data, 0, sizeof(BTLE_AdvertisingData));
    memset(&connected_device_info, 0, sizeof(BTLE_ConnectionInfo));

    for(int i = 0; i < entry_size; ++i ) {
        entries[i] = &entry_array[i];
    }
    conn_iface.size = entry_size;
    conn_iface.entries = &entries[0];
    btle_status_connect = BTLE_STATE_OFFLINE;
    btle_callback_on_init_variable();
    /* Set priority of UART1 (Serial) lower than UART2 (in BLE)for execute
      BLE function inside serial interrupt handler */
    NVIC_SetPriority (UART1_IRQn, 1);
    status = BTLE_registerService(&twic_server_callback,
                                  &twic_client_callback,
                                  &twic_smp_initiator_callback,
                                  &twic_smp_responder_callback,
                                  &twic_common_callback);
    if(status != BTLE_OK) {
        return status;
    }
    /* Set public Address */
    status = BTLE_dbInit(BTLE_UART_BAUDRATE, &device_addr.value);
    if(status != BTLE_OK) {
        return status;
    }
    /* Set default BLE device type is random address */
    if(device_addr.type == BTLE_RANDOM_STATIC_ADDRESS) {
        status = btle_gap_set_address(device_addr);
        if(status != BTLE_OK) {
            return status;
        }
    }

    status = BTLE_gaServiceInit();
    if(status != BTLE_OK) {
        return status;
    }

    status = BTLE_gattServiceInit();
    if(status != BTLE_OK) {
        return status;
    }

    command_flag |= BTLE_FLAG_LOWPOWER;

    return BTLE_OK;
}

/** Finalize BLE Controller extension
 *
 *  @param          NONE
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_finalize(void)
{
    twicIfGattDeregistration(interface_id);
    twicIfLeIoFinalize(false);
    BTLE_configureTz1emOnFinalize();
    return BTLE_OK;
}

/* APIs for tz1BLE security manager */
/** Enable the BLE Security Manager. 
 *
 *  @param enableBonding    Allow for bonding.
 *  @param requireMITM      Require protection for man-in-the-middle attacks.
 *  @param iocaps           The I/O capabilities of this peripheral.
 *  @param passkey          To specify a static passkey.
 *
 *  @returns
 *    BLE_ERROR_NONE.   Success
 *    other.            Error
 */
ble_error_t btle_security_init(bool                                         enableBonding,
                               bool                                        requireMITM,
                               SecurityManager::SecurityIOCapabilities_t   iocaps,
                               const SecurityManager::Passkey_t            passkey)
{
    if (security_initialized) {
        return BLE_ERROR_ALREADY_INITIALIZED;
    }

    if (passkey) {
        BTLE_CONVERT_BLE_TO_TWIC_PASSKEY(passkey, security_info.passKey.key);
        has_passkey = 1;
    } else {
        has_passkey = 0;
    }

    security_info.securityParams.auth_req_bonding = enableBonding;
    security_info.securityParams.auth_req_mitm_protection = requireMITM;

    security_info.securityParams.io_capability = iocaps;

    security_mode = SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK;
    security_link_status = SecurityManager::NOT_ENCRYPTED;

    security_initialized = true;
    return BLE_ERROR_NONE;
}

/** Get security manager is enable or disable
 *
 *  @param          NONE
 *
 *  @returns
 *    true.     Enable
 *    false.    Disable
 */
bool btle_security_has_initialized(void)
{
    return security_initialized;
}

/** Get the security status of a connection.
 *
 *  @param connectionHandle     Handle to identify the connection.
 *  @param securityStatusP      Security status.
 *
 *  @returns
 *    BLE_ERROR_NONE.   Success
 *    other.            Error
 */
ble_error_t btle_security_get_link(Gap::Handle_t connectionHandle,
                                   SecurityManager::LinkSecurityStatus_t *securityStatusP)
{
    (void)connectionHandle;
    *securityStatusP = security_link_status;
    return BLE_ERROR_NONE;
}

/** Set the security mode on a connection.
 *
 *  @param connectionHandle     Handle to identify the connection..
 *  @param securityMode         Requested security mode.
 *
 *  @returns
 *    BLE_ERROR_NONE.   Success
 *    other.            Error
 */
ble_error_t btle_security_set_link(Gap::Handle_t connectionHandle,
                                   SecurityManager::SecurityMode_t securityMode)
{
    if(connectionHandle != conn_iface.conn.handle) {
        return BLE_ERROR_INVALID_PARAM;
    }

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BLE_ERROR_INVALID_STATE;
    }
    security_mode = securityMode;
    if(gap_role == BTLE_DEVICE_IS_PERIPHERAL) {
        command_flag |= BTLE_FLAG_REQUEST_SECURITY;
    } else {             /* gap_role == BTLE_DEVICE_IS_CENTRAL */
        command_flag |= BTLE_FLAG_REQUEST_PAIRING;
    }

    return BLE_ERROR_NONE;
}

/* APIs BLE for GATT server*/
/** Begin Service Creation
 *
 *  @param serviceHandle        The unique number to handle the sevice.
 *  @param sevice_uuid          The service uuid 
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gatts_begin_service_creation(uint8_t *serviceHandle,
                                     BTLE_UUID sevice_uuid)
{
    twicStatus_t status;
    int api_done;

    if(serviceHandle !=NULL) {
        *serviceHandle = register_entry_index;
    }

    status = twicIfLeGattDbBeginServiceCreation(&conn_iface, register_entry_index,
             sevice_uuid.uuid_lsb, sevice_uuid.uuid_msb, sevice_uuid.uuid_type);
    api_done = BTLE_twicDbApiIsDone( TWIC_LEGATTDBBEGINSERVICECREATION, register_entry_index, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    register_entry_index++;
    return BTLE_OK;
}

/** End the Service Creation
 *
 *  @param serviceHandle        The unique number to handle the sevice.
 *  @param sevice_uuid          The service uuid 
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gatts_end_service_creation(uint8_t serviceHandle,
                                     BTLE_UUID sevice_uuid)
{
    twicStatus_t status;
    int api_done;

    status = twicIfLeGattDbEndServiceCreation(&conn_iface, serviceHandle);
    api_done = BTLE_twicDbApiIsDone(TWIC_LEGATTDBENDSERVICECREATION, serviceHandle, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Add characteristic declaration.
 *
 *  @param requiredSecurity     mbed security mode for this characteristic.
 *  @param serviceHandle        The unique number to handle the sevice contain the characteristic.
 *  @param charHandle           The unique number to handle the characteristic.
 *  @param char_uuid            The BTLE_UUID of this characteristic.
 *  @param readAuthorization    The read authorization is enabled or not.
 *  @param writeAuthorization   The write authorization is enabled or not.
 *  @param properties           The 8-bit field containing the characteristic's properties.
 *  @param ptr_value            The memory holding the initial value.
 *  @param length               The length in bytes of this characteristic's value.
 *  @param maxLength            The max length in bytes of this characteristic's value.
 *  @param has_variable_len     Whether the attribute's value length changes over time.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gatts_add_characteristics(SecurityManager::SecurityMode_t requiredSecurity,
                             uint8_t serviceHandle,
                             uint8_t *charHandle,
                             BTLE_UUID char_uuid,
                             bool readAuthorization,
                             bool writeAuthorization,
                             uint8_t properties,
                             const uint8_t *ptr_value,
                             uint16_t length,
                             uint16_t maxLength,
                             bool has_variable_len)
{
    int api_done;
    twicStatus_t status;
    uint16_t permission = 0u;
    uint8_t charIndex = register_entry_index;

    if(charHandle != NULL)  {
        *charHandle = charIndex;
    }

    if(requiredSecurity > security_mode) {
        security_mode = requiredSecurity;
    }

    if (properties & (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY |
                      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE |
                      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ)) {
        switch (requiredSecurity) {
            case SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK :
                permission = BTLE_PERMISSION_READ ;
                break;
            case SecurityManager::SECURITY_MODE_ENCRYPTION_NO_MITM :
                permission = BTLE_PERMISSION_READ | BTLE_PERMISSION_ENCRYPTION | BTLE_PERMISSION_KEY16;
                break;
            case SecurityManager::SECURITY_MODE_ENCRYPTION_WITH_MITM :
                permission = BTLE_PERMISSION_READ | BTLE_PERMISSION_ENCRYPTION | \
                             BTLE_PERMISSION_MITM | BTLE_PERMISSION_KEY16;
                break;
            case SecurityManager::SECURITY_MODE_SIGNED_NO_MITM :
                /* Not supported in current version */
                break;
            case SecurityManager::SECURITY_MODE_SIGNED_WITH_MITM :
                /* Not supported in current version */
                break;
            default:
                permission = BTLE_PERMISSION_READ;
                break;
        };

    } else if (properties & (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE|
                             GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE)) {
        switch (requiredSecurity) {
            case SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK :
                permission = BTLE_PERMISSION_WRITE;
                break;
            case SecurityManager::SECURITY_MODE_ENCRYPTION_NO_MITM :
                permission = BTLE_PERMISSION_WRITE | BTLE_PERMISSION_ENCRYPTION_WRITE | BTLE_PERMISSION_KEY16;
                break;
            case SecurityManager::SECURITY_MODE_ENCRYPTION_WITH_MITM :
                permission = BTLE_PERMISSION_WRITE | BTLE_PERMISSION_ENCRYPTION_WRITE | BTLE_PERMISSION_MITM_WRITE | BTLE_PERMISSION_KEY16;
                break;
            case SecurityManager::SECURITY_MODE_SIGNED_NO_MITM :
                /* Not supported in current version */
                break;
            case SecurityManager::SECURITY_MODE_SIGNED_WITH_MITM :
                /* Not supported in current version */
                break;
            default:
                permission = BTLE_PERMISSION_WRITE;
                break;
        };
    } else {
        permission = BTLE_PERMISSION_WRITE + BTLE_PERMISSION_READ;
    }

    if(readAuthorization && !(requiredSecurity & SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK)) {
        permission |= BTLE_PERMISSION_AUTHENTICATION;
    }
    if(writeAuthorization && !(requiredSecurity & SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK)) {
        permission = BTLE_PERMISSION_AUTHENTICATION_WRITE + BTLE_PERMISSION_AUTHENTICATION;
    }

    status = twicIfLeGattDbAddCharacteristics(&conn_iface, serviceHandle, charIndex, properties,
             char_uuid.uuid_lsb, char_uuid.uuid_msb, char_uuid.uuid_type);
    api_done = BTLE_twicDbApiIsDone(TWIC_LEGATTDBADDCHARACTERISTICS, charIndex, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    if(has_variable_len) {
        status = twicIfLeGattDbSetCharacteristicsVl(&conn_iface, charIndex, permission,
                 maxLength, length, ptr_value,
                 char_uuid.uuid_lsb, char_uuid.uuid_msb, char_uuid.uuid_type);
        api_done = BTLE_twicDbApiIsDone(TWIC_LEGATTDBSETCHARACTERISTICSVL, charIndex, status);
        if(api_done != BTLE_OK) {
            return api_done;
        }
    } else {
        status = twicIfLeGattDbSetCharacteristics(&conn_iface, charIndex, permission,
                 length, ptr_value, char_uuid.uuid_lsb, char_uuid.uuid_msb, char_uuid.uuid_type);
        api_done = BTLE_twicDbApiIsDone(TWIC_LEGATTDBSETCHARACTERISTICS, charIndex, status);
        if(api_done != BTLE_OK) {
            return api_done;
        }
    }

    register_entry_index++;

    if((properties & (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY |
                      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE))) {
        BTLE_UUID cccd_uuid = {BTLE_CHAR_CLIENT_CONFIG_DESC_UUID, 0, BTLE_UUID_16};
        api_done = btle_gatts_add_descriptor(charIndex, NULL, cccd_uuid,
                                  BTLE_PERMISSION_READ | BTLE_PERMISSION_WRITE,
                                  client_desc, sizeof(client_desc));
        if(api_done != BTLE_OK) {
            return api_done;
        }
    }

    return BTLE_OK;
}

/** Add descriptor declaration.
 *
 *  @param charHandle           The unique number to handle the characteristic contain the descriptor.
 *  @param desc_uuid            The BTLE_UUID of this descriptor.
 *  @param permission           The 8-bit field containing the characteristic's properties.
 *  @param ptr_value            The memory holding the initial value.
 *  @param length               The length in bytes of this descriptor's value.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gatts_add_descriptor(uint8_t charHandle,
                        uint8_t *descHandle,
                        BTLE_UUID desc_uuid,
                        uint8_t permission,
                        const uint8_t *ptr_value,
                        uint16_t length)
{
    int api_done;
    twicStatus_t status;
    uint8_t descIndex = register_entry_index;
    if(descHandle != NULL) {
        *descHandle = descIndex;
    }

    status = twicIfLeGattDbSetDescriptor(&conn_iface, charHandle, descIndex, permission,
                                         length, ptr_value, desc_uuid.uuid_lsb, desc_uuid.uuid_msb, desc_uuid.uuid_type);
    api_done = BTLE_twicDbApiIsDone(TWIC_LEGATTDBSETDESCRIPTOR, descIndex, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    register_entry_index++;
    return BTLE_OK;
}

/** Update the value of a characteristic/descriptor on the local GATT server.
 *
 *  @param eidx                 Handle for the value attribute of the characteristic.
 *  @param buffer               A pointer to a buffer holding the new value.
 *  @param len                  Size of the new value (in bytes).
 *  @param localOnly            Should this update be kept on the local
 *                              GATT server regardless of the state of the
 *                              notify/indicate flag in the CCCD for this
 *                              Characteristic? If set to true, no notification
 *                              or indication is generated.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gatts_update_value(uint8_t eidx, const uint8_t buffer[], uint16_t len, bool localOnly)
{
    twicStatus_t status = TWIC_STATUS_OK;
    int result;
    uint8_t fidx = 0;
    uint64_t enable_update = 0;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    /* Why can not get property from conn_iface */
    GattCharacteristic *p_char = tz1GattServer::getInstance().getCharacteristicFromHandle(eidx);
    command_flag |= BTLE_FLAG_UPDATE_VALUE;
    enable_update |= (1u << eidx);

    if(!localOnly && (notify_indicate_mask & enable_update)) {

        if((current_mtu - 3 < len) || p_char == NULL) {
            return BTLE_ERROR_PARAMETER;
        }

        if(btle_status_connect == BTLE_STATE_OFFLINE) {
            return BTLE_ERROR_CANNOT_EXECUTE;
        }

        if(p_char->getProperties() & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY) {
            fidx = TWIC_GATTNOTIFICATION;
            status = twicIfGattNotification(&conn_iface, eidx, len, buffer);
        } else if(p_char->getProperties() & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE) {
            fidx = TWIC_GATTINDICATION;
            status = twicIfGattIndication(&conn_iface, eidx, len, buffer);
        }
    } else {
        if(current_mtu - 1 < len ) {
            return BTLE_ERROR_PARAMETER;
        }
        fidx = TWIC_GATTSERVERWRITECHARACTERISTICS;
        status = twicIfGattServerWriteCharacteristics(&conn_iface, eidx, len, buffer);
    }

    result = BTLE_twicApiIsDone(fidx, status);
    command_flag &= ~BTLE_FLAG_UPDATE_VALUE;
    return result;
}

/** Determine the updates-enabled status (notification or indication) for a characteristic's CCCD.
 *
 *  @param eidx                 Handle for the value attribute of the characteristic.
 *
 *  @returns
 *    true.     Enable
 *    false.    Disable
 */
bool btle_gatts_characteristic_enable_update(uint8_t eidx)
{
    bool ret;
    uint64_t enable_update = 0;
    enable_update |= (1u << eidx);
    if(enable_update & notify_indicate_mask) {
        ret = true;
    } else {
        ret = false;
    }
    return ret;
}

/* APIs for BLE GAP */
/** Set the BTLE MAC address and type.
 *
 *  @param addr                 The BTLE_Address struct address to be set.
 *
 *  @note                       Only support RANDOM STATIC ADDRESS
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_set_address(BTLE_Address addr)
{
    twicStatus_t status;
    int api_done;
    twicBdaddr_t bdaddr;
    uint8_t fidx = 0;
    int i;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }
    
    device_addr.type = addr.type;
    memcpy(&device_addr.value, &addr.value, sizeof(uint64_t));

    for( i = 0; i < 6; ++i ) {
        bdaddr.address[i] = (device_addr.value >> (8*i)) & 0xff;
    }

    switch(device_addr.type) {
        case BTLE_PUBLIC_ADDRESS:
            /* TZ1K can set public address type before initialization */
            return BTLE_OK;
        case BTLE_RANDOM_STATIC_ADDRESS:
            fidx = TWIC_LESETRANDOMADDRESS;
            status = twicIfLeSetRandomAddress(&conn_iface, &bdaddr);
            break;
        case BTLE_RANDOM_PRIVATE_RESOLVABLE_ADDRESS:
            fidx = TWIC_LELMGENRESOLVABLEBDADDR;
            status = twicIfLeLmGenResolvableBdaddr(&conn_iface);
            break;
        case BTLE_RANDOM_PRIVATE_NON_RESOLVABLE_ADDRESS:
            return BTLE_ERROR;
        default:
            return BTLE_ERROR;
    }
    api_done = BTLE_twicApiIsDone(fidx, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Get the BTLE MAC address and type.
 *
 *  @param          NONE
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
BTLE_Address btle_gap_get_address(void)
{
    return device_addr;
}

/** Set the device name characteristic in the GAP service.
 *
 *  @param device_name          The new value for the device-name.
 *  @param length               Data to be written to gpio pin.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_set_device_name(const uint8_t *device_name, uint8_t *length)
{
    int ret;
    if((device_name == NULL) || (length == NULL)
            ||(*length > BTLE_DEVICE_NAME_MAX_LENGTH)) {
        return BTLE_ERROR_PARAMETER;
    }

    memset(btle_ga_device_name, 0, BTLE_DEVICE_NAME_MAX_LENGTH);
    memcpy(btle_ga_device_name, device_name, *length);
    ret =  btle_gatts_update_value(eidx_ga_device_name, btle_ga_device_name, *length, true);
    if(ret != BTLE_OK) {
        return BTLE_ERROR;
    }
    return BTLE_OK;
}

/** Get the value of the device name characteristic in the GAP service.
 *
 *  @param device_name          Pointer to an empty buffer to store current device name
 *  @param length               Length of the buffer pointed to by deviceName
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_get_device_name(uint8_t *device_name, uint8_t *length)
{
    if((device_name == NULL) || (length == NULL)) {
        return BTLE_ERROR_PARAMETER;
    }

    strcpy((char *)device_name, (const char *)btle_ga_device_name);
    *length = strlen((const char *) btle_ga_device_name);

    if(*length > BTLE_DEVICE_NAME_MAX_LENGTH) {
        return BTLE_ERROR;
    }
    return BTLE_OK;
}

/** Set the GAP peripheral preferred connection parameters.
 *
 *  @param params               Connection param will be set
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_set_perferred_conn_params(const BTLE_ConnectionParameter *params)
{
    int ret;

    if(params == NULL) {
        return BTLE_ERROR_PARAMETER;
    }
    if(BTLE_MIN_CONNECTION_INTERVAL > params->interval_min
            || BTLE_MAX_CONNECTION_INTERVAL < params->interval_min) {
        return BTLE_ERROR_PARAMETER;
    }
    if(BTLE_MIN_CONNECTION_INTERVAL > params->interval_max
            || BTLE_MAX_CONNECTION_INTERVAL < params->interval_max) {
        return BTLE_ERROR_PARAMETER;
    }

    if(BTLE_MAX_CONNECTION_SLAVE_LATENCY < params->slave_latency) {
        return BTLE_ERROR_PARAMETER;
    }

    if(BTLE_MIN_CONNECTION_SUPERVISION_TIMEOUT > params->supervison_timeout
            || BTLE_MAX_CONNECTION_SUPERVISION_TIMEOUT < params->supervison_timeout
            || params->interval_max > params->supervison_timeout) {
        return BTLE_ERROR_PARAMETER;
    }

    memcpy(&btle_ga_pref_conn[0], &params->interval_min, sizeof(uint16_t));
    memcpy(&btle_ga_pref_conn[2], &params->interval_max, sizeof(uint16_t));
    memcpy(&btle_ga_pref_conn[4], &params->slave_latency, sizeof(uint16_t));
    memcpy(&btle_ga_pref_conn[6], &params->supervison_timeout, sizeof(uint16_t));
    ret = btle_gatts_update_value(eidx_ga_pref_conn, btle_ga_pref_conn, BTLE_DEVICE_PREF_CONN_PARAM_VALUE_LENGTH, true);

    if(ret != BTLE_OK) {
        return BTLE_ERROR;
    }

    return BTLE_OK;
}

/** Get the GAP peripheral preferred connection parameters.
 *
 *  @param params               Pointer to store current connection parameters
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_get_perferred_conn_params(BTLE_ConnectionParameter *params)
{
    if(params == NULL) {
        return BTLE_ERROR_PARAMETER;
    }

    memcpy(&params->interval_min, &btle_ga_pref_conn[0], sizeof(uint16_t));
    memcpy(&params->interval_max, &btle_ga_pref_conn[2], sizeof(uint16_t));
    memcpy(&params->slave_latency, &btle_ga_pref_conn[4], sizeof(uint16_t));
    memcpy(&params->supervison_timeout, &btle_ga_pref_conn[6], sizeof(uint16_t));

    return BTLE_OK;
}

/** Set the appearance characteristic in the GAP service.
 *
 *  @param appearance           The new value for the device-appearance.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_set_device_appearance(const uint16_t appearance)
{
    int ret;
    TWIC_SETHARF_LE(btle_ga_appearance, appearance);

    ret = btle_gatts_update_value(eidx_ga_appearance, btle_ga_appearance, BTLE_DEVICE_APPEARANCE_VALUE_LENGTH, true);
    if(ret != BTLE_OK) {
        return BTLE_ERROR;
    }
    return BTLE_OK;
}

/** Get the appearance characteristic in the GAP service.
 *
 *  @param appearance           The current device-appearance value.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_get_device_appearance(uint16_t *appearance)
{
    uint16_t dev_appear;

    if(appearance == NULL) {
        return BTLE_ERROR_PARAMETER;
    }

    dev_appear = (uint16_t)((btle_ga_appearance[0]) | (btle_ga_appearance[1] << 8));
    *appearance = dev_appear;
    return BTLE_OK;
}

/** Set the GAP advertisingan and scan response data
 *
 *  @param adv_data_len          The advertising data length.
 *  @param adv_data              The advertising data to be set.
 *  @param scan_resp_data_len    The scan response data length.
 *  @param adv_data              The scan response data to be set.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
extern int btle_gap_set_adv_data(const uint8_t adv_data_len,
                                       const uint8_t *const adv_data,
                                       const uint8_t scan_resp_data_len,
                                       const uint8_t *const scan_resp_data)
{
#if defined(TWIC_API_LELESETADVERTISINGDATA) && defined(TWIC_BLE_HWIP_V41)
    twicStatus_t status;
    int api_done;
#endif
    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }
    
    advertising_data.advertising_data_len = adv_data_len;
    advertising_data.scan_resp_data_len = scan_resp_data_len;

    memcpy(advertising_data.advertising_data, adv_data, adv_data_len);
    memcpy(advertising_data.scan_resp_data, scan_resp_data, scan_resp_data_len);
#if defined(TWIC_API_LELESETADVERTISINGDATA) && defined(TWIC_BLE_HWIP_V41)
    status = twicIfLeSetAdvertisingData(&conn_iface,
                                        adv_data_len,
                                        advertising_data.advertising_data);

    api_done = BTLE_twicApiIsDone(TWIC_LESETADVERTISINGDATA, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }
#endif
    return BTLE_OK;
}

/** Start advertising.
 *
 *  @param prams                Advertising data will be apply
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_start_advertising(const BTLE_AdvertisingParameter *prams)
{
    twicStatus_t status;
    int api_done;
    bool own_is_random_addr;
    
    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    own_is_random_addr = (device_addr.type == BTLE_PUBLIC_ADDRESS) ? false : true;
    status = twicIfLeDiscoverable(&conn_iface,
                                  prams->min_interval,
                                  prams->max_interval,
                                  prams->advertising_type,
                                  own_is_random_addr,
                                  prams->direct_address_type,
                                  prams->direct_address,
                                  TWIC_ADV_CHANNEL_ALL,
                                  prams->advertising_filter_policy,
                                  advertising_data.advertising_data_len,
                                  advertising_data.advertising_data,
                                  advertising_data.scan_resp_data_len,
                                  advertising_data.scan_resp_data);

    api_done = BTLE_twicApiIsDone(TWIC_LEDISCOVERABLE, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Stop advertising.
 *
 *  @param          NONE
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_stop_advertising(void)
{
    twicStatus_t status;
    int api_done;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeStopAdvertising(&conn_iface);

    api_done = BTLE_twicApiIsDone(TWIC_LESTOPADVERTISING, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Create a connection
 *
 *  @param conn_addr                 Address struct of device will be connect.
 *  @param conn_params               Connection parameters will be applied after connected.
 *  @param scan_params               Scan parameters will be used.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_create_connection(BTLE_Address conn_addr,
                                  BTLE_ConnectionParameter *conn_params,
                                  BTLE_ScanParameter *scan_params)
{
    twicStatus_t status;
    int i, api_done;
    twicBdaddr_t bdaddr;
    bool connect_is_random_addr, own_is_random_addr;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    for( i = 0; i < 6; ++i ) {
        bdaddr.address[i] = (conn_addr.value >> (8*i)) & 0xff;
    }

    connect_is_random_addr = (conn_addr.type == BTLE_PUBLIC_ADDRESS) ? false : true;
    own_is_random_addr = (device_addr.type == BTLE_PUBLIC_ADDRESS) ? false : true;

    status = twicIfLeCreateConnection(&conn_iface, scan_params->interval,
                                      scan_params->window,
                                      scan_params->use_whitelist,
                                      connect_is_random_addr,
                                      &bdaddr,
                                      conn_params->interval_min,
                                      conn_params->interval_max,
                                      conn_params->slave_latency,
                                      conn_params->supervison_timeout,
                                      BTLE_MIN_CE_LENGTH,
                                      BTLE_MAX_CE_LENGTH,
                                      own_is_random_addr);

    api_done = BTLE_twicApiIsDone(TWIC_LECREATECONNECTION, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Terminate an existing GAP connection.
 *  @param conn_handle              Handle of the connection will be disconnected. 
 *  @param reason                   Reason disconnect. 
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_disconnect(uint16_t conn_handle, uint8_t reason)
{
    twicStatus_t status;
    int i, api_done;
    twicBdaddr_t bdaddr;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    if( conn_handle != connected_device_info.handle ) {
        return BTLE_ERROR_PARAMETER;
    }
    for( i = 0; i < 6; ++i ) {
        bdaddr.address[i] = (connected_device_info.peer_addr >> (8*i)) & 0xff;
    }
    status = twicIfLeDisconnect(&conn_iface, &bdaddr);

    api_done = BTLE_twicApiIsDone(TWIC_LEDISCONNECT, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Update GAP connection
 *
 *  @param conn_handle              Handle of the connection will be updated. 
 *  @param params                   The connection parameters will be updated.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_connection_update(uint16_t conn_handle,const BTLE_ConnectionParameter *params)
{
    twicStatus_t status;
    int api_done;

    if( conn_handle != connected_device_info.handle ) {
        return BTLE_ERROR_PARAMETER;
    }
    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }
    if(BTLE_MIN_CONNECTION_INTERVAL > params->interval_min
            || BTLE_MAX_CONNECTION_INTERVAL < params->interval_min) {
        return BTLE_ERROR_PARAMETER;
    }
    if(BTLE_MIN_CONNECTION_INTERVAL > params->interval_max
            || BTLE_MAX_CONNECTION_INTERVAL < params->interval_max) {
        return BTLE_ERROR_PARAMETER;
    }
    if(BTLE_MAX_CONNECTION_SLAVE_LATENCY < params->slave_latency) {
        return BTLE_ERROR_PARAMETER;
    }
    if(BTLE_MIN_CONNECTION_SUPERVISION_TIMEOUT > params->supervison_timeout
            || BTLE_MAX_CONNECTION_SUPERVISION_TIMEOUT < params->supervison_timeout
            || params->interval_max > params->supervison_timeout) {
        return BTLE_ERROR_PARAMETER;
    }

    status = twicIfLeConnectionUpdate(&conn_iface,
                                      params->interval_min,        /* min interval */
                                      params->interval_max,        /* max interval */
                                      params->slave_latency,                /* slave latency */
                                      params->supervison_timeout,            /* supervision timeout */
                                      BTLE_MIN_CE_LENGTH,                /* min ce length */
                                      BTLE_MAX_CE_LENGTH);                /* max ce length */

    api_done = BTLE_twicApiIsDone(TWIC_LECONNECTIONUPDATE, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Start scanning the BLE to set a connection.
 *
 *  @param params                 Scan parameters will be used.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_start_scan(BTLE_ScanParameter *scan_params)
{
    twicStatus_t status;
    int api_done;
    bool own_is_random_addr;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    own_is_random_addr = (device_addr.type == BTLE_PUBLIC_ADDRESS) ? false : true;
    status = twicIfLeSetScanEnable(&conn_iface,
                                   scan_params->interval,
                                   scan_params->window,
                                   scan_params->active_scanning,
                                   own_is_random_addr,
                                   scan_params->use_whitelist,
                                   scan_params->filter_duplicates);

    api_done = BTLE_twicApiIsDone(TWIC_LESETSCANENABLE, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Stop the currently scanning.
 *
 *  @param          NONE
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_stop_scan(void)
{
    twicStatus_t status;
    int api_done;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSetScanDisable(&conn_iface);

    api_done = BTLE_twicApiIsDone(TWIC_LESETSCANDISABLE, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Set tx power register.
 *
 *  @param txPower                  The value of TX power
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gap_set_tx_power(int8_t txPower)
{
    twicStatus_t status;
    int api_done;
    uint8_t dbm_idx;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }
    /* 0x00: Reserved,   0x01: 0dBm,   0x02: -4dBm,   0x03: -8dBm,
         0x04: -12dBm,   0x05: -16dBm,   0x06: -20dBm */
    switch(txPower) {
        case 0:
            dbm_idx = 0x01;
            break;
        case -4:
            dbm_idx = 0x02;
            break;
        case -8:
            dbm_idx = 0x03;
            break;
        case -12:
            dbm_idx = 0x04;
            break;
        case -16:
            dbm_idx = 0x05;
            break;
        case -20:
            dbm_idx = 0x06;
            break;
        default:
            return BTLE_ERROR_PARAMETER;
    }

    status = twicIfLeCeSetTxPower(&conn_iface, dbm_idx);
    api_done = BTLE_twicApiIsDone(TWIC_LECESETTXPOWER, status);
    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}
/* APIs for BLE GATT Client */
/**  Discover all the primary services on a server.
 *
 *  @param startHandle              The Starting Handle shall be set to 0x0001.
 *  @param endHandle                The Ending Handle shall be set to 0xFFFF.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gattc_discover_primary_service(uint16_t startHandle,
        uint16_t endHandle)
{
    twicStatus_t status;
    int api_done;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeGattClientDiscoverPrimaryService(&conn_iface,
             startHandle,
             endHandle);
    api_done = BTLE_twicApiIsDone(TWIC_GATTCLIENTDISCOVERPRIMARYSERVICE, status);

    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Discover a specific primary service on a server when only the Service UUID is known.
 *
 *  @param startHandle              The Starting Handle.
 *  @param endHandle                The Ending Handle.
 *  @param service_uuid             The UUID structure of service.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gattc_discover_primary_service_by_uuid(uint16_t startHandle,
        uint16_t endHandle,
        BTLE_UUID service_uuid)
{
    twicStatus_t status;
    int api_done;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeGattClientDiscoverPrimaryServiceByServiceUuid(&conn_iface,
             startHandle,                          /* start handle. search all handles */
             endHandle,                            /* end handle. search all handles */
             service_uuid.uuid_lsb,
             service_uuid.uuid_msb,
             service_uuid.uuid_type);
    api_done = BTLE_twicApiIsDone(TWIC_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID, status);

    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Find all the characteristic declarations within a service definition on a server 
 *  when only the service handle range is known.
 *
 *  @param startHandle              The Starting Handle
 *  @param endHandle                The Ending Handle
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gattc_discover_all_characteristics(uint16_t startHandle,
        uint16_t endHandle)
{
    twicStatus_t status;
    int api_done;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeGattClientDiscoverAllCharacteristics(&conn_iface, startHandle, endHandle);
    api_done = BTLE_twicApiIsDone(TWIC_GATTCLIENTDISCOVERALLCHARACTERISTICS, status);

    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Find all the characteristic declarations within a service definition on a server 
 *  when only the UUID service is known.
 *
 *  @param startHandle              The Starting Handle.
 *  @param endHandle                The Ending Handle.
 *  @param uuid_lsb                 The part of the Least Significant Octet in the UUID.
 *  @param uuid_msb                 The part of the Most Significant Octet in the UUID.
 *  @param uuid_len                 The bytes of UUID.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gattc_discover_characteristics_by_uuid(uint16_t startHandle,
        uint16_t endHandle,
        uint64_t uuid_lsb,
        uint64_t uuid_msb,
        uint8_t uuid_len)
{
    twicStatus_t status;
    int api_done;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeGattClientDiscoverCharacteristicsByUuid(&conn_iface,
             startHandle,                          /* start handle. search all handles */
             endHandle,                            /* end handle. search all handles */
             uuid_lsb,
             uuid_msb,
             uuid_len);
    api_done = BTLE_twicApiIsDone(TWIC_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID, status);

    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Write a Characteristic Value to a server when the client knows the Characteristic Value Handle
 *
 *  @param handle                   The Attribute Handle parameter shall be set to the this parameter.
 *  @param length                   The length of the new characteristic.
 *  @param value                    The Attribute Value parameter shall be set to the new characteristic.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gattc_write_characteristic(uint16_t handle,
                                       uint8_t length,
                                       const uint8_t *value)
{
    twicStatus_t status;
    int api_done;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeGattClientWriteCharacteristicValue(&conn_iface, handle, length, value);
    api_done = BTLE_twicApiIsDone(TWIC_GATTCLIENTWRITECHARACTERISTICVALUE, status);

    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Write a Characteristic Value to a server without response when the client knows the Characteristic Value Handle
 *
 *  @param handle                   The Attribute Handle parameter shall be set to the this parameter.
 *  @param length                   The length of the new characteristic.
 *  @param value                    The Attribute Value parameter shall be set to the new characteristic.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gattc_write_characteristic_without_response(uint16_t handle,
                                       uint8_t length,
                                       const uint8_t *value)
{
    twicStatus_t status;
    int api_done;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeGattClientWriteWithoutResponse(&conn_iface, handle, length, value);
    api_done = BTLE_twicApiIsDone(TWIC_GATTCLIENTWRITEWITHOUTRESPONSE, status);

    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/** Write a Characteristic Descriptor Value to a server when the client knows the Characteristic Descriptor Handle.
 *
 *  @param handle                   The Attribute Handle parameter shall be set to this parameter.
 *  @param length                   The length of the new characteristic descriptor value.
 *  @param value                    This parameter shall be set to the new characteristic descriptor value.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gattc_write_characteristic_descriptor(uint16_t handle,
        uint8_t length,
        const uint8_t *value)
{
    twicStatus_t status;
    int ret;
    int api_done;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    for(unsigned i=0; i<BTLE_MAX_RETRY; i++) {
        status = twicIfLeGattClientWriteCharacteristicDescriptor(&conn_iface, handle, length, value);
        api_done = BTLE_twicApiIsDoneWithRetry(TWIC_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR, status);
        if(api_done == BTLE_RETRY) {
            wait(0.01); /* delay 10ms */
        } else {
            break;
        }
    }
    if(api_done == 0) {
        ret = BTLE_OK;
    } else {
        ret = TWIC_STATUS_ERROR_IO;
    }

    return ret;
}

/** Read a Characteristic Value from a server when the client knows the Characteristic Value Handle.
 *
 *  @param handle                   Characteristic Value Handle of the Characteristic Value to be read.
 *  @param offset                   The Offset parameter shall be the offset within the Characteristic Value to be read.
 *
 *  @returns
 *    BTLE_OK.      Success
 *    other.        Error
 */
int btle_gattc_read_characteristic(uint16_t handle, uint16_t offset)
{
    twicStatus_t status;
    int api_done;
    uint8_t fidx;

    if(BTLE_internIsBusy()) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    if(offset == 0) {
        fidx = TWIC_GATTCLIENTREADCHARACTERISTICVALUE;
        status = twicIfLeGattClientReadCharacteristicValue(&conn_iface, handle);
    } else {
        fidx = TWIC_GATTCLIENTREADLONGCHARACTERISTICVALUE;
        status = twicIfLeGattClientReadLongCharacteristicValue(&conn_iface, handle, offset);
    }
    api_done = BTLE_twicApiIsDone(fidx, status);

    if(api_done != BTLE_OK) {
        return api_done;
    }

    return BTLE_OK;
}

/* APIs for scheduling and process BLE events */
/** Execute the BLE events.
 *
 *  @param          NONE
 *
 *  @returns
 *    NONE.
 */
void btle_internal_events_execute(void)
{
    btle_handle_events_pending = 0;
    twicStatus_t twic_status ;
    bool twic_no_event = false;
    bool btle_no_event = false;
    uint8_t has_btle_event;

    for( ; ; ) {

        if(!twic_no_event) {
            twic_status = twicIfPeekEvent();
            if(twic_status == TWIC_STATUS_OK) {
                twic_no_event = true;
            } else {
                twicIfDoEvents();
            }
        }

        if(!btle_no_event) {
            has_btle_event = BTLE_bufferEventIsAvaiable();
            if(has_btle_event == 0) {
                btle_no_event = true;
            } else {
                BTLE_dequeueEventData(&btle_event);
                BTLE_doEvent(&btle_event);
            }
        }

        if(twic_no_event && btle_no_event) {
            break;
        }
    }
}

/** Add the event to scheduler to process later.
 *
 *  @param          NONE
 *
 *  @returns
 *    NONE.
 */
void btle_scheduler_events_execute(void)
{
    if(!btle_handle_events_pending) {
        btle_handle_events_pending = 1;
        tz1BLE::Instance(BLE::DEFAULT_INSTANCE).signalEventsToProcess(BLE::DEFAULT_INSTANCE);
    }
}

/** Handle BLE intern error 
 *
 *  @param error_code           Code of the error
 *  @param line_num             Line number of the code error
 *  @param p_file_name          File contain the error code
 *
 *  @returns
 *    NONE.
 */
#ifdef DEBUG
void btle_error_handler(int error_code, uint32_t line_num, const uint8_t *p_file_name)
{
    LOGD("Error code %d line number %d file name %s\r\n", error_code, line_num, p_file_name);
}
#else
void btle_error_handler(int error_code, uint32_t line_num, const uint8_t *p_file_name)
{
    (void)error_code;
    (void)line_num;
    (void)p_file_name;
    error("BTLE error code %d", error_code);
    NVIC_SystemReset();
}
#endif
/**********************************************************************************
************* Implement Utils BLE functions ***************************************
***********************************************************************************
*/
/* Function check status and do event for BTLE_twicDbApiIsDone ,BTLE_twicApiIsDone and BTLE_dbInit */
/** Check the TWiC middleware status. If it has event to process, the event will be executed
 *
 *  @param status           Input of TWiC API status
 *
 *  @returns
 *    true.     The TWiC API is done for executing
 *    false.    The TWiC API execution error
 */
static bool BTLE_twicCheckAndDoEvent(twicStatus_t status)
{
    if (TWIC_STATUS_OK != status && TWIC_STATUS_UNDER_PROCESSING != status &&
            TWIC_STATUS_WAITING_FOR_ACTIVATION != status) {
        return false;
    }

    if (TWIC_STATUS_EVENT_MESSAGE == twicIfPeekEvent()) {
        twicIfDoEvents();
    }

    return true;
}

/* Function use for BTLE_twicApiIsDone and BTLE_dbInit*/
/** Check and assemble the incoming data from the internal BLE
 *
 *  @param fidx           TWiC API id
 *  @param aret           Result of assemble of TWiC APIs
 *
 *  @returns
 *    true.     Assemble success
 *    false.    Assemble error
 */
static bool BTLE_twicPeekInApi(uint8_t fidx, uint8_t *aret)
{
    uint8_t _fidx;
    twicStatus_t status;

    *aret = 0;
    twicIfLeCeHkTimeout(BTLE_TIMEOUT_COUNT, fidx);
    status = twicIfIsDone(&conn_iface, &_fidx);
    if ((TWIC_STATUS_EVENT_MESSAGE == status) && (_fidx == fidx)) {
        if (TWIC_STATUS_OK == twicIfAcceptance(&conn_iface, aret, NULL)) {
            return true;
        }
    }
    return false;
}

/** Register GATT service and TWiC funtion callback.
 *
 *  @param server_callbacks                 TWiC server callback
 *  @param client_callbacks                 TWiC client callback
 *  @param smp_i_callbacks                  TWiC smp initiator callback
 *  @param smp_r_callbacks                  TWiC smp responder callback
 *  @param le_callbacks                     TWiC common callback
 *  @returns
 *    BTLE_OK.      Success
 */
static int BTLE_registerService( const twicIfLeServerCb_t *server_callbacks,
        const twicIfLeClientCb_t *client_callbacks,
        const twicIfLeSmpICb_t *smp_i_callbacks,
        const twicIfLeSmpRCb_t *smp_r_callbacks,
        const twicIfLeCb_t *le_callbacks)
{
    twicStatus_t status;
    /* Initialize as Server */
    status = twicIfGattInitialize();
    if(status != TWIC_STATUS_OK) {
        return BTLE_ERROR;
    }

    status = twicIfGattRegistration(&interface_id);
    if(status != BTLE_OK) {
        return BTLE_ERROR;
    }

    /* Clean up and initialize all the entries and these list links of the interface */
    status = twicIfGattCleanup(interface_id, &conn_iface);
    if(status != TWIC_STATUS_OK) {
        return BTLE_ERROR;
    }

    status = twicIfLeRegisterCallback(&conn_iface, server_callbacks, client_callbacks,
                                      smp_i_callbacks, smp_r_callbacks,
                                      le_callbacks);
    if(status != TWIC_STATUS_OK) {
        return BTLE_ERROR;
    }
    return BTLE_OK;
}

/** TWiC DB initialize
 *
 *  @param baudrate                     The device UART 2 baudrate
 *  @param peripheral_bdaddr            The device peripheral public address
 *
 *  @returns
 *    BTLE_OK.      Success
 */
static int BTLE_dbInit(uint64_t baudrate,
                                const uint64_t *peripheral_bdaddr)
{
    volatile int j, db_init_sequence;
    int api_done;
    twicStatus_t status;
    uint8_t _ar;

    uint8_t patch_base[] = {
        0x10, 0x25, 0x03, 0xd0, 0x6d, 0x0b, 0x00, 0x70, 0xb5, 0x05, 0x1c, 0x9b,
        0xf7, 0x8c, 0xfb, 0x04, 0x1c, 0x11, 0xd0, 0x60, 0x78, 0x01, 0x21, 0x09,
        0x04, 0x41, 0x18, 0x08, 0x20, 0xd7, 0xf7, 0xba, 0xf9, 0x20, 0x89, 0x00,
        0x28, 0x04, 0xd0
    };
    uint8_t patch_write[] = {
        0x10, 0x2C, 0xf0, 0x24, 0x05, 0x00, 0x70, 0xb5, 0x05, 0x1c, 0x64, 0xf0,
        0xb8, 0xfc, 0x04, 0x1c, 0x07, 0xd0, 0xe1, 0x68, 0x14, 0x23, 0xc8, 0x5e,
        0x4c, 0xf0, 0xf1, 0xfc, 0x20, 0x1c, 0x64, 0xf0, 0x98, 0xfc, 0x28, 0x1c,
        0x64, 0xf0, 0xfe, 0xfc, 0x70, 0xbc, 0x08, 0xbc, 0x18, 0x47
    };
#if defined(TWIC_BLE_HWIP_V41)
    /* Patch is already applied */
    db_init_sequence = TWIC_LECESETBAUDRATE;
#else
    /* API needs to be in the loop to wait NOP. */
    for ( ; BTLE_twicPeekInApi(TWIC_LECEPATCHBASE, &_ar) != true; ) {
        status = twicIfLeCePatchBase(&conn_iface, sizeof(patch_base), patch_base);
        if(BTLE_twicCheckAndDoEvent(status) != true) {
            return status;
        }
    }
    if (_ar) {
        return BTLE_ERROR;
    }

    /* TWiC DB initialize */
    db_init_sequence = TWIC_LECEPATCHWRITE;
#endif

    for(j = 0; j < 15; j++) {
        switch( db_init_sequence ) {
            case TWIC_LECEPATCHWRITE:
                status = twicIfLeCePatchWrite(&conn_iface, sizeof(patch_write), patch_write);
                break;
            case TWIC_LECEPATCHCONTROL:
                status = twicIfLeCePatchControl(&conn_iface, 3, true);
                break;
            case TWIC_LECESETBAUDRATE:
                status = twicIfLeCeSetBaudrate(&conn_iface, (twicTzbtBr_t)baudrate, BTLE_UART2_PUNCTUATION);
                break;
            case TWIC_LEREADBDADDR:
                status = twicIfLeReadBdaddr(&conn_iface);
                break;
            case TWIC_LECEFLOWCONTROL:
                status = twicIfLeCeFlowControl(&conn_iface, true);
                break;
            case TWIC_LECEHOSTDELAY:
                status = twicIfLeCeHostDelay(&conn_iface, RPQUEUE_PM_HDD);
                break;
            case TWIC_LEWRITEBDADDR:
                status = twicIfLeWriteBdaddr(&conn_iface, peripheral_bdaddr);
                break;
            case TWIC_LECELOWPOWERCLOCKSETUP:
                status = twicIfLeCeLowPowerClockSetup(&conn_iface, NULL);
                break;
            case TWIC_LECELOWPOWERPRIMARYSETUP:
                status = twicIfLeCeLowPowerPrimarySetup(&conn_iface, 0x00FA, 0xA);
                break;
            case TWIC_LECELOWPOWERCONTROLPINSETUP:
                status = twicIfLeCeLowPowerControlPinSetup(&conn_iface, true);
                break;
            case TWIC_LECELOWPOWERDISCOVERABLESETUP:
                status = twicIfLeCeLowPowerDiscoverableSetup(&conn_iface);
                break;
            case TWIC_LECECHANGETOCM:
                status = twicIfLeCeChangeToCm(&conn_iface);
                break;
            case TWIC_LEINITIALIZEDEVICE:
                status = twicIfLeInitializeDevice(&conn_iface);
                break;
            case TWIC_LEGATTSERVERSTART:
                status = twicIfLeGattServerStart(&conn_iface);
                break;
            case TWIC_LEGATTCLIENTSTART:
                status = twicIfLeGattClientStart(&conn_iface);
                j = 15;
                break;
            default:
                return BTLE_ERROR_PARAMETER;
        }
        api_done = BTLE_twicApiIsDone(db_init_sequence, status);

        if(api_done == BTLE_OK) {
            if( TWIC_LEGATTCLIENTSTART == db_init_sequence ) {
                /* delay lowpower setting apply */
                BTLE_configureTz1emOnInitialize();
                BTLE_configureTz1emOnLowPowerOff();
            }
            /* Wait after SetBaudrate or ChangeToCm is required */
            if( TWIC_LECESETBAUDRATE == db_init_sequence ) {
                wait(0.1);
            } else if( TWIC_LECECHANGETOCM == db_init_sequence) {
                wait(0.1);
            }
            /* specify next API */
            if( TWIC_LECEPATCHCONTROL == db_init_sequence ) {
                /* skip DBUS write and memory write */
                db_init_sequence = TWIC_LECESETBAUDRATE;
            } else {
                db_init_sequence += 1;
            }
        } else {
            return api_done;
        }
    }

    return api_done;
}

/** Register Generic Access service (0x1800)
 *
 *  @param          NONE
 *
 *  @returns
 *    BTLE_OK.      Success
 */
static int BTLE_gaServiceInit()
{
    int ret;
    uint8_t eidx_ga_service;
    BTLE_UUID ga_service_uuid = {BTLE_GAP_SERVICE_UUID, 0, BTLE_UUID_16};
    BTLE_UUID chara_device_name = {BTLE_GAP_CHAR_DEVICE_NAME_UUID, 0, BTLE_UUID_16};
    BTLE_UUID chara_apperance = {BTLE_GAP_CHAR_APPEARANCE_UUID, 0, BTLE_UUID_16};
    BTLE_UUID pref_conn = {BTLE_GAP_CHAR_PERIPHERAL_PREF_CONN_PARAM_UUID, 0, BTLE_UUID_16};

    ret = btle_gatts_begin_service_creation(&eidx_ga_service, ga_service_uuid);
    if(ret != BTLE_OK) {
        return ret;
    }
    /* Add device name */
    ret = btle_gatts_add_characteristics(SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK,
                                   eidx_ga_service, &eidx_ga_device_name, chara_device_name, false, false,
                                   BTLE_PROPERTY_READ | BTLE_PROPERTY_WRITE, btle_ga_device_name,
                                   sizeof(btle_ga_device_name), BTLE_DEVICE_NAME_MAX_LENGTH, true);
    if(ret != BTLE_OK) {
        return ret;
    }
    /* Add appearance */
    ret = btle_gatts_add_characteristics(SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK,
                                   eidx_ga_service, &eidx_ga_appearance, chara_apperance, false, false, BTLE_PROPERTY_READ,
                                   btle_ga_appearance, sizeof(btle_ga_appearance), 0, false);
    if(ret != BTLE_OK) {
        return ret;
    }
    /* Add preffered connection params */
    ret = btle_gatts_add_characteristics(SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK,
                                   eidx_ga_service, &eidx_ga_pref_conn, pref_conn, false, false, BTLE_PROPERTY_READ,
                                   btle_ga_pref_conn, sizeof(btle_ga_pref_conn) / sizeof(uint8_t), 0, false);
    if(ret != BTLE_OK) {
        return ret;
    }

    ret = btle_gatts_end_service_creation(eidx_ga_service, ga_service_uuid);
    if(ret != BTLE_OK) {
        return ret;
    }

    return BTLE_OK;
}

/** Generic Attribute service (0x1801)
 *
 *  @param          NONE
 *
 *  @returns
 *    BTLE_OK.      Success
 */
static int BTLE_gattServiceInit()
{
    int ret;
    uint8_t serviceHandle;
    BTLE_UUID gatt_service_uuid = {BTLE_GATT_SERVICE_UUID, 0, BTLE_UUID_16};
    BTLE_UUID chara_sev_change = {BTLE_GATT_CHAR_SERVICE_CHANGE_UUID, 0, BTLE_UUID_16};
    /* Create service 0x1801 */
    ret = btle_gatts_begin_service_creation(&serviceHandle, gatt_service_uuid);
    if(ret != BTLE_OK) {
        return ret;
    }
    /* Add characteristic 0x2A05 */
    ret = btle_gatts_add_characteristics(SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK,
                                   serviceHandle, NULL, chara_sev_change, false, false, BTLE_PROPERTY_INDICATE, btle_gatt_service_changed, sizeof(btle_gatt_service_changed), 0, false);
    if(ret != BTLE_OK) {
        return ret;
    }

    ret = btle_gatts_end_service_creation(serviceHandle, gatt_service_uuid);
    if(ret != BTLE_OK) {
        return ret;
    }

    return BTLE_OK;
}

/* Function use for BTLE_twicDbApiIsDone */
/** Check and obtain the result code of each ‘twicIf*GattDb*’ API
 *
 *  @param fidx     TWiC API id
 *  @param eidx     The Entry index
 *  @param aret     The result data
 *
 *  @returns
 *    true.     Success
 *    false.    Error
 */
static bool BTLE_twicDbPeekInApi(uint8_t fidx, uint8_t eidx, uint8_t *aret)
{
    uint8_t _fidx, _eidx;
    twicStatus_t status;

    *aret = 0;
    twicIfLeCeHkTimeout(BTLE_TIMEOUT_COUNT, fidx);
    status = twicIfDbIsDone(&conn_iface, &_fidx, &_eidx, 0, 0);
    if ((TWIC_STATUS_EVENT_MESSAGE == status) && (_fidx == fidx) && (_eidx == eidx)) {
        if (TWIC_STATUS_OK == twicIfDbAcceptance(&conn_iface, eidx, aret)) {
            return true;
        }
    }
    return false;
}

/** Check and do ‘twicIf*GattDb*’ APIs. Return result is completed or not
 *
 *  @param fidx     TWiC API id
 *  @param eidx     The Entry index
 *  @param status   Result code of ‘twicIf*GattDb*’
 *
 *  @returns
 *    BTLE_OK.     Completed
 *    other.       Error
 */
static int BTLE_twicDbApiIsDone(const uint8_t fidx,
        const uint8_t eidx, twicStatus_t status)
{
    uint8_t _ar;

    for ( ; BTLE_twicDbPeekInApi(fidx, eidx, &_ar)!= true; ) {
        if (false == BTLE_twicCheckAndDoEvent(status)) {
            return BTLE_ERROR;
        }
    }
    if (_ar) {
        return BTLE_ERROR;
    }
    return BTLE_OK;
}

/** Check and do APIs except ‘twicIf*GattDb*’ APIs. Return result is completed or not or need to retry
 *
 *  @param fidx     TWiC API id
 *  @param status   Result code of TWiC
 *
 *  @returns
 *    BTLE_OK.          Completed
 *    BTLE_RETRY.       Need to retry execute
 *    other.            Error
 */
static int BTLE_twicApiIsDoneWithRetry(const uint8_t fidx, twicStatus_t status)
{
    uint8_t _ar;

    for ( ; BTLE_twicPeekInApi(fidx, &_ar) != true; ) {
        if (false == BTLE_twicCheckAndDoEvent(status)) {
            return BTLE_ERROR;
        }
    }
    /* GATT Request in progress or LE MNG Command in Progress */
    if (_ar == 0x10 || _ar == 0x04) {
        return BTLE_RETRY;
    }

    return _ar;
}

/** Check and do APIs except ‘twicIf*GattDb*’ APIs. Return result is completed or not
 *
 *  @param fidx     TWiC API id
 *  @param status   Result code of TWiC
 *
 *  @returns
 *    BTLE_OK.          Completed
 *    other.            Error
 */
static int BTLE_twicApiIsDone(const uint8_t fidx, twicStatus_t status)
{
    uint8_t _ar;

    for ( ; BTLE_twicPeekInApi(fidx, &_ar) != true; ) {
        if (false == BTLE_twicCheckAndDoEvent(status)) {
            return BTLE_ERROR;
        }
    }
    if (_ar) {
        return BTLE_ERROR;
    }

    return BTLE_OK;
}

/** TZ1EM configuration for initialize
 *
 *  @param              NONE
 *
 *  @returns
 *      NONE
 */
static void BTLE_configureTz1emOnInitialize(void)
{
    twicIfLeCeNcAdvPermitTz1em();
    twicIfLeCeCnAdvPermitTz1em();
    twicIfLeCeIdlePermitTz1em();
    twicIfLeCeConnPermitTz1em();
}

/** TZ1EM configuration for finalize
 *
 *  @param              NONE
 *
 *  @returns
 *      NONE
 */
static void BTLE_configureTz1emOnFinalize(void)
{
    twicIfLeCeNcAdvWithdrawalFromTz1em();
    twicIfLeCeCnAdvWithdrawalFromTz1em();
    twicIfLeCeIdleWithdrawalFromTz1em();
    twicIfLeCeConnWithdrawalFromTz1em();
}

/* Function callback use for configure wakeup callback of tz1em */
/** Function callback for wake up device use by TZ1EM
 *
 *  @param pcd      Specifies the Power Control Domain Group (TZ1EM_PCDG_*)
 *  @param op       Specifies the one of Operation Mode(TZ1EM_OP_*)
 *  @param vf       Specifies the one of VF(TZ1EM_VF_*)
 *  @param factor   Specifies the interrupt factor for which the CPU becomes active from Sleep state
 *
 *  @returns
 *      NONE
 */
static void BTLE_configureTz1emWakeupCallback(tz1smHalPcd_t pcd,
        tz1smHalOm_t op,
        tz1smHalVf_t vf,
        uint32_t factor)
{
    btle_scheduler_events_execute();
}

/** TZ1EM configuration for low power off
 *
 *  @param              NONE
 *
 *  @returns
 *      NONE
 */
static void BTLE_configureTz1emOnLowPowerOff(void)
{
    tz1emVf_t vf;

    vf = BTLE_UART_BAUDRATE < TWIC_TZBT_2304 ? TZ1EM_VF_UM : TZ1EM_VF_LM;

    if (TWIC_STATUS_OK != twicIfLeCeNcAdvConfigureTz1em(
                vf,
                TZ1EM_OP_BLE_UN,
                TZ1EM_WE_EN, BTLE_TZ1EM_WF_GPIO, TZ1EM_WE_OFF, TZ1EM_WF_UN,
                BTLE_configureTz1emWakeupCallback)) {
        BTLE_ERROR_HANDLER(BTLE_ERROR_LOWPOWER);
    }
    if (TWIC_STATUS_OK != twicIfLeCeCnAdvConfigureTz1em(
                vf,
                TZ1EM_OP_BLE_DOZE_RET,
                TZ1EM_WE_EN, BTLE_TZ1EM_WF_GPIO, TZ1EM_WE_OFF, TZ1EM_WF_UN,
                BTLE_configureTz1emWakeupCallback)) {
        BTLE_ERROR_HANDLER(BTLE_ERROR_LOWPOWER);
    }
    if (TWIC_STATUS_OK != twicIfLeCeIdleConfigureTz1em(
                vf,
                TZ1EM_OP_BLE_UN,
                TZ1EM_WE_EN, BTLE_TZ1EM_WF_GPIO, TZ1EM_WE_OFF, TZ1EM_WF_UN,
                BTLE_configureTz1emWakeupCallback)) {
        BTLE_ERROR_HANDLER(BTLE_ERROR_LOWPOWER);
    }

    if(BTLE_POWER_MODE == BTLE_POWER_HIGH_SPEED) {
        vf = TZ1EM_VF_HI;
    }

    if (TWIC_STATUS_OK != twicIfLeCeConnConfigureTz1em(
                vf,
                TZ1EM_OP_BLE_ACTIVE,
                TZ1EM_WE_EN, BTLE_TZ1EM_WF_GPIO, TZ1EM_WE_OFF, TZ1EM_WF_UN,
                BTLE_configureTz1emWakeupCallback)) {
        BTLE_ERROR_HANDLER(BTLE_ERROR_LOWPOWER);
    }
}

/** TZ1EM configuration for low power on
 *
 *  @param              NONE
 *
 *  @returns
 *      NONE
 */
static void BTLE_configureTz1emOnLowPowerOn(void)
{
    tz1emVf_t vf;

    vf = (BTLE_UART_BAUDRATE < TWIC_TZBT_2304) ? TZ1EM_VF_UM : TZ1EM_VF_LM;

    if (TWIC_STATUS_OK != twicIfLeCeNcAdvConfigureTz1em(
                vf,
                TZ1EM_OP_BLE_UN,
                TZ1EM_WE_EN, BTLE_TZ1EM_WF_GPIO, TZ1EM_WE_OFF, TZ1EM_WF_UN, NULL)) {
        BTLE_ERROR_HANDLER(BTLE_ERROR_LOWPOWER);
    }
    if (TWIC_STATUS_OK != twicIfLeCeCnAdvConfigureTz1em(
                vf,
                TZ1EM_OP_BLE_DOZE_RET,
                TZ1EM_WE_EN, BTLE_TZ1EM_WF_GPIO, TZ1EM_WE_OFF, TZ1EM_WF_UN, NULL)) {
        BTLE_ERROR_HANDLER(BTLE_ERROR_LOWPOWER);
    }
    if (TWIC_STATUS_OK != twicIfLeCeIdleConfigureTz1em(
                vf,
                TZ1EM_OP_BLE_UN,
                TZ1EM_WE_EN, BTLE_TZ1EM_WF_GPIO, TZ1EM_WE_OFF, TZ1EM_WF_UN, NULL)) {
        BTLE_ERROR_HANDLER(BTLE_ERROR_LOWPOWER);
    }
    if (TWIC_STATUS_OK != twicIfLeCeConnConfigureTz1em(
                (vf == TZ1EM_VF_UN) ? TZ1EM_VF_LO : vf,
                TZ1EM_OP_BLE_DOZE_RET,
                TZ1EM_WE_EN, TZ1EM_WF_RTC, TZ1EM_WE_OFF, TZ1EM_WF_UN, NULL)) {
        BTLE_ERROR_HANDLER(BTLE_ERROR_LOWPOWER);
    }
}

/** Initialize data for online. This function is called when disconnected.
 *
 *  @param              NONE
 *
 *  @returns
 *      NONE
 */
static void BTLE_initializeOnDisconnect(void)
{
    command_flag = 0;
    notify_indicate_mask = 0;
    security_link_status = SecurityManager::NOT_ENCRYPTED;
}

/** Parse response write authorization. The function will be invoke mbed charateristic write
 * authorization callback
 *
 *  @param  conn_handle    Connection handle
 *  @param  p_char         Pointer to charateristic will be response 
 *  @param  offset         Offset of the characteristic's value
 *  @param  len            Characteristic's value length
 *  @param  value          Characteristic's value
 *
 *  @returns
 *      uint8_t            Response code of user. It is result of mbed charateristic authorization callback
 */
static uint8_t BTLE_getServerRespWriteAuthorizationDemand(uint16_t conn_handle,
        GattCharacteristic *p_char,
        uint16_t offset,
        uint16_t len,
        uint8_t *value)
{
    uint8_t authorizeStatus;

    GattWriteAuthCallbackParams cbAuParams = {
        .connHandle = conn_handle,
        .handle     = p_char->getValueAttribute().getHandle(),
        .offset     = offset,
        .len        = len,
        .data       = value,
        .authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS /* the callback handler must leave this member
                                                           * set to AUTH_CALLBACK_REPLY_SUCCESS if the client
                                                           * request is to proceed. */
    };

    authorizeStatus = p_char->authorizeWrite(&cbAuParams);

    if (cbAuParams.authorizationReply == AUTH_CALLBACK_REPLY_SUCCESS) {
        GattWriteCallbackParams cbParams = {
            .connHandle = conn_handle,
            .handle     = p_char->getValueAttribute().getHandle(),
            .writeOp    = GattWriteCallbackParams::OP_WRITE_REQ,
            .offset     = offset,
            .len        = len,
            .data       = value
        };

        tz1GattServer::getInstance().tz1kDataWrittenEvent(&cbParams);
    }

    return authorizeStatus;
}

/** Parse response read authorization. The function will be invoke mbed charateristic read
 * authorization callback
 *
 *  @param  conn_handle    Connection handle
 *  @param  p_char         Pointer to charateristic will be response 
 *  @param  offset         Offset of the characteristic's value
 *  @param  len            Characteristic's value length
 *  @param  value          Characteristic's value
 *
 *  @returns
 *      uint8_t            Response code of user. It is result of mbed charateristic authorization callback
 */
static uint8_t BTLE_getServerRespReadAuthorizationDemand(uint16_t conn_handle,
        GattCharacteristic *p_char)
{
    uint8_t authorizeStatus;

    GattReadAuthCallbackParams cbParams = {
        .connHandle         = conn_handle,
        .handle             = p_char->getValueAttribute().getHandle(),
        .offset             = 0,
        .len                = 0,
        .data               = NULL,
        .authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS /* the callback handler must leave this member
                                                           * set to AUTH_CALLBACK_REPLY_SUCCESS if the client
                                                           * request is to proceed. */
    };

    authorizeStatus = p_char->authorizeRead(&cbParams);
    if (cbParams.authorizationReply == BTLE_CALLBACK_STATUS_OK) {
        btle_gatts_update_value(p_char->getValueAttribute().getHandle(), cbParams.data, cbParams.len, true);
    }

    return authorizeStatus;
}

/** Get SMP response code from BTLE callback status
 *
 *  @param  status    Status code
 *
 *  @returns
 *      twicSmReasonCode_t       SMP response code
 */
static twicSmReasonCode_t BTLE_getSmReasonCode(uint8_t status)
{
    twicSmReasonCode_t ret;

    switch(status) {
        case 0x00:
            ret = TWIC_SMP_SUCCESS;
            break;
        case 0x01:
            ret = TWIC_PASSKEY_ENTRY_FAILED;
            break;
        case 0x02:
            ret = TWIC_OOB_NOT_AVAILABLE;
            break;
        case 0x05:
            ret = TWIC_PAIRING_NOT_SUPPORTED;
            break;
        case 0x06:
            ret = TWIC_ENCRYPTION_KEY_SIZE;
            break;
        case 0xC0:
            ret = TWIC_SMP_TIME_OUT;
            break;
        default:
            ret = TWIC_UNSPECIFIED_REASON;
            break;
    }

    return ret;
}

/** Gets mbed BLE Security mode from twic authorization information
 *
 *  @param  bits    TWiC authorization information
 *  @param  sm      Pointer to store mbed Security mode result
 *
 *  @returns
 *      NONE
 */
static void BTLE_getSecurityModeFromAuthorizationParams(twicAuthInfo_t *bits, SecurityManager::SecurityMode_t *sm)
{
    SecurityManager::SecurityMode_t securityMode;

    if(bits->bonding_enabled == 1u && bits->mitm_enabled == 0) {
        securityMode = SecurityManager::SECURITY_MODE_ENCRYPTION_NO_MITM;
    } else if(bits->bonding_enabled == 1u && bits->mitm_enabled == 1u) {
        securityMode = SecurityManager::SECURITY_MODE_ENCRYPTION_WITH_MITM;
    } else {
        securityMode = SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK;
    }
    *sm = securityMode;
}

/** Put BTLE to queue
 *
 *  @param  event    BTLE event income
 *
 *  @returns
 *      BTLE_OK             Success
 *      BTLE_ERROR_BUSY     Error because buffer is full
 */
static TZ1K_INLINE int BTLE_enqueueEventData(BTLEEvent_t *event)
{
    BTLE_IRQ_DISABLE_SAVE();
    uint8_t h = event_buffer.head;
    uint8_t i = (uint8_t)(h + 1) & (BTLE_INNER_EVENT_BUF_NUM - 1);

    if(BTLE_bufferEventIsFull()) {
        BTLE_IRQ_ENABLE_RESTORE();
        return BTLE_ERROR_BUSY;
    }

    if(i != event_buffer.tail) {
        memset(&event_buffer.buffer[h], 0, sizeof(BTLEEvent_t));
        memcpy(&event_buffer.buffer[h], event, sizeof(BTLEEvent_t));
        event_buffer.head = i;
    }

    BTLE_IRQ_ENABLE_RESTORE();
    return BTLE_OK;
}

/** Pick BTLE from queue
 *
 *  @param  event    Pointer store BTLE event
 *
 *  @returns
 *      BTLE_OK                 Success
 *      BTLE_EVENT_BUF_EMPTY    Buffer is empty
 */
static TZ1K_INLINE int BTLE_dequeueEventData(BTLEEvent_t *event)
{
    int ret = BTLE_OK;
    BTLE_IRQ_DISABLE_SAVE();
    uint8_t t = event_buffer.tail;

    if (event_buffer.head == t) {
        return BTLE_EVENT_BUF_EMPTY;
    } else {
        memset(event, 0, sizeof(BTLEEvent_t));
        memcpy(event, &event_buffer.buffer[t], sizeof(BTLEEvent_t));
        event_buffer.tail = (uint8_t)(t + 1) & (BTLE_INNER_EVENT_BUF_NUM - 1);
    }

    BTLE_IRQ_ENABLE_RESTORE();
    return ret;
}

/** Checks BTLE callback event buffer is avaiable or not
 *
 *  @param  NONE
 *
 *  @returns
 *      int                 Number of event in buffer
 */
static TZ1K_INLINE int BTLE_bufferEventIsAvaiable(void)
{
    BTLE_IRQ_DISABLE_SAVE();
    uint8_t h = event_buffer.head;
    uint8_t t = event_buffer.tail;
    BTLE_IRQ_ENABLE_RESTORE();

    return (int)(BTLE_INNER_EVENT_BUF_NUM + h - t) & (BTLE_INNER_EVENT_BUF_NUM - 1);
}

/** Checks BTLE callback event buffer is full or not
 *
 *  @param  bits    NONE
 *
 *  @returns
 *      0.          Buffer is not full
 *      1.          Buffer is full
 */
static TZ1K_INLINE int BTLE_bufferEventIsFull(void)
{

    uint8_t h = event_buffer.head;
    uint8_t t = event_buffer.tail;

    return ((h - t) == (BTLE_INNER_EVENT_BUF_NUM - 1)) ? 1 : 0;
}

/**  Checks status of internal Buffer, that uses for preventing internal buffer overflow.
 *  If the internal buffer has higher than 1/2 BTLE_INNER_EVENT_BUF_NUM. Public APIs will return error
 *  to prefer process event income
 *
 *  @param  bits    NONE
 *
 *  @returns
 *      True.          Bussy
 *      False.         Not bussy
 */
static bool BTLE_internIsBusy(void)
{
    bool isBusy = false;

    uint8_t threshold = (BTLE_INNER_EVENT_BUF_NUM / 2);
    if(BTLE_bufferEventIsAvaiable() >= threshold) {
        isBusy = true;
    }

    return isBusy;
}
/*******************************************************
* Implement functions process event automatic response *
*******************************************************
*/
static int BTLE_eventConnected(BTLEEvent_t *event);                    /* GAP event */
static int BTLE_eventConnectionDisconnected(BTLEEvent_t *event);
static int BTLE_eventConnectionUpdate(BTLEEvent_t *event);
static int BTLE_eventAdvReport(BTLEEvent_t *event);

static int BTLE_eventMtuExchangeResp(BTLEEvent_t *event);              /* GATT server event */
static int BTLE_eventMtuExchangeResult(BTLEEvent_t *event);
static int BTLE_eventNotificationSent(BTLEEvent_t *event);
static int BTLE_eventCharMultiReadResp(BTLEEvent_t *event);
static int BTLE_eventCharReadResp(BTLEEvent_t *event);
static int BTLE_eventCharWriteinResp(BTLEEvent_t *event);
static int BTLE_eventCharWritePost(BTLEEvent_t *event);
static int BTLE_eventDespReadResp(BTLEEvent_t *event);
static int BTLE_eventDespWriteinResp(BTLEEvent_t *event);

static int BTLE_eventDiscoveredPrimaryService(BTLEEvent_t *event);              /* GATT client event */
static int BTLE_eventDiscoveredPrimaryServiceByUuid(BTLEEvent_t *event);
static int BTLE_eventDiscoveredCharacteristic(BTLEEvent_t *event);
static int BTLE_eventCharacteristicReadout(BTLEEvent_t *event);
static int BTLE_eventCharacteristicWritein(BTLEEvent_t *event);
static int BTLE_eventDescriptorWritein(BTLEEvent_t *event);
static int BTLE_eventNotificationRecieved(BTLEEvent_t *event);
static int BTLE_eventIndicationResponse(BTLEEvent_t *event);

static int BTLE_eventLowPower(BTLEEvent_t *event);

static int BTLE_eventPairingConfirm(BTLEEvent_t *event);
static int BTLE_eventRequestSecurity(BTLEEvent_t *event);
static int BTLE_eventDisplayPasskeyReply(BTLEEvent_t *event);
static int BTLE_eventKeyboardPasskeyReply(BTLEEvent_t *event);
static int BTLE_eventBondingInquiryReply(BTLEEvent_t *event);
static int BTLE_eventRequestPairing(BTLEEvent_t *event);
static int BTLE_eventStartEncryption(BTLEEvent_t *event);
static int BTLE_eventSecurityRequestResponse(BTLEEvent_t *event);
static int BTLE_eventPairingCompleted(BTLEEvent_t *event);
static int BTLE_eventEncryptionChanged(BTLEEvent_t *event);
static int BTLE_eventBondingInfoStored(BTLEEvent_t *event);
static int BTLE_eventConnectionParameter(BTLEEvent_t *event);
/* event functions corresponding to BTLE_FLAG_xxx */
int (*event_process_functions[])(BTLEEvent_t *event) = {
    BTLE_eventConnected,
    BTLE_eventConnectionDisconnected,
    BTLE_eventConnectionUpdate,
    BTLE_eventAdvReport,
    BTLE_eventMtuExchangeResp,             /* GATT Server response */
    BTLE_eventMtuExchangeResult,
    BTLE_eventNotificationSent,
    BTLE_eventCharMultiReadResp,
    BTLE_eventCharReadResp,
    BTLE_eventCharWriteinResp,
    BTLE_eventCharWritePost,
    BTLE_eventDespReadResp,
    BTLE_eventDespWriteinResp,
    BTLE_eventDiscoveredPrimaryService,         /* GATT Client response */
    BTLE_eventDiscoveredPrimaryServiceByUuid,
    BTLE_eventDiscoveredCharacteristic,
    BTLE_eventCharacteristicReadout,
    BTLE_eventCharacteristicWritein,
    BTLE_eventDescriptorWritein,
    BTLE_eventNotificationRecieved,
    BTLE_eventIndicationResponse,
    BTLE_eventLowPower,                     /* Power Mananger */
    BTLE_eventPairingConfirm,               /* Security Manager */
    BTLE_eventRequestSecurity,
    BTLE_eventDisplayPasskeyReply,
    BTLE_eventKeyboardPasskeyReply,
    BTLE_eventBondingInquiryReply,
    BTLE_eventRequestPairing,
    BTLE_eventStartEncryption,
    BTLE_eventSecurityRequestResponse,
    BTLE_eventPairingCompleted,
    BTLE_eventEncryptionChanged,
    BTLE_eventBondingInfoStored,
    BTLE_eventConnectionParameter
};

/** BTLE callback function.
 *
 *  @param          NONE
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *    NONE.
 */
void btle_done_callback(BTLEEvent_t *event)
{
    /* Update status disconnection internal early */
    if(event->event_id == BTLE_CB_DISCONNECT) {
        tz1Gap::getInstance().setConnectionHandle(BLE_CONN_HANDLE_INVALID);
        btle_status_connect = BTLE_STATE_OFFLINE;
    }

    /* Add event data to queue */
    int error = BTLE_enqueueEventData(event);
    if(BTLE_OK != error) {
        BTLE_ERROR_HANDLER(error);
    }
}

/** Process event of btle_done_callback
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *      NONE
 */
static void BTLE_doEvent(BTLEEvent_t *event)
{
    int i;
    int error_code;
    BTLEEvent_t *p_event = event;
    uint64_t command = BTLE_FLAG_CONNECTED;

    switch(p_event->event_id) {
        /* common, server callbacks */
        case BTLE_CB_CONNECTION_COMPLETE:
            command_flag |= BTLE_FLAG_CONNECTED;
            break;
        case BTLE_CB_CONNECTION_UPDATE:
            break;
#if defined(TWIC_BLE_HWIP_V41)
        case BLELIB_SM_CB_CONNECTION_PARAMETER:
            command_flag |= BTLE_FLAG_CONNECTION_PARAMETER;
            break;
#endif
        case BTLE_CB_DISCONNECT:
            BTLE_initializeOnDisconnect();
            command_flag |= BTLE_FLAG_CONNECTION_DISCONNECTED;
            break;
        case BTLE_CB_ADVERTISE_REPORT:
            command_flag |= BTLE_FLAG_ADV_REPORT;
            break;
        case BTLE_CB_MTU_EXCHANGE_DEMAND:
            command_flag |= BTLE_FLAG_EXG_MTU_RESP;
            break;
        case BTLE_CB_MTU_EXCHANGE_RESULT:
            command_flag |= BTLE_FLAG_EXG_MTU_RESULT;
            break;
        case BTLE_CB_CHAR_DESP_WRITEIN_DEMAND:
            command_flag |= BTLE_FLAG_DESP_WRITEIN_RESP;
            break;
        case BTLE_CB_CHAR_DESP_READOUT_DEMAND:
            command_flag |= BTLE_FLAG_DESP_READOUT_RESP;
            break;
        case BTLE_CB_CHAR_VAL_READOUT_DEMAND:
            if(gap_role == BTLE_DEVICE_IS_PERIPHERAL &&
                    security_mode > SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK
                    && security_link_status == SecurityManager::NOT_ENCRYPTED) {
                command_flag |= BTLE_FLAG_REQUEST_SECURITY;
            }
            command_flag |= BTLE_FLAG_CHAR_READOUT_RESP;
            break;
        case BTLE_CB_CHAR_VAL_MULTI_READOUT_DEMAND:
            command_flag |= BTLE_FLAG_CHAR_MULTI_READOUT_RESP;
            break;
        case BTLE_CB_CHAR_VAL_WRITEIN_DEMAND:
            command_flag |= BTLE_FLAG_CHAR_WRITEIN_RESP;
            break;
        case BTLE_CB_ISR_DOWNSTREAM_REQUEST:
        case BTLE_CB_ISR_WAKEUP_REQUEST:
        case BTLE_CB_INDICATION_CONFIRM:
            break;
        case BTLE_CB_NOTIFICATION_SENT:
            command_flag |= BTLE_FLAG_NOTIFICATION_SENT;
            break;
        case BTLE_CB_QUEUED_WRITE_COMPLETE:
        case BTLE_CB_READ_RESULT_RESSI:
        case BTLE_CB_READ_RESULT_TX_POWER:
        case BTLE_CB_LONG_CHAR_DESP_READOUT_DEMAND:
        case BTLE_CB_LONG_CHAR_DESP_PREPARE_WRITEIN_DEMAND:
        case BTLE_CB_CHAR_DESP_EXEC_WRITEIN_DEMAND:
            break;
        case BTLE_CB_CHAR_VAL_WRITEIN_POST:
            command_flag |= BTLE_FLAG_CHAR_WRITE_POST;
            break;
        case BTLE_CB_LONG_CHAR_VAL_READOUT_DEMAND:
        case BTLE_CB_LONG_CHAR_VAL_PREPARE_WRITEIN_DEMAND:
        case BTLE_CB_CHAR_VAL_EXEC_WRITEIN_DEMAND:
            break;
        /* Client callback */
        case BTLE_CB_DISCOVER_PRIMARY_SERVICE:
            command_flag |= BTLE_FLAG_DISCOVERED_PRIMARY_SERVICE;
            break;
        case BTLE_CB_DISCOVER_PRIMARY_SERVICE_BY_UUID:
            command_flag |= BTLE_FLAG_DISCOVERED_PRIMARY_SERVICE_BY_UUID;
            break;
        case BTLE_CB_DISCOVER_INCLUDED_SERVICE:
            break;
        case BTLE_CB_DISCOVER_CHAR:
        case BTLE_CB_DISCOVER_CHAR_BY_UUID:
            command_flag |= BTLE_FLAG_DISCOVERED_CHAR;
            break;
        case BTLE_CB_DISCOVER_DESP:
            break;
        case BTLE_CB_CHAR_VAL_READOUT:
            command_flag |= BTLE_FLAG_CHAR_VAL_READOUT;
            break;
        case BTLE_CB_CHAR_VAL_WRITEIN_RESPONSE:
            command_flag |= BTLE_FLAG_CHAR_VAL_WRITEIN_RESPONSE;
            break;
        case BTLE_CB_CHAR_VAL_MULTI_READOUT:
            break;
        case BTLE_CB_LONG_CHAR_VAL_READOUT:
            break;
        case BTLE_CB_LONG_CHAR_VAL_WRITEIN_RESPONSE:
            break;
        case BTLE_CB_CHAR_DESP_READOUT:
            break;
        case BTLE_CB_CHAR_DESP_WRITEIN_RESPONSE:
            command_flag |= BTLE_FLAG_CHAR_DESP_WRITEIN_RESPONSE;
            break;
        case BTLE_CB_LONG_CHAR_DESP_READOUT:
            break;
        case BTLE_CB_LONG_CHAR_DESP_WRITEIN_RESPONSE:
            break;
        case BTLE_CB_INDICATION_RECEIVED:
            command_flag |= BTLE_FLAG_INDICATION_RESP;
            break;
        case BTLE_CB_NOTIFICATION_RECEIVED:
            command_flag |= BTLE_FLAG_NOTIFICATION_RECEIVED;
            break;
        case BTLE_CB_CHAR_VAL_WRITEIN_STARTED:
        case BTLE_CB_RELIABLE_WRITEIN_CONFIRM:
            break;
        /* SMP callbacks */
        case BTLE_CB_INITIATOR_RESPONDER:
            break;
        case BTLE_CB_PAIRING_COMPLETED:
            command_flag |= BTLE_FLAG_PAIRING_COMPLETED;
            break;
        case BTLE_CB_DISPLAY_PASSKEY_DEMAND:
            command_flag |= BTLE_FLAG_DISPLAY_PASSKEY_REPLY;
            break;

        case BTLE_CB_KEYBOARD_PASSKEY_DEMAND:
            command_flag |= BTLE_FLAG_KEYBOARD_PASSKEY_REPLY;
            break;

        case BTLE_CB_INQUIRY_BONDING_INFORMATION:
            command_flag |= BTLE_FLAG_BONDING_REPLY;
            break;

        case BTLE_CB_PAIRING_DEMAND:
            command_flag |= BTLE_FLAG_PAIRING_CONFIRM;
            break;

        case BTLE_CB_PAIRING_RESPONSE:
            if(p_event->evt.gaps_evt.params.paring_params.status != 0) {
                command_flag |= BTLE_FLAG_PAIRING_CONFIRM;
            }
            break;
        case BTLE_CB_SECURITY_REQUEST:
            /* Always accept */
            command_flag |= BTLE_FLAG_REQUEST_PAIRING;
            break;
        case BTLE_CB_ENCRYPTION_INFORMATION:
            command_flag |= BTLE_FLAG_ENCRYPTION_CHANGED;
            break;
        case BTLE_CB_BONDING_INFO_STORED:
            command_flag |= BTLE_FLAG_BONDING_INFO_STORED;
            break;
        default:
            break;
    }

    for( i = 1; i < (BTLE_FLAG_MAX_BIT - 1); ++i ) {
        if( command_flag & command ) {
            error_code = event_process_functions[i-1](p_event);
            if(error_code != 0) BTLE_ERROR_HANDLER(error_code);
            command_flag &= ~command;
        }
        command <<= 1;
    }
}
/* BLE Common event functions */
/** Connected event process
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *   BTLE_OK                        Success
 *   other.                         Error
 */
static int BTLE_eventConnected(BTLEEvent_t *event)
{
    BLEProtocol::AddressType_t peerAddressType;
    BLEProtocol::AddressBytes_t ownAddress;
    BLEProtocol::AddressType_t ownAddressType;
    tz1Gap::ConnectionParams_t connectionParams;
    Gap::Role_t role;
    if(event->evt.gaps_evt.params.connected.role) {
        role = Gap::PERIPHERAL;
        gap_role = BTLE_DEVICE_IS_PERIPHERAL;
    } else {
        role = Gap::CENTRAL;
        gap_role = BTLE_DEVICE_IS_CENTRAL;
    }

    peerAddressType = (Gap::AddressType_t) event->evt.gaps_evt.params.connected.peer_address_type;
    tz1Gap::getInstance().setConnectionHandle(event->evt.gaps_evt.conn_handle);
    btle_status_connect = BTLE_STATE_ONLINE;
    tz1Gap::getInstance().getAddress(&ownAddressType, ownAddress);

    connectionParams.minConnectionInterval = event->evt.gaps_evt.params.connected.conn_interval;
    connectionParams.maxConnectionInterval = event->evt.gaps_evt.params.connected.conn_interval;
    connectionParams.slaveLatency = event->evt.gaps_evt.params.connected.slave_latency;
    connectionParams.connectionSupervisionTimeout = event->evt.gaps_evt.params.connected.supervision_timeout;

    connected_device_info.handle = event->evt.gaps_evt.conn_handle;
    for(int i = 0; i < 6; ++i ) {
        connected_device_info.peer_addr <<= 8;
        connected_device_info.peer_addr += event->evt.gaps_evt.params.connected.peer.address[5 - i];
    }
    tz1Gap::getInstance().processConnectionEvent(event->evt.gaps_evt.conn_handle,
            role,
            peerAddressType,
            event->evt.gaps_evt.params.connected.peer.address,
            ownAddressType,
            ownAddress,
            &connectionParams);

    return BTLE_OK;
}

/** Disonnected event process
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *   BTLE_OK                        Success
 *   other.                         Error
 */
static int BTLE_eventConnectionDisconnected(BTLEEvent_t *event)
{
    Gap::DisconnectionReason_t _reason;

    switch(event->evt.gaps_evt.params.disconnected.reason) {
        case 0x00:
            _reason = Gap::REMOTE_USER_TERMINATED_CONNECTION;
            break;
        case 0x01:
            _reason = Gap::LOCAL_HOST_TERMINATED_CONNECTION;
            break;
        case 0xff:
            _reason = Gap::CONN_INTERVAL_UNACCEPTABLE;
            break;
        default:
            _reason = Gap::CONNECTION_TIMEOUT;
            break;
    }
    tz1Gap::getInstance().processDisconnectionEvent(event->evt.gaps_evt.conn_handle, _reason);

    return BTLE_OK;
}

/** Disconnected event process
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *   BTLE_OK                        Success
 *   other.                         Error
 */
static int BTLE_eventConnectionUpdate(BTLEEvent_t *event)
{
    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    return BTLE_OK;
}
#if defined(TWIC_BLE_HWIP_V41)

/** Connection Parameter event process
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *   BTLE_OK                        Success
 *   other.                         Error
 */
static int BTLE_eventConnectionParameter(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;

    status = twicIfLeConnectionParameterRequestReply(&conn_iface,
             event->evt.gaps_evt.params.conn_params.conn_interval_min,
             event->evt.gaps_evt.params.conn_params.conn_interval_max,
             event->evt.gaps_evt.params.conn_params.conn_latency,
             event->evt.gaps_evt.params.conn_params.supervision_timeout);

    api_done = BTLE_twicApiIsDone(TWIC_LECONNECTIONPARAMETERREQUESTREPLY, status);
    if(api_done != TWIC_STATUS_OK) {
        return BTLE_ERROR_MTU_EXCHANGE_RESPONSE;
    }

    return BTLE_OK;
}
#else
static int BTLE_eventConnectionParameter(BTLEEvent_t *event)
{
    return BTLE_ERROR_CANNOT_EXECUTE;
}
#endif

/** Scan advertising data report event process
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *   BTLE_OK                        Success
 *   other.                         Error
 */
static int BTLE_eventAdvReport(BTLEEvent_t *event)
{
    BLEProtocol::AddressBytes_t availableAddr;
    int i = 0;

    for(i = 0; i < event->evt.gaps_evt.params.adv_report.num_reports; i++) {
        memcpy(&availableAddr, &event->evt.gaps_evt.params.adv_report.reports[i].bd.address, 6);
        GapAdvertisingParams::AdvertisingType_t type;
        bool isScanResponse = false;
        switch (event->evt.gaps_evt.params.adv_report.reports[i].adv_event_type) {
            case TWIC_ADV_TYPE_IND :
                type = GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED;
                break;
            case TWIC_ADV_TYPE_DIRECT_IND :
                type = GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED;
                break;
            case TWIC_ADV_TYPE_SCAN_IND :
                type = GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED;
                isScanResponse = true;
                break;
            case TWIC_ADV_TYPE_NONCONN_IND :
                type = GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED;
                break;
            default:
                type = GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED;
                break;
        }

        tz1Gap::getInstance().processAdvertisementReport(availableAddr,
                event->evt.gaps_evt.params.adv_report.reports[i].rssi,
                isScanResponse,
                type,
                event->evt.gaps_evt.params.adv_report.reports[i].length,
                event->evt.gaps_evt.params.adv_report.reports[i].data);
    }

    return BTLE_OK;
}
/* Gatt server event functions */
/** Process MTU exchange response event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_MTU_EXCHANGE_RESPONSE.    Error MTU exchange response
 */
static int BTLE_eventMtuExchangeResp(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;
    uint16_t mtu_size = BTLE_ATT_MTU_DEFAULT;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    if(event->evt.gatts_evt.params.mtu_exchange.mtu_size < BTLE_ATT_MTU_DEFAULT) {
        mtu_size = event->evt.gatts_evt.params.mtu_exchange.mtu_size;
    }

    status = twicIfGattServerExgMtuResponse(&conn_iface, 0, mtu_size);

    api_done = BTLE_twicApiIsDone(TWIC_GATTSERVEREXGMTURESPONSE, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_MTU_EXCHANGE_RESPONSE;
    }

    return BTLE_OK;
}

/** MTU exchange result event process
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *   BTLE_OK                        Success
 *   other.                         Error
 */
static int BTLE_eventMtuExchangeResult(BTLEEvent_t *event)
{
    if(event->evt.gatts_evt.params.mtu_exchange_result.status == 0x00) {
        current_mtu = event->evt.gatts_evt.params.mtu_exchange_result.negotiated_mtu_size;
    } else {
        /* Set MTU is default value if remote response time out or unlikely error*/
        current_mtu = BTLE_ATT_MTU_DEFAULT;
    }

    return BTLE_OK;
}

/** Notification package sent event process
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *   BTLE_OK                        Success
 *   other.                         Error
 */
static int BTLE_eventNotificationSent(BTLEEvent_t *event)
{
    tz1GattServer::getInstance().tz1SentDataEvent(event->evt.gatts_evt.params.hvx_count.count);
    return BTLE_OK;
}

/** Process char multi read event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_CHAR_MUTIL_READ_RESPONSE. Error characteristics multiple read response
 */
static int BTLE_eventCharMultiReadResp(BTLEEvent_t *event)
{
    (void) event;
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfGattServerCharValMultiReadOutResponse(&conn_iface, 0, 0);

    api_done = BTLE_twicApiIsDone(TWIC_GATTSERVERCHARVALMULTIREADOUTRESPONSE, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_CHAR_MUTIL_READ_RESPONSE;
    }

    return BTLE_OK;
}

/** Process charactestis read response event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_RETRY.                          Retry
 *     BTLE_ERROR_CHAR_READ_RESPONSE.       Error characteristics read response
 */
static int BTLE_eventCharReadResp(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;
    uint8_t res_status = BTLE_CALLBACK_STATUS_OK;
    GattCharacteristic *p_char = tz1GattServer::getInstance().getCharacteristicFromHandle(event->evt.gatts_evt.params.read.handle);

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    /* Read response is done after update value so that update value can be done in readout_demand callback */
    if( command_flag & BTLE_FLAG_UPDATE_VALUE ) {
        return BTLE_RETRY;
    }

    if(p_char != NULL && p_char->isReadAuthorizationEnabled()) {
        res_status = BTLE_getServerRespReadAuthorizationDemand(event->evt.gatts_evt.conn_handle, p_char);
    }

    status = twicIfGattServerCharValReadOutResponse(&conn_iface, event->evt.gatts_evt.params.read.handle, res_status);

    api_done = BTLE_twicApiIsDone(TWIC_GATTSERVERCHARVALREADOUTRESPONSE, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_CHAR_READ_RESPONSE;
    }

    return BTLE_OK;
}

/** Utils handle write from central device event 
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *   BTLE_OK                        Success
 *   other.                         Error
 */
static void BTLE_handleWriteEvent(BTLEEvent_t *event)
{
    GattWriteCallbackParams cbParams = {
        .connHandle = event->evt.gatts_evt.conn_handle,
        .handle     = event->evt.gatts_evt.params.write.handle,
        .writeOp    = GattWriteCallbackParams::OP_WRITE_REQ,
        .offset     = event->evt.gatts_evt.params.write.offset,
        .len        = event->evt.gatts_evt.params.write.len,
        .data       = event->evt.gatts_evt.params.write.value
    };

    tz1GattServer::getInstance().tz1kDataWrittenEvent(&cbParams);
}

/** Process charateristic write in from central device event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                        Success
 *     BTLE_ERROR_CANNOT_EXECUTE.     Can not execute
 */
static int BTLE_eventCharWriteinResp(BTLEEvent_t *event)
{
    twicStatus_t status;
    int ret;
    uint8_t res_status = BTLE_CALLBACK_STATUS_OK;
    GattCharacteristic *p_char = tz1GattServer::getInstance().getCharacteristicFromHandle(event->evt.gatts_evt.params.write.handle);

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    if(p_char != NULL && p_char->isWriteAuthorizationEnabled()) {
        res_status = BTLE_getServerRespWriteAuthorizationDemand(event->evt.gatts_evt.conn_handle,
                     p_char,
                     event->evt.gatts_evt.params.write.offset,
                     event->evt.gatts_evt.params.write.len,
                     event->evt.gatts_evt.params.write.value);
    } else if (p_char != NULL) {
        BTLE_handleWriteEvent(event);
    }

    ret = btle_gatts_update_value(event->evt.gatts_evt.params.write.handle,
                               event->evt.gatts_evt.params.write.value,
                               event->evt.gatts_evt.params.write.len, true);
    if(ret != BTLE_OK) {
        return ret;
    }

    status = twicIfGattServerCharValWriteInResponse(&conn_iface, event->evt.gatts_evt.params.write.handle, res_status);
    return BTLE_twicApiIsDone(TWIC_GATTSERVERCHARVALWRITEINRESPONSE, status);
}

/** Process charateristic write without response in from central device event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *   BTLE_OK                        Success
 *   other.                         Error
 */
static int BTLE_eventCharWritePost(BTLEEvent_t *event)
{
    GattCharacteristic *p_char = tz1GattServer::getInstance().getCharacteristicFromHandle(event->evt.gatts_evt.params.write.handle);

    if(p_char != NULL) {
        BTLE_handleWriteEvent(event);
    }

    return BTLE_OK;
}

/** Process descriptor read from central device event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_RETRY.                          Retry
 *     BTLE_ERROR_DESP_READ_RESPONSE.       Error desp read response
 */
static int BTLE_eventDespReadResp(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    if( command_flag & BTLE_FLAG_UPDATE_VALUE ) {
        return BTLE_RETRY;
    }

    status = twicIfGattServerCharDespReadOutResponse(&conn_iface, event->evt.gatts_evt.params.read.handle, 0);

    api_done = BTLE_twicApiIsDone(TWIC_GATTSERVERCHARDESPREADOUTRESPONSE, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_DESP_READ_RESPONSE;
    }

    return BTLE_OK;
}

/** Process descriptor write in from central device event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 */
static int BTLE_eventDespWriteinResp(BTLEEvent_t *event)
{
    int ret;
    twicStatus_t status;
    uint8_t res_status = BTLE_CALLBACK_STATUS_OK;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    GattCharacteristic *p_char = tz1GattServer::getInstance().getCharacteristicFromHandle(event->evt.gatts_evt.params.write.handle - 1);

    ret = btle_gatts_update_value(event->evt.gatts_evt.params.write.handle,
                               event->evt.gatts_evt.params.write.value,
                               event->evt.gatts_evt.params.write.len, true);
    if(ret != BTLE_OK) {
        return ret;
    }

    if(p_char != NULL && (p_char->getProperties() & (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY |
                          GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE))) {
        /* Now Check if data written in Enable or Disable */
        if(*event->evt.gatts_evt.params.write.value != 0) {
            notify_indicate_mask |= (1u << (event->evt.gatts_evt.params.write.handle - 1));
            tz1GattServer::getInstance().tz1kHandleEvent(GattServerEvents::GATT_EVENT_UPDATES_ENABLED,
                    p_char->getValueAttribute().getHandle());
        } else {
            notify_indicate_mask &= ~(1u << (event->evt.gatts_evt.params.write.handle - 1));
            tz1GattServer::getInstance().tz1kHandleEvent(GattServerEvents::GATT_EVENT_UPDATES_DISABLED,
                    p_char->getValueAttribute().getHandle());
        }
    }

    status = twicIfGattServerCharDespWriteInResponse(&conn_iface, event->evt.gatts_evt.conn_handle, res_status);
    return BTLE_twicApiIsDone(TWIC_GATTSERVERCHARDESPWRITEINRESPONSE, status);
}
/* GattClient event functions*/
/** Process Discovered Primary Service event
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                      Success
 *     other.                       Error
 */
static int BTLE_eventDiscoveredPrimaryService(BTLEEvent_t *event)
{
    tz1GattClient::getInstance().primaryServiceCB(event->evt.gattc_evt.conn_handle,
            event->evt.gattc_evt.status,
            event->evt.gattc_evt.prams.pri_serv_disc_resp.next,
            event->evt.gattc_evt.error_handle,
            event->evt.gattc_evt.prams.pri_serv_disc_resp.attribute_handle,
            event->evt.gattc_evt.prams.pri_serv_disc_resp.end_group_handle,
            event->evt.gattc_evt.prams.pri_serv_disc_resp.uuid_lsb,
            event->evt.gattc_evt.prams.pri_serv_disc_resp.uuid_msb,
            event->evt.gattc_evt.prams.pri_serv_disc_resp.uuid_len);

    return BTLE_OK;
}

/** Process discovered primary service by uuid.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                      Success
 *     other.                       Error
 */
static int BTLE_eventDiscoveredPrimaryServiceByUuid(BTLEEvent_t *event)
{
    tz1GattClient::getInstance().primaryServiceByUuidCB(event->evt.gattc_evt.conn_handle,
            event->evt.gattc_evt.status,
            event->evt.gattc_evt.prams.pri_serv_by_uuid_disc_resp.next,
            event->evt.gattc_evt.error_handle,
            event->evt.gattc_evt.prams.pri_serv_by_uuid_disc_resp.attribute_handle,
            event->evt.gattc_evt.prams.pri_serv_by_uuid_disc_resp.end_group_handle);
    return BTLE_OK;
}

/** Process discovered characteristic.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                      Success
 *     other.                       Error
 */
static int BTLE_eventDiscoveredCharacteristic(BTLEEvent_t *event)
{
    tz1GattClient::getInstance().serviceCharCB(event->evt.gattc_evt.conn_handle,
            event->evt.gattc_evt.status,
            event->evt.gattc_evt.prams.char_disc_resp.next,
            event->evt.gattc_evt.error_handle,
            event->evt.gattc_evt.prams.char_disc_resp.attribute_handle,
            event->evt.gattc_evt.prams.char_disc_resp.char_properties,
            event->evt.gattc_evt.prams.char_disc_resp.char_value_handle,
            event->evt.gattc_evt.prams.char_disc_resp.uuid_lsb,
            event->evt.gattc_evt.prams.char_disc_resp.uuid_msb,
            event->evt.gattc_evt.prams.char_disc_resp.uuid_len);

    return BTLE_OK;
}

/** Process Characteristic Readout result event
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                      Success
 *     other.                       Error
 */
static int BTLE_eventCharacteristicReadout(BTLEEvent_t *event)
{
    tz1GattClient::getInstance().charReadCB(event->evt.gattc_evt.conn_handle,
                                            event->evt.gattc_evt.status,
                                            event->evt.gattc_evt.prams.read_resp.offset,
                                            event->evt.gattc_evt.prams.read_resp.len,
                                            event->evt.gattc_evt.prams.read_resp.value);

    return BTLE_OK;
}

/** Process Characteristic Write-in result event
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                      Success
 *     other.                       Error
 */
static int BTLE_eventCharacteristicWritein(BTLEEvent_t *event)
{
    tz1GattClient::getInstance().charWriteCB(event->evt.gattc_evt.conn_handle,
            event->evt.gattc_evt.status);

    return BTLE_OK;
}

/** Process Descriptor Write-in result event
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                      Success
 *     other.                       Error
 */
static int BTLE_eventDescriptorWritein(BTLEEvent_t *event)
{
    tz1GattClient::getInstance().charWriteCB(event->evt.gattc_evt.conn_handle,
            event->evt.gattc_evt.status);

    return BTLE_OK;
}

/** Process Descriptor Write-in result event
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                      Success
 *     other.                       Error
 */
static int BTLE_eventNotificationRecieved(BTLEEvent_t *event)
{
    tz1GattClient::getInstance().handleHVXEvent(event->evt.gattc_evt.conn_handle,
            event->evt.gattc_evt.status,
            BLE_HVX_NOTIFICATION,
            event->evt.gattc_evt.prams.hvx_resp.handle,
            event->evt.gattc_evt.prams.hvx_resp.len,
            event->evt.gattc_evt.prams.hvx_resp.value);

    return BTLE_OK;
}

/** Process indication confirmation response.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_INDICATION_RESPONSE.      Error indication response
 */
static int BTLE_eventIndicationResponse(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    tz1GattClient::getInstance().handleHVXEvent(event->evt.gattc_evt.conn_handle,
            event->evt.gattc_evt.status,
            BLE_HVX_INDICATION,
            event->evt.gattc_evt.prams.hvx_resp.handle,
            event->evt.gattc_evt.prams.hvx_resp.len,
            event->evt.gattc_evt.prams.hvx_resp.value);

    status = twicIfLeGattClientIndicationConfirmationResponse(&conn_iface);

    api_done = BTLE_twicApiIsDone(TWIC_GATTCLIENTINDICATIONCONFIRMATIONRESPONSE, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_INDICATION_RESPONSE;
    }

    return BTLE_OK;
}
/* Power event functions */
/** Process low power setting event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_LOWPOWER.                 Error low power
 */
static int BTLE_eventLowPower(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;
    bool idle, conn_scan, discoverable;

    if(BTLE_POWER_MODE == BTLE_POWER_LOW_SPEED) {
        idle = conn_scan = discoverable = true;
    } else if(BTLE_POWER_MODE == BTLE_POWER_NORMAL_SPEED) {
        idle = false;
        conn_scan = discoverable = true;
    } else if(BTLE_POWER_HIGH_SPEED) {
        idle = conn_scan = discoverable = false;
    }

    status = twicIfLeCeLowPowerMode(&conn_iface, idle, discoverable, conn_scan, false);
    api_done = BTLE_twicApiIsDone(TWIC_LECELOWPOWERMODE, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_LOWPOWER;
    }

    if(BTLE_POWER_MODE == BTLE_POWER_LOW_SPEED) {
        BTLE_configureTz1emOnLowPowerOn();
    } else {
        BTLE_configureTz1emOnLowPowerOff();
    }

    return BTLE_OK;
}

/* Security manager event response */
/** Process pairing confirm sub event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_PAIRING_CONFIRM.          Error pairing confirm
 */
static int BTLE_eventResponderSendPairingConfirm(void)
{
    twicStatus_t status;
    uint64_t bleAddr;
    int api_done;
    BLEProtocol::AddressType_t addr_type;
    bool own_idkey;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    tz1Gap::getInstance().getAddress(&addr_type, (uint8_t *)&bleAddr);
    /* Check Address type for setting IDKEY bit */
    if(addr_type == BLEProtocol::AddressType::RANDOM_STATIC) {
        own_idkey = false;
    } else {
        own_idkey = true;
    }

    status = twicIfLeSmSlvPairingConfirm(&conn_iface,
                                         (twicSmpIoCapability_t) security_info.securityParams.io_capability,
                                         (BTLE_OOB_DATA_FLAG != 0) ? true : false,
                                         security_info.securityParams.auth_req_bonding,
                                         security_info.securityParams.auth_req_mitm_protection,
                                         BTLE_MAX_ENCODE_KEY_SIZE,
                                         (BTLE_INITIATOR_DISTRIBUTES_ENCKEY != 0) ? true : false,
                                         own_idkey,
                                         (BTLE_INITIATOR_DISTRIBUTES_SIGN != 0) ? true : false,
                                         (BTLE_RESPONDER_DISTRIBUTES_ENCKEY != 0) ? true : false,
                                         own_idkey,
                                         (BTLE_RESPONDER_DISTRIBUTES_SIGN != 0) ? true : false);

    api_done = BTLE_twicApiIsDone(TWIC_LESMSLVPAIRINGCONFIRM, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_PAIRING_CONFIRM;
    }

    return BTLE_OK;
}

/** Process pairing failed sub event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_PAIRING_CONFIRM.          Error pairing confirm
 */
static int BTLE_eventResponderSendPairingFailed(void)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmSlvPairingFailed(&conn_iface, TWIC_PAIRING_NOT_SUPPORTED);

    api_done = BTLE_twicApiIsDone(TWIC_LESMSLVPAIRINGFAILED, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_PAIRING_CONFIRM;
    }

    return BTLE_OK;
}

/** Process pairing confirm event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_PAIRING_CONFIRM.          Error pairing confirm
 */
static int BTLE_eventResponderPairingConfirm(BTLEEvent_t *event)
{
    /*  Temporary alway accept pairing request */
    if( BTLE_CALLBACK_STATUS_OK == event->evt.gaps_evt.params.paring_params.status ) {
        return BTLE_eventResponderSendPairingConfirm();
    } else {
        return BTLE_eventResponderSendPairingFailed();
    }
}

/** Utils to parse security mode to security paramter global variable
 *
 *  @param security_mode        Current security mode
 *
 *  @returns
 *    NONE
 */
static void BTLE_parseSecurityMode(SecurityManager::SecurityMode_t security_mode)
{
    switch(security_mode) {
        case SecurityManager::SECURITY_MODE_NO_ACCESS:
            break;
        case SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK:
            security_info.securityParams.auth_req_bonding = false;
            security_info.securityParams.auth_req_mitm_protection = false;
            break;
        case SecurityManager::SECURITY_MODE_ENCRYPTION_NO_MITM:
            security_info.securityParams.auth_req_bonding = true;
            security_info.securityParams.auth_req_mitm_protection = false;
            break;
        case SecurityManager::SECURITY_MODE_ENCRYPTION_WITH_MITM:
            security_info.securityParams.auth_req_bonding = true;
            security_info.securityParams.auth_req_mitm_protection = true;
            break;
        /* Not supported in current version */
        case SecurityManager::SECURITY_MODE_SIGNED_NO_MITM:
        case SecurityManager::SECURITY_MODE_SIGNED_WITH_MITM:
            break;
        default:
            break;
    }
}

/** Process request security event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_REQUEST_SECURITY.         Error request security
 */
static int BTLE_eventRequestSecurity(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    BTLE_parseSecurityMode(security_mode);

    status = twicIfLeSmSlvSecurityRequest(&conn_iface,
                                          security_info.securityParams.auth_req_bonding,
                                          security_info.securityParams.auth_req_mitm_protection);

    api_done = BTLE_twicApiIsDoneWithRetry(TWIC_LESMSLVSECURITYREQUEST, status);
    if(api_done != BTLE_RETRY && api_done != BTLE_OK) {
        return BTLE_ERROR_REQUEST_SECURITY;
    }

    return BTLE_OK;
}

/** Process request pairing.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_REQUEST_PAIRING.          Error request pairing
 */
static int BTLE_eventRequestPairing(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;
    uint64_t bleAddr;
    BLEProtocol::AddressType_t addr_type;
    bool own_idkey;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    BTLE_parseSecurityMode(security_mode);

    tz1Gap::getInstance().getAddress(&addr_type, (uint8_t *)&bleAddr);
    /* Check Address type for setting IDKEY bit */
    if(addr_type == BLEProtocol::AddressType::RANDOM_STATIC) {
        own_idkey = false;
    } else {
        own_idkey = true;
    }

    status = twicIfLeSmMasPairingRequest(&conn_iface,
                                         (twicSmpIoCapability_t) security_info.securityParams.io_capability,
                                         BTLE_OOB_DATA_FLAG,
                                         security_info.securityParams.auth_req_bonding,
                                         security_info.securityParams.auth_req_mitm_protection,
                                         BTLE_MAX_ENCODE_KEY_SIZE,
                                         (BTLE_INITIATOR_DISTRIBUTES_ENCKEY != 0) ? true : false,
                                         own_idkey,
                                         (BTLE_INITIATOR_DISTRIBUTES_SIGN != 0) ? true : false,
                                         (BTLE_RESPONDER_DISTRIBUTES_ENCKEY != 0) ? true : false,
                                         own_idkey,
                                         (BTLE_RESPONDER_DISTRIBUTES_SIGN != 0) ? true : false);

    api_done = BTLE_twicApiIsDone(TWIC_LESMMASPAIRINGREQUEST, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_REQUEST_PAIRING;
    }

    return BTLE_OK;
}

/** Process start encryption.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_START_ENCRYPTION.         Error start ecryption
 */
static int BTLE_eventStartEncryption(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE || command_flag & BTLE_FLAG_CONNECTION_UPDATE ) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmMasStartEncryption(&conn_iface,
                                          &bonding_inquiry_info->remoteKey.ediv,
                                          &bonding_inquiry_info->remoteKey.rand,
                                          &bonding_inquiry_info->remoteKey.ltk,
                                          bonding_inquiry_info->keySize);

    api_done = BTLE_twicApiIsDone(TWIC_LESMMASSTARTENCRYPTION, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_START_ENCRYPTION;
    }

    return BTLE_OK;
}

/** Process security request response.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_SECURITY_REQUEST_RESPONSE. Error security request respone
 */
static int BTLE_eventSecurityRequestResponse(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmMasSecurityReject(&conn_iface, TWIC_UNSPECIFIED_REASON);

    api_done = BTLE_twicApiIsDone(TWIC_LESMMASSECURITYREJECT, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_SECURITY_REQUEST_RESPONSE;
    }

    return BTLE_OK;
}

/** Process security paring completed event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 */
static int BTLE_eventPairingCompleted(BTLEEvent_t *event)
{
    SecurityManager::SecurityCompletionStatus_t securityPairStatus;
    SecurityManager::SecurityMode_t securityMode;

    switch(event->evt.gaps_evt.params.paring_status.status) {
        case TWIC_SMP_SUCCESS:
            securityPairStatus = SecurityManager::SEC_STATUS_SUCCESS;
            memcpy(&auth_info, &event->evt.gaps_evt.params.paring_status.auth_info, sizeof(twicAuthInfo_t));
            BTLE_getSecurityModeFromAuthorizationParams(&auth_info, &securityMode);
            tz1SecurityManager::getInstance().processLinkSecuredEvent(event->evt.gaps_evt.conn_handle, securityMode);
            break;
        case TWIC_PAIRING_NOT_SUPPORTED:
            securityPairStatus = SecurityManager::SEC_STATUS_PAIRING_NOT_SUPP;
            break;
        case TWIC_CONFIRM_VALUE_FAILED:
            securityPairStatus = SecurityManager::SEC_STATUS_CONFIRM_VALUE;
            break;
        case TWIC_HOST_REJECTED_PIN_KEY:
            securityPairStatus = SecurityManager::SEC_STATUS_PASSKEY_ENTRY_FAILED;
            break;
        case TWIC_AUTHENTICATION_REQUIREMENTS:
            securityPairStatus = SecurityManager::SEC_STATUS_AUTH_REQ;
            break;
        case TWIC_OOB_NOT_AVAILABLE:
            securityPairStatus = SecurityManager::SEC_STATUS_OOB_NOT_AVAILABLE;
            break;
        case TWIC_SMP_TIME_OUT:
        case TWIC_SMP_TIME_OUT_NEED_RECONNECTION:
            securityPairStatus = SecurityManager::SEC_STATUS_TIMEOUT;
            break;
        default:
            securityPairStatus = SecurityManager::SEC_STATUS_UNSPECIFIED;
            break;
    }

    tz1SecurityManager::getInstance().processSecuritySetupCompletedEvent(event->evt.gaps_evt.conn_handle, securityPairStatus);

    return BTLE_OK;
}

/** Process security encryption changed event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 */
static int BTLE_eventEncryptionChanged(BTLEEvent_t *event)
{
    SecurityManager::SecurityMode_t securityMode;

    security_link_status = (SecurityManager::LinkSecurityStatus_t) event->evt.gaps_evt.params.enc_changed.link_status;
    if( security_link_status == SecurityManager::ENCRYPTED && bonding_inquiry_info != NULL) {
        if(bonding_inquiry_info->isKeyStored) {
            BTLE_getSecurityModeFromAuthorizationParams(&auth_info, &securityMode);
            tz1SecurityManager::getInstance().processLinkSecuredEvent(event->evt.gaps_evt.conn_handle, securityMode);
        }
    }

    return BTLE_OK;
}

/** Process security bonding information can store event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 */
static int BTLE_eventBondingInfoStored(BTLEEvent_t *event)
{
    tz1SecurityManager::getInstance().processSecurityContextStoredEvent(event->evt.gaps_evt.conn_handle);

    return BTLE_OK;
}

/** Process pairing response
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_PAIRING_CONFIRM.          Error pairing confirm
 */
static int BTLE_eventInitiatorPairingResponse(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;
    twicSmReasonCode_t resp;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    if( event->evt.gaps_evt.params.paring_params.status == 0 ) {
        command_flag &= ~BTLE_FLAG_PAIRING_CONFIRM;
        return 0;
    }

    resp = BTLE_getSmReasonCode(event->evt.gaps_evt.params.paring_params.status);
    status = twicIfLeSmMasPairingFailed(&conn_iface, resp);

    api_done = BTLE_twicApiIsDone(TWIC_LESMMASPAIRINGFAILED, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_PAIRING_CONFIRM;
    }

    return BTLE_OK;
}

/** Process pairing confirm event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_PAIRING_CONFIRM.          Error pairing confirm
 */
static int BTLE_eventPairingConfirm(BTLEEvent_t *event)
{
    int ret = BTLE_OK;

    if( BTLE_INITIATOR == event->evt.gaps_evt.params.paring_params.initator_responder ) {
        ret = BTLE_eventInitiatorPairingResponse(event);
    } else {
        ret = BTLE_eventResponderPairingConfirm(event);
    }

    tz1SecurityManager::getInstance().processSecuritySetupInitiatedEvent(
        event->evt.gaps_evt.conn_handle,
        event->evt.gaps_evt.params.paring_params.bond,
        event->evt.gaps_evt.params.paring_params.mitm,
        (SecurityManager::SecurityIOCapabilities_t) event->evt.gaps_evt.params.paring_params.iocap);
    return ret;
}

/** Process responder displayed passkey reply sub event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_DISPLAY_PASSKEY_REPLY.    Error display passkey reply
 */
static int BTLE_eventResponderDisplayPasskeyPositiveReply(void)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmSlvDpPasskeyEntryReply(&conn_iface, &security_info.passKey);

    api_done = BTLE_twicApiIsDone(TWIC_LESMSLVDPPASSKEYENTRYREPLY, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_DISPLAY_PASSKEY_REPLY;
    }

    return BTLE_OK;
}

/** Process responder displayed passkey negative reply sub event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_DISPLAY_PASSKEY_REPLY.    Error display passkey reply
 */
static int BTLE_eventResponderDisplayPasskeyNegativeReply(void)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmSlvDpPasskeyEntryNegativeReply(&conn_iface);

    api_done = BTLE_twicApiIsDone(TWIC_LESMSLVDPPASSKEYENTRYNEGATIVEREPLY, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_DISPLAY_PASSKEY_REPLY;
    }

    return BTLE_OK;
}

/** Process responder displayed passkey reply.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_DISPLAY_PASSKEY_REPLY.    Error display passkey reply
 */
static int BTLE_eventResponderDisplayPasskeyReply(BTLEEvent_t *event)
{
    int ret = BTLE_OK;
    bool positive_reply = false;

    if(has_passkey) {
        positive_reply = true;
    } else {
        if(btle_utils_generate_random_twic_passkey(security_info.passKey.key) == 0) {
            positive_reply = true;
        }
    }

    if(positive_reply) {
        ret = BTLE_eventResponderDisplayPasskeyPositiveReply();
        btle_callback_display_passkey_event(event->evt.gaps_evt.conn_handle, security_info.passKey.key);
    } else {
        ret = BTLE_eventResponderDisplayPasskeyNegativeReply();
        btle_callback_display_passkey_event(event->evt.gaps_evt.conn_handle, NULL);
    }

    return ret;
}

/** Process responder keyboard passkey reply subevent.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_KEYBOARD_PASSKEY_REPLY.   Error keyboard passkey reply
 */
static int BTLE_eventResponderKeyboardPasskeyPositiveReply(void)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmSlvKbPasskeyEntryReply(&conn_iface, &security_info.passKey);

    api_done = BTLE_twicApiIsDone(TWIC_LESMSLVKBPASSKEYENTRYREPLY, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_KEYBOARD_PASSKEY_REPLY;
    }

    return BTLE_OK;
}

/** Process responder keyboard passkey negative reply subevent.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_KEYBOARD_PASSKEY_REPLY.   Error keyboard passkey reply
 */
static int BTLE_eventResponderKeyboardPasskeyNegativeReply(void)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmSlvKbPasskeyEntryNegativeReply(&conn_iface);

    api_done = BTLE_twicApiIsDone(TWIC_LESMSLVKBPASSKEYENTRYNEGATIVEREPLY, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_KEYBOARD_PASSKEY_REPLY;
    }

    return BTLE_OK;
}

/** Process responder keyboard passkey reply.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_KEYBOARD_PASSKEY_REPLY.   Error keyboard passkey reply
 */
static int BTLE_eventResponderKeyboardPasskeyReply(void)
{
    if( has_passkey ) {
        return BTLE_eventResponderKeyboardPasskeyPositiveReply();
    } else {
        return BTLE_eventResponderKeyboardPasskeyNegativeReply();
    }
}

/** Process initiator displayed passkey reply sub event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_DISPLAY_PASSKEY_REPLY.    Error display passkey reply
 */
static int BTLE_eventInitiatorDisplayPasskeyPositiveReply(void)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmMasDpPasskeyEntryReply(&conn_iface, &security_info.passKey);

    api_done = BTLE_twicApiIsDone(TWIC_LESMMASDPPASSKEYENTRYREPLY, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_DISPLAY_PASSKEY_REPLY;
    }

    return BTLE_OK;
}

/** Process initiator displayed passkey negative reply sub event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_DISPLAY_PASSKEY_REPLY.    Error display passkey reply
 */
static int BTLE_eventInitiatorDisplayPasskeyNegativeReply(void)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmMasDpPasskeyEntryNegativeReply(&conn_iface);

    api_done = BTLE_twicApiIsDone(TWIC_LESMMASDPPASSKEYENTRYNEGATIVEREPLY, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_DISPLAY_PASSKEY_REPLY;
    }

    return BTLE_OK;
}

/** Process initiator displayed passkey reply.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_DISPLAY_PASSKEY_REPLY.    Error display passkey reply
 */
static int BTLE_eventInitiatorDisplayPasskeyReply(BTLEEvent_t *event)
{
    int ret;

    if( has_passkey ) {
        ret = BTLE_eventInitiatorDisplayPasskeyPositiveReply();
        btle_callback_display_passkey_event(event->evt.gaps_evt.conn_handle, security_info.passKey.key);
    } else {
        ret = BTLE_eventInitiatorDisplayPasskeyNegativeReply();
        btle_callback_display_passkey_event(event->evt.gaps_evt.conn_handle, NULL);
    }

    return ret;
}

/** Process initiator keyboard passkey reply subevent.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_INDICATION_RESPONSE.      Indication response
 */
static int BTLE_eventInitiatorKeyboardPasskeyPositiveReply(void)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmMasKbPasskeyEntryReply(&conn_iface, &security_info.passKey);

    api_done = BTLE_twicApiIsDone(TWIC_LESMMASKBPASSKEYENTRYREPLY, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_KEYBOARD_PASSKEY_REPLY;
    }

    return BTLE_OK;
}

/** Process initiator keyboard passkey negative reply subevent.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_KEYBOARD_PASSKEY_REPLY.   Error keyboard passkey reply
 */
static int BTLE_eventInitiatorKeyboardPasskeyNegativeReply(void)
{
    twicStatus_t status;
    int api_done;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmMasKbPasskeyEntryNegativeReply(&conn_iface);

    api_done = BTLE_twicApiIsDone(TWIC_LESMMASKBPASSKEYENTRYNEGATIVEREPLY, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_KEYBOARD_PASSKEY_REPLY;
    }

    return BTLE_OK;
}

/** Process initiator keyboard passkey reply.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_KEYBOARD_PASSKEY_REPLY.   Error keyboard passkey reply
 */
static int BTLE_eventInitiatorKeyboardPasskeyReply(void)
{
    if( has_passkey ) {
        return BTLE_eventInitiatorKeyboardPasskeyPositiveReply();
    } else {
        return BTLE_eventInitiatorKeyboardPasskeyNegativeReply();
    }
}

/** Process display passkey event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_DISPLAY_PASSKEY_REPLY.    Error display passkey reply
 */
static int BTLE_eventDisplayPasskeyReply(BTLEEvent_t *event)
{
    if( BTLE_INITIATOR == event->evt.gaps_evt.params.passkey_display.initator_responder ) {
        return BTLE_eventInitiatorDisplayPasskeyReply(event);
    } else {
        return BTLE_eventResponderDisplayPasskeyReply(event);
    }
}

/** Process keyboard passkey event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_KEYBOARD_PASSKEY_REPLY.   Error keyboard passkey reply
 */
static int BTLE_eventKeyboardPasskeyReply(BTLEEvent_t *event)
{
    if( BTLE_INITIATOR == event->evt.gaps_evt.params.passkey_keyboard.initator_responder ) {
        return BTLE_eventInitiatorKeyboardPasskeyReply();
    } else {
        return BTLE_eventResponderKeyboardPasskeyReply();
    }
}

/** Process responder bonding information reply sub event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_BONDING_INQUIRY_REPLY.    Error bonding inquiry reply
 */
static int BTLE_eventResponderBondingInquiryPositiveReply(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;
    (void) event;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmSlvBondingInformationReply(&conn_iface,
             &bonding_inquiry_info->remoteKey.ediv,
             &bonding_inquiry_info->remoteKey.rand,
             &bonding_inquiry_info->remoteKey.ltk,
             &bonding_inquiry_info->remoteKey.irk,
             &bonding_inquiry_info->remoteKey.csrk,
             &bonding_inquiry_info->localKey.ediv,
             &bonding_inquiry_info->localKey.rand,
             &bonding_inquiry_info->localKey.ltk,
             &bonding_inquiry_info->localKey.irk,
             &bonding_inquiry_info->localKey.csrk,
             bonding_inquiry_info->keySize);

    api_done = BTLE_twicApiIsDone(TWIC_LESMSLVBONDINGINFORMATIONREPLY, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_BONDING_INQUIRY_REPLY;
    }

    return BTLE_OK;
}

/** Process responder bonding information negative reply sub event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_BONDING_INQUIRY_REPLY.    Error bonding inquiry reply
 */
static int BTLE_eventResponderBondingInquiryNegativeReply(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;
    (void) event;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmSlvBondingInformationNegativeReply(&conn_iface);

    api_done = BTLE_twicApiIsDone(TWIC_LESMSLVBONDINGINFORMATIONNEGATIVEREPLY, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_BONDING_INQUIRY_REPLY;
    }

    return BTLE_OK;
}

/** Process responder reply bonding information inquiry.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_BONDING_INQUIRY_REPLY.    Error bonding inquiry reply
 */
static int BTLE_eventResponderBondingInquiryReply(BTLEEvent_t *event)
{
    if( BTLE_CALLBACK_STATUS_OK == event->evt.gaps_evt.params.inquiry_bond_info.status ) {
        return BTLE_eventResponderBondingInquiryPositiveReply(event);
    } else {
        return BTLE_eventResponderBondingInquiryNegativeReply(event);
    }
}

/** Process initiator bonding information reply sub event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_BONDING_INQUIRY_REPLY.    Error bonding inquiry reply
 */
static int BTLE_eventInitiatorBondingInquiryPositiveReply(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;
    (void) event;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmMasBondingInformationReply(&conn_iface,
             &bonding_inquiry_info->remoteKey.ediv,
             &bonding_inquiry_info->remoteKey.rand,
             &bonding_inquiry_info->remoteKey.ltk,
             &bonding_inquiry_info->remoteKey.irk,
             &bonding_inquiry_info->remoteKey.csrk,
             &bonding_inquiry_info->localKey.ediv,
             &bonding_inquiry_info->localKey.rand,
             &bonding_inquiry_info->localKey.ltk,
             &bonding_inquiry_info->localKey.irk,
             &bonding_inquiry_info->localKey.csrk,
             bonding_inquiry_info->keySize);

    api_done = BTLE_twicApiIsDone(TWIC_LESMMASBONDINGINFORMATIONREPLY, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_BONDING_INQUIRY_REPLY;
    }

    return BTLE_OK;
}

/** Process initiator bonding information negative reply sub event.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_BONDING_INQUIRY_REPLY.    Error bonding inquiry reply
 */
static int BTLE_eventInitiatorBondingInquiryNegativeReply(BTLEEvent_t *event)
{
    twicStatus_t status;
    int api_done;
    (void) event;

    if(btle_status_connect == BTLE_STATE_OFFLINE) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    status = twicIfLeSmMasBondingInformationNegativeReply(&conn_iface);

    api_done = BTLE_twicApiIsDone(TWIC_LESMMASBONDINGINFORMATIONNEGATIVEREPLY, status);
    if(api_done != BTLE_OK) {
        return BTLE_ERROR_BONDING_INQUIRY_REPLY;
    }

    return BTLE_OK;
}

/** Process initiator reply bonding information inquiry.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_BONDING_INQUIRY_REPLY.    Error bonding inquiry reply
 */
static int BTLE_eventInitiatorBondingInquiryReply(BTLEEvent_t *event)
{
    if( BTLE_CALLBACK_STATUS_OK == event->evt.gaps_evt.params.inquiry_bond_info.status ) {
        return BTLE_eventInitiatorBondingInquiryPositiveReply(event);
    } else {
        return BTLE_eventInitiatorBondingInquiryNegativeReply(event);
    }
}

/** Process reply bonding information inquiry.
 *
 *  @param event                    The event of BTLE.
 *
 *  @returns
 *     BTLE_OK                              Success
 *     BTLE_ERROR_CANNOT_EXECUTE.           Can not execute
 *     BTLE_ERROR_BONDING_INQUIRY_REPLY.    Error bonding inquiry reply
 */
static int BTLE_eventBondingInquiryReply(BTLEEvent_t *event)
{
    if( command_flag & BTLE_FLAG_CONNECTION_UPDATE ) {
        return BTLE_ERROR_CANNOT_EXECUTE;
    }

    bonding_inquiry_info = event->evt.gaps_evt.params.inquiry_bond_info.info_bonding;

    if( BTLE_INITIATOR == event->evt.gaps_evt.params.inquiry_bond_info.initator_responder ) {
        return BTLE_eventInitiatorBondingInquiryReply(event);
    } else {
        return BTLE_eventResponderBondingInquiryReply(event);
    }
}
