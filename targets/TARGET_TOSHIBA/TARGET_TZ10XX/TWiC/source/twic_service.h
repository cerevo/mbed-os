/**
 * @file twic_service.h
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

#ifndef _TWIC_SERVICE_H_
#define _TWIC_SERVICE_H_

/*
 * COMMAND
 *
 */

#define TWIC_TCU_LE_CLI_DISCOVER_PRM_SVC_BY_UUID_EVENT                 (0x43)
#define TWIC_TCU_LE_CLI_DISCOVER_CHAR_DECL_EVENT                       (0x45)
#define TWIC_TCU_LE_CLI_DISCOVER_CHAR_DESP_EVENT                       (0x47)
#define TWIC_TCU_LE_CLI_READ_CHAR_VAL_EVENT                            (0x48)
#define TWIC_TCU_LE_CLI_WRITE_CHAR_DESP_EVENT                          (0x4b)
#define TWIC_TCU_MNG_LE_CREATE_CONNECTION_COMPLETE_EVENT               (0x4c)
#define TWIC_TCU_MNG_LE_CONNECTION_UPDATE_EVENT                        (0x4e)
#define TWIC_TCU_MNG_LE_READ_REMOTE_USED_FEATURES_EVENT                (0x4f)
#define TWIC_TCU_MNG_LE_L2CAP_CONNECTION_UPDATE_RESP_EVENT             (0x50)
#define TWIC_TCU_MNG_LE_CON_UPDATE_ACCEPT_RESP                         (0x51)
#define TWIC_TCU_MNG_LE_UPDATE_CONN_REQ_EVENT                          (0x52)
#define TWIC_TCU_MNG_LE_L2CAP_CMD_REJECT_EVENT                         (0x53)
#define TWIC_TCU_MNG_LE_GEN_RESOLVABLE_BDADDR_RESP                     (0x54)
#define TWIC_TCU_MNG_LE_RESOLVE_BDADDR_RESP                            (0x55)
#define TWIC_TCU_MNG_LE_READ_REMOTE_VERSION_RESP                       (0x59)
#define TWIC_TCU_MNG_LE_SET_IR_VALUE_RESP                              (0x5b)
#if defined(TWIC_BLE_HWIP_V41)
#define TWIC_TCU_MNG_LE_ENCRYPT_RESP                                   (0x5c)
/* TCU_MNG_LE_ENCRYPT_REQ: HCI LE Encrypt Command. */
#define TCU_MNG_LE_SET_ADVERTISE_DATA_RESP                             (0x5d)
#endif
#define TWIC_TCU_MNG_LE_REM_CON_PARAM_REQ_EVENT                        (0x5e)

#define TWIC_TCU_MNG_LE_INIT_RESP                                      (0x81)
#define TWIC_TCU_MNG_LE_READ_LOCAL_SUPPORTED_FEATURES_RESP             (0x82)
#define TWIC_TCU_MNG_LE_SET_RAND_ADDRESS_RESP                          (0x84)
#define TWIC_TCU_MNG_LE_READ_WHITELIST_SIZE_RESP                       (0x85)
#define TWIC_TCU_MNG_LE_ADD_WHITELIST_RESP                             (0x86)
#define TWIC_TCU_MNG_LE_DEL_WHITELIST_RESP                             (0x87)
#define TWIC_TCU_MNG_LE_START_ADVERTISE_RESP                           (0x88)
#define TWIC_TCU_MNG_LE_DISABLE_ADVERTISE_RESP                         (0x89)
#define TWIC_TCU_MNG_LE_SET_SCAN_ENABLE_RESP                           (0x8A)
#define TWIC_TCU_MNG_LE_SET_SCAN_DISABLE_RESP                          (0x8B)
#define TWIC_TCU_MNG_LE_CREATE_CONNECTION_CANCEL_RESP                  (0x8D)
#define TWIC_TCU_MNG_LE_SET_HOST_CHANNEL_CLASSIFICATION_RESP           (0x90)
#define TWIC_TCU_MNG_LE_READ_CHANNEL_MAP_RESP                          (0x91)
#define TWIC_TCU_MNG_LE_READ_SUPPORTED_STATES_RESP                     (0x92)
#define TWIC_TCU_MNG_LE_DISCONNECT_EVENT                               (0x93)
#define TWIC_TCU_MNG_LE_READ_TX_POW_LEVEL_RESP                         (0x94)
#define TWIC_TCU_MNG_LE_READ_RSSI_RESP                                 (0x95)
#define TWIC_TCU_MNG_LE_REM_CON_PARAM_ACCEPT_RESP                      (0x9E)
#define TWIC_TCU_MNG_LE_ADV_REPORT_EVENT                               (0xC1)
#define TWIC_TCU_LE_ACCEPT                                             (0xf1)
#define TWIC_TCU_MNG_LE_NOT_ACCEPT                                     (0xf2)
#define TWIC_TCU_MNG_LE_FATAL_ERROR                                    (0xfe)

#define TWIC_TCU_LE_SMP_I_PAIRING_EVENT                                (0x41)
#define TWIC_TCU_LE_SMP_I_SECURITY_EVENT                               (0xC2)
#define TWIC_TCU_LE_SMP_I_SECURITY_ACCEPT_RESP                         (0x82)
#define TWIC_TCU_LE_SMP_I_PAIRING_FAILED_EVENT                         (0x43)
#define TWIC_TCU_LE_SMP_I_KEY_ENTRY_REQ_EVENT                          (0x44)
#define TWIC_TCU_LE_SMP_I_KEY_ENTRY_WRITE_RESP                         (0x85)
#define TWIC_TCU_LE_SMP_I_DISPLAY_KEY_EVENT                            (0x46)
#define TWIC_TCU_LE_SMP_I_DISPLAY_KEY_WRITE_RESP                       (0x87)
#define TWIC_TCU_LE_SMP_I_STK_GENERATED_EVENT                          (0x48)
#define TWIC_TCU_LE_SMP_I_LTK_RECEIVED_EVENT                           (0xC9)
#define TWIC_TCU_LE_SMP_I_EDIV_RAND_RECEIVED_EVENT                     (0xCA)
#define TWIC_TCU_LE_SMP_I_STK_GEN_METHOD_EVENT                         (0xCB)
#define TWIC_TCU_LE_SMP_I_LTK_SENT_EVENT                               (0xCC)
#define TWIC_TCU_LE_SMP_I_EDIV_RAND_SENT_EVENT                         (0xCD)
#define TWIC_TCU_LE_SMP_I_ENCRYPTION_CHANGE_EVENT                      (0xCE)
#define TWIC_TCU_LE_SMP_I_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT        (0xCF)
#define TWIC_TCU_LE_SMP_I_PAIRING_COMPLETED_EVENT                      (0xD0)
#define TWIC_TCU_LE_SMP_I_PAIRING_FAILED_RESP                          (0x51)
#define TWIC_TCU_LE_SMP_I_IRK_SENT_EVENT                               (0xD1)
#define TWIC_TCU_LE_SMP_I_IDENTITY_ADDRESS_SENT_EVENT                  (0xD2)
#define TWIC_TCU_LE_SMP_I_CSRK_SENT_EVENT                              (0xD3)
#define TWIC_TCU_LE_SMP_I_IRK_RECEIVED_EVENT                           (0xD4)
#define TWIC_TCU_LE_SMP_I_IDENTITY_ADDRESS_RECEIVED_EVENT              (0xD5)
#define TWIC_TCU_LE_SMP_I_CSRK_RECEIVED_EVENT                          (0xD6)
#define TWIC_TCU_LE_SMP_I_BONDING_ENABLED_EVENT                        (0xD7)
#define TWIC_TCU_LE_SMP_I_OOB_KEY_ENTRY_REQ_EVENT                      (0x58)
#define TWIC_TCU_LE_SMP_I_OOB_KEY_ENTRY_WRITE_RESP                     (0x99)
#define TWIC_TCU_LE_SMP_I_STORE_KEY_EVENT                              (0xD8)
#define TWIC_TCU_LE_SMP_I_KEY_REQ_EVENT                                (0xD9)
#define TWIC_TCU_LE_SMP_I_KEY_ACCEPT_RESP                              (0x9B)
#define TWIC_TCU_LE_SMP_I_BONDING_ENABLED_INFO_EVENT                   (0x9A)

#define TWIC_TCU_LE_SMP_R_PAIRING_EVENT                                (0xC1)
#define TWIC_TCU_LE_SMP_R_PAIRING_ACCEPT_RESP_EVENT                    (0x81)
#define TWIC_TCU_LE_SMP_R_PAIRING_FAILED_EVENT                         (0x43)
#define TWIC_TCU_LE_SMP_R_KEY_ENTRY_REQ_EVENT                          (0x44)
#define TWIC_TCU_LE_SMP_R_KEY_ENTRY_WRITE_RESP                         (0x85)
#define TWIC_TCU_LE_SMP_R_DISPLAY_KEY_EVENT                            (0x46)
#define TWIC_TCU_LE_SMP_R_DISPLAY_KEY_WRITE_RESP                       (0x87)
#define TWIC_TCU_LE_SMP_R_STK_GENERATED_EVENT                          (0x48)
#define TWIC_TCU_LE_SMP_R_LTK_RECEIVED_EVENT                           (0xC9)
#define TWIC_TCU_LE_SMP_R_EDIV_RAND_RECEIVED_EVENT                     (0xCA)
#define TWIC_TCU_LE_SMP_R_STK_GEN_METHOD_EVENT                         (0xCB)
#define TWIC_TCU_LE_SMP_R_LTK_SENT_EVENT                               (0xCC)
#define TWIC_TCU_LE_SMP_R_EDIV_RAND_SENT_EVENT                         (0xCD)
#define TWIC_TCU_LE_SMP_R_STK_ENCRYPT_SESSION_REQ_REPLY_EVENT          (0xCE)
#define TWIC_TCU_LE_SMP_R_LTK_ENCRYPT_SESSION_REQ_REPLY_EVENT          (0xCF)
#define TWIC_TCU_LE_SMP_R_ENCRYPTION_CHANGE_EVENT                      (0xD0)
#define TWIC_TCU_LE_SMP_R_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT        (0xD1)
#define TWIC_TCU_LE_SMP_R_PAIRING_COMPLETED_EVENT                      (0xD2)
#define TWIC_TCU_LE_SMP_R_PAIRING_FAILED_RESP                          (0x53)
#define TWIC_TCU_LE_SMP_R_IRK_SENT_EVENT                               (0xD3)
#define TWIC_TCU_LE_SMP_R_IDENTITY_ADDRESS_SENT_EVENT                  (0xD4)
#define TWIC_TCU_LE_SMP_R_CSRK_SENT_EVENT                              (0xD5)
#define TWIC_TCU_LE_SMP_R_IRK_RECEIVED_EVENT                           (0xD6)
#define TWIC_TCU_LE_SMP_R_IDENTITY_ADDRESS_RECEIVED_EVENT              (0xD7)
#define TWIC_TCU_LE_SMP_R_CSRK_RECEIVED_EVENT                          (0xD8)
#define TWIC_TCU_LE_SMP_R_OOB_KEY_ENTRY_REQ_EVENT                      (0x59)
#define TWIC_TCU_LE_SMP_R_OOB_KEY_ENTRY_WRITE_RESP                     (0x9A)
#define TWIC_TCU_LE_SMP_R_STORE_KEY_EVENT                              (0xD9)
#define TWIC_TCU_LE_SMP_R_KEY_REQ_EVENT                                (0xDA)
#define TWIC_TCU_LE_SMP_R_KEY_ACCEPT_RESP                              (0x9C)
#define TWIC_TCU_LE_SMP_R_BONDING_ENABLED_INFO_EVENT                   (0x9B)

#define TWIC_TCU_LE_GATT_SDB_ADD_PRIM_SVC_RESP                         (0xA0)
#define TWIC_TCU_LE_GATT_SDB_ADD_SEC_SVC_RESP                          (0xA1)
#define TWIC_TCU_LE_GATT_SDB_ADD_CHAR_DECL_RESP                        (0xA2)
#define TWIC_TCU_LE_GATT_SDB_ADD_CHAR_ELE_RESP                         (0xA3)
#define TWIC_TCU_LE_GATT_SDB_ADD_INC_SVC_RESP                          (0xA4)
#define TWIC_TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP                         (0xA5)
#define TWIC_TCU_LE_GATT_SDB_RET_END_GRP_HLE_RESP                      (0xA6)
#define TWIC_TCU_LE_GATT_SDB_SET_ATTR_PERMS_RESP                       (0xA9)
#define TWIC_TCU_LE_GATT_SDB_GET_ATTR_PERMS_RESP                       (0xAA)
#define TWIC_TCU_LE_GATT_SDB_ADD_CHAR_VAR_ELE_RESP                     (0xAC)

#define TWIC_TCU_LE_GATT_SER_EXG_MTU_EVENT                             (0xc1)
#define TWIC_TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT                       (0xc2)
#define TWIC_TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT                      (0xc3)
#define TWIC_TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT                     (0xc4)
#define TWIC_TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT                      (0xc8)
#define TWIC_TCU_LE_GATT_SER_WRITE_WITHOUT_RESPONSE_EVENT              (0xc9)
#define TWIC_TCU_LE_GATT_SER_READ_MULTIPLE_EVENT                       (0xca)
#define TWIC_TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_EVENT               (0x45)
#define TWIC_TCU_LE_GATT_SER_CHAR_VAL_INDICATION_EVENT                 (0x46)
#define TWIC_TCU_LE_GATT_SER_EXECUTE_COMPLETE_EVENT                    (0x47)
#define TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_VAL_EVENT        (0xcb)
#define TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_DESP_EVENT       (0xcc)
#define TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_VAL_EVENT                  (0xcd)
#define TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_DESP_EVENT                 (0xce)
#define TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_VAL_EVENT              (0xcf)
#define TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_DESP_EVENT             (0xc7)

#define TWIC_TCU_LE_GATT_SER_INIT_RESP                                 (0x80)
#define TWIC_TCU_LE_GATT_SER_EXG_MTU_ACCEPT_RESP                       (0x81)
#define TWIC_TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_RESP                 (0x82)
#define TWIC_TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_RESP                (0x83)
#define TWIC_TCU_LE_GATT_SER_WRITE_CHAR_DESP_ACCEPT_RESP               (0x84)
#define TWIC_TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_RESP                (0x88)
#define TWIC_TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP                 (0x8a)
#define TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_VAL_ACCEPT_RESP            (0x8d)
#define TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_DESP_ACCEPT_RESP           (0x8e)
#define TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_VAL_ACCEPT_RESP  (0x8b)
#define TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_DESP_ACCEPT_RESP (0x8c)
#define TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_VAL_ACCEPT_RESP        (0x8f)
#define TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_DESP_ACCEPT_RESP       (0x87)

#define TWIC_TCU_LE_GATT_CLI_INIT_RESP                                 (0x80)
#define TWIC_TCU_LE_GATT_CLI_EXG_MTU_EVENT                             (0x41)
#define TWIC_TCU_LE_GATT_CLI_DISCOVER_PRIM_SVC_EVENT                   (0x42)
#define TWIC_TCU_LE_GATT_CLI_DISCOVER_PRIM_SVC_BY_UUID_EVENT           (0x43)
#define TWIC_TCU_LE_GATT_CLI_FIND_INCL_SVC_EVENT                       (0x44)
#define TWIC_TCU_LE_GATT_CLI_DISCOVER_CHAR_DECL_EVENT                  (0x45)
#define TWIC_TCU_LE_GATT_CLI_DISCOVER_CHAR_DECL_BY_UUID_EVENT          (0x46)
#define TWIC_TCU_LE_GATT_CLI_DISCOVER_CHAR_DESP_EVENT                  (0x47)
#define TWIC_TCU_LE_GATT_CLI_READ_CHAR_VAL_EVENT                       (0x48)
#define TWIC_TCU_LE_GATT_CLI_WRITE_CHAR_VAL_EVENT                      (0x49)
#define TWIC_TCU_LE_GATT_CLI_READ_CHAR_DESP_EVENT                      (0x4a)
#define TWIC_TCU_LE_GATT_CLI_WRITE_CHAR_DESP_EVENT                     (0x4b)
#define TWIC_TCU_LE_GATT_CLI_READ_MULTIPLE_CHAR_VALUES_EVENT           (0x4c)
#define TWIC_TCU_LE_GATT_CLI_READ_CHAR_VAL_UUID_EVENT                  (0x4d)
#define TWIC_TCU_LE_GATT_CLI_RELIABLE_WRITES_EVENT                     (0x4e)
#define TWIC_TCU_LE_GATT_CLI_WRITE_WITHOUT_RESPONSE_CMD_EVENT          (0x4f)
#define TWIC_TCU_LE_GATT_CLI_SIGNED_WRITE_WITHOUT_RESPONSE_CMD_EVENT   (0x51)
#define TWIC_TCU_LE_GATT_CLI_READ_LONG_CHAR_VAL_EVENT                  (0x52)
#define TWIC_TCU_LE_GATT_CLI_READ_LONG_CHAR_DESP_EVENT                 (0x53)
#define TWIC_TCU_LE_GATT_CLI_WRITE_LONG_CHAR_VAL_EVENT                 (0x54)
#define TWIC_TCU_LE_GATT_CLI_WRITE_LONG_CHAR_DESP_EVENT                (0x55)
#define TWIC_TCU_LE_GATT_CLI_INVALID_RESP                              (0x56)
#define TWIC_TCU_LE_GATT_CLI_CHAR_VAL_CONFIRMATION_ACCEPT_RESP         (0x90)
#define TWIC_TCU_LE_GATT_CLI_CHAR_VAL_NOTIFICATION_IND_EVENT           (0xc0)
#define TWIC_TCU_LE_GATT_CLI_CHAR_VAL_INDICATION_IND_EVENT             (0xd0)

#define TWIC_TCU_VEN_SET_MODULE_MAINTENANCE_EVENT                      (0x41)
#define TWIC_TCU_SYS_ACCEPT                                            (0xf1)
#define TWIC_TCU_SYS_NOT_ACCEPT                                        (0xf2)
#define TWIC_TCU_SYS_INVALID_COMMAND                                   (0xff)

#define TWIC_TCU_STATUS_CN_COMPLETE                                  (0xc400)
#define TWIC_TCU_STATUS_CN_ERROR                                     (0xc4ff)
#define TWIC_TCU_STATUS_DISCN_COMPLETE                               (0xd14c)


/*
 * RETURN CODE
 *
 */

typedef enum {
  TWIC_STATUS_OK = TZ1SM_HAL_STATUS_OK,
  TWIC_STATUS_EVENT_SIGNAL = TZ1SM_HAL_STATUS_EVENT_SIGNAL,
  TWIC_STATUS_EVENT_MESSAGE = TZ1SM_HAL_STATUS_EVENT_MESSAGE,
  TWIC_STATUS_EVENT_MAIL = TZ1SM_HAL_STATUS_EVENT_MAIL,
  TWIC_STATUS_EVENT_TIMEOUT = TZ1SM_HAL_STATUS_EVENT_TIMEOUT,
  TWIC_STATUS_ERROR_PARAMETER = TZ1SM_HAL_STATUS_ERROR_PARAMETER,
  TWIC_STATUS_ERROR_RESOURCE = TZ1SM_HAL_STATUS_ERROR_RESOURCE,
  TWIC_STATUS_ERROR_TIMEOUT_RESOURCE = TZ1SM_HAL_STATUS_ERROR_TIMEOUT_RESOURCE,
  TWIC_STATUS_ERROR_ISR = TZ1SM_HAL_STATUS_ERROR_ISR,
  TWIC_STATUS_ERROR_ISR_RECURSIVE = TZ1SM_HAL_STATUS_ERROR_ISR_RECURSIVE,
  TWIC_STATUS_ERROR_PRIORITY = TZ1SM_HAL_STATUS_ERROR_PRIORITY,
  TWIC_STATUS_ERROR_NOMEMORY = TZ1SM_HAL_STATUS_ERROR_NOMEMORY,
  TWIC_STATUS_ERROR_VALUE = TZ1SM_HAL_STATUS_ERROR_VALUE,
  TWIC_STATUS_ERROR_OS = TZ1SM_HAL_STATUS_ERROR_OS,
  TWIC_STATUS_SOURCE_STOPPED = TZ1SM_HAL_STATUS_SOURCE_STOPPED,
  TWIC_STATUS_SOURCE_PREPARING = TZ1SM_HAL_STATUS_SOURCE_PREPARING,
  TWIC_STATUS_SOURCE_RUNNING = TZ1SM_HAL_STATUS_SOURCE_RUNNING,
  TWIC_STATUS_ERROR_IO,
  TWIC_STATUS_ERROR_DATA,
  TWIC_STATUS_REQUEST_HANDLE,
  TWIC_STATUS_UNDER_PROCESSING,
  TWIC_STATUS_COMPLETION,
  TWIC_STATUS_OPERATION_NOT_PERMITTED,
  TWIC_STATUS_DISCONNECTED,
  TWIC_STATUS_COMMAND_ELIMINATION,
  TWIC_STATUS_WAITING_FOR_ACTIVATION,
  TWIC_STATUS_ERROR_HAL,
  TWIC_STATUS_ERROR_TZ1EM,
  TWIC_STATUS_RESERVED = TZ1SM_HAL_STATUS_RESERVED,
} twicStatus_t;


/*
 * GATT DB CREATION STEP
 *
 */

typedef enum {
  TWIC_BEGINSERVICECREATION,
  TWIC_ADDCHARACTERISTICS,
  TWIC_SETCHARACTERISTICS,
  TWIC_ENDSERVICECREATION
} twicGattDbStep_t;


/*
 * TIMER
 *
 */

typedef enum {
  TWIC_TMR_ONCE = TZ1SM_HAL_TIMER_ONCE,
  TWIC_TMR_PERIODIC = TZ1SM_HAL_TIMER_PERIODIC
} twicTimerBehavior_t;

#define TWIC_OK   (0x0000)
#define TWIC_BUSY (0x0001)


/*
 * Instances number for Client Applications
 *
 */

#define TWIC_INTERFACE_MAP_SIZE ((TWIC_INTERFACE_MAX-1)/TWIC_BITS_PER_LONG+1)
#define TWIC_BIT_MASK(nr) (1UL << ((nr) % TWIC_BITS_PER_LONG))
#define TWIC_BIT_WORD(nr) ((nr) / TWIC_BITS_PER_LONG)
#define TWIC_NR_IIDX(nr)  ((nr) + 1)
#define TWIC_IIDX_NR(iidx) ((iidx) - 1)


/*
 * Byte swap
 *
 */

#define TWIC_SETHARF_LE(p, x) do {     \
    (p)[1] = (uint8_t)((x) >>  8);     \
    (p)[0] = (uint8_t)((x)      );     \
} while(0)

#define TWIC_SETHARF_BE(p, x) do {     \
    (p)[0] = (uint8_t)((x) >>  8);     \
    (p)[1] = (uint8_t)((x)      );     \
} while(0)

#define TWIC_SETWORD_LE(p, x) do {     \
    (p)[3] = (uint8_t)((x) >> 24);     \
    (p)[2] = (uint8_t)((x) >> 16);     \
    (p)[1] = (uint8_t)((x) >>  8);     \
    (p)[0] = (uint8_t)((x)      );     \
} while(0)

#define TWIC_SETWORD_BE(p, x) do {     \
    (p)[0] = (uint8_t)((x) >> 24);     \
    (p)[1] = (uint8_t)((x) >> 16);     \
    (p)[2] = (uint8_t)((x) >>  8);     \
    (p)[3] = (uint8_t)((x)      );     \
} while(0)

/*
 * UUID
 *
 */

#define TWIC_UUID8   ( 1)
#define TWIC_UUID16  ( 2)
#define TWIC_UUID32  ( 4)
#define TWIC_UUID128 (16)

typedef TZ1K_PACKED_HDR struct {
  uint16_t len;
  TZ1K_PACKED_HDR union {
    uint8_t   id8[  TWIC_UUID8];
    uint8_t  id16[ TWIC_UUID16];
    uint8_t  id32[ TWIC_UUID32];
    uint8_t id128[TWIC_UUID128];
  } TZ1K_PACKED_FTR uu;
} TZ1K_PACKED_FTR twicUuid_t;

#define TWIC_UUID_ARGS(p_uuid) twicUuidLsb(p_uuid), twicUuidMsb(p_uuid)

/*
 * Bluetooth Address
 *
 */

typedef TZ1K_PACKED_HDR struct {
  uint8_t address[6];
} TZ1K_PACKED_FTR twicBdaddr_t;


/*
 * LE Security
 *
 */

typedef TZ1K_PACKED_HDR struct {
  uint8_t key[16]; /* Identity Resolving Key */
} TZ1K_PACKED_FTR twicIrk_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t value[16]; /* Diversifying Function Data */
} TZ1K_PACKED_FTR twicIr_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t key[16]; /* Short term key */
} TZ1K_PACKED_FTR twicStk_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t key[16]; /* Long term key */
} TZ1K_PACKED_FTR twicLtk_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t key[16]; /* Connection Signature Resolving Key */
} TZ1K_PACKED_FTR twicCsrk_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t value[2];
} TZ1K_PACKED_FTR twicEdiv_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t value[8];
} TZ1K_PACKED_FTR twicRand_t;

typedef enum {
  TWIC_SMP_SUCCESS,
  TWIC_PASSKEY_ENTRY_FAILED,
  TWIC_OOB_NOT_AVAILABLE,
  TWIC_AUTHENTICATION_REQUIREMENTS,
  TWIC_CONFIRM_VALUE_FAILED,
  TWIC_PAIRING_NOT_SUPPORTED,
  TWIC_ENCRYPTION_KEY_SIZE,
  TWIC_COMMAND_NOT_SUPPORTED,
  TWIC_UNSPECIFIED_REASON,
  TWIC_REPEATED_ATTEMPTS,
  TWIC_INVALID_PARAMETERS,
  TWIC_SMP_TIME_OUT = 0xC0,
  TWIC_SMP_TIME_OUT_NEED_RECONNECTION = 0xC1,
  TWIC_SMP_DEVICE_NOT_INITIALIZED = 0x81,
  TWIC_SMP_REQUEST_IN_PROGRESS = 0x82,
  TWIC_SMP_DISCONNECT_ERROR = 0x84,
  TWIC_SMP_INVALID_PARAMETER_LENGTH_ERROR = 0x86,
  TWIC_SMP_COMMAND_IN_PROGRESS = 0x92,
  TWIC_CSRK_NOT_AVAILABLE = 0x93,
  TWIC_DATA_SIGNING_ERROR = 0x94,
  TWIC_SIGNED_DATA_VERIFICATION_ERROR = 0x95,
  TWIC_SMP_NO_RESPONSE_FROM_REMOTE = 0xC2,
  TWIC_SMP_NO_RESPONSE_FROM_HOST = 0xC3,
  TWIC_HOST_REJECTED_PIN_KEY = 0xC4
} twicSmReasonCode_t;

typedef enum {
  TWIC_JUST_WORK_UNAUTHENTICATED = 0,
  TWIC_DISPLAY_RESPONDER_INPUT_AUTHENTICATED,
  TWIC_INPUT_RESPONDER_DISPLAY_AUTHENTICATED,
  TWIC_INPUT_RESPONDER_INPUT_AUTHENTICATED,
} twicStkGenMethod_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t chm[5];
} TZ1K_PACKED_FTR twicChannelMap_t;

/* 000,000 to 999,999. ex) 123456 is 0x0001E240 = {0x40, 0xE2, 0x01, 0x00} */
typedef TZ1K_PACKED_HDR struct {
  uint8_t key[4];
} TZ1K_PACKED_FTR twicPasskeyEntry_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t key[16];
} TZ1K_PACKED_FTR twicOobTk_t;

typedef struct {
  uint8_t remote_ltk_received : 1;
  uint8_t remote_irk_received : 1;
  uint8_t remote_csrk_received : 1;
  uint8_t local_ltk_sent : 1;
  uint8_t local_irk_sent : 1;
  uint8_t local_csrk_sent : 1;
  uint8_t bonding_enabled : 1;
  uint8_t mitm_enabled : 1;
} twicAuthInfo_t;

typedef enum {
  TWIC_SMP_IO_CAPABILITY_DISPLAY_ONLY = 0,
  TWIC_SMP_IO_CAPABILITY_DISPLAY_YES_NO,
  TWIC_SMP_IO_CAPABILITY_KEYBOARD_ONLY,
  TWIC_SMP_IO_CAPABILITY_NOINPUT_NO_OUTPUT,
  TWIC_SMP_IO_CAPABILITY_KEYBOARD_DISPLAY,
} twicSmpIoCapability_t;

#define TWIC_SMP_OOB_DATA_NOT_PRESENT (0)
#define TWIC_SMP_OOB_DATA_PRESENT     (1)

/*
 * Container of the transport packet.
 *
 */

#define TWIC_HCI_NOP                          (0x00)
#define TWIC_HCI_COMMAND_PACKET               (0x01)
#define TWIC_HCI_ACL_DATA_PACKET              (0x02)
#define TWIC_HCI_SYNCHRONOUS_DATA_PACKET      (0x03) /* No Support */
#define TWIC_HCI_EVENT_PACKET                 (0x04)

#define TWIC_SERVICE_ID_CONNECTION_MANAGEMENT (0xD1)
#define TWIC_SERVICE_ID_GATT_CLIENT           (0xD2)
#define TWIC_SERVICE_ID_GATT_SERVER           (0xD3)
#define TWIC_SERVICE_ID_SMP_INITIATOR         (0xD4)
#define TWIC_SERVICE_ID_SMP_RESPONDER         (0xD5)
#define TWIC_SERVICE_ID_VENDOR                (0xEF)
#define TWIC_SERVICE_ID_BASIC_MANAGEMENT      (0xE1)
#define TWIC_SERVICE_ID_NOP                   (TWIC_HCI_NOP)

/*
 * ATT MTU.
 *
 */

#define TWIC_ATT_MTU                      (23)

#if defined(TWIC_BLE_HWIP_V41)
/* Controller limitations (xOM6) */
#define TWIC_CONTROLLER_TXD_MAX           (188)
#define TWIC_TCU_VEN_INFORMATION_DATA_MAX (48)
#define TWIC_GATT_ATTR_MAX_LEN            (160)
#define TWIC_ATT_MTU_MAX                  (160)
#else
/* Controller limitations (xOM5) */
#define TWIC_CONTROLLER_TXD_MAX           (128)
#define TWIC_TCU_VEN_INFORMATION_DATA_MAX (48)
#define TWIC_GATT_ATTR_MAX_LEN            (100)
#define TWIC_ATT_MTU_MAX                  (72)
#endif

/*
 * Controller TX Protocol.
 *
 */

#define TWIC_TX_BUF_MAX (TWIC_CONTROLLER_TXD_MAX)

typedef TZ1K_PACKED_HDR struct {
  /* The actual specification of CM Packet length is 3bytes. It is
   * convenient that the length is restricted to uint16. */
  uint8_t len[2], reserved;
} TZ1K_PACKED_FTR twicHx_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t service_id, op_code;
  uint8_t len[2];
} TZ1K_PACKED_FTR twicCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t type;
  uint16_t op_code;
  uint8_t len;
} TZ1K_PACKED_FTR twicHm_t;

typedef TZ1K_PACKED_HDR struct {
  twicHx_t header;
  TZ1K_PACKED_HDR union {
    twicCm_t tcu;
    twicHm_t hci;
  } TZ1K_PACKED_FTR u;
  uint8_t param[TWIC_TX_BUF_MAX];
} TZ1K_PACKED_FTR twicCmContainer_t; /* TWIC_TX_BUF_MAX + 7 */


/*
 * Controller RX Protocol.
 *
 */

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status; /* 0x00,0x86,0x89 = Success, The abnormalities in
                   * a parameter, Initialization Already Done */
  twicBdaddr_t local;
} TZ1K_PACKED_FTR twicRespD181Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status; /* 0x00,0x86,0x81,0x82 = Success Parameter Error Dev
                     Not Initialized, Command in Progress, For other
                     errors (refer to HCI Command Error Code). */
  twicBdaddr_t le_features; /* CoreV4.0 [Vol 6] Part B, Section 4.6. */
} TZ1K_PACKED_FTR twicRespD182Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status; /* 0x00,0x86,0x81,0x82 = Success Parameter Error Dev
                     Not Initialized, Command in Progress, For other
                     errors (refer to HCI Command Error Code). */
  uint8_t white_list_size; /* Total number of white list entries that
                            *  can be stored in the Controller. */
} TZ1K_PACKED_FTR twicRespD185Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status; /* 0x00,0xF1 = Success, Initialization Already
                   * Done */
} TZ1K_PACKED_FTR twicSimpleRespCm_t;
/* D188 0x00,0x86,0x81,0x82,0xA4 = Success, Parameter Error, Dev Not
 * Initialized, Command in Progress, Device Already Advertising */
/* D189 0x00,0x86,0x81,0x82 = Success, Parameter Error, Dev Not
 * Initialized, Command in Progress */
/* D15B 0x00,0x01,0x03,0xF8 = Success, Parameter Error, Dev Not
 * Initialized, Command Unsupported */
/* D184 0x00,0x86,0x81,0x82 = Success, Parameter Error, Dev Not
 * Initialized, Command in Progress For other errors, refer to HCI
 * Command Error Code */

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status;               /* 0x00 = Success */
  uint16_t conn_handle;         /* 0x0000-0x0EFF = The Conn_handle is
                                 * used as an identifier for transmitting
                                 * and receiving data. */
  uint8_t role;                 /* 0x00,0x01,0x02-0xFF = Connection is master,
                                 * Connection is slave,
                                 * Reserved for future use. */
  uint8_t peer_address_type;    /* 0x00,0x01,0x02-0xFF = Public Device
                                 * Address, Random Device Address,
                                 * Reserved for future use. */
  twicBdaddr_t peer;            /* Public Device Address or Random Device
                                 * Address of the device to be connected. */
  uint16_t conn_interval;       /* N = 0x0006 to 0x0C80 = Time is N * 1.25
                                 * msec. Time Range: 7.5 msec to 4000
                                 * msec. */
  uint16_t slave_latency;       /* 0x0000 to 0x03E8 = Slave latency for the
                                 * connection in number of connection
                                 * events. */
  uint16_t supervision_timeout; /* Mandatory Range: 0x000A to 0x0C80
                                 * Time = N * 10 msec
                                 * Time Range: 100 msec to 32 seconds */
  uint8_t master_clock_accuracy;
} TZ1K_PACKED_FTR twicRespD14cCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status;               /* 0x00 = Success */
  uint16_t conn_handle;         /* Range 0x0000-0x0EFF (0x0F00 - 0x0FFF
                                   Reserved for future use). */
  uint16_t conn_interval;       /* Range: 0x0006 to 0x0C80 Time = N * 1.25
                                   msec Time Range: 7.5 msec to 4000 msec. */
  uint16_t conn_latency;        /* Range: 0x0006 to 0x0C80 Time = N * 1.25
                                   msec Time Range: 7.5 msec to 4000 msec. */
  uint16_t supervision_timeout; /* Mandatory Range: 0x000A to 0x0C80
                                 * Time = N * 10 msec
                                 * Time Range: 100 msec to 32 seconds */
} TZ1K_PACKED_FTR twicRespD14eCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;         /* Range 0x0000-0x0EFF (0x0F00 - 0x0FFF
                                   Reserved for future use). */
  uint16_t conn_interval_min;   /* Minimum value of the connection interval.
                                   Range: 0x0006 to 0x0C80
                                   Time = N * 1.25 ms
                                   Time Range: 7.5 msec to 4 seconds. */
  uint16_t conn_interval_max;   /* Maximum value of the connection interval.
                                   Range: 0x0006 to 0x0C80
                                   Time = N * 1.25 ms
                                   Time Range: 7.5 msec to 4 seconds. */
  uint16_t conn_latency;        /* Maximum allowed slave latency for the
                                   connection specified as the
                                   number of connection events.
                                   Range: 0x0000 to 0x01F3 (499). */
  uint16_t supervision_timeout; /* Supervision timeout for the connection.
                                   Range: 0x000A to 0x0C80
                                   Time = N * 10 ms
                                   Time Range: 100 ms to 32 seconds. */
} TZ1K_PACKED_FTR twicRespD15eCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status;            /* 0x00 = Success. For more details
                              * refer to HCI Command Error Code. */
  uint16_t conn_handle;
  uint8_t le_features[8];    /* CoreV4.0 [Vol 6] Part B, Section 4.6. */
} TZ1K_PACKED_FTR twicRespD14fCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status;
  uint16_t conn_handle;       /* Range 0x0000-0x0EFF (0x0F00 - 0x0FFF
                                 Reserved for future use). */
  uint8_t version;            /* Version of the Current LMP in the remote
                                 Controller. */
  uint16_t manufacturer_name; /* Manufacturer Name of the remote Controlle. */
  uint16_t sub_version;       /* Subversion of the LMP in the remote
                                 Controller. */
} TZ1K_PACKED_FTR twicRespD159Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status;            /* Success                          (0x00),
                                The abnormalities in a parameter (0x01),
                                Device Is Not -Initialized       (0x02),
                                Device Already Initialized       (0x03),
                                MNG Processing in progress       (0x04),
                                No ACL Link                      (0x05),
                                Device Role Slave                (0x06),
                                LE_Error                         (0x07),
                                Max_Device_Already_Connected     (0x08),
                                GATT Processing in Progress      (0x10) */
  uint8_t service_id;        /* 0xD1(Connection Management),
                                0xD2(GATT Client) = The Service Id of the
                                command which is being processed by
                                the module. */
  uint8_t op_code;           /* 0x01-0xEF = The OpCode of the command
                                which is being processed by the module. */
} TZ1K_PACKED_FTR twicRespD1F1Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t service_id;        /* 0xD1(Connection Management),
                                0xD2 (GATT Client),0xD3(GATT Server) */
  uint8_t op_code;           /* 0x01-0xEF = The OpCode of the command
                              * which is being processed by the
                              * module. */
} TZ1K_PACKED_FTR twicRespD1F2Cm_t;

typedef twicRespD1F2Cm_t twicRespE1FFCm_t;

#define TWIC_NUM_REPORTS_ROM5 (1)
#if defined(TWIC_CONFIG_ENABLE_SCAN)
#define TWIC_BYTES_REPORTS (                                      \
    ( 1 * TWIC_NUM_REPORTS_ROM5) + (1 * TWIC_NUM_REPORTS_ROM5) +  \
    ( 6 * TWIC_NUM_REPORTS_ROM5) + (1 * TWIC_NUM_REPORTS_ROM5) +  \
    (30 * TWIC_NUM_REPORTS_ROM5) + (1 * TWIC_NUM_REPORTS_ROM5))
typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t event_code;
  uint8_t data_len;
  uint8_t sub_event_code;
  uint8_t num_reports;
  uint8_t reports[TWIC_BYTES_REPORTS];
  /* for i in 1,...,num_reports (max 25);
     event_type[i]    25 byte (max),
     address_type[i]  25 byte (max),
     address[i]      150 byte (max 6 * 25byte),
     length_data[i]   25 byte (each max = 0x1F),
     data[i]         775 byte (length_data * num_reports),
     rssi[i]          25 byte (max),
     -------------- 1025 byte */
} TZ1K_PACKED_FTR twicRespD1C1Cm_t;
#endif

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;      /* Range 0x0000-0x0EFF (0x0F00 - 0x0FFF
                              * Reserved for future use). */
  uint8_t status;            /* 0x00,... = Success,... */
  uint8_t reason;            /* 0x00,... = Success,... */
} TZ1K_PACKED_FTR twicRespD193Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status;            /* 0x00,0x86,0x81,0x82,0xA1 =
                              * Success,Parameter Error, Dev Not
                              * Initialized,Command in Progress,No Acl
                              * Link */
  uint16_t conn_handle;      /* Range 0x0000-0x0EFF (0x0F00 - 0x0FFF
                              * Reserved for future use). */
  int8_t transmit_pow_Level; /* N = 0xXX Range: -30 <= N <= 20
                                ((signed integer) Units: dBm) */
} TZ1K_PACKED_FTR twicRespD194Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status;            /* 0x00,0x86,0x81,0x82,0xA1 =
                              * Success,Parameter Error, Dev Not
                              * Initialized,Command in Progress,No Acl
                              * Link */
  uint16_t conn_handle;      /* Range 0x0000-0x0EFF (0x0F00 - 0x0FFF
                              * Reserved for future use). */
  int8_t rssi_value;         /* N=0xXX Range: -127 to 20, 127 (signed
                              * integer(Units: dBm)) */
} TZ1K_PACKED_FTR twicRespD195Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status;            /* 0x00,0x86,0x81 = Success,Parameter
                              * Error, Dev Not Initialized */
  uint8_t bd_address[6];     /* Indicates the Address of the Remote
                              * Device connected. */
  uint8_t local_irk[16];     /* Identity Resolving Key of the Remote device. */
} TZ1K_PACKED_FTR twicRespD154Cm_t;

typedef twicRespD154Cm_t twicRespD155Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status;            /* 0x00,0x86,0x81 = Success, Parameter Error,
                                Dev Not Initialized
                                For more details refer to HCI Command Error
                                Code. */
  uint8_t encrypted_data[16];
} TZ1K_PACKED_FTR twicRespD15cCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t error; /* 0x01 = Memory Underflow. */
} TZ1K_PACKED_FTR twicRespD1FeCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status;
  uint16_t handle;
} TZ1K_PACKED_FTR twicRespAxCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status;
} TZ1K_PACKED_FTR twicRespA5Cm_t;
typedef twicRespA5Cm_t twicRespA9Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status;
  uint16_t attribute_permissions;
} TZ1K_PACKED_FTR twicRespAaCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint16_t client_rx_mtu_size;
} TZ1K_PACKED_FTR twicRespC1Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint16_t handle;
} TZ1K_PACKED_FTR twicRespCxCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint16_t char_value_handles[(TWIC_ATT_MTU_MAX / 2)];
} TZ1K_PACKED_FTR twicRespCaCm_t;

#if defined(TWIC_API_GATTSERVERLONGCHARVALREADOUTRESPONSE) ||       \
  defined(TWIC_API_GATTSERVERLONGCHARDESPREADOUTRESPONSE) ||        \
  defined(TWIC_API_GATTSERVERLONGCHARVALPREPAREWRITEINRESPONSE) ||  \
  defined(TWIC_API_GATTSERVERLONGCHARDESPPREPAREWRITEINRESPONSE)
typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint16_t handle;
  uint16_t offset;
} TZ1K_PACKED_FTR twicRespClCm_t;
#endif

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t acceptor;
  uint8_t information_id;
  uint8_t status;
  uint8_t information_type;
  uint8_t information_data[TWIC_TCU_VEN_INFORMATION_DATA_MAX];
} TZ1K_PACKED_FTR twicResp41Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
} TZ1K_PACKED_FTR twicResp45Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t status;
  uint16_t negotiated_mtu_size;
} TZ1K_PACKED_FTR twicResp81Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t status;
} TZ1K_PACKED_FTR twicResp8xCm_t;

typedef twicResp8xCm_t twicResp46Cm_t;
typedef twicResp8xCm_t twicResp47Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status; /* 0x00,0x86,0x81,0x82,0xA1 = Success,Parameter
                   * Error,Dev Not Initialized,Command in Progress, No
                   * Acl Link, For more details refer to HCI Command
                   * Error Code. */
  uint16_t conn_handle;
  uint8_t channel_map[5]; /* This parameter contains 37 1-bit
                           * fields. The nth such field (in the range
                           * 0 to 36) contains the value for the link
                           * layer channel index n. Channel n is bad =
                           * 0. Channel n is unknown = 1. The most
                           * significant bits are reserved and shall
                           * be set to 0. At least one channel shall
                           * be marked as unknown. */
} TZ1K_PACKED_FTR twicRespD191Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint8_t status; /* 0x00,0x86,0x81,0x82,0xA1 = Success,Parameter
                   * Error,Dev Not Initialized,Command in Progress, No
                   * Acl Link, For more details refer to HCI Command
                   * Error Code. */
  uint16_t conn_handle;
  uint8_t states[8];
} TZ1K_PACKED_FTR twicRespD192Cm_t;

typedef twicResp8xCm_t twicRespD150Cm_t;
typedef twicRespAxCm_t twicRespD151Cm_t;
typedef twicRespD15eCm_t twicRespD152Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint16_t handle;
  uint8_t value[TWIC_CONTROLLER_TXD_MAX - 4];
} TZ1K_PACKED_FTR twicRespCyCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t status;
  uint8_t io_capability;
  uint8_t oob_data_flag;
  uint8_t auth_req;
  uint8_t max_enc_key_size;
  uint8_t init_key_dist;
  uint8_t resp_key_dist;
} TZ1K_PACKED_FTR twicRespD441Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t reason;
} TZ1K_PACKED_FTR twicRespD443Cm_t;

typedef twicResp45Cm_t twicRespD444Cm_t;
typedef twicResp45Cm_t twicRespD445Cm_t;
typedef twicResp45Cm_t twicRespD446Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t key[16]; /* Short term key, Long term key,
                    * IRK(Identity Resolving Key),
                    * CSRK(Connection Signature Resolving Key) */
} TZ1K_PACKED_FTR twicRespD448Cm_t;

typedef twicRespD443Cm_t twicRespD451Cm_t;
typedef twicResp45Cm_t twicRespD458Cm_t;
typedef twicRespD443Cm_t twicRespD485Cm_t;
typedef twicRespD443Cm_t twicRespD487Cm_t;
typedef twicRespD443Cm_t twicRespD499Cm_t;
typedef twicRespD443Cm_t twicRespD49aCm_t;
typedef twicRespD443Cm_t twicRespD49bCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t auth_req;
} TZ1K_PACKED_FTR twicRespD4C2Cm_t;

typedef twicRespD448Cm_t twicRespD4C9Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t ediv[2];
  uint8_t rand[8];
} TZ1K_PACKED_FTR twicRespD4CaCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t status; /* 0x00,0x01 = Success, Key Entry Failed */
  uint8_t method; /* STK Generation Method (twicStkGenMethod_t) */
} TZ1K_PACKED_FTR twicRespD4CbCm_t;
typedef twicRespD448Cm_t twicRespD4CcCm_t;
typedef twicRespD4CaCm_t twicRespD4CdCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t reason;
  uint8_t key_type;
  uint8_t encryption_flag;
  uint8_t encryption_key_size;
} TZ1K_PACKED_FTR twicRespD4CeCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t reason;
  uint8_t key_type;
  uint8_t encryption_key_size;
} TZ1K_PACKED_FTR twicRespD4CfCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t status;
  uint8_t authentication_information; /* authentication_information */
} TZ1K_PACKED_FTR twicRespD4D0Cm_t;

typedef twicRespD448Cm_t twicRespD4D1Cm_t;
typedef twicRespD448Cm_t twicRespD4D4Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
/* TWIC_ADDR_TYPE_PUBLIC_DEVICE_ADDRESS (0)
 * TWIC_ADDR_TYPE_RANDOM_DEVICE_ADDRESS (1) */
  uint8_t address_type;
  uint8_t device_address[6];
} TZ1K_PACKED_FTR twicRespD4D2Cm_t;
typedef twicRespD4D2Cm_t twicRespD4D5Cm_t;

typedef twicRespD448Cm_t twicRespD4D3Cm_t;
typedef twicRespD448Cm_t twicRespD4D6Cm_t;
typedef twicRespD4C2Cm_t twicRespD4D7Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
/* TWIC_ADDR_TYPE_PUBLIC_DEVICE_ADDRESS (0)
 * TWIC_ADDR_TYPE_RANDOM_DEVICE_ADDRESS (1) */
  uint8_t address_type;
  uint8_t bd_address[6];
  uint8_t status;
} TZ1K_PACKED_FTR twicRespD4D8Cm_t;

typedef twicRespD4D8Cm_t twicRespD4D9Cm_t;

typedef twicRespD443Cm_t twicRespD543Cm_t;
typedef twicResp45Cm_t   twicRespD544Cm_t;
typedef twicResp45Cm_t   twicRespD546Cm_t;
typedef twicRespD448Cm_t twicRespD548Cm_t;
typedef twicRespD443Cm_t twicRespD553Cm_t;
typedef twicResp45Cm_t   twicRespD559Cm_t;
typedef twicRespD443Cm_t twicRespD585Cm_t;
typedef twicRespD443Cm_t twicRespD587Cm_t;
typedef twicRespD443Cm_t twicRespD59aCm_t;
typedef twicRespD443Cm_t twicRespD59bCm_t;
typedef twicRespD443Cm_t twicRespD59cCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t Parameter_Length;
  uint16_t conn_handle;
  uint8_t io_capability;
  uint8_t oob_data_flag;
  uint8_t auth_req;
  uint8_t max_enc_key_size;
  uint8_t init_key_dist;
  uint8_t resp_key_dist;
} TZ1K_PACKED_FTR twicRespD5C1Cm_t;

typedef twicRespD448Cm_t twicRespD5C9Cm_t;
typedef twicRespD4CaCm_t twicRespD5CaCm_t;
typedef twicRespD4CbCm_t twicRespD5CbCm_t;
typedef twicRespD448Cm_t twicRespD5CcCm_t;
typedef twicRespD4CaCm_t twicRespD5CdCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t status;
  uint8_t key[16]; /* Short term key, Long term key */
} TZ1K_PACKED_FTR twicRespD5CeCm_t;
typedef twicRespD5CeCm_t twicRespD5CfCm_t;

typedef twicRespD4CeCm_t twicRespD5D0Cm_t;
typedef twicRespD4CfCm_t twicRespD5D1Cm_t;
typedef twicRespD4D0Cm_t twicRespD5D2Cm_t;
typedef twicRespD448Cm_t twicRespD5D3Cm_t;
typedef twicRespD4D2Cm_t twicRespD5D4Cm_t;
typedef twicRespD448Cm_t twicRespD5D5Cm_t;
typedef twicRespD448Cm_t twicRespD5D6Cm_t;
typedef twicRespD4D2Cm_t twicRespD5D7Cm_t;
typedef twicRespD448Cm_t twicRespD5D8Cm_t;
typedef twicRespD4D8Cm_t twicRespD5D9Cm_t;
typedef twicRespD4D8Cm_t twicRespD5DaCm_t;

#if defined(TWIC_API_GATTCLIENTEXGMTU)
typedef twicResp81Cm_t twicRespD241Cm_t;
#endif

#define TWIC_RESP_D242_SVC_SIZE (TWIC_CONTROLLER_TXD_MAX - 4)
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICE) ||             \
  defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID) ||  \
  defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE) ||                  \
  defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS) ||           \
  defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID) ||        \
  defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS) ||               \
  defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE) ||              \
  defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR) ||         \
  defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID) ||          \
  defined(TWIC_API_GATTCLIENTREADMULTIPLECHARVALUES) ||               \
  defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE) ||          \
  defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR)
typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  uint8_t status;
  uint8_t continue_flag;
  TZ1K_PACKED_HDR union {
    uint8_t attr[TWIC_RESP_D242_SVC_SIZE];
    uint16_t error_handle;
  } TZ1K_PACKED_FTR u;
} TZ1K_PACKED_FTR twicRespD242Cm_t;
#endif

#define TWIC_RESP_D24E_TOTAL_HANDLES (54/3)
#if defined(TWIC_API_GATTCLIENTRELIABLEWRITE)
typedef TZ1K_PACKED_HDR struct {
  uint16_t parameter_length;
  uint16_t conn_handle;
  struct {
    uint8_t status;
    uint16_t handle;
  } TZ1K_PACKED_FTR resp[TWIC_RESP_D24E_TOTAL_HANDLES];
} TZ1K_PACKED_FTR twicRespD24eCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint16_t handle;
  uint16_t offset;
  uint16_t length;
  uint8_t *value;
} TZ1K_PACKED_FTR twicReliableWritein_t;
#endif

#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID)
typedef twicRespD242Cm_t twicRespD243Cm_t;
#endif
#if defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE)
typedef twicRespD242Cm_t twicRespD244Cm_t;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS)
typedef twicRespD242Cm_t twicRespD245Cm_t;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)
typedef twicRespD242Cm_t twicRespD246Cm_t;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS)
typedef twicRespD242Cm_t twicRespD247Cm_t;
#endif
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE)
typedef twicRespD242Cm_t twicRespD248Cm_t;
#endif
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR)
typedef twicRespD242Cm_t twicRespD24aCm_t;
#endif
#if defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID)
typedef twicRespD242Cm_t twicRespD24dCm_t;
#endif
#if defined(TWIC_API_GATTCLIENTREADMULTIPLECHARVALUES)
typedef twicRespD242Cm_t twicRespD24cCm_t;
#endif
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE)
typedef twicRespD242Cm_t twicRespD252Cm_t;
#endif
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR)
typedef twicRespD242Cm_t twicRespD253Cm_t;
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICVALUE)
typedef twicResp8xCm_t   twicRespD249Cm_t;
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR)
typedef twicResp8xCm_t   twicRespD24bCm_t;
#endif
#if defined(TWIC_API_GATTCLIENTWRITEWITHOUTRESPONSE)
typedef twicResp8xCm_t   twicRespD24fCm_t;
#endif
#if defined(TWIC_API_GATTCLIENTSIGNEDWRITEWITHOUTRESPONSE)
typedef twicResp8xCm_t   twicRespD251Cm_t;
#endif
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICVALUE)
typedef twicResp8xCm_t   twicRespD254Cm_t;
#endif
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR)
typedef twicResp8xCm_t   twicRespD255Cm_t;
#endif
typedef twicResp8xCm_t   twicRespD256Cm_t;
#if defined(TWIC_API_GATTCLIENTINDICATIONCONFIRMATIONRESPONSE)
typedef twicResp8xCm_t   twicRespD290Cm_t;
#endif
typedef twicRespCyCm_t   twicRespD2C0Cm_t;
typedef twicRespCyCm_t   twicRespD2D0Cm_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t op_code;
  TZ1K_PACKED_HDR union {
    twicRespD14cCm_t op_d14c;
    twicRespD14eCm_t op_d14e;
    twicRespD14fCm_t op_d14f;
    twicRespD150Cm_t op_d150;
    twicRespD151Cm_t op_d151;
    twicRespD152Cm_t op_d152;
    twicRespD154Cm_t op_d154;
    twicRespD155Cm_t op_d155;
    twicRespD159Cm_t op_d159;
    twicRespD15cCm_t op_d15c;
#if defined(TWIC_BLE_HWIP_V41)
    twicRespD15eCm_t op_d15e;
#endif
    twicRespD181Cm_t op_d181;
    twicRespD182Cm_t op_d182;
    twicRespD185Cm_t op_d185;
    twicSimpleRespCm_t op_simple;
    twicRespD191Cm_t op_d191;
    twicRespD192Cm_t op_d192;
    twicRespD193Cm_t op_d193;
    twicRespD194Cm_t op_d194;
    twicRespD195Cm_t op_d195;
#if defined(TWIC_CONFIG_ENABLE_SCAN)
    twicRespD1C1Cm_t op_d1c1;
#endif
    twicRespD1F1Cm_t op_d1f1;
    twicRespD1F2Cm_t op_d1f2;
    twicRespD1FeCm_t op_d1fe;
#if defined(TWIC_API_GATTCLIENTEXGMTU)
    twicRespD241Cm_t op_d241;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICE) ||             \
  defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID) ||  \
  defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE) ||                  \
  defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS) ||           \
  defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID) ||        \
  defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS) ||               \
  defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE) ||              \
  defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR) ||         \
  defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID) ||          \
  defined(TWIC_API_GATTCLIENTREADMULTIPLECHARVALUES) ||               \
  defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE) ||          \
  defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR)
    twicRespD242Cm_t op_d242;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID)
    twicRespD243Cm_t op_d243;
#endif
#if defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE)
    twicRespD244Cm_t op_d244;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS)
    twicRespD245Cm_t op_d245;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)
    twicRespD246Cm_t op_d246;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS)
    twicRespD247Cm_t op_d247;
#endif
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE)
    twicRespD248Cm_t op_d248;
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICVALUE)
    twicRespD249Cm_t op_d249;
#endif
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR)
    twicRespD24aCm_t op_d24a;
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR)
    twicRespD24bCm_t op_d24b;
#endif
#if defined(TWIC_API_GATTCLIENTREADMULTIPLECHARVALUES)
    twicRespD24cCm_t op_d24c;
#endif
#if defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID)
    twicRespD24dCm_t op_d24d;
#endif
#if defined(TWIC_API_GATTCLIENTRELIABLEWRITE)
    twicRespD24eCm_t op_d24e;
#endif
#if defined(TWIC_API_GATTCLIENTWRITEWITHOUTRESPONSE)
    twicRespD24fCm_t op_d24f;
#endif
#if defined(TWIC_API_GATTCLIENTSIGNEDWRITEWITHOUTRESPONSE)
    twicRespD251Cm_t op_d251;
#endif
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE)
    twicRespD252Cm_t op_d252;
#endif
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR)
    twicRespD253Cm_t op_d253;
#endif
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICVALUE)
    twicRespD254Cm_t op_d254;
#endif
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR)
    twicRespD255Cm_t op_d255;
#endif
    twicRespD256Cm_t op_d256;
#if defined(TWIC_API_GATTCLIENTINDICATIONCONFIRMATIONRESPONSE)
    twicRespD290Cm_t op_d290;
#endif
    twicRespD2C0Cm_t op_d2c0;
    twicRespD2D0Cm_t op_d2d0;
    twicRespD441Cm_t op_d441;
    twicRespD443Cm_t op_d443;
    twicRespD444Cm_t op_d444;
    twicRespD445Cm_t op_d445;
    twicRespD446Cm_t op_d446;
    twicRespD448Cm_t op_d448;
    twicRespD451Cm_t op_d451;
    twicRespD458Cm_t op_d458;
    twicRespD485Cm_t op_d485;
    twicRespD487Cm_t op_d487;
    twicRespD499Cm_t op_d499;
    twicRespD49aCm_t op_d49a;
    twicRespD49bCm_t op_d49b;
    twicRespD4C2Cm_t op_d4c2;
    twicRespD4C9Cm_t op_d4c9;
    twicRespD4CaCm_t op_d4ca;
    twicRespD4CbCm_t op_d4cb;
    twicRespD4CcCm_t op_d4cc;
    twicRespD4CdCm_t op_d4cd;
    twicRespD4CeCm_t op_d4ce;
    twicRespD4CfCm_t op_d4cf;
    twicRespD4D0Cm_t op_d4d0;
    twicRespD4D1Cm_t op_d4d1;
    twicRespD4D2Cm_t op_d4d2;
    twicRespD4D3Cm_t op_d4d3;
    twicRespD4D4Cm_t op_d4d4;
    twicRespD4D5Cm_t op_d4d5;
    twicRespD4D6Cm_t op_d4d6;
    twicRespD4D7Cm_t op_d4d7;
    twicRespD4D8Cm_t op_d4d8;
    twicRespD4D9Cm_t op_d4d9;
    twicRespD543Cm_t op_d543;
    twicRespD544Cm_t op_d544;
    twicRespD546Cm_t op_d546;
    twicRespD548Cm_t op_d548;
    twicRespD553Cm_t op_d553;
    twicRespD559Cm_t op_d559;
    twicRespD585Cm_t op_d585;
    twicRespD587Cm_t op_d587;
    twicRespD59aCm_t op_d59a;
    twicRespD59bCm_t op_d59b;
    twicRespD59cCm_t op_d59c;
    twicRespD5CaCm_t op_d5ca;
    twicRespD5CbCm_t op_d5cb;
    twicRespD5C1Cm_t op_d5c1;
    twicRespD5C9Cm_t op_d5c9;
    twicRespD5CcCm_t op_d5cc;
    twicRespD5CdCm_t op_d5cd;
    twicRespD5CeCm_t op_d5ce;
    twicRespD5CfCm_t op_d5cf;
    twicRespD5D0Cm_t op_d5d0;
    twicRespD5D1Cm_t op_d5d1;
    twicRespD5D2Cm_t op_d5d2;
    twicRespD5D3Cm_t op_d5d3;
    twicRespD5D4Cm_t op_d5d4;
    twicRespD5D5Cm_t op_d5d5;
    twicRespD5D6Cm_t op_d5d6;
    twicRespD5D7Cm_t op_d5d7;
    twicRespD5D8Cm_t op_d5d8;
    twicRespD5D9Cm_t op_d5d9;
    twicRespD5DaCm_t op_d5da;
    twicRespE1FFCm_t op_e1ff;
    twicResp41Cm_t op_41;
    twicResp45Cm_t op_45;
    twicResp47Cm_t op_46;
    twicResp47Cm_t op_47;
    twicResp81Cm_t op_81;
    twicResp8xCm_t op_8x;
    twicRespAxCm_t op_ax;
    twicRespA5Cm_t op_a5;
    twicRespA9Cm_t op_a9;
    twicRespAaCm_t op_aa;
    twicRespC1Cm_t op_c1;
    twicRespCxCm_t op_cx;
    twicRespCaCm_t op_ca;
#if defined(TWIC_API_GATTSERVERLONGCHARVALREADOUTRESPONSE) ||       \
  defined(TWIC_API_GATTSERVERLONGCHARDESPREADOUTRESPONSE) ||        \
  defined(TWIC_API_GATTSERVERLONGCHARVALPREPAREWRITEINRESPONSE) ||  \
  defined(TWIC_API_GATTSERVERLONGCHARDESPPREPAREWRITEINRESPONSE)
    twicRespClCm_t op_cl;
#endif
    twicRespCyCm_t op_cy;
  } TZ1K_PACKED_FTR u;
} TZ1K_PACKED_FTR twicRespCm_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t pidx, event;
  uint16_t bytes;
} TZ1K_PACKED_FTR twicPipeCont_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t event_code, parameter_lenght;
  uint8_t value[sizeof(twicRespCm_t) - 2]; /* use the same size anyway */
} TZ1K_PACKED_FTR twicRespUnformattedEventHc_t;

typedef TZ1K_PACKED_HDR union {
  twicRespUnformattedEventHc_t u_event;
} TZ1K_PACKED_FTR twicRespHc_t;

typedef TZ1K_PACKED_HDR union {
  twicRespCm_t tcu;
  twicRespHc_t hci;
} TZ1K_PACKED_FTR twicResp_t;

#define TWIC_ALIGN4(x) (((x)+0x3) & ~0x3)
#define TWIC_RX_BUF_MAX TWIC_ALIGN4(sizeof(twicResp_t)) /* < 65535 */

#define TWIC_HCI                             (0x0)
#define TWIC_TCU                             (0x1)

#define TWIC_RRQ_IN_PACKET_IDLE              (0x0)
#define TWIC_RRQ_IN_PACKET_HC_START          (0x1)
#define TWIC_RRQ_IN_PACKET_UNKNOWN           (0x2)
#define TWIC_RRQ_IN_PACKET_COMPLETION        (0x3)
#define TWIC_RRQ_IN_PACKET_OVERFLOWED        (0x4)

#define TWIC_SRQ_BUF_SIZE                    (TWIC_RX_BUF_MAX) /* < 65535 */
#define TWIC_SRQ_PIPE_SIZE                   (0x4)
/* +560 bytes in comparison with 0x3 */
#define TWIC_SRQ_Q_SIZE   (TWIC_SRQ_PIPE_SIZE + 1)

#define TWIC_SRQ_HC_POS_INDICATOR              (1)
#define TWIC_SRQ_HC_LPOS_CMD                   (4)
#define TWIC_SRQ_HC_LPOS_H_ACL                 (5)
#define TWIC_SRQ_HC_LPOS_SCO                   (4)
#define TWIC_SRQ_HC_LPOS_EVENT                 (3)

#define TWIC_SRQ_HC_LSIZE_CMD                  (1)
#define TWIC_SRQ_HC_LSIZE_ACL                  (2)
#define TWIC_SRQ_HC_LSIZE_SCO                  (1)
#define TWIC_SRQ_HC_LSIZE_EVENT                (1)

#define TWIC_SRQ_INDICATOR_HC_UNKNOWN       (0x00)
#define TWIC_SRQ_INDICATOR_HC_CMD           (0x01)
#define TWIC_SRQ_INDICATOR_HC_ACL           (0x02)
#define TWIC_SRQ_INDICATOR_HC_SCO           (0x03)
#define TWIC_SRQ_INDICATOR_HC_EVENT         (0x04)

#define TWIC_SRQ_EVENT_OPC_HC_HWERR         (0x10)
#define TWIC_SRQ_EVENT_OPC_HC_NOP           (0x0e)

#define RPQUEUE_DISABLE                    (false)
#define RPQUEUE_ENABLE                      (true)

#define RPQUEUE_PM_PIN        (0xFF)
#define RPQUEUE_PM_UNKNOWN    (0x00)
#define RPQUEUE_PM_SLEEP      (0x01)
#define RPQUEUE_PM_BACKUP1    (0x04)
#define RPQUEUE_PM_BACKUP2    (0x08)
#define RPQUEUE_PM_DEEP_SLEEP (0x10)
#define RPQUEUE_PM_HDU        (0xFF) /* HOST DELAY is undefined */
#define RPQUEUE_PM_HDO        (0x05) /* HOST DELAY offset */
#define RPQUEUE_PM_HDD        (0x0A) /* HOST DELAY for DOZE */

#define RPQUEUE_H4_HAL_OK                   (0x00)
#define RPQUEUE_H4_HAL_ERROR                (0x10)
#define RPQUEUE_H4_M2MSG_OK                 (0x00)
#define RPQUEUE_H4_M2MSG_UNKNOWN_DATA_TYPE  (0x02)
#define RPQUEUE_H4_M2MSG_INVALID_DATA_VALUE (0x04)

#define RPQUEUE_LPRTO (0xF0)

#define TWIC_TZBT_PUNCTUATION_MIN (0x0003)
#define TWIC_TZBT_PUNCTUATION     (0x000C)
#define TWIC_TZBT_PUNCTUATION_MAX (0x1FFF) /* 8191 us */

typedef enum {
  TWIC_TZBT_0768 = 0x1A, /* 0.160 */
  TWIC_TZBT_1152 = 0x0F, /* 0.309 */
  TWIC_TZBT_1536 = 0x0D, /* 0.160 */
  TWIC_TZBT_2304 = 0x07, /* 0.756 */
  TWIC_TZBT_3072 = 0x05, /* Higher VOLTAGE MODE than B. -0.429 */
  TWIC_TZBT_4608 = 0x04, /* Deviation 0.756 */
  TWIC_TZBT_9216 = 0x02, /* 0.756 */
} twicTzbtBr_t;

typedef TZ1K_PACKED_HDR struct {
  twicPipeCont_t buf[TWIC_SRQ_Q_SIZE];
  uint8_t top, pot, pidx, type;
  TZ1K_PACKED_HDR struct {
    uint8_t buf[TWIC_ALIGN4(TWIC_SRQ_BUF_SIZE)];
    uint32_t bytes;
  } TZ1K_PACKED_FTR pipe[TWIC_SRQ_PIPE_SIZE];
  uint32_t size, br, br_rq;
  uint8_t pos, lowpower_selector, lowpower_mode, slpto;
  uint8_t host_delay, hdr, h4;
  TZ1K_PACKED_HDR struct {
    uint8_t enable : 1;
    uint8_t ioinit : 1;
    uint8_t lowpower_clock : 1;
    uint8_t waiting_for_activation : 1;
    uint8_t lowpower_requirements : 1;
    uint8_t lowpower_enable : 1;
    uint8_t fc : 1;
    uint8_t fc_rq : 1;
//#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
    uint8_t lowpower_peripheral_h4 : 1;
    uint8_t lowpower_core : 1;
    uint8_t lowpower_core_idle : 1;
    uint8_t lowpower_core_cosc : 1;
    uint8_t lowpower_core_adve : 1;
    uint8_t lowpower_core_ncad : 1;
    uint8_t pin : 1;
    uint8_t pin_rq : 1;
//#endif
  } TZ1K_PACKED_FTR sniff;
} TZ1K_PACKED_FTR TZ1K_ALIGNED((4)) twicRxPipeQueue_t;


/* connection */
typedef void (twicCbConnComplete_t)(
  const void * const connif, const bool central, const void *const arg);
typedef void (twicCbConnUpdate_t)(
  const void * const connif, const bool central, const void *const arg);
#if defined(TWIC_BLE_HWIP_V41)
typedef void (twicCbConnParameter_t)(
  const void * const connif, const bool central, const void *const arg);
#endif
typedef void (twicCbDisconn_t)(
  const void * const connif, const bool central, const void *const arg);
typedef void (twicCbExgMtuDemand_t)(
  const void * const connif, const uint16_t client_rx_mtu_size);
typedef void (twicCbExgMtuResult_t)(
  const void * const connif, const void *const arg);
typedef void (twicCbNotificationSent_t)(
  const void * const connif);
typedef void (twicCbIndicationConfirmation_t)(
  const void * const connif, const uint8_t status);
typedef void (twicCbExecWriteReqComplete_t)(
  const void * const connif, const uint8_t status);
typedef void (twicCbReadResultRssi_t)(
  const void * const connif, const uint8_t rssi);
typedef void (twicCbReadResultTxPower_t)(
  const void * const connif, const uint8_t tx_power);
/* descriptor */
typedef void (twicCbCharDespWriteInDemand_t)(
  const void * const connif, const void *const arg, const uint8_t eidx);
typedef void (twicCbCharDespReadOutDemand_t)(
  const void * const connif, const uint8_t eidx);
typedef void (twicCbLongCharDespReadOutDemand_t)(
  const void * const connif, const uint16_t offset, const uint8_t eidx);
typedef void (twicCbLongCharDespPrepWriteDemand_t)(
  const void * const connif, const uint16_t offset, const uint8_t eidx);
typedef void (twicCbCharDespExecWriteInDemand_t)(
  const void * const connif, const void *const arg, const uint8_t eidx);
/* characteristics */
typedef void (twicCbCharValReadOutDemand_t)(
  const void * const connif, const uint8_t eidx);
typedef void (twicCbCharValMultiReadOutDemand_t)(
  const void * const connif, const void *const op_ca);
typedef void (twicCbCharValWriteInPost_t)(
  const void * const connif, const void *const arg, const uint8_t eidx);
typedef void (twicCbCharValWriteInDemand_t)(
  const void * const connif, const void *const arg, const uint8_t eidx);
typedef void (twicCbLongCharValReadOutDemand_t)(
  const void * const connif, const uint16_t offset, const uint8_t eidx);
typedef void (twicCbLongCharValPrepWriteDemand_t)(
  const void * const connif, const uint16_t offset, const uint8_t eidx);
typedef void (twicCbCharValExecWriteInDemand_t)(
  const void * const connif, const void *const arg, const uint8_t eidx);
/* controller interruption */
typedef void (twicCbDownstreamRequest_t)(void);
typedef void (twicCbWakeupRequest_t)(void);

typedef struct {
/* connection */
  twicCbExgMtuDemand_t *mtu_exchange_demand;
  twicCbExgMtuResult_t *mtu_exchange_result;
  twicCbNotificationSent_t *notification_sent;
  twicCbIndicationConfirmation_t *indication_confirmation;
  twicCbExecWriteReqComplete_t *queued_writes_complete;
/* descriptor */
  twicCbCharDespWriteInDemand_t *char_desp_writein_demand;
  twicCbCharDespReadOutDemand_t *char_desp_readout_demand;
  twicCbLongCharDespReadOutDemand_t *long_char_desp_readout_demand;
  twicCbLongCharDespPrepWriteDemand_t *long_char_desp_prepare_writein_demand;
  twicCbCharDespExecWriteInDemand_t *char_desp_exec_writein_demand;
/* characteristics */
  twicCbCharValReadOutDemand_t *char_val_readout_demand;
  twicCbCharValMultiReadOutDemand_t *char_val_multi_readout_demand;
  twicCbCharValWriteInPost_t *char_val_writein_post;
  twicCbCharValWriteInDemand_t *char_val_writein_demand;
  twicCbLongCharValReadOutDemand_t *long_char_val_readout_demand;
  twicCbLongCharValPrepWriteDemand_t *long_char_val_prepare_writein_demand;
  twicCbCharValExecWriteInDemand_t *char_val_exec_writein_demand;
} twicLeServerCb_t;

typedef void (twicCbPrimSvc_t)(
  const void * const connif,
  const uint8_t status, const bool next, const uint16_t error_handle,
  const uint16_t attribute_handle, const uint16_t end_group_handle,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
typedef void (twicCbPrimSvcByUuid_t)(
  const void * const connif,
  const uint8_t status, const bool next, const uint16_t error_handle,
  const uint16_t attribute_handle, const uint16_t end_group_handle);
typedef void (twicCbIncSvc_t)(
  const void * const connif,
  const uint8_t status, const bool next, const uint16_t error_handle,
  const uint16_t attribute_handle,
  const uint16_t included_service_attribute_handle,
  const uint16_t included_service_end_group_handle,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
typedef void (twicCbAllCharOfSvc_t)(
  const void * const connif,
  const uint8_t status, const bool next, const uint16_t error_handle,
  const uint16_t attribute_handle, const uint8_t char_properties,
  const uint16_t char_value_handle,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
typedef void (twicCbCharByUuid_t)(
  const void * const connif,
  const uint8_t status, const bool next, const uint16_t error_handle,
  const uint16_t attribute_handle, const uint8_t char_properties,
  const uint16_t char_value_handle,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
typedef void (twicCbAllDespOfSvc_t)(
  const void * const connif,
  const uint8_t status, const bool next, const uint16_t error_handle,
  const uint16_t attribute_handle,
  const uint64_t uuid_lsb, const uint64_t uuid_msb, const uint8_t uuid_len);
typedef void (twicCbCharValReadOut_t)(
  const void * const connif, const uint8_t status, const void *const arg);
typedef void (twicCbCharValReadOutUsingUuid_t)(
  const void * const connif,
  const uint8_t status, const bool next, const uint16_t error_handle,
  const uint16_t char_value_handle, const void *const resp);
typedef void (twicCbCharValWriteinResponse_t)(
  const void * const connif, const uint8_t status);
typedef void (twicCbCharDespReadout_t)(
  const void * const connif, const uint8_t status, const void *const resp);
typedef void (twicCbCharDespWriteinResponse_t)(
  const void * const connif, const uint8_t status);
typedef void (twicCbNotificationReceived_t)(
  const void * const connif,
  const uint8_t status, const uint16_t char_value_handle,
  const void *const resp);
typedef void (twicCbIndicationReceived_t)(
  const void * const connif,
  const uint8_t status, const uint16_t char_value_handle,
  const void *const resp);
typedef void (twicCbCharValWriteInPostStarted_t)(
  const void * const connif, const uint8_t status);
typedef void (twicCbMultipleCharValReadOut_t)(
  const void * const connif, const uint8_t status, const void *const arg);
typedef void (twicCbReliablWriteinConfirmation_t)(
  const void * const connif, const uint8_t status, const void *const arg);
typedef void (twicCbLongCharValReadout_t)(
  const void * const connif, const uint8_t status, const bool next,
  const void *const resp);
typedef void (twicCbLongCharDespReadout_t)(
  const void * const connif, const uint8_t status, const bool next,
  const void *const resp);
typedef void (twicCbLongCharValWriteinResponse_t)(
  const void * const connif, const uint8_t status);
typedef void (twicCbLongCharDespWriteinResponse_t)(
  const void * const connif, const uint8_t status);

typedef struct {
  twicCbExgMtuResult_t *mtu_exchange_result;
  twicCbPrimSvc_t *primary_service;
  twicCbPrimSvcByUuid_t *primary_service_by_uuid;
  twicCbIncSvc_t *included_service;
  twicCbAllCharOfSvc_t *all_char_of_service;
  twicCbCharByUuid_t *char_by_uuid;
  twicCbAllDespOfSvc_t *all_char_descriptors;
  twicCbCharValReadOut_t *char_value_readout;
  twicCbCharValReadOutUsingUuid_t *char_value_readout_using_uuid;
  twicCbCharValWriteinResponse_t *char_value_writein_response;
  twicCbCharDespReadout_t *char_desp_readout;
  twicCbCharDespWriteinResponse_t *char_desp_writein_response;
  twicCbNotificationReceived_t *notification_received;
  /* Needs to call twicIfLeGattClientIndicationConfirmationResponse */
  twicCbIndicationReceived_t *indication_received;
  twicCbCharValWriteInPostStarted_t *char_vale_writein_started;
  twicCbMultipleCharValReadOut_t *multiple_char_values_readout;
  twicCbReliablWriteinConfirmation_t *reliable_writein_confirmation;
  twicCbLongCharValReadout_t *long_char_value_readout;
  twicCbLongCharDespReadout_t *long_char_desp_readout;
  twicCbLongCharValWriteinResponse_t *long_char_value_writein_response;
  twicCbLongCharDespWriteinResponse_t *long_char_desp_writein_response;
} twicLeClientCb_t;

typedef void (twicCbSmpPairingResponse_t)(
  const void * const connif, const uint8_t status, const void *const arg);
typedef void (twicCbSmpPairingDemand_t)(
  const void * const connif, const void *const arg);
typedef void (twicCbSmpPairingAccept_t)(
  const void * const connif, const uint8_t status);
typedef void (twicCbSmpSecurityRequest_t)(
  const void * const connif, const bool bonded_device,
  const bool auth_req_bonding, const bool auth_req_mitm_protection);
typedef void (twicCbSmpPairingFailed_t)(
  const void * const connif, const twicSmReasonCode_t reason);
typedef void (twicCbSmpStkGenMethod_t)(
  const void * const connif, const uint8_t status,
  const twicStkGenMethod_t method);
typedef void (twicCbSmpInputPasskey_t)(const void * const connif);
typedef void (twicCbSmpDisplayPasskey_t)(const void * const connif);
typedef void (twicCbSmpStkGeneratedSuccessfully_t)(
  const void * const connif, const twicStk_t *const stk);
typedef void (twicCbSmpEncryptionInfo_t)(
  const void * const connif, const twicLtk_t *const ltk);
typedef void (twicCbSmpMasterIdentification_t)(
  const void * const connif,
  const twicEdiv_t *const ediv, const twicRand_t *const rand);
/* This event is generated only if EncKey is set to 1 in Initiator Key
 * Distribution field of Pairing Event Received (pairing_response)
 * from the slave device. */
typedef void (twicCbSmpEncryptionInfoSent_t)(
  const void * const connif, const twicLtk_t *const ltk);
typedef void (twicCbSmpMasterIdentificationSent_t)(
  const void * const connif, const twicEdiv_t *const ediv,
  const twicRand_t *const rand);
typedef void (twicCbSmpEncryptionChangeEvent_t)(
  const void * const connif, twicSmReasonCode_t reason,
  const uint8_t key_type, const bool encryption_enable,
  const uint8_t encryption_key_size);
typedef void (twicCbSmpEncKeyRefreshComplete_t)(
  const void * const connif, twicSmReasonCode_t reason,
  const uint8_t key_type, const uint8_t encryption_key_size);
typedef void (twicCbSmpPairingComplete_t)(
  const void * const connif, const uint8_t status, const twicAuthInfo_t bits);

/*
 * Initialtor HOST starts encryption.
 * Controller Initiator sends LL_ENC_REQ
 * Controller Responder sends LL_ENC_RESP
 * Controller Responder processes "LE Long Term Key Request"
 * Controller Responser processes "LE Long Term Key Request Reply"
 * twicCbSmpSlave___SessionRequestReplyEvent is sent to Slv HOST from
 * Controller.
 * Controller Responser sends "LL_START_ENC_REQ"
 * Controller Initiator sends "LL_START_ENC_RSP"
 * Controller Responder sends "LL_START_ENC_RSP"
 * encryption_change is sent to Mas/Slv HOST from Controller. */
typedef void (twicCbSmpSlaveStkSessionRequestReplyEvent_t)(
  const void * const connif, const uint8_t status, const twicStk_t *const stk);
typedef void (twicCbSmpSlaveLtkSessionRequestReplyEvent_t)(
  const void * const connif, const uint8_t status, const twicLtk_t *const ltk);
typedef void (twicCbSmpIdentityInformation_t)(
  const void * const connif, const twicIrk_t *const irk);
typedef void (twicCbSmpIdentityAddressInformation_t)(
  const void * const connif, const bool address_type_random,
  const twicBdaddr_t *const identity);
typedef void (twicCbSmpSigningInformation_t)(
  const void * const connif, const twicCsrk_t *const csrk);
typedef void (twicCbSmpOobKeyEntry_t)(const void * const connif);
typedef void (twicCbSmpStoreBondingInformation_t)(
  const void * const connif, const bool address_type_random,
  const twicBdaddr_t *const identity, const bool erase);
typedef void (twicCbSmpInquiryBondingInformation_t)(
  const void * const connif, const bool address_type_random,
  const twicBdaddr_t *const identity);
typedef void (twicCbSmpBondingState_t)(
  const void * const connif, const twicSmReasonCode_t reason);

typedef struct {
  twicCbSmpPairingResponse_t *pairing_response;
  twicCbSmpSecurityRequest_t *security_request;
  twicCbSmpPairingFailed_t *pairing_failed;
  twicCbSmpStkGenMethod_t *stk_generation_method;
  twicCbSmpInputPasskey_t *input_passkey;
  twicCbSmpDisplayPasskey_t *display_passkey;
  twicCbSmpStkGeneratedSuccessfully_t *stk_generated;
  twicCbSmpEncryptionInfo_t *encryption_info;
  twicCbSmpMasterIdentification_t *master_identification;
  twicCbSmpEncryptionInfoSent_t *encryption_info_sent;
  twicCbSmpMasterIdentificationSent_t *master_identification_sent;
  twicCbSmpEncryptionChangeEvent_t *encryption_change;
  twicCbSmpEncKeyRefreshComplete_t *encryption_key_refresh_complete;
  twicCbSmpPairingComplete_t *pairing_complete;
  twicCbSmpIdentityInformation_t *identity_information;
  twicCbSmpIdentityAddressInformation_t *identity_address_information;
  twicCbSmpSigningInformation_t *signing_information;
  twicCbSmpIdentityInformation_t *identity_information_sent;
  twicCbSmpIdentityAddressInformation_t *identity_address_information_sent;
  twicCbSmpSigningInformation_t *signing_information_sent;
  twicCbSmpOobKeyEntry_t *oob_information;
  twicCbSmpStoreBondingInformation_t *store_bonding_information;
  twicCbSmpInquiryBondingInformation_t *inquiry_bonding_information;
  twicCbSmpBondingState_t *bonding_state;
} twicLeSmpICb_t;

typedef struct {
  twicCbSmpPairingDemand_t *pairing_demand;
  twicCbSmpPairingAccept_t *pairing_acceptance_sent;
  twicCbSmpPairingFailed_t *pairing_failed;
  twicCbSmpStkGenMethod_t *stk_generation_method;
  twicCbSmpInputPasskey_t *input_passkey;
  twicCbSmpDisplayPasskey_t *display_passkey;
  twicCbSmpStkGeneratedSuccessfully_t *stk_generated;
  twicCbSmpEncryptionInfo_t *encryption_info;
  twicCbSmpMasterIdentification_t *master_identification;
  twicCbSmpEncryptionInfoSent_t *encryption_info_sent;
  twicCbSmpMasterIdentificationSent_t *master_identification_sent;
  twicCbSmpEncryptionChangeEvent_t *encryption_change;
  twicCbSmpEncKeyRefreshComplete_t *encryption_key_refresh_complete;
  twicCbSmpPairingComplete_t *pairing_complete;
  twicCbSmpSlaveStkSessionRequestReplyEvent_t *stk_session_request_reply_event;
  twicCbSmpSlaveLtkSessionRequestReplyEvent_t *ltk_session_request_reply_event;
  twicCbSmpIdentityInformation_t *identity_information;
  twicCbSmpIdentityAddressInformation_t *identity_address_information;
  twicCbSmpSigningInformation_t *signing_information;
  twicCbSmpIdentityInformation_t *identity_information_sent;
  twicCbSmpIdentityAddressInformation_t *identity_address_information_sent;
  twicCbSmpSigningInformation_t *signing_information_sent;
  twicCbSmpOobKeyEntry_t *oob_information;
  twicCbSmpStoreBondingInformation_t *store_bonding_information;
  twicCbSmpInquiryBondingInformation_t *inquiry_bonding_information;
  twicCbSmpBondingState_t *bonding_state;
} twicLeSmpRCb_t;

/* 0x0000,0x0001,0xEEEE,0x0084 = Accepted,Rejected,L2CAP
 * timeout,Disconnect Error */
typedef void (twicCbL2capConnectionUpdate_t)(
  const void * const connif, const bool central, const uint8_t status);
/* 0x00,0x01,0x02 = Command Not Understood,Signaling MTU
 * exceeded,Invalid CID in Request */
typedef void (twicCbRejectL2capConnectionUpdate_t)(
  const void * const connif, const bool central, const uint8_t status);
typedef void (twicCbRemoteUsedFeatures_t)(
  const void * const connif, const void *const arg);
typedef void (twicCbRemoteVersion_t)(
  const void * const connif, const void *const arg);
typedef void (twicCbAdvReport_t)(
  const void * const connif, const void *const arg);
typedef void (twicCbResolvablePrivacy_t)(
  const void * const connif, const void *const arg);
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
typedef void (twicCbEncrypt_t)(
  const void * const connif, const void *const arg);
#endif

typedef struct {
  twicCbConnComplete_t *connection_complete;
  twicCbConnUpdate_t *connection_update;
#if defined(TWIC_BLE_HWIP_V41) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
  twicCbConnParameter_t *connection_parameter;
#endif
  twicCbDisconn_t *disconnection;
  twicCbDownstreamRequest_t *isr_downstream_request;
  twicCbL2capConnectionUpdate_t *l2cap_connection_update;
  twicCbRejectL2capConnectionUpdate_t *reject_l2cap_connection_update;
  twicCbRemoteUsedFeatures_t *remote_used_features;
  twicCbRemoteVersion_t *remote_version;
  twicCbResolvablePrivacy_t *resolvable_privacy;
  twicCbResolvablePrivacy_t *resolved_privacy;
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
  twicCbEncrypt_t *encrypt;
#endif
  twicCbReadResultRssi_t *read_result_rssi;
  twicCbReadResultTxPower_t *read_result_tx_power;
  twicCbWakeupRequest_t *isr_wakeup_request;
  twicCbAdvReport_t *adv_report;
} twicLeCb_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t  value[TWIC_ALIGN4(sizeof(twicRespCm_t))];
} TZ1K_PACKED_FTR TZ1K_ALIGNED((4)) twicGattValue_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t conn : 1;
  uint8_t scan : 1;
  uint8_t adve : 1;
  uint8_t ncad : 1; /* SCAN_RESP is supposed to be processed in CE. */
  uint8_t pad : 4;
} TZ1K_PACKED_FTR twicOpMd_t;

#undef GROUP_CLI_EXG_MTU_EVENT

typedef TZ1K_PACKED_HDR struct {
  uint8_t notification : 1;
  uint8_t indication : 1;
  uint8_t smp_i_security_event : 1;
  uint8_t smp_i_bonding_enabled_event : 1;
  uint8_t smp_i_oob_key_entry_req_event : 1;
  uint8_t smp_i_key_req_event : 1;
  uint8_t smp_i_key_entry_req_event : 1;
  uint8_t smp_i_display_key_event : 1;
  uint8_t smp_r_key_req_event : 1;
  uint8_t smp_r_oob_key_entry_req_event : 1;
  uint8_t smp_r_key_entry_req_event : 1;
  uint8_t smp_r_display_key_event : 1;
  uint8_t smp_r_pairing_event : 1;
#if defined(GROUP_CLI_EXG_MTU_EVENT)
  uint8_t cli_exg_mtu_event : 1;
  uint8_t pad : 2;
#else
  uint8_t pad : 3;
#endif
} TZ1K_PACKED_FTR _twicGroup_t;

typedef TZ1K_PACKED_HDR union {
  _twicGroup_t b;
  uint16_t u16;
} TZ1K_PACKED_FTR twicGroup_t;

typedef TZ1K_PACKED_HDR struct {
  tz1utListHead_t sibling;
  const twicLeServerCb_t *server_cb;
  const twicLeClientCb_t *client_cb;
  const twicLeSmpICb_t *smp_i_cb;
  const twicLeSmpRCb_t *smp_r_cb;
  const twicLeCb_t *le_cb;
  twicGattValue_t resp;
//#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
  twicOpMd_t opmd;
  twicGroup_t group;
//#endif
  uint8_t interface;
  uint8_t role;
  uint8_t token; /* actually needs 2 bytes (service id + op code) */
  uint16_t mtu;
  void *epr;
  void *conn;
  void *signal;
} TZ1K_PACKED_FTR TZ1K_ALIGNED((4)) twicInstance_t;

typedef struct {
  uint8_t c_rt : 1; /* Controller realtime status of low power consumption. */
  uint8_t c_en : 1; /* Controller main switch of low power (LP) consumption. */
  uint8_t c_id : 1; /* Controller can try LP consumption for IDLE. */
  uint8_t c_ad : 1; /* Controller can try ADV LP consumption for IDLE. */
  uint8_t c_cs : 1; /* Controller can try CONN LP consumption for IDLE. */
  uint8_t c_sd : 1; /* Controller can do shutdown. */
  uint8_t h_rt : 1; /* Host realtime status of LP consumption. */
  uint8_t h_tb : 1; /* HOST is in LP status of TZBT. */
  uint8_t h_id : 1; /* HOST is in LP status of IDLE. */
  uint8_t h_cs : 1; /* HOST is in LP status of CONN/SCAN. */
  uint8_t h_ca : 1; /* HOST is in LP status of Connectable ADV. */
  uint8_t h_na : 1; /* HOST is in LP status of NC ADV. */
  uint8_t pad : 4;
} twicCeLeLpInfo_t;

/*
 * Description of Service Entry.
 *
 */

/* TYPE */
#define TWIC_ENTRY_TYPE_UNDEFINED                 (0)
#define TWIC_ENTRY_TYPE_SERVICE                   (1)
#define TWIC_ENTRY_TYPE_SECONDARY_SERVICE         (2)
#define TWIC_ENTRY_TYPE_CHARACTERISTIC_VALUE      (3)
#define TWIC_ENTRY_TYPE_CHARACTERISTIC_DESCRIPTOR (4)
#define TWIC_ENTRY_TYPE_INCLUDED_SERVICE          (5)
#define TWIC_ENTRY_TYPE_MNG                       (6)
#define TWIC_ENTRY_TYPE_SIGNAL                    (7)

/* REQUEST */
#define TWIC_ENTRY_RESERVED           (0x00)
#define TWIC_ENTRY_HANDLE_DEFINITION  (0x01)
#define TWIC_ENTRY_HANDLE_DECLARATION (0x02)
#define TWIC_ENTRY_MNG_MR             (0xD0)
#define TWIC_ENTRY_MNG_MW             (0xD1)
#define TWIC_ENTRY_MNG_DW             (0xC2)
#define TWIC_ENTRY_MNG_DR             (0xC3)
#define TWIC_ENTRY_MNG_POWER_MODE     (0xA1)
#define TWIC_ENTRY_MNG_FC             (0xA2)
#define TWIC_ENTRY_MNG_BR             (0x42)
#define TWIC_ENTRY_MNG_HOST_DELAY     (0xA9)
#define TWIC_ENTRY_MNG_PCP            (0xA3)
#define TWIC_ENTRY_MNG_MA             (0xAB)
#define TWIC_ENTRY_MNG_SET_TX_POWER   (0x9B)
#define TWIC_ENTRY_MNG_POWER_CLOCK    (0x9D)
#define TWIC_ENTRY_MNG_POWER_INIT     (0x68)
#define TWIC_ENTRY_MNG_WRITE_BDADDR   (0x04)
#define TWIC_ENTRY_MNG_READ_BDADDR    (0x05)
#define TWIC_ENTRY_MNG_TCUMODE        (0x06)
#define TWIC_ENTRY_MNG_TCU            (0x07)
#define TWIC_ENTRY_MNG_READ_LVI       (0x08)
#define TWIC_ENTRY_MNG_PSB            (0x55)
#define TWIC_ENTRY_MNG_PSPW           (0x56)
#define TWIC_ENTRY_MNG_PC             (0x57)
#define TWIC_ENTRY_MNG_FV             (0x0D)

#define TWIC_PROPERTY_B     (0x01) /* permits broadcasts */
#define TWIC_PROPERTY_R	    (0x02) /* permits reads */
#define TWIC_PROPERTY_WWR   (0x04) /* Write Without Response */
#define TWIC_PROPERTY_W     (0x08) /* permits writes */
#define TWIC_PROPERTY_N     (0x10) /* permits notifications */
#define TWIC_PROPERTY_I     (0x20) /* permits indications */
#define TWIC_PROPERTY_A     (0x40) /* permits signed writes */
#define TWIC_PROPERTY_E     (0x80) /* Extended Properties */

/* Attribute permissions Bit Allocation */
#define TWIC_PERMISSION_R   (0x0001) /* read */
#define TWIC_PERMISSION_W   (0x0002) /* write */
#define TWIC_PERMISSION_M0  (0x0004) /* MITM Enabled/Disabled (R or R/W) */
#define TWIC_PERMISSION_A0  (0x0008) /* Authentication (R or R/W) */
#define TWIC_PERMISSION_E0  (0x0010) /* Encryption (R or R/W) */
#define TWIC_PERMISSION_M1  (0x2020) /* MITM (W) */
#define TWIC_PERMISSION_A1  (0x2040) /* Authentication (W) */
#define TWIC_PERMISSION_E1  (0x2080) /* Encryption (W) */
#define TWIC_PERMISSION_K7  (0x0700)
#define TWIC_PERMISSION_K8  (0x0800)
#define TWIC_PERMISSION_K9  (0x0900)
#define TWIC_PERMISSION_K10 (0x0A00)
#define TWIC_PERMISSION_K11 (0x0B00)
#define TWIC_PERMISSION_K12 (0x0C00)
#define TWIC_PERMISSION_K13 (0x0D00)
#define TWIC_PERMISSION_K14 (0x0E00)
#define TWIC_PERMISSION_K15 (0x0F00)
#define TWIC_PERMISSION_K16 (0x1000)

typedef TZ1K_PACKED_HDR struct {
  uint8_t properties, eidx;
  uint16_t permissions;
  uint16_t p_handle;
  uint16_t b_handle; /* attribute handle */
} TZ1K_PACKED_FTR twicAttr_t;

typedef TZ1K_PACKED_HDR struct {
  uint8_t len;
  uint8_t value[23];
} TZ1K_PACKED_FTR twicCont_t;

/* NOTE: ROM5 dual topology is restricted to single connection.  Any
   request of master role must not be issued when APP twicEntry conn
   is queued if the dual topology is enabled in the ROM5.
   For confirmation the SAP code has to check if the acceptance is
   granted or postponed. */

typedef TZ1K_PACKED_HDR struct twicEntry {
  tz1utListHead_t o_link;
  struct twicEntry *next, *prev;
  uint8_t type, ap /* attribute perfectness */, interface, seq;
  uint8_t req, rsp, rec, result;
  uint16_t id, reserved;
  uint16_t handle, life;
  twicInstance_t *instance;
  TZ1K_PACKED_HDR union {
    twicCont_t cont;
    twicAttr_t attr;
  } TZ1K_PACKED_FTR u;
} TZ1K_PACKED_FTR TZ1K_ALIGNED((4)) twicEntry_t;

#define TWIC_ENTRY_HASH_DEF(name, size)             \
  static twicEntry_t *twicEntryHash_##name[size];   \
  static uint16_t twicHashSize_##name = size

#define TWIC_ENTRY_HASH_INIT(name) (&twicEntryHash_##name), twicHashSize_##name
#define TWIC_ENTRY_HASH_SYSTEM_INIT TWIC_ENTRY_HASH_INIT(srvif)
#define TWIC_ENTRY_HASH_SYSTEM_DEF(size) TWIC_ENTRY_HASH_DEF(srvif, size)

typedef struct {
  twicEntry_t *(*hash)[];
  uint16_t size;
} twicHash_t;

#define TWIC_HASH_DEF(name) static twicHash_t twicHash_##name
#define TWIC_HASH(name) twicHash_##name.hash
#define TWIC_HASH_SIZE(name) twicHash_##name.size

#define TWIC_IFACE_GAP_UNDEFINED         (0x00)
#define TWIC_IFACE_GAP_CENTRAL           (0x01)
#define TWIC_IFACE_GAP_PERIPHERAL        (0x02)
#define TWIC_IFACE_GAP_CREATE_CONNECTION (0x10)

typedef TZ1K_PACKED_HDR struct {
  uint32_t size;
  twicEntry_t * const * entries;
  twicEntry_t conn;
  twicEntry_t signal;
  twicInstance_t instance;
} TZ1K_PACKED_FTR TZ1K_ALIGNED((4)) twicConnIface_t;

/* NOTE: ROM5 dual topology is restricted to single connection.  Any
   request of master role must not be issued when APP twicEntry conn
   is queued if the dual topology is enabled in the ROM5. */

/* TZ1x ROM5 Configuration.
   1. The number of MNG message lock is integrated and single. This entry
      is defined and declared in the middleware code.
   2. The number of the other messages which are characteristics value and
      descriptors is separated to each of the commands. */

#if defined(TWIC_TOOLCHAIN_ARMCC)

#define TWIC_CONN_APP_DEF(name, db_size)                    \
  extern twicEntry_t * const twicEntries_##name[db_size];   \
  twicConnIface_t twicConnIface_##name = {                  \
    db_size, &twicEntries_##name[0] };                      \
  twicEntry_t * const twicEntries_##name[db_size]
#define TWIC_CONN_CLI_DEF(name)                             \
  twicConnIface_t twicConnIface_##name = {0, NULL};

#else

#define TWIC_CONN_APP_DEF(name, db_size)                    \
  extern twicEntry_t * const twicEntries_##name[db_size];   \
  twicConnIface_t twicConnIface_##name = {                  \
    .size = db_size, .entries = &twicEntries_##name[0] };   \
  twicEntry_t * const twicEntries_##name[db_size]
#define TWIC_CONN_CLI_DEF(name)                             \
  twicConnIface_t twicConnIface_##name = {                  \
    .size = 0, .entries = NULL };

#endif

#define TWIC_CONN_INIT(name) (&(twicConnIface_##name))
#define TWIC_CONN(name) twicConnIface_##name.conn
#define TWIC_IF(name) (&(twicConnIface_##name))
#define TWIC_EXTERN_IF(name) extern twicConnIface_t twicConnIface_##name
#define TWIC_IFCAST(connif) ((twicConnIface_t *)(connif))
#define TWIC_EIDX_TO_ENTRY(name, eidx) twicEntries_##name[eidx - 1]
#define TWIC_IF_TO_ENTRY(eidx) (iface->entries[eidx - 1])
#define TWIC_CONN_GAP_ROLE(name) (twicConnIface_##name.instance.role)

/*
 * Advertising.
 *
 */

#define TWIC_ADV_TYPE_IND            (0) /* Connectable undirected
                                          * advertising */
#define TWIC_ADV_TYPE_DIRECT_IND     (1) /* Connectable directed advertising */
#define TWIC_ADV_TYPE_SCAN_IND       (2) /* Scannable undirected advertising*/
#define TWIC_ADV_TYPE_NONCONN_IND    (3) /* Non connectable undirected
                                          * advertising  */
#if defined(TWIC_BLE_HWIP_V41)
#define TWIC_ADV_TYPE_DIRECT_IND_LDC (4) /* Low Duty Cycle */
#endif

#define TWIC_ADDR_TYPE_PUBLIC_DEVICE_ADDRESS (0)
#define TWIC_ADDR_TYPE_RANDOM_DEVICE_ADDRESS (1)

#define TWIC_ADV_FILTER_ANY_ANY     (0) /* Allow Scan Request from Any,
                                         * Allow Connect Request from
                                         * Any. */
#define TWIC_ADV_FILTER_WHITE_ANY   (1) /* Allow Scan Request from White
                                         * List Only, Allow Connect
                                         * Request from Any. */
#define TWIC_ADV_FILTER_ANY_WHITE   (2) /* Allow Scan Request from Any,
                                         * Allow Connect Request from
                                         * White List Only. */
#define TWIC_ADV_FILTER_WHITE_WHITE (3) /* Allow Scan Request from
                                         * White List Only, Allow
                                         * Connect Request from White
                                         * List Only. */

#define TWIC_ADV_CHANNEL_37  (0x01) /* xxxxxxx1b Enable channel 37 use */
#define TWIC_ADV_CHANNEL_38  (0x02) /* xxxxxx1xb Enable channel 38 use */
#define TWIC_ADV_CHANNEL_39  (0x04) /* xxxxx1xxb Enable channel 39 use */
#define TWIC_ADV_CHANNEL_ALL (0x07) /* 00000111b Default (all channels
                                     * enabled) */


/*
 * TZ1EM (TZ1 energy management)
 *
 */

#define TWIC_TZ1EM_NC_ADV (0)
#define TWIC_TZ1EM_CN_ADV (1)
#define TWIC_TZ1EM_IDLE   (2)
#define TWIC_TZ1EM_CONN   (3)
#define TWIC_TZ1EM_SYS    (4)

void twicSetUuid(uint8_t *uuid, uint64_t uuid_lsb, uint64_t uuid_msb);
uint64_t twicUuidLsb(twicUuid_t *uuid);
uint64_t twicUuidMsb(twicUuid_t *uuid);

/*
 * SERVICE Api.
 *
 */

twicStatus_t twicSsGattInitialize(void);
void twicSsGattCleanup(
  const uint8_t interface, twicConnIface_t * const conn_iface);
twicStatus_t twicSsLeCeLowPowerInit(
  const uint8_t seq, twicEntry_t * const conn,
  const uint32_t osc_wait, const uint8_t deep_sleep_control,
  const uint16_t osc_tolerance, const uint16_t osc_jitter,
  const uint16_t premature_wake_time);

twicStatus_t twicSsLeCePatch(
  const uint8_t seq, twicEntry_t * const conn, const uint8_t m2,
  const uint8_t length, const uint8_t * const code, const uint8_t reg,
  const bool enable);
twicStatus_t twicSsLeCeReadFwVer(const uint8_t seq, twicEntry_t * const conn);
twicStatus_t twicSsLeCeDbusRead(
  const uint8_t seq, twicEntry_t * const conn, const uint8_t addr);
twicStatus_t twicSsLeCeDbusWrite(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t addr, const uint16_t data);
twicStatus_t twicSsLeCeMemoryRead(
  const uint8_t seq, twicEntry_t * const conn, const uint32_t addr);
twicStatus_t twicSsLeCeMemoryWrite(
  const uint8_t seq, twicEntry_t * const conn,
  const uint32_t addr, const uint16_t data);
twicStatus_t twicSsLeCeXoscTriming(const uint8_t seq, twicEntry_t * const conn,
                                   const uint8_t data);
twicStatus_t twicSsLeCeSetTxPower(const uint8_t seq, twicEntry_t * const conn,
                                  const uint8_t dbm_idx);
twicStatus_t twicSsLeCeFlowControl(
  const uint8_t seq, twicEntry_t * const conn, const bool enable);
twicStatus_t twicSsLeCeSetBaudrate(
  const uint8_t seq, twicEntry_t * const conn, const twicTzbtBr_t br,
  const uint16_t pu);
twicStatus_t twicSsLeCeHostDelay(
  const uint8_t seq, twicEntry_t * const conn, const uint16_t delay);
#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t twicSsLeCeLowPowerClock(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t source, const uint16_t osc_wait,
  twicStatus_t * const osc_state);
void twicSsLeCeHkLowPowerInfo(twicCeLeLpInfo_t * const info);
twicStatus_t twicSsLeCeLowPowerMode(
  const uint8_t seq, twicEntry_t * const conn, const uint8_t selector);
twicStatus_t twicSsLeCeLowPowerControlPin(const uint8_t seq,
                                          twicEntry_t * const conn,
                                          const uint8_t pin,
                                          const uint8_t status_port);
twicStatus_t twicSsLeCeLowPowerBackup(
  const uint8_t seq, twicEntry_t * const conn,
  const uint32_t parameter1, const uint32_t parameter2);
#endif
twicStatus_t twicSsLeCeHkTimeout(
  const bool cleanup, const uint16_t cycle, const uint8_t specified_fidx);
twicStatus_t twicSsLeReadBdaddr(const uint8_t seq, twicEntry_t * const conn);
#if defined(TWIC_API_LEREADLOCALVERSIONINFORMATION)
twicStatus_t twicSsLeReadLocalVersionInformation(const uint8_t seq,
                                                 twicEntry_t * const conn);
#endif
twicStatus_t twicSsLeWriteBdaddr(
  const uint8_t seq, twicEntry_t * const conn, const uint64_t * const bdaddr);
twicStatus_t twicSsLeCeChangeToCm(const uint8_t seq, twicEntry_t * const conn);

/* 3 functions for twicSsGattServerStartService:
   twicSsLeInitializeDevice, twicSsLeGattClientStart and
   twicSsLeGattServerStart are twicReq. */

twicStatus_t twicSsLeConnectionUpdate(const uint8_t seq,
                                      twicEntry_t * const conn,
                                      const uint8_t op_code,
                                      uint16_t conn_int_min,
                                      uint16_t conn_int_max,
                                      uint16_t slave_latency,
                                      uint16_t supervision_timeout,
                                      uint16_t min_ce_length,
                                      uint16_t max_ce_length);
twicStatus_t twicSsLeDisconnect(
  const uint8_t seq, twicEntry_t * const conn, const twicBdaddr_t * const bd);
twicStatus_t twicSsGattServerAddService(
  const uint8_t seq, twicEntry_t * const entry,
  const twicUuid_t * const uuid);
twicStatus_t twicSsGattServerAddSecondaryService(
  const uint8_t seq, twicEntry_t * const entry,
  const twicUuid_t * const uuid);
twicStatus_t twicSsGattServerAddIncludeService(
  const uint8_t seq, twicEntry_t * const service,
  twicEntry_t * const inclusion, const twicUuid_t * const uuid);
twicStatus_t twicSsGattServerAddCharacteristics(
  const uint8_t seq,const uint16_t service_handle,
  twicEntry_t *entry, const twicUuid_t * const uuid);
twicStatus_t twicSsGattServerSetElementVl(const uint8_t seq,
                                          twicEntry_t * entity,
                                          const twicUuid_t * const uuid,
                                          const uint16_t max_element_length,
                                          const uint16_t element_length,
                                          const uint8_t * const element_value);
twicStatus_t twicSsGattServerSetElement(const uint8_t seq, twicEntry_t *entity,
                                        const twicUuid_t * const uuid,
                                        const uint16_t element_length,
                                        const uint8_t * const element_value);
twicStatus_t twicSsGattServerWriteElement(
  const uint8_t seq, twicEntry_t * const conn,
  const uint16_t handle, const uint16_t length, const uint8_t * const value);

/* 2 function for twicSsGattSetAdvData */
twicStatus_t twicSsLeDiscoverable(
  const uint8_t seq, twicEntry_t * const conn,
  const uint16_t min_interval, const uint16_t max_interval,
  const uint8_t advertising_type, const uint8_t own_address_type,
  const uint8_t direct_address_type, const uint64_t * const direct_address,
  const uint8_t advertising_channel_map,
  const uint8_t advertising_filter_policy,
  const uint8_t advertising_data_length,
  const uint8_t * const advertising_data,
  const uint8_t scan_resp_data_length,
  const uint8_t * const scan_resp_data);
twicStatus_t twicSsGattTransmit(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id,
  const uint16_t handle, const uint8_t length, const uint8_t * const data);
twicStatus_t twicSsDataConstructionCm(const uint8_t in_data);
twicStatus_t twicSsDataConstructionHc(const uint8_t in_data);
void twicSsIsrLeDownstreamRequest(void);
void twicSsIsrLeHostWakeup(void);
void twicSsIsrLeStatusChanged(void);
twicStatus_t twicSsPumpCompleteDataCm(void);
twicStatus_t twicSsPumpCompleteDataHc(void);
/* twicSsGattGetDeviceType */
twicStatus_t twicSsLeReadRemoteVersion(
  const uint8_t seq, twicEntry_t * const conn);
twicStatus_t twicSsLeCreateConnection(
  const uint8_t seq, twicEntry_t * const conn, uint16_t interval,
  uint16_t window, bool use_white_list, bool peer_bdaddr_is_public,
  const twicBdaddr_t *const peer_bdaddr, uint16_t conn_int_min,
  uint16_t conn_int_max, uint16_t slave_latency, uint16_t supervison_timeout,
  uint16_t min_ce_length, uint16_t max_ce_length, bool own_bdaddr_is_public);
#if defined(TWIC_API_LEADDWHITELIST)
twicStatus_t twicSsLeAddWhitelist(
  const uint8_t seq, twicEntry_t * const conn, const bool random,
  const twicBdaddr_t * const bd);
#endif
#if defined(TWIC_CONFIG_ENABLE_SCAN)
twicStatus_t twicSsLeSetScanEnable(
  const uint8_t seq, twicEntry_t * const conn, const uint16_t interval,
  const uint16_t window, const uint8_t active_scanning,
  const uint8_t own_bdaddr_is_random, const uint8_t whitelist_enable,
  const uint8_t scan_enable, const uint8_t filter_duplicates);
#endif
#if defined(TWIC_API_LEDELWHITELIST)
twicStatus_t
twicSsLeDelWhitelist(const uint8_t seq, twicEntry_t * const conn,
                     const bool random, const twicBdaddr_t * const bd);
#endif
#if defined(TWIC_API_LECLEAREWHITELIST)
twicStatus_t
twicSsLeClearWhitelist(const uint8_t seq, twicEntry_t * const conn);
#endif
twicStatus_t twicSsLeReadRemoteUsedFeatures(
  const uint8_t seq, twicEntry_t * const conn);
#if defined(TWIC_API_LEREADWHITELISTSIZE)
twicStatus_t
twicSsLeReadWhitelistSize(const uint8_t seq, twicEntry_t * const conn);
#endif
#if defined(TWIC_API_LEREADSUPPORTEDSTATES)
twicStatus_t
twicSsLeReadSupportedStates(const uint8_t seq, twicEntry_t * const conn);
#endif
#if defined(TWIC_API_LEREADCHANNELMAP)
twicStatus_t
twicSsLeReadChannelMap(const uint8_t seq, twicEntry_t * const conn);
#endif
#if defined(TWIC_API_LESETHOSTCHANNELCLASSIFICATION)
twicStatus_t
twicSsLeSetHostChannelClassification(const uint8_t seq,
                                     twicEntry_t * const conn,
                                     const twicChannelMap_t *const channel_map);
#endif
#if defined(TWIC_API_LEREADLOCALSUPPORTEDFEATURES)
twicStatus_t
twicSsLeReadLocalSupportedFeatures(const uint8_t seq, twicEntry_t * const conn);
#endif
#if defined(TWIC_API_LESETRANDOMADDRESS)
twicStatus_t
twicSsLeSetRandomAddress(const uint8_t seq, twicEntry_t * const conn,
                         const twicBdaddr_t * const bd);
#endif
#if defined(TWIC_API_LELMRESOLVEBDADDR)
twicStatus_t
twicSsLeLmResolveBdaddr(const uint8_t seq, twicEntry_t *const conn,
                        const twicBdaddr_t *const bd, const uint8_t num_of_irk,
                        const twicIrk_t * irks);
#endif
#if defined(TWIC_API_LELMRESOLVEBDADDR)
twicStatus_t twicSsLeSmSetIrValue(const uint8_t seq, twicEntry_t * const conn,
                                  const twicIr_t * const ir);
#endif
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
twicStatus_t twicSsLeSmEncrypt(const uint8_t seq, twicEntry_t * const conn,
                               const uint8_t * const key,
                               const uint8_t * const plain_text_data);
#endif
twicStatus_t twicSsReq(
  const uint8_t seq, twicEntry_t * const conn, const uint8_t op_code,
  const uint8_t service_id, const uint8_t * const argv, uint8_t argc);
twicStatus_t twicSsToReq(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id);
twicStatus_t twicSsTreReq(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id, const uint8_t tre);
#if defined(TWIC_API_GATTCLIENTEXGMTU) ||                     \
  defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE) ||      \
  defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID) ||  \
  defined(TWIC_GATTCLIENTREADCHARACTERISTICDESCRIPTOR)
twicStatus_t twicSsTreToReq(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id, const uint16_t tre);
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICE) ||             \
  defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID) ||  \
  defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE) ||                  \
  defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS) ||           \
  defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)||         \
  defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS) ||               \
  defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE) ||          \
  defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR)
twicStatus_t twicSsFyraToReq(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id,
  const uint16_t tre, const uint16_t fyra);
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID) ||  \
  defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)
twicStatus_t twicSsGattClientDiscoveryByUuid(
  const uint8_t seq, twicEntry_t * const conn, const uint8_t op_code,
  const uint16_t starting_handle, const uint16_t ending_handle,
  const twicUuid_t * const uuid);
#endif
#if defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID)
twicStatus_t twicSsGattClientAccessByUuid(
  const uint8_t seq, twicEntry_t * const conn, const uint8_t op_code,
  const uint16_t start_handle, const uint16_t end_handle,
  const twicUuid_t * const uuid);
#endif
#if defined(TWIC_API_GATTCLIENTREADMULTIPLECHARVALUES)
twicStatus_t twicSsClientReadMultipleCharValues(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t number, const uint16_t * const handles);
#endif
#if defined(TWIC_GATTCLIENTSIGNEDWRITEWITHOUTRESPONSE)
twicStatus_t twicSsGattSignedWrite(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id,
  const uint16_t handle, const uint8_t length, const uint8_t * const data);
#endif
#if defined(TWIC_API_GATTCLIENTRELIABLEWRITE)
twicStatus_t twicSsClientReliableWrite(
  const uint8_t seq, twicEntry_t * const conn,
  const uint16_t bytes, const uint16_t total,
  const twicReliableWritein_t * const characteristics_value);
#endif
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICVALUE) || \
  defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR)
twicStatus_t twicSsClientLongWrite(
  const uint8_t seq, twicEntry_t * const conn, const uint8_t op_code,
  const uint16_t handle, const uint16_t offset,
  const uint16_t length, const uint8_t * const value);
#endif

twicStatus_t twicSsFemReq(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id,
  const uint8_t tre, const uint16_t fem);

#if defined(TWIC_CONFIG_SM_INITIATOR)
twicStatus_t twicSsLeSmMasPairingRequest(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t io_capability, const uint8_t oob_data_flag,
  const uint8_t auth_req, const uint8_t max_enc_key_size,
  const uint8_t init_key_dist, const uint8_t resp_key_dist);
twicStatus_t twicSsLeSmMasStartEncryption(
  const uint8_t seq, twicEntry_t * const conn,
  const twicEdiv_t *const ediv, const twicRand_t *const rand,
  const twicLtk_t *const ltk, const uint8_t encryption_key_size);
#endif
#if defined(TWIC_CONFIG_SM_RESPONDER)
twicStatus_t twicSsLeSmSlvPairingConfirm(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t io_capability, const uint8_t oob_data_flag,
  const uint8_t auth_req, const uint8_t max_enc_key_size,
  const uint8_t init_key_dist, const uint8_t resp_key_dist);
#endif
#if defined(TWIC_CONFIG_SM_INITIATOR) || defined(TWIC_CONFIG_SM_RESPONDER)
twicStatus_t twicSsLeSmPasskeyReply(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id,
  const twicPasskeyEntry_t *const passkey);
twicStatus_t twicSsLeSmOobTkReply(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id,
  const twicOobTk_t *const tk);
twicStatus_t twicSsLeSmBondingInformationReply(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id,
  const uint8_t key_set_bitmap, const twicEdiv_t * const r_ediv,
  const twicRand_t * const r_rand, const twicLtk_t * const r_ltk,
  const twicIrk_t * const r_irk, const twicCsrk_t * const r_csrk,
  const twicEdiv_t * const l_ediv, const twicRand_t * const l_rand,
  const twicLtk_t * const l_ltk, const twicIrk_t * const l_irk,
  const twicCsrk_t * const l_csrk, const uint8_t encryption_key_size);
#endif
twicStatus_t twicSsLeIoInitialize(void);
twicStatus_t twicSsLeIoStatus(void);
twicStatus_t twicSsLeIoFinalize(const bool stop_osc32K);
void twicSsLeRequestHighPower(void);
#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t twicSsLeRequestLowPower(void);
#endif
bool twicSsLeLowPowerStatus(void);
void twicSsLeReset(const bool reset);
void twicSsHashCreate(void);
void twicSsGattRegistration(twicEntry_t * const conn);
void twicSsGattDeregistration(const uint8_t interface);
twicStatus_t twicSsGattServerPackService(const uint8_t seq,
                                         twicEntry_t *entity);
#if defined(TWIC_API_LEGATTDBSETPERMISSIONS)
twicStatus_t twicSsGattDbSetPermissions(const uint8_t seq, twicEntry_t *entity,
                                        const uint16_t permissions);
#endif
#if defined(TWIC_API_LEGATTDBGETPERMISSIONS)
twicStatus_t twicSsGattDbGetPermissions(const uint8_t seq, twicEntry_t *entity);
#endif
#endif /* _TWIC_SERVICE_H_ */
