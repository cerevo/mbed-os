/**
 * @file twic_service.c
 * @brief a source file for TZ10xx TWiC for Bluetooth 4.0/4.1 Smart
 * @version V2.0.1
 * @note
 * Please be sure to call the code of a SERVICE layer from the code of
 * an INTERFACE layer. Mutual execution feature supported by RTOS is
 * managed in the INTERFACE layer. In SERVICE layer, the mutual
 * exclusion of the resources is performed under the safeguard of the
 * wrapper of the mutex.
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#include "tz1sm_config.h"
#include "tz1ut_list.h"
#include "tz1sm_hal.h"
#include "twic_service.h"
#include "twic_hash.h"
#include "twic_interface_cb.h"
#include "twic_interface_fid.h"
#if defined(TZ1EM_POWER_PROFILE)
#include "tz1em.h"
#endif

#undef TWIC_SS_DOZE_DEBUG
#undef TWIC_SS_TRACE_HCI_NOP
#undef TWIC_SS_TRACE_SERVICE_ID_NOP

static twicCmContainer_t twic_container;
twicRxPipeQueue_t twic_eq;
TZ1UT_LIST_DEF(srv_accept_root);
TZ1UT_LIST_DEF(srv_req_root);
TZ1UT_LIST_DEF(srv_instance_root);
/* Please follow the number of twicEntry definitions as a guide and
   assign a numerical value. */
TWIC_ENTRY_HASH_SYSTEM_DEF(15);
#if defined(TZ1EM_POWER_PROFILE)
TZ1EM_DEF(BLE_NC_ADV);
TZ1EM_DEF(BLE_CN_ADV);
TZ1EM_DEF(BLE_IDLE);
TZ1EM_DEF(BLE_CONN);
#endif

#define twicSsDebugTx(data, length) {           \
    const uint8_t *const __data = data;         \
    const uint16_t __length = length;           \
    uint16_t __i;                               \
    twicTrace();                                \
    for(__i = 0 ; __i < __length ; __i++)       \
      twicPrintf("%02x ", __data[__i]);         \
    twicPrintf("\r\n");                         \
  }

twicStatus_t twicSsLeIoInitialize(void)
{
  twicStatus_t status = (twicStatus_t)tz1smHalLePmuStart();

  if (TWIC_STATUS_OK != status) {
    twic_eq.sniff.ioinit = false;
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_IO;
  }
  tz1smHalGpioInit();
  tz1smHalDebugUartInit();
  twic_eq.sniff.ioinit = true;
  twic_eq.sniff.fc = twic_eq.sniff.fc_rq = false;
  twic_eq.br = twic_eq.br_rq = 115200;
  return TWIC_STATUS_OK;
}

twicStatus_t twicSsLeIoStatus(void)
{
  if (true != twic_eq.sniff.ioinit)
    return TWIC_STATUS_ERROR_IO;

  return TWIC_STATUS_OK;
}

twicStatus_t twicSsLeIoFinalize(const bool stop_osc32K)
{
  twicStatus_t status;

  status = (twicStatus_t)tz1smHalLePmuStop(stop_osc32K);
  if (TWIC_STATUS_OK != status)
    return status;

  tz1smHalGpioUnInit();
  tz1smHalUartUnInit();
  tz1smHalDebugUartUnInit();

  return TWIC_STATUS_OK;
}

#if defined(TZ1EM_POWER_PROFILE)
static void twicSsLeTz1emCosc(void)
{
  tz1emTemporaryWithdraw(TZ1EM(BLE_NC_ADV));
  tz1emTemporaryWithdraw(TZ1EM(BLE_CN_ADV));
  tz1emTemporaryWithdraw(TZ1EM(BLE_IDLE));
  tz1emGoIntoTheShade(TZ1EM(BLE_CONN), false);
  twic_eq.sniff.lowpower_core_idle = false;
  twic_eq.sniff.lowpower_core_cosc = true;
  twic_eq.sniff.lowpower_core_adve = false;
  twic_eq.sniff.lowpower_core_ncad = false;
}

static void twicSsLeTz1emCnad(void)
{
  tz1emTemporaryWithdraw(TZ1EM(BLE_NC_ADV));
  tz1emTemporaryWithdraw(TZ1EM(BLE_CONN));
  tz1emTemporaryWithdraw(TZ1EM(BLE_IDLE));
  tz1emGoIntoTheShade(TZ1EM(BLE_CN_ADV), false);
  twic_eq.sniff.lowpower_core_idle = false;
  twic_eq.sniff.lowpower_core_cosc = false;
  twic_eq.sniff.lowpower_core_adve = true;
  twic_eq.sniff.lowpower_core_ncad = false;
}

static void twicSsLeTz1emIdle(void)
{
  tz1emTemporaryWithdraw(TZ1EM(BLE_NC_ADV));
  tz1emTemporaryWithdraw(TZ1EM(BLE_CN_ADV));
  tz1emTemporaryWithdraw(TZ1EM(BLE_CONN));
  tz1emGoIntoTheShade(TZ1EM(BLE_IDLE), false);
  twic_eq.sniff.lowpower_core_idle = true;
  twic_eq.sniff.lowpower_core_cosc = false;
  twic_eq.sniff.lowpower_core_adve = false;
  twic_eq.sniff.lowpower_core_ncad = false;
}

static void twicSsLeTz1emNcad(void)
{
  tz1emTemporaryWithdraw(TZ1EM(BLE_CN_ADV));
  tz1emTemporaryWithdraw(TZ1EM(BLE_IDLE));
  tz1emTemporaryWithdraw(TZ1EM(BLE_CONN));
  tz1emGoIntoTheShade(TZ1EM(BLE_NC_ADV), false);
  twic_eq.sniff.lowpower_core_idle = false;
  twic_eq.sniff.lowpower_core_cosc = false;
  twic_eq.sniff.lowpower_core_adve = false;
  twic_eq.sniff.lowpower_core_ncad = true;
}

static void twicSsleTz1emStandard(void)
{
  tz1emParticipateIn(TZ1EM(BLE_NC_ADV));
  tz1emParticipateIn(TZ1EM(BLE_CN_ADV));
  tz1emParticipateIn(TZ1EM(BLE_IDLE));
  tz1emParticipateIn(TZ1EM(BLE_CONN));
}

static void twicSsIsrLeTz1emCosc(void)
{
  tz1emIsrTemporaryWithdraw(TZ1EM(BLE_NC_ADV));
  tz1emIsrTemporaryWithdraw(TZ1EM(BLE_CN_ADV));
  tz1emIsrTemporaryWithdraw(TZ1EM(BLE_IDLE));
  tz1emIsrGoIntoTheShade(TZ1EM(BLE_CONN), false);
  twic_eq.sniff.lowpower_core_idle = false;
  twic_eq.sniff.lowpower_core_cosc = true;
  twic_eq.sniff.lowpower_core_adve = false;
  twic_eq.sniff.lowpower_core_ncad = false;
}

static void twicSsIsrLeTz1emCnad(void)
{
  tz1emIsrTemporaryWithdraw(TZ1EM(BLE_NC_ADV));
  tz1emIsrTemporaryWithdraw(TZ1EM(BLE_CONN));
  tz1emIsrTemporaryWithdraw(TZ1EM(BLE_IDLE));
  tz1emIsrGoIntoTheShade(TZ1EM(BLE_CN_ADV), false);
  twic_eq.sniff.lowpower_core_idle = false;
  twic_eq.sniff.lowpower_core_cosc = false;
  twic_eq.sniff.lowpower_core_adve = true;
  twic_eq.sniff.lowpower_core_ncad = false;
}

static void twicSsIsrLeTz1emIdle(void)
{
  tz1emIsrTemporaryWithdraw(TZ1EM(BLE_NC_ADV));
  tz1emIsrTemporaryWithdraw(TZ1EM(BLE_CN_ADV));
  tz1emIsrTemporaryWithdraw(TZ1EM(BLE_CONN));
  tz1emIsrGoIntoTheShade(TZ1EM(BLE_IDLE), false);
  twic_eq.sniff.lowpower_core_idle = true;
  twic_eq.sniff.lowpower_core_cosc = false;
  twic_eq.sniff.lowpower_core_adve = false;
  twic_eq.sniff.lowpower_core_ncad = false;
}

static void twicSsIsrLeTz1emNcad(void)
{
  tz1emIsrTemporaryWithdraw(TZ1EM(BLE_CN_ADV));
  tz1emIsrTemporaryWithdraw(TZ1EM(BLE_IDLE));
  tz1emIsrTemporaryWithdraw(TZ1EM(BLE_CONN));
  tz1emIsrGoIntoTheShade(TZ1EM(BLE_NC_ADV), false);
  twic_eq.sniff.lowpower_core_idle = false;
  twic_eq.sniff.lowpower_core_cosc = false;
  twic_eq.sniff.lowpower_core_adve = false;
  twic_eq.sniff.lowpower_core_ncad = true;
}
#endif

void twicSsLeRequestHighPower(void)
{
#if defined(TZ1EM_POWER_PROFILE)
  if (true == twic_eq.sniff.lowpower_core) twicSsleTz1emStandard();
#endif

#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
  if (true == twic_eq.sniff.lowpower_peripheral_h4) {
    tz1smHalUartLowPower(false);
    twic_eq.sniff.lowpower_peripheral_h4 = false;
    return;
  }
#endif
  /* request to wake and prevent it from dozing */
  tz1smHalGpioBleWakeUp(true);
}

#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
static bool twicSsLeFindSniffState(bool cons, bool cnad, bool ncad)
{
  twicInstance_t *p;
#if defined(TZ1EM_POWER_PROFILE)
  bool enable = twic_eq.sniff.lowpower_core;
#endif

  tz1utListEach(twicInstance_t, p, TZ1UT_LIST(srv_instance_root), sibling) {
    if (true == cons && (true == p->opmd.conn || true == p->opmd.scan)) {
#if defined(TZ1EM_POWER_PROFILE)
      if (0 == p->group.u16 && 0 == p->token && true == enable) {
        twicSsLeTz1emCosc();
#if defined(TWIC_SS_DOZE_DEBUG)
        twicPrintf("Allow to doze.\r\n");
#endif
      }
#endif
      return true;
    }
    if (true == cnad && (true == p->opmd.adve && false == p->opmd.ncad)) {
#if defined(TZ1EM_POWER_PROFILE)
      if (true == enable) twicSsLeTz1emCnad();
#endif
      return true;
    }
#if defined(TZ1EM_POWER_PROFILE)
    if (true == ncad && (true == p->opmd.adve && true == p->opmd.ncad)) {
      twicSsLeTz1emNcad();
      return true;
    }
#endif
  }

  return false;
}
#endif

#if defined(TZ1EM_POWER_PROFILE)
static bool twicSsIsrLeFindSniffState(bool cons, bool cnad, bool ncad)
{
  twicInstance_t *p;
  bool enable = twic_eq.sniff.lowpower_core;

  tz1utListEach(twicInstance_t, p, TZ1UT_LIST(srv_instance_root), sibling) {
    if (true == cons && (true == p->opmd.conn || true == p->opmd.scan)) {
      if (0 == p->group.u16 && 0 == p->token && true == enable) {
        twicSsIsrLeTz1emCosc();
#if defined(TWIC_SS_DOZE_DEBUG)
        twicPrintf("Allow to doze.\r\n");
#endif
      }
      return true;
    }
    if (true == cnad && (true == p->opmd.adve && false == p->opmd.ncad)) {
      if (true == enable) twicSsIsrLeTz1emCnad();
      return true;
    }
    if (true == ncad && (true == p->opmd.adve && true == p->opmd.ncad)) {
      twicSsIsrLeTz1emNcad();
      return true;
    }
  }

  return false;
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t twicSsLeRequestLowPower(void)
{
  if (false == twic_eq.sniff.lowpower_enable) {
    return TWIC_STATUS_OPERATION_NOT_PERMITTED;
  }
  if (!(tz1utListIsEmpty(TZ1UT_LIST(srv_req_root)))
      || true == twic_eq.sniff.waiting_for_activation
      || true == tz1smHalUartDataAvailable()
      || false == tz1smHalGpioBleHostWakeupStatus()) {
    return TWIC_STATUS_UNDER_PROCESSING;
  }
#if defined(TWIC_LECE_CLK32K_SLPXOIN_CLOCK) || defined(TWIC_LECE_CLK32K_SLPXOIN_XTAL)
  tz1smHalGpioBleWakeUp(false);
#endif

#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
  if (false == twic_eq.sniff.lowpower_peripheral_h4) {
    if (true == twicSsLeFindSniffState(true, false, false))
      return TWIC_STATUS_OK;
    if (true == twicSsLeFindSniffState(false, true, false))
      return TWIC_STATUS_OK;
  }

  tz1smHalUartLowPower(true);
  twic_eq.sniff.lowpower_peripheral_h4 = true;

#if defined(TZ1EM_POWER_PROFILE)
  if (true == twic_eq.sniff.lowpower_core) {
    if (true == twicSsLeFindSniffState(false, false, true))
        return TWIC_STATUS_OK;
    twicSsLeTz1emIdle();
  }
#endif /* TZ1EM_POWER_PROFILE */

#endif /* TWIC_MCU_PERIPHERAL_POWER_PROFILE */

  return TWIC_STATUS_OK;
}
#endif

bool twicSsLeLowPowerStatus(void)
{
  if (true == twic_eq.sniff.lowpower_enable)
    return tz1smHalGpioBleLowpowerStatus();

  return false;
}

bool twicSsLeHostLowPowerStatus(void)
{
  return tz1smHalGpioBleHostLowpowerStatus();
}

void twicSsLeReset(const bool reset)
{
  tz1smHalStatus_t status;

  status = tz1smHalGpioBleReset(reset);
  if (status != TZ1SM_HAL_STATUS_OK) {
    twicLog("ERROR: tz1smHalGpioBleReset %d\r\n", status);
  }
}

void twicSsHashCreate(void)
{
  twicHashCreate(TWIC_ENTRY_HASH_SYSTEM_INIT);
}

static TZ1K_INLINE void twicSsGattCleanInstance(uint8_t interface)
{
  twicInstance_t *p, *q = NULL;

  tz1utListEach(twicInstance_t, p, TZ1UT_LIST(srv_instance_root), sibling) {
    if (p->interface == interface) {
      if (q) tz1utListInit(&q->sibling);
      twicHashCleanupInterface(p->interface);
      tz1utListDel(&p->sibling);
      q = p;
    }
  }
  if (q) tz1utListInit(&q->sibling);
}

void twicSsGattRegistration(twicEntry_t * const conn)
{
  twicSsGattCleanInstance(conn->instance->interface);
  tz1utListAddTail(&conn->instance->sibling, TZ1UT_LIST(srv_instance_root));
}

#if defined(TWICSSGATTTERMINATE)
static void twicSsGattTerminate(void)
{
  return;
}
#endif

void twicSsGattDeregistration(const uint8_t interface)
{
  twicSsGattCleanInstance(interface);
#if defined(TWICSSGATTTERMINATE)
  if (tz1utListIsEmpty(TZ1UT_LIST(srv_instance_root))) {
    twicSsGattTerminate();
  }
#endif
}

static uint16_t twicSsLeHci(
  const uint16_t op_code, const uint16_t cid, const uint16_t length,
  const uint8_t req, twicEntry_t * const entry, const uint8_t seq)
{
  twic_container.u.hci.type = TWIC_HCI_COMMAND_PACKET;
  twic_container.u.hci.op_code = op_code;
  twic_container.u.hci.len = length;
  entry->life = 0;
  entry->req = req;
  entry->id = cid;
  entry->seq = seq;
  entry->u.cont.len = 0;
  tz1utListDel(&entry->o_link);
  tz1utListAddTail(&entry->o_link, TZ1UT_LIST(srv_req_root));

  return sizeof(twicHm_t) + length;
}

static uint16_t
twicSsLeSetupContainerCm(const uint8_t op_code, const uint8_t service_id,
                         const uint16_t length, const uint8_t req,
                         twicEntry_t * const entry, const uint8_t seq)
{
  uint16_t whole_length = length + sizeof(twicHx_t) + sizeof(twicCm_t);
  uint16_t cid = (uint16_t)service_id << 8 | op_code;

  TWIC_SETHARF_LE(twic_container.header.len, whole_length);
  twic_container.header.reserved = 0;
  twic_container.u.tcu.op_code = op_code;
  TWIC_SETHARF_LE(twic_container.u.tcu.len, length);
  tz1utListDel(&entry->o_link);
  entry->life = 0;
  entry->req = req;
  entry->id = cid;
  entry->seq = seq;
  entry->u.cont.len = 0;
  twic_container.u.tcu.service_id = service_id;
  tz1utListAddTail(&entry->o_link, TZ1UT_LIST(srv_req_root));

  return whole_length;
}

static TZ1K_INLINE twicStatus_t twicSsLeRetryRequest(twicEntry_t * const entry)
{
  volatile bool lowpower = twicSsLeLowPowerStatus();

  if (false == lowpower) return TWIC_STATUS_OK;

  /* BLE Controller is dozeing. The transmitted or queued message must
   * be removed. */
  entry->life = 0;
  entry->req = 0;
  entry->rsp = 0xFF;
  entry->id = 0;
  entry->seq = 0;
  entry->u.cont.len = 0;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);

  return TWIC_STATUS_WAITING_FOR_ACTIVATION;
}

static twicStatus_t
twicSsLeTxDataCm(twicEntry_t * const conn, const uint16_t len)
{
  twicStatus_t ret;

#if defined(TWIC_LECE_LOWPOWER)
  ret = twicSsLeRetryRequest(conn);
  if (TWIC_STATUS_OK == ret) {
    twic_eq.sniff.waiting_for_activation = false;
    twic_eq.slpto = 0;
  } else {
    /* BLE Controller is dozeing yet. */
    twic_eq.sniff.waiting_for_activation = true;
    if (RPQUEUE_LPRTO > twic_eq.slpto ++)
      return ret;
    twic_eq.slpto = 0;
    /* BLE Controller has dozed in the slit.
     * Send negative edge signal to the controller. */
    twic_eq.sniff.waiting_for_activation = false;
    twicSsLeRequestLowPower();

    return ret;
  }
#endif

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container, len);
#endif

  do
    ret = (twicStatus_t)tz1smHalUartPostData((uint8_t *)&twic_container, len);
  while (ret != TWIC_STATUS_OK);

#if defined(TWIC_LECE_LOWPOWER)
  ret = twicSsLeRetryRequest(conn);
  /* BLE Controller has dozed in the slit.
   * Again send negative edge signal to the controller. */
  if (TWIC_STATUS_OK != ret) {
    twicSsLeRequestLowPower();
    twicLog("Critical Doze !!\r\n");
  }
#endif

  return ret;
}

twicStatus_t
twicSsLeCePatch(const uint8_t seq, twicEntry_t * const conn, const uint8_t m2,
                const uint8_t length, const uint8_t * const code,
                const uint8_t reg, const bool enable)
{
  uint16_t len;
  tz1smHalStatus_t ret;
  uint8_t com[] = {0x00, 0xa0, 0x00, 0x00, 0x00, 0x14, 0x00, 0xff};

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  com[6] = m2;
  memcpy(twic_container.param, com, 8);
  if (TWIC_ENTRY_MNG_PC == m2) {
    twic_container.param[8] = 0x02;
    twic_container.param[9] = reg;
    twic_container.param[10] = (true == enable) ? 1 : 0;
    len = twicSsLeHci(0xfc08, 0xa000, 11, m2, conn, seq);
  } else {
    memcpy(twic_container.param + 8, code, length);
    len = twicSsLeHci(0xfc08, 0xa000, length + 8, m2, conn, seq);
  }

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}

#if defined(TWIC_API_LECEREADFWVER)
twicStatus_t twicSsLeCeReadFwVer(const uint8_t seq, twicEntry_t * const conn)
{
  uint16_t len;
  tz1smHalStatus_t ret;
  const uint8_t cmd[] = {0, 0xa1, 0, 0, 0, 0x14, TWIC_ENTRY_MNG_FV, 0xff, 0};

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  memcpy(twic_container.param, cmd, 0x09);
  len = twicSsLeHci(0xfc08, 0xa100, 9, TWIC_ENTRY_MNG_FV, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}
#endif

#if defined(TWIC_API_LECEDBUSWRITE)
twicStatus_t twicSsLeCeDbusWrite(const uint8_t seq, twicEntry_t * const conn,
                                 const uint8_t addr, const uint16_t data)
{
  uint16_t len;
  tz1smHalStatus_t ret;

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twic_container.param[0] = 0x00;
  twic_container.param[1] = TWIC_ENTRY_MNG_DW;
  twic_container.param[0x02] = addr;
  TWIC_SETHARF_LE(twic_container.param + 0x03, data);
  len = twicSsLeHci(0xfc03, 0xc200, 0x05, TWIC_ENTRY_MNG_DW, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}
#endif

#if defined(TWIC_API_LECEDBUSREAD)
twicStatus_t twicSsLeCeDbusRead(const uint8_t seq, twicEntry_t * const conn,
                                const uint8_t addr)
{
  uint16_t len;
  tz1smHalStatus_t ret;

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twic_container.param[0] = 0x00;
  twic_container.param[1] = TWIC_ENTRY_MNG_DR;
  twic_container.param[0x02] = addr;
  len = twicSsLeHci(0xfc03, 0xc300, 0x03, TWIC_ENTRY_MNG_DR, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}
#endif

#if defined(TWIC_API_LECEMEMORYREAD)
twicStatus_t twicSsLeCeMemoryRead(const uint8_t seq, twicEntry_t * const conn,
                                  const uint32_t addr)
{
  uint16_t len;
  tz1smHalStatus_t ret;

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twic_container.param[0] = 0x00;
  twic_container.param[1] = TWIC_ENTRY_MNG_MR;
  TWIC_SETWORD_LE(twic_container.param + 0x02, addr);
  twic_container.param[6] = 0x02;
  len = twicSsLeHci(0xfc03, 0xd000, 0x07, TWIC_ENTRY_MNG_MR, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}
#endif

#if defined(TWIC_API_LECEMEMORYWRITE)
twicStatus_t twicSsLeCeMemoryWrite(const uint8_t seq, twicEntry_t * const conn,
                                   const uint32_t addr, const uint16_t data)
{
  uint16_t len;
  tz1smHalStatus_t ret;

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twic_container.param[0] = 0x00;
  twic_container.param[1] = TWIC_ENTRY_MNG_MW;
  TWIC_SETWORD_LE(twic_container.param + 0x02, addr);
  TWIC_SETHARF_LE(twic_container.param + 0x06, data);
  len = twicSsLeHci(0xfc03, 0xd100, 0x08, TWIC_ENTRY_MNG_MW, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}
#endif

#if defined(TWIC_API_LECEXOSCTRIMING)
twicStatus_t twicSsLeCeXoscTriming(const uint8_t seq, twicEntry_t * const conn,
                                   const uint8_t data)
{
  uint16_t len;
  tz1smHalStatus_t ret;
  uint8_t com[] = {0x00, 0xa0, 0x00, 0x00, 0x00, 0x14, TWIC_ENTRY_MNG_MA, 0xff};

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  memcpy(twic_container.param, com, 8);
  twic_container.param[8] = 0x10;
  twic_container.param[9] = 0x06;
  TWIC_SETWORD_LE(&twic_container.param[10], 0x00061652);
  twic_container.param[14] = data;
  twic_container.param[15] = 0x00;
  len = twicSsLeHci(0xfc08, 0xa000, 16, TWIC_ENTRY_MNG_MA, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}
#endif

#if defined(TWIC_API_LECESETTXPOWER)
/* 0x00: Reserved,   0x01: 0dBm,   0x02: -4dBm,   0x03: -8dBm,
   0x04: -12dBm,   0x05: -16dBm,   0x06: -20dBm */
twicStatus_t
twicSsLeCeSetTxPower(const uint8_t seq, twicEntry_t * const conn, const uint8_t dbm_idx)
{
  uint16_t len;
  tz1smHalStatus_t ret;
  uint8_t com[] = {0x00, 0xa0, 0x00, 0x00, 0x00, 0x14, TWIC_ENTRY_MNG_SET_TX_POWER, 0xff};

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  memcpy(twic_container.param, com, 8);
  twic_container.param[8] = 0x10;
  twic_container.param[9] = 0x01;
  twic_container.param[10] = dbm_idx;
  len = twicSsLeHci(0xfc08, 0xa000, 11, TWIC_ENTRY_MNG_SET_TX_POWER, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}
#endif

twicStatus_t twicSsLeCeFlowControl(const uint8_t seq,
                                   twicEntry_t * const conn, const bool enable)
{
  uint16_t len;
  tz1smHalStatus_t ret;
  const uint8_t cmd[] = { /* M2 0x01,0x08,0xfc,0x0a, */
    0, 0xa0, 0, 0, 0, 0x14, TWIC_ENTRY_MNG_FC, 0xff, 0x01, 0xff};

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  memcpy(twic_container.param, cmd, 0x0a);
  if (false == enable) {
    /* check if it's ok whenever ROM is updated. */
    twic_container.param[0x0a - 1] = 0;
  }
  len = twicSsLeHci(0xfc08, 0xa000, 0x0a, TWIC_ENTRY_MNG_FC, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartSendData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_eq.sniff.fc_rq = enable;
  ret = tz1smHalUartControl(twic_eq.br, twic_eq.sniff.fc_rq);
  if (TZ1SM_HAL_STATUS_OK != ret) twic_eq.h4 |= RPQUEUE_H4_HAL_ERROR;
  else twic_eq.h4 &= ~RPQUEUE_H4_HAL_OK;

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return (ret == TZ1SM_HAL_STATUS_OK) ? TWIC_STATUS_OK : TWIC_STATUS_ERROR_HAL;
}

static bool _twicSsLeCeSetBaudrate(
  const twicTzbtBr_t br, const uint16_t pu, const uint8_t len)
{
  uint8_t sample;

  switch (br) {
  case TWIC_TZBT_0768: twic_eq.br_rq =  76800; sample = 0x60; break; //13
  case TWIC_TZBT_1152: twic_eq.br_rq = 115200; sample = 0x80; break; //15
  case TWIC_TZBT_1536: twic_eq.br_rq = 153600; sample = 0x60; break; //13
  case TWIC_TZBT_2304: twic_eq.br_rq = 230400; sample = 0x90; break; //16
  case TWIC_TZBT_3072: twic_eq.br_rq = 307200; sample = 0xA0; break; //17
  case TWIC_TZBT_4608: twic_eq.br_rq = 460800; sample = 0x70; break; //14
  case TWIC_TZBT_9216: twic_eq.br_rq = 921600; sample = 0x70; break; //14
  default: return false;
  }
  TWIC_SETHARF_LE(twic_container.param + (len - 9), br); /* divider */
  twic_container.param[len - 7] = sample;                /* sampling */
  TWIC_SETHARF_BE(twic_container.param + (len - 2), pu); /* punctuation */
  return true;
}

twicStatus_t
twicSsLeCeSetBaudrateHc(const uint8_t seq, twicEntry_t * const conn,
                        const twicTzbtBr_t br, const uint16_t pu)
{
  uint16_t len;
  tz1smHalStatus_t ret;
  const uint8_t cmd[] = { /* M2 0x01,0x08,0xfc,0x13, */
    0,0xa0,0,0,0,0x14,TWIC_ENTRY_MNG_BR,0xff,0x10,9,0,0,0,0x01,0,0,0,0,0 };

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  memcpy(twic_container.param, cmd, 0x13);

  if (false == _twicSsLeCeSetBaudrate(br, pu, 0x13)) {
    return TWIC_STATUS_ERROR_PARAMETER;
  }

  len = twicSsLeHci(0xfc08, 0xa000, 0x13, TWIC_ENTRY_MNG_BR, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartSendData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}

twicStatus_t
twicSsLeCeSetBaudrateCm(const uint8_t seq, twicEntry_t * const conn,
                        const twicTzbtBr_t br, const uint16_t pu)
{
  uint16_t len;
  twicStatus_t ret;
  const uint8_t cmd[] = { /* M2 0x01,0x08,0xfc,0x13, */
    0x01,TWIC_ENTRY_MNG_BR,0xff,0x10,0x09,0,0,0,0x01,0,0,0,0,0 };

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  memcpy(twic_container.param, cmd, 0x0E);
  if (false == _twicSsLeCeSetBaudrate(br, pu, 0x0E)) {
    return TWIC_STATUS_ERROR_PARAMETER;
  }

  len = twicSsLeSetupContainerCm(0x01, TWIC_SERVICE_ID_VENDOR, 0x0D,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);
  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

static twicStatus_t
twicSsLeCeHostDelayHc(const uint8_t seq,
                      twicEntry_t * const conn, const uint8_t delay)
{
  uint16_t len;
  tz1smHalStatus_t ret;
  const uint8_t cmd[] = { /* M2 0x01,0x08,0xfc,0x0b, */
    0,0xa0,0,0,0,0x14,TWIC_ENTRY_MNG_HOST_DELAY,0xff,0x02 };

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  memcpy(twic_container.param, cmd, 0x09);
  TWIC_SETHARF_LE(twic_container.param + 0x09, delay);
  len = twicSsLeHci(0xfc08, 0xa000, 11, TWIC_ENTRY_MNG_HOST_DELAY, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  if (RPQUEUE_PM_HDO > delay) twic_eq.hdr = delay + RPQUEUE_PM_HDO;
  else twic_eq.hdr = delay;

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}

static twicStatus_t
twicSsLeCeHostDelayCm(const uint8_t seq,
                      twicEntry_t * const conn, const uint8_t delay)
{
  uint16_t len;
  twicStatus_t ret;
  const uint8_t cmd[] = { /* M2 0x01,0x08,0xfc,0x0b, */
    0x01,TWIC_ENTRY_MNG_HOST_DELAY,0xff,0x02 };

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  memcpy(twic_container.param, cmd, 0x04);
  TWIC_SETHARF_LE(twic_container.param + 0x04, delay);
  len = twicSsLeSetupContainerCm(
    0x01, TWIC_SERVICE_ID_VENDOR, 0x06, TWIC_ENTRY_MNG_TCU, conn, seq);
  ret = twicSsLeTxDataCm(conn, len);

  if (TWIC_STATUS_OK == ret) {
    if (RPQUEUE_PM_HDO > delay) twic_eq.hdr = delay + RPQUEUE_PM_HDO;
    else twic_eq.hdr = delay;
  }

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsLeCeSetBaudrate(const uint8_t seq, twicEntry_t * const conn,
                                   const twicTzbtBr_t br, const uint16_t pu)
{
  if (true == twic_eq.sniff.lowpower_requirements)
    return TWIC_STATUS_OPERATION_NOT_PERMITTED;
  else if (TWIC_HCI == twic_eq.type)
    return twicSsLeCeSetBaudrateHc(seq, conn, br, pu);
  else return twicSsLeCeSetBaudrateCm(seq, conn, br, pu);
}

twicStatus_t twicSsLeCeHostDelay(const uint8_t seq,
                                 twicEntry_t * const conn, const uint16_t delay)
{
  if (RPQUEUE_PM_HDU <= delay) return TWIC_STATUS_ERROR_PARAMETER;
  else if (TWIC_HCI == twic_eq.type)
    return twicSsLeCeHostDelayHc(seq, conn, delay);
  else return twicSsLeCeHostDelayCm(seq, conn, delay);
}

#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t twicSsLeCeLowPowerInit(const uint8_t seq, twicEntry_t * const conn,
                                    const uint32_t osc_wait,
                                    const uint8_t deep_sleep_control,
                                    const uint16_t osc_tolerance,
                                    const uint16_t osc_jitter,
                                    const uint16_t premature_wake_time)
{
  uint16_t len;
  tz1smHalStatus_t ret;
  const uint8_t cmd[] = { /* M2 0x01,0x08,0xfc,0x1c, */
    0,0xa0,0,0,0,0x14,TWIC_ENTRY_MNG_POWER_INIT,0xff,0x10,0x12,0x00 };

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  memcpy(twic_container.param, cmd, 0x0b);
  /* Oscillation stabilization wait time */
  TWIC_SETWORD_LE(twic_container.param + 0x0b, osc_wait);
  twic_container.param[0x0f] = deep_sleep_control;
  TWIC_SETHARF_LE(twic_container.param + 0x10, osc_tolerance); /* ppm */
  TWIC_SETHARF_LE(twic_container.param + 0x12, osc_jitter); /* us */
  TWIC_SETHARF_LE(twic_container.param + 0x14, premature_wake_time);
  TWIC_SETHARF_LE(twic_container.param + 0x16, 0x0);
  TWIC_SETHARF_LE(twic_container.param + 0x18, 0x0);
  TWIC_SETHARF_LE(twic_container.param + 0x1a, 0x0);
  len = twicSsLeHci(0xfc08, 0xa000, 0x1c, TWIC_ENTRY_MNG_POWER_INIT, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t twicSsLeCeLowPowerClock(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t source, const uint16_t osc_wait, twicStatus_t * const osc_state)
{
  uint16_t len;
  tz1smHalStatus_t ret;
  const uint8_t cmd[] = { /* M2 0x01,0x08,0xfc,0x0d, */
    0,0xa0,0,0,0,0x14,TWIC_ENTRY_MNG_POWER_CLOCK,0xff,0x10,0x03 };

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  ret = tz1smHalLePmuClockSourceState32K();
  if (NULL != osc_state) *osc_state = (twicStatus_t)ret;
  if (TZ1SM_HAL_STATUS_SOURCE_RUNNING != ret &&
      TZ1SM_HAL_STATUS_IGNORE != ret) {
    twic_container.u.hci.type = TWIC_HCI_NOP;
    return TWIC_STATUS_UNDER_PROCESSING;
  }

  memcpy(twic_container.param, cmd, 0x0a);
  twic_container.param[0x0a] = source;
  TWIC_SETHARF_LE(twic_container.param + 0x0b, osc_wait);
  len = twicSsLeHci(0xfc08, 0xa000, 13, TWIC_ENTRY_MNG_POWER_CLOCK, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
static TZ1K_INLINE twicStatus_t
twicSsLeCeLowPowerModeHc(const uint8_t seq, twicEntry_t * const conn,
                         const uint8_t selector)
{
  uint16_t len;
  tz1smHalStatus_t ret;
  const uint8_t cmd[] = { /* M2 0x01,0x08,0xfc,0x0a, */
    0,0xa0,0,0,0,0x14,TWIC_ENTRY_MNG_POWER_MODE,0xff,0x01 };

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  memcpy(twic_container.param, cmd, 0x09);
  twic_container.param[0x09] = selector;
  len = twicSsLeHci(0xfc08, 0xa000, 10, TWIC_ENTRY_MNG_POWER_MODE, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_eq.lowpower_selector = selector;

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
static TZ1K_INLINE twicStatus_t
twicSsLeCeLowPowerModeCm(const uint8_t seq, twicEntry_t * const conn,
                         const uint8_t selector)
{
  uint16_t len;
  twicStatus_t ret;
  const uint8_t cmd[] = { /* M2 0x01,0x08,0xfc,0x0a, */
    0x01,TWIC_ENTRY_MNG_POWER_MODE,0xff,0x01  };

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  memcpy(twic_container.param, cmd, 0x04);
  twic_container.param[0x04] = selector;
  len = twicSsLeSetupContainerCm(
    0x01, TWIC_SERVICE_ID_VENDOR, 0x05, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);
  if (TWIC_STATUS_OK == ret) twic_eq.lowpower_selector = selector;

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
void twicSsLeCeHkLowPowerInfo(twicCeLeLpInfo_t * const info)
{
  uint8_t md = twic_eq.lowpower_mode;
  bool h4 = twic_eq.sniff.lowpower_peripheral_h4;

  info->h_rt = twicSsLeHostLowPowerStatus();
  info->c_rt = twicSsLeLowPowerStatus(); /* true: doze */
  info->c_en = (bool)twic_eq.sniff.lowpower_enable;
  info->c_cs = ((md & RPQUEUE_PM_SLEEP)) ? true : false;
  info->c_ad = ((md & RPQUEUE_PM_BACKUP1)) ? true : false;
  info->c_id = ((md & RPQUEUE_PM_BACKUP2)) ? true : false;
  info->c_sd = ((md & RPQUEUE_PM_DEEP_SLEEP)) ? true : false;
#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
  info->h_id = twic_eq.sniff.lowpower_core_idle;
  info->h_cs = twic_eq.sniff.lowpower_core_cosc;
  info->h_ca = twic_eq.sniff.lowpower_core_adve;
  info->h_na = twic_eq.sniff.lowpower_core_ncad;
  info->h_tb = ((true == h4)) ? true : false;
#endif
}
#endif

twicStatus_t twicSsLeCeHkTimeout(
  const bool cleanup, const uint16_t cycle, const uint8_t specified_fidx)
{
  twicEntry_t *p, *q = NULL;
  twicStatus_t status = TWIC_STATUS_OK;

  tz1utListEach(twicEntry_t, p, TZ1UT_LIST(srv_req_root), o_link) {
    if ((true == cleanup) ||
        (0 < specified_fidx && specified_fidx == p->seq)) {
      if (++ p->life >= cycle) {
        if (q) tz1utListInit(&q->o_link);
        p->rsp = p->seq;
        p->result = 0xAA;
        tz1utListDel(&p->o_link);
        q = p;
        status = TWIC_STATUS_COMMAND_ELIMINATION;
      }
    }
  }
  if (q) tz1utListInit(&q->o_link);
#if defined(TWIC_LECE_LOWPOWER)
  if (TWIC_STATUS_COMMAND_ELIMINATION == status)
    twicSsLeRequestLowPower();
#endif
  return status;
}

#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t twicSsLeCeLowPowerMode(
  const uint8_t seq, twicEntry_t * const conn, const uint8_t selector)
{
  return (TWIC_HCI == twic_eq.type) ?
    twicSsLeCeLowPowerModeHc(seq, conn, selector):
    twicSsLeCeLowPowerModeCm(seq, conn, selector);
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
static TZ1K_INLINE twicStatus_t
twicSsLeCeLowPowerControlPinHc(const uint8_t seq, twicEntry_t * const conn,
                               const uint8_t pin, const uint8_t status_port)
{
  uint16_t len;
  tz1smHalStatus_t ret;
  const uint8_t cmd[] = { /* M2 0x01,0x08,0xfc,0x0d, */
    0,0xa0,0,0,0,0x14,TWIC_ENTRY_MNG_PCP,0xff,0x03 };

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  memcpy(twic_container.param, cmd, 0x09);
  twic_container.param[0x09] = 0x00;
  twic_container.param[0x0a] = status_port;
  twic_container.param[0x0b] = pin;
  twic_container.param[0x0c] = 0x00;
  len = twicSsLeHci(0xfc08, 0xa000, 0x0d, TWIC_ENTRY_MNG_PCP, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  if (RPQUEUE_PM_PIN == pin && false == twic_eq.sniff.pin ||
      RPQUEUE_PM_PIN != pin && true == twic_eq.sniff.pin) {
    conn->rsp = conn->seq;
    conn->result = 0;
    conn->u.cont.len = 0;
    tz1utListDel(&conn->o_link);
    tz1utListInit(&conn->o_link);
    goto out;
  }

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  if (RPQUEUE_PM_PIN != status_port && RPQUEUE_PM_PIN != pin)
    twic_eq.sniff.lowpower_requirements = true;
  else
    twic_eq.sniff.lowpower_requirements = false;
  if (RPQUEUE_PM_PIN != pin) twic_eq.sniff.pin_rq = true;
  else twic_eq.sniff.pin_rq = false;

  out:
  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}

static TZ1K_INLINE twicStatus_t
twicSsLeCeLowPowerControlPinCm(const uint8_t seq, twicEntry_t * const conn,
                               const uint8_t pin, const uint8_t status_port)
{
  uint16_t len;
  twicStatus_t ret;
  const uint8_t cmd[] = { /* M2 0x01,0x08,0xfc,0x0d, */
    0x01,TWIC_ENTRY_MNG_PCP,0xff,0x03 };

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  memcpy(twic_container.param, cmd, 0x04);
  twic_container.param[0x04] = 0x00;
  twic_container.param[0x05] = status_port;
  twic_container.param[0x06] = pin;
  twic_container.param[0x07] = 0x00;
  len = twicSsLeSetupContainerCm(
    0x01, TWIC_SERVICE_ID_VENDOR, 0x08, TWIC_ENTRY_MNG_TCU, conn, seq);

  if (RPQUEUE_PM_PIN == pin && false == twic_eq.sniff.pin ||
      RPQUEUE_PM_PIN != pin && true == twic_eq.sniff.pin) {
    conn->rsp = conn->seq;
    conn->result = 0;
    conn->u.cont.len = 0;
    tz1utListDel(&conn->o_link);
    tz1utListInit(&conn->o_link);
    ret = TWIC_STATUS_OK;
    goto out;
  }

  ret = twicSsLeTxDataCm(conn, len);

  if (TWIC_STATUS_OK == ret) {
    if (RPQUEUE_PM_PIN != status_port && RPQUEUE_PM_PIN != pin)
      twic_eq.sniff.lowpower_requirements = true;
    else
      twic_eq.sniff.lowpower_requirements = false;
    if (RPQUEUE_PM_PIN != pin) twic_eq.sniff.pin_rq = true;
    else twic_eq.sniff.pin_rq = false;
  }

  out:
  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t
twicSsLeCeLowPowerControlPin(const uint8_t seq, twicEntry_t * const conn,
                             const uint8_t pin, const uint8_t status_port)
{
  if (TWIC_HCI == twic_eq.type)
    return twicSsLeCeLowPowerControlPinHc(seq, conn, pin, status_port);
  return twicSsLeCeLowPowerControlPinCm(seq, conn, pin, status_port);
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t twicSsLeCeLowPowerBackup(
  const uint8_t seq, twicEntry_t * const conn,
  const uint32_t parameter1, const uint32_t parameter2)
{
  uint16_t len;
  tz1smHalStatus_t ret;
  const uint8_t cmd[] = { /* M2 0x01,0x08,0xfc,0x13, */
    0,0xa0,0,0,0,0x14,TWIC_ENTRY_MNG_POWER_MODE,0xff,0x10,0x09,0x04 };

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  memcpy(twic_container.param, cmd, 0x0b);
  TWIC_SETWORD_LE(twic_container.param + 0x0b, parameter1);
  TWIC_SETWORD_LE(twic_container.param + 0x0f, parameter2);
  len = twicSsLeHci(0xfc08, 0xa000, 0x13, TWIC_ENTRY_MNG_POWER_MODE, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}
#endif

twicStatus_t twicSsLeReadBdaddr(const uint8_t seq, twicEntry_t * const conn)
{
  tz1smHalStatus_t ret;
  uint16_t len;

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  len = twicSsLeHci(0x1009, 0x1009, 0, TWIC_ENTRY_MNG_READ_BDADDR, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}

#if defined(TWIC_API_LEREADLOCALVERSIONINFORMATION)
twicStatus_t
twicSsLeReadLocalVersionInformation(const uint8_t seq, twicEntry_t * const conn)
{
  tz1smHalStatus_t ret;
  uint16_t len;

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  len = twicSsLeHci(0x1001, 0x1001, 0, TWIC_ENTRY_MNG_READ_LVI, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}
#endif

twicStatus_t twicSsLeWriteBdaddr(
  const uint8_t seq, twicEntry_t * const conn, const uint64_t * const bdaddr)
{
  tz1smHalStatus_t ret;
  uint16_t len;
  uint8_t idx;

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  for (idx = 0; idx < 0x06; idx++)
    twic_container.param[idx] = (uint8_t)(*bdaddr >> (idx * 8));
  len = twicSsLeHci(0x1013, 0x1013, 6, TWIC_ENTRY_MNG_WRITE_BDADDR, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}

twicStatus_t twicSsLeCeChangeToCm(const uint8_t seq, twicEntry_t * const conn)
{
  tz1smHalStatus_t ret;
  const uint8_t param[] = { 0x00,0x99,0x01 };
  uint16_t len;

  if (twic_container.u.hci.type != TWIC_HCI_NOP) {
#if defined(TWIC_SS_TRACE_HCI_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  memcpy(twic_container.param, param, 0x03);
  len = twicSsLeHci(
    0xfc08, 0x9900, 0x03, TWIC_ENTRY_MNG_TCUMODE, conn, seq);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container.u.hci, len);
#endif

  do
    ret = tz1smHalUartPostData((uint8_t *)&twic_container.u.hci, len);
  while (ret != TZ1SM_HAL_STATUS_OK);

  twic_container.u.hci.type = TWIC_HCI_NOP;

  return TWIC_STATUS_OK;
}

twicStatus_t
twicSsLeReadRemoteVersion(const uint8_t seq, twicEntry_t * const conn)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  len = twicSsLeSetupContainerCm(
    0x19, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 2,
    TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsLeCreateConnection(
  const uint8_t seq, twicEntry_t * const conn, uint16_t interval,
  uint16_t window, bool use_white_list, bool peer_bdaddr_is_random,
  const twicBdaddr_t *const peer_bdaddr, uint16_t conn_int_min,
  uint16_t conn_int_max, uint16_t slave_latency, uint16_t supervison_timeout,
  uint16_t min_ce_length, uint16_t max_ce_length, bool own_bdaddr_is_random)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (NULL != twicHashFind1stHandle(conn->handle)) {
    twicLog("handle exists.\r\n");
    return TWIC_STATUS_OPERATION_NOT_PERMITTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETWORD_LE(twic_container.param + 0, interval);
  TWIC_SETWORD_LE(twic_container.param + 2, window);
  twic_container.param[4] = (true == use_white_list ? 1 : 0);
  twic_container.param[5] = (true == peer_bdaddr_is_random ? 1 : 0);
  memcpy(twic_container.param + 6, peer_bdaddr->address, 0x06);
  TWIC_SETWORD_LE(twic_container.param + 12, conn_int_min);
  TWIC_SETWORD_LE(twic_container.param + 14, conn_int_max);
  TWIC_SETWORD_LE(twic_container.param + 16, slave_latency);
  TWIC_SETWORD_LE(twic_container.param + 18, supervison_timeout);
  TWIC_SETWORD_LE(twic_container.param + 20, min_ce_length);
  TWIC_SETWORD_LE(twic_container.param + 22, max_ce_length);
  twic_container.param[24] = (true == own_bdaddr_is_random ? 1 : 0);

  len = twicSsLeSetupContainerCm(
    0x0C, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 25,
    TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#if defined(TWIC_API_LEADDWHITELIST)
twicStatus_t
twicSsLeAddWhitelist(const uint8_t seq, twicEntry_t * const conn,
                     const bool random, const twicBdaddr_t * const bd)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  /* 1,0 = Random Device Address, Public Device Address */
  twic_container.param[0] = (true == random ? 1 : 0);
  memcpy(twic_container.param + 1, bd->address, 0x06);
  len = twicSsLeSetupContainerCm(0x06, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 7,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_CONFIG_ENABLE_SCAN)
twicStatus_t twicSsLeSetScanEnable(
  const uint8_t seq, twicEntry_t * const conn, const uint16_t interval,
  const uint16_t window, const uint8_t active_scanning,
  const uint8_t own_bdaddr_is_random, const uint8_t whitelist_enable,
  const uint8_t scan_enable, const uint8_t filter_duplicates)
{
  uint8_t *mover;
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, interval);
  TWIC_SETHARF_LE(twic_container.param + 2, window);
  mover = twic_container.param + 4;
  *mover++ = active_scanning;
  *mover++ = own_bdaddr_is_random;
  *mover++ = whitelist_enable;
  *mover++ = scan_enable;
  *mover++ = filter_duplicates;

  len = twicSsLeSetupContainerCm(0x0a, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 9,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_LEDELWHITELIST)
twicStatus_t
twicSsLeDelWhitelist(const uint8_t seq, twicEntry_t * const conn,
                     const bool random, const twicBdaddr_t * const bd)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();
  /* 1,0 = Random Device Address, Public Device Address */
  twic_container.param[0] = (true == random ? 1 : 0);
  memcpy(twic_container.param + 1, bd->address, 0x06);
  len = twicSsLeSetupContainerCm(0x07, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 7,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);
  ret = twicSsLeTxDataCm(conn, len);
  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_APP_LECLEAREWHITELIST)
twicStatus_t twicSsLeClearWhitelist(const uint8_t seq, twicEntry_t *const conn)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();
  len = twicSsLeSetupContainerCm(0x07, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 0,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);
  ret = twicSsLeTxDataCm(conn, len);
  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_LEREADWHITELISTSIZE)
twicStatus_t
twicSsLeReadWhitelistSize(const uint8_t seq, twicEntry_t * const conn)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  len = twicSsLeSetupContainerCm(0x05, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 0,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
twicStatus_t
twicSsLeReadRemoteUsedFeatures(const uint8_t seq, twicEntry_t * const conn)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  len = twicSsLeSetupContainerCm(0x0f, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT,
                                 2, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#if defined(TWIC_API_LEREADLOCALSUPPORTEDFEATURES)
twicStatus_t
twicSsLeReadLocalSupportedFeatures(const uint8_t seq, twicEntry_t * const conn)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  len = twicSsLeSetupContainerCm(0x02, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 0,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_LESETHOSTCHANNELCLASSIFICATION)
twicStatus_t
twicSsLeSetHostChannelClassification(const uint8_t seq,
                                     twicEntry_t * const conn,
                                     const twicChannelMap_t *const channel_map)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  memcpy(twic_container.param, channel_map->chm, 0x05);
  len = twicSsLeSetupContainerCm(0x10, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 5,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif

#if defined(TWIC_API_LEREADCHANNELMAP)
twicStatus_t twicSsLeReadChannelMap(const uint8_t seq, twicEntry_t * const conn)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  len = twicSsLeSetupContainerCm(0x11, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 2,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_LEREADSUPPORTEDSTATES)
twicStatus_t
twicSsLeReadSupportedStates(const uint8_t seq, twicEntry_t * const conn)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  len = twicSsLeSetupContainerCm(0x12, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT,
                                 0, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_LESETRANDOMADDRESS)
twicStatus_t
twicSsLeSetRandomAddress(const uint8_t seq, twicEntry_t * const conn,
                         const twicBdaddr_t * const bd)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  memcpy(twic_container.param, bd->address, 0x06);
  len = twicSsLeSetupContainerCm(0x04, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT,
                                 6, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_LELMRESOLVEBDADDR)
twicStatus_t
twicSsLeLmResolveBdaddr(const uint8_t seq, twicEntry_t * const conn,
                        const twicBdaddr_t * const bd,
                        const uint8_t num_of_irk, const twicIrk_t * irks)
{
  uint16_t parameter_length = sizeof(twicBdaddr_t) + 1 + 16 * num_of_irk;
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (TWIC_TX_BUF_MAX < parameter_length) {
    twicTrace();
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  memcpy(twic_container.param, bd->address, 0x06);
  twic_container.param[6] = num_of_irk;
  memcpy(twic_container.param + 7, irks->key, 16 * num_of_irk);
  len = twicSsLeSetupContainerCm(0x18, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT,
                                 parameter_length,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_LELMRESOLVEBDADDR)
twicStatus_t twicSsLeSmSetIrValue(const uint8_t seq, twicEntry_t * const conn,
                                  const twicIr_t * const ir)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  memcpy(twic_container.param + 2, ir->value, 16);
  len = twicSsLeSetupContainerCm(0x1b, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT,
                                 18, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
twicStatus_t twicSsLeSmEncrypt(const uint8_t seq, twicEntry_t * const conn,
                               const uint8_t * const key,
                               const uint8_t * const plain_text_data)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  memcpy(twic_container.param + 2, key, 16);
  memcpy(twic_container.param + 18, plain_text_data, 16);
  len = twicSsLeSetupContainerCm(0x1c, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT,
                                 34, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LELESETADVERTISINGDATA)
twicStatus_t
twicSsLeSetAdvertisingData(const uint8_t seq, twicEntry_t * const conn,
                           const uint8_t length, const uint8_t const * data)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  twic_container.param = length;
  memcpy(twic_container.param + 1, data, length);
  memset(&(twic_container.param + 1 + length), 0, 31 - length);

  len = twicSsLeSetupContainerCm(0x1d, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT,
                                 32, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_CONFIG_SM_INITIATOR)
twicStatus_t
twicSsLeSmMasPairingRequest(const uint8_t seq, twicEntry_t * const conn,
                            const uint8_t io_capability,
                            const uint8_t oob_data_flag,
                            const uint8_t auth_req,
                            const uint8_t max_enc_key_size,
                            const uint8_t init_key_dist,
                            const uint8_t resp_key_dist)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  twic_container.param[2] = io_capability;
  twic_container.param[3] = oob_data_flag;
  twic_container.param[4] = auth_req;
  twic_container.param[5] = max_enc_key_size;
  twic_container.param[6] = init_key_dist;
  twic_container.param[7] = resp_key_dist;

  len = twicSsLeSetupContainerCm(0x01, TWIC_SERVICE_ID_SMP_INITIATOR, 8,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t
twicSsLeSmMasStartEncryption(const uint8_t seq, twicEntry_t * const conn,
                             const twicEdiv_t *const ediv,
                             const twicRand_t *const rand,
                             const twicLtk_t *const ltk,
                             const uint8_t encryption_key_size)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  memcpy(twic_container.param + 2, ediv, 2);
  memcpy(twic_container.param + 4, rand, 8);
  memcpy(twic_container.param + 12, ltk, 16);
  twic_container.param[28] = encryption_key_size;

  len = twicSsLeSetupContainerCm(0x08, TWIC_SERVICE_ID_SMP_INITIATOR, 29,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

static TZ1K_INLINE void
twicSsLeSmMasPairingResponseEventCm(const twicRespD441Cm_t * const op_d441)
{
  const twicLeSmpICb_t *cb;
  twicPairingFeature_t *arg;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d441->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->pairing_response)) return;
  arg = (twicPairingFeature_t *)&conn->instance->resp.value;

  if (0 == op_d441->status) {
    arg->io_capability = op_d441->io_capability;
    arg->oob_data_present =
      (TWIC_SMP_OOB_DATA_PRESENT == op_d441->oob_data_flag) ? true : false;
    arg->auth_req_bonding = ((0x1 & op_d441->auth_req)) ? true : false;
    arg->auth_req_mitm_protection = ((0x4 & op_d441->auth_req)) ? true : false;
    arg->max_enc_key_size = op_d441->max_enc_key_size;

    arg->init_key_dist_enckey = (0x1 & op_d441->init_key_dist) ? true : false;
    arg->init_key_dist_idkey = (0x2 & op_d441->init_key_dist) ? true : false;
    arg->init_key_dist_sign = (0x4 & op_d441->init_key_dist) ? true : false;
    arg->resp_key_dist_enckey = (0x1 & op_d441->resp_key_dist) ? true : false;
    arg->resp_key_dist_idkey = (0x2 & op_d441->resp_key_dist) ? true : false;
    arg->resp_key_dist_sign = (0x4 & op_d441->resp_key_dist) ? true : false;
  } else {
    memset(arg, 0, sizeof(twicPairingFeature_t));
  }

  cb->pairing_response(conn->instance->epr, op_d441->status, arg);
}

/* In response to this event, Initiator shall generate
 * twicIfLeSmMasSecurityReject when user wishes to abort the pairing
 * request from slave device. */

static TZ1K_INLINE void
twicSsLeSmMasSecurityEventCm(const bool bonded_device,
                             const twicRespD4C2Cm_t * const op_d4c2)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  uint8_t auth_req_bonding, auth_req_mitm_protection;

  conn = twicHashFind1stHandle(op_d4c2->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb) || !(cb->security_request)) return;

  if (false == bonded_device) {
    conn->instance->group.b.smp_i_security_event = true;
#if defined(TWIC_SS_DOZE_DEBUG)
    twicPrintf("TWIC_TCU_LE_SMP_I_SECURITY_EVENT\r\n");
#endif
  } else {
    conn->instance->group.b.smp_i_bonding_enabled_event = true;
#if defined(TWIC_SS_DOZE_DEBUG)
    twicPrintf("TWIC_TCU_LE_SMP_I_BONDING_ENABLED_EVENT\r\n");
#endif
  }

  auth_req_bonding = ((0x1 & op_d4c2->auth_req)) ? true : false;
  auth_req_mitm_protection = ((0x4 & op_d4c2->auth_req)) ? true : false;

  cb->security_request(conn->instance->epr, bonded_device, auth_req_bonding,
                       auth_req_mitm_protection);
}

static TZ1K_INLINE void
twicSsLeSmMasOobKeyEntryRequestEventCm(const twicRespD458Cm_t * const op_d458)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d458->conn_handle);
  if (!(conn) || !(conn->instance)) return;

  cb = conn->instance->smp_i_cb;
  if (!(cb) || !(cb->oob_information)) return;

  conn->instance->group.b.smp_i_oob_key_entry_req_event = true;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_SMP_I_OOB_KEY_ENTRY_REQ_EVENT\r\n");
#endif
  cb->oob_information(conn->instance->epr);
}

static TZ1K_INLINE void
twicSsLeSmMasStoreKeyEventCm(const twicRespD4D8Cm_t * const op_d4d8)
{
  const twicLeSmpICb_t *cb;
  twicBdaddr_t *arg;
  twicEntry_t *conn;
  bool erase;
  bool random;

  conn = twicHashFind1stHandle(op_d4d8->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->store_bonding_information)) return;
  arg = (twicBdaddr_t *)&conn->instance->resp.value;
  memcpy(arg, &op_d4d8->bd_address, 6);
  erase = (0x02 == op_d4d8->status) ? true : false;
  random = (0x01 == op_d4d8->address_type) ? true : false;
  cb->store_bonding_information(conn->instance->epr, random, arg, erase);
}

static TZ1K_INLINE void
twicSsLeSmMasKeyReqEventCm(const twicRespD4D9Cm_t * const op_d4d9)
{
  const twicLeSmpICb_t *cb;
  twicBdaddr_t *arg;
  twicEntry_t *conn;
  bool random;

  conn = twicHashFind1stHandle(op_d4d9->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb) || !(cb->inquiry_bonding_information)) return;

  conn->instance->group.b.smp_i_key_req_event = true;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_SMP_I_KEY_REQ_EVENT\r\n");
#endif

  arg = (twicBdaddr_t *)&conn->instance->resp.value;
  memcpy(arg, &op_d4d9->bd_address, 6);
  random = (0x01 == op_d4d9->address_type) ? true : false;
  cb->inquiry_bonding_information(conn->instance->epr, random, arg);
}

static TZ1K_INLINE void
twicSsLeSmMasBondingStateEventCm(const twicRespD49aCm_t * const op_d49a)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d49a->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->bonding_state)) return;
  cb->bonding_state(
    conn->instance->epr, (twicSmReasonCode_t)(op_d49a->reason));
}

static TZ1K_INLINE void
twicSsLeSmMasPairingFailedEventCm(const twicRespD443Cm_t * const op_d443)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d443->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->pairing_failed)) return;
  cb->pairing_failed(conn->instance->epr,
                     (twicSmReasonCode_t)(op_d443->reason));
}

static TZ1K_INLINE void
twicSsLeSmMasStkGenMethodEventCm(const twicRespD4CbCm_t * const op_d4cb)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d4cb->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->stk_generation_method)) return;
  cb->stk_generation_method(conn->instance->epr,
                            op_d4cb->status,
                            (twicStkGenMethod_t)(op_d4cb->method));
}

static TZ1K_INLINE void
twicSsLeSmMasPassKeyReqEventCm(const twicRespD444Cm_t * const op_d444)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d444->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb) || !(cb->input_passkey)) return;

  conn->instance->group.b.smp_i_key_entry_req_event = true;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_SMP_I_KEY_ENTRY_REQ_EVENT\r\n");
#endif

  cb->input_passkey(conn->instance->epr);
}

/* This event is sent by the Slave Device to its application to input
 * the passkey displayed on the LE Device. This PassKey is used in the
 * PassKey Entry Pairing method. This event is generated as part of
 * pairing process. In response to this event, the application will
 * generate the key write request twicIfLeSmSlvDpPasskeyEntryReply
 * to input the Displayed Key. */

static TZ1K_INLINE void
twicSsLeSmMasDisplayPassKeyEventCm(const twicRespD446Cm_t * const op_d446)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d446->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb) || !(cb->display_passkey)) return;

  conn->instance->group.b.smp_i_display_key_event = true;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_SMP_I_DISPLAY_KEY_EVENT\r\n");
#endif

  cb->display_passkey(conn->instance->epr);
}

static TZ1K_INLINE void
twicSsLeSmMasStkGeneratedEventCm(const twicRespD448Cm_t * const op_d448)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  twicStk_t *arg;

  conn = twicHashFind1stHandle(op_d448->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  arg = (twicStk_t *)&conn->instance->resp.value;
  memcpy(arg, op_d448->key, 16);
  if (!(cb->stk_generated)) return;
  cb->stk_generated(conn->instance->epr, arg);
}

static TZ1K_INLINE void
twicSsLeSmMasLtkReceivedEventCm(const twicRespD4C9Cm_t * const op_d4c9)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  twicLtk_t *arg;

  conn = twicHashFind1stHandle(op_d4c9->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  arg = (twicLtk_t *)&conn->instance->resp.value;
  memcpy(arg, op_d4c9->key, 16);
  if (!(cb->encryption_info)) return;
  cb->encryption_info(conn->instance->epr, arg);
}

static TZ1K_INLINE void
twicSsLeSmMasEdivRandReceivedEventCm(const twicRespD4CaCm_t * const op_d4ca)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  uint8_t *arg;

  conn = twicHashFind1stHandle(op_d4ca->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  arg = (uint8_t *)&conn->instance->resp.value;
  memcpy(arg, op_d4ca->ediv, 2);
  memcpy(arg + 2, op_d4ca->rand, 8);
  if (!(cb->master_identification)) return;
  cb->master_identification(
    conn->instance->epr, (twicEdiv_t *)arg, (twicRand_t *)(arg + 2));
}

static TZ1K_INLINE void
twicSsLeSmMasEdivRandSentEventCm(const twicRespD4CdCm_t * const op_d4cd)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  uint8_t *arg;

  conn = twicHashFind1stHandle(op_d4cd->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  arg = (uint8_t *)&conn->instance->resp.value;
  memcpy(arg, op_d4cd->ediv, 2);
  memcpy(arg + 2, op_d4cd->rand, 8);
  if (!(cb->master_identification_sent)) return;
  cb->master_identification_sent(
    conn->instance->epr, (twicEdiv_t *)arg, (twicRand_t *)(arg + 2));
}

static TZ1K_INLINE void
twicSsLeSmMasEncryptionChangeEventCm(const twicRespD4CeCm_t * const op_d4ce)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  bool encryption = false;

  conn = twicHashFind1stHandle(op_d4ce->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->encryption_change)) return;

  if (0x01 == op_d4ce->encryption_flag) encryption = true;

  cb->encryption_change(
    conn->instance->epr, (twicSmReasonCode_t)(op_d4ce->reason),
    op_d4ce->key_type, encryption, op_d4ce->encryption_key_size);
}

static TZ1K_INLINE void
twicSsLeSmMasLtkSentEventCm(const twicRespD4CcCm_t * const op_d4cc)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  twicLtk_t *arg;

  conn = twicHashFind1stHandle(op_d4cc->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  arg = (twicLtk_t *)&conn->instance->resp.value;
  memcpy(arg->key, op_d4cc->key, 16);
  if (!(cb->encryption_info_sent)) return;
  cb->encryption_info_sent(conn->instance->epr, arg);
}

static TZ1K_INLINE void twicSsLeSmMasEncryptionKeyRefreshCompleteEventCm(
  const twicRespD4CfCm_t * const op_d4cf)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d4cf->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->encryption_key_refresh_complete)) return;

  cb->encryption_key_refresh_complete(
    conn->instance->epr, (twicSmReasonCode_t)(op_d4cf->reason),
    op_d4cf->key_type, op_d4cf->encryption_key_size);
}

static TZ1K_INLINE void
twicSsLeSmMasPairingCompleteEventCm(const twicRespD4D0Cm_t * const op_d4d0)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  twicAuthInfo_t bits;

  conn = twicHashFind1stHandle(op_d4d0->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->pairing_complete)) return;
  bits = *(twicAuthInfo_t *)&(op_d4d0->authentication_information);
  cb->pairing_complete(conn->instance->epr, op_d4d0->status, bits);
}

static TZ1K_INLINE void
twicSsLeSmMasIrkReceivedEventCm(const twicRespD4D4Cm_t * const op_d4d4)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  twicIrk_t *arg;

  conn = twicHashFind1stHandle(op_d4d4->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->identity_information)) return;
  arg = (twicIrk_t *)&conn->instance->resp.value;
  memcpy(arg->key, op_d4d4->key, 16);
  cb->identity_information(conn->instance->epr, arg);
}

static TZ1K_INLINE void twicSsLeSmMasIdentityAddressReceivedEventCm(
  const twicRespD4D5Cm_t * const op_d4d5)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  twicBdaddr_t *arg;
  bool random;

  conn = twicHashFind1stHandle(op_d4d5->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->identity_address_information)) return;
  arg = (twicBdaddr_t *)&conn->instance->resp.value;
  memcpy(arg->address, op_d4d5->device_address, 6);
  random = (0x01 == op_d4d5->address_type) ? true : false;
  cb->identity_address_information(conn->instance->epr, random, arg);
}

static TZ1K_INLINE void
twicSsLeSmMasCsrkReceivedEventCm(const twicRespD4D6Cm_t * const op_d4d6)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  twicCsrk_t *arg;

  conn = twicHashFind1stHandle(op_d4d6->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->identity_information)) return;
  arg = (twicCsrk_t *)&conn->instance->resp.value;
  memcpy(arg->key, op_d4d6->key, 16);
  cb->signing_information(conn->instance->epr, arg);
}

static TZ1K_INLINE void
twicSsLeSmMasCsrkSentEventCm(const twicRespD4D3Cm_t * const op_d4d3)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  twicCsrk_t *arg;

  conn = twicHashFind1stHandle(op_d4d3->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->signing_information_sent)) return;
  arg = (twicCsrk_t *)&conn->instance->resp.value;
  memcpy(arg->key, op_d4d3->key, 16);
  cb->signing_information_sent(conn->instance->epr, arg);
}

static TZ1K_INLINE void
twicSsLeSmMasIrkSentEventCm(const twicRespD4D1Cm_t * const op_d4d1)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  twicIrk_t *arg;

  conn = twicHashFind1stHandle(op_d4d1->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->identity_information_sent)) return;
  arg = (twicIrk_t *)&conn->instance->resp.value;
  memcpy(arg->key, op_d4d1->key, 16);
  cb->identity_information_sent(conn->instance->epr, arg);
}

static TZ1K_INLINE void
twicSsLeSmMasIdentityAddressSentEventCm(const twicRespD4D2Cm_t * const op_d4d2)
{
  const twicLeSmpICb_t *cb;
  twicEntry_t *conn;
  twicBdaddr_t *arg;
  bool random;

  conn = twicHashFind1stHandle(op_d4d2->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_i_cb;
  if (!(cb)) return;
  if (!(cb->identity_address_information_sent)) return;
  arg = (twicBdaddr_t *)&conn->instance->resp.value;
  memcpy(arg->address, op_d4d2->device_address, 6);
  random = (0x01 == op_d4d2->address_type) ? true : false;
  cb->identity_address_information_sent(conn->instance->epr, random, arg);
}
#endif

#if defined(TWIC_CONFIG_SM_RESPONDER)
twicStatus_t
twicSsLeSmSlvPairingConfirm(const uint8_t seq, twicEntry_t * const conn,
                            const uint8_t io_capability,
                            const uint8_t oob_data_flag,
                            const uint8_t auth_req,
                            const uint8_t max_enc_key_size,
                            const uint8_t init_key_dist,
                            const uint8_t resp_key_dist)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  twic_container.param[2] = 0x00;
  twic_container.param[3] = io_capability;
  twic_container.param[4] = oob_data_flag;
  twic_container.param[5] = auth_req;
  twic_container.param[6] = max_enc_key_size;
  twic_container.param[7] = init_key_dist;
  twic_container.param[8] = resp_key_dist;

  len = twicSsLeSetupContainerCm(0x01, TWIC_SERVICE_ID_SMP_RESPONDER, 9,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

static TZ1K_INLINE void
twicSsLeSmSlvBondingStateEventCm(const twicRespD59bCm_t * const op_d59b)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d59b->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->bonding_state)) return;
  cb->bonding_state(conn->instance->epr, (twicSmReasonCode_t)(op_d59b->reason));
}

static TZ1K_INLINE void
twicSsLeSmSlvKeyReqEventCm(const twicRespD5DaCm_t * const op_d5da)
{
  const twicLeSmpRCb_t *cb;
  twicBdaddr_t *arg;
  twicEntry_t *conn;
  bool random;

  conn = twicHashFind1stHandle(op_d5da->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb) || !(cb->inquiry_bonding_information)) return;

  conn->instance->group.b.smp_r_key_req_event = true;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_SMP_R_KEY_REQ_EVENT\r\n");
#endif

  arg = (twicBdaddr_t *)&conn->instance->resp.value;
  memcpy(arg, &op_d5da->bd_address, 6);
  random = (0x01 == op_d5da->address_type) ? true : false;
  cb->inquiry_bonding_information(conn->instance->epr, random, arg);
}

static TZ1K_INLINE void
twicSsLeSmSlvStoreKeyEventCm(const twicRespD5D9Cm_t * const op_d5d9)
{
  const twicLeSmpRCb_t *cb;
  twicBdaddr_t *arg;
  twicEntry_t *conn;
  bool erase;
  bool random;

  conn = twicHashFind1stHandle(op_d5d9->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->store_bonding_information)) return;
  arg = (twicBdaddr_t *)&conn->instance->resp.value;
  memcpy(arg, &op_d5d9->bd_address, 6);
  erase = (0x02 == op_d5d9->status) ? true : false;
  random = (0x01 == op_d5d9->address_type) ? true : false;
  cb->store_bonding_information(conn->instance->epr, random, arg, erase);
}

static TZ1K_INLINE void
twicSsLeSmSlvOobKeyEntryRequestEventCm(const twicRespD559Cm_t * const op_d559)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d559->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb) || !(cb->oob_information)) return;

  conn->instance->group.b.smp_r_oob_key_entry_req_event = true;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_SMP_R_OOB_KEY_ENTRY_REQ_EVENT\r\n");
#endif

  cb->oob_information(conn->instance->epr);
}

static TZ1K_INLINE void
twicSsLeSmSlvStkGenMethodEventCm(const twicRespD5CbCm_t * const op_d5cb)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d5cb->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->stk_generation_method)) return;
  cb->stk_generation_method(
    conn->instance->epr,
    op_d5cb->status, (twicStkGenMethod_t)(op_d5cb->method));
}

static TZ1K_INLINE void
twicSsLeSmSlvPairingFailedEventCm(const twicRespD543Cm_t * const op_d543)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d543->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->pairing_failed)) return;
  cb->pairing_failed(conn->instance->epr,
                     (twicSmReasonCode_t)(op_d543->reason));
}

/* This event is generated by the Slave Device to the application to
 * request to enter the PassKey used in the "PassKey Entry" Pairing
 * method. This event is generated as part of pairing process. In
 * response to this event, the application will generate the key write
 * request "twicIfLeSmMasKbPasskeyEntryReply" to input the Passkey. */
static TZ1K_INLINE void
twicSsLeSmSlvPassKeyReqEventCm(const twicRespD544Cm_t * const op_d544)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d544->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb) || !(cb->input_passkey)) return;

  conn->instance->group.b.smp_r_key_entry_req_event = true;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_SMP_R_KEY_ENTRY_REQ_EVENT\r\n");
#endif

  cb->input_passkey(conn->instance->epr);
}

static TZ1K_INLINE void
twicSsLeSmSlvDisplayPassKeyEventCm(const twicRespD546Cm_t * const op_d546)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d546->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb) || !(cb->display_passkey)) return;

  conn->instance->group.b.smp_r_display_key_event = true;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_SMP_R_DISPLAY_KEY_EVENT\r\n");
#endif

  cb->display_passkey(conn->instance->epr);
}

static TZ1K_INLINE void
twicSsLeSmSlvStkGeneratedEventCm(const twicRespD548Cm_t * const op_d548)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  twicStk_t *arg;

  conn = twicHashFind1stHandle(op_d548->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  arg = (twicStk_t *)&conn->instance->resp.value;
  memcpy(arg, op_d548->key, 16);
  if (!(cb->stk_generated)) return;
  cb->stk_generated(conn->instance->epr, arg);
}

static TZ1K_INLINE void
twicSsLeSmSlvLtkReceivedEventCm(const twicRespD5C9Cm_t * const op_d5c9)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  twicLtk_t *arg;

  conn = twicHashFind1stHandle(op_d5c9->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  arg = (twicLtk_t *)&conn->instance->resp.value;
  memcpy(arg, op_d5c9->key, 16);
  if (!(cb->encryption_info)) return;
  cb->encryption_info(conn->instance->epr, arg);
}

static TZ1K_INLINE void
twicSsLeSmSlvEdivRandReceivedEventCm(const twicRespD5CaCm_t * const op_d5ca)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  uint8_t *arg;

  conn = twicHashFind1stHandle(op_d5ca->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  arg = (uint8_t *)&conn->instance->resp.value;
  memcpy(arg, op_d5ca->ediv, 2);
  memcpy(arg + 2, op_d5ca->rand, 8);
  if (!(cb->master_identification)) return;
  cb->master_identification(
    conn->instance->epr, (twicEdiv_t *)arg, (twicRand_t *)(arg + 2));
}

static TZ1K_INLINE void
twicSsLeSmSlvEdivRandSentEventCm(const twicRespD5CdCm_t * const op_d5cd)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  uint8_t *arg;

  conn = twicHashFind1stHandle(op_d5cd->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  arg = (uint8_t *)&conn->instance->resp.value;
  memcpy(arg, op_d5cd->ediv, 2);
  memcpy(arg + 2, op_d5cd->rand, 8);
  if (!(cb->master_identification_sent)) return;
  cb->master_identification_sent(
    conn->instance->epr, (twicEdiv_t *)arg, (twicRand_t *)(arg + 2));
}

static TZ1K_INLINE void
twicSsLeSmSlvEncryptionChangeEventCm(const twicRespD5D0Cm_t * const op_d5d0)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d5d0->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->encryption_change)) return;

  cb->encryption_change(
    conn->instance->epr, (twicSmReasonCode_t)(op_d5d0->reason),
    op_d5d0->key_type, op_d5d0->encryption_flag, op_d5d0->encryption_key_size);
}

static TZ1K_INLINE void
twicSsLeSmSlvLtkSentEventCm(const twicRespD5CcCm_t * const op_d5cc)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  twicLtk_t *arg;

  conn = twicHashFind1stHandle(op_d5cc->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  arg = (twicLtk_t *)&conn->instance->resp.value;
  memcpy(arg->key, op_d5cc->key, 16);
  if (!(cb->encryption_info_sent)) return;
  cb->encryption_info_sent(conn->instance->epr, arg);
}

static TZ1K_INLINE void
twicSsLeSmSlvEncryptionKeyRefreshCompleteEventCm(
  const twicRespD5D1Cm_t * const op_d5d1)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d5d1->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->encryption_key_refresh_complete)) return;

  cb->encryption_key_refresh_complete(
    conn->instance->epr, (twicSmReasonCode_t)(op_d5d1->reason),
    op_d5d1->key_type, op_d5d1->encryption_key_size);
}

static TZ1K_INLINE void
twicSsLeSmSlvIrkReceivedEventCm(const twicRespD5D6Cm_t * const op_d5d6)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  twicIrk_t *arg;

  conn = twicHashFind1stHandle(op_d5d6->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->identity_information)) return;
  arg = (twicIrk_t *)&conn->instance->resp.value;
  memcpy(arg->key, op_d5d6->key, 16);
  cb->identity_information(conn->instance->epr, arg);
}

static TZ1K_INLINE void twicSsLeSmSlvIdentityAddressReceivedEventCm(
  const twicRespD5D7Cm_t * const op_d5d7)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  twicBdaddr_t *arg;
  bool random;

  conn = twicHashFind1stHandle(op_d5d7->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->identity_address_information)) return;
  arg = (twicBdaddr_t *)&conn->instance->resp.value;
  memcpy(arg->address, op_d5d7->device_address, 6);
  random = (0x01 == op_d5d7->address_type) ? true : false;
  cb->identity_address_information(conn->instance->epr, random, arg);
}

static TZ1K_INLINE void
twicSsLeSmSlvCsrkReceivedEventCm(const twicRespD5D8Cm_t * const op_d5d8)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  twicCsrk_t *arg;

  conn = twicHashFind1stHandle(op_d5d8->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->identity_information)) return;
  arg = (twicCsrk_t *)&conn->instance->resp.value;
  memcpy(arg->key, op_d5d8->key, 16);
  cb->signing_information(conn->instance->epr, arg);
}

static TZ1K_INLINE void
twicSsLeSmSlvCsrkSentEventCm(const twicRespD5D5Cm_t * const op_d5d5)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  twicCsrk_t *arg;

  conn = twicHashFind1stHandle(op_d5d5->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->signing_information_sent)) return;
  arg = (twicCsrk_t *)&conn->instance->resp.value;
  memcpy(arg->key, op_d5d5->key, 16);
  cb->signing_information_sent(conn->instance->epr, arg);
}

static TZ1K_INLINE void
twicSsLeSmSlvIrkSentEventCm(const twicRespD5D3Cm_t * const op_d5d3)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  twicIrk_t *arg;

  conn = twicHashFind1stHandle(op_d5d3->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->identity_information_sent)) return;
  arg = (twicIrk_t *)&conn->instance->resp.value;
  memcpy(arg->key, op_d5d3->key, 16);
  cb->identity_information_sent(conn->instance->epr, arg);
}

static TZ1K_INLINE void twicSsLeSmSlvIdentityAddressSentEventCm(
  const twicRespD5D4Cm_t * const op_d5d4)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  twicBdaddr_t *arg;
  bool random;

  conn = twicHashFind1stHandle(op_d5d4->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->identity_address_information_sent)) return;
  arg = (twicBdaddr_t *)&conn->instance->resp.value;
  memcpy(arg->address, op_d5d4->device_address, 6);
  random = (0x01 == op_d5d4->address_type) ? true : false;
  cb->identity_address_information_sent(conn->instance->epr, random, arg);
}

static TZ1K_INLINE void twicSsLeSmSlvPairingCompleteEventCm(
  const twicRespD5D2Cm_t * const op_d5d2)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  twicAuthInfo_t bits;

  conn = twicHashFind1stHandle(op_d5d2->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->pairing_complete)) return;
  bits = *(twicAuthInfo_t *)&(op_d5d2->authentication_information);
  cb->pairing_complete(conn->instance->epr, op_d5d2->status, bits);
}

static TZ1K_INLINE void twicSsLeSmSlvStkEncryptionSessionReqReplyEventCm(
  const twicRespD5CeCm_t * const op_d5ce)
{
    const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  twicStk_t *arg;

  conn = twicHashFind1stHandle(op_d5ce->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->stk_session_request_reply_event)) return;
  if (0x00 == op_d5ce->status) {
    arg = (twicStk_t *)&conn->instance->resp.value;
    memcpy(arg->key, op_d5ce->key, 16);
  }
  cb->stk_session_request_reply_event(conn->instance->epr, op_d5ce->status,
                                      arg);
}

static TZ1K_INLINE void twicSsLeSmSlvLtkEncryptionSessionReqReplyEventCm(
  const twicRespD5CfCm_t * const op_d5cf)
{
    const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;
  twicLtk_t *arg;

  conn = twicHashFind1stHandle(op_d5cf->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->ltk_session_request_reply_event)) return;
  if (0x00 == op_d5cf->status) {
    arg = (twicLtk_t *)&conn->instance->resp.value;
    memcpy(arg->key, op_d5cf->key, 16);
  }
  cb->ltk_session_request_reply_event(conn->instance->epr, op_d5cf->status,
                                      arg);
}

/* This event is generated by Slave Device to inform the Application
 * that a Pairing Request has been received from the Master Device. In
 * response to this event, Slave application will generate the Slave
 * Pairing Accept Request twicIfLeSmSlvPairingConfirm. This Event will
 * contain Input Output Capabilities and Authentication information of
 * the Slave Device. */

static TZ1K_INLINE void
twicSsLeSmSlvPairingRequestEventCm(const twicRespD5C1Cm_t * const op_d5c1)
{
  const twicLeSmpRCb_t *cb;
  twicPairingFeature_t *arg;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d5c1->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb) || !(cb->pairing_demand)) return;

  conn->instance->group.b.smp_r_pairing_event = true;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_SMP_R_PAIRING_EVENT\r\n");
#endif

  arg = (twicPairingFeature_t *)&conn->instance->resp.value;

  arg->io_capability = op_d5c1->io_capability;
  arg->oob_data_present =
    (TWIC_SMP_OOB_DATA_PRESENT == op_d5c1->oob_data_flag) ? true : false;
  arg->auth_req_bonding = ((0x1 & op_d5c1->auth_req)) ? true : false;
  arg->auth_req_mitm_protection = ((0x4 & op_d5c1->auth_req)) ? true : false;
  arg->max_enc_key_size = op_d5c1->max_enc_key_size;

  arg->init_key_dist_enckey = (0x1 & op_d5c1->init_key_dist) ? true : false;
  arg->init_key_dist_idkey = (0x2 & op_d5c1->init_key_dist) ? true : false;
  arg->init_key_dist_sign = (0x4 & op_d5c1->init_key_dist) ? true : false;
  arg->resp_key_dist_enckey = (0x1 & op_d5c1->resp_key_dist) ? true : false;
  arg->resp_key_dist_idkey = (0x2 & op_d5c1->resp_key_dist) ? true : false;
  arg->resp_key_dist_sign = (0x4 & op_d5c1->resp_key_dist) ? true : false;

  cb->pairing_demand(conn->instance->epr, arg);
}

static TZ1K_INLINE void
twicSsLeSmSlvPairingAcceptEventCm(const twicResp8xCm_t * const op_d581)
{
  const twicLeSmpRCb_t *cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d581->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  cb = conn->instance->smp_r_cb;
  if (!(cb)) return;
  if (!(cb->pairing_acceptance_sent)) return;
  cb->pairing_acceptance_sent(conn->instance->epr, op_d581->status);
}
#endif

static twicEntry_t *
twicSsEtEntry(const uint16_t cid, const bool enable, const uint16_t handle)
{
  twicEntry_t *entry;

  tz1utListEach(twicEntry_t, entry, TZ1UT_LIST(srv_req_root), o_link) {
    if (true == enable) {
      if (entry->id == cid && entry->handle == handle) return entry;
    } else {
      if (entry->id == cid) return entry;
    }
  }

  return (twicEntry_t *)NULL;
}

static uint64_t twicMsb64(uint8_t *bits128)
{
  uint64_t msb = 0;
  uint8_t i;

  for (i = 15; 8 <= i; i--) {
    msb <<= 8;
    msb |= bits128[i];
  }

  return msb;
}

static uint64_t twicLsb64(uint8_t *bits128)
{
  uint64_t lsb = 0;
  int8_t i;

  for (i = 7; 0 <= i; i--) {
    lsb <<= 8;
    lsb |= bits128[i];
  }

  return lsb;
}

void twicSetUuid(uint8_t *uuid, uint64_t uuid_lsb, uint64_t uuid_msb)
{
  uint8_t i;

  for (i = 0; i != 8; ++i) {
    uuid[i]     = (uuid_lsb >> (8 * i)) & 0xFF;
    uuid[i + 8] = (uuid_msb >> (8 * i)) & 0xFF;
  }

  return;
}

uint64_t twicUuidLsb(twicUuid_t *uuid)
{
  return twicLsb64(uuid->uu.id128);
}

uint64_t twicUuidMsb(twicUuid_t *uuid)
{
  return twicMsb64(uuid->uu.id128);
}

twicStatus_t twicSsReq(const uint8_t seq, twicEntry_t * const conn,
                       const uint8_t op_code, const uint8_t service_id,
                       const uint8_t * const argv, uint8_t argc)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  if (NULL == argv) {
    argc = 0;
  } else {
    memcpy(twic_container.param, argv, argc);
  }

  /* twicLog("argc = 0x%x.\r\n", argc); */

  len = twicSsLeSetupContainerCm(op_code, service_id, argc, TWIC_ENTRY_MNG_TCU,
                                 conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  /* twicSsDebugTx((uint8_t *)&twic_container, len); */

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsToReq(const uint8_t seq, twicEntry_t * const conn,
                         const uint8_t op_code, const uint8_t service_id)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);

  len = twicSsLeSetupContainerCm(
    op_code, service_id, 2, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsTreReq(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id, const uint8_t tre)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  twic_container.param[2] = tre;

  len = twicSsLeSetupContainerCm(
    op_code, service_id, 3, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

#if defined(TWIC_API_GATTCLIENTEXGMTU) ||                     \
  defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE) ||      \
  defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID) ||  \
  defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR)
twicStatus_t twicSsTreToReq(const uint8_t seq, twicEntry_t * const conn,
                            const uint8_t op_code, const uint8_t service_id,
                            const uint16_t tre)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  TWIC_SETHARF_LE(twic_container.param + 2, tre);

  len = twicSsLeSetupContainerCm(
    op_code, service_id, 4, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICE) ||             \
  defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID) ||  \
  defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE) ||                  \
  defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS) ||           \
  defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)||         \
  defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS) ||               \
  defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE) ||          \
  defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR)
twicStatus_t twicSsFyraToReq(const uint8_t seq, twicEntry_t * const conn,
                             const uint8_t op_code, const uint8_t service_id,
                             const uint16_t tre, const uint16_t fyra)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  TWIC_SETHARF_LE(twic_container.param + 2, tre);
  TWIC_SETHARF_LE(twic_container.param + 4, fyra);

  len = twicSsLeSetupContainerCm(
    op_code, service_id, 6, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif

twicStatus_t
twicSsFemReq(const uint8_t seq, twicEntry_t * const conn,
             const uint8_t op_code, const uint8_t service_id,
             const uint8_t tre, const uint16_t fem)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  twic_container.param[2] = tre;
  TWIC_SETHARF_LE(twic_container.param + 3, fem);

  len = twicSsLeSetupContainerCm(
    op_code, service_id, 5, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

#if defined(TWIC_CONFIG_SM_INITIATOR) || defined(TWIC_CONFIG_SM_RESPONDER)
twicStatus_t
twicSsLeSmPasskeyReply(const uint8_t seq, twicEntry_t * const conn,
                       const uint8_t op_code, const uint8_t service_id,
                       const twicPasskeyEntry_t *const passkey)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  twic_container.param[2] = 0x00;
  memcpy(twic_container.param + 3, passkey->key, 4);

  len = twicSsLeSetupContainerCm(op_code, service_id, 7, TWIC_ENTRY_MNG_TCU,
                                 conn, seq);
  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsLeSmOobTkReply(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id, const twicOobTk_t *const tk)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  twic_container.param[2] = 0x00;
  memcpy(twic_container.param + 3, tk->key, 16);

  len = twicSsLeSetupContainerCm(
    op_code, service_id, 19, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsLeSmBondingInformationReply(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id,
  const uint8_t key_set_bitmap, const twicEdiv_t * const r_ediv,
  const twicRand_t * const r_rand, const twicLtk_t * const r_ltk,
  const twicIrk_t * const r_irk, const twicCsrk_t * const r_csrk,
  const twicEdiv_t * const l_ediv, const twicRand_t * const l_rand,
  const twicLtk_t * const l_ltk, const twicIrk_t * const l_irk,
  const twicCsrk_t * const l_csrk, const uint8_t encryption_key_size)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  twic_container.param[2] = 0x00;
  twic_container.param[3] = key_set_bitmap;
  if (NULL != r_ediv) memcpy(twic_container.param +   4, r_ediv->value, 2);
  if (NULL != r_rand) memcpy(twic_container.param +   6, r_rand->value, 8);
  if (NULL != r_ltk)  memcpy(twic_container.param +  14, r_ltk->key, 16);
  if (NULL != r_irk)  memcpy(twic_container.param +  30, r_irk->key, 16);
  if (NULL != r_csrk) memcpy(twic_container.param +  46, r_csrk->key, 16);
  if (NULL != l_ediv) memcpy(twic_container.param +  62, l_ediv->value, 2);
  if (NULL != l_rand) memcpy(twic_container.param +  64, l_rand->value, 8);
  if (NULL != l_ltk)  memcpy(twic_container.param +  72, l_ltk->key, 16);
  if (NULL != l_irk)  memcpy(twic_container.param +  88, l_irk->key, 16);
  if (NULL != l_csrk) memcpy(twic_container.param + 104, l_csrk->key, 16);
  twic_container.param[120] = encryption_key_size;

  len = twicSsLeSetupContainerCm(
    op_code, service_id, 121, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif

#if defined(TWIC_BLE_HWIP_V41) || defined(TWIC_CONFIG_SM_INITIATOR) || defined(TWIC_CONFIG_SM_RESPONDER)
static TZ1K_INLINE twicEntry_t *
twicSsLeCmSimpleRespCm(const uint8_t mask, const uint8_t service_id,
                       const uint8_t op_code, const uint16_t handle,
                       const uint8_t reason)
{
  twicEntry_t *conn, *entry;
  uint16_t cid = (uint16_t)service_id << 8 | op_code & mask;

  conn = twicHashFind1stHandle(handle);
  if (!(conn) || !(conn->instance)) {
    /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU, TWIC_ENTRY_TYPE_SIGNAL */
    entry = twicSsEtEntry(cid, true, handle);
    if (entry) goto out;
    return NULL;
  }

  entry = conn;

  out:

  if (TWIC_ENTRY_TYPE_SIGNAL == entry->type) {
    entry->rsp = 0xFF;
    entry->req = 0;
    entry->result = 0;
    entry->id = 0;
    entry->seq = 0;
    entry->u.cont.len = 0;
  } else {
    entry->rsp = entry->seq;
    entry->result = 0;
    entry->u.cont.len = 1;
    entry->u.cont.value[0] = reason;
  }
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);

  return entry;
}
#endif

twicStatus_t twicSsLeConnectionUpdate(const uint8_t seq,
                                      twicEntry_t * const conn,
                                      const uint8_t op_code,
                                      uint16_t conn_int_min,
                                      uint16_t conn_int_max,
                                      uint16_t slave_latency,
                                      uint16_t supervision_timeout,
                                      uint16_t min_ce_length,
                                      uint16_t max_ce_length)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  if (0x0e == op_code) {
    /* TWIC_LECONNECTIONUPDATE(0x0e) */
    TWIC_SETHARF_LE(twic_container.param + 2, conn_int_min);
    TWIC_SETHARF_LE(twic_container.param + 4, conn_int_max);
    TWIC_SETHARF_LE(twic_container.param + 6, slave_latency);
    TWIC_SETHARF_LE(twic_container.param + 8, supervision_timeout);
    TWIC_SETHARF_LE(twic_container.param + 10, min_ce_length);
    TWIC_SETHARF_LE(twic_container.param + 12, max_ce_length);
    len = 14;
  } else {
    /* TWIC_LECEL2CAPSIGNALACCEPTCONNECTIONUPDATE(0x16)
       TWIC_LECONNECTIONPARAMETERREQUESTREPLY(0x1e)
       TWIC_LECONNECTIONUPDATE(0x1e) */
    twic_container.param[2] = 0; /* SUCCESS */
    TWIC_SETHARF_LE(twic_container.param + 3, conn_int_min);
    TWIC_SETHARF_LE(twic_container.param + 5, conn_int_max);
    TWIC_SETHARF_LE(twic_container.param + 7, slave_latency);
    TWIC_SETHARF_LE(twic_container.param + 9, supervision_timeout);
    len = 11;
  }

  len = twicSsLeSetupContainerCm(op_code, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT,
                                 len, TWIC_ENTRY_MNG_TCU, conn, seq);
  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsLeDisconnect(
  const uint8_t seq, twicEntry_t * const conn, const twicBdaddr_t * const bd)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  memcpy(twic_container.param, bd->address, 0x06);
  len = twicSsLeSetupContainerCm(
    0x13, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 6,
    TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

/* TZ1x ROM5 Configuration.
   1. The number of MNG message lock is integrated and single.
      This entry is defined and declared in the middleware code. */

/* 2. The number of the other messages which are characteristics value and
      descriptors is separated to each of the commands. */

twicStatus_t twicSsGattInitialize(void)
{
  const tz1smHalCb_t hal_cb = { &twicSsIsrLeDownstreamRequest,
                                &twicSsIsrLeHostWakeup,
                                &twicSsIsrLeStatusChanged };

  if (true != twic_eq.sniff.ioinit) {
    return TWIC_STATUS_ERROR_IO;
  }

  twicSsLeReset(true);
  memset(&twic_container, 0, sizeof (twicCmContainer_t));
  memset(&twic_eq, 0, sizeof (twicRxPipeQueue_t));

  twic_eq.sniff.ioinit = true;
  twic_eq.sniff.enable = RPQUEUE_DISABLE;
  twic_eq.h4 = RPQUEUE_H4_HAL_OK;
  twic_eq.slpto = 0;
  twic_eq.sniff.waiting_for_activation = false;
  twic_eq.sniff.lowpower_requirements = false;
  twic_eq.sniff.lowpower_enable = false;
  twic_eq.lowpower_selector = RPQUEUE_PM_UNKNOWN;
  twic_eq.lowpower_mode = RPQUEUE_PM_UNKNOWN;
  twic_eq.sniff.lowpower_clock = false;
  twic_eq.hdr = RPQUEUE_PM_HDU;
  twic_eq.host_delay = RPQUEUE_PM_HDU;
  twic_eq.sniff.pin = false;
  twic_eq.sniff.pin_rq = false;
#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
  twic_eq.sniff.lowpower_peripheral_h4 = false;
  twic_eq.sniff.lowpower_core = false;
  twic_eq.sniff.lowpower_core_idle = false;
  twic_eq.sniff.lowpower_core_cosc = false;
  twic_eq.sniff.lowpower_core_adve = false;
  twic_eq.sniff.lowpower_core_ncad = false;
#endif

  twic_eq.sniff.fc = twic_eq.sniff.fc_rq = false;
  twic_eq.br = twic_eq.br_rq = 115200;
  tz1smHalUartInit(twic_eq.br, twic_eq.sniff.fc);
  twicSsLeRequestHighPower();
  twicSsLeReset(false);
  tz1smHalRegisterCb(&hal_cb);

  return TWIC_STATUS_OK;
}

void
twicSsGattCleanup(const uint8_t interface, twicConnIface_t * const conn_iface)
{
  uint8_t idx;
  twicEntry_t * const * entries = conn_iface->entries;

  twicSsGattCleanInstance(interface);
  twicHashCleanupInterface(interface);
  memset(&conn_iface->instance, 0, sizeof(twicInstance_t));

  for (idx = 0; idx < conn_iface->size; idx ++) {
    memset(entries[idx], 0, sizeof(twicEntry_t));
    tz1utListInit(&(entries[idx])->o_link);
    (entries[idx])->interface = interface;
    (entries[idx])->instance = &conn_iface->instance;
    (entries[idx])->u.attr.eidx = idx + 1;
    (entries[idx])->rsp = 0xFF;
  }

  tz1utListInit(&conn_iface->conn.o_link);
  conn_iface->conn.interface = interface;
  conn_iface->conn.type = TWIC_ENTRY_TYPE_MNG;
  conn_iface->conn.instance = &conn_iface->instance;

  tz1utListInit(&conn_iface->signal.o_link);
  conn_iface->signal.interface = interface;
  conn_iface->signal.type = TWIC_ENTRY_TYPE_SIGNAL;
  conn_iface->signal.instance = &conn_iface->instance;

  /* TZ1x ROM5 Configuration.
     1. The number of MNG message lock is integrated and single.
     This entry is defined and declared in the middleware code.
  */

  conn_iface->conn.rsp = 0xFF;
  conn_iface->signal.rsp = 0xFF;

  conn_iface->instance.epr = conn_iface;
  conn_iface->instance.conn = &conn_iface->conn;
  conn_iface->instance.signal = &conn_iface->signal;
  conn_iface->instance.mtu = TWIC_ATT_MTU;
  conn_iface->instance.interface = interface;
  conn_iface->instance.role = TWIC_IFACE_GAP_UNDEFINED;
  conn_iface->instance.opmd.conn = conn_iface->instance.opmd.scan = false;
  conn_iface->instance.opmd.adve = conn_iface->instance.opmd.ncad = false;
}

twicStatus_t
twicSsGattServerAddService(const uint8_t seq, twicEntry_t * const entry,
                           const twicUuid_t * const uuid)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  twic_container.param[0] = (uint8_t)uuid->len;
  memcpy(twic_container.param + 1, (uint8_t *)&uuid->uu, uuid->len);
  entry->type = TWIC_ENTRY_TYPE_SERVICE;
  len = twicSsLeSetupContainerCm(0x20, TWIC_SERVICE_ID_GATT_SERVER,
                                 uuid->len + 1, TWIC_ENTRY_HANDLE_DEFINITION,
                                 entry, seq);

  ret = twicSsLeTxDataCm(entry, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsGattServerAddSecondaryService(
  const uint8_t seq, twicEntry_t * const entry, const twicUuid_t * const uuid)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  twic_container.param[0] = (uint8_t)uuid->len;
  memcpy(twic_container.param + 1, (uint8_t *)&uuid->uu, uuid->len);
  entry->type = TWIC_ENTRY_TYPE_SECONDARY_SERVICE;
  len = twicSsLeSetupContainerCm(
    0x21, TWIC_SERVICE_ID_GATT_SERVER,
    uuid->len + 1, TWIC_ENTRY_HANDLE_DEFINITION, entry, seq);

  ret = twicSsLeTxDataCm(entry, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsGattServerAddIncludeService(
  const uint8_t seq, twicEntry_t * const service,
  twicEntry_t * const inclusion, const twicUuid_t * const uuid)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, service->handle);
  TWIC_SETHARF_LE(twic_container.param + 3, inclusion->handle);
  TWIC_SETHARF_LE(twic_container.param + 5, inclusion->u.attr.b_handle);
  if (TWIC_UUID16 == uuid->len) {
    twic_container.param[2] = 6;
    memcpy(twic_container.param + 7, (uint8_t *)&uuid->uu, TWIC_UUID16);
    len = 9;
  } else {
    twic_container.param[2] = 4;
    TWIC_SETHARF_LE(twic_container.param + 7, 0);
    len = 7;
  }
  inclusion->type = TWIC_ENTRY_TYPE_INCLUDED_SERVICE;
  len = twicSsLeSetupContainerCm(
    0x24, TWIC_SERVICE_ID_GATT_SERVER,
    len, TWIC_ENTRY_HANDLE_DEFINITION, service, seq);

  ret = twicSsLeTxDataCm(service, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsGattServerAddCharacteristics(
  const uint8_t seq,const uint16_t service_handle,
  twicEntry_t *entry, const twicUuid_t * const uuid)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, service_handle);
  twic_container.param[2] = entry->u.attr.properties;
  twic_container.param[3] = (uint8_t)uuid->len;
  memcpy(twic_container.param + 4, (uint8_t *)&uuid->uu, uuid->len);
  entry->type = TWIC_ENTRY_TYPE_CHARACTERISTIC_VALUE;
  entry->u.attr.p_handle = service_handle;
  len = twicSsLeSetupContainerCm(
    0x22, TWIC_SERVICE_ID_GATT_SERVER,
    uuid->len + 4, TWIC_ENTRY_HANDLE_DEFINITION, entry, seq);

  ret = twicSsLeTxDataCm(entry, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsGattServerSetElementVl(const uint8_t seq,
                                          twicEntry_t * entity,
                                          const twicUuid_t * const uuid,
                                          const uint16_t max_element_length,
                                          const uint16_t element_length,
                                          const uint8_t * const element_value)
{
  uint8_t *mover;
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(entity->instance->token) &&
      !(tz1utListIsEmpty(TZ1UT_LIST(srv_req_root)))) {
    return TWIC_STATUS_UNDER_PROCESSING;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, entity->handle);
  twic_container.param[2] = (uint8_t)uuid->len;
  mover = twic_container.param + 3;
  memcpy(mover, (uint8_t *)&uuid->uu, uuid->len);
  mover += uuid->len;
  TWIC_SETHARF_LE(mover, max_element_length);
  mover += 2;
  TWIC_SETHARF_LE(mover, element_length);
  mover += 2;
  memcpy(mover, element_value, element_length);
  mover += element_length;
  TWIC_SETHARF_LE(mover, entity->u.attr.permissions);
  len = twicSsLeSetupContainerCm(0x2C, TWIC_SERVICE_ID_GATT_SERVER,
                                 7 + uuid->len + 2 + element_length,
                                 TWIC_ENTRY_HANDLE_DECLARATION, entity, seq);

  ret = twicSsLeTxDataCm(entity, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsGattServerSetElement(const uint8_t seq,
                                        twicEntry_t * entity,
                                        const twicUuid_t * const uuid,
                                        const uint16_t element_length,
                                        const uint8_t * const element_value)
{
  uint8_t *mover;
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(entity->instance->token) &&
      !(tz1utListIsEmpty(TZ1UT_LIST(srv_req_root)))) {
    return TWIC_STATUS_UNDER_PROCESSING;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, entity->handle);
  twic_container.param[2] = (uint8_t)uuid->len;
  mover = twic_container.param + 3;
  memcpy(mover, (uint8_t *)&uuid->uu, uuid->len);
  mover += uuid->len;
  TWIC_SETHARF_LE(mover, element_length);
  mover += 2;
  memcpy(mover, element_value, element_length);
  mover += element_length;
  TWIC_SETHARF_LE(mover, entity->u.attr.permissions);
  len = twicSsLeSetupContainerCm(
    0x23, TWIC_SERVICE_ID_GATT_SERVER, 7 + uuid->len + element_length,
    TWIC_ENTRY_HANDLE_DECLARATION, entity, seq);

  ret = twicSsLeTxDataCm(entity, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t
twicSsGattServerWriteElement(const uint8_t seq, twicEntry_t * const conn,
                             const uint16_t handle, const uint16_t length,
                             const uint8_t * const value)
{
  int16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, handle);
  TWIC_SETHARF_LE(twic_container.param + 2, length);
  memcpy(twic_container.param + 4, value, length);
  len = twicSsLeSetupContainerCm(0x25, TWIC_SERVICE_ID_GATT_SERVER,
                                 4 + length, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsGattServerPackService(const uint8_t seq, twicEntry_t *entity)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, entity->handle);
  entity->type = TWIC_ENTRY_TYPE_SERVICE;
  len = twicSsLeSetupContainerCm(0x26, TWIC_SERVICE_ID_GATT_SERVER,
                                 2, TWIC_ENTRY_HANDLE_DECLARATION, entity, seq);

  ret = twicSsLeTxDataCm(entity, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#if defined(TWIC_API_LEGATTDBSETPERMISSIONS)
twicStatus_t twicSsGattDbSetPermissions(
  const uint8_t seq, twicEntry_t *entity, const uint16_t permissions)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, entity->u.attr.b_handle);
  TWIC_SETHARF_LE(twic_container.param + 2, permissions);
  len = twicSsLeSetupContainerCm(0x29, TWIC_SERVICE_ID_GATT_SERVER, 4,
                                 TWIC_ENTRY_MNG_TCU, entity, seq);

  ret = twicSsLeTxDataCm(entity, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_LEGATTDBGETPERMISSIONS)
twicStatus_t twicSsGattDbGetPermissions(const uint8_t seq, twicEntry_t *entity)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, entity->u.attr.b_handle);
  len = twicSsLeSetupContainerCm(0x2A, TWIC_SERVICE_ID_GATT_SERVER, 2,
                                 TWIC_ENTRY_MNG_TCU, entity, seq);

  ret = twicSsLeTxDataCm(entity, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID) ||  \
  defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)
twicStatus_t twicSsGattClientDiscoveryByUuid(
  const uint8_t seq, twicEntry_t * const conn, const uint8_t op_code,
  const uint16_t start_handle, const uint16_t end_handle,
  const twicUuid_t * const uuid)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  if (1 == start_handle && 0xFFFF == end_handle && 0x03 == op_code) {
    TWIC_SETHARF_LE(twic_container.param, conn->handle);
    memcpy(twic_container.param + 2, (uint8_t *)&uuid->uu, uuid->len);
    len = uuid->len + 2;
  } else {
    TWIC_SETHARF_LE(twic_container.param, conn->handle);
    TWIC_SETHARF_LE(twic_container.param + 2, start_handle);
    TWIC_SETHARF_LE(twic_container.param + 4, end_handle);
    memcpy(twic_container.param + 6, (uint8_t *)&uuid->uu, uuid->len);
    len = uuid->len + 6;
  }

  len = twicSsLeSetupContainerCm(
    op_code, TWIC_SERVICE_ID_GATT_CLIENT, len, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

#if defined(TWIC_DEBUG_LOG_UART)
  twicSsDebugTx((uint8_t *)&twic_container, len);
#endif

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID)
twicStatus_t twicSsGattClientAccessByUuid(
  const uint8_t seq, twicEntry_t * const conn, const uint8_t op_code,
  const uint16_t start_handle, const uint16_t end_handle,
  const twicUuid_t * const uuid)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  TWIC_SETHARF_LE(twic_container.param + 2, start_handle);
  TWIC_SETHARF_LE(twic_container.param + 4, end_handle);
  memcpy(twic_container.param + 6, (uint8_t *)&uuid->uu, uuid->len);
  len = uuid->len + 6;

  len = twicSsLeSetupContainerCm(
    op_code, TWIC_SERVICE_ID_GATT_CLIENT, len, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_GATTCLIENTREADMULTIPLECHARVALUES)
twicStatus_t twicSsClientReadMultipleCharValues(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t number, const uint16_t * const handles)
{
  uint16_t len;
  twicStatus_t ret;
  uint8_t i;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if ((conn->instance->mtu - 1) < (number * 2)) {
    return TWIC_STATUS_ERROR_PARAMETER;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  for (i = 0; i < number; i ++) {
    TWIC_SETHARF_LE(twic_container.param + 2 + (2 * i), handles[i]);
  }
  len = 2 + 2 * number;
  len = twicSsLeSetupContainerCm(
    0x0c, TWIC_SERVICE_ID_GATT_CLIENT, len, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_GATTCLIENTSIGNEDWRITEWITHOUTRESPONSE)
twicStatus_t twicSsGattSignedWrite(
  const uint8_t seq, twicEntry_t * const conn,
  const uint8_t op_code, const uint8_t service_id,
  const uint16_t handle, const uint8_t length, const uint8_t * const data)
{
  int16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  len = length;
  if ((conn->instance->mtu - 15) < length) {
    len = conn->instance->mtu - 15;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  TWIC_SETHARF_LE(twic_container.param + 2, handle);
  memcpy(twic_container.param + 4, data, len);

  len = twicSsLeSetupContainerCm(
    op_code, service_id, len + 4, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_GATTCLIENTRELIABLEWRITE)
twicStatus_t twicSsClientReliableWrite(
  const uint8_t seq, twicEntry_t * const conn, const uint16_t bytes,
  const uint16_t total,
  const twicReliableWritein_t * const characteristics_value)
{
  uint16_t len, i;
  twicStatus_t ret;
  uint8_t *mover;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);

  mover = twic_container.param + 2;
  for (i = 0; i < total; i++) {
    TWIC_SETHARF_LE(mover, characteristics_value[i].handle); mover += 2;
    TWIC_SETHARF_LE(mover, characteristics_value[i].offset); mover += 2;
    TWIC_SETHARF_LE(mover, characteristics_value[i].length); mover += 2;
    len = characteristics_value[i].length;
    memcpy(mover, characteristics_value[i].value, len); mover += len;
  }

  len = twicSsLeSetupContainerCm(0x0e, TWIC_SERVICE_ID_GATT_CLIENT, bytes,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICVALUE) || \
  defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR)
twicStatus_t twicSsClientLongWrite(
  const uint8_t seq, twicEntry_t * const conn, const uint8_t op_code,
  const uint16_t handle, const uint16_t offset,
  const uint16_t length, const uint8_t * const value)
{
  uint16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  /* (ATT_MTU_SIZE-2)-(MAX_BUFFER_SIZE-9) */
  if ((conn->instance->mtu - 2) > length ||
      (TWIC_CONTROLLER_TXD_MAX - 9) < length) {
    return TWIC_STATUS_ERROR_PARAMETER;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  TWIC_SETHARF_LE(twic_container.param + 2, handle);
  TWIC_SETHARF_LE(twic_container.param + 4, offset);
  memcpy(twic_container.param + 6, value, length);
  len = 6 + length;

  len = twicSsLeSetupContainerCm(
    op_code, TWIC_SERVICE_ID_GATT_CLIENT, len, TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}
#endif

twicStatus_t twicSsLeDiscoverable(
  const uint8_t seq, twicEntry_t * const conn, const uint16_t min_interval,
  const uint16_t max_interval, const uint8_t advertising_type,
  const uint8_t own_address_type, const uint8_t direct_address_type,
  const uint64_t * const direct_address, const uint8_t advertising_channel_map,
  const uint8_t advertising_filter_policy,
  const uint8_t advertising_data_length,
  const uint8_t * const advertising_data,
  const uint8_t scan_resp_data_length, const uint8_t * const scan_resp_data)
{
  uint8_t *mover;
  uint16_t len;
  twicStatus_t ret;
  uint16_t idx;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  twicSsLeRequestHighPower();

  mover = twic_container.param;
  TWIC_SETHARF_LE(mover, min_interval);
  mover += 2;
  TWIC_SETHARF_LE(mover, max_interval);
  mover += 2;
  *mover++ = advertising_type;
  *mover++ = own_address_type;
  *mover++ = direct_address_type;

  for (idx = 0; idx < 6; idx++)
    *mover++ = (uint8_t)(*direct_address >> (idx * 8));

  *mover++ = advertising_channel_map;
  *mover++ = advertising_filter_policy;
  *mover++ = advertising_data_length;
  memcpy(mover, advertising_data, advertising_data_length);
  mover += 31;
  *mover++ = scan_resp_data_length;
  memcpy(mover, scan_resp_data, scan_resp_data_length);
  len = twicSsLeSetupContainerCm(
    0x08, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 0x4F,
    TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);
  if (TWIC_STATUS_OK == ret &&
      TWIC_ADV_TYPE_NONCONN_IND == advertising_type ||
      TWIC_ADV_TYPE_SCAN_IND == advertising_type) {
    conn->instance->opmd.ncad = true;
  }

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

twicStatus_t twicSsGattTransmit(const uint8_t seq, twicEntry_t * const conn,
                                const uint8_t op_code, const uint8_t service_id,
                                const uint16_t handle, const uint8_t length,
                                const uint8_t * const data)
{
  int16_t len;
  twicStatus_t ret;

  if (twic_container.u.tcu.service_id != TWIC_SERVICE_ID_NOP) {
#if defined(TWIC_SS_TRACE_SERVICE_ID_NOP)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (!(twicHashFind1stHandle(conn->handle))) {
    twicLog("No handle.\r\n");
    return TWIC_STATUS_DISCONNECTED;
  }

  len = length;

  if ((conn->instance->mtu - 3) < length) {
    len = conn->instance->mtu - 3;
  }

  twicSsLeRequestHighPower();

  TWIC_SETHARF_LE(twic_container.param, conn->handle);
  TWIC_SETHARF_LE(twic_container.param + 2, handle);
  memcpy(twic_container.param + 4, data, len);

  len = twicSsLeSetupContainerCm(op_code, service_id, len + 4,
                                 TWIC_ENTRY_MNG_TCU, conn, seq);

  ret = twicSsLeTxDataCm(conn, len);

  twic_container.u.tcu.service_id = TWIC_SERVICE_ID_NOP;

  return ret;
}

static TZ1K_INLINE twicStatus_t twicSsEnqueRcvEvent(twicPipeCont_t *event)
{
  uint8_t np;

  np = twic_eq.pot;
  if (++np >= TWIC_SRQ_Q_SIZE) np = 0;
  if (np == twic_eq.top) return TWIC_STATUS_ERROR_RESOURCE;

  twic_eq.buf[twic_eq.pot].pidx = event->pidx;
  twic_eq.buf[twic_eq.pot].event = event->event;
  twic_eq.buf[twic_eq.pot].bytes = event->bytes;
  if (++twic_eq.pot >= TWIC_SRQ_Q_SIZE) twic_eq.pot = 0;

  return TWIC_STATUS_OK;
}

static TZ1K_INLINE twicStatus_t twicSsDequeRcvEvent(twicPipeCont_t *event)
{
  uint8_t np;

  if (twic_eq.top == twic_eq.pot) return TWIC_STATUS_OK;
  np = twic_eq.top;

  event->pidx = twic_eq.buf[np].pidx;
  event->event = twic_eq.buf[np].event;
  event->bytes = twic_eq.buf[np].bytes;
  if (++np >= TWIC_SRQ_Q_SIZE) np = 0;
  twic_eq.top = np;

  return TWIC_STATUS_EVENT_MESSAGE;
}

static TZ1K_INLINE twicStatus_t twicSsRcvQueueStatus(void)
{
  uint8_t np = twic_eq.pot;

  if (twic_eq.top == np) return TWIC_STATUS_OK;

  if (++np >= TWIC_SRQ_Q_SIZE) np = 0;
  if (np == twic_eq.top) return TWIC_STATUS_ERROR_RESOURCE;

  return TWIC_STATUS_EVENT_MESSAGE;
}

twicStatus_t twicSsDataConstructionCm(const uint8_t in_data)
{
  uint8_t pidx, ret_event;
  uint16_t bytes;
  twicPipeCont_t in_event;
  twicStatus_t ret_code;
  twicInstance_t *p;

  pidx = twic_eq.pidx;
  bytes = twic_eq.pipe[pidx].bytes;
  if (TWIC_SRQ_BUF_SIZE <= bytes) {
    twic_eq.pipe[pidx].bytes = bytes = 0;
    ret_event = TWIC_RRQ_IN_PACKET_OVERFLOWED;
    ret_code = TWIC_STATUS_EVENT_MESSAGE;
  } else {
    twic_eq.pipe[pidx].buf[bytes] = in_data;
    twic_eq.pipe[pidx].bytes = ++bytes;

    if ((uint16_t)twic_eq.pipe[pidx].buf[0] < bytes) {
      ret_event = TWIC_RRQ_IN_PACKET_OVERFLOWED;
      ret_code = TWIC_STATUS_EVENT_MESSAGE;
      twic_eq.pipe[pidx].bytes = bytes = 0;
    } else if ((uint16_t)twic_eq.pipe[pidx].buf[0] == bytes) {
      ret_event = TWIC_RRQ_IN_PACKET_COMPLETION;
      ret_code = TWIC_STATUS_EVENT_MESSAGE;
    } else {
      return TWIC_STATUS_OK; /* IDLE */
    }
  }

  in_event.pidx = pidx;
  in_event.event = ret_event;
  in_event.bytes = bytes;

  if (TWIC_STATUS_OK == twicSsEnqueRcvEvent(&in_event)) {
    if (TWIC_RRQ_IN_PACKET_COMPLETION == ret_event)
      twic_eq.pidx = ((TWIC_SRQ_PIPE_SIZE <= ++pidx) ? 0 : pidx);
  } else {
    ret_code = TWIC_STATUS_ERROR_RESOURCE;
    tz1utListEach(twicInstance_t, p, TZ1UT_LIST(srv_instance_root), sibling)
      twicLog("Queue full ERROR: Unprocessed event is 0x%x.\r\n", p->token);
  }
  twic_eq.pipe[twic_eq.pidx].bytes = 0; /* setup for the next incoming d */

  return ret_code;
}

twicStatus_t twicSsDataConstructionHc(const uint8_t in_data)
{
  uint16_t size, bytes;
  uint8_t pidx, status, event;
  twicPipeCont_t in_event;
  twicStatus_t ret;

  pidx = twic_eq.pidx;
  bytes = twic_eq.pipe[pidx].bytes;
  if (bytes >= TWIC_SRQ_BUF_SIZE) {
    twic_eq.pipe[pidx].bytes = bytes = 0;
    event = TWIC_RRQ_IN_PACKET_OVERFLOWED;
    goto set_rcv_event;
  }

  event = TWIC_RRQ_IN_PACKET_IDLE;
  ret = TWIC_STATUS_OK;
  twic_eq.pipe[pidx].buf[bytes] = in_data;
  twic_eq.pipe[pidx].bytes = ++bytes;

  status = TWIC_RRQ_IN_PACKET_HC_START;
  if (TWIC_SRQ_HC_POS_INDICATOR == bytes) {
    switch (twic_eq.pipe[pidx].buf[0]) {
    case TWIC_SRQ_INDICATOR_HC_CMD:
      twic_eq.pos = TWIC_SRQ_HC_LPOS_CMD;
      twic_eq.size = TWIC_SRQ_HC_LSIZE_CMD;
      break;
    case TWIC_SRQ_INDICATOR_HC_ACL:
      twic_eq.pos = TWIC_SRQ_HC_LPOS_H_ACL;
      twic_eq.size = TWIC_SRQ_HC_LSIZE_ACL;
      break;
    case TWIC_SRQ_INDICATOR_HC_SCO:
      twic_eq.pos = TWIC_SRQ_HC_LPOS_SCO;
      twic_eq.size = TWIC_SRQ_HC_LSIZE_SCO;
      break;
    case TWIC_SRQ_INDICATOR_HC_EVENT:
      twic_eq.pos = TWIC_SRQ_HC_LPOS_EVENT;
      twic_eq.size = TWIC_SRQ_HC_LSIZE_EVENT;
      break;
    default:
      twic_eq.pos = twic_eq.size = 0;
      status = TWIC_RRQ_IN_PACKET_UNKNOWN;
      break;
    }
    goto check_status;
  }
  if (twic_eq.pos > bytes) goto check_status;

  size = (uint16_t)twic_eq.pipe[pidx].buf[twic_eq.pos - 1];
  if (TWIC_SRQ_HC_LSIZE_ACL == twic_eq.size) {
    size <<= 8;
    size |= (uint16_t)twic_eq.pipe[pidx].buf[twic_eq.pos - 2];
  }
  if (size < bytes - twic_eq.pos) status = TWIC_RRQ_IN_PACKET_OVERFLOWED;
  else {
    if (size == bytes - twic_eq.pos)
      status = TWIC_RRQ_IN_PACKET_COMPLETION;
  }

  check_status:

  switch (status) {
  case TWIC_RRQ_IN_PACKET_HC_START:
    break;
  case TWIC_RRQ_IN_PACKET_COMPLETION:
    event = status;
    break;
  default: /* Error */
    twic_eq.pipe[pidx].bytes = bytes = 0;
    event = status;
    break;
  }

  set_rcv_event:

  if (TWIC_RRQ_IN_PACKET_IDLE == event) return ret;

  ret = TWIC_STATUS_EVENT_MESSAGE;
  in_event.pidx = pidx;
  in_event.event = event;
  in_event.bytes = bytes;

  if (TWIC_STATUS_OK == twicSsEnqueRcvEvent(&in_event)) {
    if (TWIC_RRQ_IN_PACKET_COMPLETION == event)
      twic_eq.pidx = ((++pidx >= TWIC_SRQ_PIPE_SIZE) ? 0 : pidx);
  } else {
    ret = TWIC_STATUS_ERROR_RESOURCE;
    twicLog("Queue Full ERROR.\r\n");
  }
  /* setup for the next incoming d */
  twic_eq.pipe[twic_eq.pidx].bytes = 0;

  return ret;
}

static TZ1K_INLINE void
twicSsLeInitRespCm(const twicRespD181Cm_t * const op_d181)
{
  twicEntry_t *entry;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(0xd101, false, 0);
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_d181->status;
  entry->u.cont.len = sizeof(twicBdaddr_t);
  memcpy(entry->u.cont.value, &op_d181->local, sizeof(twicBdaddr_t));
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#if defined(TWIC_API_LEREADWHITELISTSIZE)
static TZ1K_INLINE void twicSsLeReadWhitelistSizeRespCm(
  const twicRespD185Cm_t * const op_d185)
{
  twicEntry_t *entry;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(0xd105, false, 0);
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_d185->status;
  entry->u.cont.len = 1;
  entry->u.cont.value[0] = op_d185->white_list_size;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#endif
#if defined(TWIC_API_LEREADLOCALSUPPORTEDFEATURES)
static TZ1K_INLINE void
twicSsLeReadLocalSupportedFeaturesRespCm(const twicRespD182Cm_t * const op_d182)
{
  twicEntry_t *entry;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(0xd102, false, 0);
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_d182->status;
  entry->u.cont.len = 8;
  memcpy(entry->u.cont.value, &op_d182->le_features, 8);
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#endif
static const twicEntry_t *
twicSsMngSimpleRespCm(const uint8_t result, const uint16_t cid)
{
  twicEntry_t *entry;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(cid, false, 0);
  if (!(entry)) return NULL;
  entry->rsp = entry->seq;
  entry->result = result;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);

  return entry;
}

static TZ1K_INLINE void
twicSsLeConnCompleteEventCm(const twicRespD14cCm_t * const op_d14c)
{
  const twicLeCb_t *le_cb;
  twicConnectionComplete_t *arg;
  twicInstance_t *p;
  twicEntry_t *conn;
  bool central = false;

  /* Search all entries which has registered by the interface index. */
  tz1utListEach(twicInstance_t, p, TZ1UT_LIST(srv_instance_root), sibling) {

    conn = twicHashFind1stHandle(((twicEntry_t *)p->conn)->handle);
    if (conn) continue; /* already has a connection. */

    conn = (twicEntry_t *)p->conn; /* New connection entry. */

    /* ROM5 & ROM6, first come, first served. Registration by
     * connection handle */

    if (0 != op_d14c->status);
    else if (0 == op_d14c->role) { /* GAP Central */
      if (!(TWIC_IFACE_GAP_CREATE_CONNECTION & conn->instance->role)) continue;
      conn->instance->role = TWIC_IFACE_GAP_CENTRAL;
    } else {
      if (!(true == conn->instance->opmd.adve &&
            false == conn->instance->opmd.ncad)) continue;
      conn->instance->role = TWIC_IFACE_GAP_PERIPHERAL;
    }

    if (0 == op_d14c->status) {
      twicHashInsert1stHandle(conn, op_d14c->conn_handle);
      conn->instance->mtu = TWIC_ATT_MTU;
      conn->instance->opmd.conn = true;
      conn->instance->opmd.adve = conn->instance->opmd.ncad = false;
    }

    le_cb = conn->instance->le_cb;
    if (!(le_cb) || !(le_cb->connection_complete)) continue;
    arg = (twicConnectionComplete_t *)&p->resp.value;
    arg->status = op_d14c->status;
    arg->role = op_d14c->role;
    arg->peer_address_type = op_d14c->peer_address_type;
    memcpy(&arg->peer.address, &op_d14c->peer.address, sizeof(twicBdaddr_t));

    arg->conn_interval = op_d14c->conn_interval;
    arg->slave_latency = op_d14c->slave_latency;
    arg->supervision_timeout = op_d14c->supervision_timeout;
    arg->master_clock_accuracy = op_d14c->master_clock_accuracy;
    if (0 == arg->role) {
      central = true;
    }
    le_cb->connection_complete(conn->instance->epr, central, arg);

    break;
  }
}

static TZ1K_INLINE void
twicSsLeConnUpdateEventCm(const twicRespD14eCm_t * const op_d14e)
{
  const twicLeCb_t *le_cb;
  twicEntry_t *conn;
  twicConnectionUpdate_t *arg;
  bool central = false;

  conn = twicHashFind1stHandle(op_d14e->conn_handle);
  if (!(conn) || !(conn->instance)) return;

  le_cb = conn->instance->le_cb;
  if (!(le_cb) || !(le_cb->connection_update)) return;

  arg = (twicConnectionUpdate_t *)&conn->instance->resp.value;
  arg->status = op_d14e->status;
  arg->conn_interval = op_d14e->conn_interval;
  arg->conn_latency = op_d14e->conn_latency;
  arg->supervision_timeout = op_d14e->supervision_timeout;
  if (conn->instance->role & TWIC_IFACE_GAP_CENTRAL ||
      conn->instance->role & TWIC_IFACE_GAP_CREATE_CONNECTION) {
    central = true;
  }
  le_cb->connection_update(conn->instance->epr, central, arg);
}

#if defined(TWIC_BLE_HWIP_V41)
static TZ1K_INLINE twicStatus_t
twicSsLeConnParameterEventCm(const twicRespD15eCm_t * const op_d15e)
{
  twicEntry_t *conn, *signal;
  const twicLeCb_t *le_cb;
  twicConnectionParameter_t *arg;
  bool central = false;

  conn = twicHashFind1stHandle(op_d15e->conn_handle);
  if (!(conn) || !(conn->instance)) return TWIC_STATUS_OK;

#if defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
  (void)le_cb;
  (void)arg;
  (void)central;

  signal = conn->instance->signal;
  if (0 != signal->req) return TWIC_STATUS_UNDER_PROCESSING;

  if (0 != conn->req &&
      (TWIC_LECEL2CAPSIGNALACCEPTCONNECTIONUPDATE == conn->seq ||
       TWIC_LECONNECTIONUPDATE == conn->seq ||
       TWIC_LECEL2CAPSIGNALACCEPTCONNECTIONUPDATE == conn->seq ||
       TWIC_LECONNECTIONPARAMETERREQUESTREPLY == conn->seq ||
       TWIC_LECONNECTIONUPDATE == conn->seq)) {
    if (TWIC_STATUS_WAITING_FOR_ACTIVATION ==
        twicSsTreReq(TWIC_LECEL2CAPSIGNALREJECTCONNECTIONUPDATE, signal, 0x16,
                     TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 0x01)) {
      return TWIC_STATUS_UNDER_PROCESSING;
    }
  } else {
    if (TWIC_STATUS_WAITING_FOR_ACTIVATION ==
        twicSsLeConnectionUpdate(TWIC_LECONNECTIONPARAMETERREQUESTREPLY, signal,
                                 0x1e, op_d15e->conn_interval_min,
                                 op_d15e->conn_interval_max,
                                 op_d15e->conn_latency,
                                 op_d15e->supervision_timeout, 0, 0)) {
      return TWIC_STATUS_UNDER_PROCESSING;
    }
  }
  return TWIC_STATUS_OK;
#else
  (void)signal;

  le_cb = conn->instance->le_cb;
  if (!(le_cb) || !(le_cb->connection_parameter)) return TWIC_STATUS_OK;

  arg = (twicConnectionParameter_t *)&conn->instance->resp.value;
  arg->conn_interval_min = op_d15e->conn_interval_min;
  arg->conn_interval_max = op_d15e->conn_interval_max;
  arg->conn_latency = op_d15e->conn_latency;
  arg->supervision_timeout = op_d15e->supervision_timeout;
  if (conn->instance->role & TWIC_IFACE_GAP_CENTRAL ||
      conn->instance->role & TWIC_IFACE_GAP_CREATE_CONNECTION) {
    central = true;
  }
  le_cb->connection_parameter(conn->instance->epr, central, arg);
  return TWIC_STATUS_OK;
#endif
}
#endif

static TZ1K_INLINE void
twicSsLeReadRemoteUsedFeaturesEventCm(const twicRespD14fCm_t * const op_d14f)
{
  const twicLeCb_t *le_cb;
  twicEntry_t *conn;
  twicRemoteUsedFeatures_t *arg;

  conn = twicHashFind1stHandle(op_d14f->conn_handle);
  if (!(conn) || !(conn->instance)) return;

  le_cb = conn->instance->le_cb;
  if (!(le_cb) || !(le_cb->remote_used_features)) return;

  arg = (twicRemoteUsedFeatures_t *)&conn->instance->resp.value;
  arg->status = op_d14f->status;
  memcpy(arg->le_features, op_d14f->le_features, 8);
  le_cb->remote_used_features(conn->instance->epr, arg);
}

void twicSsLeReadRemoteVersionRespCm(const twicRespD159Cm_t * const op_d159)
{
  const twicLeCb_t *le_cb;
  twicEntry_t *conn;
  twicRemoteVersion_t *arg;

  conn = twicHashFind1stHandle(op_d159->conn_handle);
  if (!(conn) || !(conn->instance)) return;

  le_cb = conn->instance->le_cb;
  if (!(le_cb) || !(le_cb->remote_version)) return;

  arg = (twicRemoteVersion_t *)&conn->instance->resp.value;
  arg->status = op_d159->status;
  arg->version = op_d159->version;
  arg->manufacturer_name = op_d159->manufacturer_name;
  arg->sub_version = op_d159->sub_version;
  le_cb->remote_version(conn->instance->epr, arg);
}

void twicSsLeAcceptCm(const twicRespD1F1Cm_t * const op_d1f1)
{
  uint16_t cid = (uint16_t)op_d1f1->service_id << 8 | op_d1f1->op_code;
  twicEntry_t *entry, *signal;

  /* ENTRY_TYPE_MNG, ENTRY_MNG_TCU, ENTRY_TYPE_SIGNAL */
  entry = twicSsEtEntry(cid, false, 0);
  if (!(entry)) return;

  switch (cid) {
  case ((uint16_t)TWIC_SERVICE_ID_CONNECTION_MANAGEMENT << 8 | 0x0C):
    if (!(op_d1f1->status)) {
      entry->instance->opmd.conn = true;
      entry->instance->opmd.adve = entry->instance->opmd.ncad = false;
      entry->instance->role |= TWIC_IFACE_GAP_CREATE_CONNECTION;
    }
    break;
  case ((uint16_t)TWIC_SERVICE_ID_GATT_SERVER << 8 | 0x05):
    if (!(op_d1f1->status)) entry->instance->group.b.notification = true;
    break;
  case ((uint16_t)TWIC_SERVICE_ID_GATT_SERVER << 8 | 0x06):
    if (!(op_d1f1->status)) entry->instance->group.b.indication = true;
    break;
  case ((uint16_t)TWIC_SERVICE_ID_SMP_INITIATOR << 8 | 0x08):
    entry->instance->group.b.smp_i_bonding_enabled_event = false;
#if defined(TWIC_SS_DOZE_DEBUG)
      twicPrintf("Unlock TWIC_TCU_LE_SMP_I_BONDING_ENABLED_EVENT\r\n");
#endif
    break;
  case ((uint16_t)TWIC_SERVICE_ID_SMP_RESPONDER << 8 | 0x01):
    entry->instance->group.b.smp_r_pairing_event = false;
#if defined(TWIC_SS_DOZE_DEBUG)
    twicPrintf("Unlock TWIC_TCU_LE_SMP_R_PAIRING_EVENT\r\n");
#endif
    break;
#if defined(GROUP_CLI_EXG_MTU_EVENT)
  case ((uint16_t)TWIC_SERVICE_ID_GATT_CLIENT << 8 | 0x01):
    if (!(op_d1f1->status)) entry->instance->group.b.cli_exg_mtu_event = true;
    break;
#endif
  case ((uint16_t)TWIC_SERVICE_ID_CONNECTION_MANAGEMENT << 8 | 0x0E):
  case ((uint16_t)TWIC_SERVICE_ID_CONNECTION_MANAGEMENT << 8 | 0x16):
    signal = entry->instance->signal;
    signal->rsp = 0xFF;
    signal->req = 0;
    signal->result = 0;
    signal->id = 0;
    signal->seq = 0;
    signal->u.cont.len = 0;
    tz1utListDel(&signal->o_link);
    tz1utListInit(&signal->o_link);
    break;
  default: break;
  }

  entry->rsp = entry->seq;
  entry->result = op_d1f1->status;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}

void twicSsLeDisconnEventCm(const twicRespD193Cm_t * const op_d193)
{
  const twicLeCb_t *le_cb;
  twicEntry_t *conn, *p, *q = NULL;
  twicDisconnection_t *arg;
  bool central = false;

  conn = twicHashFind1stHandle(op_d193->conn_handle);
  if (!(conn))
    return;

  twicHashRemove1stHandle(conn->handle);

  if (conn->instance && conn->instance->server_cb) {
    le_cb = conn->instance->le_cb;
    if (le_cb->disconnection) {
      arg = (twicDisconnection_t *)&conn->instance->resp.value;
      arg->status = op_d193->status;
      arg->reason = op_d193->reason;
      if (TWIC_IFACE_GAP_CENTRAL & conn->instance->role ||
          TWIC_IFACE_GAP_CREATE_CONNECTION & conn->instance->role) {
        central = true;
      }
      le_cb->disconnection(conn->instance->epr, central, arg);
    }
  }

  tz1utListEach(twicEntry_t, p, TZ1UT_LIST(srv_req_root), o_link) {
    /* All of the TWIC_ENTRY_TYPE_MNG such as 0xd305 and 0xd306 is
     * treated with the Disconnect Error. (ROM5) */
    if ((TWIC_ENTRY_TYPE_MNG == p->type || TWIC_ENTRY_TYPE_SIGNAL) &&
        p->handle == op_d193->conn_handle) {
      if (q) tz1utListInit(&q->o_link);
      p->rsp = p->seq;
      p->result = 0xA4;
      tz1utListDel(&p->o_link);
      q = p;
    }
  }
  if (q) tz1utListInit(&q->o_link);

  conn->instance->mtu = TWIC_ATT_MTU;
  conn->instance->role = TWIC_IFACE_GAP_UNDEFINED;
  conn->instance->opmd.conn = false;
  conn->instance->opmd.adve = conn->instance->opmd.ncad = false;
  conn->instance->group.u16 = 0;
  conn->instance->token = 0;
}

void twicSsLeReadTxPowLevelRespCm(const twicRespD194Cm_t * const op_d194)
{
  const twicLeCb_t *le_cb;
  twicEntry_t *entry;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(0xd114, true, op_d194->conn_handle);
  if (!(entry)) return;
  le_cb = entry->instance->le_cb;
  if (entry->instance && le_cb && le_cb->read_result_tx_power &&
      !(op_d194->status))
    le_cb->read_result_tx_power(entry->instance->epr,
                                op_d194->transmit_pow_Level);

  entry->rsp = entry->seq;
  entry->result = op_d194->status;
  entry->u.cont.len = 1;
  entry->u.cont.value[0] = (uint8_t)op_d194->transmit_pow_Level;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}

void twicSsLeReadRssiRespCm(const twicRespD195Cm_t * const op_d195)
{
  const twicLeCb_t *le_cb;
  twicEntry_t *entry;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(0xd115, true, op_d195->conn_handle);
  if (!(entry)) return;
  le_cb = entry->instance->le_cb;
  if (entry->instance && le_cb && le_cb->read_result_rssi &&
      !(op_d195->status))
    le_cb->read_result_rssi(entry->instance->epr, op_d195->rssi_value);

  entry->rsp = entry->seq;
  entry->result = op_d195->status;
  entry->u.cont.len = 1;
  entry->u.cont.value[0] = (uint8_t)op_d195->rssi_value;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}

#if defined(TWIC_CONFIG_ENABLE_SCAN)
static TZ1K_INLINE void
twicSsLeAdvReportEventCm(const twicRespD1C1Cm_t * const op_d1c1)
{
  const twicLeCb_t *le_cb;
  twicAdvReport_t *arg;
  twicInstance_t *p;
  uint8_t *mover, i, num_reports;

  /* Search all entries which has registered by the interface index. */
  tz1utListEach(twicInstance_t, p, TZ1UT_LIST(srv_instance_root), sibling) {
    le_cb = p->le_cb;
    if (!(le_cb) || !(le_cb->adv_report)) continue;

    arg = (twicAdvReport_t *)&p->resp.value;
    num_reports = arg->num_reports = op_d1c1->num_reports;

    if (TWIC_NUM_REPORTS_ROM5 < num_reports) {
      twicTrace();
      num_reports = TWIC_NUM_REPORTS_ROM5;
    }

    mover = (uint8_t *)op_d1c1->reports;
    for (i = 0; i < num_reports; i ++)
      arg->reports[i].adv_event_type = *mover ++;
    for (i = 0; i < num_reports; i ++) {
      arg->reports[i].address_type_random = (0x01 == *mover) ? true : false;
      mover++;
    }
    for (i = 0; i < num_reports; i ++) {
      memcpy(arg->reports[i].bd.address, mover, sizeof(twicBdaddr_t));
      mover += sizeof(twicBdaddr_t);
    }
    for (i = 0; i < num_reports; i ++) arg->reports[i].length = *mover++;
    for (i = 0; i < num_reports; i ++) {
      memcpy(arg->reports[i].data, mover, arg->reports[i].length);
      mover += arg->reports[i].length;
    }
    for (i = 0; i < num_reports; i ++) arg->reports[i].rssi = *mover++;

    if (true == p->opmd.scan) le_cb->adv_report(p->epr, arg);
  }
}
#endif

static TZ1K_INLINE void twicSsLeCeL2capConnectionUpdateRespEventCm(
  const twicRespD150Cm_t * const op_d150)
{
  const twicLeCb_t *le_cb;
  twicEntry_t *conn;
  uint8_t *arg;
  bool central = false;

  conn = twicHashFind1stHandle(op_d150->conn_handle);
  if (!(conn) || !(conn->instance)) return;

  le_cb = conn->instance->le_cb;
  if (!(le_cb) || !(le_cb->l2cap_connection_update)) return;

  arg = (uint8_t *)&conn->instance->resp.value;
  *arg = op_d150->status;
  if (conn->instance->role & TWIC_IFACE_GAP_CENTRAL ||
      conn->instance->role & TWIC_IFACE_GAP_CREATE_CONNECTION) {
    central = true;
  }
  le_cb->l2cap_connection_update(conn->instance->epr, central, *arg);
}

static TZ1K_INLINE void
twicSsLeCeL2capUpdateConnReqAcceptRespCm(const twicRespD151Cm_t * const op_d151)
{
  twicEntry_t *entry, *signal;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU,  TWIC_ENTRY_TYPE_SIGNAL */
  entry = twicSsEtEntry(0xd116, true, op_d151->handle);
  if (!(entry)) return;

  entry->rsp = entry->seq;
  entry->result = op_d151->status;
  entry->u.cont.len = 0;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);

  signal = entry->instance->signal;
  signal->rsp = 0xFF;
  signal->req = 0;
  signal->result = 0;
  signal->id = 0;
  signal->seq = 0;
  signal->u.cont.len = 0;
  tz1utListDel(&signal->o_link);
  tz1utListInit(&signal->o_link);
}

static TZ1K_INLINE twicStatus_t
twicSsLeCeL2capUpdateConnReqEventCm(const twicRespD152Cm_t * const op_d152)
{
  twicEntry_t *conn, *signal;

  conn = twicHashFind1stHandle(op_d152->conn_handle);
  if (!(conn) || !(conn->instance)) return TWIC_STATUS_OK;

  signal = conn->instance->signal;
  if (0 != signal->req) return TWIC_STATUS_UNDER_PROCESSING;

  if (0 != conn->req &&
      (TWIC_LECEL2CAPSIGNALACCEPTCONNECTIONUPDATE == conn->seq ||
       TWIC_LECONNECTIONUPDATE == conn->seq ||
       TWIC_LECEL2CAPSIGNALACCEPTCONNECTIONUPDATE == conn->seq ||
       TWIC_LECONNECTIONPARAMETERREQUESTREPLY == conn->seq ||
       TWIC_LECONNECTIONUPDATE == conn->seq)) {
    if (TWIC_STATUS_WAITING_FOR_ACTIVATION ==
        twicSsTreReq(TWIC_LECEL2CAPSIGNALREJECTCONNECTIONUPDATE, signal, 0x16,
                     TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, 0x01)) {
      return TWIC_STATUS_UNDER_PROCESSING;
    }
  } else {
    if (TWIC_STATUS_WAITING_FOR_ACTIVATION ==
        twicSsLeConnectionUpdate(TWIC_LECEL2CAPSIGNALACCEPTCONNECTIONUPDATE,
                                 signal, 0x16,
                                 op_d152->conn_interval_min,
                                 op_d152->conn_interval_max,
                                 op_d152->conn_latency,
                                 op_d152->supervision_timeout, 0, 0)) {
      return TWIC_STATUS_UNDER_PROCESSING;
    }
  }
  return TWIC_STATUS_OK;
}

static TZ1K_INLINE void twicSsLeCeRejectL2capConnectionUpdateRespEventCm(
    const twicResp8xCm_t * const op_d153)
{
  const twicLeCb_t *le_cb;
  twicEntry_t *conn;
  uint8_t *arg;
  bool central = false;

  conn = twicHashFind1stHandle(op_d153->conn_handle);
  if (!(conn) || !(conn->instance)) return;

  le_cb = conn->instance->le_cb;
  if (!(le_cb) || !(le_cb->reject_l2cap_connection_update)) return;

  arg = (uint8_t *)&conn->instance->resp.value;
  *arg = op_d153->status;
  if (conn->instance->role & TWIC_IFACE_GAP_CENTRAL ||
      conn->instance->role & TWIC_IFACE_GAP_CREATE_CONNECTION) {
    central = true;
  }
  le_cb->reject_l2cap_connection_update(conn->instance->epr, central, *arg);
}
#if defined(TWIC_API_LELMGENRESOLVABLEBDADDR)
static
void twicSsLeGenResolvableBdaddrRespCm(const twicRespD154Cm_t * const op_d154)
{
  const twicLeCb_t *le_cb;
  twicEntry_t *entry;
  twicPrivacy_t *arg;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(0xd117, false, 0);
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_d154->status;
  entry->u.cont.len = 22;
  memcpy(entry->u.cont.value, op_d154->bd_address, 6);
  memcpy(entry->u.cont.value + 6, op_d154->local_irk, 16);

  le_cb = entry->instance->le_cb;
  if (entry->instance && le_cb && le_cb->resolvable_privacy) {
    arg = (twicPrivacy_t *)&entry->instance->resp.value;
    memcpy(arg, &op_d154->status, sizeof(twicPrivacy_t));
    le_cb->resolvable_privacy(entry->instance->epr, arg);
  }

  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#endif
#if defined(TWIC_API_LELMRESOLVEBDADDR)
static void twicSsLeResolveBdaddrRespCm(const twicRespD155Cm_t * const op_d155)
{
  const twicLeCb_t *le_cb;
  twicEntry_t *entry;
  twicPrivacy_t *arg;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(0xd118, false, 0);
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_d155->status;
  entry->u.cont.len = 22;
  memcpy(entry->u.cont.value, op_d155->bd_address, 6);
  memcpy(entry->u.cont.value + 6, op_d155->local_irk, 16);

  le_cb = entry->instance->le_cb;
  if (entry->instance && le_cb && le_cb->resolved_privacy) {
    arg = (twicPrivacy_t *)&entry->instance->resp.value;
    memcpy(arg, &op_d155->status, sizeof(twicPrivacy_t));
    le_cb->resolved_privacy(entry->instance->epr, arg);
  }

  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#endif
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
static void twicSsLeEncryptRespCm(const twicRespD15cCm_t * const op_d15c)
{
  const twicLeCb_t *le_cb;
  twicEntry_t *entry;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(0xd11c, false, 0);
  if (!(entry)) return;

  le_cb = entry->instance->le_cb;
  if (entry->instance && le_cb && le_cb->encrypt) {
    le_cb->encrypt(entry->instance->epr, op_d15c->encrypted_data);
  }

  entry->rsp = entry->seq;
  entry->result = op_d15c->status;
  entry->u.cont.len = 16;
  memcpy(entry->u.cont.value, op_d15c->encrypted_data, 16);

  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#endif

static TZ1K_INLINE twicEntry_t *
twicSsLeResp8xCm(const twicResp8xCm_t *const op_8x, const uint16_t cid)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(cid, true, op_8x->conn_handle);
  if (!(entry)) return NULL;

  entry->rsp = entry->seq;
  entry->result = op_8x->status;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);

  return entry;
}

#if defined(TWIC_API_LEREADCHANNELMAP)
static TZ1K_INLINE void
twicSsLeReadChannelMapRespCm(const twicRespD191Cm_t * const op_d191)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(0xd111, true, op_d191->conn_handle);
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_d191->status;
  entry->u.cont.len = 5;
  memcpy(entry->u.cont.value, &op_d191->channel_map, 5);
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#endif
#if defined(TWIC_API_LEREADSUPPORTEDSTATES)
static TZ1K_INLINE void
twicSsLeReadSupportedStatesRespCm(const twicRespD192Cm_t * const op_d192)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(0xd112, true, op_d192->conn_handle);
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_d192->status;
  entry->u.cont.len = 8;
  memcpy(entry->u.cont.value, &op_d192->states, 8);
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#endif
static void twicSsLeNotAcceptCm(const twicRespD1F2Cm_t * const op_d1f2)
{
  uint16_t cid = (uint16_t)op_d1f2->service_id << 8 | op_d1f2->op_code;
  twicEntry_t *entry;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(cid, false, 0);
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = 0xf2;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
  twicTrace();
}

static TZ1K_INLINE void
twicSsLeFatalErrorCm(const twicRespD1FeCm_t * const op_d1fe)
{
  twicEntry_t *p, *q = NULL;

  tz1utListEach(twicEntry_t, p, TZ1UT_LIST(srv_req_root), o_link) {
    if (q) tz1utListInit(&q->o_link);
    p->rsp = p->seq; /* for all id(s) */
    p->result = 0xfe;
    tz1utListDel(&p->o_link);
    q = p;
  }
  if (q) tz1utListInit(&q->o_link);
  twicTrace();
}

static TZ1K_INLINE void twicSsLeCompleteEventHc(
  const twicRespHc_t * const op, const uint16_t ocf_ogf)
{
  twicEntry_t *p, *q = NULL;
  tz1smHalStatus_t ret;

  tz1utListEach(twicEntry_t, p, TZ1UT_LIST(srv_req_root), o_link) {
    if (TWIC_ENTRY_TYPE_MNG == p->type && p->id == ocf_ogf) {
      if (q) tz1utListInit(&q->o_link);
      p->result = op->u_event.value[8];
      switch (p->req) {
      case TWIC_ENTRY_MNG_PCP:
        if (p->result) twic_eq.sniff.lowpower_enable = false;
        else if (true == twic_eq.sniff.lowpower_requirements)
          twic_eq.sniff.lowpower_enable = true;
        else twic_eq.sniff.lowpower_enable = false;
        if (!(p->result)) twic_eq.sniff.pin = twic_eq.sniff.pin_rq;
        break;
      case TWIC_ENTRY_MNG_POWER_MODE:
        if (!(p->result)) twic_eq.lowpower_mode = twic_eq.lowpower_selector;
        break;
      case TWIC_ENTRY_MNG_HOST_DELAY: /* 0xfc08 . 0xa000 */
        if (!(p->result)) twic_eq.host_delay = twic_eq.hdr;
        break;
      case TWIC_ENTRY_MNG_POWER_INIT:
      case TWIC_ENTRY_MNG_PSB:
      case TWIC_ENTRY_MNG_PSPW:
      case TWIC_ENTRY_MNG_PC:
      case TWIC_ENTRY_MNG_MA:
      case TWIC_ENTRY_MNG_SET_TX_POWER:
        break;
      case TWIC_ENTRY_MNG_FC:
        if (!(p->result)) twic_eq.sniff.fc = twic_eq.sniff.fc_rq;
        else {
          ret = tz1smHalUartControl(twic_eq.br, twic_eq.sniff.fc);
          if (TZ1SM_HAL_STATUS_OK != ret) twic_eq.h4 |= RPQUEUE_H4_HAL_ERROR;
          else twic_eq.h4 &= ~RPQUEUE_H4_HAL_ERROR;
        }
        p->result |= twic_eq.h4;
        break;
      case TWIC_ENTRY_MNG_BR:
        if (!(p->result)) {
          twic_eq.br = twic_eq.br_rq;
          ret = tz1smHalUartControl(twic_eq.br, twic_eq.sniff.fc);
          if (TZ1SM_HAL_STATUS_OK != ret) twic_eq.h4 |= RPQUEUE_H4_HAL_ERROR;
          else twic_eq.h4 &= ~RPQUEUE_H4_HAL_ERROR;
        }
        p->result |= twic_eq.h4;
        break;
      case TWIC_ENTRY_MNG_POWER_CLOCK:
        if (!(p->result)) twic_eq.sniff.lowpower_clock = true;
        break;
#if defined(TWIC_API_LECEDBUSWRITE)
      case TWIC_ENTRY_MNG_DW: /* FALLTHROUGH */
#endif
#if defined(TWIC_API_LECEMEMORYWRITE)
      case TWIC_ENTRY_MNG_MW: /* FALLTHROUGH */
#endif
      case TWIC_ENTRY_MNG_WRITE_BDADDR:
        p->result = op->u_event.value[3];
        break;
      case TWIC_ENTRY_MNG_READ_BDADDR:
        p->result = op->u_event.value[3];
        p->u.cont.len = 6;
        memcpy(p->u.cont.value, op->u_event.value + 4, 0x6);
        break;
      case TWIC_ENTRY_MNG_READ_LVI:
        p->result = op->u_event.value[3];
        p->u.cont.len = 8;
        /* HCI_Version        : 1 byte
         * HCI_Revision       : 2 byte
         * LMP/PAL_Version    : 1 byte
         * Manufacturer_Name  : 2 byte
         * LMP/PAL_Subversion : 2 byte */
        memcpy(p->u.cont.value, op->u_event.value + 4, 0x8);
        break;
      case TWIC_ENTRY_MNG_TCUMODE: /* 0xfc08 . 0x9900 */
        twic_eq.type = TWIC_TCU;
        p->result = op->u_event.value[3];
        break;
#if defined(TWIC_API_LECEDBUSREAD)
      case TWIC_ENTRY_MNG_DR:
        p->result = op->u_event.value[3];
        p->u.cont.len = 2;
        /* ADDR 0xXX, [VALUE 0xXXXX] */
        memcpy(p->u.cont.value, op->u_event.value + 5, 0x2);
        break;
#endif
#if defined(TWIC_API_LECEMEMORYREAD)
      case TWIC_ENTRY_MNG_MR:
        /* op->u_event.value[2] must be TWIC_ENTRY_MNG_MR */
        p->result = op->u_event.value[3];
        p->u.cont.len = 2;
        /* CODE 0xXX, [VALUE 0xXXXX] */
        memcpy(p->u.cont.value, op->u_event.value + 5, 0x2); /* VALUE */
        break;
#endif
#if defined(TWIC_API_LECEREADFWVER)
      case TWIC_ENTRY_MNG_FV:
        /* op->u_event.value[7] must be TWIC_ENTRY_MNG_FV */
        p->u.cont.len = 21;
        /* 0x0F(Strings) 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
                         0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 */
        memcpy(p->u.cont.value, op->u_event.value + 9, 21);
        break;
#endif
      default:
        p->result = 0xE1;
        break;
      }
      p->rsp = p->seq;
      tz1utListDel(&p->o_link);
      q = p;
    }
  }
  if (q) tz1utListInit(&q->o_link);
}

static TZ1K_INLINE void
twicSsLeCompleteEventWithHwerrHc(const twicRespHc_t * const op)
{
  twicEntry_t *p, *q = NULL;

  /*
    Byte 0 10 Event code                (event_code)
    Byte 1 01 Command length            (parameter_lenght)
    Byte 2 XX Errors                    (value[0])
    20: Short of receiving packet.
    Timer of maximum transmit interval between each bytes is
    expired. Timer value is 5ms fixed. If this error code
    occurs, check the transmitting Byte interval from HOST.
    21: Stop bit error
    It is generated, if the clock deviation of UART is large.
    Please check the deviation of a clock,
    if this error code occurs. SPEC of our company is TDB.
    22: Over write error
    Since data was received from HOST during RTS control
    (data stop request), overwrite of data has occurred
    inside. If this error code occurs, please check the flow
    control of a HOST side.
  */

  tz1utListEach(twicEntry_t, p, TZ1UT_LIST(srv_req_root), o_link) {
    if (TWIC_ENTRY_TYPE_MNG == p->type) {
      if (q) tz1utListInit(&q->o_link);
      switch (p->req) {
      case TWIC_ENTRY_MNG_MA:
      case TWIC_ENTRY_MNG_SET_TX_POWER:
      case TWIC_ENTRY_MNG_PSB:
      case TWIC_ENTRY_MNG_PSPW:
      case TWIC_ENTRY_MNG_PC:
      case TWIC_ENTRY_MNG_FC:
      case TWIC_ENTRY_MNG_BR:
      case TWIC_ENTRY_MNG_POWER_INIT:
      case TWIC_ENTRY_MNG_HOST_DELAY:
      case TWIC_ENTRY_MNG_POWER_CLOCK:
      case TWIC_ENTRY_MNG_PCP:
      case TWIC_ENTRY_MNG_POWER_MODE:
      case TWIC_ENTRY_MNG_WRITE_BDADDR:
      case TWIC_ENTRY_MNG_READ_BDADDR:
      case TWIC_ENTRY_MNG_TCUMODE:
        twicLog("TWIC_ENTRY_MNG (%d) failed.\r\n", p->req);
        p->result = op->u_event.value[0];
        break;
      default:
        p->result = 0xE1;
        break;
      }
      p->rsp = p->seq;
      tz1utListDel(&p->o_link);
      q = p;
    }
  }
  if (q) tz1utListInit(&q->o_link);
}

static TZ1K_INLINE void
twicSsGattSdbAddPrimSvcRespCm(const twicRespAxCm_t * const op_a0)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(0xd320, false, 0); /* First come, first served. */
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_a0->status;
  if (!(entry->result)) {
    entry->handle = op_a0->handle; /* primary_service_handle */
    entry->u.attr.p_handle = op_a0->handle;
    entry->ap = entry->req;
  }
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}

static TZ1K_INLINE void
twicSsGattSdbAddSecSvcRespCm(const twicRespAxCm_t * const op_a1)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(0xd321, false, 0); /* First come, first served. */
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_a1->status;
  if (!(entry->result)) {
    entry->handle = op_a1->handle; /* secondary_service_handle */
    entry->ap = entry->req;
  }
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}

static TZ1K_INLINE void
twicSsGattSdbAddIncSvcRespCm(const twicRespAxCm_t * const op_a4)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(0xd324, false, 0); /* First come, first served. */
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_a4->status;
  if (!(entry->result)) {
    /* include_service_handle */
    /* twicLog("op_a4->handle = 0x%x.\r\n", op_a4->handle); */
    entry->ap = entry->req;
  }
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}

static TZ1K_INLINE void
twicSsGattSdbAddCharDeclRespCm(const twicRespAxCm_t * const op_a2)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(0xd322, false, 0); /* First come, first served. */
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_a2->status;
  if (!(entry->result)) {
    entry->handle = op_a2->handle; /* characteristic_declaration_handle */
    entry->ap = entry->req;
  }
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}

static TZ1K_INLINE void
twicSsGattSdbAddCharEleRespCm(const twicRespAxCm_t * const op_a3)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(0xd323, false, 0); /* First come, first served. */
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_a3->status;
  if (!(entry->result)) {
    /* characteristic_element_handle */
    twicHashInsert2ndHandle(entry, op_a3->handle);
    entry->ap = entry->req;
  }
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#if defined(TWIC_API_LEGATTDBSETCHARACTERISTICSVL) || \
  defined(TWIC_API_LEGATTDBUPDCHARACTERISTICSVL) ||   \
  defined(TWIC_API_LEGATTDBSETDESCRIPTORVL)
static TZ1K_INLINE void
twicSsGattSdbAddCharVarEleRespCm(const twicRespAxCm_t * const op_ac)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(0xd32C, false, 0); /* First come, first served. */
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_ac->status;
  if (!(entry->result)) {
    /* characteristic_element_handle */
    twicHashInsert2ndHandle(entry, op_ac->handle);
    entry->ap = entry->req;
  }
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#endif
static TZ1K_INLINE void
twicSsGattSdbRetEndGrpHleRespCm(const twicRespAxCm_t * const op_a6)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(0xd326, false, 0); /* First come, first served. */
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_a6->status;
  if (!(entry->result)) {
    /* end_group_handle */
    /* twicHashInsert2ndHandle(entry, op_a6->handle); */
    entry->u.attr.b_handle = op_a6->handle;
    entry->ap = entry->req;
  }
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}

static TZ1K_INLINE void
twicSsGattSdbUpdCharEleRespCm(const twicRespA5Cm_t * const op_a5)
{
  twicEntry_t *entry;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(0xd325, false, 0);
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_a5->status;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#if defined(TWIC_API_LEGATTDBSETPERMISSIONS)
static TZ1K_INLINE void
twicSsGattSdbSetAttrPermsRespCm(const twicRespA9Cm_t * const op_a9)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(0xd329, false, 0);
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_a9->status;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#endif
#if defined(TWIC_API_LEGATTDBGETPERMISSIONS)
static TZ1K_INLINE void
twicSsGattSdbGetAttrPermsRespCm(const twicRespAaCm_t * const op_aa)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(0xd32a, false, 0);
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_aa->status;
  if (!(entry->result))
    entry->u.attr.permissions = op_aa->attribute_permissions;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#endif
static TZ1K_INLINE void
twicSsSysInvalidCommandCm(const twicRespE1FFCm_t * const op_e1ff)
{
  twicSsLeNotAcceptCm(op_e1ff);
}

static TZ1K_INLINE void
twicSsVenSetModuleMaintenanceEventCm(const twicResp41Cm_t * const op_41)
{
  twicEntry_t *entry;
  tz1smHalStatus_t ret;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(0xef01, false, 0);
  entry->result = op_41->status;

  switch (op_41->information_id) {
  case TWIC_ENTRY_MNG_PCP:
    if (entry->result) twic_eq.sniff.lowpower_enable = false;
    else if (true == twic_eq.sniff.lowpower_requirements)
      twic_eq.sniff.lowpower_enable = true;
    else twic_eq.sniff.lowpower_enable = false;
    if (!(entry->result)) twic_eq.sniff.pin = twic_eq.sniff.pin_rq;
    break;
  case TWIC_ENTRY_MNG_POWER_MODE:
    if (!(entry->result)) twic_eq.lowpower_mode = twic_eq.lowpower_selector;
    break;
  case TWIC_ENTRY_MNG_HOST_DELAY:
    if (!(entry->result)) twic_eq.host_delay = twic_eq.hdr;
    break;
  case TWIC_ENTRY_MNG_BR:
    if (!(entry->result)) {
      twic_eq.br = twic_eq.br_rq;
      ret = tz1smHalUartControl(twic_eq.br, twic_eq.sniff.fc);
      if (TZ1SM_HAL_STATUS_OK != ret) twic_eq.h4 |= RPQUEUE_H4_HAL_ERROR;
      else twic_eq.h4 &= ~RPQUEUE_H4_HAL_ERROR;
    }
    entry->result |= twic_eq.h4;
    break;
  }

  entry->rsp = entry->seq;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}

#if defined(TWIC_API_GATTCLIENTEXGMTU)
static TZ1K_INLINE twicStatus_t
twicSsGattCliExgMtuEventCm(const twicRespD241Cm_t * const op_d241)
{
  const twicLeClientCb_t *client_cb;
  twicMtuExchangeResult_t *arg;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d241->conn_handle);
  if (!(conn) || !(conn->instance)) return TWIC_STATUS_ERROR_DATA;

#if defined(GROUP_CLI_EXG_MTU_EVENT)
  conn->instance->group.b.cli_exg_mtu_event = false;
#endif

  conn->instance->mtu = op_d241->negotiated_mtu_size;
  client_cb = conn->instance->client_cb;
  if (client_cb && client_cb->mtu_exchange_result) {
    arg = (twicMtuExchangeResult_t *)&conn->instance->resp.value;
    arg->status = op_d241->status;
    arg->negotiated_mtu_size = op_d241->negotiated_mtu_size;
    client_cb->mtu_exchange_result(conn->instance->epr, arg);
  }

  return TWIC_STATUS_OK;
}
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICE)
static TZ1K_INLINE void
twicSsGattCliDiscoverPrimSvcEventCm(const twicRespD242Cm_t * const op_d242)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  bool next;
  uint8_t len, group_index, processed;
  const uint8_t *mover;
  uint8_t number_of_groups, length_of_each_group;
  uint16_t attribute_handle, end_group_handle;
  twicUuid_t uuid;
  uint64_t uuid_lsb, uuid_msb;

  conn = twicHashFind1stHandle(op_d242->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->primary_service)) return;

  if (0x06 == op_d242->parameter_length) {
    client_cb->primary_service(
      conn->instance->epr, (0 == op_d242->status) ? 0x0A : op_d242->status,
      false, op_d242->u.error_handle, 0, 0, 0, 0, 0);
    return;
  }

  next = true;
  mover = op_d242->u.attr;
  len = op_d242->parameter_length - 4;
  processed = 0;
  do {
    number_of_groups = *mover++;
    length_of_each_group = *mover++;
    processed += number_of_groups * length_of_each_group + 2;
    for (group_index = 0; group_index < number_of_groups; group_index ++) {
      if ((len <= processed) && (0 == op_d242->continue_flag) &&
          ((number_of_groups - 1) == group_index)) {
        next = false;
      }
      attribute_handle = *(uint16_t *)mover; mover += 2;
      end_group_handle = *(uint16_t *)mover; mover += 2;
      memset(&uuid, 0, sizeof(twicUuid_t));
      if (6 == length_of_each_group) {
        uuid.len = TWIC_UUID16;
      } else if (20 == length_of_each_group) {
        uuid.len = TWIC_UUID128;
      } else {
        client_cb->primary_service(
          conn->instance->epr, (0 == op_d242->status) ? 0x0A : op_d242->status,
          false, 0, 0, 0, 0, 0, 0);
        return;
      }
      memcpy(&uuid.uu, mover, uuid.len); mover += uuid.len;
      uuid_lsb = twicUuidLsb(&uuid);
      uuid_msb = twicUuidMsb(&uuid);
      client_cb->primary_service(
        conn->instance->epr, op_d242->status, next, 0, attribute_handle,
        end_group_handle, uuid_lsb, uuid_msb, uuid.len);
    }

  } while (len > processed);
}
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID)
static TZ1K_INLINE void
twicSsGattCliDiscoverPrimSvcByUuidEventCm(twicRespD243Cm_t * op_d243)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  bool next;
  uint8_t len, group_index, processed;
  uint8_t *mover;
  uint8_t number_of_groups, length_of_each_group;
  uint16_t attribute_handle, end_group_handle;

  conn = twicHashFind1stHandle(op_d243->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->primary_service_by_uuid)) return;

  if (0x06 == op_d243->parameter_length) {
    client_cb->primary_service_by_uuid(
      conn->instance->epr, (0 == op_d243->status) ? 0x0A : op_d243->status,
      false, op_d243->u.error_handle, 0, 0);
    return;
  }

  next = true;
  mover = op_d243->u.attr;
  len = op_d243->parameter_length - 4;
  processed = 0;
  do {
    number_of_groups = *mover++;
    length_of_each_group = *mover++;
    processed += number_of_groups * length_of_each_group + 2;
    for (group_index = 0; group_index < number_of_groups; group_index ++) {
      if ((len <= processed) && (0 == op_d243->continue_flag) &&
          ((number_of_groups - 1) == group_index)) {
        next = false;
      }
      if (4 != length_of_each_group) {
        client_cb->primary_service_by_uuid(
          conn->instance->epr, (0 == op_d243->status) ? 0x0A : op_d243->status,
          false, op_d243->u.error_handle, 0, 0);
        return;
      }
      attribute_handle = *(uint16_t *)mover; mover += 2;
      end_group_handle = *(uint16_t *)mover; mover += 2;
      client_cb->primary_service_by_uuid(
        conn->instance->epr, op_d243->status, next, 0, attribute_handle,
        end_group_handle);
    }
  } while (len > processed);
}
#endif
#if defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE)
static TZ1K_INLINE void
twicSsGattCliFindInclSvcEventCm(twicRespD244Cm_t * op_d244)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  bool next;
  uint8_t len, group_index, processed;
  const uint8_t *mover;
  uint8_t number_of_groups, length_of_each_group;
  uint16_t attribute_handle;
  uint16_t included_service_attribute_handle;
  uint16_t included_service_end_group_handle;
  twicUuid_t uuid;
  uint64_t uuid_lsb, uuid_msb;

  conn = twicHashFind1stHandle(op_d244->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->included_service)) return;

  if (0x06 == op_d244->parameter_length) { /* error */
    /* status, next, error handle */
    client_cb->included_service(
      conn->instance->epr, (0 == op_d244->status) ? 0x0A : op_d244->status,
      false, op_d244->u.error_handle, 0, 0, 0, 0, 0, 0);
    return;
  }

  next = true;
  mover = op_d244->u.attr;
  len = op_d244->parameter_length - 4;
  processed = 0;
  do {
    number_of_groups = *mover++;
    length_of_each_group = *mover++;
    processed += number_of_groups * length_of_each_group + 2;
    for (group_index = 0; group_index < number_of_groups; group_index ++) {
      if ((len <= processed) && (0 == op_d244->continue_flag) &&
          ((number_of_groups - 1) == group_index)) {
        next = false;
      }
      attribute_handle = *(uint16_t *)mover; mover += 2;
      included_service_attribute_handle = *(uint16_t *)mover; mover += 2;
      included_service_end_group_handle = *(uint16_t *)mover; mover += 2;
      memset(&uuid, 0, sizeof(twicUuid_t));
      if (8 == length_of_each_group) {
        uuid.len = TWIC_UUID16;
      } else if (6 == length_of_each_group) {
        /* When UUID is of 16 bytes, the read by type response will
         * not contain the UUID and hence the UUID shall not be
         * contained in the event. */
        uuid.len = 0;
      } else {
        client_cb->included_service(
          conn->instance->epr, (0 == op_d244->status) ? 0x0A : op_d244->status,
          false, 0, 0, 0, 0, 0, 0, 0);
        return;
      }
      memcpy(&uuid.uu, mover, uuid.len); mover += uuid.len;
      uuid_lsb = twicUuidLsb(&uuid);
      uuid_msb = twicUuidMsb(&uuid);
      client_cb->included_service(
        conn->instance->epr, op_d244->status, next, 0, attribute_handle,
        included_service_attribute_handle, included_service_end_group_handle,
        uuid_lsb, uuid_msb, uuid.len);
    }
  } while (len > processed);
}
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS)
static TZ1K_INLINE void
twicSsGattCliDiscoverCharDeclEventCm(twicRespD245Cm_t * op_d245)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  bool next;
  uint8_t len, group_index, processed;
  const uint8_t *mover;
  uint8_t number_of_groups, length_of_each_group, char_properties;
  uint16_t attribute_handle, char_value_handle;
  twicUuid_t uuid;
  uint64_t uuid_lsb, uuid_msb;

  conn = twicHashFind1stHandle(op_d245->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->all_char_of_service)) return;

  if (0x06 == op_d245->parameter_length) { /* error */
    client_cb->all_char_of_service(
      conn->instance->epr, (0 == op_d245->status) ? 0x0A : op_d245->status,
      false, op_d245->u.error_handle, 0, 0, 0, 0, 0, 0);
    return;
  }

  next = true;
  mover = op_d245->u.attr;
  len = op_d245->parameter_length - 4;
  processed = 0;
  do {
    number_of_groups = *mover++;
    length_of_each_group = *mover++;
    processed += number_of_groups * length_of_each_group + 2;
    for (group_index = 0; group_index < number_of_groups; group_index ++) {
      if ((len <= processed) && (0 == op_d245->continue_flag) &&
          ((number_of_groups - 1) == group_index)) {
        next = false;
      }
      attribute_handle = *(uint16_t *)mover; mover += 2;
      char_properties = *mover++;
      char_value_handle = *(uint16_t *)mover; mover += 2;
      memset(&uuid, 0, sizeof(twicUuid_t));
      if (7 == length_of_each_group) {
        uuid.len = TWIC_UUID16;
      } else if (21 == length_of_each_group) {
        uuid.len = TWIC_UUID128;
      } else {
        client_cb->all_char_of_service(
          conn->instance->epr, (0 == op_d245->status) ? 0x0A : op_d245->status,
          false, 0, 0, 0, 0, 0, 0, 0);
        return;
      }
      memcpy(&uuid.uu, mover, uuid.len); mover += uuid.len;
      uuid_lsb = twicUuidLsb(&uuid);
      uuid_msb = twicUuidMsb(&uuid);
      client_cb->all_char_of_service(
        conn->instance->epr, op_d245->status, next, 0, attribute_handle,
        char_properties, char_value_handle, uuid_lsb, uuid_msb, uuid.len);
    }
  } while (len > processed);
}
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)
static TZ1K_INLINE void
twicSsGattCliDiscoverCharDeclByUuidEventCm(const twicRespD246Cm_t * op_d246)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  bool next;
  uint8_t len, group_index, processed;
  const uint8_t *mover;
  uint8_t number_of_groups, length_of_each_group, char_properties;
  uint16_t attribute_handle, char_value_handle;
  twicUuid_t uuid;
  uint64_t uuid_lsb, uuid_msb;

  conn = twicHashFind1stHandle(op_d246->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->char_by_uuid)) return;

  if (0x06 == op_d246->parameter_length) { /* error */
    client_cb->char_by_uuid(
      conn->instance->epr, (0 == op_d246->status) ? 0x0A : op_d246->status,
      false, op_d246->u.error_handle, 0, 0, 0, 0, 0, 0);
    return;
  }

  next = true;
  mover = op_d246->u.attr;
  len = op_d246->parameter_length - 4;
  processed = 0;
  do {
    number_of_groups = *mover++;
    length_of_each_group = *mover++;
    processed += number_of_groups * length_of_each_group + 2;
    for (group_index = 0; group_index < number_of_groups; group_index ++) {
      if ((len <= processed) && (0 == op_d246->continue_flag) &&
          ((number_of_groups - 1) == group_index)) {
        next = false;
      }
      attribute_handle = *(uint16_t *)mover; mover += 2;
      char_properties = *mover++;
      char_value_handle = *(uint16_t *)mover; mover += 2;
      memset(&uuid, 0, sizeof(twicUuid_t));
      if (7 == length_of_each_group) {
        uuid.len = TWIC_UUID16;
      } else if (21 == length_of_each_group) {
        uuid.len = TWIC_UUID128;
      } else {
        client_cb->char_by_uuid(
          conn->instance->epr, (0 == op_d246->status) ? 0x0A : op_d246->status,
          false, 0, 0, 0, 0, 0, 0, 0);
        return;
      }
      memcpy(&uuid.uu, mover, uuid.len); mover += uuid.len;
      uuid_lsb = twicUuidLsb(&uuid);
      uuid_msb = twicUuidMsb(&uuid);
      client_cb->char_by_uuid(
        conn->instance->epr, op_d246->status, next, 0, attribute_handle,
        char_properties, char_value_handle, uuid_lsb, uuid_msb, uuid.len);
    }
  } while (len > processed);
}
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS)
static TZ1K_INLINE void
twicSsGattCliDiscoverCharDspEventCm(twicRespD247Cm_t * op_d247)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  bool next;
  uint8_t len, group_index, processed;
  const uint8_t *mover;
  uint8_t number_of_groups, length_of_each_group;
  uint16_t attribute_handle;
  twicUuid_t uuid;
  uint64_t uuid_lsb, uuid_msb;

  conn = twicHashFind1stHandle(op_d247->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->all_char_descriptors)) return;

  if (0x06 == op_d247->parameter_length) {
    client_cb->all_char_descriptors(
      conn->instance->epr, (0 == op_d247->status) ? 0x0A : op_d247->status,
      false, op_d247->u.error_handle, 0, 0, 0, 0);
    return;
  }

  next = true;
  mover = op_d247->u.attr;
  len = op_d247->parameter_length - 4;
  processed = 0;
  do {
    number_of_groups = *mover++;
    length_of_each_group = *mover++;
    processed += number_of_groups * length_of_each_group + 2;
    for (group_index = 0; group_index < number_of_groups; group_index ++) {
      if ((len <= processed) && (0 == op_d247->continue_flag) &&
          ((number_of_groups - 1) == group_index)) {
        next = false;
      }
      attribute_handle = *(uint16_t *)mover; mover += 2;
      memset(&uuid, 0, sizeof(twicUuid_t));
      if (4 == length_of_each_group) {
        uuid.len = TWIC_UUID16;
      } else if (18 == length_of_each_group) {
        uuid.len = TWIC_UUID128;
      } else {
        client_cb->all_char_descriptors(
          conn->instance->epr, (0 == op_d247->status) ? 0x0A : op_d247->status,
          false, 0, 0, 0, 0, 0);
        return;
      }
      memcpy(&uuid.uu, mover, uuid.len); mover += uuid.len;
      uuid_lsb = twicUuidLsb(&uuid);
      uuid_msb = twicUuidMsb(&uuid);
      client_cb->all_char_descriptors(
        conn->instance->epr, op_d247->status, next, 0, attribute_handle,
        uuid_lsb, uuid_msb, uuid.len);
    }
  } while (len > processed);
}
#endif
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE)
static TZ1K_INLINE void
twicSsGattCliReadCharValEventCm(const twicRespD248Cm_t * const op_d248)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  twicAttValue_t *arg;
  uint16_t len;

  conn = twicHashFind1stHandle(op_d248->conn_handle);
  if (!(conn) || !(conn->instance)) return;

  client_cb = conn->instance->client_cb;
  if (client_cb && client_cb->char_value_readout) {
    arg = (twicAttValue_t *)&conn->instance->resp.value;
    len = op_d248->parameter_length - 4;
    if ((conn->instance->mtu - 1) < len) { /* [0 , ATT_MTU_SIZE - 1] */
      twicLog("Characteristic value len is too long.\r\n");
      /* Remote Data Length Error (Data from remote greater then MTU Size) */
      client_cb->char_value_readout(conn->instance->epr, 0xA5, arg);
    }
    arg->parameter_length = len;
    memcpy(arg->value, op_d248->u.attr, len);
    client_cb->char_value_readout(conn->instance->epr, op_d248->status, arg);
  }
}
#endif
#if defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID)
static TZ1K_INLINE void
twicSsGattCliReadCharValUuidEventCm(const twicRespD24dCm_t * op_d24d)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  bool next;
  uint8_t group_index, processed;
  const uint8_t *mover;
  uint8_t number_of_groups, length_of_each_group;
  uint16_t char_value_handle;
  twicAttValue_t *arg;
  uint16_t len;

  conn = twicHashFind1stHandle(op_d24d->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->char_value_readout_using_uuid)) return;

  if (0x06 == op_d24d->parameter_length) { /* error */
    client_cb->char_value_readout_using_uuid(
      conn->instance->epr, (0 == op_d24d->status) ? 0x0A : op_d24d->status,
      false, op_d24d->u.error_handle, 0, NULL);
    return;
  }

  next = true;
  mover = op_d24d->u.attr;
  len = op_d24d->parameter_length - 4;
  processed = 0;
  do {
    number_of_groups = *mover++;
    length_of_each_group = *mover++;
    processed += number_of_groups * length_of_each_group + 2;
    for (group_index = 0; group_index < number_of_groups; group_index ++) {
      if ((len <= processed) && (0 == op_d24d->continue_flag) &&
          ((number_of_groups - 1) == group_index)) {
        next = false;
      }
      char_value_handle = *(uint16_t *)mover; mover += 2;
      arg = (twicAttValue_t *)&conn->instance->resp.value;
      if ((conn->instance->mtu - 4) < (length_of_each_group - 2)) {
        /* [0 , ATT_MTU_SIZE - 4] */
        twicLog("Characteristic value len is too long.\r\n");
        /* Remote Data Length Error (Data from remote greater then MTU Size) */
        arg->parameter_length = conn->instance->mtu - 4;
        memcpy(arg->value, mover, conn->instance->mtu - 4);
        if (0 == op_d24d->continue_flag) {
          next = false;
        }
        client_cb->char_value_readout_using_uuid(
          conn->instance->epr, (0 == op_d24d->status) ? 0xA5 : op_d24d->status,
          next, 0, char_value_handle, arg);
        return; /* forced termination about this packet */
      } else {
        arg->parameter_length = length_of_each_group - 2;
        memcpy(arg->value, mover, length_of_each_group - 2);
        mover += arg->parameter_length;
        client_cb->char_value_readout_using_uuid(
          conn->instance->epr, op_d24d->status, next, 0, char_value_handle,
          arg);
      }
    }
  } while (len > processed);
}
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICVALUE)
static TZ1K_INLINE void
twicSsGattCliWriteCharValEventCm(const twicRespD249Cm_t * const op_d249)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d249->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->char_value_writein_response)) return;
  client_cb->char_value_writein_response(conn->instance->epr, op_d249->status);
}
#endif
#if defined(TWIC_API_GATTCLIENTRELIABLEWRITE)
static TZ1K_INLINE void twicSsGattCliReliableWritesEventCm(
  const twicRespD24eCm_t * const op_d24e)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  twicReliableWriteinResp_t *arg;
  uint16_t len;

  conn = twicHashFind1stHandle(op_d24e->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->reliable_writein_confirmation)) return;

  arg = (twicReliableWriteinResp_t *)&conn->instance->resp.value;

  len = op_d24e->parameter_length;
  if (5 > len || 56 < len) {
    client_cb->reliable_writein_confirmation(
      conn->instance->epr, 0xA5, NULL);
    return;
  }

  arg->number_of_char_values = (len - 2) / 3;
  memcpy(arg->char_value, op_d24e->resp, len - 2);
  client_cb->reliable_writein_confirmation(conn->instance->epr, 0x00, arg);
}
#endif
#if defined(TWIC_API_GATTCLIENTWRITEWITHOUTRESPONSE) || \
  defined(TWIC_API_GATTCLIENTSIGNEDWRITEWITHOUTRESPONSE)
static TZ1K_INLINE void twicSsGattCliWriteWithoutResponseCmdEventCm(
  const twicRespD24fCm_t * const op_d24f)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d24f->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->char_vale_writein_started)) return;
  client_cb->char_vale_writein_started(conn->instance->epr, op_d24f->status);
}
#define twicSsGattCliSignedWriteWithoutResponseCmdEventCm \
  twicSsGattCliWriteWithoutResponseCmdEventCm
#endif
#if defined(TWIC_API_GATTCLIENTINDICATIONCONFIRMATIONRESPONSE)
static TZ1K_INLINE void twicSsGattCliCharValConfirmationAcceptRespCm(
  const twicRespD290Cm_t * const op_d290)
{
  twicEntry_t *entry;

  entry = twicSsEtEntry(0xd210, true, op_d290->conn_handle);
  if (!(entry)) return;
  entry->rsp = entry->seq;
  entry->result = op_d290->status;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);
}
#endif
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR)
static TZ1K_INLINE void
twicSsGattCliReadCharDespEventCm(const twicRespD24aCm_t * const op_d24a)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  twicAttValue_t *arg;
  uint16_t len;

  conn = twicHashFind1stHandle(op_d24a->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->char_desp_readout)) return;

  if (0 != op_d24a->status) { /* error */
    client_cb->char_desp_readout(
      conn->instance->epr, op_d24a->status, NULL);
    return;
  }

  len = op_d24a->parameter_length - 4;
  arg = (twicAttValue_t *)&conn->instance->resp.value;
  if ((conn->instance->mtu - 1) < len) { /* [0 , ATT_MTU_SIZE - 1] */
    twicLog("Characteristic value len is too long.\r\n");
    /* Remote Data Length Error (Data from remote greater then MTU Size) */
    arg->parameter_length = conn->instance->mtu - 1;
    memcpy(arg->value, op_d24a->u.attr, conn->instance->mtu - 1);
    client_cb->char_desp_readout(conn->instance->epr, 0xA5, arg);
    return; /* forced termination about this packet */
  } else {
    arg->parameter_length = len;
    memcpy(arg->value, op_d24a->u.attr, len);
    client_cb->char_desp_readout(conn->instance->epr, op_d24a->status, arg);
  }
}
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR)
static TZ1K_INLINE void
twicSsGattCliWriteCharDespEventCm(const twicRespD24bCm_t * const op_d24b)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d24b->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->char_desp_writein_response)) return;
  client_cb->char_desp_writein_response(conn->instance->epr, op_d24b->status);
}
#endif
#if defined(TWIC_API_GATTCLIENTREADMULTIPLECHARVALUES)
static TZ1K_INLINE void twicSsGattCliReadMultipleCharValuesEventCm(
  const twicRespD24cCm_t * const op_d24c)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  twicAttValue_t *arg;
  uint16_t len;

  conn = twicHashFind1stHandle(op_d24c->conn_handle);
  if (!(conn) || !(conn->instance)) return;

  client_cb = conn->instance->client_cb;
  if (client_cb && client_cb->multiple_char_values_readout) {
    arg = (twicAttValue_t *)&conn->instance->resp.value;
    len = op_d24c->parameter_length - 4;
    if ((conn->instance->mtu - 1) < len) { /* [0 , ATT_MTU_SIZE - 1] */
      twicLog("Characteristic value len is too long.\r\n");
      arg->parameter_length = conn->instance->mtu - 1;
      memcpy(arg->value, op_d24c->u.attr, conn->instance->mtu - 1);
      /* Remote Data Length Error (Data from remote greater then MTU Size) */
      client_cb->multiple_char_values_readout(conn->instance->epr, 0xA5, arg);
    }
    arg->parameter_length = len;
    memcpy(arg->value, op_d24c->u.attr, len);
    client_cb->multiple_char_values_readout(
      conn->instance->epr, op_d24c->status, arg);
  }
}
#endif
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE)
static TZ1K_INLINE void
twicSsGattCliReadLongCharValEventCm(const twicRespD252Cm_t * const op_d252)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  twicAttValue_t *arg;
  uint16_t len;
  bool next;

  conn = twicHashFind1stHandle(op_d252->conn_handle);
  if (!(conn) || !(conn->instance)) return;

  client_cb = conn->instance->client_cb;
  if (client_cb && client_cb->long_char_value_readout) {
    next = (0 == op_d252->continue_flag) ? false : true;
    arg = (twicAttValue_t *)&conn->instance->resp.value;
    len = op_d252->parameter_length - 4;
    if (TWIC_GATT_ATTR_MAX_LEN < len) { /* [0 , MAX_BUFFER_SIZE] */
      twicLog("Characteristic value len is too long.\r\n");
      /* Remote Data Length Error (Data from remote greater then MTU Size) */
      arg->parameter_length = TWIC_GATT_ATTR_MAX_LEN;
      memcpy(arg->value, op_d252->u.attr, TWIC_GATT_ATTR_MAX_LEN);
      client_cb->long_char_value_readout(conn->instance->epr, 0xA5, next, arg);
    }
    arg->parameter_length = len;
    memcpy(arg->value, op_d252->u.attr, len);
    client_cb->long_char_value_readout(conn->instance->epr, op_d252->status,
                                       next, arg);
  }
}
#endif
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR)
static TZ1K_INLINE void
twicSsGattCliReadLongCharDespEventCm(const twicRespD253Cm_t * const op_d253)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  twicAttValue_t *arg;
  uint16_t len;
  bool next;

  conn = twicHashFind1stHandle(op_d253->conn_handle);
  if (!(conn) || !(conn->instance)) return;

  client_cb = conn->instance->client_cb;
  if (client_cb && client_cb->long_char_desp_readout) {
    next = (0 == op_d253->continue_flag) ? false : true;
    arg = (twicAttValue_t *)&conn->instance->resp.value;
    len = op_d253->parameter_length - 4;
    if (TWIC_GATT_ATTR_MAX_LEN < len) { /* [0 , MAX_BUFFER_SIZE] */
      twicLog("Characteristic value len is too long.\r\n");
      /* Remote Data Length Error (Data from remote greater then MTU Size) */
      arg->parameter_length = TWIC_GATT_ATTR_MAX_LEN;
      memcpy(arg->value, op_d253->u.attr, TWIC_GATT_ATTR_MAX_LEN);
      client_cb->long_char_desp_readout(conn->instance->epr, 0xA5, next, arg);
    }
    arg->parameter_length = len;
    memcpy(arg->value, op_d253->u.attr, len);
    client_cb->long_char_desp_readout(conn->instance->epr, op_d253->status,
                                      next, arg);
  }
}
#endif
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICVALUE)
static TZ1K_INLINE void
twicSsGattCliWriteLongCharValEventCm(const twicRespD254Cm_t * const op_d254)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d254->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->long_char_value_writein_response)) return;
  client_cb->long_char_value_writein_response(conn->instance->epr,
                                              op_d254->status);
}
#endif
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR)
static TZ1K_INLINE void
twicSsGattCliWriteLongCharDespEventCm(const twicRespD255Cm_t * const op_d255)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_d255->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->long_char_desp_writein_response)) return;
  client_cb->long_char_desp_writein_response(conn->instance->epr,
                                             op_d255->status);
}
#endif

static TZ1K_INLINE void
twicSsGattCliInvalidRespCm(const twicRespD256Cm_t * const op_d256)
{
  twicEntry_t *p, *q = NULL;
  const uint16_t kill_id = TWIC_SERVICE_ID_GATT_CLIENT << 8;

  tz1utListEach(twicEntry_t, p, TZ1UT_LIST(srv_req_root), o_link) {
    if (kill_id == (p->id & 0xFF00)) {
      if (q) tz1utListInit(&q->o_link);
      p->rsp = p->seq;
      p->result = 0x56;
      tz1utListDel(&p->o_link);
      q = p;
    }
  }
  if (q) tz1utListInit(&q->o_link);
}

static TZ1K_INLINE void twicSsGattCliCharValNotificationIndEventCm(
  const twicRespD2C0Cm_t * const op_d2c0)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  twicAttValue_t *arg;
  uint16_t len;

  conn = twicHashFind1stHandle(op_d2c0->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->notification_received)) return;

  len = op_d2c0->parameter_length - 4;
  arg = (twicAttValue_t *)&conn->instance->resp.value;
  if (0 == len) {
    client_cb->notification_received(conn->instance->epr, 0xA5, op_d2c0->handle,
                                     NULL);
  } else if ((conn->instance->mtu - 3) < len) { /* [0 , ATT_MTU_SIZE - 3] */
    twicLog("Characteristic value len is too long.\r\n");
    /* Remote Data Length Error (Data from remote greater then MTU Size) */
    arg->parameter_length = conn->instance->mtu - 3;
    memcpy(arg->value, op_d2c0->value, conn->instance->mtu - 3);
    client_cb->notification_received(conn->instance->epr, 0xA5, op_d2c0->handle,
                                     arg);
    return; /* forced termination about this packet */
  } else {
    arg->parameter_length = len;
    memcpy(arg->value, op_d2c0->value, len);
    client_cb->notification_received(conn->instance->epr, 0x00, op_d2c0->handle,
                                     arg);
  }
}

static TZ1K_INLINE void
twicSsGattCliCharValIndicationIndEventCm(const twicRespD2D0Cm_t * const op_d2d0)
{
  const twicLeClientCb_t *client_cb;
  twicEntry_t *conn;
  twicAttValue_t *arg;
  uint16_t len;

  conn = twicHashFind1stHandle(op_d2d0->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  client_cb = conn->instance->client_cb;
  if (!(client_cb) || !(client_cb->indication_received)) return;

  len = op_d2d0->parameter_length - 4;
  arg = (twicAttValue_t *)&conn->instance->resp.value;
  if (0 == len) {
    client_cb->notification_received(conn->instance->epr, 0xA5, op_d2d0->handle,
                                     NULL);
  } else if ((conn->instance->mtu - 3) < len) { /* [0 , ATT_MTU_SIZE - 3] */
    twicLog("Characteristic value len is too long.\r\n");
    /* Remote Data Length Error (Data from remote greater then MTU Size) */
    arg->parameter_length = conn->instance->mtu - 3;
    memcpy(arg->value, op_d2d0->value, conn->instance->mtu - 3);
    client_cb->indication_received(conn->instance->epr, 0xA5, op_d2d0->handle,
                                   arg);
    return; /* forced termination about this packet */
  } else {
    arg->parameter_length = len;
    memcpy(arg->value, op_d2d0->value, len);
    client_cb->indication_received(conn->instance->epr, 0x00, op_d2d0->handle,
                                   arg);
  }
}

static TZ1K_INLINE void
twicSsGattSerNotificationEventCm(const twicResp45Cm_t * const op_45)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_45->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  server_cb = conn->instance->server_cb;
  if (!(server_cb) || !(server_cb->notification_sent)) return;
  server_cb->notification_sent(conn->instance->epr);
  conn->instance->group.b.notification = false;
}

static TZ1K_INLINE void
twicSsGattSerIndicationEventCm(const twicResp46Cm_t * const op_46)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_46->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  server_cb = conn->instance->server_cb;
  if (!(server_cb) || !(server_cb->indication_confirmation)) return;
  server_cb->indication_confirmation(conn->instance->epr, op_46->status);
  conn->instance->group.b.indication = false;
}

static TZ1K_INLINE void
twicSsGattSerExecuteCompleteEvent(const twicResp47Cm_t * const op_47)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;

  conn = twicHashFind1stHandle(op_47->conn_handle);
  if (!(conn) || !(conn->instance)) return;
  server_cb = conn->instance->server_cb;
  if (!(server_cb) || !(server_cb->queued_writes_complete)) return;
  server_cb->queued_writes_complete(conn->instance->epr, op_47->status);
}

/* need twicIfGattServerExgMtuResponse */
static TZ1K_INLINE twicStatus_t
twicSsGattSerExgMtuEventCm(const twicRespC1Cm_t * const op_c1)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;
  uint16_t *arg;

  conn = twicHashFind1stHandle(op_c1->conn_handle);
  if (!(conn)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  if (conn->instance->token) return TWIC_STATUS_UNDER_PROCESSING;
  conn->instance->token = TWIC_TCU_LE_GATT_SER_EXG_MTU_EVENT;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_GATT_SER_EXG_MTU_EVENT\r\n");
#endif

  server_cb = conn->instance->server_cb;
  if (server_cb && server_cb->mtu_exchange_demand) {
    arg = (uint16_t *)&conn->instance->resp.value;
    *arg = op_c1->client_rx_mtu_size;
    server_cb->mtu_exchange_demand(conn->instance->epr, *arg);
  }

  return TWIC_STATUS_OK;
}

static TZ1K_INLINE twicEntry_t *
twicSsGattSerExgMtuAcceptRespCm(const twicResp81Cm_t * const op_81)
{
  const twicLeServerCb_t *server_cb;
  twicMtuExchangeResult_t *arg;
  twicEntry_t *entry;

  /* TWIC_ENTRY_TYPE_MNG, TWIC_ENTRY_MNG_TCU */
  entry = twicSsEtEntry(0xd301, true, op_81->conn_handle);
  if (!(entry)) return NULL;
  entry->instance->mtu = op_81->negotiated_mtu_size;
  server_cb = entry->instance->server_cb;
  if (entry->instance && server_cb && server_cb->mtu_exchange_result) {
    arg = (twicMtuExchangeResult_t *)&entry->instance->resp.value;
    arg->status = op_81->status;
    arg->negotiated_mtu_size = op_81->negotiated_mtu_size;
    server_cb->mtu_exchange_result(entry->instance->epr, arg);
  }

  entry->rsp = entry->seq;
  entry->result = op_81->status;
  tz1utListDel(&entry->o_link);
  tz1utListInit(&entry->o_link);

  return entry;
}

/* need twicIfGattServerCharDespWriteInResponse */
static TZ1K_INLINE twicStatus_t
twicSsGattSerWriteCharDespEventCm(const twicRespCyCm_t * const op_c4)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;
  twicAttValue_t *arg;
  uint16_t len;

  /* characteristic_descriptor_handle */
  conn = twicHashFind2ndHandle(op_c4->handle);
  if (!(conn)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  if (conn->instance->token) return TWIC_STATUS_UNDER_PROCESSING;
  conn->instance->token = TWIC_TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT\r\n");
#endif

  server_cb = conn->instance->server_cb;
  if (server_cb && server_cb->char_desp_writein_demand) {
    arg = (twicAttValue_t *)&conn->instance->resp.value;
    len = op_c4->parameter_length - 4; /* [0 , ATT_MTU_SIZE - 3] */
    if ((conn->instance->mtu - 3) < len) {
      twicLog("Characteristic descriptor len is too long.\r\n");
      return TWIC_STATUS_ERROR_DATA;
    } else if (0x00 == len) {
      twicLog("Characteristic descriptor len is 0.\r\n");
      return TWIC_STATUS_ERROR_DATA;
    }
    arg->parameter_length = len;
    memcpy(arg->value, op_c4->value, len);
    server_cb->char_desp_writein_demand(
      conn->instance->epr, arg, conn->u.attr.eidx);
  }

  return TWIC_STATUS_OK;
}

/* This event is generated by GATT Server when Write Without Response
 * Command and Signed Write Command has been received from the Client
 * Device.
 *
 * When Write Without Response Command is received by GATT Server
 * below check is made: If the handle is invalid or Write Without
 * Response Bit in Characteristic Properties Flag for is not set to
 * one or authentication and encryption permission are not available,
 * then request event will not be sent to server application and
 * directly GATT server shall discard this request.
 *
 * On receiving this event the server application can update the value
 * for the specified characteristic value handle with the received
 * characteristic value to server database using the API
 * twicIfGattServerWriteCharacteristics.
 *
 * If authorization is required by the remote device to write this
 * char value and host application do not want to authorize the
 * corresponding remote device, then the host shall not update this
 * value and can simply discard/ignore the event.  When Signed Write
 * Without Command is received by GATT Server below check is made:If
 * the handle is invalid or Authenticated Signed Write Bit in
 * Characteristic Properties Flag for is not set to one or Client and
 * Server device do not share a bond then this request event will not
 * be sent to server application and directly GATT server shall
 * discard this request.
 *
 * On receiving this event the server application can update the value
 * for the specified characteristic value handle with the received
 * characteristic value to server database using the API
 * twicIfGattServerWriteCharacteristics.  If authorization is required
 * by the remote device to write this char value and host application
 * do not want to authorize the corresponding remote device, then the
 * host shall not update this value and can simply discard/ignore the
 * event.  For Write Without Response Event there is no Accept Request
 * since GATT Server do not need to send response to Remote Client.
 *
 * The Write Without Response Event is generated by GATT Server for
 * below GATT Request from Remote Client:
 * 1. Write Without Response Command
 * 2. Signed Write Without Response Command
 */
static TZ1K_INLINE twicStatus_t
twicSsGattSerWriteCharWithoutResponseEventCm(const twicRespCyCm_t * const op_c9)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;
  twicAttValue_t *arg;
  uint16_t len;

  /* char_value_handle */
  conn = twicHashFind2ndHandle(op_c9->handle);
  if (!(conn) || !(conn->instance)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  server_cb = conn->instance->server_cb;
  if (!(server_cb) || !(server_cb->char_val_writein_post)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  arg = (twicAttValue_t *)&conn->instance->resp.value;
  len = op_c9->parameter_length - 4; /* [0 , ATT_MTU_SIZE - 3] */
  if ((conn->instance->mtu - 3) < len) {
    twicLog("Characteristic descriptor len is too long.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  } else if (0x00 == len) {
    twicLog("Characteristic descriptor len is 0.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }
  arg->parameter_length = len;
  memcpy(arg->value, op_c9->value, len);
  server_cb->char_val_writein_post(conn->instance->epr, arg, conn->u.attr.eidx);

  return TWIC_STATUS_OK;
}

/* need twicIfGattServerCharValWriteInResponse */
static TZ1K_INLINE twicStatus_t
twicSsGattSerWriteCharValEventCm(const twicRespCyCm_t * const op_c3)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;
  twicAttValue_t *arg;
  uint16_t len;

  /* char_value_handle */
  conn = twicHashFind2ndHandle(op_c3->handle);
  if (!(conn)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  if (conn->instance->token) return TWIC_STATUS_UNDER_PROCESSING;
  conn->instance->token = TWIC_TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT\r\n");
#endif

  server_cb = conn->instance->server_cb;
  if (server_cb && server_cb->char_val_writein_demand) {
    arg = (twicAttValue_t *)&conn->instance->resp.value;
    len = op_c3->parameter_length - 4; /* [0 , ATT_MAX_LEN 100 ]*/
    if (len > TWIC_GATT_ATTR_MAX_LEN) {
      twicLog("Characteristic descriptor len is too long.\r\n");
      return TWIC_STATUS_ERROR_DATA;
    } else if (1 > len) {
      twicLog("Characteristic descriptor len is too short.\r\n");
      return TWIC_STATUS_ERROR_DATA;
    }
    arg->parameter_length = len;
    memcpy(arg->value, op_c3->value, len);
    server_cb->char_val_writein_demand(
      conn->instance->epr, arg, conn->u.attr.eidx);
  }

  return TWIC_STATUS_OK;
}

/* need twicIfGattServerCharDespReadOutResponse */
static TZ1K_INLINE twicStatus_t
twicSsGattSerReadCharDespEventCm(const twicRespCxCm_t * const op_c8)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;

  /* characteristic_descriptor_handle */
  conn = twicHashFind2ndHandle(op_c8->handle);
  if (!(conn)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  if (conn->instance->token) return TWIC_STATUS_UNDER_PROCESSING;
  conn->instance->token = TWIC_TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT\r\n");
#endif

  server_cb = conn->instance->server_cb;
  if (server_cb && server_cb->char_desp_readout_demand)
    server_cb->char_desp_readout_demand(
      conn->instance->epr, conn->u.attr.eidx);

  return TWIC_STATUS_OK;
}

/* need twicIfGattServerCharValReadOutResponse */
static TZ1K_INLINE twicStatus_t
twicSsGattSerReadCharValEventCm(const twicRespCxCm_t * const op_c2)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;

  /* char_value_handle */
  conn = twicHashFind2ndHandle(op_c2->handle);
  if (!(conn)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  if (conn->instance->token) return TWIC_STATUS_UNDER_PROCESSING;
  conn->instance->token = TWIC_TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT\r\n");
#endif

  server_cb = conn->instance->server_cb;
  if (server_cb && server_cb->char_val_readout_demand)
    server_cb->char_val_readout_demand(
      conn->instance->epr, conn->u.attr.eidx);

  return TWIC_STATUS_OK;
}

/* need twicIfGattServerCharValMultiReadOutResponse */
static TZ1K_INLINE twicStatus_t
twicSsGattSerReadMultipleEventCm(const twicRespCaCm_t * const op_ca)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn, *entry;
  twicAttEid_t *arg;
  uint16_t len, hidx, num;

  conn = twicHashFind1stHandle(op_ca->conn_handle);
  if (!(conn)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  if (conn->instance->token) return TWIC_STATUS_UNDER_PROCESSING;
  conn->instance->token = TWIC_TCU_LE_GATT_SER_READ_MULTIPLE_EVENT;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_GATT_SER_READ_MULTIPLE_EVENT\r\n");
#endif

  server_cb = conn->instance->server_cb;
  if (server_cb && server_cb->char_val_multi_readout_demand) {
    arg = (twicAttEid_t *)&conn->instance->resp.value;
    len = op_ca->parameter_length - 2;

    num = 0;
    for (hidx = 0; hidx < len / 2; hidx++) {
      entry = twicHashFind2ndHandle(op_ca->char_value_handles[hidx]);
      if (entry) arg->eidxs[num++] = entry->u.attr.eidx;
    }
    arg->parameter_length = num;
    server_cb->char_val_multi_readout_demand(conn->instance->epr, arg);
  }

  return TWIC_STATUS_OK;
}

#if defined(TWIC_API_GATTSERVERLONGCHARVALREADOUTRESPONSE)
/* need twicIfGattServerLongCharValReadOutResponse op_8d */
static TZ1K_INLINE twicStatus_t
twicSsGattSerReadLongCharValEventCm(const twicRespClCm_t * const op_cd)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;

  /* char_value_handle */
  conn = twicHashFind2ndHandle(op_cd->handle);
  if (!(conn)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  if (conn->instance->token) return TWIC_STATUS_UNDER_PROCESSING;
  conn->instance->token = TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_VAL_EVENT;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_VAL_EVENT\r\n");
#endif

  server_cb = conn->instance->server_cb;
  if (server_cb && server_cb->long_char_val_readout_demand)
    /* characteristic_value_offset */
    server_cb->long_char_val_readout_demand(
      conn->instance->epr, op_cd->offset, conn->u.attr.eidx);

  return TWIC_STATUS_OK;
}
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARDESPREADOUTRESPONSE)
/* need twicIfGattServerLongCharDespReadOutResponse */
static TZ1K_INLINE twicStatus_t
twicSsGattSerReadLongCharDespEventCm(const twicRespClCm_t * const op_ce)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;

  /* characteristic_descriptor_handle */
  conn = twicHashFind2ndHandle(op_ce->handle);
  if (!(conn)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  if (conn->instance->token) return TWIC_STATUS_UNDER_PROCESSING;
  conn->instance->token = TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_DESP_EVENT;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_DESP_EVENT\r\n");
#endif

  server_cb = conn->instance->server_cb;
  if (server_cb && server_cb->long_char_desp_readout_demand)
    /* characteristic_descriptor_offset */
    server_cb->long_char_desp_readout_demand(
      conn->instance->epr, op_ce->offset, conn->u.attr.eidx);

  return TWIC_STATUS_OK;
}
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARVALPREPAREWRITEINRESPONSE)
/* need twicIfGattServerLongCharValPrepWriteInResponse op_8b */
static TZ1K_INLINE twicStatus_t
twicSsGattSerPrepWriteLongRelCharValEventCm(const twicRespClCm_t * const op_cb)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;

  /* char_value_handle */
  conn = twicHashFind2ndHandle(op_cb->handle);
  if (!(conn)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  if (conn->instance->token) return TWIC_STATUS_UNDER_PROCESSING;
  conn->instance->token =
    TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_VAL_EVENT;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_VAL_EVENT\r\n");
#endif

  server_cb = conn->instance->server_cb;
  if (server_cb && server_cb->long_char_val_prepare_writein_demand)
    /* characteristic_value_offset */
    server_cb->long_char_val_prepare_writein_demand(
      conn->instance->epr, op_cb->offset, conn->u.attr.eidx);

  return TWIC_STATUS_OK;
}
#endif
#if defined(TWIC_API_GATTSERVEREXECCHARVALWRITEINRESPONSE)
/* need twicIfGattServerExecCharValWriteInResponse op_8f */
static TZ1K_INLINE twicStatus_t
twicSsGattSerExecuteWriteCharValEventCm(const twicRespCyCm_t * const op_cf)
{
  twicAttValue_t *arg;
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;

  /* char_value_handle */
  conn = twicHashFind2ndHandle(op_cf->handle);
  if (!(conn)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  if (conn->instance->token) return TWIC_STATUS_UNDER_PROCESSING;
  conn->instance->token = TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_VAL_EVENT;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_VAL_EVENT\r\n");
#endif

  server_cb = conn->instance->server_cb;
  if (server_cb && server_cb->char_val_exec_writein_demand) {
    arg = (twicAttValue_t *)&conn->instance->resp.value;
    arg->parameter_length = op_cf->parameter_length - 4; /* [1 , 100] */
    memcpy(arg->value, op_cf->value, op_cf->parameter_length - 4);
    server_cb->char_val_exec_writein_demand(
      conn->instance->epr, arg, conn->u.attr.eidx);
  }

  return TWIC_STATUS_OK;
}
#endif
#if defined(TWIC_API_GATTSERVEREXECCHARDESPWRITEINRESPONSE)
/* need twicIfGattServerExecCharDespWriteInResponse op_8f */
static TZ1K_INLINE twicStatus_t
twicSsGattSerExecuteWriteCharDespEventCm(const twicRespCyCm_t * const op_cf)
{
  twicAttValue_t *arg;
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;

  /* char_value_handle */
  conn = twicHashFind2ndHandle(op_cf->handle);
  if (!(conn)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  if (conn->instance->token) return TWIC_STATUS_UNDER_PROCESSING;
  conn->instance->token = TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_DESP_EVENT;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_DESP_EVENT\r\n");
#endif

  server_cb = conn->instance->server_cb;
  if (server_cb && server_cb->char_desp_exec_writein_demand) {
    arg = (twicAttValue_t *)&conn->instance->resp.value;
    arg->parameter_length = op_cf->parameter_length - 4; /* [1 , 100] */
    memcpy(arg->value, op_cf->value, op_cf->parameter_length - 4);
    server_cb->char_desp_exec_writein_demand(
      conn->instance->epr, arg, conn->u.attr.eidx);
  }

  return TWIC_STATUS_OK;
}
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARDESPPREPAREWRITEINRESPONSE)
/* need twicIfGattServerLongCharDespPrepareWriteInResponse op_8c */
static TZ1K_INLINE twicStatus_t
twicSsGattSerPrepWriteLongRelCharDespEventCm(const twicRespClCm_t * const op_cc)
{
  const twicLeServerCb_t *server_cb;
  twicEntry_t *conn;

  /* characteristic_descriptor_handle */
  conn = twicHashFind2ndHandle(op_cc->handle);
  if (!(conn)) {
    twicLog("ERROR.\r\n");
    return TWIC_STATUS_ERROR_DATA;
  }

  if (conn->instance->token) return TWIC_STATUS_UNDER_PROCESSING;
  conn->instance->token =
    TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_DESP_EVENT;
#if defined(TWIC_SS_DOZE_DEBUG)
  twicPrintf("TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_DESP_EVENT\r\n");
#endif

  server_cb = conn->instance->server_cb;
  if (server_cb && server_cb->long_char_desp_prepare_writein_demand)
    /* characteristic_value_offset */
    server_cb->long_char_desp_prepare_writein_demand(
      conn->instance->epr, op_cc->offset, conn->u.attr.eidx);

  return TWIC_STATUS_OK;
}
#endif

static TZ1K_INLINE twicStatus_t
twicSsConnectionManagementCm(const twicRespCm_t * const resp_cm)
{
  twicEntry_t const * entry;

  if (0xf0 <= resp_cm->op_code) {
    switch (resp_cm->op_code) {
    case TWIC_TCU_LE_ACCEPT:
      twicSsLeAcceptCm(&resp_cm->u.op_d1f1); break;
    case TWIC_TCU_MNG_LE_NOT_ACCEPT:
      twicSsLeNotAcceptCm(&resp_cm->u.op_d1f2); break;
    case TWIC_TCU_MNG_LE_FATAL_ERROR:
      twicSsLeFatalErrorCm(&resp_cm->u.op_d1fe); break;
    default: break;
    }
  } else if (0xc0 <= resp_cm->op_code) {
    switch (resp_cm->op_code) {
#if defined(TWIC_CONFIG_ENABLE_SCAN)
    case TWIC_TCU_MNG_LE_ADV_REPORT_EVENT:
      twicSsLeAdvReportEventCm(&resp_cm->u.op_d1c1); break;
#endif
    default: break;
    }
  } else if (0x90 <= resp_cm->op_code) {
    switch (resp_cm->op_code) {
    case TWIC_TCU_MNG_LE_DISCONNECT_EVENT:
      twicSsLeDisconnEventCm(&resp_cm->u.op_d193); break;
    case TWIC_TCU_MNG_LE_READ_TX_POW_LEVEL_RESP:
      twicSsLeReadTxPowLevelRespCm(&resp_cm->u.op_d194); break;
    case TWIC_TCU_MNG_LE_READ_RSSI_RESP:
      twicSsLeReadRssiRespCm(&resp_cm->u.op_d195); break;
#if defined(TWIC_BLE_HWIP_V41)
    case TWIC_TCU_MNG_LE_REM_CON_PARAM_ACCEPT_RESP:
      twicSsLeCmSimpleRespCm(0x8F, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT,
                             resp_cm->op_code, resp_cm->u.op_ax.handle,
                             resp_cm->u.op_ax.status); break;
#endif
#if defined(TWIC_API_LEREADCHANNELMAP)
    case TWIC_TCU_MNG_LE_READ_CHANNEL_MAP_RESP:
      twicSsLeReadChannelMapRespCm(&resp_cm->u.op_d191); break;
#endif
#if defined(TWIC_API_LEREADSUPPORTEDSTATES)
    case TWIC_TCU_MNG_LE_READ_SUPPORTED_STATES_RESP:
      twicSsLeReadSupportedStatesRespCm(&resp_cm->u.op_d192); break;
#endif
#if defined(TWIC_API_LESETHOSTCHANNELCLASSIFICATION)
    case TWIC_TCU_MNG_LE_SET_HOST_CHANNEL_CLASSIFICATION_RESP:
      twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd110); break;
#endif
    default: break;
    }
  } else if (0x80 <= resp_cm->op_code) {
    switch (resp_cm->op_code) {
    case TWIC_TCU_MNG_LE_INIT_RESP:
      twicSsLeInitRespCm(&resp_cm->u.op_d181); break;
    case TWIC_TCU_MNG_LE_START_ADVERTISE_RESP:
      entry = twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd108);
      if (NULL != entry && 0 == resp_cm->u.op_simple.status) {
        entry->instance->opmd.adve = true;
      } else {
        entry->instance->opmd.adve = entry->instance->opmd.ncad = false;
      }
      break;
    case TWIC_TCU_MNG_LE_DISABLE_ADVERTISE_RESP:
      entry = twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd109);
      if (NULL != entry && 0 == resp_cm->u.op_simple.status) {
        entry->instance->opmd.adve = entry->instance->opmd.ncad = false;
      }
      break;
#if defined(TWIC_API_LESETRANDOMADDRESS)
    case TWIC_TCU_MNG_LE_SET_RAND_ADDRESS_RESP:
      twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd104); break;
#endif
#if defined(TWIC_API_LEADDWHITELIST)
    case TWIC_TCU_MNG_LE_ADD_WHITELIST_RESP:
      twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd106); break;
#endif
#if defined(TWIC_API_LEDELWHITELIST) || defined(TWIC_API_LECLEAREWHITELIST)
    case TWIC_TCU_MNG_LE_DEL_WHITELIST_RESP:
      twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd107); break;
#endif
#if defined(TWIC_CONFIG_ENABLE_SCAN)
    case TWIC_TCU_MNG_LE_SET_SCAN_ENABLE_RESP:
      entry = twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd10a);
      if (NULL != entry && 0 == resp_cm->u.op_simple.status) {
        entry->instance->opmd.scan = true;
      }
      break;
    case TWIC_TCU_MNG_LE_SET_SCAN_DISABLE_RESP:
      entry = twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd10b);
      if (NULL != entry && 0 == resp_cm->u.op_simple.status) {
        entry->instance->opmd.scan = false;
      }
      break;
    case TWIC_TCU_MNG_LE_CREATE_CONNECTION_CANCEL_RESP:
      entry = twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd10d);
      if (NULL != entry && 0 == resp_cm->u.op_simple.status) {
        entry->instance->role &= ~TWIC_IFACE_GAP_CREATE_CONNECTION;
      }
      break;
#endif /* TWIC_CONFIG_ENABLE_SCAN */
#if defined(TWIC_API_LEREADWHITELISTSIZE)
    case TWIC_TCU_MNG_LE_READ_WHITELIST_SIZE_RESP:
      twicSsLeReadWhitelistSizeRespCm(&resp_cm->u.op_d185); break;
#endif
#if defined(TWIC_API_LEREADLOCALSUPPORTEDFEATURES)
    case TWIC_TCU_MNG_LE_READ_LOCAL_SUPPORTED_FEATURES_RESP:
      twicSsLeReadLocalSupportedFeaturesRespCm(&resp_cm->u.op_d182); break;
#endif
    default: break;
    }
  } else if (0x50 <= resp_cm->op_code) {
    switch (resp_cm->op_code) {
    case TWIC_TCU_MNG_LE_L2CAP_CONNECTION_UPDATE_RESP_EVENT:
      twicSsLeCeL2capConnectionUpdateRespEventCm(&resp_cm->u.op_d150); break;
    case TWIC_TCU_MNG_LE_CON_UPDATE_ACCEPT_RESP: /* d151 */
      twicSsLeCeL2capUpdateConnReqAcceptRespCm(&resp_cm->u.op_d151);
    case TWIC_TCU_MNG_LE_UPDATE_CONN_REQ_EVENT: /* d152 */
      return twicSsLeCeL2capUpdateConnReqEventCm(&resp_cm->u.op_d152);
    case TWIC_TCU_MNG_LE_L2CAP_CMD_REJECT_EVENT: /* d153 */
      twicSsLeCeRejectL2capConnectionUpdateRespEventCm(&resp_cm->u.op_8x);
      break;
#if defined(TWIC_API_LELMGENRESOLVABLEBDADDR)
    case TWIC_TCU_MNG_LE_GEN_RESOLVABLE_BDADDR_RESP:
      twicSsLeGenResolvableBdaddrRespCm(&resp_cm->u.op_d154); break;
#endif
#if defined(TWIC_API_LELMRESOLVEBDADDR)
    case TWIC_TCU_MNG_LE_RESOLVE_BDADDR_RESP:
      twicSsLeResolveBdaddrRespCm(&resp_cm->u.op_d155); break;
#endif
    case TWIC_TCU_MNG_LE_READ_REMOTE_VERSION_RESP:
      twicSsLeReadRemoteVersionRespCm(&resp_cm->u.op_d159); break;
#if defined(TWIC_API_LELMRESOLVEBDADDR)
    case TWIC_TCU_MNG_LE_SET_IR_VALUE_RESP: /* d15b */
      twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd11b); break;
#endif
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
    case TWIC_TCU_MNG_LE_ENCRYPT_RESP:
      twicSsLeEncryptRespCm(&resp_cm->u.op_d15c); break;
#endif
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LELESETADVERTISINGDATA)
    case TCU_MNG_LE_SET_ADVERTISE_DATA_RESP: /* 0x5d */
      twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd11d); break;
#endif
#if defined(TWIC_BLE_HWIP_V41)
    case TWIC_TCU_MNG_LE_REM_CON_PARAM_REQ_EVENT:
      return twicSsLeConnParameterEventCm(&resp_cm->u.op_d15e);
#endif
    default: break;
    }
  } else {
    switch (resp_cm->op_code) {
    case TWIC_TCU_MNG_LE_CREATE_CONNECTION_COMPLETE_EVENT:
      twicSsLeConnCompleteEventCm(&resp_cm->u.op_d14c); break;
    case TWIC_TCU_MNG_LE_CONNECTION_UPDATE_EVENT:
      twicSsLeConnUpdateEventCm(&resp_cm->u.op_d14e); break;
    case TWIC_TCU_MNG_LE_READ_REMOTE_USED_FEATURES_EVENT:
      twicSsLeReadRemoteUsedFeaturesEventCm(&resp_cm->u.op_d14f); break;
    default: break;
    }
  }
  return TWIC_STATUS_OK;
}

static TZ1K_INLINE twicStatus_t
twicSsClientManagementCm(const twicRespCm_t * const resp_cm)
{
  if (0x90 <= resp_cm->op_code) {
    switch (resp_cm->op_code) {
#if defined(TWIC_API_GATTCLIENTINDICATIONCONFIRMATIONRESPONSE)
    case TWIC_TCU_LE_GATT_CLI_CHAR_VAL_CONFIRMATION_ACCEPT_RESP:
      twicSsGattCliCharValConfirmationAcceptRespCm(&resp_cm->u.op_d290);
      break;
#endif
    case TWIC_TCU_LE_GATT_CLI_CHAR_VAL_NOTIFICATION_IND_EVENT:
      twicSsGattCliCharValNotificationIndEventCm(&resp_cm->u.op_d2c0); break;
    case TWIC_TCU_LE_GATT_CLI_CHAR_VAL_INDICATION_IND_EVENT:
      twicSsGattCliCharValIndicationIndEventCm(&resp_cm->u.op_d2d0); break;
    default: twicLog("0x%02x\r\n", resp_cm->op_code); break;
    }
  } else if (0x80 <= resp_cm->op_code) {
    switch (resp_cm->op_code) {
    case TWIC_TCU_LE_GATT_CLI_INIT_RESP:
      twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd200); break;
    default: twicLog("0x%02x\r\n", resp_cm->op_code); break;
    }
  } else if (0x50 <= resp_cm->op_code) {
    switch (resp_cm->op_code) {
#if defined(TWIC_API_GATTCLIENTSIGNEDWRITEWITHOUTRESPONSE)
    case TWIC_TCU_LE_GATT_CLI_SIGNED_WRITE_WITHOUT_RESPONSE_CMD_EVENT:
      twicSsGattCliSignedWriteWithoutResponseCmdEventCm(
        &resp_cm->u.op_d251); break;
#endif
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE)
    case TWIC_TCU_LE_GATT_CLI_READ_LONG_CHAR_VAL_EVENT:
      twicSsGattCliReadLongCharValEventCm(&resp_cm->u.op_d252); break;
#endif
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR)
    case TWIC_TCU_LE_GATT_CLI_READ_LONG_CHAR_DESP_EVENT:
      twicSsGattCliReadLongCharDespEventCm(&resp_cm->u.op_d253); break;
#endif
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICVALUE)
    case TWIC_TCU_LE_GATT_CLI_WRITE_LONG_CHAR_VAL_EVENT:
      twicSsGattCliWriteLongCharValEventCm(&resp_cm->u.op_d254); break;
#endif
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR)
    case TWIC_TCU_LE_GATT_CLI_WRITE_LONG_CHAR_DESP_EVENT:
      twicSsGattCliWriteLongCharDespEventCm(&resp_cm->u.op_d255); break;
#endif
    case TWIC_TCU_LE_GATT_CLI_INVALID_RESP:
      twicSsGattCliInvalidRespCm(&resp_cm->u.op_d256); break;
    default: twicLog("0x%02x\r\n", resp_cm->op_code); break;
    }
  } else if (0x40 <= resp_cm->op_code) {
    switch (resp_cm->op_code) {
#if defined(TWIC_API_GATTCLIENTEXGMTU)
    case TWIC_TCU_LE_GATT_CLI_EXG_MTU_EVENT:
      return twicSsGattCliExgMtuEventCm(&resp_cm->u.op_d241);
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICE)
    case TWIC_TCU_LE_GATT_CLI_DISCOVER_PRIM_SVC_EVENT:
      twicSsGattCliDiscoverPrimSvcEventCm(&resp_cm->u.op_d242); break;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID)
    case TWIC_TCU_LE_GATT_CLI_DISCOVER_PRIM_SVC_BY_UUID_EVENT:
      twicSsGattCliDiscoverPrimSvcByUuidEventCm(
        (twicRespD243Cm_t *)&resp_cm->u.op_d243); break;
#endif
#if defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE)
    case TWIC_TCU_LE_GATT_CLI_FIND_INCL_SVC_EVENT:
      twicSsGattCliFindInclSvcEventCm(
        (twicRespD244Cm_t *)&resp_cm->u.op_d244); break;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS)
    case TWIC_TCU_LE_GATT_CLI_DISCOVER_CHAR_DECL_EVENT:
      twicSsGattCliDiscoverCharDeclEventCm(
        (twicRespD245Cm_t *)&resp_cm->u.op_d245); break;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)
    case TWIC_TCU_LE_GATT_CLI_DISCOVER_CHAR_DECL_BY_UUID_EVENT:
      twicSsGattCliDiscoverCharDeclByUuidEventCm(&resp_cm->u.op_d246); break;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS)
    case TWIC_TCU_LE_GATT_CLI_DISCOVER_CHAR_DESP_EVENT:
      twicSsGattCliDiscoverCharDspEventCm(
        (twicRespD247Cm_t *)&resp_cm->u.op_d247); break;
#endif
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE)
    case TWIC_TCU_LE_GATT_CLI_READ_CHAR_VAL_EVENT:
      twicSsGattCliReadCharValEventCm(&resp_cm->u.op_d248); break;
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICVALUE)
    case TWIC_TCU_LE_GATT_CLI_WRITE_CHAR_VAL_EVENT:
      twicSsGattCliWriteCharValEventCm(&resp_cm->u.op_d249); break;
#endif
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR)
    case TWIC_TCU_LE_GATT_CLI_READ_CHAR_DESP_EVENT:
      twicSsGattCliReadCharDespEventCm(&resp_cm->u.op_d24a); break;
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR)
    case TWIC_TCU_LE_GATT_CLI_WRITE_CHAR_DESP_EVENT:
      twicSsGattCliWriteCharDespEventCm(&resp_cm->u.op_d24b); break;
#endif
#if defined(TWIC_API_GATTCLIENTREADMULTIPLECHARVALUES)
    case TWIC_TCU_LE_GATT_CLI_READ_MULTIPLE_CHAR_VALUES_EVENT:
      twicSsGattCliReadMultipleCharValuesEventCm(&resp_cm->u.op_d24c); break;
#endif
#if defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID)
    case TWIC_TCU_LE_GATT_CLI_READ_CHAR_VAL_UUID_EVENT:
      twicSsGattCliReadCharValUuidEventCm(&resp_cm->u.op_d24d); break;
#endif
#if defined(TWIC_API_GATTCLIENTRELIABLEWRITE)
    case TWIC_TCU_LE_GATT_CLI_RELIABLE_WRITES_EVENT:
      twicSsGattCliReliableWritesEventCm(&resp_cm->u.op_d24e); break;
#endif
#if defined(TWIC_API_GATTCLIENTWRITEWITHOUTRESPONSE)
    case TWIC_TCU_LE_GATT_CLI_WRITE_WITHOUT_RESPONSE_CMD_EVENT:
      twicSsGattCliWriteWithoutResponseCmdEventCm(&resp_cm->u.op_d24f); break;
#endif
    default: twicLog("0x%02x\r\n", resp_cm->op_code); break;
    }
  } else {
    twicLog("0x%02x\r\n", resp_cm->op_code);
  }

  return TWIC_STATUS_OK;
}

static TZ1K_INLINE twicStatus_t
twicSsServerManagementCm(const twicRespCm_t * const resp_cm)
{
  twicEntry_t *entry = NULL;

  if (0xc0 <= resp_cm->op_code) {
    switch (resp_cm->op_code) {
    case TWIC_TCU_LE_GATT_SER_EXG_MTU_EVENT:
      /* need twicIfGattServerExgMtuResponse op_81 */
      return twicSsGattSerExgMtuEventCm(&resp_cm->u.op_c1);
    case TWIC_TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT:
      /* need twicIfGattServerCharValReadOutResponse op_82 */
      return twicSsGattSerReadCharValEventCm(&resp_cm->u.op_cx);
    case TWIC_TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT:
      /* need twicIfGattServerCharValWriteInResponse op_83 */
      return twicSsGattSerWriteCharValEventCm(&resp_cm->u.op_cy);
    case TWIC_TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT:
      /* need twicIfGattServerCharDespWriteInResponse op_84 */
      return twicSsGattSerWriteCharDespEventCm(&resp_cm->u.op_cy);
    case TWIC_TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT:
      /* need twicIfGattServerCharDespReadOutResponse op_88 */
      return twicSsGattSerReadCharDespEventCm(&resp_cm->u.op_cx);
    case TWIC_TCU_LE_GATT_SER_WRITE_WITHOUT_RESPONSE_EVENT:
      return twicSsGattSerWriteCharWithoutResponseEventCm(&resp_cm->u.op_cy);
    case TWIC_TCU_LE_GATT_SER_READ_MULTIPLE_EVENT:
      /* need twicIfGattServerCharValMultiReadOutResponse op_8a */
      return twicSsGattSerReadMultipleEventCm(&resp_cm->u.op_ca);
#if defined(TWIC_API_GATTSERVERLONGCHARVALREADOUTRESPONSE)
    case TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_VAL_EVENT:
      /* need twicIfGattServerLongCharValReadOutResponse op_8d */
      return twicSsGattSerReadLongCharValEventCm(&resp_cm->u.op_cl);
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARDESPREADOUTRESPONSE)
    case TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_DESP_EVENT:
      /* need twicIfGattServerLongCharDespReadOutResponse op_8e */
      return twicSsGattSerReadLongCharDespEventCm(&resp_cm->u.op_cl);
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARVALPREPAREWRITEINRESPONSE)
    case TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_VAL_EVENT:
      /* need twicIfGattServerLongCharValPrepareWriteInResponse op_8b */
      return twicSsGattSerPrepWriteLongRelCharValEventCm(&resp_cm->u.op_cl);
#endif
#if defined(TWIC_API_GATTSERVEREXECCHARVALWRITEINRESPONSE)
    case TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_VAL_EVENT:
      /* need twicIfGattServerExecCharValWriteInResponse op_8f */
      return twicSsGattSerExecuteWriteCharValEventCm(&resp_cm->u.op_cy);
#endif
#if defined(TWIC_API_GATTSERVEREXECCHARDESPWRITEINRESPONSE)
    case TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_DESP_EVENT:
      /* need twicIfGattServerExecCharDespWriteInResponse op_8f */
      return twicSsGattSerExecuteWriteCharDespEventCm(&resp_cm->u.op_cy);
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARDESPPREPAREWRITEINRESPONSE)
    case TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_DESP_EVENT:
      /* need twicIfGattServerLongCharDespPrepareWriteInResponse op_8c */
      return twicSsGattSerPrepWriteLongRelCharDespEventCm(&resp_cm->u.op_cl);
#endif
    default: break;
    }
  } else if (0xa0 <= resp_cm->op_code) {
    switch (resp_cm->op_code) {
    case TWIC_TCU_LE_GATT_SDB_ADD_PRIM_SVC_RESP:
      twicSsGattSdbAddPrimSvcRespCm(&resp_cm->u.op_ax); break;
    case TWIC_TCU_LE_GATT_SDB_ADD_SEC_SVC_RESP:
      twicSsGattSdbAddSecSvcRespCm(&resp_cm->u.op_ax); break;
    case TWIC_TCU_LE_GATT_SDB_ADD_CHAR_DECL_RESP:
      twicSsGattSdbAddCharDeclRespCm(&resp_cm->u.op_ax); break;
    case TWIC_TCU_LE_GATT_SDB_ADD_CHAR_ELE_RESP:
      twicSsGattSdbAddCharEleRespCm(&resp_cm->u.op_ax); break;
    case TWIC_TCU_LE_GATT_SDB_ADD_INC_SVC_RESP:
      twicSsGattSdbAddIncSvcRespCm(&resp_cm->u.op_ax); break;
    case TWIC_TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP:
      twicSsGattSdbUpdCharEleRespCm(&resp_cm->u.op_a5); break;
    case TWIC_TCU_LE_GATT_SDB_RET_END_GRP_HLE_RESP:
      twicSsGattSdbRetEndGrpHleRespCm(&resp_cm->u.op_ax); break;
#if defined(TWIC_API_LEGATTDBSETPERMISSIONS)
    case TWIC_TCU_LE_GATT_SDB_SET_ATTR_PERMS_RESP:
      twicSsGattSdbSetAttrPermsRespCm(&resp_cm->u.op_a9); break;
#endif
#if defined(TWIC_API_LEGATTDBGETPERMISSIONS)
    case TWIC_TCU_LE_GATT_SDB_GET_ATTR_PERMS_RESP:
      twicSsGattSdbGetAttrPermsRespCm(&resp_cm->u.op_aa); break;
#endif
#if defined(TWIC_API_LEGATTDBSETCHARACTERISTICSVL) || \
  defined(TWIC_API_LEGATTDBUPDCHARACTERISTICSVL) ||   \
  defined(TWIC_API_LEGATTDBSETDESCRIPTORVL)
    case TWIC_TCU_LE_GATT_SDB_ADD_CHAR_VAR_ELE_RESP:
      twicSsGattSdbAddCharVarEleRespCm(&resp_cm->u.op_ax); break;
#endif
    default: break;
    }
  } else if (0x80 <= resp_cm->op_code) {
    switch (resp_cm->op_code) {
    case TWIC_TCU_LE_GATT_SER_INIT_RESP:
      twicSsMngSimpleRespCm(resp_cm->u.op_simple.status, 0xd300); break;
    case TWIC_TCU_LE_GATT_SER_EXG_MTU_ACCEPT_RESP: /* 0x81 */
      entry = twicSsGattSerExgMtuAcceptRespCm(&resp_cm->u.op_81);
      /* TWIC_TCU_LE_GATT_SER_EXG_MTU_EVENT */
      break;
    case TWIC_TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_RESP: /* 0x82 */
      entry = twicSsLeResp8xCm(&resp_cm->u.op_8x, 0xd302);
      /* TWIC_TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT */
      break;
    case TWIC_TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_RESP:
      entry = twicSsLeResp8xCm(&resp_cm->u.op_8x, 0xd303);
      /* TWIC_TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT */
      break;
    case TWIC_TCU_LE_GATT_SER_WRITE_CHAR_DESP_ACCEPT_RESP:
      entry = twicSsLeResp8xCm(&resp_cm->u.op_8x, 0xd304);
      /* TWIC_TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT */
      break;
    case TWIC_TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_RESP:
      entry = twicSsLeResp8xCm(&resp_cm->u.op_8x, 0xd308);
      /* TWIC_TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT */
      break;
    case TWIC_TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP:
      entry = twicSsLeResp8xCm(&resp_cm->u.op_8x, 0xd30a);
      /* TWIC_TCU_LE_GATT_SER_READ_MULTIPLE_EVENT */
      break;
    case TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_VAL_ACCEPT_RESP:
      entry = twicSsLeResp8xCm(&resp_cm->u.op_8x, 0xd30d);
      /* TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_VAL_EVENT */
      break;
    case TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_DESP_ACCEPT_RESP:
      entry = twicSsLeResp8xCm(&resp_cm->u.op_8x, 0xd30e);
      /* TWIC_TCU_LE_GATT_SER_READ_LONG_CHAR_DESP_EVENT */
      break;
    case TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_VAL_ACCEPT_RESP:
      entry = twicSsLeResp8xCm(&resp_cm->u.op_8x, 0xd30b);
      /* TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_VAL_EVENT */
      break;
    case TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_DESP_ACCEPT_RESP:
      entry = twicSsLeResp8xCm(&resp_cm->u.op_8x, 0xd30c);
      /* TWIC_TCU_LE_GATT_SER_PREP_WRITE_LONG_REL_CHAR_DESP_EVENT */
      break;
    case TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_VAL_ACCEPT_RESP:
      entry = twicSsLeResp8xCm(&resp_cm->u.op_8x, 0xd30f);
      /* TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_VAL_EVENT */
      break;
    case TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_DESP_ACCEPT_RESP:
      entry = twicSsLeResp8xCm(&resp_cm->u.op_8x, 0xd307);
      /* TWIC_TCU_LE_GATT_SER_EXECUTE_WRITE_CHAR_DESP_EVENT */
      break;
    default: break;
    }
    if ((entry) && (resp_cm->op_code | 0x40) == entry->instance->token) {
      entry->instance->token = 0;
#if defined(TWIC_SS_DOZE_DEBUG)
      twicPrintf("Unlock 0x%x\r\n", resp_cm->op_code | 0x40);
#endif
    }
  } else {
    switch (resp_cm->op_code) {
    case TWIC_TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_EVENT:
      twicSsGattSerNotificationEventCm(&resp_cm->u.op_45); break;
    case TWIC_TCU_LE_GATT_SER_CHAR_VAL_INDICATION_EVENT:
      twicSsGattSerIndicationEventCm(&resp_cm->u.op_46); break;
    case TWIC_TCU_LE_GATT_SER_EXECUTE_COMPLETE_EVENT:
      twicSsGattSerExecuteCompleteEvent(&resp_cm->u.op_47); break;
    default: break;
    }
  }

  return TWIC_STATUS_OK;
}

#if defined(TWIC_CONFIG_SM_INITIATOR)
static TZ1K_INLINE void
twicSsLeSmpInitiatorCm(const twicRespCm_t * const resp_cm)
{
  twicEntry_t *entry = NULL;

  switch (resp_cm->op_code) {
  case TWIC_TCU_LE_SMP_I_PAIRING_EVENT:
    twicSsLeSmMasPairingResponseEventCm(&resp_cm->u.op_d441); break;
  case TWIC_TCU_LE_SMP_I_SECURITY_EVENT:
/* This event is generated by the Master Device to the application
 * when Slave Device requests for pairing.
 *
 * If the Application accepts the Slave Security Request, the Master
 * will initiate the pairing process using the request
 * "twicIfLeSmMasPairingRequest TCU_LE_SMP_MAS_PAIRING_REQ".
 *
 * If the Application rejects the Slave Security Request, the Master
 * will issue "twicIfLeSmMasSecurityReject
 * TCU_LE_SMP_MAS_SECURITY_ACCEPT_REQ". */
    twicSsLeSmMasSecurityEventCm(false, &resp_cm->u.op_d4c2); break;
  case TWIC_TCU_LE_SMP_I_SECURITY_ACCEPT_RESP:
    entry = twicSsLeResp8xCm(&resp_cm->u.op_8x, 0xd402);
    if ((entry)) entry->instance->group.b.smp_i_security_event = false;
#if defined(TWIC_SS_DOZE_DEBUG)
    twicPrintf("Unlock TWIC_TCU_LE_SMP_I_SECURITY_EVENTr\n");
#endif
    break;
  case TWIC_TCU_LE_SMP_I_PAIRING_FAILED_RESP:
    twicSsLeCmSimpleRespCm(0xBF, TWIC_SERVICE_ID_SMP_INITIATOR,
                           resp_cm->op_code, resp_cm->u.op_d451.conn_handle,
                           resp_cm->u.op_d451.reason); break;
  case TWIC_TCU_LE_SMP_I_PAIRING_FAILED_EVENT:
    twicSsLeSmMasPairingFailedEventCm(&resp_cm->u.op_d443); break;
  case TWIC_TCU_LE_SMP_I_STK_GEN_METHOD_EVENT:
    twicSsLeSmMasStkGenMethodEventCm(&resp_cm->u.op_d4cb); break;
  case TWIC_TCU_LE_SMP_I_KEY_ENTRY_REQ_EVENT:
    twicSsLeSmMasPassKeyReqEventCm(&resp_cm->u.op_d444); break;
  case TWIC_TCU_LE_SMP_I_KEY_ENTRY_WRITE_RESP:
    entry = twicSsLeCmSimpleRespCm(0x7F, TWIC_SERVICE_ID_SMP_INITIATOR,
                                   resp_cm->op_code,
                                   resp_cm->u.op_d485.conn_handle,
                                   resp_cm->u.op_d485.reason);
    if ((entry)) entry->instance->group.b.smp_i_key_entry_req_event = false;
#if defined(TWIC_SS_DOZE_DEBUG)
    twicPrintf("Unlock TWIC_TCU_LE_SMP_I_KEY_ENTRY_REQ_EVENT\r\n");
#endif
    break;
  case TWIC_TCU_LE_SMP_I_DISPLAY_KEY_EVENT:
    twicSsLeSmMasDisplayPassKeyEventCm(&resp_cm->u.op_d446); break;
  case TWIC_TCU_LE_SMP_I_DISPLAY_KEY_WRITE_RESP:
    entry = twicSsLeCmSimpleRespCm(0x7F, TWIC_SERVICE_ID_SMP_INITIATOR,
                                   resp_cm->op_code,
                                   resp_cm->u.op_d487.conn_handle,
                                   resp_cm->u.op_d487.reason);
    if ((entry)) entry->instance->group.b.smp_i_display_key_event = false;
#if defined(TWIC_SS_DOZE_DEBUG)
    twicPrintf("Unlock TWIC_TCU_LE_SMP_I_DISPLAY_KEY_EVENT\r\n");
#endif
    break;
  case TWIC_TCU_LE_SMP_I_STK_GENERATED_EVENT:
    twicSsLeSmMasStkGeneratedEventCm(&resp_cm->u.op_d448); break;
  case TWIC_TCU_LE_SMP_I_LTK_RECEIVED_EVENT:
    twicSsLeSmMasLtkReceivedEventCm(&resp_cm->u.op_d4c9); break;
  case TWIC_TCU_LE_SMP_I_EDIV_RAND_RECEIVED_EVENT:
    twicSsLeSmMasEdivRandReceivedEventCm(&resp_cm->u.op_d4ca); break;
  case TWIC_TCU_LE_SMP_I_LTK_SENT_EVENT:
    twicSsLeSmMasLtkSentEventCm(&resp_cm->u.op_d4cc); break;
  case TWIC_TCU_LE_SMP_I_EDIV_RAND_SENT_EVENT:
    twicSsLeSmMasEdivRandSentEventCm(&resp_cm->u.op_d4cd); break;
  case TWIC_TCU_LE_SMP_I_ENCRYPTION_CHANGE_EVENT:
    twicSsLeSmMasEncryptionChangeEventCm(&resp_cm->u.op_d4ce); break;
  case TWIC_TCU_LE_SMP_I_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT:
    twicSsLeSmMasEncryptionKeyRefreshCompleteEventCm(&resp_cm->u.op_d4cf);
    break;
  case TWIC_TCU_LE_SMP_I_PAIRING_COMPLETED_EVENT:
    twicSsLeSmMasPairingCompleteEventCm(&resp_cm->u.op_d4d0); break;
  case TWIC_TCU_LE_SMP_I_IRK_RECEIVED_EVENT:
    twicSsLeSmMasIrkReceivedEventCm(&resp_cm->u.op_d4d4); break;
  case TWIC_TCU_LE_SMP_I_IDENTITY_ADDRESS_RECEIVED_EVENT:
    twicSsLeSmMasIdentityAddressReceivedEventCm(&resp_cm->u.op_d4d5); break;
  case TWIC_TCU_LE_SMP_I_CSRK_RECEIVED_EVENT:
    twicSsLeSmMasCsrkReceivedEventCm(&resp_cm->u.op_d4d6); break;
  case TWIC_TCU_LE_SMP_I_IRK_SENT_EVENT:
    twicSsLeSmMasIrkSentEventCm(&resp_cm->u.op_d4d1); break;
  case TWIC_TCU_LE_SMP_I_IDENTITY_ADDRESS_SENT_EVENT:
    twicSsLeSmMasIdentityAddressSentEventCm(&resp_cm->u.op_d4d2); break;
  case TWIC_TCU_LE_SMP_I_CSRK_SENT_EVENT:
    twicSsLeSmMasCsrkSentEventCm(&resp_cm->u.op_d4d3); break;
  case TWIC_TCU_LE_SMP_I_BONDING_ENABLED_EVENT:
    /* Slave Security Request from a Bonded Slave Device. In response
     * to this event the application must issue
     * twicIfLeSmMasStartEncryption to start encryption directly since
     * bonding is done. */
    twicSsLeSmMasSecurityEventCm(true, &resp_cm->u.op_d4d7); break;
  case TWIC_TCU_LE_SMP_I_OOB_KEY_ENTRY_REQ_EVENT:
    twicSsLeSmMasOobKeyEntryRequestEventCm(&resp_cm->u.op_d458); break;
  case TWIC_TCU_LE_SMP_I_OOB_KEY_ENTRY_WRITE_RESP:
    entry = twicSsLeCmSimpleRespCm(0x7F, TWIC_SERVICE_ID_SMP_INITIATOR,
                                   resp_cm->op_code,
                                   resp_cm->u.op_d499.conn_handle,
                                   resp_cm->u.op_d499.reason);
    if ((entry)) entry->instance->group.b.smp_i_oob_key_entry_req_event = false;
#if defined(TWIC_SS_DOZE_DEBUG)
    twicPrintf("Unlock TWIC_TCU_LE_SMP_I_OOB_KEY_ENTRY_REQ_EVENT\r\n");
#endif
    break;
  case TWIC_TCU_LE_SMP_I_STORE_KEY_EVENT:
    twicSsLeSmMasStoreKeyEventCm(&resp_cm->u.op_d4d8); break;
  case TWIC_TCU_LE_SMP_I_KEY_REQ_EVENT:
    twicSsLeSmMasKeyReqEventCm(&resp_cm->u.op_d4d9); break;
  case TWIC_TCU_LE_SMP_I_KEY_ACCEPT_RESP:
    entry = twicSsLeCmSimpleRespCm(0x7F, TWIC_SERVICE_ID_SMP_INITIATOR,
                                   resp_cm->op_code,
                                   resp_cm->u.op_d49b.conn_handle,
                                   resp_cm->u.op_d49b.reason);
    if ((entry)) entry->instance->group.b.smp_i_key_req_event = false;
#if defined(TWIC_SS_DOZE_DEBUG)
    twicPrintf("Unlock TWIC_TCU_LE_SMP_I_KEY_REQ_EVENT\r\n");
#endif
    break;
  case TWIC_TCU_LE_SMP_I_BONDING_ENABLED_INFO_EVENT:
    twicSsLeSmMasBondingStateEventCm(&resp_cm->u.op_d49a); break;
  default: twicTrace(); break;
  }

  return;
}
#endif

#if defined(TWIC_CONFIG_SM_RESPONDER)
static TZ1K_INLINE void
twicSsLeSmpResponderCm(const twicRespCm_t * const resp_cm)
{
  twicEntry_t *entry = NULL;

  switch (resp_cm->op_code) {
  case TWIC_TCU_LE_SMP_R_PAIRING_EVENT:
    twicSsLeSmSlvPairingRequestEventCm(&resp_cm->u.op_d5c1); break;
  case TWIC_TCU_LE_SMP_R_PAIRING_ACCEPT_RESP_EVENT:
    twicSsLeSmSlvPairingAcceptEventCm(&resp_cm->u.op_8x); break;
  case TWIC_TCU_LE_SMP_R_STK_GEN_METHOD_EVENT:
    twicSsLeSmSlvStkGenMethodEventCm(&resp_cm->u.op_d5cb); break;
  case TWIC_TCU_LE_SMP_R_PAIRING_FAILED_EVENT:
    twicSsLeSmSlvPairingFailedEventCm(&resp_cm->u.op_d543); break;
  case TWIC_TCU_LE_SMP_R_KEY_ENTRY_REQ_EVENT:
    twicSsLeSmSlvPassKeyReqEventCm(&resp_cm->u.op_d544); break;
  case TWIC_TCU_LE_SMP_R_KEY_ENTRY_WRITE_RESP:
    entry = twicSsLeCmSimpleRespCm(0x7F, TWIC_SERVICE_ID_SMP_RESPONDER,
                                   resp_cm->op_code,
                                   resp_cm->u.op_d585.conn_handle,
                                   resp_cm->u.op_d585.reason);
    if ((entry)) entry->instance->group.b.smp_r_key_entry_req_event = false;
#if defined(TWIC_SS_DOZE_DEBUG)
    twicPrintf("Unlock TWIC_TCU_LE_SMP_R_KEY_ENTRY_REQ_EVENT\r\n");
#endif
    break;
  case TWIC_TCU_LE_SMP_R_DISPLAY_KEY_EVENT:
    twicSsLeSmSlvDisplayPassKeyEventCm(&resp_cm->u.op_d546); break;
  case TWIC_TCU_LE_SMP_R_DISPLAY_KEY_WRITE_RESP:
    entry = twicSsLeCmSimpleRespCm(0x7F, TWIC_SERVICE_ID_SMP_RESPONDER,
                                   resp_cm->op_code,
                                   resp_cm->u.op_d587.conn_handle,
                                   resp_cm->u.op_d587.reason);
    if ((entry)) entry->instance->group.b.smp_r_display_key_event = false;
#if defined(TWIC_SS_DOZE_DEBUG)
    twicPrintf("Unlock TWIC_TCU_LE_SMP_R_DISPLAY_KEY_EVENT\r\n");
#endif
    break;
  case TWIC_TCU_LE_SMP_R_STK_GENERATED_EVENT:
    twicSsLeSmSlvStkGeneratedEventCm(&resp_cm->u.op_d548); break;
  case TWIC_TCU_LE_SMP_R_LTK_RECEIVED_EVENT:
    twicSsLeSmSlvLtkReceivedEventCm(&resp_cm->u.op_d5c9); break;
  case TWIC_TCU_LE_SMP_R_EDIV_RAND_RECEIVED_EVENT:
    twicSsLeSmSlvEdivRandReceivedEventCm(&resp_cm->u.op_d5ca); break;
  case TWIC_TCU_LE_SMP_R_LTK_SENT_EVENT:
    twicSsLeSmSlvLtkSentEventCm(&resp_cm->u.op_d5cc); break;
  case TWIC_TCU_LE_SMP_R_EDIV_RAND_SENT_EVENT:
    twicSsLeSmSlvEdivRandSentEventCm(&resp_cm->u.op_d5cd); break;
  case TWIC_TCU_LE_SMP_R_ENCRYPTION_CHANGE_EVENT:
    twicSsLeSmSlvEncryptionChangeEventCm(&resp_cm->u.op_d5d0); break;
  case TWIC_TCU_LE_SMP_R_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT:
    twicSsLeSmSlvEncryptionKeyRefreshCompleteEventCm(&resp_cm->u.op_d5d1);
    break;
  case TWIC_TCU_LE_SMP_R_PAIRING_COMPLETED_EVENT:
    twicSsLeSmSlvPairingCompleteEventCm(&resp_cm->u.op_d5d2); break;
  case TWIC_TCU_LE_SMP_R_STK_ENCRYPT_SESSION_REQ_REPLY_EVENT:
    twicSsLeSmSlvStkEncryptionSessionReqReplyEventCm(&resp_cm->u.op_d5ce);
    break;
  case TWIC_TCU_LE_SMP_R_LTK_ENCRYPT_SESSION_REQ_REPLY_EVENT:
    twicSsLeSmSlvLtkEncryptionSessionReqReplyEventCm(&resp_cm->u.op_d5cf);
    break;
  case TWIC_TCU_LE_SMP_R_IRK_RECEIVED_EVENT:
    twicSsLeSmSlvIrkReceivedEventCm(&resp_cm->u.op_d5d6); break;
  case TWIC_TCU_LE_SMP_R_IDENTITY_ADDRESS_RECEIVED_EVENT:
    twicSsLeSmSlvIdentityAddressReceivedEventCm(&resp_cm->u.op_d5d7); break;
  case TWIC_TCU_LE_SMP_R_CSRK_RECEIVED_EVENT:
    twicSsLeSmSlvCsrkReceivedEventCm(&resp_cm->u.op_d5d8); break;
  case TWIC_TCU_LE_SMP_R_IRK_SENT_EVENT:
    twicSsLeSmSlvIrkSentEventCm(&resp_cm->u.op_d5d3); break;
  case TWIC_TCU_LE_SMP_R_IDENTITY_ADDRESS_SENT_EVENT:
    twicSsLeSmSlvIdentityAddressSentEventCm(&resp_cm->u.op_d5d4); break;
  case TWIC_TCU_LE_SMP_R_CSRK_SENT_EVENT:
    twicSsLeSmSlvCsrkSentEventCm(&resp_cm->u.op_d5d5); break;
  case TWIC_TCU_LE_SMP_R_OOB_KEY_ENTRY_REQ_EVENT:
    twicSsLeSmSlvOobKeyEntryRequestEventCm(&resp_cm->u.op_d559); break;
  case TWIC_TCU_LE_SMP_R_OOB_KEY_ENTRY_WRITE_RESP:
    entry = twicSsLeCmSimpleRespCm(0x7F, TWIC_SERVICE_ID_SMP_RESPONDER,
                                   resp_cm->op_code,
                                   resp_cm->u.op_d59a.conn_handle,
                                   resp_cm->u.op_d59a.reason);
    if ((entry)) entry->instance->group.b.smp_r_oob_key_entry_req_event = false;
#if defined(TWIC_SS_DOZE_DEBUG)
    twicPrintf("Unlock TWIC_TCU_LE_SMP_R_OOB_KEY_ENTRY_REQ_EVENT\r\n");
#endif
    break;
  case TWIC_TCU_LE_SMP_R_STORE_KEY_EVENT:
    twicSsLeSmSlvStoreKeyEventCm(&resp_cm->u.op_d5d9); break;
  case TWIC_TCU_LE_SMP_R_KEY_REQ_EVENT:
    twicSsLeSmSlvKeyReqEventCm(&resp_cm->u.op_d5da); break;
  case TWIC_TCU_LE_SMP_R_KEY_ACCEPT_RESP:
    entry = twicSsLeCmSimpleRespCm(0x7F, TWIC_SERVICE_ID_SMP_RESPONDER,
                                   resp_cm->op_code,
                                   resp_cm->u.op_d59c.conn_handle,
                                   resp_cm->u.op_d59c.reason);
    if ((entry)) entry->instance->group.b.smp_r_key_req_event = false;
#if defined(TWIC_SS_DOZE_DEBUG)
    twicPrintf("Unlock TWIC_TCU_LE_SMP_R_KEY_REQ_EVENT\r\n");
#endif
    break;
  case TWIC_TCU_LE_SMP_R_BONDING_ENABLED_INFO_EVENT:
    twicSsLeSmSlvBondingStateEventCm(&resp_cm->u.op_d59b); break;
  case TWIC_TCU_LE_SMP_R_PAIRING_FAILED_RESP:
    twicSsLeCmSimpleRespCm(0xBF, TWIC_SERVICE_ID_SMP_RESPONDER,
                           resp_cm->op_code, resp_cm->u.op_d553.conn_handle,
                           resp_cm->u.op_d553.reason); break;
  default: twicLog("0x%02x\r\n", resp_cm->op_code); break;
  }

  return;
}
#endif

static TZ1K_INLINE void
twicSsBasicManagementCm(const twicRespCm_t * const resp_cm)
{
  switch (resp_cm->op_code) {
  case TWIC_TCU_SYS_INVALID_COMMAND:
    twicSsSysInvalidCommandCm(&resp_cm->u.op_e1ff); break;
  case TWIC_TCU_SYS_ACCEPT: break;
  case TWIC_TCU_SYS_NOT_ACCEPT: break;
  default: twicTrace(); break;
  }
}

static TZ1K_INLINE void twicSsVendorCm(const twicRespCm_t * const resp_cm)
{
  switch (resp_cm->op_code) {
  case TWIC_TCU_VEN_SET_MODULE_MAINTENANCE_EVENT:
    twicSsVenSetModuleMaintenanceEventCm(&resp_cm->u.op_41); break;
  default: twicTrace(); break;
  }
}

void twicSsIsrLeDownstreamRequest(void)
{
  const twicLeCb_t *le_cb;
  twicInstance_t *p;

  /* Since resources acquisition of each interface is not performed,
     it considers as the code which surely checks that each pointer
     cannot be found 0. */

  /* Search all entries which has registered by the interface index. */
  tz1utListEach(twicInstance_t, p, TZ1UT_LIST(srv_instance_root), sibling) {
    le_cb = p->le_cb;
    if (le_cb && le_cb->isr_downstream_request) le_cb->isr_downstream_request();
  }
}

void twicSsIsrLeHostWakeup(void)
{
  const twicLeCb_t *le_cb;
  twicInstance_t *p;
  bool inactivation = tz1smHalGpioBleHostWakeupStatus();

#if defined(TZ1EM_POWER_PROFILE)
  /* Controller has requested the Activation. */
  if (false == inactivation) tz1emIsrParticipateIn(TZ1EM(BLE_CONN));
#endif

  if (false == twic_eq.sniff.lowpower_enable) return;

  if (false == inactivation) { /* Controller has requested the Activation. */
#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
    if (true == twic_eq.sniff.lowpower_peripheral_h4) {
      tz1smHalUartLowPower(false);
      twic_eq.sniff.lowpower_peripheral_h4 = false;
    }
#endif
    /* Search all entries which has registered by the interface index. */
    tz1utListEach(twicInstance_t, p, TZ1UT_LIST(srv_instance_root), sibling) {
      le_cb = p->le_cb;
      if (le_cb && le_cb->isr_wakeup_request) le_cb->isr_wakeup_request();
    }
    return;
  }
#if defined(TZ1EM_POWER_PROFILE)
    /* Controller was requested the Activation. The duration has ended. */
  if (!(tz1utListIsEmpty(TZ1UT_LIST(srv_req_root)))
      || true == twic_eq.sniff.waiting_for_activation
      || true == tz1smHalUartDataAvailable()) {
    return;
  }

#if defined(TWIC_LECE_CLK32K_SLPXOIN_CLOCK) || defined(TWIC_LECE_CLK32K_SLPXOIN_XTAL)
  tz1smHalGpioBleWakeUp(false);
#endif

#if defined(TWIC_MCU_PERIPHERAL_POWER_PROFILE)
  if (false == twic_eq.sniff.lowpower_peripheral_h4) {
    if (true == twicSsIsrLeFindSniffState(true, false, false)) return;
    if (true == twicSsIsrLeFindSniffState(false, true, false)) return;
  }
  tz1smHalUartLowPower(true);
  twic_eq.sniff.lowpower_peripheral_h4 = true;
  if (true == twic_eq.sniff.lowpower_core) {
    if (true == twicSsIsrLeFindSniffState(false, false, true)) return;
    twicSsIsrLeTz1emIdle();
  }
#endif /* TWIC_MCU_PERIPHERAL_POWER_PROFILE */
#endif /* TZ1EM_POWER_PROFILE */
}

void twicSsIsrLeStatusChanged(void)
{
  return;
}

twicStatus_t twicSsPumpCompleteDataCm(void)
{
  twicStatus_t status;
  twicPipeCont_t in_event;
  twicRespCm_t *resp_cm;
  uint8_t type;
  twicInstance_t *p;

  status = twicSsRcvQueueStatus();
  if (status != TWIC_STATUS_EVENT_MESSAGE) return status;
  status = twicSsDequeRcvEvent(&in_event);
  if (status != TWIC_STATUS_EVENT_MESSAGE) return status;
  if (in_event.event != TWIC_RRQ_IN_PACKET_COMPLETION) {
    /* in_event has Error code. Check UART if it's needed. */
    /* twicTrace(); */
    return TWIC_STATUS_ERROR_DATA;
  }
  status = TWIC_STATUS_OK;

  resp_cm = (twicRespCm_t *)&twic_eq.pipe[in_event.pidx].buf[4];
  /* (length = in_event.bytes - 4) */
  type = twic_eq.pipe[in_event.pidx].buf[3];

  switch (type) {
  case TWIC_SERVICE_ID_CONNECTION_MANAGEMENT:
    /* twicLog("TWIC_SERVICE_ID_CONNECTION_MANAGEMENT\r\n"); */
    if (TWIC_STATUS_UNDER_PROCESSING == twicSsConnectionManagementCm(resp_cm))
      status = twicSsEnqueRcvEvent(&in_event); /* rotate left */
    break;
  case TWIC_SERVICE_ID_GATT_CLIENT:
    /* twicLog("TWIC_SERVICE_ID_GATT_CLIENT\r\n"); */
    if (TWIC_STATUS_UNDER_PROCESSING == twicSsClientManagementCm(resp_cm))
      status = twicSsEnqueRcvEvent(&in_event); /* rotate left */
    break;
  case TWIC_SERVICE_ID_GATT_SERVER:
    /* twicLog("TWIC_SERVICE_ID_GATT_SERVER\r\n"); */
    if (TWIC_STATUS_UNDER_PROCESSING == twicSsServerManagementCm(resp_cm))
      status = twicSsEnqueRcvEvent(&in_event); /* rotate left */
    break;
  case TWIC_SERVICE_ID_VENDOR:
    /* twicLog("TWIC_SERVICE_ID_VENDOR\r\n"); */
    twicSsVendorCm(resp_cm); break;
  case TWIC_SERVICE_ID_BASIC_MANAGEMENT:
    /* twicLog("TWIC_SERVICE_ID_BASIC_MANAGEMENT\r\n"); */
    twicSsBasicManagementCm(resp_cm); break;
#if defined(TWIC_CONFIG_SM_INITIATOR)
  case TWIC_SERVICE_ID_SMP_INITIATOR:
    /* twicLog("TWIC_SERVICE_ID_SMP_INITIATOR\r\n"); */
    twicSsLeSmpInitiatorCm(resp_cm); break;
#endif
#if defined(TWIC_CONFIG_SM_RESPONDER)
  case TWIC_SERVICE_ID_SMP_RESPONDER:
    /* twicLog("TWIC_SERVICE_ID_SMP_SLAVE\r\n"); */
    twicSsLeSmpResponderCm(resp_cm); break;
#endif
  default: /* twicTrace(); */ break;
  }

  if (TWIC_STATUS_ERROR_RESOURCE == status) {
    tz1utListEach(twicInstance_t, p, TZ1UT_LIST(srv_instance_root), sibling)
      twicLog("Queue Full ERROR: Unprocessed event is 0x%x.\r\n", p->token);
  }

  return twicSsRcvQueueStatus();
}

twicStatus_t twicSsPumpCompleteDataHc(void)
{
  twicStatus_t status;
  twicPipeCont_t in_event;
  twicRespHc_t *resp_hc;
  uint8_t type;
  uint16_t ocf_ogf;

  status = twicSsRcvQueueStatus();
  if (TWIC_STATUS_EVENT_MESSAGE != status) return status;
  status = twicSsDequeRcvEvent(&in_event);
  if (TWIC_STATUS_EVENT_MESSAGE != status) return status;
  if (TWIC_RRQ_IN_PACKET_COMPLETION != in_event.event) {
    /* in_event has Error code. Check UART if it's needed. */
    twicTrace();
    return TWIC_STATUS_ERROR_DATA;
  }

  resp_hc = (twicRespHc_t *)&twic_eq.pipe[in_event.pidx].buf[1];
  /* (length = in_event.bytes - 1) */
  type = twic_eq.pipe[in_event.pidx].buf[0];

  switch (type) {
  case TWIC_SRQ_INDICATOR_HC_EVENT:
    ocf_ogf = resp_hc->u_event.value[1] | resp_hc->u_event.value[2] << 8;
    if (TWIC_SRQ_EVENT_OPC_HC_NOP == resp_hc->u_event.event_code &&
        4 == resp_hc->u_event.parameter_lenght && 0x00 == ocf_ogf) {
      twicLog("NOP\r\n");
      twic_eq.sniff.enable = RPQUEUE_ENABLE;
      tz1smHalSuppressHpd(false);
    } else if (TWIC_SRQ_EVENT_OPC_HC_HWERR == resp_hc->u_event.event_code &&
               1 == resp_hc->u_event.parameter_lenght)
      twicSsLeCompleteEventWithHwerrHc(resp_hc);
    else twicSsLeCompleteEventHc(resp_hc, ocf_ogf); break;
  default: twicTrace(); break;
  }

  return twicSsRcvQueueStatus();
}
