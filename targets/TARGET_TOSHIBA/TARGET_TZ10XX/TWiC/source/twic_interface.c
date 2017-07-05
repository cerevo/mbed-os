/**
 * @file twic_interface.c
 * @brief a source file for TZ10xx TWiC for Bluetooth 4.0/4.1 Smart
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


#include "tz1sm_config.h" /* system configuration */
#include "tz1ut_list.h"
#include "tz1sm_hal.h"
#include "twic_service.h"
#include "twic_interface_fid.h"
#include "twic_interface_cb.h"
#if defined(TZ1EM_POWER_PROFILE)
#include "tz1em.h"
#endif

#undef TWIC_IF_TRACE_MUTEX

extern twicRxPipeQueue_t twic_eq;
TZ1SM_HAL_MUTEX_INIT(rq_mutex);
static tz1smHalMutexId_t twic_rq_midx = 0;
TZ1SM_HAL_MUTEX_INIT(cb_mutex);
static tz1smHalMutexId_t twic_cb_midx = 0;
TZ1UT_LIST_EXTERN(srv_req_root);
static unsigned long twic_interface_map[TWIC_INTERFACE_MAP_SIZE];
#if defined(TZ1EM_POWER_PROFILE)
TZ1EM_EXTERN(BLE_NC_ADV);
TZ1EM_EXTERN(BLE_CN_ADV);
TZ1EM_EXTERN(BLE_IDLE);
TZ1EM_EXTERN(BLE_CONN);
#endif

/* This API initializes the HW resources of the IO and the PMU.
 * This API must be invoked first of all.
 * This API must not be invoked again without the
 * "twicIfGattDeregistration","twicIfLeIoFinalize",
 * "tz1smHalTimerDelete" and so on.
 * twicIfGattDeregistration(interface_index_1)
 * twicIfGattDeregistration(interface_index_2)
 * All interfaces must be removed if a plural interface is registered.
 * twicIfLeIoFinalize()
 * tz1smHalTimerDelete(timer_id_1) */
twicStatus_t twicIfLeIoInitialize(void)
{
#if defined(TZ1EM_POWER_PROFILE)
  twicStatus_t status;
  tz1emRequirement_t gp;
  uint8_t i;
  
  if (TWIC_STATUS_OK != (status = twicSsLeIoInitialize())) return status;
  tz1emInitializeEntry(TZ1EM(BLE_NC_ADV));
  tz1emInitializeEntry(TZ1EM(BLE_CN_ADV));
  tz1emInitializeEntry(TZ1EM(BLE_IDLE));
  tz1emInitializeEntry(TZ1EM(BLE_CONN));
  gp.pcd = TZ1EM_PCD_NONE;
  gp.mode = TZ1EM_OP_ACTIVE;
  gp.sunshine_vf = TZ1EM_VF_UN;
  gp.wakeup = NULL;
  gp.permissible_time_lag_us = TZ1EM_PTL_SAFE_ASYNCHRONOUS_IO;
  for (i = 0; i < TZ1EM_NUM_OF_TRIGGER; i++) {
    gp.trigger[i].event = TZ1EM_WE_OFF;
    gp.trigger[i].factor = TZ1EM_WF_UN;
  }
  if (TZ1EM_STATUS_OK != tz1emConfigureEntry(TZ1EM(BLE_IDLE), &gp) &&
      TZ1EM_STATUS_OK != tz1emReconfigureEntry(TZ1EM(BLE_IDLE), &gp)) {
    return TWIC_STATUS_ERROR_TZ1EM;
  }
  if (TZ1EM_STATUS_OK != tz1emParticipateIn(TZ1EM(BLE_IDLE))) {
    return TWIC_STATUS_ERROR_TZ1EM;
  }
  return TWIC_STATUS_OK;
#else
  return twicSsLeIoInitialize();
#endif
}

twicStatus_t twicIfleIoStatus(void)
{
  return twicSsLeIoStatus();
}

#if defined(TZ1EM_POWER_PROFILE)
static void twicIfLeTz1emWithdraw(void)
{
  tz1emWithdraw(TZ1EM(BLE_NC_ADV));
  tz1emWithdraw(TZ1EM(BLE_CN_ADV));
  tz1emWithdraw(TZ1EM(BLE_IDLE));
  tz1emWithdraw(TZ1EM(BLE_CONN));
}
#endif

/* @brief Finalize the HW resource of the IO and the PMU.
 * This API terminates the HW resource of IO and the PMU.
 * This API should not be invoked again without "twicIfLeIoInitialize".
 * Before this API, the "twicIfGattDeregistration" and
 * "tz1smHalTimerDelete" should be invoked.
 *
 * @param const bool stop_osc32K
 * Specify that the OSC(32KHz) can be disabled by this API.
 *
 * The TWiC needs the 32KHz as a slow clock source for the Low Power
 * Consumption. The Clock can be supplied by TZ1000 itself. If some
 * other system such as RTC is using the clock, the parameter
 * "stop_osc32K" should be set with "false". */
twicStatus_t twicIfLeIoFinalize(const bool stop_osc32K)
{
#if defined(TZ1EM_POWER_PROFILE)
  twicIfLeTz1emWithdraw();
#endif
  return twicSsLeIoFinalize(stop_osc32K);
}

static twicStatus_t _twicIfGattInitialize(void)
{
  twicStatus_t ret = TWIC_STATUS_OK;
  
  twic_cb_midx = tz1smHalMutexCreate(TZ1SM_HAL_MUTEX(cb_mutex));
  if (!twic_cb_midx)
    return TWIC_STATUS_ERROR_RESOURCE;
  
  if (tz1smHalMutexWait(twic_cb_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  twicSsHashCreate();
  ret = twicSsGattInitialize();
  
  if (tz1smHalMutexRelease(twic_cb_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

/* @brief
 * This API initializes the GATT (GENERIC ATTRIBUTE PROFILE).
 *
 * This initialization is mandatory if the "twicIf" or "twicUt" is
 * used even though any feature of GATT is not used.
 * This API shall be invoked after "twicIfLeIoInitialize".
 * The basic call sequence of this API is below.
 * twicIfLeIoInitialize();
 * twicUtHrpTimerSetup();
 * tz1smHalTimerCreate();
 * twicButtonInit();
 * twicLedInit();
 * twicButtonEnable();
 * twicIfGattInitialize();
 * twicIfGattRegistration();
 * twicIfGattCleanup();
 * twicIfLeRegisterCallback();
 * Please refer to the example source code which is contained in the
 * DFP Package.
 *
 * @return
 * TWIC_STATUS_ERROR_IO: Check if the "twicIfLeIoInitialize" was invoked.
 * TWIC_STATUS_ERROR_RESOURCE: Check the "tz1sm_hal.[ch]" porting. */
twicStatus_t twicIfGattInitialize(void)
{
  twicStatus_t ret = TWIC_STATUS_OK;
  
  memset(&twic_interface_map, 0,
         sizeof (unsigned long) * TWIC_INTERFACE_MAP_SIZE);
  twic_rq_midx = tz1smHalMutexCreate(TZ1SM_HAL_MUTEX(rq_mutex));
  if (!twic_rq_midx)
    return TWIC_STATUS_ERROR_RESOURCE;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = _twicIfGattInitialize();
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

static twicStatus_t
_twicIfGattCleanup(const uint8_t interface, twicConnIface_t * const conn_iface)
{
  const volatile unsigned long *addr = twic_interface_map;
  twicStatus_t ret = TWIC_STATUS_ERROR_PARAMETER;
  uint8_t nr = TWIC_IIDX_NR(interface);
  twicEntry_t *p, *q = NULL;

  if (TWIC_INTERFACE_MAX < interface || 0 == interface) {
    return TWIC_STATUS_ERROR_PARAMETER;
  }
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  if (1UL & (addr[TWIC_BIT_WORD(nr)] >> (nr & TWIC_BITS_PER_LONG - 1))) {
    tz1utListEach(twicEntry_t, p, TZ1UT_LIST(srv_req_root), o_link) {
      if (p->interface == interface) {
        if (q) tz1utListInit(&q->o_link);
        tz1utListDel(&p->o_link);
        q = p;
      }
    }
    if (q) tz1utListInit(&q->o_link);
    twicSsGattCleanup(interface, conn_iface);
    ret = TWIC_STATUS_OK;
  }
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

/* @brief
 * Clean up and initialize all the entries and these list links of the
 * interface.
 *
 * The second parameter is the resource of the GATT.
 * The resource can be classified by each application.
 * For example, GATT Service A can be concatenated to some plural interfaces.
 * Please refer to the "twic_util_service.[ch]" for information about
 * making use of the "conn_iface".
 * The same interface supports all roles such as Advertiser, Scanner,
 * Central, Peripheral, GATT Client and Server.
 * If an application needs to divide each role into some plural interface,
 * each interface can be registered so that a particular interface
 * has the classified GATT's resource.
 *
 * @param const uint8_t interface
 * The interface which was created by the "twicIfGattRegistration".
 *
 * @param twicConnIface_t * const conn_iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 *
 * @return
 * TWIC_STATUS_OK: Success
 * TWIC_STATUS_ERROR_IO: Check if the "twicIfLeIoInitialize" was invoked.
 * TWIC_STATUS_ERROR_RESOURCE: Check the "tz1sm_hal.[ch]" porting.
 * TWIC_STATUS_ERROR_PARAMETER: Argument Error */
twicStatus_t
twicIfGattCleanup(const uint8_t interface, twicConnIface_t * const conn_iface)
{
  twicStatus_t ret = TWIC_STATUS_ERROR_RESOURCE;
  
  if (tz1smHalMutexWait(twic_cb_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = _twicIfGattCleanup(interface, conn_iface);
  
  if (tz1smHalMutexRelease(twic_cb_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeCeHkTimeout(const bool cleanup, const uint16_t cycle,
                                  const uint8_t specified_fidx)
{
  twicStatus_t ret = TWIC_STATUS_OK;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  ret = twicSsLeCeHkTimeout(cleanup, cycle, specified_fidx);

  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  return ret;
}

twicStatus_t twicIfIsDone(twicConnIface_t * const iface, uint8_t * const fidx)
{
  twicStatus_t ret = TWIC_STATUS_OK;
  
  if (tz1smHalMutexWait(twic_cb_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (0 == iface->conn.rsp || 0xFF == iface->conn.rsp) {
    *fidx = 0;
    if (iface->conn.req) {
      ret = TWIC_STATUS_UNDER_PROCESSING;
    }
  } else if (iface->conn.rsp > TWIC_API1_IDX &&
             iface->conn.rsp < TWIC_API2_IDX) {
    *fidx = iface->conn.rsp;
    ret = TWIC_STATUS_EVENT_MESSAGE;
  } else {
    *fidx = 0;
    ret = TWIC_STATUS_ERROR_DATA;
    twicTrace();
  }
  
  if (tz1smHalMutexRelease(twic_cb_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t twicIfDbIsDone(
  twicConnIface_t * const iface, uint8_t * const fidx, uint8_t * const eidx,
  uint8_t const specified_fidx, uint8_t const specified_eidx)
{
  uint8_t _eidx, rsp = 0, req = 0;
  twicStatus_t ret = TWIC_STATUS_OK;
  
  if (tz1smHalMutexWait(twic_cb_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  *fidx = 0;
  *eidx = 0;
  
  if (0 != specified_fidx && 0 != specified_eidx) {
    if (TWIC_API2_IDX >= specified_fidx || TWIC_API3_IDX <= specified_fidx)
      return TWIC_STATUS_ERROR_PARAMETER;
    if (specified_eidx > iface->size)
      return TWIC_STATUS_ERROR_PARAMETER;

    rsp = TWIC_IF_TO_ENTRY(specified_eidx)->rsp;
    if (rsp == specified_fidx) {
      *fidx = rsp;
      *eidx = specified_eidx;
      ret = TWIC_STATUS_EVENT_MESSAGE;
    } else if ((TWIC_IF_TO_ENTRY(specified_eidx)->req)) {
      ret = TWIC_STATUS_UNDER_PROCESSING;
    }
  } else {
    for (_eidx = 1; _eidx <= iface->size; _eidx++) {
      rsp = TWIC_IF_TO_ENTRY(_eidx)->rsp;
      if (0 != TWIC_IF_TO_ENTRY(_eidx)->req)
        req = TWIC_IF_TO_ENTRY(_eidx)->req;
      if (TWIC_API2_IDX >= rsp || TWIC_API3_IDX <= rsp)
        continue;
      *fidx = rsp;
      *eidx = _eidx;
      ret = TWIC_STATUS_EVENT_MESSAGE;
      goto out;
    }
    if ((req)) ret = TWIC_STATUS_UNDER_PROCESSING;
  }

 out:
  
  if (tz1smHalMutexRelease(twic_cb_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

static twicStatus_t twicIfKnock(twicEntry_t * const conn)
{
  twicEntry_t *signal = conn->instance->signal;
  
  if ((conn->req) || (signal->req) || RPQUEUE_ENABLE != twic_eq.sniff.enable)
    return TWIC_STATUS_UNDER_PROCESSING;
  if (!(conn->instance)) return TWIC_STATUS_ERROR_PARAMETER;
  return TWIC_STATUS_OK;
}

twicStatus_t _twicIfLeCePatch(twicEntry_t * const conn, const uint8_t fid,
                              const uint8_t length, const uint8_t * const code,
                              const uint8_t reg, const bool enable)
{
  twicStatus_t ret;

  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  switch (fid) {
  case TWIC_LECEPATCHBASE:
    ret = twicSsLeCePatch(fid, conn, TWIC_ENTRY_MNG_PSB, length, code, 0, 0);
    break;
  case TWIC_LECEPATCHWRITE:
    ret = twicSsLeCePatch(fid, conn, TWIC_ENTRY_MNG_PSPW, length, code, 0, 0);
    break;
  case TWIC_LECEPATCHCONTROL:
    ret = twicSsLeCePatch(fid, conn, TWIC_ENTRY_MNG_PC, 0, NULL, reg, enable);
    break;
  default: break;
  }
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

#if defined(TWIC_API_LECEDBUSREAD)
twicStatus_t _twicIfLeCeDbusRead(twicEntry_t * const conn, const uint8_t addr)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  if (!(ret = twicIfKnock(conn))) goto out;
  
  ret = twicSsLeCeDbusRead(TWIC_LECEDBUSREAD, conn, addr);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_LECEREADFWVER)
twicStatus_t _twicIfLeCeReadFwVer(twicEntry_t * const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  if (!(ret = twicIfKnock(conn))) goto out;
  
  ret = twicSsLeCeReadFwVer(TWIC_LECEREADFWVER, conn);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_LECEDBUSWRITE)
twicStatus_t _twicIfLeCeDbusWrite(twicEntry_t * const conn, const uint8_t addr,
                                  const uint16_t data)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  if (!(ret = twicIfKnock(conn))) goto out;
  
  ret = twicSsLeCeDbusWrite(TWIC_LECEDBUSWRITE, conn, addr, data);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_LECEMEMORYREAD)
twicStatus_t
_twicIfLeCeMemoryRead(twicEntry_t * const conn, const uint32_t addr)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeCeMemoryRead(TWIC_LECEMEMORYREAD, conn, addr);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_LECEMEMORYWRITE)
twicStatus_t _twicIfLeCeMemoryWrite(twicEntry_t * const conn,
                                    const uint32_t addr, const uint16_t data)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeCeMemoryWrite(TWIC_LECEMEMORYWRITE, conn, addr, data);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_LECEXOSCTRIMING)
twicStatus_t
_twicIfLeCeXoscTriming(twicEntry_t * const conn, const uint8_t data)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeCeXoscTriming(TWIC_LECEXOSCTRIMING, conn, data);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_LECESETTXPOWER)
/* 0x00: Reserved,   0x01: 0dBm,   0x02: -4dBm,   0x03: -8dBm,
   0x04: -12dBm,   0x05: -16dBm,   0x06: -20dBm */
twicStatus_t
_twicIfLeCeSetTxPower(twicEntry_t * const conn, const uint8_t dbm_idx)
{
  twicStatus_t ret;

  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (0x1 > dbm_idx || 0x06 < dbm_idx) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;

  ret = twicSsLeCeSetTxPower(TWIC_LECESETTXPOWER, conn, dbm_idx);

 out:

  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  return ret;
}
#endif

twicStatus_t
_twicIfLeCeFlowControl(twicEntry_t * const conn, const bool enable)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;

  ret = twicSsLeCeFlowControl(TWIC_LECEFLOWCONTROL, conn, enable);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeCeSetBaudrate(twicEntry_t * const conn,
                                    const twicTzbtBr_t br, const uint16_t pu)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (TWIC_TZBT_PUNCTUATION_MIN > pu || TWIC_TZBT_PUNCTUATION_MAX < pu) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;    
  }
  
  ret = twicSsLeCeSetBaudrate(TWIC_LECESETBAUDRATE, conn, br, pu);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfLeCeHostDelay(twicEntry_t * const conn, const uint16_t delay)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeCeHostDelay(TWIC_LECEHOSTDELAY, conn, delay);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t _twicIfLeCeLowPowerPrimarySetup(twicEntry_t * const conn,
                                             const uint16_t osc_tolerance,
                                             const uint16_t osc_jitter)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeCeLowPowerInit(TWIC_LECELOWPOWERPRIMARYSETUP, conn, 0x7D0, 1,
                               osc_tolerance, osc_jitter, 9);

  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t _twicIfLeCeLowPowerClockSetup(twicEntry_t * const conn,
                                           twicStatus_t * const osc_state)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (true == twic_eq.sniff.lowpower_clock) {
    ret = TWIC_STATUS_OPERATION_NOT_PERMITTED;
    goto out;
  }

#if defined(TWIC_LECE_CLK32K_SLPXOIN_XTAL)
  ret = twicSsLeCeLowPowerClock(TWIC_LECELOWPOWERCLOCKSETUP, conn, 0, 0x0FA0,
                                osc_state);
#else /* TWIC_LECE_CLK32K_SLPXOIN_CLOCK */
  ret = twicSsLeCeLowPowerClock(TWIC_LECELOWPOWERCLOCKSETUP, conn, 1, 0x0001,
                                osc_state);
#endif
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t _twicIfLeCeLowPowerControlPinSetup(
  twicEntry_t * const conn, const bool host_wake)
{
  twicStatus_t ret;
  uint8_t pin = 0xFF;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (true == host_wake) pin = 1;
  ret = twicSsLeCeLowPowerControlPin(TWIC_LECELOWPOWERCONTROLPINSETUP, conn,
                                     pin, 2);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t _twicIfLeCeLowPowerDiscoverableSetup(twicEntry_t * const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeCeLowPowerBackup(
    TWIC_LECELOWPOWERDISCOVERABLESETUP, conn, 0x35, 0x76);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t _twicIfLeCeHkLowPowerInfo(twicCeLeLpInfo_t * const info)
{
  twicStatus_t ret = TWIC_STATUS_OK;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (NULL == info) ret = TWIC_STATUS_ERROR_PARAMETER;
  else twicSsLeCeHkLowPowerInfo(info);

  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  return ret;
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t _twicIfLeCeHkLowPowerTransition(const bool enable)
{
  twicStatus_t ret = TWIC_STATUS_OK;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (true == enable) {
    twic_eq.sniff.waiting_for_activation = false;
    twic_eq.slpto = 0;
    ret = twicSsLeRequestLowPower();
  }
  else twicSsLeRequestHighPower();

  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  return ret;
}
#endif

#if defined(TZ1EM_POWER_PROFILE)
twicStatus_t _twicIfLeCeInitializeTz1em(void)
{
  return (TZ1EM_STATUS_OK == tz1emInitializeSystem()) ?
    TWIC_STATUS_OK : TWIC_STATUS_ERROR_TZ1EM;
}

static tz1em_t * twicIfFindEntry(const uint8_t type)
{
  if (TWIC_TZ1EM_NC_ADV == type) return TZ1EM(BLE_NC_ADV);
  if (TWIC_TZ1EM_CN_ADV == type) return TZ1EM(BLE_CN_ADV);
  if (TWIC_TZ1EM_IDLE == type) return TZ1EM(BLE_IDLE);
  if (TWIC_TZ1EM_CONN == type) return TZ1EM(BLE_CONN);
  return NULL;
}  

twicStatus_t
_twicIfLeCeConfigureTz1em(const uint8_t type, tz1emRequirement_t * const gp)
{
  twicStatus_t ret = TWIC_STATUS_OK;
  tz1em_t *p;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (NULL == (p = twicIfFindEntry(type))) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  if (TWIC_TZ1EM_CN_ADV == type || TWIC_TZ1EM_CONN == type) {
    if (TZ1EM_OP_BLE_BG_S1 < gp->mode && RPQUEUE_PM_HDD > twic_eq.host_delay) {
      ret = TWIC_STATUS_ERROR_PARAMETER;
      goto out;
    }
  }
  
  gp->pcd = TZ1EM_PCDG_BLE_TZBT;
  gp->trigger[TZ1EM_NUM_OF_TRIGGER - 1].event = TZ1SM_HAL_ACT_EVENT_EDGE_BOTH;
  gp->trigger[TZ1EM_NUM_OF_TRIGGER - 1].factor = TZ1SM_HAL_ACT_FACTOR_GPIO_30;
  
  if (TWIC_TZ1EM_NC_ADV == type) {
    gp->permissible_time_lag_us = TZ1EM_PTL_SAFE_ASYNCHRONOUS_IO;    
  } else {
    gp->permissible_time_lag_us = twic_eq.host_delay * 1000; /* ms */
  }

  if (TZ1EM_STATUS_OK == tz1emConfigureEntry(p, gp)) {
    twic_eq.sniff.lowpower_core = true;
  } else if (TZ1EM_STATUS_OK == tz1emReconfigureEntry(p, gp)) {
    twic_eq.sniff.lowpower_core = true;
  } else {
    twic_eq.sniff.lowpower_core = false;
    twic_eq.sniff.lowpower_core_idle = false;
    twic_eq.sniff.lowpower_core_cosc = false;
    twic_eq.sniff.lowpower_core_adve = false;
    twic_eq.sniff.lowpower_core_ncad = false;
    ret = TWIC_STATUS_ERROR_TZ1EM;
  }
  
  out:
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  return ret;
}

twicStatus_t _twicIfLeCeWithdrawalFromTz1em(const uint8_t type)
{
  twicStatus_t ret = TWIC_STATUS_OK;
  tz1em_t *p;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (NULL == (p = twicIfFindEntry(type))) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  if (TZ1EM_STATUS_OK != tz1emWithdraw(p)) ret = TWIC_STATUS_ERROR_TZ1EM;
  if (false == TZ1EM(BLE_NC_ADV)->info.enable &&
      false == TZ1EM(BLE_CN_ADV)->info.enable &&
      false == TZ1EM(BLE_IDLE)->info.enable &&
      false == TZ1EM(BLE_CONN)->info.enable) {
    twic_eq.sniff.lowpower_core = false;
    twic_eq.sniff.lowpower_core_idle = false;
    twic_eq.sniff.lowpower_core_cosc = false;
    twic_eq.sniff.lowpower_core_adve = false;
    twic_eq.sniff.lowpower_core_ncad = false;
  }
  
  out:
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  return ret;
}

twicStatus_t _twicIfLeCePermitTz1em(const uint8_t type)
{
  twicStatus_t ret = TWIC_STATUS_OK;
  tz1em_t *p;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (NULL == (p = twicIfFindEntry(type))) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  if (TZ1EM_STATUS_OK == tz1emPermit(p)) twic_eq.sniff.lowpower_core = true;
  else ret = TWIC_STATUS_ERROR_TZ1EM;

  out:
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  return ret;
}

void _twicIfLeCeHkTz1emInfo(const uint8_t type, tz1emInfo_t * const info)
{
  tz1em_t *p;
  
  if (NULL == (p = twicIfFindEntry(type))) return;
  info->necessary_time_lag_us = p->info.necessary_time_lag_us;
  info->wakeup_reason = p->info.wakeup_reason;
  info->timer_expired = p->info.timer_expired;
  info->init = p->info.init;
  info->conf = p->info.conf;
  info->enable = p->info.enable;
  info->voltage = p->info.voltage;
  
  return;
}
#endif

#if defined(TWIC_LECE_LOWPOWER)
twicStatus_t _twicIfLeCeLowPowerMode(twicEntry_t * const conn, const bool idle,
                                     const bool discoverable,
                                     const bool conn_scan, const bool shutdown)
{
  uint8_t selector = 0;
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  /* ROM5 can not confirm Shutdown if one of Idle, Discoverable and
   * Conn_Scan is true. 20150313. */
  if (true == shutdown) {
    selector |= RPQUEUE_PM_DEEP_SLEEP;
  } else {
    if (true == conn_scan) selector |= RPQUEUE_PM_SLEEP;
    if (true == discoverable) selector |= RPQUEUE_PM_BACKUP1;
    if (true == idle) selector |= RPQUEUE_PM_BACKUP2;
  }
  
  ret = twicSsLeCeLowPowerMode(TWIC_LECELOWPOWERMODE, conn, selector);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

twicStatus_t _twicIfLeReadBdaddr(twicEntry_t * const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeReadBdaddr(TWIC_LEREADBDADDR, conn);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

#if defined(TWIC_API_LEREADLOCALVERSIONINFORMATION)
twicStatus_t _twicIfLeReadLocalVersionInformation(twicEntry_t * const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeReadLocalVersionInformation(TWIC_LEREADLOCALVERSIONINFORMATION,
                                            conn);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

twicStatus_t
_twicIfLeWriteBdaddr(twicEntry_t * const conn, const uint64_t * const bd)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeWriteBdaddr(TWIC_LEWRITEBDADDR, conn, bd);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeCeChangeToCm(twicEntry_t * const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeCeChangeToCm(TWIC_LECECHANGETOCM, conn);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeInitializeDevice(twicEntry_t * const conn)
{
  twicStatus_t ret;
#if defined(TWIC_BLE_HWIP_V41)
#if defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
  const uint8_t compatibility[3] = { 0x0,
                                     1,
                                     1 };
#else
  const uint8_t compatibility[3] = { 0x0,
                                     TWIC_BLE_HWIP_V41_MASTERS,
                                     TWIC_BLE_HWIP_V41_SLAVES };
#endif
#else
  const uint8_t compatibility[3] = { 0x0,
                                     0,
                                     0 };
#endif
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (!(conn->instance->server_cb)) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
#if defined(TWIC_BLE_HWIP_V41) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
#if 5 < (TWIC_BLE_HWIP_V41_MASTERS + TWIC_BLE_HWIP_V41_SLAVES)
#error "Maximum Masters + Maximum Slaves should be <=5."
#endif
#if 0 == (TWIC_BLE_HWIP_V41_MASTERS + TWIC_BLE_HWIP_V41_SLAVES)
#error "It is permitted that number of Master supported or number of slaves supported can be zero, but both cannot be zero."
#endif
#if 4 < TWIC_BLE_HWIP_V41_MASTERS
#error "Maximum Masters + Maximum Slaves should be <=4."
#endif
#if 2 < TWIC_BLE_HWIP_V41_SLAVES
#error "Maximum Slaves should be <=2."
#endif
#endif
#if defined(TWIC_BLE_HWIP_V41)
  ret = twicSsReq(TWIC_LEINITIALIZEDEVICE, conn, 0x01,
                  TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, &compatibility[0], 3);
#else  
  ret = twicSsReq(TWIC_LEINITIALIZEDEVICE, conn, 0x01,
                  TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, &compatibility[0], 1);
#endif
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeGattClientStart(twicEntry_t * const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsReq(TWIC_LEGATTCLIENTSTART, conn,
                  0x00, TWIC_SERVICE_ID_GATT_CLIENT, NULL, 0);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

#if defined(TWIC_CONFIG_SM_INITIATOR) || defined(WIC_CONFIG_SM_RESPONDER) || defined(TWIC_BLE_HWIP_V41)
static twicStatus_t _twicIfLeNegativeReply(const uint8_t seq,
                                           twicEntry_t * const conn,
                                           uint8_t reason)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  switch (seq) {
  case TWIC_LESMMASKBPASSKEYNEGATIVEREPLY:
    ret = twicSsTreReq(seq, conn, 0x05,
                       TWIC_SERVICE_ID_SMP_INITIATOR, reason); break;
  case TWIC_LESMMASDPPASSKEYNEGATIVEREPLY:
    ret = twicSsTreReq(seq, conn, 0x07,
                       TWIC_SERVICE_ID_SMP_INITIATOR, reason); break;
  case TWIC_LESMMASOOBTKNEGATIVEREPLY:
    ret = twicSsTreReq(seq, conn, 0x19,
                       TWIC_SERVICE_ID_SMP_INITIATOR, reason); break;
  case TWIC_LESMMASBONDINGINFORMATIONNEGATIVEREPLY:
    ret = twicSsTreReq(seq, conn, 0x1b,
                       TWIC_SERVICE_ID_SMP_INITIATOR, reason); break;
  case TWIC_LESMSLVKBPASSKEYNEGATIVEREPLY:
    ret = twicSsTreReq(seq, conn, 0x05,
                       TWIC_SERVICE_ID_SMP_RESPONDER, reason); break;
  case TWIC_LESMSLVDPPASSKEYNEGATIVEREPLY:
    ret = twicSsTreReq(seq, conn, 0x07,
                       TWIC_SERVICE_ID_SMP_RESPONDER, reason); break;
  case TWIC_LESMSLVOOBTKNEGATIVEREPLY:
    ret = twicSsTreReq(seq, conn, 0x1a,
                       TWIC_SERVICE_ID_SMP_RESPONDER, reason); break;
  case TWIC_LESMSLVBONDINGINFORMATIONNEGATIVEREPLY:
    ret = twicSsTreReq(seq, conn, 0x1c,
                       TWIC_SERVICE_ID_SMP_RESPONDER, reason); break;
  case TWIC_LECONNECTIONPARAMETERREQUESTNEGATIVEREPLY:
    ret = twicSsTreReq(seq, conn, 0x1e,
                       TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, reason); break;
  case TWIC_LECEL2CAPSIGNALREJECTCONNECTIONUPDATE:
    ret = twicSsTreReq(seq, conn, 0x16,
                       TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, reason); break;
  default:
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

static twicStatus_t _twicIfLeConnectionParameter(const uint8_t seq,
                                                 twicEntry_t * const conn,
                                                 uint16_t conn_int_min,
                                                 uint16_t conn_int_max,
                                                 uint16_t slave_latency,
                                                 uint16_t supervison_timeout,
                                                 uint16_t min_ce_length,
                                                 uint16_t max_ce_length)
{
  twicStatus_t ret;
  uint8_t opc;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = TWIC_STATUS_ERROR_PARAMETER;
  /* Time = N * 1.25 msec Time Range: 7.5 msec to 4 sec. */
  if (0x0006 > conn_int_min || 0x0C80 < conn_int_min) {twicTrace(); goto out;}
  /* Time = N * 1.25 msec Time Range: 7.5 msec to 4 sec. */
  if (0x0006 > conn_int_max || 0x0C80 < conn_int_max) {twicTrace(); goto out;}
  /* Slave latency for the connection in number of connection events. */
  if (0x01F3 < slave_latency) {twicTrace(); goto out;}
  /* Time = N * 10 msec Time Range: 100 msec to 32 seconds. */
  if (0x000A > supervison_timeout || 0x0C80 < supervison_timeout ||
      conn_int_max > supervison_timeout) {twicTrace(); goto out;}

  if (TWIC_LECONNECTIONUPDATE == seq) opc = 0x0e;
  else if (TWIC_LECEL2CAPSIGNALACCEPTCONNECTIONUPDATE == seq) opc = 0x16;
  else if (TWIC_LECONNECTIONPARAMETERREQUESTREPLY == seq ||
           TWIC_LECONNECTIONUPDATE == seq) opc = 0x1e;
  else goto out;

  ret = twicSsLeConnectionUpdate(seq, conn, opc, conn_int_min, conn_int_max,
                                 slave_latency, supervison_timeout,
                                 min_ce_length, max_ce_length);
 out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeConnectionUpdate(twicEntry_t * const conn,
                                       uint16_t conn_int_min,
                                       uint16_t conn_int_max,
                                       uint16_t slave_latency,
                                       uint16_t supervison_timeout,
                                       uint16_t min_ce_length,
                                       uint16_t max_ce_length)
{
  return _twicIfLeConnectionParameter(
    TWIC_LECONNECTIONUPDATE, conn, conn_int_min, conn_int_max,
    slave_latency, supervison_timeout, min_ce_length, max_ce_length);
}

#if defined(TWIC_BLE_HWIP_V41)
/* 0x00 Success
   0x86 Parameter Error
   0x81 Device Not Initialized
   0x82 Command Under Process
   0xA0 Device Role Slave
   For other errors, refer to HCI Command Error Code. */
twicStatus_t _twicIfLeConnectionParameterRequestReply(
  twicEntry_t * const conn, uint16_t conn_int_min,
  uint16_t conn_int_max, uint16_t slave_latency, uint16_t supervison_timeout)
{
  return _twicIfLeConnectionParameter(
    TWIC_LECONNECTIONPARAMETERREQUESTREPLY, conn,
    conn_int_min, conn_int_max, slave_latency, supervison_timeout, 0, 0);
}

/* const uint8_t reason: LIST OF ERROR CODES [1]
   0x00 Success
   0x86 Parameter Error
   0x81 Device Not Initialized
   0x82 Command Under Process
   0xA0 Device Role Slave
   For other errors, refer to HCI Command Error Code. */
twicStatus_t _twicIfLeConnectionParameterRequestNegativeReply(
  twicEntry_t * const conn, const uint8_t reason)
{
  return _twicIfLeNegativeReply(TWIC_LECONNECTIONPARAMETERREQUESTNEGATIVEREPLY,
                                conn, reason);
}
#endif

twicStatus_t
_twicIfLeDisconnect(twicEntry_t * const conn, const twicBdaddr_t * const bd)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeDisconnect(TWIC_LEDISCONNECT, conn, bd);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeGattServerStart(twicEntry_t * const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsReq(TWIC_LEGATTSERVERSTART, conn,
                  0x00, TWIC_SERVICE_ID_GATT_SERVER, NULL, 0);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfLeReadTxPowerLevel(twicEntry_t * const conn, const uint8_t type)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;

  /* 0x00:Read Current Transmit Power Level.
     0x01:Read Maximum Transmit Power Level. */
  if (!(0x00 == type || 0x01 == type)) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  ret = twicSsTreReq(TWIC_LEREADTXPOWERLEVEL,
                     conn, 0x14, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, type);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeReadRssi(twicEntry_t * const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsToReq(
    TWIC_LEREADRSSI, conn, 0x15, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeReadRemoteVersion(twicEntry_t * const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeReadRemoteVersion(TWIC_LEREADREMOTEVERSION, conn);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfLeCreateConnection(twicEntry_t * const conn, uint16_t interval,
                          uint16_t window, bool use_white_list,
                          bool peer_bdaddr_is_random,
                          const twicBdaddr_t *const peer_bdaddr,
                          uint16_t conn_int_min, uint16_t conn_int_max,
                          uint16_t slave_latency, uint16_t supervison_timeout,
                          uint16_t min_ce_length, uint16_t max_ce_length,
                          bool own_bdaddr_is_random)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = TWIC_STATUS_ERROR_PARAMETER;
  /* Time = N * 0.625 msec. Time Range: 2.5 msec to 10240 msec */
  if (0x0004 > interval || 0x4000 < interval) goto out;
  /* Time = N * 0.625 msec. Time Range: 2.5 msec to 10.24 sec */
  if (0x0004 > window || 0x4000 < window) goto out;
  /* Time = N * 1.25 msec Time Range: 7.5 msec to 4 sec. */
  if (0x0006 > conn_int_min || 0x0C80 < conn_int_min) goto out;
  /* Time = N * 1.25 msec Time Range: 7.5 msec to 4 sec. */
  if (0x0006 > conn_int_max || 0x0C80 < conn_int_max) goto out;
  /* Slave latency for the connection in number of connection events. */
  if (0x01F3 < slave_latency) goto out;
  /* Supervision timeout for the LE Link.This value will be greater
   * than Conn_Int_Max. */
  /* Time = N * 10 msec Time Range: 100 msec to 32 seconds. */
  if (0x000A > supervison_timeout || 0x0C80 < supervison_timeout ||
      conn_int_max > supervison_timeout) ret = TWIC_STATUS_ERROR_PARAMETER;

  ret = twicSsLeCreateConnection(TWIC_LECREATECONNECTION, conn, interval,
                                 window, use_white_list, peer_bdaddr_is_random,
                                 peer_bdaddr, conn_int_min, conn_int_max,
                                 slave_latency, supervison_timeout,
                                 min_ce_length, max_ce_length,
                                 own_bdaddr_is_random);
  out:
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeCreateConnectionCancel(twicEntry_t * const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
#if defined(TWIC_IF_TRACE_MUTEX)
  twicTrace();
#endif  
  ret = twicSsReq(TWIC_LECREATECONNECTIONCANCEL, conn,
                  0x0d, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, NULL, 0);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#if defined(TWIC_API_LELMGENRESOLVABLEBDADDR)
twicStatus_t _twicIfLeLmGenResolvableBdaddr(twicEntry_t * const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsReq(TWIC_LELMGENRESOLVABLEBDADDR, conn,
                  0x17, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, NULL, 0);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_API_LELMRESOLVEBDADDR)
twicStatus_t
_twicIfLeLmResolveBdaddr(twicEntry_t *const conn, const twicBdaddr_t *const bd,
                         const uint8_t num_of_irk,
                         const twicIrk_t * const irks)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeLmResolveBdaddr(TWIC_LELMRESOLVEBDADDR, conn,
                                bd, num_of_irk, irks);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_API_LEADDWHITELIST)
twicStatus_t _twicIfLeAddWhitelist(twicEntry_t *const conn, const bool random,
                                   const twicBdaddr_t *const bd)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeAddWhitelist(TWIC_LEADDWHITELIST, conn, random, bd);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_CONFIG_ENABLE_SCAN)
twicStatus_t
_twicIfLeSetScanEnable(twicEntry_t *const conn, const uint16_t interval,
                       const uint16_t window, const bool active_scanning,
                       const bool own_bdaddr_is_random,
                       const bool whitelist_enable, const bool scan_enable,
                       const bool filter_duplicates)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  /* Time = N * 0.625 msec. Time Range: 2.5 msec to 10.24 sec */
  if (0x0004 > interval || 0x4000 < interval) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  /* Time = N * 0.625 msec. Time Range: 2.5 msec to 10.24 sec */
  if (0x0004 > window || 0x4000 < window) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  ret = twicSsLeSetScanEnable(TWIC_LESETSCANENABLE, conn, interval, window,
                              (true == active_scanning ? 1 : 0),
                              (true == own_bdaddr_is_random ? 1 : 0),
                              (true == whitelist_enable ? 1 : 0),
                              (true == scan_enable ? 1 : 0),
                              (true == filter_duplicates ? 1 : 0));
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeSetScanDisable(twicEntry_t *const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsReq(TWIC_LESETSCANDISABLE, conn,
                  0x0b, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, NULL, 0);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_LEDELWHITELIST)
twicStatus_t _twicIfLeDelWhitelist(twicEntry_t *const conn, const bool random,
                                   const twicBdaddr_t *const bd)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (NULL == bd) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  ret = twicSsLeDelWhitelist(TWIC_LEDELWHITELIST, conn, random, bd);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_API_LECLEAREWHITELIST)
twicStatus_t _twicIfLeClearWhitelist(twicEntry_t *const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeClearWhitelist(TWIC_LECLEAREWHITELIST, conn);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_API_LEREADWHITELISTSIZE)
twicStatus_t _twicIfLeReadWhitelistSize(twicEntry_t *const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeReadWhitelistSize(TWIC_LEREADWHITELISTSIZE, conn);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
twicStatus_t _twicIfLeReadRemoteUsedFeatures(twicEntry_t *const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeReadRemoteUsedFeatures(TWIC_LEREADREMOTEUSEDFEATURES, conn);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#if defined(TWIC_API_LEREADLOCALSUPPORTEDFEATURES)
twicStatus_t _twicIfLeReadLocalSupportedFeatures(twicEntry_t *const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeReadLocalSupportedFeatures(TWIC_LEREADLOCALSUPPORTEDFEATURES,
                                           conn);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_API_LESETHOSTCHANNELCLASSIFICATION)
twicStatus_t
_twicIfLeSetHostChannelClassification(
  twicEntry_t *const conn, const twicChannelMap_t *const channel_map)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeSetHostChannelClassification(
    TWIC_LESETHOSTCHANNELCLASSIFICATION, conn, channel_map);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_LEREADCHANNELMAP)
twicStatus_t _twicIfLeReadChannelMap(twicEntry_t *const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeReadChannelMap(TWIC_LEREADCHANNELMAP, conn);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_API_LEREADSUPPORTEDSTATES)
twicStatus_t _twicIfLeReadSupportedStates(twicEntry_t *const conn)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeReadSupportedStates(TWIC_LEREADSUPPORTEDSTATES, conn);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_API_LESETRANDOMADDRESS)
twicStatus_t _twicIfLeSetRandomAddress(twicEntry_t *const conn,
                                       const twicBdaddr_t *const bd)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeSetRandomAddress(TWIC_LESETRANDOMADDRESS, conn, bd);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_API_LESMSETIRVALUE)
twicStatus_t
_twicIfLeSmSetIrValue(twicEntry_t * const conn, const twicIr_t * const ir)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeSmSetIrValue(TWIC_LESMSETIRVALUE, conn, ir);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_BLE_HWIP_V41) && defined(TWIC_API_LESMENCRYPT) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
twicStatus_t
_twicIfLeSmEncrypt(twicEntry_t * const conn, const uint8_t * const key,
                   const uint8_t * const plain_text_data)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeSmEncrypt(TWIC_LESMENCRYPT, conn, key, plain_text_data);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_API_LELESETADVERTISINGDATA) && defined(TWIC_BLE_HWIP_V41)
twicStatus_t
_twicIfLeSetAdvertisingData(twicEntry_t * const conn,
                            const uint8_t advertising_data_length,
                            const uint8_t * const advertising_data)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (31 < advertising_data_length) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  ret = twicSsLeSetAdvertisingData(TWIC_LESETADVERTISINGDATA, conn,
                                   advertising_data_length,
                                   advertising_data);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_CONFIG_SM_INITIATOR)
twicStatus_t
_twicIfLeSmMasPairingRequest(twicEntry_t * const conn,
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
  twicStatus_t ret;
  uint8_t auth_req = 0;
  uint8_t init_key = 0;
  uint8_t resp_key = 0;
  uint8_t oob_data_flag = 0;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (true == auth_req_bonding)         auth_req |= 0x1;
  if (true == auth_req_mitm_protection) auth_req |= 0x4;

  if (true == init_key_dist_enckey)     init_key |= 0x1;
  if (true == init_key_dist_idkey)      init_key |= 0x2;
  if (true == init_key_dist_sign)       init_key |= 0x4;

  if (true == resp_key_dist_enckey)     resp_key |= 0x1;
  if (true == resp_key_dist_idkey)      resp_key |= 0x2;
  if (true == resp_key_dist_sign)       resp_key |= 0x4;

  if (true == oob_data_present) oob_data_flag = TWIC_SMP_OOB_DATA_PRESENT;
  
  ret = twicSsLeSmMasPairingRequest(
    TWIC_LESMMASPAIRINGREQUEST, conn, io_capability, oob_data_flag,
    auth_req, max_enc_key_size, init_key, resp_key);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeSmMasSecurityReject(twicEntry_t * const conn,
                                          const twicSmReasonCode_t reason)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsTreReq(TWIC_LESMMASSECURITYREJECT, conn,
                     0x02, TWIC_SERVICE_ID_SMP_INITIATOR, reason);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeSmMasPairingFailed(twicEntry_t * const conn,
                                         const twicSmReasonCode_t reason)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsTreReq(TWIC_LESMMASPAIRINGFAILED, conn,
                     0x11, TWIC_SERVICE_ID_SMP_INITIATOR, reason);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfLeSmMasKbPasskeyReply(twicEntry_t * const conn,
                             const twicPasskeyEntry_t *const passkey)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;

  ret = twicSsLeSmPasskeyReply(TWIC_LESMMASKBPASSKEYREPLY, conn,
                               0x05, TWIC_SERVICE_ID_SMP_INITIATOR, passkey);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeSmMasKbPasskeyNegativeReply(twicEntry_t * const conn)
{
  return _twicIfLeNegativeReply(TWIC_LESMMASKBPASSKEYNEGATIVEREPLY, conn, 1);
}

twicStatus_t
_twicIfLeSmMasDpPasskeyReply(twicEntry_t * const conn,
                             const twicPasskeyEntry_t *const passkey)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeSmPasskeyReply(TWIC_LESMMASDPPASSKEYREPLY, conn, 0x07,
                               TWIC_SERVICE_ID_SMP_INITIATOR, passkey);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeSmMasDpPasskeyNegativeReply(twicEntry_t * const conn)
{
  return _twicIfLeNegativeReply(TWIC_LESMMASDPPASSKEYNEGATIVEREPLY, conn, 1);
}

twicStatus_t
_twicIfLeSmMasOobTkReply(twicEntry_t * const conn, const twicOobTk_t *const tk)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeSmOobTkReply(
    TWIC_LESMMASOOBTKREPLY, conn, 0x19, TWIC_SERVICE_ID_SMP_INITIATOR, tk);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeSmMasOobTkNegativeReply(twicEntry_t * const conn)
{
  return _twicIfLeNegativeReply(TWIC_LESMMASOOBTKNEGATIVEREPLY, conn, 1);
}

twicStatus_t
_twicIfLeSmMasBondingInformationReply(twicEntry_t * const conn,
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
                                      const uint8_t encryption_key_size)
{
  twicStatus_t ret;
  uint8_t bits = 0;
  uint8_t size = 0;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (NULL != r_ediv && NULL != r_rand && NULL != r_ltk)     bits |= 0x01;
  if (NULL != r_irk)                                         bits |= 0x02;
  if (NULL != r_csrk)                                        bits |= 0x04;
  if (NULL != l_ediv && NULL != l_rand && NULL != l_ltk)     bits |= 0x08;
  if (NULL != l_irk)                                         bits |= 0x10;
  if (NULL != l_csrk)                                        bits |= 0x20;
  if (7 <= encryption_key_size && 16 >= encryption_key_size) {
    bits |= 0x40;
    size = encryption_key_size;
  }
    
  ret = twicSsLeSmBondingInformationReply(
    TWIC_LESMMASBONDINGINFORMATIONREPLY, conn, 0x1b,
    TWIC_SERVICE_ID_SMP_INITIATOR, bits, r_ediv, r_rand, r_ltk, r_irk, r_csrk,
    l_ediv, l_rand, l_ltk, l_irk, l_csrk, size);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfLeSmMasBondingInformationNegativeReply(twicEntry_t * const conn)
{
  return _twicIfLeNegativeReply(TWIC_LESMMASBONDINGINFORMATIONNEGATIVEREPLY,
                                conn, 1);
}

twicStatus_t
_twicIfLeSmMasBondingState(twicEntry_t * const conn, const twicAuthInfo_t bits)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsTreReq(TWIC_LESMMASBONDINGSTATE, conn, 0x1a,
                     TWIC_SERVICE_ID_SMP_INITIATOR, *(uint8_t*)&bits);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeSmMasStartEncryption(twicEntry_t * const conn,
                                           const twicEdiv_t *const ediv,
                                           const twicRand_t *const rand,
                                           const twicLtk_t *const ltk,
                                           const uint8_t encryption_key_size)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeSmMasStartEncryption(TWIC_LESMMASSTARTENCRYPTION, conn, ediv,
                                     rand, ltk, encryption_key_size);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_CONFIG_SM_RESPONDER)
twicStatus_t
_twicIfLeSmSlvPairingConfirm(twicEntry_t * const conn,
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
  twicStatus_t ret;
  uint8_t auth_req = 0;
  uint8_t init_key = 0;
  uint8_t resp_key = 0;
  uint8_t oob_data_flag = 0;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (true == auth_req_bonding)         auth_req |= 0x1;
  if (true == auth_req_mitm_protection) auth_req |= 0x4;

  if (true == init_key_dist_enckey)     init_key |= 0x1;
  if (true == init_key_dist_idkey)      init_key |= 0x2;
  if (true == init_key_dist_sign)       init_key |= 0x4;

  if (true == resp_key_dist_enckey)     resp_key |= 0x1;
  if (true == resp_key_dist_idkey)      resp_key |= 0x2;
  if (true == resp_key_dist_sign)       resp_key |= 0x4;
  
  if (true == oob_data_present) oob_data_flag = TWIC_SMP_OOB_DATA_PRESENT;

  ret = twicSsLeSmSlvPairingConfirm(TWIC_LESMSLVPAIRINGCONFIRM, conn,
                                    io_capability, oob_data_flag, auth_req,
                                    max_enc_key_size, init_key, resp_key);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeSmSlvPairingFailed(twicEntry_t * const conn,
                                         const twicSmReasonCode_t reason)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsTreReq(TWIC_LESMSLVPAIRINGFAILED, conn,
                     0x13, TWIC_SERVICE_ID_SMP_RESPONDER, reason);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfLeSmSlvKbPasskeyReply(twicEntry_t * const conn,
                             const twicPasskeyEntry_t *const passkey)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeSmPasskeyReply(TWIC_LESMSLVKBPASSKEYREPLY, conn,
                               0x05, TWIC_SERVICE_ID_SMP_RESPONDER, passkey);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeSmSlvKbPasskeyNegativeReply(twicEntry_t * const conn)
{
  return _twicIfLeNegativeReply(TWIC_LESMSLVKBPASSKEYNEGATIVEREPLY, conn, 1);
}
  
twicStatus_t _twicIfLeSmSlvSecurityRequest(twicEntry_t * const conn,
                                           const bool auth_req_bonding,
                                           const bool auth_req_mitm_protection)
{
  twicStatus_t ret;
  uint8_t auth_req = 0;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (true == auth_req_bonding) auth_req |= 0x1;
  if (true == auth_req_mitm_protection) auth_req |= 0x4;

  ret = twicSsTreReq(TWIC_LESMSLVSECURITYREQUEST, conn,
                     0x02, TWIC_SERVICE_ID_SMP_RESPONDER, auth_req);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfLeSmSlvDpPasskeyReply(twicEntry_t * const conn,
                             const twicPasskeyEntry_t *const passkey)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeSmPasskeyReply(TWIC_LESMSLVDPPASSKEYREPLY, conn, 0x07,
                               TWIC_SERVICE_ID_SMP_RESPONDER, passkey);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeSmSlvDpPasskeyNegativeReply(twicEntry_t * const conn)
{
  return _twicIfLeNegativeReply(TWIC_LESMSLVDPPASSKEYNEGATIVEREPLY, conn, 1);
}

twicStatus_t
_twicIfLeSmSlvOobTkReply(twicEntry_t * const conn, const twicOobTk_t *const tk)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsLeSmOobTkReply(
    TWIC_LESMSLVOOBTKREPLY, conn, 0x1a, TWIC_SERVICE_ID_SMP_RESPONDER, tk);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeSmSlvOobTkNegativeReply(twicEntry_t * const conn)
{
  return _twicIfLeNegativeReply(TWIC_LESMSLVOOBTKNEGATIVEREPLY, conn, 1);
}

twicStatus_t
_twicIfLeSmSlvBondingInformationReply(twicEntry_t * const conn,
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
                                      const uint8_t encryption_key_size)
{
  twicStatus_t ret;
  uint8_t bits = 0;
  uint8_t size = 0;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (NULL != r_ediv && NULL != r_rand && NULL != r_ltk)     bits |= 0x01;
  if (NULL != r_irk)                                         bits |= 0x02;
  if (NULL != r_csrk)                                        bits |= 0x04;
  if (NULL != l_ediv && NULL != l_rand && NULL != l_ltk)     bits |= 0x08;
  if (NULL != l_irk)                                         bits |= 0x10;
  if (NULL != l_csrk)                                        bits |= 0x20;
  if (7 <= encryption_key_size && 16 >= encryption_key_size) {
    bits |= 0x40;
    size = encryption_key_size;
  }
    
  ret = twicSsLeSmBondingInformationReply(
    TWIC_LESMSLVBONDINGINFORMATIONREPLY, conn, 0x1c,
    TWIC_SERVICE_ID_SMP_RESPONDER, bits, r_ediv, r_rand, r_ltk, r_irk, r_csrk,
    l_ediv, l_rand, l_ltk, l_irk, l_csrk, size);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfLeSmSlvBondingInformationNegativeReply(twicEntry_t * const conn)
{
  return _twicIfLeNegativeReply(TWIC_LESMSLVBONDINGINFORMATIONNEGATIVEREPLY,
                                conn, 1);
}

twicStatus_t _twicIfLeSmSlvBondingState(
  twicEntry_t * const conn, const twicAuthInfo_t bits)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsTreReq(TWIC_LESMSLVBONDINGSTATE, conn,
                     0x1b, TWIC_SERVICE_ID_SMP_RESPONDER, *(uint8_t*)&bits);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

twicStatus_t _twicIfLeGattBeginServiceCreation(twicEntry_t * const entry,
                                               const uint64_t uuid_lsb,
                                               const uint64_t uuid_msb,
                                               const uint8_t uuid_len)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  twicUuid_t uuid;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(entry);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (entry->handle || entry->u.attr.b_handle) {
    twicTrace();
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  uuid.len = uuid_len;
  twicSetUuid((uint8_t *)&uuid.uu, uuid_lsb, uuid_msb);
  
  ret = twicSsGattServerAddService(
    TWIC_LEGATTDBBEGINSERVICECREATION, entry, &uuid);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfLeGattBeginSecondaryServiceCreation(const twicEntry_t * const service,
                                           twicEntry_t * const entry,
                                           const uint64_t uuid_lsb,
                                           const uint64_t uuid_msb,
                                           const uint8_t uuid_len)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  twicUuid_t uuid;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(entry);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (entry->handle || entry->u.attr.b_handle || !(service->handle)) {
    twicTrace();
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  uuid.len = uuid_len;
  twicSetUuid((uint8_t *)&uuid.uu, uuid_lsb, uuid_msb);
  
  entry->u.attr.p_handle = service->handle;
  ret = twicSsGattServerAddSecondaryService(
    TWIC_LEGATTDBBEGINSECONDARYSERVICECREATION, entry, &uuid);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeGattIncludeServiceCreation(twicEntry_t * const service,
                                                 twicEntry_t * const inclusion,
                                                 const uint64_t uuid_lsb,
                                                 const uint64_t uuid_msb,
                                                 const uint8_t uuid_len)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  twicUuid_t uuid;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  if (TWIC_UUID16 != uuid_len && TWIC_UUID128 != uuid_len) {
    twicTrace();
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  if (RPQUEUE_ENABLE != twic_eq.sniff.enable)
    goto out;
  
  if (TWIC_UUID16 == uuid_len) {
    uuid.len = uuid_len;
    twicSetUuid((uint8_t *)&uuid.uu, uuid_lsb, uuid_msb);
  }
  
  if ((TWIC_ENTRY_TYPE_CHARACTERISTIC_VALUE == service->type) ||
      (TWIC_ENTRY_TYPE_CHARACTERISTIC_DESCRIPTOR == service->type) ||
      (TWIC_ENTRY_TYPE_CHARACTERISTIC_VALUE == inclusion->type) ||
      (TWIC_ENTRY_TYPE_CHARACTERISTIC_DESCRIPTOR == inclusion->type)) {
    twicTrace();
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  ret = twicSsGattServerAddIncludeService(
    TWIC_LEGATTDBINCLUDESERVICECREATION, service, inclusion, &uuid);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeGattAddCharacteristics(const twicEntry_t * const service,
                                             twicEntry_t * const entry,
                                             const uint8_t properties,
                                             const uint64_t uuid_lsb,
                                             const uint64_t uuid_msb,
                                             const uint8_t uuid_len)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  twicUuid_t uuid;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(entry);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (!(service->handle) || entry->handle || entry->u.attr.b_handle) {
    twicTrace();
    ret = TWIC_STATUS_ERROR_RESOURCE;
    goto out;
  }
  
  uuid.len = uuid_len;
  twicSetUuid((uint8_t *)&uuid.uu, uuid_lsb, uuid_msb);
  
  entry->u.attr.properties = properties;
  ret = twicSsGattServerAddCharacteristics(
    TWIC_LEGATTDBADDCHARACTERISTICS, service->handle, entry, &uuid);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeGattSetCharacteristics(twicEntry_t * const entity,
                                             const bool db_init,
                                             const uint16_t permissions,
                                             const uint16_t length,
                                             const uint8_t * const value,
                                             const uint64_t uuid_lsb,
                                             const uint64_t uuid_msb,
                                             const uint8_t uuid_len)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  twicUuid_t uuid;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  if (entity->req)
    goto out;    
  
  if (!(entity->instance)) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  if (!(entity->handle)) {
    twicTrace();
    ret = TWIC_STATUS_REQUEST_HANDLE;
    goto out;
  }
  
  if (TWIC_GATT_ATTR_MAX_LEN < length) {
    twicTrace();
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  if (RPQUEUE_ENABLE != twic_eq.sniff.enable)
    goto out;
  
  uuid.len = uuid_len;
  twicSetUuid((uint8_t *)&uuid.uu, uuid_lsb, uuid_msb);
  
  entity->u.attr.permissions = permissions;
  entity->type = TWIC_ENTRY_TYPE_CHARACTERISTIC_VALUE;
  
  if (true == db_init)
    ret = twicSsGattServerSetElement(TWIC_LEGATTDBSETCHARACTERISTICS,
                                     entity, &uuid, length, value);
  else
    ret = twicSsGattServerSetElement(TWIC_LEGATTDBUPDCHARACTERISTICS,
                                     entity, &uuid, length, value);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
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
                                               const uint8_t uuid_len)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  twicUuid_t uuid;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  if (entity->req)
    goto out;    
  
  if (!(entity->instance)) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  if (!(entity->handle)) {
    twicTrace();
    ret = TWIC_STATUS_REQUEST_HANDLE;
    goto out;
  }
  
  if (TWIC_GATT_ATTR_MAX_LEN < length || TWIC_GATT_ATTR_MAX_LEN < max_length) {
    twicTrace();
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  if (RPQUEUE_ENABLE != twic_eq.sniff.enable)
    goto out;
  
  uuid.len = uuid_len;
  twicSetUuid((uint8_t *)&uuid.uu, uuid_lsb, uuid_msb);
  
  entity->u.attr.permissions = permissions;
  entity->type = TWIC_ENTRY_TYPE_CHARACTERISTIC_VALUE;
  
  if (true == db_init)
    ret = twicSsGattServerSetElementVl(TWIC_LEGATTDBSETCHARACTERISTICSVL,
                                       entity, &uuid, max_length, length,
                                       value);
  else
    ret = twicSsGattServerSetElementVl(TWIC_LEGATTDBUPDCHARACTERISTICSVL,
                                       entity, &uuid, max_length, length,
                                       value);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
/* twicIfGattAddDescriptor -> twicIfGattSetDescriptor */
twicStatus_t _twicIfLeGattSetDescriptor(const twicEntry_t * const entity,
                                        twicEntry_t * const entry,
                                        const uint16_t permissions,
                                        const uint16_t length,
                                        const uint8_t * const value,
                                        const uint64_t uuid_lsb,
                                        const uint64_t uuid_msb,
                                        const uint8_t uuid_len)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  twicUuid_t uuid;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(entry);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (TWIC_GATT_ATTR_MAX_LEN < length) {
    twicTrace();
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  uuid.len = uuid_len;
  twicSetUuid((uint8_t *)&uuid.uu, uuid_lsb, uuid_msb);
  
  entry->u.attr.permissions = permissions;
  
  if (!(entity->handle)) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    twicTrace();
    goto out;
  }
  
  /* Substantiation. */
  entry->handle = entity->handle;
  entry->u.attr.p_handle = entity->handle;
  entry->type = TWIC_ENTRY_TYPE_CHARACTERISTIC_DESCRIPTOR;
  
  ret = twicSsGattServerSetElement(TWIC_LEGATTDBSETDESCRIPTOR, entry, &uuid,
                                   length, value);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#if defined(TWIC_LEGATTDBSETDESCRIPTORVL)
/* twicIfGattAddDescriptor -> twicIfGattSetDescriptor */
twicStatus_t _twicIfLeGattSetDescriptorVl(const twicEntry_t * const entity,
                                          twicEntry_t * const entry,
                                          const uint16_t permissions,
                                          const uint16_t max_length,
                                          const uint16_t length,
                                          const uint8_t * const value,
                                          const uint64_t uuid_lsb,
                                          const uint64_t uuid_msb,
                                          const uint8_t uuid_len)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  twicUuid_t uuid;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (TWIC_GATT_ATTR_MAX_LEN < length || TWIC_GATT_ATTR_MAX_LEN < max_length) {
    twicTrace();
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  uuid.len = uuid_len;
  twicSetUuid((uint8_t *)&uuid.uu, uuid_lsb, uuid_msb);
  
  entry->u.attr.permissions = permissions;
  
  if (!(entity->handle)) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    twicTrace();
    goto out;
  }
  
  /* Substantiation. */
  entry->handle = entity->handle;
  entry->u.attr.p_handle = entity->handle;
  entry->type = TWIC_ENTRY_TYPE_CHARACTERISTIC_DESCRIPTOR;
  
  ret = twicSsGattServerSetElementVl(TWIC_LEGATTDBSETDESCRIPTORVL, entry,
                                     &uuid, max_length, length, value);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
twicStatus_t _twicIfLeEndServiceCreation(twicEntry_t * const service)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  if (service->req)
    goto out;    
  
  if (!(service->instance)) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  if (!(service->handle)) {
    twicTrace();
    ret = TWIC_STATUS_ERROR_RESOURCE;
    goto out;
  }
  
  if (RPQUEUE_ENABLE != twic_eq.sniff.enable)
    goto out;
  
  ret = twicSsGattServerPackService(TWIC_LEGATTDBENDSERVICECREATION, service);

  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

#if defined(TWIC_API_LEGATTDBSETPERMISSIONS)
twicStatus_t _twicIfLeGattDbSetPermissions(twicEntry_t * const entity,
                                           const uint16_t permissions)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (entity->req)
    goto out;    

  if (!(entity->instance)) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  if ((TWIC_ENTRY_TYPE_CHARACTERISTIC_VALUE != entity->type &&
       TWIC_ENTRY_TYPE_CHARACTERISTIC_DESCRIPTOR != entity->type) ||
      !(entity->u.attr.b_handle)) {
    twicTrace();
    ret = TWIC_STATUS_REQUEST_HANDLE;
    goto out;
  }

  if (RPQUEUE_ENABLE != twic_eq.sniff.enable)
    goto out;

  ret = twicSsGattDbSetPermissions(TWIC_LEGATTDBSETPERMISSIONS, entity,
                                   permissions);
  out:

  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_API_LEGATTDBGETPERMISSIONS)
twicStatus_t _twicIfLeGattDbGetPermissions(twicEntry_t * const entity)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }

  if (entity->req)
    goto out;    

  if (!(entity->instance)) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  if ((TWIC_ENTRY_TYPE_CHARACTERISTIC_VALUE != entity->type &&
       TWIC_ENTRY_TYPE_CHARACTERISTIC_DESCRIPTOR != entity->type) ||
      !(entity->u.attr.b_handle)) {
    twicTrace();
    ret = TWIC_STATUS_REQUEST_HANDLE;
    goto out;
  }

  if (RPQUEUE_ENABLE != twic_eq.sniff.enable)
    goto out;

  ret = twicSsGattDbGetPermissions(TWIC_LEGATTDBGETPERMISSIONS, entity);
  out:

  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif
#if defined(TWIC_API_GATTCLIENTEXGMTU)
twicStatus_t
_twicIfGattClientExgMtu(twicEntry_t * const conn, const uint16_t rx_mtu_size)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;

  if (TWIC_ATT_MTU_MAX < rx_mtu_size || TWIC_ATT_MTU > rx_mtu_size) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  ret = twicSsTreToReq(TWIC_GATTCLIENTEXGMTU, conn,
                       0x01, TWIC_SERVICE_ID_GATT_CLIENT, rx_mtu_size);

  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICE) ||             \
  defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID) ||  \
  defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE) ||                  \
  defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS) ||           \
  defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)||         \
  defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS)
twicStatus_t _twicIfLeGattClientDiscovery(twicEntry_t * const conn,
                                          const uint8_t seq,
                                          const uint16_t start_handle,
                                          const uint16_t end_handle,
                                          const uint64_t uuid_lsb,
                                          const uint64_t uuid_msb,
                                          const uint8_t uuid_len)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  twicUuid_t uuid;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (0 == start_handle || 0 == end_handle) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  if (TWIC_UUID16 == uuid_len || TWIC_UUID128 == uuid_len) {
    uuid.len = uuid_len;
    twicSetUuid((uint8_t *)&uuid.uu, uuid_lsb, uuid_msb);
  } else {
    uuid.len = 0;
  }
  
  switch (seq) {
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICE)
  case TWIC_GATTCLIENTDISCOVERPRIMARYSERVICE:
    ret = twicSsFyraToReq(
      seq, conn, 0x02, TWIC_SERVICE_ID_GATT_CLIENT, start_handle, end_handle);
    break;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID)
  case TWIC_GATTCLIENTDISCOVERPRIMARYSERVICEBYSERVICEUUID:
    if (0 == uuid.len) {
      ret = TWIC_STATUS_ERROR_PARAMETER;
      goto out;
    }
    ret = twicSsGattClientDiscoveryByUuid(
      seq, conn, 0x03, start_handle, end_handle, &uuid);
    break;
#endif    
#if defined(TWIC_API_GATTCLIENTFINDINCLUDEDSERVICE)
  case TWIC_GATTCLIENTFINDINCLUDEDSERVICE:
    ret = twicSsFyraToReq(
      seq, conn, 0x04, TWIC_SERVICE_ID_GATT_CLIENT, start_handle, end_handle);
    break;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLCHARACTERISTICS)
  case TWIC_GATTCLIENTDISCOVERALLCHARACTERISTICS:
    ret = twicSsFyraToReq(
      seq, conn, 0x05, TWIC_SERVICE_ID_GATT_CLIENT, start_handle, end_handle);
    break;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID)
  case TWIC_GATTCLIENTDISCOVERCHARACTERISTICSBYUUID:
    ret = twicSsGattClientDiscoveryByUuid(
      seq, conn, 0x06, start_handle, end_handle, &uuid);
    break;
#endif
#if defined(TWIC_API_GATTCLIENTDISCOVERALLDESCRIPTORS)
  case TWIC_GATTCLIENTDISCOVERALLDESCRIPTORS:
    ret = twicSsFyraToReq(seq, conn, 0x07, TWIC_SERVICE_ID_GATT_CLIENT,
                          start_handle, end_handle); break;
#endif
  default:
    ret = TWIC_STATUS_ERROR_PARAMETER;
    break;
  }
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE) ||    \
  defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID) ||  \
  defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR)
twicStatus_t _twicIfLeGattClientRead(twicEntry_t * const conn,
                                     const uint8_t seq,
                                     const uint16_t start_handle,
                                     const uint16_t end_handle,
                                     const uint64_t uuid_lsb,
                                     const uint64_t uuid_msb,
                                     const uint8_t uuid_len)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  twicUuid_t uuid;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (0 == start_handle) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  if (TWIC_UUID16 == uuid_len || TWIC_UUID128 == uuid_len) {
    uuid.len = uuid_len;
    twicSetUuid((uint8_t *)&uuid.uu, uuid_lsb, uuid_msb);
  } else {
    uuid.len = 0;
  }
  
  switch (seq) {
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICVALUE)
  case TWIC_GATTCLIENTREADCHARACTERISTICVALUE:
    ret = twicSsTreToReq(
      seq, conn, 0x08, TWIC_SERVICE_ID_GATT_CLIENT, start_handle);
    break;
#endif
#if defined(TWIC_API_GATTCLIENTREADUSINGCHARACTERISTICUUID)
  case TWIC_GATTCLIENTREADUSINGCHARACTERISTICUUID:
    if (0 == uuid.len || 0 == end_handle) {
      ret = TWIC_STATUS_ERROR_PARAMETER;
      goto out;
    }
    ret = twicSsGattClientAccessByUuid(
      seq, conn, 0x0d, start_handle, end_handle, &uuid);
    break;
#endif
#if defined(TWIC_API_GATTCLIENTREADCHARACTERISTICDESCRIPTOR)
  case TWIC_GATTCLIENTREADCHARACTERISTICDESCRIPTOR:
    ret = twicSsTreToReq(
      seq, conn, 0x0A, TWIC_SERVICE_ID_GATT_CLIENT, start_handle);
    break;
#endif
  default:
    ret = TWIC_STATUS_ERROR_PARAMETER;
    break;
  }
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICVALUE) ||    \
  defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR) || \
  defined(TWIC_API_GATTCLIENTWRITEWITHOUTRESPONSE)
twicStatus_t
_twicIfLeGattClientWrite(twicEntry_t * const conn, const uint8_t seq,
                         const uint16_t handle, const uint8_t length,
                         const uint8_t * const data)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (0 == handle) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  switch (seq) {
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICVALUE)
  case TWIC_GATTCLIENTWRITECHARACTERISTICVALUE:
    ret = twicSsGattTransmit(seq, conn, 0x09, TWIC_SERVICE_ID_GATT_CLIENT,
                             handle, length, data);
    break;
#endif
#if defined(TWIC_API_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR)
  case TWIC_GATTCLIENTWRITECHARACTERISTICDESCRIPTOR:
    ret = twicSsGattTransmit(seq, conn, 0x0B, TWIC_SERVICE_ID_GATT_CLIENT,
                             handle, length, data);
    break;
#endif
#if defined(TWIC_API_GATTCLIENTWRITEWITHOUTRESPONSE)
  case TWIC_GATTCLIENTWRITEWITHOUTRESPONSE:
    ret = twicSsGattTransmit(seq, conn, 0x0F, TWIC_SERVICE_ID_GATT_CLIENT,
                             handle, length, data);
    break;
#endif
  default:
    ret = TWIC_STATUS_ERROR_PARAMETER;
    break;
  }
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_GATTCLIENTSIGNEDWRITEWITHOUTRESPONSE)
twicStatus_t _twicIfLeGattClientSignedWrite(twicEntry_t * const conn,
                                            const uint16_t handle,
                                            const uint8_t length,
                                            const uint8_t * const data)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (0 == handle) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  ret = twicSsGattSignedWrite(
    TWIC_GATTCLIENTSIGNEDWRITEWITHOUTRESPONSE,
    conn, 0x11, TWIC_SERVICE_ID_GATT_CLIENT, handle, length, data);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_GATTCLIENTREADMULTIPLECHARVALUES)
twicStatus_t
_twicIfLeGattClientReadMultipleCharValues(twicEntry_t * const conn,
                                          const uint8_t number,
                                          const uint16_t * const handles)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (2 > number || NULL == handles) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  ret = twicSsClientReadMultipleCharValues(
    TWIC_GATTCLIENTREADMULTIPLECHARVALUES, conn, number, handles);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_GATTCLIENTRELIABLEWRITE)
twicStatus_t
_twicIfLeGattClientReliableWrite(
  twicEntry_t * const conn, const uint16_t total,
  const twicReliableWritein_t * const characteristics_value)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  uint16_t len, i;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;

  if (0 == total || TWIC_RESP_D24E_TOTAL_HANDLES < total) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  len = 2; /* conn_handle */
  for (i = 0; i < total; i++) {
    /* value handle, offset, length and value size */
    len += 6 + characteristics_value[total].length;
    if (TWIC_GATT_ATTR_MAX_LEN < len ||
        0 == characteristics_value[total].handle ||
        0 == characteristics_value[total].length) {
      ret = TWIC_STATUS_ERROR_PARAMETER;
      goto out;
    }
  }
  
  ret = twicSsClientReliableWrite(
    TWIC_GATTCLIENTRELIABLEWRITE, conn, len, total, characteristics_value);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE) ||  \
  defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR)
twicStatus_t _twicIfLeGattClientLongRead(twicEntry_t * const conn,
                                         const uint8_t seq,
                                         const uint16_t handle,
                                         const uint16_t offset)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (0 == handle) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  switch (seq) {
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICVALUE)
  case TWIC_GATTCLIENTREADLONGCHARACTERISTICVALUE:
    ret = twicSsFyraToReq(
      seq, conn, 0x12, TWIC_SERVICE_ID_GATT_CLIENT, handle, offset);
    break;
#endif
#if defined(TWIC_API_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR)
  case TWIC_GATTCLIENTREADLONGCHARACTERISTICDESCRIPTOR:
    ret = twicSsFyraToReq(
      seq, conn, 0x13, TWIC_SERVICE_ID_GATT_CLIENT, handle, offset);
    break;
#endif
  default:
    ret = TWIC_STATUS_ERROR_PARAMETER;
    break;
  }
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICVALUE) || \
  defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR)
twicStatus_t _twicIfLeGattClientLongWrite(twicEntry_t * const conn,
                                          const uint8_t seq,
                                          const uint16_t handle,
                                          const uint16_t offset,
                                          const uint16_t length,
                                          const uint8_t * const value)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (0 == handle) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  switch (seq) {
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICVALUE)
  case TWIC_GATTCLIENTWRITELONGCHARACTERISTICVALUE:
    ret = twicSsClientLongWrite(seq, conn, 20, length, handle, offset, value);
    break;
#endif    
#if defined(TWIC_API_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR)
  case TWIC_GATTCLIENTWRITELONGCHARACTERISTICDESCRIPTOR:
    ret = twicSsClientLongWrite(seq, conn, 21, length, handle, offset, value);
    break;
#endif
  default:
    ret = TWIC_STATUS_ERROR_PARAMETER;
    break;
  }

  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

#if defined(TWIC_API_GATTCLIENTINDICATIONCONFIRMATIONRESPONSE)
twicStatus_t _twicIfLeGattClientResponse(twicEntry_t * const conn,
                                         const uint8_t seq,
                                         const uint8_t status)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  switch (seq) {
  case TWIC_GATTCLIENTINDICATIONCONFIRMATIONRESPONSE:
    ret = twicSsToReq(seq, conn, 0x10, TWIC_SERVICE_ID_GATT_CLIENT);
    break;
  default:
    ret = TWIC_STATUS_ERROR_PARAMETER;
    break;
  }
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}
#endif

twicStatus_t _twicIfLeDiscoverable(twicEntry_t * const connection,
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
                                   const uint8_t * const scan_resp_data)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  if (connection->req)
    goto out;    
  
  ret = TWIC_STATUS_ERROR_PARAMETER;
  if (!(connection->instance)) goto out;
  if (0x1F < advertising_data_length || 0x1F < scan_resp_data_length) goto out;
  if (0x0020 > min_interval || 0x4000 < min_interval || /* N x 0.625ms */
      0x0020 > max_interval || 0x4000 < max_interval) goto out;
#if defined(TWIC_BLE_HWIP_V41)
  if (TWIC_ADV_TYPE_DIRECT_IND_LDC < advertising_type) goto out;
#else
  if (TWIC_ADV_TYPE_NONCONN_IND < advertising_type) goto out;
#endif
  if (TWIC_ADDR_TYPE_RANDOM_DEVICE_ADDRESS < own_address_type) goto out;
  if (TWIC_ADDR_TYPE_RANDOM_DEVICE_ADDRESS < direct_address_type) goto out;
  if (!(advertising_channel_map)) goto out;
  if (TWIC_ADV_FILTER_WHITE_WHITE < advertising_filter_policy) goto out;

  if ((TWIC_ADV_TYPE_NONCONN_IND == advertising_type ||
       TWIC_ADV_TYPE_SCAN_IND == advertising_type) && 
      (0x00A0 > min_interval || 0x00A0 > max_interval)) goto out;

  ret = TWIC_STATUS_UNDER_PROCESSING;
  if (RPQUEUE_ENABLE != twic_eq.sniff.enable)
    goto out;
  
  ret = twicSsLeDiscoverable(TWIC_LEDISCOVERABLE, connection, min_interval,
                             max_interval, advertising_type, own_address_type,
                             direct_address_type, &direct_address,
                             advertising_channel_map,
                             advertising_filter_policy,
                             advertising_data_length, advertising_data,
                             scan_resp_data_length, scan_resp_data);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfLeStopAdvertising(twicEntry_t * const conn)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  ret = twicSsReq(TWIC_LESTOPADVERTISING, conn,
                  0x09, TWIC_SERVICE_ID_CONNECTION_MANAGEMENT, NULL, 0);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfGattServerExgMtuResponse(twicEntry_t * const conn,
                                             const uint8_t status,
                                             const uint16_t rx_mtu_size)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (TWIC_ATT_MTU_MAX < rx_mtu_size || TWIC_ATT_MTU > rx_mtu_size) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  ret = twicSsFemReq(TWIC_GATTSERVEREXGMTURESPONSE, conn, 0x01,
                     TWIC_SERVICE_ID_GATT_SERVER, status, rx_mtu_size);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfGattServerCharValMultiReadOutResponse(twicEntry_t *const conn,
                                             const uint8_t status,
                                             const twicEntry_t *const cha)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (!(status))
    ret = twicSsFemReq(TWIC_GATTSERVERCHARVALMULTIREADOUTRESPONSE, conn,
                       0x0a, TWIC_SERVICE_ID_GATT_SERVER, 0, 0);
  else {
    if (!(cha) || !(cha->instance)) {
      ret = TWIC_STATUS_ERROR_PARAMETER;
      goto out;
    }
    ret = twicSsFemReq(TWIC_GATTSERVERCHARVALMULTIREADOUTRESPONSE, conn,
                       0x0a, TWIC_SERVICE_ID_GATT_SERVER, status,
                       cha->u.attr.b_handle);
  }

  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfGattServerAttResponse(twicEntry_t * const conn, const uint8_t seq,
                             const twicEntry_t * const entity,
                             const uint8_t status)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (!(entity->instance)) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    twicTrace();
    goto out;
  }
  
  switch (seq) {
  case TWIC_GATTSERVERCHARDESPWRITEINRESPONSE:
    ret = twicSsFemReq(/* twicIfGattServerCharDespWriteInResponse */
      seq, conn, 0x04, TWIC_SERVICE_ID_GATT_SERVER,
      status, entity->u.attr.b_handle);
    break;
  case TWIC_GATTSERVERCHARDESPREADOUTRESPONSE:
    ret = twicSsFemReq(/* twicIfGattServerCharDespReadOutResponse */
      seq, conn, 0x08, TWIC_SERVICE_ID_GATT_SERVER,
      status, entity->u.attr.b_handle);
    break;
  case TWIC_GATTSERVERCHARVALREADOUTRESPONSE:
    ret = twicSsFemReq(/* twicIfGattServerCharValReadOutResponse */
      seq, conn, 0x02, TWIC_SERVICE_ID_GATT_SERVER,
      status, entity->u.attr.b_handle);
    break;
  case TWIC_GATTSERVERCHARVALWRITEINRESPONSE:
    ret = twicSsFemReq(/* twicIfGattServerCharValWriteInResponse */
      seq, conn, 0x03, TWIC_SERVICE_ID_GATT_SERVER,
      status, entity->u.attr.b_handle);
    break;
#if defined(TWIC_API_GATTSERVERLONGCHARVALREADOUTRESPONSE)
  case TWIC_GATTSERVERLONGCHARVALREADOUTRESPONSE:
    ret = twicSsFemReq(/* twicIfGattServerLongCharValReadOutResponse */
      seq, conn, 0x0d, TWIC_SERVICE_ID_GATT_SERVER,
      status, entity->u.attr.b_handle);
    break;
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARDESPREADOUTRESPONSE)
  case TWIC_GATTSERVERLONGCHARDESPREADOUTRESPONSE:
    ret = twicSsFemReq(/* twicIfGattServerLongCharDespReadOutResponse */
      seq, conn, 0x0e, TWIC_SERVICE_ID_GATT_SERVER,
      status, entity->u.attr.b_handle);
    break;
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARVALPREPAREWRITEINRESPONSE)
  case TWIC_GATTSERVERLONGCHARVALPREPAREWRITEINRESPONSE:
    ret = twicSsFemReq(/* twicIfGattServerLongCharValPrepareWriteInResponse */
      seq, conn, 0x0b, TWIC_SERVICE_ID_GATT_SERVER,
      status, entity->u.attr.b_handle);
    break;
#endif
#if defined(TWIC_API_GATTSERVERLONGCHARDESPPREPAREWRITEINRESPONSE)
  case TWIC_GATTSERVERLONGCHARDESPPREPAREWRITEINRESPONSE:
    ret = twicSsFemReq(/* twicIfGattServerLongCharDespPrepareWriteInResponse */
      seq, conn, 0x0c, TWIC_SERVICE_ID_GATT_SERVER,
      status, entity->u.attr.b_handle);
    break;
#endif
#if defined(TWIC_API_GATTSERVEREXECCHARVALWRITEINRESPONSE)
  case TWIC_GATTSERVEREXECCHARVALWRITEINRESPONSE:
    ret = twicSsFemReq(/* twicIfGattServerExecCharValWriteInResponse */
      seq, conn, 0x0f, TWIC_SERVICE_ID_GATT_SERVER,
      status, entity->u.attr.b_handle);
    break;
#endif
#if defined(TWIC_API_GATTSERVEREXECCHARDESPWRITEINRESPONSE)
  case TWIC_GATTSERVEREXECCHARDESPWRITEINRESPONSE:
    ret = twicSsFemReq(/* twicIfGattServerExecCharDespWriteInResponse */
      seq, conn, 0x07, TWIC_SERVICE_ID_GATT_SERVER,
      status, entity->u.attr.b_handle);
    break;
#endif
  default:
    ret = TWIC_STATUS_ERROR_PARAMETER;
    break;
  }
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfGattNotification(twicEntry_t * const conn,
                        const twicEntry_t * const cha,
                        const uint8_t length, const uint8_t * const data)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (!(cha->instance) || 0 == length) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }

  ret = twicSsGattTransmit(TWIC_GATTNOTIFICATION, conn, 0x05,
                           TWIC_SERVICE_ID_GATT_SERVER, cha->u.attr.b_handle,
                           length, data);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfGattIndication(twicEntry_t * const conn, const twicEntry_t * const cha,
                      const uint8_t length, const uint8_t * const data)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if (!(cha->instance) || 0 == length) {
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  ret = twicSsGattTransmit(TWIC_GATTINDICATION, conn, 0x06,
                           TWIC_SERVICE_ID_GATT_SERVER, cha->u.attr.b_handle,
                           length, data);
  
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t
_twicIfGattServerWriteElement(const uint8_t seq, twicEntry_t * const conn,
                              const twicEntry_t * const entity,
                              const uint16_t length,
                              const uint8_t * const value)
{
  twicStatus_t ret = TWIC_STATUS_UNDER_PROCESSING;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicIfKnock(conn);
  if (TWIC_STATUS_OK != ret) goto out;
  
  if ((TWIC_ENTRY_TYPE_CHARACTERISTIC_VALUE != entity->type &&
       TWIC_ENTRY_TYPE_CHARACTERISTIC_DESCRIPTOR != entity->type) ||
      !(entity->u.attr.b_handle)) {
    twicTrace();
    ret = TWIC_STATUS_REQUEST_HANDLE;
    goto out;
  }
  
  if (TWIC_GATT_ATTR_MAX_LEN < length) {
    twicTrace();
    ret = TWIC_STATUS_ERROR_PARAMETER;
    goto out;
  }
  
  ret = twicSsGattServerWriteElement(seq, conn, entity->u.attr.b_handle,
                                     length, value);
  out:
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t twicIfPeekEvent(void)
{
  uint8_t data;
  uint16_t count;
  
  do {
    if (TZ1SM_HAL_STATUS_EVENT_MESSAGE != tz1smHalUartPeekData(&data, &count))
      return TWIC_STATUS_OK;
    if (TWIC_TCU == twic_eq.type) {
      if (TWIC_STATUS_EVENT_MESSAGE == twicSsDataConstructionCm(data))
        return TWIC_STATUS_EVENT_MESSAGE;
    } else {
      if (TWIC_STATUS_EVENT_MESSAGE == twicSsDataConstructionHc(data))
        return TWIC_STATUS_EVENT_MESSAGE;
    }
  } while (count);
  
  return TWIC_STATUS_OK;
}

static TZ1K_INLINE twicStatus_t twicIfPumpCompleteDataCm(void)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicSsPumpCompleteDataCm();
  
#if defined(TWIC_LECE_LOWPOWER)
  twicSsLeRequestLowPower();
#endif
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

static TZ1K_INLINE twicStatus_t twicIfPumpCompleteDataHc(void)
{
  twicStatus_t ret;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = twicSsPumpCompleteDataHc();
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t twicIfDoEvents(void)
{
  twicStatus_t ret = TWIC_STATUS_OK;
  
  if (tz1smHalMutexWait(twic_cb_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  if (TWIC_TCU == twic_eq.type) ret = twicIfPumpCompleteDataCm();
  else ret = twicIfPumpCompleteDataHc();

  if (tz1smHalMutexRelease(twic_cb_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

/* @brief
 * Register a interface to make use of the GATT.
 *
 * This API registers the interface to distinguish each connection.
 * The "twicIfGattDeregistration" must be invoked before the
 * "twicIfLeIoFinalize".
 *
 * @param uint8_t * const interface
 * The pointer to hold the obtained interface index. */
twicStatus_t twicIfGattRegistration(uint8_t * const interface)
{
  const volatile unsigned long *addr = twic_interface_map;
  twicStatus_t ret = TWIC_STATUS_ERROR_PARAMETER;
  uint8_t nr;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  for (nr = 0; TWIC_INTERFACE_MAX > nr; nr ++) {
    unsigned long mask = TWIC_BIT_MASK(nr);
    unsigned long *p = ((unsigned long *)addr) + TWIC_BIT_WORD(nr);
    
    if (1UL & (addr[TWIC_BIT_WORD(nr)] >> (nr & TWIC_BITS_PER_LONG - 1))) {
      continue;
    }

    *interface = TWIC_NR_IIDX(nr);
    ret = TWIC_STATUS_OK;
    *p |= mask;
    break;
  }
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

twicStatus_t _twicIfGattDeregistration(const uint8_t interface)
{
  const volatile unsigned long *addr = twic_interface_map;
  twicStatus_t ret = TWIC_STATUS_ERROR_PARAMETER;
  uint8_t nr = TWIC_IIDX_NR(interface);
  
  if (TWIC_INTERFACE_MAX < interface || 0 == interface) {
    return ret;
  }
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  if (1UL & (addr[TWIC_BIT_WORD(nr)] >> (nr & TWIC_BITS_PER_LONG - 1))) {
    unsigned long mask = TWIC_BIT_MASK(nr);
    unsigned long *a = ((unsigned long *)addr) + TWIC_BIT_WORD(nr);
    twicEntry_t *p, *q = NULL;
    
    *a &= ~mask;
    ret = TWIC_STATUS_OK;
    tz1utListEach(twicEntry_t, p, TZ1UT_LIST(srv_req_root), o_link) {
      if (p->interface == interface) {
        if (q) tz1utListInit(&q->o_link);
        tz1utListDel(&p->o_link);
        q = p;
      }
    }
    if (q) tz1utListInit(&q->o_link);
    twicSsGattDeregistration(interface);
  }
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

/* @brief Deregister the interface.
 * This API deregisters the interface which was created by
 * "twicIfGattRegistration".
 *
 * @param const uint8_t interface
 * The interface index. */
twicStatus_t twicIfGattDeregistration(const uint8_t interface)
{
  twicStatus_t ret = TWIC_STATUS_ERROR_RESOURCE;
  
  if (tz1smHalMutexWait(twic_cb_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  ret = _twicIfGattDeregistration(interface);
  
  if (tz1smHalMutexRelease(twic_cb_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

static twicStatus_t
_twicIfLeRegisterCallback(twicEntry_t * const conn,
                          const twicLeServerCb_t *server_callbacks,
                          const twicLeClientCb_t *client_callbacks,
#if defined(TWIC_CONFIG_SM_INITIATOR)
                          const twicLeSmpICb_t *smp_i_callbacks,
#endif
#if defined(TWIC_CONFIG_SM_RESPONDER)
                          const twicLeSmpRCb_t *smp_r_callbacks,
#endif
                          const twicLeCb_t *le_callbacks)
{
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  conn->instance->server_cb = server_callbacks;
  conn->instance->client_cb = client_callbacks;
#if defined(TWIC_CONFIG_SM_INITIATOR)
  conn->instance->smp_i_cb = smp_i_callbacks;
#else
  conn->instance->smp_i_cb = NULL;
#endif  
#if defined(TWIC_CONFIG_SM_RESPONDER)
  conn->instance->smp_r_cb = smp_r_callbacks;
#else
  conn->instance->smp_r_cb = NULL;
#endif  
  conn->instance->le_cb = le_callbacks;
  twicSsGattRegistration(conn);
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return TWIC_STATUS_OK;
}

/* Reference materials:
 * BLUETOOTH SPECIFICATION Version 4.0 [Vol 1].
 * Architecture.
 *
 * @brief
 * Register the callback functions.
 * 
 * This API registers the callback functions of the GATT Client,
 * Server, SMP Slave, Master and Common Feature.
 *
 * The Common Feature includes Link Manager and some basic events of
 * the LE HCI.  Please refer to the "Reference materials:Figure 2.1:
 * Bluetooth core system architecture" for information about the Link
 * Manager and the architecture of the SMP and GATT.
 *
 * The usage of each callback function is explained in the API as follows:
 *
 * " The 'le_callbacks->notification_sent' which is registered by the
 * 'twicIfLeRegisterCallback' API will be invoked from the
 * 'twicIfDoEvents' API when the Characteristic Value Notification has
 * been sent to the GATT Client. "
 *
 * If the GATT Client is not enabled, the "client_callback" argument
 * can be set with NULL.  Please refer to the "twicUtLeCeInit3" API to
 * enable the GATT Client.  If the GATT Client is enabled and the GATT
 * Notification is received, the "notification_received" callback
 * function shall be invoked.
 *
 * void (*notification_received)(
 *   const void * const connif, const uint8_t status,
 *   const uint16_t char_value_handle, const twicAttValue_t *const resp);
 *
 * If an application does not use the SMP, the
 * "TWIC_CONFIG_SM_INITIATOR" and "TWIC_CONFIG_SM_RESPONDER" can be
 * undefined. In that case, the arguments for the parameter
 * "smp_[ir]_callbacks" shall be set with NULL.
 *  
 * The "TWIC_CONFIG_SM_INITIATOR" and "TWIC_CONFIG_SM_RESPONDER" are
 * defined in the "tz1sm_config.h".
 *
 * @param twicConnIface_t * const iface
 * The pointer of the element of the GATT's resource to be used by
 * this interface.
 * @param const twicIfLeServerCb_t *server_callbacks
 * The pointer of the Callback Functions for the GATT Server.
 * @param  const twicIfLeClientCb_t *client_callbacks
 * The pointer of the Callback Functions for the GATT Client.
 * @param const twicIfLeSmpICb_t *smp_i_callbacks
 * The pointer of the Callback Functions for the SMP Initiator.
 * @param const twicIfLeSmpICb_t *smp_r_callbacks
 * The pointer of the Callback Functions for the SMP Responder.
 * @param const twicIfLeCb_t *le_callbacks
 * The pointer of the Callback Functions for the Common Feature. */
twicStatus_t
twicIfLeRegisterCallback(twicConnIface_t * const iface,
                         const twicIfLeServerCb_t *server_callbacks,
                         const twicIfLeClientCb_t *client_callbacks,
#if defined(TWIC_CONFIG_SM_INITIATOR)
                         const twicIfLeSmpICb_t *smp_i_callbacks,
#else
                         const void *smp_i_callbacks,
#endif  
#if defined(TWIC_CONFIG_SM_RESPONDER)
                         const twicIfLeSmpRCb_t *smp_r_callbacks,
#else  
                         const void *smp_r_callbacks,
#endif
                         const twicIfLeCb_t *le_callbacks)
{
  twicStatus_t ret = TWIC_STATUS_ERROR_RESOURCE;
  const twicLeServerCb_t *scb = (const twicLeServerCb_t *)server_callbacks;
  const twicLeClientCb_t *ccb = (const twicLeClientCb_t *)client_callbacks;
#if defined(TWIC_CONFIG_SM_INITIATOR)
  const twicLeSmpICb_t *smp_i_cb = (const twicLeSmpICb_t *)smp_i_callbacks;
#endif  
#if defined(TWIC_CONFIG_SM_RESPONDER)
  const twicLeSmpRCb_t *smp_r_cb = (const twicLeSmpRCb_t *)smp_r_callbacks;
#endif  
  const twicLeCb_t *le_cb = (const twicLeCb_t *)le_callbacks;
  
  if (tz1smHalMutexWait(twic_cb_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  if (!(iface->conn.instance)) {
    return TWIC_STATUS_ERROR_PARAMETER;
  }

  if (scb && !(scb->mtu_exchange_demand
               && scb->char_val_multi_readout_demand &&
               scb->char_desp_writein_demand
               && scb->char_desp_readout_demand &&
               scb->char_val_readout_demand && scb->char_val_writein_post &&
               scb->mtu_exchange_result && scb->char_val_writein_demand)) {
    return TWIC_STATUS_ERROR_PARAMETER;
  }
  
  if (le_cb && !(le_cb->connection_complete && le_cb->connection_update
#if defined(TWIC_BLE_HWIP_V41) && !defined(TWIC_BLE_HWIP_V41_COMPAT_V40)
                 && le_cb->connection_parameter
#endif  
                 && le_cb->disconnection && le_cb->isr_downstream_request)) {
    return TWIC_STATUS_ERROR_PARAMETER;
  }
  
  if (ccb && !(ccb->mtu_exchange_result)) {
    return TWIC_STATUS_ERROR_PARAMETER;
  }
  
  ret = _twicIfLeRegisterCallback(
    &iface->conn, scb, ccb,
#if defined(TWIC_CONFIG_SM_INITIATOR)
    smp_i_cb,
#endif
#if defined(TWIC_CONFIG_SM_RESPONDER)
    smp_r_cb,
#endif
    le_cb);
  
  if (tz1smHalMutexRelease(twic_cb_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return ret;
}

/* For the result, please refer to twic_interface_cb.h
 * LIST OF ERROR CODES [2][3] */
twicStatus_t
twicIfAcceptance(twicConnIface_t * const iface, uint8_t * const result,
                 uint8_t (*const data)[23])
{
  twicEntry_t *entry = &iface->conn;
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  if (!(entry->rsp) || !(entry->req)) {
    return TWIC_STATUS_ERROR_PARAMETER;
  }
  
  *result = entry->result;
  if (data) memcpy(data, entry->u.cont.value, entry->u.cont.len);
  
  if (!(entry->result)) {
    /* record the invocation (req) if it's success */
    entry->rec = entry->req;
  } else {
    twicLog("0x%x/0x%x\r\n", *result, entry->rsp);
  }
  
  entry->rsp = 0xFF;
  entry->req = 0;
  entry->result = 0;
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return TWIC_STATUS_OK;
}

/* For the result, please refer to twic_interface_cb.h
 * LIST OF ERROR CODES [2][3] */
twicStatus_t twicIfDbAcceptance(twicConnIface_t * const iface,
                                const uint8_t eidx, uint8_t * const result)
{
  twicEntry_t *entry = TWIC_IF_TO_ENTRY(eidx);
  
  if (tz1smHalMutexWait(twic_rq_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  if (!(entry->rsp) || !(entry->req)) {
    return TWIC_STATUS_ERROR_PARAMETER;
  }
  
  *result = entry->result;
  
  if (!(entry->result)) {
    /* record the invocation (req) if it's success */
    entry->rec = entry->req;
  } else {
    twicLog("ERROR (result = 0x%x)\r\n", *result);
  }
  
  entry->rsp = 0xFF;
  entry->req = 0;
  entry->result = 0;
  
  if (tz1smHalMutexRelease(twic_rq_midx) != TZ1SM_HAL_STATUS_OK) {
#if defined(TWIC_IF_TRACE_MUTEX)
    twicTrace();
#endif    
    return TWIC_STATUS_ERROR_RESOURCE;
  }
  
  return TWIC_STATUS_OK;
}
