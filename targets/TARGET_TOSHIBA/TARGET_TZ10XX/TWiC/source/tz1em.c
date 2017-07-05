/**
 * @file tz1em.c
 * @brief Source Code file for TZ10xx TWiC TZ1EM (TZ1000 Series Energy
 * Management).
 * @version V0.0.2.FS (Free Sample - The information in this code is
 * subject to change without notice and should not be construed as a
 * commitment by TOSHIBA CORPORATION SEMICONDUCTOR & STORAGE PRODUCTS
 * COMPANY.
 * @note TZ1EM provides the automatic low energy consumption. The low
 * energy consumption of interal BLE Processor and UART2 is managed by
 * the part of TWIC BLE CE (BLE Controller Extension). Please refer to
 * the twicIfLeCe API group. TZ1EM combines the HOST Core low energy
 * consumption with the BLE CE and the other peripheral modules.
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#include "tz1sm_config.h"
#include "tz1em_service.h"

TZ1UT_LIST_DEF(tz1em_vf_ac_root);
TZ1UT_LIST_DEF(tz1em_vf_na_root);
TZ1SM_HAL_MUTEX_INIT(tz1em_mutex);
static tz1smHalMutexId_t tz1em_midx = 0;

tz1emStatus_t tz1emInitializeSystem(void)
{
  tz1emStatus_t status = TZ1EM_STATUS_OK;
  TZ1SM_HAL_INTR_STATUS_DEF;

  tz1em_midx = tz1smHalMutexCreate(TZ1SM_HAL_MUTEX(tz1em_mutex));
  if (!tz1em_midx) return TZ1EM_STATUS_ERROR_RESOURCE;

  if (tz1smHalMutexWait(tz1em_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  TZ1SM_HAL_IRQ_DISABLE_SAVE();
  status = tz1emSsInitializePcdSystem();

  TZ1SM_HAL_IRQ_ENABLE_RESTORE();
  if (tz1smHalMutexRelease(tz1em_midx) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  return status;
}

tz1emStatus_t tz1emStartClock(const tz1emPcd_t pcd)
{
  tz1emStatus_t status = TZ1EM_STATUS_OK;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (tz1smHalMutexWait(tz1em_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  TZ1SM_HAL_IRQ_DISABLE_SAVE();
  status = tz1emSsStartClock(pcd);

  TZ1SM_HAL_IRQ_ENABLE_RESTORE();
  if (tz1smHalMutexRelease(tz1em_midx) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  return status;
}

tz1emStatus_t tz1emStopClock(const tz1emPcd_t pcd)
{
  tz1emStatus_t status = TZ1EM_STATUS_OK;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (tz1smHalMutexWait(tz1em_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  TZ1SM_HAL_IRQ_DISABLE_SAVE();
  status = tz1emSsStopClock(pcd);

  TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  if (tz1smHalMutexRelease(tz1em_midx) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  return status;
}

tz1emStatus_t tz1emInitializeEntry(tz1em_t * const entry)
{
  tz1emStatus_t status = TZ1EM_STATUS_OK;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (tz1smHalMutexWait(tz1em_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  TZ1SM_HAL_IRQ_DISABLE_SAVE();

  if (true == entry->info.init) {
    status = TZ1EM_STATUS_ERROR_PARAMETER;
  } else {
    tz1utListInit(&entry->sunshine); tz1utListInit(&entry->shade);
    entry->info.init = true;
    entry->info.conf = false;
    entry->info.enable = false;
  }

  TZ1SM_HAL_IRQ_ENABLE_RESTORE();
  if (tz1smHalMutexRelease(tz1em_midx) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  return status;
}

tz1emStatus_t tz1emReconfigureEntry(
  tz1em_t * const entry, tz1emRequirement_t * const condition)
{
  tz1emStatus_t status = TZ1EM_STATUS_OK;
  uint8_t i;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (tz1smHalMutexWait(tz1em_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  TZ1SM_HAL_IRQ_DISABLE_SAVE();

  if (false == entry->info.init || false == entry->info.conf)
    status = TZ1EM_STATUS_ERROR_PARAMETER;
  else if (false == entry->info.enable)
    status = TZ1EM_STATUS_OPERATION_NOT_PERMITTED;
  else {
    entry->condition.pcd = condition->pcd; /* power control domain */
    entry->condition.sunshine_vf = condition->sunshine_vf;
    entry->condition.mode = condition->mode;
    entry->condition.permissible_time_lag_us
      = condition->permissible_time_lag_us;
    entry->condition.wakeup = condition->wakeup;

    for (i = 0; i < TZ1EM_NUM_OF_TRIGGER; i++) {
      if (TZ1EM_WF_RTC == condition->trigger[i].factor) {
        entry->condition.trigger[i].event = TZ1EM_WE_EP;
      }
      entry->condition.trigger[i].event = condition->trigger[i].event;
      entry->condition.trigger[i].factor = condition->trigger[i].factor;
    }

#if defined(TZ1EM_UPDATE_WORST_REQUIREMENT_CODE_PLACE_A)
    tz1emUpdateWorstRequirement(
      true, TZ1SM_HAL_VF_NONE, TZ1SM_HAL_OM_NONE, 0);
#endif
  }

  TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  if (tz1smHalMutexRelease(tz1em_midx) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  return status;
}

tz1emStatus_t tz1emConfigureEntry(
  tz1em_t * const entry, tz1emRequirement_t * const condition)
{
  tz1emStatus_t status = TZ1EM_STATUS_OK;
  uint8_t i;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (tz1smHalMutexWait(tz1em_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  TZ1SM_HAL_IRQ_DISABLE_SAVE();

  if (!(entry) || false == entry->info.init) {
    status = TZ1EM_STATUS_ERROR_PARAMETER;
  } else {

    if (!tz1utListIsEmpty(&entry->sunshine)) {
      tz1utListDel(&entry->sunshine); tz1utListInit(&entry->sunshine);
    }
    if (!tz1utListIsEmpty(&entry->shade)) {
      tz1utListDel(&entry->shade); tz1utListInit(&entry->shade);
    }

    entry->condition.pcd = condition->pcd; /* power control domain */
    entry->condition.sunshine_vf = condition->sunshine_vf;
    entry->condition.mode = condition->mode;
    entry->condition.permissible_time_lag_us
      = condition->permissible_time_lag_us;
    entry->condition.wakeup = condition->wakeup;

    for (i = 0; i < TZ1EM_NUM_OF_TRIGGER; i++) {
      if (TZ1EM_WF_RTC == condition->trigger[i].factor) {
        entry->condition.trigger[i].event = TZ1EM_WE_EP;
      }
      entry->condition.trigger[i].event = condition->trigger[i].event;
      entry->condition.trigger[i].factor = condition->trigger[i].factor;
    }

    entry->info.necessary_time_lag_us = TZ1EM_NECESSARY_TIME_LAG_US_INIT;
    entry->info.timer_expired = TZ1EM_INFO_INIT;
    entry->info.wakeup_reason = 0;
    entry->info.voltage = TZ1SM_HAL_VF_NONE;

    entry->info.conf = true;
    entry->info.enable = true;

#if defined(TZ1EM_UPDATE_WORST_REQUIREMENT_CODE_PLACE_A)
    tz1emUpdateWorstRequirement(
      true, TZ1SM_HAL_VF_NONE, TZ1SM_HAL_OM_NONE, 0);
#endif
  }

  TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  if (tz1smHalMutexRelease(tz1em_midx) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  return status;;
}

tz1emStatus_t tz1emWithdraw(tz1em_t * const entry)
{
  tz1emStatus_t status = TZ1EM_STATUS_OK;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (tz1smHalMutexWait(tz1em_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  TZ1SM_HAL_IRQ_DISABLE_SAVE();

  if (false == entry->info.init || false == entry->info.conf)
    status = TZ1EM_STATUS_ERROR_PARAMETER;
  else {
    if (!tz1utListIsEmpty(&entry->sunshine)) {
      tz1utListDel(&entry->sunshine); tz1utListInit(&entry->sunshine);
    }
    if (!tz1utListIsEmpty(&entry->shade)) {
      tz1utListDel(&entry->shade); tz1utListInit(&entry->shade);
#if defined(TZ1EM_UPDATE_WORST_REQUIREMENT_CODE_PLACE_A)
      tz1emUpdateWorstRequirement(
        true, TZ1SM_HAL_VF_NONE, TZ1SM_HAL_OM_NONE, 0);
#endif
    }
    entry->info.enable = false;
  }

  TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  if (tz1smHalMutexRelease(tz1em_midx) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  return status;
}

static tz1emStatus_t _tz1emTemporaryWithdraw(tz1em_t * const entry)
{
  if (false == entry->info.init || false == entry->info.conf) {
    return TZ1EM_STATUS_ERROR_PARAMETER;
  } else {
    if (!tz1utListIsEmpty(&entry->sunshine)) {
      tz1utListDel(&entry->sunshine); tz1utListInit(&entry->sunshine);
    }
    if (!tz1utListIsEmpty(&entry->shade)) {
      tz1utListDel(&entry->shade); tz1utListInit(&entry->shade);
#if defined(TZ1EM_UPDATE_WORST_REQUIREMENT_CODE_PLACE_A)
      tz1emUpdateWorstRequirement(true,
                                  TZ1SM_HAL_VF_NONE, TZ1SM_HAL_OM_NONE, 0);
#endif
    }
  }

  return TZ1EM_STATUS_OK;
}

tz1emStatus_t tz1emTemporaryWithdraw(tz1em_t * const entry)
{
  tz1emStatus_t status;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (tz1smHalMutexWait(tz1em_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  TZ1SM_HAL_IRQ_DISABLE_SAVE();

  status = _tz1emTemporaryWithdraw(entry);

  TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  if (tz1smHalMutexRelease(tz1em_midx) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  return status;
}

tz1emStatus_t tz1emIsrTemporaryWithdraw(tz1em_t * const entry)
{
  tz1emStatus_t status;
  uint32_t ipsr;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (0 == (ipsr = TZ1SM_HAL_IRQ_GET_IPSR())) TZ1SM_HAL_IRQ_DISABLE_SAVE();

  status = _tz1emTemporaryWithdraw(entry);

  if (ipsr == 0) TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  return status;
}

tz1emStatus_t tz1emPermit(tz1em_t * const entry)
{
  tz1emStatus_t status = TZ1EM_STATUS_OK;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (tz1smHalMutexWait(tz1em_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  TZ1SM_HAL_IRQ_DISABLE_SAVE();

  if (false == entry->info.init || false == entry->info.conf)
    status = TZ1EM_STATUS_ERROR_PARAMETER;
  else
    entry->info.enable = true;

  TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  if (tz1smHalMutexRelease(tz1em_midx) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  return status;
}

tz1emStatus_t tz1emParticipateIn(tz1em_t * const entry)
{
  tz1emStatus_t status = TZ1EM_STATUS_OK;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (tz1smHalMutexWait(tz1em_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  TZ1SM_HAL_IRQ_DISABLE_SAVE();

  if (false == entry->info.init || false == entry->info.conf)
    status = TZ1EM_STATUS_ERROR_PARAMETER;
  else if (false == entry->info.enable)
    status = TZ1EM_STATUS_OPERATION_NOT_PERMITTED;
  else {
    if (tz1utListIsEmpty(&entry->sunshine))
      tz1utListAdd(&entry->sunshine, TZ1UT_LIST(tz1em_vf_ac_root));
    if (tz1utListIsEmpty(&entry->shade)) {
      tz1utListAdd(&entry->shade, TZ1UT_LIST(tz1em_vf_na_root));
#if defined(TZ1EM_UPDATE_WORST_REQUIREMENT_CODE_PLACE_A)
      tz1emUpdateWorstRequirement(
        false, entry->condition.sunshine_vf,
        entry->condition.mode, entry->condition.permissible_time_lag_us);
#endif
    }
  }

  TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  if (tz1smHalMutexRelease(tz1em_midx) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  return status;
}

void tz1emIsrParticipateIn(tz1em_t * const entry)
{
  tz1emInfo_t *pi = &entry->info;
  uint32_t ipsr;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (0 == (ipsr = TZ1SM_HAL_IRQ_GET_IPSR())) TZ1SM_HAL_IRQ_DISABLE_SAVE();

  if (true == pi->init && true == pi->conf && true == pi->enable) {
    if (tz1utListIsEmpty(&entry->sunshine))
      tz1utListAdd(&entry->sunshine, TZ1UT_LIST(tz1em_vf_ac_root));
    if (tz1utListIsEmpty(&entry->shade)) {
      tz1utListAdd(&entry->shade, TZ1UT_LIST(tz1em_vf_na_root));
#if defined(TZ1EM_UPDATE_WORST_REQUIREMENT_CODE_PLACE_A)
      {
        tz1emRequirement_t *pr = &entry->condition;
        tz1emUpdateWorstRequirement(
          false, pr->sunshine_vf, pr->mode, pr->permissible_time_lag_us);
      }
#endif
    }
  }

  if (ipsr == 0) TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  return;
}

static tz1emStatus_t
_tz1emGoIntoTheShade(tz1em_t * const entry, const bool schedule)
{
  /* Nothing to do when the entry did not participate in the active queue. */
  if (tz1utListIsEmpty(&entry->sunshine)) return TZ1EM_STATUS_OK;

  /* TZ1EM V0.0.1.FS will not change the power profile when someone runs. */
  tz1utListDel(&entry->sunshine); tz1utListInit(&entry->sunshine);

  if (!tz1utListIsEmpty(TZ1UT_LIST(tz1em_vf_ac_root))) return TZ1EM_STATUS_OK;

  /* Use Sched-Aret0 */
  if (true == schedule) return tz1emSsScheduleAret0Pdvfs();

  return TZ1EM_STATUS_OK;
}

tz1emStatus_t tz1emGoIntoTheShade(tz1em_t * const entry, const bool schedule)
{
  tz1emStatus_t status = TZ1EM_STATUS_OK;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (tz1smHalMutexWait(tz1em_midx, TZ1SM_WAIT) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  TZ1SM_HAL_IRQ_DISABLE_SAVE();

  status = _tz1emGoIntoTheShade(entry, schedule);

  TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  if (tz1smHalMutexRelease(tz1em_midx) != TZ1SM_HAL_STATUS_OK)
    return TZ1EM_STATUS_ERROR_RESOURCE;

  return status;
}

tz1emStatus_t tz1emIsrGoIntoTheShade(tz1em_t * const entry, const bool schedule)
{
  tz1emStatus_t status;
  uint32_t ipsr;
  TZ1SM_HAL_INTR_STATUS_DEF;

  if (0 == (ipsr = TZ1SM_HAL_IRQ_GET_IPSR())) TZ1SM_HAL_IRQ_DISABLE_SAVE();

  status = _tz1emGoIntoTheShade(entry, schedule);

  if (ipsr == 0) TZ1SM_HAL_IRQ_ENABLE_RESTORE();

  return status;
}
