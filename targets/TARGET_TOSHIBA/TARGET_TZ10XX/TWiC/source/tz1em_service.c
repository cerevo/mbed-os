/**
 * @file tz1em_service.c
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

#include "tz1em_service.h"
#include "tz1em.h"

TZ1UT_LIST_EXTERN(tz1em_vf_ac_root);
TZ1UT_LIST_EXTERN(tz1em_vf_na_root);

#if defined(TZ1EM_UPDATE_WORST_REQUIREMENT_CODE_PLACE_A)
static tz1emWorstRequirement_t tz1em_wr;

void tz1emUpdateWorstRequirement(
  const bool removal, const tz1emVf_t sunshine_vf,
  const tz1emOm_t mode, const uint32_t permissible_time_lag_us)
{
  if (true == removal) {
    tz1emSsFindVf(&tz1em_wr.sunshine_vf, &tz1em_wr.mode,
                  &tz1em_wr.permissible_time_lag_us);
    return;
  }    

  if (tz1em_wr.sunshine_vf > sunshine_vf)
    tz1em_wr.sunshine_vf = sunshine_vf;
  
  if ((tz1em_wr.mode & 0x7f) > (mode & 0x7f))
    tz1em_wr.mode = mode;
	else if ((tz1em_wr.mode & 0x7f) == mode)
		tz1em_wr.mode = mode;
  
  if (tz1em_wr.permissible_time_lag_us > permissible_time_lag_us)
    tz1em_wr.permissible_time_lag_us = permissible_time_lag_us;

  return;
}
#endif

static uint32_t
tz1emSsVfLag(const tz1emVf_t from, const tz1emVf_t to)
{
  uint32_t time_lag = 0;
  
  if (TZ1SM_HAL_VF_48M12 == from) time_lag = vf_hi_time_lag[to];
  else if (TZ1SM_HAL_VF_36M11 == from) time_lag = vf_um_time_lag[to];
  else if (TZ1SM_HAL_VF_12M10 == from) time_lag = vf_lm_time_lag[to];
  else if (TZ1SM_HAL_VF_04M09 == from) time_lag = vf_lo_time_lag[to];
  else time_lag = 0;

  return time_lag;
}

/* OperationMode lag */
static uint32_t tz1emSsOMLag(const tz1emVf_t vf, const tz1emOm_t op_mode)

{
  uint32_t time_lag = 0;

  if (vf == TZ1SM_HAL_VF_48M12) time_lag = op_hi_time_lag[op_mode];
  else if (vf == TZ1SM_HAL_VF_36M11) time_lag = op_um_time_lag[op_mode];
  else if (vf == TZ1SM_HAL_VF_12M10) time_lag = op_lm_time_lag[op_mode];
  else if (vf == TZ1SM_HAL_VF_04M09) time_lag = op_lo_time_lag[op_mode];
  else time_lag = 0;

  return time_lag;
}
    
void tz1emSsFindVf(
  tz1emVf_t * const sunshine_vf,
  tz1emOm_t * const mode, uint32_t * const permissible_time_lag_us)
{
  tz1em_t *p;
  tz1emVf_t _sunshine_vf = TZ1SM_HAL_VF_NONE;
  tz1emOm_t _mode = TZ1SM_HAL_OM_NONE;
  uint32_t _permissible_time_lag_us = TZ1EM_PTL_SAFE_ASYNCHRONOUS_IO;
  
  tz1utListEach(tz1em_t, p, TZ1UT_LIST(tz1em_vf_na_root), shade) {
    if (_sunshine_vf > p->condition.sunshine_vf)
      _sunshine_vf = p->condition.sunshine_vf;

    if (_permissible_time_lag_us > p->condition.permissible_time_lag_us)
      _permissible_time_lag_us = p->condition.permissible_time_lag_us;

    if (_mode > p->condition.mode)
      _mode = p->condition.mode;
  }
  
  *permissible_time_lag_us = _permissible_time_lag_us;
  *sunshine_vf = _sunshine_vf;
  *mode = _mode;
    
  return;
}

static TZ1K_INLINE void tz1emSsCwDisable(tz1em_t *p)
{
  const tz1emRequirement_t * const q = &p->condition;
  uint8_t i;

  for (i = 0; i < 3; i++) {
    tz1smHalConfigureWakeup(q->trigger[i].factor, TZ1SM_HAL_ACT_EVENT_DISABLE);
    tz1smHalEnableWakeup(q->trigger[i].factor, false);
  }

  return;
}

static TZ1K_INLINE void tz1emSsCwEnable(tz1em_t *p)
{
  const tz1emRequirement_t * const q = &p->condition;
  uint8_t i;
  
  for (i = 0; i < 3; i++) {
    tz1smHalConfigureWakeup(q->trigger[i].factor, q->trigger[i].event);
    tz1smHalEnableWakeup(q->trigger[i].factor, true);
  }

  return;
}

static void
tz1emSsSetVfDisableOpMode(const tz1emVf_t vf, const uint32_t time_lag)
{
  tz1emPcd_t pcd;
  uint32_t uc = 0;
#if defined(TZ1EM_REMOVE_ENTRY_WHEN_SCH)
  tz1em_t *p, *q = NULL;
#else
  tz1em_t *p;
#endif
  
  tz1utListEach(tz1em_t, p, TZ1UT_LIST(tz1em_vf_na_root), shade) {
    tz1smHalPmuLowPowerPcd(
      p->condition.pcd, &uc, TZ1SM_HAL_OM_ACTIVE, vf, NULL);
  }
  tz1smHalSetVf(vf);
  tz1utListEach(tz1em_t, p, TZ1UT_LIST(tz1em_vf_na_root), shade) {
#if defined(TZ1EM_REMOVE_ENTRY_WHEN_SCH)
    if (q) tz1utListInit(&q->shade);
#endif
    pcd = p->condition.pcd;
    tz1smHalPmuHighPowerPcd(pcd, &uc, TZ1SM_HAL_OM_ACTIVE, vf, NULL);

    tz1emSsCwDisable(p);
    p->info.necessary_time_lag_us = time_lag + 1;
    p->info.timer_expired = (time_lag < p->condition.permissible_time_lag_us) ?
      TZ1EM_INFO_NORMAL : TZ1EM_INFO_TIMER_EXPIRED;
    p->info.voltage = vf;
#if defined(TZ1EM_REMOVE_ENTRY_WHEN_SCH)
    tz1utListDel(&p->shade); q = p;
#endif
  }
#if defined(TZ1EM_REMOVE_ENTRY_WHEN_SCH)
  if (q) tz1utListInit(&q->shade);
#endif
  
  return;
}

static tz1smHalStatus_t tz1emSsSetVfEnableOpMode(
  const tz1emVf_t current_vf, const tz1emVf_t sunshine_vf,
  const tz1emOm_t mode, const uint32_t time_lag)
{
  tz1emPcd_t pcd;
  uint32_t uc = 0;
	tz1smHalStatus_t status;
#if defined(TZ1EM_REMOVE_ENTRY_WHEN_SCH)
  tz1em_t *p, *q = NULL;
#else
  tz1em_t *p;
#endif
  tz1utListEach(tz1em_t, p, TZ1UT_LIST(tz1em_vf_na_root), shade) {
    tz1smHalPmuLowPowerPcd(
      p->condition.pcd, &uc, mode, sunshine_vf, p->condition.sleep);
    tz1emSsCwEnable(p);
  }

  if (current_vf < sunshine_vf) {
    tz1smHalSetVf(sunshine_vf);
    status = tz1smHalOperationMode(mode);
  } else if (current_vf > sunshine_vf) {
    status = tz1smHalOperationMode(mode);
    tz1smHalSetVf(sunshine_vf);
  } else {
    status = tz1smHalOperationMode(mode);
  }
	
  tz1utListEach(tz1em_t, p, TZ1UT_LIST(tz1em_vf_na_root), shade) {
    pcd = p->condition.pcd;
    tz1smHalPmuHighPowerPcd(pcd, &uc, mode, sunshine_vf, p->condition.wakeup);

    tz1emSsCwDisable(p);
#if defined(TZ1EM_REMOVE_ENTRY_WHEN_SCH)
    if (q) tz1utListInit(&q->shade);
#endif    
    p->info.necessary_time_lag_us = time_lag + 1;
    p->info.timer_expired = (time_lag < p->condition.permissible_time_lag_us) ?
      TZ1EM_INFO_NORMAL : TZ1EM_INFO_TIMER_EXPIRED;
    p->info.voltage = sunshine_vf;

#if defined(TZ1EM_REMOVE_ENTRY_WHEN_SCH)
    tz1utListDel(&p->shade); q = p;
#endif    
  }
#if defined(TZ1EM_REMOVE_ENTRY_WHEN_SCH)
  if (q) tz1utListInit(&q->shade);
#endif
  return status;
}

#if 0
static void tz1emSsRemoveEntry(const uint32_t time_lag, const tz1emVf_t current_vf) __attribute__((unused));

static void
tz1emSsRemoveEntry(const uint32_t time_lag, const tz1emVf_t current_vf)
{
#if defined(TZ1EM_REMOVE_ENTRY_WHEN_SCH)
  tz1em_t *p, *q = NULL;
#else
  tz1em_t *p;
#endif

  tz1utListEach(tz1em_t, p, TZ1UT_LIST(tz1em_vf_na_root), shade) {
#if defined(TZ1EM_REMOVE_ENTRY_WHEN_SCH)
    if (q) tz1utListInit(&q->shade);
#endif    
    tz1emSsCwDisable(p);
    p->info.necessary_time_lag_us = time_lag + 1;
    p->info.timer_expired = (time_lag < p->condition.permissible_time_lag_us) ?
      TZ1EM_INFO_NORMAL : TZ1EM_INFO_TIMER_EXPIRED;
    p->info.voltage = current_vf;
#if defined(TZ1EM_REMOVE_ENTRY_WHEN_SCH)
    tz1utListDel(&p->shade); q = p;
#endif    
  }
#if defined(TZ1EM_REMOVE_ENTRY_WHEN_SCH)
  if (q) tz1utListInit(&q->shade);
#endif
  
  return;
}
#endif

tz1emStatus_t tz1emSsScheduleAret0Pdvfs(void)
{
  tz1emVf_t sunshine_vf, current_vf, lower_vf;
  tz1emOm_t mode, check_mode;
  uint32_t ntl, ptl = 0;
  uint32_t vf_time_lag_us = 0;
  uint32_t op_shade_time_lag_us = 0;
  uint32_t op_sunshine_time_lag_us = 0;
  uint32_t sleep_flag = 1;
  uint32_t deep_flag = 0;
  tz1smHalStatus_t hal_status = TZ1SM_HAL_STATUS_OK;
  tz1emStatus_t status = TZ1EM_STATUS_OK;
  
  current_vf = tz1smHalGetCurrentVf();

#if defined(TZ1EM_UPDATE_WORST_REQUIREMENT_CODE_PLACE_A)
  sunshine_vf = tz1em_wr.sunshine_vf;
  mode = tz1em_wr.mode;
  ptl = tz1em_wr.permissible_time_lag_us;
#else  
  tz1emSsFindVf(&sunshine_vf, &mode, &ptl);
#endif
  if (TZ1SM_HAL_OM_NONE == mode) {
    mode = TZ1SM_HAL_OM_ACTIVE;
  }
  if ((mode & 0x80) == 0x80) {
    deep_flag = 1u;
    mode &= 0x7f;
  }
  if (TZ1SM_HAL_VF_NONE == sunshine_vf) {
    sunshine_vf = current_vf;
  }
  if (current_vf > sunshine_vf) {
    lower_vf = current_vf;
  } else { 
    lower_vf = sunshine_vf;
  }
	
  if (TZ1SM_HAL_OM_ACTIVE < mode && mode < TZ1SM_HAL_OM_WAIT) {
    vf_time_lag_us = tz1emSsVfLag(current_vf, TZ1SM_HAL_VF_04M09);
    vf_time_lag_us += tz1emSsVfLag(TZ1SM_HAL_VF_04M09, sunshine_vf);
    
    op_shade_time_lag_us = tz1emSsOMLag(TZ1SM_HAL_VF_04M09, mode);
  } else {
    vf_time_lag_us = tz1emSsVfLag(current_vf, sunshine_vf);
    op_shade_time_lag_us = tz1emSsOMLag(lower_vf, mode);
  }
  op_sunshine_time_lag_us = tz1emSsOMLag(lower_vf, mode);
  
  /* PDVFS (pseudo dynamic voltage and frequency scaling).
   * Do everything within a given definite period of time. */

  ntl = vf_time_lag_us + op_shade_time_lag_us;

  /* current_vf >> [shade_vf] >> [op_mode] >> [sunshine_vf] */
  if (ntl < ptl) {
    if (TZ1SM_HAL_OM_ACTIVE < mode && mode < TZ1SM_HAL_OM_WAIT) {
      mode |= (deep_flag << 7);

      tz1smHalSetVf(TZ1SM_HAL_VF_04M09);
      hal_status = tz1emSsSetVfEnableOpMode(TZ1SM_HAL_VF_04M09, sunshine_vf, mode, ntl);
    } else {
      hal_status = tz1emSsSetVfEnableOpMode(current_vf, sunshine_vf, mode, ntl);
    }
		if (hal_status == TZ1SM_HAL_STATUS_OK && TZ1SM_HAL_OM_ACTIVE != mode) {
			status = TZ1EM_STATUS_HAS_SLEPT;
		}
  } else if (op_sunshine_time_lag_us >= ptl && vf_time_lag_us >= ptl && current_vf != sunshine_vf) {
    tz1emSsSetVfDisableOpMode(sunshine_vf, ntl);
  } else {
    check_mode = mode;
		/* TZ1SM_HAL_OM_STOP > TZ1SM_HAL_OM_RTC > TZ1SM_HAL_OM_RETENTION > ... 
		     > TZ1SM_HAL_OM_SLEEP1 > TZ1SM_HAL_OM_SLEEP0 > TZ1SM_HAL_OM_ACTIVE
		  */
    while(check_mode > TZ1SM_HAL_OM_ACTIVE) {
      check_mode -= 1;
      if (check_mode < TZ1SM_HAL_OM_WAIT && sleep_flag) {
        if (mode > TZ1SM_HAL_OM_SLEEP2) {
          vf_time_lag_us = tz1emSsVfLag(current_vf, TZ1SM_HAL_VF_04M09);
          vf_time_lag_us += tz1emSsVfLag(TZ1SM_HAL_VF_04M09, sunshine_vf);
        }
        while (check_mode > TZ1SM_HAL_OM_ACTIVE) {
					ntl = vf_time_lag_us + tz1emSsOMLag(TZ1SM_HAL_VF_04M09, check_mode);
          if (ntl < ptl) {
						check_mode |= (deep_flag << 7);
            tz1smHalSetVf(TZ1SM_HAL_VF_04M09);
            hal_status = tz1emSsSetVfEnableOpMode(TZ1SM_HAL_VF_04M09, sunshine_vf, check_mode, ntl);
						if (hal_status == TZ1SM_HAL_STATUS_OK && TZ1SM_HAL_OM_ACTIVE != check_mode) {
							status = TZ1EM_STATUS_HAS_SLEPT;
						}
            return status;
          }
          check_mode -= 1;
        }
      }
      if (check_mode < TZ1SM_HAL_OM_SLEEP0) {
        if (sleep_flag) {
          sleep_flag = 0;
          vf_time_lag_us = tz1emSsVfLag(current_vf, sunshine_vf);
          if (mode < TZ1SM_HAL_OM_WAIT) {
            check_mode = mode;
          } else {
            check_mode = TZ1SM_HAL_OM_SLEEP2;
          }
        } else {
          break;
        }
      }
			ntl = vf_time_lag_us + tz1emSsOMLag(lower_vf, check_mode);
      if (ntl < ptl) {
				if (check_mode < TZ1SM_HAL_OM_WAIT) {
					check_mode |= (deep_flag << 7);
					hal_status = tz1emSsSetVfEnableOpMode(current_vf, sunshine_vf, check_mode, ntl); 
        } else {
					hal_status = tz1emSsSetVfEnableOpMode(current_vf, sunshine_vf, check_mode, ntl);
				}
        if (hal_status == TZ1SM_HAL_STATUS_OK && TZ1SM_HAL_OM_ACTIVE != check_mode) {
				  status = TZ1EM_STATUS_HAS_SLEPT;
				}
        return status;
      }
    }
    tz1emSsSetVfDisableOpMode(sunshine_vf, ntl);
  }
  return status;
}

tz1emStatus_t tz1emSsInitializePcdSystem(void)
{
#if defined(TZ1EM_UPDATE_WORST_REQUIREMENT_CODE_PLACE_A)
  tz1em_wr.sunshine_vf = TZ1SM_HAL_VF_NONE;
  tz1em_wr.mode = TZ1SM_HAL_OM_NONE;
  tz1em_wr.permissible_time_lag_us = TZ1EM_PTL_SAFE_ASYNCHRONOUS_IO;
#endif  
  
  if (TZ1SM_HAL_STATUS_OK != tz1smHalInitializePcdSystem())
    return TZ1EM_STATUS_ERROR_HAL;

  return TZ1EM_STATUS_OK;
}

tz1emStatus_t tz1emSsStartClock(const tz1emPcd_t pcd)
{
  if (TZ1SM_HAL_STATUS_OK != tz1smStartClock(pcd))
    return TZ1EM_STATUS_ERROR_HAL;

  return TZ1EM_STATUS_OK;  
}

tz1emStatus_t tz1emSsStopClock(const tz1emPcd_t pcd)
{
  if (TZ1SM_HAL_STATUS_OK != tz1smStopClock(pcd))
    return TZ1EM_STATUS_ERROR_HAL;

  return TZ1EM_STATUS_OK;  
}

