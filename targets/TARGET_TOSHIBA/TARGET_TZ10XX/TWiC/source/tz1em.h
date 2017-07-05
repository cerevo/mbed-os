/**
 * @file tz1em.h
 * @brief Header file for TZ10xx TWiC TZ1EM (TZ1000 Series Energy
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

#ifndef _TZ1EM_H_
#define _TZ1EM_H_

#include "tz1ut_list.h"
#include "tz1sm_hal.h"

#define TZ1EM_INFO_INIT          (0x0)
#define TZ1EM_INFO_TIMER_EXPIRED (0x1)
#define TZ1EM_INFO_NORMAL        (0x2)

#define TZ1EM_NECESSARY_TIME_LAG_US_INIT (0xFFFFFFFF)
#define TZ1EM_PTL_SAFE_ASYNCHRONOUS_IO (0xFFFFFFFF)

#define TZ1EM_VF_HI  (TZ1SM_HAL_VF_48M12)
#define TZ1EM_VF_UM  (TZ1SM_HAL_VF_36M11)
#define TZ1EM_VF_LM  (TZ1SM_HAL_VF_12M10)
#define TZ1EM_VF_LO  (TZ1SM_HAL_VF_04M09)
#define TZ1EM_VF_UN  (TZ1SM_HAL_VF_NONE)

#define TZ1EM_WE_OFF (TZ1SM_HAL_ACT_EVENT_DISABLE)
#define TZ1EM_WE_LH  (TZ1SM_HAL_ACT_EVENT_LEVEL_HIGH)
#define TZ1EM_WE_LL  (TZ1SM_HAL_ACT_EVENT_LEVEL_LOW)
#define TZ1EM_WE_EP  (TZ1SM_HAL_ACT_EVENT_EDGE_POS)
#define TZ1EM_WE_EN  (TZ1SM_HAL_ACT_EVENT_EDGE_NEG)
#define TZ1EM_WE_EB  (TZ1SM_HAL_ACT_EVENT_EDGE_BOTH)

#define TZ1EM_WF_G0  (TZ1SM_HAL_ACT_FACTOR_GPIO_0)
#define TZ1EM_WF_G1  (TZ1SM_HAL_ACT_FACTOR_GPIO_1)
#define TZ1EM_WF_G2  (TZ1SM_HAL_ACT_FACTOR_GPIO_2)
#define TZ1EM_WF_G3  (TZ1SM_HAL_ACT_FACTOR_GPIO_3)
#define TZ1EM_WF_G4  (TZ1SM_HAL_ACT_FACTOR_GPIO_4)
#define TZ1EM_WF_G5  (TZ1SM_HAL_ACT_FACTOR_GPIO_5)
#define TZ1EM_WF_G6  (TZ1SM_HAL_ACT_FACTOR_GPIO_6)
#define TZ1EM_WF_G7  (TZ1SM_HAL_ACT_FACTOR_GPIO_7)
#define TZ1EM_WF_G24 (TZ1SM_HAL_ACT_FACTOR_GPIO_24)
#define TZ1EM_WF_G25 (TZ1SM_HAL_ACT_FACTOR_GPIO_25)
#define TZ1EM_WF_G26 (TZ1SM_HAL_ACT_FACTOR_GPIO_26)
#define TZ1EM_WF_G27 (TZ1SM_HAL_ACT_FACTOR_GPIO_27)
#define TZ1EM_WF_G30 (TZ1SM_HAL_ACT_FACTOR_GPIO_30)
#define TZ1EM_WF_BRO (TZ1SM_HAL_ACT_FACTOR_BROWNOUT)
#define TZ1EM_WF_RTC (TZ1SM_HAL_ACT_FACTOR_RTC)
#define TZ1EM_WF_UN  (0)

#define TZ1EM_PCD_NONE        (TZ1SM_HAL_PCD_NONE)
#define TZ1EM_PCD_SRAM1       (TZ1SM_HAL_PCD_SRAM1)
#define TZ1EM_PCD_FLASH       (TZ1SM_HAL_PCD_FLASH)
#define TZ1EM_PCD_UART0       (TZ1SM_HAL_PCD_UART0)
#define TZ1EM_PCD_UART1       (TZ1SM_HAL_PCD_UART1)
#define TZ1EM_PCD_UART2       (TZ1SM_HAL_PCD_UART2)
#define TZ1EM_PCD_ADC12       (TZ1SM_HAL_PCD_ADC12)
#define TZ1EM_PCD_ADC24       (TZ1SM_HAL_PCD_ADC24)
#define TZ1EM_PCD_DMAC        (TZ1SM_HAL_PCD_DMAC)
#define TZ1EM_PCD_USB         (TZ1SM_HAL_PCD_USB)
#define TZ1EM_PCD_ENC         (TZ1SM_HAL_PCD_ENC)
#define TZ1EM_PCD_SPI0        (TZ1SM_HAL_PCD_SPI0)
#define TZ1EM_PCD_SPI1        (TZ1SM_HAL_PCD_SPI1)
#define TZ1EM_PCD_SPI2        (TZ1SM_HAL_PCD_SPI2)
#define TZ1EM_PCD_SPI3        (TZ1SM_HAL_PCD_SPI3)
#define TZ1EM_PCD_I2C0        (TZ1SM_HAL_PCD_I2C0)
#define TZ1EM_PCD_I2C1        (TZ1SM_HAL_PCD_I2C1)
#define TZ1EM_PCD_I2C2        (TZ1SM_HAL_PCD_I2C2)
#define TZ1EM_PCD_WDT         (TZ1SM_HAL_PCD_WDT)

#define TZ1EM_PCD_PP0         (0)
#define TZ1EM_PCD_PP1         (TZ1SM_HAL_PCD_PP1)
#define TZ1EM_PCD_PP2         (0)
#define TZ1EM_PCD_MP          (0)


/* Power Control Domain Groups */

#define TZ1EM_PCDG_BLE_TZBT (TZ1EM_PCDG_URT2) /* TWiC Exclusive use. */
#define TZ1EM_PCDG_ACCEL    (TZ1EM_PCDG_SPI2)
#if 0
#define TZ1EM_PCDG_GYRO     (TZ1EM_PCDG_SPI3)
#define TZ1EM_PCDG_MAG      (TZ1EM_PCDG_I2C2)
#endif
#define TZ1EM_PCDG_ADC12    (TZ1EM_PCD_PP0 | TZ1EM_PCD_ADC12)
#define TZ1EM_PCDG_ADC24    (TZ1EM_PCD_PP0 | TZ1EM_PCD_ADC24)
#define TZ1EM_PCDG_USB      (TZ1EM_PCD_USB)
#define TZ1EM_PCDG_URT0     (TZ1EM_PCD_PP1 | TZ1EM_PCD_UART0)
#define TZ1EM_PCDG_URT1     (TZ1EM_PCD_PP1 | TZ1EM_PCD_UART1)
#define TZ1EM_PCDG_URT2     (TZ1EM_PCD_PP2 | TZ1EM_PCD_UART2)
#define TZ1EM_PCDG_SPI0     (TZ1EM_PCD_PP1 | TZ1EM_PCD_SPI0 )
#define TZ1EM_PCDG_SPI1     (TZ1EM_PCD_PP1 | TZ1EM_PCD_SPI1)
#define TZ1EM_PCDG_SPI2     (TZ1EM_PCD_PP0 | TZ1EM_PCD_SPI2)
#define TZ1EM_PCDG_SPI3     (TZ1EM_PCD_PP0 | TZ1EM_PCD_SPI3)
#define TZ1EM_PCDG_I2C0     (TZ1EM_PCD_PP1 | TZ1EM_PCD_I2C0)
#define TZ1EM_PCDG_I2C1     (TZ1EM_PCD_PP1 | TZ1EM_PCD_I2C1)
#define TZ1EM_PCDG_I2C2     (TZ1EM_PCD_PP0 | TZ1EM_PCD_I2C2)
#define TZ1EM_PCDG_DMAC     (TZ1EM_PCD_MP  | TZ1EM_PCD_DMAC)
#define TZ1EM_PCDG_NOR      (TZ1EM_PCD_MP  | TZ1EM_PCD_FLASH)
#define TZ1EM_PCDG_WDT      (TZ1EM_PCD_PP0 | TZ1EM_PCD_WDT)


#define TZ1EM_OP_ACTIVE         (TZ1SM_HAL_OM_ACTIVE)
#define TZ1EM_OP_SLEEP0         (TZ1SM_HAL_OM_SLEEP0)
#define TZ1EM_OP_SLEEP1         (TZ1SM_HAL_OM_SLEEP1)
#define TZ1EM_OP_SLEEP2         (TZ1SM_HAL_OM_SLEEP2)
#define TZ1EM_OP_WAIT           (TZ1SM_HAL_OM_WAIT)
#define TZ1EM_OP_WAIT_RETENTION (TZ1SM_HAL_OM_WAIT_RETENTION)
#define TZ1EM_OP_RETENTION      (TZ1SM_HAL_OM_RETENTION)
#define TZ1EM_OP_RTC            (TZ1SM_HAL_OM_RTC)
#define TZ1EM_OP_STOP           (TZ1SM_HAL_OM_STOP)
#define TZ1EM_OP_UN             (TZ1SM_HAL_OM_NONE)

#define TZ1EM_OP_ADC12_ACTIVE    (TZ1SM_HAL_OM_ACTIVE)
#define TZ1EM_OP_ADC12_BG_S0     (TZ1SM_HAL_OM_SLEEP0)
#define TZ1EM_OP_ADC12_BG_S1     (TZ1SM_HAL_OM_SLEEP1)
#define TZ1EM_OP_ADC12_DOZE_S2   (TZ1SM_HAL_OM_SLEEP2)
#define TZ1EM_OP_ADC12_DOZE_WAIT (TZ1SM_HAL_OM_WAIT)
#define TZ1EM_OP_ADC12_DOZE_WRET (TZ1SM_HAL_OM_WAIT_RETENTION)
#define TZ1EM_OP_ADC12_RESET_RET (TZ1SM_HAL_OM_RETENTION)
#define TZ1EM_OP_ADC12_POFF_RTC  (TZ1SM_HAL_OM_RTC)
#define TZ1EM_OP_ADC12_POFF_STOP (TZ1SM_HAL_OM_STOP)
#define TZ1EM_OP_ADC12_UN        (TZ1SM_HAL_OM_NONE)

#define TZ1EM_OP_ADC24_ACTIVE    (TZ1SM_HAL_OM_ACTIVE)
#define TZ1EM_OP_ADC24_BG_S0     (TZ1SM_HAL_OM_SLEEP0)
#define TZ1EM_OP_ADC24_BG_S1     (TZ1SM_HAL_OM_SLEEP1)
#define TZ1EM_OP_ADC24_DOZE_S2   (TZ1SM_HAL_OM_SLEEP2)
#define TZ1EM_OP_ADC24_DOZE_WAIT (TZ1SM_HAL_OM_WAIT)
#define TZ1EM_OP_ADC24_DOZE_WRET (TZ1SM_HAL_OM_WAIT_RETENTION)
#define TZ1EM_OP_ADC24_RESET_RET (TZ1SM_HAL_OM_RETENTION)
#define TZ1EM_OP_ADC24_POFF_RTC  (TZ1SM_HAL_OM_RTC)
#define TZ1EM_OP_ADC24_POFF_STOP (TZ1SM_HAL_OM_STOP)
#define TZ1EM_OP_ADC24_UN        (TZ1SM_HAL_OM_NONE)

#define TZ1EM_OP_USB_ACTIVE    (TZ1SM_HAL_OM_ACTIVE)
#define TZ1EM_OP_USB_POFF_S0   (TZ1SM_HAL_OM_SLEEP0)
#define TZ1EM_OP_USB_POFF_S1   (TZ1SM_HAL_OM_SLEEP1)
#define TZ1EM_OP_USB_POFF_S2   (TZ1SM_HAL_OM_SLEEP2)
#define TZ1EM_OP_USB_POFF_WAIT (TZ1SM_HAL_OM_WAIT)
#define TZ1EM_OP_USB_POFF_WRET (TZ1SM_HAL_OM_WAIT_RETENTION)
#define TZ1EM_OP_USB_POFF_RET  (TZ1SM_HAL_OM_RETENTION)
#define TZ1EM_OP_USB_POFF_RTC  (TZ1SM_HAL_OM_RTC)
#define TZ1EM_OP_USB_POFF_STOP (TZ1SM_HAL_OM_STOP)
#define TZ1EM_OP_USB_UN        (TZ1SM_HAL_OM_NONE)

#define TZ1EM_OP_URT_ACTIVE    (TZ1SM_HAL_OM_ACTIVE)
#define TZ1EM_OP_URT_BG_S0     (TZ1SM_HAL_OM_SLEEP0)
#define TZ1EM_OP_URT_BG_S1     (TZ1SM_HAL_OM_SLEEP1)
#define TZ1EM_OP_URT_DOZE_S2   (TZ1SM_HAL_OM_SLEEP2)
#define TZ1EM_OP_URT_DOZE_WAIT (TZ1SM_HAL_OM_WAIT)
#define TZ1EM_OP_URT_DOZE_WRET (TZ1SM_HAL_OM_WAIT_RETENTION)
#define TZ1EM_OP_URT_DOZE_RET  (TZ1SM_HAL_OM_RETENTION)
#define TZ1EM_OP_URT_POFF_RTC  (TZ1SM_HAL_OM_RTC)
#define TZ1EM_OP_URT_POFF_STOP (TZ1SM_HAL_OM_STOP)
#define TZ1EM_OP_URT_UN        (TZ1SM_HAL_OM_NONE)

#define TZ1EM_OP_BLE_ACTIVE    (TZ1SM_HAL_OM_ACTIVE)
#define TZ1EM_OP_BLE_BG_S0     (TZ1SM_HAL_OM_SLEEP0)
#define TZ1EM_OP_BLE_BG_S1     (TZ1SM_HAL_OM_SLEEP1)
#define TZ1EM_OP_BLE_DOZE_S2   (TZ1SM_HAL_OM_SLEEP2)
#define TZ1EM_OP_BLE_DOZE_WAIT (TZ1SM_HAL_OM_WAIT)
#define TZ1EM_OP_BLE_DOZE_WRET (TZ1SM_HAL_OM_WAIT_RETENTION)
#define TZ1EM_OP_BLE_DOZE_RET  (TZ1SM_HAL_OM_RETENTION)
#define TZ1EM_OP_BLE_POFF_RTC  (TZ1SM_HAL_OM_RTC)
#define TZ1EM_OP_BLE_POFF_STOP (TZ1SM_HAL_OM_STOP)
#define TZ1EM_OP_BLE_UN        (TZ1SM_HAL_OM_NONE)

#define TZ1EM_OP_SPI_ACTIVE    (TZ1SM_HAL_OM_ACTIVE)
#define TZ1EM_OP_SPI_BG_S0     (TZ1SM_HAL_OM_SLEEP0)
#define TZ1EM_OP_SPI_BG_S1     (TZ1SM_HAL_OM_SLEEP1)
#define TZ1EM_OP_SPI_DOZE_S2   (TZ1SM_HAL_OM_SLEEP2)
#define TZ1EM_OP_SPI_DOZE_WAIT (TZ1SM_HAL_OM_WAIT)
#define TZ1EM_OP_SPI_DOZE_WRET (TZ1SM_HAL_OM_WAIT_RETENTION)
#define TZ1EM_OP_SPI_DOZE_RET  (TZ1SM_HAL_OM_RETENTION)
#define TZ1EM_OP_SPI_POFF_RTC  (TZ1SM_HAL_OM_RTC)
#define TZ1EM_OP_SPI_POFF_STOP (TZ1SM_HAL_OM_STOP)
#define TZ1EM_OP_SPI_UN        (TZ1SM_HAL_OM_NONE)

#define TZ1EM_OP_I2C_ACTIVE    (TZ1SM_HAL_OM_ACTIVE)
#define TZ1EM_OP_I2C_BG_S0     (TZ1SM_HAL_OM_SLEEP0)
#define TZ1EM_OP_I2C_BG_S1     (TZ1SM_HAL_OM_SLEEP1)
#define TZ1EM_OP_I2C_DOZE_S2   (TZ1SM_HAL_OM_SLEEP2)
#define TZ1EM_OP_I2C_DOZE_WAIT (TZ1SM_HAL_OM_WAIT)
#define TZ1EM_OP_I2C_DOZE_WRET (TZ1SM_HAL_OM_WAIT_RETENTION)
#define TZ1EM_OP_I2C_DOZE_RET  (TZ1SM_HAL_OM_RETENTION)
#define TZ1EM_OP_I2C_POFF_RTC  (TZ1SM_HAL_OM_RTC)
#define TZ1EM_OP_I2C_POFF_STOP (TZ1SM_HAL_OM_STOP)
#define TZ1EM_OP_I2C_UN        (TZ1SM_HAL_OM_NONE)

#define TZ1EM_OP_DMAC_ACTIVE    (TZ1SM_HAL_OM_ACTIVE)
#define TZ1EM_OP_DMAC_BG_S0     (TZ1SM_HAL_OM_SLEEP0)
#define TZ1EM_OP_DMAC_DOZE_S1   (TZ1SM_HAL_OM_SLEEP1)
#define TZ1EM_OP_DMAC_DOZE_S2   (TZ1SM_HAL_OM_SLEEP2)
#define TZ1EM_OP_DMAC_DOZE_WAIT (TZ1SM_HAL_OM_WAIT)
#define TZ1EM_OP_DMAC_DOZE_WRET (TZ1SM_HAL_OM_WAIT_RETENTION)
#define TZ1EM_OP_DMAC_DOZE_RET  (TZ1SM_HAL_OM_RETENTION)
#define TZ1EM_OP_DMAC_POFF_RTC  (TZ1SM_HAL_OM_RTC)
#define TZ1EM_OP_DMAC_POFF_STOP (TZ1SM_HAL_OM_STOP)
#define TZ1EM_OP_DMAC_UN        (TZ1SM_HAL_OM_NONE)

#define TZ1EM_OP_NOR_ACTIVE    (TZ1SM_HAL_OM_ACTIVE)
#define TZ1EM_OP_NOR_BG_S0     (TZ1SM_HAL_OM_SLEEP0)
#define TZ1EM_OP_NOR_BG_S1     (TZ1SM_HAL_OM_SLEEP1)
#define TZ1EM_OP_NOR_DOZE_S2   (TZ1SM_HAL_OM_SLEEP2)
#define TZ1EM_OP_NOR_DOZE_WAIT (TZ1SM_HAL_OM_WAIT)
#define TZ1EM_OP_NOR_DOZE_WRET (TZ1SM_HAL_OM_WAIT_RETENTION)
#define TZ1EM_OP_NOR_DOZE_RET  (TZ1SM_HAL_OM_RETENTION)
#define TZ1EM_OP_NOR_POFF_RTC  (TZ1SM_HAL_OM_RTC)
#define TZ1EM_OP_NOR_POFF_STOP (TZ1SM_HAL_OM_STOP)
#define TZ1EM_OP_NOR_UN        (TZ1SM_HAL_OM_NONE)

#define TZ1EM_OP_WDT_ACTIVE    (TZ1SM_HAL_OM_ACTIVE)
#define TZ1EM_OP_WDT_BG_S0     (TZ1SM_HAL_OM_SLEEP0)
#define TZ1EM_OP_WDT_BG_S1     (TZ1SM_HAL_OM_SLEEP1)
#define TZ1EM_OP_WDT_DOZE_S2   (TZ1SM_HAL_OM_SLEEP2)
#define TZ1EM_OP_WDT_DOZE_WAIT (TZ1SM_HAL_OM_WAIT)
#define TZ1EM_OP_WDT_DOZE_WRET (TZ1SM_HAL_OM_WAIT_RETENTION)
#define TZ1EM_OP_WDT_DOZE_RET  (TZ1SM_HAL_OM_RETENTION)
#define TZ1EM_OP_WDT_POFF_RTC  (TZ1SM_HAL_OM_RTC)
#define TZ1EM_OP_WDT_POFF_STOP (TZ1SM_HAL_OM_STOP)
#define TZ1EM_OP_WDT_UN        (TZ1SM_HAL_OM_NONE)

typedef tz1smHalPcd_t tz1emPcd_t;
typedef tz1smHalWe_t tz1emWe_t;
typedef uint32_t tz1emWf_t;
typedef tz1smHalVf_t tz1emVf_t;
typedef tz1smHalOm_t tz1emOm_t;
typedef tz1smHalWakeupCb_t tz1emCallback_t;

typedef enum {
  TZ1EM_STATUS_OK = TZ1SM_HAL_STATUS_OK,
  TZ1EM_STATUS_ERROR_PARAMETER = TZ1SM_HAL_STATUS_ERROR_PARAMETER,
  TZ1EM_STATUS_ERROR_RESOURCE = TZ1SM_HAL_STATUS_ERROR_RESOURCE,
  TZ1EM_STATUS_OPERATION_NOT_PERMITTED,
  TZ1EM_STATUS_ERROR_HAL,
  TZ1EM_STATUS_HAS_SLEPT,
  TZ1EM_STATUS_RESERVED = TZ1SM_HAL_STATUS_RESERVED,
} tz1emStatus_t;

#define TZ1EM_NUM_OF_TRIGGER (3)
typedef TZ1K_PACKED_HDR struct {
  tz1emPcd_t pcd; /* power control domain */
  tz1emVf_t sunshine_vf;
  tz1emOm_t mode;
  uint32_t permissible_time_lag_us;
  tz1emCallback_t wakeup;
  tz1emCallback_t sleep;
  struct TZ1K_PACKED_FTR {
    tz1emWe_t event;
    tz1emWf_t factor;
  } TZ1K_PACKED_FTR trigger[TZ1EM_NUM_OF_TRIGGER];
} TZ1K_PACKED_FTR tz1emRequirement_t;

typedef TZ1K_PACKED_HDR struct {
  uint32_t necessary_time_lag_us;
  uint32_t wakeup_reason;
  uint8_t timer_expired : 2;
  uint8_t init : 1;
  uint8_t conf : 1;
  uint8_t enable : 1;
  uint8_t voltage : 3;
} TZ1K_PACKED_FTR tz1emInfo_t;

/* TZ1EM power profile entry definition. */

typedef TZ1K_PACKED_HDR struct {
  tz1utListHead_t sunshine;
  tz1utListHead_t shade;
  tz1emRequirement_t condition;
  tz1emInfo_t info;
 /* shade_duration_us and elapsed_times is omitted. */
} TZ1K_PACKED_FTR tz1em_t;

#define TZ1EM_DEF(name) tz1em_t tz1em_##name
#define TZ1EM_EXTERN(name) extern tz1em_t tz1em_##name
#define TZ1EM(name) (&tz1em_##name)

tz1emStatus_t tz1emInitializeSystem(void);
tz1emStatus_t tz1emStartClock(const tz1emPcd_t pcd);
tz1emStatus_t tz1emStopClock(const tz1emPcd_t pcd);
tz1emStatus_t tz1emInitializeEntry(tz1em_t * const entry);
tz1emStatus_t tz1emConfigureEntry(
  tz1em_t * const entry, tz1emRequirement_t * const condition);
tz1emStatus_t tz1emReconfigureEntry(
  tz1em_t * const entry, tz1emRequirement_t * const condition);
tz1emStatus_t tz1emReconfigureParticipateIn(
  tz1em_t * const entry, tz1emRequirement_t * const condition);
tz1emStatus_t tz1emParticipateIn(tz1em_t * const entry);
void tz1emIsrParticipateIn(tz1em_t * const entry);
tz1emStatus_t tz1emWithdraw(tz1em_t * const entry);
tz1emStatus_t tz1emTemporaryWithdraw(tz1em_t * const entry);
tz1emStatus_t tz1emIsrTemporaryWithdraw(tz1em_t * const entry);
tz1emStatus_t tz1emPermit(tz1em_t * const entry);
tz1emStatus_t tz1emGoIntoTheShade(tz1em_t * const entry, const bool schedule);
tz1emStatus_t tz1emIsrGoIntoTheShade(tz1em_t * const entry,
                                     const bool schedule);

#endif /* _TZ1EM_H_ */
