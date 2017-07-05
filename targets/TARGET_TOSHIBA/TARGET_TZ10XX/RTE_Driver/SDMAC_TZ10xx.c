/**
 * @file SDMAC_TZ10xx.c
 * @brief a header file for TZ10xx SDMAC driver
 * @date $Date:: 2015-04-17 09:01:26 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * Portions Copyright (C) 2013 Synopsys, Inc.  
 * Used with permission. All rights reserved. 
 * Synopsys & DesignWare are registered trademarks of Synopsys, Inc.
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "SDMAC_TZ10xx.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

#if RTE_SDMAC

/* Driver version */
#define TZ1000_SDMAC_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,29)

/* bitband alias */
#define BB_SDMAC_STATUSTFR(_x)          BITBAND_VALUE(&sdmac->STATUSTFR,_x)
#define BB_SDMAC_STATUSERR(_x)          BITBAND_VALUE(&sdmac->STATUSERR,_x)
#define BB_SDMAC_CH_SUSP(_x)            BITBAND_VALUE(sdmac_BASE+0x40+0x58*_x,8)
#define BB_SDMAC_FIFO_EMPTY(_x)         BITBAND_VALUE(sdmac_BASE+0x40+0x58*_x,9)

/* bit position of CFG_H register */
#define SDMAC_CFG_FIFO_MODE_POS         (1)
#define SDMAC_CFG_SRC_PER_POS           (7)
#define SDMAC_CFG_DEST_PER_POS          (11)
/* bit position of CTL register */
#define SDMAC_CTL_TT_FC_SRC_HW          (1<<21)
#define SDMAC_CTL_TT_FC_DST_HW          (1<<20)

/* extract SRC_TR_WIDTH */
#define SDMAC_CTL_TO_SRC_WIDTH(_ctl)    ((_ctl>>4)&0x7)

/* runtime resources for channel depends */
typedef struct _SDMAC_CH_INFO {
  bool                act;
  SDMAC_SignalEvent_t cb;
  uint32_t            ctl;
} SDMAC_CH_INFO;

/* resources for channel depends */
typedef struct _SDMAC_CH_RESOURCE {
  sdmac_Type     *dma;          /* pointer to dma channel depend registers */
  const IRQn_Type irq;          /* number of interrupt request for DMA channel */
  SDMAC_CH_INFO  *inf;
} SDMAC_CH_RESOURCE;

/* runtime resources for channel independent */
typedef struct _SDMAC_INFO {
  bool            init;
  ARM_POWER_STATE power;
  uint8_t         used_ch;         /* bit pattern for shared dma channel */
  uint8_t         used_hs;         /* bit pattern for shared handshake channel */
} SDMAC_INFO;

/* resources for channel independent */
typedef struct _SDMAC_RESOURCE {
  sdmac_Type        *dma;
  SDMAC_INFO        *info;
  SDMAC_CH_RESOURCE  ch[8];
} SDMAC_RESOURCE;

/* prototype declarations for implementation driver */
static SDMAC_STATUS SDMAC_CheckChannelParameter(
  SDMAC_SRC src_hw, SDMAC_DST dst_hw,
  SDMAC_FIFO_MODE mode, SDMAC_PRIORITY priority);
static void         SDMAC_EnableChannel(
  int32_t ch, SDMAC_SignalEvent_t cb_event,
  SDMAC_SRC src_hw, SDMAC_DST dst_hw,
  SDMAC_FIFO_MODE mode, SDMAC_PRIORITY priority,
  int32_t src_handshake, int32_t dst_handshake);
static SDMAC_STATUS SDMAC_DisableChannel(int32_t ch);
static ARM_DRIVER_VERSION SDMAC_GetVersion(void);
static SDMAC_STATUS SDMAC_Initialize(void);
static SDMAC_STATUS SDMAC_Uninitialize(void);
static SDMAC_STATUS SDMAC_PowerControl(ARM_POWER_STATE state);
static SDMAC_STATUS SDMAC_ChannelInitialize(
  int32_t ch, SDMAC_SignalEvent_t cb_event,
  SDMAC_SRC src_hw, SDMAC_DST dst_hw,
  SDMAC_FIFO_MODE mode, SDMAC_PRIORITY priority,
  int32_t src_handshake, int32_t dst_handshake);
static SDMAC_STATUS SDMAC_ChannelUninitialize(int32_t ch);
static int32_t      SDMAC_ChannelAcquire(
  SDMAC_SignalEvent_t cb_event,
  SDMAC_SRC src_hw, SDMAC_DST dst_hw,
  SDMAC_FIFO_MODE mode, SDMAC_PRIORITY priority);
static SDMAC_STATUS SDMAC_ChannelRelease(int32_t ch);
static SDMAC_STATUS SDMAC_ChannelTransferData(
  int32_t ch, const uint8_t *src, uint8_t *dst, uint32_t size, uint32_t ctl);
static bool SDMAC_ChannelBusy(int32_t ch);
static SDMAC_STATUS SDMAC_ChannelAbortTransfer(int32_t ch);

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  TZ1000_SDMAC_API_VERSION,
  TZ1000_SDMAC_DRV_VERSION,
};

static SDMAC_INFO     SDMAC0_INFO;
static SDMAC_CH_INFO  SDMAC0_CH_INFO[8];
static SDMAC_RESOURCE SDMAC0_RESOURCE = {
  sdmac,
  &SDMAC0_INFO,
  {{(sdmac_Type*)&sdmac->SAR0, SDMAC0_IRQn, &SDMAC0_CH_INFO[0]},
   {(sdmac_Type*)&sdmac->SAR1, SDMAC1_IRQn, &SDMAC0_CH_INFO[1]},
   {(sdmac_Type*)&sdmac->SAR2, SDMAC2_IRQn, &SDMAC0_CH_INFO[2]},
   {(sdmac_Type*)&sdmac->SAR3, SDMAC3_IRQn, &SDMAC0_CH_INFO[3]},
   {(sdmac_Type*)&sdmac->SAR4, SDMAC4_IRQn, &SDMAC0_CH_INFO[4]},
   {(sdmac_Type*)&sdmac->SAR5, SDMAC5_IRQn, &SDMAC0_CH_INFO[5]},
   {(sdmac_Type*)&sdmac->SAR6, SDMAC6_IRQn, &SDMAC0_CH_INFO[6]},
   {(sdmac_Type*)&sdmac->SAR7, SDMAC7_IRQn, &SDMAC0_CH_INFO[7]},
  }
};

/* local functions */

static void SDMAC_ResetOn(void)
{
  Driver_PMU.EnableModule(PMU_MODULE_SDMAC, 0);
  Driver_PMU.SetPowerDomainState(PMU_PD_DMAC, PMU_PD_MODE_OFF);
}

static void SDMAC_ResetOff(void)
{
  Driver_PMU.SetPowerDomainState(PMU_PD_DMAC, PMU_PD_MODE_ON);
  Driver_PMU.EnableModule(PMU_MODULE_SDMAC, 1);
}

static SDMAC_STATUS SDMAC_CheckChannelParameter(
  SDMAC_SRC src_hw, SDMAC_DST dst_hw,
  SDMAC_FIFO_MODE mode, SDMAC_PRIORITY priority)
{
  if ((src_hw < SDMAC_SRC_MEMORY) ||
      ((9 < src_hw) && (src_hw < 20)) ||
      (34 < src_hw)) {
    return SDMAC_ERROR;
  }
  if (((SDMAC_DST_MEMORY != dst_hw) && (dst_hw < 10)) ||
      (19 < dst_hw)) {
    return SDMAC_ERROR;
  }
  if (SDMAC_FIFO_MODE_UTILIZATION < mode) {
    return SDMAC_ERROR;
  }
  if (SDMAC_PRIORITY_HIGHEST < priority) {
    return SDMAC_ERROR;
  }
  return SDMAC_OK;
}

/* working under ChannelInitialize and ChannelAcquire */
static void SDMAC_EnableChannel(
  int32_t ch, SDMAC_SignalEvent_t cb_event,
  SDMAC_SRC src_hw, SDMAC_DST dst_hw,
  SDMAC_FIFO_MODE mode, SDMAC_PRIORITY priority,
  int32_t src_handshake, int32_t dst_handshake)
{
  SDMAC_RESOURCE    *res = &SDMAC0_RESOURCE;
  SDMAC_CH_RESOURCE *chr;
  SDMAC_CH_INFO     *chi;
  sdmac_Type        *chd;
  sdmac_Type        *dma;
  uint32_t          *sel;
  uint32_t           cfg_h, ctl, val;
  IRQn_Type          irq;

  chr = &res->ch[ch];
  chi = chr->inf;
  if (chi->act) {
    SDMAC_DisableChannel(ch);
  }

  chi     = chr->inf;
  chi->cb = cb_event;
  ctl     = 1;
  cfg_h   = mode << SDMAC_CFG_FIFO_MODE_POS;
  if (SDMAC_DST_MEMORY != dst_hw) {
    cfg_h |= dst_handshake << SDMAC_CFG_DEST_PER_POS;
    sel    = (uint32_t*)(&gconf->DMACREQ_SEL0) + dst_handshake;
    *sel   = dst_hw;
    ctl   |= SDMAC_CTL_TT_FC_DST_HW;
  }
  if (SDMAC_SRC_MEMORY != src_hw) {
    cfg_h |= src_handshake << SDMAC_CFG_SRC_PER_POS;
    sel    = (uint32_t*)(&gconf->DMACREQ_SEL0) + src_handshake;
    *sel   = src_hw;
    ctl   |= SDMAC_CTL_TT_FC_SRC_HW;
  }
  chi->ctl    = ctl;
  chd         = chr->dma;
  chd->CFG0   = priority;
  chd->CFG0_H = cfg_h;
  chi->act    = true;

  val = 1 << ch;
  dma           = res->dma;
  dma->CLEARTFR = val;          /* clear interrupt status */
  dma->CLEARERR = val;

  val = 0x101 << ch;
  dma->MASKTFR  = val;          /* unmask interrupt */
  dma->MASKERR  = val;          /* unmask interrupt */
  irq = chr->irq;
  NVIC_ClearPendingIRQ(irq);
  NVIC_EnableIRQ(irq);
}

static SDMAC_STATUS SDMAC_DisableChannel(int32_t ch)
{
  SDMAC_RESOURCE    *res = &SDMAC0_RESOURCE;
  SDMAC_CH_RESOURCE *chr;
  SDMAC_CH_INFO     *chi;
  sdmac_Type        *dma;
  uint32_t           ch_en, mask, val;
  IRQn_Type          irq;

  chr = &res->ch[ch];
  chi = chr->inf;
  if (false == chi->act) {
    return SDMAC_ERROR;
  }
  irq = chr->irq;
  NVIC_DisableIRQ(irq);

  val = 0x100 << ch;
  dma          = res->dma;
  dma->CHENREG = val;           /* disable channel */
  dma->MASKTFR = val;           /* mask interrupt */
  dma->MASKERR = val;

  mask = 1 << ch;
  do {
    ch_en = dma->CHENREG;
  } while (ch_en & mask);
  if (0 == ch_en) {
    dma->DMACFGREG = 0;         /* disable SDMAC */
  }

  NVIC_ClearPendingIRQ(irq);

  chi->cb  = NULL;
  chi->ctl = 0;
  chi->act = false;
  return SDMAC_OK;
}

/* driver api functions */

static ARM_DRIVER_VERSION SDMAC_GetVersion(void)
{
  return DriverVersion;
}

/*!
 * \brief       Initialize driver.
 * \return      \ref SDMAC_STATUS
 */
static SDMAC_STATUS SDMAC_Initialize(void)
{
  SDMAC_RESOURCE *res = &SDMAC0_RESOURCE;
  SDMAC_INFO     *info;

  info = res->info;
  if (true == info->init) {
    return SDMAC_ERROR;
  }
  info->init = true;
  return SDMAC_PowerControl(ARM_POWER_LOW);
}

/*!
 * \brief       Initialize driver.
 * \return      \ref SDMAC_STATUS
 */
static SDMAC_STATUS SDMAC_Uninitialize(void)
{
  SDMAC_RESOURCE *res = &SDMAC0_RESOURCE;
  SDMAC_INFO     *info;

  info = res->info;
  if (false == info->init) {
    return SDMAC_OK;
  }

  SDMAC_PowerControl(ARM_POWER_OFF);
  info->init = false;
  return SDMAC_OK;
}

static SDMAC_STATUS SDMAC_PowerControl(ARM_POWER_STATE state)
{
  SDMAC_RESOURCE *res = &SDMAC0_RESOURCE;
  SDMAC_INFO     *inf;
  int             i;

  inf = res->info;
  if (false == inf->init) {
    return SDMAC_ERROR;
  }
  switch (state) {
  case ARM_POWER_OFF:
    for (i = 0; i < 8; ++i) {
      if (RTE_SDMAC_SHARED_DMA_CHANNEL & (1 << i)) {
        SDMAC_ChannelRelease(i);
      } else {
        SDMAC_DisableChannel(i);
      }
    }
    SDMAC_ResetOn();
    inf->power = ARM_POWER_OFF;
    break;
  case ARM_POWER_LOW:
  case ARM_POWER_FULL:
    SDMAC_ResetOff();
    inf->power = state;
    break;
  default:
    return SDMAC_ERROR;
  }
  return SDMAC_OK;
}

static SDMAC_STATUS SDMAC_ChannelInitialize(
  int32_t ch, SDMAC_SignalEvent_t cb_event,
  SDMAC_SRC src_hw, SDMAC_DST dst_hw,
  SDMAC_FIFO_MODE mode, SDMAC_PRIORITY priority,
  int32_t src_handshake, int32_t dst_handshake)
{
  SDMAC_RESOURCE    *res = &SDMAC0_RESOURCE;
  SDMAC_INFO        *inf;
  SDMAC_STATUS       err;

  inf = res->info;
  if (ARM_POWER_FULL != inf->power) {
    return SDMAC_ERROR;
  }
  if ((ch < 0) || (7 < ch)) {
    return SDMAC_ERROR_CH;
  }
  if (0 != (RTE_SDMAC_SHARED_DMA_CHANNEL & (1 << ch))) {
    return SDMAC_ERROR_CH;
  }
  err = SDMAC_CheckChannelParameter(src_hw, dst_hw, mode, priority);
  if (err) {
    return err;
  }
  if (SDMAC_SRC_MEMORY != src_hw) {
    if ((src_handshake < 0) || (7 < src_handshake)) {
      return SDMAC_ERROR;
    }
    if (0 != (RTE_SDMAC_SHARED_HANDSHAKE_CHANNEL & (1 << src_handshake))) {
      return SDMAC_ERROR_HS;
    }
  }
  if (SDMAC_DST_MEMORY != dst_hw) {
    if ((dst_handshake < 0) || (7 < dst_handshake)) {
      return SDMAC_ERROR;
    }
    if (0 != (RTE_SDMAC_SHARED_HANDSHAKE_CHANNEL & (1 << dst_handshake))) {
      return SDMAC_ERROR_HS;
    }
  }
  if ((src_handshake == dst_handshake) &&
      (SDMAC_SRC_MEMORY != src_hw) &&
      (SDMAC_DST_MEMORY != dst_hw)) {
    return SDMAC_ERROR_HS;
  }
  if (true == res->ch[ch].inf->act) {
    return SDMAC_ERROR_BUSY;
  }

  SDMAC_EnableChannel(
      ch, cb_event, src_hw, dst_hw, mode, priority,
      src_handshake, dst_handshake);

  return SDMAC_OK;
}

static SDMAC_STATUS SDMAC_ChannelUninitialize(int32_t ch)
{
  SDMAC_RESOURCE    *res = &SDMAC0_RESOURCE;

  if (ARM_POWER_FULL != res->info->power) {
    return SDMAC_ERROR;
  }
  if ((ch < 0) || (7 < ch)) {
    return SDMAC_ERROR_CH;
  }
  if (0 != (RTE_SDMAC_SHARED_DMA_CHANNEL & (1 << ch))) {
    return SDMAC_ERROR_CH;
  }
  return SDMAC_DisableChannel(ch);
}

static int32_t SDMAC_ChannelAcquire(
  SDMAC_SignalEvent_t cb_event,
  SDMAC_SRC src_hw, SDMAC_DST dst_hw,
  SDMAC_FIFO_MODE mode, SDMAC_PRIORITY priority)
{
  SDMAC_RESOURCE    *res = &SDMAC0_RESOURCE;
  SDMAC_INFO        *inf;
  SDMAC_STATUS       err;
  uint32_t           free_ch, free_hs;
  int32_t            ch, shs, dhs;

  inf = res->info;
  if (ARM_POWER_FULL != inf->power) {
    return -SDMAC_ERROR;
  }
  err = SDMAC_CheckChannelParameter(src_hw, dst_hw, mode, priority);
  if (err) {
    return -err;
  }

  /* search free channel */
  free_ch = RTE_SDMAC_SHARED_DMA_CHANNEL & ~inf->used_ch;
  if (0 == free_ch) {
    return -SDMAC_ERROR_CH;
  }
  ch       = 31 - __CLZ(free_ch);
  free_ch &= ~(1 << ch);

  free_hs = RTE_SDMAC_SHARED_HANDSHAKE_CHANNEL & ~inf->used_hs;
  if (SDMAC_SRC_MEMORY != src_hw) {
    /* search handshake channel */
    if (0 == free_hs) {
      return -SDMAC_ERROR_HS;
    }
    shs      = 31 - __CLZ(free_hs);
    free_hs &= ~(1 << shs);
  } else {
    shs = 0;
  }
  if (SDMAC_DST_MEMORY != dst_hw) {
    /* search handshake channel */
    if (0 == free_hs) {
      return -SDMAC_ERROR_HS;
    }
    dhs      = 31 - __CLZ(free_hs);
    free_hs &= ~(1 << dhs);
  } else {
    dhs = 0;
  }

  inf->used_ch = ~free_ch;
  inf->used_hs = ~free_hs;

  SDMAC_EnableChannel(
      ch, cb_event, src_hw, dst_hw,
      mode, priority, shs, dhs);
  return ch;
}

static SDMAC_STATUS SDMAC_ChannelRelease(int32_t ch)
{
  SDMAC_RESOURCE    *res = &SDMAC0_RESOURCE;
  SDMAC_INFO        *info;
  SDMAC_STATUS       err;
  SDMAC_CH_RESOURCE *chr;
  int32_t            hs, shs, dhs;
  uint32_t           cfg_h, ctl;

  if ((ch < 0) || (7 < ch)) {
    return SDMAC_ERROR_CH;
  }
  if (0 == (RTE_SDMAC_SHARED_DMA_CHANNEL & (1 << ch))) {
    return SDMAC_ERROR_CH;
  }

  /* read registers before the channel disabled. */
  chr   = &res->ch[ch];
  ctl   = chr->inf->ctl;
  cfg_h = chr->dma->CFG0_H;
  err = SDMAC_DisableChannel(ch);
  if (err) {
    return err;
  }

  /* make bit pattern of using handshake channels */
  hs = 0;
  if (0 != (ctl & SDMAC_CTL_TT_FC_SRC_HW)) {
    shs = (cfg_h >> SDMAC_CFG_SRC_PER_POS) & 0xF;
    hs |= 1 << shs;
  }
  if (0 != (ctl & SDMAC_CTL_TT_FC_DST_HW)) {
    dhs = (cfg_h >> SDMAC_CFG_DEST_PER_POS) & 0xF;
    hs |= 1 << dhs;
  }

  info = res->info;
  /* unmarking dma channel */
  info->used_ch &= ~(1 << ch);
  /* unmarking handshake channel */
  info->used_hs &= ~hs;

  return SDMAC_OK;
}

static SDMAC_STATUS SDMAC_ChannelTransferData(
  int32_t ch, const uint8_t *src, uint8_t *dst, uint32_t size, uint32_t ctl)
{
  SDMAC_RESOURCE    *res = &SDMAC0_RESOURCE;
  sdmac_Type        *dma;
  SDMAC_CH_RESOURCE *chr;
  SDMAC_CH_INFO     *chi;
  sdmac_Type        *chd;
  uint32_t           ch_en;
  uint32_t           val, src_width;

  if (ARM_POWER_FULL != res->info->power) {
    return SDMAC_ERROR;
  }
  if ((ch < 0) || (7 < ch)) {
    return SDMAC_ERROR_CH;
  }
  chr = &res->ch[ch];
  chi = chr->inf;
  if (false == chi->act) {
    return SDMAC_ERROR_CH;
  }

  /* divide size by src_width */
  src_width = SDMAC_CTL_TO_SRC_WIDTH(ctl);
  switch (src_width) {
  case SDMAC_WIDTH_1: break;
  case SDMAC_WIDTH_2: size /= 2; break;
  case SDMAC_WIDTH_4: size /= 4; break;
  default: return SDMAC_ERROR;
  }
  if (0 == size) {
    return SDMAC_ERROR;
  }

  ch_en  = res->dma->CHENREG;
  ch_en &= 1 << ch;
  if (0 != ch_en) {
    return SDMAC_ERROR_BUSY;
  }

  dma = res->dma;
  /* clear interrupt status */
  val               = 1 << ch;
  dma->CLEARTFR     = val;
  dma->CLEARBLOCK   = val;
  dma->CLEARSRCTRAN = val;
  dma->CLEARDSTTRAN = val;
  dma->CLEARERR     = val;

  chd         = chr->dma;
  chd->SAR0   = (uint32_t)src;
  chd->DAR0   = (uint32_t)dst;
  chd->CTL0   = chi->ctl | ctl;
  chd->CTL0_H = size;

  dma->DMACFGREG = 1;
  dma->CHENREG   = (0x101 << ch); /* enable channel */
  return SDMAC_OK;
}

static bool SDMAC_ChannelBusy(int32_t ch)
{
  SDMAC_RESOURCE *res = &SDMAC0_RESOURCE;
  sdmac_Type     *dma;
  uint32_t        ch_en;

  if ((ch < 0) || (7 < ch)) {
    return false;
  }
  if (ARM_POWER_FULL != res->info->power) {
    return false;
  }
  dma = res->dma;
  if (0 == dma->DMACFGREG) {
    return false;
  }
  ch_en  = dma->CHENREG;
  ch_en &= 1 << ch;
  if (0 == ch_en) {
    return false;
  }
  return true;
}

static SDMAC_STATUS SDMAC_ChannelAbortTransfer(int32_t ch)
{
  SDMAC_RESOURCE *res = &SDMAC0_RESOURCE;
  sdmac_Type     *dma;
  uint32_t        ch_en, mask, empty;

  if (ARM_POWER_FULL != res->info->power) {
    return SDMAC_ERROR;
  }
  if ((ch < 0) || (7 < ch)) {
    return SDMAC_ERROR_CH;
  }
  if (false == res->ch[ch].inf->act) {
    return SDMAC_ERROR_CH;
  }

  dma = res->dma;
  if (0 == dma->DMACFGREG) {
    return SDMAC_OK;
  }

  BB_SDMAC_CH_SUSP(ch) = 1;

  mask = 1 << ch;
  if (0 == (dma->CHENREG & mask)) {
    return SDMAC_OK;
  }
  do {
    empty = BB_SDMAC_FIFO_EMPTY(ch);
  } while (!empty);

  dma->CHENREG = 0x100 << ch;
  /* wait until terminating transfer */
  do {
    ch_en = dma->CHENREG;
  } while (ch_en & mask);
  if (0 == ch_en) {
    dma->DMACFGREG = 0;         /* disable SDMAC */
  }
  return SDMAC_OK;
}

static SDMAC_STATUS SDMAC_ChannelCountBytesTransferred(int32_t ch, uint32_t *num)
{
  SDMAC_RESOURCE    *res = &SDMAC0_RESOURCE;
  SDMAC_CH_RESOURCE *chr;
  sdmac_Type        *chd;
  uint32_t           width;

  if (ARM_POWER_FULL != res->info->power) {
    return SDMAC_ERROR;
  }
  if ((ch < 0) || (7 < ch)) {
    return SDMAC_ERROR_CH;
  }
  if (NULL == num) {
    return SDMAC_ERROR;
  }
  chr = &res->ch[ch];
  if (false == chr->inf->act) {
    return SDMAC_ERROR_CH;
  }

  chd   = chr->dma;
  width = chd->CTL0_b.SRC_TR_WIDTH;
  *num  = chd->CTL0_H << width;
  return SDMAC_OK;
}

/* interrupt handler */

static void SDMAC_IRQHandler(int32_t ch, uint32_t e)
{
  SDMAC_RESOURCE      *res = &SDMAC0_RESOURCE;
  sdmac_Type          *dma;
  SDMAC_SignalEvent_t  cb;
  uint32_t             val;

  val = 1 << ch;
  dma = sdmac;

  if (e & SDMAC_EVENT_TRANSFER) {
    dma->CLEARTFR = val;
  }
  if (e & SDMAC_EVENT_ERROR) {
    dma->CLEARERR = val;
  }

  cb = res->ch[ch].inf->cb;
  if (NULL != cb) {
    cb(ch, e);
  }
}

void SDMAC0_IRQHandler(void)
{
  uint32_t e;

  e  = BB_SDMAC_STATUSTFR(0);
  e |= BB_SDMAC_STATUSERR(0) << 4;
  SDMAC_IRQHandler(0, e);
}

void SDMAC1_IRQHandler(void)
{
  uint32_t e;

  e  = BB_SDMAC_STATUSTFR(1);
  e |= BB_SDMAC_STATUSERR(1) << 4;
  SDMAC_IRQHandler(1, e);
}

void SDMAC2_IRQHandler(void)
{
  uint32_t e;

  e  = BB_SDMAC_STATUSTFR(2);
  e |= BB_SDMAC_STATUSERR(2) << 4;
  SDMAC_IRQHandler(2, e);
}

void SDMAC3_IRQHandler(void)
{
  uint32_t e;

  e  = BB_SDMAC_STATUSTFR(3);
  e |= BB_SDMAC_STATUSERR(3) << 4;
  SDMAC_IRQHandler(3, e);
}

void SDMAC4_IRQHandler(void)
{
  uint32_t e;

  e  = BB_SDMAC_STATUSTFR(4);
  e |= BB_SDMAC_STATUSERR(4) << 4;
  SDMAC_IRQHandler(4, e);
}

void SDMAC5_IRQHandler(void)
{
  uint32_t e;

  e  = BB_SDMAC_STATUSTFR(5);
  e |= BB_SDMAC_STATUSERR(5) << 4;
  SDMAC_IRQHandler(5, e);
}

void SDMAC6_IRQHandler(void)
{
  uint32_t e;

  e  = BB_SDMAC_STATUSTFR(6);
  e |= BB_SDMAC_STATUSERR(6) << 4;
  SDMAC_IRQHandler(6, e);
}

void SDMAC7_IRQHandler(void)
{
  uint32_t e;

  e  = BB_SDMAC_STATUSTFR(7);
  e |= BB_SDMAC_STATUSERR(7) << 4;
  SDMAC_IRQHandler(7, e);
}

TZ10XX_DRIVER_SDMAC Driver_SDMAC = {
  SDMAC_GetVersion,
  SDMAC_Initialize,
  SDMAC_Uninitialize,
  SDMAC_PowerControl,
  SDMAC_ChannelInitialize,
  SDMAC_ChannelUninitialize,
  SDMAC_ChannelAcquire,
  SDMAC_ChannelRelease,
  SDMAC_ChannelTransferData,
  SDMAC_ChannelBusy,
  SDMAC_ChannelAbortTransfer,
  SDMAC_ChannelCountBytesTransferred,
};
#endif
