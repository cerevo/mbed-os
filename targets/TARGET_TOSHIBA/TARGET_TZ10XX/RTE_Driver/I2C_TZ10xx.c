/**
 * @file I2C_TZ10xx.c
 * @brief a header file for TZ10xx I2C driver
 * @date $Date:: 2015-02-06 11:22:42 +0900 #$
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
#include "Driver_I2C.h"
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

#if (RTE_I2C0 || RTE_I2C1 || RTE_I2C2)

#if   (2 == RTE_I2C0_RESISTOR)
# define I2C0_RESISTOR PMU_IO_RESISTOR_PULLUP
#elif (1 == RTE_I2C0_RESISTOR)
# define I2C0_RESISTOR PMU_IO_RESISTOR_NONE
#else
# warning "Please define RTE_I2C0_RESISTOR at RTE_Device.h."
# define I2C0_RESISTOR PMU_IO_RESISTOR_PULLUP
#endif
#if   (0 == RTE_I2C0_DRIVE_CAPABILITY)
# define I2C0_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#elif (1 == RTE_I2C0_DRIVE_CAPABILITY)
# define I2C0_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_4MA
#elif (2 == RTE_I2C0_DRIVE_CAPABILITY)
# define I2C0_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_5MA
#elif (3 == RTE_I2C0_DRIVE_CAPABILITY)
# define I2C0_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_7MA
#else
# warning "Please define RTE_I2C0_DRIVE_CAPABILITY at RTE_Device.h"
# define I2C0_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#endif

#if   (2 == RTE_I2C1_RESISTOR)
# define I2C1_RESISTOR PMU_IO_RESISTOR_PULLUP
#elif (1 == RTE_I2C1_RESISTOR)
# define I2C1_RESISTOR PMU_IO_RESISTOR_NONE
#else
# warning "Please define RTE_I2C1_RESISTOR at RTE_Device.h."
# define I2C1_RESISTOR PMU_IO_RESISTOR_PULLUP
#endif
#if   (0 == RTE_I2C1_DRIVE_CAPABILITY)
# define I2C1_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#elif (1 == RTE_I2C1_DRIVE_CAPABILITY)
# define I2C1_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_4MA
#elif (2 == RTE_I2C1_DRIVE_CAPABILITY)
# define I2C1_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_5MA
#elif (3 == RTE_I2C1_DRIVE_CAPABILITY)
# define I2C1_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_7MA
#else
# warning "Please define RTE_I2C1_DRIVE_CAPABILITY at RTE_Device.h"
# define I2C1_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#endif

#if   (2 == RTE_I2C2_RESISTOR)
# define I2C2_RESISTOR PMU_IO_RESISTOR_PULLUP
#elif (1 == RTE_I2C2_RESISTOR)
# define I2C2_RESISTOR PMU_IO_RESISTOR_NONE
#else
# warning "Please define RTE_I2C2_RESISTOR at RTE_Device.h."
# define I2C2_RESISTOR PMU_IO_RESISTOR_PULLUP
#endif
#if   (0 == RTE_I2C2_DRIVE_CAPABILITY)
# define I2C2_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#elif (1 == RTE_I2C2_DRIVE_CAPABILITY)
# define I2C2_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_4MA
#elif (2 == RTE_I2C2_DRIVE_CAPABILITY)
# define I2C2_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_5MA
#elif (3 == RTE_I2C2_DRIVE_CAPABILITY)
# define I2C2_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_7MA
#else
# warning "Please define RTE_I2C2_DRIVE_CAPABILITY at RTE_Device.h"
# define I2C2_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#endif

#if (RTE_I2C0_DMA_RX || RTE_I2C1_DMA_RX || RTE_I2C2_DMA_RX)
# define I2C_DMA_RX   1
#endif

#if (I2C_DMA_RX)
# if (!RTE_SDMAC)
#  error "SDMAC is disabled. TZ10XX I2C driver needs SDMAC for DMA function."
# endif
# include "SDMAC_TZ10xx.h"
#endif

#define UNUSED_VALUE(_x)           ((void)(_x))

#define ARM_I2C_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,22)   /* driver version */

/* bitband alias */
#define BB_I2C_EN(_x)              BITBAND_VALUE(&((_x)->IC_ENABLE_STATUS),0)
#define BB_I2C_ABORT(_x)           BITBAND_VALUE(&((_x)->IC_ENABLE),1)
#define BB_I2C_RDMAE(_x)           BITBAND_VALUE(&((_x)->IC_DMA_CR),0)

#define I2C_STANDARD_SPEED_FREQ    (100000)
#define I2C_LOW_PERIOD_US          (5)
#define I2C_US_UNIT                (1000000)

#define I2C_FIFO_DEPTH             (6)

#define I2C_CON_MASTER_ENABLE      (1)
#define I2C_CON_SLAVE_DISABLE      (1<<6)
#define I2C_CON_RESTART_ENABLE     (1<<5)
#define I2C_CON_SPEED_STANDARD     (1<<1)
#define I2C_CON_SPEED_FAST         (2<<1)

#define I2C_TAR_10BITADDR          (1<<12)

#define I2C_ENABLE                 (1)
#define I2C_DISABLE                (0)
#define I2C_ABORT                  (3)

#define I2C_INTR_MST_ON_HOLD       (1<<13)
#define I2C_INTR_STOP_DET          (1<<9)
#define I2C_INTR_TX_ABRT           (1<<6)
#define I2C_INTR_TX_EMPTY          (1<<4)
#define I2C_INTR_TX_OVER           (1<<3)
#define I2C_INTR_RX_FULL           (1<<2)
#define I2C_INTR_RX_OVER           (1<<1)
#define I2C_INTR_RX_UNDER          (1<<0)
#define I2C_DATA_RESTART           (1<<10)
#define I2C_DATA_STOP              (1<<9)
#define I2C_CMD_WRITE              (0<<8)
#define I2C_CMD_READ               (1<<8)

#define I2C_TX_FLUSH_CNT(_x)       ((_x)>>24)
#define I2C_ABRT_USER_ABRT         (1<<16)
#define I2C_ARB_LOST               (1<<12)
#define I2C_ABRT_TXDATA_NOACK      (1<<3)
#define I2C_ABRT_10ADDR2_NOACK     (1<<2)
#define I2C_ABRT_10ADDR1_NOACK     (1<<1)
#define I2C_ABRT_7B_ADDR_NOACK     (1<<0)
#define I2C_RX_OVERFLOW            (1<<17)

#if I2C_DMA_RX
extern TZ10XX_DRIVER_SDMAC  Driver_SDMAC;
# define DRIVER_SDMAC       Driver_SDMAC
# define I2C_LENGTH2MSIZE(len)      ((len & 3) ? SDMAC_MSIZE_1 : SDMAC_MSIZE_4)
# define I2C_MSIZE2RX_THRESHOLD(m)  ((m) ? 3 : 0)
#endif

typedef void (*I2C_ResetOn)(void);
typedef void (*I2C_ResetOff)(void);

/* I2C Information (Run-Time) */
typedef struct {
  bool                    init;           //< Initialized flag
  ARM_POWER_STATE         power;          //< Power mode
  volatile bool           busy;           //< Data transferring flag
  ARM_I2C_SignalEvent_t   cb_event;       //< Event Callback
  uint32_t                speed;          //< IC_CON.SPEED register fieldvalue
  uint32_t                cnt;            //< Tx or Rx buffer counter
  uint8_t                *buf;            //< Tx or Rx buffer pointer
  uint32_t                stop;           //< Tx or Rx buffer byte stop flag
  uint32_t                dmy;            //< dummy write counter for Tx buffer
  uint32_t                err;
} I2C_INFO;

#if I2C_DMA_RX
typedef struct {
  int8_t              peri;               // Hardware ID
  int8_t              ch;                 // DMA channel
  int8_t              hs;                 // Handshake channel
} I2C_DMA_RESOURCES;
#endif

/* I2C Resources definition */
typedef struct {
  i2c_Type           *i2c;                //< Pointer to I2C peripheral
  IRQn_Type           irq;                //< I2C IRQ Number
  I2C_ResetOff        ResetOff;           //< I2C Reset Off
  I2C_ResetOn         ResetOn;            //< I2C Reset On
  PMU_CD              pmu_cd;             //< Peripheral bus clock
  I2C_INFO           *info;               //< Run-Time information
#if I2C_DMA_RX
  SDMAC_SignalEvent_t dma;                //< DMA Event handler
  I2C_DMA_RESOURCES   rx;                 //< DMA Resources for recept
#endif
} const I2C_RESOURCES;

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_I2C_API_VERSION,
  ARM_I2C_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_I2C_CAPABILITIES DriverCapabilities = { 0 };


static ARM_I2C_STATUS I2C_PowerControl(ARM_POWER_STATE state, I2C_RESOURCES *res);

static void I2C_ClearInfo(I2C_RESOURCES *res)
{
  I2C_INFO *info = res->info;
  info->speed = 0;
  info->cnt   = 0;
  info->buf   = NULL;
  info->stop  = 0;
  info->dmy   = 0;
  info->err   = 0;
}

static ARM_I2C_STATUS I2C_SetupBusSpeed(ARM_I2C_BUS_SPEED speed, I2C_RESOURCES *res)
{
  i2c_Type *i2c;
  uint32_t  freq;
  uint32_t  clk, hcnt, lcnt;

  freq = Driver_PMU.GetFrequency(res->pmu_cd);
  switch (speed) {
  case ARM_I2C_BUS_SPEED_STANDARD:
    if (2700000 <= freq) {
      clk  = (freq + (I2C_STANDARD_SPEED_FREQ - 1)) / I2C_STANDARD_SPEED_FREQ;
      lcnt = (I2C_LOW_PERIOD_US * freq + (I2C_US_UNIT - 1)) / I2C_US_UNIT;
      lcnt = (lcnt < 9)  ?  9 : lcnt;
      hcnt = clk - lcnt;
      hcnt = (hcnt < 14) ? 14 : hcnt;
    } else {
      hcnt = 6  + 8;
      lcnt = 12 + 1;
    }
    i2c  = res->i2c;
    i2c->IC_SS_SCL_HCNT = hcnt - 8;
    i2c->IC_SS_SCL_LCNT = lcnt - 1;
    res->info->speed    = I2C_CON_SPEED_STANDARD;
    break;
  case ARM_I2C_BUS_SPEED_FAST:
    i2c                 = res->i2c;
    i2c->IC_FS_SCL_HCNT = 6;
    i2c->IC_FS_SCL_LCNT = 15;
    res->info->speed    = I2C_CON_SPEED_FAST;
    break;
  default:
    return ARM_I2C_ERROR_UNSUPPORTED;
  }
  return ARM_I2C_OK;
}

static ARM_DRIVER_VERSION I2CX_GetVersion(void)
{
  return DriverVersion;
}

static ARM_I2C_CAPABILITIES I2CX_GetCapabilities(void)
{
  return DriverCapabilities;
}


static ARM_I2C_STATUS I2C_Initialize(ARM_I2C_SignalEvent_t cb_event, I2C_RESOURCES *res)
{
  I2C_INFO *info;

  info = res->info;
  if (info->init) {
    return ARM_I2C_ERROR;
  }
  /* Clear and Enable I2C IRQ */
  NVIC_ClearPendingIRQ(res->irq);
  NVIC_EnableIRQ(res->irq);

  /* Initialize I2C Run-Time Resources */
  I2C_ClearInfo(res);

  info->power    = ARM_POWER_OFF;
  info->cb_event = cb_event;
  info->init     = true;
  return I2C_PowerControl(ARM_POWER_LOW, res);
}

static ARM_I2C_STATUS I2C_Uninitialize(I2C_RESOURCES *res)
{
  I2C_INFO *info;

  info = res->info;
  if (false == info->init) {
    return ARM_I2C_OK;
  }
  I2C_PowerControl(ARM_POWER_OFF, res);

  // Disable I2C IRQ
  NVIC_DisableIRQ(res->irq);
  NVIC_ClearPendingIRQ(res->irq);

  // Uninitialize I2C Run-Time Resources
  info->cb_event = NULL;
  info->init     = false;
  return ARM_I2C_OK;
}

static ARM_I2C_STATUS I2C_PowerControl(ARM_POWER_STATE state, I2C_RESOURCES *res)
{
  I2C_INFO  *info;
  i2c_Type *i2c;
  ARM_POWER_STATE prv;

  info = res->info;
  if (false == info->init) return ARM_I2C_ERROR;

  prv = info->power;
  if (prv == state) {
    return ARM_I2C_OK;
  }
  i2c = res->i2c;
  switch (state) {
  case ARM_POWER_OFF:
    if (prv == ARM_POWER_FULL) {
      // Disable I2C
      BB_I2C_ABORT(i2c) = 1;
      while (BB_I2C_EN(i2c)) ;
    }
    res->ResetOn();
    // Update I2C Run-Time Resources
    I2C_ClearInfo(res);
    break;
  case ARM_POWER_LOW:
    if (prv == ARM_POWER_FULL) {
      // Disable I2C
      BB_I2C_ABORT(i2c) = 1;
      while (BB_I2C_EN(i2c)) ;
    } else {
      res->ResetOff();
      i2c->IC_INTR_MASK = I2C_INTR_TX_ABRT;
    }
    break;
  case ARM_POWER_FULL:
    res->ResetOff();
    i2c->IC_INTR_MASK = I2C_INTR_TX_ABRT;
    I2C_SetupBusSpeed(ARM_I2C_BUS_SPEED_FAST, res);
    break;
  default:
    return ARM_I2C_ERROR_UNSUPPORTED;
  }
  info->power = state;
  return ARM_I2C_OK;
}

static ARM_I2C_STATUS I2C_BusSpeed(ARM_I2C_BUS_SPEED speed, I2C_RESOURCES *res)
{
  I2C_INFO      *info;
  i2c_Type      *i2c;

  info = res->info;
  if (ARM_POWER_FULL != info->power) {
    return ARM_I2C_ERROR;
  }
  if ((ARM_I2C_BUS_SPEED_STANDARD != speed) &&
      (ARM_I2C_BUS_SPEED_FAST     != speed)) {
    return ARM_I2C_ERROR_UNSUPPORTED;
  }
  i2c  = res->i2c;
  if (BB_I2C_EN(i2c)) {
    return ARM_I2C_ERROR_BUS_BUSY;
  }
  return I2C_SetupBusSpeed(speed, res);
}
 
static ARM_I2C_STATUS I2C_BusClear(I2C_RESOURCES *res)
{
  return ARM_I2C_ERROR_UNSUPPORTED;
}
#if defined(__CC_ARM)
/* Use optimization O1 */
#pragma push
#pragma O1
#endif
static int32_t I2C_SendData(uint32_t addr, const uint8_t *data, uint32_t size, bool xfer_pending, I2C_RESOURCES *res)
{
  uint32_t i;
  I2C_INFO *info;
  i2c_Type *i2c;
  uint32_t  th, umsk;
  uint32_t  i2c_10bit;
  uint32_t  flag;
  uint32_t  cnt;
  uint32_t  err;
  uint32_t  speed;

  info = res->info;
  if (ARM_POWER_FULL != info->power) {
    return -ARM_I2C_ERROR;
  }
  if ((0 == size) || (4096 < size)){
    return -ARM_I2C_ERROR;
  }
  speed = info->speed;
  if ((I2C_CON_SPEED_STANDARD != speed) &&
      (I2C_CON_SPEED_FAST     != speed)) {
    return -ARM_I2C_ERROR;
  }
  if (addr & ARM_I2C_ADDRESS_10BIT) {
    i2c_10bit = I2C_TAR_10BITADDR;
  } else {
    i2c_10bit = 0;
  }

  umsk = I2C_INTR_STOP_DET | I2C_INTR_TX_ABRT | I2C_INTR_RX_OVER;
  if (xfer_pending) {
    umsk      |= I2C_INTR_MST_ON_HOLD;
    flag       = I2C_CMD_WRITE;
    info->stop = I2C_CMD_WRITE;
  } else {
    flag       = I2C_CMD_WRITE | I2C_DATA_STOP;
    info->stop = I2C_CMD_WRITE | I2C_DATA_STOP;
  }

  if (I2C_FIFO_DEPTH < size) {
    th        = I2C_FIFO_DEPTH / 2;
    umsk     |= I2C_INTR_TX_EMPTY;
    cnt       = I2C_FIFO_DEPTH;
    info->buf = (uint8_t*)data + I2C_FIFO_DEPTH;
    flag     &= ~I2C_DATA_STOP;
  } else {
    th        = 0;
    cnt       = size;
    info->buf = NULL;
  }
  info->cnt = size - cnt;
  info->dmy = 0;
  info->err = 0;

  i2c = res->i2c;
  i2c->IC_INTR_MASK = 0;
  i2c->IC_CON       = info->speed | I2C_CON_MASTER_ENABLE |
                      I2C_CON_SLAVE_DISABLE | I2C_CON_RESTART_ENABLE;
  i2c->IC_TAR       = i2c_10bit | (addr & 0x3FF);
  i2c->IC_TX_TL     = th;
  i2c->IC_ENABLE    = I2C_ENABLE;
  info->busy        = true;

  if (1 == cnt) {
    i2c->IC_DATA_CMD = I2C_DATA_RESTART | flag | *data;
  } else {
    i2c->IC_DATA_CMD = I2C_DATA_RESTART | I2C_CMD_WRITE | *data++;
    for (i = 1; i < cnt - 1; ++i) {
      i2c->IC_DATA_CMD = I2C_CMD_WRITE | *data++;
    }
    i2c->IC_DATA_CMD = flag | *data;
  }
  i2c->IC_INTR_MASK = umsk;

  /* wait until done. */
  while (info->busy) ;

  if (!xfer_pending) {
    i2c->IC_ENABLE = I2C_DISABLE;
    while (BB_I2C_EN(i2c)) ;
  }

  err = info->err;
  if (err) {
    if (err & (I2C_ABRT_10ADDR2_NOACK |
               I2C_ABRT_10ADDR1_NOACK |
               I2C_ABRT_7B_ADDR_NOACK)) {
      return -ARM_I2C_ERROR_NO_SLAVE;
    }
    if (err & I2C_ARB_LOST) {
      return -ARM_I2C_ERROR_BUS_BUSY;
    }
    if (err & (I2C_ABRT_TXDATA_NOACK |
               I2C_ABRT_USER_ABRT)) {
      info->cnt += I2C_TX_FLUSH_CNT(err);
    }
    while (BB_I2C_EN(i2c)) ;
  }
  return size - info->cnt;
}

static int32_t I2C_ReceiveData(uint32_t addr, uint8_t *data, uint32_t size, bool xfer_pending, I2C_RESOURCES *res)
{
  uint32_t i;
  I2C_INFO *info;
  i2c_Type *i2c;
  uint32_t  th, umsk;
  uint32_t  i2c_10bit;
  uint32_t  flag;
  uint32_t  cnt;
  uint32_t  err;
  uint32_t  speed;
#if I2C_DMA_RX
  uint32_t  msize;
  uint32_t  ctl, num;
  int32_t   rx_ch;
#endif

  info = res->info;
  if (ARM_POWER_FULL != info->power) {
    return -ARM_I2C_ERROR;
  }
  if ((0 == size) || (4096 < size)) {
    return -ARM_I2C_ERROR;
  }
  speed = info->speed;
  if ((I2C_CON_SPEED_STANDARD != speed) &&
      (I2C_CON_SPEED_FAST     != speed)) {
    return -ARM_I2C_ERROR;
  }
  if (addr & ARM_I2C_ADDRESS_10BIT) {
    i2c_10bit = I2C_TAR_10BITADDR;
  } else {
    i2c_10bit = 0;
  }

  i2c = res->i2c;
#if I2C_DMA_RX
  /* initialize dma rx channel. */
  msize = I2C_LENGTH2MSIZE(size);
  rx_ch = res->rx.ch;
  if (0 <= rx_ch) {
    err = DRIVER_SDMAC.ChannelInitialize(
      rx_ch, res->dma, (SDMAC_SRC)res->rx.peri, SDMAC_DST_MEMORY,
      SDMAC_FIFO_MODE_UTILIZATION, SDMAC_PRIORITY_LOWEST,
      res->rx.hs, 0);
    if (err) {
      return -ARM_I2C_ERROR;
    }
    ctl = SDMAC_CTL(SDMAC_WIDTH_1, SDMAC_INC_NO_CHANGE, msize,
                    SDMAC_WIDTH_1, SDMAC_INC_INCREMENT, SDMAC_MSIZE_16);
    err = DRIVER_SDMAC.ChannelTransferData(
      rx_ch, (const uint8_t*)&i2c->IC_DATA_CMD, data, size, ctl);
    if (err) {
      DRIVER_SDMAC.ChannelUninitialize(rx_ch);
      return -ARM_I2C_ERROR;
    }
    BB_I2C_RDMAE(i2c) = 1;
    i2c->IC_DMA_RDLR  = I2C_MSIZE2RX_THRESHOLD(msize);
    umsk = I2C_INTR_STOP_DET | I2C_INTR_TX_ABRT | I2C_INTR_RX_OVER;
  } else {
#endif
    umsk = I2C_INTR_STOP_DET | I2C_INTR_RX_FULL | I2C_INTR_TX_ABRT | I2C_INTR_RX_OVER;
#if I2C_DMA_RX
  }
#endif

  if (xfer_pending) {
    umsk      |= I2C_INTR_MST_ON_HOLD;
    flag       = I2C_CMD_READ;
    info->stop = I2C_CMD_READ;
  } else {
    flag       = I2C_CMD_READ | I2C_DATA_STOP;
    info->stop = I2C_CMD_READ | I2C_DATA_STOP;
  }

  if (I2C_FIFO_DEPTH < size) {
    th    = I2C_FIFO_DEPTH / 2;
    cnt   = I2C_FIFO_DEPTH;
    umsk |= I2C_INTR_TX_EMPTY;
    flag &= ~I2C_DATA_STOP;
  } else {
    th    = size;
    cnt   = size;
  }
  info->buf = data;
  info->cnt = size;
  info->dmy = size - cnt;
  info->err = 0;

  i2c->IC_INTR_MASK = 0;
  i2c->IC_CON       = info->speed | I2C_CON_MASTER_ENABLE |
                      I2C_CON_SLAVE_DISABLE | I2C_CON_RESTART_ENABLE;
  i2c->IC_TAR       = i2c_10bit | (addr & 0x3FF);
  i2c->IC_TX_TL     = th;
  i2c->IC_RX_TL     = th - 1;
  i2c->IC_ENABLE    = I2C_ENABLE;
  info->busy        = true;

  if (1 == cnt) {
    i2c->IC_DATA_CMD = I2C_DATA_RESTART | flag;
  } else {
    i2c->IC_DATA_CMD = I2C_DATA_RESTART | I2C_CMD_READ;
    for (i = 1; i < cnt - 1; ++i) {
      i2c->IC_DATA_CMD = I2C_CMD_READ;
    }
    i2c->IC_DATA_CMD = flag;
  }
  i2c->IC_INTR_MASK = umsk;

  /* wait until done. */
  while (info->busy) ;

  if (!xfer_pending) {
    i2c->IC_ENABLE = I2C_DISABLE;
    while (BB_I2C_EN(i2c)) ;
  }

  err = info->err;
  if (err) {
    if (err & (I2C_ABRT_10ADDR2_NOACK |
               I2C_ABRT_10ADDR1_NOACK |
               I2C_ABRT_7B_ADDR_NOACK)) {
      return -ARM_I2C_ERROR_NO_SLAVE;
    }
    if (err & I2C_ARB_LOST) {
      return -ARM_I2C_ERROR_BUS_BUSY;
    }
    if (err & I2C_RX_OVERFLOW) {
      return -ARM_I2C_ERROR;
    }
#if I2C_DMA_RX
    if (err & (I2C_ABRT_TXDATA_NOACK |
               I2C_ABRT_USER_ABRT)) {
      rx_ch = res->rx.ch;
      if ((0 <= rx_ch) && (0 < info->cnt)) {
        num = 0;
        DRIVER_SDMAC.ChannelAbortTransfer(rx_ch);
        DRIVER_SDMAC.ChannelCountBytesTransferred(rx_ch, &num);
        DRIVER_SDMAC.ChannelUninitialize(rx_ch);
        if (0 < info->cnt) {
          info->cnt -= num;
        }
      }
    }
#endif
    while (BB_I2C_EN(i2c)) ;
  }
  return size - info->cnt;
}


static ARM_I2C_STATUS I2C_AbortTransfer(I2C_RESOURCES *res)
{
  i2c_Type *i2c;
  I2C_INFO *info;

  info = res->info;
  if (ARM_POWER_FULL != info->power) {
    return ARM_I2C_ERROR;
  }
  i2c = res->i2c;
  BB_I2C_ABORT(i2c) = 1;
  info->busy        = false;

  /* wait until inactive i2c. */
  while (BB_I2C_EN(i2c)) ;

  return ARM_I2C_OK;
}

#if I2C_DMA_RX
static void I2C_DMARxHandler(I2C_RESOURCES *res, uint32_t e)
{
  I2C_INFO *info;
  i2c_Type *i2c;

  info = res->info;
  if (e & SDMAC_EVENT_TRANSFER) {
    info->buf = NULL;
    info->cnt = 0;
    i2c = res->i2c;
    BB_I2C_RDMAE(i2c) = 0;
    if (I2C_DISABLE == i2c->IC_ENABLE) {
      info->busy = false;
    }
  }
}
#endif

static void I2C_IRQHandler(I2C_RESOURCES *res)
{
  uint32_t i;
  i2c_Type *i2c;
  I2C_INFO *info;
  uint32_t  isr;
  uint32_t  dmy;
  uint32_t  cnt, lv, loop, gap;
  uint8_t  *buf;
  uint32_t  abrt;

  i2c  = res->i2c;
  info = res->info;
  buf  = info->buf;

  isr  = i2c->IC_INTR_STAT;
  if (isr & I2C_INTR_TX_ABRT) {
    abrt = i2c->IC_TX_ABRT_SOURCE;
    dmy  = i2c->IC_CLR_TX_ABRT;
    if (0 == info->err) {
      info->err  = abrt;
    }
    if (abrt & I2C_ABRT_USER_ABRT) {
      info->busy = false;
    } else {
      BB_I2C_ABORT(i2c) = 1;
    }
    return;
  }
  if (isr & I2C_INTR_RX_OVER) {
    dmy = i2c->IC_CLR_RX_OVER;
    i2c->IC_ENABLE = I2C_DISABLE;
    info->busy = false;
    info->err  = I2C_RX_OVERFLOW;
    return;
  }
  if (isr & I2C_INTR_RX_FULL) {
    lv  = i2c->IC_RXFLR;
    for (i = 0; i < lv; ++i) {
      *buf++ = i2c->IC_DATA_CMD;
    }
    info->buf = buf;
    cnt       = info->cnt - lv;
    info->cnt = cnt;
    if (0 == cnt) {
      info->buf = NULL;
      i2c->IC_INTR_MASK &= ~I2C_INTR_RX_FULL;
    } else if (cnt <= I2C_FIFO_DEPTH) {
      i2c->IC_RX_TL = cnt - 1;
    }
  }

  if (isr & I2C_INTR_TX_EMPTY) {
    cnt = info->dmy;
    if (cnt) {
      /* write dummy data for receiving. */
      lv   = I2C_FIFO_DEPTH - i2c->IC_TXFLR;
      loop = (lv < cnt) ? lv : cnt;
#if I2C_DMA_RX
      if (res->rx.ch < 0) {
#endif
        gap = info->cnt - cnt;
        if (I2C_FIFO_DEPTH <= gap) {
          /* To prevent rx fifo overflow, retry after receiving rx fifo. */
          loop = 0;
        } else if (I2C_FIFO_DEPTH < gap + loop) {
          /* prevent rx fifo overflow */
          loop = gap + loop - I2C_FIFO_DEPTH;
        }
#if I2C_DMA_RX
      }
#endif
      if (loop) {
        cnt      -= loop;
        info->dmy = cnt;
        if (cnt) {
          for (i = 0; i < loop; ++i) {
            i2c->IC_DATA_CMD = I2C_CMD_READ;
          }
        } else {
          for (i = 0; i < loop - 1; ++i) {
            i2c->IC_DATA_CMD = I2C_CMD_READ;
          }
          i2c->IC_DATA_CMD   = info->stop;
          i2c->IC_INTR_MASK &= ~I2C_INTR_TX_EMPTY;
        }
      } else {
      }
    } else {
      cnt       = info->cnt;
      lv        = I2C_FIFO_DEPTH - i2c->IC_TXFLR;
      loop      = (lv < cnt) ? lv : cnt;
      cnt      -= loop;
      info->cnt = cnt;
      if (cnt) {
        for (i = 0; i < loop; ++i) {
          i2c->IC_DATA_CMD = I2C_CMD_WRITE | *buf++;
        }
        info->buf = buf;
      } else {
        /* write data until the end of the data. */
        for (i = 0; i < loop - 1; ++i) {
          i2c->IC_DATA_CMD = I2C_CMD_WRITE | *buf++;
        }
        i2c->IC_DATA_CMD   = info->stop | *buf;
        i2c->IC_INTR_MASK &= ~I2C_INTR_TX_EMPTY;
        info->buf          = NULL;
      }
    }
  }

  if (isr & I2C_INTR_MST_ON_HOLD) {
    if (NULL == info->buf) {
      /* finish of transferring data without stop condition. */
      info->busy = false;
      i2c->IC_INTR_MASK &= ~I2C_INTR_MST_ON_HOLD;
    } else {
      /* transmit fifo is underflowed, nothing to do. */
    }
  }
  if (isr & I2C_INTR_STOP_DET) {
    /* finish of transferring all data. */
    dmy = i2c->IC_CLR_STOP_DET;
    i2c->IC_ENABLE = I2C_DISABLE;
    if (NULL == info->buf) {
      info->busy = false;
    }
  }
  UNUSED_VALUE(dmy);
}
#if defined(__CC_ARM)
#pragma pop
/* End #pragma O1*/
#endif

#if (RTE_I2C0)
#if (RTE_I2C0_DMA_RX)
static void I2C0_DMAHandler(uint32_t ch, uint32_t e);
#endif

static void I2C0_ResetOn(void)
{
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_I2C0_CLK,  1);
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_I2C0_DATA, 1);
  Driver_PMU.EnableModule(PMU_MODULE_I2C0, 0);
}

static void I2C0_ResetOff(void)
{
  Driver_PMU.EnableModule(PMU_MODULE_I2C0, 1);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_I2C0_DATA, I2C0_DRIVE_CAPABILITY, I2C0_RESISTOR);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_I2C0_CLK,  I2C0_DRIVE_CAPABILITY, I2C0_RESISTOR);
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_I2C0_DATA, 0);
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_I2C0_CLK,  0);
}

/* I2C0 Information (Run-Time) */
static I2C_INFO I2C0_Info;

/* I2C0 Resources */
I2C_RESOURCES I2C0_Resources = {
  i2c0,
  I2C0_IRQn,
  I2C0_ResetOff,
  I2C0_ResetOn,
  PMU_CD_PPIER1,
  &I2C0_Info,
#if I2C_DMA_RX
# if (RTE_I2C0_DMA_RX)
  I2C0_DMAHandler,
  {SDMAC_SRC_I2C0, RTE_I2C0_DMA_RX_CH, RTE_I2C0_DMA_RX_HS},
# else
  NULL,
  {SDMAC_SRC_I2C0, -1, -1},
# endif
#endif
};

static ARM_I2C_STATUS I2C0_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
  return I2C_Initialize(cb_event, &I2C0_Resources);
}

static ARM_I2C_STATUS I2C0_Uninitialize(void)
{
  return I2C_Uninitialize(&I2C0_Resources);
}

static ARM_I2C_STATUS I2C0_PowerControl(ARM_POWER_STATE state)
{
  return I2C_PowerControl(state, &I2C0_Resources);
}

static ARM_I2C_STATUS I2C0_BusSpeed(ARM_I2C_BUS_SPEED speed)
{
  return I2C_BusSpeed(speed, &I2C0_Resources);
}
 
static ARM_I2C_STATUS I2C0_BusClear(void)
{
  return I2C_BusClear(&I2C0_Resources);
}

static int32_t I2C0_SendData(uint32_t addr, const uint8_t *data, uint32_t size, bool xfer_pending)
{
  return I2C_SendData(addr, data, size, xfer_pending, &I2C0_Resources);
}

static int32_t I2C0_ReceiveData(uint32_t addr, uint8_t *data, uint32_t size, bool xfer_pending)
{
  return I2C_ReceiveData(addr, data, size, xfer_pending, &I2C0_Resources);
}

static ARM_I2C_STATUS I2C0_AbortTransfer(void)
{
  return I2C_AbortTransfer(&I2C0_Resources);
}

void I2C0_IRQHandler(void)
{
  I2C_IRQHandler(&I2C0_Resources);
}

#if (RTE_I2C0_DMA_RX)
static void I2C0_DMAHandler(uint32_t ch, uint32_t e)
{
  I2C_DMARxHandler(&I2C0_Resources, e);
  DRIVER_SDMAC.ChannelUninitialize(ch);
}
#endif

ARM_DRIVER_I2C Driver_I2C0 = {
  I2CX_GetVersion,
  I2CX_GetCapabilities,
  I2C0_Initialize,
  I2C0_Uninitialize,
  I2C0_PowerControl,
  I2C0_BusSpeed,
  I2C0_BusClear,
  I2C0_SendData,
  I2C0_ReceiveData,
  I2C0_AbortTransfer,
};
#endif

#if (RTE_I2C1)
#if (RTE_I2C1_DMA_RX)
static void I2C1_DMAHandler(uint32_t ch, uint32_t e);
#endif

static void I2C1_ResetOn(void)
{
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_I2C1_CLK,  1);
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_I2C1_DATA, 1);
  Driver_PMU.EnableModule(PMU_MODULE_I2C1, 0);
}

static void I2C1_ResetOff(void)
{
  Driver_PMU.EnableModule(PMU_MODULE_I2C1, 1);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_I2C1_DATA, I2C1_DRIVE_CAPABILITY, I2C1_RESISTOR);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_I2C1_CLK,  I2C1_DRIVE_CAPABILITY, I2C1_RESISTOR);
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_I2C1_DATA, 0);
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_I2C1_CLK,  0);
}

/* I2C1 Information (Run-Time) */
static I2C_INFO I2C1_Info;

/* I2C1 Resources */
I2C_RESOURCES I2C1_Resources = {
  i2c1,
  I2C1_IRQn,
  I2C1_ResetOff,
  I2C1_ResetOn,
  PMU_CD_PPIER1,
  &I2C1_Info,
#if I2C_DMA_RX
# if (RTE_I2C1_DMA_RX)
  I2C1_DMAHandler,
  {SDMAC_SRC_I2C1, RTE_I2C1_DMA_RX_CH, RTE_I2C1_DMA_RX_HS},
# else
  NULL,
  {SDMAC_SRC_I2C1, -1, -1},
# endif
#endif
};

static ARM_I2C_STATUS I2C1_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
  return I2C_Initialize(cb_event, &I2C1_Resources);
}

static ARM_I2C_STATUS I2C1_Uninitialize(void)
{
  return I2C_Uninitialize(&I2C1_Resources);
}

static ARM_I2C_STATUS I2C1_PowerControl(ARM_POWER_STATE state)
{
  return I2C_PowerControl(state, &I2C1_Resources);
}

static ARM_I2C_STATUS I2C1_BusSpeed(ARM_I2C_BUS_SPEED speed)
{
  return I2C_BusSpeed(speed, &I2C1_Resources);
}
 
static ARM_I2C_STATUS I2C1_BusClear(void)
{
  return I2C_BusClear(&I2C1_Resources);
}

static int32_t I2C1_SendData(uint32_t addr, const uint8_t *data, uint32_t size, bool xfer_pending)
{
  return I2C_SendData(addr, data, size, xfer_pending, &I2C1_Resources);
}

static int32_t I2C1_ReceiveData(uint32_t addr, uint8_t *data, uint32_t size, bool xfer_pending)
{
  return I2C_ReceiveData(addr, data, size, xfer_pending, &I2C1_Resources);
}

static ARM_I2C_STATUS I2C1_AbortTransfer(void)
{
  return I2C_AbortTransfer(&I2C1_Resources);
}

void I2C1_IRQHandler(void)
{
  I2C_IRQHandler(&I2C1_Resources);
}

#if (RTE_I2C1_DMA_RX)
static void I2C1_DMAHandler(uint32_t ch, uint32_t e)
{
  I2C_DMARxHandler(&I2C1_Resources, e);
  DRIVER_SDMAC.ChannelUninitialize(ch);
}
#endif

ARM_DRIVER_I2C Driver_I2C1 = {
  I2CX_GetVersion,
  I2CX_GetCapabilities,
  I2C1_Initialize,
  I2C1_Uninitialize,
  I2C1_PowerControl,
  I2C1_BusSpeed,
  I2C1_BusClear,
  I2C1_SendData,
  I2C1_ReceiveData,
  I2C1_AbortTransfer,
};
#endif

#if (RTE_I2C2)
#if (RTE_I2C2_DMA_RX)
static void I2C2_DMAHandler(uint32_t ch, uint32_t e);
#endif

static void I2C2_ResetOn(void)
{
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_I2C2_CLK,  1);
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_I2C2_DATA, 1);
  Driver_PMU.EnableModule(PMU_MODULE_I2C2, 0);
}

static void I2C2_ResetOff(void)
{
  Driver_PMU.EnableModule(PMU_MODULE_I2C2, 1);
#if (1 == RTE_I2C2_DATA_ID)
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_I2C2_DATA, I2C2_DRIVE_CAPABILITY, PMU_IO_RESISTOR_PULLUP);
#else
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_I2C2_DATA, I2C2_DRIVE_CAPABILITY, I2C2_RESISTOR);
#endif
#if (1 == RTE_I2C2_CLK_ID)
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_I2C2_CLK,  I2C2_DRIVE_CAPABILITY, PMU_IO_RESISTOR_PULLUP);
#else
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_I2C2_CLK,  I2C2_DRIVE_CAPABILITY, I2C2_RESISTOR);
#endif
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_I2C2_DATA, 0);
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_I2C2_CLK,  0);
}

/* I2C2 Information (Run-Time) */
static I2C_INFO I2C2_Info;

/* I2C2 Resources */
I2C_RESOURCES I2C2_Resources = {
  i2c2,
  I2C2_IRQn,
  I2C2_ResetOff,
  I2C2_ResetOn,
  PMU_CD_PPIER0,
  &I2C2_Info,
#if I2C_DMA_RX
# if (RTE_I2C2_DMA_RX)
  I2C2_DMAHandler,
  {SDMAC_SRC_I2C2, RTE_I2C2_DMA_RX_CH, RTE_I2C2_DMA_RX_HS},
# else
  NULL,
  {SDMAC_SRC_I2C2, -1, -1},
# endif
#endif
};

static ARM_I2C_STATUS I2C2_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
  return I2C_Initialize(cb_event, &I2C2_Resources);
}

static ARM_I2C_STATUS I2C2_Uninitialize(void)
{
  return I2C_Uninitialize(&I2C2_Resources);
}

static ARM_I2C_STATUS I2C2_PowerControl(ARM_POWER_STATE state)
{
  return I2C_PowerControl(state, &I2C2_Resources);
}

static ARM_I2C_STATUS I2C2_BusSpeed(ARM_I2C_BUS_SPEED speed)
{
  return I2C_BusSpeed(speed, &I2C2_Resources);
}
 
static ARM_I2C_STATUS I2C2_BusClear(void)
{
  return I2C_BusClear(&I2C2_Resources);
}

static int32_t I2C2_SendData(uint32_t addr, const uint8_t *data, uint32_t size, bool xfer_pending)
{
  return I2C_SendData(addr, data, size, xfer_pending, &I2C2_Resources);
}

static int32_t I2C2_ReceiveData(uint32_t addr, uint8_t *data, uint32_t size, bool xfer_pending)
{
  return I2C_ReceiveData(addr, data, size, xfer_pending, &I2C2_Resources);
}

static ARM_I2C_STATUS I2C2_AbortTransfer(void)
{
  return I2C_AbortTransfer(&I2C2_Resources);
}

void I2C2_IRQHandler(void)
{
  I2C_IRQHandler(&I2C2_Resources);
}

#if (RTE_I2C2_DMA_RX)
static void I2C2_DMAHandler(uint32_t ch, uint32_t e)
{
  I2C_DMARxHandler(&I2C2_Resources, e);
  DRIVER_SDMAC.ChannelUninitialize(ch);
}
#endif

ARM_DRIVER_I2C Driver_I2C2 = {
  I2CX_GetVersion,
  I2CX_GetCapabilities,
  I2C2_Initialize,
  I2C2_Uninitialize,
  I2C2_PowerControl,
  I2C2_BusSpeed,
  I2C2_BusClear,
  I2C2_SendData,
  I2C2_ReceiveData,
  I2C2_AbortTransfer,
};
#endif
#endif
