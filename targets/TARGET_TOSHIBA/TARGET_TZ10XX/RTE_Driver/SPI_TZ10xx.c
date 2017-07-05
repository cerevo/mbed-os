/**
 * @file SPI_TZ10xx.c
 * @brief a header file for TZ10xx SPI driver
 * @date $Date:: 2016-03-23 09:46:12 +0900 #$
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
#include "SPI_TZ10xx.h"
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

#if (RTE_SPI0 || RTE_SPI1 || RTE_SPI2 || RTE_SPI3)

#if (RTE_SPIM0_DMA_TX || RTE_SPIM1_DMA_TX || RTE_SPIM2_DMA_TX || RTE_SPIM3_DMA_TX)
# define SPI_DMA_TX   1
#endif
#if (RTE_SPIM0_DMA_RX || RTE_SPIM1_DMA_RX || RTE_SPIM2_DMA_RX || RTE_SPIM3_DMA_RX)
# define SPI_DMA_RX   1
#endif

#if (SPI_DMA_TX || SPI_DMA_RX)
# if (!RTE_SDMAC)
#  error "SDMAC is disabled. TZ10XX SPI driver needs SDMAC for DMA function."
# endif
# include "SDMAC_TZ10xx.h"
# define SPI_WITH_SDMAC
#endif

#if   (0 == RTE_SPIM0_RESISTOR)
# define SPIM0_RESISTOR PMU_IO_RESISTOR_PULLDOWN
#elif (1 == RTE_SPIM0_RESISTOR)
# define SPIM0_RESISTOR PMU_IO_RESISTOR_NONE
#elif (2 == RTE_SPIM0_RESISTOR)
# define SPIM0_RESISTOR PMU_IO_RESISTOR_PULLUP
#else
# warning "Please define RTE_SPIM0_RESISTOR at RTE_Device.h."
# define SPIM0_RESISTOR PMU_IO_RESISTOR_NONE
#endif
#if   (0 == RTE_SPIM0_DRIVE_CAPABILITY)
# define SPIM0_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#elif (1 == RTE_SPIM0_DRIVE_CAPABILITY)
# define SPIM0_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_4MA
#elif (2 == RTE_SPIM0_DRIVE_CAPABILITY)
# define SPIM0_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_5MA
#elif (3 == RTE_SPIM0_DRIVE_CAPABILITY)
# define SPIM0_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_7MA
#else
# warning "Please define RTE_SPIM0_DRIVE_CAPABILITY at RTE_Device.h"
# define SPIM0_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#endif

#if   (0 == RTE_SPIM1_RESISTOR)
# define SPIM1_RESISTOR PMU_IO_RESISTOR_PULLDOWN
#elif (1 == RTE_SPIM1_RESISTOR)
# define SPIM1_RESISTOR PMU_IO_RESISTOR_NONE
#elif (2 == RTE_SPIM1_RESISTOR)
# define SPIM1_RESISTOR PMU_IO_RESISTOR_PULLUP
#else
# warning "Please define RTE_SPIM1_RESISTOR at RTE_Device.h"
# define SPIM1_RESISTOR PMU_IO_RESISTOR_NONE
#endif
#if   (0 == RTE_SPIM1_DRIVE_CAPABILITY)
# define SPIM1_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#elif (1 == RTE_SPIM1_DRIVE_CAPABILITY)
# define SPIM1_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_4MA
#elif (2 == RTE_SPIM1_DRIVE_CAPABILITY)
# define SPIM1_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_5MA
#elif (3 == RTE_SPIM1_DRIVE_CAPABILITY)
# define SPIM1_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_7MA
#else
# warning "Please define RTE_SPIM1_DRIVE_CAPABILITY at RTE_Device.h"
# define SPIM1_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#endif

#if   (0 == RTE_SPIM2_RESISTOR)
# define SPIM2_RESISTOR PMU_IO_RESISTOR_PULLDOWN
#elif (1 == RTE_SPIM2_RESISTOR)
# define SPIM2_RESISTOR PMU_IO_RESISTOR_NONE
#elif (2 == RTE_SPIM2_RESISTOR)
# define SPIM2_RESISTOR PMU_IO_RESISTOR_PULLUP
#else
# warning "Please define RTE_SPIM2_RESISTOR at RTE_Device.h"
# define SPIM2_RESISTOR PMU_IO_RESISTOR_PULLDOWN
#endif
#if   (0 == RTE_SPIM2_DRIVE_CAPABILITY)
# define SPIM2_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#elif (1 == RTE_SPIM2_DRIVE_CAPABILITY)
# define SPIM2_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_4MA
#elif (2 == RTE_SPIM2_DRIVE_CAPABILITY)
# define SPIM2_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_5MA
#elif (3 == RTE_SPIM2_DRIVE_CAPABILITY)
# define SPIM2_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_7MA
#else
# warning "Please define RTE_SPIM2_DRIVE_CAPABILITY at RTE_Device.h"
# define SPIM2_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#endif

#if   (0 == RTE_SPIM3_RESISTOR)
# define SPIM3_RESISTOR PMU_IO_RESISTOR_PULLDOWN
#elif (1 == RTE_SPIM3_RESISTOR)
# define SPIM3_RESISTOR PMU_IO_RESISTOR_NONE
#elif (2 == RTE_SPIM3_RESISTOR)
# define SPIM3_RESISTOR PMU_IO_RESISTOR_PULLUP
#else
# warning "Please define RTE_SPIM3_RESISTOR at RTE_Device.h"
# define SPIM3_RESISTOR PMU_IO_RESISTOR_PULLDOWN
#endif
#if   (0 == RTE_SPIM3_DRIVE_CAPABILITY)
# define SPIM3_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#elif (1 == RTE_SPIM3_DRIVE_CAPABILITY)
# define SPIM3_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_4MA
#elif (2 == RTE_SPIM3_DRIVE_CAPABILITY)
# define SPIM3_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_5MA
#elif (3 == RTE_SPIM3_DRIVE_CAPABILITY)
# define SPIM3_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_7MA
#else
# warning "Please define RTE_SPIM3_DRIVE_CAPABILITY at RTE_Device.h"
# define SPIM3_DRIVE_CAPABILITY PMU_DRIVE_CAPABILITY_2MA
#endif

/* bitband alias */
#define BB_SPI_TDMAE(_x)          BITBAND_VALUE(&(_x)->DMACR,1)
#define BB_SPI_RDMAE(_x)          BITBAND_VALUE(&(_x)->DMACR,0)

#define SPI_SCPH_HI  (1u<<6)
#define SPI_SCPH_LO  (0)
#define SPI_SCPOL_HI (1u<<7)
#define SPI_SCPOL_LO (0)

#define SPI_IMR_TXEIM_MASK     (~(1<<0))
#define SPI_IMR_TXEIM_UNMASK   (1<<0)
#define SPI_IMR_RXFIM_MASK     (~(1<<4))
#define SPI_IMR_RXFIM_UNMASK   (1<<4)
#define SPI_IMR_RXOIR_MASK     (~(1<<3))
#define SPI_IMR_RXOIR_UNMASK   (1<<3)

#define SPI_ISR_RXFIS          (1<<4)
#define SPI_ISR_RXOIS          (1<<3)
#define SPI_ISR_TXEIS          (1<<0)

#define SPI_SSIENR_DISABLE     (0)
#define SPI_SSIENR_ENABLE      (1)

#define SPI0_TX_FIFO_SIZE      (8)
#define SPI1_TX_FIFO_SIZE      (8)
#define SPI2_TX_FIFO_SIZE      (2)
#define SPI3_TX_FIFO_SIZE      (2)
#define SPI_RX_FIFO_SIZE       (8)

#define SPI_DFS(x)             ((x)&0xf)
#define SPI_DFS_MASK           (~0xf)
#define SPI_DFS_8BIT           (7)
#define SPI_DFS_16BIT          (15)

#define SPI_TMOD_TX_RX         (0)
#define SPI_TMOD_TX            (1<<8)
#define SPI_TMOD_RX            (2<<8)
#define SPI_TMOD_EEPROM        (3<<8)

#define SPI_DMACR_TDMAE        (1<<1)
#define SPI_DMACR_RDMAE        (1<<0)

#define ARM_SPI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,41)   /* driver version */

#define SPI_BUSY(_sr)          (4!=((_sr)&5))
#define SPI_IDLE(_sr)          (4==((_sr)&5))

#ifdef SPI_WITH_SDMAC
extern TZ10XX_DRIVER_SDMAC Driver_SDMAC;
#define DRIVER_SDMAC       Driver_SDMAC
#endif

#ifdef SPI_DMA_RX
# define SPI_DMA_RX_UNINIT(_x) DRIVER_SDMAC.ChannelUninitialize(_x)
#else
# define SPI_DMA_RX_UNINIT(_x)
#endif

#ifdef SPI_DMA_TX
# define SPI_DMA_TX_UNINIT(_x) DRIVER_SDMAC.ChannelUninitialize(_x)
#else
# define SPI_DMA_TX_UNINIT(_x)
#endif

typedef void (*SPI_ConfigurePin)(void);

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_SPI_API_VERSION,
  ARM_SPI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_SPI_CAPABILITIES DriverCapabilities = { 1 };

/* SPI Data Information (Run-Time) */
typedef struct {
  uint8_t                *buf;            // Pointer to data buffer
  uint32_t                cnt;            // Number of data bytes
} SPI_DATA_INFO;

/* SPI Information (Run-Time) */
typedef struct {
  uint32_t                sckdv;          // BAUDR.SCKDV field
  uint32_t                ctrlr0;         // CTRLR0 register
  bool                    init;           // Initialized flag
  ARM_POWER_STATE         power;          // Power mode
  volatile bool           busy;           // Busy flag
  ARM_SPI_SignalEvent_t   cb_event;       // Event Callback
  uint16_t                out;            // output value while receiving data
  SPI_DATA_INFO           tx;
  SPI_DATA_INFO           rx;
} SPI_INFO;

#ifdef SPI_WITH_SDMAC
typedef struct {
  int8_t              peri;               // Hardware ID
  int8_t              ch;                 // DMA channel
  int8_t              hs;                 // Handshake channel
} SPI_DMA_RESOURCES;
#endif

/* SPI Resources definition */
typedef struct {
  spim_Type          *spim;               //< Pointer to SPIM peripheral
  IRQn_Type           irq;                //< SPI IRQ Number
  SPI_ConfigurePin    ConfigurePin;       //< Configure IO pin
  PMU_MODULE          pmu_mod;            //< Peripheral name
  PMU_IO_FUNC         miso;               //< MISO pin
  PMU_CD              pmu_cd;             //< Peripheral bus clock
  SPI_INFO           *info;               //< Run-Time information
  uint8_t             tx_size;            //< TX FIFO size
#ifdef SPI_WITH_SDMAC
  SDMAC_SignalEvent_t dma;                //< DMA Event handler
  SPI_DMA_RESOURCES   tx;                 //< DMA Resources for transmit
  SPI_DMA_RESOURCES   rx;                 //< DMA Resources for recept
#endif
} const SPI_RESOURCES;

static ARM_SPI_STATUS SPI_PowerControl(ARM_POWER_STATE state, SPI_RESOURCES *res);
static ARM_SPI_STATUS SPI_SendFrames(
  const uint8_t *buf, uint32_t len, uint32_t dfs, SPI_RESOURCES *res);
static ARM_SPI_STATUS SPI_ReceiveFrames(
  uint8_t *buf, uint32_t len, uint16_t out, uint32_t dfs, SPI_RESOURCES *res);

#ifdef SPI_WITH_SDMAC
static SDMAC_MSIZE SPI_Length2Msize(uint32_t len, uint32_t fifo_sizes)
{
  if( fifo_sizes < 4 ){
    return SDMAC_MSIZE_1;
  }
  
  if (len & 3) {
    return SDMAC_MSIZE_1;
  } else {
    return SDMAC_MSIZE_4;
  }
}
#endif

#ifdef SPI_DMA_RX
static uint32_t SPI_Msize2RxThreshold(SDMAC_MSIZE msize)
{
  uint32_t th;

  th = (msize << 2);
  if (th) {
    --th;
  }
  return th;
}
#endif

#ifdef SPI_DMA_TX
static uint32_t SPI_Msize2TxThreshold(SDMAC_MSIZE msize, uint32_t fifo_size)
{
  uint32_t th;
  if( fifo_size <= 2 ){
    return 0;
  }
  else{
    th = msize << 2;
    if (0 == th) {
      th += 2;
    }
    return fifo_size - th;
  }
}
#endif

/*!
 * \brief       Get driver version.
 * \return      \ref ARM_DRV_VERSION
 */
static ARM_DRIVER_VERSION SPIX_GetVersion(void)
{
  return DriverVersion;
}

/*!
 * \brief       Get driver capabilities.
 * \return      \ref ARM_SPI_CAPABILITIES
 */
static ARM_SPI_CAPABILITIES SPIX_GetCapabilities(void)
{
  return DriverCapabilities;
}

/*!
 * \brief       Initialize SPI Interface.
 * \param[in]   cb_event  Pointer to \ref ARM_SPI_SignalEvent
 * \param[in]   res       Pointer to SPI resources
 * \return      \ref ARM_SPI_STATUS
 */
static ARM_SPI_STATUS SPI_Initialize(
  ARM_SPI_SignalEvent_t cb_event,
  SPI_RESOURCES        *res)
{
  SPI_INFO *info;

  info = res->info;

  if (true == info->init) {
    return ARM_SPI_ERROR;
  }
  res->ConfigurePin();

  /* Clear and Enable SPI IRQ */
  NVIC_ClearPendingIRQ(res->irq);
  NVIC_EnableIRQ(res->irq);

  /* Initialize SPI Run-Time Resources */
  info->sckdv  = 2;
  info->ctrlr0 = 7;
  info->power  = ARM_POWER_OFF;

  info->cb_event = cb_event;
  info->init     = true;
  return SPI_PowerControl(ARM_POWER_LOW, res);
}

/*!
 * \brief       De-initialize SPI Interface.
 * \param[in]   res  Pointer to SPI resources
 * \return      \ref SPI_STATUS
 */
static ARM_SPI_STATUS SPI_Uninitialize(SPI_RESOURCES *res)
{
  SPI_INFO *info;

  info = res->info;

  if (false == info->init) {
    return ARM_SPI_OK;
  }

  SPI_PowerControl(ARM_POWER_OFF, res);

  // Disable SPI IRQ
  NVIC_DisableIRQ(res->irq);
  NVIC_ClearPendingIRQ(res->irq);

  // Uninitialize SPI Run-Time Resources
  info->cb_event = NULL;
  info->init     = false;
  return ARM_SPI_OK;
}

/*!
 * \brief       Controls SPI Interface Power.
 * \param[in]   state    Power state
 * \param[in]   res      Pointer to SPI resources
 * \return      \ref ARM_SPI_STATUS
 */
static ARM_SPI_STATUS SPI_PowerControl(ARM_POWER_STATE state, SPI_RESOURCES *res)
{
  SPI_INFO  *info;

  info = res->info;
  if (false == info->init) return ARM_SPI_ERROR;

  switch (state) {
  case ARM_POWER_OFF:
    // Disable SPI
    res->spim->SSIENR = SPI_SSIENR_DISABLE;
    Driver_PMU.EnableModule(res->pmu_mod, 0);
    Driver_PMU.StandbyInputBuffer(res->miso, 1);
    // Update SPI Run-Time Resources
    info->sckdv  = 2;
    info->ctrlr0 = 7;
    info->power  = ARM_POWER_OFF;
    info->out    = 0;
    info->tx.buf = NULL;
    info->tx.cnt = 0;
    info->rx.buf = NULL;
    info->rx.cnt = 0;
    break;
  case ARM_POWER_LOW:
    Driver_PMU.StandbyInputBuffer(res->miso, 0);
    Driver_PMU.EnableModule(res->pmu_mod, 1);
    // Disable SPI
    res->spim->SSIENR = SPI_SSIENR_DISABLE;
    info->power  = ARM_POWER_LOW;
    info->out    = 0;
    info->tx.buf = NULL;
    info->tx.cnt = 0;
    info->rx.buf = NULL;
    info->rx.cnt = 0;
    break;
  case ARM_POWER_FULL:
    Driver_PMU.StandbyInputBuffer(res->miso, 0);
    Driver_PMU.EnableModule(res->pmu_mod, 1);
    info->power = ARM_POWER_FULL;
    break;
  default:
    return ARM_SPI_ERROR_UNSUPPORTED;
  }
  return ARM_SPI_OK;
}


/*!
 * \brief       Configure SPI Interface.
 * \param[in]   frame_format  \ref ARM_SPI_FRAME_FORMAT
 * \param[in]   bit_order     \ref ARM_SPI_BIT_ORDER
 * \param[in]   res       Pointer to SPI resources
 * \return      \ref ARM_SPI_STATUS
 */
static ARM_SPI_STATUS SPI_Configure(ARM_SPI_FRAME_FORMAT  frame_format,
                                    ARM_SPI_BIT_ORDER     bit_order,
                                    SPI_RESOURCES        *res)
{
  SPI_INFO *info;
  uint32_t  ctrlr0;

  info = res->info;

  if (ARM_POWER_OFF == info->power) return ARM_SPI_ERROR;
  if (ARM_SPI_MSB_LSB != bit_order) return ARM_SPI_ERROR;

  switch (frame_format) {
    case ARM_SPI_CPOL0_CPHA0:
      ctrlr0 = SPI_SCPOL_LO | SPI_SCPH_LO;
      break;
    case ARM_SPI_CPOL0_CPHA1:
      ctrlr0 = SPI_SCPOL_LO | SPI_SCPH_HI;
      break;
    case ARM_SPI_CPOL1_CPHA0:
      ctrlr0 = SPI_SCPOL_HI | SPI_SCPH_LO;
      break;
    case ARM_SPI_CPOL1_CPHA1:
      ctrlr0 = SPI_SCPOL_HI | SPI_SCPH_HI;
      break;
    default:
      return ARM_SPI_ERROR;
  }
  info->ctrlr0 = ((info->ctrlr0) & ~SPI_DFS_MASK) | ctrlr0;

  return ARM_SPI_OK;
}

/**
  \fn          uint32_t SPI_BusSpeed (uint32_t       bps,
                                      SPI_RESOURCES *res)
  \brief       Set bus speed for SPI transfers.
  \param[in]   bps      Requested speed in bits/s
  \param[in]   res  Pointer to SPI resources
  \return      Configured bus speed in bits/s
*/
static uint32_t SPI_BusSpeed(uint32_t       bps,
                             SPI_RESOURCES *res)
{
  SPI_INFO *info;
  uint32_t  n;
  uint32_t  clock;
  uint32_t  max_bps;

  info = res->info;

  if (ARM_POWER_OFF == info->power) return 0;

  clock = Driver_PMU.GetFrequency(res->pmu_cd);
  max_bps = clock / 2;

  if (0 == bps) {
    n = 2;
  } else {
    if( max_bps % bps == 0 ){
      n = max_bps / bps * 2;
    }
    else{
      n = (max_bps / bps + 1 ) * 2;
    }
    if (n < 2) {
      n = 2;
    } else if (n > 65534) {
      n = 65534;
    }
  }
  bps = clock / n;
  info->sckdv = n;

  return bps;
}

/*!
 * \brief       Control Slave Select (SS) signal.
 * \param[in]   ss       \ref SPI_SS_SIGNAL
 * \return      \ref SPI_STATUS
 */
static ARM_SPI_STATUS SPI_SlaveSelect(ARM_SPI_SS_SIGNAL  ss, SPI_RESOURCES *res)
{
  if (ARM_POWER_OFF == res->info->power) return ARM_SPI_ERROR;
  if (ARM_SPI_SS_ACTIVE < ss) return ARM_SPI_ERROR;
  return ARM_SPI_OK;
}

/*!
 * \brief       Send and receive one byte via SPI Interface.
 * \param[in]   out      Byte to be sent to the SPI data output
 * \param[in]   res  Pointer to SPI resources
 * \return      Byte received from the SPI data input
 */
static uint8_t SPI_TransferByte(uint8_t out, SPI_RESOURCES *res)
{
  SPI_INFO  *info;
  spim_Type *spim;
  uint8_t    val;

  info = res->info;

  if (ARM_POWER_FULL != info->power) return 0;

  spim         = res->spim;
  spim->CTRLR0 = ((info->ctrlr0) & SPI_DFS_MASK) | SPI_TMOD_TX_RX | SPI_DFS_8BIT;
  spim->BAUDR  = info->sckdv;
  spim->SER    = 1;
  spim->IMR    = 0;
  spim->SSIENR = SPI_SSIENR_ENABLE;
  spim->DR[0]  = out;

  while (SPI_BUSY(spim->SR) &&
         (ARM_POWER_FULL == info->power)) {
    ;
  }

  val = spim->DR[0];

  spim->SSIENR  = SPI_SSIENR_DISABLE;

  return val;
}

/*!
 * \fn          ARM_SPI_STATUS SPI_AbortTransfer (SPI_RESOURCES *res)
 * \brief       Abort current SPI transfer.
 * \param[in]   res  Pointer to SPI resources
 * \return      \ref ARM_SPI_STATUS
 */
static ARM_SPI_STATUS SPI_AbortTransfer(SPI_RESOURCES *res)
{
  SPI_INFO *info;

  info = res->info;

  if (ARM_POWER_FULL != info->power) return ARM_SPI_ERROR;
  if (false == info->busy) return ARM_SPI_ERROR;

  res->spim->SSIENR = SPI_SSIENR_DISABLE;
  info->busy        = false;

  return ARM_SPI_OK;
}

#if (RTE_SPIM0_DMA_TX || RTE_SPIM1_DMA_TX || RTE_SPIM2_DMA_TX || RTE_SPIM3_DMA_TX)
static void SPI_DMATxHandler(SPI_RESOURCES *res, uint32_t e)
{
  SPI_INFO *info;

  info = res->info;
  if (e & SDMAC_EVENT_TRANSFER) {
    info->tx.cnt = 0;
    info->tx.buf = NULL;
  }
}
#endif

#if (RTE_SPIM0_DMA_RX || RTE_SPIM1_DMA_RX || RTE_SPIM2_DMA_RX || RTE_SPIM3_DMA_RX)
static void SPI_DMARxHandler(SPI_RESOURCES *res, uint32_t e)
{
  SPI_INFO  *info;
  spim_Type *spim;
  uint32_t   isr;

  info = res->info;
  spim = res->spim;
  isr  = spim->ISR;
  if (isr & SPI_ISR_RXOIS) {
    spim->IMR = 0;
  } else {
    spim->SSIENR = SPI_SSIENR_DISABLE;
  }

  if (e & SDMAC_EVENT_TRANSFER) {
    info->rx.cnt = 0;
    info->rx.buf = NULL;
  }
  info->busy = false;
  if (info->cb_event) {
    info->cb_event(ARM_SPI_EVENT_SEND_DONE);
  }
}
#endif

/*!
 * \brief       SPI Interrupt handler.
 * \param[in]   res  Pointer to SPI resources
 */
static void SPI_IRQHandler(SPI_RESOURCES *res)
{
  volatile uint16_t  dummy;
  SPI_INFO  *info;
  spim_Type *spim;
  int sz, cap;
  int i;
  uint8_t   *buf;
  uint16_t  *p;
  bool       two_byte_size;
  uint32_t   isr;

  info = res->info;
  spim = res->spim;
  isr  = spim->ISR;
  // Overflow Receive FIFO 
  if (isr & SPI_ISR_RXOIS) {
    spim->IMR  = 0;
    info->busy = false;
    return;
  }

  if (SPI_DFS(spim->CTRLR0) < 8) {
    two_byte_size = false;
  } else {
    two_byte_size = true;
  }

  // Receive FIFO Full
  if (isr & SPI_ISR_RXFIS) {
    sz  = spim->RXFLR;
    buf = info->rx.buf;
    if (NULL == buf) {
      // transmit only case
      for (i = 0; i < sz; ++i) {
        dummy = spim->DR[0];
      }
    } else {
      if (two_byte_size) {
        p = (uint16_t*)buf;
        for (i = 0; i < sz; ++i) {
          *p++ = spim->DR[0];
        }
        info->rx.buf = (uint8_t*)p;
      } else {
        for (i = 0; i < sz; ++i) {
          *buf++ = spim->DR[0];
        }
        info->rx.buf = buf;
      }
    }
    info->rx.cnt -= sz;
    if (0 == info->rx.cnt) {
      spim->SSIENR = SPI_SSIENR_DISABLE; // clear interrupt status by disable spim
      info->busy   = false;
      if (info->cb_event) {
        info->cb_event(ARM_SPI_EVENT_SEND_DONE);
      }
    } else if (info->rx.cnt < SPI_RX_FIFO_SIZE) {
      spim->RXFTLR = info->rx.cnt - 1;
    }
  }

  // Transmit FIFO Empty
  if (isr & SPI_ISR_TXEIS) {
    sz = info->tx.cnt;
    if (sz) {
      cap = res->tx_size - spim->TXFLR;
      if (cap < sz) {
        sz = cap;
      }
      if (SPI_IDLE(spim->SR)) {
        spim->IMR  = 0;
        info->busy = false;
        return;
      }
      buf = info->tx.buf;
      if (NULL == buf) {
        dummy = info->out;
        for (i = 0; i < sz; ++i) {
          spim->DR[0] = dummy;
        }
      } else {
        if (two_byte_size) {
          p = (uint16_t*)buf;
          for (i = 0; i < sz; ++i) {
            spim->DR[0] = *p++;
          }
          info->tx.buf = (uint8_t*)p;
        } else {
          for (i = 0; i < sz; ++i) {
            spim->DR[0] = *buf++;
          }
          info->tx.buf = buf;
        }
      }
      sz = info->tx.cnt - sz;
      info->tx.cnt = sz;
    }
    if (0 == sz) {
      // transmit FIFO fully empty, but last data is transmitting
      spim->IMR &= SPI_IMR_TXEIM_MASK;
    } else if (sz < 1) {
      spim->TXFTLR = 0;
    }
  }
}

/*!
 * \brief       Set frame size.
 * \param[in]   frame_size_m1     frame size minus one
 * \param[in]   res  Pointer to SPI resources
 */
static ARM_SPI_STATUS SPI_FrameSize(uint32_t frame_size_m1, SPI_RESOURCES *res)
{
  SPI_INFO *info;

  info = res->info;

  if (ARM_POWER_OFF == info->power) return ARM_SPI_ERROR;
  if ((frame_size_m1 < 3) || (15 < frame_size_m1)) return ARM_SPI_ERROR;

  info->ctrlr0 = ((info->ctrlr0) & SPI_DFS_MASK) | frame_size_m1;
  return ARM_SPI_OK;
}

/*!
 * \brief       Send and receive one frame via SPI Interface.
 * \param[in]   frame      A frame to be sent to the SPI data output
 * \param[in]   res        Pointer to SPI resources
 */
static uint16_t SPI_TransferFrame(uint16_t frame, SPI_RESOURCES *res)
{
  SPI_INFO  *info;
  spim_Type *spim;
  uint16_t   val;

  info = res->info;

  if (ARM_POWER_FULL != info->power) return 0;

  spim = res->spim;

  spim->CTRLR0 = info->ctrlr0;
  spim->BAUDR  = info->sckdv;
  spim->SER    = 1;
  spim->IMR    = 0;
  spim->SSIENR = SPI_SSIENR_ENABLE;
  spim->DR[0]  = frame;

  while (SPI_BUSY(spim->SR) &&
         (ARM_POWER_FULL == info->power)) {
    ;
  }

  val          = spim->DR[0];

  spim->SSIENR = SPI_SSIENR_DISABLE;
  return val;
}

/*!
 * \brief       Send frames to SPI Slave.
 * \param[in]   buf      Pointer to the buffer containing frames to send
 * \param[in]   len      Frames buffer length in bytes
 * \param[in]   dfs      frame size minus one
 * \param[in]   res      Pointer to SPI resources
 * \return      \ref SPI_STATUS
 */
static ARM_SPI_STATUS SPI_SendFrames(const uint8_t *buf, uint32_t len, uint32_t dfs, SPI_RESOURCES *res)
{
  SPI_INFO       *info;
  spim_Type      *spim;
  ARM_SPI_STATUS  ret;
  const uint16_t *p;
  int             i, tx_th, rx_th, cnt;
  bool            two_byte_size;
  uint32_t        imr;
#ifdef SPI_WITH_SDMAC
  SDMAC_STATUS    err;
  uint32_t        ctl;
  SDMAC_MSIZE     msize;
#endif
#ifdef SPI_DMA_TX
  int32_t         tx_ch;
#endif
#ifdef SPI_DMA_RX
  int32_t         rx_ch;
#endif

  info = res->info;

  if ((buf == NULL) || (len == 0)) return ARM_SPI_ERROR;
  if (ARM_POWER_FULL != info->power) return ARM_SPI_ERROR;

  if (dfs < 8) {
    two_byte_size = false;
  } else {
    two_byte_size = true;
    if (len & 1) {
      return ARM_SPI_ERROR;
    }
    len /= 2;
  }

  spim = res->spim;

  spim->CTRLR0 = ((info->ctrlr0) & SPI_DFS_MASK) | SPI_TMOD_TX_RX | dfs;
  spim->BAUDR  = info->sckdv;
  spim->SER    = 1;

#ifdef SPI_WITH_SDMAC
  /* calculate dma burst length from data length */
  msize = SPI_Length2Msize(len, res->tx_size);
  spim->DMACR = 0;
#endif
#ifdef SPI_DMA_RX
  /* initialize dma rx channel. */
  rx_ch = res->rx.ch;
  if (0 <= rx_ch) {
    err = DRIVER_SDMAC.ChannelInitialize(
      rx_ch, res->dma, (SDMAC_SRC)res->rx.peri, SDMAC_DST_MEMORY,
      SDMAC_FIFO_MODE_UTILIZATION, SDMAC_PRIORITY_LOWEST,
      res->rx.hs, 0);
    if (err) {
      return ARM_SPI_ERROR;
    }
    if (two_byte_size) {
      ctl = SDMAC_CTL(SDMAC_WIDTH_2, SDMAC_INC_NO_CHANGE, msize,
                      SDMAC_WIDTH_2, SDMAC_INC_NO_CHANGE, SDMAC_MSIZE_8);
    } else {
      ctl = SDMAC_CTL(SDMAC_WIDTH_1, SDMAC_INC_NO_CHANGE, msize,
                      SDMAC_WIDTH_2, SDMAC_INC_NO_CHANGE, SDMAC_MSIZE_8);
    }
    err = DRIVER_SDMAC.ChannelTransferData(
      rx_ch, (const uint8_t*)&spim->DR[0], (uint8_t*)&info->out,
      len << two_byte_size, ctl);
    if (err) {
      SPI_DMA_RX_UNINIT(rx_ch);
      return ARM_SPI_ERROR;
    }
    BB_SPI_RDMAE(spim) = 1;
    spim->DMARDLR = SPI_Msize2RxThreshold(msize);
    spim->RXFTLR  = 0;
    info->rx.cnt  = 0;
    info->rx.buf  = NULL;
  } else {
#endif
    if (len < SPI_RX_FIFO_SIZE) {
      rx_th = len - 1;
    } else {
      rx_th = SPI_RX_FIFO_SIZE / 2 - 1;
    }
    spim->RXFTLR = rx_th;
    info->rx.cnt = len;
    info->rx.buf = NULL;
#ifdef SPI_DMA_RX
  }
#endif

#ifdef SPI_DMA_TX
  /* initialize dma tx channel. */
  tx_ch = res->tx.ch;
  if (0 <= tx_ch) {
    err = DRIVER_SDMAC.ChannelInitialize(
      tx_ch, res->dma, SDMAC_SRC_MEMORY, (SDMAC_DST)res->tx.peri,
      SDMAC_FIFO_MODE_UTILIZATION, SDMAC_PRIORITY_LOWEST,
      0, res->tx.hs);
    if (err) {
      SPI_DMA_RX_UNINIT(rx_ch);
      return ARM_SPI_ERROR;
    }
    if (two_byte_size) {
      ctl = SDMAC_CTL(SDMAC_WIDTH_2, SDMAC_INC_INCREMENT, msize,
                      SDMAC_WIDTH_2, SDMAC_INC_NO_CHANGE, msize);
    } else {
      ctl = SDMAC_CTL(SDMAC_WIDTH_1, SDMAC_INC_INCREMENT, msize,
                      SDMAC_WIDTH_1, SDMAC_INC_NO_CHANGE, msize);
    }
    BB_SPI_TDMAE(spim) = 1;
    spim->DMATDLR      = SPI_Msize2TxThreshold(msize, res->tx_size);
    spim->TXFTLR       = 0;
    info->tx.cnt       = 0;
    info->tx.buf       = NULL;

    info->busy   = 1;
    spim->SSIENR = SPI_SSIENR_ENABLE;
    err = DRIVER_SDMAC.ChannelTransferData(
      tx_ch, buf, (uint8_t*)&spim->DR[0],
      len << two_byte_size, ctl);
    if (err) {
      spim->SSIENR = SPI_SSIENR_DISABLE;
      info->busy   = 0;
      SPI_DMA_RX_UNINIT(rx_ch);
      SPI_DMA_TX_UNINIT(tx_ch);
      return ARM_SPI_ERROR;
    }
  } else {
#endif
    if (len <= res->tx_size) {
      cnt   = len;
      tx_th = 0;
    } else {
      cnt   = res->tx_size;
      tx_th = res->tx_size / 2;
    }
    spim->TXFTLR = tx_th;
    info->tx.cnt = len - cnt;
    info->tx.buf = (uint8_t*)buf + (cnt << two_byte_size);

    info->busy   = 1;
    spim->SSIENR = SPI_SSIENR_ENABLE;
#ifdef SPI_DMA_TX
  }
#endif

  imr = SPI_IMR_RXOIR_UNMASK;
#ifdef SPI_DMA_RX
  if (rx_ch < 0) {
    imr |= SPI_IMR_RXFIM_UNMASK;
  }
#else
  imr |= SPI_IMR_RXFIM_UNMASK;
#endif
#ifdef SPI_DMA_TX
  if (tx_ch < 0) {
#endif
    if (two_byte_size) {
      p = (const uint16_t*)buf;
      for (i = 0; i < cnt; ++i) {
        spim->DR[0] = *p++;
      }
    } else {
      for (i = 0; i < cnt; ++i) {
        spim->DR[0] = *buf++;
      }
    }
    if (info->tx.cnt) {
      imr |= SPI_IMR_TXEIM_UNMASK;
    }
#ifdef SPI_DMA_TX
  }
#endif
  spim->IMR = imr;

  // wait until done
  while (info->busy) ;

  ret = ARM_SPI_OK;
  if (spim->RISR_b.RXOIR) {
    ret = ARM_SPI_ERROR;
  } else if (0 != info->tx.cnt) {
    ret = ARM_SPI_ERROR;
  }
  spim->SSIENR = SPI_SSIENR_DISABLE;
#ifdef SPI_WITH_SDMAC
  spim->DMACR  = 0;
#endif
  SPI_DMA_RX_UNINIT(rx_ch);
  SPI_DMA_TX_UNINIT(tx_ch);
  return ret;
}

/*!
 * \brief       Receive frames from SPI Slave.
 * \param[out]  buf      Pointer to the buffer for receiving frames
 * \param[in]   len      Frames buffer length in bytes
 * \param[in]   out      Byte to be sent repeatedly during receive
 * \param[in]   dfs      frame size minus one
 * \param[in]   res      Pointer to SPI resources
 * \return      \ref SPI_STATUS
 */
static ARM_SPI_STATUS SPI_ReceiveFrames(
  uint8_t *buf, uint32_t len, uint16_t out, uint32_t dfs, SPI_RESOURCES *res)
{
  SPI_INFO       *info;
  spim_Type      *spim;
  ARM_SPI_STATUS  ret;
  int             i, cnt, tx_th, rx_th;
#ifdef SPI_WITH_SDMAC
  bool            two_byte_size;
#endif
  uint32_t        imr;
#ifdef SPI_WITH_SDMAC
  uint32_t        ctl;
  SDMAC_MSIZE     msize;
  SDMAC_STATUS    err;
#endif
#ifdef SPI_DMA_TX
  int32_t         tx_ch;
#endif
#ifdef SPI_DMA_RX
  int32_t         rx_ch;
#endif

  info = res->info;

  if ((buf == NULL) || (len == 0) || (65536 < len)) return ARM_SPI_ERROR;
  if (ARM_POWER_FULL != info->power) return ARM_SPI_ERROR;

  /* adjust 'len' to frame size unit. */
  if (7 < dfs) {
#ifdef SPI_WITH_SDMAC
    two_byte_size = true;
#endif
    if (len & 1) {
       return ARM_SPI_ERROR;
    }
    len /= 2;
  }
#ifdef SPI_WITH_SDMAC
  else {
    two_byte_size = false;
  }
#endif

  spim = res->spim;

  spim->CTRLR0 = ((info->ctrlr0) & SPI_DFS_MASK) | SPI_TMOD_TX_RX | dfs;
  spim->BAUDR  = info->sckdv;
  spim->SER    = 1;

#ifdef SPI_WITH_SDMAC
  spim->DMACR = 0;
  /* calculate dma burst length from data length */
  msize = SPI_Length2Msize(len, res->tx_size);
#endif
#ifdef SPI_DMA_RX
  /* initialize dma rx channel. */
  rx_ch = res->rx.ch;
  if (0 <= rx_ch) {
    err = DRIVER_SDMAC.ChannelInitialize(
      rx_ch, res->dma, (SDMAC_SRC)res->rx.peri, SDMAC_DST_MEMORY,
      SDMAC_FIFO_MODE_UTILIZATION, SDMAC_PRIORITY_LOWEST,
      res->rx.hs, 0);
    if (err) {
      return ARM_SPI_ERROR;
    }
    if (two_byte_size) {
      ctl = SDMAC_CTL(SDMAC_WIDTH_2, SDMAC_INC_NO_CHANGE, msize,
                      SDMAC_WIDTH_4, SDMAC_INC_INCREMENT, SDMAC_MSIZE_4);
    } else {
      ctl = SDMAC_CTL(SDMAC_WIDTH_1, SDMAC_INC_NO_CHANGE, msize,
                      SDMAC_WIDTH_4, SDMAC_INC_INCREMENT, SDMAC_MSIZE_4);
    }
    err = DRIVER_SDMAC.ChannelTransferData(
      rx_ch, (const uint8_t*)&spim->DR[0], buf, len << two_byte_size, ctl);
    if (err) {
      SPI_DMA_RX_UNINIT(rx_ch);
      return ARM_SPI_ERROR;
    }
    BB_SPI_RDMAE(spim) = 1;
    spim->DMARDLR      = SPI_Msize2RxThreshold(msize);
    info->rx.cnt       = 0;
    info->rx.buf       = NULL;
  } else {
#endif
    /* calculate rx buffer threshold. */
    if (len <= SPI_RX_FIFO_SIZE) {
      rx_th = len - 1;
    } else {
      rx_th = SPI_RX_FIFO_SIZE / 2 - 1;
    }
    spim->RXFTLR = rx_th;
    info->rx.cnt = len;
    info->rx.buf = buf;
#ifdef SPI_DMA_RX
  }
#endif
  info->out = out;
#ifdef SPI_DMA_TX
  /* initialize dma tx channel. */
  tx_ch = res->tx.ch;
  if (0 <= tx_ch) {
    err = DRIVER_SDMAC.ChannelInitialize(
      tx_ch, res->dma, SDMAC_SRC_MEMORY, (SDMAC_DST)res->tx.peri,
      SDMAC_FIFO_MODE_UTILIZATION, SDMAC_PRIORITY_LOWEST,
      0, res->tx.hs);
    if (err) {
      return ARM_SPI_ERROR;
    }
    if (two_byte_size) {
      ctl = SDMAC_CTL(SDMAC_WIDTH_2, SDMAC_INC_NO_CHANGE, msize,
                      SDMAC_WIDTH_2, SDMAC_INC_NO_CHANGE, msize);
    } else {
      ctl = SDMAC_CTL(SDMAC_WIDTH_1, SDMAC_INC_NO_CHANGE, msize,
                      SDMAC_WIDTH_1, SDMAC_INC_NO_CHANGE, msize);
    }
    BB_SPI_TDMAE(spim) = 1;
    spim->DMATDLR      = SPI_Msize2TxThreshold(msize, res->tx_size);
    spim->TXFTLR       = 0;
    info->tx.cnt       = 0;
    info->tx.buf       = NULL;

    info->busy   = 1;
    spim->SSIENR = SPI_SSIENR_ENABLE;
    err = DRIVER_SDMAC.ChannelTransferData(
      tx_ch, (const uint8_t*)&info->out,
      (uint8_t*)&spim->DR[0], len << two_byte_size, ctl);
    if (err) {
      spim->SSIENR = SPI_SSIENR_DISABLE;
      info->busy   = 0;
      SPI_DMA_RX_UNINIT(rx_ch);
      SPI_DMA_TX_UNINIT(tx_ch);
      return ARM_SPI_ERROR;
    }
  } else {
#endif
    /* calculate tx buffer threshold. */
    if (len <= res->tx_size) {
      cnt   = len;
      tx_th = 0;
    } else {
      cnt   = res->tx_size;
      tx_th = res->tx_size / 2;
    }
    spim->TXFTLR = tx_th;
    info->tx.cnt = len - cnt;
    info->tx.buf = NULL;

    info->busy   = 1;
    spim->SSIENR = SPI_SSIENR_ENABLE;
#ifdef SPI_DMA_TX
  }
#endif

  imr = SPI_IMR_RXOIR_UNMASK;
#ifdef SPI_DMA_RX
  if (rx_ch < 0) {
    imr |= SPI_IMR_RXFIM_UNMASK;
  }
#else
  imr |= SPI_IMR_RXFIM_UNMASK;
#endif

#ifdef SPI_DMA_TX
  if (tx_ch < 0) {
#endif
    for (i = 0; i < cnt; ++i) {
      spim->DR[0] = out;
    }
    if (info->tx.cnt) {
      imr |= SPI_IMR_TXEIM_UNMASK;
    }
#ifdef SPI_DMA_TX
  }
#endif

  spim->IMR = imr;

  // wait for done
  while (info->busy) ;

  ret = ARM_SPI_OK;
  if (spim->RISR_b.RXOIR) {
    ret = ARM_SPI_ERROR;
  } else if (0 != info->tx.cnt) {
    ret = ARM_SPI_ERROR;
  }

  spim->SSIENR = SPI_SSIENR_DISABLE;
#ifdef SPI_WITH_SDMAC
  spim->DMACR  = 0;
#endif
  SPI_DMA_RX_UNINIT(rx_ch);
  SPI_DMA_TX_UNINIT(tx_ch);
  return ret;
}

/*!
 * \brief       Send frames to SPI Slave and then receive frames from SPI Slave.
 * \param[in]   cmd      Pointer to the buffer containing commands to send
 * \param[in]   cmd_len  Command buffer length in bytes
 * \param[out]  buf      Pointer to the buffer for receiving frames
 * \param[in]   len      Frames buffer length in bytes
 * \param[in]   out      Byte to be sent repeatedly during receive
 * \param[in]   res      Pointer to SPI resources
 * \return      \ref SPI_STATUS
 */
static ARM_SPI_STATUS SPI_ReadROM(const uint8_t *cmd, uint32_t cmd_len, uint8_t *buf, uint32_t len, SPI_RESOURCES *res)
{
  SPI_INFO       *info;
  spim_Type      *spim;
  ARM_SPI_STATUS  ret;
  const uint16_t *p;
  int             i, tx_th, rx_th, cnt, len_m1;
  bool            two_byte_size;
  uint32_t        imr;
#ifdef SPI_WITH_SDMAC
  SDMAC_STATUS    err;
  uint32_t        ctl;
  SDMAC_MSIZE     msize;
#endif
#ifdef SPI_DMA_TX
  int32_t         tx_ch;
#endif
#ifdef SPI_DMA_RX
  int32_t         rx_ch;
#endif

  info = res->info;

  if ((cmd == NULL) || (buf == NULL) ||
      (cmd_len == 0) || (len == 0) || (65536 < len)) {
    return ARM_SPI_ERROR;
  }
  if (ARM_POWER_FULL != info->power) return ARM_SPI_ERROR;

  if (SPI_DFS(info->ctrlr0) < 8) {
    two_byte_size = false;
  } else {
    two_byte_size = true;
    if ((cmd_len & 1) || (len & 1)) {
       return ARM_SPI_ERROR;
    }
    cmd_len /= 2;
    len     /= 2;
  }
  len_m1  = len - 1;

  spim = res->spim;

  spim->CTRLR0 = info->ctrlr0 | SPI_TMOD_EEPROM;
  spim->CTRLR1 = len_m1;
  spim->BAUDR  = info->sckdv;
  spim->SER    = 1;

#ifdef SPI_WITH_SDMAC
  /* calculate dma burst length from data length */
  msize = SPI_Length2Msize(len, res->tx_size);
  spim->DMACR = 0;
#endif
#ifdef SPI_DMA_RX
  /* initialize dma rx channel. */
  rx_ch = res->rx.ch;
  if (0 <= rx_ch) {
    err = DRIVER_SDMAC.ChannelInitialize(
      rx_ch, res->dma, (SDMAC_SRC)res->rx.peri, SDMAC_DST_MEMORY,
      SDMAC_FIFO_MODE_UTILIZATION, SDMAC_PRIORITY_LOWEST,
      res->rx.hs, 0);
    if (err) {
      return ARM_SPI_ERROR;
    }
    if (two_byte_size) {
      ctl = SDMAC_CTL(SDMAC_WIDTH_2, SDMAC_INC_NO_CHANGE, msize,
                      SDMAC_WIDTH_4, SDMAC_INC_INCREMENT, SDMAC_MSIZE_4);
    } else {
      ctl = SDMAC_CTL(SDMAC_WIDTH_1, SDMAC_INC_NO_CHANGE, msize,
                      SDMAC_WIDTH_4, SDMAC_INC_INCREMENT, SDMAC_MSIZE_4);
    }
    err = DRIVER_SDMAC.ChannelTransferData(
      rx_ch, (const uint8_t*)&spim->DR[0], buf,
      len << two_byte_size, ctl);
    if (err) {
      SPI_DMA_RX_UNINIT(rx_ch);
      return ARM_SPI_ERROR;
    }
    BB_SPI_RDMAE(spim) = 1;
    spim->DMARDLR = SPI_Msize2RxThreshold(msize);
    spim->RXFTLR  = 0;
    info->rx.cnt  = 0;
    info->rx.buf  = NULL;
  } else {
#endif
    if (len_m1 < SPI_RX_FIFO_SIZE) {
      rx_th = len_m1;
    } else {
      rx_th = SPI_RX_FIFO_SIZE / 2 - 1;
    }
    spim->RXFTLR = rx_th;
    info->rx.cnt = len;
    info->rx.buf = buf;
#ifdef SPI_DMA_RX
  }
#endif

  spim->IMR = 0;        /* mask all interrupt */
#ifdef SPI_DMA_TX
  /* initialize dma tx channel. */
  tx_ch = res->tx.ch;
  if (0 <= tx_ch) {
    err = DRIVER_SDMAC.ChannelInitialize(
      tx_ch, res->dma, SDMAC_SRC_MEMORY, (SDMAC_DST)res->tx.peri,
      SDMAC_FIFO_MODE_UTILIZATION, SDMAC_PRIORITY_LOWEST,
      0, res->tx.hs);
    if (err) {
      SPI_DMA_RX_UNINIT(rx_ch);
      return ARM_SPI_ERROR;
    }
    if (two_byte_size) {
      ctl = SDMAC_CTL(SDMAC_WIDTH_2, SDMAC_INC_INCREMENT, msize,
                      SDMAC_WIDTH_2, SDMAC_INC_NO_CHANGE, msize);
    } else {
      ctl = SDMAC_CTL(SDMAC_WIDTH_1, SDMAC_INC_INCREMENT, msize,
                      SDMAC_WIDTH_1, SDMAC_INC_NO_CHANGE, msize);
    }
    BB_SPI_TDMAE(spim) = 1;
    spim->DMATDLR      = SPI_Msize2TxThreshold(msize, res->tx_size);
    spim->TXFTLR       = 0;
    info->tx.cnt       = 0;
    info->tx.buf       = NULL;

    info->busy   = 1;
    spim->SSIENR = SPI_SSIENR_ENABLE;
    err = DRIVER_SDMAC.ChannelTransferData(
      tx_ch, cmd, (uint8_t*)&spim->DR[0],
      cmd_len << two_byte_size, ctl);
    if (err) {
      spim->SSIENR = SPI_SSIENR_DISABLE;
      info->busy   = 0;
      SPI_DMA_RX_UNINIT(rx_ch);
      SPI_DMA_TX_UNINIT(tx_ch);
      return ARM_SPI_ERROR;
    }
  } else {
#endif
    if (cmd_len <= res->tx_size) {
      cnt   = cmd_len;
      tx_th = 0;
    } else {
      cnt   = res->tx_size;
      tx_th = res->tx_size / 2;
    }
    spim->TXFTLR = tx_th;
    info->tx.cnt = cmd_len - cnt;
    info->tx.buf = (uint8_t*)cmd + (cnt << two_byte_size);

    info->busy   = 1;
    spim->SSIENR = SPI_SSIENR_ENABLE;
#ifdef SPI_DMA_TX
  }
#endif


  imr = SPI_IMR_RXOIR_UNMASK;
#ifdef SPI_DMA_RX
  if (rx_ch < 0) {
    imr |= SPI_IMR_RXFIM_UNMASK;
  }
#else
  imr |= SPI_IMR_RXFIM_UNMASK;
#endif
#ifdef SPI_DMA_TX
  if (tx_ch < 0) {
#endif
    if (two_byte_size) {
      p = (const uint16_t*)cmd;
      for (i = 0; i < cnt; ++i) {
        spim->DR[0] = *p++;
      }
    } else {
      for (i = 0; i < cnt; ++i) {
        spim->DR[0] = *cmd++;
      }
    }
    if (info->tx.cnt) {
      imr |= SPI_IMR_TXEIM_UNMASK;
    }
#ifdef SPI_DMA_TX
  }
#endif
  spim->IMR = imr;

  // wait for done
  while (info->busy) ;

  ret = ARM_SPI_OK;
  if (spim->RISR_b.RXOIR) {
    ret = ARM_SPI_ERROR;
  } else if (0 != info->tx.cnt) {
    ret = ARM_SPI_ERROR;
  }
  spim->SSIENR = SPI_SSIENR_DISABLE;
#ifdef SPI_WITH_SDMAC
  spim->DMACR  = 0;
#endif
  SPI_DMA_RX_UNINIT(rx_ch);
  SPI_DMA_TX_UNINIT(tx_ch);
  return ret;
}

#if (RTE_SPI0)

#if (RTE_SPIM0_DMA_TX || RTE_SPIM0_DMA_RX)
static void SPIM0_DMAHandler(uint32_t ch, uint32_t e);
#endif

static void SPI0_ConfigurePin(void)
{
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM0_MISO, PMU_DRIVE_CAPABILITY_2MA, SPIM0_RESISTOR);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM0_MOSI, SPIM0_DRIVE_CAPABILITY,   SPIM0_RESISTOR);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM0_CLK,  SPIM0_DRIVE_CAPABILITY,   PMU_IO_RESISTOR_NONE);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM0_CS_N, SPIM0_DRIVE_CAPABILITY,   PMU_IO_RESISTOR_NONE);
}

/* SPI0 Information (Run-Time) */
static SPI_INFO SPI0_Info;

/* SPI0 Resources */
static SPI_RESOURCES SPI0_Resources = {
  spim0,
  SPIM0_IRQn,
  SPI0_ConfigurePin,
  PMU_MODULE_SPIM0,
  PMU_IO_FUNC_SPIM0_MISO,
  PMU_CD_PPIER1,
  &SPI0_Info,
  SPI0_TX_FIFO_SIZE,
#ifdef SPI_WITH_SDMAC
#if (RTE_SPIM0_DMA_RX || RTE_SPIM0_DMA_TX)
  SPIM0_DMAHandler,
#else
  NULL,
#endif
#if (RTE_SPIM0_DMA_TX)
  {SDMAC_DST_SPIM0, RTE_SPIM0_DMA_TX_CH, RTE_SPIM0_DMA_TX_HS},
#else
  {SDMAC_DST_SPIM0, -1, -1},
#endif
#if (RTE_SPIM0_DMA_RX)
  {SDMAC_SRC_SPIM0, RTE_SPIM0_DMA_RX_CH, RTE_SPIM0_DMA_RX_HS},
#else
  {SDMAC_SRC_SPIM0, -1, -1},
#endif
#endif
};

static ARM_SPI_STATUS SPI0_Initialize(ARM_SPI_SignalEvent_t cb_event)
{
  return SPI_Initialize(cb_event, &SPI0_Resources);
}

static ARM_SPI_STATUS SPI0_Uninitialize(void)
{
  return SPI_Uninitialize(&SPI0_Resources);
}

static ARM_SPI_STATUS SPI0_PowerControl(ARM_POWER_STATE state)
{
  return SPI_PowerControl(state, &SPI0_Resources);
}

static ARM_SPI_STATUS SPI0_Configure(ARM_SPI_FRAME_FORMAT frame_format, ARM_SPI_BIT_ORDER bit_order)
{
  return SPI_Configure(frame_format, bit_order, &SPI0_Resources);
}

static uint32_t SPI0_BusSpeed(uint32_t bps)
{
    return SPI_BusSpeed(bps, &SPI0_Resources);
}

static ARM_SPI_STATUS SPI0_SlaveSelect(ARM_SPI_SS_SIGNAL ss)
{
  return SPI_SlaveSelect(ss, &SPI0_Resources);
}

static uint8_t SPI0_TransferByte(uint8_t out)
{
  return SPI_TransferByte(out, &SPI0_Resources);
}

static ARM_SPI_STATUS SPI0_SendData(const uint8_t *buf, uint32_t len)
{
  return SPI_SendFrames(buf, len, SPI_DFS_8BIT, &SPI0_Resources);
}

static ARM_SPI_STATUS SPI0_ReceiveData(uint8_t *buf, uint32_t len, uint8_t out)
{
  return SPI_ReceiveFrames(buf, len, out, SPI_DFS_8BIT, &SPI0_Resources);
}

static ARM_SPI_STATUS SPI0_AbortTransfer(void)
{
  return SPI_AbortTransfer(&SPI0_Resources);
}

static ARM_SPI_STATUS SPI0_FrameSize(uint32_t frame_size_m1)
{
  return SPI_FrameSize(frame_size_m1, &SPI0_Resources);
}

static uint16_t SPI0_TransferFrame(uint16_t frame)
{
  return SPI_TransferFrame(frame, &SPI0_Resources);
}

static ARM_SPI_STATUS SPI0_SendFrames(const uint8_t *buf, uint32_t len)
{
  return SPI_SendFrames(buf, len, SPI_DFS(SPI0_Resources.info->ctrlr0), &SPI0_Resources);
}

static ARM_SPI_STATUS SPI0_ReceiveFrames(uint8_t *buf, uint32_t len, uint16_t out)
{
  return SPI_ReceiveFrames(buf, len, out, SPI_DFS(SPI0_Resources.info->ctrlr0), &SPI0_Resources);
}

static ARM_SPI_STATUS SPI0_ReadROM(const uint8_t *cmd, uint32_t cmd_len, uint8_t *buf, uint32_t len)
{
  return SPI_ReadROM(cmd, cmd_len, buf, len, &SPI0_Resources);
}

void SPIM0_IRQHandler(void)
{
  SPI_IRQHandler(&SPI0_Resources);
}

#if (RTE_SPIM0_DMA_TX && RTE_SPIM0_DMA_RX)
static void SPIM0_DMAHandler(uint32_t ch, uint32_t e)
{
  if (ch == RTE_SPIM0_DMA_TX_CH) {
    SPI_DMATxHandler(&SPI0_Resources, e);
  } else {
    SPI_DMARxHandler(&SPI0_Resources, e);
  }
}
#elif (RTE_SPIM0_DMA_TX)
static void SPIM0_DMAHandler(uint32_t ch, uint32_t e)
{
  SPI_DMATxHandler(&SPI0_Resources, e);
}
#elif (RTE_SPIM0_DMA_RX)
static void SPIM0_DMAHandler(uint32_t ch, uint32_t e)
{
  SPI_DMARxHandler(&SPI0_Resources, e);
}
#endif

/* SPI0 Driver Control Block */
TZ10XX_DRIVER_SPI Driver_SPI0 = {
  SPIX_GetVersion,
  SPIX_GetCapabilities,
  SPI0_Initialize,
  SPI0_Uninitialize,
  SPI0_PowerControl,
  SPI0_Configure,
  SPI0_BusSpeed,
  SPI0_SlaveSelect,
  SPI0_TransferByte,
  SPI0_SendData,
  SPI0_ReceiveData,
  SPI0_AbortTransfer,
  SPI0_FrameSize,
  SPI0_TransferFrame,
  SPI0_SendFrames,
  SPI0_ReceiveFrames,
  SPI0_ReadROM
};
#endif

#if (RTE_SPI1)

#if (RTE_SPIM1_DMA_TX || RTE_SPIM1_DMA_RX)
static void SPIM1_DMAHandler(uint32_t ch, uint32_t e);
#endif

static void SPI1_ConfigurePin(void)
{
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM1_MISO, PMU_DRIVE_CAPABILITY_2MA, SPIM1_RESISTOR);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM1_MOSI, SPIM1_DRIVE_CAPABILITY,   SPIM1_RESISTOR);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM1_CLK,  SPIM1_DRIVE_CAPABILITY,   PMU_IO_RESISTOR_NONE);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM1_CS_N, SPIM1_DRIVE_CAPABILITY,   PMU_IO_RESISTOR_NONE);
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_SPIM1_MISO, 0);
}

/* SPI1 Information (Run-Time) */
static SPI_INFO SPI1_Info;

/* SPI1 Resources */
static SPI_RESOURCES SPI1_Resources = {
  spim1,
  SPIM1_IRQn,
  SPI1_ConfigurePin,
  PMU_MODULE_SPIM1,
  PMU_IO_FUNC_SPIM1_MISO,
  PMU_CD_PPIER1,
  &SPI1_Info,
  SPI1_TX_FIFO_SIZE,
#ifdef SPI_WITH_SDMAC
#if (RTE_SPIM1_DMA_RX || RTE_SPIM1_DMA_TX)
  SPIM1_DMAHandler,
#else
  NULL,
#endif
#if (RTE_SPIM1_DMA_TX)
  {SDMAC_DST_SPIM1, RTE_SPIM1_DMA_TX_CH, RTE_SPIM1_DMA_TX_HS},
#else
  {SDMAC_DST_SPIM1, -1, -1},
#endif
#if (RTE_SPIM1_DMA_RX)
  {SDMAC_SRC_SPIM1, RTE_SPIM1_DMA_RX_CH, RTE_SPIM1_DMA_RX_HS},
#else
  {SDMAC_SRC_SPIM1, -1, -1},
#endif
#endif
};

static ARM_SPI_STATUS SPI1_Initialize(ARM_SPI_SignalEvent_t cb_event)
{
  return SPI_Initialize(cb_event, &SPI1_Resources);
}

static ARM_SPI_STATUS SPI1_Uninitialize(void)
{
  return SPI_Uninitialize(&SPI1_Resources);
}

static ARM_SPI_STATUS SPI1_PowerControl(ARM_POWER_STATE state)
{
  return SPI_PowerControl(state, &SPI1_Resources);
}

static ARM_SPI_STATUS SPI1_Configure(ARM_SPI_FRAME_FORMAT frame_format, ARM_SPI_BIT_ORDER bit_order)
{
  return SPI_Configure(frame_format, bit_order, &SPI1_Resources);
}

static uint32_t SPI1_BusSpeed(uint32_t bps)
{
    return SPI_BusSpeed(bps, &SPI1_Resources);
}

static ARM_SPI_STATUS SPI1_SlaveSelect(ARM_SPI_SS_SIGNAL ss)
{
  return SPI_SlaveSelect(ss, &SPI1_Resources);
}

static uint8_t SPI1_TransferByte(uint8_t out)
{
  return SPI_TransferByte(out, &SPI1_Resources);
}

static ARM_SPI_STATUS SPI1_SendData(const uint8_t *buf, uint32_t len)
{
  return SPI_SendFrames(buf, len, SPI_DFS_8BIT, &SPI1_Resources);
}

static ARM_SPI_STATUS SPI1_ReceiveData(uint8_t *buf, uint32_t len, uint8_t out)
{
  return SPI_ReceiveFrames(buf, len, out, SPI_DFS_8BIT, &SPI1_Resources);
}

static ARM_SPI_STATUS SPI1_AbortTransfer(void)
{
  return SPI_AbortTransfer(&SPI1_Resources);
}

static ARM_SPI_STATUS SPI1_FrameSize(uint32_t frame_size_m1)
{
  return SPI_FrameSize(frame_size_m1, &SPI1_Resources);
}

static uint16_t SPI1_TransferFrame(uint16_t frame)
{
  return SPI_TransferFrame(frame, &SPI1_Resources);
}

static ARM_SPI_STATUS SPI1_SendFrames(const uint8_t *buf, uint32_t len)
{
  return SPI_SendFrames(buf, len, SPI_DFS(SPI1_Resources.info->ctrlr0), &SPI1_Resources);
}

static ARM_SPI_STATUS SPI1_ReceiveFrames(uint8_t *buf, uint32_t len, uint16_t out)
{
  return SPI_ReceiveFrames(buf, len, out, SPI_DFS(SPI1_Resources.info->ctrlr0), &SPI1_Resources);
}

static ARM_SPI_STATUS SPI1_ReadROM(const uint8_t *cmd, uint32_t cmd_len, uint8_t *buf, uint32_t len)
{
  return SPI_ReadROM(cmd, cmd_len, buf, len, &SPI1_Resources);
}

void SPIM1_IRQHandler(void)
{
  SPI_IRQHandler(&SPI1_Resources);
}

#if (RTE_SPIM1_DMA_TX && RTE_SPIM1_DMA_RX)
static void SPIM1_DMAHandler(uint32_t ch, uint32_t e)
{
  if (ch == RTE_SPIM1_DMA_TX_CH) {
    SPI_DMATxHandler(&SPI1_Resources, e);
  } else {
    SPI_DMARxHandler(&SPI1_Resources, e);
  }
}
#elif (RTE_SPIM1_DMA_TX)
static void SPIM1_DMAHandler(uint32_t ch, uint32_t e)
{
  SPI_DMATxHandler(&SPI1_Resources, e);
}
#elif (RTE_SPIM1_DMA_RX)
static void SPIM1_DMAHandler(uint32_t ch, uint32_t e)
{
  SPI_DMARxHandler(&SPI1_Resources, e);
}
#endif

/* SPI1 Driver Control Block */
TZ10XX_DRIVER_SPI Driver_SPI1 = {
  SPIX_GetVersion,
  SPIX_GetCapabilities,
  SPI1_Initialize,
  SPI1_Uninitialize,
  SPI1_PowerControl,
  SPI1_Configure,
  SPI1_BusSpeed,
  SPI1_SlaveSelect,
  SPI1_TransferByte,
  SPI1_SendData,
  SPI1_ReceiveData,
  SPI1_AbortTransfer,
  SPI1_FrameSize,
  SPI1_TransferFrame,
  SPI1_SendFrames,
  SPI1_ReceiveFrames,
  SPI1_ReadROM
};
#endif

#if (RTE_SPI2)

#if (RTE_SPIM2_DMA_RX || RTE_SPIM2_DMA_TX)
static void SPIM2_DMAHandler(uint32_t ch, uint32_t e);
#endif

static void SPI2_ConfigurePin(void)
{
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM2_MISO, PMU_DRIVE_CAPABILITY_2MA, SPIM2_RESISTOR);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM2_MOSI, SPIM2_DRIVE_CAPABILITY,   SPIM2_RESISTOR);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM2_CLK,  SPIM2_DRIVE_CAPABILITY,   PMU_IO_RESISTOR_NONE);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM2_CS_N, SPIM2_DRIVE_CAPABILITY,   PMU_IO_RESISTOR_NONE);
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_SPIM2_MISO, 0);
}

/* SPI2 Information (Run-Time) */
static SPI_INFO SPI2_Info;

/* SPI2 Resources */
static SPI_RESOURCES SPI2_Resources = {
  spim2,
  SPIM2_IRQn,
  SPI2_ConfigurePin,
  PMU_MODULE_SPIM2,
  PMU_IO_FUNC_SPIM2_MISO,
  PMU_CD_PPIER0,
  &SPI2_Info,
  SPI2_TX_FIFO_SIZE,
#ifdef SPI_WITH_SDMAC
#if (RTE_SPIM2_DMA_RX || RTE_SPIM2_DMA_TX)
  SPIM2_DMAHandler,
#else
  NULL,
#endif
#if (RTE_SPIM2_DMA_TX)
  {SDMAC_DST_SPIM2, RTE_SPIM2_DMA_TX_CH, RTE_SPIM2_DMA_TX_HS},
#else
  {SDMAC_DST_SPIM2, -1, -1},
#endif
#if (RTE_SPIM2_DMA_RX)
  {SDMAC_SRC_SPIM2, RTE_SPIM2_DMA_RX_CH, RTE_SPIM2_DMA_RX_HS},
#else
  {SDMAC_SRC_SPIM2, -1, -1},
#endif
#endif
};

static ARM_SPI_STATUS SPI2_Initialize(ARM_SPI_SignalEvent_t cb_event)
{
  return SPI_Initialize(cb_event, &SPI2_Resources);
}

static ARM_SPI_STATUS SPI2_Uninitialize(void)
{
  return SPI_Uninitialize(&SPI2_Resources);
}

static ARM_SPI_STATUS SPI2_PowerControl(ARM_POWER_STATE state)
{
  return SPI_PowerControl(state, &SPI2_Resources);
}

static ARM_SPI_STATUS SPI2_Configure(ARM_SPI_FRAME_FORMAT frame_format, ARM_SPI_BIT_ORDER bit_order)
{
  return SPI_Configure(frame_format, bit_order, &SPI2_Resources);
}

static ARM_SPI_STATUS SPI2_SlaveSelect(ARM_SPI_SS_SIGNAL ss)
{
  return SPI_SlaveSelect(ss, &SPI2_Resources);
}

static uint32_t SPI2_BusSpeed(uint32_t bps)
{
    return SPI_BusSpeed(bps, &SPI2_Resources);
}

static uint8_t SPI2_TransferByte(uint8_t out)
{
  return SPI_TransferByte(out, &SPI2_Resources);
}

static ARM_SPI_STATUS SPI2_SendData(const uint8_t *buf, uint32_t len)
{
  return SPI_SendFrames(buf, len, SPI_DFS_8BIT, &SPI2_Resources);
}

static ARM_SPI_STATUS SPI2_ReceiveData(uint8_t *buf, uint32_t len, uint8_t out)
{
  return SPI_ReceiveFrames(buf, len, out, SPI_DFS_8BIT, &SPI2_Resources);
}

static ARM_SPI_STATUS SPI2_AbortTransfer(void)
{
  return SPI_AbortTransfer(&SPI2_Resources);
}

static ARM_SPI_STATUS SPI2_FrameSize(uint32_t frame_size_m1)
{
  return SPI_FrameSize(frame_size_m1, &SPI2_Resources);
}

static uint16_t SPI2_TransferFrame(uint16_t frame)
{
  return SPI_TransferFrame(frame, &SPI2_Resources);
}

static ARM_SPI_STATUS SPI2_SendFrames(const uint8_t *buf, uint32_t len)
{
  return SPI_SendFrames(buf, len, SPI_DFS(SPI2_Resources.info->ctrlr0), &SPI2_Resources);
}

static ARM_SPI_STATUS SPI2_ReceiveFrames(uint8_t *buf, uint32_t len, uint16_t out)
{
  return SPI_ReceiveFrames(buf, len, out, SPI_DFS(SPI2_Resources.info->ctrlr0), &SPI2_Resources);
}

static ARM_SPI_STATUS SPI2_ReadROM(const uint8_t *cmd, uint32_t cmd_len, uint8_t *buf, uint32_t len)
{
  return SPI_ReadROM(cmd, cmd_len, buf, len, &SPI2_Resources);
}

void SPIM2_IRQHandler(void)
{
  SPI_IRQHandler(&SPI2_Resources);
}

#if (RTE_SPIM2_DMA_TX && RTE_SPIM2_DMA_RX)
static void SPIM2_DMAHandler(uint32_t ch, uint32_t e)
{
  if (ch == RTE_SPIM2_DMA_TX_CH) {
    SPI_DMATxHandler(&SPI2_Resources, e);
  } else {
    SPI_DMARxHandler(&SPI2_Resources, e);
  }
}
#elif (RTE_SPIM2_DMA_TX)
static void SPIM2_DMAHandler(uint32_t ch, uint32_t e)
{
  SPI_DMATxHandler(&SPI2_Resources, e);
}
#elif (RTE_SPIM2_DMA_RX)
static void SPIM2_DMAHandler(uint32_t ch, uint32_t e)
{
  SPI_DMARxHandler(&SPI2_Resources, e);
}
#endif

/* SPI2 Driver Control Block */
TZ10XX_DRIVER_SPI Driver_SPI2 = {
  SPIX_GetVersion,
  SPIX_GetCapabilities,
  SPI2_Initialize,
  SPI2_Uninitialize,
  SPI2_PowerControl,
  SPI2_Configure,
  SPI2_BusSpeed,
  SPI2_SlaveSelect,
  SPI2_TransferByte,
  SPI2_SendData,
  SPI2_ReceiveData,
  SPI2_AbortTransfer,
  SPI2_FrameSize,
  SPI2_TransferFrame,
  SPI2_SendFrames,
  SPI2_ReceiveFrames,
  SPI2_ReadROM
};
#endif

#if (RTE_SPI3)

#if (RTE_SPIM3_DMA_RX || RTE_SPIM3_DMA_TX)
static void SPIM3_DMAHandler(uint32_t ch, uint32_t e);
#endif

static void SPI3_ConfigurePin(void)
{
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM3_MISO, PMU_DRIVE_CAPABILITY_2MA, SPIM3_RESISTOR);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM3_MOSI, SPIM3_DRIVE_CAPABILITY,   SPIM3_RESISTOR);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM3_CLK,  SPIM3_DRIVE_CAPABILITY,   PMU_IO_RESISTOR_NONE);
  Driver_PMU.ConfigureIOCell(PMU_IO_FUNC_SPIM3_CS_N, SPIM3_DRIVE_CAPABILITY,   PMU_IO_RESISTOR_NONE);
  Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_SPIM3_MISO, 0);
}

/* SPI3 Information (Run-Time) */
static SPI_INFO SPI3_Info;

/* SPI3 Resources */
static SPI_RESOURCES SPI3_Resources = {
  spim3,
  SPIM3_IRQn,
  SPI3_ConfigurePin,
  PMU_MODULE_SPIM3,
  PMU_IO_FUNC_SPIM3_MISO,
  PMU_CD_PPIER0,
  &SPI3_Info,
  SPI3_TX_FIFO_SIZE,
#ifdef SPI_WITH_SDMAC
#if (RTE_SPIM3_DMA_RX || RTE_SPIM3_DMA_TX)
  SPIM3_DMAHandler,
#else
  NULL,
#endif
#if (RTE_SPIM3_DMA_TX)
  {SDMAC_DST_SPIM3, RTE_SPIM3_DMA_TX_CH, RTE_SPIM3_DMA_TX_HS},
#else
  {SDMAC_DST_SPIM3, -1, -1},
#endif
#if (RTE_SPIM3_DMA_RX)
  {SDMAC_SRC_SPIM3, RTE_SPIM3_DMA_RX_CH, RTE_SPIM3_DMA_RX_HS},
#else
  {SDMAC_SRC_SPIM3, -1, -1},
#endif
#endif
};

static ARM_SPI_STATUS SPI3_Initialize(ARM_SPI_SignalEvent_t cb_event)
{
  return SPI_Initialize(cb_event, &SPI3_Resources);
}

static ARM_SPI_STATUS SPI3_Uninitialize(void)
{
  return SPI_Uninitialize(&SPI3_Resources);
}

static ARM_SPI_STATUS SPI3_PowerControl(ARM_POWER_STATE state)
{
  return SPI_PowerControl(state, &SPI3_Resources);
}

static ARM_SPI_STATUS SPI3_Configure(ARM_SPI_FRAME_FORMAT frame_format, ARM_SPI_BIT_ORDER bit_order)
{
  return SPI_Configure(frame_format, bit_order, &SPI3_Resources);
}

static uint32_t SPI3_BusSpeed(uint32_t bps)
{
    return SPI_BusSpeed(bps, &SPI3_Resources);
}

static ARM_SPI_STATUS SPI3_SlaveSelect(ARM_SPI_SS_SIGNAL ss)
{
  return SPI_SlaveSelect(ss, &SPI3_Resources);
}

static uint8_t SPI3_TransferByte(uint8_t out)
{
  return SPI_TransferByte(out, &SPI3_Resources);
}

static ARM_SPI_STATUS SPI3_SendData(const uint8_t *buf, uint32_t len)
{
  return SPI_SendFrames(buf, len, SPI_DFS_8BIT, &SPI3_Resources);
}

static ARM_SPI_STATUS SPI3_ReceiveData(uint8_t *buf, uint32_t len, uint8_t out)
{
  return SPI_ReceiveFrames(buf, len, out, SPI_DFS_8BIT, &SPI3_Resources);
}

static ARM_SPI_STATUS SPI3_AbortTransfer(void)
{
  return SPI_AbortTransfer(&SPI3_Resources);
}

static ARM_SPI_STATUS SPI3_FrameSize(uint32_t frame_size_m1)
{
  return SPI_FrameSize(frame_size_m1, &SPI3_Resources);
}

static uint16_t SPI3_TransferFrame(uint16_t frame)
{
  return SPI_TransferFrame(frame, &SPI3_Resources);
}

static ARM_SPI_STATUS SPI3_SendFrames(const uint8_t *buf, uint32_t len)
{
  return SPI_SendFrames(buf, len, SPI_DFS(SPI3_Resources.info->ctrlr0), &SPI3_Resources);
}

static ARM_SPI_STATUS SPI3_ReceiveFrames(uint8_t *buf, uint32_t len, uint16_t out)
{
  return SPI_ReceiveFrames(buf, len, out, SPI_DFS(SPI3_Resources.info->ctrlr0), &SPI3_Resources);
}

static ARM_SPI_STATUS SPI3_ReadROM(const uint8_t *cmd, uint32_t cmd_len, uint8_t *buf, uint32_t len)
{
  return SPI_ReadROM(cmd, cmd_len, buf, len, &SPI3_Resources);
}

void SPIM3_IRQHandler(void)
{
  SPI_IRQHandler(&SPI3_Resources);
}

#if (RTE_SPIM3_DMA_TX && RTE_SPIM3_DMA_RX)
static void SPIM3_DMAHandler(uint32_t ch, uint32_t e)
{
  if (ch == RTE_SPIM3_DMA_TX_CH) {
    SPI_DMATxHandler(&SPI3_Resources, e);
  } else {
    SPI_DMARxHandler(&SPI3_Resources, e);
  }
}
#elif (RTE_SPIM3_DMA_TX)
static void SPIM3_DMAHandler(uint32_t ch, uint32_t e)
{
  SPI_DMATxHandler(&SPI3_Resources, e);
}
#elif (RTE_SPIM3_DMA_RX)
static void SPIM3_DMAHandler(uint32_t ch, uint32_t e)
{
  SPI_DMARxHandler(&SPI3_Resources, e);
}
#endif

/* SPI3 Driver Control Block */
TZ10XX_DRIVER_SPI Driver_SPI3 = {
  SPIX_GetVersion,
  SPIX_GetCapabilities,
  SPI3_Initialize,
  SPI3_Uninitialize,
  SPI3_PowerControl,
  SPI3_Configure,
  SPI3_BusSpeed,
  SPI3_SlaveSelect,
  SPI3_TransferByte,
  SPI3_SendData,
  SPI3_ReceiveData,
  SPI3_AbortTransfer,
  SPI3_FrameSize,
  SPI3_TransferFrame,
  SPI3_SendFrames,
  SPI3_ReceiveFrames,
  SPI3_ReadROM
};
#endif
#endif
