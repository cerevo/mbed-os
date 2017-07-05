/**
 * @file SDMAC_TZ10xx.h
 * @brief a header file for TZ10xx SDMAC driver
 * @date $Date:: 2014-07-10 10:50:34 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */
#ifndef SDMAC_TZ10XX_H
#define SDMAC_TZ10XX_H

#include "Driver_Common.h"
#include "RTE_Device.h"

#ifdef __cplusplus
extern "C" {
#endif

/* API version */
#define TZ1000_SDMAC_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)

#define SDMAC_CTL(_src_width, _src_inc, _src_msize, _dst_width, _dst_inc, _dst_msize) \
    (((_src_width)<<4) | ((_src_inc)<<9) | ((_src_msize)<<14) | ((_dst_width)<<1) | ((_dst_inc)<<7) | ((_dst_msize)<<11))

typedef enum _SDMAC_STATUS {
  SDMAC_OK,
  SDMAC_ERROR,
  SDMAC_ERROR_CH,
  SDMAC_ERROR_HS,
  SDMAC_ERROR_BUSY,
} SDMAC_STATUS;

typedef enum _SDMAC_EVENT {
  SDMAC_EVENT_TRANSFER    = 1,
  SDMAC_EVENT_ERROR       = 16,
} SDMAC_EVENT;

typedef enum _SDMAC_SRC {
  SDMAC_SRC_MEMORY      = -1,
  SDMAC_SRC_SPIM0       = 0,
  SDMAC_SRC_SPIM1       = 1,
  SDMAC_SRC_SPIM2       = 2,
  SDMAC_SRC_SPIM3       = 3,
  SDMAC_SRC_I2C0        = 4,
  SDMAC_SRC_I2C1        = 5,
  SDMAC_SRC_I2C2        = 6,
  SDMAC_SRC_UART0       = 7,
  SDMAC_SRC_UART1       = 8,
  SDMAC_SRC_UART2       = 9,
  SDMAC_SRC_ADC12_CH0   = 20,
  SDMAC_SRC_ADC12_CH1   = 21,
  SDMAC_SRC_ADC12_CH2   = 22,
  SDMAC_SRC_ADC12_CH3   = 23,
  SDMAC_SRC_ADVTMR_CMP0 = 24,
  SDMAC_SRC_ADVTMR_CMP1 = 25,
  SDMAC_SRC_ADVTMR_CMP2 = 26,
  SDMAC_SRC_ADVTMR_CMP3 = 27,
  SDMAC_SRC_ADVTMR_CAP0 = 28,
  SDMAC_SRC_ADVTMR_CAP1 = 29,
  SDMAC_SRC_ADVTMR_CAP2 = 30,
  SDMAC_SRC_ADVTMR_CAP3 = 31,
  SDMAC_SRC_ADC24_CH0   = 32,
  SDMAC_SRC_ADC24_CH1   = 33,
  SDMAC_SRC_ADC24_CH2   = 34,
} SDMAC_SRC;

typedef enum _SDMAC_DST {
  SDMAC_DST_MEMORY      = -1,
  SDMAC_DST_SPIM0       = 10,
  SDMAC_DST_SPIM1       = 11,
  SDMAC_DST_SPIM2       = 12,
  SDMAC_DST_SPIM3       = 13,
  SDMAC_DST_I2C0        = 14,
  SDMAC_DST_I2C1        = 15,
  SDMAC_DST_I2C2        = 16,
  SDMAC_DST_UART0       = 17,
  SDMAC_DST_UART1       = 18,
  SDMAC_DST_UART2       = 19,
} SDMAC_DST;

typedef enum _SDMAC_WIDTH {
  SDMAC_WIDTH_1,
  SDMAC_WIDTH_2,
  SDMAC_WIDTH_4,
} SDMAC_WIDTH;

typedef enum _SDMAC_INC {
  SDMAC_INC_INCREMENT,          /* increment after single transaction */
  SDMAC_INC_DECREMENT,          /* decrement after single transaction */
  SDMAC_INC_NO_CHANGE,          /* no change address */
} SDMAC_INC;

typedef enum _SDMAC_MSIZE {
  SDMAC_MSIZE_1,
  SDMAC_MSIZE_4,
  SDMAC_MSIZE_8,
  SDMAC_MSIZE_16,
} SDMAC_MSIZE;

typedef enum _SDMAC_FIFO_MODE {
  SDMAC_FIFO_MODE_LATENCY,
  SDMAC_FIFO_MODE_UTILIZATION,
} SDMAC_FIFO_MODE;

typedef enum _SDMAC_PRIORITY {
  SDMAC_PRIORITY_LOWEST  = 0,
  SDMAC_PRIORITY_1       = (1<<5),
  SDMAC_PRIORITY_2       = (2<<5),
  SDMAC_PRIORITY_3       = (3<<5),
  SDMAC_PRIORITY_4       = (4<<5),
  SDMAC_PRIORITY_5       = (5<<5),
  SDMAC_PRIORITY_6       = (6<<5),
  SDMAC_PRIORITY_HIGHEST = (7<<5),
} SDMAC_PRIORITY;

typedef void (*SDMAC_SignalEvent_t)(uint32_t ch, uint32_t event);

typedef struct _TZ10XX_DRIVER_SDMAC {
  ARM_DRIVER_VERSION (*GetVersion)                  (void);
  SDMAC_STATUS    (*Initialize)                  (void);
  SDMAC_STATUS    (*Uninitialize)                (void);
  SDMAC_STATUS    (*PowerControl)                (ARM_POWER_STATE state);
  SDMAC_STATUS    (*ChannelInitialize)           (int32_t ch, SDMAC_SignalEvent_t cb_event,
                                                  SDMAC_SRC src_hw, SDMAC_DST dst_hw,
                                                  SDMAC_FIFO_MODE mode, SDMAC_PRIORITY priority,
                                                  int32_t src_handshake, int32_t dst_handshake);
  SDMAC_STATUS    (*ChannelUninitialize)         (int32_t ch);
  int32_t         (*ChannelAcquire)              (SDMAC_SignalEvent_t cb_event,
                                                  SDMAC_SRC src_hw, SDMAC_DST dst_hw,
                                                  SDMAC_FIFO_MODE mode, SDMAC_PRIORITY priority);
  SDMAC_STATUS    (*ChannelRelease)              (int32_t ch);
  SDMAC_STATUS    (*ChannelTransferData)         (int32_t ch, const uint8_t *src, uint8_t* dst,
                                                  uint32_t size, uint32_t ctl);
  bool            (*ChannelBusy)                 (int32_t ch);
  SDMAC_STATUS    (*ChannelAbortTransfer)        (int32_t ch);
  SDMAC_STATUS    (*ChannelCountBytesTransferred)(int32_t ch, uint32_t *num);
} const TZ10XX_DRIVER_SDMAC;

#ifdef __cplusplus
}
#endif

#endif /* SDMAC_TZ10XX_H */
