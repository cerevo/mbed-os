/**
 * @file SPI_TZ10xx.h
 * @brief a header file for TZ10xx SPI driver
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
#ifndef SPI_TZ10XX_H
#define SPI_TZ10XX_H

#include <Driver_SPI.h>
#include <RTE_Device.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _TZ10XX_DRIVER_SPI {
  ARM_DRIVER_VERSION      (*GetVersion)(void);
  ARM_SPI_CAPABILITIES (*GetCapabilities)(void);
  ARM_SPI_STATUS       (*Initialize)     (ARM_SPI_SignalEvent_t cb_event);
  ARM_SPI_STATUS       (*Uninitialize)   (void);
  ARM_SPI_STATUS       (*PowerControl)   (ARM_POWER_STATE state);
  ARM_SPI_STATUS       (*Configure)      (ARM_SPI_FRAME_FORMAT frame_format,
                                          ARM_SPI_BIT_ORDER    bit_order);
  uint32_t             (*BusSpeed)       (      uint32_t bps);
  ARM_SPI_STATUS       (*SlaveSelect)    (ARM_SPI_SS_SIGNAL ss);
  uint8_t              (*TransferByte)   (      uint8_t out);
  ARM_SPI_STATUS       (*SendData)       (const uint8_t *buf, uint32_t len);
  ARM_SPI_STATUS       (*ReceiveData)    (      uint8_t *buf, uint32_t len,
                                                uint8_t  out);
  ARM_SPI_STATUS       (*AbortTransfer)  (void);
  ARM_SPI_STATUS       (*FrameSize)      (      uint32_t frame_size);
  uint16_t              (*TransferFrame)  (      uint16_t frame);
  ARM_SPI_STATUS       (*SendFrames)     (const uint8_t *buf, uint32_t len);
  ARM_SPI_STATUS       (*ReceiveFrames)  (      uint8_t *buf, uint32_t len,
                                                uint16_t out);
  ARM_SPI_STATUS       (*ReadROM)        (const uint8_t *cmd, uint32_t cmd_len,
                                                uint8_t *buf, uint32_t len);
} const TZ10XX_DRIVER_SPI;


#if (RTE_SPI0)
extern TZ10XX_DRIVER_SPI Driver_SPI0;
#endif
#if (RTE_SPI1)
extern TZ10XX_DRIVER_SPI Driver_SPI1;
#endif
#if (RTE_SPI2)
extern TZ10XX_DRIVER_SPI Driver_SPI2;
#endif
#if (RTE_SPI3)
extern TZ10XX_DRIVER_SPI Driver_SPI3;
#endif

#ifdef __cplusplus
}
#endif

#endif /* SPI_TZ10XX_H */
