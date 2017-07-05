/**
 * @file ip_config.h
 * @brief DesignWare Cores USB 2.0 Hi-Speed On-The-Go IP configuration of TZ1000
 * @version V0.0
 * @date $Date:: 2014-07-09 09:31:57 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef __IP_CONFIG_H__
#define __IP_CONFIG_H__

// balvenie_usb2fs_spec.pdf

// balvenie_usb2fs_spec_ports_registers.xls
//   GHWCFG1
//   GHWCFG2
//   GHWCFG3
//   GHWCFG4

#define OTG_MODE 4
#define OTG_ARCHITECTURE 2
#define OTG_SINGLE_POINT 0
#define OTG_ENABLE_LPM 0
#define OTG_EN_DED_TX_FIFO 0
#define OTG_EN_DESC_DMA 0   // OTG_EN_DED_TX_FIFO == 1 の場合に有効
#define OTG_MULTI_PROC_INTRPT 1
#define OTG_HSPHY_INTERFACE 0
#define OTG_HSPHY_DWIDTH 2
#define OTG_FSPHY_INTERFACE 1
#define OTG_ENABLE_IC_USB // invalid
#define OTG_SELECT_IC_USB // invalid
#define OTG_ENABLE_HSIC 0
#define OTG_I2C_INTERFACE 0
#define OTG_ULPI_CARKIT   // invalid
#define OTG_ADP_SUPPORT 0
#define OTG_BC_SUPPORT 0
#define OTG_VENDOR_CTL_INTERFACE 0
#define OTG_NUM_EPS 3
#define OTG_NUM_PERIO_EPS 1
#define OTG_NUM_IN_EPS 2  // OTG_EN_DED_TX_FIFO == 1 の場合に有効
#define OTG_NUM_CRL_EPS 0

// EP_LOC_CNT : EPnum+Dirを1として数えた合計の数。
  // EP0IN,EP0OUT, EP1IN,EP1OUT, EP2IN,EP2OUT, EP3IN,EP3OUT, 
#define OTG_EP_LOC_CNT 8

#define OTG_DFIFO_DEPTH 160
#define OTG_DFIFO_DYNAMIC 1
#define OTG_RX_DFIFO_DEPTH 160
#define OTG_TX_HNPERIO_DFIFO_DEPTH // Host用なのでInvalid
#define OTG_TX_NPERIO_DFIFO_DEPTH 32
#define OTG_TX_HPERIO_DFIFO_DEPTH // Host用なのでInvalid
#define OTG_NPERIO_TX_QUEUE_DEPTH 8  // OTG_EN_DED_TX_FIFO == 1 の場合に有効
#define OTG_PERIO_TX_QUEUE_DEPTH 2
#define OTG_TOKEN_QUEUE_DEPTH 8
#define OTG_TRANS_COUNT_WIDTH 19
#define OTG_PACKET_COUNT_WIDTH 10
#define OTG_RM_OPT_FEATURES 1
#define OTG_USERID // invalid
#define OTG_EN_PWROPT 1 // PartialPowerDown
#define OTG_MIN_AHB_FREQ_LESSTHAN_60 1
#define OTG_SYNC_RESET_TYPE 0 // Asynchronous
#define OTG_INST_DSYNC_FLOPS 0
#define OTG_EN_IDDIG_FILTER 0
#define OTG_EN_VBUSVALID_FILTER 0
#define OTG_EN_A_VALID_FILTER 0
#define OTG_EN_B_VALID_FILTER 0
#define OTG_EN_SESSIONEND_FILTER 0
#define OTG_EXCP_CNTL_XFER_FLOW 1

// EP毎に有効な転送方向
#define OTG_EP_DIR_IN    0x1
#define OTG_EP_DIR_OUT   0x2
#define OTG_EP_DIR_INOUT (OTG_EP_DIR_OUT | OTG_EP_DIR_IN)
// OTG_NUM_EPS の数だけ
#define OTG_EP_DIR_1  OTG_EP_DIR_INOUT
#define OTG_EP_DIR_2  OTG_EP_DIR_INOUT
#define OTG_EP_DIR_3  OTG_EP_DIR_INOUT

#if OTG_EN_DED_TX_FIFO == 1
// TxFIFO Numberは、EP Numberとは別の独立したもの。
// diepctlN, txfnum フィールドでTxFIFO Numberを指定する。
// 送信FIFOの数
#define OTG_TX_DINEP_DFIFO_NUM 0
#define OTG_TX_DINEP_DFIFO_DEPTH_0
#define OTG_TX_DINEP_DFIFO_DEPTH_1
#define OTG_TX_DINEP_DFIFO_DEPTH_2
#define OTG_TX_DINEP_DFIFO_DEPTH_3
#define OTG_TX_DINEP_DFIFO_DEPTH_4
#endif

#define OTG_TX_DPERIO_DFIFO_DEPTH_1  32
#define OTG_TX_DPERIO_DFIFO_DEPTH_2  32
#define OTG_TX_DPERIO_DFIFO_DEPTH_3  32
#define OTG_TX_DPERIO_DFIFO_DEPTH_4  32
#define OTG_TX_DPERIO_DFIFO_DEPTH_5  32
#define OTG_TX_DPERIO_DFIFO_DEPTH_6  32
#define OTG_TX_DPERIO_DFIFO_DEPTH_7  32
#define OTG_TX_DPERIO_DFIFO_DEPTH_8  32
#define OTG_TX_DPERIO_DFIFO_DEPTH_9  32
#define OTG_TX_DPERIO_DFIFO_DEPTH_10 32
#define OTG_TX_DPERIO_DFIFO_DEPTH_11 32
#define OTG_TX_DPERIO_DFIFO_DEPTH_12 32
#define OTG_TX_DPERIO_DFIFO_DEPTH_13 32
#define OTG_TX_DPERIO_DFIFO_DEPTH_14 32
#define OTG_TX_DPERIO_DFIFO_DEPTH_15 32

// １度にDMAできるバイト数
#define OTG_TRANS_MAX_SIZE   ((1 << OTG_TRANS_COUNT_WIDTH) - 1)
// １度にDMAできるパケット個数
#define OTG_PACKET_MAX_COUNT ((1 << OTG_PACKET_COUNT_WIDTH) - 1)

#endif /* __IP_CONFIG_H__ */
