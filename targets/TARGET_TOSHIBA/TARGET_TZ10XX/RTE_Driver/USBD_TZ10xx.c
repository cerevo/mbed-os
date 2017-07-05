/**
 * @file USBD_TZ10xx.c
 * @brief TZ10xx USB Device driver
 */

/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_pcd_intr.c $
 * $Revision: #125 $
 * $Date: 2013/05/20 $
 * $Change: 2234037 $
 * 
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_pcd.c $
 * $Revision: #105 $
 * $Date: 2013/05/16 $
 * $Change: 2231774 $
 * 
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_cil_intr.c $
 * $Revision: #37 $
 * $Date: 2013/04/16 $
 * $Change: 2207267 $
 * 
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_cil.c $
 * $Revision: #203 $
 * $Date: 2013/05/16 $
 * $Change: 2231774 $
 * 
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_cil.h $
 * $Revision: #128 $
 * $Date: 2013/05/16 $
 * $Change: 2231774 $
 * 
 * Synopsys HS OTG Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================== */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 * ALL RIGHTS RESERVED
 *
 * Portions Copyright (C) 2013 Synopsys, Inc.  
 * Used with permission. All rights reserved. 
 * Synopsys & DesignWare are registered trademarks of Synopsys, Inc.
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 199901L)
#error ISO/IEC 9899:1999 or later version is required to compile this C source.
#endif

#include "usb_misc.h"
#include "usbdebug.h"
#include <string.h>   // memcpy

#include "USBD_TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "TZ10xx.h"
#include "RTE_Device.h"
#include "RTE_Components.h"

#include "ip_config.h"
#include "dwc_otg_regs.h"
#include "dwc_os.h"
#include "usb_device.h"
#include "usb_desc.h"

#define ARM_USBD_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,38)   /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
	ARM_USBD_API_VERSION,
	ARM_USBD_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_USBD_CAPABILITIES DriverCapabilities = {
	.event_power_on      = 0,
	.event_power_off     = 0,
	.event_connect       = 0,
	.event_disconnect    = 0,
	.event_reset         = 1,
	.event_high_speed    = 0,
	.event_suspend       = 1,
	.event_resume        = 1,
	.event_remote_wakeup = 0,
	.reserved            = 0,
};

/* Number of endpoints*/
#if RTE_USB2FS_NUM_EPS > (1 + OTG_NUM_EPS)
#error RTE_USB2FS_NUM_EPS value exceeded.
#else
#define USB_NUM_EPS     RTE_USB2FS_NUM_EPS
#endif

// Driver_USBD.h/typedef struct _ARM_DRIVER_USBD {
static ARM_DRIVER_VERSION       USBD_GetVersion        (void);
static ARM_USBD_CAPABILITIES USBD_GetCapabilities   (void);
static ARM_USBD_STATUS       USBD_PowerControl       (ARM_POWER_STATE state);
static ARM_USBD_STATUS       USBD_Initialize         (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                                      ARM_USBD_SignalEndpointEvent_t cb_endpoint_event);
static ARM_USBD_STATUS       USBD_Uninitialize       (void);
static ARM_USBD_STATUS       USBD_DeviceConnect      (void);
static ARM_USBD_STATUS       USBD_DeviceDisconnect   (void);
static ARM_USBD_STATUS       USBD_DeviceDisconnect_core   (void);
static ARM_USBD_STATE        USBD_DeviceGetState     (void);
static ARM_USBD_STATUS       USBD_DeviceRemoteWakeup (void);
static ARM_USBD_STATUS       USBD_DeviceSetAddress   (uint8_t dev_addr, ARM_USBD_SET_ADDRESS_STAGE stage);
static ARM_USBD_STATUS       USBD_DeviceConfigure    (bool configure);
static ARM_USBD_STATUS       USBD_EndpointConfigure  (uint8_t ep_addr,
                                                      ARM_USB_ENDPOINT_TYPE ep_type,
                                                      uint16_t ep_max_packet_size);
static ARM_USBD_STATUS       USBD_EndpointConfigure_core  (uint8_t ep_addr,
                                           ARM_USB_ENDPOINT_TYPE ep_type,
                                           uint16_t ep_max_packet_size);
static ARM_USBD_STATUS       USBD_EndpointUnconfigure(uint8_t ep_addr);
static ARM_USBD_STATUS       USBD_EndpointUnconfigure_core(uint8_t ep_addr);

static ARM_USBD_STATUS       USBD_EndpointStall      (uint8_t ep_addr, bool stall);
static ARM_USBD_STATUS       USBD_EndpointReadStart  (uint8_t ep_addr,       uint8_t *buf, uint32_t len);
static int32_t               USBD_EndpointRead       (uint8_t ep_addr,       uint8_t *buf, uint32_t len);
static int32_t               USBD_EndpointWrite      (uint8_t ep_addr, const uint8_t *buf, uint32_t len);
static ARM_USBD_STATUS       USBD_EndpointAbort      (uint8_t ep_addr);
static uint16_t              USBD_GetFrameNumber     (void);
// } Driver_USBD.h/typedef struct _ARM_DRIVER_USBD

static ARM_USBD_STATUS       USBD_EndpointWriteStart (uint8_t ep_addr, const uint8_t *buf, uint32_t len);
static uint32_t              USBD_EndpointTransferGetResult (uint8_t ep_addr);
static ARM_USBD_STATUS       USBD_ReadSetupPacket (uint8_t *setup);
static ARM_USBD_STATUS       USBD_EndpointSetDPID     (uint8_t ep_addr, uint8_t toggle);
static ARM_USBD_STATUS       USBD_DeviceSetTestMode   (uint8_t mode);
static USBD_TZ10xx_ERROR     USBD_GetError   (uint32_t *param);

static void                  USBD_SignalDeviceEvent   (ARM_USBD_EVENT  event);
static void                  USBD_SignalEndpointEvent (uint8_t ep_addr, ARM_USBD_EP_EVENT ep_event);
static void                  USBD_SignalLinkSpeed     (uint8_t link_speed);

static struct {
	USBD_TZ10xx_ERROR status;
	uint32_t param;
} stError;

/* USB Event flag */
static volatile usb_event_t UsbEventFlag;

typedef struct USB_ctrlXfer {
	USB_devReq_t    stReq; 
	USB_xfer_t      stDataXfer; 
	uint8_t         stp_rollover;
	struct {
		uint32_t        ulDataLen;  /* データステージの転送長 */
		uint32_t        ulTotalLen; /* 実際に転送した長さ */
	} DataPhase;
} USB_ctrlXfer_t; 

/* USB device data structure which store all device
	and endpoint information. */
typedef struct USB_device {
	
	/* DWC_otg core specific */
		/* Global Register of the controller*/
		dwc_otg_core_global_regs_t     *pstGRegs;
		/* Device registers of the controller*/
		dwc_otg_device_global_regs_t   *pstDRegs;
		/* Power and Clock gate registers of the controller*/
		dwc_otg_pcg_global_regs_t      *pstPRegs;
		/* PHY initialization of one-time-only */
		bool_t isPhyInitDone;
		/** Start predict NextEP based on Learning Queue if equal 1,
		 * also used as counter of disabled NP IN EP's */
		uint8_t start_predict;
		/** NextEp sequence, including EP0: nextep_seq[] = EP if non-periodic and 
		 * active, 0xff otherwise */
		uint8_t nextep_seq[USB_NUM_EPS];
		/** Index of fisrt EP in nextep_seq array which should be re-enabled **/
		uint8_t first_in_nextep_seq;
		/** ginnakeff **/
		uint8_t ginnakeff_disabled_count;
		uint8_t ginnakeff_disabled_ep[USB_NUM_EPS];
		union {
			struct {
				uint8_t timeout : 1;
				uint8_t epmis : 1;
				uint8_t stall_abort : 1;
				uint8_t ep0_premature_status : 1;
			} bit ;
			uint8_t byte;
		} ginnak_factor;
	
	/* USB Device */
		/*Connection speed of USB*/
		uint8_t                 iSpeed;
		/*State of the USB device*/
		USB_state_t             iDevState;
		uint8_t                 ucDevAddr;
	
	/* EP0, Default Control pipe */
		/* Information (device request) Control transfer */
		USB_ctrlXfer_t          stCtrlXfer;
		EP0_State_t             EP0State;
		bool_t                  EP0State_PhaseEntered;
		bool_t                  EP0State_StatusPhaseHandled;
		uint8_t                 otg_ver : 1; // 0(for OTG 1.3 support) or 1(for OTG 2.0 support)
	
	/* Endpoints context */
	struct {
		/* In enpoint registers of the controller*/
		dwc_otg_dev_in_ep_regs_t  *pstInEP;
		/* out enpoint registers of the controller*/
		dwc_otg_dev_out_ep_regs_t *pstOutEP;
		/* Pointer to hold transfer information for bidirectional endpoint */
		USB_xfer_t          *pstXfer[2];      
		/* Transfered length */
		uint32_t             ActLen[2];
		/* Max Packet Size */
		uint16_t            usMPS[2];
		/* Endpoint type */
		uint8_t             eptype[2];
		/* Enpoint status */
		volatile USB_ep_state_t      eState[2];
		/* IN Endpoint TxFIFO Number */
		uint8_t             txfnum;
	} stEPs[USB_NUM_EPS];
	
	USB_xfer_t      stXfer[USB_NUM_EPS][2];
} USB_device_t; 

/* ***************************************************************************** */
/*                            Static Function declaration                        */
/* ***************************************************************************** */
static void USB_otg_core_init(void);
static void USB_otg_flush_tx_fifo(const int num);
static void USB_otg_flush_rx_fifo(void);

static uint32_t USB_otg_read_core_intr(void);
static uint32_t USB_read_dev_all_out_ep_intr(void);
static uint32_t USB_read_dev_all_in_ep_intr(void);
static void USB_intrUSBReset(void);
static void USB_intrEnumDone(void);
static void USB_intrInEP(void);
static void USB_intrOutEP(void);

static USB_err_t USB_EP0DataInXfer(USB_xfer_t *pstXfer);
static USB_err_t USB_EP0DataOutXfer(USB_xfer_t *pstXfer);
static void USB_EP_Activate(uint8_t ep_num, uint8_t Dir);
static void USB_StartXfer(uint8_t ep_addr, USB_xfer_t *pstXfer);
static USB_err_t USB_EPAbort(uint8_t ucEP, uint8_t Dir);
static USB_err_t USB_EPSetStall(uint8_t ucEP, uint8_t Dir);
static USB_err_t USB_EPClearStall(uint8_t ucEP, uint8_t Dir);
static USB_err_t USB_ep0_out_start(void);

static USB_err_t USB_EP0SetupXferDone(USB_ctrlXfer_t *pstCXfer);
static void USB_intrOutEP0(doepint_data_t doepint);
static void USB_handle_ep(uint32_t epnum, uint8_t Dir);
static USB_err_t USB_EP0StatusXferDone(USB_ctrlXfer_t *pstCXfer);
static USB_err_t handle_ep0(void);

static USB_err_t USB_xferEP0(uint8_t kind, USB_ctrlXfer_t *pstCXfer);

static void reset_predict(void);
static void restart_transfer(uint8_t epnum);
static void predict_nextep_seq( void );
static void USB_ResumeEarly(void);
static void USB_Suspend(void);
static void USB_SetAddress(uint8_t dev_addr);

/* ***************************************************************************** */
/*                             Global variable definitions                       */
/* ***************************************************************************** */
static USB_device_t     stUSBDevice; 
static USB_device_t     *pstUSBDev = NULL;
#ifdef __ICCARM__
#pragma data_alignment=32
#endif
static int8_t      EP0outSetupBuf[8 * 5] TZ1K_ALIGNED((32));

#if 0
static void USB_dump_register()
{
	volatile uint8_t  *addr;
	int32_t offset;
	
	addr = (int8_t *)BASE_USB;
	
	usbprint("USB Global Register");
	for(offset=0; offset<0x0060; offset+=4)
	{
		usbprint("addr=0x%08X value=0x%08X",   ((int32_t)addr)+offset,   DWC_READ_REG32(addr+offset) );
	}
	DWC_DELAY_US(200 * 1000);
	for(offset=0x100; offset<0x0120; offset+=4)
	{
		usbprint("addr=0x%08X value=0x%08X",   ((int32_t)addr)+offset,   DWC_READ_REG32(addr+offset) );
	}
	DWC_DELAY_US(200 * 1000);
	
	usbprint("USB Device Register");
	for(offset=0x800; offset<0x0838; offset+=4)
	{
		usbprint("addr=0x%08X value=0x%08X",   ((int32_t)addr)+offset,   DWC_READ_REG32(addr+offset) );
	}
	DWC_DELAY_US(200 * 1000);
	
	usbprint("USB EPIN Register");
	for(offset=0x900; offset<0x0940; offset+=4)
	{
		usbprint("addr=0x%08X value=0x%08X",   ((int32_t)addr)+offset,   DWC_READ_REG32(addr+offset) );
	}
	DWC_DELAY_US(200 * 1000);
	
	usbprint("USB EPOUT Register");
	for(offset=0xB00; offset<0xB40; offset+=4)
	{
		usbprint("addr=0x%08X value=0x%08X",   ((int32_t)addr)+offset,   DWC_READ_REG32(addr+offset) );
	}
	DWC_DELAY_US(200 * 1000);
	
}

static int USB_GintMsk(void)
{
	gahbcfg_data_t ahbcfg;
	ahbcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gahbcfg);
	if (ahbcfg.b.glblintrmsk) {
		ahbcfg.b.glblintrmsk = 0; // 0: Mask the interrupt assertion to the application.
		DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gahbcfg, ahbcfg.d32);
		return 1;
	} else {
		return 0;
	}
}

static int USB_GintUnmsk(void)
{
	gahbcfg_data_t ahbcfg;
	ahbcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gahbcfg);
	if (!ahbcfg.b.glblintrmsk) {
		ahbcfg.b.glblintrmsk = 1; // 1: Unmask the interrupt assertion to the application.
		DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gahbcfg, ahbcfg.d32);
		return 0;
	} else {
		return 1;
	}
}
#endif

/* ***************************************************************************** */
/*                           USB OTG Core initialization function                */
/* ***************************************************************************** */

/**
*	@brief Soft reset USB OTG Core 
*	@note  dwc_otg_cil.c / dwc_otg_core_reset
**/
static void USB_otg_core_soft_reset(void)
{
	volatile grstctl_t greset = {.d32 = 0 };
	int count = 0;
	
	/* Wait for AHB master IDLE state. */
	do {
		greset.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->grstctl);
		if (++count > 100000) {
			log_error("%s TIMEOUT! AHB Idle GRSTCTL=%0x", __func__, greset.d32);
			stError.status = USBD_TZ10xx_ERROR_CORE_HANG;
			stError.param  = __LINE__;
			USBD_SignalDeviceEvent((ARM_USBD_EVENT)-1);
			return;
		}
		DWC_DELAY_US(1 * 1000);
	}
	while (greset.b.ahbidle == 0);
	
	/* Core Soft Reset */
	count = 0;
	greset.b.csftrst = 1;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->grstctl, greset.d32);
	do {
		greset.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->grstctl);
		if (++count > 10000) {
			log_error("%s TIMEOUT! Core Soft Reset GRSTCTL=%0x", __func__, greset.d32);
			stError.status = USBD_TZ10xx_ERROR_CORE_HANG;
			stError.param  = __LINE__;
			USBD_SignalDeviceEvent((ARM_USBD_EVENT)-1);
			return;
		}
		DWC_DELAY_US(1 * 1000);
	}
	while (greset.b.csftrst == 1);
	
	/* Wait for 3 PHY Clocks */
	DWC_DELAY_US(1 * 1000);
}

/**
*	@brief Initializes USB OTG Core registers. 
*	@note  dwc_otg_cil.c / dwc_otg_core_init
**/
static void USB_otg_core_init(void)
{
	gusbcfg_data_t usbcfg;
	
	/* Reset the Controller */
	USB_otg_core_soft_reset();
	
	/* 1.4 Core Initialization */
	
	log_info("guid    0x%08X", DWC_READ_REG32(&pstUSBDev->pstGRegs->guid));
	log_info("gsnpsid 0x%08X", DWC_READ_REG32(&pstUSBDev->pstGRegs->gsnpsid));
	
	log_info("ghwcfg1 0x%08X", DWC_READ_REG32(&pstUSBDev->pstGRegs->ghwcfg1));
	log_info("ghwcfg2 0x%08X", DWC_READ_REG32(&pstUSBDev->pstGRegs->ghwcfg2));
	log_info("ghwcfg3 0x%08X", DWC_READ_REG32(&pstUSBDev->pstGRegs->ghwcfg3));
	log_info("ghwcfg4 0x%08X", DWC_READ_REG32(&pstUSBDev->pstGRegs->ghwcfg4));
	
	/* GOTGCTL */
	pstUSBDev->otg_ver = 0;
	log_info("pstUSBDev->otg_ver %d", pstUSBDev->otg_ver);
	
	/* GUSBCFG */
	if (!pstUSBDev->isPhyInitDone) {
		pstUSBDev->isPhyInitDone = true;
		
		// Table 2-3 System Clock Speeds
		// utmifs_clk48: 48MHz
		// ■ GUSBCFG.PHYSel = 1’b1
		// ■ GUSBCFG.ULPI_UTMI_Sel = 1’bx
		// ■ GUSBCFG.PHYIf = 1’bx
		// ■ OTG_FSPHY_INTERFACE = 1
		// USB 1.1 Full Speed Serial Transceiver interface: Dedicated FS interface
		// FS: 48 MHz (utmifs_clk48)
		// LS:  6 MHz (utmifs_clk48/8)
		// (FS or LS is controlled by internal wpc_xcvrselect signal.)	
		usbcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gusbcfg);
		usbcfg.b.physel = 1;
		usbcfg.b.ulpi_utmi_sel = 1;
		usbcfg.b.phyif = 1;
		DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gusbcfg, usbcfg.d32);
		USB_otg_core_soft_reset();
		
		dcfg_data_t dcfg;
		dcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dcfg);
		dcfg.b.devspd  = 3;  // 2'b11: Full speed (USB 1.1 transceiver clock is 48 MHz)
		DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dcfg, dcfg.d32);
		
		#if (OTG_MODE == 0) || (OTG_MODE == 1) || (OTG_MODE == 2)
		// Device modeでない場合はDevice modeにする
		gintsts_data_t gintsts;
		gintsts.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gintsts);
		if (DWC_HOST_MODE == gintsts.b.curmode) {
			usbcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gusbcfg);
			usbcfg.b.force_dev_mode = 1;
			DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gusbcfg, usbcfg.d32);
			DWC_DELAY_US(25 * 1000); // After setting the force bit, the application must wait at least 25 ms before the change to take effect.
		}
		#endif
	}
	
	/* Global AHB Configuration */
	gahbcfg_data_t ahbcfg;
	ahbcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gahbcfg);
	ahbcfg.b.dmaenable = 1;
//	ahbcfg.b.ahbsingle = 1;
	ahbcfg.b.hburstlen = 0x7; // 0, 1, 3, 5, 7
	ahbcfg.b.glblintrmsk = 1; // Unmask
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gahbcfg, ahbcfg.d32);
	
	usbcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gusbcfg);
	usbcfg.b.hnpcap = 0;
	usbcfg.b.srpcap = 0;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gusbcfg, usbcfg.d32);
	
	/* clear interrupt status */
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gotgint, 0xFFFFFFFF);
	gintsts_data_t gintr_status;
	gintr_status.d32 = 0;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, ~gintr_status.d32);
	
	/* Global Interrupt Mask */
	// 有効にした割り込み要因は USB_IRQHandler() でハンドルすること
	gintmsk_data_t intr_mask;
	intr_mask.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gintmsk);
	intr_mask.b.modemismatch = 1;
	intr_mask.b.otgintr = 1;
	intr_mask.b.rxstsqlvl = 0;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintmsk, intr_mask.d32);
	
	log_info("%s", __func__);
	log_info("gahbcfg   addr=0x%08X value=0x%08X written=0x%08X",   &pstUSBDev->pstGRegs->gahbcfg  , DWC_READ_REG32(&pstUSBDev->pstGRegs->gahbcfg), ahbcfg.d32 );
	log_info("gintmsk   addr=0x%08X value=0x%08X written=0x%08X",   &pstUSBDev->pstGRegs->gintmsk  , DWC_READ_REG32(&pstUSBDev->pstGRegs->gintmsk), intr_mask.d32 );
	log_info("gusbcfg   addr=0x%08X value=0x%08X written=0x%08X",   &pstUSBDev->pstGRegs->gusbcfg  , DWC_READ_REG32(&pstUSBDev->pstGRegs->gusbcfg), usbcfg.d32 );
}

/**
*	@brief Initializes USB OTG device registers.
*	@note  dwc_otg_cil.c / dwc_otg_core_dev_init
**/ 
static void USB_otg_device_init()
{
	/* 7.1 Device Initialization */
	
	/* Device Configuration Register */
	dcfg_data_t dcfg;
	dcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dcfg);
	dcfg.b.devspd  = 3;  // 2'b11: Full speed (USB 1.1 transceiver clock is 48 MHz)
	dcfg.b.descdma = 0;  // GAHBCFG.DMAEn=1,DCFG.DescDMA=0 => Buffer DMA mode
	DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dcfg, dcfg.d32);
	
	/* Flush the Learning Queue. */
	grstctl_t resetctl = {.d32 = 0 };
	resetctl.b.intknqflsh = 1;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->grstctl, resetctl.d32);
	
	reset_predict();
	
	pstUSBDev->ginnakeff_disabled_count = 0;
	memset(pstUSBDev->ginnakeff_disabled_ep, 0xff, sizeof(pstUSBDev->ginnakeff_disabled_ep));
	
	pstUSBDev->ginnak_factor.bit.timeout = 0;
	pstUSBDev->ginnak_factor.bit.epmis = 0;
	pstUSBDev->ginnak_factor.bit.stall_abort = 0;
	pstUSBDev->ginnak_factor.bit.ep0_premature_status = 0;
	
	/* Device Threshold Control Register */
	// Threshold feature unused.
//	dthrctl_data_t dthrctl;
//	dthrctl.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dtknqr3_dthrctl);
//	// 
//	DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dtknqr3_dthrctl, dthrctl.d32);
	
	/* Exit test mode */
	dctl_data_t dctl;
	dctl.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dctl);
	dctl.b.tstctl = 0; // Test mode disabled
	DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32);
	
	/* GINTMSK */
	// 有効にした割り込み要因は USB_IRQHandler() でハンドルすること
	gintmsk_data_t intr_mask;
	intr_mask.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gintmsk);
	if (DriverCapabilities.event_reset) {
		intr_mask.b.usbreset = 1;
		intr_mask.b.enumdone = 1;
	}
	if (DriverCapabilities.event_suspend) {
		intr_mask.b.erlysuspend = 1;
		intr_mask.b.usbsuspend = 1; 
	}
	if (DriverCapabilities.event_resume) {
		intr_mask.b.wkupintr = 1;   
	}
	if (DriverCapabilities.event_power_on) {
		intr_mask.b.sessreqintr = 1;
	}
	intr_mask.b.outepintr = 1;
	intr_mask.b.inepintr = 1; 
	intr_mask.b.epmismatch = 1; // shared fifo
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintmsk, intr_mask.d32);
	
	log_info("%s", __func__);
	log_info("dcfg    addr=0x%08X value=0x%08X written=0x%08X", &pstUSBDev->pstDRegs->dcfg     , DWC_READ_REG32(&pstUSBDev->pstDRegs->dcfg), dcfg.d32  );
	log_info("dctl    addr=0x%08X value=0x%08X written=0x%08X", &pstUSBDev->pstDRegs->dctl     , DWC_READ_REG32(&pstUSBDev->pstDRegs->dctl), dctl.d32  );
	log_info("gintmsk addr=0x%08X value=0x%08X"               , &pstUSBDev->pstGRegs->gintmsk  , DWC_READ_REG32(&pstUSBDev->pstGRegs->gintmsk)  );
	log_info("gintsts addr=0x%08X value=0x%08X"               , &pstUSBDev->pstGRegs->gintsts  , DWC_READ_REG32(&pstUSBDev->pstGRegs->gintsts)  );
}

/**
*	@brief Initializes USB OTG fifo registers.
*	@warning If you want to use two or more the InterruptIN EP, please set properly TxFIFO Number and FIFO configuration.
*	@see USBD_EndpointConfigure_core()
**/ 
static void USB_otg_fifo_init(void)
{
	/* 4. Set up the Data FIFO RAM for each of the FIFOs (only if Dynamic FIFO Sizing is enabled) */
	// 2.1.3.1Shared FIFO Operation
	
	fifosize_data_t fifo_before;
	fifosize_data_t fifo_cur;
	
	// GRXFSIZ
	fifo_cur.d32 = 0;
	fifo_cur.b.startaddr = 0;
	fifo_cur.b.depth = 78;  // 78words (312bytes)
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->grxfsiz, fifo_cur.b.depth);  // grxfsiz[15:0] = depth
	fifo_before.d32 = fifo_cur.d32;
	// GNPTXFSIZ
	fifo_cur.b.startaddr = fifo_before.b.startaddr + fifo_before.b.depth;
	fifo_cur.b.depth     = OTG_TX_NPERIO_DFIFO_DEPTH;  // 32words (128bytes)  Non-Periodic TX FIFO, Num 0
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gnptxfsiz, fifo_cur.d32);  // gnptxfsiz[15:0] = startaddr, gnptxfsiz[31:16] = depth
	fifo_before.d32 = fifo_cur.d32;
	// DPTXF_1 (on Shared FIFO)
	fifo_cur.b.startaddr = fifo_before.b.startaddr + fifo_before.b.depth;
	fifo_cur.b.depth     = OTG_TX_DPERIO_DFIFO_DEPTH_1; // 32words (128bytes)  Periodic TX FIFO, Num 1
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->dtxfsiz[0], fifo_cur.d32);  // dtxfsiz[15:0] = startaddr, dtxfsiz[31:16] = depth
	fifo_before.d32 = fifo_cur.d32;
	// GDFIFOCFG
	gdfifocfg_data_t gdfifocfg;
	gdfifocfg.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gdfifocfg);
	gdfifocfg.b.epinfobase = fifo_before.b.startaddr + fifo_before.b.depth;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gdfifocfg, gdfifocfg.d32);
	
	/* The TxFIFOs and the RxFIFO must be flushed after the RAM allocation is done, for proper FIFO functioning. */
	USB_otg_flush_tx_fifo(0x10);
	USB_otg_flush_rx_fifo();
	
	#ifdef DEBUG
	log_info("%s", __func__);
	log_info("grxfsiz    addr=0x%08X value=0x%08X",   &pstUSBDev->pstGRegs->grxfsiz  ,  DWC_READ_REG32(&pstUSBDev->pstGRegs->grxfsiz  ));
	log_info("gnptxfsiz  addr=0x%08X value=0x%08X",   &pstUSBDev->pstGRegs->gnptxfsiz,  DWC_READ_REG32(&pstUSBDev->pstGRegs->gnptxfsiz));
	for (int i=0; i<1; i++) {
		log_info("dtxfsiz[%d] addr=0x%08X value=0x%08X",   i, &pstUSBDev->pstGRegs->dtxfsiz[i], DWC_READ_REG32(&pstUSBDev->pstGRegs->dtxfsiz[i]));
	}
	log_info("gdfifocfg  addr=0x%08X value=0x%08X",   &pstUSBDev->pstGRegs->gdfifocfg,  DWC_READ_REG32(&pstUSBDev->pstGRegs->gdfifocfg));
	#endif
}

/**
*	@brief Flush a Tx FIFO.
*	@note  dwc_otg_cil.c / dwc_otg_flush_tx_fifo
**/ 
static void USB_otg_flush_tx_fifo(const int num)
{
	volatile grstctl_t greset = {.d32 = 0 };
	int count = 0;

	greset.b.txfflsh = 1;
	greset.b.txfnum = num;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->grstctl, greset.d32);

	do {
		greset.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->grstctl);
		if (++count > 10000) {
			log_warning("%s() HANG! GRSTCTL=%0x GNPTXSTS=0x%08x\n",
				 __func__, greset.d32,
				 DWC_READ_REG32(&pstUSBDev->pstGRegs->gnptxsts));
			stError.status = USBD_TZ10xx_ERROR_CORE_HANG;
			stError.param  = __LINE__;
			USBD_SignalDeviceEvent((ARM_USBD_EVENT)-1);
			break;
		}
		DWC_DELAY_US(1);
	} while (greset.b.txfflsh == 1);

	/* Wait for 3 PHY Clocks */
	DWC_DELAY_US(1);
}

/**
*	@brief Flush Rx FIFO.
*	@note  dwc_otg_cil.c / dwc_otg_flush_rx_fifo
**/ 
static void USB_otg_flush_rx_fifo(void)
{
	volatile grstctl_t greset = {.d32 = 0 };
	int count = 0;

	greset.b.rxfflsh = 1;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->grstctl, greset.d32);

	do {
		greset.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->grstctl);
		if (++count > 10000) {
			log_warning("%s() HANG! GRSTCTL=%0x\n", __func__,
				 greset.d32);
			stError.status = USBD_TZ10xx_ERROR_CORE_HANG;
			stError.param  = __LINE__;
			USBD_SignalDeviceEvent((ARM_USBD_EVENT)-1);
			break;
		}
		DWC_DELAY_US(1);
	} while (greset.b.rxfflsh == 1);

	/* Wait for 3 PHY Clocks */
	DWC_DELAY_US(1);
}

/**
*	@brief  Initializes USB device driver.
**/ 
static void USB_Init(void)
{
	stError.status = USBD_TZ10xx_ERROR_NONE;
	stError.param = 0;
	
	memset(&stUSBDevice, 0, sizeof(stUSBDevice)); 
	pstUSBDev = &stUSBDevice; 
	
	/* Register base offset value */
	pstUSBDev->pstGRegs = (dwc_otg_core_global_regs_t *) usb2fs_BASE;
	pstUSBDev->pstDRegs = (dwc_otg_device_global_regs_t*) (usb2fs_BASE + DWC_DEV_GLOBAL_REG_OFFSET);
	pstUSBDev->pstPRegs = (dwc_otg_pcg_global_regs_t*) (usb2fs_BASE + DWC_PCG_GLOBAL_REG_OFFSET);
 	for(int i=0; i<USB_NUM_EPS; i++) {
		pstUSBDev->stEPs[i].pstInEP  =  (dwc_otg_dev_in_ep_regs_t *)(usb2fs_BASE + DWC_DEV_IN_EP_REG_OFFSET  + (i * DWC_EP_REG_OFFSET));
		pstUSBDev->stEPs[i].pstOutEP = (dwc_otg_dev_out_ep_regs_t *)(usb2fs_BASE + DWC_DEV_OUT_EP_REG_OFFSET + (i * DWC_EP_REG_OFFSET));
 	}
	
	/*State of the USB device*/
	pstUSBDev->iDevState = USB_STATE_DETACH;
	pstUSBDev->isPhyInitDone = false;
}

/* ***************************************************************************** */
/*                      USB Interrupt handling functions                         */
/* ***************************************************************************** */

/**
*	@brief Read OTG core interrupt register
*	@note  dwc_otg_cil.h / dwc_otg_read_core_intr
**/ 
static TZ1K_INLINE uint32_t USB_otg_read_core_intr()
{
	return (DWC_READ_REG32(&pstUSBDev->pstGRegs->gintsts) & DWC_READ_REG32(&pstUSBDev->pstGRegs->gintmsk));
}

/**
*	@brief Reads device all out endpoint interrupt status.
*	@note  dwc_otg_cil.h / dwc_otg_read_dev_all_out_ep_intr
**/ 
static TZ1K_INLINE uint32_t USB_read_dev_all_out_ep_intr()
{
	uint32_t v;
	v = (DWC_READ_REG32(&pstUSBDev->pstDRegs->daint) & DWC_READ_REG32(&pstUSBDev->pstDRegs->daintmsk));
	return ((v & 0xffff0000) >> 16);
}

/**
*	@brief Reads device all in endpoint interrupt status.
*	@note  dwc_otg_cil.h / dwc_otg_read_dev_all_in_ep_intr
**/ 
static TZ1K_INLINE uint32_t USB_read_dev_all_in_ep_intr()
{
	uint32_t v;
	v = (DWC_READ_REG32(&pstUSBDev->pstDRegs->daint) & DWC_READ_REG32(&pstUSBDev->pstDRegs->daintmsk));
	return (v & 0xffff);
}

/**
*	@brief Reads device in endpoint interrupt status.
*	@note  dwc_otg_cil.h / dwc_otg_read_dev_in_ep_intr
**/ 
static TZ1K_INLINE uint32_t USB_read_dev_in_ep_intr(int8_t epnum)
{
	uint32_t v;
	v = DWC_READ_REG32(&pstUSBDev->stEPs[epnum].pstInEP->diepint) & DWC_READ_REG32(&pstUSBDev->pstDRegs->diepmsk);
	return v;
}

/**
*	@brief Reads device out endpoint interrupt status.
*	@note  dwc_otg_cil.h / dwc_otg_read_dev_out_ep_intr
**/ 
static TZ1K_INLINE uint32_t USB_read_dev_out_ep_intr(int8_t epnum)
{
	uint32_t v;
	doepint_data_t msk;
	msk.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->doepmsk);
	msk.b.sr = 1; // このビットはステータスには存在しますが、マスクには存在しません。
	v = DWC_READ_REG32(&pstUSBDev->stEPs[epnum].pstOutEP->doepint) & msk.d32;
	return v;
}

/**
*	@brief Update EP disable status
**/ 
static TZ1K_INLINE USB_ep_state_t USB_EPStopped(uint8_t ucEP, uint8_t Dir)
{
	if (USB_EP_ABORTING == pstUSBDev->stEPs[ucEP].eState[Dir]) {
		pstUSBDev->stEPs[ucEP].eState[Dir] = USB_EP_ABORTED;
		
	} else if (USB_EP_STALLING == pstUSBDev->stEPs[ucEP].eState[Dir]) {
		pstUSBDev->stEPs[ucEP].eState[Dir] = USB_EP_HALTED;
	} else {
//		log_debug("%s invalid eState %d", pstUSBDev->stEPs[ucEP].eState[Dir]);
	}
	return pstUSBDev->stEPs[ucEP].eState[Dir];
}


/**
*	@brief This function handle IN EP Nak Effective
**/ 
static void USB_intrINEPNakEff(uint8_t ucEP)
{
	// IN EPのstallやdisableを行う。
	const uint8_t Dir = 1; // IN
	depctl_data_t depctl;
	bool_t isSetEpdis;
	
	isSetEpdis = false;
	depctl.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->diepctl);
	if (USB_EP_ABORTING == pstUSBDev->stEPs[ucEP].eState[Dir]  ||
		USB_EP_STALLING == pstUSBDev->stEPs[ucEP].eState[Dir]) {
		// DisableかStallの目的を持つepだったので、割り込みをマスクする
		const diepmsk_data_t intr_msk = {.d32 = 0, .b.inepnakeff = 1 };
		DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->diepmsk, intr_msk.d32, 0);
//		log_debug("%s EP%d Dir%d depctl 0x%08X", __func__, ucEP, Dir, depctl.d32);
	} else {
		log_warning("%s EP%d Dir%d depctl 0x%08X invalid eState %d", __func__, ucEP, Dir, depctl.d32, pstUSBDev->stEPs[ucEP].eState[Dir]);
	}
	
	if (USB_EP_ABORTING == pstUSBDev->stEPs[ucEP].eState[Dir]) {
		if (depctl.b.epena) {
			depctl.b.epdis = 1;
			depctl.b.snak = 1;
			isSetEpdis = true;
			DWC_WRITE_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->diepctl, depctl.d32); 
		} else {
			USB_EPStopped(ucEP,Dir);
		}
		
	} else if (USB_EP_STALLING == pstUSBDev->stEPs[ucEP].eState[Dir]) {
		if (depctl.b.epena) {
			depctl.b.epdis = 1;
			isSetEpdis = true;
		} else {
			USB_EPStopped(ucEP,Dir);
		}
		depctl.b.stall = 1; // stallはepena==0の場合でもsetできます
		DWC_WRITE_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->diepctl, depctl.d32); 
	}
	
	if (!isSetEpdis) {
		// stop処理し終えた。
	} else {
		
	}
}


/**
 * @brief This function reset nextep sequnce.
 * @note  dwc_otg_cil.c / dwc_otg_core_dev_init
 */
static void reset_predict(void)
{
	pstUSBDev->start_predict = 0;
	for (uint8_t i = 0; i<USB_NUM_EPS; ++i) {
		pstUSBDev->nextep_seq[i] = 0xff;	// 0xff - EP not active
	}
	pstUSBDev->nextep_seq[0] = 0;	// EP0 active
	pstUSBDev->first_in_nextep_seq = 0;
	
	depctl_data_t diepctl;
	diepctl.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstInEP->diepctl);
	diepctl.b.nextep = 0;
	DWC_WRITE_REG32(&pstUSBDev->stEPs[0].pstInEP->diepctl, diepctl.d32);
	
	/* Update IN Endpoint Mismatch Count by active IN NP EP count + 1 */
	dcfg_data_t dcfg;
	dcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dcfg);
	dcfg.b.epmscnt = 2;
	DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dcfg, dcfg.d32);
}

/**
 * @brief Restart transfer
 * @note  dwc_otg_pcd_intr.c / restart_transfer
 */
static void restart_transfer(uint8_t epnum)
{
	const uint8_t Dir = 1;
	
	deptsiz_data_t dieptsiz;
	dieptsiz.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[epnum].pstInEP->dieptsiz);
	
	/*
	 * If xfersize is 0 and pktcnt in not 0, resend the last packet.
	 */
	USB_xfer_t* pstXfer = pstUSBDev->stEPs[epnum].pstXfer[Dir];
	log_warning("%s pvBuffer %p usActLen %d usLen %d", __func__, pstXfer->pvBuffer, pstXfer->usActLen, pstXfer->usLen);
	
	if (dieptsiz.b.pktcnt && dieptsiz.b.xfersize == 0 &&
		pstXfer) {
		// USB_StartXfer() では、送信長xfersize = pstXfer->usLen - pstXfer->usActLen; と計算される。
		// EP0転送では、1度に1MPS以下ずつしか転送しないようになっている。
		if (pstXfer->usLen <= pstUSBDev->stEPs[epnum].usMPS[Dir]) {
			#if 0
			// 0-length packetを送信したい
			pstXfer->usActLen = pstXfer->usLen;
			#else
			// usLenがMPS以下なので一度の転送で終わるものだった。改めてもう一度送信したい
			pstXfer->usActLen = 0;
			#endif
		} else {
			// 「すでに転送完了した長さからmpsを引いた値」を転送開始したい。
			pstXfer->usActLen -= pstUSBDev->stEPs[epnum].usMPS[Dir];
		}
		
		if (epnum == 0) {
			log_warning("%s usActLen %d usLen %d EP0State %d", __func__, pstXfer->usActLen, pstXfer->usLen, pstUSBDev->EP0State);
			
			USB_EP0DataInXfer(pstXfer);
			switch (pstUSBDev->EP0State) {
			case EP0_IN_DATA_PHASE:
				// 現在の状態がControlRead transfer, DataIN phase, の場合は、EP0 OUTを受信可能な状態にする。(cnak=1)
				// otg_ver == 0の場合はすでにcnak=1されている。
				if (1) {
					depctl_data_t depctl = {.d32 = 0};
					depctl.b.cnak = 1;
					DWC_MODIFY_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepctl, 0, depctl.d32);
				}
				break;
			case EP0_IN_STATUS_PHASE:
				// ControlWriteやNoDataのStatusIN phaseの場合とくになにもしない。
				break;
			default:
				// ここには来ないはず
				break;
			}
			
		} else {
			log_warning("%s usActLen %d usLen %d", __func__, pstXfer->usActLen, pstXfer->usLen);
			USB_StartXfer(USB_ep_addr(epnum,Dir), pstXfer);
		}
	}
}

/**
 * @brief This function create new nextep sequnce based on Learn Queue.
 * @note  dwc_otg_pcd_intr.c / predict_nextep_seq
 */
static void predict_nextep_seq( void )
{
	dwc_otg_device_global_regs_t *dev_global_regs = pstUSBDev->pstDRegs;
	const uint32_t TOKEN_Q_DEPTH = OTG_TOKEN_QUEUE_DEPTH;
	/* Number of Token Queue Registers */
//	const int DTKNQ_REG_CNT = (TOKEN_Q_DEPTH + 7) / 8;
	int DTKNQ_REG_CNT;
	dtknq1_data_t dtknqr1;
	uint32_t in_tkn_epnums[4];
	uint8_t seqnum[USB_NUM_EPS];
	uint8_t intkn_seq[OTG_TOKEN_QUEUE_DEPTH];
	grstctl_t resetctl = {.d32 = 0 };
	uint8_t temp;
	int ndx = 0;
	int start = 0;
	int end = 0;
	int sort_done = 0;
	uint32_t i = 0;
	volatile uint32_t *addr = &dev_global_regs->dtknqr1;

	log_warning("dev_token_q_depth=%d",TOKEN_Q_DEPTH);

	/* Count the number of DTKNQ Registers */
	// dtknqr1 ... Token  0 to  5
	// dtqnqr2 ... Token  6 to 13
	// dtqnqr3 ... Token 14 to 21
	// dtqnqr4 ... Token 22 to 29
	if (TOKEN_Q_DEPTH <= 6) {
		DTKNQ_REG_CNT = 1;
	} else if (TOKEN_Q_DEPTH <= 14) {
		DTKNQ_REG_CNT = 2;
	} else if (TOKEN_Q_DEPTH <= 22) {
		DTKNQ_REG_CNT = 3;
	} else {
		DTKNQ_REG_CNT = 4;
	}

	/* Read the DTKNQ Registers */
	for (i = 0; i < DTKNQ_REG_CNT; i++) {
		in_tkn_epnums[i] = DWC_READ_REG32(addr);
		log_warning("DTKNQR%d=0x%08x", i + 1, in_tkn_epnums[i]);
		if (addr == &dev_global_regs->dvbusdis) {
			addr = &dev_global_regs->dtknqr3_dthrctl;
		} else {
			++addr;
		}
	}

	/* Copy the DTKNQR1 data to the bit field. */
	dtknqr1.d32 = in_tkn_epnums[0];
	if (dtknqr1.b.wrap_bit) {
		ndx = dtknqr1.b.intknwptr;
		end = ndx - 1;
		if (end < 0) 
			end = TOKEN_Q_DEPTH - 1;
	} else {
		ndx = 0;
		end = dtknqr1.b.intknwptr - 1;
		if (end < 0) 
			end = 0;
	}
	start = ndx;
	
	/* Fill seqnum[] by initial values: EP number + 31 */
	for (i = 0; i < USB_NUM_EPS; i++) {
		seqnum[i] = i + 31;
	}
	
	/* Fill intkn_seq[] from in_tkn_epnums[0] */
	for (i = 0; (i < 6) && (i < TOKEN_Q_DEPTH); i++)
		intkn_seq[i] = (in_tkn_epnums[0] >> ((7 - i) * 4)) & 0xf;
	
	if (TOKEN_Q_DEPTH > 6) {
		/* Fill intkn_seq[] from in_tkn_epnums[1] */
		for (i = 6; (i < 14) && (i < TOKEN_Q_DEPTH); i++)
			intkn_seq[i] = (in_tkn_epnums[1] >> ((7 - (i - 6)) * 4)) & 0xf;
	}
	
	if (TOKEN_Q_DEPTH > 14) {
		/* Fill intkn_seq[] from in_tkn_epnums[1] */
		for (i = 14; (i < 22) && (i < TOKEN_Q_DEPTH); i++)
			intkn_seq[i] = (in_tkn_epnums[2] >> ((7 - (i - 14)) * 4)) & 0xf;
	}

	if (TOKEN_Q_DEPTH > 22) {
		/* Fill intkn_seq[] from in_tkn_epnums[1] */
		for (i = 22; (i < 30) && (i < TOKEN_Q_DEPTH); i++)
			intkn_seq[i] = (in_tkn_epnums[3] >> ((7 - (i - 22)) * 4)) & 0xf;
	}

	log_warning("%s start=%d end=%d intkn_seq[]:", __func__, start, end);
	for (i=0; i<TOKEN_Q_DEPTH; i++) 
		log_warning("%d", intkn_seq[i]);
	
	/* Update seqnum based on intkn_seq[] */
	i = 0;
	do {
		seqnum[intkn_seq[ndx]] = i;
		ndx++;
		i++;
		if (ndx == TOKEN_Q_DEPTH) 
			ndx = 0;
	} while ( i < TOKEN_Q_DEPTH );
	
	/* Mark non active EP's in seqnum[] by 0xff */
	for (i=0; i<USB_NUM_EPS; i++) {
		if (pstUSBDev->nextep_seq[i] == 0xff )
			seqnum[i] = 0xff;
	}
	
	/* Sort seqnum[] */
	sort_done = 0;
	while (!sort_done) {
		sort_done = 1;
		for (i=0; i<(USB_NUM_EPS-1); i++) {
			if (seqnum[i] > seqnum[i+1]) {
				temp = seqnum[i];
				seqnum[i] = seqnum[i+1];
				seqnum[i+1] = temp;
				sort_done = 0;
			}
		}
	}

	ndx = start + seqnum[0];
	if (ndx >= TOKEN_Q_DEPTH) 
		ndx = ndx % TOKEN_Q_DEPTH;
	pstUSBDev->first_in_nextep_seq = intkn_seq[ndx];
	
	/* Update seqnum[] by EP numbers  */
	for (i=0; i<USB_NUM_EPS; i++) {
		ndx = start + i;
		if (seqnum[i] < 31) {
			ndx = start + seqnum[i];
			if (ndx >= TOKEN_Q_DEPTH) 
				ndx = ndx % TOKEN_Q_DEPTH;
			seqnum[i] = intkn_seq[ndx];
		} else {
			if (seqnum[i] < 0xff) {
				seqnum[i] = seqnum[i] - 31;
			} else {
				break;
			}
		}
	}

	/* Update nextep_seq[] based on seqnum[] */
	for (i=0; i<(USB_NUM_EPS-1); i++) {
		if (seqnum[i] != 0xff) {
			if (seqnum[i+1] != 0xff) {
				pstUSBDev->nextep_seq[seqnum[i]] = seqnum[i+1];
			} else {
				pstUSBDev->nextep_seq[seqnum[i]] = pstUSBDev->first_in_nextep_seq;
				break;
			}
		} else {
			break;
		}
	}
	
	log_warning("%s first_in_nextep_seq= %2d; nextep_seq[]:", __func__, pstUSBDev->first_in_nextep_seq);
	for (i=0; i < USB_NUM_EPS; i++) {
		log_warning("%2d", pstUSBDev->nextep_seq[i]);
	}

	/* Flush the Learning Queue */
	resetctl.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->grstctl);
	resetctl.b.intknqflsh = 1;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->grstctl, resetctl.d32);
	
}


/**
 * @brief This function handle IN EP Disabled
 * @note  dwc_otg_pcd_intr.c / handle_in_ep_disable_intr
 */
static void USB_intrINEPDisable(uint8_t ucEP)
{
	const uint8_t Dir = 1;
	
	depctl_data_t depctl;
	deptsiz_data_t dieptsiz;
	dctl_data_t dctl;
	uint8_t i;

	dieptsiz.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->dieptsiz);
	depctl.d32   = DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->diepctl);
	log_warning("pktcnt=%d size=%d diepctl%d=%0x factor 0x%02X", dieptsiz.b.pktcnt, dieptsiz.b.xfersize, ucEP, depctl.d32, pstUSBDev->ginnak_factor.byte);
	
	USB_EPStopped(ucEP,Dir);
	if (pstUSBDev->stEPs[ucEP].eState[Dir] == USB_EP_TIMEOUT) {
		pstUSBDev->stEPs[ucEP].eState[Dir] = USB_EP_RUNNING;
	}
	
	if ((pstUSBDev->stEPs[ucEP].eState[Dir] & USB_EP_STOPPED) && 
		USB_EP_IS_PERIODIC(depctl.b.eptype))
	{
		USB_otg_flush_tx_fifo(pstUSBDev->stEPs[ucEP].txfnum);
		return;
	}
	
	if (pstUSBDev->start_predict == 0) {
		if (pstUSBDev->ginnakeff_disabled_count) {
			pstUSBDev->ginnakeff_disabled_count--;
		}
		log_warning("Non-Periodic IN EP is disabled now 2 remain %d", pstUSBDev->ginnakeff_disabled_count);
		if (pstUSBDev->ginnakeff_disabled_count == 0) {
			/* Global IN NAK  and  All EPs disabled. */
			/* Flush Shared NP TxFIFO */
			USB_otg_flush_tx_fifo(0);
			/* restart disabled EPs */
			for (uint8_t i=0; i<USB_NUM_EPS && pstUSBDev->ginnakeff_disabled_ep[i]!=0xff; i++) {
				if ((pstUSBDev->stEPs[i].eState[Dir] & USB_EP_STOPPED) == 0) {
					uint8_t ep_num = ((pstUSBDev->ginnakeff_disabled_ep[i]) & ARM_USB_ENDPOINT_NUMBER_MASK);
					/* Restart the transaction */
					dieptsiz.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[ep_num].pstInEP->dieptsiz);
					if (dieptsiz.b.pktcnt != 0 || dieptsiz.b.xfersize != 0) {
						restart_transfer(ep_num);
						pstUSBDev->stEPs[ep_num].eState[Dir] = USB_EP_RUNNING;
					}
				}
			}
			/* Clear the global non-periodic IN NAK handshake */
			dctl.d32 = 0;
			dctl.b.cgnpinnak = 1;
			DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32, dctl.d32);
			/* clear ginnak factors */
			pstUSBDev->ginnak_factor.bit.timeout = 0;
			pstUSBDev->ginnak_factor.bit.stall_abort = 0;
			pstUSBDev->ginnak_factor.bit.ep0_premature_status = 0;
		}
		
		return;
	}
	
	if (pstUSBDev->start_predict) {
		pstUSBDev->start_predict--;	// Non-Periodic IN EP is disabled now
		log_warning("Non-Periodic IN EP is disabled now remain %d", pstUSBDev->start_predict);
	}
	
	if (pstUSBDev->start_predict == 1) {	// All Non-Periodic IN Ep's disabled now

		predict_nextep_seq();
			
		/* Update all active IN EP's NextEP field based of nextep_seq[] */
		for (i = 0; i < USB_NUM_EPS; i++) {
			depctl.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[i].pstInEP->diepctl);
			if (pstUSBDev->nextep_seq[i] != 0xff) {	// Active NP IN EP
				depctl.b.nextep = pstUSBDev->nextep_seq[i];
				DWC_WRITE_REG32(&pstUSBDev->stEPs[i].pstInEP->diepctl, depctl.d32);
			}
		}
		/* Flush Shared NP TxFIFO */
		USB_otg_flush_tx_fifo(0);
		
		/* Rewind buffers */
		{
			uint32_t diepdma;
			uint32_t remain_to_transfer = 0;
			uint32_t xfer_size;
			i = pstUSBDev->first_in_nextep_seq;
			do {
				USB_xfer_t *xfer = pstUSBDev->stEPs[i].pstXfer[Dir];
				uint16_t maxpacket = pstUSBDev->stEPs[i].usMPS[Dir];
				
				dieptsiz.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[i].pstInEP->dieptsiz);
				xfer_size = xfer->usLen - xfer->usActLen;
//				depctl.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[i].pstInEP->diepctl);
				if (dieptsiz.b.pktcnt != 0) {
					if (xfer_size == 0) {
						remain_to_transfer = 0;
					} else {
						if ((xfer_size % maxpacket) == 0) {
							remain_to_transfer = dieptsiz.b.pktcnt * maxpacket;
						} else {
							remain_to_transfer = ((dieptsiz.b.pktcnt - 1) * maxpacket) + (xfer_size % maxpacket);
						}
					}
//					diepdma = DWC_READ_REG32(&pstUSBDev->stEPs[i].pstInEP->diepdma);
					dieptsiz.b.xfersize = remain_to_transfer;
					DWC_WRITE_REG32(&pstUSBDev->stEPs[i].pstInEP->dieptsiz, dieptsiz.d32);
					diepdma = (uint32_t)xfer->pvBuffer + (xfer_size - remain_to_transfer);
					DWC_WRITE_REG32(&pstUSBDev->stEPs[i].pstInEP->diepdma, diepdma);
					log_warning("EP %d written diepdma 0x%08X dieptsiz 0x%08X", i, diepdma, dieptsiz.d32);
					assert_ep_xfer(dieptsiz.b.pktcnt, dieptsiz.b.xfersize, diepdma);
				}
				i = pstUSBDev->nextep_seq[i];
			} while (i != pstUSBDev->first_in_nextep_seq);
		}
		
		/* Restart transfers in predicted sequences */
		i = pstUSBDev->first_in_nextep_seq;
		do {
			dieptsiz.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[i].pstInEP->dieptsiz);
//			depctl.d32   = DWC_READ_REG32(&pstUSBDev->stEPs[i].pstInEP->diepctl);
			if (dieptsiz.b.pktcnt != 0) {
				depctl.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[i].pstInEP->diepctl);
				depctl.b.epena = 1;
				depctl.b.cnak = 1;
				DWC_WRITE_REG32(&pstUSBDev->stEPs[i].pstInEP->diepctl, depctl.d32);
				log_warning("EP %d written depctl 0x%08X", i, depctl.d32);
			}
			i = pstUSBDev->nextep_seq[i];
		} while (i != pstUSBDev->first_in_nextep_seq);

		/* Clear the global non-periodic IN NAK handshake */
		dctl.d32 = 0;
		dctl.b.cgnpinnak = 1;
		DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32, dctl.d32); 
			
		/* Unmask EP Mismatch interrupt */
		gintmsk_data_t gintmsk_data;
		gintmsk_data.d32 = 0;
		gintmsk_data.b.epmismatch = 1;
		DWC_MODIFY_REG32(&pstUSBDev->pstGRegs->gintmsk, 0, gintmsk_data.d32);
		
		pstUSBDev->start_predict = 0;
		pstUSBDev->ginnak_factor.bit.epmis = 0;
	}
}

/**
 * @brief  Handler for the IN EP timeout handshake interrupt.
 *   7.6.7 Timeout for Non-Periodic IN Data Transfers: Shared FIFO
 * @note  dwc_otg_pcd_intr.c / handle_in_ep_timeout_intr
 */
static void USB_intrINEPTimeout(uint8_t ucEP)
{
	/*
	 * Non-periodic EP
	 */
	pstUSBDev->ginnak_factor.bit.timeout = 1;
	
	/* Enable the Global IN NAK Effective Interrupt */
	gintmsk_data_t intr_mask;
	intr_mask.d32 = 0;
	intr_mask.b.ginnakeff = 1;
	DWC_MODIFY_REG32(&pstUSBDev->pstGRegs->gintmsk, 0, intr_mask.d32);

	/* Set Global IN NAK */
	dctl_data_t dctl;
	dctl.d32 = 0;
	dctl.b.sgnpinnak = 1;
	DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32, dctl.d32);

	pstUSBDev->stEPs[ucEP].eState[1] = USB_EP_TIMEOUT;

	#ifdef DEBUG
	{
//		deptsiz_data_t dieptsiz = {.d32 = 0 };
//		uint32_t num = 0;
//		dieptsiz.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[num].pstInEP->dieptsiz);
//		log_debug("pktcnt=%d size=%d\n", dieptsiz.b.pktcnt, dieptsiz.b.xfersize);
	}
	#endif
}

/**
* @brief  This function handle SOF
* @note  dwc_otg_pcd_intr.c / dwc_otg_pcd_handle_sof_intr
**/ 
static void USB_intrSOF(void)
{
	gintsts_data_t gintsts;

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.sofintr = 1;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, gintsts.d32);
	
	USBD_SignalDeviceEvent((ARM_USBD_EVENT)-2);
}

/**
* @brief  This function handle Global OUT Nak Effective
* @note  dwc_otg_pcd_intr.c / dwc_otg_pcd_handle_out_nak_effective
**/ 
static void USB_intrGlobalOUTNakEff(void)
{
	// 割り込みをマスクする
	const gintmsk_data_t intr_mask = {.d32 = 0, .b.goutnakeff = 1 };
	DWC_MODIFY_REG32(&pstUSBDev->pstGRegs->gintmsk, intr_mask.d32, 0);
	
	// OUT EPsのstallやdisableを行う。
	uint8_t ucEP;
	for(ucEP=0; ucEP<USB_NUM_EPS; ucEP++) {
		const uint8_t Dir = 0; // OUT
		depctl_data_t depctl;
		bool_t isSetEpdis;
		
		isSetEpdis = false;
		depctl.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstOutEP->doepctl);
		#ifdef DEBUG
		if (USB_EP_ABORTING == pstUSBDev->stEPs[ucEP].eState[Dir]  ||
			USB_EP_STALLING == pstUSBDev->stEPs[ucEP].eState[Dir]) {
			log_debug("%s EP%d Dir%d depctl 0x%08X", __func__, ucEP, Dir, depctl.d32);
		} else {
			
		}
		#endif
		
		if (USB_EP_ABORTING == pstUSBDev->stEPs[ucEP].eState[Dir]) {
			if (depctl.b.epena) {
				depctl.b.epdis = 1;
				depctl.b.snak = 1;
				isSetEpdis = true;
				DWC_WRITE_REG32(&pstUSBDev->stEPs[ucEP].pstOutEP->doepctl, depctl.d32); 
			} else {
				USB_EPStopped(ucEP,Dir);
			}
			
		} else if (USB_EP_STALLING == pstUSBDev->stEPs[ucEP].eState[Dir]) {
			if (depctl.b.epena) {
				depctl.b.epdis = 1;
				isSetEpdis = true;
			} else {
				USB_EPStopped(ucEP,Dir);
			}
			depctl.b.stall = 1; // stallはepena==0の場合でもsetできます
			DWC_WRITE_REG32(&pstUSBDev->stEPs[ucEP].pstOutEP->doepctl, depctl.d32); 
		}
		
		if (isSetEpdis) {
			break;
		}
	}
	if (ucEP >= USB_NUM_EPS) {
		// 全EPを処理し終えたのでGlobal OUT Nakステータスをクリアする。
		//   gintr_status.b.goutnakeff は ReadOnlyで、dctl.b.cgoutnakによってクリアされます。
		const dctl_data_t dctl = {.d32 = 0, .b.cgoutnak = 1 };
		DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->dctl, 0, dctl.d32);
	} else {
		// epdis=1をwriteしたので、各EPのハンドラ側に処理を移します。
		// Global OUT Nak ステータスは全EPを処理し終えるまでそのままにします
	}
}

/**
 * @brief This function handle USB reset interrupt.
 * @note  dwc_otg_pcd_intr.c / dwc_otg_pcd_handle_usb_reset_intr
 */
static void USB_intrUSBReset()
{
	if (USB_STATE_GETSTATE(pstUSBDev->iDevState) < USB_STATE_POWERED) {
		goto end;
	}
	
	USB_ResumeEarly();
	
	/* 7.4.1 Initialization on USB Reset */
	
	/* Clear the Remote Wakeup Signalling */
	dctl_data_t dctl;
	dctl.d32 = 0;
	dctl.b.rmtwkupsig = 1;
	DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32, 0);
	
	/*	Set NAK for all OUT EPs */
	depctl_data_t doepctl;
	doepctl.d32 = 0;
	doepctl.b.snak = 1;
 	for(int i=0; i<USB_NUM_EPS; i++) {
 		// EP0以外のEPは、USB_evtUSBReset()のほうで後片づけします
		DWC_MODIFY_REG32(&pstUSBDev->stEPs[i].pstOutEP->doepctl, 0, doepctl.d32);
	}
	
	USB_otg_flush_tx_fifo(0x10);
	
	/* Flush the Learning Queue. */
	grstctl_t resetctl;
	resetctl.d32 = 0;
	resetctl.b.intknqflsh = 1;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->grstctl, resetctl.d32);
	
	reset_predict();
	
	pstUSBDev->ginnakeff_disabled_count = 0;
	memset(pstUSBDev->ginnakeff_disabled_ep, 0xff, sizeof(pstUSBDev->ginnakeff_disabled_ep));
	
	pstUSBDev->ginnak_factor.bit.timeout = 0;
	pstUSBDev->ginnak_factor.bit.epmis = 0;
	pstUSBDev->ginnak_factor.bit.stall_abort = 0;
	pstUSBDev->ginnak_factor.bit.ep0_premature_status = 0;
	
	doepmsk_data_t doepmsk;
	doepmsk.d32 = 0;
//	doepmsk.b.sr = 1; // このビットはステータスには存在しますが、マスクには存在しません。
	doepmsk.b.back2backsetup = 1;
	doepmsk.b.stsphsercvd = 1;
	doepmsk.b.setup = 1;
	doepmsk.b.xfercompl = 1;
	doepmsk.b.ahberr = 1;
	doepmsk.b.epdisabled = 1;
	DWC_WRITE_REG32(&pstUSBDev->pstDRegs->doepmsk, doepmsk.d32);

	diepmsk_data_t diepmsk;
	diepmsk.d32 = 0;
	diepmsk.b.xfercompl = 1;
	diepmsk.b.timeout = 1;
	diepmsk.b.epdisabled = 1;
	diepmsk.b.ahberr = 1;
	diepmsk.b.intknepmis = 1;
	DWC_WRITE_REG32(&pstUSBDev->pstDRegs->diepmsk, diepmsk.d32);
	
	/* Reset Device Address */
	dcfg_data_t dcfg;
	dcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dcfg);
	dcfg.b.devaddr = 0;
	DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dcfg, dcfg.d32);
	
	/* Reset device context */
	pstUSBDev->iSpeed = USB_SPEED_FULL; // EnumDoneまでの仮定
	pstUSBDev->iDevState = USB_STATE_DEFAULT; 
	pstUSBDev->ucDevAddr = 0; 
	
	USBD_EndpointUnconfigure_core(USB_ep_addr(0,0));
	USBD_EndpointUnconfigure_core(USB_ep_addr(0,1));
	
	/* raise event */
	UsbEventFlag.b.reset = 1;
	
end:
	{
		/* Clear interrupt */
		const gintsts_data_t gintsts = {.d32 = 0, .b.usbreset=1};
		DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, gintsts.d32);
	}
	
//	log_debug("USB_intrUSBReset()");
//	log_debug("doepctl0 addr=0x%08X value=0x%08X",   &pstUSBDev->stEPs[0].pstOutEP->doepctl, DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepctl));
//	log_debug("gintmsk  addr=0x%08X value=0x%08X",   &pstUSBDev->pstGRegs->gintmsk         , DWC_READ_REG32(&pstUSBDev->pstGRegs->gintmsk)         );
//	log_debug("daintmsk addr=0x%08X value=0x%08X",   &pstUSBDev->pstDRegs->daintmsk        , DWC_READ_REG32(&pstUSBDev->pstDRegs->daintmsk)        );
//	log_debug("doepmsk  addr=0x%08X value=0x%08X",   &pstUSBDev->pstDRegs->doepmsk         , DWC_READ_REG32(&pstUSBDev->pstDRegs->doepmsk )        );
//	log_debug("diepmsk  addr=0x%08X value=0x%08X",   &pstUSBDev->pstDRegs->diepmsk         , DWC_READ_REG32(&pstUSBDev->pstDRegs->diepmsk )        );
//	log_debug("dcfg     addr=0x%08X value=0x%08X",   &pstUSBDev->pstDRegs->dcfg            , DWC_READ_REG32(&pstUSBDev->pstDRegs->dcfg    )        );
}

/**
 * @brief Function to handle Enumeration Done interrupt.
 * @note  dwc_otg_pcd_intr.c / dwc_otg_pcd_handle_enum_done_intr
 */
static void USB_intrEnumDone()
{
	if (USB_STATE_GETSTATE(pstUSBDev->iDevState) != USB_STATE_DEFAULT) {
		goto end;
	}
	
	uint16_t ep0mps;
	dsts_data_t dsts;
	depctl_data_t diepctl;
	
	dsts.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dsts);
	diepctl.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstInEP->diepctl);
	
	log_info("dsts.b.enumspd = %d", dsts.b.enumspd);
	
	switch (dsts.b.enumspd) {
	case DWC_DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ: // HS
		pstUSBDev->iSpeed = ARM_USB_SPEED_HIGH;
		break;
	case DWC_DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ: // FS
	case DWC_DSTS_ENUMSPD_FS_PHY_48MHZ:
		pstUSBDev->iSpeed = ARM_USB_SPEED_FULL;
		break;
	case DWC_DSTS_ENUMSPD_LS_PHY_6MHZ: // LS
	default:
		pstUSBDev->iSpeed = ARM_USB_SPEED_LOW;
		break;
	}
	USBD_SignalLinkSpeed(pstUSBDev->iSpeed);
	
	switch(pstUSBDev->iSpeed)
	{
	case ARM_USB_SPEED_HIGH:
		diepctl.b.mps = DWC_DEP0CTL_MPS_64;
		ep0mps = 64;
		break;
		
	case ARM_USB_SPEED_FULL:
		switch(RTE_USB2FS_MPS0) {
		case 64:
			diepctl.b.mps = DWC_DEP0CTL_MPS_64;
			break;
		case 32:
			diepctl.b.mps = DWC_DEP0CTL_MPS_32;
			break;
		case 16:
			diepctl.b.mps = DWC_DEP0CTL_MPS_16;
			break;
		case  8:
			diepctl.b.mps = DWC_DEP0CTL_MPS_8;
			break;
		default:
			log_error("Invalid MPS setting");
			assert(0);
		}
		ep0mps = RTE_USB2FS_MPS0;
		break;
		
	case ARM_USB_SPEED_LOW:
	default:
		diepctl.b.mps = DWC_DEP0CTL_MPS_8;
		ep0mps = 8;
		break;
	}
	DWC_WRITE_REG32(&pstUSBDev->stEPs[0].pstInEP->diepctl, diepctl.d32);
	
	dctl_data_t dctl;
	dctl.d32 = 0;
	dctl.b.cgnpinnak = 1;
	DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32, dctl.d32);
	
	/* Reset EP0 */
	USBD_EndpointConfigure_core(USB_ep_addr(0,0), ARM_USB_ENDPOINT_CONTROL, ep0mps);
	USBD_EndpointConfigure_core(USB_ep_addr(0,1), ARM_USB_ENDPOINT_CONTROL, ep0mps);
	USB_xferEP0(2, &pstUSBDev->stCtrlXfer);
	
	/* Set USB turnaround time based on device speed and PHY interface. */
	// 7.9 Choosing the Value of GUSBCFG.USBTrdTim
	if (1 == Driver_PMU.GetPrescaler(PMU_CD_USBI)) {
		// PHY 48MHz
		gusbcfg_data_t gusbcfg;
		gusbcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gusbcfg);
		switch(Driver_PMU.GetPrescaler(PMU_CD_USBB))
		{
		case 1: // AHB 48MHz
			gusbcfg.b.usbtrdtim = 5; 
			break;
		case 2: // AHB 24MHz
			gusbcfg.b.usbtrdtim = 9; 
			break;
		default:
			break;
		}
		DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gusbcfg, gusbcfg.d32);
	} else {
		log_error("%s Unknown PHY setting", __func__);
	}
	
	/* raise event */
	UsbEventFlag.b.enumdone = 1;
	
	USBD_SignalDeviceEvent(ARM_USBD_EVENT_RESET);
	
end:
	{
		/* Clear interrupt */
		const gintsts_data_t gintsts = {.d32 = 0, .b.enumdone=1};
		DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, gintsts.d32);
	}
	
//	log_debug("USB_intrEnumDone()");
//	log_debug("diepctl0 addr=0x%08X value=0x%08X",   &pstUSBDev->stEPs[0].pstInEP->diepctl   , DWC_READ_REG32(&pstUSBDev->stEPs[0].pstInEP->diepctl)  );
//	log_debug("doepctl0 addr=0x%08X value=0x%08X",   &pstUSBDev->stEPs[0].pstOutEP->doepctl  , DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepctl) );
//	log_debug("dsts     addr=0x%08X value=0x%08X",   &pstUSBDev->pstDRegs->dsts              , DWC_READ_REG32(&pstUSBDev->pstDRegs->dsts)    );
}

/**
 * @brief Function to handle IN Endpoint interrupts.
 * @note  dwc_otg_pcd_intr.c / dwc_otg_pcd_handle_in_ep_intr
 */
static void USB_intrInEP()
{
#define CLEAR_IN_EP_INTR(__epnum,__intr) \
do { \
	diepint_data_t diepint = {0}; \
	diepint.b.__intr = 1; \
	DWC_WRITE_REG32(&pstUSBDev->stEPs[__epnum].pstInEP->diepint, diepint.d32); \
} while (0)
	
	const uint8_t Dir = 1;
	diepint_data_t diepint;
	uint32_t ep_intr;
	int8_t epnum = 0;
	ep_intr = USB_read_dev_all_in_ep_intr();
//	log_debug("USB_intrInEP() ep_intr=0x%08X", ep_intr);
	
	while (ep_intr) {
		if (ep_intr & 0x1) {
			diepint.d32 = USB_read_dev_in_ep_intr(epnum);
			if (diepint.b.xfercompl) {
				CLEAR_IN_EP_INTR(epnum, xfercompl);
				if (epnum == 0) {
					handle_ep0();
				} else {
					USB_handle_ep(epnum,Dir);
				}
			}
			/* Endpoint disable      */
			if (diepint.b.epdisabled) {
				USB_intrINEPDisable(epnum);
				CLEAR_IN_EP_INTR(epnum,epdisabled);
				log_info("USB_intrInEP %d epdisabled", epnum);
			}
			/* AHB Error */
			if (diepint.b.ahberr) {
				CLEAR_IN_EP_INTR(epnum,ahberr);
				log_error("USB_intrInEP %d ahberr", epnum);
				stError.status = USBD_TZ10xx_ERROR_EP_AHB_ERROR;
				stError.param  = USB_ep_addr(epnum,Dir);
				USBD_SignalEndpointEvent(stError.param, (ARM_USBD_EP_EVENT)-1);
			}
			/* TimeOUT Handshake (non-ISOC IN EPs) */
			if (diepint.b.timeout) {
				USB_intrINEPTimeout(epnum);
				CLEAR_IN_EP_INTR(epnum,timeout);
				log_error("USB_intrInEP %d timeout", epnum);
				stError.status = USBD_TZ10xx_ERROR_EP_TIMEOUT;
				stError.param  = USB_ep_addr(epnum,Dir);
				USBD_SignalEndpointEvent(stError.param, (ARM_USBD_EP_EVENT)-1);
			}
//			/** IN Token received with TxF Empty */
//			if (diepint.b.intktxfemp) {
//				CLEAR_IN_EP_INTR(epnum, intktxfemp);
//				log_info("USB_intrInEP %d intktxfemp", epnum);
//			}
			/** IN Token Received with EP mismatch */
			if (diepint.b.intknepmis) {
				CLEAR_IN_EP_INTR(epnum,intknepmis);
				log_error("USB_intrInEP %d intknepmis", epnum);
				stError.status = USBD_TZ10xx_ERROR_EP_MISMATCH;
				stError.param  = USB_ep_addr(epnum,Dir);
				USBD_SignalEndpointEvent(stError.param, (ARM_USBD_EP_EVENT)-1);
			}
			/** IN Endpoint NAK Effective */
			if (diepint.b.inepnakeff) {
				USB_intrINEPNakEff(epnum);
				CLEAR_IN_EP_INTR(epnum,inepnakeff);
				log_info("USB_intrInEP %d inepnakeff", epnum);
			}
//			/** IN EP Tx FIFO Empty Intr */
//			if (diepint.b.emptyintr) {
//				CLEAR_IN_EP_INTR(epnum, emptyintr);
//				log_error("USB_intrInEP %d emptyintr", epnum);
//			}
//			/* NAK Interrutp */
//			if (diepint.b.nak) {
//				
//			}
			
			#ifdef DEBUG
			if (0 < epnum) {
				diepint.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[epnum].pstInEP->diepint);
				if (diepint.d32) {
					log_debug("unhandled diepint %d 0x%08X", epnum, diepint.d32);
				}
			}
			#endif
		}
		epnum++;
		ep_intr >>= 1;
	}
}


/**
 * @brief Function to handle Out Endpoint interrupts.
 * @note  dwc_otg_pcd_intr.c / dwc_otg_pcd_handle_out_ep_intr
 */
static void USB_intrOutEP()
{
#define CLEAR_OUT_EP_INTR(__epnum,__intr) \
do { \
	doepint_data_t doepint = {0}; \
	doepint.b.__intr = 1; \
	DWC_WRITE_REG32(&pstUSBDev->stEPs[__epnum].pstOutEP->doepint, doepint.d32);  \
} while (0)
	
	const uint8_t Dir = 0;
	uint32_t ep_intr;
	doepint_data_t doepint;
	uint8_t epnum = 0;
	ep_intr = USB_read_dev_all_out_ep_intr();
//	log_debug("USB_intrOutEP() ep_intr=0x%08X", ep_intr);
	
	while (ep_intr) {
		if (ep_intr & 0x1) {
			doepint.d32 = USB_read_dev_out_ep_intr(epnum);
			if (epnum == 0) {
				USB_intrOutEP0(doepint);
			} else {
				if (doepint.b.xfercompl) {
					CLEAR_OUT_EP_INTR(epnum,xfercompl);
					USB_handle_ep(epnum,Dir);
				}
			}
			/* ep common */
			if (doepint.b.ahberr) {
				CLEAR_OUT_EP_INTR(epnum,ahberr);
				log_error("USB_intrOutEP %d ahberr", epnum);
				stError.status = USBD_TZ10xx_ERROR_EP_AHB_ERROR;
				stError.param  = USB_ep_addr(epnum,Dir);
				USBD_SignalEndpointEvent(stError.param, (ARM_USBD_EP_EVENT)-1);
			}
			if (doepint.b.epdisabled) {
				USB_EPStopped(epnum,Dir);
				CLEAR_OUT_EP_INTR(epnum,epdisabled);
				const gintmsk_data_t gintmsk = {.d32 = 0, .b.goutnakeff = 1 };
				// 再び割り込みを発生させ、USB_intrGlobalOUTNakEff()に処理を移します。
				DWC_MODIFY_REG32(&pstUSBDev->pstGRegs->gintmsk, 0, gintmsk.d32);
				log_info("USB_intrOutEP %d epdisabled", epnum);
			}
			
			#ifdef DEBUG
			if (0 < epnum) {
				doepint.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[epnum].pstOutEP->doepint);
				if (doepint.d32) {
					log_debug("unhandled doepint %d 0x%08X", epnum, doepint.d32);
				}
			}
			#endif
		}
		epnum++;
		ep_intr >>= 1;
	}
}

/**
 * @brief Function to handle EP Mismatch interrupts.
 * @note  dwc_otg_pcd_intr.c / dwc_otg_pcd_handle_ep_mismatch_intr
 */
static void USB_intrEPMismatch(void)
{
	gintsts_data_t gintsts;
	dctl_data_t dctl;
	gintmsk_data_t intr_mask = {.d32 = 0 };
	
	pstUSBDev->start_predict = 1;
	
	gintsts.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gintsts);
	if (!gintsts.b.ginnakeff) {
		/* Disable EP Mismatch interrupt */
		intr_mask.d32 = 0;
		intr_mask.b.epmismatch = 1;
		DWC_MODIFY_REG32(&pstUSBDev->pstGRegs->gintmsk, intr_mask.d32, 0);
		/* Enable the Global IN NAK Effective Interrupt */
		pstUSBDev->ginnak_factor.bit.epmis = 1;
		intr_mask.d32 = 0;
		intr_mask.b.ginnakeff = 1;
		DWC_MODIFY_REG32(&pstUSBDev->pstGRegs->gintmsk, 0, intr_mask.d32);
		/* Set the global non-periodic IN NAK handshake */
		dctl.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dctl);
		dctl.b.sgnpinnak = 1;
		DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32);
	} else {
		log_warning("%s gintsts.b.ginnakeff = 1! dctl.b.sgnpinnak not set", __func__);
	}
	/* Disabling of all EP's will be done in dwc_otg_pcd_handle_in_nak_effective()
	 * handler after Global IN NAK Effective interrupt will be asserted */
	
	gintsts.d32 = 0;
	gintsts.b.epmismatch = 1;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, gintsts.d32);
	
	stError.status = USBD_TZ10xx_ERROR_MISMATCH;
	stError.param  = 0;
	USBD_SignalDeviceEvent((ARM_USBD_EVENT)-1);
}

/**
 * @brief Function to handle Global IN NAK Effective interrupts.
 * @note dwc_otg_pcd_intr.c / dwc_otg_pcd_handle_in_nak_effective
 */
static void USB_intrGlobalINNakEff(void)
{
	const uint8_t Dir = 1;
	
	uint8_t disabled_count = 0;
	/* Clear disabled eps */
	pstUSBDev->ginnakeff_disabled_count = 0;
	memset(pstUSBDev->ginnakeff_disabled_ep, 0xff, sizeof(pstUSBDev->ginnakeff_disabled_ep));
	
	/* Disable all active Non-Periodic IN EPs */
	for (uint8_t i = 0; i < USB_NUM_EPS; i++) {
		depctl_data_t diepctl;
		diepctl.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[i].pstInEP->diepctl);
		if (USB_EP_IS_PERIODIC(diepctl.b.eptype)) {
			continue;
		}
		switch(pstUSBDev->stEPs[i].eState[Dir])
		{
		case USB_EP_ABORTING:
			if (diepctl.b.epena) {
				diepctl.b.epdis = 1; // diepint.epdisabled interrupt will be asserted
				diepctl.b.snak = 1;
				DWC_WRITE_REG32(&pstUSBDev->stEPs[i].pstInEP->diepctl, diepctl.d32); 
				log_warning("%s aborting ep %d disable", __func__, i); 
			} else {
				USB_EPStopped(i,Dir);
			}
			break;
			
		case USB_EP_STALLING:
			if (diepctl.b.epena) {
				diepctl.b.epdis = 1; // diepint.epdisabled interrupt will be asserted
				log_warning("%s stalling ep %d disable", __func__, i); 
			} else {
				USB_EPStopped(i,Dir);
			}
			diepctl.b.stall = 1; // stallはepena==0の場合でもsetできます
			DWC_WRITE_REG32(&pstUSBDev->stEPs[i].pstInEP->diepctl, diepctl.d32); 
			break;
			
		case USB_EP_TIMEOUT:
		case USB_EP_RUNNING:
			if (diepctl.b.epena) {
				diepctl.b.epdis = 1;
				diepctl.b.snak = 1;
				DWC_WRITE_REG32(&pstUSBDev->stEPs[i].pstInEP->diepctl, diepctl.d32);
				log_warning("%s ginnakeff epdis %d", __func__, i); 
			}
			break;
			
		default:
			break;
		}
		if (diepctl.b.epena && diepctl.b.epdis) {
			disabled_count++;
			if (pstUSBDev->start_predict > 0) {
				pstUSBDev->start_predict++;
			} else {
				pstUSBDev->ginnakeff_disabled_ep[pstUSBDev->ginnakeff_disabled_count] = i;
				pstUSBDev->ginnakeff_disabled_count++;
			}
		}
	}
	
	log_warning("%s ginnakeff start_predict %d ginnakeff_disabled_count %d", __func__, pstUSBDev->start_predict, pstUSBDev->ginnakeff_disabled_count); 
	
	if (disabled_count == 0) {
		log_warning("%s disabled_count == 0 factor 0x%02X", __func__, pstUSBDev->ginnak_factor.byte);
		/* Global IN NAK  and  All EPs disabled. */
		/* Flush Shared NP TxFIFO */
		USB_otg_flush_tx_fifo(0);
		if (pstUSBDev->ginnak_factor.bit.epmis) {
			pstUSBDev->ginnak_factor.bit.epmis = 0;
			pstUSBDev->start_predict = 0;
			/* Unmask EP Mismatch interrupt */
			gintmsk_data_t gintmsk_data;
			gintmsk_data.d32 = 0;
			gintmsk_data.b.epmismatch = 1;
			DWC_MODIFY_REG32(&pstUSBDev->pstGRegs->gintmsk, 0, gintmsk_data.d32);
		}
		if (pstUSBDev->ginnak_factor.bit.timeout) {
			pstUSBDev->ginnak_factor.bit.timeout = 0;
			/* Set eState running */
			for (uint8_t ucEP=0; ucEP<USB_NUM_EPS; ucEP++) {
				if (pstUSBDev->stEPs[ucEP].eState[Dir] == USB_EP_TIMEOUT) {
					pstUSBDev->stEPs[ucEP].eState[Dir] = USB_EP_RUNNING;
				}
			}
		}
		if (pstUSBDev->ginnak_factor.bit.ep0_premature_status) {
			pstUSBDev->ginnak_factor.bit.ep0_premature_status = 0;
		}
		if (pstUSBDev->ginnak_factor.bit.stall_abort) {
			pstUSBDev->ginnak_factor.bit.stall_abort = 0;
		}
		/* Clear the global non-periodic IN NAK handshake */
		dctl_data_t dctl;
		dctl.d32 = 0;
		dctl.b.cgnpinnak = 1;
		DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32, dctl.d32); 
	}
	
	/* Disable the Global IN NAK Effective Interrupt */
	gintmsk_data_t intr_mask;
	intr_mask.d32 = 0;
	intr_mask.b.ginnakeff = 1;
	DWC_MODIFY_REG32(&pstUSBDev->pstGRegs->gintmsk, intr_mask.d32, 0);
	
	/* Clear interrupt */
	gintsts_data_t gintsts;
	gintsts.d32 = 0;
	gintsts.b.ginnakeff = 1;
	DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, gintsts.d32);
	
}


/**
 * @brief Function to service all USB interrupt events.
 * @note  dwc_otg_pcd_intr.c / dwc_otg_pcd_handle_intr
 * @note  dwc_otg_cil_intr.c / dwc_otg_handle_common_intr
 */
void USB_IRQHandler(void)
{
	// Figure 5-5 Core Interrupt Handler
	
	// EPの割り込み
	static const gintsts_data_t EPInt = {
		.d32 = 0,
		.b.inepint = 1,
		.b.outepintr = 1
	};
	
	gintsts_data_t gintr_status;
	
	gintr_status.d32 = USB_otg_read_core_intr();
	
	if (~EPInt.d32 & gintr_status.d32) {
		/* EP以外の割り込みがあった */
		
		/* OTG interrupt event*/
		if (gintr_status.b.otgintr) {
			gotgint_data_t gotgint;
			gotgint.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gotgint);
			log_info("%s OTG int 0x%08X", __func__, gotgint.d32); 
			/* Disconnect interupt event*/
			if(DriverCapabilities.event_power_off && gotgint.b.sesenddet) {
				USB_ResumeEarly();
				UsbEventFlag.b.sesenddet = 1;
				USBD_SignalDeviceEvent(ARM_USBD_EVENT_POWER_OFF);
				log_info("%s sesenddet (VBUS off)", __func__); 
			}
			/* clear root intsts */
			DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gotgint, gotgint.d32);
		}
		
		/* Host/Device common Interrupt */
		if(gintr_status.b.wkupintr) {
			const gintsts_data_t gintsts = {.d32 = 0, .b.wkupintr = 1};
			DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, gintsts.d32);
			if (USB_STATE_GETSTATE(pstUSBDev->iDevState) >= USB_STATE_POWERED) {
				USB_ResumeEarly();
				pstUSBDev->iDevState &= ~USB_STATE_SUSPEND_BIT;
				UsbEventFlag.b.wkupintr =1;
				USBD_SignalDeviceEvent(ARM_USBD_EVENT_RESUME);
			}
			log_warning("%s wkupintr (resume)", __func__); 
		}
		if(DriverCapabilities.event_power_on && gintr_status.b.sessreqintr) {
			const gintsts_data_t gintsts = {.d32 = 0, .b.sessreqintr = 1};
			DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, gintsts.d32);
			if (USB_STATE_GETSTATE(pstUSBDev->iDevState) == USB_STATE_DETACH) {
				USB_ResumeEarly();
				pstUSBDev->iDevState = USB_STATE_POWERED;
				UsbEventFlag.b.sessreqintr =1;
				USBD_SignalDeviceEvent(ARM_USBD_EVENT_POWER_ON);
			}
			log_info("%s sessreqintr (VBUS on)", __func__); 
		}
		if(gintr_status.b.modemismatch) {
			const gintsts_data_t gintsts = {.d32 = 0, .b.modemismatch = 1};
			DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, gintsts.d32);
			log_error("%s modemismatch", __func__); 
		}
		
		/* Device global Interrupt */
		if (gintr_status.b.sofintr) {
			USB_intrSOF();
		}
//		Synopsys Linux Driverでは、これはDMAを使わない場合(slave mode)に利用する割り込みの模様。
//		if (gintr_status.b.nptxfempty) {
//			const gintsts_data_t gintsts = {.d32 = 0, .b.nptxfempty = 1};
//			DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, gintsts.d32);
//			log_error("%s nptxfempty", __func__); 
//		}
		if(gintr_status.b.goutnakeff) {
			USB_intrGlobalOUTNakEff();
			log_info("%s goutnakeff", __func__); 
		}
		if(gintr_status.b.erlysuspend) {
			const gintsts_data_t gintsts = {.d32 = 0, .b.erlysuspend = 1};
			DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, gintsts.d32);
			UsbEventFlag.b.erlysuspend =1;
			log_warning("%s erlysuspend", __func__); 
		}
		if(gintr_status.b.usbsuspend) {
			const gintsts_data_t gintsts = {.d32 = 0, .b.usbsuspend = 1};
			DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, gintsts.d32);
			if (USB_STATE_GETSTATE(pstUSBDev->iDevState) >= USB_STATE_POWERED) {
				pstUSBDev->iDevState |= USB_STATE_SUSPEND_BIT;
				UsbEventFlag.b.usbsuspend =1;
				USBD_SignalDeviceEvent(ARM_USBD_EVENT_SUSPEND);
				USB_Suspend();
			}
			log_warning("%s usbsuspend (suspend)", __func__);
		}
		if (gintr_status.b.usbreset) {
			USB_intrUSBReset();
			log_warning("%s usbreset", __func__); 
		}
		if (gintr_status.b.enumdone) {
			USB_intrEnumDone();
			log_warning("%s enumdone (linkup)", __func__); 
		}
	}
	
	/* IN Endpoint interrupt*/
	if (gintr_status.b.inepint) {
		USB_intrInEP();
	}
	/* Out Endpoint interrupt*/
	if (gintr_status.b.outepintr) {
		USB_intrOutEP();
	}
	
	if (gintr_status.b.epmismatch) {
		USB_intrEPMismatch();
		log_warning("%s epmismatch", __func__); 
	}
	
//	これはSynopsys Linux Driverでも使っていない模様
//	if (gintr_status.b.fetsusp) {
//		const gintsts_data_t gintsts = {.d32 = 0, .b.fetsusp = 1};
//		DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintsts, gintsts.d32);
//		log_warning("%s fetsusp", __func__); 
//	}
	
	if (gintr_status.b.ginnakeff) {
		USB_intrGlobalINNakEff();
	}
}


/* ***************************************************************************** */
/*                            Endpoint handling functions                        */
/* ***************************************************************************** */

static TZ1K_INLINE void USB_EP0IntMask(bool_t mask)
{
	const daint_data_t daintmsk = {
		.d32 = 0,
		.b.inep0 = 1,
		.b.outep0 = 1,
	};
	
	if (mask) {
		// マスクする(割り込み無効)
		DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->daintmsk, daintmsk.d32, 0);
	} else {
		// アンマスク(割り込み有効)
		DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->daintmsk, 0, daintmsk.d32);
	}
}

/**
 * @brief Function to configure EP0 to receive Setup packet.
 * @note  dwc_otg_pcd_intr.c / ep0_out_start
 */
static USB_err_t USB_ep0_out_start(void)
{
	deptsiz0_data_t doeptsize0;
	depctl_data_t doepctl;
	
	doeptsize0.d32 = 0;
	doeptsize0.b.supcnt = 3;
	doeptsize0.b.pktcnt = 1;
	doeptsize0.b.xfersize = 8 * 3;  // But, application-programmed initial transfer size is 8
	DWC_WRITE_REG32(&pstUSBDev->stEPs[0].pstOutEP->doeptsiz, doeptsize0.d32);
	
	// ドライバを呼ぶ側が提供するバッファサイズは8byteですが、
	// SETUPパケット３つ(8*3byte)がDMA転送される可能性があるため、ドライバのローカルのバッファに転送します。
	DWC_WRITE_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepdma, (uint32_t)EP0outSetupBuf);
	
	doepctl.d32 = 0;
	doepctl.b.epena = 1;
	DWC_MODIFY_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepctl, 0, doepctl.d32);
	
	assert_ep_xfer(doeptsize0.b.pktcnt, doeptsize0.b.xfersize, EP0outSetupBuf);
	
	log_debug("USB_ep0_out_start doeptsiz0 addr=0x%08X value=0x%08X, doepdma0 addr=0x%08X value=0x%08X",
		&pstUSBDev->stEPs[0].pstOutEP->doeptsiz, DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doeptsiz),
		&pstUSBDev->stEPs[0].pstOutEP->doepdma,  DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepdma)
	);
	log_debug("USB_ep0_out_start doepctl0 addr=0x%08X value=0x%08X written=0x%08X",
		&pstUSBDev->stEPs[0].pstOutEP->doepctl,  DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepctl), doepctl.d32
	);
	
	return USB_OK;
}

/**
 * @brief Function to configure EP0OUT to NAK state.
 */
static USB_err_t USB_ep0_out_snak(void)
{
	depctl_data_t doepctl;
	
	doepctl.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepctl);
	doepctl.b.snak = 1;
	DWC_WRITE_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepctl, doepctl.d32);
	
	return USB_OK;
}


/**
 * @brief Function to configure EP0 to transmit IN data packets.
 * @note  dwc_otg_cil.c / dwc_otg_ep0_start_transfer
 */
static USB_err_t
USB_EP0DataInXfer(USB_xfer_t *pstXfer)
{
	const uint8_t Dir = 1;
	dwc_otg_dev_in_ep_regs_t *in_regs;
	in_regs = pstUSBDev->stEPs[0].pstInEP;
	
	depctl_data_t depctl;
	deptsiz0_data_t deptsiz;
	depctl.d32 = DWC_READ_REG32(&in_regs->diepctl);
	deptsiz.d32 = DWC_READ_REG32(&in_regs->dieptsiz);
	
	if (pstUSBDev->EP0State_PhaseEntered) {
		if (depctl.b.epena) {
			log_warning("%s Already started", __func__);
			return USB_OK; // Already started
		}
	}
	
	if ((pstXfer->usLen == 0) || (pstXfer->usLen == pstXfer->usActLen)) {
		deptsiz.b.xfersize = 0;
		deptsiz.b.pktcnt = 1;
	} else {
		/* Program the transfer size and packet count
		 *      as follows: xfersize = N * maxpacket +
		 *      short_packet pktcnt = N + (short_packet
		 *      exist ? 1 : 0) 
		 */
		if (pstXfer->usLen > pstUSBDev->stEPs[0].usMPS[Dir]) {
			pstXfer->usLen = pstUSBDev->stEPs[0].usMPS[Dir];
			deptsiz.b.xfersize = pstUSBDev->stEPs[0].usMPS[Dir];
		} else {
			deptsiz.b.xfersize = pstXfer->usLen;
		}
		deptsiz.b.pktcnt = 1;
	}
	/* application-programmed initial transfer size */
	pstXfer->usTrfSz = deptsiz.b.xfersize;
	
	DWC_WRITE_REG32(&in_regs->dieptsiz, deptsiz.d32);
	DWC_WRITE_REG32(&in_regs->diepdma, (uint32_t)pstXfer->pvBuffer);
	
	depctl.b.epena = 1;
	depctl.b.cnak = 1;
	depctl.b.nextep = pstUSBDev->nextep_seq[0];
	DWC_WRITE_REG32(&in_regs->diepctl, depctl.d32);
	
//	log_debug("diepctl0  addr=0x%08X value=0x%08X written=0x%08X",   &in_regs->diepctl   , DWC_READ_REG32(&in_regs->diepctl)  , depctl.d32  );
//	log_debug("diepdma0  addr=0x%08X value=0x%08X written=0x%08X",   &in_regs->diepdma   , DWC_READ_REG32(&in_regs->diepdma)  , (uint32_t)pstXfer->pvBuffer );
//	log_debug("dieptsiz0 addr=0x%08X value=0x%08X written=0x%08X",   &in_regs->dieptsiz  , DWC_READ_REG32(&in_regs->dieptsiz) , deptsiz.d32 );
	
	assert_ep_xfer(deptsiz.b.pktcnt, deptsiz.b.xfersize, pstXfer->pvBuffer);
	
	return USB_OK; 
}

/**
*	@brief  Function to EP0 data packet transmit done.
**/ 
static USB_err_t
USB_EP0DataXferDone(USB_ctrlXfer_t *pstCXfer, uint8_t Dir)
{
	const uint8_t ucEP = 0;
	
	USB_xfer_t *pstXfer; 
	pstXfer = pstUSBDev->stEPs[ucEP].pstXfer[Dir];
	if(pstXfer) {
		if (USB_EP_RUNNING != pstUSBDev->stEPs[ucEP].eState[Dir]) {
			pstXfer->iStatus = USB_EABORT;
			log_warning("%s eState %d", __func__, pstUSBDev->stEPs[ucEP].eState[Dir]);
		} else {
			pstXfer->iStatus = USB_OK;
		}
		/* 転送を削除 */
		pstUSBDev->stEPs[ucEP].pstXfer[Dir] = NULL;
		if (Dir) {
			USBD_SignalEndpointEvent(USB_ep_addr(0,Dir), ARM_USBD_EP_EVENT_IN);
		} else {
			USBD_SignalEndpointEvent(USB_ep_addr(0,Dir), ARM_USBD_EP_EVENT_OUT);
		}
		return USB_OK;
	} else {
		log_error("%d pstXfer NULL", __func__);
	}
	
	return USB_EIO;
}

/**
 * @brief Function to configure EP0 to receive OUT data packets.
 * @note  dwc_otg_cil.c / dwc_otg_ep0_start_transfer
 */
static USB_err_t
USB_EP0DataOutXfer(USB_xfer_t *pstXfer)
{
	const uint8_t Dir = 0;
	dwc_otg_dev_out_ep_regs_t *out_regs;
	out_regs = pstUSBDev->stEPs[0].pstOutEP;
	
	depctl_data_t depctl;
	deptsiz0_data_t deptsiz;
	depctl.d32  = DWC_READ_REG32(&out_regs->doepctl);
	deptsiz.d32 = DWC_READ_REG32(&out_regs->doeptsiz);
	
	/* Program the transfer size and packet count as follows:
	 *      xfersize = N * (maxpacket + 4 - (maxpacket % 4))
	 *      pktcnt = N
	 */
	deptsiz.b.xfersize = pstUSBDev->stEPs[0].usMPS[Dir];
	deptsiz.b.pktcnt = 1;
	if (pstUSBDev->EP0State_PhaseEntered) {
		deptsiz.b.supcnt = 3; // OUT DATA Phase first Xfer, or, OUT STATUS Phase first Xfer
	}
	/* application-programmed initial transfer size */
	pstXfer->usTrfSz = deptsiz.b.xfersize;
	
	DWC_WRITE_REG32(&out_regs->doeptsiz, deptsiz.d32);
	DWC_WRITE_REG32(&out_regs->doepdma, ((int32_t)pstXfer->pvBuffer));
	
	depctl.b.epena = 1;
	depctl.b.cnak = 1;
	DWC_WRITE_REG32(&out_regs->doepctl, depctl.d32);
	
	assert_ep_xfer(deptsiz.b.pktcnt, deptsiz.b.xfersize, pstXfer->pvBuffer);
	
	return USB_OK; 
}

/**
*	@brief Function to activate endpoint.
*	@note  dwc_otg_cil.c / dwc_otg_ep_activate
**/ 
static void USB_EP_Activate(uint8_t ep_num, uint8_t Dir)
{
	depctl_data_t depctl;
	volatile uint32_t *addr;
	daint_data_t daintmsk = {.d32 = 0 };
	
	log_warning("USB_EP_Activate ep_num=%d Dir=%d", ep_num, Dir);
	
	if (0 == ep_num) return;
	
	#ifdef DEBUG
	if (Dir == 1) {
		// IN
		switch(ep_num) {
		case 1:  assert(OTG_EP_DIR_1 & OTG_EP_DIR_IN); break;
		case 2:  assert(OTG_EP_DIR_2 & OTG_EP_DIR_IN); break;
		case 3:  assert(OTG_EP_DIR_3 & OTG_EP_DIR_IN); break;
		default: assert(0); break;
		}
		
	} else {
		// OUT
		switch(ep_num) {
		case 1:  assert(OTG_EP_DIR_1 & OTG_EP_DIR_OUT); break;
		case 2:  assert(OTG_EP_DIR_2 & OTG_EP_DIR_OUT); break;
		case 3:  assert(OTG_EP_DIR_3 & OTG_EP_DIR_OUT); break;
		default: assert(0); break;
		}
	}
	#endif
	
	if (Dir == 1) {
		addr = &pstUSBDev->stEPs[ep_num].pstInEP->diepctl;
		daintmsk.ep.in = 1 << ep_num;
	} else {
		addr = &pstUSBDev->stEPs[ep_num].pstOutEP->doepctl;
		daintmsk.ep.out = 1 << ep_num;
	}
	
	depctl.d32 = DWC_READ_REG32(addr);
	if (!depctl.b.usbactep) {
		depctl.b.mps = pstUSBDev->stEPs[ep_num].usMPS[Dir];
		depctl.b.eptype = pstUSBDev->stEPs[ep_num].eptype[Dir];
		depctl.b.txfnum = pstUSBDev->stEPs[ep_num].txfnum;
		
		depctl.b.setd0pid = 1;
		depctl.b.usbactep = 1;
		depctl.b.stall    = 0;   // clear stall condition
		
		/* Update nextep_seq array and EPMSCNT in DCFG*/
		if (!USB_EP_IS_PERIODIC(depctl.b.eptype) && (Dir == 1)) {	// Non-Periodic IN
			uint8_t i;
			for (i=0; i < USB_NUM_EPS; i++) {
				if (pstUSBDev->nextep_seq[i] == pstUSBDev->first_in_nextep_seq) break;
			}
			pstUSBDev->nextep_seq[i] = ep_num;	
			pstUSBDev->nextep_seq[ep_num] = pstUSBDev->first_in_nextep_seq;
			depctl.b.nextep = pstUSBDev->nextep_seq[ep_num];
			
			dcfg_data_t dcfg;
			dcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dcfg);
			dcfg.b.epmscnt++;
			DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dcfg, dcfg.d32);
			
			log_warning("%s first_in_nextep_seq= %2d; nextep_seq[]:", __func__, pstUSBDev->first_in_nextep_seq);
			for (i=0; i < USB_NUM_EPS; i++) {
				log_warning("%2d", pstUSBDev->nextep_seq[i]);
			}
		}
		
		DWC_WRITE_REG32(addr, depctl.d32);
		log_warning("dXepctlN addr=0x%08X value=0x%08X", addr, DWC_READ_REG32(addr));
	}
	
	DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->daintmsk, 0, daintmsk.d32);
	log_warning("daintmsk addr=0x%08X value=0x%08X", &pstUSBDev->pstDRegs->daintmsk, DWC_READ_REG32(&pstUSBDev->pstDRegs->daintmsk));
}

/**
*	@brief Function to deactivate endpoint.
*	@note  dwc_otg_cil.c / dwc_otg_ep_deactivate
**/ 
static void USB_EP_Deactivate(uint8_t ep_num, uint8_t Dir)
{
	depctl_data_t depctl;
	volatile uint32_t *addr;
	daint_data_t daintmsk = {.d32 = 0 };
	
	log_warning("USB_EP_Deactivate ep_num=%d Dir=%d", ep_num, Dir);
	
	if (0 == ep_num) return;
	
	if (Dir == 1) {
		addr = &pstUSBDev->stEPs[ep_num].pstInEP->diepctl;
		daintmsk.ep.in  = 1 << ep_num;
	} else {
		addr = &pstUSBDev->stEPs[ep_num].pstOutEP->doepctl;
		daintmsk.ep.out = 1 << ep_num;
	}
	
	depctl.d32 = DWC_READ_REG32(addr);
	if (depctl.b.usbactep) {	// If it is not the bus reset or deactivation after activation
		/* Update nextep_seq array and EPMSCNT in DCFG*/
		if (!USB_EP_IS_PERIODIC(depctl.b.eptype) && (Dir == 1)) {	// Non-Periodic IN
			uint8_t i;
			for (i = 0; i < USB_NUM_EPS; i++) {
				if (pstUSBDev->nextep_seq[i] == ep_num) break;
			}
			pstUSBDev->nextep_seq[i] = pstUSBDev->nextep_seq[ep_num];	
			if (pstUSBDev->first_in_nextep_seq == ep_num)
				pstUSBDev->first_in_nextep_seq = i;
			pstUSBDev->nextep_seq[ep_num] = 0xff;
			
			dcfg_data_t dcfg;
			dcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dcfg);
			dcfg.b.epmscnt--;
			DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dcfg, dcfg.d32);
			
			log_warning("%s first_in_nextep_seq= %2d; nextep_seq[]:", __func__, pstUSBDev->first_in_nextep_seq);
			for (i=0; i < USB_NUM_EPS; i++) {
				log_warning("%2d\n", pstUSBDev->nextep_seq[i]);
			}
		}
	}
	
	depctl.b.usbactep = 0;
	if (Dir == 1) {
		depctl.b.txfnum = 0;
		depctl.b.nextep = 0;
	}
	
	DWC_WRITE_REG32(addr, depctl.d32);
	log_warning("dXepctlN addr=0x%08X value=0x%08X", addr, *addr);
	
	DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->daintmsk, daintmsk.d32, 0);
	log_warning("daintmsk addr=0x%08X value=0x%08X", &pstUSBDev->pstDRegs->daintmsk, DWC_READ_REG32(&pstUSBDev->pstDRegs->daintmsk));
}

/**
*	@brief Function to configure EP to transmit/receive packet.
*	@note  dwc_otg_cil.c / dwc_otg_ep_start_transfer
**/ 
static void USB_StartXfer(uint8_t ep_addr, USB_xfer_t *pstXfer)
{
	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
//	uint8_t ep_type = pstUSBDev->stEPs[ep_num].eptype[ep_dir];
	uint32_t pktcnt;
	uint32_t xfersize;
	void *pvBufferPos;
	deptsiz_data_t deptsiz;
	depctl_data_t depctl;
	
	if (ep_dir) {
		/* IN transfer */
		dwc_otg_dev_in_ep_regs_t *in_regs = pstUSBDev->stEPs[ep_num].pstInEP;
		depctl.d32 = DWC_READ_REG32(&in_regs->diepctl);
		deptsiz.d32 = DWC_READ_REG32(&in_regs->dieptsiz);
		
		pvBufferPos = ((int8_t*)pstXfer->pvBuffer) + pstXfer->usActLen;
		if ((pstXfer->usLen == 0) || (pstXfer->usLen == pstXfer->usActLen)) {
			xfersize = 0;
			pktcnt = 1;
		} else {
			/* Program the transfer size and packet count as follows:
			*       xfersize = N * maxpacket + short_packet
			*       pktcnt   = N + (short_packet exist ? 1 : 0) 
			*/
			xfersize = pstXfer->usLen - pstXfer->usActLen;
			pktcnt = (xfersize + (pstUSBDev->stEPs[ep_num].usMPS[ep_dir] - 1)) / pstUSBDev->stEPs[ep_num].usMPS[ep_dir];
		}
		
		deptsiz.b.pktcnt   = pktcnt;
		deptsiz.b.xfersize = xfersize;
		pstXfer->usTrfSz   = xfersize;  /* application-programmed initial transfer size */
		if (USB_EP_IS_PERIODIC(depctl.b.eptype)) {	// Periodic IN
			deptsiz.b.mc = 1;
		}
		DWC_WRITE_REG32(&in_regs->dieptsiz, deptsiz.d32);
		DWC_WRITE_REG32(&in_regs->diepdma, (uint32_t)pvBufferPos);
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		if (!USB_EP_IS_PERIODIC(depctl.b.eptype)) {	// Non-Periodic IN
			depctl.b.nextep = pstUSBDev->nextep_seq[ep_num];
		}
		DWC_WRITE_REG32(&in_regs->diepctl, depctl.d32);
		
		assert_ep_xfer(pktcnt, xfersize, pvBufferPos);
		log_debug("%s dieptsiz addr=0x%08X value=0x%08X, diepdma addr=0x%08X value=0x%08X, diepctl addr=0x%08X value=0x%08X",
			 __func__, 
			&in_regs->dieptsiz, DWC_READ_REG32(&in_regs->dieptsiz),
			&in_regs->diepdma,  DWC_READ_REG32(&in_regs->diepdma),
			&in_regs->diepctl,  DWC_READ_REG32(&in_regs->diepctl)
		);
		
	} else {
		/* OUT transfer */
		/* 9.5 Non-Isochronous OUT Data Transfers Without Thresholding */
		dwc_otg_dev_out_ep_regs_t *out_regs = pstUSBDev->stEPs[ep_num].pstOutEP;
		deptsiz.d32 = DWC_READ_REG32(&out_regs->doeptsiz);
		depctl.d32  = DWC_READ_REG32(&out_regs->doepctl);
		
		/* Program the transfer size and packet count as follows:
		 *      pktcnt = N
		 *      xfersize = N * maxpacket
		 */
		pvBufferPos = ((int8_t*)pstXfer->pvBuffer) + pstXfer->usActLen;
		if ((pstXfer->usLen == 0) || (pstXfer->usLen == pstXfer->usActLen)) {
			pktcnt   = 1;
			xfersize = pstUSBDev->stEPs[ep_num].usMPS[ep_dir];
		} else {
			uint32_t remain = pstXfer->usLen - pstXfer->usActLen;
			pktcnt   = (remain + (pstUSBDev->stEPs[ep_num].usMPS[ep_dir] - 1)) / pstUSBDev->stEPs[ep_num].usMPS[ep_dir];
			xfersize = pktcnt * pstUSBDev->stEPs[ep_num].usMPS[ep_dir];
		}
		
		deptsiz.b.pktcnt   = pktcnt;
		deptsiz.b.xfersize = xfersize;
		pstXfer->usTrfSz   = xfersize;  /* application-programmed initial transfer size */
		DWC_WRITE_REG32(&out_regs->doeptsiz, deptsiz.d32);
		DWC_WRITE_REG32(&out_regs->doepdma, (uint32_t)pvBufferPos);
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		DWC_WRITE_REG32(&out_regs->doepctl, depctl.d32);
		
		assert_ep_xfer(pktcnt, xfersize, pvBufferPos);
		log_debug("%s doeptsiz addr=0x%08X value=0x%08X, doepdma addr=0x%08X value=0x%08X, doepctl addr=0x%08X value=0x%08X written_value=0x%08X",
			__func__,
			&out_regs->doeptsiz, DWC_READ_REG32(&out_regs->doeptsiz),
			&out_regs->doepdma,  DWC_READ_REG32(&out_regs->doepdma),
			&out_regs->doepctl,  DWC_READ_REG32(&out_regs->doepctl),  depctl.d32
		);
	}
}

/**
*	@brief Wait for EP transfer/stopping completion.
**/ 
static USB_err_t USB_waitEPcomplete(uint8_t ucEP, uint8_t Dir)
{
	USB_err_t iErr = USB_OK;
	int count = 0;
	
	while(1) {
		DWC_DELAY_US(10);
		if (USB_EP_STOPPED & pstUSBDev->stEPs[ucEP].eState[Dir]) break;
		if (UsbEventFlag.d32 & USB_EVENT_EP(ucEP,Dir)) break;
		if (++count > 100) {
			log_warning("%s EP%d Dir%d TIMEOUT! ", __func__, ucEP, Dir);
			iErr = USB_ETIMEOUT;
			stError.status = USBD_TZ10xx_ERROR_CORE_HANG;
			stError.param  = __LINE__;
			USBD_SignalDeviceEvent((ARM_USBD_EVENT)-1);
			break;
		}
	}
	
	UsbEventFlag.d32 &= ~(USB_EVENT_EP(ucEP,Dir));
	return iErr;
}

/**
*	@brief 
**/ 
static bool_t USB_SetINEPNak(uint8_t ucEP)
{
	diepint_data_t diepint;
	depctl_data_t depctl;
	diepmsk_data_t diepmsk;
	
	bool_t isSet = false;
	
	diepint.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->diepint);
	diepmsk.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->diepmsk);
	depctl.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->diepctl);
	// 現在転送中の場合にNakをセットする
	if (depctl.b.epena && depctl.b.usbactep) {
		if (!diepmsk.b.inepnakeff) {
			diepmsk.b.inepnakeff = 1;
			DWC_WRITE_REG32(&pstUSBDev->pstDRegs->diepmsk, diepmsk.d32);
		}
		if (!diepint.b.inepnakeff) {
			depctl.b.snak = 1;
		}
		DWC_WRITE_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->diepctl, depctl.d32);
		isSet = true;
	}
	
	return isSet;
}

/**
*	@brief 
**/ 
static void USB_ClearINEPNak(uint8_t ucEP)
{
//	diepint_data_t diepint;
	diepmsk_data_t diepmsk;
//	depctl_data_t depctl;
	
//	diepint.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->diepint);
	diepmsk.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->diepmsk);
//	depctl.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->diepctl);
	if (diepmsk.b.inepnakeff) {
		diepmsk.b.inepnakeff = 0;
		DWC_WRITE_REG32(&pstUSBDev->pstDRegs->diepmsk, diepmsk.d32);
	}
//	if (diepint.b.inepnakeff) {
//		depctl.b.cnak = 1;
//		depctl.b.snak = 0;
//	}
//	DWC_WRITE_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->diepctl, depctl.d32);
}


/**
*	@brief Set Global OUT Nak
**/ 
static bool_t USB_SetGlobalOUTNak(uint8_t ucEP)
{
	dctl_data_t dctl = {.d32 = 0 };
	gintsts_data_t gintsts = {.d32 = 0 };
	gintmsk_data_t gintmsk = {.d32 = 0 };
	depctl_data_t depctl = {.d32 = 0 };
	
	bool_t isSet = false;
	
	dctl.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dctl);
	gintsts.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gintsts);
	gintmsk.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gintmsk);
	depctl.d32  = DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstOutEP->doepctl);
	// 現在転送中の場合にNakをセットする
	if (depctl.b.epena && depctl.b.usbactep) {
		if (!gintmsk.b.goutnakeff) {
			gintmsk.b.goutnakeff = 1;
			DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintmsk, gintmsk.d32);
		}
		if (!gintsts.b.goutnakeff) {
			dctl.b.sgoutnak = 1;
		}
		DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32);
		isSet = true;
	}
	
	return isSet;
}

/**
*	@brief 
**/ 
static void USB_ClearGlobalOUTNak(void)
{
	dctl_data_t dctl = {.d32 = 0 };
	gintsts_data_t gintsts = {.d32 = 0 };
	gintmsk_data_t gintmsk = {.d32 = 0 };
	
	dctl.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dctl);
	gintsts.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gintsts);
	gintmsk.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gintmsk);
	if (gintmsk.b.goutnakeff) {
		gintmsk.b.goutnakeff = 0;
		DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintmsk, gintmsk.d32);
	}
	if (gintsts.b.goutnakeff) {
		dctl.b.cgoutnak = 1;
		dctl.b.sgoutnak = 0;
	}
	DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32);
}

/**
*	@brief 7.6.2 Setting Global Non-Periodic IN Endpoint NAK
**/ 
static bool_t USB_SetGlobalNPINNak(uint8_t ucEP)
{
	dctl_data_t dctl = {.d32 = 0 };
	gintsts_data_t gintsts = {.d32 = 0 };
	gintmsk_data_t gintmsk = {.d32 = 0 };
	depctl_data_t depctl = {.d32 = 0 };
	
	bool_t isSet = false;
	
	dctl.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dctl);
	gintsts.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gintsts);
	gintmsk.d32 = DWC_READ_REG32(&pstUSBDev->pstGRegs->gintmsk);
	depctl.d32  = DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->diepctl);
	// 現在転送中の場合にNakをセットする
	if (depctl.b.epena && depctl.b.usbactep) {
		pstUSBDev->ginnak_factor.bit.stall_abort = 1;
		if (!gintmsk.b.ginnakeff) {
			gintmsk.b.ginnakeff = 1;
			DWC_WRITE_REG32(&pstUSBDev->pstGRegs->gintmsk, gintmsk.d32);
		}
		if (!gintsts.b.ginnakeff) {
			dctl.b.sgnpinnak = 1;
		}
		DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32);
		isSet = true;
	}
	
	return isSet;
}

/**
*	@brief  Function to abort endpoint transaction.
**/ 
static USB_err_t
USB_EPAbort(uint8_t ucEP, uint8_t Dir)
{
	USB_err_t iErr = USB_OK;
	
	// Wait for the completion of ginnak in use by other factors
	while(pstUSBDev->ginnak_factor.bit.epmis || pstUSBDev->ginnak_factor.bit.timeout);
	
	switch(pstUSBDev->stEPs[ucEP].eState[Dir]) {
	case USB_EP_ABORTING:
	case USB_EP_ABORTED:
	case USB_EP_STALLING:
	case USB_EP_HALTED:
		log_warning("%s Already abort or halt (%d, %d) eState %d", __func__, ucEP, Dir, pstUSBDev->stEPs[ucEP].eState[Dir]);
		return USB_OK;
	case USB_EP_RUNNING:
		break;// Try abort
	default:
		log_error("%s Invalid (%d, %d) eState %d", __func__, ucEP, Dir, pstUSBDev->stEPs[ucEP].eState[Dir]);
		return USB_EINVAL;
	}
	
	pstUSBDev->stEPs[ucEP].eState[Dir] = USB_EP_ABORTING;
	
	if(Dir == 1){ // IN転送
		bool_t isSet;
		if (USB_EP_IS_PERIODIC(pstUSBDev->stEPs[ucEP].eptype[Dir])) {
			isSet = USB_SetINEPNak(ucEP);
			if (isSet) {
				if (USB_OK != USB_waitEPcomplete(ucEP, Dir)) {
					USB_ClearINEPNak(ucEP);
					log_warning("%s USB_waitEPcomplete(%d, %d) timeout", __func__, ucEP, Dir);
					iErr = USB_ETIMEOUT;
				}
			} else {
				// すでに転送中ではなかった
			}
			
		} else {
			isSet = USB_SetGlobalNPINNak(ucEP);
			if (isSet) {
				if (USB_OK != USB_waitEPcomplete(ucEP, Dir)) {
					log_warning("%s USB_waitEPcomplete(%d, %d) timeout", __func__, ucEP, Dir);
					iErr = USB_ETIMEOUT;
				}
			} else {
				// すでに転送中ではなかった
			}
		}
		
	} else { // OUT転送
		if (ucEP == 0) {
			/// EP0 OUTはアボートできません。
			/// DOEPCTL0.EPDis ビットがReadOnlyなので、Disableできません。
			/// DOEPCTL0.EPEna ビットはR_WS_SC属性のためゼロクリアできません。
			/// @sa 5.3.5.21 Device Control OUT Endpoint 0 Control Register (DOEPCTL0)
			
		} else if (USB_SetGlobalOUTNak(ucEP)) {
			if (USB_OK != USB_waitEPcomplete(ucEP, Dir)) {
				USB_ClearGlobalOUTNak();
				iErr = USB_ETIMEOUT;
			}
		} else {
			// すでに転送中ではなかった
		}
	}
	
	// 転送を削除
	pstUSBDev->stEPs[ucEP].pstXfer[Dir] = NULL;
	
	pstUSBDev->stEPs[ucEP].eState[Dir] = USB_EP_RUNNING;
	
	return iErr; 
}

/**
*	@brief  Function to set stall endpoint transaction.
**/ 
static USB_err_t
USB_EPSetStall(uint8_t ucEP, uint8_t Dir)
{
	USB_err_t iErr = USB_OK;
	
	depctl_data_t depctl;
	
	// Wait for the completion of ginnak in use by other factors
	while(pstUSBDev->ginnak_factor.bit.epmis || pstUSBDev->ginnak_factor.bit.timeout);
	
	switch(pstUSBDev->stEPs[ucEP].eState[Dir]) {
	case USB_EP_ABORTING:
	case USB_EP_ABORTED:
	case USB_EP_STALLING:
	case USB_EP_HALTED:
		log_warning("%s Already abort or halt (%d, %d) eState %d", __func__, ucEP, Dir, pstUSBDev->stEPs[ucEP].eState[Dir]);
		return USB_OK;
	case USB_EP_RUNNING:
		break; // Try halt
	default:
		log_error("%s Invalid (%d, %d) eState %d", __func__, ucEP, Dir, pstUSBDev->stEPs[ucEP].eState[Dir]);
		return USB_EINVAL;
	}
	
	pstUSBDev->stEPs[ucEP].eState[Dir] = USB_EP_STALLING;
	
	if(Dir == 1){ // IN転送
		/* 
		7.6.8 Stalling Non-Isochronous IN Endpoints
		  7.6.4 Disabling Non-Periodic IN Endpoints in Shared FIFO Operation
		    7.6.2 Setting Global Non-Periodic IN Endpoint NAK
		  7.6.5 Disabling Periodic IN Endpoints in Shared FIFO Operation
		*/
		bool_t isSet;
		if (USB_EP_IS_PERIODIC(pstUSBDev->stEPs[ucEP].eptype[Dir])) {
			isSet = USB_SetINEPNak(ucEP);
			if (isSet) {
				if (USB_OK != USB_waitEPcomplete(ucEP, Dir)) {
					USB_ClearINEPNak(ucEP);
					log_warning("%s USB_waitEPcomplete(%d, %d) timeout", __func__, ucEP, Dir);
					iErr = USB_ETIMEOUT;
				}
			}
			
		} else {
			isSet = USB_SetGlobalNPINNak(ucEP);
			if (isSet) {
				if (USB_OK != USB_waitEPcomplete(ucEP, Dir)) {
					log_warning("%s USB_waitEPcomplete(%d, %d) timeout", __func__, ucEP, Dir);
					iErr = USB_ETIMEOUT;
				}
			}
		}
		
		if (!isSet) {
			// 転送中ではなかったので単純にstallビットを立てる
			depctl.d32 = 0;
			depctl.b.stall = 1;
			DWC_MODIFY_REG32(&pstUSBDev->stEPs[ucEP].pstInEP->diepctl, 0, depctl.d32);
		}
		
	} else { // OUT転送
		bool_t bStallSimple = false;
		
		if (ucEP == 0) {
			/// EP0 OUTは、DOEPCTL0.EPDis bitがReadOnlyで常に1のため、EP Disableできません。
			/// @sa 5.3.5.21 Device Control OUT Endpoint 0 Control Register (DOEPCTL0)
			bStallSimple = true;
			
		} else if (USB_SetGlobalOUTNak(ucEP)) {
			if (USB_OK != USB_waitEPcomplete(ucEP, Dir)) {
				USB_ClearGlobalOUTNak();
				iErr = USB_ETIMEOUT;
			}
		} else {
			// 転送中ではなかった
			bStallSimple = true;
		}
		
		if (bStallSimple) {
			depctl.d32 = 0;
			depctl.b.stall = 1;
			DWC_MODIFY_REG32(&pstUSBDev->stEPs[ucEP].pstOutEP->doepctl, 0, depctl.d32);
		}
	}
	
	pstUSBDev->stEPs[ucEP].eState[Dir] = USB_EP_HALTED;
	return iErr;
}

/**
*	@brief  Function to clear stall endpoint transaction.
*	@note   dwc_otg_cil.c / dwc_otg_ep_clear_stall
**/
static USB_err_t
USB_EPClearStall(uint8_t ucEP, uint8_t Dir)
{
	depctl_data_t depctl;
	volatile uint32_t *depctl_addr;
	
	if (Dir == 1) {
		depctl_addr = &(pstUSBDev->stEPs[ucEP].pstInEP->diepctl);
	} else {
		depctl_addr = &(pstUSBDev->stEPs[ucEP].pstOutEP->doepctl);
	}

	depctl.d32 = DWC_READ_REG32(depctl_addr);

	/* clear the stall bits */
	depctl.b.stall = 0;
	
	switch(depctl.b.eptype)
	{
	case USB_EP_CTRL:
		// Control EP IN/OUTのstallビットは、SETUPパケットの受信によってクリアされます。
		return USB_EINVAL;
	case USB_EP_ISOC:
		break;
	case USB_EP_BULK:
	case USB_EP_INTR:
		// BulkとInterruptの場合はトグルをDATA0にする
		depctl.b.setd0pid = 1;
	}

	DWC_WRITE_REG32(depctl_addr, depctl.d32);
	
	pstUSBDev->stEPs[ucEP].eState[Dir] = USB_EP_RUNNING;
	
	return USB_OK;
}


/* ***************************************************************************** */
/*                      Enumeration state machine handling functions             */
/* ***************************************************************************** */

/**
*	@brief  Function is used to populate recevied Setup request to control transfer structure.
*	@note   dwc_otg_pcd_intr.c / pcd_setup
*/
static USB_err_t
USB_EP0SetupXferDone(USB_ctrlXfer_t *pstCXfer)
{
	const uint8_t ucEP = 0;
	const uint8_t Dir = 0;
	
	if ((USB_EP_RUNNING == pstUSBDev->stEPs[ucEP].eState[Dir]) || 
		(USB_EP_HALTED  == pstUSBDev->stEPs[ucEP].eState[Dir]) ) {
		// SETUPパケットを受信すると、EP0 OUT/IN のstallビットがクリアされます。
		pstUSBDev->stEPs[ucEP].eState[0] = USB_EP_RUNNING;
		pstUSBDev->stEPs[ucEP].eState[1] = USB_EP_RUNNING;
	} else {
		log_error("%s Invalid eState %d", __func__, pstUSBDev->stEPs[ucEP].eState[Dir]);
		return USB_EIO;
	}
	
	// SETUPパケット8byteがバッファのどの位置に格納されたか判断して取り出す
	const uint16_t* buf;
	doepint_data_t doepint;
	doepint.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstOutEP->doepint);
	if (doepint.b.back2backsetup) {
		CLEAR_OUT_EP_INTR(0, back2backsetup);
		buf = (uint16_t*)(DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstOutEP->doepdma) - 8);
		log_error("%s doepint setup & back2backsetup 0x%08X", __func__, doepint.d32);
	} else {
		deptsiz0_data_t doeptsize0;
		doeptsize0.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[ucEP].pstOutEP->doeptsiz);
		if (doeptsize0.b.supcnt < 2) {
			log_error("\n\n-----------	 CANNOT handle > 1 setup packet in DMA mode\n\n");
		}
		if (doeptsize0.b.supcnt == 3 && pstCXfer->stp_rollover == 0) {
			log_error(" !!! Setup packet count was not updated by the core");
			stError.status = USBD_TZ10xx_ERROR_EP_SETUP_LOST;
			stError.param  = USB_ep_addr(ucEP,Dir);
			USBD_SignalEndpointEvent(stError.param, (ARM_USBD_EP_EVENT)-1);
			return USB_EIO;
		}
		int offset = (4 - (2 * (doeptsize0.b.supcnt - pstCXfer->stp_rollover))) * 4;
		buf = (uint16_t*)( ((uint8_t *)EP0outSetupBuf) + offset );
		if (doeptsize0.b.supcnt < 2) {
			log_error("%s doeptsize0.b.supcnt %d", __func__, doeptsize0.b.supcnt);
		}
		log_debug("%s SETUP received. %04X %04X %04X %04X ", __func__, buf[0], buf[1], buf[2], buf[3]);
	}
	
	USB_devReq_t *pstReq = &pstCXfer->stReq;
	pstReq->bmRequestType = (USB_byte_t)(buf[0]      & 0xff); 
	pstReq->bRequest      = (USB_byte_t)(buf[0] >> 8 & 0xff);
	USB_SETW(pstReq->wValue,  buf[1]);
	USB_SETW(pstReq->wIndex,  buf[2]);
	USB_SETW(pstReq->wLength, buf[3]); 
	
	UsbEventFlag.b.request = 1;
	
	if (USB_DR_GETDIR(pstReq->bmRequestType) == USB_DR_DIR_D2H) {
		pstUSBDev->EP0State = EP0_IN_DATA_PHASE;
	} else {
		pstUSBDev->EP0State = EP0_OUT_DATA_PHASE;
	}
	
	pstCXfer->DataPhase.ulTotalLen = 0;
	pstCXfer->DataPhase.ulDataLen = USB_GETW(pstReq->wLength);
	if (pstCXfer->DataPhase.ulDataLen == 0) {
		pstUSBDev->EP0State = EP0_IN_STATUS_PHASE;
	}
	pstUSBDev->EP0State_PhaseEntered = true;
	pstUSBDev->EP0State_StatusPhaseHandled = false;
	
	USBD_SignalEndpointEvent(USB_ep_addr(0,0), ARM_USBD_EP_EVENT_SETUP);
	
	return USB_OK;
}

/**
*	@brief  Function used to handle ep transfer
*/
static void USB_handle_ep(uint32_t epnum, uint8_t Dir)
{
	deptsiz_data_t deptsiz;
	int32_t byte_count;
	USB_xfer_t* pTrf; 
	bool complete = false;
	
//	log_debug("USB_handle_ep epnum=%d Dir=%d", epnum, Dir);
	
	if (USB_EP_RUNNING != pstUSBDev->stEPs[epnum].eState[Dir]) {
		log_warning("%s Invalid (%d, %d) eState %d", __func__, epnum, Dir, pstUSBDev->stEPs[epnum].eState[Dir]);
		UsbEventFlag.d32 |= USB_EVENT_EP(epnum,Dir);
		return;
	}
	
	pTrf = pstUSBDev->stEPs[epnum].pstXfer[Dir];
	
	if(Dir == 1){
		deptsiz.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[epnum].pstInEP->dieptsiz);
		if ((deptsiz.b.xfersize == 0) && (deptsiz.b.pktcnt == 0)) {
			byte_count = pTrf->usTrfSz - deptsiz.b.xfersize;
			pTrf->usActLen += byte_count;
			pstUSBDev->stEPs[epnum].ActLen[Dir] = pTrf->usActLen;
			if (pTrf->usActLen < pTrf->usLen){
				USB_StartXfer(USB_ep_addr(epnum,Dir), pTrf);
			} else {
				complete = true;
			}
		} else {
			log_warning("Incomplete transfer (EP%dIN [siz=%d pkt=%d])",
				epnum, deptsiz.b.xfersize, deptsiz.b.pktcnt);
		}
	} else{
		deptsiz.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[epnum].pstOutEP->doeptsiz);
		byte_count = pTrf->usTrfSz - deptsiz.b.xfersize;
		pTrf->usActLen += byte_count;
		pstUSBDev->stEPs[epnum].ActLen[Dir] = pTrf->usActLen;
		if (pTrf->usActLen < pTrf->usLen){
			USB_StartXfer(USB_ep_addr(epnum,Dir), pTrf);
		} else {
			complete = true;
		}
	}
	
	if (complete) {
		pstUSBDev->stEPs[epnum].pstXfer[Dir] = NULL;
		UsbEventFlag.d32 |= USB_EVENT_EP(epnum,Dir);
		USBD_SignalEndpointEvent(USB_ep_addr(epnum,Dir), Dir ? ARM_USBD_EP_EVENT_IN : ARM_USBD_EP_EVENT_OUT);
	}
}

/**
*	@brief  Function used to handle ep0 transfer
*	@note   dwc_otg_pcd_intr.c / handle_ep0
*	@note   see USBD_EndpointReadStart() and USBD_EndpointWriteStart()
*/
static USB_err_t handle_ep0(void)
{
	USB_err_t iErr = USB_EIO;
	USB_ctrlXfer_t *pstCXfer = &pstUSBDev->stCtrlXfer;
	
	switch (pstUSBDev->EP0State) {
	case EP0_DISCONNECT:
		break;

	case EP0_IDLE:
		UsbEventFlag.d32 &= ~(USB_EVENT_EP(0,0) | USB_EVENT_EP(0,1));
		iErr = USB_EP0SetupXferDone(pstCXfer);
		break;

	case EP0_IN_DATA_PHASE:
		{
			USB_xfer_t* pTrf = &pstCXfer->stDataXfer;
			deptsiz0_data_t deptsiz;
			uint32_t byte_count;
			
			deptsiz.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstInEP->dieptsiz);
			byte_count = pTrf->usTrfSz - deptsiz.b.xfersize;
			pTrf->usActLen += byte_count;
			pstUSBDev->stEPs[0].ActLen[1] = pTrf->usActLen;
			iErr = USB_EP0DataXferDone(pstCXfer,1);
			pstCXfer->DataPhase.ulTotalLen += byte_count;
			if (pstCXfer->DataPhase.ulTotalLen >= pstCXfer->DataPhase.ulDataLen) {
				// Data phase complete
			}
			if (pstUSBDev->EP0State_StatusPhaseHandled) {
				// Data phase complete, Status phase complete
				/* Prepare for more SETUP Packets */
				log_info("%s StatusPhaseHandled", __func__);
				USB_ep0_out_start();
			}
			
			UsbEventFlag.d32 |= USB_EVENT_EP(0,1);
		}
		break;
		
	case EP0_OUT_DATA_PHASE:
		{
			USB_xfer_t* pTrf = &pstCXfer->stDataXfer;
			deptsiz0_data_t deptsiz;
			uint32_t byte_count;
			
			deptsiz.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doeptsiz);
			byte_count = pTrf->usTrfSz - deptsiz.b.xfersize;
			pTrf->usActLen += byte_count;
			pstUSBDev->stEPs[0].ActLen[0] = pTrf->usActLen;
			iErr = USB_EP0DataXferDone(pstCXfer, 0);
			pstCXfer->DataPhase.ulTotalLen += byte_count;
			if (pstCXfer->DataPhase.ulTotalLen >= pstCXfer->DataPhase.ulDataLen) {
				// Data phase complete
				log_debug("Enable out ep before in status phase");
				iErr = USB_ep0_out_start();
			}
			UsbEventFlag.d32 |= USB_EVENT_EP(0,0);
		}
		break;

	case EP0_IN_STATUS_PHASE:
	case EP0_OUT_STATUS_PHASE:
		iErr = USB_EP0StatusXferDone(pstCXfer);
		pstUSBDev->EP0State = EP0_IDLE;
		pstUSBDev->EP0State_StatusPhaseHandled = false;
		/* Prepare for more SETUP Packets */
		iErr = USB_ep0_out_start();
		break;

	case EP0_STALL:
		log_warning("%s EP0 STALLed", __func__);
		break;
	}
	
	return iErr;
}


/**
*	@brief  Function used to handle control transfer
*	@note   dwc_otg_pcd_intr.c / dwc_otg_pcd_handle_out_ep_intr
*/
static void USB_intrOutEP0(doepint_data_t doepint)
{
	const uint8_t epnum = 0;
	const uint8_t dir = 0;

	if (doepint.b.xfercompl) {
		CLEAR_OUT_EP_INTR(0,xfercompl);
		
		{
			doepint_data_t doepint_temp;
			deptsiz0_data_t doeptsize0;
			doepint_temp.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepint);
			doeptsize0.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doeptsiz);
			
			USB_xfer_t* pTrf;
			pTrf = &pstUSBDev->stCtrlXfer.stDataXfer;
			if (((pTrf->usActLen == pTrf->usLen || doeptsize0.b.xfersize == 64) && pstUSBDev->EP0State == EP0_OUT_DATA_PHASE && doepint.b.stsphsercvd) ||
				(doeptsize0.b.xfersize == 24 && pstUSBDev->EP0State == EP0_IN_STATUS_PHASE) ) {
				CLEAR_OUT_EP_INTR(epnum, xfercompl);
				log_warning("WA for xfercompl along with stsphs");
				doepint.b.xfercompl = 0;
				// restart ep0out setup
				USB_ep0_out_start();
				goto exit_xfercompl;
			}
			
			if (pstUSBDev->EP0State == EP0_IDLE) {
				if (doepint_temp.b.sr) {
					CLEAR_OUT_EP_INTR(epnum, sr);	
					/* Delay is needed for core to update setup 
					 * packet count from 3 to 2 after receiving 
					 * setup packet*/
					DWC_DELAY_US(100);
					doepint.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepint);
					if (doeptsize0.b.supcnt == 3) {
						log_warning("Rolling over!!!!!!!");
						pstUSBDev->stCtrlXfer.stp_rollover = 1;
					}
					if (doepint.b.setup) {
						// Case C?, xfercompl=1, setup=1, stsphsercvd=0
retry:
						/* Already started data phase, clear setup */
						CLEAR_OUT_EP_INTR(epnum, setup);
						doepint.b.setup = 0;
						log_debug("handle_ep0 %d", __LINE__);
						handle_ep0();
						pstUSBDev->stCtrlXfer.stp_rollover = 0;
						/* Prepare for more setup packets */
						if (pstUSBDev->EP0State == EP0_IN_STATUS_PHASE || 
							pstUSBDev->EP0State == EP0_IN_DATA_PHASE) {
							USB_ep0_out_start();
							if (!pstUSBDev->otg_ver) {
								depctl_data_t depctl = {.d32 = 0};
								depctl.b.cnak = 1;
								DWC_MODIFY_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepctl, 0, depctl.d32);
							}
						}
						goto exit_xfercompl;
					} else {
						// Case A?, xfercompl=1, setup=0, stsphsercvd=0
						// Case E?, xfercompl=1, setup=0, stsphsercvd=1
						/* Prepare for more setup packets */
						log_warning("EP0_IDLE SR=1 setup=0 new setup comes");
						doepint.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepint);
						if(doepint.b.setup) {
							goto retry;
						}
						// restart ep0out setup
						USB_ep0_out_start();
					}
				}
					
			} else {
				uint8_t in_phase_ep_result = 0;
				int count;
				
				if (pstUSBDev->EP0State == EP0_IN_DATA_PHASE ||
					pstUSBDev->EP0State == EP0_IN_STATUS_PHASE) {
					diepint_data_t diepint0;
					depctl_data_t diepctl0;
					diepint0.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstInEP->diepint);
					diepctl0.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstInEP->diepctl);
					
					/* EP0Stateがep0in転送されている可能性のある状態 */
					/* A state that might EP0State is being transferred ep0in. */
					if (diepint0.b.xfercompl) {
						in_phase_ep_result |= 0x1;
						DWC_WRITE_REG32(&pstUSBDev->stEPs[0].pstInEP->diepint, diepint0.d32);
					}
					if (diepctl0.b.epena) {
						in_phase_ep_result |= 0x2;
						/* Abort ep0in transfer  */
						diepint_data_t diepint = {.d32 = 0};
						diepctl0.b.snak = 1;
						DWC_WRITE_REG32(&pstUSBDev->stEPs[0].pstInEP->diepctl, diepctl0.d32);
						count = 0;
						do {
							if (++count > 1000) {
								log_warning("%s() %d HANG! DIEPINT0=0x%08x", __func__, __LINE__, DWC_READ_REG32(&pstUSBDev->stEPs[0].pstInEP->diepint));
								stError.status = USBD_TZ10xx_ERROR_CORE_HANG;
								stError.param  = __LINE__;
								USBD_SignalDeviceEvent((ARM_USBD_EVENT)-1);
								break;
							}
							DWC_DELAY_US(10);
							diepint.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstInEP->diepint);
						} while (!diepint.b.inepnakeff); 
						diepint.b.inepnakeff = 1;
						DWC_WRITE_REG32(&pstUSBDev->stEPs[0].pstInEP->diepint, diepint.d32);
						diepctl0.d32 = 0;
						diepctl0.b.epdis = 1;
						DWC_WRITE_REG32(&pstUSBDev->stEPs[0].pstInEP->diepctl, diepctl0.d32);
						count = 0;
						do {
							if (++count > 1000) {
								log_warning("%s() %d HANG! DIEPINT0=0x%08x", __func__, __LINE__, DWC_READ_REG32(&pstUSBDev->stEPs[0].pstInEP->diepint));
								stError.status = USBD_TZ10xx_ERROR_CORE_HANG;
								stError.param  = __LINE__;
								USBD_SignalDeviceEvent((ARM_USBD_EVENT)-1);
								break;
							}
							DWC_DELAY_US(10);
							diepint.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstInEP->diepint);
						} while (!diepint.b.epdisabled); 
						diepint.b.epdisabled = 1;
						DWC_WRITE_REG32(&pstUSBDev->stEPs[0].pstInEP->diepint,diepint.d32);
						
						// @note このepena=1の場合にabortを完了した以降にDIEPTSIZ0を読み取ると、xfercompl割り込みで所望のLengthを転送完了した場合と同じ値が読み取れます。
						
						/* Need to clear the shared TxFIFO */
						{
							pstUSBDev->ginnak_factor.bit.ep0_premature_status = 1;
							
							/* Enable the Global IN NAK Effective Interrupt */
							gintmsk_data_t intr_mask;
							intr_mask.d32 = 0;
							intr_mask.b.ginnakeff = 1;
							DWC_MODIFY_REG32(&pstUSBDev->pstGRegs->gintmsk, 0, intr_mask.d32);
							
							/* Set Global IN NAK */
							dctl_data_t dctl;
							dctl.d32 = 0;
							dctl.b.sgnpinnak = 1;
							DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32, dctl.d32);
						}
					}
				}
				doepint_data_t doepint_temp;
				doepint_temp.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepint);
				if (doepint_temp.b.sr) {
					// SETUPパケットだった
					CLEAR_OUT_EP_INTR(epnum, sr);
					/* EP0Stateがsetup受信待ち(EP0_IDLE)でないのにsetupを受信した場合 */
					pstUSBDev->EP0State = EP0_IDLE;
					if (doepint.b.setup) {
						log_warning("EP0_IDLE SR=1 setup=1");
						/* Data phase started, clear setup */
						CLEAR_OUT_EP_INTR(epnum, setup);
						doepint.b.setup = 0;
						log_info("handle_ep0 %d", __LINE__);
						handle_ep0();
						/* Prepare for setup packets if ep0in was enabled*/
						if (pstUSBDev->EP0State == EP0_IN_STATUS_PHASE) {
							USB_ep0_out_start();
							if (!pstUSBDev->otg_ver) {
								depctl_data_t depctl = {.d32 = 0};
								depctl.b.cnak = 1;
								DWC_MODIFY_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepctl, 0, depctl.d32);
							}
						}
						goto exit_xfercompl;
					} else {
						/* Prepare for more setup packets */
						log_warning("EP0_IDLE SR=1 setup=0 new setup comes 2");
						// restart ep0out setup
						USB_ep0_out_start();
					}
					
				} else {
					// OUTパケットだった
					if (pstUSBDev->EP0State == EP0_IN_DATA_PHASE) {
						// OUT Status phase already xfercompleted and handled.
						pstUSBDev->EP0State_StatusPhaseHandled = true;
						log_warning("EP0_IN_DATA_PHASE early status out comes. 0x%02X", in_phase_ep_result);
						
					} else if (pstUSBDev->EP0State == EP0_IN_STATUS_PHASE) {
						log_warning("EP0_IN_STATUS_PHASE out packet comes. 0x%02X", in_phase_ep_result);
					}
				}
			}
		}
		
		if (pstUSBDev->EP0State != EP0_IDLE) {
			// EP0_OUT_DATA_PHASE or EP0_OUT_STATUS_PHASE or 
			// EP0_IN_DATA_PHASE or EP0_IN_STATUS_PHASE
			log_debug("handle_ep0 %d", __LINE__);
			handle_ep0();
		}
		
exit_xfercompl:
		log_debug("after DOEPINT=%x doepint=%x\n", USB_read_dev_out_ep_intr(epnum), doepint.d32);
	}
	
	// Case D?, xfercompl=0, setup=0, stsphsercvd=1
	// Case E?, xfercompl=1, setup=0, stsphsercvd=1
	/* Host has switched to Control Write Status phase. */
	if (doepint.b.stsphsercvd) {
		CLEAR_OUT_EP_INTR(epnum, stsphsercvd);
		/*
		There is a possibility that in a deadlock state as follows: 
		1. Device(middleware) is waiting for the OUT DATA
		2. Host are waiting for IN STATUS
		 */
		log_debug("stsphsercvd %d", __LINE__);
		stError.status = USBD_TZ10xx_ERROR_EP_STSPHSERCVD;
		stError.param  = USB_ep_addr(epnum,dir);
		USBD_SignalEndpointEvent(stError.param, (ARM_USBD_EP_EVENT)-1);
	}
	
	// Case B?, xfercompl=0, setup=1, stsphsercvd=0
	/* Setup Phase Done (contorl EPs) */
	if (doepint.b.setup) {
		CLEAR_OUT_EP_INTR(epnum, setup);
		if (pstUSBDev->EP0State == EP0_IDLE) {
			deptsiz0_data_t doeptsize0;
			doeptsize0.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doeptsiz);
			if (doeptsize0.b.supcnt == 3) {
				log_warning("Rolling over!!!!!!! 2");
				pstUSBDev->stCtrlXfer.stp_rollover = 1;
			}
			log_debug("handle_ep0 %d", __LINE__);
			handle_ep0();
			pstUSBDev->stCtrlXfer.stp_rollover = 0;
			
		} else {
			log_debug("handle_ep0 %d", __LINE__);
			handle_ep0();
		}
	}
}

/**
*	@brief  Function to configure device state according to setup request.
**/ 
static USB_err_t USB_EP0StatusXferDone(USB_ctrlXfer_t *pstCXfer)
{
	const uint8_t ucEP = 0;
	uint8_t Dir;
	
	switch(pstUSBDev->EP0State)
	{
	case EP0_IN_STATUS_PHASE:
		Dir = 1;
		break;
	case EP0_OUT_STATUS_PHASE:
		Dir = 0;
		break;
	default:
		log_error("%s Invalid EP0State %d", __func__, pstUSBDev->EP0State);
		return USB_EIO;
	}
	
	USB_xfer_t *pstXfer; 
	pstXfer = pstUSBDev->stEPs[ucEP].pstXfer[Dir];
	if(pstXfer) {
		if (USB_EP_RUNNING != pstUSBDev->stEPs[ucEP].eState[Dir]) {
			pstXfer->iStatus = USB_EABORT;
			log_warning("%s eState %d", __func__, pstUSBDev->stEPs[ucEP].eState[Dir]);
		} else {
			pstXfer->iStatus = USB_OK;
		}
		/* 転送を削除 */
		pstUSBDev->stEPs[ucEP].pstXfer[Dir] = NULL;
		if (Dir) {
			USBD_SignalEndpointEvent(USB_ep_addr(0,1), ARM_USBD_EP_EVENT_IN); // STATUS_IN
		} else {
			USBD_SignalEndpointEvent(USB_ep_addr(0,0), ARM_USBD_EP_EVENT_OUT); // STATUS_OUT
		}
		
		return USB_OK;
	} else {
		log_error("%d pstXfer NULL", __func__);
	}
	
	return USB_EIO;
}


#include "TZ10xx.h" // for pmulv

/**
*	@brief Assert the reset signal and stop the clock to USB hardware module.
**/
static void USB_ModuleShutdown(void)
{
	/* Assert reset to USB Controller */
	pmulv->SRST_ON_PU_b.SRST_asyncrst_h2hdnu_hrstn = 1;
	pmulv->SRST_ON_PU_b.SRST_asyncrst_h2hupu_hrstn = 1;
	pmulv->SRST_ON_PU_b.SRST_asyncrst_usb2fs_hrstn = 1;
	pmulv->SRST_ON_PU_b.SRST_asyncrst_usb2fs_usbrstn = 1;

	/* Stop clock supply to USB Controller */
	pmulv->CG_ON_PU_b.CG_mpierclk_h2hdnu_hclk = 1;
	pmulv->CG_ON_PU_b.CG_mpierclk_h2hupu_hclk = 1;
	pmulv->CG_ON_PU_b.CG_usbbclk_usb2fs_hclk = 1;
	pmulv->CG_ON_PU_b.CG_usbiclk_usb2fs_usbclk = 1;
	pmulv->CG_ON_POWERDOMAIN_b.CG_PU = 1;

	/* Enable Isolation */
	pmulv->ISO_PU_b.INISOEN_PU = 1;
	pmulv->ISO_PU_b.OUTISOEN_PU = 1;

	/* Turn off PU */
	pmulv->PSW_IO_USB_b.PSW_IO_USB_VDDCS = 0;
	pmulv->PSW_IO_USB_b.PSW_IO_USB_VDDCW = 0;
	pmulv->PSW_PU_b.PSW_PU_VDDCS = 0;
	pmulv->PSW_PU_b.PSW_PU_VDDCW = 0;
	
	pstUSBDev->isPhyInitDone = false;
}

/**
*	@brief Deassert the reset signal and start the clock to USB hardware module.
**/
static void USB_ModuleStart(void)
{
	/* Turn on PU */
	pmulv->PSW_PU_b.PSW_PU_VDDCW = 1;
	pmulv->PSW_PU_b.PSW_PU_VDDCS = 1;
	pmulv->PSW_IO_USB_b.PSW_IO_USB_VDDCW = 1;
	pmulv->PSW_IO_USB_b.PSW_IO_USB_VDDCS = 1;
	DWC_DELAY_US(1); // The wait 250ns to stabilize

	/* Disable Isolation */
	pmulv->ISO_PU_b.INISOEN_PU = 0;
	pmulv->ISO_PU_b.OUTISOEN_PU = 0;

	/* Start clock supply to USB Controller */
	pmulv->CG_OFF_POWERDOMAIN_b.CG_PU = 1;
	pmulv->CG_OFF_PU_b.CG_mpierclk_h2hdnu_hclk = 1;
	pmulv->CG_OFF_PU_b.CG_mpierclk_h2hupu_hclk = 1;
	pmulv->CG_OFF_PU_b.CG_usbbclk_usb2fs_hclk = 1;
	pmulv->CG_OFF_PU_b.CG_usbiclk_usb2fs_usbclk = 1;

	/* Deassert reset to USB Controller */
	pmulv->SRST_OFF_PU_b.SRST_asyncrst_usb2fs_usbrstn = 1;
	DWC_DELAY_US(3); /* wait 12 cycles, AHB 48MHzの周期は208ns程度, * 12 = 2496ns */
	pmulv->SRST_OFF_PU_b.SRST_asyncrst_h2hdnu_hrstn = 1;
	pmulv->SRST_OFF_PU_b.SRST_asyncrst_h2hupu_hrstn = 1;
	pmulv->SRST_OFF_PU_b.SRST_asyncrst_usb2fs_hrstn = 1;
	DWC_DELAY_US(2); /* wait 6 cycles, AHB 48MHzの周期は208ns程度, * 6 = 1248ns */
}

/**
*	@brief Disable D+/D- pin pull up
**/
static void USB_SoftDisconnect(uint32_t msec)
{
	dctl_data_t dctl;
	dctl.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dctl);
	dctl.b.sftdiscon = 1;
	DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32);
	if (msec) {
		DWC_DELAY_US(msec * 1000);
	}
}

/**
*	@brief Enable D+/D- pin pull up
**/
static void USB_SoftConnect(void)
{
	dctl_data_t dctl;
	dctl.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dctl);
	dctl.b.sftdiscon = 0;
	DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32);
}


/**
*	@brief  This function configure endpoint transfer.
**/ 
static USB_err_t
USB_xferEP(uint8_t ep_addr, uint8_t *buf, uint32_t len, USB_xfer_t *pstXfer)
{
	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
//	uint8_t ep_type = pstUSBDev->stEPs[ep_num].eptype[ep_dir];
	
	USB_err_t iErr = USB_OK; 
	USB_state_t current = USB_STATE_GETSTATE(pstUSBDev->iDevState);
	
	assert(NULL != pstXfer);
	
	if ((current != USB_STATE_CONFIGURED) &&
		(current != USB_STATE_ADDRESSED) ) {
		iErr = USB_EIO;
		log_warning("%s EP 0x%02X Invalid Device State %d", __func__, ep_addr, current);
		goto end;
	}
	if (USB_EP_RUNNING != pstUSBDev->stEPs[ep_num].eState[ep_dir]) {
		iErr = USB_EBUSY;
		log_warning("%s EP 0x%02X not Running ", __func__, ep_addr);
		goto end; 
	}
	if(pstUSBDev->stEPs[ep_num].pstXfer[ep_dir]) {
		iErr = USB_EBUSY;
		log_warning("%s EP 0x%02X transfer busy", __func__, ep_addr);
		goto end; 
	}
	// Setup transfer structure
	pstXfer->pvBuffer = (void *)buf;
	pstXfer->usLen    = len;
	pstXfer->usActLen = 0;
	pstXfer->usTrfSz  = 0;
	pstXfer->iStatus  = USB_EIO;
	pstUSBDev->stEPs[ep_num].ActLen[ep_dir] = 0;
	pstUSBDev->stEPs[ep_num].pstXfer[ep_dir] = pstXfer; 
	UsbEventFlag.d32 &= ~(USB_EVENT_EP(ep_num,ep_dir));
	USB_StartXfer(ep_addr, pstXfer);
end:
	return iErr; 
}

/**
*	@brief  This function configure endpoint0 transfer.
**/ 
static USB_err_t
USB_xferEP0(uint8_t kind, USB_ctrlXfer_t *pstCXfer)
{
	assert(kind <= 3);
	
	const uint8_t ucEP = 0; 
	const uint8_t Dir = (1 == kind) ? 1 : 0;
	USB_err_t iErr = USB_OK; 
	USB_state_t current = USB_STATE_GETSTATE(pstUSBDev->iDevState);
	
	if (current < USB_STATE_DEFAULT) {
		iErr = USB_EIO;
		goto end;
	}
	
	if (2 == kind) {
		/* Check EP state */
		// OUT SETUPの場合はHALTEDを許容する。
		if ((USB_EP_RUNNING != pstUSBDev->stEPs[ucEP].eState[Dir]) &&
			(USB_EP_HALTED != pstUSBDev->stEPs[ucEP].eState[Dir]) ) {
			iErr = USB_EBUSY;  // EP not Running
			log_warning("%s EP not Running kind %d", __func__, kind);
			goto end; 
		}
		iErr = USB_ep0_out_start();
		
	} else {
		/* Check EP state */
		if (USB_EP_RUNNING != pstUSBDev->stEPs[ucEP].eState[Dir]) {
			iErr = USB_EBUSY;  // EP not Running
			log_warning("%s EP not Running kind %d", __func__, kind);
			goto end; 
		}
		if(pstUSBDev->stEPs[ucEP].pstXfer[Dir]) {
			iErr = USB_EBUSY;  // transfer busy
			log_warning("%s EP transfer busy %d", __func__, kind);
			goto end; 
		}
		
		USB_xfer_t *pstXfer;
		pstXfer = &pstCXfer->stDataXfer;
		pstXfer->usActLen = 0; 
		pstXfer->iStatus = USB_EIO; 
		pstUSBDev->stEPs[ucEP].ActLen[Dir] = 0;
		pstUSBDev->stEPs[ucEP].pstXfer[Dir] = pstXfer; 
		UsbEventFlag.d32 &= ~(USB_EVENT_EP(ucEP,Dir));
		if (0 == kind) {
			iErr = USB_EP0DataOutXfer(pstXfer);
		} else if (1 == kind) {
			iErr = USB_EP0DataInXfer(pstXfer);
		}
		if (USB_OK != iErr) {
			// Transfer not started
			pstUSBDev->stEPs[ucEP].pstXfer[Dir] = NULL; 
		}
	}
	
end:
	if (iErr != USB_OK) {
		log_error("%s iErr %d", __func__, iErr);
	}
	return iErr; 
}

#if RTE_USB2FS

typedef struct _USBD_CONTEXT {
	ARM_DRIVER_VERSION version;
	ARM_USBD_CAPABILITIES cap;
	bool init;
	ARM_POWER_STATE power_state;
	
	ARM_USBD_STATE state;
	ARM_USBD_SignalDeviceEvent_t cb_device_event;
	ARM_USBD_SignalEndpointEvent_t cb_endpoint_event;
	
	struct {
		uint8_t ep_addr;
		volatile bool wait;
	} blocking_xfer;
	
} USBD_CONTEXT;

static USBD_CONTEXT usbd = {
	.init = false,
	.power_state = ARM_POWER_OFF
};

/**
  @brief       Get driver version.
  @return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION USBD_GetVersion(void)
{
	return DriverVersion;
}

/**
  @brief       Get driver capabilities
  @return      \ref ARM_USBD_CAPABILITIES
*/
static ARM_USBD_CAPABILITIES USBD_GetCapabilities(void)
{
	return DriverCapabilities;
}

/**
  @brief       Control USB Device Interface Power.
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_PowerControl       (ARM_POWER_STATE state)
{
	if (!usbd.init) {
		return ARM_USBD_ERROR;
	}

	if (state == usbd.power_state) {
		return ARM_USBD_OK;
	}

	switch(state)
	{
	case ARM_POWER_OFF:
		if (!usbd.state.active) {
			USB_ModuleShutdown();
		} else {
			log_error("%s Cannot ARM_POWER_OFF in active state.", __func__);
			return ARM_USBD_ERROR;
		}
		break;
	case ARM_POWER_LOW:
		break;
	case ARM_POWER_FULL:
		if (47972352 > Driver_PMU.GetFrequency(PMU_CD_MPIER)) {
			log_error("%s USB module work only on 48MHz.", __func__);
			return ARM_USBD_ERROR;
		}
		// この時点でVBusが供給されている状態であるものとします。
		// Assume that is a state in which VBus is supplied at this time.
		USB_ModuleStart();
		USB_SoftDisconnect(0);
		break;
	default:
		log_error("%s invalid parameter %d", __func__, state);
		return ARM_USBD_ERROR;
	}

	usbd.power_state = state;
	return ARM_USBD_OK;
}

/**
  @brief       Initialize USB Device Interface. 
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_Initialize         (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                                      ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
{
	if (usbd.init) {
		return ARM_USBD_ERROR;
	}
	
	USB_Init();
	usbd.version = DriverVersion;
	usbd.cap = DriverCapabilities;
	
	usbd.state.powered = 0;
	usbd.state.connected = 0;
	usbd.state.active = 0;
	usbd.state.speed = 0;
	
	usbd.init = true;
	
	if (ARM_USBD_OK == USBD_PowerControl(ARM_POWER_LOW)) {
		usbd.cb_device_event = cb_device_event;
		usbd.cb_endpoint_event = cb_endpoint_event;
		return ARM_USBD_OK;
		
	} else {
		usbd.init = false;
		log_error("%s failure", __func__);
		return ARM_USBD_ERROR;
	}
}

/**
  @brief       De-initialize USB Device Interface.
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_Uninitialize       (void)
{
	USBD_DeviceDisconnect_core();
	
	usbd.cb_device_event = NULL;
	usbd.cb_endpoint_event = NULL;
	
	USBD_PowerControl(ARM_POWER_OFF);
	usbd.init = false;
	
	return ARM_USBD_OK;
}

/**
  @brief  USBD state check macro
 */
#define usbd_CheckState(ctx,ret)  if (!(ctx).init || (ctx).power_state != ARM_POWER_FULL) {return ret;}

/**
  @brief  USBD state check macro 
 */
#define usbd_CheckStateVoid(ctx)  if (!(ctx).init || (ctx).power_state != ARM_POWER_FULL) {return;}

/**
  @brief       Connect USB Device.
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_DeviceConnect      (void)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	if (usbd.state.active) return ARM_USBD_OK;
	
	USB_otg_core_init();
	USB_otg_fifo_init();
	USB_otg_device_init();
	if (DriverCapabilities.event_power_on == 0) {
		pstUSBDev->iDevState = USB_STATE_POWERED; // Vbus検出機構を持たない場合、Connect時点でPoweredと見なす。
	} else {
		pstUSBDev->iDevState = USB_STATE_DETACH;
	}
	// Init endpoints
	for (int i = 0; i<USB_NUM_EPS; ++i) {
		USBD_EndpointUnconfigure_core(USB_ep_addr(i,0));
		USBD_EndpointUnconfigure_core(USB_ep_addr(i,1));
	}
	
	USB_SoftConnect();
	
	usbd.state.active = 1;
	usbd.blocking_xfer.ep_addr = 0xff;
	usbd.blocking_xfer.wait = false;
	return ARM_USBD_OK;
}

/**
  @brief       Disconnect USB Device.
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_DeviceDisconnect   (void)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
	return USBD_DeviceDisconnect_core();
}

static ARM_USBD_STATUS       USBD_DeviceDisconnect_core   (void)
{
	if (!usbd.state.active) return ARM_USBD_OK;
	
	// USBD_EndpointAbort();
	USB_ep0_out_snak();
	
	// USBD_EndpointUnconfigure();
	
	USB_SoftDisconnect(4);  // 3ms + 2.5us, depends on link speed
	if (DriverCapabilities.event_power_on == 0) {
		pstUSBDev->iDevState = USB_STATE_DETACH; // Vbus検出機構を持たない場合、この時点でDETACHと見なす。
	} else {
		pstUSBDev->iDevState = USB_STATE_POWERED;
	}
	
	USB_SetAddress(0);
	usbd.state.powered = 0;
	usbd.state.connected = 0;
	usbd.state.active = 0;
	usbd.state.speed = 0;
	
	return ARM_USBD_OK;
}

/**
  @brief       Get current USB Device State.
  @return      \ref ARM_USBD_STATE
*/
static ARM_USBD_STATE        USBD_DeviceGetState     (void)
{
	return usbd.state;
}

/**
  @brief       Trigger USB Remote Wakeup.
  @return      \ref ARM_USBD_STATUS
  @note        dwc_otg_pcd.c / dwc_otg_pcd_rem_wkup_from_suspend
*/
static ARM_USBD_STATUS       USBD_DeviceRemoteWakeup (void)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
	dctl_data_t dctl = { 0 };
	dsts_data_t dsts;

	dsts.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dsts);
	if (!dsts.b.suspsts) {
		log_warning("Remote wakeup while is not in suspend state");
	}
	
	dctl.b.rmtwkupsig = 1;
	DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->dctl, 0, dctl.d32);
	log_debug("Set Remote Wakeup");

	DWC_DELAY_US(2000);
	DWC_MODIFY_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32, 0);
	log_debug("Clear Remote Wakeup");
	
	return ARM_USBD_OK;
}

/**
  @brief       Write USB Device Address register
*/
static void USB_SetAddress(uint8_t dev_addr)
{
	dcfg_data_t dcfg;
	dcfg.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dcfg);
	dcfg.b.devaddr = dev_addr;
	DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dcfg, dcfg.d32);
	pstUSBDev->ucDevAddr = dev_addr; 
}

/**
  @brief       Set USB Device Address.
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_DeviceSetAddress   (uint8_t dev_addr, ARM_USBD_SET_ADDRESS_STAGE stage)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
	ARM_USBD_STATUS status = ARM_USBD_OK;
	
	USB_state_t current = USB_STATE_GETSTATE(pstUSBDev->iDevState);
	
	if ((USB_STATE_DEFAULT   == current) ||
		(USB_STATE_ADDRESSED == current) ) {
		if (dev_addr) {
			pstUSBDev->iDevState = USB_STATE_ADDRESSED;
		} else {
			pstUSBDev->iDevState = USB_STATE_DEFAULT;
		}
	}
	
	USB_SetAddress(dev_addr);
	
	return status;
}

/**
  @brief       Configure/unconfigure USB Device.
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_DeviceConfigure    (bool configure)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
	ARM_USBD_STATUS status = ARM_USBD_OK;
	
	USB_state_t current = USB_STATE_GETSTATE(pstUSBDev->iDevState);
	
	if ((USB_STATE_ADDRESSED == current) && configure) {
		pstUSBDev->iDevState = USB_STATE_CONFIGURED;
	} else if ((USB_STATE_CONFIGURED == current) && !configure) {
		pstUSBDev->iDevState = USB_STATE_ADDRESSED;
	} else {
		log_error("%s Device State %d", __func__, current);
		status = ARM_USBD_ERROR;
	}
	
	return status;
}

/**
  @brief       Configure USB Endpoint.
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_EndpointConfigure  (uint8_t ep_addr,
                                           ARM_USB_ENDPOINT_TYPE ep_type,
                                           uint16_t ep_max_packet_size)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
//	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	
	if (ep_num > 0) {
		return USBD_EndpointConfigure_core(ep_addr, ep_type, ep_max_packet_size);
	} else {
		return ARM_USBD_OK; // no op
	}
}

/**
  @brief       Configure USB Endpoint core
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_EndpointConfigure_core  (uint8_t ep_addr,
                                           ARM_USB_ENDPOINT_TYPE ep_type,
                                           uint16_t ep_max_packet_size)
{
	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	if (ep_num >= USB_NUM_EPS) {
		log_error("%s Invalid ep_num %d", __func__, ep_num);
		return ARM_USBD_ERROR;
	}
	
	// check the restrictions
	if (ARM_USB_ENDPOINT_ISOCHRONOUS == ep_type) {
		log_error("%s ep_type = Isochronous is Not supported", __func__);
		return ARM_USBD_ERROR;
	}
	
	if ((ARM_USB_ENDPOINT_CONTROL == ep_type) && (0 != ep_num)) {
		log_error("%s ep_type = Control and ep!=0 is Not supported", __func__);
		return ARM_USBD_ERROR;
	}
	
	if ((ARM_USB_ENDPOINT_CONTROL != ep_type) && (0 == ep_num)) {
		log_error("%s ep_type != Control and ep=0 is Not supported", __func__);
		return ARM_USBD_ERROR;
	}
	
	pstUSBDev->stEPs[ep_num].pstXfer[ep_dir] = NULL;
	pstUSBDev->stEPs[ep_num].ActLen[ep_dir] = 0;
	pstUSBDev->stEPs[ep_num].usMPS[ep_dir] = ep_max_packet_size;
	pstUSBDev->stEPs[ep_num].eptype[ep_dir] = ep_type;
	pstUSBDev->stEPs[ep_num].eState[ep_dir] = USB_EP_RUNNING;
	// @warning If you want to use two or more the InterruptIN EP, please set properly TxFIFO Number and FIFO configuration.
	// @see USB_otg_fifo_init()
	if (USB_EP_IS_PERIODIC(ep_type)) {
		pstUSBDev->stEPs[ep_num].txfnum = 1; // DPTXF_1
	} else {
		pstUSBDev->stEPs[ep_num].txfnum = 0; // Shared TxFIFO
	}
	
	if (0 == ep_num) {
		memset(&pstUSBDev->stCtrlXfer, 0, sizeof(pstUSBDev->stCtrlXfer));
		pstUSBDev->EP0State = EP0_IDLE;
		pstUSBDev->EP0State_PhaseEntered = false;
		pstUSBDev->EP0State_StatusPhaseHandled = false;
		USB_EP0IntMask(false);
	} else {
		USB_EP_Activate(ep_num, ep_dir);
	}
	log_warning("activate ep_num=%d Dir=%d", ep_num, ep_dir);
	
	return ARM_USBD_OK;
}

/**
  @brief       Unconfigure USB Endpoint.
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_EndpointUnconfigure(uint8_t ep_addr)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
//	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	
	// USBD_EndpointAbort();
	
	if (ep_num > 0) {
		return USBD_EndpointUnconfigure_core(ep_addr);
	} else {
		return ARM_USBD_OK; // no op
	}
}

/**
  @brief       Unconfigure USB Endpoint core
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_EndpointUnconfigure_core(uint8_t ep_addr)
{
	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	if (ep_num >= USB_NUM_EPS) {
		log_error("%s Invalid ep_num %d", __func__, ep_num);
		return ARM_USBD_ERROR;
	}
	
	pstUSBDev->stEPs[ep_num].pstXfer[ep_dir] = NULL;
	pstUSBDev->stEPs[ep_num].ActLen[ep_dir] = 0;
	pstUSBDev->stEPs[ep_num].usMPS[ep_dir] = 0;
	pstUSBDev->stEPs[ep_num].eptype[ep_dir] = 0;
	pstUSBDev->stEPs[ep_num].eState[ep_dir] = USB_EP_CLOSED;
	
	if (0 == ep_num) {
		USB_EP0IntMask(true);
		memset(&pstUSBDev->stCtrlXfer, 0, sizeof(pstUSBDev->stCtrlXfer));
		pstUSBDev->EP0State = EP0_DISCONNECT;
		pstUSBDev->EP0State_PhaseEntered = false;
		pstUSBDev->EP0State_StatusPhaseHandled = false;
	} else {
		USB_EP_Deactivate(ep_num, ep_dir);
	}
	log_warning("deactivate ep_num=%d Dir=%d", ep_num, ep_dir);
	
	return ARM_USBD_OK;
}

/**
  @brief       Set/Clear Stall for USB Endpoint.
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_EndpointStall      (uint8_t ep_addr, bool stall)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	if (ep_num >= USB_NUM_EPS) {
		log_error("%s Invalid ep_num %d", __func__, ep_num);
		return ARM_USBD_ERROR;
	}
	
	USB_err_t iErr;
	
	if (stall) {
		iErr = USB_EPSetStall(ep_num, ep_dir);
		pstUSBDev->stEPs[ep_num].pstXfer[ep_dir] = NULL;
		if (USB_ep_addr(0,0) == ep_addr) {
			// EP0をstallした場合、ControlTransferを途中のphaseで中断したものとみなす。
			// Shall be deemed that if stall the EP0, was interrupted in the middle phase of the ControlTransfer.
			pstUSBDev->EP0State = EP0_STALL;
			// Always receive SETUP packet.
			iErr = USB_ep0_out_start();
		}
		
	} else {
		iErr = USB_EPClearStall(ep_num, ep_dir);
	}
	
	return (iErr == USB_OK) ? ARM_USBD_OK : ARM_USBD_ERROR;
}

/**
  @brief       Start USB Endpoint Read operation.
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_EndpointReadStart  (uint8_t ep_addr,       uint8_t *buf, uint32_t len)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	if (ep_num >= USB_NUM_EPS) {
		log_error("%s Invalid ep_num %d", __func__, ep_num);
		return ARM_USBD_ERROR;
	}
	if (ep_dir) {
		log_error("%s Direction mismatch", __func__);
		return ARM_USBD_ERROR;
	}
	
	USB_err_t iErr; 
	
	if (0 == ep_num) {
		pstUSBDev->stCtrlXfer.stDataXfer.usLen = len;
		pstUSBDev->stCtrlXfer.stDataXfer.pvBuffer = (void *)buf;
		EP0_State_t current = pstUSBDev->EP0State;
		uint8_t kind = 0;
		bool_t StatusPhaseHandled = false;
		switch(current)
		{
		case EP0_IDLE:
			log_error("%s EP0State %d", __func__, current);
			return ARM_USBD_ERROR;
			
		case EP0_IN_DATA_PHASE:
			if (0 == len) {
				// Control_Read(StatusOutPhase) 
				pstUSBDev->EP0State = EP0_OUT_STATUS_PHASE;
				pstUSBDev->EP0State_PhaseEntered = true;
				
				if (pstUSBDev->EP0State_StatusPhaseHandled) {
					log_info("%s Status phase already completed and handled.", __func__);
					kind = 3; // out status, but no register write.
					StatusPhaseHandled = true;
					
				} else {
					/// @note  dwc_otg_pcd_intr.c / do_setup_out_status_phase
					/* If there is xfercomplete on EP0 OUT do not start OUT Status phase.
					 * xfercomplete means that ZLP was already received as EP0 OUT is enabled 
					 * during IN Data phase
					 */
					doepint_data_t doepint;
					doepint.d32 = DWC_READ_REG32(&pstUSBDev->stEPs[0].pstOutEP->doepint);
					if (doepint.b.xfercompl == 1) {
						/* Should not clear the xfercompl status. Will be processed by USB_intrOutEP0() */
						log_info("%s Status phase already completed", __func__);
						kind = 3; // out status, but no register write.
					}
				}
				
			} else {
				log_error("%s EP0State %d", __func__, current);
				return ARM_USBD_ERROR;
			}
			break;
			
		case EP0_OUT_DATA_PHASE:
			// Control_Write(DataOutPhase) 
			break;
			
		case EP0_IN_STATUS_PHASE:
		case EP0_OUT_STATUS_PHASE:
		case EP0_STALL:
		default:
			log_error("%s EP0State %d", __func__, current);
			return ARM_USBD_ERROR;
		}
		iErr = USB_xferEP0(kind, &pstUSBDev->stCtrlXfer); // DATA OUT or STATUS OUT
		pstUSBDev->EP0State_PhaseEntered = false;
		if (StatusPhaseHandled) {
			handle_ep0(); // @note 再びUSBD_SignalEndpointEvent()が呼ばれてusbstack.cへ行くので、関数コールがとても深くなるため、スタックオーバーフローに注意する。
			log_info("%s StatusPhaseHandled handle_ep0() returned", __func__);
		}
		return (iErr == USB_OK) ? ARM_USBD_OK : ARM_USBD_ERROR;
		
	} else {
		USB_xfer_t *pstXfer;
		
		/* Allocate transfer structure */
		switch(pstUSBDev->stEPs[ep_num].eptype[ep_dir]) {
		case USB_EP_BULK:
		case USB_EP_INTR:
			pstXfer = &pstUSBDev->stXfer[ep_num][ep_dir];
			break;
		default:
			log_error("%s invalid eptype %d", __func__, pstUSBDev->stEPs[ep_num].eptype[ep_dir]);
			return ARM_USBD_ERROR;
		}
		iErr = USB_xferEP(ep_addr, buf, len, pstXfer);
		return (iErr == USB_OK) ? ARM_USBD_OK : ARM_USBD_ERROR;
	}
}

/**
  @brief       Read data from USB Endpoint.
  @return      >=0 : number of data bytes read or execution status
                <0 : negative value of ARM_USBD_STATUS
*/
static int32_t               USBD_EndpointRead       (uint8_t ep_addr,       uint8_t *buf, uint32_t len)
{
	usbd_CheckState(usbd, -ARM_USBD_ERROR);
	
	if (0xff != usbd.blocking_xfer.ep_addr) {
		return -ARM_USBD_ERROR;
	}
	
	usbd.blocking_xfer.ep_addr = ep_addr;
	usbd.blocking_xfer.wait    = true;
	
	ARM_USBD_STATUS status;
	status = USBD_EndpointReadStart(ep_addr, buf, len);
	if (ARM_USBD_OK == status) {
		while(usbd.blocking_xfer.wait); // Wait for read completion (Blocking)
		uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
		uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
		return pstUSBDev->stEPs[ep_num].ActLen[ep_dir];
	} else {
		return -ARM_USBD_ERROR;
	}
}

/**
  @brief       Write data to USB Endpoint.
  @return      >=0 : number of data bytes written or execution status
                <0 : negative value of ARM_USBD_STATUS
*/
static int32_t               USBD_EndpointWrite      (uint8_t ep_addr, const uint8_t *buf, uint32_t len)
{
	usbd_CheckState(usbd, -ARM_USBD_ERROR);
	
	if (0xff != usbd.blocking_xfer.ep_addr) {
		return -ARM_USBD_ERROR;
	}
	
	usbd.blocking_xfer.ep_addr = ep_addr;
	usbd.blocking_xfer.wait    = true;
	
	ARM_USBD_STATUS status;
	status = USBD_EndpointWriteStart(ep_addr, buf, len);
	if (ARM_USBD_OK == status) {
		while(usbd.blocking_xfer.wait); // Wait for Write completion (Blocking)
		uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
		uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
		return pstUSBDev->stEPs[ep_num].ActLen[ep_dir];
	} else {
		return -ARM_USBD_ERROR;
	}
}

/**
  @brief       Abort current USB Endpoint transfer.
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_EndpointAbort      (uint8_t ep_addr)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	if (ep_num >= USB_NUM_EPS) {
		log_error("%s Invalid ep_num %d", __func__, ep_num);
		return ARM_USBD_ERROR;
	}
	
	USB_err_t iErr;
	iErr = USB_EPAbort(ep_num, ep_dir);
	
	if (USB_ep_addr(0,0) == ep_addr) {
		// Always receive SETUP packet.
		iErr = USB_ep0_out_start();
	}
	
	return (iErr == USB_OK) ? ARM_USBD_OK : ARM_USBD_ERROR;
}

/**
  @brief       Get current USB Frame Number.
  @return      Frame Number 
  @note        dwc_otg_cil.c / dwc_otg_get_frame_number
*/
static uint16_t              USBD_GetFrameNumber     (void)
{
	usbd_CheckState(usbd, 0);
	
	dsts_data_t dsts;
	dsts.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dsts);
	
	return dsts.b.soffn;
}

/**
  @brief       Start USB Endpoint Write operation.
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_EndpointWriteStart (uint8_t ep_addr, const uint8_t *buf, uint32_t len)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	if (ep_num >= USB_NUM_EPS) {
		log_error("%s Invalid ep_num %d", __func__, ep_num);
		return ARM_USBD_ERROR;
	}
	
	if (0 == ep_dir) {
		log_error("%s Direction mismatch", __func__);
		return ARM_USBD_ERROR;
	}
	
	USB_err_t iErr = USB_EINVAL;
	
	if (0 == ep_num) {
		pstUSBDev->stCtrlXfer.stDataXfer.usLen = len;
		pstUSBDev->stCtrlXfer.stDataXfer.pvBuffer = (void *)buf;
		EP0_State_t current = pstUSBDev->EP0State;
		switch(current)
		{
		case EP0_IDLE:
			log_error("%s EP0State %d", __func__, current);
			return ARM_USBD_ERROR;
			
		case EP0_IN_DATA_PHASE:
			// Control_Read(DataInPhase) 
			if (pstUSBDev->EP0State_StatusPhaseHandled) {
				log_info("%s Status phase already completed and handled.", __func__);
				// handle_ep0()と似た処理を行います。
				pstUSBDev->stEPs[0].ActLen[1] = len;
				pstUSBDev->stCtrlXfer.DataPhase.ulTotalLen += len;
				USBD_SignalEndpointEvent(USB_ep_addr(0,1), ARM_USBD_EP_EVENT_IN);
				// @note 再びUSBD_SignalEndpointEvent()が呼ばれてusbstack.cへ行くので、関数コールがとても深くなるため、スタックオーバーフローに注意する。
				log_info("%s pseudo handle_ep0() returned", __func__);
				return ARM_USBD_OK;
			}
			break;
			
		case EP0_OUT_DATA_PHASE:
			if (0 == len) {
				// Control_Write(StatusInPhase) 
				pstUSBDev->EP0State = EP0_IN_STATUS_PHASE;
				pstUSBDev->EP0State_PhaseEntered = true;
			} else {
				log_error("%s EP0State %d", __func__, current);
				return ARM_USBD_ERROR;
			}
			break;
			
		case EP0_IN_STATUS_PHASE:
			if (0 == len) {
				// Control_Write(NoData,StatusInPhase) 
			} else {
				log_error("%s EP0State %d", __func__, current);
				return ARM_USBD_ERROR;
			}
			break;
			
		case EP0_OUT_STATUS_PHASE:
		case EP0_STALL:
		default:
			log_error("%s EP0State %d", __func__, current);
			return ARM_USBD_ERROR;
		}
		iErr = USB_xferEP0(1, &pstUSBDev->stCtrlXfer); // DATA IN or STATUS IN
		pstUSBDev->EP0State_PhaseEntered = false;
		return (iErr == USB_OK) ? ARM_USBD_OK : ARM_USBD_ERROR;
		
	} else {
		USB_xfer_t *pstXfer;
		
		/* Allocate transfer structure */
		switch(pstUSBDev->stEPs[ep_num].eptype[ep_dir]) {
		case USB_EP_BULK:
		case USB_EP_INTR:
			pstXfer = &pstUSBDev->stXfer[ep_num][ep_dir];
			break;
		default:
			log_error("%s invalid eptype %d", __func__, pstUSBDev->stEPs[ep_num].eptype[ep_dir]);
			return ARM_USBD_ERROR;
		}
		
		iErr = USB_xferEP(ep_addr, (void *)buf, len, pstXfer);
		log_debug("%s USB_xferEP iErr %d", __func__, iErr);
		return (iErr == USB_OK) ? ARM_USBD_OK : ARM_USBD_ERROR;
	}
}

/**
  @brief       Get result of USB Endpoint transfer.
               Returns the number of successfully transferred data bytes started by EndpointRead/Write operation. 
               Except SETUP packet .
  @param[in]   ep_addr  Endpoint Address
                ep_addr.0..3: Address
                ep_addr.7:    Direction
  @return      number of successfully transfered data bytes
*/
static uint32_t USBD_EndpointTransferGetResult (uint8_t ep_addr)
{
	usbd_CheckState(usbd, 0);
	
	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	if (ep_num >= USB_NUM_EPS) {
		log_error("%s Invalid ep_num %d", __func__, ep_num);
		return 0;
	}
	
	return pstUSBDev->stEPs[ep_num].ActLen[ep_dir];
}

/**
  @brief       Read setup packet received over Control Endpoint. 
  @param[in]   setup      read buffer
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS USBD_ReadSetupPacket (uint8_t *setup)
{
	USB_ctrlXfer_t *pstCXfer = &pstUSBDev->stCtrlXfer;
	if (setup) {
		memcpy(setup, &pstCXfer->stReq, 8);
		return ARM_USBD_OK;
	} else {
		return ARM_USBD_ERROR;
	}
}

/**
  @brief       Set Endpoint DATAn PID.
  @param[in]   ep_addr  Endpoint Address
                ep_addr.0..3: Address
                ep_addr.7:    Direction
  @param[in]   toggle  DATA toggle 0 or 1
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_EndpointSetDPID     (uint8_t ep_addr, uint8_t toggle)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
	uint8_t ep_dir = (((ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	if (ep_num >= USB_NUM_EPS) {
		log_error("%s Invalid ep_num %d", __func__, ep_num);
		return ARM_USBD_ERROR;
	}
	
	depctl_data_t depctl;
	volatile uint32_t *addr;
	
	if (ep_dir) {
		addr = &pstUSBDev->stEPs[ep_num].pstInEP->diepctl;
	} else {
		addr = &pstUSBDev->stEPs[ep_num].pstOutEP->doepctl;
	}
	depctl.d32 = DWC_READ_REG32(addr);
	if (depctl.b.usbactep) {
		if (toggle) {
			depctl.b.setd1pid = 1;
		} else {
			depctl.b.setd0pid = 1;
		}
		DWC_WRITE_REG32(addr, depctl.d32);
//		log_info("dXepctlN addr=0x%08X value=0x%08X", addr, DWC_READ_REG32(addr));
	}
	
	return ARM_USBD_OK;
}

/**
  @brief       Set Endpoint DATAn PID.
  @param[in]   mode  Test Mode selector
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS       USBD_DeviceSetTestMode   (uint8_t mode)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
	dctl_data_t dctl;
	dctl.d32 = DWC_READ_REG32(&pstUSBDev->pstDRegs->dctl);
	if (5 < mode) {
		mode = 0; // Test mode disabled
	}
	dctl.b.tstctl = mode;
	DWC_WRITE_REG32(&pstUSBDev->pstDRegs->dctl, dctl.d32);
	
	return ARM_USBD_OK;
}

/**
  @brief       Get Error status
  @param[out]  param  Error parameter
  @return      Error status
*/
static USBD_TZ10xx_ERROR     USBD_GetError   (uint32_t *param)
{
	if (param) {
		*param = stError.param;
	}
	return stError.status;
}

/**
  @brief       Control USB Interface
  @param[in]   control Operation
                control.0..7: Control code
                control.8..15: Endpoint
                control.16..31: reserved
  @param[in]   arg     Argument of operation (optional) 
  @return      \ref ARM_USBD_STATUS
*/
static ARM_USBD_STATUS     USBD_Control   (uint32_t  control, uint32_t arg)
{
	usbd_CheckState(usbd, ARM_USBD_ERROR);
	
	ARM_USBD_STATUS status = ARM_USBD_OK;
	
	bool enable_event = false;
	uint8_t control_ep_addr = (control & USBD_TZ10xx_CONTROL_EP_ADDR_Mask) >> USBD_TZ10xx_CONTROL_EP_ADDR_Pos;
//	uint8_t ep_dir = (((control_ep_addr) & ARM_USB_ENDPOINT_DIRECTION_MASK) ? 1 : 0);
	uint8_t ep_num = ((control_ep_addr) & ARM_USB_ENDPOINT_NUMBER_MASK);
	if (ep_num >= USB_NUM_EPS) {
		log_error("%s Invalid ep_num %d", __func__, ep_num);
		return ARM_USBD_ERROR;
	}
	
	uint8_t control_code = (control & USBD_TZ10xx_CONTROL_CODE_Mask) >> USBD_TZ10xx_CONTROL_CODE_Pos;
	switch(control_code)
	{
	case USBD_TZ10xx_CONTROL_ENABLE_EVENT:
		enable_event = true;
		/* FALLTHROUGH */
	case USBD_TZ10xx_CONTROL_DISABLE_EVENT:
		switch((USBD_TZ10xx_EVENT)arg)
		{
		case USBD_TZ10xx_EVENT_SOF:
			{
				gintmsk_data_t gintmsk;
				gintmsk.d32 = 0;
				gintmsk.b.sofintr = 1;
				if (enable_event) {
					/* Unmask interrupt */
					DWC_MODIFY_REG32(&pstUSBDev->pstGRegs->gintmsk, 0, gintmsk.d32);
				} else {
					/* Mask interrupt */
					DWC_MODIFY_REG32(&pstUSBDev->pstGRegs->gintmsk, gintmsk.d32, 0);
				}
			}
			break;
			
		default:
			status = ARM_USBD_ERROR;
			break;
		}
		break;
		
	default:
		status = ARM_USBD_ERROR;
		break;
	}
	
	return status;
}


/**
  \fn          void ARM_USBD_SignalDeviceEvent (ARM_USBD_EVENT event)
  \brief       Signal USB Device Event.
  \param[in]   event \ref ARM_USBD_EVENT
  \return      none
*/
static void USBD_SignalDeviceEvent (ARM_USBD_EVENT  event)
{
	usbd_CheckStateVoid(usbd);
	
	if (usbd.cb_device_event) {
		usbd.cb_device_event(event);
	}
}

/**
  \fn          void ARM_USBD_SignalEndpointEvent (uint8_t ep_addr, ARM_USBD_EP_EVENT ep_event)
  \brief       Signal USB Endpoint Event.
               Occurs when Endpoint Read/Write completes.
  \param[in]   ep_addr  Endpoint Address
                ep_addr.0..3: Address
                ep_addr.7:    Direction
  \param[in]   ep_event \ref ARM_USBD_EP_EVENT
  \return      none
*/
static void USBD_SignalEndpointEvent(uint8_t ep_addr, ARM_USBD_EP_EVENT ep_event)
{
	usbd_CheckStateVoid(usbd);
	
	// Signal block transfer wait completion
	switch(ep_event)
	{
	case ARM_USBD_EP_EVENT_OUT :
	case ARM_USBD_EP_EVENT_IN  :
		if (usbd.blocking_xfer.ep_addr == ep_addr) {
			usbd.blocking_xfer.ep_addr = 0xff;
			usbd.blocking_xfer.wait = false;
			return;
		}
		break;
	default:
		break;
	}
	
	if (usbd.cb_endpoint_event)  {
		usbd.cb_endpoint_event(ep_addr, ep_event);
	}
}

/**
  \fn          void USBD_SignalLinkSpeed(uint8_t link_speed)
  \brief       Signal USB Link speed
               Occurs when usb reset completes.
  \param[in]   uint8_t  link_speed
  \return      none
*/
static void USBD_SignalLinkSpeed(uint8_t link_speed)
{
	usbd_CheckStateVoid(usbd);
	if (!usbd.state.active) return;
	
	switch(link_speed)
	{
	case ARM_USB_SPEED_LOW:
	case ARM_USB_SPEED_FULL:
	case ARM_USB_SPEED_HIGH:
		usbd.state.speed = link_speed;
		break;
	default:
		break;
	}
}

/**
 * @brief   Suspend
 * @param   なし
 * @return  なし
 */
static void USB_Suspend(void)
{
	pcgcctl_data_t	 pcgcctl;
	pcgcctl.d32 = DWC_READ_REG32(&pstUSBDev->pstPRegs->pcgcctl);
	pcgcctl.b.stoppclk = 1;
	DWC_WRITE_REG32(&pstUSBDev->pstPRegs->pcgcctl, pcgcctl.d32);
}

/**
 * @brief   Resume
 * @param   なし
 * @return  なし
 */
static void USB_ResumeEarly(void)
{
	pcgcctl_data_t	 pcgcctl;
	pcgcctl.d32 = DWC_READ_REG32(&pstUSBDev->pstPRegs->pcgcctl);
	pcgcctl.b.stoppclk = 0;
	DWC_WRITE_REG32(&pstUSBDev->pstPRegs->pcgcctl, pcgcctl.d32);
}

/* USBD0 Driver Control Block */
TZ10XX_DRIVER_USBD Driver_USBD0 = {
	// Driver_USBD.h/typedef struct _ARM_DRIVER_USBD {
	USBD_GetVersion,
	USBD_GetCapabilities,
	USBD_Initialize,
	USBD_Uninitialize,
	USBD_PowerControl,
	USBD_DeviceConnect,
	USBD_DeviceDisconnect,
	USBD_DeviceGetState,
	USBD_DeviceRemoteWakeup,
	USBD_DeviceSetAddress,
	USBD_DeviceConfigure,
	USBD_EndpointConfigure,
	USBD_EndpointUnconfigure,
	USBD_EndpointStall,
	USBD_EndpointReadStart,
	USBD_EndpointRead,
	USBD_EndpointWrite,
	USBD_EndpointAbort,
	USBD_GetFrameNumber,
	// } Driver_USBD.h/typedef struct _ARM_DRIVER_USBD
	USBD_EndpointWriteStart,
	USBD_EndpointTransferGetResult,
	USBD_ReadSetupPacket,
	USBD_EndpointSetDPID,
	USBD_DeviceSetTestMode,
	USBD_GetError,
	USBD_Control
};

#endif /* RTE_USB2FS */
