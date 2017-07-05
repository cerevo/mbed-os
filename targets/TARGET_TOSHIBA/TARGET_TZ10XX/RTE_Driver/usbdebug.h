/**
 * @file usbdebug.h
 * @brief USB module debug code
 * @version V0.0
 * @date $Date:: 2014-10-14 18:41:03 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef USB_DBG
#define USB_DBG

#include "RTE_Device.h"

#if RTE_USB2FS_DEBUG
#define DEBUG
#endif

extern void sample_print(const char *format, ...);
extern void sample_print_init(void);
extern void sample_print_set_mask(unsigned char mask);
extern unsigned char sample_print_get_mask(void);

#ifdef DEBUG

	#define USB_USE_UART0  /* if defined, comments are writen by using uart0. */
	#define log_error(...)    sample_print("E/" __VA_ARGS__)
	#define log_warning(...)  sample_print("W/" __VA_ARGS__)
	#define log_info(...)     sample_print("I/" __VA_ARGS__)
//	#define log_debug(...)    sample_print("D/" __VA_ARGS__)
	#define log_debug(...)    
	#define SCSI_DBGMES(...)  sample_print("S/" __VA_ARGS__)

	#include "ip_config.h"
	#define assert_ep_xfer(pktcnt, xfersize, addr)  \
		assert(OTG_PACKET_MAX_COUNT >= (pktcnt));   \
		assert(OTG_TRANS_MAX_SIZE >= (xfersize));   

#else
	#define log_error(format, ...)
	#define log_warning(format, ...)
	#define log_info(format, ...)
	#define log_debug(format, ...)
	#define usbprint(format, ...)
	#define SCSI_DBGMES(format, ...)
	#define assert_ep_xfer(a,b,c)

#endif


#endif // USB_DBG
