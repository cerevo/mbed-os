/**
 * @file dwc_os.h
 * @brief OS conversion layer
 */

/* =========================================================================
 * $File: //dwh/usb_iip/dev/software/dwc_common_port_2/dwc_os.h $
 * $Revision: #14 $
 * $Date: 2010/11/04 $
 * $Change: 1621695 $
 *
 * Synopsys Portability Library Software and documentation
 * (hereinafter, "Software") is an Unsupported proprietary work of
 * Synopsys, Inc. unless otherwise expressly agreed to in writing
 * between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product
 * under any End User Software License Agreement or Agreement for
 * Licensed Product with Synopsys or any supplement thereto. You are
 * permitted to use and redistribute this Software in source and binary
 * forms, with or without modification, provided that redistributions
 * of source code must retain this notice. You may not view, use,
 * disclose, copy or distribute this file or any information contained
 * herein except pursuant to this license grant from Synopsys. If you
 * do not agree with this notice, including the disclaimer below, then
 * you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS"
 * BASIS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE HEREBY DISCLAIMED. IN NO EVENT SHALL
 * SYNOPSYS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================= */

#ifndef _DWC_OS_H_
#define _DWC_OS_H_

/** Reads the content of a 32-bit register. */
static inline uint32_t DWC_READ_REG32(uint32_t volatile *reg)
{
	uint32_t	data;
	data = *((const volatile uint32_t *) reg);
	return(data);
}

/** Writes to a 32-bit register. */
static inline void DWC_WRITE_REG32(uint32_t volatile *reg, uint32_t value)
{
	*((volatile uint32_t *) reg) = value;
}

/**
 * Modify bit values in a register.  Using the
 * algorithm: (reg_contents & ~clear_mask) | set_mask.
 */
static inline void DWC_MODIFY_REG32(uint32_t volatile *reg, uint32_t clear_mask, uint32_t set_mask)
{
	uint32_t	data;
	data = *((const volatile uint32_t *) reg);
	data = (data & ~clear_mask) | set_mask;
	*((volatile uint32_t *) reg) = data;
}

#include "TZ10xx.h"

static void DWC_DELAY_US(uint32_t usec)
{
	int32_t t = ( SystemCoreClock / 1000000 ) * usec / 8; // 8cycle/loop
	while (--t > 0) {
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}
}

#endif
