/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#include <stdint.h>
#ifdef  __ICCARM__
#include "intrinsics.h"
#define TZ10xx_FWCOPIER  @ ".fwcopier"
#else
#define TZ10xx_FWCOPIER
#endif

#if defined ( __arm )
#define rev(x)     __rev(x)
#define nop()      __nop()
#elif defined ( __ICCARM__ )
#define rev(x)     __REV(x)
#define nop()      __no_operation()
#else
#define rev(x)     __builtin_bswap32(x)
#define nop()      __asm__ volatile("nop")
#endif

#define ADDRESS_FOR_NORMAL_FW         (0x08000)
#define ADDRESS_FOR_FAILSAFE_FW       (0xB7000)
#define SIZE_FOR_NORMAL_FW            (0x48000)

#define PAGE_SIZE                     (0x100)
#define SECTOR_SIZE                   (0x1000)

#define RDEND                         (0x1)
#define WREND                         (0x2)

#define REG(adr)   (*((volatile unsigned long*)(CNT_BASE + (adr))))

#define CORE_CLOCK   48000000

#define CNT_BASE     0x40004000 /* Controller base address */

#define BUSY_WAIT(usec)    do {	uint32_t i;	i = CORE_CLOCK / 1000000 * usec;	do { nop();	} while (--i); } while(0)

void fwcopier_tz10xx(void)		TZ10xx_FWCOPIER
{
	uint32_t s, st;
	uint32_t adr, cmd;
	uint32_t ofs, sz, beg, rem;
	uint32_t ctl, val, size;
	uint32_t *p;
	uint32_t i, j;

	/* save registers */
	ctl  = REG(0x030);
	val  = REG(0x100);
	size = REG(0x024);

	/* erase all sectors */
	sz  = size;
	rem = SIZE_FOR_NORMAL_FW - sz;
	beg = ADDRESS_FOR_NORMAL_FW;
	for (j = 0; j < 2; ++j) {
		/* load normal firmware */
		REG(0x024) = sz;
		REG(0x030) = ctl;
		REG(0x100) = rev(beg) | val;
		REG(0x034) = 1;
		do {
			s  = REG(0x0A0);
			s &= RDEND;
		} while (0 == s);
		REG(0x0A0) = 0x0000000F;

		for (adr = beg; adr < beg + sz; adr += SECTOR_SIZE) {
			/* check sector */
			p = (uint32_t*)(0x10000000 + (adr - beg));
			for (i = 0; i < SECTOR_SIZE / sizeof(uint32_t); ++i, ++p) {
				if (*p != 0xFFFFFFFF) {
					break;
				}
			}
			if (i == SECTOR_SIZE / sizeof(uint32_t)) {
				continue;
			}
			/* write enable */
			REG(0x030) = 0x00000310;
			REG(0x100) = 0x00000006;
			REG(0x034) = 0x00000001;
			do {
				s  = REG(0x0A0);
				s &= WREND;
			} while (0 == s);
			REG(0x0A0) = 0x0000000F;

			BUSY_WAIT(9);

			/* erase sector */
			REG(0x030) = 0x00030310;
			REG(0x100) = rev(adr) | 0x20;
			REG(0x034) = 0x00000001;
			do {
				s  = REG(0x0A0);
				s &= WREND;
			} while (0 == s);
			REG(0x0A0) = 0x0000000F;

			/* read status */
			do {
				BUSY_WAIT(200);
				REG(0x030) = 0x00000230;
				REG(0x100) = 0x00000005;
				REG(0x034) = 0x00000001;
				do {
					s  = REG(0x0A0);
					s &= RDEND;
				} while (0 == s);
				REG(0x0A0) = 0x0000000F;
				st = REG(0x200);
			st &= 1;
			} while (0 != st);
		}
		/* update parameter for next iteration. */
		beg += sz;
		sz   = rem;
	}

	/* select command for program pages */
	if (ctl == 0x00040010) {
		cmd  = 0x02;			/* single */
	} else {
		cmd  = 0x32;			/* quad */
	}

	/* copy failsafe firmware from tail-end */
	sz  = size;
	ofs = SIZE_FOR_NORMAL_FW    - sz;
	beg = ADDRESS_FOR_NORMAL_FW + ofs;
	for (j = 0; j < 2; ++j) {
		/* load failsafe firmware into SRAM */
		REG(0x020) = 0x10000000;
		REG(0x024) = sz;
		REG(0x030) = ctl;
		REG(0x100) = rev(ADDRESS_FOR_FAILSAFE_FW + ofs) | val;
		REG(0x034) = 1;
		do {
			s  = REG(0x0A0);
			s &= RDEND;
		} while (0 == s);
		REG(0x0A0) = 0x0000000F;

		REG(0x024) = PAGE_SIZE;
		for (adr = beg + sz - PAGE_SIZE; beg <= adr; adr -= PAGE_SIZE) {
			/* check sector */
			p = (uint32_t*)(0x10000000 + (adr - beg));
			for (i = 0; i < PAGE_SIZE / sizeof(uint32_t); ++i, ++p) {
				if (*p != 0xFFFFFFFF) {
					break;
				}
			}
			if (i == PAGE_SIZE / sizeof(uint32_t)) {
				continue;
			}
			/* write enable */
			REG(0x030) = 0x00000310;
			REG(0x100) = 0x00000006;
			REG(0x034) = 0x00000001;
			do {
				s  = REG(0x0A0);
				s &= WREND;
			} while (0 == s);
			REG(0x0A0) = 0x0000000F;

			BUSY_WAIT(9);

			/* program page */
			REG(0x020) = 0x10000000 + (adr - beg);
			REG(0x030) = 0x00030110;
			REG(0x100) = rev(adr) | cmd;
			REG(0x034) = 0x00000001;
			do {
				s  = REG(0x0A0);
				s &= WREND;
			} while (0 == s);
			REG(0x0A0) = 0x0000000F;

			/* read status */
			do {
				BUSY_WAIT(20);
				REG(0x030) = 0x00000230;
				REG(0x100) = 0x00000005;
				REG(0x034) = 0x00000001;
				do {
					s  = REG(0x0A0);
					s &= RDEND;
				} while (0 == s);
				REG(0x0A0) = 0x0000000F;
				st = REG(0x200);
				st &= 1;
			} while (0 != st);
		}
		/* update parameter for next iteration. */
		beg -= ofs;
		sz   = ofs;
		ofs  = 0;
	}

	/* restore registers */
	REG(0x020) = 0x10000000;
	REG(0x024) = size;
	REG(0x030) = ctl;
	REG(0x100) = val;
	REG(0x200) = 0;
}
