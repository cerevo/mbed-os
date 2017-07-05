/*
// COPYRIGHT (C) 2017
// TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
//
// This software is released under the MIT License.
// http://opensource.org/licenses/mit-license.php
 */

	.syntax	unified
	.arch	armv7-m

	.equ	vtor, 0xe000ed08

	.macro mov32, reg, val
	movw \reg, #:lower16:\val
	movt \reg, #:upper16:\val
	.endm

	.section .boot, "ax"

	.align	2
__InitialVectors:
	.long	__initial_sp /* Top of Stack */
	.long	BootLoader           /* Reset Handler */
	.long	InfiniteLoop /* NMI Handler */
	.long	InfiniteLoop /* Hard Fualt Handler */
	.long	InfiniteLoop /* Hard Fault Handler */
	.long	InfiniteLoop /* MPU Fault Handler */
	.long	InfiniteLoop /* Bus Fault Handler */
	.long	InfiniteLoop /* Usage Fault Handler */
	.long	0            /* Reserved */
	.long	0            /* Reserved */
	.long	0            /* Reserved */
	.long	0            /* Reserved */
	.long	InfiniteLoop /* SVCall Handler */
	.long	InfiniteLoop /* Debug Monitor Handler */
	.long	0            /* Reserved */
	.long	InfiniteLoop /* PendSV Handler */
	.long	InfiniteLoop /* SysTick Handler */

	.thumb
	.thumb_func
	.align	2
	.globl	BootLoader
	.type	BootLoader, %function
BootLoader:
	/* copy code */
	ldr	r1, =__eboot
	ldr	r2, =__code_start__
	ldr	r3, =__code_end__
.L_loop0:
	cmp	r2, r3
	ittt	lt
	ldrlt	r0, [r1], #4
	strlt	r0, [r2], #4
	blt	.L_loop0
	/* copy extab */
	ldr	r1, =__ecode
	ldr	r2, =__extab_start
	ldr	r3, =__extab_end
.L_loop1:
	cmp	r2, r3
	ittt	lt
	ldrlt	r0, [r1], #4
	strlt	r0, [r2], #4
	blt	.L_loop1
	/* copy extab */
	ldr	r1, =__eextab
	ldr	r2, =__exidx_start
	ldr	r3, =__exidx_end
.L_loop2:
	cmp	r2, r3
	ittt	lt
	ldrlt	r0, [r1], #4
	strlt	r0, [r2], #4
	blt	.L_loop2
	/* copy data */
	ldr	r1, =__etext
	ldr	r2, =__data_start__
	ldr	r3, =__data_end__
.L_loop3:
	cmp	r2, r3
	ittt	lt
	ldrlt	r0, [r1], #4
	strlt	r0, [r2], #4
	blt	.L_loop3
	/* clear bss */
	ldr	r1, =__bss_start__
	ldr	r2, =__bss_end__
	movs	r0, 0
.L_loop4:
	cmp	r1, r2
	itt	lt
	strlt	r0, [r1], #4
	blt	.L_loop4
	mov32	r0, __Vectors
	mov32	r1, vtor
	str	r0, [r1]
	mov32	r0, Reset_Handler
	bx	r0

	.pool
	.size	BootLoader, . - BootLoader

	.thumb_func
	.align	2
	.type	InfiniteLoop, %function
InfiniteLoop:
	b	.
	.end
