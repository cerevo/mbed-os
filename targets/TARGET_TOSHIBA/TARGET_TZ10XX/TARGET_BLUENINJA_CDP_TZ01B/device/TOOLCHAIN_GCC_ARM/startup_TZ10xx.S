/*
// COPYRIGHT (C) 2017
// TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
//
// This software is released under the MIT License.
// http://opensource.org/licenses/mit-license.php
 */

	.equ Stack_Size,0x00000800

/*
// <h> Heap Configuration
//   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
// </h>
 */

	.equ Heap_Size,0x00001C00

	.syntax	unified
	.arch	armv7-m

	.section .stack, "w"
	.align	3
	.globl	__initial_sp
	.globl	__StackLimit
__stack_limit:
	.space	Stack_Size
__initial_sp:

	.section .heap, "w"
	.align	3
	.globl	__heap_base
	.globl	__heap_limit
__heap_base:
	.ifne	Heap_Size
	.space	Heap_Size
	.endif
__heap_limit:


	.section .vectors
	.align	2
	.globl	__Vectors
__Vectors:
	.long	__initial_sp          /* Top of Stack */
	.long	Reset_Handler         /* Reset Handler */
	.long	NMI_Handler           /* NMI Handler */
	.long	HardFault_Handler     /* Hard Fault Handler */
	.long	MemManage_Handler     /* MPU Fault Handler */
	.long	BusFault_Handler      /* Bus Fault Handler */
	.long	UsageFault_Handler    /* Usage Fault Handler */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	0                     /* Reserved */
	.long	SVC_Handler           /* SVCall Handler */
	.long	DebugMon_Handler      /* Debug Monitor Handler */
	.long	0                     /* Reserved */
	.long	PendSV_Handler        /* PendSV Handler */
	.long	SysTick_Handler       /* SysTick Handler */

	/* External interrupts */
	.long	GPIO_Pin0_IRQHandler    /*  0: GPIO Pin0    */
	.long	GPIO_Pin1_IRQHandler    /*  1: GPIO Pin1    */
	.long	GPIO_Pin2_IRQHandler    /*  2: GPIO Pin2    */
	.long	GPIO_Pin3_IRQHandler    /*  3: GPIO Pin3    */
	.long	GPIO_Pin4_IRQHandler    /*  4: GPIO Pin4    */
	.long	GPIO_Pin5_IRQHandler    /*  5: GPIO Pin5    */
	.long	GPIO_Pin6_IRQHandler    /*  6: GPIO Pin6    */
	.long	GPIO_Pin7_IRQHandler    /*  7: GPIO Pin7    */
	.long	GPIO_Pin8_IRQHandler    /*  8: GPIO Pin8    */
	.long	GPIO_Pin9_IRQHandler    /*  9: GPIO Pin9    */
	.long	GPIO_Pin10_IRQHandler   /* 10: GPIO Pin10   */
	.long	GPIO_Pin11_IRQHandler   /* 11: GPIO Pin11   */
	.long	GPIO_Pin12_IRQHandler   /* 12: GPIO Pin12   */
	.long	GPIO_Pin13_IRQHandler   /* 13: GPIO Pin13   */
	.long	GPIO_Pin14_IRQHandler   /* 14: GPIO Pin14   */
	.long	GPIO_Pin15_IRQHandler   /* 15: GPIO Pin15   */
	.long	GPIO_Pin16_IRQHandler   /* 16: GPIO Pin16   */
	.long	GPIO_Pin17_IRQHandler   /* 17: GPIO Pin17   */
	.long	GPIO_Pin18_IRQHandler   /* 18: GPIO Pin18   */
	.long	GPIO_Pin19_IRQHandler   /* 19: GPIO Pin19   */
	.long	GPIO_Pin20_IRQHandler   /* 20: GPIO Pin20   */
	.long	GPIO_Pin21_IRQHandler   /* 21: GPIO Pin21   */
	.long	GPIO_Pin22_IRQHandler   /* 22: GPIO Pin22   */
	.long	GPIO_Pin23_IRQHandler   /* 23: GPIO Pin23   */
	.long	GPIO_Pin24_IRQHandler   /* 24: GPIO Pin24   */
	.long	GPIO_Pin25_IRQHandler   /* 25: GPIO Pin25   */
	.long	GPIO_Pin26_IRQHandler   /* 26: GPIO Pin26   */
	.long	GPIO_Pin27_IRQHandler   /* 27: GPIO Pin27   */
	.long	GPIO_Pin28_IRQHandler   /* 28: GPIO Pin28   */
	.long	GPIO_Pin29_IRQHandler   /* 29: GPIO Pin29   */
	.long	GPIO_Pin30_IRQHandler   /* 30: GPIO Pin30   */
	.long	GPIO_Pin31_IRQHandler   /* 31: GPIO Pin31   */
	.long	SPIM0_IRQHandler        /* 32: SPIM0        */
	.long	SPIM1_IRQHandler        /* 33: SPIM1        */
	.long	SPIM2_IRQHandler        /* 34: SPIM2        */
	.long	SPIM3_IRQHandler        /* 35: SPIM3        */
	.long	I2C0_IRQHandler         /* 36: I2C0         */
	.long	I2C1_IRQHandler         /* 37: I2C1         */
	.long	I2C2_IRQHandler         /* 38: I2C2         */
	.long	RTC_IRQHandler          /* 39: RTC          */
	.long	UART0_IRQHandler        /* 40: UART0        */
	.long	UART1_IRQHandler        /* 41: UART1        */
	.long	UART2_IRQHandler        /* 42: UART2        */
	.long	USB_IRQHandler          /* 43: USB          */
	.long	SRAM_M2M_IRQHandler     /* 44: SRAM M2M     */
	.long	SRAM_ERR_IRQHandler     /* 45: SRAM ERR     */
	.long	SPIC_IRQHandler         /* 46: SPIC         */
	.long	AESA_IRQHandler         /* 47: AESA         */
	.long	SDMAC0_IRQHandler       /* 48: SDMAC0       */
	.long	SDMAC1_IRQHandler       /* 49: SDMAC1       */
	.long	SDMAC2_IRQHandler       /* 50: SDMAC2       */
	.long	SDMAC3_IRQHandler       /* 51: SDMAC3       */
	.long	SDMAC4_IRQHandler       /* 52: SDMAC4       */
	.long	SDMAC5_IRQHandler       /* 53: SDMAC5       */
	.long	SDMAC6_IRQHandler       /* 54: SDMAC6       */
	.long	SDMAC7_IRQHandler       /* 55: SDMAC7       */
	.long	ADCC12_IRQHandler       /* 56: ADCC12       */
	.long	ADCC24_IRQHandler       /* 57: ADCC24       */
	.long	0                       /* 58: Reserved     */
	.long	WDT_IRQHandler          /* 59: WDT          */
	.long	TMR0_IRQHandler         /* 60: TMR0         */
	.long	TMR1_IRQHandler         /* 61: TMR1         */
	.long	0                       /* 62: Reserved     */
	.long	0                       /* 63: Reserved     */
	.long	ADVTMR0_IRQHandler      /* 64: ADVTMR0      */
	.long	ADVTMR1_IRQHandler      /* 65: ADVTMR1      */
	.long	ADVTMR2_IRQHandler      /* 66: ADVTMR2      */
	.long	ADVTMR3_IRQHandler      /* 67: ADVTMR3      */
	.long	ADVTMR0_CAP_IRQHandler  /* 68: ADVTMR0_CAP  */
	.long	ADVTMR1_CAP_IRQHandler  /* 69: ADVTMR1_CAP  */
	.long	ADVTMR2_CAP_IRQHandler  /* 70: ADVTMR2_CAP  */
	.long	ADVTMR3_CAP_IRQHandler  /* 71: ADVTMR3_CAP  */
	.long	ADVTMR0_CMP_IRQHandler  /* 72: ADVTMR0_CMP  */
	.long	ADVTMR1_CMP_IRQHandler  /* 73: ADVTMR1_CMP  */
	.long	ADVTMR2_CMP_IRQHandler  /* 74: ADVTMR2_CMP  */
	.long	ADVTMR3_CMP_IRQHandler  /* 75: ADVTMR3_CMP  */
	.long	PMU_WKUP_IRQHandler     /* 76: PMU_WKUP     */
	.long	BOR_IRQHandler          /* 77: BOR          */
	.long	0                       /* 78: Reserved     */
	.long	FPU_IRQHandler          /* 79: FPU          */
__Vectors_End:
	.equ	__Vectors_Size, __Vectors_End - __Vectors

	.thumb

	.section .startup, "x"

	.macro mov32, reg, val
	movw \reg, #:lower16:\val
	movt \reg, #:upper16:\val
	.endm

	.equ	vtor, 0xe000ed08

	.thumb_func
	.align	2
	.globl	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
	mov32	r0, __Vectors
	mov32	r1, vtor
	str	r0, [r1]
	bl	SystemInit
	bl	_start

	.pool
	.size	Reset_Handler, . - Reset_Handler

	.text

	.align	1
	.thumb_func
	.weak	Default_Handler
	.type	Default_Handler, %function
Default_Handler:
	b	.
	.size	Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
	.macro	def_irq_handler	handler_name
	.weak	\handler_name
	.set	\handler_name, Default_Handler
	.endm

	def_irq_handler	NMI_Handler
	def_irq_handler	HardFault_Handler
	def_irq_handler	MemManage_Handler
	def_irq_handler	BusFault_Handler
	def_irq_handler	UsageFault_Handler
	def_irq_handler	SVC_Handler
	def_irq_handler	DebugMon_Handler
	def_irq_handler	PendSV_Handler
	def_irq_handler	SysTick_Handler

	def_irq_handler	GPIO_Pin0_IRQHandler
	def_irq_handler	GPIO_Pin1_IRQHandler
	def_irq_handler	GPIO_Pin2_IRQHandler
	def_irq_handler	GPIO_Pin3_IRQHandler
	def_irq_handler	GPIO_Pin4_IRQHandler
	def_irq_handler	GPIO_Pin5_IRQHandler
	def_irq_handler	GPIO_Pin6_IRQHandler
	def_irq_handler	GPIO_Pin7_IRQHandler
	def_irq_handler	GPIO_Pin8_IRQHandler
	def_irq_handler	GPIO_Pin9_IRQHandler
	def_irq_handler	GPIO_Pin10_IRQHandler
	def_irq_handler	GPIO_Pin11_IRQHandler
	def_irq_handler	GPIO_Pin12_IRQHandler
	def_irq_handler	GPIO_Pin13_IRQHandler
	def_irq_handler	GPIO_Pin14_IRQHandler
	def_irq_handler	GPIO_Pin15_IRQHandler
	def_irq_handler	GPIO_Pin16_IRQHandler
	def_irq_handler	GPIO_Pin17_IRQHandler
	def_irq_handler	GPIO_Pin18_IRQHandler
	def_irq_handler	GPIO_Pin19_IRQHandler
	def_irq_handler	GPIO_Pin20_IRQHandler
	def_irq_handler	GPIO_Pin21_IRQHandler
	def_irq_handler	GPIO_Pin22_IRQHandler
	def_irq_handler	GPIO_Pin23_IRQHandler
	def_irq_handler	GPIO_Pin24_IRQHandler
	def_irq_handler	GPIO_Pin25_IRQHandler
	def_irq_handler	GPIO_Pin26_IRQHandler
	def_irq_handler	GPIO_Pin27_IRQHandler
	def_irq_handler	GPIO_Pin28_IRQHandler
	def_irq_handler	GPIO_Pin29_IRQHandler
	def_irq_handler	GPIO_Pin30_IRQHandler
	def_irq_handler	GPIO_Pin31_IRQHandler
	def_irq_handler	SPIM0_IRQHandler
	def_irq_handler	SPIM1_IRQHandler
	def_irq_handler	SPIM2_IRQHandler
	def_irq_handler	SPIM3_IRQHandler
	def_irq_handler	I2C0_IRQHandler
	def_irq_handler	I2C1_IRQHandler
	def_irq_handler	I2C2_IRQHandler
	def_irq_handler	RTC_IRQHandler
	def_irq_handler	UART0_IRQHandler
	def_irq_handler	UART1_IRQHandler
	def_irq_handler	UART2_IRQHandler
	def_irq_handler	USB_IRQHandler
	def_irq_handler	SRAM_M2M_IRQHandler
	def_irq_handler	SRAM_ERR_IRQHandler
	def_irq_handler	SPIC_IRQHandler
	def_irq_handler	AESA_IRQHandler
	def_irq_handler	SDMAC0_IRQHandler
	def_irq_handler	SDMAC1_IRQHandler
	def_irq_handler	SDMAC2_IRQHandler
	def_irq_handler	SDMAC3_IRQHandler
	def_irq_handler	SDMAC4_IRQHandler
	def_irq_handler	SDMAC5_IRQHandler
	def_irq_handler	SDMAC6_IRQHandler
	def_irq_handler	SDMAC7_IRQHandler
	def_irq_handler	ADCC12_IRQHandler
	def_irq_handler	ADCC24_IRQHandler
	def_irq_handler	WDT_IRQHandler
	def_irq_handler	TMR0_IRQHandler
	def_irq_handler	TMR1_IRQHandler
	def_irq_handler	ADVTMR0_IRQHandler
	def_irq_handler	ADVTMR1_IRQHandler
	def_irq_handler	ADVTMR2_IRQHandler
	def_irq_handler	ADVTMR3_IRQHandler
	def_irq_handler	ADVTMR0_CAP_IRQHandler
	def_irq_handler	ADVTMR1_CAP_IRQHandler
	def_irq_handler	ADVTMR2_CAP_IRQHandler
	def_irq_handler	ADVTMR3_CAP_IRQHandler
	def_irq_handler	ADVTMR0_CMP_IRQHandler
	def_irq_handler	ADVTMR1_CMP_IRQHandler
	def_irq_handler	ADVTMR2_CMP_IRQHandler
	def_irq_handler	ADVTMR3_CMP_IRQHandler
	def_irq_handler	PMU_WKUP_IRQHandler
	def_irq_handler	BOR_IRQHandler
	def_irq_handler	FPU_IRQHandler

	.end
