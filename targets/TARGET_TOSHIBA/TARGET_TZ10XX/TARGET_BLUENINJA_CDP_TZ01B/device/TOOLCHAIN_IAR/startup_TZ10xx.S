;/**************************************************************************//**
; * @file     startup_TZ10xx.s
; * @brief    CMSIS Core Device Startup File for TZ1000 Series
; * @version  V1.09
; * @date     09 September 2015
; *
; * @note
; *
; ******************************************************************************/
; COPYRIGHT (C) 2014-2017
; TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
;
; This software is released under the MIT License.
; http://opensource.org/licenses/mit-license.php

                MODULE  ?cstartup

                ;; Forward declaration of sections.
                SECTION CSTACK:DATA:NOROOT(3)
                SECTION .startup:CODE:ROOT(2)
                
                EXTERN  __iar_program_start
                EXTERN  SystemInit
                PUBLIC  __vector_table_ram


                DATA
__vector_table_ram

                DCD     sfe(CSTACK)
                DCD     Reset_Handler

                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     GPIO_Pin0_IRQHandler      ;  0: GPIO Pin0
                DCD     GPIO_Pin1_IRQHandler      ;  1: GPIO Pin1
                DCD     GPIO_Pin2_IRQHandler      ;  2: GPIO Pin2
                DCD     GPIO_Pin3_IRQHandler      ;  3: GPIO Pin3
                DCD     GPIO_Pin4_IRQHandler      ;  4: GPIO Pin4
                DCD     GPIO_Pin5_IRQHandler      ;  5: GPIO Pin5
                DCD     GPIO_Pin6_IRQHandler      ;  6: GPIO Pin6
                DCD     GPIO_Pin7_IRQHandler      ;  7: GPIO Pin7
                DCD     GPIO_Pin8_IRQHandler      ;  8: GPIO Pin8
                DCD     GPIO_Pin9_IRQHandler      ;  9: GPIO Pin9
                DCD     GPIO_Pin10_IRQHandler     ; 10: GPIO Pin10
                DCD     GPIO_Pin11_IRQHandler     ; 11: GPIO Pin11
                DCD     GPIO_Pin12_IRQHandler     ; 12: GPIO Pin12
                DCD     GPIO_Pin13_IRQHandler     ; 13: GPIO Pin13
                DCD     GPIO_Pin14_IRQHandler     ; 14: GPIO Pin14
                DCD     GPIO_Pin15_IRQHandler     ; 15: GPIO Pin15
                DCD     GPIO_Pin16_IRQHandler     ; 16: GPIO Pin16
                DCD     GPIO_Pin17_IRQHandler     ; 17: GPIO Pin17
                DCD     GPIO_Pin18_IRQHandler     ; 18: GPIO Pin18
                DCD     GPIO_Pin19_IRQHandler     ; 19: GPIO Pin19
                DCD     GPIO_Pin20_IRQHandler     ; 20: GPIO Pin20
                DCD     GPIO_Pin21_IRQHandler     ; 21: GPIO Pin21
                DCD     GPIO_Pin22_IRQHandler     ; 22: GPIO Pin22
                DCD     GPIO_Pin23_IRQHandler     ; 23: GPIO Pin23
                DCD     GPIO_Pin24_IRQHandler     ; 24: GPIO Pin24
                DCD     GPIO_Pin25_IRQHandler     ; 25: GPIO Pin25
                DCD     GPIO_Pin26_IRQHandler     ; 26: GPIO Pin26
                DCD     GPIO_Pin27_IRQHandler     ; 27: GPIO Pin27
                DCD     GPIO_Pin28_IRQHandler     ; 28: GPIO Pin28
                DCD     GPIO_Pin29_IRQHandler     ; 29: GPIO Pin29
                DCD     GPIO_Pin30_IRQHandler     ; 30: GPIO Pin30
                DCD     GPIO_Pin31_IRQHandler     ; 31: GPIO Pin31
                DCD     SPIM0_IRQHandler          ; 32: SPIM0
                DCD     SPIM1_IRQHandler          ; 33: SPIM1
                DCD     SPIM2_IRQHandler          ; 34: SPIM2
                DCD     SPIM3_IRQHandler          ; 35: SPIM3
                DCD     I2C0_IRQHandler           ; 36: I2C0
                DCD     I2C1_IRQHandler           ; 37: I2C1
                DCD     I2C2_IRQHandler           ; 38: I2C2
                DCD     RTC_IRQHandler            ; 39: RTC
                DCD     UART0_IRQHandler          ; 40: UART0
                DCD     UART1_IRQHandler          ; 41: UART1
                DCD     UART2_IRQHandler          ; 42: UART2
                DCD     USB_IRQHandler            ; 43: USB
                DCD     SRAM_M2M_IRQHandler       ; 44: SRAM_M2M
                DCD     SRAM_ERR_IRQHandler       ; 45: SRAM_ERR
                DCD     SPIC_IRQHandler           ; 46: SPIC
                DCD     AESA_IRQHandler           ; 47: AESA
                DCD     SDMAC0_IRQHandler         ; 48: SDMAC0
                DCD     SDMAC1_IRQHandler         ; 49: SDMAC1
                DCD     SDMAC2_IRQHandler         ; 50: SDMAC2
                DCD     SDMAC3_IRQHandler         ; 51: SDMAC3
                DCD     SDMAC4_IRQHandler         ; 52: SDMAC4
                DCD     SDMAC5_IRQHandler         ; 53: SDMAC5
                DCD     SDMAC6_IRQHandler         ; 54: SDMAC6
                DCD     SDMAC7_IRQHandler         ; 55: SDMAC7
                DCD     ADCC12_IRQHandler         ; 56: ADCC12
                DCD     ADCC24_IRQHandler         ; 57: ADCC24
                DCD     0                         ; 58: Reserved
                DCD     WDT_IRQHandler            ; 59: WDT
                DCD     TMR0_IRQHandler           ; 60: TMR0
                DCD     TMR1_IRQHandler           ; 61: TMR1
                DCD     0                         ; 62: Reserved
                DCD     0                         ; 63: Reserved
                DCD     ADVTMR0_IRQHandler        ; 64: ADVTMR0
                DCD     ADVTMR1_IRQHandler        ; 65: ADVTMR1
                DCD     ADVTMR2_IRQHandler        ; 66: ADVTMR2
                DCD     ADVTMR3_IRQHandler        ; 67: ADVTMR3
                DCD     ADVTMR0_CAP_IRQHandler    ; 68: ADVTMR0_CAP
                DCD     ADVTMR1_CAP_IRQHandler    ; 69: ADVTMR1_CAP
                DCD     ADVTMR2_CAP_IRQHandler    ; 70: ADVTMR2_CAP
                DCD     ADVTMR3_CAP_IRQHandler    ; 71: ADVTMR3_CAP
                DCD     ADVTMR0_CMP_IRQHandler    ; 72: ADVTMR0_CMP
                DCD     ADVTMR1_CMP_IRQHandler    ; 73: ADVTMR1_CMP
                DCD     ADVTMR2_CMP_IRQHandler    ; 74: ADVTMR2_CMP
                DCD     ADVTMR3_CMP_IRQHandler    ; 75: ADVTMR3_CMP
                DCD     PMU_WKUP_IRQHandler       ; 76: PMU_WKUP
                DCD     BOR_IRQHandler            ; 77: BOR
                DCD     0                         ; 78: Reserved
                DCD     FPU_IRQHandler            ; 79: FPU

                THUMB
; Dummy Exception Handlers (infinite loops which can be modified)

                PUBWEAK Reset_Handler
                SECTION .startup:CODE:REORDER:NOROOT(2)
Reset_Handler
                ; switch vector table to RAM @ 0x1000_0000
                ; *VTOR = 0x1000_0000
                LDR     r0, =0x10000000
                LDR     r1, =0xe000ed08 ; VTOR
                STR     r0, [r1, #0]
                ; MSP = *(uint32_t *)(0x1000_0000)
                LDR     sp, [r0, #0]
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__iar_program_start
                BX      R0

                PUBWEAK NMI_Handler
                SECTION .startup:CODE:REORDER:NOROOT(1)
NMI_Handler
                B       NMI_Handler

                PUBWEAK HardFault_Handler
                SECTION .startup:CODE:REORDER:NOROOT(1)
HardFault_Handler
                B       HardFault_Handler

                PUBWEAK MemManage_Handler
                SECTION .startup:CODE:REORDER:NOROOT(1)
MemManage_Handler
                B       MemManage_Handler

                PUBWEAK BusFault_Handler
                SECTION .startup:CODE:REORDER:NOROOT(1)
BusFault_Handler
                B       BusFault_Handler

                PUBWEAK UsageFault_Handler
                SECTION .startup:CODE:REORDER:NOROOT(1)
UsageFault_Handler
                B       UsageFault_Handler

                PUBWEAK SVC_Handler
                SECTION .startup:CODE:REORDER:NOROOT(1)
SVC_Handler
                B       SVC_Handler

                PUBWEAK DebugMon_Handler
                SECTION .startup:CODE:REORDER:NOROOT(1)
DebugMon_Handler
                B       DebugMon_Handler

                PUBWEAK PendSV_Handler
                SECTION .startup:CODE:REORDER:NOROOT(1)
PendSV_Handler
                B       PendSV_Handler

                PUBWEAK SysTick_Handler
                SECTION .startup:CODE:REORDER:NOROOT(1)
SysTick_Handler
                B       SysTick_Handler

                PUBWEAK GPIO_Pin0_IRQHandler
                PUBWEAK GPIO_Pin1_IRQHandler
                PUBWEAK GPIO_Pin2_IRQHandler
                PUBWEAK GPIO_Pin3_IRQHandler
                PUBWEAK GPIO_Pin4_IRQHandler
                PUBWEAK GPIO_Pin5_IRQHandler
                PUBWEAK GPIO_Pin6_IRQHandler
                PUBWEAK GPIO_Pin7_IRQHandler
                PUBWEAK GPIO_Pin8_IRQHandler
                PUBWEAK GPIO_Pin9_IRQHandler
                PUBWEAK GPIO_Pin10_IRQHandler
                PUBWEAK GPIO_Pin11_IRQHandler
                PUBWEAK GPIO_Pin12_IRQHandler
                PUBWEAK GPIO_Pin13_IRQHandler
                PUBWEAK GPIO_Pin14_IRQHandler
                PUBWEAK GPIO_Pin15_IRQHandler
                PUBWEAK GPIO_Pin16_IRQHandler
                PUBWEAK GPIO_Pin17_IRQHandler
                PUBWEAK GPIO_Pin18_IRQHandler
                PUBWEAK GPIO_Pin19_IRQHandler
                PUBWEAK GPIO_Pin20_IRQHandler
                PUBWEAK GPIO_Pin21_IRQHandler
                PUBWEAK GPIO_Pin22_IRQHandler
                PUBWEAK GPIO_Pin23_IRQHandler
                PUBWEAK GPIO_Pin24_IRQHandler
                PUBWEAK GPIO_Pin25_IRQHandler
                PUBWEAK GPIO_Pin26_IRQHandler
                PUBWEAK GPIO_Pin27_IRQHandler
                PUBWEAK GPIO_Pin28_IRQHandler
                PUBWEAK GPIO_Pin29_IRQHandler
                PUBWEAK GPIO_Pin30_IRQHandler
                PUBWEAK GPIO_Pin31_IRQHandler
                PUBWEAK SPIM0_IRQHandler
                PUBWEAK SPIM1_IRQHandler
                PUBWEAK SPIM2_IRQHandler
                PUBWEAK SPIM3_IRQHandler
                PUBWEAK I2C0_IRQHandler
                PUBWEAK I2C1_IRQHandler
                PUBWEAK I2C2_IRQHandler
                PUBWEAK RTC_IRQHandler
                PUBWEAK UART0_IRQHandler
                PUBWEAK UART1_IRQHandler
                PUBWEAK UART2_IRQHandler
                PUBWEAK USB_IRQHandler
                PUBWEAK SRAM_M2M_IRQHandler
                PUBWEAK SRAM_ERR_IRQHandler
                PUBWEAK SPIC_IRQHandler
                PUBWEAK AESA_IRQHandler
                PUBWEAK SDMAC0_IRQHandler
                PUBWEAK SDMAC1_IRQHandler
                PUBWEAK SDMAC2_IRQHandler
                PUBWEAK SDMAC3_IRQHandler
                PUBWEAK SDMAC4_IRQHandler
                PUBWEAK SDMAC5_IRQHandler
                PUBWEAK SDMAC6_IRQHandler
                PUBWEAK SDMAC7_IRQHandler
                PUBWEAK ADCC12_IRQHandler
                PUBWEAK ADCC24_IRQHandler
                PUBWEAK WDT_IRQHandler
                PUBWEAK TMR0_IRQHandler
                PUBWEAK TMR1_IRQHandler
                PUBWEAK ADVTMR0_IRQHandler
                PUBWEAK ADVTMR1_IRQHandler
                PUBWEAK ADVTMR2_IRQHandler
                PUBWEAK ADVTMR3_IRQHandler
                PUBWEAK ADVTMR0_CAP_IRQHandler
                PUBWEAK ADVTMR1_CAP_IRQHandler
                PUBWEAK ADVTMR2_CAP_IRQHandler
                PUBWEAK ADVTMR3_CAP_IRQHandler
                PUBWEAK ADVTMR0_CMP_IRQHandler
                PUBWEAK ADVTMR1_CMP_IRQHandler
                PUBWEAK ADVTMR2_CMP_IRQHandler
                PUBWEAK ADVTMR3_CMP_IRQHandler
                PUBWEAK PMU_WKUP_IRQHandler
                PUBWEAK BOR_IRQHandler
                PUBWEAK FPU_IRQHandler

GPIO_Pin0_IRQHandler
GPIO_Pin1_IRQHandler
GPIO_Pin2_IRQHandler
GPIO_Pin3_IRQHandler
GPIO_Pin4_IRQHandler
GPIO_Pin5_IRQHandler
GPIO_Pin6_IRQHandler
GPIO_Pin7_IRQHandler
GPIO_Pin8_IRQHandler
GPIO_Pin9_IRQHandler
GPIO_Pin10_IRQHandler
GPIO_Pin11_IRQHandler
GPIO_Pin12_IRQHandler
GPIO_Pin13_IRQHandler
GPIO_Pin14_IRQHandler
GPIO_Pin15_IRQHandler
GPIO_Pin16_IRQHandler
GPIO_Pin17_IRQHandler
GPIO_Pin18_IRQHandler
GPIO_Pin19_IRQHandler
GPIO_Pin20_IRQHandler
GPIO_Pin21_IRQHandler
GPIO_Pin22_IRQHandler
GPIO_Pin23_IRQHandler
GPIO_Pin24_IRQHandler
GPIO_Pin25_IRQHandler
GPIO_Pin26_IRQHandler
GPIO_Pin27_IRQHandler
GPIO_Pin28_IRQHandler
GPIO_Pin29_IRQHandler
GPIO_Pin30_IRQHandler
GPIO_Pin31_IRQHandler
SPIM0_IRQHandler
SPIM1_IRQHandler
SPIM2_IRQHandler
SPIM3_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
I2C2_IRQHandler
RTC_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
UART2_IRQHandler
USB_IRQHandler
SRAM_M2M_IRQHandler
SRAM_ERR_IRQHandler
SPIC_IRQHandler
AESA_IRQHandler
SDMAC0_IRQHandler
SDMAC1_IRQHandler
SDMAC2_IRQHandler
SDMAC3_IRQHandler
SDMAC4_IRQHandler
SDMAC5_IRQHandler
SDMAC6_IRQHandler
SDMAC7_IRQHandler
ADCC12_IRQHandler
ADCC24_IRQHandler
WDT_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
ADVTMR0_IRQHandler
ADVTMR1_IRQHandler
ADVTMR2_IRQHandler
ADVTMR3_IRQHandler
ADVTMR0_CAP_IRQHandler
ADVTMR1_CAP_IRQHandler
ADVTMR2_CAP_IRQHandler
ADVTMR3_CAP_IRQHandler
ADVTMR0_CMP_IRQHandler
ADVTMR1_CMP_IRQHandler
ADVTMR2_CMP_IRQHandler
ADVTMR3_CMP_IRQHandler
PMU_WKUP_IRQHandler
BOR_IRQHandler
FPU_IRQHandler
		B .


#if 0
#ifdef TZ10XX_BOOT_FLASH
; --------------------
; Dummy handler placed in ROM
Dummy_Handler_ROM
                B Dummy_Handler_ROM
#endif ;; TZ10XX_BOOT_FLASH
#endif    
 
                END
