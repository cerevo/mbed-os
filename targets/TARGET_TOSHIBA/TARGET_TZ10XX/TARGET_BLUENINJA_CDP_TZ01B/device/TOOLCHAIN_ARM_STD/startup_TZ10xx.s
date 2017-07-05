;/**************************************************************************//**
; * @file     startup_TZ10xx.s
; * @brief    CMSIS Core Device Startup File for TZ1000 Series
; * @date     23. November 2012
; *
; * @note
; *
; ******************************************************************************/
; COPYRIGHT (C) 2015-2017
; TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
;
; This software is released under the MIT License.
; http://opensource.org/licenses/mit-license.php

;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000800

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00001C00

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; vector table @ 0x1000_0000

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
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
                DCD     FPU_IRQHandler            ; 79: BOR
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                ; switch vector table to RAM @ 0x1000_0000
                ; *VTOR = 0x1000_0000
                LDR     r0, =0x10000000
                LDR     r1, =0xe000ed08 ; VTOR
                STR     r0, [r1, #0]
                ; MSP = *(uint32_t *)(0x1000_0000)
                LDR     sp, [r0, #0]
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  GPIO_Pin0_IRQHandler      [WEAK]
                EXPORT  GPIO_Pin1_IRQHandler      [WEAK]
                EXPORT  GPIO_Pin2_IRQHandler      [WEAK]
                EXPORT  GPIO_Pin3_IRQHandler      [WEAK]
                EXPORT  GPIO_Pin4_IRQHandler      [WEAK]
                EXPORT  GPIO_Pin5_IRQHandler      [WEAK]
                EXPORT  GPIO_Pin6_IRQHandler      [WEAK]
                EXPORT  GPIO_Pin7_IRQHandler      [WEAK]
                EXPORT  GPIO_Pin8_IRQHandler      [WEAK]
                EXPORT  GPIO_Pin9_IRQHandler      [WEAK]
                EXPORT  GPIO_Pin10_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin11_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin12_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin13_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin14_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin15_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin16_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin17_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin18_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin19_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin20_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin21_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin22_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin23_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin24_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin25_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin26_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin27_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin28_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin29_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin30_IRQHandler     [WEAK]
                EXPORT  GPIO_Pin31_IRQHandler     [WEAK]
                EXPORT  SPIM0_IRQHandler          [WEAK]
                EXPORT  SPIM1_IRQHandler          [WEAK]
                EXPORT  SPIM2_IRQHandler          [WEAK]
                EXPORT  SPIM3_IRQHandler          [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  I2C1_IRQHandler           [WEAK]
                EXPORT  I2C2_IRQHandler           [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  UART2_IRQHandler          [WEAK]
                EXPORT  USB_IRQHandler            [WEAK]
                EXPORT  SRAM_M2M_IRQHandler       [WEAK]
                EXPORT  SRAM_ERR_IRQHandler       [WEAK]
                EXPORT  SPIC_IRQHandler           [WEAK]
                EXPORT  AESA_IRQHandler           [WEAK]
                EXPORT  SDMAC0_IRQHandler         [WEAK]
                EXPORT  SDMAC1_IRQHandler         [WEAK]
                EXPORT  SDMAC2_IRQHandler         [WEAK]
                EXPORT  SDMAC3_IRQHandler         [WEAK]
                EXPORT  SDMAC4_IRQHandler         [WEAK]
                EXPORT  SDMAC5_IRQHandler         [WEAK]
                EXPORT  SDMAC6_IRQHandler         [WEAK]
                EXPORT  SDMAC7_IRQHandler         [WEAK]
                EXPORT  ADCC12_IRQHandler         [WEAK]
                EXPORT  ADCC24_IRQHandler         [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  TMR0_IRQHandler           [WEAK]
                EXPORT  TMR1_IRQHandler           [WEAK]
                EXPORT  ADVTMR0_IRQHandler        [WEAK]
                EXPORT  ADVTMR1_IRQHandler        [WEAK]
                EXPORT  ADVTMR2_IRQHandler        [WEAK]
                EXPORT  ADVTMR3_IRQHandler        [WEAK]
                EXPORT  ADVTMR0_CAP_IRQHandler    [WEAK]
                EXPORT  ADVTMR1_CAP_IRQHandler    [WEAK]
                EXPORT  ADVTMR2_CAP_IRQHandler    [WEAK]
                EXPORT  ADVTMR3_CAP_IRQHandler    [WEAK]
                EXPORT  ADVTMR0_CMP_IRQHandler    [WEAK]
                EXPORT  ADVTMR1_CMP_IRQHandler    [WEAK]
                EXPORT  ADVTMR2_CMP_IRQHandler    [WEAK]
                EXPORT  ADVTMR3_CMP_IRQHandler    [WEAK]
                EXPORT  PMU_WKUP_IRQHandler       [WEAK]
                EXPORT  BOR_IRQHandler            [WEAK]
                EXPORT  FPU_IRQHandler            [WEAK]

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
                B       .

                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit
                EXPORT  Heap_Size
                EXPORT  Stack_Size
                ELSE
                EXPORT  Stack_Mem
                EXPORT  __heap_base
                EXPORT  Heap_Size
                EXPORT  Stack_Size
                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
