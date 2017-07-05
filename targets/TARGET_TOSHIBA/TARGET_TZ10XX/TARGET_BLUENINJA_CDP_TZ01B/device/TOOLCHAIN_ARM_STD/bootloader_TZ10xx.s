; A simple boot loader for TZ10xx

; COPYRIGHT (C) 2015-2017
; TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
;
; This software is released under the MIT License.
; http://opensource.org/licenses/mit-license.php

                PRESERVE8
                THUMB
#ifdef USE_TZ10XX_BOOT_FLASH
                GBLL    TZ10XX_BOOT_FLASH
TZ10XX_BOOT_FLASH           SETL    {TRUE}
#endif
ER_STARTUP_LEN  EQU     0x00000300

                ; initial vector table @ 0x0000_0000
                AREA    BOOTSTRAP, DATA, READONLY
                DCD     0x20004000                ; Top of Stack
                IF      :DEF:TZ10XX_BOOT_FLASH
                DCD     BootLoader                ; Reset Handler
                ELSE
                DCD     InfiniteLoop              ; Reset Handler
                ENDIF ; :DEF:TZ10XX_BOOT_FLASH
                DCD     InfiniteLoop              ; NMI Handler
                DCD     InfiniteLoop              ; Hard Fault Handler

                AREA    |.text|, CODE, READONLY
                IF      :DEF:TZ10XX_BOOT_FLASH
BootLoader      PROC
                ; copy ER_STARTUP section from flash to sram
                IMPORT  |Image$$ER_STARTUP$$Base|
                IMPORT  |Load$$ER_STARTUP$$Base|
                LDR     r7, =0x10000000
                LDR     r0, =|Load$$ER_STARTUP$$Base|
                LDR     r1, =|Image$$ER_STARTUP$$Base|
                LDR     r2, =ER_STARTUP_LEN
                BL      copymem
validate_firmware
                ; MSP = *(uint32_t *)(0x1000_0000)
                LDR     r2, [r7, #0]
                SUB     r2, r2, #0x20000000
                CMP     r2, #0x10000
                BLS     start_firmware
                LDR     r2, [r7, #0]
                SUB     r1, r2, #0x10000000
                CMP     r1, #0x40000
                BLS     start_firmware
copy_failsafe
                IMPORT  fwcopier_tz10xx
                IMPORT  |Image$$ER_FWCOPIER$$Base|
                IMPORT  |Load$$ER_FWCOPIER$$Length|
                LDR     r0, =fwcopier_tz10xx              ; dummy access to privent elimination of section
                LDR     r0, =|Image$$ER_FWCOPIER$$Base|
                LDR     r1, =0x20000000
                LDR     r2, =|Load$$ER_FWCOPIER$$Length|
                BL      copymem
                LDR     r0, =0x20000001
                BLX     r0
                LDR     r7, =0x10000000
                LDR     r0, =|Load$$ER_STARTUP$$Base|
                LDR     r1, =|Image$$ER_STARTUP$$Base|
                LDR     r2, =ER_STARTUP_LEN
                BL      copymem
                ; MSP = *(uint32_t *)(0x1000_0000)
                LDR     r2, [r7, #0]
                SUB     r2, r2, #0x20000000
                CMP     r2, #0x10000
                BLS     start_firmware
                LDR     r2, [r7, #0]
                SUB     r1, r2, #0x10000000
                CMP     r1, #0x40000
                BLS     start_firmware
                B       InfiniteLoop
start_firmware
                ; MSP = *(uint32_t *)(0x1000_0000)
                LDR     sp, [r7, #0]
                ; PC = *(uint32_t *)(0x1000_0004)
                LDR     r0, [r7, #4]
                BX      r0
                ENDP

copymem         PROC
                LDM     r0!, {r3-r6}
                STM     r1!, {r3-r6}
                SUBS    r2, r2, #0x10
                BHI     copymem
                BX      lr
                ENDP
                ENDIF ; :DEF:TZ10XX_BOOT_FLASH

InfiniteLoop    PROC
                B       .
                ENDP

                ALIGN

                END
