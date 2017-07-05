; A simple boot loader for TZ10xx

; COPYRIGHT (C) 2015-2017
; TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
;
; This software is released under the MIT License.
; http://opensource.org/licenses/mit-license.php

#ifdef USE_TZ10XX_BOOT_FLASH
#define TZ10XX_BOOT_FLASH
#endif
                MODULE  ?cstartup

                EXTERN  fwcopier_tz10xx
                ; initial vector table @ 0x0000_0000
                SECTION CSTACK:DATA:NOROOT(3)
                SECTION .startup:CODE:ROOT(2)
                SECTION .startup_init:CODE:ROOT(2)
                SECTION .fwcopier:CODE:ROOT(2)                
                
                SECTION .intvec:CODE:ROOT(2)
                
                PUBLIC __vector_table

                DATA
__vector_table
                DCD     0x20004000                ; Top of Stack
#ifdef TZ10XX_BOOT_FLASH
                DCD     BootLoader                ; Reset Handler
#else
                DCD     InfiniteLoop              ; Reset Handler
#endif ;; TZ10XX_BOOT_FLASH
                DCD     InfiniteLoop              ; NMI Handler
                DCD     InfiniteLoop              ; Hard Fault Handler
            
          

                ; copy ER_STARTUP section from flash to sram
#ifdef  TZ10XX_BOOT_FLASH
                CODE
                
BootLoader
                LDR     r7, =0x10000000
                LDR     r0, =sfb(.startup_init)
                LDR     r1, =sfb(.startup)
                LDR     r2, =sizeof(.startup)                
                BL      copymem
copy_section_rodata
                EXTERN  RODATA_RAM_INIT$$Base
                EXTERN  RODATA_RAM$$Length
                EXTERN  RODATA_RAM$$Base
                LDR     r0, =RODATA_RAM_INIT$$Base
                LDR     r1, =RODATA_RAM$$Base
                LDR     r2, =RODATA_RAM$$Length
                BL      copymem
copy_section_text
                EXTERN  TEXT_RAM_INIT$$Base
                EXTERN  TEXT_RAM$$Length
                EXTERN  TEXT_RAM$$Base
                LDR     r0, =TEXT_RAM_INIT$$Base
                LDR     r1, =TEXT_RAM$$Base
                LDR     r2, =TEXT_RAM$$Length
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
                LDR     r0, =fwcopier_tz10xx         
                LDR     r0, =sfb(.fwcopier)                
                LDR     r1, =0x20000000
                LDR     r2, =sizeof(.fwcopier)
                BL      copymem
                LDR     r0, =0x20000001
                BLX     r0
                LDR     r7, =0x10000000
                LDR     r0, =sfb(.startup_init)
                LDR     r1, =sfb(.startup)
                LDR     r2, =sizeof(.startup)
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
copymem
                LDM     r0!, {r3-r6}
                STM     r1!, {r3-r6}
                SUBS    r2, r2, #0x10
                BHI     copymem
                BX      lr
#endif  ;; TZ10XX_BOOT_FLASH

InfiniteLoop
                B       .

                END
