/*
 * Copyright (c) 2008-2017 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVINCED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

        .arch armv7-m
        .syntax unified

        .section .stack, "aw"
        .align  3
#ifdef __STACK_SIZE
        .space  __STACK_SIZE
#else
        .space  4096
#endif

        .section .vectors
        .align  2
        .globl  __Vectors
__Vectors:
        .long   0xbe00be00            /* Top of Stack */
        .long   Reset_Handler         /* Reset Handler */
        .long   NMI_Handler           /* NMI Handler */
        .long   HardFault_Handler     /* Hard Fault Handler */
        .long   MemManage_Handler     /* MPU Fault Handler */
        .long   BusFault_Handler      /* Bus Fault Handler */
        .long   UsageFault_Handler    /* Usage Fault Handler */
        .long   0                     /* Reserved */
        .long   0                     /* Reserved */
        .long   0                     /* Reserved */
        .long   0                     /* Reserved */
        .long   SVC_Handler           /* SVCall Handler */
        .long   DebugMon_Handler      /* Debug Monitor Handler */
        .long   0                     /* Reserved */
        .long   PendSV_Handler        /* PendSV Handler */
        .long   SysTick_Handler       /* SysTick Handler */

        /* External interrupts */
        .long   WWDG_IRQHandler
        .long   PVD_PVM_IRQHandler
        .long   TAMP_STAMP_LSECSS_IRQHandler
        .long   RTC_WKUP_IRQHandler
        .long   FLASH_IRQHandler
        .long   RCC_IRQHandler
        .long   EXTI0_IRQHandler
        .long   EXTI1_IRQHandler
        .long   EXTI2_IRQHandler
        .long   EXTI3_IRQHandler
        .long   EXTI4_IRQHandler
        .long   DMA1_Channel1_IRQHandler
        .long   DMA1_Channel2_IRQHandler
        .long   DMA1_Channel3_IRQHandler
        .long   DMA1_Channel4_IRQHandler
        .long   DMA1_Channel5_IRQHandler
        .long   DMA1_Channel6_IRQHandler
        .long   DMA1_Channel7_IRQHandler
        .long   ADC1_IRQHandler
        .long   USB_HP_IRQHandler
        .long   USB_LP_IRQHandler
        .long   C2SEV_PWR_C2H_IRQHandler
        .long   COMP_IRQHandler
        .long   EXTI9_5_IRQHandler
        .long   TIM1_BRK_IRQHandler
        .long   TIM1_UP_TIM16_IRQHandler
        .long   TIM1_TRG_COM_TIM17_IRQHandler
        .long   TIM1_CC_IRQHandler
        .long   TIM2_IRQHandler
        .long   PKA_IRQHandler
        .long   I2C1_EV_IRQHandler
        .long   I2C1_ER_IRQHandler
        .long   I2C3_EV_IRQHandler
        .long   I2C3_ER_IRQHandler
        .long   SPI1_IRQHandler
        .long   SPI2_IRQHandler
        .long   USART1_IRQHandler
        .long   LPUART1_IRQHandler
        .long   SAI1_IRQHandler
        .long   TSC_IRQHandler
        .long   EXTI15_10_IRQHandler
        .long   RTC_Alarm_IRQHandler
        .long   CRS_IRQHandler
        .long   PWR_SOTF_BLEACT_802ACT_RFPHASE_IRQHandler
        .long   IPCC_C1_RX_IRQHandler
        .long   IPCC_C1_TX_IRQHandler
        .long   HSEM_IRQHandler
        .long   LPTIM1_IRQHandler
        .long   LPTIM2_IRQHandler
        .long   LCD_IRQHandler
        .long   QUADSPI_IRQHandler
        .long   AES1_IRQHandler
        .long   AES2_IRQHandler
        .long   RNG_IRQHandler
        .long   FPU_IRQHandler
        .long   DMA2_Channel1_IRQHandler
        .long   DMA2_Channel2_IRQHandler
        .long   DMA2_Channel3_IRQHandler
        .long   DMA2_Channel4_IRQHandler
        .long   DMA2_Channel5_IRQHandler
        .long   DMA2_Channel6_IRQHandler
        .long   DMA2_Channel7_IRQHandler
        .long   DMAMUX1_OVR_IRQHandler

        .size   __Vectors, . - __Vectors


        .text
        .thumb
        .thumb_func
        .align  2
        .globl  Reset_Handler
        .type   Reset_Handler, %function
Reset_Handler:
        ldr     r0, =__StackTop
        mov     sp, r0
        bl      SystemInit

        /*
         *  The __copy_table has the following format per entry:
         *
         *    offset 0: LMA of start of a DATA section to copy from
         *    offset 4: VMA of start of a DATA section to copy to
         *    offset 8: Size of this DATA sectiony. Must be multiply of 4
         *
         *  All addresses must be aligned to 4 bytes boundary.
         */

        ldr     r1, =__copy_table_start__
        ldr     r2, =__copy_table_end__
        b.n     3f
1:      ldmia   r1!, { r3, r4, r5 }
        cbz.n   r5, 3f
2:      subs    r5, #4
        ldr     r0, [r3, r5]
        str     r0, [r4, r5]
        bne.n   2b
3:      cmp     r1, r2
        bne.n   1b

        /*
         *  The __zero_table has the following format per entry:
         *
         *    offset 0: VMA of start of a BSS section to zero
         *    offset 4: Size of this BSS section. Must be multiply of 4
         *
         *  All addresses must be aligned to 4 bytes boundary.
         */

        movs    r0, #0
        ldr     r1, =__zero_table_start__
        ldr     r2, =__zero_table_end__
        b.n     6f
4:      ldmia   r1!, { r3, r4 }
        cbz.n   r4, 6f
5:      subs    r4, #4
        str     r0, [r3, r4]
        bne.n   5b
6:      cmp     r1, r2
        bne.n   4b
        
        /* ##### default starts is "_start" */
#ifndef __START
#define __START main
#endif
        b       __START

        .size   Reset_Handler, . - Reset_Handler
