/*
 * Copyright (c) 2014-2021 Thomas Roell.  All rights reserved.
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "armv7m.h"
#include "stm32wb_system.h"

typedef struct _armv7m_core_control_t {
    uint32_t                         udelay_scale;
} armv7m_core_control_t;

static __attribute__((section(".noinit"))) armv7m_core_control_t armv7m_core_control;

void __armv7m_core_initialize(void)
{
    armv7m_core_control.udelay_scale = SystemCoreClock / 15625;

    NVIC_SetPriority(MemManage_IRQn, ARMV7M_IRQ_PRIORITY_MEMMANAGE);
    NVIC_SetPriority(BusFault_IRQn, ARMV7M_IRQ_PRIORITY_BUSFAULT);
    NVIC_SetPriority(UsageFault_IRQn, ARMV7M_IRQ_PRIORITY_USAGEFAULT);
    NVIC_SetPriority(DebugMon_IRQn, ARMV7M_IRQ_PRIORITY_DEBUGMON);

    // SCB->CCR = 0;
    SCB->CCR = SCB_CCR_STKALIGN_Msk | SCB_CCR_DIV_0_TRP_Msk;
    SCB->SHCSR = (SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk);
#if (__FPU_PRESENT == 1)
    SCB->CPACR = 0x00f00000;
#endif /* __FPU_PRESENT == 1 */

    __DSB();
    
    __armv7m_pendsv_initialize();
    __armv7m_svcall_initialize();
    __armv7m_rtt_initialize(); 
    __armv7m_systick_initialize();
}

void armv7m_core_configure(void)
{
    armv7m_core_control.udelay_scale = SystemCoreClock / 15625;
}

void armv7m_core_udelay(uint32_t delay)
{
    uint32_t n;

    n = (delay * armv7m_core_control.udelay_scale + 255) / 256;

    __asm__ __volatile__(
                         "1: subs %0, #1 \n"
                         "   nop         \n"
                         "   bne  1b     \n"
                         : "+r" (n));
}

/* The format of a member pointer is defined in the C++ ABI.
 * For ARM that document is IHI0041D, which describes the differences
 * to the Itanium C++ ABI.
 */

void armv7m_core_cxx_method(const void *method, const void *object, armv7m_core_callback_t *p_callback_return, void **p_context_return)
{
    void *ptr = (void*)(((const uint32_t*)method)[0]);
    ptrdiff_t adj = ((ptrdiff_t)(((const uint32_t*)method)[1]) >> 1);

    if (!((((const uint32_t*)method)[1]) & 1))
    {
        /* non-virtual function */
        *p_callback_return = ptr;
    }
    else
    {
        /* virtual function */
        void *vptr = *((void**)((uintptr_t)object + adj)); 
        *p_callback_return = *((void**)((uint8_t*)vptr + (ptrdiff_t)ptr));
    }

    *p_context_return = (void*)((uintptr_t)object + adj);
}

__SECTION_FATAL __attribute__((used)) armv7m_assert_info_t armv7m_assert_info;

__attribute__((naked)) void __aeabi_assert(const char *assertion, const char *file, uint32_t line)
{
    __asm__(
        "    push    { r0, r1, r3, lr }                        \n"
        "    .cfi_def_cfa_offset 16                            \n"
        "    .cfi_offset  0, -16                               \n"
        "    .cfi_offset  1, -12                               \n"
        "    .cfi_offset  3, -8                                \n"
        "    .cfi_offset 14, -4                                \n"
	"    ldr     r3, =armv7m_assert_info                   \n"
        "    str     r0, [r3, #0x00]                           \n"
        "    str     r1, [r3, #0x04]                           \n"
        "    str     r2, [r3, #0x08]                           \n"
        "    movs    r0, %[const_RESET_CAUSE_ASSERT]           \n"
	"    mov     r1, #0                                    \n"
        "    bl      stm32wb_system_fatal                      \n"
        :
        : [const_RESET_CAUSE_ASSERT] "I" (STM32WB_SYSTEM_RESET_CAUSE_ASSERT)
        );
}

__attribute__((naked)) void __assert_func(const char *file, int line, const char *function, const char *assertion)
{
    __asm__(
        "    push    { r0, r1, r2, lr }                        \n"
        "    .cfi_def_cfa_offset 16                            \n"
        "    .cfi_offset  0, -16                               \n"
        "    .cfi_offset  1, -12                               \n"
        "    .cfi_offset  2, -8                                \n"
        "    .cfi_offset 14, -4                                \n"
	"    ldr     r2, =armv7m_assert_info                   \n"
        "    str     r3, [r2, #0x00]                           \n"
        "    str     r0, [r2, #0x04]                           \n"
        "    str     r1, [r2, #0x08]                           \n"
        "    movs    r0, %[const_RESET_CAUSE_ASSERT]           \n"
	"    mov     r1, #0                                    \n"
        "    bl      stm32wb_system_fatal                      \n"
        :
        : [const_RESET_CAUSE_ASSERT] "I" (STM32WB_SYSTEM_RESET_CAUSE_ASSERT)
        );
}

__attribute__((naked)) void __assert(const char *file, uint32_t line, const char *assertion)
{
    __asm__(
        "    push    { r0, r1, r3, lr }                        \n"
        "    .cfi_def_cfa_offset 16                            \n"
        "    .cfi_offset  0, -16                               \n"
        "    .cfi_offset  1, -12                               \n"
        "    .cfi_offset  3, -8                                \n"
        "    .cfi_offset 14, -4                                \n"
	"    ldr     r3, =armv7m_assert_info                   \n"
        "    str     r2, [r3, #0x00]                           \n"
        "    str     r0, [r3, #0x04]                           \n"
        "    str     r1, [r3, #0x08]                           \n"
        "    movs    r0, %[const_RESET_CAUSE_ASSERT]           \n"
	"    mov     r1, #0                                    \n"
        "    bl      stm32wb_system_fatal                      \n"
        :
        : [const_RESET_CAUSE_ASSERT] "I" (STM32WB_SYSTEM_RESET_CAUSE_ASSERT)
        );
}

__SECTION_FATAL __attribute__((used)) armv7m_fault_info_t armv7m_fault_info;

static __attribute__((naked)) void armv7m_core_exception(void)
{
    __asm__(
        "    mov      r2, sp                                   \n"
        "    mov      r3, lr                                   \n"
        "    lsrs     r0, r3, #(2+1)                           \n" // bit 2 is SPSEL
	"    bcc      1f                                       \n"
        "    mrs      r2, PSP                                  \n"
        "1:  push     { r2, r3 }                               \n"
        "   .cfi_def_cfa_offset 8                              \n"
        "   .cfi_offset 14, -4                                 \n"
        "    ldr      r1, =armv7m_fault_info                   \n"
        "    ldr      r0, [r2, #0x00]                          \n" // R0
        "    str      r0, [r1, #0x00]                          \n"
        "    ldr      r0, [r2, #0x04]                          \n" // R1
        "    str      r0, [r1, #0x04]                          \n"
        "    ldr      r0, [r2, #0x08]                          \n" // R2
        "    str      r0, [r1, #0x08]                          \n"
        "    ldr      r0, [r2, #0x0c]                          \n" // R3
        "    str      r0, [r1, #0x0c]                          \n"
        "    str      r4, [r1, #0x10]                          \n" // R4
        "    str      r5, [r1, #0x14]                          \n" // R5
        "    str      r6, [r1, #0x18]                          \n" // R6
        "    str      r7, [r1, #0x1c]                          \n" // R7
        "    mov      r0, r8                                   \n" // R8
        "    str      r0, [r1, #0x20]                          \n"
        "    mov      r0, r9                                   \n" // R9
        "    str      r0, [r1, #0x24]                          \n"
        "    mov      r0, r10                                  \n" // R10
        "    str      r0, [r1, #0x28]                          \n"
        "    mov      r0, r11                                  \n" // R11
        "    str      r0, [r1, #0x2c]                          \n"
        "    ldr      r0, [r2, #0x10]                          \n" // R12
        "    str      r0, [r1, #0x30]                          \n"
        "    ldr      r0, [r2, #0x14]                          \n" // LR
        "    str      r0, [r1, #0x38]                          \n"
        "    ldr      r0, [r2, #0x18]                          \n" // PC
        "    str      r0, [r1, #0x3c]                          \n"
        "    ldr      r0, [r2, #0x1c]                          \n" // XPSR
        "    str      r0, [r1, #0x40]                          \n"
        "    lsrs     r0, r0, #(9+1)                           \n" // bit 9 is filler
        "    bcc      2f                                       \n"
        "    adds     r2, #0x04                                \n"
        "2:  adds     r2, #0x20                                \n"
#if (__FPU_PRESENT == 1)
        "    lsrs     r0, r3, #(4+1)                           \n" // bit 4 is nFPCA
        "    bcs      3f                                       \n"
        "    adds     r2, #(0x68-0x20)                         \n"
#endif /* __FPU_PRESENT == 1 */
        "3:  str      r2, [r1, #0x34]                          \n"
        "    ldr      r3, =0xe000ed00                          \n"
        "    ldr      r1, [r3, 0x28]                           \n" // CFSR
        "    ldr      r2, [r3, 0x2c]                           \n" // HFSR
        "    orrs     r1, r2                                   \n"
        "    movs     r0, %[const_RESET_CAUSE_FAULT]           \n"
        "    bl       stm32wb_system_fatal                     \n"
        :
        : [const_RESET_CAUSE_FAULT] "I" (STM32WB_SYSTEM_RESET_CAUSE_FAULT)
        );
}

void NMI_Handler(void) __attribute__ ((weak, alias("armv7m_core_exception")));
void HardFault_Handler(void) __attribute__ ((alias("armv7m_core_exception")));
void MemManage_Handler(void) __attribute__ ((alias("armv7m_core_exception")));
void BusFault_Handler(void) __attribute__ ((alias("armv7m_core_exception")));
void UsageFault_Handler(void) __attribute__ ((alias("armv7m_core_exception")));
void DebugMon_Handler(void) __attribute__ ((alias("armv7m_core_exception")));

        
