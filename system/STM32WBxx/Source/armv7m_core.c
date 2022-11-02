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

__attribute__((section(".fatal_info"))) armv7m_core_fatal_info_t armv7m_core_fatal_info;

void __armv7m_core_initialize(void)
{
    armv7m_core_control.udelay_scale = SystemCoreClock / 15625;

    NVIC_SetPriority(MemManage_IRQn, ARMV7M_IRQ_PRIORITY_MEMMANAGE);
    NVIC_SetPriority(BusFault_IRQn, ARMV7M_IRQ_PRIORITY_BUSFAULT);
    NVIC_SetPriority(UsageFault_IRQn, ARMV7M_IRQ_PRIORITY_USAGEFAULT);
    NVIC_SetPriority(DebugMon_IRQn, ARMV7M_IRQ_PRIORITY_DEBUGMON);

    SCB->CCR = 0;
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

void __attribute__((noreturn, used)) armv7m_core_fatal(uint32_t code, uint32_t pc, uint32_t lr, uint32_t sp)
{
    armv7m_core_fatal_info.signature = 0xdead0000 | code;
    armv7m_core_fatal_info.pc = pc;
    armv7m_core_fatal_info.lr = lr;
    armv7m_core_fatal_info.sp = sp;

    stm32wb_system_fatal();
}

static __attribute__((naked)) void armv7m_core_fault(void)
{
    __asm__(
        "    cpsid    i                                        \n"
        "    movw     r7, #:lower16:armv7m_core_fatal_info     \n"
        "    movt     r7, #:upper16:armv7m_core_fatal_info     \n"
        "    tst      lr, #0x00000004                          \n" // LR bit 2 is SPSEL
        "    ite      ne                                       \n"
        "    mrsne    r6, PSP                                  \n"
        "    moveq    r6, sp                                   \n"
        "    ldr      r0, [r6, #28]                            \n" // XPSR
        "    ldr      r1, [r6, #24]                            \n" // PC
        "    ldr      r2, [r6, #20]                            \n" // LR
#if (__FPU_PRESENT == 1)
        "    tst      lr, #0x00000010                          \n" // LR bit 4 is nFPCA
        "    ite      ne                                       \n"
        "    addne    r3, r6, #0x68                            \n"
        "    addeq    r3, r6, #0x20                            \n"
#else  /* __FPU_PRESENT == 1 */
        "    add      r3, r6, #0x20                            \n"
#endif /* __FPU_PRESENT == 1 */
        "    lsr      r5, r0, #7                               \n"
        "    and      r5, #4                                   \n"
        "    add      r3, r5                                   \n" // drop aligner if needed
        "    ldr      r4, [r1, #1]                             \n" // PC
        "    cmp      r4, #0xbe                                \n" // PC
        "    ite      eq                                       \n"
        "    moveq    r0, #0                                   \n"
        "    andne    r0, #255                                 \n"
        "    b        armv7m_core_fatal                        \n"
        );
}

void NMI_Handler(void) __attribute__ ((weak, alias("armv7m_core_fault")));
void HardFault_Handler(void) __attribute__ ((alias("armv7m_core_fault")));
void MemManage_Handler(void) __attribute__ ((alias("armv7m_core_fault")));
void BusFault_Handler(void) __attribute__ ((alias("armv7m_core_fault")));
void UsageFault_Handler(void) __attribute__ ((alias("armv7m_core_fault")));
void DebugMon_Handler(void) __attribute__ ((alias("armv7m_core_fault")));

        
