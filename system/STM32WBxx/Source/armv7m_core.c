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

__attribute__((section(".noinit"))) uint32_t SystemCoreClock;

static __attribute__((section(".noinit"))) uint32_t armv7m_core_udelay_scale;

static __attribute__((section(".noinit2a"), used)) armv7m_core_info_t armv7m_core_info;
static __attribute__((section(".noinit2a"), used)) armv7m_core_state_t armv7m_core_state;

void __armv7m_core_initialize(void)
{
    NVIC_SetPriority(MemManage_IRQn, ARMV7M_IRQ_PRIORITY_MEMMANAGE);
    NVIC_SetPriority(BusFault_IRQn, ARMV7M_IRQ_PRIORITY_BUSFAULT);
    NVIC_SetPriority(UsageFault_IRQn, ARMV7M_IRQ_PRIORITY_USAGEFAULT);
    NVIC_SetPriority(DebugMon_IRQn, ARMV7M_IRQ_PRIORITY_DEBUGMON);

    SCB->CCR = 0;
    SCB->SHCSR = (SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk);
    
    __armv7m_pendsv_initialize();
    __armv7m_svcall_initialize();
    __armv7m_systick_initialize();
    __armv7m_rtos_initialize();
}

void armv7m_core_system_clock(uint32_t clock)
{
    SystemCoreClock = clock;

    armv7m_core_udelay_scale = clock / 15625;
}

void armv7m_core_udelay(uint32_t delay)
{
    uint32_t n;

    n = (delay * armv7m_core_udelay_scale + 255) / 256;

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

void __attribute__((naked, used, noreturn)) armv7m_core_fatal(uint32_t code, const char *file, uint32_t line, const armv7m_core_state_t *state)
{
    armv7m_core_info.code = code;
    armv7m_core_info.file = file;
    armv7m_core_info.line = line;
    armv7m_core_info.state = state;

    stm32wb_system_fatal(&armv7m_core_info);
}

static __attribute__((naked, used, noreturn)) void armv7m_core_fault(void)
{
    __asm__(
	"    mrs      r0, PRIMASK                              \n"
	"    cpsid    i                                        \n"
	"    movw     r3, #:lower16:armv7m_core_state          \n"
	"    movt     r3, #:upper16:armv7m_core_state          \n"
	"    str      r0, [r3, %[offset_PRIMASK]]              \n"
        "    tst      lr, #0x00000004                          \n" // bit 2 is SPSEL
	"    ite      eq                                       \n"
        "    moveq    r2, sp                                   \n"
        "    mrsne    r2, PSP                                  \n"
	"    add      r1, r3, %[offset_R4]                     \n"
	"    stmia    r1, { r4-r11 }                           \n"
        "    ldmia    r2!, { r4-r11 }                          \n" // r4-r11 = { r0, r1, r2, r3, r12, lr, pc, xpsr }
	"    str      r4, [r3, %[offset_R0]]                   \n"
	"    str      r5, [r3, %[offset_R1]]                   \n"
	"    str      r6, [r3, %[offset_R2]]                   \n"
	"    str      r7, [r3, %[offset_R3]]                   \n"
	"    str      r8, [r3, %[offset_R12]]                  \n"
	"    str      r9, [r3, %[offset_LR]]                   \n"
	"    str      r10, [r3, %[offset_PC]]                  \n"
	"    str      r11, [r3, %[offset_XPSR]]                \n"
	"    mrs      r0, BASEPRI                              \n"
	"    str      r0, [r3, %[offset_BASEPRI]]              \n"
	"    mrs      r0, FAULTMASK                            \n"
	"    str      r0, [r3, %[offset_FAULTMASK]]            \n"
	"    mrs      r0, CONTROL                              \n"
	"    str      r0, [r3, %[offset_CONTROL]]              \n"
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
        "    tst      lr, #0x00000010                          \n"
        "    bne.n    1f                                       \n"
	"    add      r1, r3, %[offset_D8]                     \n"
        "    vstmia   r1, { d8-d15 }                           \n"
	"    add      r1, r3, %[offset_D0]                     \n"
        "    vldmia   r2!, { d0-d7 }                           \n"
        "    vstmia   r2, { d0-d7 }                            \n"
        "    ldmia    r2!, { r4-r5 }                           \n" // r4-r5 = { fpscr, reserved }
	"    str      r4, [r3, %[offset_FPSCR]]                \n"
	"1:  bic      r1, r0, #0x04                            \n"
	"    msr      CONTROL, r1                              \n"
#endif /* __VFP_FP__ && !__SOFTFP__ */
        "    tst      lr, #0x00000004                          \n" // bit 2 is SPSEL
	"    ite      eq                                       \n"
        "    moveq    sp, r2                                   \n"
        "    msrne    PSP, r2                                  \n"
	"    mrs      r2, MSP                                  \n"
	"    str      r2, [r3, %[offset_MSP]]                  \n"
	"    mrs      r2, PSP                                  \n"
	"    str      r2, [r3, %[offset_PSP]]                  \n"
	"    mrs      r0, IPSR                                 \n" // code
	"    mov      r1, #0                                   \n" // file
	"    mov      r2, #0                                   \n" // line
	"    b        armv7m_core_fatal                        \n"
	:
	: [offset_R0]         "I" (offsetof(armv7m_core_state_t, r0)),
	  [offset_R1]         "I" (offsetof(armv7m_core_state_t, r1)),
	  [offset_R2]         "I" (offsetof(armv7m_core_state_t, r2)),
	  [offset_R3]         "I" (offsetof(armv7m_core_state_t, r3)),
	  [offset_R4]         "I" (offsetof(armv7m_core_state_t, r4)),
	  [offset_R12]        "I" (offsetof(armv7m_core_state_t, r12)),
	  [offset_LR]         "I" (offsetof(armv7m_core_state_t, lr)),
	  [offset_PC]         "I" (offsetof(armv7m_core_state_t, pc)),
	  [offset_XPSR]       "I" (offsetof(armv7m_core_state_t, xpsr)),
	  [offset_MSP]        "I" (offsetof(armv7m_core_state_t, msp)),
	  [offset_PSP]        "I" (offsetof(armv7m_core_state_t, psp)),
	  [offset_PRIMASK]    "I" (offsetof(armv7m_core_state_t, primask)),
	  [offset_BASEPRI]    "I" (offsetof(armv7m_core_state_t, basepri)),
	  [offset_FAULTMASK]  "I" (offsetof(armv7m_core_state_t, faultmask)),
	  [offset_CONTROL]    "I" (offsetof(armv7m_core_state_t, control))
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
	  ,
	  [offset_D0]         "I" (offsetof(armv7m_core_state_t, d0)),
	  [offset_D8]         "I" (offsetof(armv7m_core_state_t, d8)),
	  [offset_FPSCR]      "I" (offsetof(armv7m_core_state_t, fpscr))
#endif /* __VFP_FP__ && !__SOFTFP__ */
	);
}

void NMI_Handler(void) __attribute__ ((weak, alias("armv7m_core_fault")));
void HardFault_Handler(void) __attribute__ ((weak, alias("armv7m_core_fault")));
void MemManage_Handler(void) __attribute__ ((weak, alias("armv7m_core_fault")));
void BusFault_Handler(void) __attribute__ ((weak, alias("armv7m_core_fault")));
void UsageFault_Handler(void) __attribute__ ((weak, alias("armv7m_core_fault")));
void DebugMon_Handler(void) __attribute__ ((weak, alias("armv7m_core_fault")));
