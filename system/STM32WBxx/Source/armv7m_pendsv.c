/*
 * Copyright (c) 2014-2020 Thomas Roell.  All rights reserved.
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

extern void PendSV_Handler(void);

typedef struct _armv7m_pendsv_control_t {
    volatile uint32_t                swi_pending;
    armv7m_pendsv_callback_t         hook_callback;
} armv7m_pendsv_control_t;

static __attribute__((section(".noinit"))) armv7m_pendsv_control_t armv7m_pendsv_control;

static const armv7m_pendsv_callback_t armv7m_pendsv_swi_callback[] = {
    SWI0_SWIHandler,
    SWI1_SWIHandler,
    SWI2_SWIHandler,
    SWI3_SWIHandler,
    SWI4_SWIHandler,
    SWI5_SWIHandler,
    SWI6_SWIHandler,
    SWI7_SWIHandler,
    SWI8_SWIHandler,
    SWI9_SWIHandler,
    SWI10_SWIHandler,
    SWI11_SWIHandler,
    SWI12_SWIHandler,
    SWI13_SWIHandler,
    SWI14_SWIHandler,
    SWI15_SWIHandler,
    SWI16_SWIHandler,
    SWI17_SWIHandler,
    SWI18_SWIHandler,
    SWI19_SWIHandler,
    SWI20_SWIHandler,
    SWI21_SWIHandler,
    SWI22_SWIHandler,
    SWI23_SWIHandler,
    SWI24_SWIHandler,
    SWI25_SWIHandler,
    SWI26_SWIHandler,
    SWI27_SWIHandler,
    SWI28_SWIHandler,
    SWI29_SWIHandler,
    SWI30_SWIHandler,
    SWI31_SWIHandler,
};
    
void __armv7m_pendsv_initialize(void)
{
    armv7m_pendsv_control.swi_pending = 0;
    armv7m_pendsv_control.hook_callback = NULL;

    NVIC_SetPriority(PendSV_IRQn, ARMV7M_IRQ_PRIORITY_PENDSV);
}

void __attribute__((optimize("O3"))) armv7m_pendsv_hook(armv7m_pendsv_callback_t callback)
{
    armv7m_pendsv_control.hook_callback = callback;

    if (!armv7m_core_is_in_pendsv())
    {
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
}

void __attribute__((optimize("O3"))) armv7m_pendsv_raise(uint32_t mask)
{
    __armv7m_atomic_or(&armv7m_pendsv_control.swi_pending, mask);

    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

bool __attribute__((optimize("O3"))) armv7m_pendsv_is_pending(uint32_t mask)
{
    return !!(armv7m_pendsv_control.swi_pending & mask);
}

static void __attribute__((optimize("O3"), used)) armv7m_pendsv_process(uint32_t mask)
{
    uint32_t index;

    __armv7m_atomic_and(&armv7m_pendsv_control.swi_pending, ~mask);
    
    do
    {
        index = __builtin_ctz(mask);
        mask &= ~(1u << index);
        
        (*armv7m_pendsv_swi_callback[index])();
    }
    while (mask);
}

void __attribute__((naked)) PendSV_Handler(void)
{
    __asm__(
        "   movw    r3, #:lower16:armv7m_pendsv_control  \n"
        "   movt    r3, #:upper16:armv7m_pendsv_control  \n"
        "   ldr     r0, [r3, %[offset_SWI_PENDING]]      \n"
        "   cbz.n   r0, 1f                               \n"
        "   push    { r3, lr }                           \n"
        "   bl      armv7m_pendsv_process                \n" // R0 is swi_pending
        "   pop     { r3, lr }                           \n"
        "1: ldr     r0, [r3, %[offset_HOOK_CALLBACK]]    \n"
        "   cbnz.n  r0, 2f                               \n"
        "   dsb                                          \n"
        "   bx      lr                                   \n"
        "2: mov     r1, #0                               \n"
        "   str     r1, [r3, %[offset_HOOK_CALLBACK]]    \n"
        "   bx      r0                                   \n"
        :
        : [offset_SWI_PENDING]     "I" (offsetof(armv7m_pendsv_control_t, swi_pending)),
          [offset_HOOK_CALLBACK]   "I" (offsetof(armv7m_pendsv_control_t, hook_callback))
        );
}

static void __attribute__((naked)) Default_SWIHandler(void)
{
}

void SWI0_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI1_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI2_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI3_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI4_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI5_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI6_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI7_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI8_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI9_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI10_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI11_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI12_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI13_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI14_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI15_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI16_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI17_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI18_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI19_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI20_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI21_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI22_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI23_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI24_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI25_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI26_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI27_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI28_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI29_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI30_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
void SWI31_SWIHandler(void) __attribute__ ((weak, alias("Default_SWIHandler")));
