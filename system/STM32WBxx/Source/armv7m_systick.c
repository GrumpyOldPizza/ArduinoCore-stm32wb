/*
 * Copyright (c) 2017-2022 Thomas Roell.  All rights reserved.
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
#include "stm32wb_lptim.h"

typedef struct _armv7m_systick_control_t {
    volatile uint64_t         clock;
    volatile uint32_t         micros;
    volatile uint32_t         millis;
    uint32_t                  cycle;
    uint32_t                  scale[2];
    uint32_t                  step[3];
} armv7m_systick_control_t;

static  __attribute__((section(".noinit"))) armv7m_systick_control_t armv7m_systick_control;

void __armv7m_systick_initialize(void)
{
    NVIC_SetPriority(SysTick_IRQn, ARMV7M_IRQ_PRIORITY_SYSTICK);
}

void armv7m_systick_configure(void)
{
    armv7m_systick_control.clock = 0;
    armv7m_systick_control.micros = 0;
    armv7m_systick_control.millis = 0;

    armv7m_systick_control.cycle = (SystemCoreClock / 8) -1;
    armv7m_systick_control.scale[0] = ((uint64_t)125000ull * 0x100000000ull) / SystemCoreClock;
    armv7m_systick_control.scale[1] = ((uint64_t)125ull    * 0x100000000ull) / SystemCoreClock;
    
    armv7m_systick_control.step[0] = SystemCoreClock / 8;
    armv7m_systick_control.step[1] = 125000;
    armv7m_systick_control.step[2] = 125;
    
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    SysTick->VAL = armv7m_systick_control.cycle;
    SysTick->LOAD = armv7m_systick_control.cycle;
}

__attribute__((optimize("O3"))) void armv7m_systick_enable(void)
{
    SysTick->CTRL |= (SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
    SysTick->CTRL;

    __DSB();
}

__attribute__((optimize("O3"))) void armv7m_systick_disable(void)
{
    SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
    SysTick->CTRL;

    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

    __DSB();
}

__attribute__((optimize("O3"))) uint64_t armv7m_systick_clock(void)
{
    uint64_t clock, clock_previous;
    uint32_t count;

    clock = armv7m_systick_control.clock;

    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
    {
        clock += armv7m_systick_control.step[0];
    }

    do
    {
        clock_previous = clock;
        
        count = (armv7m_systick_control.cycle - SysTick->VAL);

        clock = armv7m_systick_control.clock;

        if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
        {
            clock += armv7m_systick_control.step[0];
        }
    }
    while (clock != clock_previous);
    
    return clock + count;
}

__attribute__((optimize("O3"))) uint32_t armv7m_systick_micros(void)
{
    uint32_t micros, micros_previous, count;

    micros = armv7m_systick_control.micros;

    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
    {
        micros += armv7m_systick_control.step[1];
    }

    do
    {
        micros_previous = micros;
        
        count = (armv7m_systick_control.cycle - SysTick->VAL);

        micros = armv7m_systick_control.micros;

        if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
        {
            micros += armv7m_systick_control.step[1];
        }
    }
    while (micros != micros_previous);
    
    return micros + (uint32_t)(((uint64_t)count * (uint64_t)armv7m_systick_control.scale[0]) >> 32);
}

__attribute__((optimize("O3"))) uint32_t armv7m_systick_millis(void)
{
    uint32_t millis, millis_previous, count;

    millis = armv7m_systick_control.millis;

    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
    {
        millis += armv7m_systick_control.step[2];
    }

    do
    {
        millis_previous = millis;
        
        count = (armv7m_systick_control.cycle - SysTick->VAL);

        millis = armv7m_systick_control.millis;

        if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
        {
            millis += armv7m_systick_control.step[2];
        }
    }
    while (millis != millis_previous);
    
    return millis + (uint32_t)(((uint64_t)count * (uint64_t)armv7m_systick_control.scale[1]) >> 32);
}

__attribute__((optimize("O3"))) void SysTick_Handler(void)
{
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
    {
        armv7m_systick_control.clock  += armv7m_systick_control.step[0];
        armv7m_systick_control.micros += armv7m_systick_control.step[1];
        armv7m_systick_control.millis += armv7m_systick_control.step[2];
    }

    __DSB();
}

void armv7m_systick_udelay(uint32_t udelay)
{
    uint32_t start, end;

    start = armv7m_systick_micros();

    do
    {
        end = armv7m_systick_micros();
    }
    while ((end - start) < udelay);
}
