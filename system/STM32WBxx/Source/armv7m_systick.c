/*
 * Copyright (c) 2017-2021 Thomas Roell.  All rights reserved.
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
#include "stm32wb_rtc.h"

typedef struct _armv7m_systick_control_t {
    volatile uint64_t         micros;
    volatile uint64_t         millis;
    uint32_t                  cycle;
    uint32_t                  scale[2];
} armv7m_systick_control_t;

static  __attribute__((section(".noinit"))) armv7m_systick_control_t armv7m_systick_control;

void __armv7m_systick_initialize(void)
{
    armv7m_systick_control.micros = 0;
    armv7m_systick_control.millis = 0;
    armv7m_systick_control.cycle = 0;
    armv7m_systick_control.scale[0] = 0;
    armv7m_systick_control.scale[1] = 0;
  
    NVIC_SetPriority(SysTick_IRQn, ARMV7M_IRQ_PRIORITY_SYSTICK);
}

void armv7m_systick_configure(void)
{
    uint32_t count;
    
    if (armv7m_systick_control.cycle)
    {
        count = (armv7m_systick_control.cycle - SysTick->VAL);

        armv7m_systick_control.micros += (uint32_t)(((uint64_t)count * (uint64_t)armv7m_systick_control.scale[0]) >> 32);
        armv7m_systick_control.millis += (uint32_t)(((uint64_t)count * (uint64_t)armv7m_systick_control.scale[1]) >> 32);
    }

    if (SystemCoreClock <= 2000000)
    {
        armv7m_systick_control.cycle = SystemCoreClock / 8 -1;
        
        SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    }
    else
    {
        armv7m_systick_control.cycle = SystemCoreClock / 64 -1;
        
        SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
    }

    /* micros = 125000 * (count / (cycle +1))
     * micros = (125000 / (cycle +1)) * count
     * micros = ((125000 / (cycle +1) * (2 ^ 32)) * count) / (2^32)
     */

    armv7m_systick_control.scale[0] = (uint64_t)(125000ull * 0x100000000ull) / (uint64_t)(armv7m_systick_control.cycle +1);
    armv7m_systick_control.scale[1] = (uint64_t)(125ull * 0x100000000ull) / (uint64_t)(armv7m_systick_control.cycle +1);

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

__attribute__((optimize("O3"))) uint64_t armv7m_systick_micros(void)
{
    uint32_t count;

    count = (armv7m_systick_control.cycle - SysTick->VAL);

    return armv7m_systick_control.micros + (uint32_t)(((uint64_t)count * (uint64_t)armv7m_systick_control.scale[0]) >> 32);
}

__attribute__((optimize("O3"))) uint64_t armv7m_systick_millis(void)
{
    uint32_t count;

    count = (armv7m_systick_control.cycle - SysTick->VAL);

    return armv7m_systick_control.millis + (uint32_t)(((uint64_t)count * (uint64_t)armv7m_systick_control.scale[1]) >> 32);
}

__attribute__((optimize("O3"))) void SysTick_Handler(void)
{
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
    {
        armv7m_systick_control.micros += 125000;
        armv7m_systick_control.millis += 125;
    }

    __DSB();
}
