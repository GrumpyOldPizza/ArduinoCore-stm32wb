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
#include "stm32wb_lptim.h"

typedef struct _armv7m_systick_control_t {
    volatile uint64_t         clock;
    volatile uint32_t         micros;
    volatile uint32_t         millis;
    uint32_t                  cycle;
    uint32_t                  scale[2];
    uint32_t                  params[6];
    volatile uint32_t         calib[5];
    volatile uint8_t          index;
    volatile uint8_t          sync;
    volatile uint8_t          state;
} armv7m_systick_control_t;

static  __attribute__((section(".noinit"))) armv7m_systick_control_t armv7m_systick_control;

void __armv7m_systick_initialize(void)
{
    NVIC_SetPriority(SysTick_IRQn, ARMV7M_IRQ_PRIORITY_SYSTICK);

    armv7m_systick_control.index = 0;
    armv7m_systick_control.sync = 0;
    armv7m_systick_control.state = 0;
}

static void armv7m_systick_params(void)
{
    armv7m_systick_control.params[3] = ((armv7m_systick_control.calib[0] + armv7m_systick_control.calib[1] + armv7m_systick_control.calib[2] + armv7m_systick_control.calib[3] + 32) / 64) -1;

    /* micros = 125000 * (count / coeff)
     * micros = (125000 / coeff) * count
     * micros = (((125000 * (2^32)) / coeff) * count) / (2^32)
     * micros = ((125000 / (coeff * (2 ^ 32)) * count) / (2^32)
     */

    armv7m_systick_control.params[4] = ((uint64_t)125000ull * 0x100000000ull) / (uint32_t)(armv7m_systick_control.params[3]);
    armv7m_systick_control.params[5] = ((uint64_t)125ull    * 0x100000000ull) / (uint32_t)(armv7m_systick_control.params[3]);
}

void armv7m_systick_configure(uint32_t clock)
{
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    armv7m_systick_control.calib[0] = clock * 2;
    armv7m_systick_control.calib[1] = clock * 2;
    armv7m_systick_control.calib[2] = clock * 2;
    armv7m_systick_control.calib[3] = clock * 2;
    
    armv7m_systick_params();
}

void __armv7m_systick_calibrate(void)
{
    uint32_t clock, clock_previous, count;

    if (armv7m_systick_control.state != 0)
    {
        clock = (uint32_t)armv7m_systick_control.clock;
        
        if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
        {
            clock += (armv7m_systick_control.cycle +1);
        }
        
        do
        {
            clock_previous = clock;
            
            count = (armv7m_systick_control.cycle - SysTick->VAL);
            
            clock = (uint32_t)armv7m_systick_control.clock;
            
            if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
            {
                clock += (armv7m_systick_control.cycle +1);
            }
        }
        while (clock != clock_previous);
        
        clock += count;

        if (armv7m_systick_control.state == 1)
        {
            armv7m_systick_control.state = 2;

            armv7m_systick_control.calib[4] = clock;
        }
        else
        {
            armv7m_systick_control.state = 3;

            armv7m_systick_control.calib[armv7m_systick_control.index] = clock - armv7m_systick_control.calib[4];
            armv7m_systick_control.calib[4] = clock;

            armv7m_systick_control.index = (armv7m_systick_control.index +1) & 3;
            
        }

        armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_SYSTICK);
    }
}

__attribute__((optimize("O3"))) void armv7m_systick_enable(void)
{
    uint64_t clock;
    uint32_t frequency, seconds, ticks, count;

    frequency = (armv7m_systick_control.calib[0] + armv7m_systick_control.calib[1] + armv7m_systick_control.calib[2] + armv7m_systick_control.calib[3] + 4) / 8;
    
    armv7m_systick_control.params[0] = armv7m_systick_control.params[3];
    armv7m_systick_control.params[1] = armv7m_systick_control.params[4];
    armv7m_systick_control.params[2] = armv7m_systick_control.params[5];

    armv7m_systick_control.cycle    = armv7m_systick_control.params[0];
    armv7m_systick_control.scale[0] = armv7m_systick_control.params[1];
    armv7m_systick_control.scale[1] = armv7m_systick_control.params[2];
    
    clock = stm32wb_lptim_timeout_clock();
    ticks = clock >> 12;

    armv7m_systick_control.micros = 125000 * ticks;
    armv7m_systick_control.millis = 125 * ticks;

    ticks = clock & 4095;

    /* 2 partial products to avoid a long multiplication
     */
    count = (((armv7m_systick_control.cycle & 4095) * ticks) >> 12) + ((armv7m_systick_control.cycle >> 12) * ticks);

    seconds = clock / 32768;
    ticks = clock & 32767;

    armv7m_systick_control.clock = ((uint64_t)seconds * (uint64_t)frequency) + (((uint64_t)ticks * (uint64_t)frequency) / 32768) - count;
    
    SysTick->VAL = armv7m_systick_control.cycle - count;
    SysTick->LOAD = armv7m_systick_control.cycle;

    SysTick->CTRL |= (SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
    SysTick->CTRL;

    armv7m_systick_control.state = 1;
    
    __DSB();
}

__attribute__((optimize("O3"))) void armv7m_systick_disable(void)
{
    armv7m_systick_control.state = 0;

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
        clock += (armv7m_systick_control.cycle +1);
    }

    do
    {
        clock_previous = clock;
        
        count = (armv7m_systick_control.cycle - SysTick->VAL);

        clock = armv7m_systick_control.clock;

        if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
        {
            clock += (armv7m_systick_control.cycle +1);
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
        micros += 125000;
    }

    do
    {
        micros_previous = micros;
        
        count = (armv7m_systick_control.cycle - SysTick->VAL);

        micros = armv7m_systick_control.micros;

        if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
        {
            micros += 125000;
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
        millis += 125;
    }

    do
    {
        millis_previous = millis;
        
        count = (armv7m_systick_control.cycle - SysTick->VAL);

        millis = armv7m_systick_control.millis;

        if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
        {
            millis += 125;
        }
    }
    while (millis != millis_previous);
    
    return millis + (uint32_t)(((uint64_t)count * (uint64_t)armv7m_systick_control.scale[1]) >> 32);
}

__attribute__((optimize("O3"))) void SysTick_Handler(void)
{
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
    {
        armv7m_systick_control.clock  += (armv7m_systick_control.cycle +1);
        armv7m_systick_control.micros += 125000;
        armv7m_systick_control.millis += 125;

        armv7m_systick_control.cycle    = armv7m_systick_control.params[0];
        armv7m_systick_control.scale[0] = armv7m_systick_control.params[1];
        armv7m_systick_control.scale[1] = armv7m_systick_control.params[2];

        if (armv7m_systick_control.sync)
        {
            armv7m_systick_control.sync = 0;
        
            armv7m_systick_control.params[0] = armv7m_systick_control.params[3];
            armv7m_systick_control.params[1] = armv7m_systick_control.params[4];
            armv7m_systick_control.params[2] = armv7m_systick_control.params[5];

            SysTick->LOAD = armv7m_systick_control.params[0];
        }
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

void SYSTICK_SWIHandler(void)
{
    if (armv7m_systick_control.state == 3)
    {
        armv7m_systick_control.sync = 0;

        armv7m_systick_params();

        armv7m_systick_control.sync = 1;
    }
}
