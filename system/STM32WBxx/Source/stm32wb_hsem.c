/*
 * Copyright (c) 2020 Thomas Roell.  All rights reserved.
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
#include "stm32wbxx.h"

#include "stm32wb_hsem.h"
#include "stm32wb_system.h"

typedef void (*stm32wb_hsem_handler_t)(void);

#define STM32WB_HSEM_INDEX_NONE 255

typedef struct _stm32wb_hsem_device_t {
    volatile uint8_t        index[STM32WB_HSEM_PROCID_COUNT];
} stm32wb_hsem_device_t;

static stm32wb_hsem_device_t stm32wb_hsem_device;

static const stm32wb_hsem_handler_t stm32wb_hsem_handler[STM32WB_HSEM_PROCID_COUNT] = {
    PROCID1_HSEMHandler,
    PROCID2_HSEMHandler,
    PROCID3_HSEMHandler,
    PROCID4_HSEMHandler,
    PROCID5_HSEMHandler,
    PROCID6_HSEMHandler,
    PROCID7_HSEMHandler,
    PROCID8_HSEMHandler,
};

void __stm32wb_hsem_initialize(void)
{
    uint32_t slot;

    for (slot = 0; slot < STM32WB_HSEM_PROCID_COUNT; slot++)
    {
        stm32wb_hsem_device.index[slot] = STM32WB_HSEM_INDEX_NONE;
    }

    NVIC_SetPriority(HSEM_IRQn, STM32WB_HSEM_IRQ_PRIORITY);
    NVIC_EnableIRQ(HSEM_IRQn);

    EXTI->IMR2 |= EXTI_IMR2_IM38;
}
	
bool __attribute__((optimize("O3"))) __stm32wb_hsem_lock(uint32_t index, uint32_t procid)
{
    HSEM->C1ICR = (1u << index);

    HSEM->R[index] = (HSEM_RLR_LOCK | HSEM_R_COREID_CPU1 | procid);

    if (HSEM->R[index] == (HSEM_RLR_LOCK | HSEM_R_COREID_CPU1 | procid))
    {
        return true;
    }

    stm32wb_hsem_device.index[procid -1] = index;
    
    armv7m_atomic_or(&HSEM->C1IER, (1u << index));

    return false;
}

void  __attribute__((optimize("O3"))) HSEM_IRQHandler(void)
{
    uint32_t mask, index, slot;

    mask = HSEM->C1ISR;

    if (mask)
    {
	do
	{
	    index = __builtin_ctz(mask);
	    mask = (1u << index);
	    
	    HSEM->C1ICR = mask;

            armv7m_atomic_and(&HSEM->C1IER, ~mask);

            for (slot = 0; slot < STM32WB_HSEM_PROCID_COUNT; slot++)
            {
                if (stm32wb_hsem_device.index[slot] == index)
                {
                    stm32wb_hsem_device.index[slot] = STM32WB_HSEM_INDEX_NONE;

                    (*stm32wb_hsem_handler[slot])();
                }
            }
	    
	    mask = HSEM->C1ISR;
	}
	while (mask);
    }
    
    __DSB();
}

static void __attribute__((naked)) Default_HSEMHandler(void)
{
}

void PROCID1_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void PROCID2_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void PROCID3_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void PROCID4_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void PROCID5_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void PROCID6_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void PROCID7_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void PROCID8_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
