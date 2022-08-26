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

static const stm32wb_hsem_handler_t stm32wb_hsem_handler[] = {
    HSEM0_HSEMHandler,
    HSEM1_HSEMHandler,
    HSEM2_HSEMHandler,
    HSEM3_HSEMHandler,
    HSEM4_HSEMHandler,
    HSEM5_HSEMHandler,
    HSEM6_HSEMHandler,
    HSEM7_HSEMHandler,
    HSEM8_HSEMHandler,
    HSEM9_HSEMHandler,
    HSEM10_HSEMHandler,
    HSEM11_HSEMHandler,
    HSEM12_HSEMHandler,
    HSEM13_HSEMHandler,
    HSEM14_HSEMHandler,
    HSEM15_HSEMHandler,
    HSEM16_HSEMHandler,
    HSEM17_HSEMHandler,
    HSEM18_HSEMHandler,
    HSEM19_HSEMHandler,
    HSEM20_HSEMHandler,
    HSEM21_HSEMHandler,
    HSEM22_HSEMHandler,
    HSEM23_HSEMHandler,
    HSEM24_HSEMHandler,
    HSEM25_HSEMHandler,
    HSEM26_HSEMHandler,
    HSEM27_HSEMHandler,
    HSEM28_HSEMHandler,
    HSEM29_HSEMHandler,
    HSEM30_HSEMHandler,
    HSEM31_HSEMHandler,
};

void __stm32wb_hsem_initialize(void)
{
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
        // armv7m_rtt_printf("HSEM_LOCK(%d [%d])\n", index, procid);

        
        return true;
    }
	
    armv7m_atomic_or(&HSEM->C1IER, (1u << index));

    return false;
}

void  __attribute__((optimize("O3"))) HSEM_IRQHandler(void)
{
    uint32_t mask, index;

    mask = HSEM->C1ISR;

    if (mask)
    {
	do
	{
	    index = __builtin_ctz(mask);
	    mask = (1u << index);
	    
	    HSEM->C1ICR = mask;

            armv7m_atomic_and(&HSEM->C1IER, ~mask);

            (*stm32wb_hsem_handler[index])();
	    
	    mask = HSEM->C1ISR;
	}
	while (mask);
    }
    
    __DSB();
}

static void __attribute__((naked)) Default_HSEMHandler(void)
{
}

void HSEM0_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM1_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM2_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM3_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM4_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM5_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM6_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM7_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM8_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM9_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM10_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM11_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM12_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM13_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM14_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM15_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM16_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM17_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM18_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM19_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM20_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM21_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM22_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM23_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM24_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM25_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM26_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM27_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM28_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM29_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM30_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
void HSEM31_HSEMHandler(void) __attribute__ ((weak, alias("Default_HSEMHandler")));
