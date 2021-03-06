/*
 * Copyright (c) 2017-2020 Thomas Roell.  All rights reserved.
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

#include "stm32wb_exti.h"
#include "stm32wb_gpio.h"

typedef struct _stm32wb_exti_t {
    stm32wb_exti_callback_t volatile callback;
    void * volatile                  context;
} stm32wb_exti_t;

typedef struct _stm32wb_exti_device_t {
    uint16_t                events;
    uint16_t                mask;
    volatile uint16_t       pending;
    volatile uint16_t       priority[4];
    stm32wb_exti_t          channels[16];
} stm32wb_exti_device_t;

static stm32wb_exti_device_t stm32wb_exti_device;

void __stm32wb_exti_initialize(void)
{
    stm32wb_exti_device.events = 0;
    stm32wb_exti_device.mask = ~0;
    stm32wb_exti_device.pending = 0;
    stm32wb_exti_device.priority[0] = 0;
    stm32wb_exti_device.priority[1] = 0;
    stm32wb_exti_device.priority[2] = 0;
    stm32wb_exti_device.priority[3] = 0;

    NVIC_SetPriority(EXTI0_IRQn, ARMV7M_IRQ_PRIORITY_CRITICAL);
    NVIC_SetPriority(EXTI1_IRQn, ARMV7M_IRQ_PRIORITY_CRITICAL);
    NVIC_SetPriority(EXTI2_IRQn, ARMV7M_IRQ_PRIORITY_CRITICAL);
    NVIC_SetPriority(EXTI3_IRQn, ARMV7M_IRQ_PRIORITY_CRITICAL);
    NVIC_SetPriority(EXTI4_IRQn, ARMV7M_IRQ_PRIORITY_CRITICAL);
    NVIC_SetPriority(EXTI9_5_IRQn, ARMV7M_IRQ_PRIORITY_CRITICAL);
    NVIC_SetPriority(EXTI15_10_IRQn, ARMV7M_IRQ_PRIORITY_CRITICAL);
    NVIC_SetPriority(SPI1_IRQn, ARMV7M_IRQ_PRIORITY_HIGH);
    NVIC_SetPriority(SPI2_IRQn, ARMV7M_IRQ_PRIORITY_MEDIUM);

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(SPI1_IRQn);
    NVIC_EnableIRQ(SPI2_IRQn);
}

__attribute__((optimize("O3"))) bool stm32wb_exti_attach(uint16_t pin, uint32_t control, stm32wb_exti_callback_t callback, void *context)
{
    unsigned int mask, index, group;

    index = (pin & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT;
    group = (pin & STM32WB_GPIO_PIN_GROUP_MASK) >> STM32WB_GPIO_PIN_GROUP_SHIFT;

    mask = 1ul << index;

    if (stm32wb_exti_device.events & mask)
    {
        return false;
    }

    stm32wb_exti_device.channels[index].callback = NULL;
    stm32wb_exti_device.channels[index].context = context;
    stm32wb_exti_device.channels[index].callback = callback;

    armv7m_atomic_modify(&SYSCFG->EXTICR[index >> 2], (0x0000000f << ((index & 3) << 2)), (group << ((index & 3) << 2)));
    
    armv7m_atomic_modify(&EXTI->RTSR1, mask, ((control & STM32WB_EXTI_CONTROL_EDGE_RISING)  ? mask : 0));
    armv7m_atomic_modify(&EXTI->FTSR1, mask, ((control & STM32WB_EXTI_CONTROL_EDGE_FALLING) ? mask : 0));
    
    armv7m_atomic_andh(&stm32wb_exti_device.priority[0], ~mask);
    armv7m_atomic_andh(&stm32wb_exti_device.priority[1], ~mask);
    armv7m_atomic_andh(&stm32wb_exti_device.priority[2], ~mask);
    armv7m_atomic_andh(&stm32wb_exti_device.priority[3], ~mask);
    armv7m_atomic_orh(&stm32wb_exti_device.priority[(control & STM32WB_EXTI_CONTROL_PRIORITY_MASK) >> STM32WB_EXTI_CONTROL_PRIORITY_SHIFT], mask);
    armv7m_atomic_orh(&stm32wb_exti_device.events, mask);

    if (stm32wb_exti_device.mask & mask)
    {
	armv7m_atomic_or(&EXTI->IMR1, mask);
    }

    return true;
}

__attribute__((optimize("O3"))) void stm32wb_exti_detach(uint16_t pin)
{
    unsigned int mask, index;

    index = (pin & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT;

    mask = 1ul << index;

    armv7m_atomic_and(&EXTI->IMR1, ~mask);
    
    armv7m_atomic_andh(&stm32wb_exti_device.events, ~mask);
}

__attribute__((optimize("O3"))) void stm32wb_exti_block(uint32_t mask)
{
    mask &= stm32wb_exti_device.events;

    if (mask)
    {
        armv7m_atomic_and(&EXTI->IMR1, ~mask);
    }

    armv7m_atomic_andh(&stm32wb_exti_device.mask, ~mask);
}

__attribute__((optimize("O3"))) void stm32wb_exti_unblock(uint32_t mask, bool cancel)
{
    mask &= stm32wb_exti_device.events;

    armv7m_atomic_orh(&stm32wb_exti_device.mask, mask);

    if (mask)
    {
	if (cancel)
	{
	    EXTI->PR1 = (mask & 0x0000ffff);
	}

	armv7m_atomic_or(&EXTI->IMR1, (mask & 0x0000ffff));
    }
}

static inline void stm32wb_exti_interrupt_1(uint32_t mask, uint32_t index)
{
    stm32wb_exti_callback_t callback;
    void *context;
    
    if (mask & (1 << index))
    {
	armv7m_atomic_load_2((volatile uint32_t*)&stm32wb_exti_device.channels[index].callback, (uint32_t*)&callback, (uint32_t*)&context);

	if (callback)
	{
	    (*callback)(context);
	}
    }
}

static inline void stm32wb_exti_interrupt_2(uint32_t mask)
{
    if (mask)
    {
        __armv7m_atomic_orh(&stm32wb_exti_device.pending, mask);
        
        if (stm32wb_exti_device.priority[1] & mask)
        {
            NVIC_SetPendingIRQ(SPI1_IRQn);
        }
        else
        {
            if (stm32wb_exti_device.priority[2] & mask)
            {
                NVIC_SetPendingIRQ(SPI2_IRQn);
            }
            else
            {
                armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_EXTI);
            }
        }
    }
}

__attribute__((optimize("O3"))) void EXTI0_IRQHandler(void)
{
    uint32_t mask, mask_1, mask_2;

    mask = (EXTI->PR1 & stm32wb_exti_device.mask) & 0x0001;
    
    EXTI->PR1 = mask;

    mask_1 = mask & stm32wb_exti_device.priority[0];
    
    stm32wb_exti_interrupt_1(mask_1, 0);

    mask_2 = mask & ~stm32wb_exti_device.priority[0];

    stm32wb_exti_interrupt_2(mask_2);

    __DSB();
}

__attribute__((optimize("O3"))) void EXTI1_IRQHandler(void)
{
    uint32_t mask, mask_1, mask_2;

    mask = (EXTI->PR1 & stm32wb_exti_device.mask) & 0x0002;
    
    EXTI->PR1 = mask;

    mask_1 = mask & stm32wb_exti_device.priority[0];
    
    stm32wb_exti_interrupt_1(mask_1, 1);

    mask_2 = mask & ~stm32wb_exti_device.priority[0];

    stm32wb_exti_interrupt_2(mask_2);

    __DSB();
}

__attribute__((optimize("O3"))) void EXTI2_IRQHandler(void)
{
    uint32_t mask, mask_1, mask_2;

    mask = (EXTI->PR1 & stm32wb_exti_device.mask) & 0x0004;
    
    EXTI->PR1 = mask;

    mask_1 = mask & stm32wb_exti_device.priority[0];
    
    stm32wb_exti_interrupt_1(mask_1, 2);

    mask_2 = mask & ~stm32wb_exti_device.priority[0];

    stm32wb_exti_interrupt_2(mask_2);

    __DSB();
}

__attribute__((optimize("O3"))) void EXTI3_IRQHandler(void)
{
    uint32_t mask, mask_1, mask_2;

    mask = (EXTI->PR1 & stm32wb_exti_device.mask) & 0x0008;
    
    EXTI->PR1 = mask;

    mask_1 = mask & stm32wb_exti_device.priority[0];
    
    stm32wb_exti_interrupt_1(mask_1, 3);

    mask_2 = mask & ~stm32wb_exti_device.priority[0];

    stm32wb_exti_interrupt_2(mask_2);

    __DSB();
}

__attribute__((optimize("O3"))) void EXTI4_IRQHandler(void)
{
    uint32_t mask, mask_1, mask_2;

    mask = (EXTI->PR1 & stm32wb_exti_device.mask) & 0x0010;
    
    EXTI->PR1 = mask;

    mask_1 = mask & stm32wb_exti_device.priority[0];
    
    stm32wb_exti_interrupt_1(mask_1, 4);

    mask_2 = mask & ~stm32wb_exti_device.priority[0];

    stm32wb_exti_interrupt_2(mask_2);

    __DSB();
}

__attribute__((optimize("O3"))) void EXTI9_5_IRQHandler(void)
{
    uint32_t mask, mask_1, mask_2;

    mask = (EXTI->PR1 & stm32wb_exti_device.mask) & 0x03e0;
    
    EXTI->PR1 = mask;

    mask_1 = mask & stm32wb_exti_device.priority[0];
    
    stm32wb_exti_interrupt_1(mask_1, 5);
    stm32wb_exti_interrupt_1(mask_1, 6);
    stm32wb_exti_interrupt_1(mask_1, 7);
    stm32wb_exti_interrupt_1(mask_1, 8);
    stm32wb_exti_interrupt_1(mask_1, 9);

    mask_2 = mask & ~stm32wb_exti_device.priority[0];

    stm32wb_exti_interrupt_2(mask_2);

    __DSB();
}

__attribute__((optimize("O3"))) void EXTI15_10_IRQHandler(void)
{
    uint32_t mask, mask_1, mask_2;

    mask = (EXTI->PR1 & stm32wb_exti_device.mask) & 0xfc00;
    
    EXTI->PR1 = mask;

    mask_1 = mask & stm32wb_exti_device.priority[0];
    
    stm32wb_exti_interrupt_1(mask_1, 10);
    stm32wb_exti_interrupt_1(mask_1, 11);
    stm32wb_exti_interrupt_1(mask_1, 12);
    stm32wb_exti_interrupt_1(mask_1, 13);
    stm32wb_exti_interrupt_1(mask_1, 14);
    stm32wb_exti_interrupt_1(mask_1, 15);

    mask_2 = mask & ~stm32wb_exti_device.priority[0];

    stm32wb_exti_interrupt_2(mask_2);

    __DSB();
}

 __attribute__((optimize("O3"))) void SPI1_IRQHandler(void)
{
    uint32_t mask, index;
    stm32wb_exti_callback_t callback;
    void *context;

    NVIC_ClearPendingIRQ(SPI1_IRQn);

    mask = (stm32wb_exti_device.pending & stm32wb_exti_device.mask) & stm32wb_exti_device.priority[1];

    armv7m_atomic_andh(&stm32wb_exti_device.pending, ~mask);

    while (mask) 
    {
        index = __builtin_ctz(mask);

        mask &= ~(1ul << index); 

	armv7m_atomic_load_2((volatile uint32_t*)&stm32wb_exti_device.channels[index].callback, (uint32_t*)&callback, (uint32_t*)&context);

	if (callback)
	{
	    (*callback)(context);
	}
    }

    __DSB();
}

 __attribute__((optimize("O3"))) void SPI2_IRQHandler(void)
{
    uint32_t mask, index;
    stm32wb_exti_callback_t callback;
    void *context;

    NVIC_ClearPendingIRQ(SPI2_IRQn);

    mask = (stm32wb_exti_device.pending & stm32wb_exti_device.mask) & stm32wb_exti_device.priority[2];

    armv7m_atomic_andh(&stm32wb_exti_device.pending, ~mask);

    while (mask) 
    {
        index = __builtin_ctz(mask);

        mask &= ~(1ul << index); 

	armv7m_atomic_load_2((volatile uint32_t*)&stm32wb_exti_device.channels[index].callback, (uint32_t*)&callback, (uint32_t*)&context);

	if (callback)
	{
	    (*callback)(context);
	}
    }

    __DSB();
}

 __attribute__((optimize("O3"))) void EXTI_SWIHandler(void)
{
    uint32_t mask, index;
    stm32wb_exti_callback_t callback;
    void *context;

    mask = (stm32wb_exti_device.pending & stm32wb_exti_device.mask) & stm32wb_exti_device.priority[3];
    
    armv7m_atomic_andh(&stm32wb_exti_device.pending, ~mask);

    while (mask) 
    {
        index = __builtin_ctz(mask);

        mask &= ~(1ul << index); 

	armv7m_atomic_load_2((volatile uint32_t*)&stm32wb_exti_device.channels[index].callback, (uint32_t*)&callback, (uint32_t*)&context);

	if (callback)
	{
	    (*callback)(context);
	}
    }
}
