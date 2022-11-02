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

#include "stm32wb_exti.h"
#include "stm32wb_gpio.h"

typedef struct _stm32wb_exti_t {
    stm32wb_exti_callback_t volatile callback;
    void * volatile                  context;
} stm32wb_exti_t;

typedef struct _stm32wb_exti_device_t {
    volatile uint32_t       events;
    volatile uint32_t       mask;
    volatile uint32_t       pending;
    volatile uint32_t       priority[4];
    stm32wb_exti_t          channels[21];
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

void __attribute__((optimize("O3"))) __stm32wb_exti_catch(uint32_t index, uint32_t priority, stm32wb_exti_callback_t callback, void *context)
{
    unsigned int mask;

    mask = 1ul << index;

    armv7m_atomic_and(&stm32wb_exti_device.events, ~mask);
    armv7m_atomic_and(&stm32wb_exti_device.pending, ~mask);

    stm32wb_exti_device.channels[index].callback = NULL;
    
    if (callback)
    {
        stm32wb_exti_device.channels[index].context = context;
        stm32wb_exti_device.channels[index].callback = callback;
        
        armv7m_atomic_and(&stm32wb_exti_device.priority[0], ~mask);
        armv7m_atomic_and(&stm32wb_exti_device.priority[1], ~mask);
        armv7m_atomic_and(&stm32wb_exti_device.priority[2], ~mask);
        armv7m_atomic_and(&stm32wb_exti_device.priority[3], ~mask);
        armv7m_atomic_or(&stm32wb_exti_device.priority[priority], mask);
        armv7m_atomic_or(&stm32wb_exti_device.events, mask);
    }
}

void __attribute__((optimize("O3"))) __stm32wb_exti_interrupt(uint32_t mask)
{
    uint32_t mask_0, mask_1, mask_2, index;
    stm32wb_exti_callback_t callback;
    void *context;

    mask_0 = (mask | stm32wb_exti_device.pending) & stm32wb_exti_device.mask;
    
    if (mask_0)
    {
        mask_1 = mask & stm32wb_exti_device.priority[0];
        mask_2 = mask & ~mask_1;
        
        if (mask_1)
        {
            do
            {
                index = __builtin_ctz(mask_1);
                
                mask_1 &= ~(1ul << index); 

                callback = stm32wb_exti_device.channels[index].callback;
                context = stm32wb_exti_device.channels[index].context;

                if (callback)
                {
                    (*callback)(context);
                }
            }
            while (mask_1);
        }

        if (mask_2)
        {
            __armv7m_atomic_or(&stm32wb_exti_device.pending, mask_2);
            
            NVIC_SetPendingIRQ(SPI1_IRQn);
        }
    }

    mask &= stm32wb_exti_device.events;

    if (mask)
    {
        __armv7m_atomic_or(&stm32wb_exti_device.pending, mask);
    }
}

__attribute__((optimize("O3"))) bool stm32wb_exti_catch(uint16_t pin, uint32_t control, stm32wb_exti_callback_t callback, void *context)
{
    unsigned int mask, index, group;

    index = (pin & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT;
    group = (pin & STM32WB_GPIO_PIN_GROUP_MASK) >> STM32WB_GPIO_PIN_GROUP_SHIFT;

    mask = 1ul << index;

    armv7m_atomic_and(&EXTI->IMR1, ~mask);

    __stm32wb_exti_catch(index, ((control & STM32WB_EXTI_CONTROL_PRIORITY_MASK) >> STM32WB_EXTI_CONTROL_PRIORITY_SHIFT), callback, context);

    if (control & (STM32WB_EXTI_CONTROL_EDGE_FALLING | STM32WB_EXTI_CONTROL_EDGE_RISING))
    {
        armv7m_atomic_modify(&SYSCFG->EXTICR[index >> 2], (0x0000000f << ((index & 3) << 2)), (group << ((index & 3) << 2)));
        armv7m_atomic_modify(&EXTI->RTSR1, mask, ((control & STM32WB_EXTI_CONTROL_EDGE_RISING)  ? mask : 0));
        armv7m_atomic_modify(&EXTI->FTSR1, mask, ((control & STM32WB_EXTI_CONTROL_EDGE_FALLING) ? mask : 0));

        armv7m_atomic_or(&EXTI->IMR1, mask);
    }
    
    return true;
}

__attribute__((optimize("O3"))) void stm32wb_exti_block(uint32_t mask)
{
    mask &= stm32wb_exti_device.events;

    armv7m_atomic_and(&stm32wb_exti_device.mask, ~mask);
}

__attribute__((optimize("O3"))) void stm32wb_exti_unblock(uint32_t mask)
{
    mask &= stm32wb_exti_device.events;

    if (~armv7m_atomic_or(&stm32wb_exti_device.mask, mask) & mask)
    {
        NVIC_SetPendingIRQ(EXTI0_IRQn);
    }
}

static __attribute__((optimize("O3"),noinline)) void stm32wb_exti_interrupt(void)
{
    uint32_t mask;

    NVIC_ClearPendingIRQ(EXTI0_IRQn); // stm32wb_exti_unblock() ...
    
    mask = (EXTI->PR1 & 0xffff);
    
    EXTI->PR1 = mask;

    __stm32wb_exti_interrupt(mask);

    __DSB();
}

void EXTI0_IRQHandler(void) __attribute__ ((alias("stm32wb_exti_interrupt")));
void EXTI1_IRQHandler(void) __attribute__ ((alias("stm32wb_exti_interrupt")));
void EXTI2_IRQHandler(void) __attribute__ ((alias("stm32wb_exti_interrupt")));
void EXTI3_IRQHandler(void) __attribute__ ((alias("stm32wb_exti_interrupt")));
void EXTI4_IRQHandler(void) __attribute__ ((alias("stm32wb_exti_interrupt")));
void EXTI9_5_IRQHandler(void) __attribute__ ((alias("stm32wb_exti_interrupt")));
void EXTI15_10_IRQHandler(void) __attribute__ ((alias("stm32wb_exti_interrupt")));

 __attribute__((optimize("O3"))) void SPI1_IRQHandler(void)
{
    uint32_t mask, mask_1, mask_2, index;
    stm32wb_exti_callback_t callback;
    void *context;

    NVIC_ClearPendingIRQ(SPI1_IRQn);
    
    mask = (stm32wb_exti_device.pending & stm32wb_exti_device.mask);

    if (mask)
    {
        mask_1 = mask & stm32wb_exti_device.priority[1];
        mask_2 = mask & ~mask_1;
        
        if (mask_1)
        {
            __armv7m_atomic_and(&stm32wb_exti_device.pending, ~mask_1);
            
            do
            {
                index = __builtin_ctz(mask_1);
                
                mask_1 &= ~(1ul << index); 
                
                callback = stm32wb_exti_device.channels[index].callback;
                context = stm32wb_exti_device.channels[index].context;

                if (callback)
                {
                    (*callback)(context);
                }
            }
            while (mask_1);
        }
        
        if (mask_2)
        {
            if (mask_2 & stm32wb_exti_device.priority[2])
            {
                NVIC_SetPendingIRQ(SPI2_IRQn);
            }
            
            if (mask_2 & stm32wb_exti_device.priority[3])
            {
                armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_EXTI);
            }
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
    
    if (mask)
    {
        __armv7m_atomic_and(&stm32wb_exti_device.pending, ~mask);
        
        do
        {
            index = __builtin_ctz(mask);
            
            mask &= ~(1ul << index); 
            
            callback = stm32wb_exti_device.channels[index].callback;
            context = stm32wb_exti_device.channels[index].context;

            if (callback)
            {
                (*callback)(context);
            }
        }
        while (mask);
    }

    __DSB();
}

 __attribute__((optimize("O3"))) void EXTI_SWIHandler(void)
{
    uint32_t mask, index;
    stm32wb_exti_callback_t callback;
    void *context;

    mask = (stm32wb_exti_device.pending & stm32wb_exti_device.mask) & stm32wb_exti_device.priority[3];

    if (mask)
    {
        armv7m_atomic_and(&stm32wb_exti_device.pending, ~mask);

        do
        {
            index = __builtin_ctz(mask);
            
            mask &= ~(1ul << index); 

            callback = stm32wb_exti_device.channels[index].callback;
            context = stm32wb_exti_device.channels[index].context;

            if (callback)
            {
                (*callback)(context);
            }
        }
        while (mask);
    }
}
