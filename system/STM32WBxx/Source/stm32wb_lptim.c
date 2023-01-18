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
#include "stm32wbxx.h"

#include "stm32wb_lptim.h"
#include "stm32wb_system.h"

#include "stm32wb_gpio.h"

typedef struct _stm32wb_lptim_device_t {
    volatile uint32_t                  event_epoch;
    uint32_t                           event_reference[2];
    stm32wb_lptim_event_callback_t     event_callback;
    void                               *event_context;
    uint32_t                           event_mask;
    volatile uint8_t                   event_lock;
    volatile uint8_t                   event_active;
    volatile uint8_t                   timeout_busy;
    volatile uint8_t                   timeout_sync;
    volatile uint16_t                  timeout_compare[2];
    volatile uint32_t                  timeout_epoch;
    volatile uint32_t                  timeout_clock;
    volatile uint32_t                  timeout_reference;
    stm32wb_lptim_timeout_t            *timeout_queue;
    stm32wb_lptim_timeout_t * volatile timeout_modify;
} stm32wb_lptim_device_t;

static stm32wb_lptim_device_t stm32wb_lptim_device;

#define STM32WB_LPTIM_TIMEOUT_SENTINEL ((stm32wb_lptim_timeout_t*)0x00000001)

/* There is some APB to async LPTIM clock domain sync delay.
 * Measured this is max 5 LPTIM clock cycles (post prescaler).
 * Hence for EVENT use, we simply wait in case of a quick back
 * to back updates. For TIMEOUT, this is handled via ISR, mainly
 * to implement a non-blocking update queue.
 */
#define STM32WB_LPTIM_EVENT_SYNC_THRESHOLD 5

void __stm32wb_lptim_initialize(void)
{
    stm32wb_lptim_device.event_epoch = 0;
    stm32wb_lptim_device.event_lock = 0;
    stm32wb_lptim_device.event_active = 0;
    
    stm32wb_lptim_device.timeout_busy = false;
    stm32wb_lptim_device.timeout_sync = false;
    stm32wb_lptim_device.timeout_epoch = 0;
    stm32wb_lptim_device.timeout_clock = 0;
    stm32wb_lptim_device.timeout_reference = 0;
    stm32wb_lptim_device.timeout_queue = NULL;
    stm32wb_lptim_device.timeout_modify = STM32WB_LPTIM_TIMEOUT_SENTINEL;

    armv7m_atomic_or(&EXTI->IMR1, EXTI_IMR1_IM29);

    NVIC_SetPriority(LPTIM1_IRQn, STM32WB_LPTIM1_IRQ_PRIORITY);
    NVIC_EnableIRQ(LPTIM1_IRQn);
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t stm32wb_lptim_event_clock_read()
{
    uint32_t clock, clock_previous;

    clock = stm32wb_lptim_device.event_epoch + (LPTIM2->CNT & 0xffff);

    do
    {
	clock_previous = clock;
	
	clock = stm32wb_lptim_device.event_epoch + (LPTIM2->CNT & 0xffff);
    }
    while (clock != clock_previous);

    return clock;
}

bool stm32wb_lptim_event_lock(void)
{
    return (armv7m_atomic_casb(&stm32wb_lptim_device.event_lock, 0, 1) == 0);
}

void stm32wb_lptim_event_unlock(void)
{
    stm32wb_lptim_device.event_lock = 0;
}

bool stm32wb_lptim_event_start(uint32_t compare, uint32_t period, uint32_t control, uint8_t priority, stm32wb_lptim_event_callback_t callback, void *context)
{
    if (!stm32wb_lptim_device.event_lock)
    {
	return false;
    }
    
    if (stm32wb_lptim_device.event_active)
    {
	return false;
    }

    NVIC_SetPriority(LPTIM2_IRQn, priority);

    stm32wb_lptim_device.event_callback = callback;
    stm32wb_lptim_device.event_context = context;
    stm32wb_lptim_device.event_reference[0] = 0;
    stm32wb_lptim_device.event_reference[1] = 0;
    stm32wb_lptim_device.event_mask = (control & (STM32WB_LPTIM_CONTROL_COMPARE | STM32WB_LPTIM_CONTROL_PERIOD));
    stm32wb_lptim_device.event_active = 1;

    stm32wb_system_hsi16_enable();

    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_LPTIM2);

    armv7m_atomic_or(&EXTI->IMR1, EXTI_IMR1_IM30);
    
    LPTIM2->IER = LPTIM_IER_ARRMIE | ((control & STM32WB_LPTIM_CONTROL_COMPARE) ? LPTIM_IER_CMPMIE : 0);
    LPTIM2->CFGR = (((control & STM32WB_LPTIM_CONTROL_PRESCALE_MASK) >> STM32WB_LPTIM_CONTROL_PRESCALE_SHIFT) << LPTIM_CFGR_PRESC_Pos) | (!(control & STM32WB_LPTIM_CONTROL_COMPARE) ? LPTIM_CFGR_PRELOAD : 0);

    LPTIM2->CR = LPTIM_CR_ENABLE;
    LPTIM2->CMP = compare & 0xffff;
    LPTIM2->ARR = (period - 1) & 0xffff;
    LPTIM2->CR = LPTIM_CR_CNTSTRT | LPTIM_CR_ENABLE;
    
    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);

    NVIC_EnableIRQ(LPTIM2_IRQn);

    return true;
}

bool stm32wb_lptim_event_stop(void)
{
    if (!stm32wb_lptim_device.event_active)
    {
	return false;
    }

    NVIC_DisableIRQ(LPTIM2_IRQn);

    stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);

    armv7m_atomic_and(&EXTI->IMR1, ~EXTI_IMR1_IM30);
    
    /* ERRATA: MCU may remain stuck in LPTIM interrupt when entering Stop mode
     */
    stm32wb_system_periph_reset(STM32WB_SYSTEM_PERIPH_LPTIM2);

    stm32wb_system_hsi16_disable();

    stm32wb_lptim_device.event_active = 0;
    stm32wb_lptim_device.event_mask = 0;
    stm32wb_lptim_device.event_epoch = 0;

    return true;
}

bool stm32wb_lptim_event_restart(uint32_t period)
{
    uint32_t clock;
    
    if (!stm32wb_lptim_device.event_active)
    {
	return false;
    }

    do
    {
	clock = stm32wb_lptim_event_clock_read();
    }
    while ((clock - stm32wb_lptim_device.event_reference[0]) <= STM32WB_LPTIM_EVENT_SYNC_THRESHOLD);

    stm32wb_lptim_device.event_reference[0] = clock;
    
    LPTIM2->ARR = (period -1) & 0xffff;

    return true;
}

bool stm32wb_lptim_event_compare(uint32_t compare)
{
    uint32_t clock;
    
    if (!stm32wb_lptim_device.event_active)
    {
	return false;
    }

    do
    {
	clock = stm32wb_lptim_event_clock_read();
    }
    while ((clock - stm32wb_lptim_device.event_reference[1]) <= STM32WB_LPTIM_EVENT_SYNC_THRESHOLD);

    stm32wb_lptim_device.event_reference[1] = clock;
    
    LPTIM2->CMP = compare & 0xffff;

    return true;
}

static inline uint32_t __attribute__((optimize("O3"),always_inline)) stm32wb_lptim_timeout_clock(void)
{
    uint32_t clock, clock_previous;

    if (!stm32wb_lptim_device.timeout_busy)
    {
	return 0;
    }
    
    clock = stm32wb_lptim_device.timeout_epoch + (LPTIM1->CNT & 0xffff);

    do
    {
	clock_previous = clock;
	
	clock = stm32wb_lptim_device.timeout_epoch + (LPTIM1->CNT & 0xffff);
    }
    while (clock != clock_previous);

    return clock;
}

uint32_t __stm32wb_lptim_timeout_clock(void)
{
    return stm32wb_lptim_timeout_clock();
}

static void stm32wb_lptim_timeout_remove(stm32wb_lptim_timeout_t *timeout)
{
    if (timeout->next == timeout)
    {
	stm32wb_lptim_device.timeout_queue = NULL;
    }
    else
    {
	if (timeout == stm32wb_lptim_device.timeout_queue)
	{
	    stm32wb_lptim_device.timeout_queue = timeout->next;
	}
	
	timeout->next->previous = timeout->previous;
	timeout->previous->next = timeout->next;
    }
    
    timeout->next = NULL;
    timeout->previous = NULL;
}

static void stm32wb_lptim_timeout_insert(stm32wb_lptim_timeout_t *timeout, uint32_t reference, uint32_t ticks)
{
    stm32wb_lptim_timeout_t *timeout_element, *timeout_next;
    uint32_t clock, element_clock;

    clock = ticks + reference;

    reference = stm32wb_lptim_device.timeout_reference;
    
    if (stm32wb_lptim_device.timeout_queue == NULL)
    {
	stm32wb_lptim_device.timeout_queue = timeout;

	timeout->next = timeout;
	timeout->previous = timeout;
    }
    else
    {
	timeout_element = stm32wb_lptim_device.timeout_queue;

	do
	{
	    timeout_next = timeout_element->next;

            element_clock = timeout_element->clock;

	    if (!timeout_element->modify)
	    {
                if ((clock - reference) < (element_clock - reference))
		{
		    if (timeout_element == stm32wb_lptim_device.timeout_queue)
		    {
			stm32wb_lptim_device.timeout_queue = timeout;
		    }
                    
		    break;
		}
	    }
	    else
	    {
		stm32wb_lptim_timeout_remove(timeout_element);
	    }

	    timeout_element = timeout_next;
	}
	while (timeout_element != stm32wb_lptim_device.timeout_queue);

	if (stm32wb_lptim_device.timeout_queue == NULL)
	{
	    stm32wb_lptim_device.timeout_queue = timeout;

	    timeout->next = timeout;
	    timeout->previous = timeout;
	}
	else
	{
	    timeout->previous = timeout_element->previous;
	    timeout->next = timeout_element;
	    
	    timeout->previous->next = timeout;
	    timeout->next->previous = timeout;
	}
    }

    armv7m_atomic_cas((volatile uint32_t*)&timeout->clock, ticks, clock);
}

static void __attribute__((optimize("O3"))) stm32wb_lptim_timeout_routine(void)
{
    stm32wb_lptim_timeout_t *timeout, *timeout_next, *timeout_previous;
    stm32wb_lptim_timeout_callback_t callback;
    void *context;
    uint32_t clock, reference, ticks;
    uint16_t compare;

    reference = stm32wb_lptim_timeout_clock();

    if (stm32wb_lptim_device.timeout_modify != STM32WB_LPTIM_TIMEOUT_SENTINEL)
    {
        timeout = (stm32wb_lptim_timeout_t*)armv7m_atomic_swap((volatile uint32_t*)&stm32wb_lptim_device.timeout_modify, (uint32_t)STM32WB_LPTIM_TIMEOUT_SENTINEL);

	/* Revert the modify queue, and process it.
	 */
	for (timeout_previous = STM32WB_LPTIM_TIMEOUT_SENTINEL; timeout != STM32WB_LPTIM_TIMEOUT_SENTINEL; timeout = timeout_next)
	{
	    timeout_next = timeout->modify;
		
	    timeout->modify = timeout_previous;
		
	    timeout_previous = timeout;
	}

	timeout = timeout_previous;
	    
	while (timeout != STM32WB_LPTIM_TIMEOUT_SENTINEL)
	{
	    timeout_next = timeout->modify;

	    if (timeout->next)
	    {
		stm32wb_lptim_timeout_remove(timeout);
	    }

	    ticks = timeout->clock;
            
	    timeout->modify = NULL;

	    if (!timeout->modify)
	    {
		if (ticks)
		{
		    stm32wb_lptim_timeout_insert(timeout, reference, ticks);
		}
	    }
		
	    timeout = timeout_next;
	}
    }
    
    ticks = reference - stm32wb_lptim_device.timeout_reference;
    
    timeout = stm32wb_lptim_device.timeout_queue;

    if (timeout)
    {
	do
	{
            clock = timeout->clock;
            callback = timeout->callback;
            context = timeout->context;
		
	    if (!timeout->modify)
	    {
		if (ticks < (clock - stm32wb_lptim_device.timeout_reference))
		{
		    break;
		}

		stm32wb_lptim_timeout_remove(timeout);

		(*callback)(context);
	    }
	    else
	    {
		stm32wb_lptim_timeout_remove(timeout);
	    }

            timeout = stm32wb_lptim_device.timeout_queue;
	}
	while (timeout);
    }

    stm32wb_lptim_device.timeout_reference = reference;
    
    if (!armv7m_pendsv_is_pending(ARMV7M_PENDSV_SWI_LPTIM_TIMEOUT))
    {
        timeout = stm32wb_lptim_device.timeout_queue;
        
        if (timeout)
        {
            clock = timeout->clock;
            
            if (!timeout->modify)
            {
                if (stm32wb_lptim_device.timeout_clock != clock)
                {
                    compare = clock & 0xffff;

                    if (!stm32wb_lptim_device.timeout_busy)
                    {
                        stm32wb_lptim_device.timeout_busy = true;
                        stm32wb_lptim_device.timeout_sync = true;
                        stm32wb_lptim_device.timeout_compare[0] = compare;
                        stm32wb_lptim_device.timeout_compare[1] = compare;
                        stm32wb_lptim_device.timeout_epoch = 0;

                        stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_LPTIM1);
                        
                        LPTIM1->IER = LPTIM_IER_CMPOKIE | LPTIM_IER_CMPMIE | LPTIM_IER_ARRMIE;
                        LPTIM1->CFGR = 0;
                        
                        LPTIM1->CR = LPTIM_CR_ENABLE;
                        LPTIM1->CMP = compare;
                        LPTIM1->ARR = 0xffff;
                        LPTIM1->CR = LPTIM_CR_CNTSTRT | LPTIM_CR_ENABLE;

                        stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
                    }
                    else
                    {
                        if (stm32wb_lptim_device.timeout_compare[1] != compare)
                        {
                            stm32wb_lptim_device.timeout_compare[1] = compare;
                            
                            if (!stm32wb_lptim_device.timeout_sync)
                            {
                                stm32wb_lptim_device.timeout_sync = true;
                                stm32wb_lptim_device.timeout_compare[0] = compare;
                                
                                stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
                                
                                LPTIM1->CMP = compare;
                            }
                        }
                    }

                    stm32wb_lptim_device.timeout_clock = clock;
                }
            }
        }
        else
        {
            if (stm32wb_lptim_device.timeout_busy)
            {
                stm32wb_system_periph_reset(STM32WB_SYSTEM_PERIPH_LPTIM1);

                stm32wb_lptim_device.timeout_busy = false;
                stm32wb_lptim_device.timeout_sync = false;
                stm32wb_lptim_device.timeout_epoch = 0;
                stm32wb_lptim_device.timeout_clock = 0;
                stm32wb_lptim_device.timeout_reference = 0;
            }
        }
    }
}

static void __svc_stm32wb_lptim_timeout_modify(stm32wb_lptim_timeout_t *timeout, uint32_t ticks, stm32wb_lptim_timeout_callback_t callback, void *context)
{
    stm32wb_lptim_timeout_t *timeout_modify;
    
    __armv7m_atomic_store_3_restart((volatile uint32_t*)&timeout->clock, ticks, (uint32_t)callback, (uint32_t)context);
    
    if (armv7m_atomic_cas((volatile uint32_t*)&timeout->modify, (uint32_t)NULL, (uint32_t)STM32WB_LPTIM_TIMEOUT_SENTINEL) == (uint32_t)NULL)
    {
        timeout_modify = (stm32wb_lptim_timeout_t*)armv7m_atomic_swap((volatile uint32_t*)&stm32wb_lptim_device.timeout_modify, (uint32_t)timeout);

        timeout->modify = timeout_modify;

        if (timeout_modify == STM32WB_LPTIM_TIMEOUT_SENTINEL)
        {
            armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_LPTIM_TIMEOUT);
        }
    }
}

bool stm32wb_lptim_timeout_start(stm32wb_lptim_timeout_t *timeout, uint32_t ticks, stm32wb_lptim_timeout_callback_t callback, void *context)
{
    if (!ticks)
    {
        return false;
    }
    
    if (!armv7m_core_is_in_thread())
    {
	__svc_stm32wb_lptim_timeout_modify(timeout, ticks, callback, context);
    }
    else
    {
	armv7m_svcall_4((uint32_t)&__svc_stm32wb_lptim_timeout_modify, (uint32_t)timeout, ticks, (uint32_t)callback, (uint32_t)context);
    }

    return true;
}

void stm32wb_lptim_timeout_stop(stm32wb_lptim_timeout_t *timeout)
{
    if (!armv7m_core_is_in_thread())
    {
        __svc_stm32wb_lptim_timeout_modify(timeout, 0, NULL, NULL);
    }
    else
    {
	armv7m_svcall_4((uint32_t)&__svc_stm32wb_lptim_timeout_modify, (uint32_t)timeout, 0, (uint32_t)NULL, (uint32_t)NULL);
    }
}

bool stm32wb_lptim_timeout_is_pending(stm32wb_lptim_timeout_t *timeout)
{
    stm32wb_lptim_timeout_t *previous, *modify;
    uint32_t clock;

    __armv7m_atomic_load_3_restart((volatile uint32_t*)&timeout->previous, (uint32_t*)&previous, (uint32_t*)&modify, &clock);

    return ((modify != NULL) ? (clock != 0) : (previous != NULL));
}

__attribute__((optimize("O3"))) void LPTIM1_IRQHandler(void)
{
    uint32_t lptim_isr, clock;
    uint16_t compare;
    
    lptim_isr = LPTIM1->ISR;
    
    if (lptim_isr & LPTIM_ISR_CMPOK)
    {
        LPTIM1->ICR = LPTIM_ICR_CMPOKCF;
        
        compare = stm32wb_lptim_device.timeout_compare[1];
        
        if (compare != stm32wb_lptim_device.timeout_compare[0])
        {
            stm32wb_lptim_device.timeout_compare[0] = compare;
            
            LPTIM1->CMP = compare;
        }
        else
        {
            stm32wb_lptim_device.timeout_sync = 0;
            
            stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);
        }
    }
    
    if (lptim_isr & LPTIM_ISR_CMPM)
    {
        LPTIM1->ICR = LPTIM_ICR_CMPMCF;
    }
    
    if (lptim_isr & LPTIM_ISR_ARRM)
    {
        LPTIM1->ICR = LPTIM_ICR_ARRMCF;
        
        stm32wb_lptim_device.timeout_epoch += 0x00010000;
    }

    clock = stm32wb_lptim_timeout_clock();

    if ((clock - stm32wb_lptim_device.timeout_reference) >= (stm32wb_lptim_device.timeout_clock - stm32wb_lptim_device.timeout_reference))
    {
        armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_LPTIM_TIMEOUT);
    }
    
    __DSB();
}

__attribute__((optimize("O3"))) void LPTIM2_IRQHandler(void)
{
    uint32_t lptim_isr, events;
    stm32wb_lptim_event_callback_t callback;
    void *context;

    events = 0;
    
    lptim_isr = LPTIM2->ISR;

    if (lptim_isr & LPTIM_ISR_ARRM)
    {
	LPTIM2->ICR = LPTIM_ICR_ARRMCF;

	stm32wb_lptim_device.event_epoch += 0x00010000u;
	
	if (stm32wb_lptim_device.event_mask & STM32WB_LPTIM_EVENT_PERIOD)
	{
	    events |= STM32WB_LPTIM_EVENT_PERIOD;
	}
    }
    
    if (stm32wb_lptim_device.event_mask & STM32WB_LPTIM_EVENT_COMPARE)
    {
	if (lptim_isr & LPTIM_ISR_CMPM)
	{
	    LPTIM2->ICR = LPTIM_ICR_CMPMCF;

	    events |= STM32WB_LPTIM_EVENT_COMPARE;
	}
    }

    if (events)
    {
	callback = stm32wb_lptim_device.event_callback;
	context = stm32wb_lptim_device.event_context;

	if (callback)
	{
	    (*callback)(context, events);
	}
    }

    __DSB();
}

void LPTIM_TIMEOUT_SWIHandler(void)
{
    stm32wb_lptim_timeout_routine();
}
