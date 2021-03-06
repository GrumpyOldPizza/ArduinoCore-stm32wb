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
#include "stm32wbxx.h"

#include "stm32wb_random.h"
#include "stm32wb_system.h"
#include "stm32wb_hsem.h"

#define STM32WB_RANDOM_FIFO_SIZE 64

typedef struct _stm32wb_random_device_t {
    bool                           busy;
    uint8_t                        head;
    uint8_t                        tail;
    volatile uint8_t               count;
    uint8_t                        fifo[STM32WB_RANDOM_FIFO_SIZE];
    volatile uint8_t               *xf_status;
    stm32wb_random_done_callback_t xf_callback;
    void                           *xf_context;
    uint8_t * volatile             xf_data;
    uint8_t * volatile             xf_data_e;
} stm32wb_random_device_t;

static stm32wb_random_device_t stm32wb_random_device;

void __stm32wb_random_initialize(void)
{
    NVIC_SetPriority(RNG_IRQn, STM32WB_RANDOM_IRQ_PRIORITY);
    NVIC_EnableIRQ(RNG_IRQn);

    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RNG);
}

bool stm32wb_random(uint8_t *data, uint32_t count, volatile uint8_t *p_status_return, stm32wb_random_done_callback_t callback, void *context)
{
    uint8_t *data_e;

    data_e = data + count;
    
    if (armv7m_atomic_cas((volatile uint32_t*)&stm32wb_random_device.xf_data_e, (uint32_t)NULL, (uint32_t)data_e) != (uint32_t)NULL)
    {
	return false;
    }

    *p_status_return = STM32WB_RANDOM_STATUS_BUSY;

    stm32wb_random_device.xf_status = p_status_return;
    stm32wb_random_device.xf_callback = callback;
    stm32wb_random_device.xf_context = context;
    stm32wb_random_device.xf_data = data;
    
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RNG);
    
    return true;
}

static void stm32wb_random_process(void)
{
    uint32_t tail, count, size;
    volatile uint8_t *xf_status;
    stm32wb_random_done_callback_t xf_callback;
    void *xf_context;
    
    if (stm32wb_random_device.xf_data)
    {
	count = stm32wb_random_device.count;

	if (count)
	{
	    size = stm32wb_random_device.xf_data_e - stm32wb_random_device.xf_data;
	    
	    if (count > size)
	    {
		count = size;
	    }
	    
	    tail = stm32wb_random_device.tail;
	    size = (STM32WB_RANDOM_FIFO_SIZE - tail);
	    
	    if (size > count)
	    {
		size = count;
	    }
	    
	    memcpy(stm32wb_random_device.xf_data, &stm32wb_random_device.fifo[tail], size);
	    
	    stm32wb_random_device.xf_data += size;
	    
	    tail += size;
	    
	    if (tail == STM32WB_RANDOM_FIFO_SIZE)
	    {
		tail = 0;
	    }
	    
	    if (size != count)
	    {
		memcpy(stm32wb_random_device.xf_data, &stm32wb_random_device.fifo[tail], (count - size));
		
		stm32wb_random_device.xf_data += (count - size);
		
		tail += (count - size);
	    }
	    
	    stm32wb_random_device.tail = tail;
	    
	    armv7m_atomic_subb(&stm32wb_random_device.count, count);
	    
	    if (stm32wb_random_device.xf_data == stm32wb_random_device.xf_data_e)
	    {
		xf_status = stm32wb_random_device.xf_status;
		xf_callback = stm32wb_random_device.xf_callback;
		xf_context = stm32wb_random_device.xf_context;
		
		stm32wb_random_device.xf_data = NULL;

		armv7m_atomic_store((volatile uint32_t*)&stm32wb_random_device.xf_data_e, (uint32_t)NULL);

		if (xf_status)
		{
		    *xf_status = STM32WB_RANDOM_STATUS_SUCCESS;
		}
		
		if (xf_callback)
		{
		    (*xf_callback)(xf_context);
		}
	    }
	}
    }

    if (stm32wb_random_device.busy)
    {
	if (stm32wb_random_device.count > (STM32WB_RANDOM_FIFO_SIZE - 16))
	{
	    armv7m_atomic_and(&RNG->CR, ~(RNG_CR_RNGEN | RNG_CR_IE));

	    while (RNG->SR & RNG_SR_DRDY)
	    {
		RNG->DR;
	    }
	    
	    RNG->SR = (RNG_SR_CECS | RNG_SR_SECS);

	    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_RNG);

	    stm32wb_system_unreference(STM32WB_SYSTEM_REFERENCE_RNG);
		
	    stm32wb_system_clk48_disable();
	    
	    stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP_1);
	    
	    stm32wb_hsem_unlock(STM32WB_HSEM_RNG, 0);

	    stm32wb_random_device.busy = false;
	}
    }
    else
    {
	if (stm32wb_random_device.count <= (STM32WB_RANDOM_FIFO_SIZE - 16))
	{
	    if (!stm32wb_hsem_lock(STM32WB_HSEM_RNG, 0))
	    {
		return;
	    }
		
	    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP_1);

	    stm32wb_system_clk48_enable();

	    stm32wb_system_reference(STM32WB_SYSTEM_REFERENCE_RNG);
	    
	    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_RNG);

	    armv7m_atomic_or(&RNG->CR, RNG_CR_RNGEN);

	    stm32wb_random_device.busy = true;
	}
    }

    if (stm32wb_random_device.busy)
    {
	armv7m_atomic_or(&RNG->CR, RNG_CR_IE);
    }
}

static void stm32wb_random_interrupt(void)
{
    uint32_t data[4], head, *fifo;

    data[0] = RNG->DR;
    data[1] = RNG->DR;
    data[2] = RNG->DR;
    data[3] = RNG->DR;
    
    if (!(RNG->SR & (RNG_SR_CECS | RNG_SR_SECS)))
    {
	if (stm32wb_random_device.count <= (STM32WB_RANDOM_FIFO_SIZE - 16))
	{
	    head = stm32wb_random_device.head;
	    fifo = ((uint32_t*)&stm32wb_random_device.fifo[head]);
	    
	    fifo[0] = data[0];
	    fifo[1] = data[1];
	    fifo[2] = data[2];
	    fifo[3] = data[3];
	    
	    head += 16;
	    
	    if (head == STM32WB_RANDOM_FIFO_SIZE)
	    {
		head = 0;
	    }
	    
	    stm32wb_random_device.head = head;
	    
	    armv7m_atomic_addb(&stm32wb_random_device.count, 16);
	    
	    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RNG);
	}
	else
	{
	    armv7m_atomic_and(&RNG->CR, ~RNG_CR_IE);
	}
    }
    
    RNG->SR = 0;
}

void RNG_SWIHandler(void)
{
    stm32wb_random_process();
}

void RNG_HSEMHandler(void)
{
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RNG);
}

void RNG_IRQHandler(void)
{
    stm32wb_random_interrupt();

    __DSB();
}
