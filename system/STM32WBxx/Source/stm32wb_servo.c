/*
 * Copyright (c) 2016-2021 Thomas Roell.  All rights reserved.
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

#include <stdio.h>

#include "stm32wbxx.h"

#include "armv7m.h"

#include "stm32wb_gpio.h"
#include "stm32wb_servo.h"
#include "stm32wb_system.h"

typedef struct _stm32wb_servo_schedule_t {
    uint32_t                            entries;
    struct {
	GPIO_TypeDef                        *GPIO;
	uint16_t                            mask;
	uint16_t                            width;
    }                                   slot[STM32WB_SERVO_CHANNEL_COUNT];
} stm32wb_servo_schedule_t;

typedef struct _stm32wb_servo_channel_t {
    uint16_t                            pin;
    uint16_t                            width;
} stm32wb_servo_channel_t;

typedef struct _stm32wb_servo_device_t {
    volatile uint8_t                    lock;
    uint8_t                             priority;
    uint8_t                             slot;
    uint16_t                            compare;
    stm32wb_servo_schedule_t * volatile current;
    stm32wb_servo_schedule_t * volatile next;
    stm32wb_servo_schedule_t            schedule[3];
    volatile stm32wb_servo_channel_t    channels[STM32WB_SERVO_CHANNEL_COUNT];
    stm32wb_servo_callback_t            callback;
    void                                *context;
} stm32wb_servo_device_t;

static stm32wb_servo_device_t stm32wb_servo_device;

static void stm32wb_servo_callback(void *context, uint32_t events)
{
    uint32_t slot;
    stm32wb_servo_schedule_t *current;
    stm32wb_servo_callback_t callback;
    
    if (events & STM32WB_LPTIM_EVENT_COMPARE)
    {
	slot = stm32wb_servo_device.slot;
	current = stm32wb_servo_device.current;

	if (current && (slot != current->entries))
	{
	    current->slot[slot].GPIO->BRR = current->slot[slot].mask;
	    slot++;

	    if (slot != current->entries)
	    {
		current->slot[slot].GPIO->BSRR = current->slot[slot].mask;

		stm32wb_servo_device.compare += current->slot[slot].width;

		stm32wb_lptim_event_compare(stm32wb_servo_device.compare -1);
	    }

	    stm32wb_servo_device.slot = slot;
	}
    }
    
    if (events & STM32WB_LPTIM_EVENT_PERIOD)
    {
	stm32wb_servo_device.slot = 0;
	stm32wb_servo_device.current = stm32wb_servo_device.next;
	
	current = stm32wb_servo_device.current;

	if (current)
	{
	    if (current->entries)
	    {
		current->slot[0].GPIO->BSRR = current->slot[0].mask;
		
		stm32wb_servo_device.compare = current->slot[0].width;
		
		stm32wb_lptim_event_compare(stm32wb_servo_device.compare -1);
	    }
	    else
	    {
		stm32wb_lptim_event_stop();
		
		stm32wb_servo_device.current = NULL;
	    }

	    callback = stm32wb_servo_device.callback;
	    context = stm32wb_servo_device.context;
	    
	    if (callback)
	    {
		(*callback)(context);
	    }
	}
    }
}

static void stm32wb_servo_routine(void)
{
    uint32_t slot, index;
    stm32wb_servo_schedule_t *next;

    if (stm32wb_servo_device.next)
    {
	next = stm32wb_servo_device.next +1;

	if (next > &stm32wb_servo_device.schedule[2])
	{
	    next = &stm32wb_servo_device.schedule[0];
	}

	if (next == stm32wb_servo_device.current)
	{
	    next = next +1;
	
	    if (next > &stm32wb_servo_device.schedule[2])
	    {
		next = &stm32wb_servo_device.schedule[0];
	    }
	}
    }
    else
    {
	next = &stm32wb_servo_device.schedule[0];
    }
    
    for (slot = 0, index = 0; index < STM32WB_SERVO_CHANNEL_COUNT; index++)
    {
	if (stm32wb_servo_device.channels[index].pin != STM32WB_GPIO_PIN_NONE)
	{
	    next->slot[slot].GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * ((stm32wb_servo_device.channels[index].pin & STM32WB_GPIO_PIN_GROUP_MASK) >> STM32WB_GPIO_PIN_GROUP_SHIFT));
	    next->slot[slot].mask = (1ul << ((stm32wb_servo_device.channels[index].pin & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT));
	    next->slot[slot].width = stm32wb_servo_device.channels[index].width;
	    slot++;
	}
    }

    next->entries = slot;

    stm32wb_servo_device.next = next;

    if (!stm32wb_servo_device.current)
    {
	stm32wb_servo_device.slot = 0;
	stm32wb_servo_device.compare = next->slot[0].width;
	stm32wb_servo_device.current = next;

	next->slot[0].GPIO->BSRR = next->slot[0].mask;
	
	stm32wb_lptim_event_start(stm32wb_servo_device.compare -1, STM32WB_SERVO_FRAME_WIDTH, (STM32WB_LPTIM_CONTROL_COMPARE | STM32WB_LPTIM_CONTROL_PERIOD | STM32WB_LPTIM_CONTROL_PRESCALE_16),
				  stm32wb_servo_device.priority, (stm32wb_lptim_event_callback_t)stm32wb_servo_callback, NULL);
    }
}

bool stm32wb_servo_enable(uint8_t priority, stm32wb_servo_callback_t callback, void *context)
{
    uint32_t index;
  
    if (armv7m_atomic_casb(&stm32wb_servo_device.lock, 0, 1) != 0)
    {
	return false;
    }

    if (!stm32wb_lptim_event_lock())
    {
	stm32wb_servo_device.lock = 0;

	return false;
    }

    stm32wb_servo_device.priority = priority;
    stm32wb_servo_device.callback = callback;
    stm32wb_servo_device.context = context;

    for (index = 0; index < STM32WB_SERVO_CHANNEL_COUNT; index++)
    {
	stm32wb_servo_device.channels[index].pin = STM32WB_GPIO_PIN_NONE;
    }
    
    stm32wb_servo_device.current = NULL;
    
    return true;
}

void stm32wb_servo_disable(void)
{
    if (stm32wb_servo_device.lock)
    {
	stm32wb_lptim_event_stop();
	stm32wb_lptim_event_unlock();

	stm32wb_servo_device.current = NULL;
	stm32wb_servo_device.next = NULL;
	stm32wb_servo_device.lock = 0;
    }
}

static bool __svc_stm32wb_servo_channel(uint32_t index, uint16_t pin, uint16_t width)
{
    if (!stm32wb_servo_device.lock)
    {
	return false;
    }

    if (index >= STM32WB_SERVO_CHANNEL_COUNT)
    {
	return false;
    }

    if (width && ((width < STM32WB_SERVO_PULSE_WIDTH_MIN) || (width > STM32WB_SERVO_PULSE_WIDTH_MAX)))
    {
	return false;
    }

    if ((pin == STM32WB_GPIO_PIN_NONE) || (width == 0))
    {
	stm32wb_servo_device.channels[index].pin = STM32WB_GPIO_PIN_NONE;
	stm32wb_servo_device.channels[index].width = 0;
    }
    else
    {
	stm32wb_servo_device.channels[index].pin = pin;
	stm32wb_servo_device.channels[index].width = width;
    }

    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_SERVO);
    
    return true;
}

bool stm32wb_servo_channel(uint32_t index, uint16_t pin, uint16_t width)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_3((uint32_t)&__svc_stm32wb_servo_channel, (uint32_t)index, (uint32_t)pin, (uint32_t)width);
    }
    else
    {
	return __svc_stm32wb_servo_channel(index, pin, width);
    }
}

void SERVO_SWIHandler(void)
{
    stm32wb_servo_routine();
}
