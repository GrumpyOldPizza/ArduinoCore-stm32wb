/*
 * Copyright (c) 2019-2022 Thomas Roell.  All rights reserved.
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
#include "stm32wb_system.h"
#include "stm32wb_lptim.h"

#include "rtos_api.h"
#include "rtos_core.h"

/******************************************************************************************************************************/

static void armv7m_rtos_alarm_insert(k_alarm_t *alarm, uint64_t clock);
static void armv7m_rtos_alarm_remove(k_alarm_t *alarm);
static void armv7m_rtos_alarm_routine(void);
static void armv7m_rtos_alarm_schedule(void);
static void armv7m_rtos_alarm_callback(void *context);
static void armv7m_rtos_alarm_modify(k_alarm_t *alarm, uint64_t clock, uint32_t seconds, uint32_t millis, uint32_t period);

static int __svc_armv7m_rtos_alarm_init(k_alarm_t *alarm, k_alarm_routine_t routine, void *context);
static int __svc_armv7m_rtos_alarm_deinit(k_alarm_t *alarm);
static int __svc_armv7m_rtos_alarm_absolute(k_alarm_t *alarm, uint32_t clock_l, uint32_t clock_h, uint32_t period);
static int __svc_armv7m_rtos_alarm_relative(k_alarm_t *alarm, uint32_t ticks, uint32_t period);
static int __svc_armv7m_rtos_alarm_cancel(k_alarm_t *alarm);

/******************************************************************************************************************************/

static void armv7m_rtos_alarm_insert(k_alarm_t *alarm, uint64_t clock)
{
    k_alarm_t *alarm_element, *alarm_next;
    uint64_t element_clock;

    if (armv7m_rtos_control.alarm_queue == NULL)
    {
	armv7m_rtos_control.alarm_queue = alarm;

	alarm->next = alarm;
	alarm->previous = alarm;
    }
    else
    {
	alarm_element = armv7m_rtos_control.alarm_queue;

	do
	{
	    alarm_next = alarm_element->next;

            element_clock = (((uint64_t)alarm_element->clock_l << 0) | ((uint64_t)alarm_element->clock_h << 32));

	    if (!alarm_element->modify)
	    {
		if (clock < element_clock)
		{
		    if (alarm_element == armv7m_rtos_control.alarm_queue)
		    {
			armv7m_rtos_control.alarm_queue = alarm;
		    }
		    break;
		}
	    }
	    else
	    {
		armv7m_rtos_alarm_remove(alarm_element);
	    }

	    alarm_element = alarm_next;
	}
	while (alarm_element != armv7m_rtos_control.alarm_queue);

	if (armv7m_rtos_control.alarm_queue == NULL)
	{
	    armv7m_rtos_control.alarm_queue = alarm;

	    alarm->next = alarm;
	    alarm->previous = alarm;
	}
	else
	{
	    alarm->previous = alarm_element->previous;
	    alarm->next = alarm_element;
	    
	    alarm->previous->next = alarm;
	    alarm->next->previous = alarm;
	}
    }
}

static void armv7m_rtos_alarm_remove(k_alarm_t *alarm)
{
    if (alarm->next == alarm)
    {
	armv7m_rtos_control.alarm_queue = NULL;
    }
    else
    {
	if (alarm == armv7m_rtos_control.alarm_queue)
	{
	    armv7m_rtos_control.alarm_queue = alarm->next;
	}
	
	alarm->next->previous = alarm->previous;
	alarm->previous->next = alarm->next;
    }
    
    alarm->next = NULL;
    alarm->previous = NULL;
}

static void armv7m_rtos_alarm_schedule(void)
{
    k_alarm_t *alarm;
    uint64_t clock;
    
    alarm = armv7m_rtos_control.alarm_queue;

    if (alarm)
    {
        clock = (((uint64_t)alarm->clock_l << 0) | ((uint64_t)alarm->clock_h << 32));

        if (armv7m_rtos_control.alarm_clock != clock)
        {
            armv7m_rtos_control.alarm_clock = clock;

            stm32wb_lptim_timeout_absolute(&armv7m_rtos_control.alarm_timer, clock, armv7m_rtos_alarm_callback, NULL);
        }
    }
    else
    {
        if (armv7m_rtos_control.alarm_clock)
        {
            armv7m_rtos_control.alarm_clock = 0;

            stm32wb_lptim_timeout_cancel(&armv7m_rtos_control.alarm_timer);
        }
    }
}

static void armv7m_rtos_alarm_routine(void)
{
    k_alarm_t *alarm, *alarm_next, *alarm_previous;
    uint64_t clock;

    alarm = (k_alarm_t*)armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.alarm_modify, (uint32_t)K_ALARM_SENTINEL);
    
    if (alarm != K_ALARM_SENTINEL)
    {
	/* Revert the modify queue, and process it.
	 */
	for (alarm_previous = K_ALARM_SENTINEL; alarm != K_ALARM_SENTINEL; alarm = alarm_next)
	{
	    alarm_next = alarm->modify;
		
	    alarm->modify = alarm_previous;
		
	    alarm_previous = alarm;
	}

	alarm = alarm_previous;
	    
	while (alarm != K_ALARM_SENTINEL)
	{
	    alarm_next = alarm->modify;

	    if (alarm->next)
	    {
		armv7m_rtos_alarm_remove(alarm);
	    }

	    clock = (((uint64_t)alarm->clock_l << 0) | ((uint64_t)alarm->clock_h << 32));
		
	    alarm->modify = NULL;

	    if (!alarm->modify)
	    {
		if (clock)
		{
		    armv7m_rtos_alarm_insert(alarm, clock);
		}
	    }
		
	    alarm = alarm_next;
	}

        armv7m_rtos_alarm_schedule();
    }
}

static void armv7m_rtos_alarm_callback(void *context)
{
    k_alarm_t *alarm;
    uint64_t clock, reference;
    uint32_t clock_l, clock_h, ticks, delta, accum, error;
    
    alarm = armv7m_rtos_control.alarm_queue;

    if (alarm)
    {
        reference = stm32wb_lptim_timeout_clock();

	do
	{
	    clock = (((uint64_t)alarm->clock_l << 0) | ((uint64_t)alarm->clock_h << 32));
            ticks = alarm->ticks;
            delta = alarm->delta;
		
	    if (!alarm->modify)
	    {
		if (clock > reference)
		{
		    break;
		}

		armv7m_rtos_alarm_remove(alarm);
                
                armv7m_rtos_work_submit(&alarm->work);
        
                if (ticks)
                {
                    clock += ticks;
                  
                    accum = (delta >> 16);
                    error = (delta & 0xffff);
                    
                    accum += error;
                    
                    if (accum >= 1000) {
                        accum -= 1000;
                        
                        clock -= 1;
                    }
                    
                    clock_l = (uint32_t)(clock >> 0);
                    clock_h = (uint32_t)(clock >> 32);
                    delta = ((accum << 16) | error);

                    armv7m_atomic_store_4_restart((volatile uint32_t*)&alarm->clock_l, clock_l, clock_h, ticks, delta);

                    if (!alarm->modify)
                    {
                        armv7m_rtos_alarm_insert(alarm, clock);
                    }
                }
                else
                {
                    armv7m_atomic_store_2_restart((volatile uint32_t*)&alarm->clock_l, 0, 0);
                }
	    }
	    else
	    {
		armv7m_rtos_alarm_remove(alarm);
	    }

            alarm = armv7m_rtos_control.alarm_queue;
	}
	while (alarm);

        armv7m_rtos_alarm_schedule();
    }
}

static void armv7m_rtos_alarm_modify(k_alarm_t *alarm, uint64_t clock, uint32_t seconds, uint32_t millis, uint32_t period)
{
    k_alarm_t *alarm_modify;
    uint32_t clock_l, clock_h, ticks, delta, accum, error;

    if (period)
    {
        ticks = (millis * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND) / 1000;
        error = (millis * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND) - (ticks * 1000);
        accum = 0;

        if (error) { ticks += 1; accum = 1000 - error; }
        
        clock += ((uint64_t)(seconds * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND) + ticks);
        
        seconds = period / 1000;
        millis = period - (seconds * 1000);
        
        ticks = (millis * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND) / 1000;
        error = (millis * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND) - (ticks * 1000);
        
        if (error) { ticks += 1; error = 1000 - error; }

        if (seconds > (0xffffffff / STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND)) {
            seconds = 0xffffffff / STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND;
        }
        
        ticks += (seconds * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND);
    }
    else
    {
        ticks = (millis * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND + 999) / 1000;
    
        clock += ((uint64_t)(seconds * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND) + ticks);

        ticks = 0;
        accum = 0;
        error = 0;
    }

    clock_l = (uint32_t)(clock >> 0);
    clock_h = (uint32_t)(clock >> 32);
    delta = (accum << 16) | (error << 0);
    
    armv7m_atomic_store_4_restart((volatile uint32_t*)&alarm->clock_l, clock_l, clock_h, ticks, delta);
    
    if (armv7m_atomic_cas((volatile uint32_t*)&alarm->modify, (uint32_t)NULL, (uint32_t)K_ALARM_SENTINEL) == (uint32_t)NULL)
    {
        alarm_modify = (k_alarm_t*)armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.alarm_modify, (uint32_t)alarm);

        alarm->modify = alarm_modify;

        if (alarm_modify == K_ALARM_SENTINEL)
        {
            armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_ALARM);
        }
    }
}

/******************************************************************************************************************************/

static int __svc_armv7m_rtos_alarm_init(k_alarm_t *alarm, k_alarm_routine_t routine, void *context)
{
    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }
    
    if (!alarm)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (!routine)
    {
        return K_ERR_INVALID_PARAMETER;
    }

    *alarm = K_ALARM_INIT(routine, context);

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_alarm_deinit(k_alarm_t *alarm)
{
    k_alarm_t * volatile *alarm_previous;
    k_alarm_t *alarm_modify;

    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    if (!alarm || !alarm->work.routine)
    {
        return K_ERR_INVALID_OBJECT;
    }

    armv7m_rtos_work_deinit(&alarm->work);
    
    armv7m_atomic_store_2_restart((volatile uint32_t*)&alarm->clock_l, 0, 0);

    if (alarm->modify)
    {
        if (armv7m_atomic_cas((volatile uint32_t*)&armv7m_rtos_control.alarm_modify, (uint32_t)alarm, (uint32_t)alarm->modify) != (uint32_t)alarm)
        {
            for (alarm_previous = &armv7m_rtos_control.alarm_modify, alarm_modify = *alarm_previous; alarm_modify != K_ALARM_SENTINEL; alarm_previous = &alarm_modify->modify, alarm_modify = *alarm_previous)
            {
                if (alarm_modify == alarm)
                {
                    *alarm_previous = alarm->modify;
                    
                    alarm->modify = NULL;

                    break;
                }
            }
        }
    }

    if (alarm->next)
    {
        armv7m_rtos_alarm_remove(alarm);
    }

    armv7m_rtos_alarm_schedule();

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_alarm_absolute(k_alarm_t *alarm, uint32_t clock_l, uint32_t clock_h, uint32_t period)
{
    uint64_t clock;
    uint32_t seconds, millis;

    if (!alarm || !alarm->work.routine)
    {
        return K_ERR_INVALID_OBJECT;
    }
    
    if (!clock_l && !clock_h)
    {
        return K_ERR_INVALID_PARAMETER;
    }

    clock = (((uint64_t)clock_l << 0) | ((uint64_t)clock_h << 32));
    
    seconds = clock / 1000;
    millis = clock - (uint64_t)(seconds * 1000);
    
    armv7m_rtos_alarm_modify(alarm, 0, seconds, millis, period);

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_alarm_relative(k_alarm_t *alarm, uint32_t delay, uint32_t period)
{
    uint32_t seconds, millis;

    if (!alarm || !alarm->work.routine)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (!delay)
    {
        return K_ERR_INVALID_PARAMETER;
    }
    
    seconds = delay / 1000;
    millis = delay - (seconds * 1000);

    armv7m_rtos_alarm_modify(alarm, stm32wb_lptim_timeout_clock(), seconds, millis, period);

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_alarm_cancel(k_alarm_t *alarm)
{
    k_alarm_t *alarm_modify;
    uint32_t clock_l, clock_h;

    if (!alarm || !alarm->work.routine)
    {
        return K_ERR_INVALID_OBJECT;
    }

    armv7m_atomic_load_2_restart(&alarm->clock_l, &clock_l, &clock_h);

    if (!(clock_l | clock_h))
    {
        return K_ERR_ALARM_NOT_ACTIVE;
    }
    
    armv7m_atomic_store_2_restart((volatile uint32_t*)&alarm->clock_l, 0, 0);
    
    if (armv7m_atomic_cas((volatile uint32_t*)&alarm->modify, (uint32_t)NULL, (uint32_t)K_ALARM_SENTINEL) == (uint32_t)NULL)
    {
        alarm_modify = (k_alarm_t*)armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.alarm_modify, (uint32_t)alarm);

        alarm->modify = alarm_modify;

        if (alarm_modify == K_ALARM_SENTINEL)
        {
            armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_ALARM);
        }
    }

    return K_NO_ERROR;
}

/******************************************************************************************************************************/

int k_alarm_init(k_alarm_t *alarm, k_alarm_routine_t routine, void *context)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }
    
    return armv7m_svcall_3((uint32_t)&__svc_armv7m_rtos_alarm_init, (uint32_t)alarm, (uint32_t)routine, (uint32_t)context);
}

int k_alarm_deinit(k_alarm_t *alarm)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }
    
    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_alarm_deinit, (uint32_t)alarm);
}

int k_alarm_absolute(k_alarm_t *alarm, uint64_t clock, uint32_t period)
{
    uint32_t clock_l, clock_h;

    clock_l = (uint32_t)(clock >> 0);
    clock_h = (uint32_t)(clock >> 32);
    
    if (armv7m_core_is_in_interrupt())
    {
        return __svc_armv7m_rtos_alarm_absolute(alarm, clock_l, clock_h, period);
    }

    return armv7m_svcall_4((uint32_t)&__svc_armv7m_rtos_alarm_absolute, (uint32_t)alarm, (uint32_t)clock_l, (uint32_t)clock_h, (uint32_t)period);
}

int k_alarm_relative(k_alarm_t *alarm, uint32_t delay, uint32_t period)
{
    if (armv7m_core_is_in_interrupt())
    {
        return __svc_armv7m_rtos_alarm_relative(alarm, delay, period);
    }

    return armv7m_svcall_3((uint32_t)&__svc_armv7m_rtos_alarm_relative, (uint32_t)alarm, (uint32_t)delay, (uint32_t)period);
}

int k_alarm_cancel(k_alarm_t *alarm)
{
    if (armv7m_core_is_in_interrupt())
    {
        return __svc_armv7m_rtos_alarm_cancel(alarm);
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_alarm_cancel, (uint32_t)alarm);
}

bool k_alarm_is_active(k_alarm_t *alarm)
{
    uint32_t clock_l, clock_h;

    armv7m_atomic_load_2_restart(&alarm->clock_l, &clock_l, &clock_h);

    return !!(clock_l | clock_h);
}

/******************************************************************************************************************************/

void RTOS_ALARM_SWIHandler(void)
{
    armv7m_rtos_alarm_routine();
}

/******************************************************************************************************************************/
