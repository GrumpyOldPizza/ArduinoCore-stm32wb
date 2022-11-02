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

typedef struct _k_event_receive_params_t {
    uint32_t                    mask;
    uint32_t                    mode;
    uint32_t                    timeout;
    uint32_t                    *p_mask_return;
} k_event_receive_params_t;

static void armv7m_rtos_event_send_process(k_event_t *event);
static void armv7m_rtos_event_send_routine(void);

static int __svc_armv7m_rtos_event_init(k_event_t *event);
static int __svc_armv7m_rtos_event_deinit(k_event_t *event);
static int __svc_armv7m_rtos_event_send(k_event_t *event, uint32_t mask);
static int __svc_armv7m_rtos_event_receive(k_event_t *event, const k_event_receive_params_t *params);

/******************************************************************************************************************************/

void armv7m_rtos_event_unblock(k_task_t *task)
{
    k_event_t *event;

    event = task->wait.event.event;
    
    armv7m_rtos_task_queue_remove(&event->waiting, task);
}

static void armv7m_rtos_event_send_process(k_event_t *event)
{
    k_task_t *task, *task_next, *task_head, *task_tail;
    k_event_t *event_send;
    uint32_t mask, mask_reset, mask_return;
    
    do
    {
        event_send = event->send;

        event->send = NULL;

        mask = event->mask;
        mask_reset = 0;
        
        task_head = event->waiting;

        if (task_head)
        {
            task_tail = task_head->previous;
            task_next = task_head;
            
            do
            {
                task = task_next;
                task_next = task->next;

                mask_return = mask & task->wait.event.mask;

                if (task->state & K_TASK_STATE_EVENT_ALL)
                {
                    if (mask_return != task->wait.event.mask)
                    {
                        mask_return = 0;
                    }
                }

                if (mask_return)
                {
                    if (task->state & K_TASK_STATE_EVENT_CLEAR)
                    {
                        mask_reset |= mask_return;
                    }

                    if (task->wait.event.p_mask_return)
                    {
                        *task->wait.event.p_mask_return = mask_return;
                    }
                    
                    armv7m_rtos_task_queue_remove(&event->waiting, task);

                    armv7m_rtos_task_release(task);
                }
            }
            while (task != task_tail);

            armv7m_atomic_and(&event->mask, ~mask_reset);
        }
        
        event = event_send;
    }
    while (event != K_EVENT_SENTINEL);

    armv7m_rtos_task_timeout_schedule();
    
    armv7m_rtos_task_schedule();
}

static void armv7m_rtos_event_send_routine(void)
{
    k_event_t *event_send, *event_previous, *event_next;

    event_send = (k_event_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.event_send, (uint32_t)K_EVENT_SENTINEL);

    if (event_send != K_EVENT_SENTINEL)
    {
        if (event_send->send != K_EVENT_SENTINEL)
        {
            for (event_previous = K_EVENT_SENTINEL; event_send != K_EVENT_SENTINEL; event_send = event_next)
            {
                event_next = event_send->send;
                
                event_send->send = event_previous;
                
                event_previous = event_send;
            }
            
            event_send = event_previous;
        }
        
        armv7m_rtos_event_send_process(event_send);
    }
}

/******************************************************************************************************************************/

static int __svc_armv7m_rtos_event_init(k_event_t *event)
{
    // armv7m_rtt_printf("k_eventinit(event%08x)\n", event);
    
    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }
    
    if (!event)
    {
        return K_ERR_INVALID_OBJECT;
    }
    
    *event = K_EVENT_INIT();

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_event_deinit(k_event_t *event)
{
    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    if (!event)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (event->waiting)
    {
        do
        {
            armv7m_rtos_task_unblock(event->waiting, K_ERR_UNSATISFIED);
        }
        while (event->waiting);

        armv7m_rtos_task_timeout_schedule();
        
        armv7m_rtos_task_schedule();
    }
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_event_send(k_event_t *event, uint32_t mask)
{
    k_event_t *event_send;
    
    // armv7m_rtt_printf("k_event_send(event=%08x, mask=%08x)\n", event, mask);

    if (!event)
    {
        return K_ERR_INVALID_OBJECT;
    }

    armv7m_atomic_or(&event->mask, mask);

    if (__armv7m_atomic_cas((volatile uint32_t*)&event->send, (uint32_t)NULL, (uint32_t)K_EVENT_SENTINEL) == (uint32_t)NULL)
    {
        if (armv7m_core_is_in_svcall_or_pendsv())
        {
            armv7m_rtos_event_send_process(event);
        }
        else
        {
            event_send = (k_event_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.event_send, (uint32_t)event);
            
            event->send = event_send;
            
            if (event_send == K_EVENT_SENTINEL)
            {
                armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_EVENT_SEND);
            }
        }
    }

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_event_receive(k_event_t *event, const k_event_receive_params_t *params)
{
    k_task_t *self;
    uint32_t mask, mask_return;

    // armv7m_rtt_printf("k_event_receive(event=%08x, mask=%08x, mode=%08x, timeout=%d, p_mask_return=%08x)\n", event, params->mask, params->mode, params->timeout, params->p_mask_return);

    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    self = armv7m_rtos_control.task_self;

    if (!self)
    {
        return K_ERR_ILLEGAL_USE;
    }

    if (!event)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (armv7m_rtos_control.system_state != K_STATE_RUNNING)
    {
        if (params->timeout != K_TIMEOUT_NONE)
        {
            return K_ERR_INVALID_PARAMETER;
        }
    }

    mask = event->mask;
    mask_return = params->mask & mask;

    if (params->mode & K_EVENT_ALL)
    {
        if (mask_return != mask)
        {
            mask_return = 0;
        }
    }
    
    if (mask_return)
    {
        if (params->mode & K_EVENT_CLEAR)
        {
            armv7m_atomic_and(&event->mask, ~mask_return);
        }
        
        if (params->p_mask_return)
        {
            *params->p_mask_return = mask_return;
        }

        return K_NO_ERROR;
    }

    if (params->timeout == K_TIMEOUT_NONE)
    {
        return K_ERR_UNSATISFIED;
    }

    armv7m_rtos_task_ready_remove(self);

    self->wait.event.event = event;
    self->wait.event.mask = params->mask;
    self->wait.event.p_mask_return = params->p_mask_return;

    armv7m_rtos_task_queue_insert(&event->waiting, self);
    
    self->state |= (K_TASK_STATE_WAIT_EVENT | ((params->mode & K_EVENT_ALL) ? K_TASK_STATE_EVENT_ALL : 0) | ((params->mode & K_EVENT_CLEAR) ? K_TASK_STATE_EVENT_CLEAR : 0));
    
    if (params->timeout != K_TIMEOUT_FOREVER)
    {
        armv7m_rtos_task_timeout_relative(self, params->timeout);
    }

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_block)(self, (self->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */

    armv7m_rtos_task_schedule();

    return K_NO_ERROR;
}

/******************************************************************************************************************************/

int k_event_init(k_event_t *event)
{
    // armv7m_rtt_printf("k_event_init(event=%08x)\n", event);
    
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_event_init, (uint32_t)event);
}

int k_event_deinit(k_event_t *event)
{
    // armv7m_rtt_printf("k_event_deinit(event=%08x)\n", event);

    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_event_deinit, (uint32_t)event);
}

int k_event_send(k_event_t *event, uint32_t mask)
{
    if (armv7m_core_is_in_interrupt())
    {
        return __svc_armv7m_rtos_event_send(event, mask);
    }

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_event_send, (uint32_t)event, (uint32_t)mask);
}

int k_event_receive(k_event_t *event, uint32_t mask, uint32_t mode, uint32_t timeout, uint32_t *p_mask_return)
{
    k_event_receive_params_t params;
    uint32_t mask_return;

    if (armv7m_core_is_in_interrupt())
    {
        if (!event)
        {
            return K_ERR_INVALID_OBJECT;
        }

        if (timeout != K_TIMEOUT_NONE)
        {
            return K_ERR_INVALID_PARAMETER;
        }

        mask_return = event->mask & mask;

        if (mode & K_EVENT_ALL)
        {
            if (mask_return != mask)
            {
                mask_return = 0;
            }
        }
    
        if (mask_return)
        {
            if (mode & K_EVENT_CLEAR)
            {
                armv7m_atomic_and(&event->mask, ~mask_return);
            }
            
            if (p_mask_return)
            {
                *p_mask_return = mask_return;
            }

            return K_NO_ERROR;
        }

        return K_ERR_UNSATISFIED;
    }

    params.mask = mask;
    params.mode = mode;
    params.timeout = timeout;
    params.p_mask_return = p_mask_return;
    
    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_event_receive, (uint32_t)event, (uint32_t)&params);
}

uint32_t k_event_mask(k_event_t *event)
{
    if (!event)
    {
        return 0;
    }

    return event->mask;
}

/******************************************************************************************************************************/

void RTOS_EVENT_SEND_SWIHandler(void)
{
    armv7m_rtos_event_send_routine();
}
