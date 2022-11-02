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
#include "stm32wb_gpio.h"

#include "rtos_api.h"
#include "rtos_core.h"

typedef struct _k_task_params_t {
    const char                  *name;
    k_task_routine_t            routine;
    void                        *context;
    uint32_t                    priority;
    void                        *stack_base;
    uint32_t                    stack_size;
    uint32_t                    options;
} k_task_params_t;

static uint32_t armv7m_rtos_task_state(k_task_t *task);

static void armv7m_rtos_task_resume_process(k_task_t *task);
static void armv7m_rtos_task_resume_routine(void);

static void armv7m_rtos_task_wakeup_process(k_task_t *task);
static void armv7m_rtos_task_wakeup_routine(void);

static int __svc_armv7m_rtos_task_create(k_task_t *task, const k_task_params_t *params);
static int __svc_armv7m_rtos_task_terminate(k_task_t *task);
static int __svc_armv7m_rtos_task_detach(k_task_t *task);
static int __svc_armv7m_rtos_task_join(k_task_t *task);
static int __svc_armv7m_rtos_task_enumerate(uint32_t *p_count_return, k_task_t **p_task_return, uint32_t count);
static int __svc_armv7m_rtos_task_info(k_task_t *task, k_task_info_t *p_task_info_return);
static int __svc_armv7m_rtos_task_stack(k_task_t *task, uint32_t *p_stack_base_return, uint32_t *p_stack_size_return, uint32_t *p_stack_space_return);
static int __svc_armv7m_rtos_task_unblock(k_task_t *task);
static int __svc_armv7m_rtos_task_suspend(k_task_t *task);
static int __svc_armv7m_rtos_task_resume(k_task_t *task);
static int __svc_armv7m_rtos_task_set_priority(k_task_t *task, uint32_t priority, uint32_t *p_priority_return);
static int __svc_armv7m_rtos_task_get_priority(k_task_t *task, uint32_t *p_priority_return);
static int __svc_armv7m_rtos_task_delay(uint32_t delay);
static int __svc_armv7m_rtos_task_delay_until(uint32_t clock_l, uint32_t clock_h);
static int __svc_armv7m_rtos_task_sleep(uint32_t timeout);
static int __svc_armv7m_rtos_task_wakeup(k_task_t *task);
static int __svc_armv7m_rtos_task_yield(void);

/******************************************************************************************************************************/

static uint32_t armv7m_rtos_task_state(k_task_t *task)
{
    return ((task->state & K_TASK_STATE_TERMINATED)
            ? K_STATE_TERMINATED
            : ((task->state & K_TASK_STATE_SUSPENDED)
               ? K_STATE_SUSPENDED
               : ((task->state & K_TASK_STATE_WAIT_MASK)
                  ? K_STATE_BLOCKED
                  : ((task->state & K_TASK_STATE_READY)
                     ? ((task != armv7m_rtos_control.task_self) ? K_STATE_READY : K_STATE_RUNNING)
                     : K_STATE_INACTIVE))));
}

void armv7m_rtos_join_release(k_task_t *task)
{
    armv7m_rtos_task_destroy(task->wait.join.task);

    task->state &= ~K_TASK_STATE_WAIT_MASK;

    if (!(task->state & K_TASK_STATE_SUSPENDED))
    {
        armv7m_rtos_task_ready_insert(task);
                
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
        (*armv7m_rtos_control.hook_table->task_ready)(task);
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
    }
    else
    {
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
        (*armv7m_rtos_control.hook_table->task_block)(task, (task->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
    }
}

void armv7m_rtos_join_unblock(k_task_t *task)
{
    task->wait.join.task->join = NULL;
}

void armv7m_rtos_delay_unblock(k_task_t *task)
{
}

void armv7m_rtos_sleep_unblock(k_task_t *task)
{
}

static void armv7m_rtos_task_resume_process(k_task_t *task)
{
    k_task_t *task_resume;
  
    do
    {
        task_resume = task->resume;
            
        task->state &= ~K_TASK_STATE_SUSPENDED;

        if (!(task->state & K_TASK_STATE_TERMINATED))
        {
            if (!(task->state & K_TASK_STATE_WAIT_MASK))
            {
                armv7m_rtos_task_ready_insert(task);
                
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
                (*armv7m_rtos_control.hook_table->task_ready)(task);
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
            }
            else
            {
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
                (*armv7m_rtos_control.hook_table->task_block)(task, (task->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
            }
        }
            
        task->resume = NULL;

        task = task_resume;
    }
    while (task != K_TASK_SENTINEL);

    armv7m_rtos_task_schedule();
}

static void armv7m_rtos_task_resume_routine(void)
{
    k_task_t *task_resume, *task_previous, *task_next;

    task_resume = (k_task_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_resume, (uint32_t)K_TASK_SENTINEL);

    if (task_resume != K_TASK_SENTINEL)
    {
        if (task_resume->resume != K_TASK_SENTINEL)
        {
            for (task_previous = K_TASK_SENTINEL; task_resume != K_TASK_SENTINEL; task_resume = task_next)
            {
                task_next = task_resume->resume;
                
                task_resume->resume = task_previous;
                
                task_previous = task_resume;
            }
            
            task_resume = task_previous;
        }
        
        armv7m_rtos_task_resume_process(task_resume);
    }
}

static void armv7m_rtos_task_wakeup_process(k_task_t *task)
{
    k_task_t *task_wakeup;

    do
    {
        task_wakeup = task->wakeup;

        task->state |= K_TASK_STATE_WAKEUP;
        
        if ((task->state & K_TASK_STATE_WAIT_MASK) == K_TASK_STATE_WAIT_SLEEP)
        {
            task->state &= ~K_TASK_STATE_WAKEUP;

            armv7m_rtos_task_release(task);
        }

        task->wakeup = NULL;

        task = task_wakeup;
    }
    while (task != K_TASK_SENTINEL);

    armv7m_rtos_task_timeout_schedule();

    armv7m_rtos_task_schedule();
}

static void armv7m_rtos_task_wakeup_routine(void)
{
    k_task_t *task_wakeup, *task_previous, *task_next;

    task_wakeup = (k_task_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_wakeup, (uint32_t)K_TASK_SENTINEL);

    if (task_wakeup != K_TASK_SENTINEL)
    {
        if (task_wakeup->wakeup != K_TASK_SENTINEL)
        {
            for (task_previous = K_TASK_SENTINEL; task_wakeup != K_TASK_SENTINEL; task_wakeup = task_next)
            {
                task_next = task_wakeup->wakeup;
                
                task_wakeup->wakeup = task_previous;
                
                task_previous = task_wakeup;
            }
            
            task_wakeup = task_previous;
        }
        
        armv7m_rtos_task_wakeup_process(task_wakeup);
    }
}

/******************************************************************************************************************************/

static int __svc_armv7m_rtos_task_create(k_task_t *task, const k_task_params_t *params)
{
    // armv7m_rtt_printf("k_task_create(task=%08x, name=\"%s\", routine=%08x, context=%0x, priority=%d, stack_base=%08x, stack_size=%d, options=%08x)\n", task, params->name, params->routine, params->context, params->priority, params->stack_base, params->stack_size, params->options);

    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }
    
    if (!task)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (!params->routine)
    {
        return K_ERR_INVALID_PARAMETER;
    }

    if ((params->priority < K_PRIORITY_MAX) || (params->priority > K_PRIORITY_MIN))
    {
        return K_ERR_INVALID_PARAMETER;
    }

    if ((uint32_t)params->stack_base & 7)
    {
        return K_ERR_INVALID_PARAMETER;
    }

    if ((params->stack_size & 7) || (params->stack_size < K_STACK_SIZE_MIN))
    {
        return K_ERR_INVALID_PARAMETER;
    }
    
    if (params->options & ~(K_TASK_SUSPENDED | K_TASK_JOINABLE))
    {
        return K_ERR_INVALID_PARAMETER;
    }
    
    armv7m_rtos_task_create(task, params->name, params->priority, params->stack_base, params->stack_size, params->options);

    armv7m_rtos_task_start(task, params->routine, params->context);
    
    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_terminate(k_task_t *task)
{
    // armv7m_rtt_printf("k_task_terminate(task=%08x)\n", task);

    /* No need to check for K_STATE_LOCKED here. Because we do
     * not allow task_self to be terminated, there is not way
     * to switch the current task out and end up with a locked up
     * system.
     */

    if (task == armv7m_rtos_control.task_self)
    {
        return K_ERR_ILLEGAL_USE;
    }
    
    if (task == armv7m_rtos_control.task_default)
    {
        return K_ERR_ILLEGAL_USE;
    }

    if (!task || !task->link || (task->state & K_TASK_STATE_TERMINATED))
    {
        return K_ERR_INVALID_OBJECT;
    }

    task->state |= K_TASK_STATE_TERMINATED;

    if (task->state & K_TASK_STATE_SUSPENDED)
    {
        if (armv7m_atomic_cas((volatile uint32_t*)&task->resume, (uint32_t)NULL, (uint32_t)K_TASK_SENTINEL) != (uint32_t)NULL)
        {
            armv7m_rtos_task_resume_routine();
        }
    }
    
    if (task->state & K_TASK_STATE_WAIT_MASK)
    {
        if ((task->state & K_TASK_STATE_WAIT_MASK) == K_TASK_STATE_WAIT_SLEEP)
        {
            if (armv7m_atomic_cas((volatile uint32_t*)&task->wakeup, (uint32_t)NULL, (uint32_t)K_TASK_SENTINEL) != (uint32_t)NULL)
            {
                armv7m_rtos_task_wakeup_routine();
            }
        }
        else
        {
            armv7m_rtos_task_unblock(task, K_ERR_UNSATISFIED);
        }
    }

    if (task->state & K_TASK_STATE_READY)
    {
        armv7m_rtos_task_ready_remove(task);
    }

    if (task->join)
    {
        armv7m_rtos_join_release(task->join);
    }

    armv7m_rtos_mutex_destroy(task);

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_terminate)(task);
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
    
    if (!(task->state & K_TASK_STATE_JOINABLE))
    {
        armv7m_rtos_task_destroy(task);
    }

    armv7m_rtos_task_timeout_schedule();
    
    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_detach(k_task_t *task)
{
    // armv7m_rtt_printf("k_task_detach(task=%08x)\n", task);

    if (!task || !task->link)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (task == armv7m_rtos_control.task_default)
    {
        return K_ERR_ILLEGAL_USE;
    }

    if (!(task->state & K_TASK_STATE_JOINABLE))
    {
        return K_ERR_TASK_ALREADY_DETACHED;
    }

    if (task->join)
    {
        return K_ERR_TASK_ALREADY_JOINED;
    }

    task->state &= ~K_TASK_STATE_JOINABLE;

    if (task->state & K_TASK_STATE_TERMINATED)
    {
        armv7m_rtos_task_destroy(task);
    }
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_join(k_task_t *task)
{
    k_task_t *self;

    // armv7m_rtt_printf("k_task_join(task=%08x)\n", task);

    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }
    
    if (armv7m_rtos_control.system_state != K_STATE_RUNNING)
    {
        return K_ERR_ILLEGAL_USE;
    }

    if (task == armv7m_rtos_control.task_default)
    {
        return K_ERR_ILLEGAL_USE;
    }
    
    if (!task || !task->link)
    {
        return K_ERR_INVALID_OBJECT;
    }
    
    if (!(task->state & K_TASK_STATE_JOINABLE))
    {
        return K_ERR_TASK_NOT_JOINABLE;
    }
    
    if (task->join)
    {
        return K_ERR_TASK_ALREADY_JOINED;
    }

    if (task->state & K_TASK_STATE_TERMINATED)
    {
        armv7m_rtos_task_destroy(task);
    }
    else
    {
        self = armv7m_rtos_control.task_self;

        task->join = self;
        
        armv7m_rtos_task_ready_remove(self);
        
        self->wait.join.task = task;
        
        self->state |= K_TASK_STATE_WAIT_JOIN;
        
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
        (*armv7m_rtos_control.hook_table->task_block)(self, (self->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
        
        armv7m_rtos_task_schedule();
    }
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_enumerate(uint32_t *p_count_return, k_task_t **p_task_return, uint32_t count)
{
    k_task_t *task;
    uint32_t index;

    // armv7m_rtt_printf("k_task_enumerate(p_count_return=%08x, p_task_return=%08x, count=%d)\n", p_count_return, p_task_return, count);

    for (task = armv7m_rtos_control.task_link, index = 0; task != K_TASK_SENTINEL; task = task->link, index++)
    {
        if (p_task_return)
        {
            if (index < count)
            {
                p_task_return[index] = task;
            }
        }
    }
    
    if (p_count_return)
    {
        *p_count_return = index;
    }

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_info(k_task_t *task, k_task_info_t *p_task_info_return)
{
    // armv7m_rtt_printf("k_task_info(task=%08x, p_task_info_return=%08x)\n", task, p_task_info_return);

    if (!task || !task->link)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (p_task_info_return)
    {
        p_task_info_return->name = task->name;
        p_task_info_return->priority = task->priority;
        p_task_info_return->bpriority = task->bpriority;
        p_task_info_return->state = armv7m_rtos_task_state(task);
        p_task_info_return->wait = (task->state & K_TASK_STATE_WAIT_MASK);
    }
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_stack(k_task_t *task, uint32_t *p_stack_base_return, uint32_t *p_stack_size_return, uint32_t *p_stack_space_return)
{
    uint32_t *stack_end;
    
    // armv7m_rtt_printf("k_task_stack(task=%08x, p_stack_base_return=%08x, p_stack_size_return=%08x, p_stack_space_return=%08x)\n", task, p_stack_base_return, p_stack_size_return, p_stack_space_return);

    if (!task || !task->link || (task->state & K_TASK_STATE_TERMINATED))
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (p_stack_base_return)
    {
        *p_stack_base_return = (uint32_t)task->stack_limit;
    }
    
    if (p_stack_size_return)
    {
        *p_stack_size_return = (uint32_t)task->stack_base - (uint32_t)task->stack_limit;
    }

    if (p_stack_space_return)
    {
        if (task->stack_end > task->stack_top)
        {
            task->stack_end = task->stack_top;
        }
            
        for (stack_end = (uint32_t*)task->stack_limit; stack_end < (uint32_t*)task->stack_end; stack_end += 2)
        {
            if ((stack_end[0] != 0xaaaaaaaa) || (stack_end[1] != 0x77777777))
            {
                break;
            }
        }

        task->stack_end = stack_end;

        *p_stack_space_return = (uint32_t)task->stack_end - (uint32_t)task->stack_limit;
    }
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_unblock(k_task_t *task)
{
    // armv7m_rtt_printf("k_task_unblock(task=%08x)\n", task);

    if (!task || !task->link || (task->state & K_TASK_STATE_TERMINATED))
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (!(task->state & K_TASK_STATE_WAIT_MASK))
    {
        return K_ERR_TASK_NOT_BLOCKED;
    }

    armv7m_rtos_task_unblock(task, K_ERR_UNSATISFIED);

    armv7m_rtos_task_timeout_schedule();
    
    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_suspend(k_task_t *task)
{
    // armv7m_rtt_printf("k_task_suspend(task=%08x)\n", task);

    if (task == armv7m_rtos_control.task_self)
    {
        if (armv7m_rtos_control.system_state != K_STATE_RUNNING)
        {
            return K_ERR_ILLEGAL_USE;
        }
    }

    if (!task || !task->link || (task->state & K_TASK_STATE_TERMINATED))
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (task->state & K_TASK_STATE_SUSPENDED)
    {
        return K_ERR_TASK_ALREADY_SUSPENDED;
    }

    if (task->state & K_TASK_STATE_READY)
    {
        armv7m_rtos_task_ready_remove(task);
    }

    task->state |= K_TASK_STATE_SUSPENDED;

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_block)(task, (task->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
    
    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_resume(k_task_t *task)
{
    k_task_t *task_resume;
    
    // armv7m_rtt_printf("k_task_resume(task=%08x)\n", task);

    if (!task || !task->link || (task->state & K_TASK_STATE_TERMINATED))
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (!(task->state & K_TASK_STATE_SUSPENDED))
    {
        return K_ERR_TASK_NOT_SUSPENDED;
    }

    if (__armv7m_atomic_cas((volatile uint32_t*)&task->resume, (uint32_t)NULL, (uint32_t)K_TASK_SENTINEL) != (uint32_t)NULL)
    {
        return K_ERR_TASK_NOT_SUSPENDED;
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        armv7m_rtos_task_resume_process(task);
    }
    else
    {
        task_resume = (k_task_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_resume, (uint32_t)task);
        
        task->resume = task_resume;

        if (task_resume == K_TASK_SENTINEL)
        {
            armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_TASK_RESUME);
        }
    }

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_set_priority(k_task_t *task, uint32_t priority, uint32_t *p_priority_return)
{
    uint32_t priority_previous;

    // armv7m_rtt_printf("k_task_set_priority(task=%08x, priority=%d, p_priority_return=%08x)\n", task, priority, p_priority_return);

    if (!task || !task->link || (task->state & K_TASK_STATE_TERMINATED))
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (priority > K_PRIORITY_MIN)
    {
        return K_ERR_INVALID_PARAMETER;
    }

    priority_previous = task->bpriority;
    
    if (priority != K_PRIORITY_CURRENT)
    {
        if (task->bpriority != priority)
        {
            task->bpriority = priority;

            armv7m_rtos_task_priority(task);

            armv7m_rtos_task_schedule();
        }
    }
    
    if (p_priority_return)
    {
        *p_priority_return = priority_previous;
    }

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_get_priority(k_task_t *task, uint32_t *p_priority_return)
{
    // armv7m_rtt_printf("k_task_get_priority(task=%08x, p_priority_return=%08x)\n", task, p_priority_return);

    if (!task || !task->link || (task->state & K_TASK_STATE_TERMINATED))
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (p_priority_return)
    {
      *p_priority_return = task->priority;
    }

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_delay(uint32_t delay)
{
    k_task_t *self;

    // armv7m_rtt_printf("k_task_delay(delay=%d)\n", delay);

    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }
    
    if (armv7m_rtos_control.system_state != K_STATE_RUNNING)
    {
        return K_ERR_ILLEGAL_USE;
    }

    if (!delay)
    {
        return K_ERR_INVALID_PARAMETER;
    }

    self = armv7m_rtos_control.task_self;
    
    armv7m_rtos_task_ready_remove(self);
        
    self->state |= K_TASK_STATE_WAIT_DELAY;

    armv7m_rtos_task_timeout_relative(self, delay);
    
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_block)(self, (self->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */

    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_delay_until(uint32_t clock_l, uint32_t clock_h)
{
    k_task_t *self;
    uint64_t clock;

    // armv7m_rtt_printf("k_task_delay_until(clock=%08x%08x)\n", clock_h, clock_l);

    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    if (armv7m_rtos_control.system_state != K_STATE_RUNNING)
    {
        return K_ERR_ILLEGAL_USE;
    }

    if (!clock_l && !clock_h)
    {
        return K_ERR_INVALID_PARAMETER;
    }
    
    clock = (((uint64_t)clock_l << 0) | ((uint64_t)clock_h << 32));

    self = armv7m_rtos_control.task_self;

    armv7m_rtos_task_ready_remove(self);

    self->state |= K_TASK_STATE_WAIT_DELAY;
        
    armv7m_rtos_task_timeout_absolute(self, clock);

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_block)(self, (self->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */

    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_sleep(uint32_t timeout)
{
    k_task_t *self;

    // armv7m_rtt_printf("k_task_sleep(timeout=%d)\n", timeout);

    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    self = armv7m_rtos_control.task_self;

    if (!self)
    {
        return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.system_state != K_STATE_RUNNING)
    {
        if (timeout != K_TIMEOUT_NONE)
        {
            return K_ERR_INVALID_PARAMETER;
        }
    }

    if (self->state & K_TASK_STATE_WAKEUP)
    {
        self->state &= ~K_TASK_STATE_WAKEUP;

        return K_NO_ERROR;
    }

    if (timeout == K_TIMEOUT_NONE)
    {
        return K_ERR_UNSATISFIED;
    }

    armv7m_rtos_task_ready_remove(self);

    self->state |= K_TASK_STATE_WAIT_SLEEP;
    
    if (timeout != K_TIMEOUT_FOREVER)
    {
        armv7m_rtos_task_timeout_relative(self, timeout);
    }

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_block)(self, (self->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */

    armv7m_rtos_task_schedule();

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_wakeup(k_task_t *task)
{
    k_task_t *task_wakeup;

    // armv7m_rtt_printf("k_task_wakeup(task=%08x)\n", task);

    if (!task || !task->link || (task->state & K_TASK_STATE_TERMINATED))
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (__armv7m_atomic_cas((volatile uint32_t*)&task->wakeup, (uint32_t)NULL, (uint32_t)K_TASK_SENTINEL) != (uint32_t)NULL)
    {
        return K_NO_ERROR;
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        armv7m_rtos_task_wakeup_process(task);
    }
    else
    {
        task_wakeup = (k_task_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_wakeup, (uint32_t)task);
        
        task->wakeup = task_wakeup;

        if (task_wakeup == K_TASK_SENTINEL)
        {
            armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_TASK_WAKEUP);
        }
    }

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_yield(void)
{
    k_task_t *self;

    // armv7m_rtt_printf("k_task_yield()\n");
    
    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    if (armv7m_rtos_control.system_state != K_STATE_RUNNING)
    {
        return K_ERR_ILLEGAL_USE;
    }

    self = armv7m_rtos_control.task_self;

    armv7m_rtos_task_ready_remove(self);
    armv7m_rtos_task_ready_insert(self);

    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

/******************************************************************************************************************************/

int k_task_create(k_task_t *task, const char *name, k_task_routine_t routine, void *context, uint32_t priority, void *stack_base, uint32_t stack_size, uint32_t options)
{
    k_task_params_t params;

    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    params.name = name;
    params.routine = routine;
    params.context = context;
    params.priority = priority;
    params.stack_base = stack_base;
    params.stack_size = stack_size;
    params.options = options;

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_task_create, (uint32_t)task, (uint32_t)&params);
}

void k_task_exit(void)
{
    if (!armv7m_core_is_in_thread())
    {
        return;
    }
    
    if (armv7m_rtos_control.work_self)
    {
        return;
    }
    
    armv7m_rtos_task_exit();
}

int k_task_terminate(k_task_t *task)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    if ((task == armv7m_rtos_control.task_self) && !armv7m_rtos_control.work_self)
    {
        armv7m_rtos_task_exit();
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_terminate, (uint32_t)task);
}

int k_task_detach(k_task_t *task)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_detach, (uint32_t)task);
}

int k_task_join(k_task_t *task)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_join, (uint32_t)task);
}

bool k_task_is_joinable(k_task_t *task)
{
    return (task && task->link && (task->state & K_TASK_STATE_JOINABLE));
}

k_task_t * __attribute__((optimize("O3"))) k_task_self(void)
{
    return armv7m_rtos_control.task_self;
}

k_task_t * __attribute__((optimize("O3"))) k_task_default(void)
{
    return armv7m_rtos_control.task_default;
}

bool __attribute__((optimize("O3"))) k_task_is_in_progress(void)
{
    return (armv7m_core_is_in_thread() && !armv7m_rtos_control.work_self);
}

int k_task_enumerate(uint32_t *p_count_return, k_task_t **p_task_return, uint32_t count)
{
    if (armv7m_core_is_in_interrupt())
    {
        if (!armv7m_core_is_in_pendsv())
        {
            return K_ERR_ILLEGAL_CONTEXT;
        }

        return __svc_armv7m_rtos_task_enumerate(p_count_return, p_task_return, count);
    }

    return armv7m_svcall_3((uint32_t)&__svc_armv7m_rtos_task_enumerate, (uint32_t)p_count_return, (uint32_t)p_task_return, (uint32_t)count);
}

int k_task_info(k_task_t *task, k_task_info_t *p_task_info_return)
{
    if (armv7m_core_is_in_interrupt())
    {
        if (!armv7m_core_is_in_pendsv())
        {
            return K_ERR_ILLEGAL_CONTEXT;
        }

        return __svc_armv7m_rtos_task_info(task, p_task_info_return);
    }

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_task_info, (uint32_t)task, (uint32_t)p_task_info_return);
}

int k_task_stack(k_task_t *task, uint32_t *p_stack_base_return, uint32_t *p_stack_size_return, uint32_t *p_stack_space_return)
{
    if (armv7m_core_is_in_interrupt())
    {
        if (!armv7m_core_is_in_pendsv())
        {
            return K_ERR_ILLEGAL_CONTEXT;
        }

        return __svc_armv7m_rtos_task_stack(task, p_stack_base_return, p_stack_size_return, p_stack_space_return);
     }

    return armv7m_svcall_4((uint32_t)&__svc_armv7m_rtos_task_stack, (uint32_t)task, (uint32_t)p_stack_base_return, (uint32_t)p_stack_size_return, (uint32_t)p_stack_space_return);
}

int k_task_unblock(k_task_t *task)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_unblock, (uint32_t)task);
}

int k_task_suspend(k_task_t *task)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_suspend, (uint32_t)task);
}

int k_task_resume(k_task_t *task)
{
    if (armv7m_core_is_in_interrupt())
    {
        return __svc_armv7m_rtos_task_resume(task);
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_resume, (uint32_t)task);
}

bool k_task_is_suspended(k_task_t *task)
{
    return (task && task->link && (task->state & K_TASK_STATE_SUSPENDED) && !task->resume);
}

int k_task_set_priority(k_task_t *task, uint32_t priority, uint32_t *p_priority_return)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_3((uint32_t)&__svc_armv7m_rtos_task_set_priority, (uint32_t)task, (uint32_t)priority, (uint32_t)p_priority_return);
}

int k_task_get_priority(k_task_t *task, uint32_t *p_priority_return)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_task_get_priority, (uint32_t)task, (uint32_t)p_priority_return);
}

int k_task_delay(uint32_t delay)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_delay, (uint32_t)delay);
}

int k_task_delay_until(uint64_t clock)
{
    uint32_t clock_l, clock_h;

    clock_l = (uint32_t)(clock >> 0);
    clock_h = (uint32_t)(clock >> 32);
    
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_task_delay_until, (uint32_t)clock_l, (uint32_t)clock_h);
}

int k_task_sleep(uint32_t timeout)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_sleep, (uint32_t)timeout);
}

int k_task_wakeup(k_task_t *task)
{
    if (armv7m_core_is_in_interrupt())
    {
        return __svc_armv7m_rtos_task_wakeup(task);
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_wakeup, (uint32_t)task);
}

int k_task_yield(void)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_0((uint32_t)&__svc_armv7m_rtos_task_yield);
}

/******************************************************************************************************************************/

void RTOS_TASK_RESUME_SWIHandler(void)
{
    armv7m_rtos_task_resume_routine();
}

void RTOS_TASK_WAKEUP_SWIHandler(void)
{
    armv7m_rtos_task_wakeup_routine();
}
