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

static void armv7m_rtos_sem_release_process(k_sem_t *sem);
static void armv7m_rtos_sem_release_routine(void);

static int __svc_armv7m_rtos_sem_init(k_sem_t *sem, uint32_t count, uint32_t limit);
static int __svc_armv7m_rtos_sem_deinit(k_sem_t *sem);
static int __svc_armv7m_rtos_sem_acquire(k_sem_t *sem, uint32_t timeout);
static int __svc_armv7m_rtos_sem_release(k_sem_t *sem);

/******************************************************************************************************************************/

void armv7m_rtos_sem_unblock(k_task_t *task)
{
    k_sem_t *sem;

    sem = task->wait.sem.sem;
    
    armv7m_rtos_task_queue_remove(&sem->waiting, task);
}

static void armv7m_rtos_sem_release_process(k_sem_t *sem)
{
    k_task_t *task, *task_next, *task_head, *task_tail;
    k_sem_t *sem_release;
    
    do
    {
        sem_release = sem->release;

        sem->release = NULL;
        
        task_head = sem->waiting;

        if (task_head)
        {
            task_tail = task_head->previous;
            task_next = task_head;
            
            do
            {
                task = task_next;
                task_next = task->next;

                if (armv7m_atomic_dech(&sem->count))
                {
                    armv7m_rtos_task_queue_remove(&sem->waiting, task);

                    armv7m_rtos_task_release(task);
                }
                else
                {
                    break;
                }
            }
            while (task != task_tail);
        }
        
        sem = sem_release;
    }
    while (sem != K_SEM_SENTINEL);

    armv7m_rtos_task_timeout_schedule();
    
    armv7m_rtos_task_schedule();
}

static void armv7m_rtos_sem_release_routine(void)
{
    k_sem_t *sem_release, *sem_previous, *sem_next;

    sem_release = (k_sem_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.sem_release, (uint32_t)K_SEM_SENTINEL);

    if (sem_release != K_SEM_SENTINEL)
    {
        if (sem_release->release != K_SEM_SENTINEL)
        {
            for (sem_previous = K_SEM_SENTINEL; sem_release != K_SEM_SENTINEL; sem_release = sem_next)
            {
                sem_next = sem_release->release;
                
                sem_release->release = sem_previous;
                
                sem_previous = sem_release;
            }
            
            sem_release = sem_previous;
        }
        
        armv7m_rtos_sem_release_process(sem_release);
    }
}

/******************************************************************************************************************************/

static int __svc_armv7m_rtos_sem_init(k_sem_t *sem, uint32_t count, uint32_t limit)
{
    // armv7m_rtt_printf("k_sem_init(sem=%08x, count=%d, limit=%d)\n", sem, count, limit);
    
    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }
    
    if (!sem)
    {
        return K_ERR_INVALID_OBJECT;
    }
    
    if ((count > 0xffff) || (limit > 0xffff))
    {
        return K_ERR_INVALID_PARAMETER;
    }
    
    *sem = K_SEM_INIT(count, limit);

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_sem_deinit(k_sem_t *sem)
{
    // armv7m_rtt_printf("k_sem_deinit(sem=%08x)\n", sem);

    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }
    
    if (!sem)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (sem->waiting)
    {
        do
        {
            armv7m_rtos_task_unblock(sem->waiting, K_ERR_UNSATISFIED);
        }
        while (sem->waiting);

        armv7m_rtos_task_timeout_schedule();
        
        armv7m_rtos_task_schedule();
    }

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_sem_acquire(k_sem_t *sem, uint32_t timeout)
{
    k_task_t *self;

    // armv7m_rtt_printf("k_sem_acquire(sem=%08x, timeout=%d)\n", sem, timeout);

    if (!sem)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (armv7m_rtos_control.work_self)
    {
        if (timeout != K_TIMEOUT_NONE)
        {
            return K_ERR_INVALID_PARAMETER;
        }
    }
    else
    {
        if (armv7m_rtos_control.system_state != K_STATE_RUNNING)
        {
            if (timeout != K_TIMEOUT_NONE)
            {
                return K_ERR_INVALID_PARAMETER;
            }
        }
    }
    
    if (armv7m_atomic_dech(&sem->count))
    {
        return K_NO_ERROR;
    }

    if (timeout == K_TIMEOUT_NONE)
    {
        return K_ERR_UNSATISFIED;
    }

    self = armv7m_rtos_control.task_self;

    armv7m_rtos_task_ready_remove(self);

    self->wait.sem.sem = sem;

    armv7m_rtos_task_queue_insert(&sem->waiting, self);

    self->state |= K_TASK_STATE_WAIT_SEM;
    
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

static int __svc_armv7m_rtos_sem_release(k_sem_t *sem)
{
    k_sem_t *sem_release;
  
    // armv7m_rtt_printf("k_sem_release(sem=%08x)\n", sem);
    
    if (!sem)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (armv7m_atomic_inch(&sem->count, sem->limit) == sem->limit)
    {
        return K_ERR_SEM_OVERFLOW;
    }

    if (__armv7m_atomic_cas((volatile uint32_t*)&sem->release, (uint32_t)NULL, (uint32_t)K_SEM_SENTINEL) == (uint32_t)NULL)
    {
        if (armv7m_core_is_in_svcall_or_pendsv())
        {
            armv7m_rtos_sem_release_process(sem);
        }
        else
        {
            sem_release = (k_sem_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.sem_release, (uint32_t)sem);
            
            sem->release = sem_release;
            
            if (sem_release == K_SEM_SENTINEL)
            {
                armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_SEM_RELEASE);
            }
        }
    }
    
    return K_NO_ERROR;
}

/******************************************************************************************************************************/

int k_sem_init(k_sem_t *sem, uint32_t count, uint32_t limit)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_3((uint32_t)&__svc_armv7m_rtos_sem_init, (uint32_t)sem, count, limit);
}

int k_sem_deinit(k_sem_t *sem)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_sem_deinit, (uint32_t)sem);
}

int k_sem_acquire(k_sem_t *sem, uint32_t timeout)
{
    uint32_t count;
    
    if (armv7m_core_is_in_interrupt())
    {
        if (!sem)
        {
            return K_ERR_INVALID_OBJECT;
        }

        if (timeout != K_TIMEOUT_NONE)
        {
            return K_ERR_INVALID_PARAMETER;
        }

        count = armv7m_atomic_dech(&sem->count);

        if (count)
        {
            return K_NO_ERROR;
        }

        return K_ERR_TIMEOUT;
    }

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_sem_acquire, (uint32_t)sem, (uint32_t)timeout);
}

int k_sem_release(k_sem_t *sem)
{
    if (armv7m_core_is_in_interrupt())
    {
        return __svc_armv7m_rtos_sem_release(sem);
    }
    
    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_sem_release, (uint32_t)sem);
}

uint32_t k_sem_count(k_sem_t *sem)
{
    if (!sem)
    {
        return 0;
    }

    return sem->count;
}

/******************************************************************************************************************************/

void RTOS_SEM_RELEASE_SWIHandler(void)
{
    armv7m_rtos_sem_release_routine();
}
