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

static void armv7m_rtos_mutex_queue_insert(k_mutex_t **p_mutex_head, k_mutex_t *mutex);
static void armv7m_rtos_mutex_queue_remove(k_mutex_t **p_mutex_head, k_mutex_t *mutex);
static void armv7m_rtos_mutex_owner_insert(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_owner_remove(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_owner_attach(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_owner_detach(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_wait_insert(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_wait_remove(k_mutex_t *mutex, k_task_t *task);

static void armv7m_rtos_mutex_release(k_task_t *task);

static int __svc_armv7m_rtos_mutex_init(k_mutex_t *mutex, uint32_t priority, uint32_t options);
static int __svc_armv7m_rtos_mutex_deinit(k_mutex_t *mutex);
static int __svc_armv7m_rtos_mutex_set_priority(k_mutex_t *mutex, uint32_t priority, uint32_t *p_priority_return);
static int __svc_armv7m_rtos_mutex_lock(k_mutex_t *mutex, uint32_t timeout);
static int __svc_armv7m_rtos_mutex_unlock(k_mutex_t *mutex);

/******************************************************************************************************************************/

static void __attribute__((optimize("O3"), noinline)) armv7m_rtos_mutex_queue_insert(k_mutex_t **p_mutex_head, k_mutex_t *mutex)
{
    k_mutex_t *element;

    if (*p_mutex_head == NULL)
    {
        *p_mutex_head = mutex;

        mutex->next = mutex;
        mutex->previous = mutex;
    }
    else
    {
        element = *p_mutex_head;

        do
        {
            if (mutex->priority < element->priority)
            {
                if (*p_mutex_head == element)
                {
                    *p_mutex_head = mutex;
                }
                
                break;
            }

            element = element->next;
        }
        while (*p_mutex_head != element);

        mutex->next = element;
        mutex->previous = element->previous;
        
        mutex->previous->next = mutex;
        mutex->next->previous = mutex;
    }
}

static void __attribute__((optimize("O3"), noinline)) armv7m_rtos_mutex_queue_remove(k_mutex_t **p_mutex_head, k_mutex_t *mutex)
{
    if (mutex->next == mutex)
    {
        *p_mutex_head = NULL;
    }
    else
    {
        if (*p_mutex_head == mutex)
        {
            *p_mutex_head = mutex->next;
        }
        
        mutex->next->previous = mutex->previous;
        mutex->previous->next = mutex->next;
    }

    mutex->next = NULL;
    mutex->previous = NULL;
}

static inline void armv7m_rtos_mutex_owner_insert(k_mutex_t *mutex, k_task_t *task)
{
    armv7m_rtos_mutex_queue_insert(&task->mutex, mutex);
}

static inline void armv7m_rtos_mutex_owner_remove(k_mutex_t *mutex, k_task_t *task)
{
    armv7m_rtos_mutex_queue_remove(&task->mutex, mutex);
}

static void armv7m_rtos_mutex_owner_attach(k_mutex_t *mutex, k_task_t *task)
{
    armv7m_rtos_mutex_queue_insert(&task->mutex, mutex);

    mutex->owner = task;
    mutex->level = 1;
}

static void armv7m_rtos_mutex_owner_detach(k_mutex_t *mutex, k_task_t *task)
{
    mutex->level = 0;
    mutex->owner = NULL;

    armv7m_rtos_mutex_queue_remove(&task->mutex, mutex);
}

static inline void armv7m_rtos_mutex_wait_insert(k_mutex_t *mutex, k_task_t *task)
{
    armv7m_rtos_task_queue_insert(&mutex->waiting, task);
}

static inline void armv7m_rtos_mutex_wait_remove(k_mutex_t *mutex, k_task_t *task)
{
    armv7m_rtos_task_queue_remove(&mutex->waiting, task);
}

k_task_t * armv7m_rtos_mutex_priority(k_task_t *task)
{
    k_task_t *owner;
    k_mutex_t *mutex;
  
    owner = NULL;

    if ((task->state & K_TASK_STATE_WAIT_MASK) == K_TASK_STATE_WAIT_MUTEX)
    {
        mutex = task->wait.mutex.mutex;
        
        armv7m_rtos_mutex_wait_remove(mutex, task);
        armv7m_rtos_mutex_wait_insert(mutex, task);
        
        if (mutex->options & K_MUTEX_PRIORITY_INHERIT)
        {
            if (mutex->priority != mutex->waiting->priority)
            {
                mutex->priority = mutex->waiting->priority;
                
                owner = mutex->owner;
                
                armv7m_rtos_mutex_owner_remove(mutex, owner);
                armv7m_rtos_mutex_owner_insert(mutex, owner);
            }
        }
    }

    return owner;
}

void armv7m_rtos_mutex_unblock(k_task_t *task)
{
    k_task_t *owner;
    k_mutex_t *mutex;
    uint32_t priority;

    mutex = task->wait.mutex.mutex;
        
    armv7m_rtos_mutex_wait_remove(mutex, task);

    if (mutex->options & K_MUTEX_PRIORITY_INHERIT)
    {
        priority = (mutex->waiting ? mutex->waiting->priority : K_PRIORITY_MIN);
        
        if (mutex->priority != priority)
        {
            mutex->priority = priority;
            
            owner = mutex->owner;

            armv7m_rtos_mutex_owner_remove(mutex, owner);
            armv7m_rtos_mutex_owner_insert(mutex, owner);
            
            armv7m_rtos_task_priority(owner);
        }
    }
}

static void armv7m_rtos_mutex_release(k_task_t *task)
{
    k_mutex_t *mutex;

    mutex = task->wait.mutex.mutex;
                
    armv7m_rtos_mutex_wait_remove(mutex, task);

    if (mutex->options & K_MUTEX_PRIORITY_INHERIT)
    {
        mutex->priority = (mutex->waiting ? mutex->waiting->priority : K_PRIORITY_MIN);
    }
    
    armv7m_rtos_mutex_owner_attach(mutex, task);

    if (mutex->options & (K_MUTEX_PRIORITY_INHERIT | K_MUTEX_PRIORITY_PROTECT))
    {
        armv7m_rtos_task_priority(task);
    }

    armv7m_rtos_task_release(task);
}

void armv7m_rtos_mutex_destroy(k_task_t *task)
{
    k_mutex_t *mutex;

    mutex = task->mutex;

    if (mutex)
    {
        do
        {
            armv7m_rtos_mutex_owner_detach(mutex, task);

            if (mutex->waiting)
            {
                armv7m_rtos_mutex_release(mutex->waiting);
            }
            
            mutex = task->mutex;
        }
        while (mutex);
    }
}

/******************************************************************************************************************************/

static int __svc_armv7m_rtos_mutex_init(k_mutex_t *mutex, uint32_t priority, uint32_t options)
{
    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    if (!mutex)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (options & ~(K_MUTEX_RECURSIVE | K_MUTEX_PRIORITY_INHERIT | K_MUTEX_PRIORITY_PROTECT))
    {
        return K_ERR_INVALID_PARAMETER;
    }

    if (options & K_MUTEX_PRIORITY_PROTECT)
    {
        if ((priority < K_PRIORITY_MAX) || (priority > K_PRIORITY_MIN))
        {
            return K_ERR_INVALID_PARAMETER;
        }
    }
    else
    {
        priority = K_PRIORITY_MIN;
    }

    *mutex = K_MUTEX_INIT(priority, options);

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_mutex_deinit(k_mutex_t *mutex)
{
    k_task_t *owner;

    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    if (!mutex)
    {
        return K_ERR_INVALID_OBJECT;
    }

    owner = mutex->owner;

    if (owner)
    {
        armv7m_rtos_mutex_owner_detach(mutex, owner);

        if (mutex->options & K_MUTEX_PRIORITY_INHERIT)
        {
            armv7m_rtos_task_priority(owner);
        }
    }

    if (mutex->waiting)
    {
        do
        {
            armv7m_rtos_task_unblock(mutex->waiting, K_ERR_UNSATISFIED);
        }
        while (mutex->waiting);
    }

    armv7m_rtos_task_timeout_schedule();
    
    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_mutex_set_priority(k_mutex_t *mutex, uint32_t priority, uint32_t *p_priority_return)
{
    uint32_t priority_previous;

    // armv7m_rtt_printf("k_mutex_set_priority(mutex=%08x, priority=%d, p_priority_return=%08x)\n", mutex, priority, p_priority_return);

    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    if (!mutex || !(mutex->options & K_MUTEX_PRIORITY_PROTECT))
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (priority > K_PRIORITY_MIN)
    {
        return K_ERR_INVALID_PARAMETER;
    }

    if (mutex->owner)
    {
        return K_ERR_MUTEX_ALREADY_LOCKED;
    }

    priority_previous = mutex->priority;
    
    if (priority != K_PRIORITY_CURRENT)
    {
        mutex->priority = priority;
    }
    
    if (p_priority_return)
    {
        *p_priority_return = priority_previous;
    }

    return K_NO_ERROR;
}

static int  __attribute__((optimize("O3"))) __svc_armv7m_rtos_mutex_lock(k_mutex_t *mutex, uint32_t timeout)
{
    k_task_t *self, *owner;
    
    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    self = armv7m_rtos_control.task_self;
    
    if (!self)
    {
        return K_ERR_ILLEGAL_USE;
    }

    if (!mutex)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (armv7m_rtos_control.system_state != K_STATE_RUNNING)
    {
        if (timeout != K_TIMEOUT_NONE)
        {
            return K_ERR_INVALID_PARAMETER;
        }
    }

    if (mutex->owner == self)
    {
        if (mutex->options & K_MUTEX_RECURSIVE)
        {
            if (mutex->level == 0xffff)
            {
                return K_ERR_MUTEX_OVERFLOW;
            }
            
            mutex->level++;
            
            return K_NO_ERROR;
        }
        else
        {
            return K_ERR_MUTEX_ALREADY_LOCKED;
        }
    }

    if (!mutex->owner)
    {
        if (mutex->options & K_MUTEX_PRIORITY_INHERIT)
        {
            mutex->priority = self->priority;
        }

        armv7m_rtos_mutex_owner_attach(mutex, self);
        
        if (mutex->options & K_MUTEX_PRIORITY_PROTECT)
        {
            armv7m_rtos_task_priority(self);
        }
        
        return K_NO_ERROR;
    }

    if (timeout == K_TIMEOUT_NONE)
    {
        return K_ERR_UNSATISFIED;
    }
    
    armv7m_rtos_task_ready_remove(self);

    self->wait.mutex.mutex = mutex;

    self->state |= K_TASK_STATE_WAIT_MUTEX;

    if (timeout != K_TIMEOUT_FOREVER)
    {
        armv7m_rtos_task_timeout_relative(self, timeout);
    }

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_block)(self, (self->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */

    armv7m_rtos_mutex_wait_insert(mutex, self);

    if (mutex->options & K_MUTEX_PRIORITY_INHERIT)
    {
        if (mutex->priority != mutex->waiting->priority)
        {
            mutex->priority = mutex->waiting->priority;
            
            owner = mutex->owner;

            armv7m_rtos_mutex_owner_remove(mutex, owner);
            armv7m_rtos_mutex_owner_insert(mutex, owner);

            armv7m_rtos_task_priority(owner);
        }
    }
    
    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_mutex_unlock(k_mutex_t *mutex)
{
    k_task_t *self;
    
    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    self = armv7m_rtos_control.task_self;

    if (!self)
    {
        return K_ERR_ILLEGAL_USE;
    }

    if (!mutex)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (mutex->owner != self)
    {
        return K_ERR_NOT_OWNER_OF_MUTEX;
    }

    if (mutex->options & K_MUTEX_RECURSIVE)
    {
        if (mutex->level != 1)
        {
            mutex->level--;
            
            return K_NO_ERROR;
        }
    }

    armv7m_rtos_mutex_owner_detach(mutex, self);
    
    if (mutex->waiting)
    {
        armv7m_rtos_mutex_release(mutex->waiting);
    }

    if (mutex->options & (K_MUTEX_PRIORITY_INHERIT | K_MUTEX_PRIORITY_PROTECT))
    {
        armv7m_rtos_task_priority(self);
    }

    armv7m_rtos_task_timeout_schedule();

    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

/******************************************************************************************************************************/

int k_mutex_init(k_mutex_t *mutex, uint32_t priority, uint32_t options)
{
    // armv7m_rtt_printf("k_mutex_init(mutex=%08x, priority=%d, options=%08x)\n", mutex, priority, options);
    
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_3((uint32_t)&__svc_armv7m_rtos_mutex_init, (uint32_t)mutex, (uint32_t)priority, (uint32_t)options);
}

int k_mutex_deinit(k_mutex_t *mutex)
{
    // armv7m_rtt_printf("k_mutex_deinit(mutex=%08x)\n", mutex);

    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_mutex_deinit, (uint32_t)mutex);
}

int k_mutex_set_priority(k_mutex_t *mutex, uint32_t priority, uint32_t *p_priority_return)
{
    // armv7m_rtt_printf("k_mutex_set_priority(mutex=%08x, priority=%d, p_priority_return=%08x)\n", mutex, priority, p_priority_return);

    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_3((uint32_t)&__svc_armv7m_rtos_mutex_set_priority, (uint32_t)mutex, (uint32_t)priority, (uint32_t)p_priority_return);
}

int k_mutex_lock(k_mutex_t *mutex, uint32_t timeout)
{
    // armv7m_rtt_printf("k_mutex_lock(mutex=%08x, timeout=%d)\n", mutex, timeout);

    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_mutex_lock, (uint32_t)mutex, (uint32_t)timeout);
}

int k_mutex_unlock(k_mutex_t *mutex)
{
    // armv7m_rtt_printf("k_mutex_unlock(mutex=%08x)\n", mutex);

    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_mutex_unlock, (uint32_t)mutex);
}

uint32_t k_mutex_level(k_mutex_t *mutex)
{
    if (!mutex)
    {
        return 0;
    }

    return mutex->level;
}

k_task_t* k_mutex_owner(k_mutex_t *mutex)
{
    if (!mutex)
    {
        return NULL;
    }

    return mutex->owner;
}

/******************************************************************************************************************************/
