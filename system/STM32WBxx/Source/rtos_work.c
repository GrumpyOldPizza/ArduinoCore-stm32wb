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

static void armv7m_rtos_work_routine(void);

static int __svc_armv7m_rtos_work_init(k_work_t *work, k_work_routine_t routine, void *context);
static int __svc_armv7m_rtos_work_deinit(k_work_t *work);
static int __svc_armv7m_rtos_work_submit(k_work_t *work);

/******************************************************************************************************************************/

int armv7m_rtos_work_deinit(k_work_t *work)
{
    k_work_t * volatile *work_previous;
    k_work_t *work_submit;

    armv7m_atomic_store_2_restart((volatile uint32_t*)&work->routine, 0, 0);

    if (work->next)
    {
        if (__armv7m_atomic_cas((volatile uint32_t*)&armv7m_rtos_control.work_submit, (uint32_t)work, (uint32_t)work->next) != (uint32_t)work)
        {
            for (work_previous = &armv7m_rtos_control.work_submit, work_submit = *work_previous; work_submit != K_WORK_SENTINEL; work_previous = &work_submit->next, work_submit = *work_previous)
            {
                if (work_submit == work)
                {
                    *work_previous = work->next;

                    work->next = NULL;

                    break;
                }
            }
        }
    }

    return K_NO_ERROR;
}

int  __attribute__((optimize("O3"))) armv7m_rtos_work_submit(k_work_t *work)
{
    k_work_t *work_submit;
  
    if (__armv7m_atomic_cas((volatile uint32_t*)&work->next, (uint32_t)NULL, (uint32_t)K_WORK_SENTINEL) != (uint32_t)NULL)
    {
        return K_ERR_WORK_ALREADY_SUBMITTED;
    }

    work_submit = (k_work_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.work_submit, (uint32_t)work);

    work->next = work_submit;
    
    if (work_submit == K_WORK_SENTINEL)
    {
        armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_WORK);
    }

    return K_NO_ERROR;
}

void __attribute__((optimize("O3"), used)) armv7m_rtos_work_return(void)
{
    k_work_t *work;
    
    armv7m_rtos_control.work_self = NULL;
    
    work = armv7m_rtos_control.work_head;
    
    if (work != K_WORK_SENTINEL)
    {
        if (work == armv7m_rtos_control.work_tail)
        {
            armv7m_rtos_control.work_head = K_WORK_SENTINEL;
            armv7m_rtos_control.work_tail = K_WORK_SENTINEL;
        }
        else
        {
            armv7m_rtos_control.work_head = work->next;
        }
        
        armv7m_rtos_control.work_self = work;
        
        work->next = NULL;
    }
    else
    {
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
        if (armv7m_rtos_control.task_self == armv7m_rtos_control.task_next)
        {
            (*armv7m_rtos_control.hook_table->task_run)(armv7m_rtos_control.task_self);
        }
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
    }
}

void __attribute__((optimize("O3"))) armv7m_rtos_work_schedule(void)
{
    k_work_t *work;

    if (armv7m_rtos_control.system_state == K_STATE_RUNNING)
    {
        work = armv7m_rtos_control.work_head;

        if (work != K_WORK_SENTINEL)
        {
            if (!armv7m_rtos_control.work_self)
            {
                if (work == armv7m_rtos_control.work_tail)
                {
                    armv7m_rtos_control.work_head = K_WORK_SENTINEL;
                    armv7m_rtos_control.work_tail = K_WORK_SENTINEL;
                }
                else
                {
                    armv7m_rtos_control.work_head = work->next;
                }
                
                armv7m_rtos_control.work_self = work;
            
                work->next = NULL;
                
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
                (*armv7m_rtos_control.hook_table->task_run)(K_TASK_WORK);
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */

                armv7m_pendsv_hook(armv7m_rtos_pendsv_epilogue);
            }
        }
    }
}

static void __attribute__((optimize("O3"))) armv7m_rtos_work_routine(void)
{
    k_work_t *work_submit, *work_next, *work_head, *work_tail;
     
    if (armv7m_rtos_control.work_submit != K_WORK_SENTINEL)
    {
        work_submit = (k_work_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.work_submit, (uint32_t)K_WORK_SENTINEL);

        for (work_head = K_WORK_SENTINEL, work_tail = work_submit; work_submit != K_WORK_SENTINEL; work_submit = work_next)
        {
            work_next = work_submit->next;

            work_submit->next = work_head;

            work_head = work_submit;
        }
        
        if (armv7m_rtos_control.work_head == K_WORK_SENTINEL)
        {
            armv7m_rtos_control.work_head = work_head;
        }
        else
        {
            armv7m_rtos_control.work_tail->next = work_head;
        }
        
        armv7m_rtos_control.work_tail = work_tail;

        armv7m_rtos_work_schedule();
    }
}

/******************************************************************************************************************************/

static int __svc_armv7m_rtos_work_init(k_work_t *work, k_work_routine_t routine, void *context)
{
    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    if (!work)
    {
        return K_ERR_INVALID_OBJECT;
    }

    *work = K_WORK_INIT(routine, context);

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_work_deinit(k_work_t *work)
{
    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    if (!work || !work->routine)
    {
        return K_ERR_INVALID_OBJECT;
    }

    return armv7m_rtos_work_deinit(work);
}

static int __svc_armv7m_rtos_work_submit(k_work_t *work)
{
    if (!work || !work->routine)
    {
        return K_ERR_INVALID_OBJECT;
    }

    return armv7m_rtos_work_submit(work);
}

/******************************************************************************************************************************/

int k_work_init(k_work_t *work, k_work_routine_t routine, void *context)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_3((uint32_t)&__svc_armv7m_rtos_work_init, (uint32_t)work, (uint32_t)routine, (uint32_t)context);
}

int k_work_deinit(k_work_t *work)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_work_deinit, (uint32_t)work);
}

int __attribute__((optimize("O3"))) k_work_submit(k_work_t *work)
{
    if (armv7m_core_is_in_interrupt())
    {
        return __svc_armv7m_rtos_work_submit(work);
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_work_submit, (uint32_t)work);
}

k_work_t * __attribute__((optimize("O3"))) k_work_self(void)
{
    return armv7m_rtos_control.work_self;

}

bool __attribute__((optimize("O3"))) k_work_is_in_progress(void)
{
    return (armv7m_core_is_in_thread() && armv7m_rtos_control.work_self);
}

/******************************************************************************************************************************/

void RTOS_WORK_SWIHandler(void)
{
    armv7m_rtos_work_routine();
}
