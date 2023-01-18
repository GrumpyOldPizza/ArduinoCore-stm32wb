/*
 * Copyright (c) 2021-2022 Thomas Roell.  All rights reserved.
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
#include "../../CMSIS/RTOS2/Include/cmsis_os2.h"

#include <malloc.h>

extern void __malloc_lock(struct _reent *);
extern void __malloc_unlock(struct _reent *);

#define OS_VERSION_API       20010003         // API version (2.1.3)
#define OS_VERSION_KERNEL    10000000         // Kernel version (1.0.0)
#define OS_KERNEL_NAME       "Orchid V1.0.0"  // Kernel identification string

#define OS_THREAD_MAGIC      0xff544844
#define OS_TIMER_MAGIC       0xff544d52
#define OS_MUTEX_MAGIC       0xff4e5458
#define OS_SEMAPHORE_MAGIC   0xff53454d

typedef struct _osThread_t {
    uint32_t                      magic;
    const char                    *name;
    k_task_t                      task;
    osThreadFunc_t                func;
    void                          *argument;
    void                          *cb_mem;
    void                          *stack_mem;
} osThread_t;

typedef struct _osTimer_t {
    uint32_t                      magic;
    const char                    *name;
    k_alarm_t                     alarm;
    osTimerType_t                 type;
    void                          *cb_mem;
} osTimer_t;
  
typedef struct osMutex_t {
    uint32_t                      magic;
    const char                    *name;
    k_mutex_t                     mutex;
    void                          *cb_mem;
} osMutex_t;
  
typedef struct _osSemaphore_t {
    uint32_t                      magic;
    const char                    *name;
    k_sem_t                       sem;
    void                          *cb_mem;
} osSemaphore_t;

static const osThreadAttr_t osThreadAttrDefault =
{
    .name = NULL,
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
    .stack_mem = NULL,
    .stack_size = 0,
    .priority = 0,
    .tz_module = 0,
    .reserved = 0
};

static const osTimerAttr_t osTimerAttrDefault =
{
    .name = NULL,
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0
};

static const osMutexAttr_t osMutexAttrDefault =
{
    .name = NULL,
    .attr_bits = 0,
    .cb_mem = 0,
    .cb_size = 0
};

static const osSemaphoreAttr_t osSemaphoreAttrDefault =
{
    .name = NULL,
    .attr_bits = 0,
    .cb_mem = 0,
    .cb_size = 0
};

static osKernelState_t osXlateKernelState[] =
{
    osKernelInactive, // K_STATE_INACTIVE
    osKernelReady,    // K_STATE_READY
    osKernelRunning,  // K_STATE_RUNNING
    osKernelLocked,   // K_STATE_LOCKED
};
    
static osStatus_t osXlateKernelStatus[] =
{
    osOK,             // K_NO_ERROR
    osErrorISR,       // K_ERR_ILLEGAL_CONTEXT
    osError,          // K_ERR_ILLEGAL_USE
    osErrorParameter, // K_ERR_INVALID_OBJECT
    osErrorParameter, // K_ERR_INVALID_PARAMETER
    osErrorTimeout,   // K_ERR_TIMEOUT
    osErrorResource,  // K_ERR_UNSATISFIED
    osErrorResource,  // K_ERR_TASK_NOT_BLOCKED
    osErrorResource,  // K_ERR_TASK_ALREADY_DETACHED
    osErrorResource,  // K_ERR_TASK_ALREADY_JOINED
    osErrorResource,  // K_ERR_TASK_NOT_JOINABLE
    osErrorResource,  // K_ERR_TASK_ALREADY_SUSPENDED
    osErrorResource,  // K_ERR_TASK_NOT_SUSPENDED
    osErrorResource,  // K_ERR_SEM_OVERFLOW
    osErrorResource,  // K_ERR_NOT_OWNER_OF_MUTEX
    osErrorResource,  // K_ERR_MUTEX_ALREADY_LOCKED
    osErrorResource,  // K_ERR_MUTEX_NOT_LOCKED
    osErrorResource,  // K_ERR_MUTEX_OVERFLOW
    osErrorResource,  // K_ERR_WORK_ALREADY_SUBMITTED
    osErrorResource,  // K_ERR_ALARM_NOT_ACTIVE
};

static uint32_t osXlateKernelFlags[] =
{
    0,                     // K_NO_ERROR
    osFlagsErrorISR,       // K_ERR_ILLEGAL_CONTEXT
    osFlagsErrorUnknown,   // K_ERR_ILLEGAL_USE
    osFlagsErrorParameter, // K_ERR_INVALID_OBJECT
    osFlagsErrorParameter, // K_ERR_INVALID_PARAMETER
    osFlagsErrorTimeout,   // K_ERR_TIMEOUT
    osFlagsErrorResource,  // K_ERR_UNSATISFIED
    osFlagsErrorResource,  // K_ERR_TASK_NOT_BLOCKED
    osFlagsErrorResource,  // K_ERR_TASK_ALREADY_DETACHED
    osFlagsErrorResource,  // K_ERR_TASK_ALREADY_JOINED
    osFlagsErrorResource,  // K_ERR_TASK_NOT_JOINABLE
    osFlagsErrorResource,  // K_ERR_TASK_ALREADY_SUSPENDED
    osFlagsErrorResource,  // K_ERR_TASK_NOT_SUSPENDED
    osFlagsErrorResource,  // K_ERR_SEM_OVERFLOW
    osFlagsErrorResource,  // K_ERR_NOT_OWNER_OF_MUTEX
    osFlagsErrorResource,  // K_ERR_MUTEX_ALREADY_LOCKED
    osFlagsErrorResource,  // K_ERR_MUTEX_NOT_LOCKED
    osFlagsErrorResource,  // K_ERR_MUTEX_OVERFLOW
    osFlagsErrorResource,  // K_ERR_WORK_ALREADY_SUBMITTED
    osFlagsErrorResource,  // K_ERR_ALARM_NOT_ACTIVE
};

static uint32_t osXlateAPIPriority(osPriority_t priority)
{
    return (K_PRIORITY_MIN -1) - (priority - osPriorityLow);
}

static osPriority_t osXlateKernelPriority(uint32_t priority)
{
    if (priority <= ((K_PRIORITY_MIN -1) - (osPriorityISR - osPriorityLow)))
    {
        return osPriorityError;
    }

    return osPriorityLow + ((K_PRIORITY_MIN -1) - priority);
}
    
osStatus_t osKernelGetInfo(osVersion_t *version, char *name, uint32_t size)
{
    if (version)
    {
        version->api = OS_VERSION_API;
        version->kernel = OS_VERSION_KERNEL;
    }

    if (name && size)
    {
        if (size > sizeof(OS_KERNEL_NAME))
        {
            size = sizeof(OS_KERNEL_NAME);
        }

        memcpy(name, OS_KERNEL_NAME, size);
    }

    return osOK;
}

osKernelState_t osKernelGetState(void)
{
    uint32_t state;

    state = k_system_state();
    
    return (osKernelState_t)osXlateKernelState[state];
}

osStatus_t osKernelInitialize(void)
{
    int status;

    // status = k_system_initialize(&armv7m_rtt_hook_table);
    status = k_system_initialize(NULL);

    return osXlateKernelStatus[status];
}

osStatus_t osKernelStart(void)
{
    int status;

    // status = k_system_start(osKernelRoutine, NULL);
    status = k_system_start(NULL, NULL);

    return osXlateKernelStatus[status];
}
 
int32_t osKernelLock(void)
{
    if (armv7m_core_is_in_interrupt())
    {
        return osErrorISR;
    }

    if (!k_task_is_in_progress())
    {
        return osError;
    }
    
    return k_system_lock();
}

int32_t osKernelUnlock(void)
{
    int32_t lock;
    
    if (armv7m_core_is_in_interrupt())
    {
        return osErrorISR;
    }

    if (!k_task_is_in_progress())
    {
        return osError;
    }

    lock = k_system_is_locked();
    
    k_system_unlock();

    return lock;
}

int32_t osKernelRestoreLock(int32_t lock)
{
    if (armv7m_core_is_in_interrupt())
    {
        return osErrorISR;
    }

    if (!k_task_is_in_progress())
    {
        return osError;
    }

    if (lock)
    {
        k_system_lock();
    }
    else
    {
        k_system_unlock();
    }

    return k_system_is_locked();
}

uint32_t osKernelGetTickCount(void)
{
    return (uint32_t)k_system_clock();
}
 
uint32_t osKernelGetTickFreq(void)
{
    return 1000;
}
              
uint32_t osKernelGetSysTimerCount(void)
{
    return (uint32_t)armv7m_systick_clock();
}
 
uint32_t osKernelGetSysTimerFreq(void)
{
    return SystemCoreClock;
}

static void __attribute__((noreturn)) osThreadRoutine(osThread_t *thread)
{
    (thread->func)(thread->argument);

    osThreadExit();
}

static void osThreadDelete(osThread_t *thread, bool lock)
{
    thread->magic = ~OS_THREAD_MAGIC;

    if (thread->cb_mem || thread->stack_mem)
    {
        /* This below is a tad convoluted. "lock" is true when subsequently
         * k_task_exit() will be called. Hence we take take the malloc lock
         * (which is a recursive lock), and then free the allocations.
         * k_task_exit() later on releases the mutexed, include the malloc lock,
         * which means the allocations are not touched (but freed).
         *
         * N.b. nano-malloc will nuke the first 4 bytes of the allocation, which
         * is why we did put magic/name there.
         */

        if (lock)
        {
            __enable_irq();
                
            k_system_unlock();
                
            __malloc_lock(NULL);
        }
        
        if (thread->stack_mem)
        {
            free(thread->stack_mem);
        }
        
        if (thread->cb_mem)
        {
            free(thread->cb_mem);
        }
    }
}

osThreadId_t osThreadNew(osThreadFunc_t func, void *argument, const osThreadAttr_t *attr)
{
    osThread_t *thread;
    void *stack;
    const char *name;
    uint32_t attr_bits, stack_size;
    osPriority_t priority;
    
    if (armv7m_core_is_in_interrupt())
    {
        return (osThreadId_t)NULL;
    }

    if (!func)
    {
        return (osThreadId_t)NULL;
    }
            
    if (!attr)
    {
        attr = &osThreadAttrDefault;
    }

    name = attr->name;

    attr_bits = attr->attr_bits;
    
    thread = (osThread_t*)attr->cb_mem;

    if (thread)
    {
        if (((uint32_t)thread & 3) || (attr->cb_size < sizeof(osThread_t)))
        {
            return (osThreadId_t)NULL;
        }
    }
    
    stack = (void*)attr->stack_mem;
    stack_size = attr->stack_size;
    
    priority = attr->priority;
    
    if (priority == osPriorityNone)
    {
        priority = osPriorityNormal;
    }
    else
    {
        if ((priority < osPriorityIdle) || (priority > osPriorityISR))
        {
            return (osThreadId_t)NULL;
        }
    }

    if (!thread)
    {
        thread = (osThread_t*)malloc(sizeof(osThread_t));

        if (!thread)
        {
            return (osThreadId_t)NULL;
        }
    }

    if (!stack)
    {
        if (stack_size == 0)
        {
            stack_size = K_STACK_SIZE_DEFAULT;
        }
        
        stack = (void*)memalign(8, stack_size);

        if (!stack)
        {
            if (!attr->cb_mem)
            {
                free(thread);
            }
            
            return (osThreadId_t)NULL;
        }
    }


    thread->magic = OS_THREAD_MAGIC;
    thread->name = name;
    thread->func = func;
    thread->argument = argument;
    thread->cb_mem = attr->cb_mem ? NULL : thread;
    thread->stack_mem = attr->stack_mem ? NULL : stack;
    
    if (k_task_create(&thread->task, name, (k_task_routine_t)osThreadRoutine, (void*)thread, osXlateAPIPriority(priority), stack, stack_size, ((attr_bits & osThreadJoinable) ? K_TASK_JOINABLE : 0)) != K_NO_ERROR)
    {
        osThreadDelete(thread, false);

        return (osThreadId_t)NULL;
    }
    
    return (osThreadId_t)thread;
}

const char *osThreadGetName(osThreadId_t thread_id)
{
    osThread_t *thread = (osThread_t*)thread_id;
    
    if (armv7m_core_is_in_interrupt())
    {
        return NULL;
    }
    
    if (!thread || (thread->magic != OS_THREAD_MAGIC))
    {
        return NULL;
    }

    return thread->name;
}
 
osThreadId_t osThreadGetId(void)
{
    k_task_t *task = k_task_self();
    
    if (((osThread_t*)(((uint32_t)task) - offsetof(osThread_t, task)))->magic == OS_THREAD_MAGIC)
    {
        return (osThread_t*)(((uint32_t)task) - offsetof(osThread_t, task));
    }
    
    return (osThreadId_t)NULL;
}

osThreadState_t osThreadGetState(osThreadId_t thread_id)
{
    osThread_t *thread = (osThread_t*)thread_id;
    osThreadState_t state;
    k_task_info_t info;
    
    if (armv7m_core_is_in_interrupt())
    {
        return osThreadError;
    }
    
    if (!thread || (thread->magic != OS_THREAD_MAGIC))
    {
        return osThreadError;
    }

    if (k_task_info(&thread->task, &info) != K_NO_ERROR)
    {
        return osThreadError;
    }
    
    switch (info.state) {
    case K_STATE_INACTIVE:
        state = osThreadInactive;
        break;
    case K_STATE_RUNNING:
        state = osThreadRunning;
        break;
    case K_STATE_READY:
        state = osThreadReady;
        break;
    case K_STATE_BLOCKED:
    case K_STATE_SUSPENDED:
        state = osThreadBlocked;
        break;
    case K_STATE_TERMINATED:
        state = osThreadTerminated;
        break;
    default:
        state = osThreadError;
        break;
    }
    
    return state;
}
 
uint32_t osThreadGetStackSize(osThreadId_t thread_id)
{
    osThread_t *thread = (osThread_t*)thread_id;
    uint32_t stack_size;
    
    if (!thread || (thread->magic != OS_THREAD_MAGIC))
    {
        return 0;
    }

    if (k_task_stack(&thread->task, NULL, &stack_size, NULL) != K_NO_ERROR)
    {
        return 0;
    }

    return stack_size;
}
 
uint32_t osThreadGetStackSpace(osThreadId_t thread_id)
{
    osThread_t *thread = (osThread_t*)thread_id;
    uint32_t stack_space;
    
    if (!thread || (thread->magic != OS_THREAD_MAGIC))
    {
        return 0;
    }

    if (k_task_stack(&thread->task, NULL, NULL, &stack_space) != K_NO_ERROR)
    {
        return 0;
    }

    return stack_space;
}
 
osStatus_t osThreadSetPriority(osThreadId_t thread_id, osPriority_t priority)
{
    osThread_t *thread = (osThread_t*)thread_id;
    int status;
    
    if (!thread || (thread->magic != OS_THREAD_MAGIC))
    {
        return osErrorParameter;
    }

    if ((priority < osPriorityIdle) || (priority > osPriorityISR))
    {
        return osErrorParameter;
    }

    status = k_task_set_priority(&thread->task, osXlateAPIPriority(priority), NULL);

    return osXlateKernelStatus[status];
}
 
osPriority_t osThreadGetPriority(osThreadId_t thread_id)
{
    osThread_t *thread = (osThread_t*)thread_id;
    uint32_t priority;
    
    if (!thread || (thread->magic != OS_THREAD_MAGIC))
    {
        return osPriorityError;
    }

    if (k_task_get_priority(&thread->task, &priority) == K_NO_ERROR)
    {
        return osXlateKernelPriority(priority);
    }
    else
    {
        return osPriorityError;
    }
}
 
osStatus_t osThreadYield(void)
{
    int status;
    
    status = k_task_yield();

    return osXlateKernelStatus[status];
}
 
osStatus_t osThreadSuspend(osThreadId_t thread_id)
{
    osThread_t *thread = (osThread_t*)thread_id;
    int status;
    
    if (!thread || (thread->magic != OS_THREAD_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_task_suspend(&thread->task);

    return osXlateKernelStatus[status];
}
 
osStatus_t osThreadResume(osThreadId_t thread_id)
{
    osThread_t *thread = (osThread_t*)thread_id;
    int status;

    if (armv7m_core_is_in_interrupt())
    {
        return osErrorISR;
    }
    
    if (!thread || (thread->magic != OS_THREAD_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_task_resume(&thread->task);

    return osXlateKernelStatus[status];
}

osStatus_t osThreadDetach(osThreadId_t thread_id)
{
    osThread_t *thread = (osThread_t*)thread_id;
    int status;

    if (!thread || (thread->magic != OS_THREAD_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_task_detach(&thread->task);

    if (status == K_NO_ERROR)
    {
        if (k_task_info(&thread->task, NULL) == K_ERR_INVALID_OBJECT)
        {
            osThreadDelete(thread, false);
        }
    }
    
    return osXlateKernelStatus[status];
}
 
osStatus_t osThreadJoin(osThreadId_t thread_id)
{
    osThread_t *thread = (osThread_t*)thread_id;
    int status;
        
    if (!thread || (thread->magic != OS_THREAD_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_task_join(&thread->task);

    if (status == K_NO_ERROR)
    {
        osThreadDelete(thread, false);
    }
    
    return osXlateKernelStatus[status];
}
 
__NO_RETURN void osThreadExit(void)
{
    osThread_t *thread;
    k_task_t *task;

    if (k_task_is_in_progress())
    {
        task = k_task_self();

        thread = (osThread_t*)(((uint32_t)task) - offsetof(osThread_t, task));

        if (thread->magic == OS_THREAD_MAGIC)
        {
            if (!k_task_is_joinable(&thread->task))
            {
                osThreadDelete(thread, true);
            }
        }
        
        k_task_exit();
    }
    
    while (1)
    {
    }
}
 
osStatus_t osThreadTerminate(osThreadId_t thread_id)
{
    osThread_t *thread = (osThread_t*)thread_id;
    int status;
    
    if (!thread || (thread->magic != OS_THREAD_MAGIC))
    {
        return osErrorParameter;
    }

    if (k_task_is_in_progress() && (&thread->task == k_task_self()))
    {
        if (!k_task_is_joinable(&thread->task))
        {
            osThreadDelete(thread, true);
        }

        k_task_exit();

        while (1)
        {
        }
    }
    
    status = k_task_terminate(&thread->task);

    if (status == K_NO_ERROR)
    {
        if (!k_task_is_joinable(&thread->task))
        {
            osThreadDelete(thread, false);
        }
    }
    
    return osXlateKernelStatus[status];
}
 
uint32_t osThreadGetCount(void)
{
    uint32_t count;
    
    if (k_task_enumerate(&count, NULL, 0) != K_NO_ERROR)
    {
        return 0;
    }

    return count;
}

uint32_t osThreadEnumerate(osThreadId_t *thread_array, uint32_t array_items)
{
    uint32_t i, n, count;
    
    if (k_task_enumerate(&n, (k_task_t**)thread_array, array_items) != K_NO_ERROR)
    {
        return 0;
    }

    for (i = 0, count = 0; i < n; i++)
    {
        if (((osThread_t*)(((uint32_t)thread_array[i]) - offsetof(osThread_t, task)))->magic == OS_THREAD_MAGIC)
        {
            thread_array[count++] = (osThread_t*)(((uint32_t)thread_array[i]) - offsetof(osThread_t, task));
        }
    }
    
    return count;
}


uint32_t osThreadFlagsSet(osThreadId_t thread_id, uint32_t flags)
{
    osThread_t *thread = (osThread_t*)thread_id;
    int status;
    
    if (!thread || (thread->magic != OS_THREAD_MAGIC))
    {
        return osFlagsErrorParameter;
    }

    if (flags & 0x80000000)
    {
        return osFlagsErrorParameter;
    }

    status = k_event_send(&thread->task, flags);

    if (status != K_NO_ERROR)
    {
        return osXlateKernelFlags[status];
    }

    return thread->task.events;
}
 
uint32_t osThreadFlagsClear(uint32_t flags)
{
    k_task_t *self;
    uint32_t flags_return;
    int status;
    
    if (flags & 0x80000000)
    {
        return osFlagsErrorParameter;
    }

    status = k_event_receive(flags, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_NONE, &flags_return);

    if (status != K_NO_ERROR)
    {
        return osXlateKernelFlags[status];
    }

    self = k_task_self();

    return self->events | flags_return;
}

uint32_t osThreadFlagsGet(void)
{
    k_task_info_t info;
    int status;
    
    status = k_task_info(k_task_self(), &info);

    if (status != K_NO_ERROR)
    {
        return osXlateKernelFlags[status];
    }
    
    return info.events;
}
 
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout)
{
    k_task_info_t info;
    uint32_t flags_return;
    int status;
    
    if (flags & 0x80000000)
    {
        return osFlagsErrorParameter;
    }

    if (options & ~(osFlagsWaitAll | osFlagsNoClear))
    {
        return osFlagsErrorParameter;
    }

    status = k_event_receive(flags, (((options & osFlagsWaitAll) ? K_EVENT_ALL : K_EVENT_ANY) | ((options & osFlagsNoClear) ? 0 : K_EVENT_CLEAR)), timeout, &flags_return);

    if (status != K_NO_ERROR)
    {
        return osXlateKernelFlags[status];
    }

    status = k_task_info(k_task_self(), &info);

    if (status != K_NO_ERROR)
    {
        return osXlateKernelFlags[status];
    }
    
    return info.events | flags_return;
}    

osStatus_t osDelay(uint32_t ticks)
{
    int status;
    
    status = k_task_delay(ticks);

    return osXlateKernelStatus[status];
}

osStatus_t osDelayUntil(uint32_t ticks)
{
    uint64_t clock;
    uint32_t reference;
    int status;
    
    clock = k_system_clock();

    reference = clock;
    
    status = k_task_delay_until(clock + ((uint32_t)(ticks - reference)));

    return osXlateKernelStatus[status];
}

osTimerId_t osTimerNew(osTimerFunc_t func, osTimerType_t type, void *argument, const osTimerAttr_t *attr)
{
    osTimer_t *timer;
    const char *name;
    
    if (armv7m_core_is_in_interrupt())
    {
        return (osTimerId_t)NULL;
    }
    
    if (!func)
    {
        return (osTimerId_t)NULL;
    }

    if ((type != osTimerOnce) && (type != osTimerPeriodic))
    {
        return (osTimerId_t)NULL;
    }

    if (!attr)
    {
        attr = &osTimerAttrDefault;
    }

    name = attr->name;

    timer = (osTimer_t*)attr->cb_mem;

    if (timer)
    {
        if (((uint32_t)timer & 3) || (attr->cb_size < sizeof(osTimer_t)))
        {
            return (osTimerId_t)NULL;
        }
    }
    
    if (!timer)
    {
        timer = (osTimer_t*)malloc(sizeof(osTimer_t));

        if (!timer)
        {
            return (osTimerId_t)NULL;
        }
    }

    timer->magic = OS_TIMER_MAGIC;
    timer->name = name;
    timer->type = type;
    timer->cb_mem = attr->cb_mem ? NULL : timer;
    
    if (k_alarm_init(&timer->alarm, (k_alarm_routine_t)func, (void*)argument) != K_NO_ERROR)
    {
        timer->magic = ~OS_TIMER_MAGIC;

        if (!attr->cb_mem)
        {
            free(timer);
        }

        return (osTimerId_t)NULL;
    }

    return (osTimerId_t)timer;
}
 
const char *osTimerGetName(osTimerId_t timer_id)
{
    osTimer_t *timer = (osTimer_t*)timer_id;
    
    if (armv7m_core_is_in_interrupt())
    {
        return NULL;
    }
    
    if (!timer || (timer->magic != OS_TIMER_MAGIC))
    {
        return NULL;
    }

    return timer->name;
}

osStatus_t osTimerStart(osTimerId_t timer_id, uint32_t ticks)
{
    osTimer_t *timer = (osTimer_t*)timer_id;
    int status;

    if (armv7m_core_is_in_interrupt())
    {
        return osErrorISR;
    }
    
    if (!timer || (timer->magic != OS_TIMER_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_alarm_relative(&timer->alarm, ticks, ((timer->type == osTimerPeriodic) ? ticks : 0));

    return osXlateKernelStatus[status];
}

osStatus_t osTimerStop(osTimerId_t timer_id)
{
    osTimer_t *timer = (osTimer_t*)timer_id;
    int status;

    if (armv7m_core_is_in_interrupt())
    {
        return osErrorISR;
    }
    
    if (!timer || (timer->magic != OS_TIMER_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_alarm_cancel(&timer->alarm);

    return osXlateKernelStatus[status];
}
 
uint32_t osTimerIsRunning(osTimerId_t timer_id)
{
    osTimer_t *timer = (osTimer_t*)timer_id;

    if (armv7m_core_is_in_interrupt())
    {
        return 0;
    }

    if (!timer || (timer->magic != OS_TIMER_MAGIC))
    {
        return 0;
    }

    return k_alarm_is_active(&timer->alarm);
}
 
osStatus_t osTimerDelete(osTimerId_t timer_id)
{
    osTimer_t *timer = (osTimer_t*)timer_id;
    int status;

    if (!timer || (timer->magic != OS_TIMER_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_alarm_deinit(&timer->alarm);

    if (status == K_NO_ERROR)
    {
        timer->magic = ~OS_TIMER_MAGIC;
    
        if (timer->cb_mem)
        {
            free(timer->cb_mem);
        }
    }
    
    return osXlateKernelStatus[status];
}

 
osMutexId_t osMutexNew(const osMutexAttr_t *attr)
{
    osMutex_t *mutex;
    const char *name;
    uint32_t attr_bits;
    
    if (armv7m_core_is_in_interrupt())
    {
        return (osMutexId_t)NULL;
    }
    
    if (!attr)
    {
        attr = &osMutexAttrDefault;
    }

    name = attr->name;

    attr_bits = attr->attr_bits;
    
    mutex = (osMutex_t*)attr->cb_mem;

    if (mutex)
    {
        if (((uint32_t)mutex & 3) || (attr->cb_size < sizeof(osMutex_t)))
        {
            return (osMutexId_t)NULL;
        }
    }
    
    if (!mutex)
    {
        mutex = (osMutex_t*)malloc(sizeof(osMutex_t));

        if (!mutex)
        {
            return (osMutexId_t)NULL;
        }
    }

    mutex->magic = OS_MUTEX_MAGIC;
    mutex->name = name;
    mutex->cb_mem = attr->cb_mem ? NULL : mutex;
    
    if (k_mutex_init(&mutex->mutex, K_PRIORITY_MIN, (((attr_bits & osMutexRecursive) ? K_MUTEX_RECURSIVE : 0) | ((attr_bits & osMutexPrioInherit) ? K_MUTEX_PRIORITY_INHERIT : 0))) != K_NO_ERROR)
    {
        mutex->magic = ~OS_MUTEX_MAGIC;

        if (!attr->cb_mem)
        {
            free(mutex);
        }

        return (osMutexId_t)NULL;
    }

    return (osMutexId_t)mutex;
}
 
const char *osMutexGetName(osMutexId_t mutex_id)
{
    osMutex_t *mutex = (osMutex_t*)mutex_id;

    if (armv7m_core_is_in_interrupt())
    {
        return NULL;
    }
    
    if (!mutex || (mutex->magic != OS_MUTEX_MAGIC))
    {
        return NULL;
    }

    return mutex->name;
}
 
osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout)
{
    osMutex_t *mutex = (osMutex_t*)mutex_id;
    int status;
    
    if (!mutex || (mutex->magic != OS_MUTEX_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_mutex_lock(&mutex->mutex, timeout);

    return osXlateKernelStatus[status];
}
 
osStatus_t osMutexRelease(osMutexId_t mutex_id)
{
    osMutex_t *mutex = (osMutex_t*)mutex_id;
    int status;

    if (!mutex || (mutex->magic != OS_MUTEX_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_mutex_unlock(&mutex->mutex);

    return osXlateKernelStatus[status];
}
 
osThreadId_t osMutexGetOwner(osMutexId_t mutex_id)
{
    osMutex_t *mutex = (osMutex_t*)mutex_id;
    k_task_t *task;
    
    if (armv7m_core_is_in_interrupt())
    {
        return NULL;
    }
    
    if (!mutex || (mutex->magic != OS_MUTEX_MAGIC))
    {
        return NULL;
    }

    task = k_mutex_owner(&mutex->mutex);

    if (task && (((osThread_t*)(((uint32_t)task) - offsetof(osThread_t, task)))->magic == OS_THREAD_MAGIC))
    {
        return (osThread_t*)(((uint32_t)task) - offsetof(osThread_t, task));
    }
    else
    {
        return NULL;
    }
}
 
osStatus_t osMutexDelete(osMutexId_t mutex_id)
{
    osMutex_t *mutex = (osMutex_t*)mutex_id;
    int status;
    
    if (!mutex || (mutex->magic != OS_MUTEX_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_mutex_deinit(&mutex->mutex);

    if (status == K_NO_ERROR)
    {
        mutex->magic = ~OS_MUTEX_MAGIC;
    
        if (mutex->cb_mem)
        {
            free(mutex->cb_mem);
        }
    }
    
    return osXlateKernelStatus[status];
}    


osSemaphoreId_t osSemaphoreNew(uint32_t max_count, uint32_t initial_count, const osSemaphoreAttr_t *attr)
{
    osSemaphore_t *semaphore;
    const char *name;
    
    if (armv7m_core_is_in_interrupt())
    {
        return (osSemaphoreId_t)NULL;
    }

    if ((max_count > 0xffff) || (initial_count > 0xffff))
    {
        return (osSemaphoreId_t)NULL;
    }
    
    if (!attr)
    {
        attr = &osSemaphoreAttrDefault;
    }

    name = attr->name;

    semaphore = (osSemaphore_t*)attr->cb_mem;

    if (semaphore)
    {
        if (((uint32_t)semaphore & 3) || (attr->cb_size < sizeof(osSemaphore_t)))
        {
            return (osSemaphoreId_t)NULL;
        }
    }
    else
    {
        if (attr->cb_size != 0)
        {
            return (osSemaphoreId_t)NULL;
        }
    }

    if (!semaphore)
    {
        semaphore = (osSemaphore_t*)malloc(sizeof(osSemaphore_t));

        if (!semaphore)
        {
            return (osSemaphoreId_t)NULL;
        }
    }

    semaphore->magic = OS_SEMAPHORE_MAGIC;
    semaphore->name = name;
    semaphore->cb_mem = attr->cb_mem ? NULL : semaphore;
    
    if (k_sem_init(&semaphore->sem, initial_count, max_count) != K_NO_ERROR)
    {
        semaphore->magic = ~OS_SEMAPHORE_MAGIC;

        if (!attr->cb_mem)
        {
            free(semaphore);
        }

        return (osSemaphoreId_t)NULL;
    }

    return (osSemaphoreId_t)semaphore;
}
 
const char *osSemaphoreGetName(osSemaphoreId_t semaphore_id)
{
    osSemaphore_t *semaphore = (osSemaphore_t*)semaphore_id;

    if (armv7m_core_is_in_interrupt())
    {
        return NULL;
    }
    
    if (!semaphore || (semaphore->magic != OS_SEMAPHORE_MAGIC))
    {
        return NULL;
    }

    return semaphore->name;
}
 
osStatus_t osSemaphoreAcquire(osSemaphoreId_t semaphore_id, uint32_t timeout)
{
    osSemaphore_t *semaphore = (osSemaphore_t*)semaphore_id;
    int status;
    
    if (!semaphore || (semaphore->magic != OS_SEMAPHORE_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_sem_acquire(&semaphore->sem, timeout);

    return osXlateKernelStatus[status];
}
 
osStatus_t osSemaphoreRelease(osSemaphoreId_t semaphore_id)
{
    osSemaphore_t *semaphore = (osSemaphore_t*)semaphore_id;
    int status;
    
    if (!semaphore || (semaphore->magic != OS_SEMAPHORE_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_sem_release(&semaphore->sem);

    return osXlateKernelStatus[status];
}

uint32_t osSemaphoreGetCount(osSemaphoreId_t semaphore_id)
{
    osSemaphore_t *semaphore = (osSemaphore_t*)semaphore_id;

    if (!semaphore || (semaphore->magic != OS_SEMAPHORE_MAGIC))
    {
        return 0;
    }

    return k_sem_count(&semaphore->sem);
}
 
osStatus_t osSemaphoreDelete(osSemaphoreId_t semaphore_id)
{
    osSemaphore_t *semaphore = (osSemaphore_t*)semaphore_id;
    int status;
    
    if (!semaphore || (semaphore->magic != OS_SEMAPHORE_MAGIC))
    {
        return osErrorParameter;
    }

    status = k_sem_deinit(&semaphore->sem);

    if (status == K_NO_ERROR)
    {
        semaphore->magic = ~OS_SEMAPHORE_MAGIC;
    
        if (semaphore->cb_mem)
        {
            free(semaphore->cb_mem);
        }
    }

    return osXlateKernelStatus[status];
}

