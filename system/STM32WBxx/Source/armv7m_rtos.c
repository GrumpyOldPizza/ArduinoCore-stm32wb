/*
 * Copyright (c) 2019-2021 Thomas Roell.  All rights reserved.
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
#include "stm32wb_lptim.h"

#define ARMV7M_RTOS_DEBUG_SUPPORTED 1

extern uint32_t __StackTop[];
extern uint32_t __StackLimit[];

typedef struct _armv7m_rtos_control_t {
    k_work_routine_t        work_routine;
    void                    *work_context;
    k_work_t *              work_head;
    k_work_t *              work_tail;
    k_work_t * volatile     work_submit;
    k_task_t                *task_self;
    k_task_t                *task_next;
    k_task_t                *task_ready;
    k_task_t                *task_timeout;
    k_task_t * volatile     task_created;
    k_task_t * volatile     task_suspend;
    k_task_t * volatile     task_resume;
    k_task_t * volatile     task_release;
    k_task_t                task_default;
    stm32wb_lptim_timeout_t lptim_timeout;
    uint32_t                lptim_ticks;
} armv7m_rtos_control_t;

static armv7m_rtos_control_t armv7m_rtos_control;

  
#define K_TASK_MODE_NOPREEMPT        K_MODE_NOPREEMPT
#define K_TASK_MODE_NOCONCURRENT     K_MODE_NOCONCURRENT
#define K_TASK_MODE_NOTERMINATE      K_MODE_NOTERMINATE
#define K_TASK_MODE_NODISPATCH       0x80

#define K_TASK_STATE_READY           K_STATE_READY
#define K_TASK_STATE_BLOCKED         K_STATE_BLOCKED
#define K_TASK_STATE_SUSPENDED       K_STATE_SUSPENDED
#define K_TASK_STATE_TIMEOUT         0x0010
#define K_TASK_STATE_TERMINATED      0x0020
#define K_TASK_STATE_WAIT_MASK       0xff00
#define K_TASK_STATE_WAIT_DELAY      0x0100
#define K_TASK_STATE_WAIT_EVENT_ALL  0x0200
#define K_TASK_STATE_WAIT_EVENT_ANY  0x0400
#define K_TASK_STATE_WAIT_SEM        0x0800
#define K_TASK_STATE_WAIT_MUTEX      0x1000

#define K_TASK_SENTINEL              ((k_task_t*)0xffffffff)
#define K_WORK_SENTINEL              ((k_work_t*)0xffffffff)

static void armv7m_rtos_task_ready_insert(k_task_t *task);
static void armv7m_rtos_task_ready_remove(k_task_t *task);
static void armv7m_rtos_task_schedule(void);
static bool armv7m_rtos_task_unblock(k_task_t *task, uint32_t status);
static bool armv7m_rtos_task_release(k_task_t *task, uint32_t status);
static bool armv7m_rtos_task_suspend(k_task_t *task);
static bool armv7m_rtos_task_resume(k_task_t *task);
static void armv7m_rtos_task_priority(k_task_t *task);

static void armv7m_rtos_task_timeout_insert(k_task_t *task, uint32_t ticks);
static void armv7m_rtos_task_timeout_remove(k_task_t *task);
static void armv7m_rtos_task_timeout_start(void);
static void armv7m_rtos_task_timeout_restart(void);
static void armv7m_rtos_task_timeout_callback(stm32wb_lptim_timeout_t *timeout);

static void armv7m_rtos_sem_wait_insert(k_sem_t *sem, k_task_t *task);
static void armv7m_rtos_sem_wait_remove(k_sem_t *sem, k_task_t *task);
static k_task_t* armv7m_rtos_sem_wait_release(k_sem_t *sem);
static void armv7m_rtos_sem_wait_acquire(k_sem_t *sem);

static void armv7m_rtos_mutex_owner_attach(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_owner_detach(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_wait_insert(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_wait_remove(k_mutex_t *mutex, k_task_t *task);

static void armv7m_rtos_work_dispatch(void);
static void armv7m_rtos_work_schedule(void);

static void __svc_armv7m_rtos_task_schedule(void);
static int __svc_armv7m_rtos_task_destroy(k_task_t *task);
static int __svc_armv7m_rtos_task_info(k_task_t *task, k_task_info_t *p_task_info_return, uint32_t a2, uint32_t a3, void *exc_stack, uint32_t exc_return);
static int __svc_armv7m_rtos_task_start(k_task_t *task, k_task_routine_t routine, void *context, uint32_t argument);
static int __svc_armv7m_rtos_task_restart(k_task_t *task, uint32_t argument);
static int __svc_armv7m_rtos_task_suspend(k_task_t *task);
static int __svc_armv7m_rtos_task_resume(k_task_t *task);
static int __svc_armv7m_rtos_task_set_priority(k_task_t *task, uint32_t priority, uint32_t *p_priority_return);
static int __svc_armv7m_rtos_task_delay(uint32_t delay, uint32_t a1, uint32_t a2, uint32_t a3, void *exc_stack, uint32_t exc_return);

static int __svc_armv7m_rtos_event_send(k_task_t *task, uint32_t events);
static int __svc_armv7m_rtos_event_receive(uint32_t events, uint32_t mode, uint32_t timeout, uint32_t *p_events_return, void *exc_stack, uint32_t exc_return);

static int __svc_armv7m_rtos_sem_destroy(k_sem_t *sem);
static int __svc_armv7m_rtos_sem_acquire(k_sem_t *sem, uint32_t timeout, uint32_t a2, uint32_t a3, void *exc_stack, uint32_t exc_return);
static int __svc_armv7m_rtos_sem_release(k_sem_t *sem);

static int __svc_armv7m_rtos_mutex_destroy(k_mutex_t *mutex);
static int __svc_armv7m_rtos_mutex_trylock(k_mutex_t *mutex);
static int __svc_armv7m_rtos_mutex_lock(k_mutex_t *mutex, uint32_t a1, uint32_t a2, uint32_t a3, void *exc_stack, uint32_t exc_return);
static int __svc_armv7m_rtos_mutex_unlock(k_mutex_t *mutex);

static void __svc_armv7m_rtos_work_schedule(void);
static int __svc_armv7m_rtos_work_destroy(k_work_t *work);
static bool __svc_armv7m_rtos_work_submit(k_work_t *work);

static void __attribute__((naked, used)) armv7m_rtos_pendsv_epilogue(void)
{
    __asm__(
	"    .cfi_def_cfa_offset 8                             \n"
        "    .cfi_offset 2, -8                                 \n"
        "    .cfi_offset 14, -4                                \n"
	"1:  movw     r3, #:lower16:armv7m_rtos_control        \n"
	"    movt     r3, #:upper16:armv7m_rtos_control        \n"
        "    ldr      r0, [r3, %[offset_CONTROL_WORK_ROUTINE]] \n"
        "    cbz.n    r0, 4f                                   \n"
        "    ldr      r1, [r3, %[offset_CONTROL_WORK_CONTEXT]] \n"
        "    bic      r0, r0, #1                               \n"
	"    movw     r2, #:lower16:2f +1                      \n"
	"    movt     r2, #:upper16:2f +1                      \n"
        "    mov      r3, #0xf1000000                          \n"
        "    mov      lr, #0xfffffff9                          \n"
        "    sub      sp, #0x20                                \n"
        "    str      r1, [sp, #0x00]                          \n"
        "    str      r2, [sp, #0x14]                          \n"
        "    str      r0, [sp, #0x18]                          \n"
        "    str      r3, [sp, #0x1c]                          \n"
        "    dsb                                               \n"
        "    bx       lr                                       \n"
        "    .align 2                                          \n"
        "2:  movw     r12, #:lower16:3f +1                     \n"
	"    movt     r12, #:upper16:3f +1                     \n"
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
	"    mrs      r0, CONTROL                              \n"
	"    bic      r0, #0x04                                \n" // clear FPCA (LSPACT is zero)
	"    msr      CONTROL, r0                              \n"
#endif /* __VFP_FP__ && !__SOFTFP__ */
        "    svc      0                                        \n"
        "    .align 2                                          \n"
	"3:  add      sp, #0x28                                \n"
	"    bl       armv7m_rtos_work_dispatch                \n"
        "    b.n      1b                                       \n"
	"4:  pop      { r2, lr }                               \n"
	"    ldr      r0, [r3, %[offset_CONTROL_TASK_SELF]]    \n"
	"    ldr      r1, [r3, %[offset_CONTROL_TASK_NEXT]]    \n"
        "    cbz.n    r0, 7f                                   \n" // task_self == NULL means either DESTROY or WAIT to task_next
	"    cmp      r0, r1                                   \n"
	"    bne.n    5f                                       \n"
	"    dsb                                               \n"
	"    bx       lr                                       \n"
        "    .align 2                                          \n"
        "5:  tst      lr, #0x00000004                          \n" // bit 2 is SPSEL
        "    itte     eq                                       \n"
	"    moveq    r2, sp                                   \n"
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
	"    subeq    sp, sp, #0x60                            \n"
#else  /* __VFP_FP__ && !__SOFTFP__ */
	"    subeq    sp, sp, #0x20                            \n"
#endif /* __VFP_FP__ && !__SOFTFP__ */
        "    mrsne    r2, PSP                                  \n"
	"    stmdb    r2!, { r4-r11 }                          \n"
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
        "    tst      lr, #0x00000010                          \n" // bit 4 is nFPCA
        "    it       eq                                       \n"
        "    vstmdbeq r2!, { d8-d15 }                          \n"
#endif /* __VFP_FP__ && !__SOFTFP__ */
        "    tst      lr, #0x00000004                          \n" // bit 2 is SPSEL
        "    it       eq                                       \n"
        "    moveq    sp, r2                                   \n"
	"    strb     lr, [r0, %[offset_TASK_EXC_RETURN]]      \n"
	"    str      r2, [r0, %[offset_TASK_STACK_CURRENT]]   \n"
	"6:  str      r1, [r3, %[offset_CONTROL_TASK_SELF]]    \n"
        "    cbz.n    r1, 9f                                   \n"
        "    clrex                                             \n" // make sure atomics get reset via CLREX on a task switch
	"    ldrsb    lr, [r1, %[offset_TASK_EXC_RETURN]]      \n"
	"    ldr      r2, [r1, %[offset_TASK_STACK_CURRENT]]   \n"
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
        "    tst      lr, #0x00000010                          \n" // bit 4 is nFPCA
        "    it       eq                                       \n"
	"    vldmiaeq r2!, { d8-d15 }                          \n"
#endif /* __VFP_FP__ && !__SOFTFP__ */
        "    ldmia    r2!, { r4-r11 }                          \n"
        "    tst      lr, #0x00000004                          \n" // bit 2 is SPSEL
        "    ite      eq                                       \n"
        "    moveq    sp, r2                                   \n"
        "    msrne    PSP, r2                                  \n"
        "    dsb                                               \n"
        "    bx       lr                                       \n"
        "    .align 2                                          \n"
	"7:  tst      lr, #0x00000004                          \n" // bit 2 is SPSEL
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
        "    beq.n    8f                                       \n"
	"    mrs      r0, CONTROL                              \n"
	"    bic      r0, #0x04                                \n" // clear FPCA (LSPACT is zero)
	"    msr      CONTROL, r0                              \n"
        "    b.n      6b                                       \n" // if SPSEL was set, this is a DESTORY, hence nothing to save, just restore
#else  /* __VFP_FP__ && !__SOFTFP__ */
        "    bne.n    6b                                       \n" // if SPSEL was set, this is a DESTORY, hence nothing to save, just restore
#endif /* __VFP_FP__ && !__SOFTFP__ */
        "8:  cbz.n    r1, 10f                                  \n" // if task_next is also NULL, this is a WAIT to WAIT
        "    add      sp, #0x20                                \n"
        "    b.n      6b                                       \n"
	"9:  movw     r0, #:lower16:armv7m_rtos_task_wait      \n"
	"    movt     r0, #:upper16:armv7m_rtos_task_wait      \n"
        "    mov      r3, #0x01000000                          \n"
        "    mov      lr, #0xfffffff9                          \n"
        "    sub      sp, #0x20                                \n"
        "    str      r0, [sp, #0x18]                          \n"
        "    str      r3, [sp, #0x1c]                          \n"
        "10: dsb                                               \n"
        "    bx       lr                                       \n"
	:
	: [offset_CONTROL_WORK_ROUTINE] "I" (offsetof(armv7m_rtos_control_t, work_routine)),
          [offset_CONTROL_WORK_CONTEXT] "I" (offsetof(armv7m_rtos_control_t, work_context)),
          [offset_CONTROL_TASK_SELF]    "I" (offsetof(armv7m_rtos_control_t, task_self)),
          [offset_CONTROL_TASK_NEXT]    "I" (offsetof(armv7m_rtos_control_t, task_next)),
          [offset_WORK_ROUTINE]         "I" (offsetof(k_work_t, routine)),
          [offset_WORK_CONTEXT]         "I" (offsetof(k_work_t, context)),
          [offset_TASK_EXC_RETURN]      "I" (offsetof(k_task_t, exc_return)),
	  [offset_TASK_STACK_CURRENT]   "I" (offsetof(k_task_t, stack_current))
        );
}

static void __attribute__((naked, used)) armv7m_rtos_task_wait(void)
{
    while (1)
    {
	__WFE();
    }
}

static void __attribute__((naked, used)) armv7m_rtos_task_exit(void)
{
    armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_destroy, (uint32_t)armv7m_rtos_control.task_self);
}

static void armv7m_rtos_task_start(k_task_t *task, uint32_t argument)
{
    armv7m_context_t *stack_current;
    
    stack_current = (armv7m_context_t*)(task->stack_top - sizeof(armv7m_context_t));
    stack_current->r0 = (uint32_t)task->context;
    stack_current->r1 = argument;
#if (ARMV7M_RTOS_DEBUG_SUPPORTED == 1)
    stack_current->r2 = 0x00000000;
    stack_current->r3 = 0x00000000;
    stack_current->r4 = 0x00000000;
    stack_current->r5 = 0x00000000;
    stack_current->r6 = 0x00000000;
    stack_current->r7 = 0x00000000;
    stack_current->r8 = 0x00000000;
    stack_current->r9 = 0x00000000;
    stack_current->r10 = 0x00000000;
    stack_current->r11 = 0x00000000;
#endif /* (ARMV7M_RTOS_DEBUG_SUPPORTED == 1) */
    stack_current->lr = (uint32_t)armv7m_rtos_task_exit;
    stack_current->pc = (uint32_t)task->routine & ~1;
    stack_current->xpsr = 0x01000000;
    
    task->exc_return = (int8_t)0xfffffffd;
    task->stack_current = (void*)stack_current;
}

static void armv7m_rtos_task_yield(k_task_t *task, void *exc_stack, uint32_t exc_return)
{
    task->exc_return = (int8_t)exc_return;

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    if (!(exc_return & 0x00000010))
    {
	task->stack_current = (void*)((uint32_t)exc_stack - 0x60);
    }
    else
#endif /* __VFP_FP__ && !__SOFTFP__ */
    {
	task->stack_current = (void*)((uint32_t)exc_stack - 0x20);
    }
}

static void armv7m_rtos_task_return(k_task_t *task, uint32_t status)
{
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    if (!(task->exc_return & 0x00000010))
    {
	((armv7m_context_fpu_t*)task->stack_current)->r0 = status;
    }
    else
#endif /* __VFP_FP__ && !__SOFTFP__ */
    {
	((armv7m_context_t*)task->stack_current)->r0 = status;
    }
}

/******************************************************************************************************************************/

typedef struct _k_node_t {
    struct _k_node_t    *next;
    struct _k_node_t    *previous;
    uint8_t             priority;
} k_node_t;

static void armv7m_rtos_queue_insert(k_node_t **p_node_head, k_node_t *node)
{
    k_node_t *element;

    if (*p_node_head == NULL)
    {
        *p_node_head = node;
    }
    else
    {
        element = *p_node_head;

        do
        {
            if (node->priority < element->priority)
            {
                if (*p_node_head == element)
                {
                    *p_node_head = node;
                }
                
                break;
            }

            element = element->next;
        }
        while (*p_node_head != element);

        node->previous = element->previous;
        node->next = element;
        
        node->previous->next = node;
        node->next->previous = node;
    }
}

static void armv7m_rtos_queue_remove(k_node_t **p_node_head, k_node_t *node)
{
    if (node->next == node)
    {
        *p_node_head = NULL;
    }
    else
    {
        if (*p_node_head == node)
        {
            *p_node_head = node->next;
        }
        
        node->next->previous = node->previous;
        node->previous->next = node->next;

	node->next = node;
	node->previous = node;
    }
}

static inline void armv7m_rtos_task_ready_insert(k_task_t *task)
{
    armv7m_rtos_queue_insert((k_node_t**)&armv7m_rtos_control.task_ready, (k_node_t*)task);

    task->state |= K_TASK_STATE_READY;
}

static inline void armv7m_rtos_task_ready_remove(k_task_t *task)
{
    task->state &= ~K_TASK_STATE_READY;

    armv7m_rtos_queue_remove((k_node_t**)&armv7m_rtos_control.task_ready, (k_node_t*)task);
}

static void armv7m_rtos_task_schedule(void)
{
    k_task_t *task, *task_resume, *task_suspend, *task_release, *task_previous, *task_next, *task_timeout;
    k_sem_t *sem;
    k_mutex_t *mutex;
    
    task_timeout = armv7m_rtos_control.task_timeout;

    if (armv7m_rtos_control.task_suspend != K_TASK_SENTINEL)
    {
	task_suspend = (k_task_t*)armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_suspend, (uint32_t)K_TASK_SENTINEL);

	do
	{
	    task = task_suspend;
	    task_suspend = task->suspend;
	    
	    task->state |= K_TASK_STATE_SUSPENDED;
	    
	    armv7m_atomic_store((volatile uint32_t*)&task->suspend, (uint32_t)NULL);

	    armv7m_rtos_task_ready_remove(task);
	}
	while (task_suspend != K_TASK_SENTINEL);
    }
    
    if (armv7m_rtos_control.task_resume != K_TASK_SENTINEL)
    {
	task_resume = (k_task_t*)armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_resume, (uint32_t)K_TASK_SENTINEL);

	if (task_resume->resume != K_TASK_SENTINEL)
	{
	    for (task_previous = K_TASK_SENTINEL; task_resume != K_TASK_SENTINEL; task_resume = task_next)
	    {
		task_next = task_resume->resume;
		
		armv7m_atomic_store((volatile uint32_t*)&task_resume->resume, (uint32_t)task_previous);
		
		task_previous = task_resume;
	    }
	    
	    task_resume = task_previous;
	}
	
	do
	{
	    task = task_resume;
	    task_resume = task->resume;
	    
	    task->state &= ~K_TASK_STATE_SUSPENDED;
	    
	    armv7m_atomic_store((volatile uint32_t*)&task->resume, (uint32_t)NULL);

	    if (!(task->state & (K_TASK_STATE_TERMINATED | K_TASK_STATE_BLOCKED)))
	    {
		armv7m_rtos_task_ready_insert(task);
	    }
	}
	while (task_resume != K_TASK_SENTINEL);
    }
    
    if (armv7m_rtos_control.task_release != K_TASK_SENTINEL)
    {
	task_release = (k_task_t*)armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_release, (uint32_t)K_TASK_SENTINEL);

	if (task_release->release != K_TASK_SENTINEL)
	{
	    for (task_previous = K_TASK_SENTINEL; task_release != K_TASK_SENTINEL; task_release = task_next)
	    {
		task_next = task_release->release;
		
		armv7m_atomic_store((volatile uint32_t*)&task_release->release, (uint32_t)task_previous);
		
		task_previous = task_release;
	    }
	    
	    task_release = task_previous;
	}
	
	do
	{
	    task = task_release;
	    task_release = task->release;

	    if (task->state & K_TASK_STATE_TIMEOUT)
	    {
		armv7m_rtos_task_timeout_remove(task);
	    }
	    
	    if (task->state & K_TASK_STATE_WAIT_SEM)
	    {
		/* This is tricky here. The code to remove the
		 * head of a semphore wait list is called out of
		 * order. So here we just need to remove one task from
		 * the waiting list for evry task task_release had
		 * be called.
		 */

		sem = task->wait.sem.sem;
		
		armv7m_rtos_sem_wait_acquire(sem);
	    }

	    if (task->state & K_TASK_STATE_WAIT_MUTEX)
	    {
		mutex = task->wait.mutex.mutex;
		
		armv7m_rtos_mutex_wait_remove(mutex, task);

		mutex->priority = (mutex->waiting ? mutex->waiting->priority : K_PRIORITY_MIN);

		armv7m_rtos_mutex_owner_attach(mutex, task);

		armv7m_rtos_task_priority(task);
	    }
	    
	    task->state &= ~(K_TASK_STATE_BLOCKED | K_TASK_STATE_WAIT_MASK);
	    
	    armv7m_atomic_store((volatile uint32_t*)&task->release, (uint32_t)NULL);

	    if (!(task->state & (K_TASK_STATE_TERMINATED | K_TASK_STATE_SUSPENDED)))
	    {
		armv7m_rtos_task_ready_insert(task);
	    }
	}
	while (task_release != K_TASK_SENTINEL);
    }

    if (task_timeout != armv7m_rtos_control.task_timeout)
    {
	armv7m_rtos_task_timeout_restart();
    }

    task = armv7m_rtos_control.task_self;

    if (task != armv7m_rtos_control.task_ready)
    {
	if (!task ||
	    !(task->state & K_TASK_STATE_READY) ||
	    !(task->mode & (K_TASK_MODE_NOCONCURRENT | K_TASK_MODE_NODISPATCH | K_TASK_MODE_NOPREEMPT)))
	{
	    if (armv7m_rtos_control.task_next != armv7m_rtos_control.task_ready)
	    {
		armv7m_rtos_control.task_next = armv7m_rtos_control.task_ready;
		
		armv7m_pendsv_hook(armv7m_rtos_pendsv_epilogue);
	    }
	}
    }
}

static bool armv7m_rtos_task_unblock(k_task_t *task, uint32_t status)
{
    k_task_t *task_release, *task_owner;
    k_sem_t *sem;
    k_mutex_t *mutex;
    uint32_t priority;

    if (__armv7m_atomic_cas((volatile uint32_t*)&task->release, (uint32_t)NULL, (uint32_t)K_TASK_SENTINEL) != (uint32_t)NULL)
    {
	return false;
    }

    task_release = (k_task_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_release, (uint32_t)task);

    armv7m_atomic_store((volatile uint32_t*)&task->release, (uint32_t)task_release);

    armv7m_rtos_task_return(task, status);

    if (task->state & K_TASK_STATE_TIMEOUT)
    {
	armv7m_rtos_task_timeout_remove(task);
    }

    if (task->state & K_TASK_STATE_WAIT_SEM)
    {
	/* We can always remove this task, because the async sem release logic
	 * skips tasks with a non-NULL task->release.
	 */
	sem = task->wait.sem.sem;

	armv7m_rtos_sem_wait_remove(sem, task);
    }

    if (task->state & K_TASK_STATE_WAIT_MUTEX)
    {
	mutex = task->wait.mutex.mutex;
	
	armv7m_rtos_mutex_wait_remove(mutex, task);

	priority = (mutex->waiting ? mutex->waiting->priority : K_PRIORITY_MIN);

	if (mutex->priority != priority)
	{
	    mutex->priority = priority;
	    
	    task_owner = mutex->owner;

	    armv7m_rtos_mutex_owner_detach(mutex, task_owner);
	    armv7m_rtos_mutex_owner_attach(mutex, task_owner);
	    
	    armv7m_rtos_task_priority(task_owner);
	}
    }
    
    task->state &= ~(K_TASK_STATE_BLOCKED | K_TASK_STATE_WAIT_MASK);

    return true;
}

static bool armv7m_rtos_task_release(k_task_t *task, uint32_t status)
{
    k_task_t *task_release;
    
    if (__armv7m_atomic_cas((volatile uint32_t*)&task->release, (uint32_t)NULL, (uint32_t)K_TASK_SENTINEL) != (uint32_t)NULL)
    {
	return false;
    }

    task_release = (k_task_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_release, (uint32_t)task);

    armv7m_atomic_store((volatile uint32_t*)&task->release, (uint32_t)task_release);

    armv7m_rtos_task_return(task, status);

    return true;
}

static bool armv7m_rtos_task_suspend(k_task_t *task)
{
    k_task_t *task_suspend;

    if (armv7m_atomic_cas((volatile uint32_t*)&task->suspend, (uint32_t)NULL, (uint32_t)K_TASK_SENTINEL) != (uint32_t)NULL)
    {
	return false;
    }
    
    task_suspend = (k_task_t*)armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_suspend, (uint32_t)task);

    armv7m_atomic_store((volatile uint32_t*)&task->suspend, (uint32_t)task_suspend);
    
    return true;
}

static bool armv7m_rtos_task_resume(k_task_t *task)
{
    k_task_t *task_resume;

    if (armv7m_atomic_cas((volatile uint32_t*)&task->resume, (uint32_t)NULL, (uint32_t)K_TASK_SENTINEL) != (uint32_t)NULL)
    {
	return false;
    }
    
    task_resume = (k_task_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_resume, (uint32_t)task);

    armv7m_atomic_store((volatile uint32_t*)&task->resume, (uint32_t)task_resume);
    
    return true;
}

static void armv7m_rtos_task_priority(k_task_t *task)
{
    k_task_t *task_next;
    k_mutex_t *mutex;
    uint32_t priority;

    do
    {
	task_next = NULL;

	priority = task->bpriority;

	mutex = task->mutex;

	if (mutex && (priority > mutex->priority))
	{
	    priority = mutex->priority;
	}
    
	if (task->priority != priority)
	{
	    task->priority = priority;
	    
	    if (task->state & K_TASK_STATE_READY)
	    {
		armv7m_rtos_task_ready_remove(task);
		armv7m_rtos_task_ready_insert(task);
	    }

	    if (task->state & K_TASK_STATE_WAIT_MUTEX)
	    {
		mutex = task->wait.mutex.mutex;

		armv7m_rtos_mutex_wait_remove(mutex, task);
		armv7m_rtos_mutex_wait_insert(mutex, task);
		
		if (mutex->priority != mutex->waiting->priority)
		{
		    mutex->priority = mutex->waiting->priority;
			    
		    task_next = mutex->owner;
			    
		    armv7m_rtos_mutex_owner_detach(mutex, task_next);
		    armv7m_rtos_mutex_owner_attach(mutex, task_next);
		}
	    }
	}

	task = task_next;
    }
    while (task);
}

static void armv7m_rtos_task_timeout_insert(k_task_t *task, uint32_t ticks)
{
    k_task_t *task_this;

    if (armv7m_rtos_control.task_timeout == NULL)
    {
        armv7m_rtos_control.task_timeout = task;
    }
    else
    {
        task_this = armv7m_rtos_control.task_timeout;

        do
        {
            if (ticks < task_this->timeout.ticks)
            {
                if (armv7m_rtos_control.task_timeout == task_this)
                {
                    armv7m_rtos_control.task_timeout = task;
                }

		task_this->timeout.ticks -= ticks;
		
                break;
            }

	    ticks -= task_this->timeout.ticks;
	    
            task_this = task_this->timeout.next;
        }
        while (armv7m_rtos_control.task_timeout != task_this);

        task->timeout.previous = task_this->timeout.previous;
        task->timeout.next = task_this;
        
        task->timeout.previous->timeout.next = task;
        task->timeout.next->timeout.previous = task;
    }

    task->timeout.ticks = ticks;

    task->state |= K_TASK_STATE_TIMEOUT;
}

static void armv7m_rtos_task_timeout_remove(k_task_t *task)
{
    task->state &= ~K_TASK_STATE_TIMEOUT;

    if (task->timeout.next == task)
    {
        armv7m_rtos_control.task_timeout = NULL;
    }
    else
    {
        if (armv7m_rtos_control.task_timeout == task)
        {
            armv7m_rtos_control.task_timeout = task->timeout.next;
        }

	task->timeout.next->timeout.ticks += task->timeout.ticks;
        
        task->timeout.next->timeout.previous = task->timeout.previous;
        task->timeout.previous->timeout.next = task->timeout.next;

        task->timeout.next = task;
        task->timeout.previous = task;
    }
}

static void armv7m_rtos_task_timeout_start(void)
{
    if (armv7m_rtos_control.lptim_ticks)
    {
	if (armv7m_rtos_control.lptim_ticks != armv7m_rtos_control.task_timeout->timeout.ticks)
	{
	    armv7m_rtos_control.lptim_ticks = armv7m_rtos_control.task_timeout->timeout.ticks;
	    
	    stm32wb_lptim_timeout_restart(&armv7m_rtos_control.lptim_timeout, armv7m_rtos_control.lptim_ticks, armv7m_rtos_task_timeout_callback);
	}
    }
    else
    {
	armv7m_rtos_control.lptim_ticks = armv7m_rtos_control.task_timeout->timeout.ticks;
	    
	stm32wb_lptim_timeout_start(&armv7m_rtos_control.lptim_timeout, armv7m_rtos_control.lptim_ticks, armv7m_rtos_task_timeout_callback);
    }
}

static void armv7m_rtos_task_timeout_restart(void)
{
    if (armv7m_rtos_control.task_timeout)
    {
	if (armv7m_rtos_control.lptim_ticks != armv7m_rtos_control.task_timeout->timeout.ticks)
	{
	    armv7m_rtos_control.lptim_ticks = armv7m_rtos_control.task_timeout->timeout.ticks;
	    
	    stm32wb_lptim_timeout_restart(&armv7m_rtos_control.lptim_timeout, armv7m_rtos_control.lptim_ticks, armv7m_rtos_task_timeout_callback);
	}
    }
    else
    {
	if (armv7m_rtos_control.lptim_ticks)
	{
	    armv7m_rtos_control.lptim_ticks = 0;
	    
	    stm32wb_lptim_timeout_stop(&armv7m_rtos_control.lptim_timeout);
	}
    }
}

static void armv7m_rtos_task_timeout_callback(stm32wb_lptim_timeout_t *timeout)
{
    k_task_t *task;

    armv7m_rtos_control.lptim_ticks = 0;
    
    if (armv7m_rtos_control.task_timeout)
    {
	task = armv7m_rtos_control.task_timeout;

	task->timeout.ticks = 0;

	do
	{
	    armv7m_rtos_task_unblock(task, (((task->state & K_TASK_STATE_WAIT_MASK) == K_TASK_STATE_WAIT_DELAY) ? K_NO_ERROR : K_ERR_TIMEOUT));

	    task = armv7m_rtos_control.task_timeout;
	}
	while (task && (task->timeout.ticks == 0));

	if (armv7m_rtos_control.task_timeout)
	{
	    armv7m_rtos_control.lptim_ticks = armv7m_rtos_control.task_timeout->timeout.ticks;
	    
	    stm32wb_lptim_timeout_restart(&armv7m_rtos_control.lptim_timeout, armv7m_rtos_control.lptim_ticks, armv7m_rtos_task_timeout_callback);
	}

	armv7m_rtos_task_schedule();
    }
}

static void armv7m_rtos_sem_wait_insert(k_sem_t *sem, k_task_t *task)
{
    task->next = NULL;
    task->previous = NULL;

    if (sem->waiting.head == NULL)
    {
	sem->waiting.head = task;
    }
    else
    {
	task->previous = sem->waiting.tail;
	task->previous->next = task;
    }

    sem->waiting.tail = task;

    armv7m_atomic_cas((volatile uint32_t*)&sem->waiting.release, (uint32_t)NULL, (uint32_t)task);
}

static void armv7m_rtos_sem_wait_remove(k_sem_t *sem, k_task_t *task)
{
    if (sem->waiting.head == task)
    {
	if (task->next == NULL)
	{
	    sem->waiting.head = NULL;
	    sem->waiting.tail = NULL;
	}
	else
	{
	    task->next->previous = NULL;

	    sem->waiting.head = task->next;
	}
    }
    else
    {
	if (task->next == NULL)
	{
	    task->previous->next = NULL;

	    sem->waiting.tail = task->previous;
	}
	else
	{
	    task->next->previous = task->previous;
	    task->previous->next = task->next;
	}
    }
    
    armv7m_atomic_cas((volatile uint32_t*)&sem->waiting.release, (uint32_t)task, (uint32_t)task->next);

    task->next = task;
    task->previous = task;
}

static k_task_t* armv7m_rtos_sem_wait_release(k_sem_t *sem)
{
    k_task_t *task;
    
    task = sem->waiting.release;

    if (task)
    {
	do
	{
	    if (armv7m_atomic_cas((volatile uint32_t*)&sem->waiting.release, (uint32_t)task, (uint32_t)task->next) == (uint32_t)task)
	    {
		if (!task->release)
		{
		    break;
		}
	    }
	    
	    task = sem->waiting.release;
	}
	while (task);
    }

    return task;
}

static void armv7m_rtos_sem_wait_acquire(k_sem_t *sem)
{
    k_task_t *task;

    task = sem->waiting.head;

    if (!task) { __BKPT(); }
    
    if (task->next == NULL)
    {
	sem->waiting.head = NULL;
	sem->waiting.tail = NULL;
    }
    else
    {
	task->next->previous = NULL;
	
	sem->waiting.head = task->next;
    }

    task->next = task;
    task->previous = task;
}

static inline void armv7m_rtos_mutex_owner_attach(k_mutex_t *mutex, k_task_t *task)
{
    armv7m_rtos_queue_insert((k_node_t**)&task->mutex, (k_node_t*)mutex);

    mutex->owner = task;
}

static inline void armv7m_rtos_mutex_owner_detach(k_mutex_t *mutex, k_task_t *task)
{
    mutex->owner = NULL;

    armv7m_rtos_queue_remove((k_node_t**)&task->mutex, (k_node_t*)mutex);
}

static inline void armv7m_rtos_mutex_wait_insert(k_mutex_t *mutex, k_task_t *task)
{
    armv7m_rtos_queue_insert((k_node_t**)&mutex->waiting, (k_node_t*)task);
}

static inline void armv7m_rtos_mutex_wait_remove(k_mutex_t *mutex, k_task_t *task)
{
    armv7m_rtos_queue_remove((k_node_t**)&mutex->waiting, (k_node_t*)task);
}

static void __attribute__((optimize("O3"), used)) armv7m_rtos_work_dispatch(void)
{
    k_work_t *work;
    
    armv7m_rtos_control.work_routine = NULL;

    if (armv7m_rtos_control.work_head != K_WORK_SENTINEL)
    {
	work = armv7m_rtos_control.work_head;
	    
	if (armv7m_rtos_control.work_head == armv7m_rtos_control.work_tail)
	{
	    armv7m_rtos_control.work_head = K_WORK_SENTINEL;
	    armv7m_rtos_control.work_tail = K_WORK_SENTINEL;
	}
	else
	{
	    armv7m_rtos_control.work_head = work->next;
	}
	
	armv7m_rtos_control.work_routine = work->routine;
	armv7m_rtos_control.work_context = work->context;
	
	armv7m_atomic_store((volatile uint32_t*)&work->next, (uint32_t)NULL);
    }
}

static void __attribute__((optimize("O3"), used)) armv7m_rtos_work_schedule(void)
{
    k_work_t *work, *work_submit, *work_next, *work_head, *work_tail;
    k_task_t *task;
    
    if (armv7m_rtos_control.work_submit != K_WORK_SENTINEL)
    {
	work_submit = (k_work_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.work_submit, (uint32_t)K_WORK_SENTINEL);

	for (work_head = K_WORK_SENTINEL, work_tail = work_submit; work_submit != K_WORK_SENTINEL; work_submit = work_next)
        {
            work_next = work_submit->next;

            armv7m_atomic_store((volatile uint32_t*)&work_submit->next, (uint32_t)work_head);

            work_head = work_submit;
        }
	
        if (armv7m_rtos_control.work_head == K_WORK_SENTINEL)
        {
            armv7m_rtos_control.work_head = work_head;
        }
        else
        {
	    armv7m_atomic_store((volatile uint32_t*)&armv7m_rtos_control.work_tail->next, (uint32_t)work_head);
        }
	
        armv7m_rtos_control.work_tail = work_tail;
    
	if (!armv7m_rtos_control.work_routine)
	{
	    task = armv7m_rtos_control.task_self;

	    if (!task ||
		!(task->state & K_TASK_STATE_READY) ||
		!(task->mode & K_TASK_MODE_NOCONCURRENT))
	    {
		work = armv7m_rtos_control.work_head;
		
		if (armv7m_rtos_control.work_head == armv7m_rtos_control.work_tail)
		{
		    armv7m_rtos_control.work_head = K_WORK_SENTINEL;
		    armv7m_rtos_control.work_tail = K_WORK_SENTINEL;
		}
		else
		{
		    armv7m_rtos_control.work_head = work->next;
		}
		
		armv7m_rtos_control.work_routine = work->routine;
		armv7m_rtos_control.work_context = work->context;
		
		armv7m_atomic_store((volatile uint32_t*)&work->next, (uint32_t)NULL);
		
		armv7m_pendsv_hook(armv7m_rtos_pendsv_epilogue);
	    }
	}
    }
}

/******************************************************************************************************************************/

static void __svc_armv7m_rtos_task_schedule(void)
{
    armv7m_rtos_task_schedule();
}

static int __svc_armv7m_rtos_task_destroy(k_task_t *task)
{
    k_task_t *task_created, *task_element, *task_previous;
    k_mutex_t *mutex;
    bool success;
    
    if (!task)
    {
	return K_ERR_INVALID_OBJECT;
    }

    if (task == &armv7m_rtos_control.task_default)
    {
	return K_ERR_OBJECT_PROTECTED;
    }

    if (task != armv7m_rtos_control.task_self)
    {
	if (task->mode & K_TASK_MODE_NOTERMINATE)
	{
	    return K_ERR_OBJECT_PROTECTED;
	}
    }

    /* Async single linked list remove. If removing the head, check whether somebody had not
     * written a new head, in which case, retry the remove.
     */
    do
    {
	success = false;

	for (task_previous = NULL, task_created = armv7m_rtos_control.task_created, task_element = task_created; task_element != K_TASK_SENTINEL; task_previous = task_element, task_element = task_element->next)
	{
	    if (task_element == task)
	    {
		if (task_previous)
		{
		    armv7m_atomic_store((volatile uint32_t*)&task_previous->next, (uint32_t)task->next);

		    success = true;
		}
		else
		{
 	 	    if ((k_task_t*)armv7m_atomic_cas((volatile uint32_t*)&armv7m_rtos_control.task_created, (uint32_t)task_created, (uint32_t)task->next) == task_created)
		    {
			success = true;
		    }
		}

		break;
	    }
	}
    }
    while (!success);

    task->state |= K_TASK_STATE_TERMINATED;
    
    armv7m_atomic_store((volatile uint32_t*)&task->created, (uint32_t)NULL);

    if (task->state & K_TASK_STATE_READY)
    {
	armv7m_rtos_task_ready_remove(task);
    }

    armv7m_rtos_task_unblock(task, K_ERR_UNSATISFIED);
    
    /* Nuke task_self, so schedulewill be forced to schedule the next task,
     * but after unblock, so that unblock still can put the resturn code
     * somewhere, even though it's unused.
     */
    if (task == armv7m_rtos_control.task_self)
    {
	armv7m_rtos_control.task_self = NULL;
    }

    while (task->mutex)
    {
	mutex = task->mutex;

	mutex->level = 1;
	
	armv7m_rtos_mutex_owner_detach(mutex, task);

	if (mutex->waiting)
	{
	    armv7m_rtos_task_release(mutex->waiting, K_NO_ERROR);
	}
    }
    
    armv7m_rtos_task_schedule();

    task->state = 0;
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_info(k_task_t *task, k_task_info_t *p_task_info_return, uint32_t a2, uint32_t a3, void *exc_stack, uint32_t exc_return)
{
    uint32_t *stack_limit;
    
    if (!task)
    {
	return K_ERR_INVALID_OBJECT;
    }

    if (task->state & K_TASK_STATE_TERMINATED)
    {
	return K_ERR_OBJECT_DELETED;
    }

    if (!p_task_info_return)
    {
	return K_ERR_INVALID_ADDRESS;
    }


    for (stack_limit = (uint32_t*)task->stack_limit; stack_limit < (uint32_t*)task->stack_top; stack_limit += 2)
    {
	if ((stack_limit[0] != 0xaaaaaaaa) || (stack_limit[1] != 0x77777777))
	{
	    break;
	}
    }
    
    p_task_info_return->priority = task->priority;
    p_task_info_return->bpriority = task->bpriority;
    p_task_info_return->mode = task->mode & (K_MODE_NOPREEMPT | K_MODE_NOCONCURRENT | K_MODE_NOTERMINATE);
    p_task_info_return->state = (((task == armv7m_rtos_control.task_self) ? K_STATE_RUNNING : 0) | (task->state & (K_STATE_READY | K_STATE_BLOCKED | K_STATE_SUSPENDED)));
    p_task_info_return->events = task->events;
    p_task_info_return->stack_current = (uint32_t)task->stack_top - ((task == armv7m_rtos_control.task_self) ? (uint32_t)exc_stack : (uint32_t)task->stack_current);
    p_task_info_return->stack_limit = (uint32_t)task->stack_top - (uint32_t)stack_limit;
    p_task_info_return->stack_size = (uint32_t)task->stack_top - (uint32_t)task->stack_limit;
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_start(k_task_t *task, k_task_routine_t routine, void *context, uint32_t argument)
{
    if (!task)
    {
	return K_ERR_INVALID_OBJECT;
    }

    if (task->state & K_TASK_STATE_TERMINATED)
    {
	return K_ERR_OBJECT_DELETED;
    }

    if (task == &armv7m_rtos_control.task_default)
    {
	return K_ERR_OBJECT_PROTECTED;
    }
    
    if ((k_task_routine_t)armv7m_atomic_cas((volatile uint32_t*)&task->routine, (uint32_t)NULL, (uint32_t)routine) != NULL)
    {
	return K_ERR_TASK_ALREADY_STARTED;
    }

    task->context = context;
    
    armv7m_rtos_task_start(task, argument);

    armv7m_rtos_task_release(task, argument);

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
	armv7m_rtos_task_schedule();
    }
    else
    {
	armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_TASK_SCHEDULE);
    }
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_restart(k_task_t *task, uint32_t argument)
{
    if (task == &armv7m_rtos_control.task_default)
    {
	return K_ERR_OBJECT_PROTECTED;
    }

    if (task != armv7m_rtos_control.task_self)
    {
	if (task->mode & K_TASK_MODE_NOTERMINATE)
	{
	    return K_ERR_OBJECT_PROTECTED;
	}
    }
    
    if (task->routine == NULL)
    {
	return K_ERR_TASK_NOT_STARTED;
    }
    
    task->state &= ~(K_TASK_STATE_BLOCKED | K_TASK_STATE_WAIT_MASK);
    
    armv7m_rtos_task_unblock(task, K_ERR_UNSATISFIED);

    task->priority = task->ipriority;
    task->mode = task->imode;
    task->events = 0;
    
    armv7m_rtos_task_start(task, argument);

    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_suspend(k_task_t *task)
{
    if (!task)
    {
	return K_ERR_INVALID_OBJECT;
    }

    if (task->state & K_TASK_STATE_TERMINATED)
    {
	return K_ERR_OBJECT_DELETED;
    }

    if (task != armv7m_rtos_control.task_self)
    {
	if (task->mode & K_TASK_MODE_NOPREEMPT)
	{
	    return K_ERR_OBJECT_PROTECTED;
	}
    }

    if ((task->state & K_TASK_STATE_SUSPENDED) || task->suspend)
    {
	return K_ERR_TASK_ALREADY_SUSPENDED;
    }

    if (!armv7m_rtos_task_suspend(task))
    {
	return K_ERR_TASK_ALREADY_SUSPENDED;
    }
    
    if (armv7m_core_is_in_pendsv_or_svcall())
    {
	armv7m_rtos_task_schedule();
    }
    else
    {
	armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_TASK_SCHEDULE);
    }
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_resume(k_task_t *task)
{
    if (!task)
    {
	return K_ERR_INVALID_OBJECT;
    }

    if (task->state & K_TASK_STATE_TERMINATED)
    {
	return K_ERR_OBJECT_DELETED;
    }

    if (!(task->state & K_TASK_STATE_SUSPENDED) && !task->suspend)
    {
	return K_ERR_TASK_NOT_SUSPENDED;
    }

    if (!armv7m_rtos_task_resume(task))
    {
	return K_ERR_TASK_NOT_SUSPENDED;
    }

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
	armv7m_rtos_task_schedule();
    }
    else
    {
	armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_TASK_SCHEDULE);
    }
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_set_priority(k_task_t *task, uint32_t priority, uint32_t *p_priority_return)
{
    uint32_t priority_return;
    
    if (task == &armv7m_rtos_control.task_default)
    {
	if (priority != K_PRIORITY_CURRENT)
	{
	    return K_ERR_OBJECT_PROTECTED;
	}
    }

    priority_return = task->priority;
    
    if (priority != K_PRIORITY_CURRENT)
    {
	if (priority > K_PRIORITY_MIN)
	{
	    return K_ERR_INVALID_PRIORITY;
	}
	
	if (task->bpriority != priority)
	{
	    task->bpriority = priority;

	    armv7m_rtos_task_priority(task);

	    armv7m_rtos_task_schedule();
	}
    }
    
    if (p_priority_return)
    {
	*p_priority_return = priority_return;
    }

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_task_delay(uint32_t delay, uint32_t a1 __attribute__((unused)), uint32_t a2 __attribute__((unused)), uint32_t a3 __attribute__((unused)), void *exc_stack, uint32_t exc_return)
{
    k_task_t *task;
    
    task = armv7m_rtos_control.task_self;

    armv7m_rtos_task_ready_remove(task);

    armv7m_rtos_task_yield(task, exc_stack, exc_return);
    
    task->state |= (K_TASK_STATE_BLOCKED | K_TASK_STATE_WAIT_DELAY);
    
    if (delay)
    {
	armv7m_rtos_task_timeout_insert(task, delay);

	armv7m_rtos_task_timeout_start();
    }
    else
    {
	armv7m_rtos_task_unblock(task, K_NO_ERROR);
    }

    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_event_send(k_task_t *task, uint32_t events)
{
    uint32_t events_return, *p_events_return;

    if (!task)
    {
	return K_ERR_INVALID_OBJECT;
    }

    if (task->state & K_TASK_STATE_TERMINATED)
    {
	return K_ERR_OBJECT_DELETED;
    }

    armv7m_atomic_or(&task->events, events);

    if (task->state & (K_TASK_STATE_WAIT_EVENT_ALL | K_TASK_STATE_WAIT_EVENT_ANY))
    {
	events_return = task->events & task->wait.event.events;

	if (task->state & K_TASK_STATE_WAIT_EVENT_ALL)
	{
	    if (events_return != task->wait.event.events)
	    {
		events_return = 0;
	    }
	}

	if (events_return)
	{
	    if (armv7m_rtos_task_release(task, K_NO_ERROR))
	    {
		armv7m_atomic_and(&task->events, ~events_return);

		p_events_return = task->wait.event.p_events_return;
		
		if (p_events_return)
		{
		    *p_events_return = events_return;
		}

		if (armv7m_core_is_in_pendsv_or_svcall())
		{
		    armv7m_rtos_task_schedule();
		}
		else
		{
		    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_TASK_SCHEDULE);
		}
	    }
	}
    }

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_event_receive(uint32_t events, uint32_t mode, uint32_t timeout, uint32_t *p_events_return, void *exc_stack, uint32_t exc_return)
{
    k_task_t *task;
    uint32_t events_return;

    task = armv7m_rtos_control.task_self;

    events_return = task->events & events;

    if (mode == K_EVENT_ALL)
    {
	if (events_return != events)
	{
	    events_return = 0;
	}
    }
    
    if (events_return)
    {
	armv7m_atomic_and(&task->events, ~events_return);

	if (p_events_return)
	{
	    *p_events_return = events_return;
	}

	return K_NO_ERROR;
    }

    if (timeout == K_TIMEOUT_NONE)
    {
	return K_ERR_UNSATISFIED;
    }

    armv7m_rtos_task_ready_remove(task);

    armv7m_rtos_task_yield(task, exc_stack, exc_return);
    
    task->wait.event.events = events;
    task->wait.event.p_events_return = p_events_return;

    task->state |= (K_TASK_STATE_BLOCKED | ((mode == K_EVENT_ALL) ? K_TASK_STATE_WAIT_EVENT_ALL : K_TASK_STATE_WAIT_EVENT_ANY));
    
    if (timeout != K_TIMEOUT_FOREVER)
    {
	armv7m_rtos_task_timeout_insert(task, timeout);

	armv7m_rtos_task_timeout_start();
    }

    events_return = task->events & events;

    if (mode == K_EVENT_ALL)
    {
	if (events_return != events)
	{
	    events_return = 0;
	}
    }
    
    if (events_return)
    {
	if (armv7m_rtos_task_release(task, K_NO_ERROR))
	{
	    armv7m_atomic_and(&task->events, ~events_return);
	    
	    if (p_events_return)
	    {
		*p_events_return = events_return;
	    }
	}
    }

    armv7m_rtos_task_schedule();

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_sem_destroy(k_sem_t *sem)
{
    k_task_t *task;

    if (!sem)
    {
	return K_ERR_INVALID_OBJECT;
    }

    do
    {
	task = armv7m_rtos_sem_wait_release(sem);
	
	if (task)
	{
	    armv7m_rtos_task_release(task, K_ERR_UNSATISFIED);
	}
    }
    while (task);

    armv7m_rtos_task_schedule();

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_sem_acquire(k_sem_t *sem, uint32_t timeout, uint32_t a2 __attribute__((unused)), uint32_t a3 __attribute__((unused)), void *exc_stack, uint32_t exc_return)
{
    uint32_t count;
    k_task_t *task;

    if (!sem)
    {
	return K_ERR_INVALID_OBJECT;
    }
    
    task = armv7m_rtos_control.task_self;

    count = armv7m_atomic_dec(&sem->count);

    if (count)
    {
	return K_NO_ERROR;
    }

    if (timeout == K_TIMEOUT_NONE)
    {
	return K_ERR_UNSATISFIED;
    }
    
    armv7m_rtos_task_ready_remove(task);

    armv7m_rtos_task_yield(task, exc_stack, exc_return);
    
    task->wait.sem.sem = sem;

    task->state |= (K_TASK_STATE_BLOCKED | K_TASK_STATE_WAIT_SEM);
    
    if (timeout != K_TIMEOUT_FOREVER)
    {
	armv7m_rtos_task_timeout_insert(task, timeout);

	armv7m_rtos_task_timeout_start();
    }

    armv7m_rtos_sem_wait_insert(sem, task);

    if (sem->count)
    {
	task = armv7m_rtos_sem_wait_release(sem);

	if (task)
	{
	    armv7m_atomic_dec(&sem->count);

	    armv7m_rtos_task_release(task, K_NO_ERROR);
	}
    }

    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_sem_release(k_sem_t *sem)
{
    k_task_t *task;
    
    if (!sem)
    {
	return K_ERR_INVALID_OBJECT;
    }

    task = armv7m_rtos_sem_wait_release(sem);
    
    if (task)
    {
	armv7m_rtos_task_release(task, K_NO_ERROR);

	if (armv7m_core_is_in_pendsv_or_svcall())
	{
	    armv7m_rtos_task_schedule();
	}
	else
	{
	    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_TASK_SCHEDULE);
	}
    }
    else
    {
	armv7m_atomic_inc(&sem->count, 0xffffffff);
    }

    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_mutex_destroy(k_mutex_t *mutex)
{
    k_task_t *task;

    if (!mutex)
    {
	return K_ERR_INVALID_OBJECT;
    }
    
    if (mutex->owner)
    {
	task = armv7m_rtos_control.task_self;

	if (mutex->owner != task)
	{
	    return K_ERR_NOT_OWNER_OF_MUTEX;
	}

	mutex->level = 1;

	armv7m_rtos_mutex_owner_detach(mutex, task);

	armv7m_rtos_task_priority(task);
    }

    while (mutex->waiting)
    {
	armv7m_rtos_task_unblock(mutex->waiting, K_ERR_UNSATISFIED);
    }

    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_mutex_trylock(k_mutex_t *mutex)
{
  k_task_t *task;

    task = armv7m_rtos_control.task_self;

    if (!mutex->waiting)
    {
	armv7m_rtos_mutex_owner_attach(mutex, task);

	/* This is here because the k_mutex_trylock() code
	 * had task->mode set to K_TASK_MODE_NODISPATCH, but did not
	 * trigger a schedule on the SVCALL path.
	 */
	if (task != armv7m_rtos_control.task_ready)
	{
	    armv7m_rtos_task_schedule();
	}
	
	return K_NO_ERROR;
    }

    return K_ERR_UNSATISFIED;
}

static int __svc_armv7m_rtos_mutex_lock(k_mutex_t *mutex, uint32_t a1, uint32_t a2, uint32_t a3, void *exc_stack, uint32_t exc_return)
{
    k_task_t *task, *task_owner;

    task = armv7m_rtos_control.task_self;

    if (!mutex->waiting)
    {
	armv7m_rtos_mutex_owner_attach(mutex, task);

	/* This is here because the k_mutex_lock() code
	 * had task->mode set to K_TASK_MODE_NODISPATCH, but did not
	 * trigger a schedule on the SVCALL path.
	 */
	if (task != armv7m_rtos_control.task_ready)
	{
	    armv7m_rtos_task_schedule();
	}
	
	return K_NO_ERROR;
    }
    
    armv7m_rtos_task_ready_remove(task);

    armv7m_rtos_task_yield(task, exc_stack, exc_return);
    
    task->wait.mutex.mutex = mutex;

    task->state |= (K_TASK_STATE_BLOCKED | K_TASK_STATE_WAIT_MUTEX);

    armv7m_rtos_mutex_wait_insert(mutex, task);

    if (mutex->priority != mutex->waiting->priority)
    {
	mutex->priority = mutex->waiting->priority;
			    
	task_owner = mutex->owner;
			    
	armv7m_rtos_mutex_owner_detach(mutex, task_owner);
	armv7m_rtos_mutex_owner_attach(mutex, task_owner);

	armv7m_rtos_task_priority(task_owner);
    }

    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __svc_armv7m_rtos_mutex_unlock(k_mutex_t *mutex)
{
    k_task_t *task;

    task = armv7m_rtos_control.task_self;

    armv7m_rtos_mutex_owner_detach(mutex, task);

    armv7m_rtos_task_priority(task);

    if (mutex->waiting)
    {
	armv7m_rtos_task_release(mutex->waiting, K_NO_ERROR);
    }
	
    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static void __svc_armv7m_rtos_work_schedule(void)
{
    armv7m_rtos_work_schedule();
    armv7m_rtos_task_schedule();
}

static int __svc_armv7m_rtos_work_destroy(k_work_t *work)
{
    k_work_t *work_submit, *work_element, *work_previous;
    bool success;

    /* This is a tad painful. If work->next points to something, then first walk the work_head/work_tail list. 
     * Should work not be in that list, walk the submit list, and do a async remove with rety.
     */
    if (work->next)
    {
	for (work_previous = NULL, work_element = armv7m_rtos_control.work_head; work_element != K_WORK_SENTINEL; work_previous = work_element, work_element = work_element->next)
	{
	    if (work_element == work)
	    {
		if (work_previous)
		{
		    armv7m_atomic_store((volatile uint32_t*)&work_previous->next, (uint32_t)work->next);
		}
		else
		{
		    armv7m_rtos_control.work_head = work->next;
		}
		
		if (work->next == K_WORK_SENTINEL)
		{
		    armv7m_rtos_control.work_tail = K_WORK_SENTINEL;
		}
		
		armv7m_atomic_store((volatile uint32_t*)&work->next, (uint32_t)NULL);

		break;
	    }
	}

	if (work->next)
	{
	    do
	    {
		success = false;

		for (work_previous = NULL, work_submit = armv7m_rtos_control.work_submit, work_element = work_submit; work_element != K_WORK_SENTINEL; work_previous = work_element, work_element = work_element->next)
		{
		    if (work_element == work)
		    {
			if (work_previous)
			{
			    armv7m_atomic_store((volatile uint32_t*)&work_previous->next, (uint32_t)work->next);

			    success = true;
			}
			else
			{
			    if ((k_work_t*)armv7m_atomic_cas((volatile uint32_t*)&armv7m_rtos_control.work_submit, (uint32_t)work_submit, (uint32_t)work->next) == work_submit)
			    {
				success = true;
			    }
			}

			break;
		    }
		}
	    }
	    while (!success);

	    armv7m_atomic_store((volatile uint32_t*)&work->next, (uint32_t)NULL);
	}
    }

    return K_NO_ERROR;
}

static __attribute__((optimize("O3"))) bool __svc_armv7m_rtos_work_submit(k_work_t *work)
{
    k_work_t *work_submit;

    work_submit = (k_work_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.work_submit, (uint32_t)work);

    armv7m_atomic_store((volatile uint32_t*)&work->next, (uint32_t)work_submit);
    
    if (work_submit == K_WORK_SENTINEL)
    {
	armv7m_rtos_work_schedule();
    }
    
    return true;
}

/******************************************************************************************************************************/

void __armv7m_rtos_initialize()
{
    uint32_t *stack;

    armv7m_rtos_control.work_routine = NULL;
    armv7m_rtos_control.work_context = NULL;
    armv7m_rtos_control.work_head = K_WORK_SENTINEL;
    armv7m_rtos_control.work_tail = K_WORK_SENTINEL;
    armv7m_rtos_control.work_submit = K_WORK_SENTINEL;

    armv7m_rtos_control.task_self = &armv7m_rtos_control.task_default;
    armv7m_rtos_control.task_next = &armv7m_rtos_control.task_default;
    armv7m_rtos_control.task_ready = &armv7m_rtos_control.task_default;
    armv7m_rtos_control.task_timeout = NULL;
    armv7m_rtos_control.task_created = &armv7m_rtos_control.task_default;
    armv7m_rtos_control.task_suspend = K_TASK_SENTINEL;
    armv7m_rtos_control.task_resume = K_TASK_SENTINEL;
    armv7m_rtos_control.task_release = K_TASK_SENTINEL;

    armv7m_rtos_control.task_default.next = &armv7m_rtos_control.task_default;
    armv7m_rtos_control.task_default.previous = &armv7m_rtos_control.task_default;
    armv7m_rtos_control.task_default.priority = K_PRIORITY_MIN;
    armv7m_rtos_control.task_default.mode = 0;
    armv7m_rtos_control.task_default.state = K_TASK_STATE_READY;
    armv7m_rtos_control.task_default.name = "DEFAULT";
    armv7m_rtos_control.task_default.events = 0;
    armv7m_rtos_control.task_default.bpriority = K_PRIORITY_MIN;
    armv7m_rtos_control.task_default.ipriority = K_PRIORITY_MIN;
    armv7m_rtos_control.task_default.imode = 0;
    armv7m_rtos_control.task_default.exc_return = 0;
    armv7m_rtos_control.task_default.stack_current = NULL;
    armv7m_rtos_control.task_default.stack_limit = (void*)&__StackLimit[0];
    armv7m_rtos_control.task_default.stack_top = (void*)&__StackTop[0];
    armv7m_rtos_control.task_default.routine = NULL;
    armv7m_rtos_control.task_default.context = NULL;
    armv7m_rtos_control.task_default.mutex = NULL;
    armv7m_rtos_control.task_default.created = K_TASK_SENTINEL;
    armv7m_rtos_control.task_default.suspend = NULL;
    armv7m_rtos_control.task_default.resume = NULL;
    armv7m_rtos_control.task_default.release = NULL;
    armv7m_rtos_control.task_default.timeout.next = &armv7m_rtos_control.task_default;
    armv7m_rtos_control.task_default.timeout.previous = &armv7m_rtos_control.task_default;
    armv7m_rtos_control.task_default.timeout.ticks = 0;

    stm32wb_lptim_timeout_create(&armv7m_rtos_control.lptim_timeout);
    armv7m_rtos_control.lptim_ticks = 0;

    for (stack = (uint32_t*)&__StackLimit[0]; stack < (uint32_t*)((uint32_t)__builtin_frame_address(0) & ~7); stack += 2)
    {
	stack[0] = 0xaaaaaaaa;
	stack[1] = 0x77777777;
    }
}

int k_task_create(k_task_t *task, const char *name, uint32_t priority, void *stack_base, uint32_t stack_size, uint32_t mode)
{
    uint32_t *stack;
    k_task_t *task_created;

    if ((priority < K_PRIORITY_MAX) || (priority > K_PRIORITY_MIN))
    {
	return K_ERR_INVALID_PRIORITY;
    }

    if ((uint32_t)stack_base & 7)
    {
	return K_ERR_INVALID_ADDRESS;
    }

    if (stack_size & 7)
    {
	return K_ERR_INVALID_SIZE;
    }
    
    if (mode & ~(K_MODE_NOPREEMPT | K_MODE_NOCONCURRENT | K_MODE_NOTERMINATE))
    {
	return K_ERR_INVALID_MODE;
    }
    
    task->next = task;
    task->previous = task;
    task->priority = priority;
    task->mode = mode;
    task->state = 0;
    task->events = 0;
    task->name = name;
    task->bpriority = priority;
    task->ipriority = priority;
    task->imode = mode;
    task->exc_return = 0;
    task->stack_current = (void*)((uint32_t)stack_base + stack_size);
    task->stack_limit = (void*)((uint32_t)stack_base);
    task->stack_top = (void*)((uint32_t)stack_base + stack_size);
    task->routine = NULL;
    task->context = NULL;
    task->mutex = NULL;
    task->created = K_TASK_SENTINEL;
    task->suspend = NULL;
    task->resume = NULL;
    task->release = NULL;
    task->timeout.next = task;
    task->timeout.previous = task;
    task->timeout.ticks = 0;

    do
    {
	task_created = armv7m_rtos_control.task_created;
	
	armv7m_atomic_store((volatile uint32_t*)&task->created, (uint32_t)task_created);
    }
    while ((k_task_t*)__armv7m_atomic_cas((volatile uint32_t*)&armv7m_rtos_control.task_created, (uint32_t)task_created, (uint32_t)task) != task_created);
    
    for (stack = (uint32_t*)task->stack_limit; stack < (uint32_t*)task->stack_top; stack += 2)
    {
	stack[0] = 0xaaaaaaaa;
	stack[1] = 0x77777777;
    }

    return K_NO_ERROR;
}
  
int k_task_destroy(k_task_t *task)
{
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.work_context)
    {
	return K_ERR_ILLEGAL_USE;
    }
    
    if (task == K_TASK_SELF)
    {
	task = armv7m_rtos_control.task_self;
    }
	
    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_destroy, (uint32_t)task);
}

int k_task_info(k_task_t *task, k_task_info_t *p_task_info_return)
{
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (task == K_TASK_SELF)
    {
	if (armv7m_rtos_control.work_routine)
	{
	    return K_ERR_ILLEGAL_USE;
	}
	
	task = armv7m_rtos_control.task_self;
    }

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_task_info, (uint32_t)task, (uint32_t)p_task_info_return);
}

int k_task_start(k_task_t *task, k_task_routine_t routine, void *context, uint32_t argument)
{
    if (armv7m_core_is_in_thread())
    {
	return armv7m_svcall_4((uint32_t)&__svc_armv7m_rtos_task_start, (uint32_t)task, (uint32_t)routine, (uint32_t)context, (uint32_t)argument);
    }

    return __svc_armv7m_rtos_task_start(task, routine, context, argument);
}

int k_task_restart(k_task_t *task, uint32_t argument)
{
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.work_routine)
    {
	return K_ERR_ILLEGAL_USE;
    }
    
    if (task == K_TASK_SELF)
    {
	task = armv7m_rtos_control.task_self;
    }
	
    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_task_restart, (uint32_t)task, (uint32_t)argument);
}

k_task_t* k_task_self(void)
{
    return armv7m_rtos_control.task_self;
}

k_task_t *k_task_default(void)
{
    return &armv7m_rtos_control.task_default;
}

int k_task_suspend(k_task_t *task)
{
    if (!armv7m_core_is_in_thread())
    {
	if (task == K_TASK_SELF)
	{
	    if (armv7m_rtos_control.work_routine)
	    {
		return K_ERR_ILLEGAL_USE;
	    }
	    
	    task = armv7m_rtos_control.task_self;
	}

        return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_suspend, (uint32_t)task);
    }
    
    return __svc_armv7m_rtos_task_suspend(task);
}

int k_task_resume(k_task_t *task)
{
    if (armv7m_core_is_in_thread())
    {
	return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_resume, (uint32_t)task);
    }

    return __svc_armv7m_rtos_task_resume(task);
}

bool k_task_is_suspended(k_task_t *task)
{
    return (((task->state & K_TASK_STATE_SUSPENDED) || task->suspend) && !task->resume);
}

int k_task_set_priority(k_task_t *task, uint32_t priority, uint32_t *p_priority_return)
{
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.work_routine)
    {
	return K_ERR_ILLEGAL_USE;
    }
    
    if (task == K_TASK_SELF)
    {
	task = armv7m_rtos_control.task_self;
    }
	
    return armv7m_svcall_3((uint32_t)&__svc_armv7m_rtos_task_set_priority, (uint32_t)task, (uint32_t)priority, (uint32_t)p_priority_return);
}

int __attribute__((optimize("O3"))) k_task_set_mode(uint32_t mode, uint32_t mask, uint32_t *p_mode_return)
{
    k_task_t *task;
    
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.work_routine)
    {
	return K_ERR_ILLEGAL_USE;
    }
    
    task = armv7m_rtos_control.task_self;
    
    if (p_mode_return)
    {
	*p_mode_return = task->mode;
    }

    if (mask & K_MODE_NOPREEMPT)
    {
	if (mode & K_MODE_NOPREEMPT)
	{
	    task->mode |= K_TASK_MODE_NOPREEMPT;
	}
	else
	{
	    task->mode &= ~K_TASK_MODE_NOPREEMPT;

	    if (task != armv7m_rtos_control.task_ready)
	    {
		armv7m_svcall_0((uint32_t)&__svc_armv7m_rtos_task_schedule);
	    }
	}
    }

    if (mask & K_MODE_NOCONCURRENT)
    {
	if (mode & K_MODE_NOCONCURRENT)
	{
	    task->mode |= K_TASK_MODE_NOCONCURRENT;
	}
	else
	{
	    task->mode &= ~K_TASK_MODE_NOCONCURRENT;

	    if (armv7m_rtos_control.work_head)
	    {
		armv7m_svcall_0((uint32_t)&__svc_armv7m_rtos_work_schedule);
	    }
	    else
	    {
		if (task != armv7m_rtos_control.task_ready)
		{
		    armv7m_svcall_0((uint32_t)&__svc_armv7m_rtos_task_schedule);
		}
	    }
	}
    }
    
    if (mask & K_MODE_NOTERMINATE)
    {
	if (mode & K_MODE_NOTERMINATE)
	{
	    task->mode |= K_TASK_MODE_NOTERMINATE;
	}
	else
	{
	    task->mode &= ~K_TASK_MODE_NOTERMINATE;
	}
    }
    
    return K_NO_ERROR;
}

int k_task_delay(uint32_t delay)
{
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.work_routine)
    {
	return K_ERR_ILLEGAL_USE;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_task_delay, (uint32_t)delay);
}

int k_event_send(k_task_t *task, uint32_t events)
{
    if (armv7m_core_is_in_thread())
    {
	if (task == K_TASK_SELF)
	{
	    if (armv7m_rtos_control.work_routine)
	    {
		return K_ERR_ILLEGAL_USE;
	    }
	    
	    task = armv7m_rtos_control.task_self;
	}

	return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_event_send, (uint32_t)task, (uint32_t)events);
    }

    return __svc_armv7m_rtos_event_send(task, events);
}

int k_event_receive(uint32_t events, uint32_t mode, uint32_t timeout, uint32_t *p_events_return)
{
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.work_routine)
    {
	return K_ERR_ILLEGAL_USE;
    }

    return armv7m_svcall_4((uint32_t)&__svc_armv7m_rtos_event_receive, (uint32_t)events, (uint32_t)mode, (uint32_t)timeout, (uint32_t)p_events_return);
}

int k_sem_create(k_sem_t *sem, uint32_t count)
{
    sem->count = count;
    sem->waiting.head = NULL;
    sem->waiting.tail = NULL;
    sem->waiting.release = NULL;

    return K_NO_ERROR;
}

int k_sem_destroy(k_sem_t *sem)
{
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.work_routine)
    {
	return K_ERR_ILLEGAL_USE;
    }

    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_sem_destroy, (uint32_t)sem);
}

int k_sem_acquire(k_sem_t *sem, uint32_t timeout)
{
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.work_routine)
    {
	return K_ERR_ILLEGAL_USE;
    }

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_sem_acquire, (uint32_t)sem, (uint32_t)timeout);
}

int k_sem_release(k_sem_t *sem)
{
    if (armv7m_core_is_in_thread())
    {
	return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_sem_release, (uint32_t)sem);
    }

    return __svc_armv7m_rtos_sem_release(sem);
}

int k_mutex_create(k_mutex_t *mutex)
{
    mutex->next = mutex;
    mutex->previous = mutex;
    mutex->priority = K_PRIORITY_MIN;
    mutex->level = 0;
    mutex->owner = NULL;
    mutex->waiting = NULL;

    return K_NO_ERROR;
}

int k_mutex_destroy(k_mutex_t *mutex)
{
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.work_routine)
    {
	return K_ERR_ILLEGAL_USE;
    }
    
    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_mutex_destroy, (uint32_t)mutex);
}

int __attribute__((optimize("O3"))) k_mutex_trylock(k_mutex_t *mutex)
{
    k_task_t *task;
    uint32_t mode;
    
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.work_routine)
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (!mutex)
    {
	return K_ERR_INVALID_OBJECT;
    }
    
    task = armv7m_rtos_control.task_self;

    if (mutex->owner == task)
    {
	if (mutex->level == 0xffff)
	{
	    return K_ERR_INVALID_STATE;
	}
	
	mutex->level++;

	return K_NO_ERROR;
    }

    if (!task->mutex)
    {
	mode = task->mode;
	
	task->mode = mode | K_TASK_MODE_NODISPATCH;

	/* Ok, if we get here, then we cannot be preempted or destroyed.
	 * So do a quick check whether the mutex can be locked, or not.
	 */
	
	if (!mutex->owner)
	{
	    mutex->owner = task;
	    
	    task->mutex = mutex;
	    
	    task->mode = mode;
	    
	    if (task != armv7m_rtos_control.task_ready)
	    {
		armv7m_svcall_0((uint32_t)&__svc_armv7m_rtos_task_schedule);
	    }
	    
	    return K_NO_ERROR;
	}
	else
	{
	    task->mode = mode;
	    
	    if (task != armv7m_rtos_control.task_ready)
	    {
		armv7m_svcall_0((uint32_t)&__svc_armv7m_rtos_task_schedule);
	    }
	    
	    return K_ERR_UNSATISFIED;
	}

	task->mode = mode;
    }
	
    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_mutex_trylock, (uint32_t)mutex);
}

int __attribute__((optimize("O3"))) k_mutex_lock(k_mutex_t *mutex)
{
    k_task_t *task;
    uint32_t mode;
    
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.work_routine)
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (!mutex)
    {
	return K_ERR_INVALID_OBJECT;
    }
    
    task = armv7m_rtos_control.task_self;

    if (mutex->owner == task)
    {
	if (mutex->level == 0xffff)
	{
	    return K_ERR_INVALID_STATE;
	}

	mutex->level++;

	return K_NO_ERROR;
    }

    if (!task->mutex)
    {
	mode = task->mode;
	
	task->mode = mode | K_TASK_MODE_NODISPATCH;

	/* Ok, if we get here, then we cannot be preempted or destroyed.
	 * So do a quick check whether the mutex can be locked, or not.
	 * We also know that there had not been a mutex locked before.
	 */
	
	if (!mutex->owner)
	{
	    mutex->owner = task;
	    
	    task->mutex = mutex;
	    
	    task->mode = mode;
	    
	    if (task != armv7m_rtos_control.task_ready)
	    {
		armv7m_svcall_0((uint32_t)&__svc_armv7m_rtos_task_schedule);
	    }
	    
	    return K_NO_ERROR;
	}

	task->mode = mode;
    }
	
    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_mutex_lock, (uint32_t)mutex);
}

int __attribute__((optimize("O3"))) k_mutex_unlock(k_mutex_t *mutex)
{
    k_task_t *task;
    uint32_t mode;
    
    if (!armv7m_core_is_in_thread())
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (armv7m_rtos_control.work_routine)
    {
	return K_ERR_ILLEGAL_USE;
    }

    if (!mutex)
    {
	return K_ERR_INVALID_OBJECT;
    }
    
    task = armv7m_rtos_control.task_self;

    if (mutex->owner != task)
    {
	return K_ERR_NOT_OWNER_OF_MUTEX;
    }

    if (mutex->level)
    {
	mutex->level--;
	
	return K_NO_ERROR;
    }

    if (mutex->next == mutex)
    {
	mode = task->mode;
	
	task->mode = mode | K_TASK_MODE_NODISPATCH;

	/* Ok, if we get here, then we cannot be preempted or destroyed.
	 * So do a quick check whether the mutex can unlocked without
	 * waiting tasks. We also know that this is the only mutex the task
	 * had locked.
	 */

	if (!mutex->waiting)
	{
	    mutex->owner = NULL;
	    
	    task->mutex = NULL;
	    
	    task->mode = mode;
	    
	    if (task != armv7m_rtos_control.task_ready)
	    {
		armv7m_svcall_0((uint32_t)&__svc_armv7m_rtos_task_schedule);
	    }
	    
	    return K_NO_ERROR;
	}
	
	task->mode = mode;
    }
    
    return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_mutex_unlock, (uint32_t)mutex);
}

void k_work_create(k_work_t *work, k_work_routine_t routine, void *context)
{
    work->next = NULL;
    work->routine = routine;
    work->context = context;
}

int k_work_destroy(k_work_t *work)
{
    if (armv7m_core_is_in_thread())
    {
	return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_work_destroy, (uint32_t)work);
    }

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
	return __svc_armv7m_rtos_work_destroy(work);
    }
    
    return K_ERR_ILLEGAL_USE;
}

bool __attribute__((optimize("O3"))) k_work_submit(k_work_t *work)
{
    k_work_t *work_submit;

    if (__armv7m_atomic_cas((volatile uint32_t*)&work->next, (uint32_t)NULL, (uint32_t)K_WORK_SENTINEL) != (uint32_t)NULL)
    {
	return false;
    }

    if (armv7m_core_is_in_thread())
    {
	return armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_work_submit, (uint32_t)work);
    }

    work_submit = (k_work_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.work_submit, (uint32_t)work);

    armv7m_atomic_store((volatile uint32_t*)&work->next, (uint32_t)work_submit);
    
    if (work_submit == K_WORK_SENTINEL)
    {
	if (armv7m_core_is_in_pendsv_or_svcall())
	{
	    armv7m_rtos_work_schedule();
	}
	else
	{
	    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_WORK_SCHEDULE);
	}
    }
    
    return true;
}

bool k_work_is_in_progress(void)
{
    return (armv7m_rtos_control.work_routine != NULL);
}

/******************************************************************************************************************************/

void RTOS_TASK_SCHEDULE_SWIHandler(void)
{
    armv7m_rtos_task_schedule();
}

void RTOS_WORK_SCHEDULE_SWIHandler(void)
{
    armv7m_rtos_work_schedule();
}

/******************************************************************************************************************************/
