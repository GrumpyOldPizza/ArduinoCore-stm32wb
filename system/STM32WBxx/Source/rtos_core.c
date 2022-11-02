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

extern uint32_t __HeapBase[];
extern uint32_t __HeapLimit[];
extern uint32_t __StackTop[];
extern uint32_t __StackLimit[];

armv7m_rtos_control_t armv7m_rtos_control =
{
    .task_self = NULL,
    .task_next = NULL,
    .task_ready = NULL,
    .task_default = NULL,
    .task_link = K_TASK_SENTINEL,
    .task_resume = K_TASK_SENTINEL,
    .task_wakeup = K_TASK_SENTINEL,
    .task_timeout = NULL,
    .task_clock = 0,
    .task_timer = STM32WB_LPTIM_TIMEOUT_INIT(),
    .event_send = K_EVENT_SENTINEL,
    .sem_release = K_SEM_SENTINEL,
    .work_self = NULL,
    .work_head = K_WORK_SENTINEL,
    .work_tail = K_WORK_SENTINEL,
    .work_submit = K_WORK_SENTINEL,
    .alarm_queue = NULL,
    .alarm_modify = K_ALARM_SENTINEL,
    .alarm_clock = 0,
    .alarm_timer = STM32WB_LPTIM_TIMEOUT_INIT(),
    .system_state = K_STATE_INACTIVE,
    .system_policy = K_POLICY_SLEEP,
    .heap_current = (void*)&__HeapBase[0]
};
  
static k_task_t armv7m_rtos_task_default;

typedef struct _k_task_params_t {
    const char                  *name;
    k_task_routine_t            routine;
    void                        *context;
    uint32_t                    priority;
    void                        *stack_base;
    uint32_t                    stack_size;
    uint32_t                    options;
} k_task_params_t;

static void armv7m_rtos_system_initialize(const k_hook_table_t *hook_table);
static void armv7m_rtos_system_start(k_task_routine_t routine, void *context) __attribute__((noreturn));

static void armv7m_rtos_task_timeout_insert(k_task_t *task, uint64_t clock);
static void armv7m_rtos_task_timeout_callback(void *context);

static void * __svc_armv7m_rtos_heap_allocate(uint32_t size);

static void __svc_armv7m_rtos_system_start(k_task_routine_t routine, void *context);
static bool __svc_armv7m_rtos_system_lock(void);
static void __svc_armv7m_rtos_system_unlock(void);
static int __svc_armv7m_rtos_system_set_policy(uint32_t policy, uint32_t *p_policy_return);

       void __svc_armv7m_rtos_task_exit(void);

/******************************************************************************************************************************/

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)

/*
 * RTT/SYSVIEW notes:
 *
 * - task_run(NULL) means switch to idle
 * - work_enter(work) means a switch to the WORK task (first one), work_exit() means switch back to current task 
 *
 * name/priority are just used for the GUI, to associate the "task" address with something readable, and to 
 * order things on screen.
 *
 * "stack_base" is displayed in HEX, while "stack_size" is displayed in dedcimal. So "stack_size" could be the total
 * size, or the used size, or the free size. So ideally it would be the maximum used size.
 *
 * So somewhere in the idle routine we'd want to go throu the list of stacks and check.
 */

static void armv7m_rtos_hook_task_create(k_task_t *task, const char *name, uint32_t priority, void *stack_base, uint32_t stack_size)
{
} 

static void armv7m_rtos_hook_task_destroy(k_task_t *task)
{
}

static void armv7m_rtos_hook_task_exit(void)
{
}

static void armv7m_rtos_hook_task_terminate(k_task_t *task)
{
}

static void armv7m_rtos_hook_task_block(k_task_t *task, uint32_t cause)
{
}

static void armv7m_rtos_hook_task_ready(k_task_t *task)
{
}

static void armv7m_rtos_hook_task_run(k_task_t *task)
{
}

static const k_hook_table_t armv7m_rtos_hook_table =
{
    .task_create    = armv7m_rtos_hook_task_create,
    .task_destroy   = armv7m_rtos_hook_task_destroy,
    .task_exit      = armv7m_rtos_hook_task_exit,      
    .task_terminate = armv7m_rtos_hook_task_terminate,
    .task_block     = armv7m_rtos_hook_task_block,
    .task_ready     = armv7m_rtos_hook_task_ready,
    .task_run       = armv7m_rtos_hook_task_run,
};

#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */


void __attribute__((naked, noreturn)) armv7m_rtos_pendsv_epilogue(void)
{
    __asm__(
        "1: movw     r3, #:lower16:armv7m_rtos_control         \n"
        "   movt     r3, #:upper16:armv7m_rtos_control         \n"
        "   ldr      r0, [r3, %[offset_CONTROL_WORK_SELF]]     \n"
        "   cbz.n    r0, 4f                                    \n"
        "   push     { r7, lr }                                \n"
        "   ldr      r1, [r0, %[offset_WORK_ROUTINE]]          \n"
        "   bic      r1, r1, #1                                \n"
        "   ldr      r2, [r0, %[offset_WORK_CONTEXT]]          \n"
        "   mov      r3, #0x01000000                           \n"
        "   mov      lr, #0xfffffff9                           \n"
        "   adr      r7, 2f                                    \n"
        "   orr      r7, r7, #1                                \n"
        "   sub      sp, #0x20                                 \n"
        "   str      r2, [sp, #0x00]                           \n"
        "   str      r7, [sp, #0x14]                           \n"
        "   str      r1, [sp, #0x18]                           \n"
        "   str      r3, [sp, #0x1c]                           \n"
        "   dsb                                                \n"
        "   bx       lr                                        \n"
        "   .align 2                                           \n"
        "2: cpsie    i                                         \n"
        "   adr      r7, 3f                                    \n"
        "   add      r7, #1                                    \n"
#if (__FPU_PRESENT == 1)
        "   mrs      r0, CONTROL                               \n"
        "   bic      r0, #0x04                                 \n" // clear FPCA 
        "   msr      CONTROL, r0                               \n"
#endif /* __FPU_PRESENT == 1 */
        "   svc      0                                         \n"
        "   .align 2                                           \n"
        "3: add      sp, #0x28                                 \n" // 32 bytes for the exception stack, 8 bytes for SVCALL frame
        "   bl       armv7m_rtos_work_return                   \n"
        "   pop      { r7, lr }                                \n"
        "   b.n      1b                                        \n"
        "   .align 2                                           \n"
        "4: ldr      r0, [r3, %[offset_CONTROL_TASK_NEXT]]     \n"
        "   ldr      r1, [r3, %[offset_CONTROL_TASK_SELF]]     \n"
        "   cmp      r0, r1                                    \n"
        "   bne.n    5f                                        \n"
        "   dsb                                                \n"
        "   bx       lr                                        \n"
        "5: cmp      r1, %[const_TASK_TERMINATED]              \n"
        "   bls.n    6f                                        \n"
        "   tst      lr, #0x00000004                           \n" // bit 2 is SPSEL
        "   itte     eq                                        \n"
        "   moveq    r2, sp                                    \n"
#if (__FPU_PRESENT == 1)
        "   subeq    sp, sp, #0x60                             \n"
#else  /* __FPU_PRESENT == 1 */
        "   subeq    sp, sp, #0x20                             \n"
#endif /* __FPU_PRESENT == 1 */
        "   mrsne    r2, PSP                                   \n"
        "   stmdb    r2!, { r4-r11 }                           \n"
#if (__FPU_PRESENT == 1)
        "   tst      lr, #0x00000010                           \n" // bit 4 is nFPCA
        "   it       eq                                        \n"
        "   vstmdbeq r2!, { d8-d15 }                           \n"
#endif /* __FPU_PRESENT == 1 */
        "   tst      lr, #0x00000004                           \n" // bit 2 is SPSEL
        "   it       eq                                        \n"
        "   moveq    sp, r2                                    \n"
        "   str      lr, [r1, %[offset_TASK_EXC_RETURN]]       \n"
        "   str      r2, [r1, %[offset_TASK_STACK_TOP]]        \n"
        "6: str      r0, [r3, %[offset_CONTROL_TASK_SELF]]     \n"
        "   cbz.n    r0, 7f                                    \n"
        "   ldr      lr, [r0, %[offset_TASK_EXC_RETURN]]       \n"
        "   ldr      r2, [r0, %[offset_TASK_STACK_TOP]]        \n"
#if (__FPU_PRESENT == 1)
        "   tst      lr, #0x00000010                           \n" // bit 4 is nFPCA
        "   it       eq                                        \n"
        "   vldmiaeq r2!, { d8-d15 }                           \n"
#endif /* __FPU_PRESENT == 1 */
        "   ldmia    r2!, { r4-r11 }                           \n"
        "   tst      lr, #0x00000004                           \n" // bit 2 is SPSEL
        "   ite      eq                                        \n"
        "   moveq    sp, r2                                    \n"
        "   msrne    PSP, r2                                   \n"
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
        "   ldr      r3, [r3, %[offset_CONTROL_HOOK_TABLE]]    \n"
        "   ldr      r3, [r3, %[offset_HOOK_TABLE_TASK_RUN]]   \n"
        "   bx       r3                                        \n"
#else /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
        "   dsb                                                \n"
        "   bx       lr                                        \n"
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
        "   .align 2                                           \n"
        "7: movw     r1, #:lower16:armv7m_rtos_idle_routine    \n"
        "   movt     r1, #:upper16:armv7m_rtos_idle_routine    \n"
        "   mov      r2, #0x01000000                           \n"
        "   mov      lr, #0xfffffff9                           \n"
        "   sub      sp, #0x20                                 \n"
        "   str      r1, [sp, #0x18]                           \n"
        "   str      r2, [sp, #0x1c]                           \n"
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
        "   ldr      r3, [r3, %[offset_CONTROL_HOOK_TABLE]]    \n"
        "   ldr      r3, [r3, %[offset_HOOK_TABLE_TASK_RUN]]   \n"
        "   bx       r3                                        \n"
#else /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
        "   dsb                                                \n"
        "   bx       lr                                        \n"
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
        :
        : [offset_CONTROL_WORK_SELF]        "I" (offsetof(armv7m_rtos_control_t, work_self)),
          [offset_CONTROL_TASK_SELF]        "I" (offsetof(armv7m_rtos_control_t, task_self)),
          [offset_CONTROL_TASK_NEXT]        "I" (offsetof(armv7m_rtos_control_t, task_next)),
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
          [offset_CONTROL_HOOK_TABLE]       "I" (offsetof(armv7m_rtos_control_t, hook_table)),
          [offset_HOOK_TABLE_TASK_RUN]      "I" (offsetof(k_hook_table_t, task_run)),
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
          [offset_TASK_EXC_RETURN]          "I" (offsetof(k_task_t, exc_return)),
          [offset_TASK_STACK_TOP]           "I" (offsetof(k_task_t, stack_top)),
          [offset_TASK_STACK_LIMIT]         "I" (offsetof(k_task_t, stack_limit)),
          [offset_WORK_ROUTINE]             "I" (offsetof(k_work_t, routine)),
          [offset_WORK_CONTEXT]             "I" (offsetof(k_work_t, context)),
          [const_TASK_TERMINATED]           "I" (K_TASK_TERMINATED),
          [const_WORK_SENTINEL]             "I" (K_WORK_SENTINEL)
        );
}

static void __attribute__((naked, used, noreturn)) armv7m_rtos_idle_routine(void)
{
    __asm__(
        "   movw     r3, #:lower16:armv7m_rtos_control         \n"
        "   movt     r3, #:upper16:armv7m_rtos_control         \n"
        "   ldrb     r0, [r3, %[offset_CONTROL_SYSTEM_POLICY]] \n"
        "   bl       stm32wb_system_sleep                      \n"
        "   adr      r7, 1f                                    \n"
        "   add      r7, #1                                    \n"
        "   svc      0                                         \n"
        "   .align 2                                           \n"
        "1: add      sp, #0x28                                 \n" // 32 bytes for the exception stack, 8 bytes for SVCALL frame
        "   movw     r3, #:lower16:armv7m_rtos_control         \n"
        "   movt     r3, #:upper16:armv7m_rtos_control         \n"
        "   ldr      r0, [r3, %[offset_CONTROL_TASK_READY]]    \n"
        "   str      r0, [r3, %[offset_CONTROL_TASK_NEXT]]     \n"        
        "   mov      r0, %[const_TASK_TERMINATED]              \n"
        "   str      r0, [r3, %[offset_CONTROL_TASK_SELF]]     \n"        
        "   b        armv7m_rtos_pendsv_epilogue               \n"
        :
        : [offset_CONTROL_SYSTEM_POLICY] "I" (offsetof(armv7m_rtos_control_t, system_policy)),
          [offset_CONTROL_TASK_SELF]     "I" (offsetof(armv7m_rtos_control_t, task_self)),
          [offset_CONTROL_TASK_NEXT]     "I" (offsetof(armv7m_rtos_control_t, task_next)),
          [offset_CONTROL_TASK_READY]    "I" (offsetof(armv7m_rtos_control_t, task_ready)),
          [const_TASK_TERMINATED]        "I" (K_TASK_TERMINATED)
        );
}

static void armv7m_rtos_system_initialize(const k_hook_table_t *hook_table)
{
    k_task_t *task;
    void *stack_base, *stack_limit;
    
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    armv7m_rtos_control.hook_table = hook_table ? hook_table : &armv7m_rtos_hook_table;
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */

    stack_base = (void*)&__StackTop[0];
    stack_limit = (void*)(((uint32_t)armv7m_rtos_control.heap_current + 31) & ~31);
    
    if (stack_limit < (void*)&__HeapLimit[0])
    {
        stack_limit = (void*)&__HeapLimit[0];
    }
    
    stack_limit += 32;
        
#if (ARMV7M_RTOS_MPU_SUPPORTED == 1)
    MPU->RBAR = (((uint32_t)stack_limit - 32) & 0xffffff00) | MPU_RBAR_VALID_Msk | (7 << MPU_RBAR_REGION_Pos);
    MPU->RASR = (MPU_RASR_SRD_Msk ^ ((1 << MPU_RASR_SRD_Pos) << ((((uint32_t)stack_limit - 32) & 0x000000ff) >> 5))) | (7 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;
    MPU->CTRL = (MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_HFNMIENA_Msk | MPU_CTRL_ENABLE_Msk);
    
    __DSB();
    __ISB();
#endif /* ARMV7M_RTOS_MPU_SUPPORTED == 1 */
        
    task = &armv7m_rtos_task_default;
    
    armv7m_rtos_control.task_default = task;
        
    armv7m_rtos_task_create(task, "DEFAULT", K_PRIORITY_MIN, stack_limit, (stack_base - stack_limit), K_TASK_JOINABLE);
    
    armv7m_rtos_control.system_state = K_STATE_READY;
}

static void __attribute__((naked, noreturn)) armv7m_rtos_system_start(k_task_routine_t routine, void *context)
{
    __asm__(
        "   cpsie    i                                         \n"
        "   adr      r7, 1f                                    \n"
        "   add      r7, #1                                    \n"
#if (__FPU_PRESENT == 1)
        "   mrs      r2, CONTROL                               \n"
        "   bic      r2, #0x04                                 \n" // clear FPCA
        "   msr      CONTROL, r2                               \n"
#endif /* __FPU_PRESENT == 1 */
        "   svc      0                                         \n"
        "   .align 2                                           \n"
        "1: sub      sp, #0x18                                 \n" // 0x20 from exception, 0x08 from SVCALL frame, so 0x18 needed
        "   bl       __svc_armv7m_rtos_system_start            \n"
        "   movw     r3, #:lower16:armv7m_rtos_control         \n"
        "   movt     r3, #:upper16:armv7m_rtos_control         \n"
        "   ldr      r0, [r3, %[offset_CONTROL_TASK_DEFAULT]]  \n"
        "   ldr      r2, [r0, %[offset_TASK_STACK_TOP]]        \n"
        "   mov      sp, r2                                    \n"
        "   b        armv7m_rtos_pendsv_epilogue               \n"
        :
        : [offset_CONTROL_TASK_DEFAULT]     "I" (offsetof(armv7m_rtos_control_t, task_default)),
          [offset_TASK_STACK_TOP]           "I" (offsetof(k_task_t, stack_top))
        );
}

void __attribute__((naked, noreturn)) armv7m_rtos_task_exit(void)
{
    __asm__(
        "   cpsie    i                                         \n"
        "   adr      r7, 1f                                    \n"
        "   add      r7, #1                                    \n"
#if (__FPU_PRESENT == 1)
        "   mrs      r2, CONTROL                               \n"
        "   bic      r2, #0x04                                 \n" // clear FPCA
        "   msr      CONTROL, r2                               \n"
#endif /* __FPU_PRESENT == 1 */
        "   svc      0                                         \n"
        "   .align 2                                           \n"
        "1: add      sp, #0x08                                 \n" // 8 bytes for SVCALL frame
        "   bl       __svc_armv7m_rtos_task_exit               \n"
        "   b        armv7m_rtos_pendsv_epilogue               \n"
        :
        :
        );
}

static void armv7m_rtos_task_status(k_task_t *task, uint32_t status)
{
#if (__FPU_PRESENT == 1)
    if (!(task->exc_return & 0x00000010))
    {
        ((armv7m_context_fpu_t*)task->stack_top)->r0 = status;
    }
    else
#endif /* __FPU_PRESENT == 1 */
    {
        ((armv7m_context_t*)task->stack_top)->r0 = status;
    }
}

/******************************************************************************************************************************/

void __attribute__((optimize("O3"), noinline)) armv7m_rtos_task_queue_insert(k_task_t **p_task_head, k_task_t *task)
{
    k_task_t *element;

    if (*p_task_head == NULL)
    {
        *p_task_head = task;

        task->next = task;
        task->previous = task;
    }
    else
    {
        element = *p_task_head;

        do
        {
            if (task->priority < element->priority)
            {
                if (*p_task_head == element)
                {
                    *p_task_head = task;
                }
                
                break;
            }

            element = element->next;
        }
        while (*p_task_head != element);

        task->next = element;
        task->previous = element->previous;
        
        task->previous->next = task;
        task->next->previous = task;
    }
}

void __attribute__((optimize("O3"), noinline)) armv7m_rtos_task_queue_remove(k_task_t **p_task_head, k_task_t *task)
{
    if (task->next == task)
    {
        *p_task_head = NULL;
    }
    else
    {
        if (*p_task_head == task)
        {
            *p_task_head = task->next;
        }
        
        task->next->previous = task->previous;
        task->previous->next = task->next;
    }

    task->next = NULL;
    task->previous = NULL;
}

void armv7m_rtos_task_ready_insert(k_task_t *task)
{
    task->state |= K_TASK_STATE_READY;

    armv7m_rtos_task_queue_insert(&armv7m_rtos_control.task_ready, task);
}

void armv7m_rtos_task_ready_remove(k_task_t *task)
{
    task->state &= ~K_TASK_STATE_READY;

    armv7m_rtos_task_queue_remove(&armv7m_rtos_control.task_ready, task);
}

void armv7m_rtos_task_start(k_task_t *task, k_task_routine_t routine, void *context)
{
    armv7m_context_t *stack;
    
    stack = (armv7m_context_t*)((uint32_t)task->stack_top - sizeof(armv7m_context_t));
    stack->r0 = (uint32_t)context;
#if (ARMV7M_RTOS_DEBUG_SUPPORTED == 1)
    stack->r1 = 0x00000000;
    stack->r2 = 0x00000000;
    stack->r3 = 0x00000000;
    stack->r4 = 0x00000000;
    stack->r5 = 0x00000000;
    stack->r6 = 0x00000000;
    stack->r7 = 0x00000000;
    stack->r8 = 0x00000000;
    stack->r9 = 0x00000000;
    stack->r10 = 0x00000000;
    stack->r11 = 0x00000000;
#endif /* (ARMV7M_RTOS_DEBUG_SUPPORTED == 1) */
    stack->lr = (uint32_t)armv7m_rtos_task_exit;
    stack->pc = (uint32_t)routine & ~1;
    stack->xpsr = 0x01000000;
    
    task->exc_return = ((task == armv7m_rtos_control.task_default) ? 0xfffffff9 : 0xfffffffd);
    task->stack_top = (void*)stack;
    task->stack_end = (void*)stack;

    if (!(task->state & K_TASK_STATE_SUSPENDED))
    {
        armv7m_rtos_task_ready_insert(task);
      
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
        (*armv7m_rtos_control.hook_table->task_ready)(task);
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
    }
}

void armv7m_rtos_task_create(k_task_t *task, const char *name, uint32_t priority, void *stack_base, uint32_t stack_size, uint32_t options)
{
    uint32_t *stack, *stack_e;

    task->next = NULL;
    task->previous = NULL;
    task->priority = priority;
    task->bpriority = priority;
    task->state = 0;
    task->events = 0;
    task->name = name;
    task->link = armv7m_rtos_control.task_link;
    task->exc_return = 0;
    task->stack_top = stack_base + stack_size;
    task->stack_end = stack_base + stack_size;
    task->stack_base = stack_base + stack_size;
    task->stack_limit = stack_base;
    task->join = NULL;
    task->mutex = NULL;
    task->resume = NULL;
    task->wakeup = NULL;
    task->timeout.next = NULL;
    task->timeout.previous = NULL;
    task->timeout.clock_l = 0;
    task->timeout.clock_h = 0;
    
    armv7m_rtos_control.task_link = task;

    if (options & K_TASK_JOINABLE)
    {
        task->state |= K_TASK_STATE_JOINABLE;
    }

    if (options & K_TASK_SUSPENDED)
    {
        task->state |= K_TASK_STATE_SUSPENDED;
    }
    
    if (task == armv7m_rtos_control.task_default)
    {
        stack_e = (uint32_t*)((uint32_t)__builtin_frame_address(0) & ~31) - 32;
    }
    else
    {
        stack_e = (uint32_t*)task->stack_end;
    }
    
    for (stack = (uint32_t*)task->stack_limit; stack < stack_e; stack += 2)
    {
        stack[0] = 0xaaaaaaaa;
        stack[1] = 0x77777777;
    }

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_create)(task, task->name, task->priority, task->stack_limit, (task->stack_base - task->stack_limit));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */

    if (task->state & K_TASK_STATE_SUSPENDED)
    {
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
        (*armv7m_rtos_control.hook_table->task_block)(task, K_TASK_STATE_SUSPENDED);
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
    }
}

void armv7m_rtos_task_destroy(k_task_t *task)
{
    k_task_t **p_task_previous, *task_link;
    
    for (p_task_previous = &armv7m_rtos_control.task_link, task_link = armv7m_rtos_control.task_link; task_link != K_TASK_SENTINEL; p_task_previous = &task_link->link, task_link = task_link->link)
    {
        if (task_link == task)
        {
            *p_task_previous = task->link;

            break;
        }
    }

    task->state = 0;
    task->link = NULL;

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_destroy)(task);
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
}

void armv7m_rtos_task_release(k_task_t *task)
{
    if (task->state & K_TASK_STATE_TIMEOUT)
    {
        armv7m_rtos_task_timeout_remove(task);
    }
    
    task->state &= ~(K_TASK_STATE_EVENT_ALL | K_TASK_STATE_EVENT_CLEAR | K_TASK_STATE_WAIT_MASK);

    if (!(task->state & K_TASK_STATE_TERMINATED))
    {
        if (!(task->state & K_TASK_STATE_SUSPENDED))
        {
            armv7m_rtos_task_ready_insert(task);
            
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
            (*armv7m_rtos_control.hook_table->task_ready)(task);
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
        }
    }
}

void armv7m_rtos_task_unblock(k_task_t *task, uint32_t status)
{
    armv7m_rtos_task_status(task, status);

    switch (task->state & K_TASK_STATE_WAIT_MASK) {
    case K_TASK_STATE_WAIT_NONE:
        break;

    case K_TASK_STATE_WAIT_JOIN:
        armv7m_rtos_join_unblock(task);
        break;

    case K_TASK_STATE_WAIT_DELAY:
        armv7m_rtos_delay_unblock(task);
        break;

    case K_TASK_STATE_WAIT_SLEEP:
        armv7m_rtos_sleep_unblock(task);
        break;
            
    case K_TASK_STATE_WAIT_EVENT:
        armv7m_rtos_event_unblock(task);
        break;
            
    case K_TASK_STATE_WAIT_SEM:
        armv7m_rtos_sem_unblock(task);
        break;
            
    case K_TASK_STATE_WAIT_MUTEX:
        armv7m_rtos_mutex_unblock(task);
        break;
            
    default:
        break;
    }

    armv7m_rtos_task_release(task);
}

void armv7m_rtos_task_priority(k_task_t *task)
{
    k_task_t *task_next;
    k_mutex_t *mutex;
    uint32_t priority;

    do
    {
        task_next = NULL;

        priority = task->bpriority;

        mutex = task->mutex;

        if (mutex)
        {
            if (mutex->options & (K_MUTEX_PRIORITY_INHERIT | K_MUTEX_PRIORITY_PROTECT))
            {
                if (priority > mutex->priority)
                {
                    priority = mutex->priority;
                }
            }
        }
    
        if (task->priority != priority)
        {
            task->priority = priority;

            if (task->state & K_TASK_STATE_READY)
            {
                armv7m_rtos_task_ready_remove(task);
                armv7m_rtos_task_ready_insert(task);
            }

            task_next = armv7m_rtos_mutex_priority(task);
        }

        task = task_next;
    }
    while (task);
}

void armv7m_rtos_task_schedule(void)
{
    if (armv7m_rtos_control.system_state == K_STATE_RUNNING)
    {
        if (armv7m_rtos_control.task_next != armv7m_rtos_control.task_ready)
        {
            /* Do not switch away from the idle_routine. That is handled within the idle
             * reoutine.
             */

            if (armv7m_rtos_control.task_self)
            {
                armv7m_rtos_control.task_next = armv7m_rtos_control.task_ready;
                
                if (armv7m_rtos_control.task_self != armv7m_rtos_control.task_next)
                {
                    if (!armv7m_rtos_control.work_self)
                    {
                        armv7m_pendsv_hook(armv7m_rtos_pendsv_epilogue);
                    }
                }
            }
            else
            {
                stm32wb_system_wakeup();
            }
        }
    }
}

static void armv7m_rtos_task_timeout_insert(k_task_t *task, uint64_t clock)
{
    k_task_t *task_timeout;
    uint64_t timeout_clock;

    if (armv7m_rtos_control.task_timeout == NULL)
    {
        armv7m_rtos_control.task_timeout = task;

	task->timeout.next = task;
	task->timeout.previous = task;
    }
    else
    {
	task_timeout = armv7m_rtos_control.task_timeout;

	do
	{
            timeout_clock = (((uint64_t)task_timeout->timeout.clock_l << 0) | ((uint64_t)task_timeout->timeout.clock_h << 32));

            if (clock < timeout_clock)
            {
                if (task_timeout == armv7m_rtos_control.task_timeout)
                {
                    armv7m_rtos_control.task_timeout = task;
                }
                break;
            }
            
	    task_timeout = task_timeout->timeout.next;
	}
	while (task_timeout != armv7m_rtos_control.task_timeout);
        
        task->timeout.previous = task_timeout->timeout.previous;
        task->timeout.next = task_timeout;
            
        task->timeout.previous->timeout.next = task;
        task->timeout.next->timeout.previous = task;
    }

    task->timeout.clock_l = (uint32_t)(clock >> 0);
    task->timeout.clock_h = (uint32_t)(clock >> 32);

    task->state |= K_TASK_STATE_TIMEOUT;
}

void armv7m_rtos_task_timeout_remove(k_task_t *task)
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

        task->timeout.next->timeout.previous = task->timeout.previous;
        task->timeout.previous->timeout.next = task->timeout.next;
    }

    task->timeout.next = NULL;
    task->timeout.previous = NULL;
}

void armv7m_rtos_task_timeout_absolute(k_task_t *task, uint64_t clock)
{
    uint32_t seconds, millis;

    seconds = clock / 1000;
    millis = clock - seconds * 1000;

    clock = (((uint64_t)seconds * (uint64_t)STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND) + ((millis * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND + 999) / 1000));

    armv7m_rtos_task_timeout_insert(task, clock);

    armv7m_rtos_task_timeout_schedule();
}

static void armv7m_rtos_task_timeout_callback(void *context)
{
    k_task_t *task;
    uint64_t clock, reference;

    task = armv7m_rtos_control.task_timeout;

    if (task)
    {
        reference = stm32wb_lptim_timeout_clock();

	do
	{
	    clock = (((uint64_t)task->timeout.clock_l << 0) | ((uint64_t)task->timeout.clock_h << 32));
		
            if (clock > reference)
            {
                break;
            }

            armv7m_rtos_task_unblock(task, (((task->state & K_TASK_STATE_WAIT_MASK) == K_TASK_STATE_WAIT_DELAY) ? K_NO_ERROR : K_ERR_TIMEOUT));
            
            task = armv7m_rtos_control.task_timeout;
	}
	while (task);

        armv7m_rtos_task_timeout_schedule();
    
        armv7m_rtos_task_schedule();
    }
}

void armv7m_rtos_task_timeout_relative(k_task_t *task, uint32_t delay)
{
    uint64_t clock;
    uint32_t seconds, millis;

    clock = stm32wb_lptim_timeout_clock();

    if (delay <= ((0xffffffff - 999) / STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND))
    {
        clock += ((delay * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND + 999) / 1000);
    }
    else
    {
        seconds = delay / 1000;
        millis = delay - seconds * 1000;

        clock += (((uint64_t)seconds * (uint64_t)STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND) + ((millis * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND + 999) / 1000));
    }
    
    armv7m_rtos_task_timeout_insert(task, clock);

    armv7m_rtos_task_timeout_schedule();
}

void armv7m_rtos_task_timeout_schedule(void)
{
    k_task_t *task;
    uint64_t clock;
    
    task = armv7m_rtos_control.task_timeout;

    if (task)
    {
        clock = (((uint64_t)task->timeout.clock_l << 0) | ((uint64_t)task->timeout.clock_h << 32));

        if (armv7m_rtos_control.task_clock != clock)
        {
            armv7m_rtos_control.task_clock = clock;

            stm32wb_lptim_timeout_absolute(&armv7m_rtos_control.task_timer, clock, armv7m_rtos_task_timeout_callback, NULL);
        }
    }
    else
    {
        if (armv7m_rtos_control.task_clock)
        {
            armv7m_rtos_control.task_clock = 0;

            stm32wb_lptim_timeout_cancel(&armv7m_rtos_control.task_timer);
        }
    }
}

/******************************************************************************************************************************/

static void * __svc_armv7m_rtos_heap_allocate(uint32_t size)
{
    k_task_t *task;
    void *heap_current, *heap_limit, *stack_limit;

    // armv7m_rtt_printf("k_heap_allocate(size=%d)\n", size);
    
    heap_current = armv7m_rtos_control.heap_current;
    heap_limit = &__StackLimit[0];

    if (size > (uint32_t)(heap_limit - heap_current))
    {
        return NULL;
    }

    task = armv7m_rtos_control.task_default;

    if (task)
    {
        stack_limit = (void*)(((uint32_t)(heap_current + size) + 31) & ~31) + 32;

        if (task->stack_limit < stack_limit)
        {
            if ((((uint32_t*)stack_limit)[-8] != 0xaaaaaaaa) ||
                (((uint32_t*)stack_limit)[-7] != 0x77777777) ||
                (((uint32_t*)stack_limit)[-6] != 0xaaaaaaaa) ||
                (((uint32_t*)stack_limit)[-5] != 0x77777777) ||
                (((uint32_t*)stack_limit)[-4] != 0xaaaaaaaa) ||
                (((uint32_t*)stack_limit)[-3] != 0x77777777) ||
                (((uint32_t*)stack_limit)[-2] != 0xaaaaaaaa) ||
                (((uint32_t*)stack_limit)[-1] != 0x77777777))
            {
                return NULL;
            }

            task->stack_limit = stack_limit;
        
#if (ARMV7M_RTOS_MPU_SUPPORTED == 1)
            MPU->RBAR = (((uint32_t)stack_limit - 32) & 0xffffff00) | MPU_RBAR_VALID_Msk | (7 << MPU_RBAR_REGION_Pos);
            MPU->RASR = (MPU_RASR_SRD_Msk ^ ((1 << MPU_RASR_SRD_Pos) << ((((uint32_t)stack_limit - 32) & 0x000000ff) >> 5))) | (7 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;
#endif /* ARMV7M_RTOS_MPU_SUPPORTED == 1 */
        }
    }
    
    armv7m_rtos_control.heap_current += size;

    return heap_current;
}

static void  __attribute__((used)) __svc_armv7m_rtos_system_start(k_task_routine_t routine, void *context)
{
    if (routine)
    {
        armv7m_rtos_task_start(armv7m_rtos_control.task_default, routine, context);
    }
    
    armv7m_rtos_control.system_state = K_STATE_RUNNING;

    /* No special dispatch needed, as this case is handled via armv7m_rtos_system_start.
     * That code on the tail end call armv7m_rtos_pendsv_callback, to avoid cycling 
     * throu PENDSV and possibly fetching a bad exception frame off the stack.
     */

    armv7m_rtos_control.task_self = K_TASK_TERMINATED;
    armv7m_rtos_control.task_next = armv7m_rtos_control.task_ready;

    armv7m_rtos_work_schedule();
}

static bool __svc_armv7m_rtos_system_lock(void)
{
    uint8_t system_state;

    system_state = armv7m_rtos_control.system_state;
    
    if (!armv7m_rtos_control.work_self)
    {
        if (system_state == K_STATE_RUNNING)
        {
            armv7m_rtos_control.system_state = K_STATE_LOCKED;
        }
    }

    return (system_state == K_STATE_LOCKED);
}

static void __svc_armv7m_rtos_system_unlock(void)
{
    if (!armv7m_rtos_control.work_self)
    {
        if (armv7m_rtos_control.system_state == K_STATE_LOCKED)
        {
            armv7m_rtos_control.system_state = K_STATE_RUNNING;
            
            armv7m_rtos_task_schedule();

            armv7m_rtos_work_schedule();
        }
    }
}

static int __svc_armv7m_rtos_system_set_policy(uint32_t policy, uint32_t *p_policy_return)
{
    uint32_t policy_previous;

    if (policy > K_POLICY_STOP)
    {
        return K_ERR_INVALID_PARAMETER;
    }
    
    if (policy == K_POLICY_CURRENT)
    {
        policy_previous = armv7m_rtos_control.system_policy;
    }
    else
    {
        policy_previous = __armv7m_atomic_swapb(&armv7m_rtos_control.system_policy, policy);
    }

    if (p_policy_return)
    {
        *p_policy_return = policy_previous;
    }

    return K_NO_ERROR;
}

/* static */ void __svc_armv7m_rtos_task_exit(void)
{
    k_task_t *self;

    // armv7m_rtt_printf("k_task_exit()\n");
    
    self = armv7m_rtos_control.task_self;

    self->state |= K_TASK_STATE_TERMINATED;

    armv7m_rtos_task_ready_remove(self);
    
    if (self->join)
    {
        armv7m_rtos_join_release(self->join);
    }

    armv7m_rtos_mutex_destroy(self);

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_exit)();
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
    
    if (!(self->state & K_TASK_STATE_JOINABLE))
    {
        armv7m_rtos_task_destroy(self);
    }

    /* No special dispatch needed, as this case is handled via armv7m_rtos_task_exit.
     * That code on the tail end call armv7m_rtos_pendsv_callback, to avoid cycling 
     * throu PENDSV and possibly fetching a bad exception frame off the stack.
     */

    armv7m_rtos_control.task_self = K_TASK_TERMINATED;
    armv7m_rtos_control.task_next = armv7m_rtos_control.task_ready;

    if (armv7m_rtos_control.system_state == K_STATE_LOCKED)
    {
        armv7m_rtos_control.system_state = K_STATE_RUNNING;

        armv7m_rtos_work_schedule();
    }
}

/******************************************************************************************************************************/

void* k_heap_allocate(uint32_t size)
{

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_armv7m_rtos_heap_allocate(size);
    }

    if (!armv7m_core_is_in_interrupt())
    {
        return (void*)armv7m_svcall_1((uint32_t)&__svc_armv7m_rtos_heap_allocate, (uint32_t)size);
    }

    return NULL;
}

uint64_t k_system_clock(void)
{
    uint64_t clock;
    uint32_t seconds, ticks;
    
    clock = stm32wb_lptim_timeout_clock();

    seconds = clock / STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND;
    ticks = clock & (STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND -1);

    return ((uint64_t)(seconds * 1000) + ((ticks * 1000) / STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND));
}

uint32_t k_system_state(void)
{
    return armv7m_rtos_control.system_state;
}

int k_system_initialize(const k_hook_table_t *hook_table)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    if (armv7m_rtos_control.system_state != K_STATE_INACTIVE)
    {
        return K_ERR_ILLEGAL_USE;
    }

    armv7m_rtos_system_initialize(hook_table);

    return K_NO_ERROR;
}

int k_system_start(k_task_routine_t routine, void *context)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    if (armv7m_rtos_control.system_state != K_STATE_READY)
    {
        return K_ERR_ILLEGAL_USE;
    }

    armv7m_rtos_system_start(routine, context);
}

bool k_system_lock(void)
{
    if (!armv7m_core_is_in_thread())
    {
        return (armv7m_rtos_control.system_state == K_STATE_LOCKED);
    }

    if (armv7m_core_is_in_interrupt())
    {
        return __svc_armv7m_rtos_system_lock();
    }
    else
    {
        return armv7m_svcall_0((uint32_t)&__svc_armv7m_rtos_system_lock);
    }
}

void k_system_unlock(void)
{
    if (!armv7m_core_is_in_thread())
    {
        return;
    }
          
    if (armv7m_core_is_in_interrupt())
    {
        __svc_armv7m_rtos_system_unlock();
    }
    else
    {
        armv7m_svcall_0((uint32_t)&__svc_armv7m_rtos_system_unlock);
    }
}

bool k_system_is_locked(void)
{
    return (armv7m_rtos_control.system_state == K_STATE_LOCKED);
}

int k_system_set_policy(uint32_t policy, uint32_t *p_policy_return)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_system_set_policy, (uint32_t)policy, (uint32_t)p_policy_return);
}

/******************************************************************************************************************************/

void __attribute__((weak)) armv7m_rtos_join_release(k_task_t *task)
{
}

void __attribute__((weak)) armv7m_rtos_join_unblock(k_task_t *task)
{
}

void __attribute__((weak)) armv7m_rtos_delay_unblock(k_task_t *task)
{
}

void __attribute__((weak)) armv7m_rtos_sleep_unblock(k_task_t *task)
{
}

void __attribute__((weak)) armv7m_rtos_event_unblock(k_task_t *task)
{
}

void __attribute__((weak)) armv7m_rtos_sem_unblock(k_task_t *task)
{
}

k_task_t * __attribute__((weak)) armv7m_rtos_mutex_priority(k_task_t *task)
{
    return NULL;
}

void __attribute__((weak)) armv7m_rtos_mutex_unblock(k_task_t *task)
{
}

void __attribute__((weak)) armv7m_rtos_mutex_destroy(k_task_t *task)
{
}

void __attribute__((weak)) armv7m_rtos_work_return(void)
{
}

void __attribute__((weak)) armv7m_rtos_work_schedule(void)
{
}

/******************************************************************************************************************************/
