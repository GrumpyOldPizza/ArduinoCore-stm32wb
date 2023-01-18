/*
 * Copyright (c) 2019-2023 Thomas Roell.  All rights reserved.
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
#include "stm32wb_rtc.h"

extern uint32_t __HeapBase[];
extern uint32_t __HeapLimit[];
extern uint32_t __StackTop[];
extern uint32_t __StackLimit[];


#define K_TASK_STATE_WAIT_MASK       0x0007
#define K_TASK_STATE_WAIT_NONE       0x0000
#define K_TASK_STATE_WAIT_JOIN       0x0001
#define K_TASK_STATE_WAIT_EVENT      0x0002
#define K_TASK_STATE_WAIT_SEM        0x0003
#define K_TASK_STATE_WAIT_MUTEX      0x0004
#define K_TASK_STATE_TIMEOUT         0x0008
#define K_TASK_STATE_SUSPENDED       0x0010
#define K_TASK_STATE_TERMINATED      0x0020
#define K_TASK_STATE_JOINABLE        0x0040
#define K_TASK_STATE_READY           0x0080
#define K_TASK_STATE_EVENT_ALL       0x0400
#define K_TASK_STATE_EVENT_CLEAR     0x0800
#define K_TASK_STATE_AFFINITY_MASK   0xf000
#define K_TASK_STATE_AFFINITY_SHIFT  12

#define K_TASK_STATE_CAUSE_MASK      0x003f

/* Magic TASK values. Idea is that NULL means really "no task", i.e. the
 * "idle routine". However there is some special value required to separate
 * a switch from a terminated task to another task (including the idle routine.
 * Hence if K_TASK_NONE / K_TASK_TERMINATED get switched out, then no context
 * needs to be saved. Hence a transition from K_TASK_NONE to K_TASK_NONE is
 * like any other "no context switch required" case. Tricky part is that
 * armv7m_rtos_task_schedule() cannot switch from K_TASK_NONE to another
 * task. That case is handled in armv7m_rtos_idle_routine. This way the
 * idle routine can clean up the stack after itself.
 */

#define K_TASK_NONE                  ((k_task_t*)0x00000000)
#define K_TASK_TERMINATED            ((k_task_t*)0x00000001)

#define K_TASK_SENTINEL              ((k_task_t*)0x00000001)
#define K_WORK_SENTINEL              ((k_work_t*)0x00000001)
#define K_ALARM_SENTINEL             ((k_alarm_t*)0x00000001)

typedef void (*armv7m_rtos_wait_release_routine_t)(k_task_t *task);
typedef void (*armv7m_rtos_wait_unblock_routine_t)(k_task_t *task);

typedef struct _armv7m_rtos_wait_entry_t {
    armv7m_rtos_wait_release_routine_t release;
    armv7m_rtos_wait_unblock_routine_t unblock;
} armv7m_rtos_wait_entry_t;

typedef void (*armv7m_rtos_task_alarm_routine_t)(void);
typedef void (*armv7m_rtos_task_resume_routine_t)(void);
typedef void (*armv7m_rtos_task_release_routine_t)(void);

static const armv7m_rtos_wait_entry_t armv7m_rtos_join_wait_entry;
static const armv7m_rtos_wait_entry_t armv7m_rtos_event_wait_entry;
static const armv7m_rtos_wait_entry_t armv7m_rtos_sem_wait_entry;
static const armv7m_rtos_wait_entry_t armv7m_rtos_mutex_wait_entry;

typedef struct armv7m_rtos_control_t {
    k_task_t                *task_self;
    k_task_t                *task_next;
    k_task_t                *task_ready;
    k_task_t                *task_default;
    k_task_t                *task_link;
    k_task_t * volatile     task_resume;
    k_task_t * volatile     task_release;
    k_task_t                *task_timeout;
    uint64_t                task_clock;
    stm32wb_rtc_alarm_t     task_timer;
    k_work_t *              work_self;
    k_work_t *              work_head;
    k_work_t *              work_tail;
    k_work_t * volatile     work_submit;
    k_alarm_t               *alarm_queue;
    k_alarm_t * volatile    alarm_modify;
    uint64_t                alarm_clock;
    stm32wb_rtc_alarm_t     alarm_timer;
    volatile uint8_t        system_state;
    volatile uint8_t        system_policy;
    void                    *heap_current;
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    const k_hook_table_t    *hook_table;
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
    const armv7m_rtos_wait_entry_t *wait_table[K_TASK_STATE_WAIT_MUTEX - K_TASK_STATE_WAIT_JOIN + 1];
    armv7m_rtos_task_alarm_routine_t alarm_routine;
    armv7m_rtos_task_resume_routine_t resume_routine;
    armv7m_rtos_task_release_routine_t release_routine;
} armv7m_rtos_control_t;

static armv7m_rtos_control_t armv7m_rtos_control =
{
    .task_self = NULL,
    .task_next = NULL,
    .task_ready = NULL,
    .task_default = NULL,
    .task_link = K_TASK_SENTINEL,
    .task_resume = K_TASK_SENTINEL,
    .task_release = K_TASK_SENTINEL,
    .task_timeout = NULL,
    .task_clock = 0,
    .task_timer = STM32WB_RTC_ALARM_INIT(),
    .work_self = NULL,
    .work_head = K_WORK_SENTINEL,
    .work_tail = K_WORK_SENTINEL,
    .work_submit = K_WORK_SENTINEL,
    .alarm_queue = NULL,
    .alarm_modify = K_ALARM_SENTINEL,
    .alarm_clock = 0,
    .alarm_timer = STM32WB_RTC_ALARM_INIT(),
    .system_state = K_STATE_INACTIVE,
    .system_policy = K_POLICY_SLEEP,
    .heap_current = (void*)&__HeapBase[0],
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    .hook_table = NULL,
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
    .wait_table = { NULL, NULL, NULL, NULL },
    .alarm_routine = NULL,
    .resume_routine = NULL,
    .release_routine = NULL,
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

static void armv7m_rtos_pendsv_epilogue(void);

static void armv7m_rtos_system_initialize(const k_hook_table_t *hook_table);
static void armv7m_rtos_system_start(k_task_routine_t routine, void *context) __attribute__((noreturn));

static uint32_t armv7m_rtos_task_state(k_task_t *task);
static void armv7m_rtos_task_status(k_task_t *task, uint32_t status);
static void armv7m_rtos_task_start(k_task_t *task, k_task_routine_t routine, void *context);
static void armv7m_rtos_task_return(void);
static void armv7m_rtos_task_exit(void);

static void armv7m_rtos_task_queue_insert(k_task_t **p_task_head, k_task_t *task);
static void armv7m_rtos_task_queue_remove(k_task_t **p_task_head, k_task_t *task);
static void armv7m_rtos_task_ready_insert(k_task_t *task);
static void armv7m_rtos_task_ready_remove(k_task_t *task);

static void armv7m_rtos_task_create(k_task_t *task, const char *name, uint32_t priority, void *stack_base, uint32_t stack_size, uint32_t options);
static void armv7m_rtos_task_destroy(k_task_t *task);
static void armv7m_rtos_task_priority(k_task_t *task);

static void armv7m_rtos_task_resume_process(k_task_t *task);
static void armv7m_rtos_task_resume_dequeue(void);
static bool armv7m_rtos_task_resume_enqueue(k_task_t *task);

static void armv7m_rtos_task_release_process(k_task_t *task);
static void armv7m_rtos_task_release_dequeue(void);
static bool armv7m_rtos_task_release_enqueue(k_task_t *task);

static bool armv7m_rtos_task_unblock(k_task_t *task, uint32_t status);
static void armv7m_rtos_task_schedule(void);

static uint64_t armv7m_rtos_clock_convert(uint64_t clock);
static uint64_t armv7m_rtos_clock_offset(uint64_t reference, uint32_t delay);

static void armv7m_rtos_timeout_insert(k_task_t *task, uint64_t clock);
static void armv7m_rtos_timeout_remove(k_task_t *task);
static void armv7m_rtos_timeout_absolute(k_task_t *task, uint64_t clock);
static void armv7m_rtos_timeout_relative(k_task_t *task, uint32_t delay);
static void armv7m_rtos_timeout_callback(void *context, uint64_t reference);
static void armv7m_rtos_timeout_schedule(void);

static void armv7m_rtos_join_release(k_task_t *task);
static void armv7m_rtos_join_unblock(k_task_t *task);

static void armv7m_rtos_event_release(k_task_t *task);
static void armv7m_rtos_event_unblock(k_task_t *task);

static void armv7m_rtos_sem_wait_insert(k_sem_t *sem, k_task_t *task);
static void armv7m_rtos_sem_wait_remove(k_sem_t *sem, k_task_t *task);
static k_task_t* armv7m_rtos_sem_wait_release(k_sem_t *sem);
static void armv7m_rtos_sem_wait_acquire(k_sem_t *sem);
static void armv7m_rtos_sem_release(k_task_t *task);
static void armv7m_rtos_sem_unblock(k_task_t *task);

static void armv7m_rtos_mutex_queue_insert(k_mutex_t **p_mutex_head, k_mutex_t *mutex);
static void armv7m_rtos_mutex_queue_remove(k_mutex_t **p_mutex_head, k_mutex_t *mutex);
static void armv7m_rtos_mutex_owner_insert(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_owner_remove(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_owner_attach(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_owner_detach(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_wait_insert(k_mutex_t *mutex, k_task_t *task);
static void armv7m_rtos_mutex_wait_remove(k_mutex_t *mutex, k_task_t *task);
static k_task_t * armv7m_rtos_mutex_priority(k_task_t *task);
static void armv7m_rtos_mutex_destroy(k_task_t *task);
static void armv7m_rtos_mutex_release(k_task_t *task);
static void armv7m_rtos_mutex_unblock(k_task_t *task);

static int armv7m_rtos_work_deinit(k_work_t *work);
static int armv7m_rtos_work_submit(k_work_t *work);
static void armv7m_rtos_work_return(void);
static void armv7m_rtos_work_schedule(void);
static void armv7m_rtos_work_routine(void);


static void armv7m_rtos_alarm_insert(k_alarm_t *alarm, uint64_t clock);
static void armv7m_rtos_alarm_remove(k_alarm_t *alarm);
static void armv7m_rtos_alarm_routine(void);
static void armv7m_rtos_alarm_schedule(void);
static void armv7m_rtos_alarm_callback(void *context, uint64_t reference);
static void armv7m_rtos_alarm_modify(k_alarm_t *alarm, uint64_t clock, uint32_t period);

/******************************************************************************************************************************/

static void * __svc_armv7m_rtos_heap_allocate(uint32_t size);

static void __svc_armv7m_rtos_system_start(k_task_routine_t routine, void *context);
static bool __svc_armv7m_rtos_system_lock(void);
static void __svc_armv7m_rtos_system_unlock(void);
static int __svc_armv7m_rtos_system_set_policy(uint32_t policy, uint32_t *p_policy_return);

static int __svc_armv7m_rtos_task_create(k_task_t *task, const k_task_params_t *params);
       void __svc_armv7m_rtos_task_return(void);
       void __svc_armv7m_rtos_task_exit(void);
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
static int __svc_armv7m_rtos_task_yield(void);

static int __svc_armv7m_rtos_event_send(k_task_t *task, uint32_t events);
static int __svc_armv7m_rtos_event_receive(uint32_t events, uint32_t mode, uint32_t timeout, uint32_t *p_events_return);

static int __svc_armv7m_rtos_sem_init(k_sem_t *sem, uint32_t count, uint32_t limit);
static int __svc_armv7m_rtos_sem_deinit(k_sem_t *sem);
static int __svc_armv7m_rtos_sem_acquire(k_sem_t *sem, uint32_t timeout);
static int __svc_armv7m_rtos_sem_release(k_sem_t *sem);

static int __svc_armv7m_rtos_mutex_init(k_mutex_t *mutex, uint32_t priority, uint32_t options);
static int __svc_armv7m_rtos_mutex_deinit(k_mutex_t *mutex);
static int __svc_armv7m_rtos_mutex_set_priority(k_mutex_t *mutex, uint32_t priority, uint32_t *p_priority_return);
static int __svc_armv7m_rtos_mutex_lock(k_mutex_t *mutex, uint32_t timeout);
static int __svc_armv7m_rtos_mutex_unlock(k_mutex_t *mutex);

static int __svc_armv7m_rtos_work_init(k_work_t *work, k_work_routine_t routine, void *context);
static int __svc_armv7m_rtos_work_deinit(k_work_t *work);

static int __svc_armv7m_rtos_alarm_init(k_alarm_t *alarm, k_alarm_routine_t routine, void *context);
static int __svc_armv7m_rtos_alarm_deinit(k_alarm_t *alarm);
static int __svc_armv7m_rtos_alarm_absolute(k_alarm_t *alarm, uint32_t clock_l, uint32_t clock_h, uint32_t period);
static int __svc_armv7m_rtos_alarm_relative(k_alarm_t *alarm, uint32_t delay, uint32_t period);
static int __svc_armv7m_rtos_alarm_cancel(k_alarm_t *alarm);

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

/******************************************************************************************************************************/

static const armv7m_rtos_wait_entry_t armv7m_rtos_join_wait_entry = {
    .release = armv7m_rtos_join_release,
    .unblock = armv7m_rtos_join_unblock
};

static const armv7m_rtos_wait_entry_t armv7m_rtos_event_wait_entry = {
    .release = armv7m_rtos_event_release,
    .unblock = armv7m_rtos_event_unblock
};

static const armv7m_rtos_wait_entry_t armv7m_rtos_sem_wait_entry = {
    .release = armv7m_rtos_sem_release,
    .unblock = armv7m_rtos_sem_unblock
};

static const armv7m_rtos_wait_entry_t armv7m_rtos_mutex_wait_entry = {
    .release = armv7m_rtos_mutex_release,
    .unblock = armv7m_rtos_mutex_unblock
};

/******************************************************************************************************************************/

static void __attribute__((naked, noreturn)) armv7m_rtos_pendsv_epilogue(void)
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
        "7: movw     r1, #:lower16:armv7m_rtos_system_idle     \n"
        "   movt     r1, #:upper16:armv7m_rtos_system_idle     \n"
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

static void __attribute__((naked, used, noreturn)) armv7m_rtos_system_idle(void)
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

static uint32_t armv7m_rtos_task_state(k_task_t *task)
{
    return ((task->state & K_TASK_STATE_TERMINATED)
            ? K_STATE_TERMINATED
            : ((task->state & K_TASK_STATE_SUSPENDED)
               ? K_STATE_SUSPENDED
               : ((task->state & (K_TASK_STATE_TIMEOUT | K_TASK_STATE_WAIT_MASK))
                  ? K_STATE_BLOCKED
                  : ((task->state & K_TASK_STATE_READY)
                     ? ((task != armv7m_rtos_control.task_self) ? K_STATE_READY : K_STATE_RUNNING)
                     : K_STATE_INACTIVE))));
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

static void armv7m_rtos_task_start(k_task_t *task, k_task_routine_t routine, void *context)
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
    stack->lr = (uint32_t)armv7m_rtos_task_return;
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

static void __attribute__((naked, noreturn)) armv7m_rtos_task_return(void)
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
        "   bl       __svc_armv7m_rtos_task_return             \n"
        "   b        armv7m_rtos_pendsv_epilogue               \n"
        :
        :
        );
}

static void __attribute__((naked, noreturn)) armv7m_rtos_task_exit(void)
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

/******************************************************************************************************************************/

static void __attribute__((optimize("O3"), noinline)) armv7m_rtos_task_queue_insert(k_task_t **p_task_head, k_task_t *task)
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

static void __attribute__((optimize("O3"), noinline)) armv7m_rtos_task_queue_remove(k_task_t **p_task_head, k_task_t *task)
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

static void armv7m_rtos_task_ready_insert(k_task_t *task)
{
    task->state |= K_TASK_STATE_READY;

    armv7m_rtos_task_queue_insert(&armv7m_rtos_control.task_ready, task);
}

static void armv7m_rtos_task_ready_remove(k_task_t *task)
{
    task->state &= ~K_TASK_STATE_READY;

    armv7m_rtos_task_queue_remove(&armv7m_rtos_control.task_ready, task);
}

static void armv7m_rtos_task_create(k_task_t *task, const char *name, uint32_t priority, void *stack_base, uint32_t stack_size, uint32_t options)
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
    task->release = NULL;
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

static void armv7m_rtos_task_destroy(k_task_t *task)
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

static void armv7m_rtos_task_resume_process(k_task_t *task)
{
    k_task_t *task_resume;

    do
    {
        task_resume = task->resume;
            
        task->state &= ~K_TASK_STATE_SUSPENDED;
            
        task->resume = NULL;

        if (task->link && !(task->state & K_TASK_STATE_TERMINATED))
        {
            if (!(task->state & (K_TASK_STATE_TIMEOUT | K_TASK_STATE_WAIT_MASK)))
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

        task = task_resume;
    }
    while (task != K_TASK_SENTINEL);

    armv7m_rtos_task_schedule();
}

static void armv7m_rtos_task_resume_dequeue(void)
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

static bool armv7m_rtos_task_resume_enqueue(k_task_t *task)
{
    k_task_t *task_resume;

    if (__armv7m_atomic_cas((volatile uint32_t*)&task->resume, (uint32_t)NULL, (uint32_t)K_TASK_SENTINEL) != (uint32_t)NULL)
    {
        return false;
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        armv7m_rtos_task_resume_process(task);
    }
    else
    {
        armv7m_rtos_control.resume_routine = armv7m_rtos_task_resume_dequeue;

        task_resume = (k_task_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_resume, (uint32_t)task);
        
        task->resume = task_resume;

        if (task_resume == K_TASK_SENTINEL)
        {
            armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_TASK_RESUME);
        }
    }
    
    return true;
}

static void armv7m_rtos_task_release_process(k_task_t *task)
{
    k_task_t *task_release, *task_timeout;

    task_timeout = armv7m_rtos_control.task_timeout;
    
    do
    {
        task_release = task->release;
            
        if (task->state & K_TASK_STATE_TIMEOUT)
        {
            armv7m_rtos_timeout_remove(task);
        }

        if (task->state & K_TASK_STATE_WAIT_MASK)
        {
            (*armv7m_rtos_control.wait_table[(task->state & K_TASK_STATE_WAIT_MASK) - K_TASK_STATE_WAIT_JOIN]->release)(task);
        }
        
        task->state &= ~(K_TASK_STATE_EVENT_ALL | K_TASK_STATE_EVENT_CLEAR | K_TASK_STATE_WAIT_MASK);

        task->release = NULL;
        
        if (task->link && !(task->state & K_TASK_STATE_TERMINATED))
        {
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
        
        task = task_release;
    }
    while (task != K_TASK_SENTINEL);

    if (task_timeout != armv7m_rtos_control.task_timeout)
    {
        armv7m_rtos_timeout_schedule();
    }
    
    armv7m_rtos_task_schedule();
}

static void armv7m_rtos_task_release_dequeue(void)
{
    k_task_t *task_release, *task_previous, *task_next;

    task_release = (k_task_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_release, (uint32_t)K_TASK_SENTINEL);

    if (task_release != K_TASK_SENTINEL)
    {
        if (task_release->release != K_TASK_SENTINEL)
        {
            for (task_previous = K_TASK_SENTINEL; task_release != K_TASK_SENTINEL; task_release = task_next)
            {
                task_next = task_release->release;
                
                task_release->release = task_previous;
                
                task_previous = task_release;
            }
            
            task_release = task_previous;
        }
        
        armv7m_rtos_task_release_process(task_release);
    }
}

static bool armv7m_rtos_task_release_enqueue(k_task_t *task)
{
    k_task_t *task_release;
    
    if (__armv7m_atomic_cas((volatile uint32_t*)&task->release, (uint32_t)NULL, (uint32_t)K_TASK_SENTINEL) != (uint32_t)NULL)
    {
        return false;
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        armv7m_rtos_task_release_process(task);
    }
    else
    {
        armv7m_rtos_control.release_routine = armv7m_rtos_task_release_dequeue;
      
        task_release = (k_task_t*)__armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.task_release, (uint32_t)task);

        task->release = task_release;

        if (task_release == K_TASK_SENTINEL)
        {
            armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_TASK_RELEASE);
        }
    }
    
    return true;
}

static bool armv7m_rtos_task_unblock(k_task_t *task, uint32_t status)
{
    if (__armv7m_atomic_cas((volatile uint32_t*)&task->release, (uint32_t)NULL, (uint32_t)K_TASK_SENTINEL) != (uint32_t)NULL)
    {
        return false;
    }

    armv7m_rtos_task_status(task, status);

    if (task->state & K_TASK_STATE_TIMEOUT)
    {
        armv7m_rtos_timeout_remove(task);
    }

    if (task->state & K_TASK_STATE_WAIT_MASK)
    {
        (*armv7m_rtos_control.wait_table[(task->state & K_TASK_STATE_WAIT_MASK) - K_TASK_STATE_WAIT_JOIN]->unblock)(task);
    }

    task->state &= ~(K_TASK_STATE_EVENT_ALL | K_TASK_STATE_EVENT_CLEAR | K_TASK_STATE_WAIT_MASK);

    task->release = NULL;

    if (task->link && !(task->state & K_TASK_STATE_TERMINATED))
    {
        if (!(task->state & K_TASK_STATE_SUSPENDED))
        {
            armv7m_rtos_task_ready_insert(task);
            
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
            (*armv7m_rtos_control.hook_table->task_ready)(task);
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
        }
    }
    
    return true;
}

static void armv7m_rtos_task_schedule(void)
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

static uint64_t armv7m_rtos_clock_convert(uint64_t clock)
{
    uint32_t seconds, millis;

    seconds = clock / 1000;
    millis = clock - seconds * 1000;

    clock = (((uint64_t)seconds * (uint64_t)STM32WB_RTC_CLOCK_TICKS_PER_SECOND) + ((millis * STM32WB_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000));

    return clock;
}

static uint64_t armv7m_rtos_clock_offset(uint64_t reference, uint32_t delay)
{
    uint64_t clock;
    uint32_t ticks, seconds, millis;

    seconds = delay / 1000;
    millis = delay - seconds * 1000;

    ticks = reference & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND -1);
    clock = reference & ~(STM32WB_RTC_CLOCK_TICKS_PER_SECOND -1);

    millis += ((ticks * 1000) / STM32WB_RTC_CLOCK_TICKS_PER_SECOND);
    
    clock += (((uint64_t)seconds * (uint64_t)STM32WB_RTC_CLOCK_TICKS_PER_SECOND) + ((millis * STM32WB_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000));

    return clock;
}


static void armv7m_rtos_timeout_insert(k_task_t *task, uint64_t clock)
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

static void armv7m_rtos_timeout_remove(k_task_t *task)
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

static void armv7m_rtos_timeout_absolute(k_task_t *task, uint64_t clock)
{
    armv7m_rtos_timeout_insert(task, armv7m_rtos_clock_convert(clock));

    armv7m_rtos_timeout_schedule();
}

static void armv7m_rtos_timeout_relative(k_task_t *task, uint32_t delay)
{
    armv7m_rtos_timeout_insert(task, armv7m_rtos_clock_offset(stm32wb_rtc_clock_read(), delay));

    armv7m_rtos_timeout_schedule();
}

static void armv7m_rtos_timeout_callback(void *context, uint64_t reference)
{
    k_task_t *task;
    uint64_t clock;

    task = armv7m_rtos_control.task_timeout;

    if (task)
    {
        do
        {
            clock = (((uint64_t)task->timeout.clock_l << 0) | ((uint64_t)task->timeout.clock_h << 32));
                
            if (clock > reference)
            {
                break;
            }

            armv7m_rtos_task_unblock(task, (((task->state & K_TASK_STATE_WAIT_MASK) == K_TASK_STATE_WAIT_NONE) ? K_NO_ERROR : K_ERR_TIMEOUT));
            
            task = armv7m_rtos_control.task_timeout;
        }
        while (task);

        armv7m_rtos_timeout_schedule();
    
        armv7m_rtos_task_schedule();
    }
}

static void armv7m_rtos_timeout_schedule(void)
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

            stm32wb_rtc_alarm_start(&armv7m_rtos_control.task_timer, clock, armv7m_rtos_timeout_callback, NULL);
        }
    }
    else
    {
        if (armv7m_rtos_control.task_clock)
        {
            armv7m_rtos_control.task_clock = 0;

            stm32wb_rtc_alarm_stop(&armv7m_rtos_control.task_timer);
        }
    }
}

static void armv7m_rtos_join_release(k_task_t *task)
{
    armv7m_rtos_task_destroy(task->wait.join.task);
}
    
static void armv7m_rtos_join_unblock(k_task_t *task)
{
    task->wait.join.task->join = NULL;
}

static void armv7m_rtos_event_release(k_task_t *task)
{
    uint32_t events_return, *p_events_return;

    events_return = task->events & task->wait.event.events;

    if (task->state & K_TASK_STATE_EVENT_CLEAR)
    {
        armv7m_atomic_and(&task->events, ~events_return);
    }
    
    p_events_return = task->wait.event.p_events_return;
    
    if (p_events_return)
    {
        *p_events_return = events_return;
    }
}

static void armv7m_rtos_event_unblock(k_task_t *task)
{
}


static void armv7m_rtos_sem_wait_insert(k_sem_t *sem, k_task_t *task)
{
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

    task->next = NULL;
    task->previous = NULL;
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

    task->next = NULL;
    task->previous = NULL;
}

static void armv7m_rtos_sem_release(k_task_t *task)
{
    /* This is tricky here. The code to remove the
     * head of a semphore wait list is called out of
     * order. So here we just need to remove one task from
     * the waiting list for evry task task_release had
     * be called.
     *
     * If the reference in wait.sem.sem is gone means
     * that k_sem_deinit() is doing the cleanup.
     */

    if (task->wait.sem.sem)
    {
        armv7m_rtos_sem_wait_acquire(task->wait.sem.sem);
    }
}

static void armv7m_rtos_sem_unblock(k_task_t *task)
{
    /* We can always remove this task, because the async sem release logic
     * skips tasks with a non-NULL task->release.
     */

    armv7m_rtos_sem_wait_remove(task->wait.sem.sem, task);
}


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

static k_task_t * armv7m_rtos_mutex_priority(k_task_t *task)
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

static void armv7m_rtos_mutex_destroy(k_task_t *task)
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
                armv7m_rtos_task_release_enqueue(mutex->waiting);
            }
            
            mutex = task->mutex;
        }
        while (mutex);
    }
}

static void armv7m_rtos_mutex_unblock(k_task_t *task)
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
}


static int armv7m_rtos_work_deinit(k_work_t *work)
{
    k_work_t * volatile *work_previous;
    k_work_t *work_submit;

    __armv7m_atomic_store_2_restart((volatile uint32_t*)&work->routine, 0, 0);

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

static int  __attribute__((optimize("O3"))) armv7m_rtos_work_submit(k_work_t *work)
{
    k_work_t *work_submit;
  
    if (__armv7m_atomic_cas((volatile uint32_t*)&work->next, (uint32_t)NULL, (uint32_t)K_WORK_SENTINEL) != (uint32_t)NULL)
    {
        return K_ERR_WORK_ALREADY_SUBMITTED;
    }

    do
    {
        work_submit = armv7m_rtos_control.work_submit;

        work->next = work_submit;
    }
    while ((k_work_t*)__armv7m_atomic_cas((volatile uint32_t*)&armv7m_rtos_control.work_submit, (uint32_t)work_submit, (uint32_t)work) != work_submit);
    
    if (work_submit == K_WORK_SENTINEL)
    {
        armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_WORK);
    }

    return K_NO_ERROR;
}

static void __attribute__((optimize("O3"), used)) armv7m_rtos_work_return(void)
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

static void __attribute__((optimize("O3"))) armv7m_rtos_work_schedule(void)
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

                armv7m_pendsv_hook(armv7m_rtos_pendsv_epilogue);
                
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
                (*armv7m_rtos_control.hook_table->task_run)(K_TASK_WORK);
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
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

            stm32wb_rtc_alarm_start(&armv7m_rtos_control.alarm_timer, clock, armv7m_rtos_alarm_callback, NULL);
        }
    }
    else
    {
        if (armv7m_rtos_control.alarm_clock)
        {
            armv7m_rtos_control.alarm_clock = 0;

            stm32wb_rtc_alarm_stop(&armv7m_rtos_control.alarm_timer);
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

static void armv7m_rtos_alarm_callback(void *context, uint64_t reference)
{
    k_alarm_t *alarm;
    uint64_t clock;
    uint32_t clock_l, clock_h, ticks, period, seconds, millis;
    
    alarm = armv7m_rtos_control.alarm_queue;

    if (alarm)
    {
        do
        {
            clock = (((uint64_t)alarm->clock_l << 0) | ((uint64_t)alarm->clock_h << 32));
            period = alarm->period;
                
            if (!alarm->modify)
            {
                if (clock > reference)
                {
                    break;
                }

                armv7m_rtos_alarm_remove(alarm);
                
                armv7m_rtos_work_submit(&alarm->work);
        
                if (period)
                {
                    seconds = period / 1000;
                    millis = period - seconds * 1000;

                    ticks = clock & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND -1);
                    clock = clock & ~(STM32WB_RTC_CLOCK_TICKS_PER_SECOND -1);

                    millis += ((ticks * 1000) / STM32WB_RTC_CLOCK_TICKS_PER_SECOND);
    
                    clock += (((uint64_t)seconds * (uint64_t)STM32WB_RTC_CLOCK_TICKS_PER_SECOND) + ((millis * STM32WB_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000));
                    
                    clock_l = (uint32_t)(clock >> 0);
                    clock_h = (uint32_t)(clock >> 32);

                    if (!alarm->modify)
                    {
                        __armv7m_atomic_store_3_restart((volatile uint32_t*)&alarm->clock_l, clock_l, clock_h, period);

                        armv7m_rtos_alarm_insert(alarm, clock);
                    }
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

static void armv7m_rtos_alarm_modify(k_alarm_t *alarm, uint64_t clock, uint32_t period)
{
    k_alarm_t *alarm_modify;
    uint32_t clock_l, clock_h;

    clock_l = (uint32_t)(clock >> 0);
    clock_h = (uint32_t)(clock >> 32);
    
    __armv7m_atomic_store_3_restart((volatile uint32_t*)&alarm->clock_l, clock_l, clock_h, period);
    
    if (armv7m_atomic_cas((volatile uint32_t*)&alarm->modify, (uint32_t)NULL, (uint32_t)K_ALARM_SENTINEL) == (uint32_t)NULL)
    {
        armv7m_rtos_control.alarm_routine = armv7m_rtos_alarm_routine;

        alarm_modify = (k_alarm_t*)armv7m_atomic_swap((volatile uint32_t*)&armv7m_rtos_control.alarm_modify, (uint32_t)alarm);

        alarm->modify = alarm_modify;

        if (alarm_modify == K_ALARM_SENTINEL)
        {
            armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTOS_ALARM);
        }
    }
}

/******************************************************************************************************************************/

static void * __attribute__((noinline)) __svc_armv7m_rtos_heap_allocate(uint32_t size)
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

static void  __attribute__((noinline, used)) __svc_armv7m_rtos_system_start(k_task_routine_t routine, void *context)
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

static bool __attribute__((noinline)) __svc_armv7m_rtos_system_lock(void)
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

static void __attribute__((noinline)) __svc_armv7m_rtos_system_unlock(void)
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

static int __attribute__((noinline)) __svc_armv7m_rtos_system_set_policy(uint32_t policy, uint32_t *p_policy_return)
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


static int __attribute__((noinline)) __svc_armv7m_rtos_task_create(k_task_t *task, const k_task_params_t *params)
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

/* static */ void __attribute__((noinline)) __svc_armv7m_rtos_task_return(void)
{
    k_task_t *self;

    // armv7m_rtt_printf("k_task_return()\n");
    
    self = armv7m_rtos_control.task_self;

    armv7m_rtos_task_ready_remove(self);

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

/* static */ void __attribute__((noinline)) __svc_armv7m_rtos_task_exit(void)
{
    k_task_t *self;

    // armv7m_rtt_printf("k_task_exit()\n");
    
    self = armv7m_rtos_control.task_self;

    self->state |= K_TASK_STATE_TERMINATED;

    armv7m_rtos_task_ready_remove(self);
    
    if (self->join)
    {
        armv7m_rtos_task_release_enqueue(self->join);
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

static int __attribute__((noinline)) __svc_armv7m_rtos_task_terminate(k_task_t *task)
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

    if (task->state & (K_TASK_STATE_TIMEOUT | K_TASK_STATE_WAIT_MASK))
    {
        armv7m_rtos_task_unblock(task, K_ERR_UNSATISFIED);
    }

    if (task->state & K_TASK_STATE_SUSPENDED)
    {
        armv7m_rtos_task_resume_enqueue(task);
    }

    // Resolve all the deferred resume/resolve logic
    armv7m_rtos_task_resume_dequeue();
    armv7m_rtos_task_release_dequeue();

    if (task->state & K_TASK_STATE_READY)
    {
        armv7m_rtos_task_ready_remove(task);
    }

    if (task->join)
    {
        armv7m_rtos_task_release_enqueue(task->join);
    }

    armv7m_rtos_mutex_destroy(task);

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_terminate)(task);
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
    
    if (!(task->state & K_TASK_STATE_JOINABLE))
    {
        armv7m_rtos_task_destroy(task);
    }

    armv7m_rtos_timeout_schedule();
    
    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_task_detach(k_task_t *task)
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

static int __attribute__((noinline)) __svc_armv7m_rtos_task_join(k_task_t *task)
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

    armv7m_rtos_control.wait_table[K_TASK_STATE_WAIT_JOIN - K_TASK_STATE_WAIT_JOIN] = &armv7m_rtos_join_wait_entry;

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

static int __attribute__((noinline)) __svc_armv7m_rtos_task_enumerate(uint32_t *p_count_return, k_task_t **p_task_return, uint32_t count)
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

static int __attribute__((noinline)) __svc_armv7m_rtos_task_info(k_task_t *task, k_task_info_t *p_task_info_return)
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
        p_task_info_return->events = task->events;
    }
    
    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_task_stack(k_task_t *task, uint32_t *p_stack_base_return, uint32_t *p_stack_size_return, uint32_t *p_stack_space_return)
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

static int __attribute__((noinline)) __svc_armv7m_rtos_task_unblock(k_task_t *task)
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

    armv7m_rtos_timeout_schedule();
    
    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_task_suspend(k_task_t *task)
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

static int __attribute__((noinline)) __svc_armv7m_rtos_task_resume(k_task_t *task)
{
    // armv7m_rtt_printf("k_task_resume(task=%08x)\n", task);

    if (!task || !task->link || (task->state & K_TASK_STATE_TERMINATED))
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (!(task->state & K_TASK_STATE_SUSPENDED))
    {
        return K_ERR_TASK_NOT_SUSPENDED;
    }

    if (!armv7m_rtos_task_resume_enqueue(task))
    {
        return K_ERR_TASK_NOT_SUSPENDED;
    }

    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_task_set_priority(k_task_t *task, uint32_t priority, uint32_t *p_priority_return)
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

static int __attribute__((noinline)) __svc_armv7m_rtos_task_get_priority(k_task_t *task, uint32_t *p_priority_return)
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

static int __attribute__((noinline)) __svc_armv7m_rtos_task_delay(uint32_t delay)
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
        
    armv7m_rtos_timeout_relative(self, delay);
    
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_block)(self, (self->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */

    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_task_delay_until(uint32_t clock_l, uint32_t clock_h)
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

    armv7m_rtos_timeout_absolute(self, clock);

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_block)(self, (self->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */

    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_task_yield(void)
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


static int __attribute__((noinline)) __svc_armv7m_rtos_event_send(k_task_t *task, uint32_t events)
{
    uint32_t events_return;

    // armv7m_rtt_printf("k_event_send(task=%08x, events=%08x)\n", task, events);

    if (!task || !task->link || (task->state & K_TASK_STATE_TERMINATED))
    {
        return K_ERR_INVALID_OBJECT;
    }

    armv7m_atomic_or(&task->events, events);

    if ((task->state & K_TASK_STATE_WAIT_MASK) == K_TASK_STATE_WAIT_EVENT)
    {
        events_return = task->events & task->wait.event.events;

        if (task->state & K_TASK_STATE_EVENT_ALL)
        {
            if (events_return != task->wait.event.events)
            {
                events_return = 0;
            }
        }

        if (events_return)
        {
            armv7m_rtos_task_release_enqueue(task);
        }
    }

    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_event_receive(uint32_t events, uint32_t mode, uint32_t timeout, uint32_t *p_events_return)
{
    k_task_t *self;
    uint32_t events_return;

    // armv7m_rtt_printf("k_event_receive(events=%08x, mode=%08x, timeout=%d, p_events_return=%08x)\n", events, mode, timeout, p_events_return);

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

    armv7m_rtos_control.wait_table[K_TASK_STATE_WAIT_EVENT - K_TASK_STATE_WAIT_JOIN] = &armv7m_rtos_event_wait_entry;
    
    events_return = self->events & events;

    if (mode & K_EVENT_ALL)
    {
        if (events_return != events)
        {
            events_return = 0;
        }
    }
    
    if (events_return)
    {
        if (mode & K_EVENT_CLEAR)
        {
            armv7m_atomic_and(&self->events, ~events_return);
        }
        
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

    armv7m_rtos_task_ready_remove(self);

    self->wait.event.events = events;
    self->wait.event.p_events_return = p_events_return;

    self->state |= (K_TASK_STATE_WAIT_EVENT | ((mode & K_EVENT_ALL) ? K_TASK_STATE_EVENT_ALL : 0) | ((mode & K_EVENT_CLEAR) ? K_TASK_STATE_EVENT_CLEAR : 0));
    
    if (timeout != K_TIMEOUT_FOREVER)
    {
        armv7m_rtos_timeout_relative(self, timeout);
    }

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_block)(self, (self->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */

    armv7m_rtos_task_schedule();
    
    events_return = self->events & events;

    if (mode & K_EVENT_ALL)
    {
        if (events_return != events)
        {
            events_return = 0;
        }
    }

    if (events_return)
    {
        armv7m_rtos_task_release_enqueue(self);
    }

    return K_NO_ERROR;
}



static int __attribute__((noinline)) __svc_armv7m_rtos_sem_init(k_sem_t *sem, uint32_t count, uint32_t limit)
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

static int __attribute__((noinline)) __svc_armv7m_rtos_sem_deinit(k_sem_t *sem)
{
    k_task_t *task;

    // armv7m_rtt_printf("k_sem_deinit(sem=%08x)\n", sem);

    if (armv7m_rtos_control.work_self)
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }
    
    if (!sem)
    {
        return K_ERR_INVALID_OBJECT;
    }

    task = sem->waiting.head;

    if (task)
    {
        sem->waiting.release = NULL;
        
        do
        {
            if (!armv7m_rtos_task_unblock(task, K_ERR_UNSATISFIED))
            {
                /* Means the task got already released, so just remove
                 * it from the waiting list, and let the release logic
                 * handle the rest.
                 */
                armv7m_rtos_sem_wait_remove(sem, task);

                task->wait.sem.sem = NULL;
            }
            
            task = sem->waiting.head;
        }
        while (task);

        armv7m_rtos_timeout_schedule();
        
        armv7m_rtos_task_schedule();
    }

    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_sem_acquire(k_sem_t *sem, uint32_t timeout)
{
    uint32_t count;
    k_task_t *self, *task;

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

    armv7m_rtos_control.wait_table[K_TASK_STATE_WAIT_SEM - K_TASK_STATE_WAIT_JOIN] = &armv7m_rtos_sem_wait_entry;
    
    count = armv7m_atomic_dech(&sem->count);

    if (count)
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

    self->state |= K_TASK_STATE_WAIT_SEM;
    
    if (timeout != K_TIMEOUT_FOREVER)
    {
        armv7m_rtos_timeout_relative(self, timeout);
    }

#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    (*armv7m_rtos_control.hook_table->task_block)(self, (self->state & K_TASK_STATE_CAUSE_MASK));
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */

    armv7m_rtos_task_schedule();

    armv7m_rtos_sem_wait_insert(sem, self);

    count = armv7m_atomic_dech(&sem->count);

    if (count)
    {
        task = armv7m_rtos_sem_wait_release(sem);

        if (task)
        {
            armv7m_rtos_task_release_enqueue(task);
        }
        else
        {
            if (armv7m_atomic_inch(&sem->count, sem->limit) == sem->limit)
            {
                return K_ERR_SEM_OVERFLOW;
            }
        }
    }
    
    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_sem_release(k_sem_t *sem)
{
    k_task_t *task;

    // armv7m_rtt_printf("k_sem_release(sem=%08x)\n", sem);
    
    if (!sem)
    {
        return K_ERR_INVALID_OBJECT;
    }

    task = armv7m_rtos_sem_wait_release(sem);
    
    if (task)
    {
        armv7m_rtos_task_release_enqueue(task);
    }
    else
    {
        if (armv7m_atomic_inch(&sem->count, sem->limit) == sem->limit)
        {
            return K_ERR_SEM_OVERFLOW;
        }
    }

    return K_NO_ERROR;
}


static int __attribute__((noinline)) __svc_armv7m_rtos_mutex_init(k_mutex_t *mutex, uint32_t priority, uint32_t options)
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

static int __attribute__((noinline)) __svc_armv7m_rtos_mutex_deinit(k_mutex_t *mutex)
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
        while (mutex->waiting)
        {
            armv7m_rtos_task_unblock(mutex->waiting, K_ERR_UNSATISFIED);
        }

        armv7m_rtos_timeout_schedule();
    }
    
    armv7m_rtos_task_schedule();
    
    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_mutex_set_priority(k_mutex_t *mutex, uint32_t priority, uint32_t *p_priority_return)
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

static int __attribute__((noinline, optimize("O3"))) __svc_armv7m_rtos_mutex_lock(k_mutex_t *mutex, uint32_t timeout)
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

    armv7m_rtos_control.wait_table[K_TASK_STATE_WAIT_MUTEX - K_TASK_STATE_WAIT_JOIN] = &armv7m_rtos_mutex_wait_entry;

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
        armv7m_rtos_timeout_relative(self, timeout);
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

static int __attribute__((noinline)) __svc_armv7m_rtos_mutex_unlock(k_mutex_t *mutex)
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
        armv7m_rtos_task_release_enqueue(mutex->waiting);
    }

    if (mutex->options & (K_MUTEX_PRIORITY_INHERIT | K_MUTEX_PRIORITY_PROTECT))
    {
        armv7m_rtos_task_priority(self);

        armv7m_rtos_task_schedule();
    }
    
    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_work_init(k_work_t *work, k_work_routine_t routine, void *context)
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

static int __attribute__((noinline)) __svc_armv7m_rtos_work_deinit(k_work_t *work)
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

static int __attribute__((noinline)) __svc_armv7m_rtos_alarm_init(k_alarm_t *alarm, k_alarm_routine_t routine, void *context)
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

static int __attribute__((noinline)) __svc_armv7m_rtos_alarm_deinit(k_alarm_t *alarm)
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
    
    __armv7m_atomic_store_2_restart((volatile uint32_t*)&alarm->clock_l, 0, 0);

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

static int __attribute__((noinline)) __svc_armv7m_rtos_alarm_absolute(k_alarm_t *alarm, uint32_t clock_l, uint32_t clock_h, uint32_t period)
{
    uint64_t clock;

    if (!alarm || !alarm->work.routine)
    {
        return K_ERR_INVALID_OBJECT;
    }
    
    if (!clock_l && !clock_h)
    {
        return K_ERR_INVALID_PARAMETER;
    }

    clock = (((uint64_t)clock_l << 0) | ((uint64_t)clock_h << 32));
    
    armv7m_rtos_alarm_modify(alarm, armv7m_rtos_clock_convert(clock), period);

    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_alarm_relative(k_alarm_t *alarm, uint32_t delay, uint32_t period)
{
    if (!alarm || !alarm->work.routine)
    {
        return K_ERR_INVALID_OBJECT;
    }

    if (!delay)
    {
        return K_ERR_INVALID_PARAMETER;
    }
    
    armv7m_rtos_alarm_modify(alarm, armv7m_rtos_clock_offset(stm32wb_rtc_clock_read(), delay), period);

    return K_NO_ERROR;
}

static int __attribute__((noinline)) __svc_armv7m_rtos_alarm_cancel(k_alarm_t *alarm)
{
    k_alarm_t *alarm_previous, *alarm_modify;
    uint32_t clock_l, clock_h;

    if (!alarm || !alarm->work.routine)
    {
        return K_ERR_INVALID_OBJECT;
    }

    __armv7m_atomic_load_4_restart((volatile uint32_t*)&alarm->previous, (uint32_t*)&alarm_previous, (uint32_t*)&alarm_modify, &clock_l, &clock_h);
    
    if (!(alarm_modify ? ((clock_l | clock_h) != 0) : (alarm_previous != NULL)))
    {
        return K_ERR_ALARM_NOT_ACTIVE;
    }

    armv7m_rtos_alarm_modify(alarm, 0, 0);

    return K_NO_ERROR;
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
    
    clock = stm32wb_rtc_clock_read();

    seconds = clock / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
    ticks = clock & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND -1);

    return ((uint64_t)(seconds * 1000) + ((ticks * 1000) / STM32WB_RTC_CLOCK_TICKS_PER_SECOND));
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

int k_task_yield(void)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_0((uint32_t)&__svc_armv7m_rtos_task_yield);
}


int k_event_send(k_task_t *task, uint32_t events)
{
    if (armv7m_core_is_in_interrupt())
    {
        return __svc_armv7m_rtos_event_send(task, events);
    }

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_event_send, (uint32_t)task, (uint32_t)events);
}

int k_event_receive(uint32_t events, uint32_t mode, uint32_t timeout, uint32_t *p_events_return)
{
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_4((uint32_t)&__svc_armv7m_rtos_event_receive, (uint32_t)events, (uint32_t)mode, (uint32_t)timeout, (uint32_t)p_events_return);
}


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


int k_mutex_init(k_mutex_t *mutex, uint32_t priority, uint32_t options)
{
    // armv7m_rtt_printf("k_mutex_init(mutex=%08x, priority=%d, options=%08x)\n", mutex, priority, options);
    
    if (armv7m_core_is_in_interrupt())
    {
        return K_ERR_ILLEGAL_CONTEXT;
    }

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtos_mutex_init, (uint32_t)mutex, (uint32_t)options);
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
    if (!work || !work->routine)
    {
        return K_ERR_INVALID_OBJECT;
    }

    return armv7m_rtos_work_submit(work);
}

k_work_t * __attribute__((optimize("O3"))) k_work_self(void)
{
    return armv7m_rtos_control.work_self;

}

bool __attribute__((optimize("O3"))) k_work_is_in_progress(void)
{
    return (armv7m_core_is_in_thread() && armv7m_rtos_control.work_self);
}


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
    k_alarm_t *alarm_previous, *alarm_modify;
    uint32_t clock_l, clock_h;

    if (!alarm || !alarm->work.routine)
    {
        return false;
    }
    
    __armv7m_atomic_load_4_restart((volatile uint32_t*)&alarm->previous, (uint32_t*)&alarm_previous, (uint32_t*)&alarm_modify, &clock_l, &clock_h);
    
    return (alarm_modify ? ((clock_l | clock_h) != 0) : (alarm_previous != NULL));
}

/******************************************************************************************************************************/

void RTOS_ALARM_SWIHandler(void)
{
    if (armv7m_rtos_control.alarm_routine)
    {
        (*armv7m_rtos_control.alarm_routine)();
    }
}

void RTOS_WORK_SWIHandler(void)
{
    armv7m_rtos_work_routine();
}

void RTOS_TASK_RESUME_SWIHandler(void)
{
    if (armv7m_rtos_control.resume_routine)
    {
        (*armv7m_rtos_control.resume_routine)();
    }
}

void RTOS_TASK_RELEASE_SWIHandler(void)
{
    if (armv7m_rtos_control.release_routine)
    {
        (*armv7m_rtos_control.release_routine)();
    }
}

/******************************************************************************************************************************/
