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

#if !defined(RTOS_CORE_H)
#define _RTOS_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#define ARMV7M_RTOS_MPU_SUPPORTED   1
#define ARMV7M_RTOS_HOOK_SUPPORTED  0
#define ARMV7M_RTOS_DEBUG_SUPPORTED 1
  
typedef struct _armv7m_context_t {
    uint32_t            r4;
    uint32_t            r5;
    uint32_t            r6;
    uint32_t            r7;
    uint32_t            r8;
    uint32_t            r9;
    uint32_t            r10;
    uint32_t            r11;
    uint32_t            r0;
    uint32_t            r1;
    uint32_t            r2;
    uint32_t            r3;
    uint32_t            r12;
    uint32_t            lr;
    uint32_t            pc;
    uint32_t            xpsr;
} armv7m_context_t;

#if (__FPU_PRESENT == 1)

typedef struct _armv7m_context_fpu_t {
    uint64_t            d8;
    uint64_t            d9;
    uint64_t            d10;
    uint64_t            d11;
    uint64_t            d12;
    uint64_t            d13;
    uint64_t            d14;
    uint64_t            d15;
    uint32_t            r4;
    uint32_t            r5;
    uint32_t            r6;
    uint32_t            r7;
    uint32_t            r8;
    uint32_t            r9;
    uint32_t            r10;
    uint32_t            r11;
    uint32_t            r0;
    uint32_t            r1;
    uint32_t            r2;
    uint32_t            r3;
    uint32_t            r12;
    uint32_t            lr;
    uint32_t            pc;
    uint32_t            xpsr;
    uint64_t            d0;
    uint64_t            d1;
    uint64_t            d2;
    uint64_t            d3;
    uint64_t            d4;
    uint64_t            d5;
    uint64_t            d6;
    uint64_t            d7;
    uint32_t            fpscr;
    uint32_t            reserved;
} armv7m_context_fpu_t;
  
#endif /* __FPU_PRESENT == 1 */

typedef struct armv7m_rtos_control_t {
    k_task_t                *task_self;
    k_task_t                *task_next;
    k_task_t                *task_ready;
    k_task_t                *task_default;
    k_task_t                *task_link;
    k_task_t * volatile     task_resume;
    k_task_t * volatile     task_wakeup;
    k_task_t                *task_timeout;
    uint64_t                task_clock;
    stm32wb_lptim_timeout_t task_timer;
    k_event_t * volatile    event_send;
    k_sem_t * volatile      sem_release;
    k_work_t *              work_self;
    k_work_t *              work_head;
    k_work_t *              work_tail;
    k_work_t * volatile     work_submit;
    k_alarm_t               *alarm_queue;
    k_alarm_t * volatile    alarm_modify;
    uint64_t                alarm_clock;
    stm32wb_lptim_timeout_t alarm_timer;
    volatile uint8_t        system_state;
    volatile uint8_t        system_policy;
    void                    *heap_current;
#if (ARMV7M_RTOS_HOOK_SUPPORTED == 1)
    const k_hook_table_t    *hook_table;
#endif /* ARMV7M_RTOS_HOOK_SUPPORTED == 1 */
} armv7m_rtos_control_t;

extern armv7m_rtos_control_t armv7m_rtos_control;

#define K_TASK_STATE_WAIT_MASK       0x000f
#define K_TASK_STATE_WAIT_NONE       0x0000
#define K_TASK_STATE_WAIT_JOIN       0x0001
#define K_TASK_STATE_WAIT_DELAY      0x0002
#define K_TASK_STATE_WAIT_SLEEP      0x0003
#define K_TASK_STATE_WAIT_EVENT      0x0004
#define K_TASK_STATE_WAIT_SEM        0x0005
#define K_TASK_STATE_WAIT_MUTEX      0x0006
#define K_TASK_STATE_TIMEOUT         0x0010
#define K_TASK_STATE_SUSPENDED       0x0020
#define K_TASK_STATE_TERMINATED      0x0040
#define K_TASK_STATE_READY           0x0080
#define K_TASK_STATE_JOINABLE        0x0100
#define K_TASK_STATE_WAKEUP          0x0200
#define K_TASK_STATE_EVENT_ALL       0x0400
#define K_TASK_STATE_EVENT_CLEAR     0x0800
#define K_TASK_STATE_AFFINITY_MASK   0xf000
#define K_TASK_STATE_AFFINITY_SHIFT  12

#define K_TASK_STATE_CAUSE_MASK      0x007f

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
#define K_TASK_WORK                  ((k_task_t*)0xf0000000)

#define K_TASK_SENTINEL              ((k_task_t*)0x00000001)
#define K_EVENT_SENTINEL             ((k_event_t*)0x00000001)
#define K_SEM_SENTINEL               ((k_sem_t*)0x00000001)
#define K_WORK_SENTINEL              ((k_work_t*)0x00000001)
#define K_ALARM_SENTINEL             ((k_alarm_t*)0x00000001)

extern void armv7m_rtos_pendsv_epilogue(void);

extern void armv7m_rtos_task_queue_insert(k_task_t **p_task_head, k_task_t *task);
extern void armv7m_rtos_task_queue_remove(k_task_t **p_task_head, k_task_t *task);
extern void armv7m_rtos_task_ready_insert(k_task_t *task);
extern void armv7m_rtos_task_ready_remove(k_task_t *task);
  
extern void armv7m_rtos_task_start(k_task_t *task, k_task_routine_t routine, void *context);
extern void armv7m_rtos_task_exit(void) __attribute__((noreturn));
extern void armv7m_rtos_task_create(k_task_t *task, const char *name, uint32_t priority, void *stack_base, uint32_t stack_size, uint32_t options);
extern void armv7m_rtos_task_destroy(k_task_t *task);
extern void armv7m_rtos_task_schedule(void);
extern void armv7m_rtos_task_priority(k_task_t *task);
extern void armv7m_rtos_task_release(k_task_t *task);
extern void armv7m_rtos_task_unblock(k_task_t *task, uint32_t status);

extern void armv7m_rtos_task_timeout_remove(k_task_t *task);
extern void armv7m_rtos_task_timeout_absolute(k_task_t *task, uint64_t clock);
extern void armv7m_rtos_task_timeout_relative(k_task_t *task, uint32_t delay);
extern void armv7m_rtos_task_timeout_schedule(void);

extern void armv7m_rtos_join_release(k_task_t *task);
  
extern void armv7m_rtos_join_unblock(k_task_t *task);
extern void armv7m_rtos_delay_unblock(k_task_t *task);
extern void armv7m_rtos_sleep_unblock(k_task_t *task);
extern void armv7m_rtos_event_unblock(k_task_t *task);
extern void armv7m_rtos_sem_unblock(k_task_t *task);
extern void armv7m_rtos_mutex_unblock(k_task_t *task);

extern k_task_t * armv7m_rtos_mutex_priority(k_task_t *task);
extern void armv7m_rtos_mutex_destroy(k_task_t *task);
  
extern int armv7m_rtos_work_deinit(k_work_t *work);
extern int armv7m_rtos_work_submit(k_work_t *work);
extern void armv7m_rtos_work_return(void);
extern void armv7m_rtos_work_schedule(void);
  
#ifdef __cplusplus
}
#endif

#endif /* _RTOS_CORE_H */
