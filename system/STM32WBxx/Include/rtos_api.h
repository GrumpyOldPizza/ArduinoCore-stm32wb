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

#if !defined(_RTOS_API_H)
#define _RTOS_API_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _k_task_t  k_task_t;
typedef struct _k_event_t k_event_t;
typedef struct _k_sem_t   k_sem_t;
typedef struct _k_mutex_t k_mutex_t;
typedef struct _k_work_t  k_work_t;
typedef struct _k_alarm_t k_alarm_t;
  
typedef void (*k_task_routine_t)(void *context);

struct _k_task_t {
    k_task_t                     *next;
    k_task_t                     *previous;
    uint8_t                      priority;
    uint8_t                      bpriority;
    uint16_t                     state;
    volatile uint32_t            events;
  
    const char                   *name;
    k_task_t                     *link;

    uint32_t                     exc_return;
    void                         *stack_top;
    void                         *stack_end;
    void                         *stack_base;
    void                         *stack_limit;

    k_task_t                     *join;
    k_mutex_t                    *mutex;
    k_task_t * volatile          resume;
    k_task_t * volatile          wakeup;
  
    struct {
	k_task_t                     *next;
	k_task_t                     *previous;
	uint32_t                     clock_l;
	uint32_t                     clock_h;
    }                            timeout;

    union {
	struct {
	    k_task_t                     *task;
	}                            join;
	struct {
	    k_event_t                    *event;
	    uint32_t                     mask;
	    uint32_t                     *p_mask_return;
	}                            event;
	struct {
	    k_sem_t                      *sem;
	}                            sem;
	struct {
	    k_mutex_t                    *mutex;
	}                            mutex;
    }                            wait;
};

struct _k_event_t {
    volatile uint32_t            mask;
    k_event_t * volatile         send;
    k_task_t                     *waiting;
};

struct _k_sem_t {
    volatile uint16_t            count;
    uint16_t                     limit;
    k_sem_t * volatile           release;
    k_task_t                     *waiting;
};

struct _k_mutex_t {
    k_mutex_t                    *next;
    k_mutex_t                    *previous;
    uint8_t                      priority;
    uint8_t                      options;
    uint16_t                     level;
    k_task_t                     *owner;
    k_task_t                     *waiting;
};


typedef void (*k_work_routine_t)(void *context);
  
struct _k_work_t {
    k_work_t * volatile          next;
    k_work_routine_t             routine;
    void                         *context;
};


typedef void (*k_alarm_routine_t)(void *context);
  
struct _k_alarm_t {
    k_work_t                     work;
    k_alarm_t                    *next;
    k_alarm_t                    *previous;
    k_alarm_t * volatile         modify;
    volatile uint32_t            clock_l;
    volatile uint32_t            clock_h;
    volatile uint32_t            ticks;
    volatile uint32_t            delta;
};

#define K_NO_ERROR                   0
#define K_ERR_ILLEGAL_CONTEXT        1
#define K_ERR_ILLEGAL_USE            2
#define K_ERR_INVALID_OBJECT         3
#define K_ERR_INVALID_PARAMETER      4
#define K_ERR_TIMEOUT                5
#define K_ERR_UNSATISFIED            6
#define K_ERR_TASK_NOT_BLOCKED       7
#define K_ERR_TASK_ALREADY_DETACHED  8
#define K_ERR_TASK_ALREADY_JOINED    9
#define K_ERR_TASK_NOT_JOINABLE      10
#define K_ERR_TASK_ALREADY_SUSPENDED 11
#define K_ERR_TASK_NOT_SUSPENDED     12
#define K_ERR_SEM_OVERFLOW           13
#define K_ERR_NOT_OWNER_OF_MUTEX     14
#define K_ERR_MUTEX_ALREADY_LOCKED   15
#define K_ERR_MUTEX_NOT_LOCKED       16
#define K_ERR_MUTEX_OVERFLOW         17
#define K_ERR_WORK_ALREADY_SUBMITTED 18
#define K_ERR_ALARM_NOT_ACTIVE       19

#define K_STATE_INACTIVE             0
#define K_STATE_READY                1
#define K_STATE_RUNNING              2
#define K_STATE_LOCKED               3

#define K_POLICY_CURRENT             0
#define K_POLICY_RUN                 1
#define K_POLICY_SLEEP               2
#define K_POLICY_STOP                3
  
#define K_PRIORITY_CURRENT           0
#define K_PRIORITY_MAX               1
#define K_PRIORITY_MIN               255

#define K_STACK_SIZE_MIN             128
#define K_STACK_SIZE_DEFAULT         1024
  
#define K_TASK_JOINABLE              0x00000001
#define K_TASK_SUSPENDED             0x00000002

#define K_STATE_INACTIVE             0
#define K_STATE_READY                1
#define K_STATE_RUNNING              2
#define K_STATE_BLOCKED              3
#define K_STATE_SUSPENDED            4
#define K_STATE_TERMINATED           5
#define K_STATE_UNKNOWN              6

#define K_WAIT_NONE                  0
#define K_WAIT_JOIN                  1
#define K_WAIT_DELAY                 2
#define K_WAIT_SLEEP                 3
#define K_WAIT_EVENT                 4
#define K_WAIT_SEM                   5
#define K_WAIT_MUTEX                 6
  
#define K_EVENT_ANY                  0x00000000
#define K_EVENT_ALL                  0x00000001
#define K_EVENT_CLEAR                0x00000002

#define K_TIMEOUT_NONE               0x00000000
#define K_TIMEOUT_FOREVER            0xffffffff

#define K_MUTEX_RECURSIVE            0x00000001
#define K_MUTEX_PRIORITY_INHERIT     0x00000002
#define K_MUTEX_PRIORITY_PROTECT     0x00000004
  
#define K_SYSTEM_TICKS_PER_SECOND    32768
  
typedef struct _k_task_info_t {
    const char                  *name;
    uint8_t                     priority;
    uint8_t                     bpriority;
    uint8_t                     state;
    uint8_t                     wait;
} k_task_info_t;
  
typedef void (*k_task_create_hook_t)(k_task_t *task, const char *name, uint32_t priority, void *stack_base, uint32_t stack_size);
typedef void (*k_task_destroy_hook_t)(k_task_t *task);
typedef void (*k_task_exit_hook_t)(void);
typedef void (*k_task_terminate_hook_t)(k_task_t *task);
typedef void (*k_task_stack_hook_t)(k_task_t *task, void *stack_base, uint32_t stack_size);
typedef void (*k_task_block_hook_t)(k_task_t *task, uint32_t cause);
typedef void (*k_task_ready_hook_t)(k_task_t *task);
typedef void (*k_task_run_hook_t)(k_task_t *task);
  
typedef struct _k_hook_table_t {
    k_task_create_hook_t    task_create;
    k_task_destroy_hook_t   task_destroy;
    k_task_exit_hook_t      task_exit;
    k_task_terminate_hook_t task_terminate;
    k_task_block_hook_t     task_block;
    k_task_ready_hook_t     task_ready;
    k_task_run_hook_t       task_run;
} k_hook_table_t;

extern void * k_heap_allocate(uint32_t size);
  
extern uint64_t k_system_clock(void);
extern uint32_t k_system_state(void);
extern int k_system_initialize(const k_hook_table_t *hook_table);
extern int k_system_start(k_task_routine_t routine, void *context);
extern bool k_system_lock(void);
extern void k_system_unlock(void);
extern bool k_system_is_locked(void);
extern int k_system_set_policy(uint32_t policy, uint32_t *p_policy_return);
  
extern int k_task_create(k_task_t *task, const char *name, k_task_routine_t routine, void *context, uint32_t priority, void *stack_base, uint32_t stack_size, uint32_t options);
extern void k_task_exit(void);
extern int k_task_terminate(k_task_t *task);
extern int k_task_detach(k_task_t *task);
extern int k_task_join(k_task_t *task);
extern bool k_task_is_joinable(k_task_t *task);
extern k_task_t * k_task_self(void);
extern k_task_t * k_task_default(void);
extern bool k_task_is_in_progress(void);
extern int k_task_enumerate(uint32_t *p_count_return, k_task_t **p_task_return, uint32_t count);
extern int k_task_info(k_task_t *task, k_task_info_t *p_task_info_return);
extern int k_task_stack(k_task_t *task, uint32_t *p_stack_base_return, uint32_t *p_stack_size_return, uint32_t *p_stack_space_return);
extern int k_task_unblock(k_task_t *task);
extern int k_task_suspend(k_task_t *task);
extern int k_task_resume(k_task_t *task);
extern bool k_task_is_suspended(k_task_t *task);
extern int k_task_set_priority(k_task_t *task, uint32_t priority, uint32_t *p_priority_return);
extern int k_task_get_priority(k_task_t *task, uint32_t *p_priority_return);
extern int k_task_delay(uint32_t ticks);
extern int k_task_delay_until(uint64_t clock);
extern int k_task_sleep(uint32_t timeout);
extern int k_task_wakeup(k_task_t *task);
extern int k_task_yield(void);

#define K_EVENT_INIT()                          \
(k_event_t)                                     \
{                                               \
    .mask = 0,                                  \
    .send = NULL,                               \
    .waiting = NULL                             \
}

extern int k_event_init(k_event_t *event);
extern int k_event_deinit(k_event_t *event);
extern int k_event_send(k_event_t *event, uint32_t mask);
extern int k_event_receive(k_event_t *event, uint32_t mask, uint32_t mode, uint32_t timeout, uint32_t *p_mask_return);
extern uint32_t k_event_mask(k_event_t *event);

#define K_SEM_INIT(_count, _limit)              \
(k_sem_t)                                       \
{                                               \
    .count = (uint16_t)(_count),                \
    .limit = (uint16_t)(_limit),                \
    .release = NULL,                            \
    .waiting = NULL                             \
}

extern int k_sem_init(k_sem_t *sem, uint32_t count, uint32_t limit);
extern int k_sem_deinit(k_sem_t *sem);
extern int k_sem_acquire(k_sem_t *sem, uint32_t timeout);
extern int k_sem_release(k_sem_t *sem);
extern uint32_t k_sem_count(k_sem_t *sem);

#define K_MUTEX_INIT(_priority, _options)       \
(k_mutex_t)                                     \
{                                               \
    .next = NULL,                               \
    .previous = NULL,                           \
    .priority = (uint8_t)(_priority),           \
    .options = (uint8_t)(_options),             \
    .level = 0,                                 \
    .owner = NULL,                              \
    .waiting = NULL                             \
}

extern int k_mutex_init(k_mutex_t *mutex, uint32_t priority, uint32_t options);
extern int k_mutex_deinit(k_mutex_t *mutex);
extern int k_mutex_set_priority(k_mutex_t *mutex, uint32_t priority, uint32_t *p_priority_return);
extern int k_mutex_lock(k_mutex_t *mutex, uint32_t timeout);
extern int k_mutex_unlock(k_mutex_t *mutex);
extern uint32_t k_mutex_level(k_mutex_t *mutex);
extern k_task_t * k_mutex_owner(k_mutex_t *mutex);

#define K_WORK_INIT(_routine,_context)          \
(k_work_t)                                      \
{                                               \
    .next = NULL,                               \
    .routine = (k_work_routine_t)(_routine),    \
    .context = (void*)(_context)                \
}

extern int k_work_init(k_work_t *work, k_work_routine_t routine, void *context);
extern int k_work_deinit(k_work_t *work);
extern int k_work_submit(k_work_t *work);
extern k_work_t * k_work_self(void);
extern bool k_work_is_in_progress(void);

#define K_ALARM_INIT(_routine,_context)         \
(k_alarm_t)                                     \
{                                               \
    .work = K_WORK_INIT(_routine, _context),    \
    .next = NULL,                               \
    .previous = NULL,                           \
    .modify = NULL,                             \
    .clock_l = 0,                               \
    .clock_h = 0,                               \
    .ticks = 0,                                 \
    .delta = 0                                  \
}
  
extern int k_alarm_init(k_alarm_t *alarm, k_alarm_routine_t routine, void *context);
extern int k_alarm_deinit(k_alarm_t *alarm);
extern int k_alarm_absolute(k_alarm_t *alarm, uint64_t clock, uint32_t period);
extern int k_alarm_relative(k_alarm_t *alarm, uint32_t delay, uint32_t period);
extern int k_alarm_cancel(k_alarm_t *alarm);
extern bool k_alarm_is_active(k_alarm_t *alarm);
  
#ifdef __cplusplus
}
#endif

#endif /* _RTOS_API_H */
