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

#if !defined(_ARMV7M_RTOS_H)
#define _ARMV7M_RTOS_H

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

typedef struct _k_task_t    k_task_t;
typedef struct _k_mutex_t   k_mutex_t;
typedef struct _k_wait_t    k_wait_t;
typedef struct _k_notify_t  k_notify_t;
typedef struct _k_sem_t     k_sem_t;
typedef struct _k_queue_t   k_queue_t;
typedef struct _k_work_t    k_work_t;
typedef struct _k_alarm_t   k_alarm_t;
  
typedef void (*k_task_routine_t)(void *context);

struct _k_task_t {
    k_task_t                     *next;
    k_task_t                     *previous;
    uint8_t                      priority;
    uint8_t                      bpriority;
    uint16_t                     state;
    volatile uint32_t            events;
  
    const char                   *name;
    k_task_t                     *list;

    uint8_t                      rfu[2];
    uint8_t                      control;
    uint8_t                      exc_return;
    void                         *stack_top;
    void                         *stack_base;
    void                         *stack_limit;

    k_task_t                     *join;
    k_mutex_t                    *mutex;
    k_task_t * volatile          resume;
    k_task_t * volatile          release;
  
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
	    uint32_t                     events;
	    uint32_t                     *p_events_return;
	}                            event;
	struct {
	    k_mutex_t                    *mutex;
	}                            mutex;
	struct {
	    k_sem_t                      *sem;
	}                            sem;
	struct {
	    k_queue_t                    *queue;
	    void                         *p_data_return;
	}                            queue;
    }                            wait;
};
  
struct _k_mutex_t {
    k_task_t                     *waiting;
    k_mutex_t                    *next;
    k_mutex_t                    *previous;
    uint8_t                      priority;
    uint8_t                      options;
    uint16_t                     level;
    k_task_t                     *owner;
};

struct _k_wait_t {
    k_task_t                     *head;
    k_task_t                     *tail;
    k_task_t * volatile          release;
};

struct _k_notify_t {
    k_task_t * volatile          task;
    uint32_t                     events;
};

struct _k_sem_t {
    k_wait_t                     waiting;
    k_notify_t                   notify;
    volatile uint16_t            count;
    uint16_t                     limit;
};

struct _k_queue_t {
    k_wait_t                     waiting;
    k_notify_t                   notify;
    volatile uint16_t            count;
    uint16_t                     size;
    void                         *base;
    uint32_t                     limit;
    volatile uint32_t            head;
    volatile uint32_t            tail;
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
    volatile uint32_t            period;
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
#define K_ERR_NOT_OWNER_OF_MUTEX     13
#define K_ERR_MUTEX_OWNER_DESTROYED  14
#define K_ERR_MUTEX_ALREADY_LOCKED   15
#define K_ERR_MUTEX_NOT_LOCKED       16
#define K_ERR_MUTEX_OVERFLOW         17
#define K_ERR_SEM_OVERFLOW           18
#define K_ERR_QUEUE_OVERFLOW         19
#define K_ERR_WORK_ALREADY_SUBMITTED 20
#define K_ERR_ALARM_NOT_ACTIVE       21

#define K_STATE_INACTIVE             0
#define K_STATE_READY                1
#define K_STATE_RUNNING              2
#define K_STATE_LOCKED               3
  
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
#define K_WAIT_DELAY                 1
#define K_WAIT_JOIN                  2
#define K_WAIT_EVENT                 3
#define K_WAIT_SEM                   4
#define K_WAIT_MUTEX                 5
#define K_WAIT_QUEUE                 6
  
#define K_EVENT_ANY                  0x00000000
#define K_EVENT_ALL                  0x00000001
#define K_EVENT_CLEAR                0x00000002

#define K_TIMEOUT_NONE               0x00000000
#define K_TIMEOUT_FOREVER            0xffffffff

#define K_MUTEX_RECURSIVE            0x00000001
#define K_MUTEX_PRIORITY_INHERIT     0x00000002
#define K_MUTEX_PRIORITY_PROTECT     0x00000004
#define K_MUTEX_ROBUST               0x00000008

typedef struct _k_task_info_t {
    const char                  *name;
    uint8_t                     priority;
    uint8_t                     bpriority;
    uint8_t                     state;
    uint8_t                     wait;
    uint32_t                    events;
} k_task_info_t;
  
typedef void (*k_task_create_hook_t)(k_task_t *task, const char *name, uint32_t priority, void *stack_base, uint32_t stack_size);
typedef void (*k_task_destroy_hook_t)(k_task_t *task);
typedef void (*k_task_block_hook_t)(k_task_t *task, uint32_t cause);
typedef void (*k_task_ready_hook_t)(k_task_t *task);
typedef void (*k_task_run_hook_t)(k_task_t *task);
  
typedef struct _k_hook_table_t {
    k_task_create_hook_t    task_create;
    k_task_destroy_hook_t   task_destroy;
    k_task_block_hook_t     task_block;
    k_task_ready_hook_t     task_ready;
    k_task_run_hook_t       task_run;
} k_hook_table_t;

typedef struct _k_system_info_t {
    uint32_t                system_state;
    k_task_t                *task_self;
} k_system_info_t;

extern k_system_info_t __k_system_info;
    
extern void * k_heap_allocate(uint32_t size);
  
extern uint64_t k_system_clock(void);
extern uint32_t k_system_state(void);
extern int k_system_initialize(const k_hook_table_t *hook_table);
extern int k_system_start(k_task_routine_t routine, void *context);
extern bool k_system_lock(void);
extern void k_system_unlock(void);
extern bool k_system_is_locked(void);

#define K_TASK_SELF()  (__k_system_info.task_self)
  
extern int k_task_create(k_task_t *task, const char *name, k_task_routine_t routine, void *context, uint32_t priority, void *stack_base, uint32_t stack_size, uint32_t options);
extern void k_task_exit(void);
extern int k_task_terminate(k_task_t *task);
extern int k_task_detach(k_task_t *task);
extern int k_task_join(k_task_t *task);
extern bool k_task_is_joinable(k_task_t *task);
extern k_task_t * k_task_self(void);
extern int k_task_enumerate(uint32_t *p_count_return, k_task_t **p_task_return, uint32_t count);
extern int k_task_info(k_task_t *task, k_task_info_t *p_task_info_return);
extern int k_task_stack(k_task_t *task, uint32_t *p_stack_size_return, uint32_t *p_stack_space_return);
extern int k_task_unblock(k_task_t *task);
extern int k_task_suspend(k_task_t *task);
extern int k_task_resume(k_task_t *task);
extern bool k_task_is_suspended(k_task_t *task);
extern int k_task_set_priority(k_task_t *task, uint32_t priority, uint32_t *p_priority_return);
extern int k_task_get_priority(k_task_t *task, uint32_t *p_priority_return);
extern int k_task_delay(uint32_t ticks);
extern int k_task_delay_until(uint64_t clock);
extern int k_task_yield(void);

extern int k_event_send(k_task_t *task, uint32_t events);
extern int k_event_receive(uint32_t events, uint32_t mode, uint32_t timeout, uint32_t *p_events_return);


#define K_MUTEX_INIT(_priority, _options)              \
(k_mutex_t)                                            \
{                                                      \
    .waiting = NULL,                                   \
    .next = NULL,                                      \
    .previous = NULL,                                  \
    .priority = (uint8_t)(_priority),                  \
    .options = (uint8_t)(_options),                    \
    .level = 0,                                        \
    .owner = NULL                                      \
}

#define K_MUTEX_OWNER(_mutex)                          \
  ((_mutex)->owner)
  
extern int k_mutex_init(k_mutex_t *mutex, uint32_t priority, uint32_t options);
extern int k_mutex_set_priority(k_mutex_t *mutex, uint32_t priority, uint32_t *p_priority_return);
extern int k_mutex_consistent(k_mutex_t *mutex);
extern int k_mutex_lock(k_mutex_t *mutex, uint32_t timeout);
extern int k_mutex_unlock(k_mutex_t *mutex);
extern k_task_t * k_mutex_owner(k_mutex_t *mutex);


#define K_SEM_INIT(_count, _limit)                     \
(k_sem_t)                                              \
{                                                      \
    .waiting = { NULL, NULL, NULL },                   \
    .notify = { NULL, 0 },                             \
    .count = (uint16_t)(_count),                       \
    .limit = (uint16_t)(_limit),                       \
}

extern int k_sem_init(k_sem_t *sem, uint32_t count, uint32_t limit);
extern int k_sem_notify(k_sem_t *sem, uint32_t events);
extern int k_sem_acquire(k_sem_t *sem, uint32_t timeout);
extern int k_sem_release(k_sem_t *sem);
extern uint32_t k_sem_count(k_sem_t *sem);


#define K_QUEUE_INIT(_base, _count, _size)             \
(k_queue_t)                                            \
{                                                      \
    .waiting = { NULL, NULL, NULL },                   \
    .notify = { NULL, 0 },                             \
    .count = 0,                                        \
    .size = (uint32_t)(_size),                         \
    .base  = (void*)(_base),                           \
    .limit = ((uint32_t)(_count) * (uint32_t)(_size)), \
    .head = 0,                                         \
    .tail = 0                                          \
}
  
extern int k_queue_init(k_queue_t *queue, void *base, uint32_t count, uint32_t size);
extern int k_queue_notify(k_queue_t *queue, uint32_t events);
extern int k_queue_send(k_queue_t *queue, const void *data);
extern int k_queue_urgent(k_queue_t *queue, const void *data);
extern int k_queue_receive(k_queue_t *queue, void *p_data_return, uint32_t timeout);
extern int k_queue_flush(k_queue_t *queue, uint32_t *p_count_return);

#define K_WORK_INIT(_routine,_context)                 \
(k_work_t)                                             \
{                                                      \
    .next = NULL,                                      \
    .routine = (k_work_routine_t)(_routine),           \
    .context = (void*)(_context)                       \
}

extern int k_work_init(k_work_t *work, k_work_routine_t routine, void *context);
extern int k_work_submit(k_work_t *work);

#define K_ALARM_INIT(_routine,_context)                \
(k_alarm_t)                                            \
{                                                      \
    .work = K_WORK_INIT(_routine, _context),           \
    .next = NULL,                                      \
    .previous = NULL,                                  \
    .modify = NULL,                                    \
    .clock_l = 0,                                      \
    .clock_h = 0,                                      \
    .period = 0                                        \
}
  
extern int k_alarm_init(k_alarm_t *alarm, k_alarm_routine_t routine, void *context);
extern int k_alarm_absolute(k_alarm_t *alarm, uint64_t clock, uint32_t period);
extern int k_alarm_relative(k_alarm_t *alarm, uint32_t delay, uint32_t period);
extern int k_alarm_cancel(k_alarm_t *alarm);
extern bool k_alarm_is_active(k_alarm_t *alarm);
  
#ifdef __cplusplus
}
#endif

#endif /* _ARMV7M_RTOS_H */
