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

#if !defined(_ARMV7M_RTOS_H)
#define _ARMV7M_RTOS_H

#ifdef __cplusplus
extern "C" {
#endif

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

#if defined (__VFP_FP__) && !defined(__SOFTFP__)

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

#endif /* __VFP_FP__ && !__SOFTFP__ */
  
typedef void (*k_task_routine_t)(void *context, uint32_t argument);

typedef struct _k_task_t {
    struct _k_task_t            *next;
    struct _k_task_t            *previous;
    uint8_t                     priority;
    uint8_t                     mode;
    uint16_t                    state;
    volatile uint32_t           events;
  
    const char                  *name;
    uint8_t                     bpriority;
    uint8_t                     ipriority;
    uint8_t                     imode;

    int8_t                      exc_return;
    void                        *stack_current;
    void                        *stack_limit;
    void                        *stack_top;

    k_task_routine_t            routine;
    void                        *context;

    struct _k_mutex_t           *mutex;

    struct _k_task_t * volatile created;
    struct _k_task_t * volatile suspend;
    struct _k_task_t * volatile resume;
    struct _k_task_t * volatile release;
  
    struct {
	struct _k_task_t            *next;
	struct _k_task_t            *previous;
	uint32_t                    ticks;
    }                           timeout;

    union {
	struct {
	    uint32_t                    events;
	    uint32_t                    *p_events_return;
	}                           event;
	struct {
	    struct _k_sem_t             *sem;
	}                           sem;
	struct {
	    struct _k_mutex_t           *mutex;
	}                           mutex;
    }                           wait;
} k_task_t;
  
typedef struct _k_sem_t {
    volatile uint32_t           count;
    struct {
	k_task_t                    *head;
	k_task_t                    *tail;
	k_task_t * volatile         release;
    }                           waiting;
} k_sem_t;

typedef struct _k_mutex_t {
    struct _k_mutex_t           *next;
    struct _k_mutex_t           *previous;
    uint8_t                     priority;
    uint8_t                     mode;
    uint16_t                    level;
    k_task_t                    *owner;
    k_task_t                    *waiting;
} k_mutex_t;

typedef void (*k_work_routine_t)(void *context);
  
typedef struct _k_work_t {
    struct _k_work_t * volatile next;
    k_work_routine_t            routine;
    void                        *context;
} k_work_t;



#define K_TASK_SELF                  ((k_task_t*)NULL)

#define K_NO_ERROR                   0
#define K_ERR_ILLEGAL_USE            1   // context
#define K_ERR_INVALID_ADDRESS        2
#define K_ERR_INVALID_COUNT          3
#define K_ERR_INVALID_SIZE           4
#define K_ERR_INVALID_MODE           5
#define K_ERR_INVALID_OPTIONS        6
#define K_ERR_INVALID_PARAMETER      7
#define K_ERR_INVALID_PRIORITY       8
#define K_ERR_INVALID_STATE          9
#define K_ERR_INVALID_OBJECT         10
#define K_ERR_NOT_DEFINED            11
#define K_ERR_OBJECT_DELETED         12
#define K_ERR_OBJECT_PROTECTED       13
#define K_ERR_TASK_ALREADY_STARTED   14
#define K_ERR_TASK_NOT_STARTED       15
#define K_ERR_TASK_ALREADY_SUSPENDED 16
#define K_ERR_TASK_NOT_SUSPENDED     17
#define K_ERR_TIMEOUT                18
#define K_ERR_UNSATISFIED            19
#define K_ERR_NOT_OWNER_OF_MUTEX     20
#define K_ERR_MUTEX_ALREADY_LOCKED   21
#define K_ERR_MUTEX_NOT_LOCKED       22
#define K_ERR_MUTEX_IN_USE           23
  
#define K_PRIORITY_CURRENT           0
#define K_PRIORITY_MAX               1
#define K_PRIORITY_MIN               255

#define K_STATE_RUNNING              0x00000001
#define K_STATE_READY                0x00000002
#define K_STATE_BLOCKED              0x00000004
#define K_STATE_SUSPENDED            0x00000008

#define K_MODE_NOPREEMPT             0x00000001
#define K_MODE_NOCONCURRENT          0x00000002
#define K_MODE_NOTERMINATE           0x00000004

#define K_EVENT_ALL                  0x00000000
#define K_EVENT_ANY                  0x00000001
  
#define K_TIMEOUT_NONE               0x00000000
#define K_TIMEOUT_FOREVER            0xffffffff

#define K_CLOCK_TICKS_PER_SECOND     32768
  
typedef struct _k_task_info_t {
    uint8_t                     priority;
    uint8_t                     bpriority;
    uint8_t                     mode;
    uint8_t                     state;
    uint32_t                    events;
    uint32_t                    stack_current;
    uint32_t                    stack_limit;
    uint32_t                    stack_size;
} k_task_info_t;
  
extern void __armv7m_rtos_initialize(void);

extern int k_task_create(k_task_t *task, const char *name, uint32_t priority, void *stack_base, uint32_t stack_size, uint32_t mode);
extern int k_task_destroy(k_task_t *task);
extern int k_task_info(k_task_t *task, k_task_info_t *p_task_info_return);
extern int k_task_start(k_task_t *task, k_task_routine_t routine, void *context, uint32_t argument);
extern int k_task_restart(k_task_t *task, uint32_t argument);
extern k_task_t* k_task_self(void);
extern k_task_t* k_task_default(void);
extern int k_task_suspend(k_task_t *task);
extern int k_task_resume(k_task_t *task);
extern bool k_task_is_suspended(k_task_t *task);
extern int k_task_set_priority(k_task_t *task, uint32_t priority, uint32_t *p_priority_return);
extern int k_task_set_mode(uint32_t mode, uint32_t mask, uint32_t *p_mode_return);
extern int k_task_delay(uint32_t delay);

extern int k_event_send(k_task_t *task, uint32_t events);
extern int k_event_receive(uint32_t events, uint32_t mode, uint32_t timeout, uint32_t *p_events_return);

extern int k_sem_create(k_sem_t *sem, uint32_t count);
extern int k_sem_destroy(k_sem_t *sem);
extern int k_sem_acquire(k_sem_t *sem, uint32_t timeout);
extern int k_sem_release(k_sem_t *sem);

extern int k_mutex_create(k_mutex_t *mutex);
extern int k_mutex_destroy(k_mutex_t *mutex);
extern int k_mutex_trylock(k_mutex_t *mutex);
extern int k_mutex_lock(k_mutex_t *mutex);
extern int k_mutex_unlock(k_mutex_t *mutex);
  
extern void k_work_create(k_work_t *work, k_work_routine_t routine, void *context);
extern int  k_work_destroy(k_work_t *work);
extern bool k_work_submit(k_work_t *work);
extern bool k_work_is_in_progress(void);

static inline uint32_t k_clock_ticks_per_second(void)
{
    return K_CLOCK_TICKS_PER_SECOND;
}
  
static inline uint32_t k_clock_micros_to_ticks(uint32_t micros)
{
    if (micros < 1000000)
    {
        return ((micros * K_CLOCK_TICKS_PER_SECOND + 999999) / 1000000);
    }
    else
    {   
        uint32_t seconds;

        seconds = (micros / 1000000);
        micros = micros - seconds * 1000000;

        return (seconds * K_CLOCK_TICKS_PER_SECOND) + ((micros * K_CLOCK_TICKS_PER_SECOND + 999999) / 1000000);
    }
}

static inline uint32_t k_clock_millis_to_ticks(uint32_t millis)
{
    if (millis < 60000)
    {
        return ((millis * K_CLOCK_TICKS_PER_SECOND + 999) / 1000);
    }
    else
    {
        uint32_t seconds;

        seconds = (millis / 1000);
        millis = millis - seconds * 1000;

        return (seconds * K_CLOCK_TICKS_PER_SECOND) + ((millis * K_CLOCK_TICKS_PER_SECOND + 999) / 1000);
    }
}

static inline uint32_t k_clock_seconds_to_ticks(uint32_t seconds)
{
    return (seconds * K_CLOCK_TICKS_PER_SECOND);
}
  
#ifdef __cplusplus
}
#endif

#endif /* _ARMV7M_RTOS_H */
