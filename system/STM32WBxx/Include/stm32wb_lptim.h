/*
 * Copyright (c) 2017-2020 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32WB_LPTIM_H)
#define _STM32WB_LPTIM_H

#include "armv7m.h"
#include "stm32wbxx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32WB_LPTIM1_IRQ_PRIORITY   ARMV7M_IRQ_PRIORITY_MEDIUM

#define STM32WB_LPTIM_EVENT_COMPARE                           0x00000001
#define STM32WB_LPTIM_EVENT_PERIOD                            0x00000002

#define STM32WB_LPTIM_CONTROL_COMPARE                         0x00000001
#define STM32WB_LPTIM_CONTROL_PERIOD                          0x00000002  
#define STM32WB_LPTIM_CONTROL_PRESCALE_MASK                   0x0000001c
#define STM32WB_LPTIM_CONTROL_PRESCALE_SHIFT                  2
#define STM32WB_LPTIM_CONTROL_PRESCALE_1                      0x00000000
#define STM32WB_LPTIM_CONTROL_PRESCALE_2                      0x00000004
#define STM32WB_LPTIM_CONTROL_PRESCALE_4                      0x00000008
#define STM32WB_LPTIM_CONTROL_PRESCALE_8                      0x0000000c
#define STM32WB_LPTIM_CONTROL_PRESCALE_16                     0x00000010
#define STM32WB_LPTIM_CONTROL_PRESCALE_32                     0x00000014
#define STM32WB_LPTIM_CONTROL_PRESCALE_64                     0x00000018
#define STM32WB_LPTIM_CONTROL_PRESCALE_128                    0x0000001c
  
  
typedef void (*stm32wb_lptim_event_callback_t)(void *context, uint32_t events);
  
typedef struct _stm32wb_lptim_timeout_t stm32wb_lptim_timeout_t;

typedef void (*stm32wb_lptim_timeout_callback_t)(stm32wb_lptim_timeout_t *timeout);

struct _stm32wb_lptim_timeout_t {
    stm32wb_lptim_timeout_t                   *next;
    stm32wb_lptim_timeout_t                   *previous;
    stm32wb_lptim_timeout_t * volatile        modify;
    volatile uint32_t                         clock;
    volatile uint32_t                         ticks;
    volatile stm32wb_lptim_timeout_callback_t callback;
};

#define STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND 32768

#define STM32WB_LPTIM_TIMEOUT_INIT(_timeout)         \
{                                                    \
    (_timeout).next = NULL;                          \
    (_timeout).previous = NULL;                      \
    (_timeout).modify = NULL;                        \
    (_timeout).clock = 0;                            \
    (_timeout).ticks = 0;                            \
    (_timeout).callback = NULL;                      \
}

extern void __stm32wb_lptim_initialize(void);

extern bool stm32wb_lptim_event_lock(void);
extern void stm32wb_lptim_event_unlock(void);
extern bool stm32wb_lptim_event_start(uint32_t compare, uint32_t period, uint32_t control, uint8_t priority, stm32wb_lptim_event_callback_t callback, void *context);
extern bool stm32wb_lptim_event_stop(void);
extern bool stm32wb_lptim_event_restart(uint32_t period);
extern bool stm32wb_lptim_event_compare(uint32_t compare);
  
extern void stm32wb_lptim_timeout_create(stm32wb_lptim_timeout_t *timeout);
extern void stm32wb_lptim_timeout_destroy(stm32wb_lptim_timeout_t *timeout);
extern void stm32wb_lptim_timeout_start(stm32wb_lptim_timeout_t *timeout, uint32_t ticks, stm32wb_lptim_timeout_callback_t callback);
extern void stm32wb_lptim_timeout_restart(stm32wb_lptim_timeout_t *timeout, uint32_t ticks, stm32wb_lptim_timeout_callback_t callback);
extern void stm32wb_lptim_timeout_stop(stm32wb_lptim_timeout_t *timeout);
extern bool stm32wb_lptim_timeout_done(stm32wb_lptim_timeout_t *timeout);

static inline uint32_t stm32wb_lptim_timeout_micros_to_ticks(uint32_t micros)
{
    if (micros < 1000000)
    {
        return ((micros * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND + 999999) / 1000000);
    }
    else
    {   
        uint32_t seconds;

        seconds = (micros / 1000000);
        micros = micros - seconds * 1000000;

        return (seconds * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND) + ((micros * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND + 999999) / 1000000);
    }
}

static inline uint32_t stm32wb_lptim_timeout_millis_to_ticks(uint32_t millis)
{
    if (millis < 60000)
    {
        return ((millis * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND + 999) / 1000);
    }
    else
    {
        uint32_t seconds;

        seconds = (millis / 1000);
        millis = millis - seconds * 1000;

        return (seconds * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND) + ((millis * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND + 999) / 1000);
    }
}

static inline uint32_t stm32wb_lptim_timeout_seconds_to_ticks(uint32_t seconds)
{
    return (seconds * STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND);
}

static inline uint32_t stm32wb_lptim_timeout_ticks_to_micros(uint32_t ticks)
{
    uint32_t seconds;

    seconds = ticks / STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND;
    ticks = ticks & (STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND -1);

    return (seconds * 1000000) + ((ticks * 10000000) / STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND);
}

static inline uint32_t stm32wb_lptim_timeout_ticks_to_millis(uint32_t ticks)
{
    uint32_t seconds;

    seconds = ticks / STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND;
    ticks = ticks & (STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND -1);

    return (seconds * 1000) + ((ticks * 1000) / STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND);
}

static inline uint32_t stm32wb_lptim_timeout_ticks_to_seconds(uint32_t ticks)
{
    return ticks / STM32WB_LPTIM_TIMEOUT_TICKS_PER_SECOND;
}

#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_LPTIM_H */
