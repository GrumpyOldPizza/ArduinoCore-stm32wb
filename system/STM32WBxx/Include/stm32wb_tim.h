/*
 * Copyright (c) 2016-2020 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32WB_TIM_H)
#define _STM32WB_TIM_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32wbxx.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    STM32WB_TIM_INSTANCE_TIM1 = 0,   /* ADVANCED 16 */
    STM32WB_TIM_INSTANCE_TIM2,       /* GENERAL  32 */
    STM32WB_TIM_INSTANCE_TIM16,      /* GENERAL  16 */
    STM32WB_TIM_INSTANCE_TIM17,      /* GENERAL  16 */
    STM32WB_TIM_INSTANCE_COUNT
};

#define STM32WB_TIM_CHANNEL_1                               0
#define STM32WB_TIM_CHANNEL_2                               1
#define STM32WB_TIM_CHANNEL_3                               2
#define STM32WB_TIM_CHANNEL_4                               3

#define STM32WB_TIM_OPTION_CONTINOUS                        0x00000001
#define STM32WB_TIM_OPTION_TIM16_CHANNEL_1_INPUT_MASK       0x00000006 
#define STM32WB_TIM_OPTION_TIM16_CHANNEL_1_INPUT_SHIFT      1
#define STM32WB_TIM_OPTION_TIM16_CHANNEL_1_INPUT_GPIO       0x00000000
#define STM32WB_TIM_OPTION_TIM16_CHANNEL_1_INPUT_LSI        0x00000002
#define STM32WB_TIM_OPTION_TIM16_CHANNEL_1_INPUT_LSE        0x00000004
#define STM32WB_TIM_OPTION_TIM16_CHANNEL_1_INPUT_RTC_WAKEUP 0x00000006
#define STM32WB_TIM_OPTION_TIM17_CHANNEL_1_INPUT_MASK       0x00000006 
#define STM32WB_TIM_OPTION_TIM17_CHANNEL_1_INPUT_SHIFT      1
#define STM32WB_TIM_OPTION_TIM17_CHANNEL_1_INPUT_GPIO       0x00000000
#define STM32WB_TIM_OPTION_TIM17_CHANNEL_1_INPUT_MSI        0x00000002
#define STM32WB_TIM_OPTION_TIM17_CHANNEL_1_INPUT_HSE_BY_32  0x00000004
#define STM32WB_TIM_OPTION_TIM17_CHANNEL_1_INPUT_MCO        0x00000006

#define STM32WB_TIM_EVENT_CHANNEL_1                         0x00000001
#define STM32WB_TIM_EVENT_CHANNEL_2                         0x00000002
#define STM32WB_TIM_EVENT_CHANNEL_3                         0x00000004
#define STM32WB_TIM_EVENT_CHANNEL_4                         0x00000008
#define STM32WB_TIM_EVENT_PERIOD                            0x00000010


#define STM32WB_TIM_CONTROL_DISABLE                         0x00000000
#define STM32WB_TIM_CONTROL_CAPTURE_RISING_EDGE             0x00000001
#define STM32WB_TIM_CONTROL_CAPTURE_FALLING_EDGE            0x00000002
#define STM32WB_TIM_CONTROL_CAPTURE_ALTERNATE               0x00000004 /* take input from even/odd channel */
#define STM32WB_TIM_CONTROL_CAPTURE_PRESCALE_MASK           0x00000018
#define STM32WB_TIM_CONTROL_CAPTURE_PRESCALE_SHIFT          3
#define STM32WB_TIM_CONTROL_CAPTURE_PRESCALE_1              0x00000000
#define STM32WB_TIM_CONTROL_CAPTURE_PRESCALE_2              0x00000008
#define STM32WB_TIM_CONTROL_CAPTURE_PRESCALE_4              0x00000010
#define STM32WB_TIM_CONTROL_CAPTURE_PRESCALE_8              0x00000018
#define STM32WB_TIM_CONTROL_CAPTURE_FILTER_MASK             0x000001e0
#define STM32WB_TIM_CONTROL_CAPTURE_FILTER_SHIFT            5
#define STM32WB_TIM_CONTROL_CAPTURE_FILTER(_n)              (((_n) << STM32WB_TIM_CONTROL_CAPTURE_FILTER_SHIFT) & STM32WB_TIM_CONTROL_CAPTURE_FILTER_MASK)
#define STM32WB_TIM_CONTROL_COMPARE                         0x00000200
#define STM32WB_TIM_CONTROL_PWM                             0x00000400
#define STM32WB_TIM_CONTROL_PWM_INVERTED                    0x00000800
#define STM32WB_TIM_CONTROL_PWM_COMPLEMENTARY               0x00001000

typedef void (*stm32wb_tim_callback_t)(void *context, uint32_t events);

#define STM32WB_TIM_STATE_NONE                              0
#define STM32WB_TIM_STATE_INIT                              1
#define STM32WB_TIM_STATE_NOT_READY                         2
#define STM32WB_TIM_STATE_READY                             3
#define STM32WB_TIM_STATE_ACTIVE                            4

typedef struct _stm32wb_tim_t {
    TIM_TypeDef                 *TIM;
    volatile uint8_t            state;
    uint8_t                     instance;
    uint8_t                     priority;
    stm32wb_tim_callback_t      callback;
    void                        *context;
} stm32wb_tim_t;

extern bool     stm32wb_tim_create(stm32wb_tim_t *tim, unsigned int instance, unsigned int priority);
extern bool     stm32wb_tim_destroy(stm32wb_tim_t *tim);
extern uint32_t stm32wb_tim_clock(stm32wb_tim_t *tim);
extern bool     stm32wb_tim_enable(stm32wb_tim_t *tim, uint32_t option, stm32wb_tim_callback_t callback, void *context);
extern bool     stm32wb_tim_disable(stm32wb_tim_t *tim);
extern bool     stm32wb_tim_configure(stm32wb_tim_t *tim, uint32_t option);
extern bool     stm32wb_tim_start(stm32wb_tim_t *tim, uint32_t prescale, uint32_t reload);
extern bool     stm32wb_tim_restart(stm32wb_tim_t *tim, uint32_t reload);
extern bool     stm32wb_tim_stop(stm32wb_tim_t *tim);
extern uint32_t stm32wb_tim_count(stm32wb_tim_t *tim);
extern bool     stm32wb_tim_channel(stm32wb_tim_t *tim, unsigned int channel, uint32_t compare, uint32_t control);
extern bool     stm32wb_tim_compare(stm32wb_tim_t *tim, unsigned int channel, uint32_t compare);
extern uint32_t stm32wb_tim_capture(stm32wb_tim_t *tim, unsigned int channel);

#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_TIM_H */
