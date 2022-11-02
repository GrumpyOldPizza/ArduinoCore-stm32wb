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

#if !defined(_STM32WB_EXTI_H)
#define _STM32WB_EXTI_H

#include "armv7m.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define STM32WB_EXTI_CHANNEL_INDEX_RTC_TAMP1    16
#define STM32WB_EXTI_CHANNEL_INDEX_RTC_TAMP2    17
#define STM32WB_EXTI_CHANNEL_INDEX_RTC_TAMP3    18
#define STM32WB_EXTI_CHANNEL_INDEX_COMP1        19
#define STM32WB_EXTI_CHANNEL_INDEX_COMP2        20
#define STM32WB_EXTI_CHANNEL_COUNT              21

#define STM32WB_EXTI_CHANNEL_MASK_RTC_TAMP1     (1u << STM32WB_EXTI_CHANNEL_INDEX_RTC_TAMP1)
#define STM32WB_EXTI_CHANNEL_MASK_RTC_TAMP2     (1u << STM32WB_EXTI_CHANNEL_INDEX_RTC_TAMP2)
#define STM32WB_EXTI_CHANNEL_MASK_RTC_TAMP3     (1u << STM32WB_EXTI_CHANNEL_INDEX_RTC_TAMP3)
#define STM32WB_EXTI_CHANNEL_MASK_COMP1         (1u << STM32WB_EXTI_CHANNEL_INDEX_COMP1)
#define STM32WB_EXTI_CHANNEL_MASK_COMP2         (1u << STM32WB_EXTI_CHANNEL_INDEX_COMP2)

#define STM32WB_EXTI_CONTROL_PRIORITY_SHIFT     0
#define STM32WB_EXTI_CONTROL_PRIORITY_MASK      0x00000003
#define STM32WB_EXTI_CONTROL_PRIORITY_CRITICAL  0x00000000
#define STM32WB_EXTI_CONTROL_PRIORITY_HIGH      0x00000001
#define STM32WB_EXTI_CONTROL_PRIORITY_MEDIUM    0x00000002
#define STM32WB_EXTI_CONTROL_PRIORITY_LOW       0x00000003
#define STM32WB_EXTI_CONTROL_EDGE_FALLING       0x00000004
#define STM32WB_EXTI_CONTROL_EDGE_RISING        0x00000008

typedef void (*stm32wb_exti_callback_t)(void *context);

extern void __stm32wb_exti_initialize(void);
extern void __stm32wb_exti_catch(uint32_t index, uint32_t priority, stm32wb_exti_callback_t callback, void *context);
extern void __stm32wb_exti_interrupt(uint32_t mask);
   
extern bool stm32wb_exti_catch(uint16_t pin, uint32_t control, stm32wb_exti_callback_t callback, void *context);
extern void stm32wb_exti_block(uint32_t mask);
extern void stm32wb_exti_unblock(uint32_t mask);

#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_EXTI_H */
