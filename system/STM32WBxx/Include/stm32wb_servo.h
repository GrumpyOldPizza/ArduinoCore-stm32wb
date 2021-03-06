/*
 * Copyright (c) 2016-2021 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32WB_SERVO_H)
#define _STM32WB_SERVO_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32wbxx.h"

#include "stm32wb_lptim.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*stm32wb_servo_callback_t)(void *context);

#define STM32WB_SERVO_STATE_NONE                         0
#define STM32WB_SERVO_STATE_INIT                         1
#define STM32WB_SERVO_STATE_NOT_READY                    2
#define STM32WB_SERVO_STATE_READY                        3
#define STM32WB_SERVO_STATE_ACTIVE                       4

#define STM32WB_SERVO_CHANNEL_COUNT                      9

#define STM32WB_SERVO_FRAME_WIDTH                        20000   /* the default RC servo frame is 20000us                */
#define STM32WB_SERVO_PULSE_WIDTH_MIN                    800     /* below a pulse is deemed illegal                      */
#define STM32WB_SERVO_PULSE_WIDTH_MAX                    2200    /* above a pulse is deemed illegal                      */

extern bool stm32wb_servo_enable(uint8_t priority, stm32wb_servo_callback_t callback, void *context);
extern void stm32wb_servo_disable(void);
extern bool stm32wb_servo_channel(uint32_t index, uint16_t pin, uint16_t width);

#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_SERVO_H */
