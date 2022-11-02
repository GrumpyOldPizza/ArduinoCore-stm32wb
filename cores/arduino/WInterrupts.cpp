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

#include "Arduino.h"
#include "wiring_private.h"

void attachInterrupt(uint32_t pin, void(*callback)(void), uint32_t mode, int priority) {
    attachInterrupt(pin, Callback(callback), mode, priority);
}

void attachInterrupt(uint32_t pin, Callback callback, uint32_t mode, int priority) {
    uint32_t control;
    
    if ((pin >= PINS_COUNT) || !(g_APinDescription[pin].attr & (PIN_ATTR_EXTI | PIN_ATTR_TAMP)) || !callback) {
        return;
    }

    if (g_APinDescription[pin].attr & PIN_ATTR_EXTI) {
        switch (mode) {
        case CHANGE:
            control = (STM32WB_EXTI_CONTROL_EDGE_FALLING | STM32WB_EXTI_CONTROL_EDGE_RISING);
            break;
        case FALLING:
            control = (STM32WB_EXTI_CONTROL_EDGE_FALLING);
            break;
        case RISING:
            control = (STM32WB_EXTI_CONTROL_EDGE_RISING);
            break;
        default:
            return;
        }

        switch (priority) {
        case 0:
            control |= STM32WB_EXTI_CONTROL_PRIORITY_CRITICAL;
            break;
        case 1:
            control |= STM32WB_EXTI_CONTROL_PRIORITY_HIGH;
            break;
        case 2:
            control |= STM32WB_EXTI_CONTROL_PRIORITY_MEDIUM;
            break;
        case 3:
            control |= STM32WB_EXTI_CONTROL_PRIORITY_LOW;
            break;
        default:
            return;
        }
        
        stm32wb_exti_catch(g_APinDescription[pin].pin, control, (stm32wb_exti_callback_t)callback.callback(), callback.context());
    } else {
        switch (mode) {
        case FALLING:
            control = (STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING);
            break;
        case RISING:
            control = (STM32WB_RTC_TAMP_CONTROL_EDGE_RISING);
            break;
        default:
            return;
        }

        switch (priority) {
        case 0:
            control |= STM32WB_RTC_TAMP_CONTROL_PRIORITY_CRITICAL;
            break;
        case 1:
            control |= STM32WB_RTC_TAMP_CONTROL_PRIORITY_HIGH;
            break;
        case 2:
            control |= STM32WB_RTC_TAMP_CONTROL_PRIORITY_MEDIUM;
            break;
        case 3:
            control |= STM32WB_RTC_TAMP_CONTROL_PRIORITY_LOW;
            break;
        default:
            return;
        }

        stm32wb_rtc_tamp_catch(g_APinDescription[pin].pin, control, (stm32wb_rtc_tamp_callback_t)callback.callback(), callback.context());
    }
}

extern "C" {

void attachInterrupt(uint32_t pin, void(*callback)(void), uint32_t mode) {
    attachInterrupt(pin, Callback(callback), mode, 2);
}

void detachInterrupt(uint32_t pin) {
    if ((pin >= PINS_COUNT) || !(g_APinDescription[pin].attr & (PIN_ATTR_EXTI | PIN_ATTR_TAMP))) {
        return;
    }

    if (g_APinDescription[pin].attr & PIN_ATTR_EXTI) {
        stm32wb_exti_catch(g_APinDescription[pin].pin, 0, NULL, NULL);
    } else {
        stm32wb_rtc_tamp_catch(g_APinDescription[pin].pin, 0, NULL, NULL);
    }
}

}
