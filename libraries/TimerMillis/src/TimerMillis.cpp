/*
 * Copyright (c) 2016-2022 Thomas Roell.  All rights reserved.
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
#include "TimerMillis.h"

/**********************************************************************************************************************************************************************/

TimerMillis::TimerMillis() {
    m_alarm = K_ALARM_INIT(TimerMillis::timeout, this);
    m_callback = Callback();
}

TimerMillis::~TimerMillis() {
    stop();
}

bool TimerMillis::start(void(*callback)(void), uint32_t delay, uint32_t period) {
    return start(Callback(callback), delay, period);
}

bool TimerMillis::start(Callback callback, uint32_t delay, uint32_t period) {
    if (!callback) {
        return false;
    }

    if (m_callback) {
        return false;
    }

    if (delay == 0) {
        if (period == 0) {
            return false;
        }
        
        delay = period;
    }

    m_callback = callback;

    k_alarm_relative(&m_alarm, delay, period);

    return true;
}

bool TimerMillis::restart(uint32_t delay, uint32_t period) {
    if (!m_callback) {
        return false;
    }

    if (delay == 0) {
        if (period == 0) {
            return false;
        }
        
        delay = period;
    }

    k_alarm_relative(&m_alarm, delay, period);

    return true;
}

void TimerMillis::stop() {
    k_alarm_cancel(&m_alarm);

    m_callback = Callback();
}

bool TimerMillis::active() {
    return m_callback;
}

void TimerMillis::timeout(void *context) {
    TimerMillis *self;
    Callback callback;
    
    self = (TimerMillis*)context;

    callback = self->m_callback;

    if (!k_alarm_is_active(&self->m_alarm))
    {
        self->m_callback = Callback();
    }

    callback();
}
