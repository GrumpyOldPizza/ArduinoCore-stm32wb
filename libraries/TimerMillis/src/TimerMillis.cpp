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
#include "TimerMillis.h"
#include "wiring_private.h"

class TimerMillisInstance {
public:
    virtual ~TimerMillisInstance() = default;
  
    virtual void reference();
    virtual void unreference();

    virtual bool start(Callback callback, uint32_t delay, uint32_t period);
    virtual bool restart(uint32_t delay, uint32_t period);
    virtual void stop();
    virtual bool active();
};

static TimerMillisInstance NullTimerMillis;

void TimerMillisInstance::reference() {
}

void TimerMillisInstance::unreference() {
}

bool TimerMillisInstance::start(Callback callback __attribute__((unused)), uint32_t delay __attribute__((unused)), uint32_t period __attribute__((unused))) {
    return false;
}

bool TimerMillisInstance::restart(uint32_t delay __attribute__((unused)), uint32_t period __attribute__((unused))) {
    return false;
}

void TimerMillisInstance::stop() {
}

bool TimerMillisInstance::active() {
    return false;
}

class TimerMillisRTC : public TimerMillisInstance {
public:
    TimerMillisRTC();
    ~TimerMillisRTC() override;
  
    void reference() override;
    void unreference() override;

    bool start(Callback callback, uint32_t delay, uint32_t period) override;
    bool restart(uint32_t delay, uint32_t period) override;
    void stop() override;
    bool active() override;

private:
    volatile uint32_t m_refcount;

protected:
    stm32wb_rtc_timer_t m_timer;
    k_work_t m_work;
    uint64_t m_clock;
    uint32_t m_delay;
    volatile uint32_t m_period;
    volatile uint8_t m_active;

    Callback m_callback;
    
    static void timeout(TimerMillisRTC *self, uint64_t clock);
    static void routine(TimerMillisRTC *self);
};

TimerMillisRTC::TimerMillisRTC() {
    m_refcount = 1;
  
    stm32wb_rtc_timer_create(&m_timer, (stm32wb_rtc_timer_callback_t)TimerMillisRTC::timeout, this);
    k_work_create(&m_work, (k_work_routine_t)TimerMillisRTC::routine, this);

    m_active = 0;

    m_callback = Callback();
}

TimerMillisRTC::~TimerMillisRTC() {
}

void TimerMillisRTC::reference() {
    __armv7m_atomic_inc(&m_refcount, 0xffffffff);
}

void TimerMillisRTC::unreference() {
    if (__armv7m_atomic_dec(&m_refcount) == 1) {
        delete this;
    }
}

bool TimerMillisRTC::start(Callback callback, uint32_t delay, uint32_t period) {
    uint32_t seconds, ticks;

    if (m_active) {
        return false;
    }

    if (delay == 0) {
        if (period == 0) {
            return false;
        }

        delay = period;
    }

    seconds = delay / 1000;
    ticks = ((delay - seconds * 1000) * STM32WB_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000;
    
    m_clock = stm32wb_rtc_clock_read() + (seconds * STM32WB_RTC_CLOCK_TICKS_PER_SECOND);
    m_delay = delay - (seconds * 1000);
    m_period = period;
    m_callback = callback;

    if (armv7m_atomic_swapb(&m_active, true) == false) {
        reference();
    }
    
    stm32wb_rtc_timer_start(&m_timer, m_clock + ticks);

    return true;
}

bool TimerMillisRTC::restart(uint32_t delay, uint32_t period) {
    uint32_t seconds, ticks;

    if (delay == 0) {
        if (period == 0) {
            return false;
        }

        delay = period;
    }

    seconds = delay / 1000;
    ticks = ((delay - seconds * 1000) * STM32WB_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000;

    m_clock = stm32wb_rtc_clock_read() + (seconds * STM32WB_RTC_CLOCK_TICKS_PER_SECOND);
    m_delay = delay - (seconds * 1000);
    m_period = period;

    if (armv7m_atomic_swapb(&m_active, true) == false) {
        reference();
    }
    
    stm32wb_rtc_timer_start(&m_timer, m_clock + ticks);

    return true;
}

void TimerMillisRTC::stop() {
    stm32wb_rtc_timer_stop(&m_timer);
}

bool TimerMillisRTC::active() {
    return m_active;
}

void TimerMillisRTC::timeout(TimerMillisRTC *self, uint64_t clock) {
    uint32_t delay, seconds, ticks;

    if (clock) {
        if (self->m_period) {
            delay = self->m_delay + self->m_period;
            seconds = delay / 1000;
            ticks = ((delay - seconds * 1000) * STM32WB_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000;
            
            self->m_clock += (seconds * STM32WB_RTC_CLOCK_TICKS_PER_SECOND);
            self->m_delay = delay - (seconds * 1000);

            self->reference();

            stm32wb_rtc_timer_start(&self->m_timer, self->m_clock + ticks);
        } else {
            armv7m_atomic_storeb(&self->m_active, false);
        }

        if (!k_work_submit(&self->m_work)) {
            self->unreference();
        }
    } else {
        armv7m_atomic_storeb(&self->m_active, false);

        self->unreference();
    }
}

void TimerMillisRTC::routine(TimerMillisRTC *self) {
    self->m_callback();

    self->unreference();
}



TimerMillis::TimerMillis() {
    m_instance = new TimerMillisRTC();

    if (m_instance == nullptr) {
        m_instance = &NullTimerMillis;
    }
}

TimerMillis::~TimerMillis() {
    m_instance->unreference();
}

TimerMillis::TimerMillis(const TimerMillis &other) {
    m_instance = other.m_instance;

    (*m_instance).reference();
}

TimerMillis &TimerMillis::operator=(const TimerMillis &other) {
    TimerMillisInstance *device = m_instance;

    m_instance = other.m_instance;

    (*m_instance).reference();

    device->unreference();
    
    return *this;
}

TimerMillis::TimerMillis(TimerMillis &&other) {
    m_instance = other.m_instance;

    other.m_instance = &NullTimerMillis;
}

TimerMillis &TimerMillis::operator=(TimerMillis &&other) {
    TimerMillisInstance *device = m_instance;

    m_instance = other.m_instance;

    other.m_instance = &NullTimerMillis;

    device->unreference();

    return *this;
}

TimerMillis::operator bool() const {
    return m_instance != &NullTimerMillis;
}

bool TimerMillis::start(void(*callback)(void), uint32_t delay, uint32_t period) {
    return start(Callback(callback), delay, period);
}

bool TimerMillis::start(Callback callback, uint32_t delay, uint32_t period) {
    return (*m_instance).start(callback, delay, period);
}

bool TimerMillis::restart(uint32_t delay, uint32_t period) {
    return (*m_instance).restart(delay, period);
}

void TimerMillis::stop() {
    (*m_instance).stop();
}

bool TimerMillis::active() {
    return (*m_instance).active();
}
