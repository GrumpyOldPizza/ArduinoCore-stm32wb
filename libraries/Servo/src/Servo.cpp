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

#include "Arduino.h"
#include "wiring_private.h"
#include <Servo.h>

#define INVALID_SERVO 255

static uint32_t ServosAttached = 0;

Servo::Servo() {
    m_index = INVALID_SERVO;
}

uint8_t Servo::attach(uint32_t pin, int min, int max) {
    unsigned int index;

    if ((pin >= PINS_COUNT) || (g_APinDescription[pin].GPIO == NULL)) {
        return INVALID_SERVO;
    }

    for (index = 0; index < MAX_SERVOS; index++) {
        if (!(ServosAttached & (1 << index))) {
            break;
        }
    }

    if (index == MAX_SERVOS) {
        return INVALID_SERVO;
    }

    if (!ServosAttached) {
        if (!stm32wb_servo_enable(STM32WB_SERVO_IRQ_PRIORITY, NULL, NULL)) {
            return INVALID_SERVO;
        }
    }
    
    ServosAttached |= (1 << index);

    m_index = index;
    m_width = DEFAULT_PULSE_WIDTH;
    m_pin = g_APinDescription[pin].pin;
    m_min = min;
    m_max = max;

    pinMode(pin, OUTPUT);

    stm32wb_servo_channel(m_index, m_pin, m_width);

    return m_index;
}

void Servo::detach() {
    if (m_index >= MAX_SERVOS) {
        return;
    }

    stm32wb_servo_channel(m_index, STM32WB_GPIO_PIN_NONE, 0);

    m_index = MAX_SERVOS;
    m_pin = STM32WB_GPIO_PIN_NONE;

    ServosAttached &= ~(1 << m_index);

    if (!ServosAttached) {
        stm32wb_servo_disable();
    }
}

void Servo::write(int angle) {
    if (m_index >= MAX_SERVOS) {
        return;
    }
    
    if (angle < 200) {
        if (angle < 0) {
            angle = 0;
        }

        if (angle > 180) {
            angle = 180;
        }

        m_width = map(angle, 0, 180, m_min, m_max);
    } else {
        if (angle < m_min) {
            angle = m_min;
        }
        
        if (angle > m_max) {
            angle = m_max;
        }

        m_width = angle;
    }

    stm32wb_servo_channel(m_index, m_pin, m_width);
}

void Servo::writeMicroseconds(int width) {
    if (m_index >= MAX_SERVOS) {
        return;
    }

    if (width < m_min) {
        width = m_min;
    }

    if (width > m_max) {
        width = m_max;
    }

    m_width = width;
    
    stm32wb_servo_channel(m_index, m_pin, m_width);
}

int Servo::read() {
    if (m_index >= MAX_SERVOS) {
        return 0;
    }

    return map(m_width+1, m_min, m_max, 0, 180);
}

int Servo::readMicroseconds() {
    if (m_index >= MAX_SERVOS) {
        return 0;
    }

    return m_width;
}

bool Servo::attached() {
    return (m_index < MAX_SERVOS);
}
