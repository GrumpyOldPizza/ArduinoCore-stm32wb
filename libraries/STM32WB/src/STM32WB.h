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

#ifndef STM32WB_H
#define STM32WB_H

#include <Arduino.h>

#define RESET_POWERON        0
#define RESET_EXTERNAL       1
#define RESET_INTERNAL       2
#define RESET_SOFTWARE       3
#define RESET_FIREWALL       4
#define RESET_WATCHDOG       5
#define RESET_CRASH          6
#define RESET_STANDBY        7

#define WAKEUP_PIN_1         0x00000001
#define WAKEUP_PIN_2         0x00000002
#define WAKEUP_PIN_3         0x00000004
#define WAKEUP_PIN_4         0x00000008
#define WAKEUP_PIN_5         0x00000010
#define WAKEUP_TIMEOUT       0x00000100
#define WAKEUP_WATCHDOG      0x00000200
#define WAKEUP_RESET         0x00000400

#define FLASHSTART           ((uint32_t)(&__FlashBase))
#define FLASHEND             ((uint32_t)(&__FlashLimit))

class STM32WBClass {
public:
    static uint64_t getSerial();
    static void getUID(uint32_t uid[3]);

    static float readBattery();
    static float readTemperature();
    static float readVDDA();

    static uint32_t resetCause();
    static uint32_t wakeupReason();

    static bool setClocks(uint32_t hclk, uint32_t pclk1 = 0, uint32_t pclk2 = 0);
    static void setClocks(uint32_t &hclk, uint32_t &pclk1, uint32_t &pclk2);
    
    static void wakeup();
    static void sleep(uint32_t timeout = 0);
    static void stop(uint32_t timeout = 0);
    static void standby(uint32_t timeout = 0);
    static void standby(uint32_t pin, uint32_t mode, uint32_t timeout = 0);
    static void shutdown();
    static void shutdown(uint32_t pin, uint32_t mode);
    static void reset();
    static void dfu();

    static void swdEnable();
    static void swdDisable();

    static void wdtEnable(uint32_t timeout);
    static void wdtReset();

    static bool flashErase(uint32_t address, uint32_t count);
    static bool flashProgram(uint32_t address, const void *data, uint32_t count);
};

extern STM32WBClass STM32WB;

#endif // STM32WB_H
