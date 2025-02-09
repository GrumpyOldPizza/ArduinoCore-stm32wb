/*
 * Copyright (c) 2017-2022 Thomas Roell.  All rights reserved.
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

#define RESET_HARDWARE         0
#define RESET_SOFTWARE         1
#define RESET_WAKEUP           2
#define RESET_WATCHDOG         3
#define RESET_FAULT            4
#define RESET_ASSERT           5
#define RESET_PANIC            6

#define WAKEUP_PIN_1           0x00000001
#define WAKEUP_PIN_2           0x00000002
#define WAKEUP_PIN_3           0x00000004
#define WAKEUP_PIN_4           0x00000008
#define WAKEUP_PIN_5           0x00000010
#define WAKEUP_TIMEOUT         0x00000100
#define WAKEUP_WATCHDOG        0x00000200
#define WAKEUP_RESET           0x00000400

#define PANIC_CODE             0x0000ffff
#define PANIC_SPURIOUS         0x00010000
#define PANIC_APPLICATION      0x80000000

#define FAULT_MEM_IACCVIOL     0x00000001
#define FAULT_MEM_DACCVIOL     0x00000002
#define FAULT_MEM_UNSTKERR     0x00000008
#define FAULT_MEM_STKERR       0x00000010
#define FAULT_MEM_LSPERR       0x00000020
#define FAULT_BUS_IBUSERRL     0x00000100
#define FAULT_BUS_PRECISERR    0x00000200
#define FAULT_BUS_IMPRECISERR  0x00000400
#define FAULT_BUS_UNSTKERR     0x00000800
#define FAULT_BUS_STKERR       0x00001000
#define FAULT_BUS_LSPERR       0x00002000
#define FAULT_USAGE_UNDEFINSTR 0x00010000  
#define FAULT_USAGE_INVSTATE   0x00020000  
#define FAULT_USAGE_INVPC      0x00040000  
#define FAULT_USAGE_NOCP       0x00080000  
#define FAULT_USAGE_STKOF      0x00100000  
#define FAULT_USAGE_UNALIGNED  0x01000000  
#define FAULT_USAGE_DIVBYZERO  0x02000000  
#define FAULT_HARD_FORCED      0x40000000  
#define FAULT_HARD_DEBUGEVT    0x80000000  

#define POLICY_RUN             1
#define POLICY_SLEEP           2
#define POLICY_STOP            3

#define TIMEOUT_NONE           0x00000000
#define TIMEOUT_FOREVER        0xffffffff

#define FLASHSTART             ((uint32_t)(&__FlashBase))
#define FLASHEND               ((uint32_t)(&__FlashLimit))

class STM32WBClass {
public:
    static void getUID(uint32_t uid[3]);

    static float readBattery();
    static float readTemperature();
    static float readVREF();

    static uint32_t resetCause();
    static uint32_t wakeupReason();
    static uint32_t faultReason(const struct _armv7m_fault_info_t * &info);
    static uint32_t assertReason(const char * &assertion, const char * &file, uint32_t &line);
    static uint32_t panicReason();

    static bool setClocks(uint32_t hclk);
    static bool setClocks(uint32_t sysclk, uint32_t hclk, uint32_t pclk1, uint32_t pclk2);
    static void getClocks(uint32_t &sysclk, uint32_t &hclk, uint32_t &pclk1, uint32_t &pclk2);

    static bool boostEnable();
    static bool boostDisable();
  
    static void wakeup();
    static bool sleep();
    static bool sleep(uint32_t timeout);
    static bool sleep(uint32_t policy, uint32_t timeout);
    static bool stop();
    static bool stop(uint32_t timeout);
    static bool delay(uint32_t policy, uint32_t delay);
    static void standby();
    static void standby(uint32_t timeout);
    static void standby(uint32_t pin, uint32_t mode);
    static void standby(uint32_t pin, uint32_t mode, uint32_t timeout);
    static void shutdown();
    static void shutdown(uint32_t pin, uint32_t mode);
    static void reset();
    static void dfu();

    static void onStandby(void(*callback)(void));
    static void onStandby(Callback callback);
    static void onShutdown(void(*callback)(void));
    static void onShutdown(Callback callback);
    static void onReset(void(*callback)(void));
    static void onReset(Callback callback);

    static void swdEnable();
    static void swdDisable();

    static void wdtEnable(uint32_t timeout);
    static void wdtReset();

    static bool flashErase(uint32_t address, uint32_t count);
    static bool flashProgram(uint32_t address, const void *data, uint32_t count);
};

extern STM32WBClass STM32WB;

#endif // STM32WB_H
