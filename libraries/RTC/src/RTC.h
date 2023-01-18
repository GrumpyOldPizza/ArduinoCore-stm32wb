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

#ifndef _RTC_H
#define _RTC_H

#include "Arduino.h"
#include "stm32wb_rtc.h"

enum RTCAlarmMatch {
    RTC_MATCH_OFF          = 0,      // Never
    RTC_MATCH_ANY          = 1,      // Every Second
    RTC_MATCH_SS           = 2,      // Every Minute
    RTC_MATCH_MMSS         = 3,      // Every Hour
    RTC_MATCH_HHMMSS       = 4,      // Every Day
    RTC_MATCH_YYMMDDHHMMSS = 5,      // Once, on a specific date and a specific time
};

enum RTCStatus {
    RTC_STATUS_TIME_WRITTEN = 1,
    RTC_STATUS_ZONE_WRITTEN = 2,
    RTC_STATUS_DST_WRITTEN = 4,
    RTC_STATUS_LEAP_SECONDS_WRITTEN = 8,
};

class RTCClass {
public:
    RTCClass();
    RTCClass(const RTCClass&) = delete;
    RTCClass& operator=(const RTCClass&) = delete;

    void begin(bool resetTime = false);

    uint16_t getMilliSeconds();
    uint8_t getSeconds();
    uint8_t getMinutes();
    uint8_t getHours();
    uint8_t getDay();
    uint8_t getMonth();
    uint8_t getYear();
    void getTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds);
    void getTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint16_t &milliSeconds);
    void getDate(uint8_t &day, uint8_t &month, uint8_t &year);
    void getDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds);
    void getDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, int32_t &zone, uint32_t &dst);
    void getDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint16_t &milliSeconds);
    void getDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint16_t &milliSeconds, int32_t &zone, uint32_t &dst);

    void setSeconds(uint8_t seconds);
    void setMinutes(uint8_t minutes);
    void setHours(uint8_t hours);
    void setDay(uint8_t day);
    void setMonth(uint8_t month);
    void setYear(uint8_t year);
    void setTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
    void setDate(uint8_t day, uint8_t month, uint8_t year);
    void setDateTime(uint8_t day, uint8_t month, uint8_t year, uint8_t hours, uint8_t minutes, uint8_t seconds);

    uint32_t getEpoch();
    uint64_t getEpochMilliSeconds();
    bool setEpoch(uint32_t epoch);

    bool convertEpoch(uint32_t epoch, uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds);
    bool convertEpoch(uint32_t epoch, uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, int32_t &zone, uint32_t &dst);
    bool convertEpochMilliSeconds(uint64_t epoch, uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint16_t &milliSeconds);
    bool convertEpochMilliSeconds(uint64_t epoch, uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint16_t &milliSeconds, int32_t &zone, uint32_t &dst);
  
    uint32_t getY2kEpoch();
    uint64_t getY2kEpochMilliSeconds();
    void setY2kEpoch(uint32_t epoch);
    
    uint8_t getAlarmSeconds();
    uint8_t getAlarmMinutes();
    uint8_t getAlarmHours();
    uint8_t getAlarmDay();
    uint8_t getAlarmMonth();
    uint8_t getAlarmYear();
    void getAlarmTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds);
    void getAlarmDate(uint8_t &day, uint8_t &month, uint8_t &year);
    void getAlarmDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds);

    void setAlarmSeconds(uint8_t seconds);
    void setAlarmMinutes(uint8_t minutes);
    void setAlarmHours(uint8_t hours);
    void setAlarmDay(uint8_t day);
    void setAlarmMonth(uint8_t month);
    void setAlarmYear(uint8_t year);
    void setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
    void setAlarmDate(uint8_t day, uint8_t month, uint8_t year);
    void setAlarmDateTime(uint8_t day, uint8_t month, uint8_t year, uint8_t hours, uint8_t minutes, uint8_t seconds);
    void setAlarmEpoch(uint32_t seconds);
    
    void enableAlarm(RTCAlarmMatch match);
    void disableAlarm();

    void onAlarm(void(*callback)(void) = nullptr);
    void onAlarm(Callback callback);

    int32_t getZone();
    void setZone(int32_t seconds);

    uint32_t getDst();
    void setDst(uint32_t seconds);

    int32_t getLeapSeconds();
    void setLeapSeconds(int32_t leapSeconds);

    uint32_t status();

     __attribute__((deprecated("Use RTC.onAlarm(callback) instead."))) void attachInterrupt(void(*callback)(void)) { onAlarm(callback); };
     __attribute__((deprecated("Use RTC.onAlarm() instead."))) void detachInterrupt() { onAlarm(); };
  
private:
    static struct RTCAlarm {
        uint8_t match;
        uint8_t seconds;
        uint8_t minutes;
        uint8_t hours;
        uint8_t day;
        uint8_t month;
        uint8_t year;
        Callback callback;
        k_work_t work;
        stm32wb_rtc_tod_t tod;
        stm32wb_rtc_alarm_t alarm;
    } m_alarm;

    static void getTod(stm32wb_rtc_tod_t *tod);
    static void setTod(const stm32wb_rtc_tod_t *tod);

    static void alarmSync();
    static void alarmStart();
    static void alarmStop();
    static void alarmCallback(void *context, uint64_t reference);
    static void alarmNotify(void *context);
    static void alarmEvent(void *context, uint32_t events);
};

extern RTCClass RTC;

#endif // _RTC_H
