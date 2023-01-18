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
#include "RTC.h"
#include "wiring_private.h"

#define Y2K_TO_GPS_OFFSET    630720000
#define Y2K_UTC_OFFSET       13
#define Y2K_UNIX_TIME        946684800
#define UNIX_TO_GPS_OFFSET   -315964800

static const uint8_t days_in_month[4][16] = {
    {   0,  31,  29,  31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  0,  0,  0 },
    {   0,  31,  28,  31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  0,  0,  0 },
    {   0,  31,  28,  31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  0,  0,  0 },
    {   0,  31,  28,  31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  0,  0,  0 },
};

struct RTCClass::RTCAlarm RTCClass::m_alarm = {
    .match = RTC_MATCH_OFF,
    .seconds = 0,
    .minutes = 0,
    .hours = 0,
    .day = 0,
    .month = 0,
    .year = 0,
    .callback = Callback(),
    .work = K_WORK_INIT(RTCClass::alarmNotify, nullptr),
    .tod = { },
    .alarm = STM32WB_RTC_ALARM_INIT()
};

RTCClass::RTCClass() {
}

void RTCClass::begin(bool resetTime) {
    if (resetTime && !(stm32wb_rtc_status() & STM32WB_RTC_STATUS_TIME_WRITTEN)) {
        stm32wb_rtc_time_write(Y2K_TO_GPS_OFFSET + Y2K_UTC_OFFSET);
    }
}

uint16_t RTCClass::getMilliSeconds() {
    uint32_t seconds, ticks;
    
    stm32wb_rtc_time_read(&seconds, &ticks);

    return (1000 * ticks) / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
}

uint8_t RTCClass::getSeconds() {
    stm32wb_rtc_tod_t tod;
    
    getTod(&tod);

    return tod.seconds;
}

uint8_t RTCClass::getMinutes() {
    stm32wb_rtc_tod_t tod;

    getTod(&tod);

    return tod.minutes;
}

uint8_t RTCClass::getHours() {
    stm32wb_rtc_tod_t tod;

    getTod(&tod);

    return tod.hours;
}

uint8_t RTCClass::getDay() {
    stm32wb_rtc_tod_t tod;

    getTod(&tod);

    return tod.day;
}

uint8_t RTCClass::getMonth() {
    stm32wb_rtc_tod_t tod;

    getTod(&tod);

    return tod.month;
}

uint8_t RTCClass::getYear() {
    stm32wb_rtc_tod_t tod;

    getTod(&tod);

    return tod.year;
}

void RTCClass::getTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds) {
    stm32wb_rtc_tod_t tod;

    getTod(&tod);

    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
}

void RTCClass::getTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint16_t &milliSeconds) {
    stm32wb_rtc_tod_t tod;

    getTod(&tod);

    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
    milliSeconds = (1000 * tod.ticks) / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
}

void RTCClass::getDate(uint8_t &day, uint8_t &month, uint8_t &year) {
    stm32wb_rtc_tod_t tod;

    getTod(&tod);

    day = tod.day;
    month = tod.month;
    year = tod.year;
}

void RTCClass::getDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds) {
    stm32wb_rtc_tod_t tod;

    getTod(&tod);

    day = tod.day;
    month = tod.month;
    year = tod.year;
    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
}

void RTCClass::getDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, int32_t &zone, uint32_t &dst) {
    stm32wb_rtc_tod_t tod;

    getTod(&tod);

    day = tod.day;
    month = tod.month;
    year = tod.year;
    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;

    zone = stm32wb_rtc_get_zone();
    dst = stm32wb_rtc_get_dst();
}

void RTCClass::getDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint16_t &milliSeconds) {
    stm32wb_rtc_tod_t tod;

    getTod(&tod);

    day = tod.day;
    month = tod.month;
    year = tod.year;
    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
    milliSeconds = (1000 * tod.ticks) / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
}

void RTCClass::getDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint16_t &milliSeconds, int32_t &zone, uint32_t &dst) {
    stm32wb_rtc_tod_t tod;

    getTod(&tod);

    day = tod.day;
    month = tod.month;
    year = tod.year;
    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
    milliSeconds = (1000 * tod.ticks) / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;

    zone = stm32wb_rtc_get_zone();
    dst = stm32wb_rtc_get_dst();
}

void RTCClass::setSeconds(uint8_t seconds) {
    stm32wb_rtc_tod_t tod;
    
    getTod(&tod);

    tod.seconds = seconds;

    setTod(&tod);
}

void RTCClass::setMinutes(uint8_t minutes) {
    stm32wb_rtc_tod_t tod;
    
    getTod(&tod);

    tod.minutes = minutes;

    setTod(&tod);
}

void RTCClass::setHours(uint8_t hours) {
    stm32wb_rtc_tod_t tod;
    
    getTod(&tod);

    tod.hours = hours;

    setTod(&tod);
}

void RTCClass::setDay(uint8_t day) {
    stm32wb_rtc_tod_t tod;
    
    getTod(&tod);

    tod.day = day;

    setTod(&tod);
}

void RTCClass::setMonth(uint8_t month) {
    stm32wb_rtc_tod_t tod;
    
    getTod(&tod);

    tod.month = month;

    setTod(&tod);
}

void RTCClass::setYear(uint8_t year) {
    stm32wb_rtc_tod_t tod;
    
    getTod(&tod);

    tod.year = year;

    setTod(&tod);
}

void RTCClass::setTime(uint8_t hours, uint8_t minutes, uint8_t seconds) {
    stm32wb_rtc_tod_t tod;
    
    getTod(&tod);

    tod.hours = hours;
    tod.minutes = minutes;
    tod.seconds = seconds;

    setTod(&tod);
}

void RTCClass::setDate(uint8_t day, uint8_t month, uint8_t year) {
    stm32wb_rtc_tod_t tod;
    
    getTod(&tod);

    tod.day = day;
    tod.month = month;
    tod.year = year;

    setTod(&tod);
}

void RTCClass::setDateTime(uint8_t day, uint8_t month, uint8_t year, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    stm32wb_rtc_tod_t tod;
    
    getTod(&tod);

    tod.day = day;
    tod.month = month;
    tod.year = year;
    tod.hours = hours;
    tod.minutes = minutes;
    tod.seconds = seconds;

    setTod(&tod);
}


uint32_t RTCClass::getEpoch() {
    uint32_t tseconds, tticks;

    stm32wb_rtc_time_read(&tseconds, &tticks);

    return tseconds - UNIX_TO_GPS_OFFSET - stm32wb_rtc_get_leap_seconds();
}

uint64_t RTCClass::getEpochMilliSeconds() {
    uint32_t tseconds, tticks, seconds;
    
    stm32wb_rtc_time_read(&tseconds, &tticks);

    seconds = tseconds - UNIX_TO_GPS_OFFSET - stm32wb_rtc_get_leap_seconds();

    return (uint64_t)((uint64_t)seconds * 1000) + ((tticks * 1000) / STM32WB_RTC_CLOCK_TICKS_PER_SECOND);
}

bool RTCClass::setEpoch(uint32_t epoch) {

    if (epoch < Y2K_UNIX_TIME) {
        return false;
    }

    stm32wb_rtc_time_write(epoch + UNIX_TO_GPS_OFFSET + stm32wb_rtc_get_leap_seconds());

    return true;
}

bool RTCClass::convertEpoch(uint32_t epoch, uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds)
{
    stm32wb_rtc_tod_t tod;

    if (epoch < Y2K_UNIX_TIME) {
        return false;
    }

    stm32wb_rtc_time_to_tod(epoch - Y2K_UNIX_TIME + stm32wb_rtc_get_zone() + stm32wb_rtc_get_dst(), 0, &tod);

    day = tod.day;
    month = tod.month;
    year = tod.year;
    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
    
    return true;
}

bool RTCClass::convertEpoch(uint32_t epoch, uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, int32_t &zone, uint32_t &dst)
{
    stm32wb_rtc_tod_t tod;

    if (epoch < Y2K_UNIX_TIME) {
        return false;
    }

    zone = stm32wb_rtc_get_zone();
    dst = stm32wb_rtc_get_dst();

    stm32wb_rtc_time_to_tod(epoch - Y2K_UNIX_TIME + zone + dst, 0, &tod);

    day = tod.day;
    month = tod.month;
    year = tod.year;
    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
    
    return true;
}

bool RTCClass::convertEpochMilliSeconds(uint64_t epoch, uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint16_t &milliSeconds)
{
    stm32wb_rtc_tod_t tod;
    uint32_t eseconds;

    eseconds = epoch / 1000;
    
    if (eseconds < Y2K_UNIX_TIME) {
        return false;
    }

    stm32wb_rtc_time_to_tod(eseconds - Y2K_UNIX_TIME + stm32wb_rtc_get_zone() + stm32wb_rtc_get_dst(), 0, &tod);

    day = tod.day;
    month = tod.month;
    year = tod.year;
    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
    milliSeconds = (epoch - ((uint64_t)eseconds * 1000));
    
    return true;
}

bool RTCClass::convertEpochMilliSeconds(uint64_t epoch, uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint16_t &milliSeconds, int32_t &zone, uint32_t &dst)
{
    stm32wb_rtc_tod_t tod;
    uint32_t eseconds;

    eseconds = epoch / 1000;
    
    if (eseconds < Y2K_UNIX_TIME) {
        return false;
    }

    zone = stm32wb_rtc_get_zone();
    dst = stm32wb_rtc_get_dst();
    
    stm32wb_rtc_time_to_tod(eseconds - Y2K_UNIX_TIME + zone + dst, 0, &tod);

    day = tod.day;
    month = tod.month;
    year = tod.year;
    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
    milliSeconds = (epoch - ((uint64_t)eseconds * 1000));
    
    return true;
}

uint32_t RTCClass::getY2kEpoch() {
    uint32_t tseconds, tticks;

    stm32wb_rtc_time_read(&tseconds, &tticks);

    return tseconds - Y2K_TO_GPS_OFFSET - stm32wb_rtc_get_leap_seconds();
}

uint64_t RTCClass::getY2kEpochMilliSeconds() {
    uint32_t tseconds, tticks, seconds;
    
    stm32wb_rtc_time_read(&tseconds, &tticks);

    seconds = tseconds - Y2K_TO_GPS_OFFSET - stm32wb_rtc_get_leap_seconds();

    return (uint64_t)((uint64_t)seconds * 1000) + ((tticks * 1000) / STM32WB_RTC_CLOCK_TICKS_PER_SECOND);
}

void RTCClass::setY2kEpoch(uint32_t seconds) {
    seconds += Y2K_TO_GPS_OFFSET;

    stm32wb_rtc_time_write(seconds + stm32wb_rtc_get_leap_seconds());
}

uint8_t RTCClass::getAlarmSeconds() {
    return m_alarm.seconds;
}

uint8_t RTCClass::getAlarmMinutes() {
    return m_alarm.minutes;
}

uint8_t RTCClass::getAlarmHours() {
    return m_alarm.hours;
}

uint8_t RTCClass::getAlarmDay() {
    return m_alarm.day;
}

uint8_t RTCClass::getAlarmMonth() {
    return m_alarm.month;
}

uint8_t RTCClass::getAlarmYear() {
    return m_alarm.year;
}

void RTCClass::getAlarmTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds) {
    hours = m_alarm.hours;
    minutes = m_alarm.minutes;
    seconds = m_alarm.seconds;
}

void RTCClass::getAlarmDate(uint8_t &day, uint8_t &month, uint8_t &year) {
    day = m_alarm.day;
    month = m_alarm.month;
    year = m_alarm.year;
}

void RTCClass::getAlarmDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds) {
    day = m_alarm.day;
    month = m_alarm.month;
    year = m_alarm.year;
    hours = m_alarm.hours;
    minutes = m_alarm.minutes;
    seconds = m_alarm.seconds;
}

void RTCClass::setAlarmSeconds(uint8_t seconds) {
    m_alarm.seconds = seconds;

    alarmSync();
}

void RTCClass::setAlarmMinutes(uint8_t minutes) {
    m_alarm.minutes = minutes;

    alarmSync();
}

void RTCClass::setAlarmHours(uint8_t hours) {
    m_alarm.hours = hours;

    alarmSync();
}

void RTCClass::setAlarmDay(uint8_t day) {
    m_alarm.day = day;

    alarmSync();
}

void RTCClass::setAlarmMonth(uint8_t month) {
    m_alarm.month = month;

    alarmSync();
}

void RTCClass::setAlarmYear(uint8_t year) {
    m_alarm.year = year;

    alarmSync();
}

void RTCClass::setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds) {
    m_alarm.hours = hours;
    m_alarm.minutes = minutes;
    m_alarm.seconds = seconds;

    alarmSync();
}

void RTCClass::setAlarmDate(uint8_t day, uint8_t month, uint8_t year) {
    m_alarm.day = day;
    m_alarm.month = month;
    m_alarm.year = year;

    alarmSync();
}

void RTCClass::setAlarmDateTime(uint8_t day, uint8_t month, uint8_t year, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    m_alarm.day = day;
    m_alarm.month = month;
    m_alarm.year = year;
    m_alarm.hours = hours;
    m_alarm.minutes = minutes;
    m_alarm.seconds = seconds;

    alarmSync();
}

void RTCClass::setAlarmEpoch(uint32_t seconds) {
    stm32wb_rtc_tod_t tod;
    
    if (seconds < Y2K_UNIX_TIME) {
        return;
    }

    seconds += UNIX_TO_GPS_OFFSET;
    seconds -= Y2K_TO_GPS_OFFSET;

    stm32wb_rtc_time_to_tod(seconds + stm32wb_rtc_get_zone() + stm32wb_rtc_get_dst(), 0, &tod);

    m_alarm.day = tod.day;
    m_alarm.month = tod.month;
    m_alarm.year = tod.year;
    m_alarm.hours = tod.hours;
    m_alarm.minutes = tod.minutes;
    m_alarm.seconds = tod.seconds;
    
    alarmSync();
}

void RTCClass::enableAlarm(RTCAlarmMatch match) {
    m_alarm.match = match;

    alarmSync();
}

void RTCClass::disableAlarm() {
    if (m_alarm.match != RTC_MATCH_OFF) {
        m_alarm.match = RTC_MATCH_OFF;

        alarmSync();
    }
}

void RTCClass::onAlarm(void(*callback)(void)) {
    onAlarm(Callback(callback));
}

void RTCClass::onAlarm(Callback callback) {
    m_alarm.callback = callback;

    alarmSync();
}

int32_t RTCClass::getZone() {
    return stm32wb_rtc_get_zone();
}

void RTCClass::setZone(int32_t seconds) {
    stm32wb_rtc_set_zone(seconds);
}

uint32_t RTCClass::getDst() {
    return stm32wb_rtc_get_dst();
}

void RTCClass::setDst(uint32_t seconds) {
    stm32wb_rtc_set_dst(seconds);
}

int32_t RTCClass::getLeapSeconds() {
    return stm32wb_rtc_get_leap_seconds();
}

void RTCClass::setLeapSeconds(int32_t seconds) {
    stm32wb_rtc_set_leap_seconds(seconds);
}

uint32_t RTCClass::status() {
    return stm32wb_rtc_status();
}

void RTCClass::getTod(stm32wb_rtc_tod_t *tod) {
    uint32_t seconds, ticks;
    int32_t leapSeconds;
    
    stm32wb_rtc_time_read(&seconds, &ticks);

    leapSeconds = stm32wb_rtc_get_leap_seconds();

    seconds -= Y2K_TO_GPS_OFFSET;

    stm32wb_rtc_time_to_tod(seconds - leapSeconds + stm32wb_rtc_get_zone() + stm32wb_rtc_get_dst(), ticks, tod);
}

void RTCClass::setTod(const stm32wb_rtc_tod_t *tod) {
    uint32_t seconds, ticks;
    int32_t leapSeconds;

    stm32wb_rtc_tod_to_time(tod, &seconds, &ticks);

    seconds += Y2K_TO_GPS_OFFSET;

    leapSeconds = stm32wb_rtc_get_leap_seconds();

    stm32wb_rtc_time_write(seconds + leapSeconds - stm32wb_rtc_get_zone() - stm32wb_rtc_get_dst());
}

void RTCClass::alarmSync() {
    stm32wb_rtc_notify(RTCClass::alarmEvent, NULL);
    
    if (m_alarm.match != RTC_MATCH_OFF) {
        getTod(&m_alarm.tod);

        switch (m_alarm.match) {
        case RTC_MATCH_ANY:          // Every Second
            m_alarm.tod.ticks = 0;
            break;
        
        case RTC_MATCH_SS:           // Every Minute
            m_alarm.tod.ticks = 0;
            m_alarm.tod.seconds = m_alarm.seconds;
            break;
        
        case RTC_MATCH_MMSS:         // Every Hour
            m_alarm.tod.ticks = 0;
            m_alarm.tod.seconds = m_alarm.seconds;
            m_alarm.tod.minutes = m_alarm.minutes;
            break;
        
        case RTC_MATCH_HHMMSS:       // Every Day
            m_alarm.tod.ticks = 0;
            m_alarm.tod.seconds = m_alarm.seconds;
            m_alarm.tod.minutes = m_alarm.minutes;
            m_alarm.tod.hours = m_alarm.hours;
            break;
        
        case RTC_MATCH_YYMMDDHHMMSS: // Once
            m_alarm.tod.ticks = 0;
            m_alarm.tod.seconds = m_alarm.seconds;
            m_alarm.tod.minutes = m_alarm.minutes;
            m_alarm.tod.hours = m_alarm.hours;
            m_alarm.tod.day = m_alarm.day;
            m_alarm.tod.month = m_alarm.month;
            m_alarm.tod.year = m_alarm.year;
            break;
        }

        alarmStart();
    } else {
        alarmStop();
    }
}

void RTCClass::alarmStart() {
    uint32_t alarm_seconds, alarm_ticks;

    switch (m_alarm.match) {
    case RTC_MATCH_ANY: // Every Second
        m_alarm.tod.seconds = m_alarm.tod.seconds + 1;
                
        if (m_alarm.tod.seconds >= 60) {
            m_alarm.tod.seconds = 0;
                    
            m_alarm.tod.minutes = m_alarm.tod.minutes + 1;
                    
            if (m_alarm.tod.minutes >= 60) {
                m_alarm.tod.minutes = 0;
                        
                m_alarm.tod.hours = m_alarm.tod.hours + 1;
                    
                if (m_alarm.tod.hours >= 24) {
                    m_alarm.tod.hours = 0;
                        
                    m_alarm.tod.day = m_alarm.tod.day + 1;
                        
                    if (m_alarm.tod.day > days_in_month[m_alarm.tod.year & 3][m_alarm.tod.month]) {
                        m_alarm.tod.day = 1;
                        m_alarm.tod.month = m_alarm.tod.month + 1;
                            
                        if (m_alarm.tod.month > 12) {
                            m_alarm.tod.month = 1;
                            m_alarm.tod.year = m_alarm.tod.year + 1;
                        }
                    }
                }
            }
        }
        break;
                    
    case RTC_MATCH_SS:  // Every Minute
        m_alarm.tod.minutes = m_alarm.tod.minutes + 1;
                
        if (m_alarm.tod.minutes >= 60) {
            m_alarm.tod.minutes = 0;
                
            m_alarm.tod.hours = m_alarm.tod.hours + 1;
                
            if (m_alarm.tod.hours >= 24) {
                m_alarm.tod.hours = 0;
                    
                m_alarm.tod.day = m_alarm.tod.day + 1;
                    
                if (m_alarm.tod.day > days_in_month[m_alarm.tod.year & 3][m_alarm.tod.month]) {
                    m_alarm.tod.day = 1;
                    m_alarm.tod.month = m_alarm.tod.month + 1;
                        
                    if (m_alarm.tod.month > 12) {
                        m_alarm.tod.month = 1;
                        m_alarm.tod.year = m_alarm.tod.year + 1;
                    }
                }
            }
        }
        break;
                    
    case RTC_MATCH_MMSS:  // Every Hour
        m_alarm.tod.hours = m_alarm.tod.hours + 1;
            
        if (m_alarm.tod.hours >= 24) {
            m_alarm.tod.hours = 0;
                
            m_alarm.tod.day = m_alarm.tod.day + 1;
                
            if (m_alarm.tod.day > days_in_month[m_alarm.tod.year & 3][m_alarm.tod.month]) {
                m_alarm.tod.day = 1;
                m_alarm.tod.month = m_alarm.tod.month + 1;
                    
                if (m_alarm.tod.month > 12) {
                    m_alarm.tod.month = 1;
                    m_alarm.tod.year = m_alarm.tod.year + 1;
                }
            }
        }
        break;
                    
    case RTC_MATCH_HHMMSS:  // Every Day
        m_alarm.tod.day = m_alarm.tod.day + 1;
                
        if (m_alarm.tod.day > days_in_month[m_alarm.tod.year & 3][m_alarm.tod.month]) {
            m_alarm.tod.day = 1;
            m_alarm.tod.month = m_alarm.tod.month + 1;
                
            if (m_alarm.tod.month > 12) {
                m_alarm.tod.month = 1;
                m_alarm.tod.year = m_alarm.tod.year + 1;
            }
        }
        break;

    case RTC_MATCH_YYMMDDHHMMSS:
        break;
    }

    stm32wb_rtc_tod_to_time(&m_alarm.tod, &alarm_seconds, &alarm_ticks);

    alarm_seconds = (alarm_seconds + Y2K_TO_GPS_OFFSET) + stm32wb_rtc_get_leap_seconds() - stm32wb_rtc_get_zone() - stm32wb_rtc_get_dst();

    stm32wb_rtc_alarm_start(&m_alarm.alarm, stm32wb_rtc_time_to_clock(alarm_seconds, alarm_ticks), RTCClass::alarmCallback, NULL);
}

void RTCClass::alarmStop() {
    stm32wb_rtc_alarm_stop(&m_alarm.alarm);
}

void RTCClass::alarmEvent(__attribute__((unused)) void *context, __attribute__((unused)) uint32_t events) {
    if (m_alarm.match != RTC_MATCH_OFF) {
        alarmSync();
    }
}

void RTCClass::alarmCallback(__attribute__((unused)) void *context, uint64_t reference) {
    if (m_alarm.match == RTC_MATCH_YYMMDDHHMMSS) {
        m_alarm.match = RTC_MATCH_OFF;
    } else {
        alarmStart();
    }
    
    k_work_submit(&m_alarm.work);
}

void RTCClass::alarmNotify(__attribute__((unused)) void *context) {
    m_alarm.callback();
}

RTCClass RTC;
