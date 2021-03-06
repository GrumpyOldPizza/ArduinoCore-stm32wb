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
#include "RTC.h"
#include "wiring_private.h"

#define Y2K_TO_GPS_OFFSET    630720000
#define Y2K_UTC_OFFSET       13
#define Y2K_UNIX_TIME        946684800
#define UNIX_TO_GPS_OFFSET   -315964800

extern uint8_t __RTC_ZONE__;
extern uint8_t __RTC_DST__;

RTCClass::RTCClass() {
    m_zone = (int32_t)&__RTC_ZONE__;
    m_dst = (int32_t)&__RTC_DST__;
    m_alarm_match = RTC_MATCH_OFF;
    m_alarm_seconds = 0;
    m_alarm_minutes = 0;
    m_alarm_hours = 0;
    m_alarm_day = 1;
    m_alarm_month = 1;
    m_alarm_year = 0;
    m_alarm_timeout = 0;
    m_alarm_callback = Callback(__wakeupCallback);
}

void RTCClass::begin(bool resetTime) {
    if (resetTime && !(stm32wb_rtc_status() & (STM32WB_RTC_STATUS_TIME_INTERNAL | STM32WB_RTC_STATUS_TIME_EXTERNAL))) {
        stm32wb_rtc_time_write(0, Y2K_TO_GPS_OFFSET + Y2K_UTC_OFFSET, 0, false);
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

    return tseconds - UNIX_TO_GPS_OFFSET - stm32wb_rtc_time_to_utc_offset(tseconds);
}

void RTCClass::getEpoch(uint32_t &seconds, uint16_t &milliSeconds) {
    uint32_t tseconds, tticks;
    
    stm32wb_rtc_time_read(&tseconds, &tticks);

    seconds = tseconds - UNIX_TO_GPS_OFFSET - stm32wb_rtc_time_to_utc_offset(tseconds);
    milliSeconds = (1000 * tticks) / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
}

void RTCClass::setEpoch(uint32_t seconds) {
    if (seconds < Y2K_UNIX_TIME) {
        return;
    }

    seconds += UNIX_TO_GPS_OFFSET;

    stm32wb_rtc_time_write(0, seconds + stm32wb_rtc_utc_to_utc_offset(seconds), 0, false);
}

uint32_t RTCClass::getY2kEpoch() {
    uint32_t tseconds, tticks;

    stm32wb_rtc_time_read(&tseconds, &tticks);

    return tseconds - Y2K_TO_GPS_OFFSET - stm32wb_rtc_time_to_utc_offset(tseconds);
}

void RTCClass::getY2kEpoch(uint32_t &seconds, uint16_t &milliSeconds) {
    uint32_t tseconds, tticks;
    
    stm32wb_rtc_time_read(&tseconds, &tticks);

    seconds = tseconds - Y2K_TO_GPS_OFFSET - stm32wb_rtc_time_to_utc_offset(tseconds);
    milliSeconds = (1000 * tticks) / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
}

void RTCClass::setY2kEpoch(uint32_t seconds) {
    seconds += Y2K_TO_GPS_OFFSET;

    stm32wb_rtc_time_write(0, seconds + stm32wb_rtc_utc_to_utc_offset(seconds), 0, false);
}

uint32_t RTCClass::getGpsEpoch() {
    uint32_t tseconds, tticks;

    stm32wb_rtc_time_read(&tseconds, &tticks);

    return tseconds;
}

void RTCClass::getGpsEpoch(uint32_t &seconds, uint16_t &milliSeconds) {
    uint32_t tseconds, tticks;
    
    stm32wb_rtc_time_read(&tseconds, &tticks);

    seconds = tseconds;
    milliSeconds = (1000 * tticks) / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
}

void RTCClass::setGpsEpoch(uint32_t seconds) {
    stm32wb_rtc_time_write(0, seconds, 0, false);
}

uint8_t RTCClass::getAlarmSeconds() {
    return m_alarm_seconds;
}

uint8_t RTCClass::getAlarmMinutes() {
    return m_alarm_minutes;
}

uint8_t RTCClass::getAlarmHours() {
    return m_alarm_hours;
}

uint8_t RTCClass::getAlarmDay() {
    return m_alarm_day;
}

uint8_t RTCClass::getAlarmMonth() {
    return m_alarm_month;
}

uint8_t RTCClass::getAlarmYear() {
    return m_alarm_year;
}

void RTCClass::getAlarmTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds) {
    hours = m_alarm_hours;
    minutes = m_alarm_minutes;
    seconds = m_alarm_seconds;
}

void RTCClass::getAlarmDate(uint8_t &day, uint8_t &month, uint8_t &year) {
    day = m_alarm_day;
    month = m_alarm_month;
    year = m_alarm_year;
}

void RTCClass::getAlarmDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds) {
    day = m_alarm_day;
    month = m_alarm_month;
    year = m_alarm_year;
    hours = m_alarm_hours;
    minutes = m_alarm_minutes;
    seconds = m_alarm_seconds;
}

void RTCClass::setAlarmSeconds(uint8_t seconds) {
    m_alarm_seconds = seconds;

    syncAlarm();
}

void RTCClass::setAlarmMinutes(uint8_t minutes) {
    m_alarm_minutes = minutes;

    syncAlarm();
}

void RTCClass::setAlarmHours(uint8_t hours) {
    m_alarm_hours = hours;

    syncAlarm();
}

void RTCClass::setAlarmDay(uint8_t day) {
    m_alarm_day = day;

    syncAlarm();
}

void RTCClass::setAlarmMonth(uint8_t month) {
    m_alarm_month = month;

    syncAlarm();
}

void RTCClass::setAlarmYear(uint8_t year) {
    m_alarm_year = year;

    syncAlarm();
}

void RTCClass::setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds) {
    m_alarm_hours = hours;
    m_alarm_minutes = minutes;
    m_alarm_seconds = seconds;

    syncAlarm();
}

void RTCClass::setAlarmDate(uint8_t day, uint8_t month, uint8_t year) {
    m_alarm_day = day;
    m_alarm_month = month;
    m_alarm_year = year;

    syncAlarm();
}

void RTCClass::setAlarmDateTime(uint8_t day, uint8_t month, uint8_t year, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    m_alarm_day = day;
    m_alarm_month = month;
    m_alarm_year = year;
    m_alarm_hours = hours;
    m_alarm_minutes = minutes;
    m_alarm_seconds = seconds;

    syncAlarm();
}

void RTCClass::setAlarmEpoch(uint32_t seconds) {
    stm32wb_rtc_tod_t tod;
    
    if (seconds < Y2K_UNIX_TIME) {
        return;
    }

    seconds += UNIX_TO_GPS_OFFSET;
    seconds -= Y2K_TO_GPS_OFFSET;

    stm32wb_rtc_time_to_tod(seconds + (m_zone + m_dst), 0, &tod);

    m_alarm_day = tod.day;
    m_alarm_month = tod.month;
    m_alarm_year = tod.year;
    m_alarm_hours = tod.hours;
    m_alarm_minutes = tod.minutes;
    m_alarm_seconds = tod.seconds;
    
    syncAlarm();
}

void RTCClass::enableAlarm(RTCAlarmMatch match) {
    m_alarm_match = match;

    syncAlarm();
}

void RTCClass::disableAlarm() {
    if (m_alarm_match != RTC_MATCH_OFF) {
        m_alarm_match = RTC_MATCH_OFF;

        stm32wb_rtc_alarm_stop();
    }
}

void RTCClass::attachInterrupt(void(*callback)(void)) {
    attachInterrupt(Callback(callback));
}

void RTCClass::attachInterrupt(Callback callback) {
    m_alarm_callback = callback ? callback : Callback(__emptyCallback);

    syncAlarm();
}

void RTCClass::detachInterrupt() {
    m_alarm_callback = Callback(__wakeupCallback); 

    syncAlarm();
}

int32_t RTCClass::getZone() {
    return m_zone;
}

void RTCClass::setZone(int32_t seconds) {
    m_zone = seconds;
}

int32_t RTCClass::getDst() {
    return m_dst;
}

void RTCClass::setDst(int32_t seconds) {
    m_dst = seconds;
}

int32_t RTCClass::getUtcOffset() {
    uint32_t seconds, ticks;
    
    stm32wb_rtc_time_read(&seconds, &ticks);

    return stm32wb_rtc_time_to_utc_offset(seconds);
}

void RTCClass::setUtcOffset(int32_t seconds) {
    stm32wb_rtc_set_utc_offset(seconds, false);
}

uint32_t RTCClass::status() {
    return stm32wb_rtc_status();
}

void RTCClass::getTod(stm32wb_rtc_tod_t *tod) {
    uint32_t seconds, ticks;
    int32_t utc_offset;
    
    stm32wb_rtc_time_read(&seconds, &ticks);

    utc_offset = stm32wb_rtc_time_to_utc_offset(seconds);

    seconds -= Y2K_TO_GPS_OFFSET;

    stm32wb_rtc_time_to_tod(seconds - utc_offset + (m_zone + m_dst), ticks, tod);
}

void RTCClass::setTod(const stm32wb_rtc_tod_t *tod) {
    uint32_t seconds, ticks;
    int32_t utc_offset;

    stm32wb_rtc_tod_to_time(tod, &seconds, &ticks);

    seconds += Y2K_TO_GPS_OFFSET;

    utc_offset = stm32wb_rtc_time_to_utc_offset(seconds);

    stm32wb_rtc_time_write(0, seconds + utc_offset - (m_zone + m_dst), 0, false);
}

void RTCClass::syncAlarm() {
    stm32wb_rtc_tod_t tod, alarm_tod;
    uint32_t seconds, ticks;

    static const uint8_t days_in_month[4][16] = {
        {   0,  31,  29,  31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  0,  0,  0 },
        {   0,  31,  28,  31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  0,  0,  0 },
        {   0,  31,  28,  31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  0,  0,  0 },
        {   0,  31,  28,  31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  0,  0,  0 },
    };

    if (m_alarm_match != RTC_MATCH_OFF) {
        getTod(&alarm_tod);

        switch (m_alarm_match) {
        case RTC_MATCH_ANY: // Every Second
            alarm_tod.ticks = 0;
            break;
        
        case RTC_MATCH_SS:  // Every Minute
            alarm_tod.ticks = 0;
            alarm_tod.seconds = m_alarm_seconds;
            break;
        
        case RTC_MATCH_MMSS:  // Every Hour
            alarm_tod.ticks = 0;
            alarm_tod.seconds = m_alarm_seconds;
            alarm_tod.minutes = m_alarm_minutes;
            break;
        
        case RTC_MATCH_HHMMSS:  // Every Day
            alarm_tod.ticks = 0;
            alarm_tod.seconds = m_alarm_seconds;
            alarm_tod.minutes = m_alarm_minutes;
            alarm_tod.hours = m_alarm_hours;
            break;
        
        case RTC_MATCH_DDHHMMSS:  // Every Month
            alarm_tod.ticks = 0;
            alarm_tod.seconds = m_alarm_seconds;
            alarm_tod.minutes = m_alarm_minutes;
            alarm_tod.hours = m_alarm_hours;
            alarm_tod.day = m_alarm_day;
            break;
        
        case RTC_MATCH_MMDDHHMMSS:  // Every Year
            alarm_tod.ticks = 0;
            alarm_tod.seconds = m_alarm_seconds;
            alarm_tod.minutes = m_alarm_minutes;
            alarm_tod.hours = m_alarm_hours;
            alarm_tod.day = m_alarm_day;
            alarm_tod.month = m_alarm_month;
            break;
        
        case RTC_MATCH_YYMMDDHHMMSS:  // Once
            alarm_tod.ticks = 0;
            alarm_tod.seconds = m_alarm_seconds;
            alarm_tod.minutes = m_alarm_minutes;
            alarm_tod.hours = m_alarm_hours;
            alarm_tod.day = m_alarm_day;
            alarm_tod.month = m_alarm_month;
            alarm_tod.year = m_alarm_year;
            break;
        }

        if (m_alarm_match == RTC_MATCH_YYMMDDHHMMSS) {
            stm32wb_rtc_tod_to_time(&alarm_tod, &m_alarm_timeout, &ticks);
        } else {
            while (1) {
                getTod(&tod);

                stm32wb_rtc_tod_to_time(&alarm_tod, &m_alarm_timeout, &ticks);
                stm32wb_rtc_tod_to_time(&tod, &seconds, &ticks);

                if (m_alarm_timeout > seconds) {
                    break;
                }
            
                switch (m_alarm_match) {
                case RTC_MATCH_ANY: // Every Second
                    alarm_tod.seconds = alarm_tod.seconds + 1;
                
                    if (alarm_tod.seconds >= 60) {
                        alarm_tod.seconds = 0;
                    
                        alarm_tod.minutes = alarm_tod.minutes + 1;
                    
                        if (alarm_tod.minutes >= 60) {
                            alarm_tod.minutes = 0;
                        
                            alarm_tod.hours = alarm_tod.hours + 1;
                    
                            if (alarm_tod.hours >= 24) {
                                alarm_tod.hours = 0;
                        
                                alarm_tod.day = alarm_tod.day + 1;
                        
                                if (alarm_tod.day > days_in_month[alarm_tod.year & 3][alarm_tod.month]) {
                                    alarm_tod.day = 1;
                                    alarm_tod.month = alarm_tod.month + 1;
                            
                                    if (alarm_tod.month > 12) {
                                        alarm_tod.month = 1;
                                        alarm_tod.year = alarm_tod.year + 1;
                                    }
                                }
                            }
                        }
                    }
                    break;
                    
                case RTC_MATCH_SS:  // Every Minute
                    alarm_tod.minutes = alarm_tod.minutes + 1;
                
                    if (alarm_tod.minutes >= 60) {
                        alarm_tod.minutes = 0;
                
                        alarm_tod.hours = alarm_tod.hours + 1;
                
                        if (alarm_tod.hours >= 24) {
                            alarm_tod.hours = 0;
                    
                            alarm_tod.day = alarm_tod.day + 1;
                    
                            if (alarm_tod.day > days_in_month[alarm_tod.year & 3][alarm_tod.month]) {
                                alarm_tod.day = 1;
                                alarm_tod.month = alarm_tod.month + 1;
                        
                                if (alarm_tod.month > 12) {
                                    alarm_tod.month = 1;
                                    alarm_tod.year = alarm_tod.year + 1;
                                }
                            }
                        }
                    }
                    break;
                    
                case RTC_MATCH_MMSS:  // Every Hour
                    alarm_tod.hours = alarm_tod.hours + 1;
            
                    if (alarm_tod.hours >= 24) {
                        alarm_tod.hours = 0;
                
                        alarm_tod.day = alarm_tod.day + 1;
                
                        if (alarm_tod.day > days_in_month[alarm_tod.year & 3][alarm_tod.month]) {
                            alarm_tod.day = 1;
                            alarm_tod.month = alarm_tod.month + 1;
                    
                            if (alarm_tod.month > 12) {
                                alarm_tod.month = 1;
                                alarm_tod.year = alarm_tod.year + 1;
                            }
                        }
                    }
                    break;
                    
                case RTC_MATCH_HHMMSS:  // Every Day
                    alarm_tod.day = alarm_tod.day + 1;
                
                    if (alarm_tod.day > days_in_month[alarm_tod.year & 3][alarm_tod.month]) {
                        alarm_tod.day = 1;
                        alarm_tod.month = alarm_tod.month + 1;
                
                        if (alarm_tod.month > 12) {
                            alarm_tod.month = 1;
                            alarm_tod.year = alarm_tod.year + 1;
                        }
                    }
                    break;
                    
                case RTC_MATCH_DDHHMMSS:  // Every Month
                    do {
                        alarm_tod.month = alarm_tod.month + 1;
                
                        if (alarm_tod.month > 12) {
                            alarm_tod.month = 1;
                            alarm_tod.year = alarm_tod.year + 1;
                        }
                    }
                    while (alarm_tod.day > days_in_month[alarm_tod.year & 3][alarm_tod.month]);
                    break;
                    
                case RTC_MATCH_MMDDHHMMSS:  // Every Year
                    do {
                        alarm_tod.year = alarm_tod.year + 1;
                    }
                    while (alarm_tod.day > days_in_month[alarm_tod.year & 3][alarm_tod.month]);
                    break;
                }
            }
        }
        seconds = m_alarm_timeout + Y2K_TO_GPS_OFFSET - (m_zone + m_dst);

        Callback alarm_callback = Callback(&RTCClass::alarmCallback, this);
        
        stm32wb_rtc_alarm_start(seconds, 0, alarm_callback.callback(), alarm_callback.context());
    }
}

void RTCClass::alarmCallback() {
    if (m_alarm_match == RTC_MATCH_YYMMDDHHMMSS) {
        m_alarm_match = RTC_MATCH_OFF;
    } else {
        syncAlarm();
    }
    
    m_alarm_callback();
}

RTCClass RTC;
