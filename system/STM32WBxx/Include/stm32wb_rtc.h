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

#if !defined(_STM32WB_RTC_H)
#define _STM32WB_RTC_H

#include "armv7m.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************/

typedef struct _stm32wb_rtc_capture_t stm32wb_rtc_capture_t;
typedef struct _stm32wb_rtc_tod_t stm32wb_rtc_tod_t;

struct _stm32wb_rtc_capture_t {
    uint32_t dr;
    uint32_t tr;
    uint32_t ssr;
};

struct _stm32wb_rtc_tod_t {
    uint8_t                        year;
    uint8_t                        month;
    uint8_t                        day;
    uint8_t                        hours;
    uint8_t                        minutes;
    uint8_t                        seconds;
    uint16_t                       ticks;
};

#define STM32WB_RTC_EVENT_TIME_WRITTEN                0x00000001
#define STM32WB_RTC_EVENT_ZONE_WRITTEN                0x00000002
#define STM32WB_RTC_EVENT_DST_WRITTEN                 0x00000004
#define STM32WB_RTC_EVENT_LEAP_SECONDS_WRITTEN        0x00000008

typedef void (*stm32wb_rtc_event_callback_t)(void *context, uint32_t events);
  
typedef void (*stm32wb_rtc_alarm_callback_t)(void *context, uint64_t clock);

typedef void (*stm32wb_rtc_tamp_callback_t)(void *context);

typedef struct _stm32wb_rtc_alarm_t {
    struct _stm32wb_rtc_alarm_t            *next;
    struct _stm32wb_rtc_alarm_t            *previous;
    struct _stm32wb_rtc_alarm_t * volatile modify;
    volatile uint32_t                      clock_l;
    volatile uint32_t                      clock_h;
    volatile stm32wb_rtc_alarm_callback_t  callback;
    void * volatile                        context;
} stm32wb_rtc_alarm_t;
  
#define STM32WB_RTC_IRQ_PRIORITY                      ARMV7M_IRQ_PRIORITY_RTC

#define STM32WB_RTC_STATUS_TIME_WRITTEN               0x00000001
#define STM32WB_RTC_STATUS_ZONE_WRITTEN               0x00000002
#define STM32WB_RTC_STATUS_DST_WRITTEN                0x00000004
#define STM32WB_RTC_STATUS_LEAP_SECONDS_WRITTEN       0x00000008

#define STM32WB_RTC_ALARM_PRIORITY_CRITICAL           0x00000000
#define STM32WB_RTC_ALARM_PRIORITY_HIGH               0x00000001
#define STM32WB_RTC_ALARM_PRIORITY_MEDIUM             0x00000002
#define STM32WB_RTC_ALARM_PRIORITY_LOW                0x00000003
  
#define STM32WB_RTC_TAMP_CONTROL_PRIORITY_SHIFT       0
#define STM32WB_RTC_TAMP_CONTROL_PRIORITY_MASK        0x00000003
#define STM32WB_RTC_TAMP_CONTROL_PRIORITY_CRITICAL    0x00000000
#define STM32WB_RTC_TAMP_CONTROL_PRIORITY_HIGH        0x00000001
#define STM32WB_RTC_TAMP_CONTROL_PRIORITY_MEDIUM      0x00000002
#define STM32WB_RTC_TAMP_CONTROL_PRIORITY_LOW         0x00000003
#define STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING         0x00000004
#define STM32WB_RTC_TAMP_CONTROL_EDGE_RISING          0x00000008

#define STM32WB_RTC_PREDIV_S                          2048
#define STM32WB_RTC_PREDIV_A                          16
  
#define STM32WB_RTC_ALRMSSR_MASKSS                    (RTC_ALRMBSSR_MASKSS_3 | RTC_ALRMBSSR_MASKSS_1 | RTC_ALRMBSSR_MASKSS_0)
  
#define STM32WB_RTC_CLOCK_TICKS_PER_SECOND            STM32WB_RTC_PREDIV_S

#define STM32WB_RTC_BKP16R_DATA_MASK                  0x0000ffff
#define STM32WB_RTC_BKP16R_DATA_SHIFT                 0
#define STM32WB_RTC_BKP16R_NOT_DATA_MASK              0xffff0000
#define STM32WB_RTC_BKP16R_NOT_DATA_SHIFT             16
#define STM32WB_RTC_BKP16R_REVISION_MASK              0x000000ff
#define STM32WB_RTC_BKP16R_REVISION_SHIFT             0
#define STM32WB_RTC_BKP16R_REVISION_CURRENT           0x00000003
#define STM32WB_RTC_BKP16R_FATAL_MASK                 0x00000100
#define STM32WB_RTC_BKP16R_FATAL_SHIFT                8
#define STM32WB_RTC_BKP16R_FATAL                      STM32WB_RTC_BKP16R_FATAL_MASK
#define STM32WB_RTC_BKP16R_DFU_MASK                   0x00000200
#define STM32WB_RTC_BKP16R_DFU_SHIFT                  9
#define STM32WB_RTC_BKP16R_DFU                        STM32WB_RTC_BKP16R_DFU_MASK                 
#define STM32WB_RTC_BKP16R_OTA_MASK                   0x00000400
#define STM32WB_RTC_BKP16R_OTA_SHIFT                  10
#define STM32WB_RTC_BKP16R_OTA                        STM32WB_RTC_BKP16R_OTA_MASK
  
#define STM32WB_RTC_BKP17R_HSECLK_MASK                0x00000001
#define STM32WB_RTC_BKP17R_HSECLK_SHIFT               0
#define STM32WB_RTC_BKP17R_HSECLK                     STM32WB_RTC_BKP17R_HSECLK_MASK
  
#define STM32WB_RTC_BKP18R_SECONDS_OFFSET_MASK        0xffffffff
#define STM32WB_RTC_BKP18R_SECONDS_OFFSET_SHIFT       0

#define STM32WB_RTC_BKP19R_TICKS_OFFSET_MASK          0x000007ff
#define STM32WB_RTC_BKP19R_TICKS_OFFSET_SHIFT         0
#define STM32WB_RTC_BKP19R_ZONE_MASK                  0x0003f800
#define STM32WB_RTC_BKP19R_ZONE_SHIFT                 11
#define STM32WB_RTC_BKP19R_ZONE_SIZE                  7
#define STM32WB_RTC_BKP19R_ZONE_BIAS                  (12 * 3600)
#define STM32WB_RTC_BKP19R_ZONE_SCALE                 900
#define STM32WB_RTC_BKP19R_DST_MASK                   0x000c0000
#define STM32WB_RTC_BKP19R_DST_SHIFT                  18
#define STM32WB_RTC_BKP19R_DST_SIZE                   2
#define STM32WB_RTC_BKP19R_DST_BIAS                   0
#define STM32WB_RTC_BKP19R_DST_SCALE                  3600
#define STM32WB_RTC_BKP19R_LEAP_SECONDS_MASK          0x0ff00000
#define STM32WB_RTC_BKP19R_LEAP_SECONDS_SHIFT         20
#define STM32WB_RTC_BKP19R_LEAP_SECONDS_SIZE          8
#define STM32WB_RTC_BKP19R_LEAP_SECONDS_BIAS          128
#define STM32WB_RTC_BKP19R_TIME_WRITTEN_MASK          0x10000000
#define STM32WB_RTC_BKP19R_TIME_WRITTEN_SHIFT         28
#define STM32WB_RTC_BKP19R_TIME_WRITTEN               STM32WB_RTC_BKP19R_TIME_WRITTEN_MASK
#define STM32WB_RTC_BKP19R_ZONE_WRITTEN_MASK          0x20000000
#define STM32WB_RTC_BKP19R_ZONE_WRITTEN_SHIFT         29
#define STM32WB_RTC_BKP19R_ZONE_WRITTEN               STM32WB_RTC_BKP19R_ZONE_WRITTEN_MASK
#define STM32WB_RTC_BKP19R_DST_WRITTEN_MASK           0x40000000
#define STM32WB_RTC_BKP19R_DST_WRITTEN_SHIFT          30
#define STM32WB_RTC_BKP19R_DST_WRITTEN                STM32WB_RTC_BKP19R_DST_WRITTEN_MASK
#define STM32WB_RTC_BKP19R_LEAP_SECONDS_WRITTEN_MASK  0x80000000
#define STM32WB_RTC_BKP19R_LEAP_SECONDS_WRITTEN_SHIFT 31
#define STM32WB_RTC_BKP19R_LEAP_SECONDS_WRITTEN       STM32WB_RTC_BKP19R_LEAP_SECONDS_WRITTEN_MASK

extern void __stm32wb_rtc_initialize(void);

extern uint32_t stm32wb_rtc_status(void);
extern void stm32wb_rtc_notify(stm32wb_rtc_event_callback_t callback, void *context);

extern int32_t stm32wb_rtc_get_calibration(void);
extern void stm32wb_rtc_set_calibration(int32_t calibration);

extern void stm32wb_rtc_clock_capture(stm32wb_rtc_capture_t *data);
extern uint64_t stm32wb_rtc_clock_convert(const stm32wb_rtc_capture_t *data);
extern uint64_t stm32wb_rtc_clock_read();
extern void stm32wb_rtc_clock_to_time(uint64_t clock, uint32_t *p_seconds, uint32_t *p_ticks);

extern uint64_t stm32wb_rtc_time_to_clock(uint32_t seconds, uint32_t ticks);
extern void stm32wb_rtc_time_read(uint32_t *p_seconds, uint32_t *p_ticks);
extern void stm32wb_rtc_time_write(uint32_t seconds);
extern void stm32wb_rtc_time_adjust(int32_t ticks);
extern int32_t stm32wb_rtc_get_leap_seconds(void);
extern void stm32wb_rtc_set_leap_seconds(int32_t leap_seconds);
extern int32_t stm32wb_rtc_get_zone(void);
extern void stm32wb_rtc_set_zone(int32_t zone);
extern uint32_t stm32wb_rtc_get_dst(void);
extern void stm32wb_rtc_set_dst(uint32_t dst);

extern void stm32wb_rtc_time_to_tod(uint32_t seconds, uint32_t ticks, stm32wb_rtc_tod_t *p_tod);
extern void stm32wb_rtc_tod_to_time(const stm32wb_rtc_tod_t *tod, uint32_t *p_seconds, uint32_t *p_ticks);
  
#define STM32WB_RTC_ALARM_INIT()                \
(stm32wb_rtc_alarm_t)                           \
{                                               \
    .next = NULL,                               \
    .previous = NULL,                           \
    .modify = NULL,                             \
    .clock_l = 0,                               \
    .clock_h = 0,                               \
    .callback = NULL,                           \
    .context = NULL,                            \
}

extern bool stm32wb_rtc_alarm_start(stm32wb_rtc_alarm_t *alarm, uint64_t clock, stm32wb_rtc_alarm_callback_t callback, void *context);
extern void stm32wb_rtc_alarm_stop(stm32wb_rtc_alarm_t *alarm);
extern bool stm32wb_rtc_alarm_is_pending(stm32wb_rtc_alarm_t *alarm);

extern bool stm32wb_rtc_tamp_catch(uint16_t pin, uint32_t control, stm32wb_rtc_tamp_callback_t callback, void *context);

extern void stm32wb_rtc_standby(uint32_t timeout);
extern void stm32wb_rtc_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_RTC_H */

