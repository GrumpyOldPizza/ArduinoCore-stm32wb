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

#include "armv7m.h"

#include "stm32wb_rtc.h"
#include "stm32wb_gpio.h"
#include "stm32wb_exti.h"
#include "stm32wb_system.h"

/*******************************************************************************************************************/

typedef void (*stm32wb_rtc_sync_routine_t)(void);
typedef void (*stm32wb_rtc_alarm_routine_t)(void);

typedef struct _stm32wb_rtc_device_t {
    uint64_t                               clock_offset;
    volatile uint64_t                      time_offset; 
    volatile int32_t                       zone;
    volatile uint16_t                      dst;
    volatile int8_t                        leap_seconds;
    volatile uint8_t                       status;
    volatile stm32wb_rtc_event_callback_t  event_callback;
    void * volatile                        event_context;
    volatile uint32_t                      events;
    volatile uint8_t                       alarm_busy;
    volatile uint8_t                       alarm_modify;
    volatile uint8_t                       alarm_event;
    volatile uint64_t                      alarm_clock;
    volatile stm32wb_rtc_alarm_callback_t  alarm_callback;
    void * volatile                        alarm_context;
    volatile stm32wb_rtc_sync_routine_t    sync_routine;
    volatile stm32wb_rtc_alarm_routine_t   alarm_routine;
} stm32wb_rtc_device_t;

static stm32wb_rtc_device_t stm32wb_rtc_device;

typedef struct _stm32wb_rtc_info_t {
    uint32_t                               epoch;
    int32_t                                zone;
    uint16_t                               dst;
    int16_t                                leap_seconds;
} stm32wb_rtc_info_t;

extern const stm32wb_rtc_info_t __rtc_info__;

static uint64_t __attribute__((section(".noinit2"))) __stm32wb_rtc_alarm_clock;

static void stm32wb_rtc_sync_routine();
static void stm32wb_rtc_alarm_routine();

/*******************************************************************************************************************/

static const uint8_t stm32wb_rtc_int_to_bcd[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 
};

static const uint8_t stm32wb_rtc_bcd_to_int[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

static const uint16_t stm32wb_rtc_days_since_year[100] = {
    0,     366,   731,   1096,  1461,  1827,  2192,  2557,  2922,  3288, 
    3653,  4018,  4383,  4749,  5114,  5479,  5844,  6210,  6575,  6940, 
    7305,  7671,  8036,  8401,  8766,  9132,  9497,  9862,  10227, 10593, 
    10958, 11323, 11688, 12054, 12419, 12784, 13149, 13515, 13880, 14245, 
    14610, 14976, 15341, 15706, 16071, 16437, 16802, 17167, 17532, 17898, 
    18263, 18628, 18993, 19359, 19724, 20089, 20454, 20820, 21185, 21550, 
    21915, 22281, 22646, 23011, 23376, 23742, 24107, 24472, 24837, 25203, 
    25568, 25933, 26298, 26664, 27029, 27394, 27759, 28125, 28490, 28855, 
    29220, 29586, 29951, 30316, 30681, 31047, 31412, 31777, 32142, 32508, 
    32873, 33238, 33603, 33969, 34334, 34699, 35064, 35430, 35795, 36160};

static const uint16_t stm32wb_rtc_days_since_month[4][16] = {
    {   0,   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335, 335, 335, 335 },
    {   0,   0,  31,  59,  90, 120, 151, 181, 212, 243, 273, 304, 334, 334, 334, 334 },
    {   0,   0,  31,  59,  90, 120, 151, 181, 212, 243, 273, 304, 334, 334, 334, 334 },
    {   0,   0,  31,  59,  90, 120, 151, 181, 212, 243, 273, 304, 334, 334, 334, 334 },
};

static const uint8_t stm32wb_rtc_days_in_month[4][16] = {
    {   0,  31,  29,  31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  0,  0,  0 },
    {   0,  31,  28,  31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  0,  0,  0 },
    {   0,  31,  28,  31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  0,  0,  0 },
    {   0,  31,  28,  31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  0,  0,  0 },
};


/*
 * https://github.com/eggert/tz/blob/master/leap-seconds.list
 *
 * GPS Time 0 in terms of TAI: 2524521600 + 432000
 */

static const uint32_t stm32wb_rtc_leap_seconds_table[] = {
    0,          // [ 0]  1 Jan 1980
    46828800,   // [ 1]  1 Jul 1981
    78364801,   // [ 2]  1 Jul 1982
    109900802,  // [ 3]  1 Jul 1983
    173059203,  // [ 4]  1 Jul 1985
    252028804,  // [ 5]  1 Jan 1988
    315187205,  // [ 6]  1 Jan 1990
    346723206,  // [ 7]  1 Jan 1991
    393984007,  // [ 8]  1 Jul 1992
    425520008,  // [ 9]  1 Jul 1993
    457056009,  // [10]  1 Jul 1994
    504489610,  // [11]  1 Jan 1996
    551750411,  // [12]  1 Jul 1997
    599184012,  // [13]  1 Jan 1999
    820108813,  // [14]  1 Jan 2006
    914803214,  // [15]  1 Jan 2009
    1025136015, // [16]  1 Jul 2012
    1119744016, // [17]  1 Jul 2015
    1167264017, // [18]  1 Jan 2017
};

#define STM32WB_RTC_SECONDS_OFFSET_DEFAULT      1167264018 //  1 Jan 2017 00:00:00 in GPS TIME
#define STM32WB_RTC_TICKS_OFFSET_DEFAULT        0
#define STM32WB_RTC_LEAP_SECONDS_DEFAULT        18
#define STM32WB_RTC_LEAP_SECONDS_EXPIRATION     1293494417 // 31 Dec 2020 23:59:59 in GPS TIME

void __stm32wb_rtc_initialize(void)
{
    uint32_t lseclk, hseclk;
    int32_t calibration;
    stm32wb_rtc_capture_t capture;
    
    if (!(RCC->BDCR & RCC_BDCR_RTCEN))
    {
        /* Use LSE/LSI as source for RTC */

        lseclk = stm32wb_system_lseclk();
        hseclk = stm32wb_system_hseclk();
        
	RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | ((lseclk ? RCC_BDCR_RTCSEL_0 : RCC_BDCR_RTCSEL_1) | RCC_BDCR_RTCEN);

	RTC->WPR = 0xca;
	RTC->WPR = 0x53;
	
        RTC->ISR = RTC_ISR_INIT;
        
        while (!(RTC->ISR & RTC_ISR_INITF))
        {
        }
        
        RTC->CR = RTC_CR_BYPSHAD;
	RTC->TAMPCR = RTC_TAMPCR_TAMP1NOERASE | RTC_TAMPCR_TAMP2NOERASE | RTC_TAMPCR_TAMP3NOERASE | RTC_TAMPCR_TAMPPUDIS;

        RTC->PRER = (STM32WB_RTC_PREDIV_S -1) << RTC_PRER_PREDIV_S_Pos;
        RTC->PRER |= (STM32WB_RTC_PREDIV_A -1) << RTC_PRER_PREDIV_A_Pos;
        
        RTC->ISR = ~RTC_ISR_INIT;

	RTC->BKP16R = (STM32WB_RTC_BKP16R_NOT_DATA_MASK & ~(STM32WB_RTC_BKP16R_REVISION_CURRENT << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT)) | STM32WB_RTC_BKP16R_REVISION_CURRENT;
	RTC->BKP17R = hseclk ? STM32WB_RTC_BKP17R_HSECLK : 0;
	RTC->BKP18R = __rtc_info__.epoch ? (__rtc_info__.epoch - 315964800 + __rtc_info__.leap_seconds) : 0;
	RTC->BKP19R = ((((__rtc_info__.zone / 900) << STM32WB_RTC_BKP19R_ZONE_SHIFT) & STM32WB_RTC_BKP19R_ZONE_MASK) |
                       (((__rtc_info__.dst / 3600) << STM32WB_RTC_BKP19R_DST_SHIFT) & STM32WB_RTC_BKP19R_DST_MASK) |
                       ((__rtc_info__.leap_seconds << STM32WB_RTC_BKP19R_LEAP_SECONDS_SHIFT) & STM32WB_RTC_BKP19R_LEAP_SECONDS_MASK));

        if (lseclk)
        {
            calibration = 32 * (lseclk - 32768);

            if (calibration >= 0)
            {
                if (calibration > 511)
                {
                    calibration = 511;
                }

                RTC->CALR = ((uint32_t)(calibration) << RTC_CALR_CALM_Pos);
            }
            else
            {
                if (calibration < -512)
                {
                    calibration = -512;
                }
                
                RTC->CALR = RTC_CALR_CALP | ((uint32_t)(512 + calibration) << RTC_CALR_CALM_Pos);
            }
        }
    }
    else
    {
	RTC->WPR = 0xca;
	RTC->WPR = 0x53;
    }
    
    EXTI->IMR1 &= ~(EXTI_IMR1_IM17 | EXTI_IMR1_IM18 | EXTI_IMR1_IM19);
    
    EXTI->PR1 = EXTI_PR1_PIF17 | EXTI_PR1_PIF18 | EXTI_PR1_PIF19;

    NVIC_SetPriority(TAMP_STAMP_LSECSS_IRQn, STM32WB_RTC_IRQ_PRIORITY);
    NVIC_SetPriority(RTC_WKUP_IRQn, STM32WB_RTC_IRQ_PRIORITY);
    NVIC_SetPriority(RTC_Alarm_IRQn, STM32WB_RTC_IRQ_PRIORITY);

    NVIC_EnableIRQ(TAMP_STAMP_LSECSS_IRQn);
    NVIC_EnableIRQ(RTC_WKUP_IRQn);
    NVIC_EnableIRQ(RTC_Alarm_IRQn);

    stm32wb_rtc_clock_capture(&capture);
    stm32wb_rtc_device.clock_offset = stm32wb_rtc_clock_convert(&capture);
    stm32wb_rtc_device.time_offset = ((uint64_t)RTC->BKP18R * STM32WB_RTC_CLOCK_TICKS_PER_SECOND) + ((RTC->BKP19R & STM32WB_RTC_BKP19R_TICKS_OFFSET_MASK) >> STM32WB_RTC_BKP19R_TICKS_OFFSET_SHIFT);
    stm32wb_rtc_device.zone = ((int32_t)(RTC->BKP19R << (32 - STM32WB_RTC_BKP19R_ZONE_SHIFT - STM32WB_RTC_BKP19R_ZONE_SIZE)) >> (32 - STM32WB_RTC_BKP19R_ZONE_SIZE)) * 900;
    stm32wb_rtc_device.dst = ((RTC->BKP19R & STM32WB_RTC_BKP19R_DST_MASK) >> STM32WB_RTC_BKP19R_DST_SHIFT) * 3600;
    stm32wb_rtc_device.leap_seconds = (int32_t)(RTC->BKP19R << (32 - STM32WB_RTC_BKP19R_LEAP_SECONDS_SHIFT - STM32WB_RTC_BKP19R_LEAP_SECONDS_SIZE)) >> (32 - STM32WB_RTC_BKP19R_LEAP_SECONDS_SIZE);
    stm32wb_rtc_device.status = (((RTC->BKP19R & STM32WB_RTC_BKP19R_TIME_WRITTEN) ? STM32WB_RTC_STATUS_TIME_WRITTEN : 0) |
                                 ((RTC->BKP19R & STM32WB_RTC_BKP19R_ZONE_WRITTEN) ? STM32WB_RTC_STATUS_ZONE_WRITTEN : 0) |
                                 ((RTC->BKP19R & STM32WB_RTC_BKP19R_DST_WRITTEN) ? STM32WB_RTC_STATUS_DST_WRITTEN : 0) |
                                 ((RTC->BKP19R & STM32WB_RTC_BKP19R_LEAP_SECONDS_WRITTEN) ? STM32WB_RTC_STATUS_LEAP_SECONDS_WRITTEN : 0));

    RTC->CR = (RTC->CR & ~RTC_CR_WUCKSEL);

    EXTI->RTSR1 |= (EXTI_RTSR1_RT17 | EXTI_RTSR1_RT18 | EXTI_RTSR1_RT19);
    EXTI->IMR1 |= (EXTI_IMR1_IM17 | EXTI_IMR1_IM18 | EXTI_IMR1_IM19);
}

uint32_t stm32wb_rtc_status(void)
{
    return stm32wb_rtc_device.status;
}

void stm32wb_rtc_notify(stm32wb_rtc_event_callback_t callback, void *context)
{
    stm32wb_rtc_device.event_callback = NULL;
    stm32wb_rtc_device.event_context = context;
    stm32wb_rtc_device.event_callback = callback;
}

int32_t stm32wb_rtc_get_calibration(void)
{
    if (RTC->CALR & RTC_CALR_CALP)
    {
        return 512 - ((int32_t)((RTC->CALR & RTC_CALR_CALM_Msk) >> RTC_CALR_CALM_Pos));
    }
    else
    {
        return - ((int32_t)((RTC->CALR & RTC_CALR_CALM_Msk) >> RTC_CALR_CALM_Pos));
    }
}

void stm32wb_rtc_set_calibration(int32_t calibration)
{
    if (calibration > 0)
    {
        if (calibration > 512)
        {
            calibration = 512;
        }

        RTC->CALR = RTC_CALR_CALP | ((uint32_t)(512 - calibration) << RTC_CALR_CALM_Pos);
    }
    else
    {
        if (calibration < -511)
        {
            calibration = -511;
        }

        RTC->CALR = ((uint32_t)(-calibration) << RTC_CALR_CALM_Pos);
    }

    while (RTC->ISR & RTC_ISR_RECALPF)
    {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }
}

static void stm32wb_rtc_sync_routine(void)
{
    stm32wb_rtc_event_callback_t callback;
    void *context;
    uint32_t events;
    uint64_t time_offset, time_offset_previous;
    int32_t zone, zone_previous;
    uint16_t dst, dst_previous;
    int8_t leap_seconds, leap_seconds_previous;
    uint8_t status, status_previous;

    events = armv7m_atomic_swap(&stm32wb_rtc_device.events, 0);

    armv7m_atomic_orb(&stm32wb_rtc_device.status, events);

    time_offset = stm32wb_rtc_device.time_offset;
    zone = stm32wb_rtc_device.zone;
    dst = stm32wb_rtc_device.dst;
    leap_seconds = stm32wb_rtc_device.leap_seconds;
    status = stm32wb_rtc_device.status;
    
    do
    {
        time_offset_previous = time_offset;
        zone_previous = zone;
        dst_previous = dst;
        leap_seconds_previous = leap_seconds;
        status_previous = status;

        time_offset = stm32wb_rtc_device.time_offset;
        zone = stm32wb_rtc_device.zone;
        dst = stm32wb_rtc_device.dst;
        leap_seconds = stm32wb_rtc_device.leap_seconds;
        status = stm32wb_rtc_device.status;
    }
    while ((time_offset != time_offset_previous) ||
           (zone != zone_previous) ||
           (dst != dst_previous) ||
           (leap_seconds != leap_seconds_previous) ||
           (status != status_previous));
           
    RTC->BKP18R = (uint32_t)(time_offset / STM32WB_RTC_CLOCK_TICKS_PER_SECOND);
    RTC->BKP19R = ((((time_offset & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND - 1)) << STM32WB_RTC_BKP19R_TICKS_OFFSET_SHIFT) & STM32WB_RTC_BKP19R_TICKS_OFFSET_MASK) |
                   (((zone / 900) << STM32WB_RTC_BKP19R_ZONE_SHIFT) & STM32WB_RTC_BKP19R_ZONE_MASK) |
                   (((dst / 3600) << STM32WB_RTC_BKP19R_DST_SHIFT) & STM32WB_RTC_BKP19R_DST_MASK) |
                   ((leap_seconds << STM32WB_RTC_BKP19R_LEAP_SECONDS_SHIFT) & STM32WB_RTC_BKP19R_LEAP_SECONDS_MASK) |
                   ((status & STM32WB_RTC_STATUS_TIME_WRITTEN) ? STM32WB_RTC_BKP19R_TIME_WRITTEN : 0) |
                   ((status & STM32WB_RTC_STATUS_ZONE_WRITTEN) ? STM32WB_RTC_BKP19R_ZONE_WRITTEN : 0) |
                   ((status & STM32WB_RTC_STATUS_DST_WRITTEN) ? STM32WB_RTC_BKP19R_DST_WRITTEN : 0) |
                   ((status & STM32WB_RTC_STATUS_LEAP_SECONDS_WRITTEN) ? STM32WB_RTC_BKP19R_LEAP_SECONDS_WRITTEN : 0));

    callback = stm32wb_rtc_device.event_callback;
    context = stm32wb_rtc_device.event_context;
    
    if (callback)
    {
        (*callback)(context, events);
    }
}

static inline __attribute__((optimize("O3"),always_inline)) void __stm32wb_rtc_clock_capture(stm32wb_rtc_capture_t *p_capture)
{
    uint32_t rtc_ssr_previous, rtc_tr_previous, rtc_dr_previous, rtc_ssr, rtc_tr, rtc_dr;
    
    rtc_ssr = RTC->SSR;
    rtc_tr = RTC->TR;
    rtc_dr = RTC->DR;

    do
    {
        rtc_ssr_previous = rtc_ssr;
        rtc_tr_previous = rtc_tr;
        rtc_dr_previous = rtc_dr;

        rtc_ssr = RTC->SSR;
        rtc_tr = RTC->TR;
        rtc_dr = RTC->DR;
    }
    while ((rtc_ssr != rtc_ssr_previous) || (rtc_tr != rtc_tr_previous) || (rtc_dr != rtc_dr_previous));

    p_capture->dr = rtc_dr;
    p_capture->tr = rtc_tr;
    p_capture->ssr = rtc_ssr;
}

static inline __attribute__((optimize("O3"),always_inline)) uint64_t __stm32wb_rtc_clock_convert(const stm32wb_rtc_capture_t *capture, stm32wb_rtc_tod_t *p_tod)
{
    uint32_t year, month, day, hours, minutes, seconds, ticks;

    year    = stm32wb_rtc_bcd_to_int[(capture->dr >> RTC_DR_YU_Pos) & ((RTC_DR_YU_Msk | RTC_DR_YT_Msk) >> RTC_DR_YU_Pos)];
    month   = stm32wb_rtc_bcd_to_int[(capture->dr >> RTC_DR_MU_Pos) & ((RTC_DR_MU_Msk | RTC_DR_MT_Msk) >> RTC_DR_MU_Pos)];
    day     = stm32wb_rtc_bcd_to_int[(capture->dr >> RTC_DR_DU_Pos) & ((RTC_DR_DU_Msk | RTC_DR_DT_Msk) >> RTC_DR_DU_Pos)];
    hours   = stm32wb_rtc_bcd_to_int[(capture->tr >> RTC_TR_HU_Pos)  & ((RTC_TR_HU_Msk  | RTC_TR_HT_Msk)  >> RTC_TR_HU_Pos)];
    minutes = stm32wb_rtc_bcd_to_int[(capture->tr >> RTC_TR_MNU_Pos) & ((RTC_TR_MNU_Msk | RTC_TR_MNT_Msk) >> RTC_TR_MNU_Pos)];
    seconds = stm32wb_rtc_bcd_to_int[(capture->tr >> RTC_TR_SU_Pos)  & ((RTC_TR_SU_Msk  | RTC_TR_ST_Msk)  >> RTC_TR_SU_Pos)];
    ticks   = (STM32WB_RTC_PREDIV_S - 1) - (capture->ssr & (STM32WB_RTC_PREDIV_S - 1));

    if (p_tod)
    {
        p_tod->year    = year;
        p_tod->month   = month;
        p_tod->day     = day;
        p_tod->hours   = hours;
        p_tod->minutes = minutes;
        p_tod->seconds = seconds;
        p_tod->ticks   = ticks;
    }

    return ((uint64_t)(((((stm32wb_rtc_days_since_year[year] + stm32wb_rtc_days_since_month[year & 3][month] + (day - 1)) * 24)
                         + hours) * 60
                        + minutes) * 60
                       + seconds) * STM32WB_RTC_CLOCK_TICKS_PER_SECOND) | ticks;
}

static void __attribute__((optimize("O3"))) __stm32wb_rtc_clock_offset(stm32wb_rtc_tod_t *tod, uint32_t seconds, uint32_t ticks)
{
    uint32_t days, hours, minutes;

    days    = ((seconds >> 7) * 198842) >> 27; seconds -= (days    * 86400);
    hours   = (seconds * 37283) >> 27;         seconds -= (hours   *  3600);
    minutes = (seconds * 1118482) >> 26;       seconds -= (minutes *    60);
    
    tod->ticks += ticks;
                    
    if (tod->ticks >= STM32WB_RTC_CLOCK_TICKS_PER_SECOND)
    {
        tod->ticks -= STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
        tod->seconds += 1;
    }
                    
    tod->seconds += seconds;
                    
    if (tod->seconds >= 60)
    {
        tod->seconds -= 60;
        tod->minutes += 1;
    }
                    
    tod->minutes += minutes;
                    
    if (tod->minutes >= 60)
    {
        tod->minutes -= 60;
        tod->hours += 1;
    }
    
    tod->hours += hours;
    
    if (tod->hours >= 24)
    {
        tod->hours -= 24;
        tod->day += 1;
    }
                    
    tod->day += days;
    
    if (tod->day > stm32wb_rtc_days_in_month[tod->year & 3][tod->month])
    {
        tod->day -= stm32wb_rtc_days_in_month[tod->year & 3][tod->month];
        tod->month += 1;
        
        if (tod->month > 12)
        {
            tod->month -= 12;
            tod->year++;
        }
    }
}

void __attribute__((optimize("O3"))) stm32wb_rtc_clock_capture(stm32wb_rtc_capture_t *p_capture)
{
    __stm32wb_rtc_clock_capture(p_capture);
}

uint64_t __attribute__((optimize("O3"))) stm32wb_rtc_clock_convert(const stm32wb_rtc_capture_t *capture)
{
    return __stm32wb_rtc_clock_convert(capture, NULL) - stm32wb_rtc_device.clock_offset;
}

uint64_t __attribute__((optimize("O3"))) stm32wb_rtc_clock_read()
{
    stm32wb_rtc_capture_t capture;

    __stm32wb_rtc_clock_capture(&capture);

    return __stm32wb_rtc_clock_convert(&capture, NULL) - stm32wb_rtc_device.clock_offset;
}

void stm32wb_rtc_clock_to_time(uint64_t clock, uint32_t *p_seconds, uint32_t *p_ticks)
{
    uint64_t time_offset;
    uint32_t seconds, ticks, time_offset_l, time_offset_h;

    armv7m_atomic_load_2_restart((volatile uint32_t *)&stm32wb_rtc_device.time_offset, &time_offset_l, &time_offset_h);

    time_offset = (((uint64_t)time_offset_l << 0) | ((uint64_t)time_offset_h << 32));

    clock += (stm32wb_rtc_device.clock_offset + time_offset);

    seconds = clock / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
    ticks = clock & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND - 1);

    *p_seconds = seconds;
    *p_ticks = ticks;
}

uint64_t stm32wb_rtc_time_to_clock(uint32_t seconds, uint32_t ticks)
{
    uint64_t clock, time_offset;
    uint32_t time_offset_l, time_offset_h;

    clock = ((uint64_t)seconds * STM32WB_RTC_CLOCK_TICKS_PER_SECOND) + ticks;

    armv7m_atomic_load_2_restart((volatile uint32_t *)&stm32wb_rtc_device.time_offset, &time_offset_l, &time_offset_h);

    time_offset = (((uint64_t)time_offset_l << 0) | ((uint64_t)time_offset_h << 32));
    
    clock -= (stm32wb_rtc_device.clock_offset + time_offset);

    return clock;
}

void stm32wb_rtc_time_read(uint32_t *p_seconds, uint32_t *p_ticks)
{
    stm32wb_rtc_capture_t capture;
    uint64_t clock, time_offset;
    uint32_t seconds, ticks, time_offset_l, time_offset_h;
    
    __stm32wb_rtc_clock_capture(&capture);

    clock = __stm32wb_rtc_clock_convert(&capture, NULL);

    armv7m_atomic_load_2_restart((volatile uint32_t *)&stm32wb_rtc_device.time_offset, &time_offset_l, &time_offset_h);
    
    time_offset = (((uint64_t)time_offset_l << 0) | ((uint64_t)time_offset_h << 32));

    clock += time_offset;

    seconds = clock / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
    ticks = clock & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND -1);

    *p_seconds = seconds;
    *p_ticks = ticks;
}

static void __svc_stm32wb_rtc_time_write(uint32_t seconds)
{
    stm32wb_rtc_capture_t capture;
    uint64_t o_clock, n_clock, time_offset;
    uint32_t ticks, time_offset_l, time_offset_h;
    
    stm32wb_rtc_device.sync_routine = stm32wb_rtc_sync_routine;

    __stm32wb_rtc_clock_capture(&capture);

    o_clock = __stm32wb_rtc_clock_convert(&capture, NULL);

    armv7m_atomic_load_2_restart((volatile uint32_t *)&stm32wb_rtc_device.time_offset, &time_offset_l, &time_offset_h);
    
    time_offset = (((uint64_t)time_offset_l << 0) | ((uint64_t)time_offset_h << 32));

    o_clock += time_offset;

    ticks = o_clock & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND -1);

    n_clock = ((uint64_t)seconds * STM32WB_RTC_CLOCK_TICKS_PER_SECOND) + ticks;

    time_offset += (n_clock - o_clock);

    time_offset_l = (time_offset >> 0);
    time_offset_h = (time_offset >> 32);

    armv7m_atomic_store_2_restart((volatile uint32_t *)&stm32wb_rtc_device.time_offset, time_offset_l, time_offset_h);
    
    armv7m_atomic_or(&stm32wb_rtc_device.events, STM32WB_RTC_EVENT_TIME_WRITTEN);
    
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_SYNC);
}

void stm32wb_rtc_time_write(uint32_t seconds)
{
    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_1((uint32_t)&__svc_stm32wb_rtc_time_write, seconds);
    }
    else
    {
	__svc_stm32wb_rtc_time_write(seconds);
    }
}

static void __svc_stm32wb_rtc_time_adjust(int32_t ticks)
{
    uint64_t time_offset;
    uint32_t time_offset_l, time_offset_h;

    stm32wb_rtc_device.sync_routine = stm32wb_rtc_sync_routine;
    
    armv7m_atomic_load_2_restart((volatile uint32_t *)&stm32wb_rtc_device.time_offset, &time_offset_l, &time_offset_h);
    
    time_offset = (((uint64_t)time_offset_l << 0) | ((uint64_t)time_offset_h << 32));

    time_offset += ticks;

    time_offset_l = (time_offset >> 0);
    time_offset_h = (time_offset >> 32);

    armv7m_atomic_store_2_restart((volatile uint32_t *)&stm32wb_rtc_device.time_offset, time_offset_l, time_offset_h);
    
    armv7m_atomic_or(&stm32wb_rtc_device.events, STM32WB_RTC_EVENT_TIME_WRITTEN);
    
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_SYNC);
}

void stm32wb_rtc_time_adjust(int32_t ticks)
{
    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_1((uint32_t)&__svc_stm32wb_rtc_time_adjust, (uint32_t)ticks);
    }
    else
    {
	__svc_stm32wb_rtc_time_adjust(ticks);
    }
}

int32_t stm32wb_rtc_time_to_leap_seconds(uint32_t seconds)
{
    int32_t leap_seconds;
    
    if (seconds > STM32WB_RTC_LEAP_SECONDS_EXPIRATION)
    {
        return stm32wb_rtc_device.leap_seconds;
    }

    for (leap_seconds = STM32WB_RTC_LEAP_SECONDS_DEFAULT; leap_seconds >= 0; leap_seconds--)
    {
        if (seconds >= stm32wb_rtc_leap_seconds_table[leap_seconds])
        {
            break;
        }
    }
    
    return leap_seconds;
}

int32_t stm32wb_rtc_utc_to_leap_seconds(uint32_t seconds)
{
    int32_t leap_seconds;

    if ((seconds + STM32WB_RTC_LEAP_SECONDS_DEFAULT) > STM32WB_RTC_LEAP_SECONDS_EXPIRATION)
    {
        return stm32wb_rtc_device.leap_seconds;
    }

    for (leap_seconds = STM32WB_RTC_LEAP_SECONDS_DEFAULT; leap_seconds >= 0; leap_seconds--)
    {
        if ((seconds + leap_seconds) >= stm32wb_rtc_leap_seconds_table[leap_seconds])
        {
            break;
        }
    }

    return leap_seconds;
}

int32_t stm32wb_rtc_get_leap_seconds(void)
{
    return stm32wb_rtc_device.leap_seconds;
}

static void __svc_stm32wb_rtc_set_leap_seconds(int32_t leap_seconds)
{
    stm32wb_rtc_device.sync_routine = stm32wb_rtc_sync_routine;

    stm32wb_rtc_device.leap_seconds = leap_seconds;
    
    armv7m_atomic_or(&stm32wb_rtc_device.events, STM32WB_RTC_EVENT_LEAP_SECONDS_WRITTEN);

    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_SYNC);
}

void stm32wb_rtc_set_leap_seconds(int32_t leap_seconds)
{
    if (armv7m_core_is_in_thread())
    {
      armv7m_svcall_1((uint32_t)&__svc_stm32wb_rtc_set_leap_seconds, (uint32_t)leap_seconds);
    }
    else
    {
        __svc_stm32wb_rtc_set_leap_seconds(leap_seconds);
    }
}

int32_t stm32wb_rtc_get_zone(void)
{
    return stm32wb_rtc_device.zone;
}

static void __svc_stm32wb_rtc_set_zone(int32_t zone)
{
    stm32wb_rtc_device.sync_routine = stm32wb_rtc_sync_routine;

    stm32wb_rtc_device.zone = zone;

    armv7m_atomic_or(&stm32wb_rtc_device.events, STM32WB_RTC_EVENT_ZONE_WRITTEN);
    
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_SYNC);
}

void stm32wb_rtc_set_zone(int32_t zone)
{
    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_1((uint32_t)&__svc_stm32wb_rtc_set_zone, (uint32_t)zone);
    }
    else
    {
        __svc_stm32wb_rtc_set_zone(zone);
    }
}

uint32_t stm32wb_rtc_get_dst(void)
{
    return stm32wb_rtc_device.dst;
}

static void __svc_stm32wb_rtc_set_dst(uint32_t dst)
{
    stm32wb_rtc_device.sync_routine = stm32wb_rtc_sync_routine;

    stm32wb_rtc_device.dst = dst;

    armv7m_atomic_or(&stm32wb_rtc_device.events, STM32WB_RTC_EVENT_DST_WRITTEN);

    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_SYNC);
}

void stm32wb_rtc_set_dst(uint32_t dst)
{
    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_1((uint32_t)&__svc_stm32wb_rtc_set_dst, (uint32_t)dst);
    }
    else
    {
        __svc_stm32wb_rtc_set_dst(dst);
    }
}

void stm32wb_rtc_time_to_tod(uint32_t seconds, uint32_t ticks, stm32wb_rtc_tod_t *p_tod)
{
    uint32_t minutes, hours, days, months, years;

    p_tod->ticks = ticks;
    p_tod->seconds = seconds % 60; minutes = seconds / 60;
    p_tod->minutes = minutes % 60; hours = minutes / 60;
    p_tod->hours = hours % 24; days = hours / 24;
    
    years = (days / 365);
    
    if (((years * 365) + ((years + 3) / 4)) > days)
    {
        years--;
    }

    days -= ((years * 365) + ((years + 3) / 4));
    
    months = (days / 29);

    if ((months >= 12) || (stm32wb_rtc_days_since_month[years & 3][months +1] > days))
    {
        months--;
    }

    days -= stm32wb_rtc_days_since_month[years & 3][months +1];
    
    p_tod->day = days +1;
    p_tod->month = months +1;
    p_tod->year = years;
}

void stm32wb_rtc_tod_to_time(const stm32wb_rtc_tod_t *tod, uint32_t *p_seconds, uint32_t *p_ticks)
{
    *p_seconds = ((((((tod->year * 365) + ((tod->year + 3) / 4)) + 
                     stm32wb_rtc_days_since_month[tod->year & 3][tod->month] +
                     (tod->day - 1)) * 24 +
                    tod->hours) * 60 +
                   tod->minutes) * 60 +
                  tod->seconds);
    *p_ticks = tod->ticks;
}

static void  __attribute__((optimize("O3"))) stm32wb_rtc_alarm_routine(void)
{
    stm32wb_rtc_capture_t capture;
    stm32wb_rtc_tod_t tod;
    stm32wb_rtc_alarm_callback_t callback;
    void *context;
    uint64_t clock, reference, timeout;
    uint32_t seconds, ticks, alrmr, alrmssr;

    if (stm32wb_rtc_device.alarm_modify)
    {
        if (stm32wb_rtc_device.alarm_busy)
        {
            armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
            
            RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);
            
            stm32wb_rtc_device.alarm_busy = false;
            stm32wb_rtc_device.alarm_event = false;
        }

        stm32wb_rtc_device.alarm_modify = false;

        clock = stm32wb_rtc_device.alarm_clock;

        if (!stm32wb_rtc_device.alarm_modify)
        {
            __stm32wb_rtc_alarm_clock = clock;

            if (clock)
            {
                __stm32wb_rtc_clock_capture(&capture);
                
                reference = __stm32wb_rtc_clock_convert(&capture, &tod);
                
                if (clock <= reference)
                {
                    stm32wb_rtc_device.alarm_event = true;
                }
                else
                {
                    timeout = clock - reference;
                    
                    if (timeout < 2)
                    {
                        timeout = 2;
                    }
                    
                    clock = reference + timeout;

                    seconds = timeout / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
                    ticks   = timeout & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND - 1);

                    __stm32wb_rtc_clock_offset(&tod, seconds, ticks);
                    
                    alrmr = ((stm32wb_rtc_int_to_bcd[tod.seconds] << RTC_ALRMAR_SU_Pos) |
                             (stm32wb_rtc_int_to_bcd[tod.minutes] << RTC_ALRMAR_MNU_Pos) |
                             (stm32wb_rtc_int_to_bcd[tod.hours] << RTC_ALRMAR_HU_Pos) |
                             (stm32wb_rtc_int_to_bcd[tod.day] << RTC_ALRMAR_DU_Pos));
                    
                    alrmssr = ((STM32WB_RTC_PREDIV_S - 1) - tod.ticks) | STM32WB_RTC_ALRMSSR_MASKSS;

                    while (!(RTC->ISR & RTC_ISR_ALRAWF))
                    {
                        __NOP();
                        __NOP();
                        __NOP();
                        __NOP();
                    }
                    
                    RTC->ALRMAR = alrmr;
                    RTC->ALRMASSR = alrmssr;
                    
                    stm32wb_rtc_device.alarm_busy = true;

                    armv7m_atomic_or(&RTC->CR, (RTC_CR_ALRAIE | RTC_CR_ALRAE));
            
                    if (!stm32wb_rtc_device.alarm_event)
                    {
                        __stm32wb_rtc_clock_capture(&capture);
                        
                        if (!stm32wb_rtc_device.alarm_event)
                        {
                            reference = __stm32wb_rtc_clock_convert(&capture, NULL);
                            
                            if (clock <= (reference + 1))
                            {
                                if (!stm32wb_rtc_device.alarm_event)
                                {
                                    armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
                                    
                                    RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);
                                    
                                    stm32wb_rtc_device.alarm_busy = false;
                                    stm32wb_rtc_device.alarm_event = false;
                                    
                                    // retry
                                    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_ALARM);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    if (stm32wb_rtc_device.alarm_event)
    {
        stm32wb_rtc_device.alarm_event = false;
        
        clock = stm32wb_rtc_device.alarm_clock;
        callback = stm32wb_rtc_device.alarm_callback;
        context = stm32wb_rtc_device.alarm_context;
        
        if (!stm32wb_rtc_device.alarm_modify)
        {
            __stm32wb_rtc_clock_capture(&capture);
            
            reference = __stm32wb_rtc_clock_convert(&capture, NULL);
            
            if (clock <= reference)
            {
                armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
                
                RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);
                
                stm32wb_rtc_device.alarm_busy = false;
                stm32wb_rtc_device.alarm_event = false;

                __stm32wb_rtc_alarm_clock = 0;
                
                if (callback)
                {
                    (*callback)(context);
                }
            }
        }
    }
}

static void __svc_stm32wb_rtc_alarm_start(uint32_t seconds, uint32_t ticks, stm32wb_rtc_alarm_callback_t callback, void *context)
{
    uint64_t clock, time_offset;
    uint32_t clock_l, clock_h, time_offset_l, time_offset_h;
    
    if (stm32wb_rtc_device.alarm_busy)
    {
        armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
        
        RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);
        
        stm32wb_rtc_device.alarm_busy = false;
        stm32wb_rtc_device.alarm_event = false;
    }

    armv7m_atomic_load_2_restart((volatile uint32_t *)&stm32wb_rtc_device.time_offset, &time_offset_l, &time_offset_h);

    time_offset = (((uint64_t)time_offset_l << 0) | ((uint64_t)time_offset_h << 32));

    clock = (((uint64_t)seconds * STM32WB_RTC_CLOCK_TICKS_PER_SECOND) + (uint64_t)ticks) - time_offset;

    clock_l = (uint32_t)(clock >> 0);
    clock_h = (uint32_t)(clock >> 32);

    stm32wb_rtc_device.alarm_modify = true;

    armv7m_atomic_store_4_restart((volatile uint32_t *)&stm32wb_rtc_device.alarm_clock, clock_l, clock_h, (uint32_t)callback, (uint32_t)context);
    
    stm32wb_rtc_device.alarm_routine = stm32wb_rtc_alarm_routine;

    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_ALARM);
}

static void __svc_stm32wb_rtc_alarm_stop(void)
{
    if (stm32wb_rtc_device.alarm_busy)
    {
        armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
        
        RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);
        
        stm32wb_rtc_device.alarm_busy = false;
        stm32wb_rtc_device.alarm_event = false;
    }

    stm32wb_rtc_device.alarm_modify = true;

    armv7m_atomic_store_2_restart((volatile uint32_t *)&stm32wb_rtc_device.alarm_clock, 0, 0);
    
    stm32wb_rtc_device.alarm_routine = stm32wb_rtc_alarm_routine;

    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_ALARM);
}

void stm32wb_rtc_alarm_start(uint32_t seconds, uint32_t ticks, stm32wb_rtc_alarm_callback_t callback, void *context)
{
    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_4((uint32_t)&__svc_stm32wb_rtc_alarm_start, seconds, ticks, (uint32_t)callback, (uint32_t)context);
    }
    else
    {
        __svc_stm32wb_rtc_alarm_start(seconds, ticks, callback, context);
    }
}

void stm32wb_rtc_alarm_stop(void)
{
    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_0((uint32_t)&__svc_stm32wb_rtc_alarm_stop);
    }
    else
    {
        __svc_stm32wb_rtc_alarm_stop();
    }
}

bool stm32wb_rtc_alarm_busy(void)
{
    return stm32wb_rtc_device.alarm_busy;
}

bool stm32wb_rtc_tamp_catch(uint16_t pin, uint32_t control, stm32wb_rtc_tamp_callback_t callback, void *context)
{
    pin &= STM32WB_GPIO_PIN_IO_MASK;

    if (pin == STM32WB_GPIO_PIN_PC13_RTC_TAMP1)
    {
        armv7m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1TRG | RTC_TAMPCR_TAMP1E));

        RTC->ISR = ~(RTC_ISR_TAMP1F | RTC_ISR_INIT);

        __stm32wb_exti_catch(STM32WB_EXTI_CHANNEL_INDEX_RTC_TAMP1, ((control & STM32WB_RTC_TAMP_CONTROL_PRIORITY_MASK) >> STM32WB_RTC_TAMP_CONTROL_PRIORITY_SHIFT), callback, context);

        if (control & (STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING | STM32WB_RTC_TAMP_CONTROL_EDGE_RISING))
        {
            armv7m_atomic_or(&RTC->TAMPCR, (RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1E | ((control & STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING) ? RTC_TAMPCR_TAMP1TRG : 0)));
        }

        return true;
    }

    if (pin == STM32WB_GPIO_PIN_PA0_RTC_TAMP2)
    {
        armv7m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2TRG | RTC_TAMPCR_TAMP2E));

        RTC->ISR = ~(RTC_ISR_TAMP2F | RTC_ISR_INIT);

        __stm32wb_exti_catch(STM32WB_EXTI_CHANNEL_INDEX_RTC_TAMP2, ((control & STM32WB_RTC_TAMP_CONTROL_PRIORITY_MASK) >> STM32WB_RTC_TAMP_CONTROL_PRIORITY_SHIFT), callback, context);

        if (control & (STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING | STM32WB_RTC_TAMP_CONTROL_EDGE_RISING))
        {
            armv7m_atomic_or(&RTC->TAMPCR, (RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2E | ((control & STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING) ? RTC_TAMPCR_TAMP2TRG : 0)));
        }

        return true;
    }

    if (pin == STM32WB_GPIO_PIN_PC12_RTC_TAMP3)
    {
        armv7m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP3IE | RTC_TAMPCR_TAMP3TRG | RTC_TAMPCR_TAMP3E));

        RTC->ISR = ~(RTC_ISR_TAMP3F | RTC_ISR_INIT);

        __stm32wb_exti_catch(STM32WB_EXTI_CHANNEL_INDEX_RTC_TAMP3, ((control & STM32WB_RTC_TAMP_CONTROL_PRIORITY_MASK) >> STM32WB_RTC_TAMP_CONTROL_PRIORITY_SHIFT), callback, context);

        if (control & (STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING | STM32WB_RTC_TAMP_CONTROL_EDGE_RISING))
        {
            armv7m_atomic_or(&RTC->TAMPCR, (RTC_TAMPCR_TAMP3IE | RTC_TAMPCR_TAMP3E | ((control & STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING) ? RTC_TAMPCR_TAMP3TRG : 0)));
        }

        return true;
    }
    
    return false;
}

void stm32wb_rtc_standby_enter(uint32_t timeout)
{
    stm32wb_rtc_capture_t capture;
    stm32wb_rtc_tod_t tod;
    uint32_t seconds, ticks;
    
    /* Called with interrupts disabled.
     */

    RTC->CR &= ~(RTC_CR_TSIE | RTC_CR_WUTIE | RTC_CR_ALRBIE | RTC_CR_TSE | RTC_CR_WUTE | RTC_CR_ALRBE);
    RTC->TAMPCR &= ~(RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1E | RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2E | RTC_TAMPCR_TAMP3IE | RTC_TAMPCR_TAMP3E);
    RTC->ISR = 0;
    
    if (timeout != 0xffffffff)
    {
        if (timeout > 2419200000)
        {
            seconds = 2419200000;
        }

        seconds = timeout / 1000;
        ticks = ((timeout - seconds * 1000) * STM32WB_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000;
        
        __stm32wb_rtc_clock_capture(&capture);
        __stm32wb_rtc_clock_convert(&capture, &tod);
        __stm32wb_rtc_clock_offset(&tod, seconds, ticks);
        
        while (!(RTC->ISR & RTC_ISR_ALRBWF))
        {
            __NOP();
            __NOP();
            __NOP();
            __NOP();
        }
        
        RTC->ALRMBR = ((stm32wb_rtc_int_to_bcd[tod.seconds] << RTC_ALRMAR_SU_Pos) |
                       (stm32wb_rtc_int_to_bcd[tod.minutes] << RTC_ALRMAR_MNU_Pos) |
                       (stm32wb_rtc_int_to_bcd[tod.hours] << RTC_ALRMAR_HU_Pos) |
                       (stm32wb_rtc_int_to_bcd[tod.day] << RTC_ALRMAR_DU_Pos));
        
        RTC->ALRMBSSR = ((STM32WB_RTC_PREDIV_S - 1) - tod.ticks) | STM32WB_RTC_ALRMSSR_MASKSS;
        
        RTC->CR |= (RTC_CR_ALRBIE | RTC_CR_ALRBE);
    }

    EXTI->PR1 = EXTI_PR1_PIF18 | EXTI_PR1_PIF19;
    
    /* Lock RTC throu reset */
    RTC->WPR = 0x00;
}

uint32_t stm32wb_rtc_standby_leave(uint32_t wakeup)
{
    stm32wb_rtc_capture_t capture;
    uint64_t reference;

    if (RTC->ISR & RTC_ISR_ALRAF)
    {
        __stm32wb_rtc_clock_capture(&capture);
            
        reference = __stm32wb_rtc_clock_convert(&capture, NULL);
            
        if (__stm32wb_rtc_alarm_clock <= reference)
        {
            wakeup |= STM32WB_SYSTEM_WAKEUP_ALARM;
        }
    }

    if (RTC->ISR & RTC_ISR_ALRBF)
    {
        wakeup |= STM32WB_SYSTEM_WAKEUP_TIMEOUT;
    }

    if (wakeup)
    {
	RTC->WPR = 0xca;
	RTC->WPR = 0x53;

        RTC->CR &= ~(RTC_CR_ALRBIE | RTC_CR_ALRAIE | RTC_CR_ALRBE | RTC_CR_ALRAE);
        RTC->ISR = 0;

        RTC->WPR = 0x00;
    }

    return wakeup;
}

void stm32wb_rtc_reset(void)
{
    /* Called with interrupts disabled.
     */

    RTC->CR &= ~(RTC_CR_TSIE | RTC_CR_WUTIE | RTC_CR_ALRBIE | RTC_CR_ALRAIE | RTC_CR_TSE | RTC_CR_WUTE | RTC_CR_ALRBE | RTC_CR_ALRAE);
    RTC->TAMPCR &= ~(RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1E | RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2E | RTC_TAMPCR_TAMP3IE | RTC_TAMPCR_TAMP3E);
    RTC->ISR = 0;

    EXTI->PR1 = EXTI_PR1_PIF17 | EXTI_PR1_PIF18 | EXTI_PR1_PIF19;

    /* Lock RTC throu reset */
    RTC->WPR = 0x00;
}

/*******************************************************************************************************************/

void RTC_Alarm_IRQHandler(void)
{
    do
    {
        EXTI->PR1 = EXTI_PR1_PIF17;
        
        if (RTC->ISR & RTC_ISR_ALRAF)
        {
            RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);

            stm32wb_rtc_device.alarm_event = 1;
                
            armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_ALARM);
        }
    }
    while (RTC->ISR & RTC_ISR_ALRAF);
    
    __DSB();
}

void TAMP_STAMP_LSECSS_IRQHandler(void)
{
    do
    {
	EXTI->PR1 = EXTI_PR1_PIF18;

	if (RTC->ISR & RTC_ISR_TAMP1F)
	{
	    RTC->ISR = ~(RTC_ISR_TAMP1F | RTC_ISR_INIT);

            __stm32wb_exti_interrupt(STM32WB_EXTI_CHANNEL_MASK_RTC_TAMP1);
	}
            
	if (RTC->ISR & RTC_ISR_TAMP2F)
	{
	    RTC->ISR = ~(RTC_ISR_TAMP2F | RTC_ISR_INIT);

            __stm32wb_exti_interrupt(STM32WB_EXTI_CHANNEL_MASK_RTC_TAMP2);
	}

	if (RTC->ISR & RTC_ISR_TAMP3F)
	{
	    RTC->ISR = ~(RTC_ISR_TAMP3F | RTC_ISR_INIT);

            __stm32wb_exti_interrupt(STM32WB_EXTI_CHANNEL_MASK_RTC_TAMP3);
	}
    }
    while (RTC->ISR & (RTC_ISR_TAMP1F | RTC_ISR_TAMP2F | RTC_ISR_TAMP3F));

    __DSB();
}

void RTC_SYNC_SWIHandler(void)
{
    stm32wb_rtc_sync_routine_t routine;

    routine = stm32wb_rtc_device.sync_routine;
    
    if (routine)
    {
        (*routine)();
    }
}

void RTC_ALARM_SWIHandler(void)
{
    stm32wb_rtc_alarm_routine_t routine;

    routine = stm32wb_rtc_device.alarm_routine;
    
    if (routine)
    {
        (*routine)();
    }
}
