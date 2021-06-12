/*
 * Copyright (c) 2017-2021 Thomas Roell.  All rights reserved.
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

typedef void (*stm32wb_rtc_modify_routine_t)(void);
typedef void (*stm32wb_rtc_alarm_routine_t)(void);
typedef void (*stm32wb_rtc_timer_routine_t)(void);

typedef struct _stm32wb_rtc_time_t {
    volatile uint32_t                      seconds_offset; 
    volatile int32_t                       local_offset;
    volatile uint16_t                      ticks_offset;
    volatile int8_t                        utc_offset;
    volatile uint8_t                       status;
} stm32wb_rtc_time_t;

typedef struct _stm32wb_rtc_alarm_t {
    volatile uint32_t                      seconds;
    volatile uint32_t                      ticks;
    volatile stm32wb_rtc_alarm_callback_t  callback;
    void * volatile                        context;
} stm32wb_rtc_alarm_t;

typedef struct _stm32wb_rtc_device_t {
    uint64_t                               clock_offset;
    stm32wb_rtc_time_t                     time_current; 
    stm32wb_rtc_time_t                     time_next; 
    volatile uint32_t                      time_sequence;
    uint64_t                               alarm_clock;
    stm32wb_rtc_alarm_t                    alarm_current;
    stm32wb_rtc_alarm_t                    alarm_next;
    volatile uint32_t                      alarm_sequence[2];
    volatile uint8_t                       alarm_active;
    volatile uint8_t                       alarm_busy;
    volatile uint8_t                       alarm_continue;
    volatile uint8_t                       alarm_event;
    uint64_t                               timer_clock;
    stm32wb_rtc_timer_t                    *timer_queue;
    stm32wb_rtc_timer_t * volatile         timer_modify;
    volatile uint8_t                       timer_busy;
    volatile uint8_t                       wakeup_busy;
    volatile stm32wb_rtc_wakeup_callback_t wakeup_callback;
    void * volatile                        wakeup_context;
    volatile stm32wb_rtc_callback_t        tamp1_callback;
    void * volatile                        tamp1_context;
    volatile stm32wb_rtc_callback_t        tamp2_callback;
    void * volatile                        tamp2_context;
    volatile stm32wb_rtc_callback_t        tamp3_callback;
    void * volatile                        tamp3_context;
    volatile stm32wb_rtc_modify_routine_t  modify_routine;
    volatile stm32wb_rtc_alarm_routine_t   alarm_routine;
    volatile stm32wb_rtc_timer_routine_t   timer_routine;
} stm32wb_rtc_device_t;

static stm32wb_rtc_device_t stm32wb_rtc_device;

#define STM32WB_RTC_TIMER_SENTINEL ((stm32wb_rtc_timer_t*)0xffffffff)
#define STM32WB_RTC_TIMER_INVALID  (0x80000000)

static void stm32wb_rtc_modify_routine();
static void stm32wb_rtc_alarm_routine();
static void stm32wb_rtc_timer_routine();

extern uint8_t __RTC_EPOCH__;
extern uint8_t __RTC_ZONE__;
extern uint8_t __RTC_DST__;

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

static const uint32_t stm32wb_rtc_utc_offset_table[] = {
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
#define STM32WB_RTC_UTC_OFFSET_DEFAULT          18
#define STM32WB_RTC_UTC_OFFSET_EXPIRATION       1293494417 // 31 Dec 2020 23:59:59 in GPS TIME

void __stm32wb_rtc_initialize(void)
{
    stm32wb_rtc_capture_t capture;
    
    if (!(RCC->BDCR & RCC_BDCR_RTCEN))
    {
        /* Use LSE/LSI as source for RTC */
        
	RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | ((stm32wb_system_lseclk() ? RCC_BDCR_RTCSEL_0 : RCC_BDCR_RTCSEL_1) | RCC_BDCR_RTCEN);

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
	RTC->BKP17R = stm32wb_system_hseclk() ? STM32WB_RTC_BKP17R_HSECLK : 0;
	RTC->BKP18R = (uint32_t)&__RTC_EPOCH__ - 315964800 + STM32WB_RTC_UTC_OFFSET_DEFAULT;
	RTC->BKP19R = (((STM32WB_RTC_UTC_OFFSET_DEFAULT << STM32WB_RTC_BKP19R_UTC_OFFSET_SHIFT) & STM32WB_RTC_BKP19R_UTC_OFFSET_MASK) |
                       (((((int32_t)&__RTC_ZONE__ + (int32_t)&__RTC_DST__) / 60) << STM32WB_RTC_BKP19R_LOCAL_OFFSET_SHIFT) & STM32WB_RTC_BKP19R_LOCAL_OFFSET_MASK));
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
    stm32wb_rtc_device.time_current.seconds_offset = RTC->BKP18R;
    stm32wb_rtc_device.time_current.local_offset = ((int32_t)(RTC->BKP19R << (32 - STM32WB_RTC_BKP19R_LOCAL_OFFSET_SHIFT - STM32WB_RTC_BKP19R_LOCAL_OFFSET_SIZE)) >> (32 - STM32WB_RTC_BKP19R_LOCAL_OFFSET_SIZE)) * 60;
    stm32wb_rtc_device.time_current.ticks_offset = (RTC->BKP19R & STM32WB_RTC_BKP19R_TICKS_OFFSET_MASK) >> STM32WB_RTC_BKP19R_TICKS_OFFSET_SHIFT;
    stm32wb_rtc_device.time_current.utc_offset = (int32_t)(RTC->BKP19R << (32 - STM32WB_RTC_BKP19R_UTC_OFFSET_SHIFT - STM32WB_RTC_BKP19R_UTC_OFFSET_SIZE)) >> (32 - STM32WB_RTC_BKP19R_UTC_OFFSET_SIZE);
    stm32wb_rtc_device.time_current.status = (((RTC->BKP19R & STM32WB_RTC_BKP19R_TIME_WRITTEN) ? STM32WB_RTC_STATUS_TIME_INTERNAL : 0) |
					      ((RTC->BKP19R & STM32WB_RTC_BKP19R_UTC_OFFSET_WRITTEN) ? STM32WB_RTC_STATUS_UTC_OFFSET_INTERNAL : 0) |
					      ((RTC->BKP19R & STM32WB_RTC_BKP19R_LOCAL_OFFSET_WRITTEN) ? STM32WB_RTC_STATUS_LOCAL_OFFSET_INTERNAL : 0));

    stm32wb_rtc_device.time_next = stm32wb_rtc_device.time_current;
    
    stm32wb_rtc_device.timer_queue = NULL;
    stm32wb_rtc_device.timer_modify = STM32WB_RTC_TIMER_SENTINEL;

    RTC->CR = (RTC->CR & ~RTC_CR_WUCKSEL) | RTC_CR_WUCKSEL_2;

    EXTI->RTSR1 |= (EXTI_RTSR1_RT17 | EXTI_RTSR1_RT18 | EXTI_RTSR1_RT19);
    EXTI->IMR1 |= (EXTI_IMR1_IM17 | EXTI_IMR1_IM18 | EXTI_IMR1_IM19);
}

uint32_t stm32wb_rtc_status(void)
{
    return stm32wb_rtc_device.time_current.status;
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

static void stm32wb_rtc_modify_routine(void)
{
    stm32wb_rtc_alarm_routine_t routine;
    uint32_t sequence, n_seconds;
    int32_t n_local_offset;
    uint16_t n_ticks;
    int8_t n_utc_offset;
    uint8_t n_status;

    do
    {
	sequence = stm32wb_rtc_device.time_sequence;

	n_seconds = stm32wb_rtc_device.time_next.seconds_offset;
	n_local_offset = stm32wb_rtc_device.time_next.local_offset;
	n_ticks = stm32wb_rtc_device.time_next.ticks_offset;
	n_utc_offset = stm32wb_rtc_device.time_next.utc_offset;
	n_status = stm32wb_rtc_device.time_next.status;
    }
    while (stm32wb_rtc_device.time_sequence != sequence);
    
    stm32wb_rtc_device.time_current.seconds_offset = n_seconds;
    stm32wb_rtc_device.time_current.local_offset = n_local_offset;
    stm32wb_rtc_device.time_current.ticks_offset = n_ticks;
    stm32wb_rtc_device.time_current.utc_offset = n_utc_offset;
    stm32wb_rtc_device.time_current.status = n_status; 

    RTC->BKP18R = n_seconds;
    RTC->BKP19R = (((n_ticks << STM32WB_RTC_BKP19R_TICKS_OFFSET_SHIFT) & STM32WB_RTC_BKP19R_TICKS_OFFSET_MASK) |
                   ((n_utc_offset << STM32WB_RTC_BKP19R_UTC_OFFSET_SHIFT) & STM32WB_RTC_BKP19R_UTC_OFFSET_MASK) |
                   (((n_local_offset / 60) << STM32WB_RTC_BKP19R_LOCAL_OFFSET_SHIFT) & STM32WB_RTC_BKP19R_LOCAL_OFFSET_MASK) |
                   ((n_status & (STM32WB_RTC_STATUS_TIME_INTERNAL | STM32WB_RTC_STATUS_TIME_EXTERNAL)) ? STM32WB_RTC_BKP19R_TIME_WRITTEN : 0) |
                   ((n_status & (STM32WB_RTC_STATUS_UTC_OFFSET_INTERNAL | STM32WB_RTC_STATUS_UTC_OFFSET_EXTERNAL)) ? STM32WB_RTC_BKP19R_UTC_OFFSET_WRITTEN : 0) |
                   ((n_status & (STM32WB_RTC_STATUS_LOCAL_OFFSET_INTERNAL | STM32WB_RTC_STATUS_LOCAL_OFFSET_EXTERNAL)) ? STM32WB_RTC_BKP19R_LOCAL_OFFSET_WRITTEN : 0));

    routine = stm32wb_rtc_device.alarm_routine;
    
    if (routine)
    {
        (*routine)();
    }
}

int32_t stm32wb_rtc_get_utc_offset(void)
{
    return stm32wb_rtc_device.time_current.utc_offset;
}

static void __svc_stm32wb_rtc_set_utc_offset(int32_t utc_offset, bool external)
{
    uint32_t sequence;
    int8_t o_utc_offset, n_utc_offset;
    uint8_t o_status, n_status;

    do
    {
	o_utc_offset = stm32wb_rtc_device.time_next.utc_offset;
	o_status = stm32wb_rtc_device.time_next.status;
    }
    while ((stm32wb_rtc_device.time_next.utc_offset != o_utc_offset) || (stm32wb_rtc_device.time_next.status != o_status));

    if ((o_status & STM32WB_RTC_STATUS_UTC_OFFSET_EXTERNAL) && !external)
    {
        return;
    }
    
    n_utc_offset = utc_offset;
    n_status = ((o_status & ~(STM32WB_RTC_STATUS_UTC_OFFSET_INTERNAL | STM32WB_RTC_STATUS_UTC_OFFSET_EXTERNAL)) |
		(external ? STM32WB_RTC_STATUS_UTC_OFFSET_EXTERNAL : STM32WB_RTC_STATUS_UTC_OFFSET_INTERNAL));

    if ((n_utc_offset != o_utc_offset) || (n_status != o_status))
    {
	stm32wb_rtc_device.modify_routine = stm32wb_rtc_modify_routine;

	do
	{
	    sequence = stm32wb_rtc_device.time_sequence;

	    stm32wb_rtc_device.time_next.utc_offset = n_utc_offset;
	    stm32wb_rtc_device.time_next.status = n_status;
	}
	while (armv7m_atomic_cas(&stm32wb_rtc_device.time_sequence, sequence, (sequence + 1)) != sequence);

	armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_MODIFY);
    }
}

void stm32wb_rtc_set_utc_offset(int32_t utc_offset, bool external)
{
    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_2((uint32_t)&__svc_stm32wb_rtc_set_utc_offset, (uint32_t)utc_offset, (uint32_t)external);
    }
    else
    {
        __svc_stm32wb_rtc_set_utc_offset(utc_offset, external);
    }
}

int32_t stm32wb_rtc_get_local_offset(void)
{
    return stm32wb_rtc_device.time_current.local_offset;
}

static void __svc_stm32wb_rtc_set_local_offset(int32_t local_offset, bool external)
{
    uint32_t sequence;
    int32_t o_local_offset, n_local_offset;
    uint8_t o_status, n_status;

    do
    {
	o_local_offset = stm32wb_rtc_device.time_next.local_offset;
	o_status = stm32wb_rtc_device.time_next.status;
    }
    while ((stm32wb_rtc_device.time_next.local_offset != o_local_offset) || (stm32wb_rtc_device.time_next.status != o_status));

    if ((o_status & STM32WB_RTC_STATUS_LOCAL_OFFSET_EXTERNAL) && !external)
    {
        return;
    }
    
    n_local_offset = local_offset;
    n_status = ((o_status & ~(STM32WB_RTC_STATUS_LOCAL_OFFSET_INTERNAL | STM32WB_RTC_STATUS_LOCAL_OFFSET_EXTERNAL)) |
		(external ? STM32WB_RTC_STATUS_LOCAL_OFFSET_EXTERNAL : STM32WB_RTC_STATUS_LOCAL_OFFSET_INTERNAL));

    if ((n_local_offset != o_local_offset) || (n_status != o_status))
    {
	stm32wb_rtc_device.modify_routine = stm32wb_rtc_modify_routine;

	do
	{
	    sequence = stm32wb_rtc_device.time_sequence;

	    stm32wb_rtc_device.time_next.local_offset = n_local_offset;
	    stm32wb_rtc_device.time_next.status = n_status;
	}
	while (armv7m_atomic_cas(&stm32wb_rtc_device.time_sequence, sequence, (sequence + 1)) != sequence);

	armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_MODIFY);
    }
}

void stm32wb_rtc_set_local_offset(int32_t local_offset, bool external)
{
    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_2((uint32_t)&__svc_stm32wb_rtc_set_local_offset, (uint32_t)local_offset, (uint32_t)external);
    }
    else
    {
        __svc_stm32wb_rtc_set_local_offset(local_offset, external);
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
    uint32_t seconds, ticks, o_seconds, o_ticks;

    do
    {
	o_seconds = stm32wb_rtc_device.time_current.seconds_offset;
	o_ticks = stm32wb_rtc_device.time_current.ticks_offset;
    }
    while ((stm32wb_rtc_device.time_current.seconds_offset != o_seconds) || (stm32wb_rtc_device.time_current.ticks_offset != o_ticks));

    clock += stm32wb_rtc_device.clock_offset;

    seconds = clock / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
    ticks = clock & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND - 1);

    seconds += o_seconds;
    ticks += o_ticks;

    if (ticks >= STM32WB_RTC_CLOCK_TICKS_PER_SECOND)
    {
        seconds++;
        ticks -= STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
    }

    *p_seconds = seconds;
    *p_ticks = ticks;
}

uint64_t stm32wb_rtc_time_to_clock(uint32_t seconds, uint32_t ticks)
{
    uint64_t clock;
    uint32_t o_seconds, o_ticks;
    
    do
    {
	o_seconds = stm32wb_rtc_device.time_current.seconds_offset;
	o_ticks = stm32wb_rtc_device.time_current.ticks_offset;
    }
    while ((stm32wb_rtc_device.time_current.seconds_offset != o_seconds) || (stm32wb_rtc_device.time_current.ticks_offset != o_ticks));

    if (ticks < o_ticks)
    {
        seconds--;
        ticks += STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
    }

    seconds -= o_seconds;
    ticks -= o_ticks;

    clock = ((uint64_t)seconds * STM32WB_RTC_CLOCK_TICKS_PER_SECOND) | (uint64_t)ticks;

    clock -= stm32wb_rtc_device.clock_offset;

    return clock;
}

void stm32wb_rtc_time_read(uint32_t *p_seconds, uint32_t *p_ticks)
{
    stm32wb_rtc_capture_t capture;
    uint64_t clock;
    uint32_t seconds, ticks, o_seconds, o_ticks;
    
    __stm32wb_rtc_clock_capture(&capture);

    clock = __stm32wb_rtc_clock_convert(&capture, NULL);

    do
    {
	o_seconds = stm32wb_rtc_device.time_current.seconds_offset;
	o_ticks = stm32wb_rtc_device.time_current.ticks_offset;
    }
    while ((stm32wb_rtc_device.time_current.seconds_offset != o_seconds) || (stm32wb_rtc_device.time_current.ticks_offset != o_ticks));

    seconds = clock / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
    ticks = clock & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND -1);

    seconds += o_seconds;
    ticks += o_ticks;

    if (ticks >= STM32WB_RTC_CLOCK_TICKS_PER_SECOND)
    {
        seconds++;
        ticks -= STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
    }

    *p_seconds = seconds;
    *p_ticks = ticks;
}

static void __svc_stm32wb_rtc_time_write(uint32_t clock_l, uint32_t clock_h, uint32_t seconds, uint32_t ticks)
{
    uint64_t clock;
    uint32_t sequence, i_seconds, c_seconds, t_seconds, o_seconds, n_seconds, i_ticks, c_ticks, t_ticks, o_ticks, n_ticks;
    uint8_t o_status, n_status;
    bool external;

    external = !!(ticks & 0x00008000);
    ticks = ticks & 0x00007fff;
    clock = (((uint64_t)clock_l << 0) | ((uint64_t)clock_h << 32));
    
    do
    {
	o_seconds = stm32wb_rtc_device.time_next.seconds_offset;
	o_ticks = stm32wb_rtc_device.time_next.ticks_offset;
	o_status = stm32wb_rtc_device.time_next.status;
    }
    while ((stm32wb_rtc_device.time_next.seconds_offset != o_seconds) || (stm32wb_rtc_device.time_next.ticks_offset != o_ticks) || (stm32wb_rtc_device.time_next.status != o_status));

    if ((o_status & STM32WB_RTC_STATUS_TIME_EXTERNAL) && !external)
    {
        return;
    }

    c_seconds = clock / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
    c_ticks = clock & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND -1);

    i_seconds = c_seconds + o_seconds;
    i_ticks = c_ticks + o_ticks;

    if (i_ticks >= STM32WB_RTC_CLOCK_TICKS_PER_SECOND)
    {
        i_seconds++;
        i_ticks -= STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
    }

    t_seconds = seconds;
    t_ticks = ticks;

    if (t_ticks < c_ticks)
    {
        t_seconds--;
        t_ticks += STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
    }

    n_seconds = t_seconds - c_seconds;
    n_ticks = t_ticks - c_ticks;
    n_status = ((o_status & ~(STM32WB_RTC_STATUS_TIME_INTERNAL | STM32WB_RTC_STATUS_TIME_EXTERNAL)) |
		(external ? STM32WB_RTC_STATUS_TIME_EXTERNAL : STM32WB_RTC_STATUS_TIME_INTERNAL));

    if ((o_seconds != n_seconds) || (o_ticks != n_ticks) || (o_status != n_status))
    {
	stm32wb_rtc_device.modify_routine = stm32wb_rtc_modify_routine;

	do
	{
	    sequence = stm32wb_rtc_device.time_sequence;

	    stm32wb_rtc_device.time_next.seconds_offset = n_seconds;
	    stm32wb_rtc_device.time_next.ticks_offset = n_ticks;
	    stm32wb_rtc_device.time_next.status = n_status;
	}
	while (armv7m_atomic_cas(&stm32wb_rtc_device.time_sequence, sequence, (sequence + 1)) != sequence);

	armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_MODIFY);
    }
}

void stm32wb_rtc_time_write(uint64_t clock, uint32_t seconds, uint32_t ticks, bool external)
{
    stm32wb_rtc_capture_t capture;
    uint32_t clock_l, clock_h;

    if (clock == 0)
    {
        __stm32wb_rtc_clock_capture(&capture);

        clock = __stm32wb_rtc_clock_convert(&capture, NULL);
    }
    else
    {
        clock += stm32wb_rtc_device.clock_offset;
    }

    clock_l = (clock >> 0);
    clock_h = (clock >> 32);
    ticks = (ticks & 0x7fff) | (external ? 0x8000 : 0x0000);
    
    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_4((uint32_t)&__svc_stm32wb_rtc_time_write, (uint32_t)clock_l, (uint32_t)clock_h, (uint32_t)seconds, (uint32_t)ticks);
    }
    else
    {
	__svc_stm32wb_rtc_time_write(clock_l, clock_h, seconds, ticks);
    }
}

int32_t stm32wb_rtc_time_to_utc_offset(uint32_t seconds)
{
    int32_t utc_offset;
    
    if (seconds > STM32WB_RTC_UTC_OFFSET_EXPIRATION)
    {
        return stm32wb_rtc_device.time_current.utc_offset;
    }

    for (utc_offset = STM32WB_RTC_UTC_OFFSET_DEFAULT; utc_offset >= 0; utc_offset--)
    {
        if (seconds >= stm32wb_rtc_utc_offset_table[utc_offset])
        {
            break;
        }
    }
    
    return utc_offset;
}

int32_t stm32wb_rtc_utc_to_utc_offset(uint32_t seconds)
{
    int32_t utc_offset;

    if ((seconds + STM32WB_RTC_UTC_OFFSET_DEFAULT) > STM32WB_RTC_UTC_OFFSET_EXPIRATION)
    {
        return stm32wb_rtc_device.time_current.utc_offset;
    }

    for (utc_offset = STM32WB_RTC_UTC_OFFSET_DEFAULT; utc_offset >= 0; utc_offset--)
    {
        if ((seconds + utc_offset) >= stm32wb_rtc_utc_offset_table[utc_offset])
        {
            break;
        }
    }

    return utc_offset;
}

static void  __attribute__((optimize("O3"))) stm32wb_rtc_alarm_routine(void)
{
    stm32wb_rtc_capture_t capture;
    stm32wb_rtc_tod_t tod;
    stm32wb_rtc_callback_t callback;
    void *context;
    uint32_t sequence, seconds, ticks, o_seconds, o_ticks, c_seconds, c_ticks, alrmr, alrmssr;
    uint64_t alarm_clock, clock, timeout;
    uint8_t alarm_continue;

retry:
    do
    {
        sequence = stm32wb_rtc_device.alarm_sequence[1];

        seconds = stm32wb_rtc_device.alarm_next.seconds;
	ticks = stm32wb_rtc_device.alarm_next.ticks;
	callback = stm32wb_rtc_device.alarm_next.callback;
	context = stm32wb_rtc_device.alarm_next.context;
    }
    while (stm32wb_rtc_device.alarm_sequence[1] != sequence);
    
    if (stm32wb_rtc_device.alarm_sequence[0] != sequence)
    {
        if (stm32wb_rtc_device.alarm_busy)
        {
            armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
            
            RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);
            
            stm32wb_rtc_device.alarm_busy = 0;
        }
            
        stm32wb_rtc_device.alarm_clock = 0;

        stm32wb_rtc_device.alarm_active = 0;
        stm32wb_rtc_device.alarm_continue = 0;
        stm32wb_rtc_device.alarm_event = 0;
        
        stm32wb_rtc_device.alarm_current.seconds = seconds;
        stm32wb_rtc_device.alarm_current.ticks = ticks;
        stm32wb_rtc_device.alarm_current.callback = callback;
        stm32wb_rtc_device.alarm_current.context = context;

        stm32wb_rtc_device.alarm_sequence[0] = sequence;
    }
    
    if (!armv7m_pendsv_is_pending(ARMV7M_PENDSV_SWI_RTC_ALARM))
    {
	seconds = stm32wb_rtc_device.alarm_current.seconds;
	ticks = stm32wb_rtc_device.alarm_current.ticks;
            
	if (seconds || ticks)
	{
	    stm32wb_rtc_device.alarm_active = 1;
	    
	    __stm32wb_rtc_clock_capture(&capture);
	    
	    clock = __stm32wb_rtc_clock_convert(&capture, &tod);
	    
	    do
	    {
		o_seconds = stm32wb_rtc_device.time_current.seconds_offset;
		o_ticks = stm32wb_rtc_device.time_current.ticks_offset;
	    }
	    while ((stm32wb_rtc_device.time_current.seconds_offset != o_seconds) || (stm32wb_rtc_device.time_current.ticks_offset != o_ticks));
	    
	    c_seconds = clock / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
	    c_ticks = clock & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND -1);
	    
	    c_seconds += o_seconds;
	    c_ticks += o_ticks;
	    
	    if (c_ticks >= STM32WB_RTC_CLOCK_TICKS_PER_SECOND)
	    {
		c_seconds++;
		c_ticks -= STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
	    }
	    
	    c_seconds -= stm32wb_rtc_time_to_utc_offset(c_seconds);
	    
	    if ((seconds < c_seconds) || ((seconds == c_seconds) && (ticks <= c_ticks)))
	    {
		if (stm32wb_rtc_device.alarm_busy)
		{
		    armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
                    
		    RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);
                    
		    stm32wb_rtc_device.alarm_busy = 0;
		}
                
		stm32wb_rtc_device.alarm_clock = 0;

		stm32wb_rtc_device.alarm_event = 1;
                
		NVIC_SetPendingIRQ(RTC_Alarm_IRQn);
	    }
	    else
	    {
		if (ticks < c_ticks)
		{
		    ticks += STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
		    seconds--;
		}
                
		seconds -= c_seconds;
		ticks -= c_ticks;
                
		timeout = ((uint64_t)seconds * STM32WB_RTC_CLOCK_TICKS_PER_SECOND) + ticks; 
                
		if (timeout > (2419200ull * STM32WB_RTC_CLOCK_TICKS_PER_SECOND))
		{
		    timeout = (2419200ull * STM32WB_RTC_CLOCK_TICKS_PER_SECOND);
                    
		    alarm_continue = 1;
		}
		else
		{
		    if (timeout <= 2)
		    {
			timeout = 2;
		    }
                    
		    alarm_continue = 0;
		}
                
		alarm_clock = clock + timeout;
                
		if (stm32wb_rtc_device.alarm_clock != alarm_clock)
		{
		    if (stm32wb_rtc_device.alarm_busy)
		    {
			armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
                        
			RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);
                        
			stm32wb_rtc_device.alarm_busy = 0;
		    }
                    
		    stm32wb_rtc_device.alarm_clock = alarm_clock;
		    stm32wb_rtc_device.alarm_continue = alarm_continue;
                    
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

		    stm32wb_rtc_device.alarm_busy = 1;
                    
		    armv7m_atomic_or(&RTC->CR, (RTC_CR_ALRAIE | RTC_CR_ALRAE));

		    if (stm32wb_rtc_device.alarm_busy)
		    {
			__stm32wb_rtc_clock_capture(&capture);
                        
			if (stm32wb_rtc_device.alarm_busy)
			{
			    clock = __stm32wb_rtc_clock_convert(&capture, NULL);
                            
			    if (alarm_clock <= (clock + 1))
			    {
				if (stm32wb_rtc_device.alarm_busy)
				{
				    armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
                                    
				    RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);
                                    
				    stm32wb_rtc_device.alarm_busy = 0;
                                    
				    stm32wb_rtc_device.alarm_clock = 0;
                                    
				    if (!armv7m_pendsv_is_pending(ARMV7M_PENDSV_SWI_RTC_ALARM))
				    {
					goto retry;
				    }
				}
			    }
			}
		    }
		}
	    }
	}
	else
	{
	    if (stm32wb_rtc_device.alarm_busy)
	    {
		armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
                
		RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);
                
		stm32wb_rtc_device.alarm_busy = 0;
	    }
            
	    stm32wb_rtc_device.alarm_clock = 0;
	}
    }
}

static void __svc_stm32wb_rtc_alarm_modify(const stm32wb_rtc_alarm_t *alarm)
{
    uint32_t sequence;

    stm32wb_rtc_device.alarm_routine = stm32wb_rtc_alarm_routine;
    
    do
    {
	sequence = stm32wb_rtc_device.alarm_sequence[1];

	stm32wb_rtc_device.alarm_next.seconds = alarm->seconds;
	stm32wb_rtc_device.alarm_next.ticks = alarm->ticks;
	stm32wb_rtc_device.alarm_next.callback = alarm->callback;
	stm32wb_rtc_device.alarm_next.context = alarm->context;
    }
    while (armv7m_atomic_cas(&stm32wb_rtc_device.alarm_sequence[1], sequence, (sequence + 1)) != sequence);

    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_ALARM);
}

void stm32wb_rtc_alarm_start(uint32_t seconds, uint32_t ticks, stm32wb_rtc_alarm_callback_t callback, void *context)
{
    stm32wb_rtc_alarm_t alarm;
    
    alarm.seconds = seconds;
    alarm.ticks = ticks;
    alarm.callback = callback;
    alarm.context = context;

    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_1((uint32_t)&__svc_stm32wb_rtc_alarm_modify, (uint32_t)&alarm);
    }
    else
    {
        __svc_stm32wb_rtc_alarm_modify(&alarm);
    }
}

void stm32wb_rtc_alarm_stop(void)
{
    stm32wb_rtc_alarm_t alarm;
    
    alarm.seconds = 0;
    alarm.ticks = 0;
    alarm.callback = NULL;
    alarm.context = NULL;

    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_1((uint32_t)&__svc_stm32wb_rtc_alarm_modify, (uint32_t)&alarm);
    }
    else
    {
        __svc_stm32wb_rtc_alarm_modify(&alarm);
    }
}

bool stm32wb_rtc_alarm_done(void)
{
    return !stm32wb_rtc_device.alarm_active;
}

static void stm32wb_rtc_timer_remove(stm32wb_rtc_timer_t *timer)
{
    if (timer->next == timer)
    {
	stm32wb_rtc_device.timer_queue = NULL;
    }
    else
    {
	if (timer == stm32wb_rtc_device.timer_queue)
	{
	    stm32wb_rtc_device.timer_queue = timer->next;
	}
	
	timer->next->previous = timer->previous;
	timer->previous->next = timer->next;
    }
    
    timer->next = NULL;
    timer->previous = NULL;
}

static void stm32wb_rtc_timer_insert(stm32wb_rtc_timer_t *timer, uint64_t clock)
{
    stm32wb_rtc_timer_t *timer_element, *timer_next;
    uint64_t timeout_clock;

    if (stm32wb_rtc_device.timer_queue == NULL)
    {
	timer->next = timer;
	timer->previous = timer;

	stm32wb_rtc_device.timer_queue = timer;
    }
    else
    {
	timer_element = stm32wb_rtc_device.timer_queue;

	do
	{
	    timer_next = timer_element->next;

            timeout_clock = (((uint64_t)timer_element->clock[0] << 0) | ((uint64_t)timer_element->clock[1] << 32));

	    if (!timer_element->modify)
	    {
		if (clock < timeout_clock)
		{
		    if (timer_element == stm32wb_rtc_device.timer_queue)
		    {
			stm32wb_rtc_device.timer_queue = timer;
		    }
		    break;
		}
	    }
	    else
	    {
		stm32wb_rtc_timer_remove(timer_element);
	    }

	    timer_element = timer_next;
	}
	while (timer_element != stm32wb_rtc_device.timer_queue);

	if (stm32wb_rtc_device.timer_queue == NULL)
	{
	    timer->next = timer;
	    timer->previous = timer;
	    
	    stm32wb_rtc_device.timer_queue = timer;
	}
	else
	{
	    timer->previous = timer_element->previous;
	    timer->next = timer_element;
	    
	    timer->previous->next = timer;
	    timer->next->previous = timer;
	}
    }
}

static __attribute__((optimize("O3"))) void stm32wb_rtc_timer_routine(void)
{
    stm32wb_rtc_capture_t capture;
    stm32wb_rtc_tod_t tod;
    stm32wb_rtc_timer_t *timer, *timer_previous, *timer_next;
    uint64_t reference, clock, timeout;
    uint32_t seconds, ticks, alrmr, alrmssr;
    
retry:
    __stm32wb_rtc_clock_capture(&capture);
        
    reference = __stm32wb_rtc_clock_convert(&capture, &tod);
	    
    if (stm32wb_rtc_device.timer_modify != STM32WB_RTC_TIMER_SENTINEL)
    {
	timer = (stm32wb_rtc_timer_t*)__armv7m_atomic_swap((volatile uint32_t*)&stm32wb_rtc_device.timer_modify, (uint32_t)STM32WB_RTC_TIMER_SENTINEL);

	/* Revert the modify queue, and process it.
	 */
	for (timer_previous = STM32WB_RTC_TIMER_SENTINEL; timer != STM32WB_RTC_TIMER_SENTINEL; timer = timer_next)
	{
	    timer_next = timer->modify;
		
	    armv7m_atomic_store((volatile uint32_t*)&timer->modify, (uint32_t)timer_previous);
		
	    timer_previous = timer;
	}

	timer = timer_previous;
	    
	while (timer != STM32WB_RTC_TIMER_SENTINEL)
	{
	    timer_next = timer->modify;

	    if (timer->next)
	    {
		stm32wb_rtc_timer_remove(timer);
	    }

	    clock = (((uint64_t)timer->clock[0] << 0) | ((uint64_t)timer->clock[1] << 32));
		
	    armv7m_atomic_store((volatile uint32_t*)&timer->modify, (uint32_t)NULL);

	    if (!timer->modify)
	    {
		if (clock)
		{
		    stm32wb_rtc_timer_insert(timer, clock);
		}
		else
		{
		    (*timer->callback)(timer->context, 0);
		}
	    }
		
	    timer = timer_next;
	}
    }
	
    if (stm32wb_rtc_device.timer_queue)
    {
	do
	{
	    timer = stm32wb_rtc_device.timer_queue;
		
	    clock = (((uint64_t)timer->clock[0] << 0) | ((uint64_t)timer->clock[1] << 32));
		
	    if (!timer->modify)
	    {
		if (clock > reference)
		{
		    break;
		}

		stm32wb_rtc_timer_remove(timer);
			
		(*timer->callback)(timer->context, clock - stm32wb_rtc_device.clock_offset);
	    }
	    else
	    {
		stm32wb_rtc_timer_remove(timer);
	    }
	}
	while (stm32wb_rtc_device.timer_queue);
    }

    if (!armv7m_pendsv_is_pending(ARMV7M_PENDSV_SWI_RTC_TIMER))
    {
	if (stm32wb_rtc_device.timer_queue)
	{
	    timer = stm32wb_rtc_device.timer_queue;
	    
	    clock = (((uint64_t)timer->clock[0] << 0) | ((uint64_t)timer->clock[1] << 32));
	    
	    if (!timer->modify)
	    {
		timeout = clock - reference;
                
		if (timeout > (2419200ull * STM32WB_RTC_CLOCK_TICKS_PER_SECOND))
		{
		    timeout = (2419200ull * STM32WB_RTC_CLOCK_TICKS_PER_SECOND);
		}
		else
		{
		    if (timeout <= 2)
		    {
			timeout = 2;
		    }
		}
                
		clock = timeout + reference;
		
		if (clock != stm32wb_rtc_device.timer_clock)
		{
		    if (stm32wb_rtc_device.timer_busy)
		    {
			armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRBIE | RTC_CR_ALRBE));
                        
			RTC->ISR = ~(RTC_ISR_ALRBF | RTC_ISR_INIT);
                        
			if (stm32wb_rtc_device.timer_busy)
			{
			    stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_STOP_2);
                            
			    stm32wb_rtc_device.timer_busy = false;
			}
		    }
                    
		    stm32wb_rtc_device.timer_clock = clock;
                    
		    seconds = timeout / STM32WB_RTC_CLOCK_TICKS_PER_SECOND;
		    ticks   = timeout & (STM32WB_RTC_CLOCK_TICKS_PER_SECOND - 1);
                    
		    __stm32wb_rtc_clock_offset(&tod, seconds, ticks);
                    
		    alrmr = ((stm32wb_rtc_int_to_bcd[tod.seconds] << RTC_ALRMAR_SU_Pos) |
			     (stm32wb_rtc_int_to_bcd[tod.minutes] << RTC_ALRMAR_MNU_Pos) |
			     (stm32wb_rtc_int_to_bcd[tod.hours] << RTC_ALRMAR_HU_Pos) |
			     (stm32wb_rtc_int_to_bcd[tod.day] << RTC_ALRMAR_DU_Pos));
                    
		    alrmssr = ((STM32WB_RTC_PREDIV_S - 1) - tod.ticks) | STM32WB_RTC_ALRMSSR_MASKSS;
                    
		    while (!(RTC->ISR & RTC_ISR_ALRBWF))
		    {
			__NOP();
			__NOP();
			__NOP();
			__NOP();
		    }
                    
		    RTC->ALRMBR = alrmr;
		    RTC->ALRMBSSR = alrmssr;

		    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_STOP_2);
                    
		    stm32wb_rtc_device.timer_busy = true;
		    
		    armv7m_atomic_or(&RTC->CR, (RTC_CR_ALRBIE | RTC_CR_ALRBE));
		    
		    if (stm32wb_rtc_device.timer_busy)
		    {
			__stm32wb_rtc_clock_capture(&capture);
                        
			if (stm32wb_rtc_device.timer_busy)
			{
			    reference = __stm32wb_rtc_clock_convert(&capture, &tod);
                            
			    if (clock <= (reference + 1))
			    {
				if (stm32wb_rtc_device.timer_busy)
				{
				    armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRBIE | RTC_CR_ALRBE));
                                    
				    RTC->ISR = ~(RTC_ISR_ALRBF | RTC_ISR_INIT);
                                    
				    if (stm32wb_rtc_device.timer_busy)
				    {
					stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_STOP_2);
					
					stm32wb_rtc_device.timer_busy = false;
				    }
                                    
				    goto retry;
				}
			    }
			}
		    }
		}
	    }
	}
	else
	{
	    if (stm32wb_rtc_device.timer_busy)
	    {
		armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRBIE | RTC_CR_ALRBE));
		
		RTC->ISR = ~(RTC_ISR_ALRBF | RTC_ISR_INIT);
		
		if (stm32wb_rtc_device.timer_busy)
		{
		    stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_STOP_2);
		    
		    stm32wb_rtc_device.timer_busy = false;
		}
	    }
	    
	    stm32wb_rtc_device.timer_clock = 0;
	}
    }
}

static void __svc_stm32wb_rtc_timer_modify(stm32wb_rtc_timer_t *timer, uint32_t clock_l, uint32_t clock_h)
{
    stm32wb_rtc_timer_t *timer_modify;

    stm32wb_rtc_device.timer_routine = stm32wb_rtc_timer_routine;

    armv7m_atomic_store_2_restart(&timer->clock[0], clock_l, clock_h);
    
    if (__armv7m_atomic_cas((volatile uint32_t*)&timer->modify, (uint32_t)NULL, (uint32_t)STM32WB_RTC_TIMER_SENTINEL) == (uint32_t)NULL)
    {
	do
	{
	    timer_modify = stm32wb_rtc_device.timer_modify;

	    armv7m_atomic_store((volatile uint32_t*)&timer->modify, (uint32_t)timer_modify);
	}
	while ((stm32wb_rtc_timer_t*)__armv7m_atomic_cas((volatile uint32_t*)&stm32wb_rtc_device.timer_modify, (uint32_t)timer_modify, (uint32_t)timer) != timer_modify);

	if (timer_modify == STM32WB_RTC_TIMER_SENTINEL)
	{
	    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_TIMER);
	}
    }
}

void stm32wb_rtc_timer_create(stm32wb_rtc_timer_t *timer, stm32wb_rtc_timer_callback_t callback, void *context)
{
    timer->next = NULL;
    timer->previous = NULL;
    timer->modify = NULL;
    timer->callback = callback;
    timer->context = context;
    timer->clock[0] = 0;
    timer->clock[1] = 0;
}

bool stm32wb_rtc_timer_destroy(stm32wb_rtc_timer_t *timer)
{
    return stm32wb_rtc_timer_done(timer);
}

bool stm32wb_rtc_timer_start(stm32wb_rtc_timer_t *timer, uint64_t clock)
{
    if (clock == 0)
    {
	return false;
    }

    clock = clock + stm32wb_rtc_device.clock_offset;

    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_3((uint32_t)&__svc_stm32wb_rtc_timer_modify, (uint32_t)timer, (uint32_t)(clock >> 0), (uint32_t)(clock >> 32));
    }
    else
    {
        __svc_stm32wb_rtc_timer_modify(timer, (uint32_t)(clock >> 0), (uint32_t)(clock >> 32));
    }

    return true;
}

void stm32wb_rtc_timer_stop(stm32wb_rtc_timer_t *timer)
{
    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_3((uint32_t)&__svc_stm32wb_rtc_timer_modify, (uint32_t)timer, 0, 0);
    }
    else
    {
        __svc_stm32wb_rtc_timer_modify(timer, 0, 0);
    }
}

bool stm32wb_rtc_timer_done(stm32wb_rtc_timer_t *timer)
{
    return (timer->next == NULL) && (timer->modify == NULL);
}

bool stm32wb_rtc_wakeup_start(uint32_t seconds, stm32wb_rtc_wakeup_callback_t callback, void *context)
{
    if (seconds > 65536)
    {
	return false;
    }
    
    if (stm32wb_rtc_device.wakeup_busy)
    {
        armv7m_atomic_and(&RTC->CR, ~(RTC_CR_WUTIE | RTC_CR_WUTE));

        RTC->ISR = ~(RTC_ISR_WUTF | RTC_ISR_INIT);

        EXTI->PR1 = EXTI_PR1_PIF19;

        if (stm32wb_rtc_device.wakeup_busy)
        {
            stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_STOP_2);

            stm32wb_rtc_device.wakeup_busy = 0;
        }
    }

    stm32wb_rtc_device.wakeup_callback = NULL;
    stm32wb_rtc_device.wakeup_context = context;
    stm32wb_rtc_device.wakeup_callback = callback;

    while (!(RTC->ISR & RTC_ISR_WUTWF))
    {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }

    RTC->WUTR = seconds -1;

    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_STOP_2);

    stm32wb_rtc_device.wakeup_busy = 1;
    
    armv7m_atomic_or(&RTC->CR, RTC_CR_WUTIE | RTC_CR_WUTE);

    return true;
}

void stm32wb_rtc_wakeup_stop()
{
    if (stm32wb_rtc_device.wakeup_busy)
    {
        armv7m_atomic_and(&RTC->CR, ~(RTC_CR_WUTIE | RTC_CR_WUTE));

        RTC->ISR = ~(RTC_ISR_WUTF | RTC_ISR_INIT);

        EXTI->PR1 = EXTI_PR1_PIF19;

        if (stm32wb_rtc_device.wakeup_busy)
        {
            stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_STOP_2);

            stm32wb_rtc_device.wakeup_busy = 0;
        }
    }
}

bool stm32wb_rtc_wakeup_done()
{
    return !stm32wb_rtc_device.wakeup_busy;
}

bool stm32wb_rtc_tamp_attach(uint16_t pin, uint32_t control, stm32wb_rtc_callback_t callback, void *context)
{
    pin &= STM32WB_GPIO_PIN_IO_MASK;

    if (pin == STM32WB_GPIO_PIN_PC13)
    {
        armv7m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1TRG | RTC_TAMPCR_TAMP1E));

        RTC->ISR = ~(RTC_ISR_TAMP1F | RTC_ISR_INIT);

        if (control & (STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING | STM32WB_RTC_TAMP_CONTROL_EDGE_RISING))
        {
            stm32wb_rtc_device.tamp1_callback = NULL;
            stm32wb_rtc_device.tamp1_context = context;
            stm32wb_rtc_device.tamp1_callback = callback;

            armv7m_atomic_or(&RTC->TAMPCR, (RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1E | ((control & STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING) ? RTC_TAMPCR_TAMP1TRG : 0)));;
        }

        return true;
    }

    if (pin == STM32WB_GPIO_PIN_PA0)
    {
        armv7m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2TRG | RTC_TAMPCR_TAMP2E));

        RTC->ISR = ~(RTC_ISR_TAMP2F | RTC_ISR_INIT);

        if (control & (STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING | STM32WB_RTC_TAMP_CONTROL_EDGE_RISING))
        {
            stm32wb_rtc_device.tamp2_callback = NULL;
            stm32wb_rtc_device.tamp2_context = context;
            stm32wb_rtc_device.tamp2_callback = callback;

            armv7m_atomic_or(&RTC->TAMPCR, (RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2E | ((control & STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING) ? RTC_TAMPCR_TAMP2TRG : 0)));;
        }

        return true;
    }

    if (pin == STM32WB_GPIO_PIN_PC12)
    {
        armv7m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP3IE | RTC_TAMPCR_TAMP3TRG | RTC_TAMPCR_TAMP3E));

        RTC->ISR = ~(RTC_ISR_TAMP3F | RTC_ISR_INIT);

        if (control & (STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING | STM32WB_RTC_TAMP_CONTROL_EDGE_RISING))
        {
	    stm32wb_rtc_device.tamp3_callback = NULL;
            stm32wb_rtc_device.tamp3_context = context;
            stm32wb_rtc_device.tamp3_callback = callback;

            armv7m_atomic_or(&RTC->TAMPCR, (RTC_TAMPCR_TAMP3IE | RTC_TAMPCR_TAMP3E | ((control & STM32WB_RTC_TAMP_CONTROL_EDGE_FALLING) ? RTC_TAMPCR_TAMP3TRG : 0)));;
        }

        return true;
    }
    
    return false;
}

bool stm32wb_rtc_tamp_detach(uint16_t pin)
{
    pin &= STM32WB_GPIO_PIN_IO_MASK;

    if (pin == STM32WB_GPIO_PIN_PC13)
    {
        armv7m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1E));

        RTC->ISR = ~(RTC_ISR_TAMP1F | RTC_ISR_INIT);

        return true;
    }

    if (pin == STM32WB_GPIO_PIN_PA0)
    {
        armv7m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2E));

        RTC->ISR = ~(RTC_ISR_TAMP2F | RTC_ISR_INIT);

        return true;
    }

    if (pin == STM32WB_GPIO_PIN_PC12)
    {
        armv7m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP3IE | RTC_TAMPCR_TAMP3E));

        RTC->ISR = ~(RTC_ISR_TAMP3F | RTC_ISR_INIT);

        return true;
    }
    
    return false;
}

void stm32wb_rtc_stamp_attach(uint32_t control, stm32wb_rtc_callback_t callback, void *context)
{
    armv7m_atomic_and(&RTC->CR, ~(RTC_CR_TSIE | RTC_CR_TSE | RTC_CR_TSEDGE));

    RTC->ISR = ~(RTC_ISR_TSF | RTC_ISR_INIT);
    
    if (control & (STM32WB_RTC_STAMP_CONTROL_EDGE_FALLING | STM32WB_RTC_STAMP_CONTROL_EDGE_RISING))
    {
        stm32wb_rtc_device.tamp1_callback = NULL;
        stm32wb_rtc_device.tamp1_context = context;
        stm32wb_rtc_device.tamp1_callback = callback;

        armv7m_atomic_or(&RTC->CR, (RTC_CR_TSIE | RTC_CR_TSE | ((control & STM32WB_RTC_STAMP_CONTROL_EDGE_FALLING) ? RTC_CR_TSEDGE : 0)));
    }
}

void stm32wb_rtc_stamp_detach()
{
    armv7m_atomic_and(&RTC->CR, ~(RTC_CR_TSIE | RTC_CR_TSE));

    RTC->ISR = ~(RTC_ISR_TSF | RTC_ISR_INIT);
}

uint64_t stm32wb_rtc_stamp_read()
{
    stm32wb_rtc_capture_t ts, capture;

    ts.ssr = RTC->TSSSR;
    ts.tr = RTC->TSTR;
    ts.dr = RTC->TSDR;
    
    stm32wb_rtc_clock_capture(&capture);
        
    if ((stm32wb_rtc_bcd_to_int[(ts.dr >> RTC_DR_MU_Pos) & ((RTC_DR_MU_Msk | RTC_DR_MT_Msk) >> RTC_DR_MU_Pos)] == 12) &&
        (stm32wb_rtc_bcd_to_int[(capture.dr >> RTC_DR_MU_Pos) & ((RTC_DR_MU_Msk | RTC_DR_MT_Msk) >> RTC_DR_MU_Pos)] != 12))
    {
        ts.dr = capture.dr;
    }

    return stm32wb_rtc_clock_convert(&ts);
}

void stm32wb_rtc_standby(uint32_t timeout)
{
    stm32wb_rtc_capture_t capture;
    stm32wb_rtc_tod_t tod;
    uint32_t seconds, ticks;
    
    /* Called with interrupts disabled.
     */

    RTC->CR &= ~(RTC_CR_TSIE | RTC_CR_WUTIE | RTC_CR_ALRBIE | RTC_CR_ALRAIE | RTC_CR_TSE | RTC_CR_WUTE | RTC_CR_ALRBE | RTC_CR_ALRAE);
    RTC->TAMPCR &= ~(RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1E | RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2E | RTC_TAMPCR_TAMP3IE | RTC_TAMPCR_TAMP3E);
    RTC->ISR = 0;
    
    if (timeout)
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
        
        while (!(RTC->ISR & RTC_ISR_ALRAWF))
        {
            __NOP();
            __NOP();
            __NOP();
            __NOP();
        }
        
        RTC->ALRMAR = ((stm32wb_rtc_int_to_bcd[tod.seconds] << RTC_ALRMAR_SU_Pos) |
                       (stm32wb_rtc_int_to_bcd[tod.minutes] << RTC_ALRMAR_MNU_Pos) |
                       (stm32wb_rtc_int_to_bcd[tod.hours] << RTC_ALRMAR_HU_Pos) |
                       (stm32wb_rtc_int_to_bcd[tod.day] << RTC_ALRMAR_DU_Pos));
        
        RTC->ALRMASSR = ((STM32WB_RTC_PREDIV_S - 1) - tod.ticks) | STM32WB_RTC_ALRMSSR_MASKSS;
        
        RTC->CR |= (RTC_CR_ALRAIE | RTC_CR_ALRAE);
    }

    EXTI->PR1 = EXTI_PR1_PIF17 | EXTI_PR1_PIF18 | EXTI_PR1_PIF19;
    
    /* Lock RTC throu reset */
    RTC->WPR = 0x00;
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

/*******************************************************************************************************************/

void TAMP_STAMP_LSECSS_IRQHandler(void)
{
    stm32wb_rtc_callback_t callback;
    void *context;
    
    do
    {
	EXTI->PR1 = EXTI_PR1_PIF18;

	if (RTC->ISR & (RTC_ISR_TAMP1F | RTC_ISR_TSF))
	{
	    RTC->ISR = ~(RTC_ISR_TAMP1F | RTC_ISR_TSF | RTC_ISR_INIT);

	    callback = stm32wb_rtc_device.tamp1_callback;
	    context = stm32wb_rtc_device.tamp1_context;
	    
	    if (callback)
	    {
		(*callback)(context);
	    }
	}
            
	if (RTC->ISR & RTC_ISR_TAMP2F)
	{
	    RTC->ISR = ~(RTC_ISR_TAMP2F | RTC_ISR_INIT);

	    callback = stm32wb_rtc_device.tamp2_callback;
	    context = stm32wb_rtc_device.tamp2_context;

	    if (callback)
	    {
		(*callback)(context);
	    }
	}

	if (RTC->ISR & RTC_ISR_TAMP3F)
	{
	    RTC->ISR = ~(RTC_ISR_TAMP3F | RTC_ISR_INIT);

	    callback = stm32wb_rtc_device.tamp3_callback;
	    context = stm32wb_rtc_device.tamp3_context;

	    if (callback)
	    {
		(*callback)(context);
	    }
	}
    }
    while (RTC->ISR & (RTC_ISR_TAMP1F | RTC_ISR_TAMP2F | RTC_ISR_TAMP3F | RTC_ISR_TSF));

    __DSB();
}

void RTC_WKUP_IRQHandler(void)
{
    stm32wb_rtc_callback_t callback;
    void *context;

    EXTI->PR1 = EXTI_PR1_PIF19;

    if (RTC->ISR & RTC_ISR_WUTF)
    {
	RTC->ISR = ~(RTC_ISR_WUTF | RTC_ISR_INIT);
	
	if (stm32wb_rtc_device.wakeup_busy)
	{
	    callback = stm32wb_rtc_device.wakeup_callback;
	    context = stm32wb_rtc_device.wakeup_context;
            
	    if (callback)
	    {
		(*callback)(context);
	    }
	}
    }

    __DSB();
}

void RTC_Alarm_IRQHandler(void)
{
    stm32wb_rtc_callback_t callback;
    void *context;

    do
    {
	EXTI->PR1 = EXTI_PR1_PIF17;

	if (RTC->ISR & RTC_ISR_ALRAF)
	{
	    if (stm32wb_rtc_device.alarm_busy)
	    {
		armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));

		RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);

		stm32wb_rtc_device.alarm_busy = 0;
		stm32wb_rtc_device.alarm_clock = 0;

		if (stm32wb_rtc_device.alarm_continue)
		{
		    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_ALARM);
		}
		else
		{
		    stm32wb_rtc_device.alarm_event = true;
		}
	    }
	    else
	    {
		RTC->ISR = ~(RTC_ISR_ALRAF | RTC_ISR_INIT);
	    }
	}

	if (RTC->ISR & RTC_ISR_ALRBF)
	{
	    if (stm32wb_rtc_device.timer_busy)
	    {
		armv7m_atomic_and(&RTC->CR, ~(RTC_CR_ALRBIE | RTC_CR_ALRBE));

		RTC->ISR = ~(RTC_ISR_ALRBF | RTC_ISR_INIT);

		stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_STOP_2);

		stm32wb_rtc_device.timer_busy = false;
		stm32wb_rtc_device.timer_clock = 0;
		
		armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_RTC_TIMER);
	    }
	    else
	    {
		RTC->ISR = ~(RTC_ISR_ALRBF | RTC_ISR_INIT);
	    }
	}
    }
    while (RTC->ISR & (RTC_ISR_ALRAF | RTC_ISR_ALRBF));

    if (stm32wb_rtc_device.alarm_event)
    {
        stm32wb_rtc_device.alarm_event = 0;

        callback = stm32wb_rtc_device.alarm_current.callback;
	context = stm32wb_rtc_device.alarm_current.context;

	stm32wb_rtc_device.alarm_active = 0;
	    
        if (callback)
        {
            (*callback)(context);
        }
    }

    __DSB();
}

void RTC_MODIFY_SWIHandler(void)
{
    stm32wb_rtc_modify_routine_t routine;

    routine = stm32wb_rtc_device.modify_routine;
    
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

void RTC_TIMER_SWIHandler(void)
{
    stm32wb_rtc_timer_routine_t routine;
    
    routine = stm32wb_rtc_device.timer_routine;
    
    if (routine)
    {
        (*routine)();
    }
}
