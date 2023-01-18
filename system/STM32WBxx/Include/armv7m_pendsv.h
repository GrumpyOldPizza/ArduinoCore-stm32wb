/*
 * Copyright (c) 2014-2020 Thomas Roell.  All rights reserved.
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

#if !defined(_ARMV7M_PENDSV_H)
#define _ARMV7M_PENDSV_H

#ifdef __cplusplus
extern "C" {
#endif
  
typedef void (*armv7m_pendsv_callback_t)(void);

extern void __armv7m_pendsv_initialize(void);

extern void armv7m_pendsv_hook(armv7m_pendsv_callback_t callback);
extern void armv7m_pendsv_raise(uint32_t mask);
extern bool armv7m_pendsv_is_pending(uint32_t mask);

extern void SWI0_SWIHandler(void);
extern void SWI1_SWIHandler(void);
extern void SWI2_SWIHandler(void);
extern void SWI3_SWIHandler(void);
extern void SWI4_SWIHandler(void);
extern void SWI5_SWIHandler(void);
extern void SWI6_SWIHandler(void);
extern void SWI7_SWIHandler(void);
extern void SWI8_SWIHandler(void);
extern void SWI9_SWIHandler(void);
extern void SWI10_SWIHandler(void);
extern void SWI11_SWIHandler(void);
extern void SWI12_SWIHandler(void);
extern void SWI13_SWIHandler(void);
extern void SWI14_SWIHandler(void);
extern void SWI15_SWIHandler(void);
extern void SWI16_SWIHandler(void);
extern void SWI17_SWIHandler(void);
extern void SWI18_SWIHandler(void);
extern void SWI19_SWIHandler(void);
extern void SWI20_SWIHandler(void);
extern void SWI21_SWIHandler(void);
extern void SWI22_SWIHandler(void);
extern void SWI23_SWIHandler(void);
extern void SWI24_SWIHandler(void);
extern void SWI25_SWIHandler(void);
extern void SWI26_SWIHandler(void);
extern void SWI27_SWIHandler(void);
extern void SWI28_SWIHandler(void);
extern void SWI29_SWIHandler(void);
extern void SWI30_SWIHandler(void);
extern void SWI31_SWIHandler(void);

  
#define ARMV7M_PENDSV_SWI_RADIO             (1ul << 0)
#define ARMV7M_PENDSV_SWI_EXTI              (1ul << 1)
#define ARMV7M_PENDSV_SWI_LPTIM_TIMEOUT     (1ul << 2)
#define ARMV7M_PENDSV_SWI_SERVO             (1ul << 3)
#define ARMV7M_PENDSV_SWI_RTC_SYNC          (1ul << 4)
#define ARMV7M_PENDSV_SWI_RTC_ALARM         (1ul << 5)
#define ARMV7M_PENDSV_SWI_USBD_PVM1         (1ul << 6)
#define ARMV7M_PENDSV_SWI_USBD_DCD          (1ul << 7)
#define ARMV7M_PENDSV_SWI_RNG               (1ul << 8)
#define ARMV7M_PENDSV_SWI_EEPROM            (1ul << 9)
#define ARMV7M_PENDSV_SWI_FLASH             (1ul << 10)
#define ARMV7M_PENDSV_SWI_RTOS_ALARM        (1ul << 11)
#define ARMV7M_PENDSV_SWI_RTOS_WORK         (1ul << 12)
#define ARMV7M_PENDSV_SWI_RTOS_TASK_RESUME  (1ul << 13)
#define ARMV7M_PENDSV_SWI_RTOS_TASK_RELEASE (1ul << 14)

#define RADIO_SWIHandler                    SWI0_SWIHandler
#define EXTI_SWIHandler                     SWI1_SWIHandler
#define LPTIM_TIMEOUT_SWIHandler            SWI2_SWIHandler
#define SERVO_SWIHandler                    SWI3_SWIHandler
#define RTC_SYNC_SWIHandler                 SWI4_SWIHandler
#define RTC_ALARM_SWIHandler                SWI5_SWIHandler
#define USBD_PVM1_SWIHandler                SWI6_SWIHandler
#define USBD_DCD_SWIHandler                 SWI7_SWIHandler
#define RNG_SWIHandler                      SWI8_SWIHandler
#define EEPROM_SWIHandler                   SWI9_SWIHandler
#define FLASH_SWIHandler                    SWI10_SWIHandler
#define RTOS_ALARM_SWIHandler               SWI11_SWIHandler
#define RTOS_WORK_SWIHandler                SWI12_SWIHandler
#define RTOS_TASK_RESUME_SWIHandler         SWI13_SWIHandler
#define RTOS_TASK_RELEASE_SWIHandler        SWI14_SWIHandler
  
#ifdef __cplusplus
}
#endif

#endif /* _ARMV7M_PENDSV_H */
