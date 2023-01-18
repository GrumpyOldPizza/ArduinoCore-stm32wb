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

#pragma once

#include "stm32wb_adc.h"
#include "stm32wb_dma.h"
#include "stm32wb_exti.h"
#include "stm32wb_flash.h"
#include "stm32wb_gpio.h"
#include "stm32wb_i2c.h"
#include "stm32wb_iwdg.h"
#include "stm32wb_lptim.h"
#include "stm32wb_random.h"
#include "stm32wb_rtc.h"
#include "stm32wb_sai.h"
#include "stm32wb_sdspi.h"
#include "stm32wb_servo.h"
#include "stm32wb_sflash.h"
#include "stm32wb_sfspi.h"
#include "stm32wb_sfsqi.h"
#include "stm32wb_spi.h"
#include "stm32wb_system.h"
#include "stm32wb_tim.h"
#include "stm32wb_uart.h"
#include "stm32wb_usbd.h"
#include "stm32wb_usbd_info.h"
#include "stm32wb_usbd_dcd.h"
#include "stm32wb_usbd_cdc.h"
#include "stm32wb_usbd_msc.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void dosfs_sflash_initialize();

#define WIRING_EVENT_TRANSIENT  0x80000000
#define WIRING_EVENT_WAKEUP     0x40000000    

extern const uint32_t g_pinModeConfiguration[];
  
extern void (*g_serialEventRun)(void);

extern uint32_t __analogReadChannel(uint32_t channel, uint32_t smp);
extern void __analogWriteDisable(uint32_t pin);

/*
 * TIM1   PWM
 * TIM2   PWM
 * TIM16  PWM
 * TIM17  PWM
 * LPTIM2 TONE/SERVO
 */
#define STM32WB_PWM_IRQ_PRIORITY     ARMV7M_IRQ_PRIORITY_REALTIME
#define STM32WB_USB_IRQ_PRIORITY     ARMV7M_IRQ_PRIORITY_USB
#define STM32WB_LPTIM_IRQ_PRIORITY   ARMV7M_IRQ_PRIORITY_LPTIM
#define STM32WB_I2C_IRQ_PRIORITY     ARMV7M_IRQ_PRIORITY_I2C
#define STM32WB_SPI_IRQ_PRIORITY     ARMV7M_IRQ_PRIORITY_SPI
#define STM32WB_QUADSPI_IRQ_PRIORITY ARMV7M_IRQ_PRIORITY_MEDIUM
#define STM32WB_UART_IRQ_PRIORITY    ARMV7M_IRQ_PRIORITY_HIGH
#define STM32WB_SAI_IRQ_PRIORITY     ARMV7M_IRQ_PRIORITY_SAI
#define STM32WB_TONE_IRQ_PRIORITY    ARMV7M_IRQ_PRIORITY_REALTIME
#define STM32WB_SERVO_IRQ_PRIORITY   ARMV7M_IRQ_PRIORITY_REALTIME

#ifdef __cplusplus
} // extern "C"
#endif
