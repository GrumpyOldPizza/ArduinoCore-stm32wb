/*
 * Copyright (c) 2016-2020 Thomas Roell.  All rights reserved.
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
#include "Tone.h"
#include "wiring_private.h"

static GPIO_TypeDef *toneGPIO = NULL;
static uint16_t toneBit = 0x0000;
static uint32_t toneCount = 0;

static __attribute__((optimize("O3"))) void tone_event_callback(void *context, uint32_t events)
{
    (void)context;
    (void)events;

    if (toneCount) {
      if (toneGPIO->ODR & toneBit) {
            toneGPIO->BRR = toneBit;
        } else {
            toneGPIO->BSRR = toneBit;
        }

        if (toneCount <= 0xfffffffe) {
            toneCount--;
        }
    } else {
        stm32wb_lptim_event_stop();
        stm32wb_lptim_event_unlock();

        toneGPIO->BRR = toneBit;
        toneGPIO = NULL;
    }
}

void tone(uint32_t pin, uint32_t frequency, uint32_t duration)
{
    uint32_t period;
    GPIO_TypeDef *GPIO;
    uint32_t bit;

    if (frequency == 0) {
        return;
    }

    if ((pin >= PINS_COUNT) || (g_APinDescription[pin].GPIO == NULL)) {
        return;
    }

    GPIO = (GPIO_TypeDef *)g_APinDescription[pin].GPIO;
    bit = g_APinDescription[pin].bit;

    if ((toneGPIO != GPIO) || (toneBit != bit)) {
        if (toneGPIO) {
            stm32wb_lptim_event_stop();
            stm32wb_lptim_event_unlock();

            toneGPIO->BRR = toneBit;
            toneGPIO = NULL;
        }
    }

    /* Use 4MHz as a carrier frequency. The Arduino UNO spec says we need to be able
     * to hit 31Hz at the bottom, which means a 62Hz period to toggle the GPIO.
     */
    period = 4000000 / (frequency * 2);

    if (period < 1    ) { period = 1;     }
    if (period > 65536) { period = 63356; }

    frequency = 4000000 / (period * 2);

    if (duration) {
        toneCount = (frequency * duration * 4) / 1000;
    } else {
        toneCount = 0xffffffff;
    }

    if (toneGPIO) {
        stm32wb_lptim_event_restart(period);
    } else {
        if (stm32wb_lptim_event_lock()) {
            stm32wb_gpio_pin_configure(g_APinDescription[pin].pin, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_MEDIUM | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));
            GPIO->BRR = bit;
            
            toneGPIO = GPIO;
            toneBit = bit;
            
            stm32wb_lptim_event_start(65535, period, (STM32WB_LPTIM_CONTROL_PERIOD | STM32WB_LPTIM_CONTROL_PRESCALE_4), STM32WB_TONE_IRQ_PRIORITY, tone_event_callback, NULL);
        }
    }
}

void noTone(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t bit;

    if ((pin >= PINS_COUNT) || (g_APinDescription[pin].GPIO == NULL)) {
        return;
    }

    GPIO = (GPIO_TypeDef *)g_APinDescription[pin].GPIO;
    bit = g_APinDescription[pin].bit;

    if ((toneGPIO == GPIO) && (toneBit == bit)) {
        stm32wb_lptim_event_stop();
        stm32wb_lptim_event_unlock();

        toneGPIO->BRR = toneBit;
        toneGPIO = NULL;
    }
}

