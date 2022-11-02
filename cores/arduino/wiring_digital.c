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
#include "wiring_private.h"

extern void __analogWriteDisable(uint32_t pin)  __attribute__((weak));

void __analogWriteDisable(uint32_t pin)
{
    (void)pin;
}

const uint32_t g_pinModeConfiguration[] = {
    (STM32WB_GPIO_PUPD_NONE     | STM32WB_GPIO_OSPEED_MEDIUM | STM32WB_GPIO_OTYPE_PUSHPULL  | STM32WB_GPIO_MODE_INPUT),  /* INPUT */
    (STM32WB_GPIO_PUPD_NONE     | STM32WB_GPIO_OSPEED_MEDIUM | STM32WB_GPIO_OTYPE_PUSHPULL  | STM32WB_GPIO_MODE_OUTPUT), /* OUTPUT */
    (STM32WB_GPIO_PUPD_PULLUP   | STM32WB_GPIO_OSPEED_MEDIUM | STM32WB_GPIO_OTYPE_PUSHPULL  | STM32WB_GPIO_MODE_INPUT),  /* INPUT_PULLUP */
    (STM32WB_GPIO_PUPD_PULLDOWN | STM32WB_GPIO_OSPEED_MEDIUM | STM32WB_GPIO_OTYPE_PUSHPULL  | STM32WB_GPIO_MODE_INPUT),  /* INPUT_PULLDOWN */
    (STM32WB_GPIO_PUPD_NONE     | STM32WB_GPIO_OSPEED_MEDIUM | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_OUTPUT), /* OUTPUT_OPENDRAIN */
    (STM32WB_GPIO_PUPD_NONE                                                                 | STM32WB_GPIO_MODE_ANALOG), /* HIGHZ */
};

void pinMode(uint32_t pin, uint32_t mode)
{
    if (mode > HIGHZ) {
        return;
    }
    
    if (pin >= PINS_COUNT) {
        return;
    }

    if (g_APinDescription[pin].GPIO == NULL) {
#if defined(STM32WB_CONFIG_PIN_BUTTON)
        if (g_APinDescription[pin].pin != STM32WB_CONFIG_PIN_BUTTON) {
            return;
        }

        if ((mode == OUTPUT) || (mode == OUTPUT_OPENDRAIN)) {
            return;
        }
#else /* defined(STM32WB_CONFIG_PIN_BUTTON) */
        return;
#endif /* defined(STM32WB_CONFIG_PIN_BUTTON) */
    } else {
        if (g_APinDescription[pin].attr & PIN_ATTR_SWD) {
            stm32wb_system_swd_disable();
        }
        
        if (g_APinDescription[pin].pwm_instance != PWM_INSTANCE_NONE) {
            __analogWriteDisable(pin);
        }
    }
    
    stm32wb_gpio_pin_configure(g_APinDescription[pin].pin, g_pinModeConfiguration[mode]);
}

void __attribute__((optimize("O3"))) digitalWrite(uint32_t pin, uint32_t output)
{
    GPIO_TypeDef *GPIO;
    uint32_t bit;

    if (pin >= PINS_COUNT) {
        return;
    }

    if (g_APinDescription[pin].GPIO == NULL) {
        return;
    }
    
    GPIO = g_APinDescription[pin].GPIO;
    bit = g_APinDescription[pin].bit;

    if (output) {
        GPIO->BSRR = bit;
    } else {
        GPIO->BRR = bit;
    }
}

int __attribute__((optimize("O3"))) digitalRead(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t bit;

    if (pin >= PINS_COUNT) {
        return 0;
    }

    if (g_APinDescription[pin].GPIO == NULL) {
#if defined(STM32WB_CONFIG_PIN_BUTTON)
        if (g_APinDescription[pin].pin != STM32WB_CONFIG_PIN_BUTTON) {
            return 0;
        }
        return !!stm32wb_gpio_pin_read(STM32WB_CONFIG_PIN_BUTTON);
#else /* defined(STM32WB_CONFIG_PIN_BUTTON) */
        return 0;
#endif /* defined(STM32WB_CONFIG_PIN_BUTTON) */
    }

    GPIO = g_APinDescription[pin].GPIO;
    bit = g_APinDescription[pin].bit;

    return !!(GPIO->IDR & bit);
}

