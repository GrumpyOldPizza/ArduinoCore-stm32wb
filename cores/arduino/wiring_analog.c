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
#include "wiring_private.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const unsigned int g_PWMInstances[PWM_INSTANCE_COUNT];

static stm32wb_tim_t __analog_pwm[PWM_INSTANCE_COUNT];
static uint8_t __analog_channels[PWM_INSTANCE_COUNT];

static int __analog_readResolution = 10;
static int __analog_readPeriod = 2;
static int __analog_writeResolution = 8;
static int __analog_writeFrequency = 0;
static k_mutex_t __analog_mutex = K_MUTEX_INIT(K_PRIORITY_MIN, K_MUTEX_PRIORITY_INHERIT | K_MUTEX_RECURSIVE);;

void analogReference(eAnalogReference reference)
{
    (void)reference;
}

void analogReadResolution(int resolution)
{
    if ((resolution >= 1) && (resolution <= 12)) {
        __analog_readResolution = resolution;
    }
}

void analogReadPeriod(int period)
{
    __analog_readPeriod = period;
}

uint32_t __analogReadChannel(uint32_t channel, uint32_t period)
{
    uint32_t data;

    if (!k_task_is_in_progress()) {
        return 0;
    }

    k_mutex_lock(&__analog_mutex, K_TIMEOUT_FOREVER);

    stm32wb_adc_enable();

    data = stm32wb_adc_read(channel, period);

    stm32wb_adc_disable();

    k_mutex_unlock(&__analog_mutex);
    
    return data;
}

uint32_t analogRead(uint32_t pin)
{
    uint32_t input;

    if ( pin < A0 )
    {
        pin += A0 ;
    }

    if ((pin >= PINS_COUNT) || (g_APinDescription[pin].adc_channel == ADC_CHANNEL_NONE)) {
        return 0;
    }
  
    if (g_APinDescription[pin].pwm_instance != PWM_INSTANCE_NONE) {
        __analogWriteDisable(pin);
    }

    stm32wb_gpio_pin_configure(g_APinDescription[pin].pin, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_MODE_ANALOG));
    
    input = __analogReadChannel((STM32WB_ADC_CHANNEL_1 + g_APinDescription[pin].adc_channel), __analog_readPeriod);

    if (__analog_readResolution != 12) {
        input = (input * ((1 << __analog_readResolution) -1)) / 4095;
    }
    
    return input;
}

void analogWriteResolution( int resolution )
{
    if ((resolution >= 1) && (resolution <= 12))
    {
        __analog_writeResolution = resolution;
    }
}

void analogWriteFrequency( unsigned long frequency )
{
    uint32_t instance, clock, carrier, modulus, prescale;

    __analog_writeFrequency = frequency;

    for (instance = 0; instance < PWM_INSTANCE_COUNT; instance++)
    {
        if (__analog_pwm[instance].state == STM32WB_TIM_STATE_ACTIVE)
        {
            clock = stm32wb_tim_clock(&__analog_pwm[instance]);

            modulus = 4095;

            if (__analog_writeFrequency)
            {
                carrier = __analog_writeFrequency * modulus;
            }
            else
            {
                carrier = 2000000;
            }
                
            if (carrier > clock)
            {
                carrier = (clock / modulus) * modulus;
            }
            
            prescale = (clock + (carrier - 1)) / carrier;

            stm32wb_tim_stop(&__analog_pwm[instance]);
            stm32wb_tim_start(&__analog_pwm[instance], prescale, modulus -1);
        }
    }
}
    
void analogWrite(uint32_t pin, uint32_t output)
{
    uint32_t instance, clock, carrier, modulus, prescale;
    GPIO_TypeDef *GPIO;
    uint32_t bit;

    // Handle the case the pin isn't usable as PIO
    if ((pin >= PINS_COUNT) || (g_APinDescription[pin].GPIO == NULL)) {
        return;
    }

    if (__analog_writeResolution != 12) {
        output = (output * 4095) / ((1 << __analog_writeResolution) -1);
    }

    if (output > 4095) {
        output = 4095;
    }
                    
    if (g_APinDescription[pin].pwm_instance != PWM_INSTANCE_NONE)
    {
        instance = g_APinDescription[pin].pwm_instance;

        if (__analog_channels[instance] & (1u << g_APinDescription[pin].pwm_channel)) {
            stm32wb_tim_compare(&__analog_pwm[instance], g_APinDescription[pin].pwm_channel, output);
        } else {
            __analog_channels[instance] |= (1u << g_APinDescription[pin].pwm_channel);

            if (__analog_pwm[instance].state == STM32WB_TIM_STATE_NONE) {
                stm32wb_tim_create(&__analog_pwm[instance], g_PWMInstances[instance], STM32WB_PWM_IRQ_PRIORITY);
            }
            
            if (__analog_pwm[instance].state == STM32WB_TIM_STATE_INIT) {
                stm32wb_tim_enable(&__analog_pwm[instance], 0, NULL, NULL);

                clock = stm32wb_tim_clock(&__analog_pwm[instance]);

                modulus = 4095;

                if (__analog_writeFrequency) {
                    carrier = __analog_writeFrequency * modulus;
                } else {
                    carrier = 2000000;
                }

                if (carrier > clock) {
                    carrier = (clock / modulus) * modulus;
                }

                prescale = (clock + (carrier - 1)) / carrier;
                
                stm32wb_tim_start(&__analog_pwm[instance], prescale, modulus -1);
            }
            
            stm32wb_gpio_pin_configure(g_APinDescription[pin].pin, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
            
            stm32wb_tim_channel(&__analog_pwm[instance], g_APinDescription[pin].pwm_channel, output, STM32WB_TIM_CONTROL_PWM);
        }

        return;
    }

    stm32wb_gpio_pin_configure(g_APinDescription[pin].pin, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_MEDIUM | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));

    GPIO = g_APinDescription[pin].GPIO;
    bit = g_APinDescription[pin].bit;

    if (output >= 2048) {
        GPIO->BSRR = bit;
    } else {
        GPIO->BRR = bit;
    }
}

void __analogWriteDisable(uint32_t pin)
{
    uint32_t instance;

    instance = g_APinDescription[pin].pwm_instance;

    if (__analog_channels[instance] & (1u << g_APinDescription[pin].pwm_channel)) {
        stm32wb_tim_channel(&__analog_pwm[instance], g_APinDescription[pin].pwm_channel, 0, STM32WB_TIM_CONTROL_DISABLE);
            
        stm32wb_gpio_pin_configure(g_APinDescription[pin].pin, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_MODE_ANALOG));

        __analog_channels[instance] &= ~(1u << g_APinDescription[pin].pwm_channel);
            
        if (!__analog_channels[instance]) {
            stm32wb_tim_stop(&__analog_pwm[instance]);
            stm32wb_tim_disable(&__analog_pwm[instance]);
        }
    }
}

#ifdef __cplusplus
}
#endif

