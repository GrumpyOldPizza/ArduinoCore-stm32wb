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

#ifdef __cplusplus
extern "C" {
#endif

extern const unsigned int g_PWMInstances[PWM_INSTANCE_COUNT];

static stm32wb_tim_t _pwm[PWM_INSTANCE_COUNT];

static uint8_t _channels[PWM_INSTANCE_COUNT];

static int _readResolution = 10;
static int _readPeriod = 2;
static int _writeResolution = 8;
static int _writeFrequency = 0;

void analogReference(eAnalogReference reference)
{
    (void)reference;
}

void analogReadResolution(int resolution)
{
    if ((resolution >= 1) && (resolution <= 12)) {
        _readResolution = resolution;
    }
}

void analogReadPeriod(int period)
{
    _readPeriod = period;
}

static uint32_t __analogReadRoutine(uint32_t channel, uint32_t period)
{
    uint32_t data;

    stm32wb_adc_enable();

    data = stm32wb_adc_read(channel, period);

    stm32wb_adc_disable();

    return data;
}

uint32_t __analogReadInternal(uint32_t channel, uint32_t period)
{
    if (!armv7m_core_is_in_thread()) {
        return 0;
    }

    return (uint32_t)armv7m_svcall_2((uint32_t)&__analogReadRoutine, (uint32_t)channel, (uint32_t)period);
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
    
    input = __analogReadInternal((STM32WB_ADC_CHANNEL_1 + g_APinDescription[pin].adc_channel), _readPeriod);

    if (_readResolution != 12) {
        input = (input * ((1 << _readResolution) -1)) / 4095;
    }
    
    return input;
}

void analogWriteResolution( int resolution )
{
    if ((resolution >= 1) && (resolution <= 12))
    {
        _writeResolution = resolution;
    }
}

void analogWriteFrequency( unsigned long frequency )
{
    uint32_t instance, clock, carrier, modulus, prescale;

    _writeFrequency = frequency;

    for (instance = 0; instance < PWM_INSTANCE_COUNT; instance++)
    {
        if (_pwm[instance].state == STM32WB_TIM_STATE_ACTIVE)
        {
            clock = stm32wb_tim_clock(&_pwm[instance]);

            modulus = 4095;

            if (_writeFrequency)
            {
                carrier = _writeFrequency * modulus;
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

            stm32wb_tim_stop(&_pwm[instance]);
            stm32wb_tim_start(&_pwm[instance], prescale, modulus -1);
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

    if (_writeResolution != 12) {
        output = (output * 4095) / ((1 << _writeResolution) -1);
    }

    if (output > 4095) {
        output = 4095;
    }
                    
    if (g_APinDescription[pin].pwm_instance != PWM_INSTANCE_NONE)
    {
        instance = g_APinDescription[pin].pwm_instance;

        if (_channels[instance] & (1u << g_APinDescription[pin].pwm_channel)) {
            stm32wb_tim_compare(&_pwm[instance], g_APinDescription[pin].pwm_channel, output);
        } else {
            _channels[instance] |= (1u << g_APinDescription[pin].pwm_channel);

            if (_pwm[instance].state == STM32WB_TIM_STATE_NONE) {
                stm32wb_tim_create(&_pwm[instance], g_PWMInstances[instance], STM32WB_PWM_IRQ_PRIORITY);
            }
            
            if (_pwm[instance].state == STM32WB_TIM_STATE_INIT) {
                stm32wb_tim_enable(&_pwm[instance], 0, NULL, NULL);

                clock = stm32wb_tim_clock(&_pwm[instance]);

                modulus = 4095;

                if (_writeFrequency) {
                    carrier = _writeFrequency * modulus;
                } else {
                    carrier = 2000000;
                }

                if (carrier > clock) {
                    carrier = (clock / modulus) * modulus;
                }

                prescale = (clock + (carrier - 1)) / carrier;
                
                stm32wb_tim_start(&_pwm[instance], prescale, modulus -1);
            }
            
            stm32wb_gpio_pin_configure(g_APinDescription[pin].pin, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
            
            stm32wb_tim_channel(&_pwm[instance], g_APinDescription[pin].pwm_channel, output, STM32WB_TIM_CONTROL_PWM);
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

    if (_channels[instance] & (1u << g_APinDescription[pin].pwm_channel)) {
        stm32wb_tim_channel(&_pwm[instance], g_APinDescription[pin].pwm_channel, 0, STM32WB_TIM_CONTROL_DISABLE);
            
        stm32wb_gpio_pin_configure(g_APinDescription[pin].pin, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_MODE_ANALOG));

        _channels[instance] &= ~(1u << g_APinDescription[pin].pwm_channel);
            
        if (!_channels[instance]) {
            stm32wb_tim_stop(&_pwm[instance]);
            stm32wb_tim_disable(&_pwm[instance]);
        }
    }
}

#ifdef __cplusplus
}
#endif

