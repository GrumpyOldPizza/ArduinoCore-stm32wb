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
  
extern const unsigned int g_PWMInstances[PWM_INSTANCE_COUNT];

static stm32wb_tim_t __analog_pwm[PWM_INSTANCE_COUNT];
static uint8_t __analog_channels[PWM_INSTANCE_COUNT];

static uint32_t __analog_readPeriod = 2000;
static uint32_t __analog_readOptions = (STM32WB_ADC_OPTION_RATIO_1 | STM32WB_ADC_OPTION_WIDTH_12);
static uint32_t __analog_readWidth = 12;
static uint32_t __analog_readResolution = 10;

static int __analog_writeResolution = 8;
static int __analog_writeFrequency = 0;

void analogReference(eAnalogReference reference)
{
    stm32wb_system_vref_configure((uint32_t)reference);
}

void analogReadPeriod(float period)
{
    if ((period >= 0.0f) && (period <= 40.0f)) {
        __analog_readPeriod = (int)(period * 1000.0f + 0.5f);
    }
}

void analogReadSamples(int samples)
{
    if      (samples >= 256) { __analog_readOptions = (STM32WB_ADC_OPTION_RATIO_256 | STM32WB_ADC_OPTION_WIDTH_16); __analog_readWidth = 16; }
    else if (samples >= 128) { __analog_readOptions = (STM32WB_ADC_OPTION_RATIO_128 | STM32WB_ADC_OPTION_WIDTH_16); __analog_readWidth = 16; }
    else if (samples >=  64) { __analog_readOptions = (STM32WB_ADC_OPTION_RATIO_64  | STM32WB_ADC_OPTION_WIDTH_16); __analog_readWidth = 16; }
    else if (samples >=  32) { __analog_readOptions = (STM32WB_ADC_OPTION_RATIO_32  | STM32WB_ADC_OPTION_WIDTH_16); __analog_readWidth = 16; }
    else if (samples >=  16) { __analog_readOptions = (STM32WB_ADC_OPTION_RATIO_16  | STM32WB_ADC_OPTION_WIDTH_16); __analog_readWidth = 16; }
    else if (samples >=   8) { __analog_readOptions = (STM32WB_ADC_OPTION_RATIO_8   | STM32WB_ADC_OPTION_WIDTH_15); __analog_readWidth = 15; }
    else if (samples >=   4) { __analog_readOptions = (STM32WB_ADC_OPTION_RATIO_4   | STM32WB_ADC_OPTION_WIDTH_14); __analog_readWidth = 14; }
    else if (samples >=   2) { __analog_readOptions = (STM32WB_ADC_OPTION_RATIO_2   | STM32WB_ADC_OPTION_WIDTH_13); __analog_readWidth = 13; }
    else                     { __analog_readOptions = (STM32WB_ADC_OPTION_RATIO_1   | STM32WB_ADC_OPTION_WIDTH_12); __analog_readWidth = 12; }
}
    
void analogReadResolution(int resolution)
{
    if ((resolution >= 1) && (resolution <= 16)) {
        __analog_readResolution = resolution;
    }
}

uint32_t __attribute__((optimize("O3"))) analogRead(uint32_t pin)
{
    uint8_t channels[1];
    uint16_t data[1];

    if ((pin >= PINS_COUNT) || (g_APinDescription[pin].adc_channel == ADC_CHANNEL_NONE)) {
        return 0;
    }
  
    if (g_APinDescription[pin].pwm_instance != PWM_INSTANCE_NONE) {
        __analogWriteDisable(pin);
    }

    channels[0] = STM32WB_ADC_CHANNEL_1 + g_APinDescription[pin].adc_channel;
    
    if (!stm32wb_adc_convert(&channels[0], &data[0], 1,__analog_readPeriod, (__analog_readOptions | STM32WB_ADC_OPTION_STOP))) {
        return 0;
    }

    if (__analog_readResolution != __analog_readWidth) {
        return ((uint32_t)data[0] << __analog_readResolution) >> __analog_readWidth;
    }

    return data[0];
}

uint32_t __attribute__((optimize("O3"))) analogRead(uint32_t pin, bool stop)
{
    uint8_t channels[1];
    uint16_t data[1];

    if ((pin >= PINS_COUNT) || (g_APinDescription[pin].adc_channel == ADC_CHANNEL_NONE)) {
        return 0;
    }
  
    if (g_APinDescription[pin].pwm_instance != PWM_INSTANCE_NONE) {
        __analogWriteDisable(pin);
    }

    stm32wb_gpio_pin_configure(g_APinDescription[pin].pin, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_MODE_ANALOG));

    channels[0] = STM32WB_ADC_CHANNEL_1 + g_APinDescription[pin].adc_channel;

    if (!stm32wb_adc_convert(&channels[0], &data[0], 1,__analog_readPeriod, (__analog_readOptions | (stop ? STM32WB_ADC_OPTION_STOP : 0)))) {
        return 0;
    }
    
    if (__analog_readResolution != __analog_readWidth) {
        return ((uint32_t)data[0] << __analog_readResolution) >> __analog_readWidth;
    }

    return data[0];
}

bool __attribute__((optimize("O3"))) analogRead(const uint8_t pins[], uint16_t data[], uint32_t count, bool stop)
{
    uint32_t index, pin;
    uint8_t channels[8];

    if (!count || (count > 8)) {
        return false;
    }

    for (index = 0; index < count; index++) {
        pin = pins[index];
        
        if ((pin >= PINS_COUNT) || (g_APinDescription[pin].adc_channel == ADC_CHANNEL_NONE)) {
            return false;
        }
  
        if (g_APinDescription[pin].pwm_instance != PWM_INSTANCE_NONE) {
            __analogWriteDisable(pin);
        }

        stm32wb_gpio_pin_configure(g_APinDescription[pin].pin, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_MODE_ANALOG));

        channels[index] = STM32WB_ADC_CHANNEL_1 + g_APinDescription[pin].adc_channel;
    }
    
    if (!stm32wb_adc_convert(&channels[0], &data[0], count, __analog_readPeriod, (__analog_readOptions | (stop ? STM32WB_ADC_OPTION_STOP : 0)))) {
        return false;
    }

    if (__analog_readResolution != __analog_readWidth) {
        data[index] = ((uint32_t)data[index] << __analog_readResolution) >> __analog_readWidth;
    }

    return true;
}

void analogWriteResolution( int resolution )
{
    if ((resolution >= 1) && (resolution <= 12)) {
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

    GPIO = (GPIO_TypeDef*)g_APinDescription[pin].GPIO;
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
