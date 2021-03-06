/*
 * Copyright (c) 2016-2017 Thomas Roell.  All rights reserved.
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
#include "stm32wbxx.h"

#include "stm32wb_adc.h"
#include "stm32wb_system.h"

#define ADC_CFGR2_CKMODE_SYSCLK     0
#define ADC_CFGR2_CKMODE_PCLK_DIV_2 (ADC_CFGR2_CKMODE_0)
#define ADC_CFGR2_CKMODE_PCLK_DIV_4 (ADC_CFGR2_CKMODE_1)
#define ADC_CFGR2_CKMODE_PCLK_DIV_1 (ADC_CFGR2_CKMODE_1 | ADC_CFGR2_CKMODE_0)

#define ADC_SAMPLE_TIME_2_5         0
#define ADC_SAMPLE_TIME_6_5         1
#define ADC_SAMPLE_TIME_12_5        2
#define ADC_SAMPLE_TIME_24_5        3
#define ADC_SAMPLE_TIME_47_5        4
#define ADC_SAMPLE_TIME_92_5        5
#define ADC_SAMPLE_TIME_247_5       6
#define ADC_SAMPLE_TIME_640_5       7

#define ADC_CCR_PRESC_DIV_1         0
#define ADC_CCR_PRESC_DIV_2         (ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_4         (ADC_CCR_PRESC_1)
#define ADC_CCR_PRESC_DIV_6         (ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_8         (ADC_CCR_PRESC_2)
#define ADC_CCR_PRESC_DIV_10        (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_12        (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1)
#define ADC_CCR_PRESC_DIV_16        (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_32        (ADC_CCR_PRESC_3)
#define ADC_CCR_PRESC_DIV_64        (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_128       (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1)
#define ADC_CCR_PRESC_DIV_256       (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0)

#define STM32WB_ADC_STATE_NONE      0
#define STM32WB_ADC_STATE_READY     1

typedef struct _stm32wb_adc_device_t {
    volatile uint8_t             state;
    uint8_t                      calibration;
    uint16_t                     period;
} stm32wb_adc_device_t;

static stm32wb_adc_device_t stm32wb_adc_device;

bool stm32wb_adc_enable(void)
{
    if (stm32wb_adc_device.state != STM32WB_ADC_STATE_NONE)
    {
        return false;
    }

    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_ADC);

    if (stm32wb_system_hclk() <= 48000000)
    {
	armv7m_atomic_modify(&ADC1_COMMON->CCR, ADC_CCR_CKMODE, ADC_CCR_CKMODE_0); /* HCLK / 1 */
    }
    else
    {
	armv7m_atomic_modify(&ADC1_COMMON->CCR, ADC_CCR_CKMODE, ADC_CCR_CKMODE_1); /* HCLK / 2 */
    }

    armv7m_atomic_or(&ADC1_COMMON->CCR, (ADC_CCR_VBATEN | ADC_CCR_VREFEN));

    ADC1->CR &= ~ADC_CR_DEEPPWD;

    ADC1->CR |= ADC_CR_ADVREGEN;
    
    armv7m_core_udelay(20);
    
    /* Finally turn on the ADC */
    
    ADC1->ISR = ADC_ISR_ADRDY;

    do
    {
	ADC1->CR |= ADC_CR_ADEN;
    }
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
    
    ADC1->CFGR = ADC_CFGR_OVRMOD | ADC_CFGR_JQDIS;

    if (!stm32wb_adc_device.calibration)
    {
	ADC1->CR |= ADC_CR_ADDIS;
	
	while (ADC1->CR & ADC_CR_ADEN)
	{
	}
	
	/* Single-Ended Input Calibration */
	ADC1->CR &= ~ADC_CR_ADCALDIF;
	ADC1->CR |= ADC_CR_ADCAL;
	
	while (ADC1->CR & ADC_CR_ADCAL)
	{
	}
	
	/* Differential Input Calibration */
	ADC1->CR |= (ADC_CR_ADCALDIF | ADC_CR_ADCAL);
	
	while (ADC1->CR & ADC_CR_ADCAL)
	{
	}
	
	armv7m_core_udelay(100);
	
	ADC1->ISR = ADC_ISR_ADRDY;
	
	do
	{
	    ADC1->CR |= ADC_CR_ADEN;
	}
	while (!(ADC1->ISR & ADC_ISR_ADRDY));

	stm32wb_adc_device.calibration = 1;
    }
    
    stm32wb_adc_device.state = STM32WB_ADC_STATE_READY;

    return true;
}

bool stm32wb_adc_disable(void)
{
    if (stm32wb_adc_device.state != STM32WB_ADC_STATE_READY)
    {
        return false;
    }

    ADC1->CR |= ADC_CR_ADDIS;

    while (ADC1->CR & ADC_CR_ADEN)
    {
    }

    ADC1->CR &= ~ADC_CR_ADVREGEN;
    ADC1->CR |= ADC_CR_DEEPPWD;

    armv7m_atomic_and(&ADC1_COMMON->CCR, ~(ADC_CCR_VBATEN | ADC_CCR_VREFEN));

    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_ADC);

    stm32wb_adc_device.state = STM32WB_ADC_STATE_NONE;

    return true;
}

uint32_t stm32wb_adc_read(unsigned int channel, unsigned int period)
{
    uint32_t convert, threshold, adcclk, adc_smp;

    if (stm32wb_adc_device.state != STM32WB_ADC_STATE_READY)
    {
        return 0;
    }

    /* Silicon ERRATA 2.4.4. Wrong ADC conversion results when delay between
     * calibration and first conversion or between 2 consecutive conversions is too long. 
     */

    if (channel == STM32WB_ADC_CHANNEL_TSENSE)
    {
	ADC1->CR |= ADC_CR_ADDIS;

	while (ADC1->CR & ADC_CR_ADEN)
	{
	}
	    
	armv7m_atomic_or(&ADC1_COMMON->CCR, ADC_CCR_TSEN);

	ADC1->ISR = ADC_ISR_ADRDY;

	do
	{
	    ADC1->CR |= ADC_CR_ADEN;
	}
	while (!(ADC1->ISR & ADC_ISR_ADRDY));

	armv7m_core_udelay(120);
    }

    if (stm32wb_system_hclk() <= 48000000)
    {
	adcclk = stm32wb_system_hclk();
    }
    else
    {
	adcclk = stm32wb_system_hclk() / 2;
    }

    /* period is in uS. 1e6 / adcclk is one tick in terms of uS.
     *
     * (period * adcclk) / 1e6 is the threshold for the sampling time.
     *
     * The upper limit for period is 50uS, and adcclk limited to 48MHz,
     * which means no overflow handling is needed.
     */

    if (period > 50)
    {
        period = 50;
    }
    
    threshold = ((uint32_t)period * adcclk);

    if      (threshold < (uint32_t)(  2.5 * 1e6)) { adc_smp = ADC_SAMPLE_TIME_2_5;   } 
    else if (threshold < (uint32_t)(  6.5 * 1e6)) { adc_smp = ADC_SAMPLE_TIME_6_5;   } 
    else if (threshold < (uint32_t)( 12.5 * 1e6)) { adc_smp = ADC_SAMPLE_TIME_12_5;  } 
    else if (threshold < (uint32_t)( 24.5 * 1e6)) { adc_smp = ADC_SAMPLE_TIME_24_5;  } 
    else if (threshold < (uint32_t)( 47.5 * 1e6)) { adc_smp = ADC_SAMPLE_TIME_47_5;  } 
    else if (threshold < (uint32_t)( 92.5 * 1e6)) { adc_smp = ADC_SAMPLE_TIME_92_5;  } 
    else if (threshold < (uint32_t)(247.5 * 1e6)) { adc_smp = ADC_SAMPLE_TIME_247_5; } 
    else                                          { adc_smp = ADC_SAMPLE_TIME_640_5; } 

    ADC1->SQR1 = (channel << 6);
    ADC1->SMPR1 = (channel < 10) ? (adc_smp << (channel * 3)) : 0;
    ADC1->SMPR2 = (channel >= 10) ? (adc_smp << ((channel * 3) - 30)) : 0;

    ADC1->CR |= ADC_CR_ADSTART;
    
    while (!(ADC1->ISR & ADC_ISR_EOC))
    {
    }
    
    convert = ADC1->DR & ADC_DR_RDATA;
	
    ADC1->ISR = ADC_ISR_EOC;
    
    ADC1->CR |= ADC_CR_ADSTART;
    
    while (!(ADC1->ISR & ADC_ISR_EOC))
    {
    }
    
    convert = ADC1->DR & ADC_DR_RDATA;

    ADC1->ISR = ADC_ISR_EOC;

    if (channel == STM32WB_ADC_CHANNEL_TSENSE)
    {
	ADC1->CR |= ADC_CR_ADDIS;

	while (ADC1->CR & ADC_CR_ADEN)
	{
	}
	
	armv7m_atomic_and(&ADC1_COMMON->CCR, ~ADC_CCR_TSEN);

	ADC1->ISR = ADC_ISR_ADRDY;

	do
	{
	    ADC1->CR |= ADC_CR_ADEN;
	}
	while (!(ADC1->ISR & ADC_ISR_ADRDY));
    }

    return convert;
}
