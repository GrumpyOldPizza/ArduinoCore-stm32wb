/*
 * Copyright (c) 2016-2023 Thomas Roell.  All rights reserved.
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
#include "stm32wb_dma.h"
#include "stm32wb_system.h"

#define ADC_SAMPLE_TIME_2_5         0
#define ADC_SAMPLE_TIME_6_5         1
#define ADC_SAMPLE_TIME_12_5        2
#define ADC_SAMPLE_TIME_24_5        3
#define ADC_SAMPLE_TIME_47_5        4
#define ADC_SAMPLE_TIME_92_5        5
#define ADC_SAMPLE_TIME_247_5       6
#define ADC_SAMPLE_TIME_640_5       7

#define ADC_CCR_CKMODE_SYSCLK       0
#define ADC_CCR_CKMODE_HCLK_DIV_1   (ADC_CCR_CKMODE_0)
#define ADC_CCR_CKMODE_HCLK_DIV_2   (ADC_CCR_CKMODE_1)
#define ADC_CCR_CKMODE_HCLK_DIV_4   (ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1)

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

#define ADC_CFGR2_OVSR_RATIO_2      (0 << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_RATIO_4      (1 << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_RATIO_8      (2 << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_RATIO_16     (3 << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_RATIO_32     (4 << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_RATIO_64     (5 << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_RATIO_128    (6 << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_RATIO_256    (7 << ADC_CFGR2_OVSR_Pos)

#define ADC_CFGR2_OVSS_SHIFT_0      (0 << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_SHIFT_1      (1 << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_SHIFT_2      (2 << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_SHIFT_3      (3 << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_SHIFT_4      (4 << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_SHIFT_5      (5 << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_SHIFT_6      (6 << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_SHIFT_7      (7 << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_SHIFT_8      (8 << ADC_CFGR2_OVSS_Pos)
  
#define STM32WB_ADC_CALFACT_UNDEFINED 0xffffffff

#define STM32WB_ADC_SAMPLE_TIME(_ticks, _adcclk) ((uint32_t)((((double)(_ticks) * (double)1e9) / (double)(_adcclk)) + 0.5))

static const uint32_t stm32wb_adc_threshold_2[8] = {
    STM32WB_ADC_SAMPLE_TIME(2.5,   2000000),
    STM32WB_ADC_SAMPLE_TIME(6.5,   2000000),
    STM32WB_ADC_SAMPLE_TIME(12.5,  2000000),
    STM32WB_ADC_SAMPLE_TIME(24.5,  2000000),
    STM32WB_ADC_SAMPLE_TIME(47.5,  2000000),
    STM32WB_ADC_SAMPLE_TIME(92.5,  2000000),
    STM32WB_ADC_SAMPLE_TIME(247.5, 2000000),
    STM32WB_ADC_SAMPLE_TIME(640.5, 2000000),
};

static const uint32_t stm32wb_adc_threshold_16[8] = {
    STM32WB_ADC_SAMPLE_TIME(2.5,   16000000),
    STM32WB_ADC_SAMPLE_TIME(6.5,   16000000),
    STM32WB_ADC_SAMPLE_TIME(12.5,  16000000),
    STM32WB_ADC_SAMPLE_TIME(24.5,  16000000),
    STM32WB_ADC_SAMPLE_TIME(47.5,  16000000),
    STM32WB_ADC_SAMPLE_TIME(92.5,  16000000),
    STM32WB_ADC_SAMPLE_TIME(247.5, 16000000),
    STM32WB_ADC_SAMPLE_TIME(640.5, 16000000),
};

static const uint32_t stm32wb_adc_threshold_32[8] = {
    STM32WB_ADC_SAMPLE_TIME(2.5,   32000000),
    STM32WB_ADC_SAMPLE_TIME(6.5,   32000000),
    STM32WB_ADC_SAMPLE_TIME(12.5,  32000000),
    STM32WB_ADC_SAMPLE_TIME(24.5,  32000000),
    STM32WB_ADC_SAMPLE_TIME(47.5,  32000000),
    STM32WB_ADC_SAMPLE_TIME(92.5,  32000000),
    STM32WB_ADC_SAMPLE_TIME(247.5, 32000000),
    STM32WB_ADC_SAMPLE_TIME(640.5, 32000000),
};

#define STM32WB_ADC_CALFACT_UNDEFINED 0xffffffff

#define STM32WB_ADC_DMA_CHANNEL (STM32WB_DMA_CHANNEL_DMA1_CH1_INDEX | STM32WB_DMA_CHANNEL_SELECT_ADC1)
#define STM32WB_ADC_DMA_OPTIONS (STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |    \
                                 STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_16 | \
                                 STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_16 |     \
                                 STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
                                 STM32WB_DMA_OPTION_PRIORITY_VERY_HIGH)

#define STM32WB_ADC_STATE_STOP  0
#define STM32WB_ADC_STATE_START 1
#define STM32WB_ADC_STATE_DONE  2

typedef struct _stm32wb_adc_device_t {
    volatile uint8_t          state;
    uint8_t                   count;
    uint32_t                  mask;
    volatile uint16_t         data[8];
    uint32_t                  ovs;
    uint32_t                  smp;
    uint32_t                  calfact;
    uint32_t                  period;
    uint32_t                  option;
    uint32_t                  hclk;
    const uint32_t            *threshold;
} stm32wb_adc_device_t;

static stm32wb_adc_device_t stm32wb_adc_device;

void __stm32wb_adc_initialize(void)
{
    NVIC_SetPriority(ADC1_IRQn, STM32WB_ADC_IRQ_PRIORITY);
    NVIC_EnableIRQ(ADC1_IRQn);
}

static bool __attribute__((optimize("O3"))) __svc_stm32wb_adc_start(const uint8_t *channels, uint32_t count, uint32_t period, uint32_t option)
{
    uint32_t hclk, ckmode, ccr, ccr_next, sqr1, sqr2, smp, ratio, width, index, channel, mask;

    if (!count || (count > 8))
    {
        return false;
    }

    if (stm32wb_adc_device.state != STM32WB_ADC_STATE_DONE)
    {
        if (stm32wb_adc_device.state == STM32WB_ADC_STATE_START)
        {
            return false;
        }

        stm32wb_system_lock(STM32WB_SYSTEM_LOCK_CLOCKS);
        stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
        
        stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_ADC);

        hclk = stm32wb_system_hclk();
        
        if (stm32wb_adc_device.hclk != hclk)
        {
            if (hclk == 2000000)
            {
                ckmode = ADC_CCR_CKMODE_HCLK_DIV_1;
                
                stm32wb_adc_device.threshold = stm32wb_adc_threshold_2;
            }
            else if (hclk == 16000000)
            {
                ckmode = ADC_CCR_CKMODE_HCLK_DIV_1;
                
                stm32wb_adc_device.threshold = stm32wb_adc_threshold_16;
            }
            else
            {
                if   (hclk == 32000000) { ckmode = ADC_CCR_CKMODE_HCLK_DIV_1; }
                else                    { ckmode = ADC_CCR_CKMODE_HCLK_DIV_2; }
                
                stm32wb_adc_device.threshold = stm32wb_adc_threshold_32;
            }
            
            ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~(ADC_CCR_CKMODE | ADC_CCR_PRESC)) | ckmode;
            
            stm32wb_adc_device.count = 0;
            stm32wb_adc_device.mask = 0;
            stm32wb_adc_device.ovs = 0;
            stm32wb_adc_device.smp = 0;
            stm32wb_adc_device.calfact = STM32WB_ADC_CALFACT_UNDEFINED;
            stm32wb_adc_device.hclk = hclk;
            stm32wb_adc_device.period = 0;
            stm32wb_adc_device.option = 0;
        }

        ADC1->CR = 0; // reset ADC_CR_DEEPPWD
        
        ADC1->CR = ADC_CR_ADVREGEN;
        
        armv7m_core_udelay(20);
        
        if (stm32wb_adc_device.calfact == STM32WB_ADC_CALFACT_UNDEFINED)
        {
            ADC1->CR = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
            
            while (ADC1->CR & ADC_CR_ADCAL)
            {
                __NOP();
                __NOP();
                __NOP();
                __NOP();
            }
            
            stm32wb_adc_device.calfact = ADC1->CALFACT;
        }
    
        ADC1->ISR = ADC_ISR_ADRDY;

        do
        {
            ADC1->CR = ADC_CR_ADVREGEN | ADC_CR_ADEN;
        }
        while (!(ADC1->ISR & ADC_ISR_ADRDY));

        stm32wb_dma_enable(STM32WB_ADC_DMA_CHANNEL, 0, NULL, NULL);
        
        stm32wb_adc_device.state = STM32WB_ADC_STATE_DONE;
    }
    
    if (stm32wb_adc_device.period != period)
    {
        stm32wb_adc_device.period = period;

        for (smp = 0; smp < 7; smp++)
        {
            if (period <= stm32wb_adc_device.threshold[smp])
            {
                break;
            }
        }

        stm32wb_adc_device.smp = (smp << 0) | (smp << 3) | (smp << 6) | (smp << 9) | (smp << 12) | (smp << 15) | (smp << 18) | (smp << 21) | (smp << 24) | (smp << 27);
    }

    if (stm32wb_adc_device.option != (option & (STM32WB_ADC_OPTION_WIDTH_MASK | STM32WB_ADC_OPTION_RATIO_MASK)))
    {
        stm32wb_adc_device.option = (option & (STM32WB_ADC_OPTION_WIDTH_MASK | STM32WB_ADC_OPTION_RATIO_MASK));

        if (option & STM32WB_ADC_OPTION_RATIO_MASK)
        {
            ratio = (option & STM32WB_ADC_OPTION_RATIO_MASK) >> STM32WB_ADC_OPTION_RATIO_SHIFT;
            width = (option & STM32WB_ADC_OPTION_WIDTH_MASK) >> STM32WB_ADC_OPTION_WIDTH_SHIFT;

            if (width > ratio)
            {
                width = ratio;
            }
            
            stm32wb_adc_device.ovs = ADC_CFGR2_ROVSE | ((ratio - 1) << ADC_CFGR2_OVSR_Pos) | ((ratio - width) << ADC_CFGR2_OVSS_Pos);
        }
        else
        {
            stm32wb_adc_device.ovs = 0;
        }
    }


    for (index = 0, mask = 0, sqr1 = (count -1), sqr2 = 0; index < count; index++)
    {
        channel = channels[index];
      
        mask |= (1u << channel);
        
        if (index <= 3)
        {
            sqr1 |= (channel << (6 * index + 6));
        }
        else
        {
            sqr2 |= (channel << (6 * (index - 4)));
        }
    }
    
    if ((stm32wb_adc_device.mask ^ mask) & ((1u << STM32WB_ADC_CHANNEL_VREFINT) | (1u << STM32WB_ADC_CHANNEL_TSENSE) | (1u << STM32WB_ADC_CHANNEL_VBAT)))
    {
        ccr = ccr_next = ADC1_COMMON->CCR & ~(ADC_CCR_VBATEN | ADC_CCR_TSEN | ADC_CCR_VREFEN);

        if (mask & (1u << STM32WB_ADC_CHANNEL_VREFINT))
        {
            ccr_next |= ADC_CCR_VREFEN;
        }
        
        if (mask & (1u << STM32WB_ADC_CHANNEL_TSENSE))
        {
            ccr_next |= ADC_CCR_TSEN;
        }
        
        if (mask & (1u << STM32WB_ADC_CHANNEL_VBAT))
        {
            ccr_next |= ADC_CCR_VBATEN;
        }
        
        ADC1_COMMON->CCR = ccr_next;
        ADC1_COMMON->CCR;
        
        if ((ccr ^ ccr_next) & ADC_CCR_TSEN)
        {
            armv7m_core_udelay(120);
        }
    }

    stm32wb_adc_device.count = count;
    stm32wb_adc_device.mask = mask;
    
    /* Silicon ERRATA 2.4.4. Wrong ADC conversion results when delay between
     * calibration and first conversion or between 2 consecutive conversions is too long. 
     */

    ADC1->CALFACT = stm32wb_adc_device.calfact;

    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS;
    ADC1->IER = 0;
    ADC1->CFGR = ADC_CFGR_OVRMOD | ADC_CFGR_JQDIS;
    ADC1->CFGR2 = 0;
    ADC1->SMPR1 = 0;
    ADC1->SMPR2 = 0;
    ADC1->SQR1 = (channels[0] << 6);
    ADC1->SQR2 = 0;

    ADC1->CR = ADC_CR_ADVREGEN | ADC_CR_ADSTART;

    /* Setup the DMA early, so that it overlaps with waiting for the initial converstion.
     */
    stm32wb_dma_start(STM32WB_ADC_DMA_CHANNEL, (uint32_t)&stm32wb_adc_device.data[0], (uint32_t)&ADC1->DR, count, STM32WB_ADC_DMA_OPTIONS);
    
    while (!(ADC1->ISR & ADC_ISR_EOS))
    {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }

    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS;
    ADC1->IER = ADC_IER_EOSIE;

    ADC1->CFGR = ADC_CFGR_OVRMOD | ADC_CFGR_JQDIS | ADC_CFGR_DMAEN;
    ADC1->CFGR2 = stm32wb_adc_device.ovs;
    ADC1->SMPR1 = stm32wb_adc_device.smp;
    ADC1->SMPR2 = stm32wb_adc_device.smp;
    ADC1->SQR1 = sqr1;
    ADC1->SQR2 = sqr2;
        
    stm32wb_adc_device.state = STM32WB_ADC_STATE_START;
    
    ADC1->CR = ADC_CR_ADVREGEN | ADC_CR_ADSTART;

    return true;
}

static bool __attribute__((optimize("O3"))) __svc_stm32wb_adc_stop(void)
{
    if (stm32wb_adc_device.state == STM32WB_ADC_STATE_DONE)
    {
        stm32wb_dma_disable(STM32WB_ADC_DMA_CHANNEL);

        ADC1->CR = ADC_CR_ADVREGEN | ADC_CR_ADDIS;

        while (ADC1->CR & ADC_CR_ADEN)
        {
            __NOP();
            __NOP();
            __NOP();
            __NOP();
        }
        
        ADC1->CR = 0; // reset ADC_CR_ADVREGEN
        
        ADC1->CR = ADC_CR_DEEPPWD;

        ADC1_COMMON->CCR &= ~(ADC_CCR_VREFEN | ADC_CCR_VBATEN | ADC_CCR_TSEN);
        ADC1_COMMON->CCR;
        
        stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_ADC);

        stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);
        stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_CLOCKS);

        stm32wb_adc_device.state = STM32WB_ADC_STATE_STOP;
        stm32wb_adc_device.count = 0;
        stm32wb_adc_device.mask = 0;

        return true;
    }

    if (stm32wb_adc_device.state == STM32WB_ADC_STATE_STOP)
    {
        return true;
    }
        
    return false;
}

bool stm32wb_adc_start(const uint8_t *channels, uint32_t count, uint32_t period, uint32_t option)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_4((uint32_t)&__svc_stm32wb_adc_start, (uint32_t)channels, count, period, option);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_adc_start(channels, count, period, option);
    }

    return false;
}

bool stm32wb_adc_stop(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_adc_stop);
    }
    
    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_adc_stop();
    }

    return false;
}

bool __attribute__((optimize("O3"))) stm32wb_adc_done(void)
{
    return (stm32wb_adc_device.state == STM32WB_ADC_STATE_START);
}

uint32_t __attribute__((optimize("O3"))) stm32wb_adc_data(uint16_t *data, uint32_t count)
{
    uint32_t index;

    if (stm32wb_adc_device.state != STM32WB_ADC_STATE_DONE)
    {
        return 0;
    }

    if (count > stm32wb_adc_device.count)
    {
        count = stm32wb_adc_device.count;
    }
    
    for (index = 0; index < count; index++)
    {
        data[index] = stm32wb_adc_device.data[index];
    }

    return count;
}

uint32_t __attribute__((optimize("O3"))) stm32wb_adc_convert(const uint8_t *channels, uint16_t *data, uint32_t count, uint32_t period, uint32_t option)
{
    uint32_t index;
  
    if (!stm32wb_adc_start(channels, count, period, option))
    {
        return 0;
    }

    while (stm32wb_adc_device.state != STM32WB_ADC_STATE_DONE)
    {
        __WFE();
    }

    for (index = 0; index < count; index++)
    {
        data[index] = stm32wb_adc_device.data[index];
    }

    if (option & STM32WB_ADC_OPTION_STOP)
    {
        stm32wb_adc_stop();
    }

    return count;
}

void ADC1_IRQHandler(void)
{
    ADC1->IER = 0;
    ADC1->ISR = ADC_ISR_EOS;

    stm32wb_adc_device.state = STM32WB_ADC_STATE_DONE;

    __DSB();
}

/***********************************************************************************************/
