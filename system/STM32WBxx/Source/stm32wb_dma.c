/*
 * Copyright (c) 2017-2020 Thomas Roell.  All rights reserved.
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
#include "stm32wb_system.h"
#include "stm32wb_dma.h"

static DMA_Channel_TypeDef * const  stm32wb_dma_xlate_DMA[14] = {
    DMA1_Channel1,
    DMA1_Channel2,
    DMA1_Channel3,
    DMA1_Channel4,
    DMA1_Channel5,
    DMA1_Channel6,
    DMA1_Channel7,
    DMA2_Channel1,
    DMA2_Channel2,
    DMA2_Channel3,
    DMA2_Channel4,
    DMA2_Channel5,
    DMA2_Channel6,
    DMA2_Channel7,
};

static const IRQn_Type stm32wb_dma_interrupt_table[14] = {
    DMA1_Channel1_IRQn,
    DMA1_Channel2_IRQn,
    DMA1_Channel3_IRQn,
    DMA1_Channel4_IRQn,
    DMA1_Channel5_IRQn,
    DMA1_Channel6_IRQn,
    DMA1_Channel7_IRQn,
    DMA2_Channel1_IRQn,
    DMA2_Channel2_IRQn,
    DMA2_Channel3_IRQn,
    DMA2_Channel4_IRQn,
    DMA2_Channel5_IRQn,
    DMA2_Channel6_IRQn,
    DMA2_Channel7_IRQn,
};

static DMAMUX_Channel_TypeDef * const  stm32wb_dma_xlate_DMAMUX[14] = {
    DMAMUX1_Channel0,
    DMAMUX1_Channel1,
    DMAMUX1_Channel2,
    DMAMUX1_Channel3,
    DMAMUX1_Channel4,
    DMAMUX1_Channel5,
    DMAMUX1_Channel6,
    DMAMUX1_Channel7,
    DMAMUX1_Channel8,
    DMAMUX1_Channel9,
    DMAMUX1_Channel10,
    DMAMUX1_Channel11,
    DMAMUX1_Channel12,
    DMAMUX1_Channel13,
};

typedef struct _stm32wb_dma_t {
    volatile uint16_t               channel;
    uint16_t                        size;
    stm32wb_dma_callback_t volatile callback;
    void * volatile                 context;
} stm32wb_dma_t;

typedef struct _stm32wb_dma_device_t {
    stm32wb_dma_t                   channels[14];
    volatile uint32_t               dma;
    volatile uint32_t               flash;
} stm32wb_dma_device_t;

static stm32wb_dma_device_t stm32wb_dma_device;

static inline void stm32wb_dma1_interrupt(stm32wb_dma_t *dma, DMA_Channel_TypeDef *DMA, uint32_t dma_isr, uint32_t shift)
{
    uint32_t events;
    stm32wb_dma_callback_t callback;
    void *context;
    
    events = (dma_isr >> shift) & 0x0000000e;

    if (events & DMA->CCR)
    {
	DMA1->IFCR = (15 << shift);

	__armv7m_atomic_load_2_restart((volatile uint32_t*)&dma->callback, (uint32_t*)&callback, (uint32_t*)&context);

	if (callback)
	{
	    (callback)(context, events);
	}
    }
}

static inline void stm32wb_dma2_interrupt(stm32wb_dma_t *dma, DMA_Channel_TypeDef *DMA, uint32_t dma_isr, uint32_t shift)
{
    uint32_t events;
    stm32wb_dma_callback_t callback;
    void *context;
    
    events = (dma_isr >> shift) & 0x0000000e;

    if (events & DMA->CCR)
    {
	DMA2->IFCR = (15 << shift);

	__armv7m_atomic_load_2_restart((volatile uint32_t*)&dma->callback, (uint32_t*)&callback, (uint32_t*)&context);

	if (callback)
	{
	    (callback)(context, events);
	}
    }
}

void __stm32wb_dma_initialize(void)
{
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);

    NVIC_EnableIRQ(DMA2_Channel1_IRQn);
    NVIC_EnableIRQ(DMA2_Channel2_IRQn);
    NVIC_EnableIRQ(DMA2_Channel3_IRQn);
    NVIC_EnableIRQ(DMA2_Channel4_IRQn);
    NVIC_EnableIRQ(DMA2_Channel5_IRQn);
    NVIC_EnableIRQ(DMA2_Channel6_IRQn);
    NVIC_EnableIRQ(DMA2_Channel7_IRQn);
}

__attribute__((optimize("O3"))) bool stm32wb_dma_channel(uint16_t channel)
{
    uint32_t mask;

    if (channel == STM32WB_DMA_CHANNEL_NONE)
    {
	return false;
    }

    mask = 1ul << (channel & 15);

    if (!(stm32wb_dma_device.dma & mask))
    {
	return false;
    }
    
    return (stm32wb_dma_device.channels[(channel & 15)].channel == channel);
}

bool stm32wb_dma_enable(uint16_t channel, uint8_t priority, stm32wb_dma_callback_t callback, void *context)
{
    DMAMUX_Channel_TypeDef *DMAMUX = stm32wb_dma_xlate_DMAMUX[channel & 15];
    stm32wb_dma_t *dma = &stm32wb_dma_device.channels[channel & 15];
    uint32_t mask;

    mask = 1 << (channel & 15);

    if (channel == STM32WB_DMA_CHANNEL_NONE)
    {
	return false;
    }

    if (armv7m_atomic_cash(&dma->channel, STM32WB_DMA_CHANNEL_NONE, channel) != STM32WB_DMA_CHANNEL_NONE)
    {
	return false;
    }
    
    NVIC_SetPriority(stm32wb_dma_interrupt_table[channel & 15], priority);

    armv7m_atomic_or(&stm32wb_dma_device.dma, mask);
    armv7m_atomic_or(&RCC->AHB1ENR, (RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMAMUX1EN));

    RCC->AHB1ENR;

    armv7m_atomic_or(&RCC->AHB1SMENR, RCC_AHB1SMENR_SRAM1SMEN);
    armv7m_atomic_or(&RCC->AHB3SMENR, RCC_AHB3SMENR_SRAM2SMEN);

    DMAMUX->CCR = (channel >> 4) & DMAMUX_CxCR_DMAREQ_ID;

    dma->callback = NULL;
    dma->context = context;
    dma->callback = callback;

    return true;
}

void stm32wb_dma_disable(uint16_t channel)
{
    stm32wb_dma_t *dma = &stm32wb_dma_device.channels[channel & 15];
    uint32_t mask;

    mask = 1 << (channel & 15);

    armv7m_atomic_and(&stm32wb_dma_device.dma, ~mask);

    armv7m_atomic_andz(&RCC->AHB1SMENR, ~RCC_AHB1SMENR_SRAM1SMEN, &stm32wb_dma_device.dma);
    armv7m_atomic_andz(&RCC->AHB3SMENR, ~RCC_AHB3SMENR_SRAM2SMEN, &stm32wb_dma_device.dma);

    armv7m_atomic_andz(&RCC->AHB1ENR, ~(RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMAMUX1EN), &stm32wb_dma_device.dma);

    RCC->AHB1ENR;

    dma->channel = STM32WB_DMA_CHANNEL_NONE;
}

__attribute__((optimize("O3"))) void stm32wb_dma_start(uint16_t channel, uint32_t tx_data, uint32_t rx_data, uint16_t xf_count, uint32_t option)
{
    DMA_Channel_TypeDef *DMA = stm32wb_dma_xlate_DMA[channel & 15];
    stm32wb_dma_t *dma = &stm32wb_dma_device.channels[channel & 15];
    uint32_t shift, mask, xf_address;

    shift = (channel & 15) << 2;

    if (shift < 28)
    {
	DMA1->IFCR = (15 << shift);
    }
    else
    {
	DMA2->IFCR = (15 << (shift - 28));
    }

    dma->size = xf_count;

    DMA->CNDTR = xf_count;
	   
    if (option & STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL)
    {
        DMA->CMAR = rx_data;
        DMA->CPAR = tx_data;

	xf_address = rx_data;
    }
    else
    {
        DMA->CMAR = tx_data;
        DMA->CPAR = rx_data;

	xf_address = tx_data;
    }
    
    DMA->CCR = option | DMA_CCR_EN;

    if (xf_address < 0x10000000)
    {
	mask = 1 << (channel & 15);

	armv7m_atomic_or(&stm32wb_dma_device.flash, mask);

	armv7m_atomic_or(&RCC->AHB3SMENR, RCC_AHB3SMENR_FLASHSMEN);
    }
}

__attribute__((optimize("O3"))) uint16_t stm32wb_dma_stop(uint16_t channel)
{
    DMA_Channel_TypeDef *DMA = stm32wb_dma_xlate_DMA[channel & 15];
    stm32wb_dma_t *dma = &stm32wb_dma_device.channels[channel & 15];
    uint32_t mask;

    DMA->CCR &= ~(DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);

    mask = 1 << (channel & 15);

    if (stm32wb_dma_device.flash & mask)
    {
	armv7m_atomic_and(&stm32wb_dma_device.flash, ~mask);

	armv7m_atomic_andz(&RCC->AHB3SMENR, ~RCC_AHB3SMENR_FLASHSMEN, &stm32wb_dma_device.flash);
    }
    
    return dma->size - (DMA->CNDTR & 0xffff);
}

__attribute__((optimize("O3"))) uint16_t stm32wb_dma_count(uint16_t channel)
{
    DMA_Channel_TypeDef *DMA = stm32wb_dma_xlate_DMA[channel & 15];
    stm32wb_dma_t *dma = &stm32wb_dma_device.channels[channel & 15];

    return dma->size - (DMA->CNDTR & 0xffff);
}

__attribute__((optimize("O3"))) bool stm32wb_dma_done(uint16_t channel)
{
    uint32_t shift;

    shift = (channel & 15) << 2;

    if (shift < 28)
    {
	return !!(DMA1->ISR & (DMA_ISR_TCIF1 << shift));
    }
    else
    {
	return !!(DMA2->ISR & (DMA_ISR_TCIF1 << (shift - 28)));
    }
}

__attribute__((optimize("O3"))) void DMA1_Channel1_IRQHandler(void)
{
    stm32wb_dma1_interrupt(&stm32wb_dma_device.channels[0], DMA1_Channel1, DMA1->ISR, 0);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA1_Channel2_IRQHandler(void)
{
    stm32wb_dma1_interrupt(&stm32wb_dma_device.channels[1], DMA1_Channel2, DMA1->ISR, 4);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA1_Channel3_IRQHandler(void)
{
    stm32wb_dma1_interrupt(&stm32wb_dma_device.channels[2], DMA1_Channel3, DMA1->ISR, 8);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA1_Channel4_IRQHandler(void)
{
    stm32wb_dma1_interrupt(&stm32wb_dma_device.channels[3], DMA1_Channel4, DMA1->ISR, 12);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA1_Channel5_IRQHandler(void)
{
    stm32wb_dma1_interrupt(&stm32wb_dma_device.channels[4], DMA1_Channel5, DMA1->ISR, 16);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA1_Channel6_IRQHandler(void)
{
    stm32wb_dma1_interrupt(&stm32wb_dma_device.channels[5], DMA1_Channel6, DMA1->ISR, 20);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA1_Channel7_IRQHandler(void)
{
    stm32wb_dma1_interrupt(&stm32wb_dma_device.channels[6], DMA1_Channel7, DMA1->ISR, 24);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA2_Channel1_IRQHandler(void)
{
    stm32wb_dma2_interrupt(&stm32wb_dma_device.channels[7], DMA2_Channel1, DMA2->ISR, 0);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA2_Channel2_IRQHandler(void)
{
    stm32wb_dma2_interrupt(&stm32wb_dma_device.channels[8], DMA2_Channel2, DMA2->ISR, 4);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA2_Channel3_IRQHandler(void)
{
    stm32wb_dma2_interrupt(&stm32wb_dma_device.channels[9], DMA2_Channel3, DMA2->ISR, 8);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA2_Channel4_IRQHandler(void)
{
    stm32wb_dma2_interrupt(&stm32wb_dma_device.channels[10], DMA2_Channel4, DMA2->ISR, 12);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA2_Channel5_IRQHandler(void)
{
    stm32wb_dma2_interrupt(&stm32wb_dma_device.channels[11], DMA2_Channel5, DMA2->ISR, 16);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA2_Channel6_IRQHandler(void)
{
    stm32wb_dma2_interrupt(&stm32wb_dma_device.channels[12], DMA2_Channel6, DMA2->ISR, 20);

    __DSB();
}

__attribute__((optimize("O3"))) void DMA2_Channel7_IRQHandler(void)
{
    stm32wb_dma2_interrupt(&stm32wb_dma_device.channels[13], DMA2_Channel7, DMA2->ISR, 24);

    __DSB();
}
