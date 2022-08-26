/*
 * Copyright (c) 2016-2021 Thomas Roell.  All rights reserved.
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

#include <stdio.h>

#include "stm32wbxx.h"

#include "armv7m.h"

#include "stm32wb_gpio.h"
#include "stm32wb_sai.h"
#include "stm32wb_dma.h"
#include "stm32wb_system.h"

typedef struct _stm32wb_sai_device_t {
    stm32wb_sai_t * volatile instances[STM32WB_SAI_INSTANCE_COUNT];
} stm32wb_sai_device_t;

static stm32wb_sai_device_t stm32wb_sai_device;

#define STM32WB_SAI_BUSY_START    0
#define STM32WB_SAI_BUSY_RESTART  1
#define STM32WB_SAI_BUSY_CONTINUE 2

#define STM32WB_SAI_DMA_OPTION_RECEIVE_8	  \
    (STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE |	  \
     STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |	  \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_32 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |	  \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |	  \
     STM32WB_DMA_OPTION_PRIORITY_HIGH)

#define STM32WB_SAI_DMA_OPTION_RECEIVE_16	  \
    (STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE |	  \
     STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |	  \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_32 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_16 |	  \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |	  \
     STM32WB_DMA_OPTION_PRIORITY_HIGH)

#define STM32WB_SAI_DMA_OPTION_RECEIVE_32	  \
    (STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE |	  \
     STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |	  \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_32 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_32 |	  \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |	  \
     STM32WB_DMA_OPTION_PRIORITY_HIGH)

#define STM32WB_SAI_DMA_OPTION_TRANSMIT_16	  \
    (STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE |	  \
     STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_32 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_16 |	  \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |	  \
     STM32WB_DMA_OPTION_PRIORITY_HIGH)

#define STM32WB_SAI_DMA_OPTION_TRANSMIT_32	  \
    (STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE |	  \
     STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |	  \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_32 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_32 |	  \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |	  \
     STM32WB_DMA_OPTION_PRIORITY_HIGH)

static void stm32wb_sai_dma_callback(stm32wb_sai_t *sai, uint32_t events);

static __attribute__((optimize("O3"))) void stm32wb_sai_start(stm32wb_sai_t *sai)
{
    SAI_Block_TypeDef *SAIx = sai->SAIx;

    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_SAI1 + sai->instance);

    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);

    stm32wb_dma_enable(sai->dma, sai->priority, (stm32wb_dma_callback_t)stm32wb_sai_dma_callback, sai);
	
    SAIx->CR1 = sai->cr1;
    SAIx->CR2 = sai->cr2 | SAI_xCR2_FFLUSH;
    SAIx->FRCR = sai->frcr;
    SAIx->SLOTR = sai->slotr;
    SAIx->CLRFR = ~0;
    SAIx->IMR = 0;

    if (sai->option & STM32WB_SAI_OPTION_FORMAT_PDM)
    {
	SAI1->PDMCR = SAI_PDMCR_PDMEN | SAI_PDMCR_MICNBR_0 | SAI_PDMCR_CKEN1 | SAI_PDMCR_CKEN2;
    }
    else
    {
	SAI1->PDMCR = 0;
    }
}

static __attribute__((optimize("O3"))) void stm32wb_sai_stop(stm32wb_sai_t *sai)
{
    SAI_Block_TypeDef *SAIx = sai->SAIx;

    armv7m_atomic_storeb(&sai->busy, STM32WB_SAI_BUSY_START);
    
    SAIx->CR1 &= ~SAI_xCR1_SAIEN;

    stm32wb_dma_disable(sai->dma);

    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_SAI1 + sai->instance);

    stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);
    
    sai->state = STM32WB_SAI_STATE_READY;
}

static __attribute__((optimize("O3"))) void stm32wb_sai_dma_callback(stm32wb_sai_t *sai, uint32_t events)
{
    stm32wb_dma_stop(sai->dma);
    
    sai->state = STM32WB_SAI_STATE_READY;

    if ((sai->option & STM32WB_SAI_OPTION_DIRECTION_MASK) == STM32WB_SAI_OPTION_DIRECTION_TRANSMIT)
    {
	(*sai->ev_callback)(sai->ev_context, STM32WB_SAI_EVENT_TRANSMIT);
    }
    else
    {
	(*sai->ev_callback)(sai->ev_context, STM32WB_SAI_EVENT_RECEIVE);

	if (sai->state == STM32WB_SAI_STATE_READY)
	{
	    armv7m_atomic_storeb(&sai->busy, STM32WB_SAI_BUSY_RESTART);
	}
    }
}

bool stm32wb_sai_create(stm32wb_sai_t *sai, const stm32wb_sai_params_t *params)
{
    sai->state = STM32WB_SAI_STATE_INIT;
    sai->instance = params->instance;
    sai->config = params->config;

    if (sai->config & STM32WB_SAI_CONFIG_BLOCK_A)
    {
	sai->SAIx = SAI1_Block_A;
    }
    else
    {
	sai->SAIx = SAI1_Block_B;
    }

    sai->interrupt = SAI1_IRQn;
    sai->priority = params->priority;
    sai->pins = params->pins;

    sai->dma = params->dma;

    return true;
}

bool stm32wb_sai_destroy(stm32wb_sai_t *sai)
{
    if (sai->state != STM32WB_SAI_STATE_INIT)
    {
	return false;
    }

    return true;
}

bool stm32wb_sai_enable(stm32wb_sai_t *sai, uint32_t clock, uint32_t width, uint32_t option, stm32wb_sai_event_callback_t callback, void *context)
{
    uint32_t sai_cr1, sai_cr2, sai_frcr, sai_slotr, saiclk;

    if (sai->state != STM32WB_SAI_STATE_INIT)
    {
	return false;
    }

    if (clock)
    {
	switch (clock) {
	case  44100: saiclk = STM32WB_SYSTEM_SAICLK_11289600; sai_cr1 = ( 1 << SAI_xCR1_MCKDIV_Pos); break;
	case   8000: saiclk = STM32WB_SYSTEM_SAICLK_24576000; sai_cr1 = (12 << SAI_xCR1_MCKDIV_Pos); break;
	case  16000: saiclk = STM32WB_SYSTEM_SAICLK_24576000; sai_cr1 = ( 6 << SAI_xCR1_MCKDIV_Pos); break;
	case  24000: saiclk = STM32WB_SYSTEM_SAICLK_24576000; sai_cr1 = ( 4 << SAI_xCR1_MCKDIV_Pos); break;
	case  32000: saiclk = STM32WB_SYSTEM_SAICLK_24576000; sai_cr1 = ( 3 << SAI_xCR1_MCKDIV_Pos); break;
	case  48000: saiclk = STM32WB_SYSTEM_SAICLK_24576000; sai_cr1 = ( 2 << SAI_xCR1_MCKDIV_Pos); break;
	default:
	    return false;
	}
    }
    else
    {
	if (option & STM32WB_SAI_OPTION_FORMAT_PDM)
	{
	    return false;
	}

	saiclk = STM32WB_SYSTEM_SAICLK_NONE;
	    
	sai_cr1 = SAI_xCR1_MODE_1;
    }
    
    if ((option & STM32WB_SAI_OPTION_DIRECTION_MASK) == STM32WB_SAI_OPTION_DIRECTION_RECEIVE)
    {
	sai_cr1 |= SAI_xCR1_MODE_0;
	sai_cr2 = SAI_xCR2_FTH_0;
    }
    else
    {
	if (option & STM32WB_SAI_OPTION_FORMAT_PDM)
	{
	    return false;
	}

	sai_cr2 = SAI_xCR2_FTH_0 | SAI_xCR2_FTH_1;
    }

    if (option & STM32WB_SAI_OPTION_FORMAT_PDM)
    {
	if (!(sai->config & STM32WB_SAI_CONFIG_BLOCK_A))
	{
	    return false;
	}
	
	if (!(sai->config & (STM32WB_SAI_CONFIG_PDM_DI1 | STM32WB_SAI_CONFIG_PDM_DI2)))
	{
	    return false;
	}

	switch (width) {
	case 8:
	    sai_cr1 |= (SAI_xCR1_DS_1 | SAI_xCR1_NODIV);
	    sai_frcr = ((32 -1) << SAI_xFRCR_FRL_Pos) | (0 << SAI_xFRCR_FSALL_Pos) | SAI_xFRCR_FSPOL;
	    sai_slotr = ((4 -1) << SAI_xSLOTR_NBSLOT_Pos) | (((sai->config & STM32WB_SAI_CONFIG_PDM_DI1) ? 0x0001 : 0x0004) << SAI_xSLOTR_SLOTEN_Pos);
	    break;

	case 16:
	    sai_cr1 |= (SAI_xCR1_DS_2 | SAI_xCR1_NODIV);
	    sai_frcr = ((32 -1) << SAI_xFRCR_FRL_Pos) | (0 << SAI_xFRCR_FSALL_Pos) | SAI_xFRCR_FSPOL;
	    sai_slotr = ((2 -1) << SAI_xSLOTR_NBSLOT_Pos) | (((sai->config & STM32WB_SAI_CONFIG_PDM_DI1) ? 0x0001 : 0x0002) << SAI_xSLOTR_SLOTEN_Pos);
	    break;

	default:
	    return false;
	}
    }
    else
    {
	switch (option & STM32WB_SAI_OPTION_SIZE_MASK) {
	case STM32WB_SAI_OPTION_SIZE_16:
	    if (width != 16)
	    {
		return false;
	    }
	    
	    switch (option & STM32WB_SAI_OPTION_FORMAT_MASK) {
	    case STM32WB_SAI_OPTION_FORMAT_I2S:
		sai_cr1 |= (SAI_xCR1_DS_2 | SAI_xCR1_CKSTR);
		sai_frcr = ((32 -1) << SAI_xFRCR_FRL_Pos) | ((16 -1) << SAI_xFRCR_FSALL_Pos) | SAI_xFRCR_FSDEF | SAI_xFRCR_FSOFF;
		sai_slotr = ((2 -1) << SAI_xSLOTR_NBSLOT_Pos) | (0x0003 << SAI_xSLOTR_SLOTEN_Pos) | SAI_xSLOTR_SLOTSZ_0;
		break;
		
	    case STM32WB_SAI_OPTION_FORMAT_LEFT_JUSTIFIED:
		sai_cr1 |= (SAI_xCR1_DS_2 | SAI_xCR1_CKSTR);
		sai_frcr = ((32 -1) << SAI_xFRCR_FRL_Pos) | ((16 -1) << SAI_xFRCR_FSALL_Pos) | SAI_xFRCR_FSDEF | SAI_xFRCR_FSPOL;
		sai_slotr = ((2 -1) << SAI_xSLOTR_NBSLOT_Pos) | (0x0003 << SAI_xSLOTR_SLOTEN_Pos) | SAI_xSLOTR_SLOTSZ_0;
		break;
		
	    case STM32WB_SAI_OPTION_FORMAT_RIGHT_JUSTIFIED:
		sai_cr1 |= (SAI_xCR1_DS_2 | SAI_xCR1_CKSTR);
		sai_frcr = ((32 -1) << SAI_xFRCR_FRL_Pos) | ((16 -1) << SAI_xFRCR_FSALL_Pos) | SAI_xFRCR_FSDEF | SAI_xFRCR_FSPOL;
		sai_slotr = ((2 -1) << SAI_xSLOTR_NBSLOT_Pos) | (0x0003 << SAI_xSLOTR_SLOTEN_Pos) | SAI_xSLOTR_SLOTSZ_0 | ((16 - width) << SAI_xSLOTR_FBOFF_Pos);
		break;
		
	    default:
		return false;
	    }
	    break;

	case STM32WB_SAI_OPTION_SIZE_32:
	    switch (width) {
	    case 16: sai_cr1 |= (SAI_xCR1_DS_2);         	                 break;
	    case 24: sai_cr1 |= (SAI_xCR1_DS_2 | SAI_xCR1_DS_1);                 break;
	    case 32: sai_cr1 |= (SAI_xCR1_DS_2 | SAI_xCR1_DS_1 | SAI_xCR1_DS_0); break;
	    default:
		return false;
	    }
	
	    switch (option & STM32WB_SAI_OPTION_FORMAT_MASK) {
	    case STM32WB_SAI_OPTION_FORMAT_I2S:
		sai_cr1 |= SAI_xCR1_CKSTR;
		sai_frcr = ((64 -1) << SAI_xFRCR_FRL_Pos) | ((32 -1) << SAI_xFRCR_FSALL_Pos) | SAI_xFRCR_FSDEF | SAI_xFRCR_FSOFF;
		sai_slotr = ((2 -1) << SAI_xSLOTR_NBSLOT_Pos) | (0x0003 << SAI_xSLOTR_SLOTEN_Pos) | SAI_xSLOTR_SLOTSZ_1;
		break;
		
	    case STM32WB_SAI_OPTION_FORMAT_LEFT_JUSTIFIED:
		sai_cr1 |= SAI_xCR1_CKSTR;
		sai_frcr = ((64 -1) << SAI_xFRCR_FRL_Pos) | ((32 -1) << SAI_xFRCR_FSALL_Pos) | SAI_xFRCR_FSDEF | SAI_xFRCR_FSPOL;
		sai_slotr = ((2 -1) << SAI_xSLOTR_NBSLOT_Pos) | (0x0003 << SAI_xSLOTR_SLOTEN_Pos) | SAI_xSLOTR_SLOTSZ_1;
		break;
		
	    case STM32WB_SAI_OPTION_FORMAT_RIGHT_JUSTIFIED:
		sai_cr1 |= SAI_xCR1_CKSTR;
		sai_frcr = ((64 -1) << SAI_xFRCR_FRL_Pos) | ((32 -1) << SAI_xFRCR_FSALL_Pos) | SAI_xFRCR_FSDEF | SAI_xFRCR_FSPOL;
		sai_slotr = ((2 -1) << SAI_xSLOTR_NBSLOT_Pos) | (0x0003 << SAI_xSLOTR_SLOTEN_Pos) | SAI_xSLOTR_SLOTSZ_1 | ((32 - width) << SAI_xSLOTR_FBOFF_Pos);
		break;
		
	    default:
		return false;
	    }
	    break;

	default:
	    return false;
	}
    }
    
    if (armv7m_atomic_cas((volatile uint32_t*)&stm32wb_sai_device.instances[sai->instance], (uint32_t)NULL, (uint32_t)sai) != (uint32_t)NULL)
    {
	return false;
    }
    
    if (!stm32wb_system_saiclk_configure(saiclk))
    {
        armv7m_atomic_store((volatile uint32_t*)&stm32wb_sai_device.instances[sai->instance], (uint32_t)NULL);
	
	return false;
    }
    
    sai->width  = width;
    sai->option = option;
    
    sai->cr1 = sai_cr1 | SAI_xCR1_DMAEN;
    sai->cr2 = sai_cr2;
    sai->frcr = sai_frcr;
    sai->slotr = sai_slotr;

    sai->ev_callback = callback;
    sai->ev_context = context;

    stm32wb_gpio_pin_configure(sai->pins.sck, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));

    stm32wb_gpio_pin_configure(sai->pins.sd, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));

    if (sai->pins.fs != STM32WB_GPIO_PIN_NONE)
    {
	stm32wb_gpio_pin_configure(sai->pins.fs, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    }

    if (sai->pins.mck != STM32WB_GPIO_PIN_NONE)
    {
	stm32wb_gpio_pin_configure(sai->pins.mck, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    }

    armv7m_atomic_storeb(&sai->busy, STM32WB_SAI_BUSY_START);
    
    sai->state = STM32WB_SAI_STATE_READY;

    return true;
}

bool stm32wb_sai_disable(stm32wb_sai_t *sai)
{
    if (sai->state < STM32WB_SAI_STATE_READY)
    {
	return false;
    }
    
    stm32wb_sai_stop(sai);
    
    stm32wb_system_saiclk_configure(STM32WB_SYSTEM_SAICLK_NONE);
    
    stm32wb_gpio_pin_configure(sai->pins.sck, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_MODE_ANALOG));

    stm32wb_gpio_pin_configure(sai->pins.sd, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_MODE_ANALOG));

    if (sai->pins.fs != STM32WB_GPIO_PIN_NONE)
    {
	stm32wb_gpio_pin_configure(sai->pins.fs, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_MODE_ANALOG));
    }
    
    if (sai->pins.mck != STM32WB_GPIO_PIN_NONE)
    {
	stm32wb_gpio_pin_configure(sai->pins.mck, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_MODE_ANALOG));
    }

    armv7m_atomic_store((volatile uint32_t*)&stm32wb_sai_device.instances[sai->instance], (uint32_t)NULL);
    
    sai->state = STM32WB_SAI_STATE_INIT;

    return true;
}

bool stm32wb_sai_transmit(stm32wb_sai_t *sai, const uint8_t *tx_data, uint16_t tx_count)
{
    SAI_Block_TypeDef *SAIx = sai->SAIx;
    uint32_t dma_option;
    uint8_t busy;
    
    if ((sai->option & STM32WB_SAI_OPTION_DIRECTION_MASK) != STM32WB_SAI_OPTION_DIRECTION_TRANSMIT)
    {
	return false;
    }

    if (sai->state != STM32WB_SAI_STATE_READY)
    {
	return false;
    }
    
    sai->state = STM32WB_SAI_STATE_TRANSMIT;

    if (sai->width > 16)
    {
	dma_option = STM32WB_SAI_DMA_OPTION_TRANSMIT_32;
	tx_count = tx_count / 4;
    }
    else
    {
	dma_option = STM32WB_SAI_DMA_OPTION_TRANSMIT_16;
	tx_count = tx_count / 2;
    }
    
    busy = armv7m_atomic_swapb(&sai->busy, STM32WB_SAI_BUSY_CONTINUE);

    if (busy == STM32WB_SAI_BUSY_CONTINUE)
    {
	stm32wb_dma_start(sai->dma, (uint32_t)&SAIx->DR, (uint32_t)tx_data, tx_count, dma_option);
    }
    else
    {
	stm32wb_sai_start(sai);

	stm32wb_dma_start(sai->dma, (uint32_t)&SAIx->DR, (uint32_t)tx_data, tx_count, dma_option);

	SAIx->CR1 |= SAI_xCR1_SAIEN;
    }
    
    return true;
}

bool stm32wb_sai_receive(stm32wb_sai_t *sai, uint8_t *rx_data, uint16_t rx_count)
{
    SAI_Block_TypeDef *SAIx = sai->SAIx;
    uint32_t dma_option;
    uint8_t busy;

    if ((sai->option & STM32WB_SAI_OPTION_DIRECTION_MASK) != STM32WB_SAI_OPTION_DIRECTION_RECEIVE)
    {
	return false;
    }

    if (sai->state != STM32WB_SAI_STATE_READY)
    {
	return false;
    }

    sai->state = STM32WB_SAI_STATE_RECEIVE;

    if (sai->width > 16)
    {
	dma_option = STM32WB_SAI_DMA_OPTION_RECEIVE_32;
	rx_count = rx_count / 4;
    }
    else
    {
	if (sai->width > 8)
	{
	    dma_option = STM32WB_SAI_DMA_OPTION_RECEIVE_16;
	    rx_count = rx_count / 2;
	}
	else
	{
	    dma_option = STM32WB_SAI_DMA_OPTION_RECEIVE_8;
	}
    }
    
    busy = armv7m_atomic_swapb(&sai->busy, STM32WB_SAI_BUSY_CONTINUE);

    if (busy == STM32WB_SAI_BUSY_CONTINUE)
    {
	stm32wb_dma_start(sai->dma, (uint32_t)rx_data, (uint32_t)&SAIx->DR, rx_count, dma_option);
    }
    else
    {
	if (busy == STM32WB_SAI_BUSY_RESTART)
	{
	    /* The FLUSH operation does not work, so simply pop off all data before starting
	     * the DMA again.
	     */
	    SAIx->DR;
  	    SAIx->DR;
  	    SAIx->DR;
  	    SAIx->DR;
  	    SAIx->DR;
  	    SAIx->DR;
  	    SAIx->DR;
  	    SAIx->DR;

	    stm32wb_dma_start(sai->dma, (uint32_t)rx_data, (uint32_t)&SAIx->DR, rx_count, dma_option);
	}
	else
	{
	    stm32wb_sai_start(sai);
	    
	    stm32wb_dma_start(sai->dma, (uint32_t)rx_data, (uint32_t)&SAIx->DR, rx_count, dma_option);

	    SAIx->CR1 |= SAI_xCR1_SAIEN;
	}
    }

    return true;
}
