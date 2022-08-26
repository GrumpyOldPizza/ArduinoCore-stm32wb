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
#include "stm32wb_spi.h"
#include "stm32wb_dma.h"
#include "stm32wb_exti.h"
#include "stm32wb_system.h"

typedef struct _stm32wb_spi_device_t {
    stm32wb_spi_t * volatile instances[STM32WB_SPI_INSTANCE_COUNT];
    stm32wb_system_notify_t notify;
} stm32wb_spi_device_t;

static stm32wb_spi_device_t stm32wb_spi_device;

static __attribute__((section(".dma"))) uint16_t stm32wb_spi_dma_rx_none;
static __attribute__((section(".dma"))) uint16_t stm32wb_spi_dma_tx_default;

#define SPI_CR1_BR_DIV2   (0)
#define SPI_CR1_BR_DIV4   (SPI_CR1_BR_0)
#define SPI_CR1_BR_DIV8   (SPI_CR1_BR_1)
#define SPI_CR1_BR_DIV16  (SPI_CR1_BR_0 | SPI_CR1_BR_1)
#define SPI_CR1_BR_DIV32  (SPI_CR1_BR_2)
#define SPI_CR1_BR_DIV64  (SPI_CR1_BR_0 | SPI_CR1_BR_2)
#define SPI_CR1_BR_DIV128 (SPI_CR1_BR_1 | SPI_CR1_BR_2)
#define SPI_CR1_BR_DIV256 (SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2)

#define SPI_CR2_DS_8BIT   (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2)
#define SPI_CR2_DS_16BIT  (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_DS_3)

#define STM32WB_SPI_OPTION_RX_DMA 0x40
#define STM32WB_SPI_OPTION_TX_DMA 0x80


#define STM32WB_SPI_RX_DMA_OPTION_RECEIVE_8       \
    (STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 |  \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_SPI_RX_DMA_OPTION_TRANSMIT_8      \
    (STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 |  \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_SPI_RX_DMA_OPTION_TRANSFER_8      \
    (STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 |  \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_SPI_RX_DMA_OPTION_RECEIVE_16      \
    (STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_16 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_16 |     \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_SPI_RX_DMA_OPTION_TRANSMIT_16     \
    (STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_16 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_16 |     \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_SPI_RX_DMA_OPTION_TRANSFER_16     \
    (STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_16 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_16 |     \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_SPI_TX_DMA_OPTION_RECEIVE_8       \
    (STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 |  \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_SPI_TX_DMA_OPTION_TRANSMIT_8      \
    (STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 |  \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_SPI_TX_DMA_OPTION_TRANSFER_8      \
    (STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 |  \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_SPI_TX_DMA_OPTION_RECEIVE_16      \
    (STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_16 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_16 |     \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_SPI_TX_DMA_OPTION_TRANSMIT_16     \
    (STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_16 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_16 |     \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_SPI_TX_DMA_OPTION_TRANSFER_16     \
    (STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_16 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_16 |     \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

static SPI_TypeDef * const stm32wb_i2c_xlate_SPI[STM32WB_SPI_INSTANCE_COUNT] = {
    SPI1,
    SPI2,
};

static void stm32wb_spi_notify_callback(void *context, uint32_t notify)
{
    if (stm32wb_spi_device.instances[STM32WB_SPI_INSTANCE_SPI1])
    {
        stm32wb_spi_device.instances[STM32WB_SPI_INSTANCE_SPI1]->clock = 0;
    }

    if (stm32wb_spi_device.instances[STM32WB_SPI_INSTANCE_SPI2])
    {
        stm32wb_spi_device.instances[STM32WB_SPI_INSTANCE_SPI2]->clock = 0;
    }
}

static void stm32wb_spi_dma_callback(stm32wb_spi_t *spi, uint32_t events)
{
    SPI_TypeDef *SPI = spi->SPI;
    stm32wb_spi_done_callback_t callback;
    void *context;
    volatile uint8_t *p_status_return;

    p_status_return = spi->xf_status;

    if (!p_status_return)
    {
        return;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&spi->xf_status, (uint32_t)p_status_return, (uint32_t)NULL) != (uint32_t)p_status_return)
    {
        return;
    }

    stm32wb_dma_stop(spi->tx_dma);
    stm32wb_dma_stop(spi->rx_dma);

    while (SPI->SR & SPI_SR_BSY)
    {
    }
    
    SPI->CR1 = spi->cr1;
    SPI->CR2 = spi->cr2;
    SPI->CR1 = spi->cr1 | SPI_CR1_SPE;

    callback = spi->xf_callback;
    context = spi->xf_context;

    spi->state = STM32WB_SPI_STATE_DATA;
    
    *p_status_return = STM32WB_SPI_STATUS_SUCCESS;

    if (callback)
    {
        (*callback)(context);
    }
}

bool stm32wb_spi_create(stm32wb_spi_t *spi, const stm32wb_spi_params_t *params)
{
    if (spi->state != STM32WB_SPI_STATE_NONE)
    {
        return false;
    }

    spi->SPI = stm32wb_i2c_xlate_SPI[params->instance];
    spi->instance = params->instance;
    spi->priority = params->priority;
    spi->rx_dma = params->rx_dma;
    spi->tx_dma = params->tx_dma;
    spi->pins = params->pins;

    stm32wb_spi_dma_tx_default = 0xffff;

    spi->state = STM32WB_SPI_STATE_INIT;

    if (!stm32wb_spi_device.notify.callback)
    {
        stm32wb_system_register(&stm32wb_spi_device.notify, stm32wb_spi_notify_callback, NULL, (STM32WB_SYSTEM_NOTIFY_CLOCKS));
    }
    
    return true;
}

bool stm32wb_spi_destroy(stm32wb_spi_t *spi)
{
    if (spi->state != STM32WB_SPI_STATE_INIT)
    {
        return false;
    }

    spi->state = STM32WB_SPI_STATE_NONE;

    return true;
}

bool stm32wb_spi_enable(stm32wb_spi_t *spi)
{
    if (spi->state != STM32WB_SPI_STATE_INIT)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&stm32wb_spi_device.instances[spi->instance], (uint32_t)NULL, (uint32_t)spi) != (uint32_t)NULL)
    {
        return false;
    }
    
    spi->clock = ~0;
    spi->control = 0;
    spi->mask = 0;
    spi->cr1 = 0;
    spi->cr2 = 0;

    armv7m_atomic_store((volatile uint32_t*)&spi->xf_status, (uint32_t)NULL);

#if 1    
    stm32wb_gpio_pin_configure(spi->pins.mosi, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    stm32wb_gpio_pin_configure(spi->pins.miso, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    stm32wb_gpio_pin_configure(spi->pins.sck,  (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
#endif

#if 0    
    stm32wb_gpio_pin_configure(spi->pins.mosi, (STM32WB_GPIO_PARK_PULLDOWN | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    stm32wb_gpio_pin_configure(spi->pins.miso, (STM32WB_GPIO_PARK_PULLDOWN | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    stm32wb_gpio_pin_configure(spi->pins.sck,  (STM32WB_GPIO_PARK_PULLDOWN | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
#endif
    
    spi->state = STM32WB_SPI_STATE_READY;

    return true;
}

bool stm32wb_spi_disable(stm32wb_spi_t *spi)
{
    if (spi->state != STM32WB_SPI_STATE_READY)
    {
        return false;
    }

    stm32wb_gpio_pin_configure(spi->pins.mosi, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_MODE_ANALOG));
    stm32wb_gpio_pin_configure(spi->pins.miso, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_MODE_ANALOG));
    stm32wb_gpio_pin_configure(spi->pins.sck,  (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_MODE_ANALOG));
    
    armv7m_atomic_store((volatile uint32_t*)&stm32wb_spi_device.instances[spi->instance], (uint32_t)NULL);
        
    spi->state = STM32WB_SPI_STATE_INIT;

    return true;
}

bool stm32wb_spi_block(stm32wb_spi_t *spi, uint16_t pin)
{
    if (spi->state != STM32WB_SPI_STATE_READY)
    {
        return false;
    }

    spi->mask |= (1ul << ((pin & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT));

    return true;
}

bool stm32wb_spi_unblock(stm32wb_spi_t *spi, uint16_t pin)
{
    if (spi->state != STM32WB_SPI_STATE_READY)
    {
        return false;
    }

    spi->mask &= ~(1ul << ((pin & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT));

    return true;
}

__attribute__((optimize("O3"))) bool stm32wb_spi_acquire(stm32wb_spi_t *spi, uint32_t clock, uint32_t control)
{
    SPI_TypeDef *SPI = spi->SPI;
    uint32_t spiclk, spidiv;

    if (spi->state != STM32WB_SPI_STATE_READY)
    {
        return false;
    }

    if (spi->mask)
    {
        stm32wb_exti_block(spi->mask);
    }

    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
    stm32wb_system_reference(STM32WB_SYSTEM_REFERENCE_SPI1 << spi->instance);

    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_SPI1 + spi->instance);

    if ((spi->clock != clock) || (spi->control != control))
    {
        if (spi->instance == STM32WB_SPI_INSTANCE_SPI1)
        {
            spiclk = stm32wb_system_pclk2() / 2;
        }
        else
        {
            spiclk = stm32wb_system_pclk1() / 2;
        }

        spi->cr1 = SPI_CR1_SSM | SPI_CR1_SSI | (control & (STM32WB_SPI_CONTROL_CPHA | STM32WB_SPI_CONTROL_CPOL | STM32WB_SPI_CONTROL_LSB_FIRST));
        spi->cr2 = SPI_CR2_DS_8BIT | SPI_CR2_FRXTH;
        
        if (clock)
        {
            spidiv = 0;

            while ((spiclk > clock) && (spidiv < 7))
            {
                spiclk >>= 1;
                spidiv++;
            }
            
            spi->cr1 |= (SPI_CR1_MSTR | (spidiv << SPI_CR1_BR_Pos));
        }
        
        SPI->SR = 0;
        SPI->CR1 = spi->cr1;
        SPI->CR2 = spi->cr2;

        spi->clock = clock;
        spi->control = control;
    }

    SPI->CR1 = spi->cr1 | SPI_CR1_SPE;

    spi->state = STM32WB_SPI_STATE_DATA;

    return true;
}

__attribute__((optimize("O3"))) bool stm32wb_spi_release(stm32wb_spi_t *spi)
{
    SPI_TypeDef *SPI = spi->SPI;

    if (spi->state != STM32WB_SPI_STATE_DATA)
    {
        return false;
    }

    while (SPI->SR & SPI_SR_BSY)
    {
    }

    SPI->CR1 = spi->cr1;

    if (spi->option & (STM32WB_SPI_OPTION_RX_DMA | STM32WB_SPI_OPTION_TX_DMA))
    {
        if (spi->option & STM32WB_SPI_OPTION_RX_DMA)
        {
            stm32wb_dma_disable(spi->rx_dma);
        }

        if (spi->option & STM32WB_SPI_OPTION_TX_DMA)
        {
            stm32wb_dma_disable(spi->tx_dma);
        }
        
        spi->option &= ~(STM32WB_SPI_OPTION_RX_DMA | STM32WB_SPI_OPTION_TX_DMA);
    }

    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_SPI1 + spi->instance);

    stm32wb_system_unreference(STM32WB_SYSTEM_REFERENCE_SPI1 << spi->instance);
    stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);

    spi->state = STM32WB_SPI_STATE_READY;

    if (spi->mask)
    {
        stm32wb_exti_unblock(spi->mask, false);
    }

    return true;
}

__attribute__((optimize("O3"))) uint8_t stm32wb_spi_data(stm32wb_spi_t *spi, uint8_t data)
{
    SPI_TypeDef *SPI = spi->SPI;

    STM32WB_SPI_WRITE_8(SPI, data);

    while (!(SPI->SR & SPI_SR_RXNE))
    {
    }

    return STM32WB_SPI_READ_8(SPI);
}

__attribute__((optimize("O3"))) uint16_t stm32wb_spi_data16le(stm32wb_spi_t *spi, uint16_t data)
{
    SPI_TypeDef *SPI = spi->SPI;

    STM32WB_SPI_WRITE_8(SPI, data >> 0);
    STM32WB_SPI_WRITE_8(SPI, data >> 8);
    
    while (!(SPI->SR & SPI_SR_RXNE))
    {
    }

    data = (STM32WB_SPI_READ_8(SPI) << 0);

    while (!(SPI->SR & SPI_SR_RXNE))
    {
    }

    data |= (STM32WB_SPI_READ_8(SPI) << 8);

    return data;
}

__attribute__((optimize("O3"))) uint16_t stm32wb_spi_data16be(stm32wb_spi_t *spi, uint16_t data)
{
    SPI_TypeDef *SPI = spi->SPI;

    STM32WB_SPI_WRITE_8(SPI, data >> 8);
    STM32WB_SPI_WRITE_8(SPI, data >> 0);
    
    while (!(SPI->SR & SPI_SR_RXNE))
    {
    }

    data = (STM32WB_SPI_READ_8(SPI) << 8);

    while (!(SPI->SR & SPI_SR_RXNE))
    {
    }

    data |= (STM32WB_SPI_READ_8(SPI) << 0);

    return data;
}

__attribute__((optimize("O3"))) void stm32wb_spi_data_receive(stm32wb_spi_t *spi, uint8_t *rx_data, uint32_t rx_count)
{
    SPI_TypeDef *SPI = spi->SPI;
    uint8_t *rx_data_e;
    const uint8_t tx_default = 0xff;
    
    if (rx_count < 4)
    {
        if (rx_count == 3)
        {
            STM32WB_SPI_WRITE_8(SPI, tx_default);
            STM32WB_SPI_WRITE_8(SPI, tx_default);
            STM32WB_SPI_WRITE_8(SPI, tx_default);
            
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }
            
            rx_data[0] = STM32WB_SPI_READ_8(SPI);
            
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }
            
            rx_data[1] = STM32WB_SPI_READ_8(SPI);

            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }
            
            rx_data[2] = STM32WB_SPI_READ_8(SPI);
        }
        else if (rx_count == 2)
        {
            STM32WB_SPI_WRITE_8(SPI, tx_default);
            STM32WB_SPI_WRITE_8(SPI, tx_default);
            
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }
            
            rx_data[0] = STM32WB_SPI_READ_8(SPI);
            
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }
            
            rx_data[1] = STM32WB_SPI_READ_8(SPI);
        }
        else
        {
            STM32WB_SPI_WRITE_8(SPI, tx_default);
            
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }
            
            rx_data[0] = STM32WB_SPI_READ_8(SPI);
        }
    }
    else
    {
        rx_data_e = rx_data + rx_count - 4;
        
        STM32WB_SPI_WRITE_8(SPI, tx_default);
        STM32WB_SPI_WRITE_8(SPI, tx_default);
        STM32WB_SPI_WRITE_8(SPI, tx_default);
        STM32WB_SPI_WRITE_8(SPI, tx_default);
        
        while (rx_data != rx_data_e)
        {
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }
            
            *rx_data++ = STM32WB_SPI_READ_8(SPI);
            
            STM32WB_SPI_WRITE_8(SPI, tx_default);
        }
        
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }
        
        rx_data[0] = STM32WB_SPI_READ_8(SPI);
        
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }
        
        rx_data[1] = STM32WB_SPI_READ_8(SPI);
        
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }
        
        rx_data[2] = STM32WB_SPI_READ_8(SPI);

        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }
        
        rx_data[3] = STM32WB_SPI_READ_8(SPI);
    }
}

__attribute__((optimize("O3"))) void stm32wb_spi_data_transmit(stm32wb_spi_t *spi, const uint8_t *tx_data, uint32_t tx_count)
{
    SPI_TypeDef *SPI = spi->SPI;
    const uint8_t *tx_data_e;

    if (tx_count < 4)
    {
        if (tx_count == 3)
        {
            STM32WB_SPI_WRITE_8(SPI, tx_data[0]);
            STM32WB_SPI_WRITE_8(SPI, tx_data[1]);
            STM32WB_SPI_WRITE_8(SPI, tx_data[2]);
                    
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            STM32WB_SPI_READ_8(SPI);
                    
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            STM32WB_SPI_READ_8(SPI);
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            STM32WB_SPI_READ_8(SPI);
        }
        else if (tx_count == 2)
        {
            STM32WB_SPI_WRITE_8(SPI, tx_data[0]);
            STM32WB_SPI_WRITE_8(SPI, tx_data[1]);
                    
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            STM32WB_SPI_READ_8(SPI);
                    
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            STM32WB_SPI_READ_8(SPI);
        }
        else
        {
            STM32WB_SPI_WRITE_8(SPI, tx_data[0]);
                    
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            STM32WB_SPI_READ_8(SPI);
        }
    }
    else
    {
        tx_data_e = tx_data + tx_count;
        
        STM32WB_SPI_WRITE_8(SPI, tx_data[0]);
        STM32WB_SPI_WRITE_8(SPI, tx_data[1]);
        STM32WB_SPI_WRITE_8(SPI, tx_data[2]);
        STM32WB_SPI_WRITE_8(SPI, tx_data[3]);
        tx_data += 4;
                
        while (tx_data != tx_data_e)
        {
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            STM32WB_SPI_READ_8(SPI);
            STM32WB_SPI_WRITE_8(SPI, *tx_data++);
        }
                
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        STM32WB_SPI_READ_8(SPI);
                
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        STM32WB_SPI_READ_8(SPI);
                
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        STM32WB_SPI_READ_8(SPI);
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        STM32WB_SPI_READ_8(SPI);
    }
}

__attribute__((optimize("O3"))) void stm32wb_spi_data_transfer(stm32wb_spi_t *spi, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count)
{
    SPI_TypeDef *SPI = spi->SPI;
    const uint8_t *tx_data_e;

    if (xf_count < 4)
    {
        if (xf_count == 3)
        {
            STM32WB_SPI_WRITE_8(SPI, tx_data[0]);
            STM32WB_SPI_WRITE_8(SPI, tx_data[1]);
            STM32WB_SPI_WRITE_8(SPI, tx_data[2]);
                    
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            rx_data[0] = STM32WB_SPI_READ_8(SPI);
                    
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            rx_data[1] = STM32WB_SPI_READ_8(SPI);

            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            rx_data[2] = STM32WB_SPI_READ_8(SPI);
        }
        else if (xf_count == 2)
        {
            STM32WB_SPI_WRITE_8(SPI, tx_data[0]);
            STM32WB_SPI_WRITE_8(SPI, tx_data[1]);
                    
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            rx_data[0] = STM32WB_SPI_READ_8(SPI);
                    
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            rx_data[1] = STM32WB_SPI_READ_8(SPI);
        }
        else
        {
            STM32WB_SPI_WRITE_8(SPI, tx_data[0]);
                    
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            rx_data[0] = STM32WB_SPI_READ_8(SPI);
        }
    }
    else
    {
        tx_data_e = tx_data + xf_count;
        
        STM32WB_SPI_WRITE_8(SPI, tx_data[0]);
        STM32WB_SPI_WRITE_8(SPI, tx_data[1]);
        STM32WB_SPI_WRITE_8(SPI, tx_data[2]);
        STM32WB_SPI_WRITE_8(SPI, tx_data[3]);
        tx_data += 4;

        while (tx_data != tx_data_e)
        {
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }

            *rx_data++ = STM32WB_SPI_READ_8(SPI);
            STM32WB_SPI_WRITE_8(SPI, *tx_data++);
        }
                
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        rx_data[0] = STM32WB_SPI_READ_8(SPI);
                
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        rx_data[1] = STM32WB_SPI_READ_8(SPI);
                
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        rx_data[2] = STM32WB_SPI_READ_8(SPI);

        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        rx_data[3] = STM32WB_SPI_READ_8(SPI);
    }
}


__attribute__((optimize("O3"))) bool stm32wb_spi_data_dma_receive(stm32wb_spi_t *spi, uint8_t *rx_data, uint32_t rx_count, volatile uint8_t *p_status_return, stm32wb_spi_done_callback_t callback, void *context)
{
    SPI_TypeDef *SPI = spi->SPI;
    uint32_t spi_cr2, rx_option, tx_option;

    if (spi->state != STM32WB_SPI_STATE_DATA)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&spi->xf_status, (uint32_t)NULL, (uint32_t)p_status_return) != (uint32_t)NULL)
    {
        return false;
    }

    if (!(spi->option & STM32WB_SPI_OPTION_RX_DMA))
    {
        if (!stm32wb_dma_enable(spi->rx_dma, spi->priority, (stm32wb_dma_callback_t)stm32wb_spi_dma_callback, spi))
        {
            armv7m_atomic_store((volatile uint32_t*)&spi->xf_status, (uint32_t)NULL);
            
            return false;
        }
        
        spi->option |= STM32WB_SPI_OPTION_RX_DMA;
    }
    
    if (!(spi->option & STM32WB_SPI_OPTION_TX_DMA))
    {
        if (!stm32wb_dma_enable(spi->tx_dma, spi->priority, NULL, NULL))
        {
            armv7m_atomic_store((volatile uint32_t*)&spi->xf_status, (uint32_t)NULL);
            
            return false;
        }
        
        spi->option |= STM32WB_SPI_OPTION_TX_DMA;
    }

    spi->state = STM32WB_SPI_STATE_DATA_DMA;
    
    *p_status_return = STM32WB_SPI_STATUS_BUSY;
    
    spi->xf_status = p_status_return;
    spi->xf_callback = callback;
    spi->xf_context = context;
    spi->rx_data = rx_data;

    if (!spi->clock || ((uint32_t)rx_data & 1) || (rx_count & 1))
    {
        spi_cr2 = spi->cr2 | (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
                
        rx_option = STM32WB_SPI_RX_DMA_OPTION_RECEIVE_8;
        tx_option = STM32WB_SPI_TX_DMA_OPTION_RECEIVE_8;
    }
    else
    {
        spi_cr2 = (spi->cr2 & ~SPI_CR2_FRXTH) | (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

        rx_option = STM32WB_SPI_RX_DMA_OPTION_RECEIVE_16;
        tx_option = STM32WB_SPI_TX_DMA_OPTION_RECEIVE_16;

        rx_count = rx_count / 2;
    }

    while (SPI->SR & (SPI_SR_FTLVL | SPI_SR_FRLVL | SPI_SR_BSY))
    {
    }
    
    SPI->CR1 = spi->cr1;
    SPI->CR2 = spi_cr2;
    SPI->CR1 = spi->cr1 | SPI_CR1_SPE;

    stm32wb_dma_start(spi->rx_dma, (uint32_t)rx_data, (uint32_t)&SPI->DR, rx_count, rx_option | STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE);
    stm32wb_dma_start(spi->tx_dma, (uint32_t)&SPI->DR, (uint32_t)&stm32wb_spi_dma_tx_default, rx_count, tx_option);
        
    return true;
}

__attribute__((optimize("O3"))) bool stm32wb_spi_data_dma_transmit(stm32wb_spi_t *spi, const uint8_t *tx_data, uint32_t tx_count, volatile uint8_t *p_status_return, stm32wb_spi_done_callback_t callback, void *context)
{
    SPI_TypeDef *SPI = spi->SPI;
    uint32_t spi_cr2, rx_option, tx_option;
    
    if (spi->state != STM32WB_SPI_STATE_DATA)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&spi->xf_status, (uint32_t)NULL, (uint32_t)p_status_return) != (uint32_t)NULL)
    {
        return false;
    }

    if (!(spi->option & STM32WB_SPI_OPTION_RX_DMA))
    {
        if (!stm32wb_dma_enable(spi->rx_dma, spi->priority, (stm32wb_dma_callback_t)stm32wb_spi_dma_callback, spi))
        {
            armv7m_atomic_store((volatile uint32_t*)&spi->xf_status, (uint32_t)NULL);
            
            return false;
        }
        
        spi->option |= STM32WB_SPI_OPTION_RX_DMA;
    }
    
    if (!(spi->option & STM32WB_SPI_OPTION_TX_DMA))
    {
        if (!stm32wb_dma_enable(spi->tx_dma, spi->priority, NULL, NULL))
        {
            armv7m_atomic_store((volatile uint32_t*)&spi->xf_status, (uint32_t)NULL);
            
            return false;
        }
        
        spi->option |= STM32WB_SPI_OPTION_TX_DMA;
    }

    spi->state = STM32WB_SPI_STATE_DATA_DMA;
    
    *p_status_return = STM32WB_SPI_STATUS_BUSY;

    spi->xf_status = p_status_return;
    spi->xf_callback = callback;
    spi->xf_context = context;
    spi->rx_data = NULL;

    if (!spi->clock || ((uint32_t)tx_data & 1) || (tx_count & 1))
    {
        spi_cr2 = spi->cr2 | (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
                
        rx_option = STM32WB_SPI_RX_DMA_OPTION_TRANSMIT_8;
        tx_option = STM32WB_SPI_TX_DMA_OPTION_TRANSMIT_8;
    }
    else
    {
        spi_cr2 = (spi->cr2 & ~SPI_CR2_FRXTH) | (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

        rx_option = STM32WB_SPI_RX_DMA_OPTION_TRANSMIT_16;
        tx_option = STM32WB_SPI_TX_DMA_OPTION_TRANSMIT_16;

        tx_count = tx_count / 2;
    }
    
    while (SPI->SR & (SPI_SR_FTLVL | SPI_SR_FRLVL | SPI_SR_BSY))
    {
    }

    SPI->CR1 = spi->cr1;
    SPI->CR2 = spi_cr2;
    SPI->CR1 = spi->cr1 | SPI_CR1_SPE;
                
    stm32wb_dma_start(spi->rx_dma, (uint32_t)&stm32wb_spi_dma_rx_none, (uint32_t)&SPI->DR, tx_count, rx_option | STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE);
    stm32wb_dma_start(spi->tx_dma, (uint32_t)&SPI->DR, (uint32_t)tx_data, tx_count, tx_option);

    return true;
}

__attribute__((optimize("O3"))) bool stm32wb_spi_data_dma_transfer(stm32wb_spi_t *spi, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count, volatile uint8_t *p_status_return, stm32wb_spi_done_callback_t callback, void *context)
{
    SPI_TypeDef *SPI = spi->SPI;
    uint32_t spi_cr2, rx_option, tx_option;
    
    if (spi->state != STM32WB_SPI_STATE_DATA)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&spi->xf_status, (uint32_t)NULL, (uint32_t)p_status_return) != (uint32_t)NULL)
    {
        return false;
    }

    if (!(spi->option & STM32WB_SPI_OPTION_RX_DMA))
    {
        if (!stm32wb_dma_enable(spi->rx_dma, spi->priority, (stm32wb_dma_callback_t)stm32wb_spi_dma_callback, spi))
        {
            armv7m_atomic_store((volatile uint32_t*)&spi->xf_status, (uint32_t)NULL);
            
            return false;
        }
        
        spi->option |= STM32WB_SPI_OPTION_RX_DMA;
    }
    
    if (!(spi->option & STM32WB_SPI_OPTION_TX_DMA))
    {
        if (!stm32wb_dma_enable(spi->tx_dma, spi->priority, NULL, NULL))
        {
            armv7m_atomic_store((volatile uint32_t*)&spi->xf_status, (uint32_t)NULL);
            
            return false;
        }
        
        spi->option |= STM32WB_SPI_OPTION_TX_DMA;
    }

    spi->state = STM32WB_SPI_STATE_DATA_DMA;
    
    *p_status_return = STM32WB_SPI_STATUS_BUSY;

    spi->xf_status = p_status_return;
    spi->xf_callback = callback;
    spi->xf_context = context;
    spi->rx_data = rx_data;

    if (!spi->clock || ((uint32_t)tx_data & 1) || ((uint32_t)rx_data & 1) || (xf_count & 1))
    {
        spi_cr2 = spi->cr2 | (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
                
        rx_option = STM32WB_SPI_RX_DMA_OPTION_TRANSFER_8;
        tx_option = STM32WB_SPI_TX_DMA_OPTION_TRANSFER_8;
    }
    else
    {
        spi_cr2 = (spi->cr2 & ~SPI_CR2_FRXTH) | (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

        rx_option = STM32WB_SPI_RX_DMA_OPTION_TRANSFER_16;
        tx_option = STM32WB_SPI_TX_DMA_OPTION_TRANSFER_16;

        xf_count = xf_count / 2;
    }

    while (SPI->SR & (SPI_SR_FTLVL | SPI_SR_FRLVL | SPI_SR_BSY))
    {
    }
    
    SPI->CR1 = spi->cr1;
    SPI->CR2 = spi_cr2;
    SPI->CR1 = spi->cr1 | SPI_CR1_SPE;

    stm32wb_dma_start(spi->rx_dma, (uint32_t)rx_data, (uint32_t)&SPI->DR, xf_count, rx_option | STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE);
    stm32wb_dma_start(spi->tx_dma, (uint32_t)&SPI->DR, (uint32_t)tx_data, xf_count, tx_option);

    return true;
}

uint32_t stm32wb_spi_data_dma_cancel(stm32wb_spi_t *spi)
{
    SPI_TypeDef *SPI = spi->SPI;
    uint32_t xf_count, xf_16bit;
    uint8_t rx_data;
    volatile uint8_t *p_status_return;

    p_status_return = spi->xf_status;

    if (!p_status_return)
    {
        return 0;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&spi->xf_status, (uint32_t)p_status_return, (uint32_t)NULL) != (uint32_t)p_status_return)
    {
        return 0;
    }

    xf_16bit = (SPI->CR2 & SPI_CR2_FRXTH) ? 0 : 1;

    stm32wb_dma_stop(spi->tx_dma);
        
    while (SPI->SR & (SPI_SR_FTLVL | SPI_SR_BSY))
    {
    }

    SPI->CR1 = spi->cr1;
    SPI->CR2 = spi->cr2;
    
    xf_count = stm32wb_dma_stop(spi->rx_dma) << xf_16bit;
    
    while (SPI->SR & SPI_SR_FRLVL)
    {
        rx_data = STM32WB_SPI_READ_8(SPI);
        
        if (spi->rx_data)
        {
            spi->rx_data[xf_count] = rx_data;
        }
        
        xf_count++;
    }
        
    SPI->CR1 = spi->cr1 | SPI_CR1_SPE;

    spi->state = STM32WB_SPI_STATE_DATA;
    
    *p_status_return = STM32WB_SPI_STATUS_FAILURE;
    
    return xf_count;
}

bool stm32wb_spi_data_dma_busy(stm32wb_spi_t *spi)
{
    return (spi->state == STM32WB_SPI_STATE_DATA_DMA);
}
