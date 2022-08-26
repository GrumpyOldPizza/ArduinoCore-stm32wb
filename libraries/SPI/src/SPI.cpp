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

#include "Arduino.h"
#include "SPI.h"
#include "wiring_private.h"

struct SPIInterface {
    bool     (*enable)(stm32wb_spi_t *spi);
    bool     (*disable)(stm32wb_spi_t *spi);
    bool     (*block)(stm32wb_spi_t *spi, uint16_t pin);
    bool     (*unblock)(stm32wb_spi_t *spi, uint16_t pin);
    bool     (*acquire)(stm32wb_spi_t *spi, uint32_t clock, uint32_t control);
    bool     (*release)(stm32wb_spi_t *spi);
    uint8_t  (*data)(stm32wb_spi_t *spi, uint8_t data);
    uint16_t (*data16le)(stm32wb_spi_t *spi, uint16_t data);
    uint16_t (*data16be)(stm32wb_spi_t *spi, uint16_t data);
    void     (*data_receive)(stm32wb_spi_t *spi, uint8_t *rx_data, uint32_t rx_count);
    void     (*data_transmit)(stm32wb_spi_t *spi, const uint8_t *tx_data, uint32_t tx_count);
    void     (*data_transfer)(stm32wb_spi_t *spi, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count);
    bool     (*data_dma_receive)(stm32wb_spi_t *spi, uint8_t *rx_data, uint32_t rx_count, volatile uint8_t *p_status_return, stm32wb_spi_done_callback_t callback, void *context);
    bool     (*data_dma_transmit)(stm32wb_spi_t *spi, const uint8_t *tx_data, uint32_t tx_count, volatile uint8_t *p_status_return, stm32wb_spi_done_callback_t callback, void *context);
    bool     (*data_dma_transfer)(stm32wb_spi_t *spi, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count, volatile uint8_t *p_status_return, stm32wb_spi_done_callback_t callback, void *context);
    uint32_t (*data_dma_cancel)(stm32wb_spi_t *spi);
    bool     (*data_dma_busy)(stm32wb_spi_t *spi);
};

static const struct SPIInterface stm32wb_spi_interface =
{
    stm32wb_spi_enable,
    stm32wb_spi_disable,
    stm32wb_spi_block,
    stm32wb_spi_unblock,
    stm32wb_spi_acquire,
    stm32wb_spi_release,
    stm32wb_spi_data,
    stm32wb_spi_data16le,
    stm32wb_spi_data16be,
    stm32wb_spi_data_receive,
    stm32wb_spi_data_transmit,
    stm32wb_spi_data_transfer,
    stm32wb_spi_data_dma_receive,
    stm32wb_spi_data_dma_transmit,
    stm32wb_spi_data_dma_transfer,
    stm32wb_spi_data_dma_cancel,
    stm32wb_spi_data_dma_busy
};

struct UARTInterface {
    bool     (*enable)(stm32wb_uart_t *uart);
    bool     (*disable)(stm32wb_uart_t *uart);
    bool     (*block)(stm32wb_uart_t *uart, uint16_t pin);
    bool     (*unblock)(stm32wb_uart_t *uart, uint16_t pin);
    bool     (*acquire)(stm32wb_uart_t *uart, uint32_t clock, uint32_t control);
    bool     (*release)(stm32wb_uart_t *uart);
    uint8_t  (*data)(stm32wb_uart_t *uart, uint8_t data);
    uint16_t (*data16le)(stm32wb_uart_t *uart, uint16_t data);
    uint16_t (*data16be)(stm32wb_uart_t *uart, uint16_t data);
    void     (*data_receive)(stm32wb_uart_t *uart, uint8_t *rx_data, uint32_t rx_count);
    void     (*data_transmit)(stm32wb_uart_t *uart, const uint8_t *tx_data, uint32_t tx_count);
    void     (*data_transfer)(stm32wb_uart_t *uart, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count);
    bool     (*data_dma_receive)(stm32wb_uart_t *uart, uint8_t *rx_data, uint32_t rx_count, volatile uint8_t *p_status_return, stm32wb_uart_done_callback_t callback, void *context);
    bool     (*data_dma_transmit)(stm32wb_uart_t *uart, const uint8_t *tx_data, uint32_t tx_count, volatile uint8_t *p_status_return, stm32wb_uart_done_callback_t callback, void *context);
    bool     (*data_dma_transfer)(stm32wb_uart_t *uart, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count, volatile uint8_t *p_status_return, stm32wb_uart_done_callback_t callback, void *context);
    uint32_t (*data_dma_cancel)(stm32wb_uart_t *uart);
    bool     (*data_dma_busy)(stm32wb_uart_t *uart);
};

static const struct UARTInterface stm32wb_uart_interface =
{
    stm32wb_uart_spi_enable,
    stm32wb_uart_spi_disable,
    stm32wb_uart_spi_block,
    stm32wb_uart_spi_unblock,
    stm32wb_uart_spi_acquire,
    stm32wb_uart_spi_release,
    stm32wb_uart_spi_data,
    stm32wb_uart_spi_data16le,
    stm32wb_uart_spi_data16be,
    stm32wb_uart_spi_data_receive,
    stm32wb_uart_spi_data_transmit,
    stm32wb_uart_spi_data_transfer,
    stm32wb_uart_spi_data_dma_receive,
    stm32wb_uart_spi_data_dma_transmit,
    stm32wb_uart_spi_data_dma_transfer,
    stm32wb_uart_spi_data_dma_cancel,
    stm32wb_uart_spi_data_dma_busy
};

SPIClass::SPIClass(struct _stm32wb_spi_t *spi, const struct _stm32wb_spi_params_t *params) :
    m_spi(spi),
    m_interface(&stm32wb_spi_interface)
{
    stm32wb_spi_create(spi, params);

    m_transaction = false;
    
    m_data_8 = null_data_8;
    m_data_16 = null_data_16;

    k_work_create(&m_work, (k_work_routine_t)SPIClass::workCallback, (void*)this);
}

SPIClass::SPIClass(struct _stm32wb_uart_t *uart, const struct _stm32wb_uart_params_t *params) :
    m_spi((stm32wb_spi_t*)uart),
    m_interface((const struct SPIInterface*)&stm32wb_uart_interface)
{
    stm32wb_uart_create(uart, params);

    m_transaction = false;
    
    m_data_8 = null_data_8;
    m_data_16 = null_data_16;

    k_work_create(&m_work, (k_work_routine_t)SPIClass::workCallback, (void*)this);
}

__attribute__((optimize("O3"))) uint8_t SPIClass::transfer(uint8_t data) {
    return m_data_8(m_spi, data);
}

__attribute__((optimize("O3"))) uint16_t SPIClass::transfer16(uint16_t data) {
    return m_data_16(m_spi, data);
}

__attribute__((optimize("O3"))) void SPIClass::transfer(void *buffer, size_t count) {
    if (m_transaction) {
        m_interface->data_transfer(m_spi, (const uint8_t*)buffer, (uint8_t*)buffer, count);
    }
}

void SPIClass::usingInterrupt(int interruptNumber) {
    if ((interruptNumber < 0) || (interruptNumber >= (int)PINS_COUNT) || !(g_APinDescription[interruptNumber].attr & PIN_ATTR_EXTI)) {
        return;
    }

    m_interface->block(m_spi, g_APinDescription[interruptNumber].pin);
}

void SPIClass::notUsingInterrupt(int interruptNumber) {
    if ((interruptNumber < 0) || (interruptNumber >= (int)PINS_COUNT) || !(g_APinDescription[interruptNumber].attr & PIN_ATTR_EXTI)) {
        return;
    }

    m_interface->unblock(m_spi, g_APinDescription[interruptNumber].pin);
}

__attribute__((optimize("O3"))) void SPIClass::beginTransaction(SPISettings settings) {
    if (!m_transaction) {
        m_interface->acquire(m_spi, settings.m_clock, settings.m_control);

        m_data_8 = m_interface->data;
        m_data_16 = (settings.m_control & STM32WB_SPI_CONTROL_LSB_FIRST) ? m_interface->data16le : m_interface->data16be;

        m_transaction = true;
    }
}

__attribute__((optimize("O3"))) void SPIClass::endTransaction(void) {
    if (m_transaction) {
        m_transaction = false;

        m_interface->release(m_spi);

        m_data_8 = null_data_8;
        m_data_16 = null_data_16;
    }
}

// SPI Configuration methods
void SPIClass::attachInterrupt() {
}

void SPIClass::detachInterrupt() {
}

void SPIClass::begin() {
    if (!m_transaction) {
        m_interface->enable(m_spi);
    }
}

void SPIClass::end() {
    if (!m_transaction) {
        m_interface->disable(m_spi);
    }
}

__attribute__((optimize("O3"))) void SPIClass::transfer(const void *txBuffer, void *rxBuffer, size_t count) {
    if (m_transaction) {
        if (txBuffer) {
            if (rxBuffer) {
                m_interface->data_transfer(m_spi, (const uint8_t*)txBuffer, (uint8_t*)rxBuffer, count);
            } else {
                m_interface->data_transmit(m_spi, (const uint8_t*)txBuffer, count);
            }
        } else {
            m_interface->data_receive(m_spi, (uint8_t*)rxBuffer, count);
        }
    }
}

bool SPIClass::transfer(const void *txBuffer, void *rxBuffer, size_t count, volatile uint8_t &status) {
    return transfer(txBuffer, rxBuffer, count, status, Callback());
}

bool SPIClass::transfer(const void *txBuffer, void *rxBuffer, size_t count, volatile uint8_t &status, void(*callback)(void)) {
    return transfer(txBuffer, rxBuffer, count, status, Callback(callback));
}

bool SPIClass::transfer(const void *txBuffer, void *rxBuffer, size_t count, volatile uint8_t &status, Callback callback) {
    if (!m_transaction) {
        return false;
    }

    m_done_callback = callback;

    if (txBuffer) {
        if (rxBuffer) {
            if (!m_interface->data_dma_transfer(m_spi, (const uint8_t*)txBuffer, (uint8_t*)rxBuffer, count, &status, (stm32wb_spi_done_callback_t)SPIClass::doneCallback, (void*)this)) {
                return false;
            }
        } else {
            if (!m_interface->data_dma_transmit(m_spi, (const uint8_t*)txBuffer, count, &status, (stm32wb_spi_done_callback_t)SPIClass::doneCallback, (void*)this)) {
                return false;
            }
        }
    }
    else
    {
        if (!m_interface->data_dma_receive(m_spi, (uint8_t*)rxBuffer, count, &status, (stm32wb_spi_done_callback_t)SPIClass::doneCallback, (void*)this)) {
            return false;
        }
    }

    return true;
}

size_t SPIClass::cancel(void) {
    return m_interface->data_dma_cancel(m_spi);
}

uint8_t SPIClass::null_data_8(stm32wb_spi_t *spi __attribute__((unused)), uint8_t data __attribute__((unused))) {
    return 0xff;
}

uint16_t SPIClass::null_data_16(stm32wb_spi_t *spi __attribute__((unused)), uint16_t data __attribute__((unused))) {
    return 0xffff;
}

void SPIClass::workCallback(SPIClass *self) {
    if (self->m_done_callback) {
        self->m_done_callback();
    }
}

void SPIClass::doneCallback(SPIClass *self) {
    k_work_submit(&self->m_work);
}

#if SPI_INTERFACES_COUNT > 0

extern stm32wb_spi_t g_SPI;
extern const stm32wb_spi_params_t g_SPIParams;

SPIClass SPI(&g_SPI, &g_SPIParams);

#endif

#if SPI_INTERFACES_COUNT > 1

#if (STM32WB_CONFIG_SYSOPT & STM32WB_SYSTEM_OPTION_USART1_SYSCLK)

static stm32wb_uart_t g_SPI1;
extern const stm32wb_uart_params_t g_SPI1Params;

#else /* STM32WB_CONFIG_SYSOPT & STM32WB_SYSTEM_OPTION_USART1_SYSCLK */

static stm32wb_spi_t g_SPI1;
extern const stm32wb_spi_params_t g_SPI1Params;

#endif /* STM32WB_CONFIG_SYSOPT & STM32WB_SYSTEM_OPTION_USART1_SYSCLK */

SPIClass SPI1(&g_SPI1, &g_SPI1Params);

#endif
