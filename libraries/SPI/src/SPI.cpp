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

static SPIClass *g_spi_class[STM32WB_SPI_INSTANCE_COUNT];

SPIClass::SPIClass(struct _stm32wb_spi_t *spi, const struct _stm32wb_spi_params_t *params) {
    m_spi = spi;

    stm32wb_spi_create(spi, params);

    m_active = false;

    m_clock = 4000000;
    m_control = 0;

    m_data_8 = null_data_8;
    m_data_16 = null_data_16;
    m_data_transfer = null_data_transfer;

    Callback work_callback = Callback(&SPIClass::workCallback, this);

    k_work_create(&m_work, work_callback.callback(), work_callback.context());
    
    g_spi_class[spi->instance] = this;
}

void SPIClass::begin() {
    if (m_active) {
        stm32wb_spi_release(m_spi);

        m_active = false;
    }

    stm32wb_spi_enable(m_spi);

    m_data_8 = select_data_8;
    m_data_16 = select_data_16;
    m_data_transfer = select_data_transfer;
}

void SPIClass::end() {
    if (m_active) {
        stm32wb_spi_release(m_spi);

        m_active = false;
    }

    m_data_8 = null_data_8;
    m_data_16 = null_data_16;
    m_data_transfer = null_data_transfer;
    
    stm32wb_spi_disable(m_spi);
}

void SPIClass::usingInterrupt(uint32_t pin) {
    if ((pin >= PINS_COUNT) || !(g_APinDescription[pin].attr & PIN_ATTR_EXTI)) {
        return;
    }

    stm32wb_spi_block(m_spi, pin);
}

void SPIClass::notUsingInterrupt(uint32_t pin) {
    if ((pin >= PINS_COUNT) || !(g_APinDescription[pin].attr & PIN_ATTR_EXTI)) {
        return;
    }

    stm32wb_spi_unblock(m_spi, pin);
}

__attribute__((optimize("O3"))) void SPIClass::beginTransaction(SPISettings settings) {
    if (m_active) {
        stm32wb_spi_release(m_spi);
    }

    stm32wb_spi_acquire(m_spi, settings.m_clock, settings.m_control);

    m_active = true;

    m_data_8 = stm32wb_spi_data;
    m_data_16 = (settings.m_control & STM32WB_SPI_CONTROL_LSB_FIRST) ? stm32wb_spi_data16le : stm32wb_spi_data16be;
    m_data_transfer = stm32wb_spi_data_transfer;
}

__attribute__((optimize("O3"))) void SPIClass::endTransaction(void) {
    if (m_active) {
        stm32wb_spi_release(m_spi);

        m_data_8 = select_data_8;
        m_data_16 = select_data_16;
        m_data_transfer = select_data_transfer;

        m_active = false;
    }
}

void SPIClass::setBitOrder(BitOrder bitOrder) {
    if (m_active) {
        stm32wb_spi_release(m_spi);

        m_data_8 = select_data_8;
        m_data_16 = select_data_16;
        m_data_transfer = select_data_transfer;

        m_active = false;
    }

    m_control = (m_control & ~STM32WB_SPI_CONTROL_LSB_FIRST) | ((bitOrder == LSBFIRST) ? STM32WB_SPI_CONTROL_LSB_FIRST : 0);
}

void SPIClass::setDataMode(uint8_t dataMode) {
    if (m_active) {
        stm32wb_spi_release(m_spi);

        m_data_8 = select_data_8;
        m_data_16 = select_data_16;
        m_data_transfer = select_data_transfer;

        m_active = false;
    }

    m_control = (m_control & ~3) | (dataMode & 3);
}

void SPIClass::setClockDivider(uint8_t divider) {
    if (m_active) {
        stm32wb_spi_release(m_spi);

        m_data_8 = select_data_8;
        m_data_16 = select_data_16;
        m_data_transfer = select_data_transfer;

        m_active = false;
    }

    if (divider != 0) {
        m_clock = SystemCoreClock / divider;
    }
}

void SPIClass::attachInterrupt() {
    // Should be enableInterrupt()
}

void SPIClass::detachInterrupt() {
    // Should be disableInterrupt()
}

bool SPIClass::transfer(const void *txBuffer, void *rxBuffer, size_t count, volatile uint8_t &status) {
    return transfer(txBuffer, rxBuffer, count, status, Callback(__wakeupCallback));
}

bool SPIClass::transfer(const void *txBuffer, void *rxBuffer, size_t count, volatile uint8_t &status, void(*callback)(void)) {
    return transfer(txBuffer, rxBuffer, count, status, Callback(callback));
}

bool SPIClass::transfer(const void *txBuffer, void *rxBuffer, size_t count, volatile uint8_t &status, Callback callback) {
    if (!m_active) {
        return false;
    }

    m_done_callback = callback;

    Callback done_callback = callback ? Callback(&SPIClass::doneCallback, this) : Callback();
    
    if (txBuffer) {
        if (rxBuffer) {
            if (!stm32wb_spi_data_dma_transfer(m_spi, (const uint8_t*)txBuffer, (uint8_t*)rxBuffer, count, &status, done_callback.callback(), done_callback.context())) {
                return false;
            }
        } else {
            if (!stm32wb_spi_data_dma_transmit(m_spi, (const uint8_t*)txBuffer, count, &status, done_callback.callback(), done_callback.context())) {
                return false;
            }
        }
    }
    else
    {
        if (!stm32wb_spi_data_dma_receive(m_spi, (uint8_t*)rxBuffer, count, &status, done_callback.callback(), done_callback.context())) {
            return false;
        }
    }

    return true;
}

size_t SPIClass::cancel(void) {
    return stm32wb_spi_data_dma_cancel(m_spi);
}

uint8_t SPIClass::select_data_8(struct _stm32wb_spi_t *spi, uint8_t data) {
    SPIClass *self = g_spi_class[spi->instance];

    stm32wb_spi_acquire(spi, self->m_clock, self->m_control);

    self->m_active = true;

    self->m_data_8 = stm32wb_spi_data;
    self->m_data_16 = (self->m_control & STM32WB_SPI_CONTROL_LSB_FIRST) ? stm32wb_spi_data16le : stm32wb_spi_data16be;
    self->m_data_transfer = stm32wb_spi_data_transfer;
  
    return (*self->m_data_8)(spi, data);
}

uint16_t SPIClass::select_data_16(struct _stm32wb_spi_t *spi, uint16_t data) {
    SPIClass *self = g_spi_class[spi->instance];

    stm32wb_spi_acquire(spi, self->m_clock, self->m_control);

    self->m_active = true;

    self->m_data_8 = stm32wb_spi_data;
    self->m_data_16 = (self->m_control & STM32WB_SPI_CONTROL_LSB_FIRST) ? stm32wb_spi_data16le : stm32wb_spi_data16be;
    self->m_data_transfer = stm32wb_spi_data_transfer;
  
    return (*self->m_data_16)(spi, data);
}

void SPIClass::select_data_transfer(struct _stm32wb_spi_t *spi, const uint8_t *tx_data, uint8_t *rx_data, uint32_t count) {
    SPIClass *self = g_spi_class[spi->instance];

    stm32wb_spi_acquire(spi, self->m_clock, self->m_control);

    self->m_active = true;

    self->m_data_8 = stm32wb_spi_data;
    self->m_data_16 = (self->m_control & STM32WB_SPI_CONTROL_LSB_FIRST) ? stm32wb_spi_data16le : stm32wb_spi_data16be;
    self->m_data_transfer = stm32wb_spi_data_transfer;
  
    return (*self->m_data_transfer)(spi, tx_data, rx_data, count);
}

uint8_t SPIClass::null_data_8(struct _stm32wb_spi_t *spi __attribute__((unused)), uint8_t data __attribute__((unused))) {
    return 0;
}

uint16_t SPIClass::null_data_16(struct _stm32wb_spi_t *spi __attribute__((unused)), uint16_t data __attribute__((unused))) {
    return 0;
}

void SPIClass::null_data_transfer(struct _stm32wb_spi_t *spi __attribute__((unused)), const uint8_t *tx_data __attribute__((unused)), uint8_t *rx_data __attribute__((unused)), uint32_t count __attribute__((unused))) {
}

void SPIClass::workCallback() {
    m_done_callback();
}

void SPIClass::doneCallback() {
    k_work_submit(&m_work);
}

#if SPI_INTERFACES_COUNT > 0

extern stm32wb_spi_t g_SPI;
extern const stm32wb_spi_params_t g_SPIParams;

SPIClass SPI(&g_SPI, &g_SPIParams);

#endif

#if SPI_INTERFACES_COUNT > 1

static __attribute__((section(".bss2"))) stm32wb_spi_t g_SPI1;
extern const stm32wb_spi_params_t g_SPI1Params;

SPIClass SPI1(&g_SPI1, &g_SPI1Params);

#endif
