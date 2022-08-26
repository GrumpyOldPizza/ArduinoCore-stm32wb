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
#include "Wire.h"
#include "wiring_private.h"

TwoWire::TwoWire(struct _stm32wb_i2c_t *i2c, const struct _stm32wb_i2c_params_t *params, struct _stm32wb_i2c_transaction_t *transaction) {
    m_i2c = i2c;
    m_transaction = transaction;

    stm32wb_i2c_create(m_i2c, params);

    m_transaction->next = NULL;
    m_transaction->status = STM32WB_I2C_STATUS_SUCCESS;
    
    m_option = STM32WB_I2C_OPTION_MODE_100K;
    m_timeout = 0;
    m_ev_address = 0;
    m_xf_address = 0;

    m_rx_read = 0;
    m_rx_count = 0;

    m_tx_write = 0;
    m_tx_active = false;

    m_receive_callback = NULL;
    m_request_callback = NULL;
    m_transmit_callback = NULL;

    m_done_callback = Callback();

    k_work_create(&m_work, (k_work_routine_t)TwoWire::workRoutine, (void*)this);
}

void TwoWire::begin() {
    m_ev_address = 0;

    m_option &= ~(STM32WB_I2C_OPTION_GENERAL_CALL | STM32WB_I2C_OPTION_ADDRESS_MASK | STM32WB_I2C_OPTION_MODE_MASK);
    m_option |= STM32WB_I2C_OPTION_MODE_100K;

    stm32wb_i2c_enable(m_i2c, m_option, m_timeout, NULL, NULL);
}

void TwoWire::begin(uint8_t address, bool generalCall) {
    m_ev_address = address;

    m_option &= ~(STM32WB_I2C_OPTION_GENERAL_CALL | STM32WB_I2C_OPTION_ADDRESS_MASK | STM32WB_I2C_OPTION_MODE_MASK);

    if (address) {
        m_option |= (STM32WB_I2C_OPTION_WAKEUP | (address << STM32WB_I2C_OPTION_ADDRESS_SHIFT));
    }
    
    if (generalCall) {
        m_option |= STM32WB_I2C_OPTION_GENERAL_CALL;
    }

    stm32wb_i2c_enable(m_i2c, m_option, m_timeout, (stm32wb_i2c_event_callback_t)TwoWire::eventCallback, (void*)this);
}

void TwoWire::end() {
    stm32wb_i2c_disable(m_i2c);
}

void TwoWire::setClock(uint32_t clock) {
    uint32_t option;

    if (m_ev_address) {
        return;
    }

    option = m_option & ~STM32WB_I2C_OPTION_MODE_MASK;

    if      (clock > 400000) { option |= STM32WB_I2C_OPTION_MODE_1000K; }
    else if (clock > 100000) { option |= STM32WB_I2C_OPTION_MODE_400K;  }
    else                     { option |= STM32WB_I2C_OPTION_MODE_100K;  }

    if (stm32wb_i2c_configure(m_i2c, option, m_timeout)) {
        m_option = option;
    }
}

void TwoWire::beginTransmission(uint8_t address) {
    if (!armv7m_core_is_in_thread() || k_work_is_in_progress()) {
        return;
    }

    if (m_transaction->status == STM32WB_I2C_STATUS_BUSY) {
        return;
    }

    if (m_ev_address) {
        return;
    }

    if (m_tx_active) {
        return;
    }
    
    m_tx_write = 0;
    m_tx_address = address;
    m_tx_active = true;
}

uint8_t TwoWire::endTransmission(bool stopBit) {
    if (!m_tx_active) {
        return WIRE_STATUS_FAILURE;
    }

    m_tx_active = false;

    if (m_xf_address && (m_xf_address != m_tx_address)) {
        return WIRE_STATUS_FAILURE;
    }

    m_transaction->control = stopBit ? 0 : STM32WB_I2C_CONTROL_RESTART;
    m_transaction->address = m_tx_address;
    m_transaction->tx_data = &m_tx_data[0];
    m_transaction->rx_data = NULL;
    m_transaction->tx_count = m_tx_write;
    m_transaction->rx_count = 0;
    m_transaction->callback = NULL;
    m_transaction->context = NULL;

    if (!stm32wb_i2c_submit(m_i2c, m_transaction)) {
        return WIRE_STATUS_FAILURE;
    }

    m_xf_address = 0;

    while (m_transaction->status == STM32WB_I2C_STATUS_BUSY) {
        __WFE();
    }

    if (m_transaction->status == STM32WB_I2C_STATUS_SUCCESS) {
        if (!stopBit) {
            m_xf_address = m_tx_address;
        }
    } 

    return m_transaction->status;
}

size_t TwoWire::requestFrom(uint8_t address, size_t size, bool stopBit) {
    if (!armv7m_core_is_in_thread() || k_work_is_in_progress()) {
        return 0;
    }

    if (m_transaction->status == STM32WB_I2C_STATUS_BUSY) {
        return 0;
    }

    if (m_ev_address) {
        return 0;
    }

    if (m_xf_address && (m_xf_address != address)) {
        return 0;
    }

    if (size == 0) {
        return 0;
    }

    if (size > WIRE_BUFFER_LENGTH) {
        size = WIRE_BUFFER_LENGTH;
    }

    m_transaction->control = stopBit ? 0 : STM32WB_I2C_CONTROL_RESTART;
    m_transaction->address = address;
    m_transaction->tx_data = NULL;
    m_transaction->rx_data = &m_rx_data[0];
    m_transaction->tx_count = 0;
    m_transaction->rx_count = size;
    m_transaction->callback = NULL;
    m_transaction->context = NULL;

    if (!stm32wb_i2c_submit(m_i2c, m_transaction)) {
        return 0;
    }

    m_xf_address = 0;

    m_rx_read = 0;
    m_rx_count = 0;

    while (m_transaction->status == STM32WB_I2C_STATUS_BUSY) {
        __WFE();
    }

    if (m_transaction->status == STM32WB_I2C_STATUS_SUCCESS) {
        if (!stopBit) {
            m_xf_address = address;
        }

        m_rx_count = size;
    }

    return m_rx_count;
}

size_t TwoWire::requestFrom(uint8_t address, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit) {
    uint8_t tx_data[4];
    uint16_t tx_count;
    
    if (!armv7m_core_is_in_thread() || k_work_is_in_progress()) {
        return 0;
    }

    if (m_transaction->status == STM32WB_I2C_STATUS_BUSY) {
        return 0;
    }

    if (m_ev_address) {
        return 0;
    }

    if (m_xf_address && (m_xf_address != address)) {
        return 0;
    }

    if (size == 0) {
        return 0;
    }

    if (size > WIRE_BUFFER_LENGTH) {
        size = WIRE_BUFFER_LENGTH;
    }

    tx_count = 0;

    if (isize >= 3) { tx_data[tx_count++] = iaddress >> 16; }
    if (isize >= 2) { tx_data[tx_count++] = iaddress >> 8;  }
    if (isize >= 1) { tx_data[tx_count++] = iaddress >> 0;  }

    m_transaction->control = stopBit ? 0 : STM32WB_I2C_CONTROL_RESTART;
    m_transaction->address = address;
    m_transaction->tx_data = &tx_data[0];
    m_transaction->rx_data = &m_rx_data[0];
    m_transaction->tx_count = tx_count;
    m_transaction->rx_count = size;
    m_transaction->callback = NULL;
    m_transaction->context = NULL;

    if (!stm32wb_i2c_submit(m_i2c, m_transaction)) {
        return 0;
    }

    m_xf_address = 0;

    m_rx_read = 0;
    m_rx_count = 0;

    while (m_transaction->status == STM32WB_I2C_STATUS_BUSY) {
        __WFE();
    }


    if (m_transaction->status == STM32WB_I2C_STATUS_SUCCESS) {
        if (!stopBit) {
            m_xf_address = address;
        }

        m_rx_count = size;
    }

    return m_rx_count;
}


int TwoWire::available(void) {
    return (m_rx_count - m_rx_read);
}

int TwoWire::peek(void) {
    if (m_rx_read >= m_rx_count) {
        return -1;
    }

    return m_rx_data[m_rx_read];
}

int TwoWire::read(void) {
    if (m_rx_read >= m_rx_count) {
        return -1;
    }

    return m_rx_data[m_rx_read++];
}

int TwoWire::read(uint8_t *buffer, size_t size) {
    if (size > (unsigned int)(m_rx_count - m_rx_read))
    {
        size = (m_rx_count - m_rx_read);
    }

    memcpy(buffer, &m_rx_data[m_rx_read], size);

    m_rx_read += size;

    return size;
}

int TwoWire::availableForWrite(void) {
    return (WIRE_BUFFER_LENGTH - m_tx_write);
}

size_t TwoWire::write(uint8_t data) {
    if (!m_tx_active) {
        return 0;
    }

    if (m_tx_write >= WIRE_BUFFER_LENGTH) {
        return 0;
    }

    m_tx_data[m_tx_write++] = data;

    return 1;
}

size_t TwoWire::write(const uint8_t *data, size_t size) {
    if (!m_tx_active) {
        return 0;
    }

    if (size > (unsigned int)(WIRE_BUFFER_LENGTH - m_tx_write)) {
        size = WIRE_BUFFER_LENGTH - m_tx_write;
    }

    memcpy(&m_tx_data[m_tx_write], data, size);

    m_tx_write += size;

    return size;
}

void TwoWire::flush(void) {
}

void TwoWire::setClockLowTimeout(unsigned long timeout) {
    if (m_ev_address) {
        return;
    }

    if (stm32wb_i2c_configure(m_i2c, m_option, timeout)) {
        m_timeout = timeout;
    }
}

bool TwoWire::isGeneralCall() {
    return (m_rx_address == 0);
}

uint8_t TwoWire::transfer(uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, bool stopBit) {
    if (!armv7m_core_is_in_thread() || k_work_is_in_progress()) {
        return WIRE_STATUS_FAILURE;
    }

    if (m_transaction->status == STM32WB_I2C_STATUS_BUSY) {
        return WIRE_STATUS_FAILURE;
    }
    
    if (m_ev_address) {
        return WIRE_STATUS_FAILURE;
    }

    if (m_xf_address && (m_xf_address != address)) {
        return WIRE_STATUS_FAILURE;
    }

    if (!txBuffer && txSize)  {
        return WIRE_STATUS_FAILURE;
    }

    if (!rxBuffer && rxSize)  {
        return WIRE_STATUS_FAILURE;
    }

    if (rxBuffer && !rxSize)  {
        return WIRE_STATUS_FAILURE;
    }

    if ((txSize > 65535) || (rxSize > 65536))  {
        return WIRE_STATUS_FAILURE;
    }

    m_transaction->control = stopBit ? 0 : STM32WB_I2C_CONTROL_RESTART;
    m_transaction->address = address;
    m_transaction->tx_data = txBuffer;
    m_transaction->rx_data = rxBuffer;
    m_transaction->tx_count = txSize;
    m_transaction->rx_count = rxSize;
    m_transaction->callback = NULL;
    m_transaction->context = NULL;

    if (!stm32wb_i2c_submit(m_i2c, m_transaction)) {
        return WIRE_STATUS_FAILURE;
    }

    while (m_transaction->status == STM32WB_I2C_STATUS_BUSY) {
        __WFE();
    }

    if (m_transaction->status == STM32WB_I2C_STATUS_SUCCESS) {
        if (!stopBit) {
            m_xf_address = address;
        }
    } 

    return m_transaction->status;
}

bool TwoWire::transfer(uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, bool stopBit, volatile uint8_t &status) {
    return transfer(address, txBuffer, txSize, rxBuffer, rxSize, stopBit, status, Callback());
}

bool TwoWire::transfer(uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, bool stopBit, volatile uint8_t &status, void(*callback)(void)) {
    return transfer(address, txBuffer, txSize, rxBuffer, rxSize, stopBit, status, Callback(callback));
}

bool TwoWire::transfer(uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, bool stopBit, volatile uint8_t &status, Callback callback) {
    if (!armv7m_core_is_in_thread() || k_work_is_in_progress()) {
        return false;
    }

    if (m_transaction->status == STM32WB_I2C_STATUS_BUSY) {
        return false;
    }
    
    if (m_ev_address) {
        return false;
    }

    if (m_xf_address && (m_xf_address != address)) {
        return false;
    }

    if (!txBuffer && txSize)  {
        return false;
    }

    if (!rxBuffer && rxSize)  {
        return false;
    }

    if (rxBuffer && !rxSize)  {
        return false;
    }

    if ((txSize > 65535) || (rxSize > 65536))  {
        return false;
    }

    m_xf_status = &status;

    m_done_callback = callback;
	
    m_transaction->control = stopBit ? 0 : STM32WB_I2C_CONTROL_RESTART;
    m_transaction->address = address;
    m_transaction->tx_data = txBuffer;
    m_transaction->rx_data = rxBuffer;
    m_transaction->tx_count = txSize;
    m_transaction->rx_count = rxSize;
    m_transaction->callback = (stm32wb_i2c_done_callback_t)TwoWire::doneCallback;
    m_transaction->context = (void*)this;

    if (!stm32wb_i2c_submit(m_i2c, m_transaction)) {
	return false;
    }

    status = WIRE_STATUS_BUSY;

    return true;
}

void TwoWire::reset() {
    if (!armv7m_core_is_in_thread() || k_work_is_in_progress()) {
        return;
    }

    if (m_transaction->status == STM32WB_I2C_STATUS_BUSY) {
        return;
    }
    
    if (stm32wb_i2c_suspend(m_i2c, NULL, NULL)) {
        while (m_i2c->state != STM32WB_I2C_STATE_SUSPENDED) {
            __WFE();
        }
    }

    stm32wb_i2c_reset(m_i2c);

    stm32wb_i2c_resume(m_i2c);
}

void TwoWire::onReceive(void(*callback)(int)) {
    m_receive_callback = callback;
}

void TwoWire::onRequest(void(*callback)(void)) {
    m_request_callback = callback;
}

void TwoWire::onTransmit(void(*callback)(int)) {
    m_transmit_callback = callback;
}

void TwoWire::workRoutine(class TwoWire *self) {
    (self->m_done_callback)();
}

void TwoWire::doneCallback(class TwoWire *self) {
    *self->m_xf_status = self->m_transaction->status;
    
    k_work_submit(&self->m_work);
}

void TwoWire::eventCallback(class TwoWire *self, uint32_t events) {
    if (events & STM32WB_I2C_EVENT_RECEIVE_REQUEST) {
        self->m_rx_address = (events & STM32WB_I2C_EVENT_ADDRESS_MASK) >> STM32WB_I2C_EVENT_ADDRESS_SHIFT;

        stm32wb_i2c_receive(self->m_i2c, &self->m_rx_data[0], WIRE_BUFFER_LENGTH);
    }

    if (events & STM32WB_I2C_EVENT_RECEIVE_DONE) {
        self->m_rx_read = 0;
        self->m_rx_count = (events & STM32WB_I2C_EVENT_COUNT_MASK) >> STM32WB_I2C_EVENT_COUNT_SHIFT;

        if (self->m_receive_callback) {
            (*self->m_receive_callback)(self->m_rx_count);
        }
    }
    
    if (events & STM32WB_I2C_EVENT_TRANSMIT_REQUEST) {
        self->m_tx_active = true;
        self->m_tx_write = 0;

        if (self->m_request_callback) {
            (*self->m_request_callback)();
        }

        self->m_tx_active = false;

        stm32wb_i2c_transmit(self->m_i2c, &self->m_tx_data[0], self->m_tx_write);
    }

    if (events & STM32WB_I2C_EVENT_TRANSMIT_DONE) {
        if (self->m_transmit_callback) {
            (*self->m_transmit_callback)((events & STM32WB_I2C_EVENT_COUNT_MASK) >> STM32WB_I2C_EVENT_COUNT_SHIFT);
        }
    }
}

#if WIRE_INTERFACES_COUNT > 0

extern stm32wb_i2c_t g_Wire;
static stm32wb_i2c_transaction_t g_WireTransaction;
extern const stm32wb_i2c_params_t g_WireParams;

TwoWire Wire(&g_Wire, &g_WireParams, &g_WireTransaction);

#endif

#if WIRE_INTERFACES_COUNT > 1

static stm32wb_i2c_t g_Wire1;
static stm32wb_i2c_transaction_t g_Wire1Transaction;
extern const stm32wb_i2c_params_t g_Wire1Params;

TwoWire Wire1(&g_Wire1, &g_Wire1Params, &g_Wire1Transaction);

#endif
