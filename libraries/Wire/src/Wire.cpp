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
#include "Wire.h"
#include "wiring_private.h"

TwoWire::TwoWire(stm32wb_i2c_t *i2c, const stm32wb_i2c_params_t *params) {
    m_i2c = i2c;

    stm32wb_i2c_create(m_i2c, params);
    
    m_option = STM32WB_I2C_OPTION_MODE_100K;
    m_timeout = 0;
    m_ev_address = 0;
    m_ev_broadcast = false;

    m_xf_address = 0;
    
    m_tx_address = 0;
    m_tx_index = 0;
    m_tx_busy = false;

    m_rx_index = 0;
    m_rx_count = 0;

    m_receive_callback = NULL;
    m_request_callback = NULL;
    m_transmit_callback = NULL;
}

void TwoWire::begin() {
    m_ev_address = 0;

    m_option &= ~(STM32WB_I2C_OPTION_GENERAL_CALL | STM32WB_I2C_OPTION_ADDRESS_MASK | STM32WB_I2C_OPTION_MODE_MASK);
    m_option |= STM32WB_I2C_OPTION_MODE_100K;

    stm32wb_i2c_enable(m_i2c, m_option, m_timeout, NULL, NULL);
}

void TwoWire::begin(uint8_t address) {
    begin(address, false);
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
    if (!k_task_is_in_progress()) {
        return;
    }

    if (m_tx_busy) {
        return;
    }
    
    m_tx_address = address;
    m_tx_index = 0;
    m_tx_busy = true;
}

uint8_t TwoWire::endTransmission() {
    return endTransmission(true);
}

uint8_t TwoWire::endTransmission(bool stopBit) {
    stm32wb_i2c_transaction_t transaction;

    if (!k_task_is_in_progress()) {
        return 4;
    }

    if (m_xf_address && (m_xf_address != m_tx_address)) {
        return 4;
    }

    if (!m_tx_busy) {
        return 4;
    }
    
    transaction.next = NULL;
    transaction.address = m_tx_address;
    transaction.control = (stopBit ? 0 : STM32WB_I2C_CONTROL_RESTART);
    transaction.tx_data = &m_tx_data[0];
    transaction.rx_data = NULL;
    transaction.tx_count = m_tx_index;
    transaction.rx_count = 0;
    transaction.callback = (stm32wb_i2c_done_callback_t)TwoWire::doneCallback;
    transaction.context = (void*)k_task_self();

    if (!stm32wb_i2c_submit(m_i2c, &transaction)) {
        return 4;
    }

    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

    m_tx_busy = false;
    
    if (transaction.status == STM32WB_I2C_STATUS_SUCCESS) {
        m_xf_address = stopBit ? 0 : m_tx_address;
        
        return 0;
    }

    m_xf_address = 0;
    
    return transaction.status;
}

size_t TwoWire::requestFrom(uint8_t address, size_t size) {
    return requestFrom(address, size, true);
}

size_t TwoWire::requestFrom(uint8_t address, size_t size, bool stopBit) {
    stm32wb_i2c_transaction_t transaction;
    
    if (!k_task_is_in_progress()) {
        return 0;
    }
    
    if (size > WIRE_BUFFER_LENGTH) {
        return 0;
    }
    
    if (!k_task_is_in_progress()) {
        return 0;
    }
    
    if (m_ev_address) {
        return 0;
    }

    if (m_xf_address && (m_xf_address != address)) {
        return 0;
    }

    transaction.next = NULL;
    transaction.address = address;
    transaction.control = stopBit ? 0 : STM32WB_I2C_CONTROL_RESTART;
    transaction.tx_data = NULL;
    transaction.rx_data = &m_rx_data[0];
    transaction.tx_count = 0;
    transaction.rx_count = size;
    transaction.callback = (stm32wb_i2c_done_callback_t)TwoWire::doneCallback;
    transaction.context = (void*)k_task_self();

    if (!stm32wb_i2c_submit(m_i2c, &transaction)) {
        return 0;
    }
    
    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

    m_rx_index = 0;

    if (transaction.status == STM32WB_I2C_STATUS_SUCCESS) {
        m_xf_address = stopBit ? 0 : address;
        m_rx_count = size;
        
        return size;
    }

    m_xf_address = 0;
    m_rx_count = 0;
    
    return 0;
}

size_t TwoWire::requestFrom(uint8_t address, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit) {
    stm32wb_i2c_transaction_t transaction;
    uint32_t xf_count;
    uint8_t xf_data[3];
    
    if (!k_task_is_in_progress()) {
        return 0;
    }
    
    if (size > WIRE_BUFFER_LENGTH) {
        return 0;
    }
    
    if (!k_task_is_in_progress()) {
        return 0;
    }
    
    if (m_ev_address) {
        return 0;
    }

    if (m_xf_address && (m_xf_address != address)) {
        return 0;
    }

    xf_count = 0;

    if (isize >= 3) { xf_data[xf_count++] = iaddress >> 16; }
    if (isize >= 2) { xf_data[xf_count++] = iaddress >> 8;  }
    if (isize >= 1) { xf_data[xf_count++] = iaddress >> 0;  }

    transaction.next = NULL;
    transaction.address = address;
    transaction.control = stopBit ? 0 : STM32WB_I2C_CONTROL_RESTART;
    transaction.tx_data = &xf_data[0];
    transaction.rx_data = &m_rx_data[0];
    transaction.tx_count = xf_count;
    transaction.rx_count = size;
    transaction.callback = (stm32wb_i2c_done_callback_t)TwoWire::doneCallback;
    transaction.context = (void*)k_task_self();

    if (!stm32wb_i2c_submit(m_i2c, &transaction)) {
        return 0;
    }
    
    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

    m_rx_index = 0;

    if (transaction.status == STM32WB_I2C_STATUS_SUCCESS) {
        m_xf_address = stopBit ? 0 : address;
        m_rx_count = size;
        
        return size;
    }

    m_xf_address = 0;
    m_rx_count = 0;
    
    return 0;
}

int TwoWire::available(void) {
    return (m_rx_count - m_rx_index);
}

int TwoWire::peek(void) {
    if (m_rx_index >= m_rx_count) {
        return -1;
    }

    return m_rx_data[m_rx_index];
}

int TwoWire::read(void) {
    if (m_rx_index >= m_rx_count) {
        return -1;
    }

    return m_rx_data[m_rx_index++];
}

int TwoWire::read(uint8_t *buffer, size_t size) {
    if (size > (unsigned int)(m_rx_count - m_rx_index))
    {
        size = (m_rx_count - m_rx_index);
    }

    memcpy(buffer, &m_rx_data[m_rx_index], size);

    m_rx_index += size;

    return size;
}

int TwoWire::availableForWrite(void) {
    return (WIRE_BUFFER_LENGTH - m_tx_index);
}

size_t TwoWire::write(uint8_t data) {
    if (!m_tx_busy) {
        return 0;
    }

    if (m_tx_index >= WIRE_BUFFER_LENGTH) {
        return 0;
    }

    m_tx_data[m_tx_index++] = data;

    return 1;
}

size_t TwoWire::write(const uint8_t *data, size_t size) {
    if (!m_tx_busy) {
        return 0;
    }

    if (size > (unsigned int)(WIRE_BUFFER_LENGTH - m_tx_index)) {
        size = WIRE_BUFFER_LENGTH - m_tx_index;
    }

    memcpy(&m_tx_data[m_tx_index], data, size);

    m_tx_index += size;

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
    return m_ev_broadcast;
}

uint8_t TwoWire::transfer(uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *xfBuffer, size_t xfSize, bool request, bool stopBit) {
    stm32wb_i2c_transaction_t transaction;

    if (!txBuffer && txSize)  {
        return 1;
    }

    if (!xfBuffer && xfSize)  {
        return 1;
    }

    if (xfBuffer && !xfSize)  {
        return 1;
    }

    if ((txSize > 65535) || (xfSize > 65536))  {
        return 1;
    }

    if (!k_task_is_in_progress()) {
        return 4;
    }

    if (m_xf_address && (m_xf_address != address)) {
        return 4;
    }

    transaction.next = NULL;
    transaction.address = address;
    transaction.control = (stopBit ? 0 : STM32WB_I2C_CONTROL_RESTART) | (request ? 0 : STM32WB_I2C_CONTROL_DIRECTION);
    transaction.tx_data = txBuffer;
    transaction.rx_data = xfBuffer;
    transaction.tx_count = txSize;
    transaction.rx_count = xfSize;
    transaction.callback = (stm32wb_i2c_done_callback_t)TwoWire::doneCallback;
    transaction.context = (void*)k_task_self();
    
    if (!stm32wb_i2c_submit(m_i2c, &transaction)) {
        return 4;
    }

    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

    if (transaction.status == STM32WB_I2C_STATUS_SUCCESS) {
        m_xf_address = stopBit ? 0 : address;
        
        return 0;
    }

    m_xf_address = 0;
    
    return transaction.status;
}

void TwoWire::reset() {
    if (!k_task_is_in_progress()) {
        return;
    }

    if (!stm32wb_i2c_suspend(m_i2c, (stm32wb_i2c_done_callback_t)TwoWire::doneCallback, k_task_self())) {
        return;
    }

    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

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

void TwoWire::doneCallback(void *context) {
    k_task_t *task = (k_task_t*)context;
    
    k_event_send(task, WIRING_EVENT_TRANSIENT);
}

void TwoWire::eventCallback(void *context, uint32_t events) {
    TwoWire *self = (TwoWire *)context;
    
    if (events & STM32WB_I2C_EVENT_RECEIVE_REQUEST) {
        self->m_ev_broadcast = ((events & STM32WB_I2C_EVENT_ADDRESS_MASK) >> STM32WB_I2C_EVENT_ADDRESS_SHIFT) == 0x00;

        stm32wb_i2c_receive(self->m_i2c, &self->m_rx_data[0], WIRE_BUFFER_LENGTH);
    }

    if (events & STM32WB_I2C_EVENT_RECEIVE_DONE) {
        self->m_rx_index = 0;
        self->m_rx_count = (events & STM32WB_I2C_EVENT_COUNT_MASK) >> STM32WB_I2C_EVENT_COUNT_SHIFT;

        if (self->m_receive_callback) {
            (*self->m_receive_callback)(self->m_rx_count);
        }
    }
    
    if (events & STM32WB_I2C_EVENT_TRANSMIT_REQUEST) {
        self->m_ev_broadcast = false;

        self->m_tx_index = 0;
        self->m_tx_busy = true;

        if (self->m_request_callback) {
            (*self->m_request_callback)();
        }

        self->m_tx_busy = false;

        stm32wb_i2c_transmit(self->m_i2c, &self->m_tx_data[0], self->m_tx_index);
    }

    if (events & STM32WB_I2C_EVENT_TRANSMIT_DONE) {
        self->m_ev_broadcast = false;

        if (self->m_transmit_callback) {
            (*self->m_transmit_callback)((events & STM32WB_I2C_EVENT_COUNT_MASK) >> STM32WB_I2C_EVENT_COUNT_SHIFT);
        }
    }
}


#define TRANSACTION_QUEUE_SENTINEL ((TwoWireTransaction*)1)

struct TwoWireTransaction::TwoWireTransactionQueue TwoWireTransaction::m_queue = {
    .notify = TRANSACTION_QUEUE_SENTINEL,
    .work = K_WORK_INIT(TwoWireTransaction::notify, nullptr)
};

TwoWireTransaction::TwoWireTransaction() {
    m_busy = 0;

    m_transaction.next = nullptr;
    m_transaction.address = 0;
    m_transaction.callback = (stm32wb_i2c_done_callback_t)TwoWireTransaction::done;
    m_transaction.context = (void*)this;
    m_notify = nullptr;
    m_callback = Callback();
}

TwoWireTransaction::~TwoWireTransaction() {
}

bool TwoWireTransaction::submit(TwoWire &wire, uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *xfBuffer, size_t xfSize, void(*callback)(void), bool request, bool stopBit) {
    return submit(wire, address, txBuffer, txSize, xfBuffer, xfSize, Callback(callback), request, stopBit);
}

bool TwoWireTransaction::submit(TwoWire &wire, uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *xfBuffer, size_t xfSize, Callback callback, bool request, bool stopBit) {
    if (!txBuffer && txSize)  {
        return false;
    }

    if (!xfBuffer && xfSize)  {
        return false;
    }

    if (xfBuffer && !xfSize)  {
        return false;
    }

    if ((txSize > 65535) || (xfSize > 65536))  {
        return false;
    }

    if (wire.m_ev_address) {
        return false;
    }

    if (armv7m_atomic_casb(&m_busy, 0, 1) != 0) {
        return false;
    }

    if (m_transaction.address && (m_transaction.address != address)) {
        m_busy = 0;
        
        return false;
    }

    m_transaction.address = address;
    m_transaction.control = (stopBit ? 0 : STM32WB_I2C_CONTROL_RESTART) | (request ? 0 : STM32WB_I2C_CONTROL_DIRECTION);
    m_transaction.tx_data = txBuffer;
    m_transaction.rx_data = xfBuffer;
    m_transaction.tx_count = txSize;
    m_transaction.rx_count = xfSize;
    m_callback = callback;
    
    if (!stm32wb_i2c_submit(wire.m_i2c, &m_transaction)) {
        m_busy = 0;

        return false;
    }

    return true;
}

uint8_t TwoWireTransaction::status() {
    return m_transaction.status;
}

bool TwoWireTransaction::busy() {
    return m_busy;
}

void TwoWireTransaction::done(void *context) {
    TwoWireTransaction *self, *notify;

    self = (TwoWireTransaction *)context;

    if ((self->m_transaction.status != STM32WB_I2C_STATUS_SUCCESS) || (self->m_transaction.control & STM32WB_I2C_CONTROL_RESTART)) {
        self->m_transaction.address = 0;
    }
    
    if (!self->m_notify) {
        notify = (TwoWireTransaction*)armv7m_atomic_swap((volatile uint32_t*)&m_queue.notify, (uint32_t)self);

        self->m_notify = notify;

        if (notify == TRANSACTION_QUEUE_SENTINEL) {
            k_work_submit(&m_queue.work);
        }
    }
}

void TwoWireTransaction::notify(void *context __attribute__((unused))) {
    TwoWireTransaction *transaction, *transaction_next, *transaction_previous;
    Callback callback;
    
    if (m_queue.notify != TRANSACTION_QUEUE_SENTINEL) {
        transaction = (TwoWireTransaction*)armv7m_atomic_swap((volatile uint32_t*)&m_queue.notify, (uint32_t)TRANSACTION_QUEUE_SENTINEL);

        for (transaction_previous = TRANSACTION_QUEUE_SENTINEL; transaction != TRANSACTION_QUEUE_SENTINEL; transaction = transaction_next) {
            transaction_next = transaction->m_notify;
                
            transaction->m_notify = transaction_previous;
                
            transaction_previous = transaction;
        }

        transaction = transaction_previous;
            
        while (transaction != TRANSACTION_QUEUE_SENTINEL) {
            transaction_next = transaction->m_notify;

            callback = transaction->m_callback;
            
            transaction->m_notify = nullptr;
            transaction->m_busy = 0;
            
            callback();
                
            transaction = transaction_next;
        }
    }
}

#if WIRE_INTERFACES_COUNT > 0

extern stm32wb_i2c_t g_Wire;
extern const stm32wb_i2c_params_t g_WireParams;

TwoWire Wire(&g_Wire, &g_WireParams);

#endif

#if WIRE_INTERFACES_COUNT > 1

static stm32wb_i2c_t g_Wire1;
extern const stm32wb_i2c_params_t g_Wire1Params;

TwoWire Wire1(&g_Wire1, &g_Wire1Params);

#endif
