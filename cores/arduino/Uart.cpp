/*
 * Copyright (c) 2016-2020 Thomas Roell.  All rights reserved.
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
#include "Uart.h"
#include "wiring_private.h"

#define UART_EVENT_TRANSMIT 0x00000001
#define UART_EVENT_RECEIVE  0x00000002

Uart::Uart(struct _stm32wb_uart_t *uart, const struct _stm32wb_uart_params_t *params, void (*serialEventRun)(void)) {
    m_uart = uart;

    m_tx_read = 0;
    m_tx_write = 0;
    m_tx_count = 0;
    m_tx_size = 0;

    m_tx_data2 = NULL;
    m_tx_size2 = 0;

    m_tx_busy = false;
    m_nonblocking = false;

    if (serialEventRun) {
        g_serialEventRun = serialEventRun;
    }

    stm32wb_uart_create(uart, params);

    m_transmit_callback = Callback(__emptyCallback);
    m_receive_callback = Callback(__wakeupCallback);

    k_work_create(&m_work, (k_work_routine_t)Uart::workRoutine, this);
}

void Uart::begin(unsigned long baudrate) {
    begin(baudrate, SERIAL_8N1);
}

void Uart::begin(unsigned long baudrate, uint32_t config) {
    if (!baudrate) {
        return;
    }
        
    if (m_enabled) {
        flush();
        stm32wb_uart_disable(m_uart);
    }

    m_enabled = (stm32wb_uart_enable(m_uart, baudrate, config, &m_rx_data[0], sizeof(m_rx_data), NULL, (stm32wb_uart_event_callback_t)Uart::eventCallback, (void*)this));
}

void Uart::end() {
    if (m_enabled) {
        flush();
        stm32wb_uart_disable(m_uart);

        m_enabled = 0;
    }
}

int Uart::available() {
    return stm32wb_uart_count(m_uart);
}

int Uart::availableForWrite() {
    if (!m_enabled) {
        return 0;
    }

    if (m_tx_size2 != 0) {
        return 0;
    }

    return UART_TX_BUFFER_SIZE - m_tx_count;
}

int Uart::peek() {
    return stm32wb_uart_peek(m_uart);
}

int Uart::read() {
    uint8_t data;

    if (!stm32wb_uart_read(m_uart, &data, 1)) {
        return -1;
    }

    return data;
}

int Uart::read(uint8_t *buffer, size_t size) {
    return stm32wb_uart_read(m_uart, (uint8_t*)buffer, size);
}

void Uart::flush() {
    if (armv7m_core_is_in_thread() && !k_work_is_in_progress()) {
        while (m_tx_busy) {
            __WFE();
        }
    }
}

size_t Uart::write(const uint8_t data) {
    return write(&data, 1);
}

size_t Uart::write(const uint8_t *buffer, size_t size) {
    unsigned int tx_read, tx_write, tx_count, tx_size;
    size_t count;

    if (!m_enabled) {
        return 0;
    }

    if (size == 0) {
        return 0;
    }

    if (m_tx_size2 != 0) {
        if (m_nonblocking || !armv7m_core_is_in_thread() || k_work_is_in_progress()) {
            return 0;
        }
        
        while (m_tx_size2 != 0) {
            __WFE();
        }
    }

    count = 0;

    while (count < size) {
        tx_count = UART_TX_BUFFER_SIZE - m_tx_count;

        if (tx_count == 0) {
            if (m_nonblocking || !armv7m_core_is_in_thread() || k_work_is_in_progress()) {
                break;
            }

            if (!m_tx_busy) {
                tx_size = m_tx_count;
                tx_read = m_tx_read;

                if (tx_size > (UART_TX_BUFFER_SIZE - tx_read)) {
                    tx_size = (UART_TX_BUFFER_SIZE - tx_read);
                }
                
                if (tx_size > UART_TX_PACKET_SIZE) {
                    tx_size = UART_TX_PACKET_SIZE;
                }
                
                m_tx_size = tx_size;
                m_tx_busy = true;

                if (!stm32wb_uart_transmit(m_uart, &m_tx_data[tx_read], tx_size, &m_tx_status, (stm32wb_uart_done_callback_t)Uart::transmitCallback, (void*)this)) {
                    m_tx_busy = false;

                    m_tx_size = 0;
                    m_tx_count = 0;
                    m_tx_read = m_tx_write;
                }
            }

            while (UART_TX_BUFFER_SIZE == m_tx_count) {
                __WFE();
            }

            tx_count = UART_TX_BUFFER_SIZE - m_tx_count;
        }

        tx_write = m_tx_write;

        if (tx_count > (UART_TX_BUFFER_SIZE - tx_write)) {
            tx_count = (UART_TX_BUFFER_SIZE - tx_write);
        }

        if (tx_count > (size - count)) {
            tx_count = (size - count);
        }

        memcpy(&m_tx_data[tx_write], &buffer[count], tx_count);
        count += tx_count;
      
        m_tx_write = (unsigned int)(tx_write + tx_count) & (UART_TX_BUFFER_SIZE -1);

        armv7m_atomic_add(&m_tx_count, tx_count);
    }

    if (!m_tx_busy) {
        tx_size = m_tx_count;
        
        if (tx_size) {
            tx_read = m_tx_read;

            if (tx_size > (UART_TX_BUFFER_SIZE - tx_read)) {
                tx_size = (UART_TX_BUFFER_SIZE - tx_read);
            }
            
            if (tx_size > UART_TX_PACKET_SIZE) {
                tx_size = UART_TX_PACKET_SIZE;
            }
            
            m_tx_size = tx_size;
            m_tx_busy = true;
            
            if (!stm32wb_uart_transmit(m_uart, &m_tx_data[tx_read], tx_size, &m_tx_status, (stm32wb_uart_done_callback_t)Uart::transmitCallback, (void*)this)) {
                m_tx_busy = false;

                m_tx_size = 0;
                m_tx_count = 0;
                m_tx_read = m_tx_write;
            }
        }
    }

    return count;
}

bool Uart::write(const uint8_t *buffer, size_t size, volatile uint8_t &status) {
    return write(buffer, size, status, Callback(__wakeupCallback));
}

bool Uart::write(const uint8_t *buffer, size_t size, volatile uint8_t &status, void(*callback)(void)) {
    return write(buffer, size, status, Callback(callback));
}

bool Uart::write(const uint8_t *buffer, size_t size, volatile uint8_t &status, Callback callback) {
    if (!m_enabled) {
        return false;
    }

    if (size == 0) {
        return false;
    }

    if (m_tx_size2 != 0) {
        return false;
    }

    m_tx_data2 = buffer;
    m_tx_size2 = size;
    m_tx_status2 = &status;

    m_transmit_callback = callback ? callback : Callback(__emptyCallback);
    
    if (!m_tx_busy) {
        m_tx_busy = true;

        if (!stm32wb_uart_transmit(m_uart, m_tx_data2, m_tx_size2, m_tx_status2, (stm32wb_uart_done_callback_t)Uart::transmitCallback, (void*)this)) {
            m_tx_busy = false;

            m_tx_size2 = 0;
            m_tx_data2 = NULL;

            return false;
        }
    }

    return true;
}

void Uart::setNonBlocking(bool enable) {
    m_nonblocking = enable;
}

void Uart::onReceive(Callback callback) {
    m_receive_callback = callback ? callback : Callback(__wakeupCallback);
}

void Uart::workRoutine(class Uart *self) {
    uint32_t events;

    events = armv7m_atomic_swap(&self->m_events, 0);

    if (events & UART_EVENT_TRANSMIT) {
        self->m_transmit_callback();
    }

    if (events & UART_EVENT_RECEIVE) {
        self->m_receive_callback();
    }
}

void Uart::transmitCallback(class Uart *self) {
    unsigned int tx_read, tx_size;

    self->m_tx_busy = false;

    tx_size = self->m_tx_size;

    if (tx_size != 0) {
        self->m_tx_read = (self->m_tx_read + tx_size) & (UART_TX_BUFFER_SIZE -1);
      
        armv7m_atomic_sub(&self->m_tx_count, tx_size);
      
        self->m_tx_size = 0;

        if (self->m_tx_count != 0) {
            tx_size = self->m_tx_count;
            tx_read = self->m_tx_read;

            if (tx_size > (UART_TX_BUFFER_SIZE - tx_read)) {
                tx_size = (UART_TX_BUFFER_SIZE - tx_read);
            }
          
            if (tx_size > UART_TX_PACKET_SIZE) {
                tx_size = UART_TX_PACKET_SIZE;
            }
            
            self->m_tx_size = tx_size;
            self->m_tx_busy = true;
            
            if (!stm32wb_uart_transmit(self->m_uart, &self->m_tx_data[tx_read], tx_size, &self->m_tx_status, (stm32wb_uart_done_callback_t)Uart::transmitCallback, (void*)self)) {
                self->m_tx_busy = false;

                self->m_tx_size = 0;
                self->m_tx_count = 0;
                self->m_tx_read = self->m_tx_write;

                if (self->m_tx_size2 != 0) {
                    self->m_tx_size2 = 0;
                    self->m_tx_data2 = NULL;

                    armv7m_atomic_or(&self->m_events, UART_EVENT_TRANSMIT);

                    k_work_submit(&self->m_work);
                }
            }
        } else {
            if (self->m_tx_size2 != 0) {
                self->m_tx_busy = true;

                if (!stm32wb_uart_transmit(self->m_uart, self->m_tx_data2, self->m_tx_size2, self->m_tx_status2, (stm32wb_uart_done_callback_t)Uart::transmitCallback, (void*)self)) {
                    self->m_tx_busy = false;

                    self->m_tx_size2 = 0;
                    self->m_tx_data2 = NULL;
                    
                    armv7m_atomic_or(&self->m_events, UART_EVENT_TRANSMIT);

                    k_work_submit(&self->m_work);
                }
            }
        }
    } else {
        self->m_tx_size2 = 0;
        self->m_tx_data2 = NULL;

        armv7m_atomic_or(&self->m_events, UART_EVENT_TRANSMIT);

        k_work_submit(&self->m_work);
    }
}

void Uart::eventCallback(class Uart *self, uint32_t events) {
    if (events & STM32WB_UART_EVENT_RECEIVE) {
        armv7m_atomic_or(&self->m_events, UART_EVENT_RECEIVE);

        k_work_submit(&self->m_work);
    }
}

