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

    m_enabled = false;
    m_nonblocking = false;

    m_rx_size = UART_RX_BUFFER_SIZE;
    m_tx_size = UART_TX_BUFFER_SIZE;
    
    m_tx_read = 0;
    m_tx_write = 0;
    m_tx_write_next = 0;
    m_tx_count = 0;

    m_tx_busy = false;
    m_tx_lock = false;

    m_tx2_data = nullptr;
    m_tx2_count = 0;
    
    if (serialEventRun) {
        g_serialEventRun = serialEventRun;
    }

    m_receive_callback = Callback();
    m_transmit_callback = Callback();

    m_mask = 0;
    
    m_work = K_WORK_INIT((k_work_routine_t)&Uart::notifyRoutine, this);

    m_sem = K_SEM_INIT(0, 1);

    stm32wb_uart_create(uart, params);
}

void Uart::begin(unsigned long baudrate) {
    begin(baudrate, SERIAL_8N1);
}

void Uart::begin(unsigned long baudrate, uint32_t config) {
    uint32_t option;
    
    if (!baudrate) {
        return;
    }
        
    end();

    option = config & (STM32WB_UART_OPTION_STOP_MASK | STM32WB_UART_OPTION_PARITY_MASK | STM32WB_UART_OPTION_DATA_SIZE_MASK | STM32WB_UART_OPTION_RTS | STM32WB_UART_OPTION_CTS | STM32WB_UART_OPTION_WAKEUP);

    m_enabled = (stm32wb_uart_enable(m_uart, baudrate, option, &m_rx_data[0], sizeof(m_rx_data), ~0ul, (stm32wb_uart_event_callback_t)Uart::eventCallback, (void*)this));
}

void Uart::end() {
    if (m_enabled) {
        flush();

        m_enabled = false;
        
        stm32wb_uart_disable(m_uart);
    }
}

int Uart::available() {
    return stm32wb_uart_count(m_uart);
}

int Uart::availableForWrite() {
    uint32_t tx_read, tx_write, tx_size;

    if (!m_enabled) {
        return 0;
    }

    if (m_tx2_count != 0) {
        return 0;
    }

    tx_read = m_tx_read;
    tx_write = m_tx_write_next;
    
    if (tx_write >= tx_read) {
        tx_size = ((m_tx_size - tx_write) + tx_read) - 1;
    } else {
        tx_size = (tx_read - tx_write) - 1;
    }
    
    return tx_size;
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
    if (k_task_is_in_progress()) {
        while (m_tx_busy) {
            k_sem_acquire(&m_sem, K_TIMEOUT_FOREVER);
        }
    }
}

size_t Uart::write(const uint8_t data) {
    return write(&data, 1);
}

size_t Uart::write(const uint8_t *buffer, size_t size) {
    k_task_t *self = nullptr;
    uint32_t tx_read, tx_write, tx_write_next, tx_count, tx_size;
    uint8_t tx_lock;
    size_t count;
    
    if (!m_enabled) {
        return 0;
    }

    if (size == 0) {
        return 0;
    }

    count = 0;

    if (k_task_is_in_progress()) {
        self = k_task_self();
    }
    
    tx_lock = __armv7m_atomic_swapb(&m_tx_lock, 1);
    
    do {
        do {
            if (m_tx2_count != 0) {
                goto finish;
            }

            tx_read = m_tx_read;
            tx_write = m_tx_write_next;
            
            if (tx_write >= tx_read) {
                tx_size = ((m_tx_size - tx_write) + tx_read) - 1;
            } else {
                tx_size = (tx_read - tx_write) - 1;
            }

            if (tx_size == 0) {
                if (m_nonblocking || !self) {
                    goto finish;
                }

                if (!m_tx_busy) {
                    tx_read = m_tx_read;
                    tx_write = m_tx_write;

                    if (tx_write >= tx_read) {
                        tx_count = tx_write - tx_read;
                    } else {
                        tx_count = m_tx_size - tx_read;
                    }

                    if (tx_count > UART_TX_PACKET_SIZE) {
                        tx_count = UART_TX_PACKET_SIZE;
                    }
                    
                    if (tx_count) {
                        if (armv7m_atomic_casb((volatile uint8_t*)&m_tx_busy, false, true) == false) {
                            m_tx_count = tx_count;

                            if (!stm32wb_uart_transmit(m_uart, &m_tx_data[tx_read], tx_count, (stm32wb_uart_done_callback_t)Uart::transmitCallback, (void*)this)) {
                                m_tx_count = 0;
                                m_tx_read = tx_write;

                                m_tx_busy = false;

                                goto finish;
                            }
                        }
                    }
                }

                if (m_tx_busy) {
                    k_sem_acquire(&m_sem, K_TIMEOUT_FOREVER);
                }
            }
        } while (tx_size == 0);

        if (tx_size > (m_tx_size - tx_write)) {
            tx_size = (m_tx_size - tx_write);
        }
            
        if (tx_size > (size - count)) {
            tx_size = (size - count);
        }
        
        tx_write_next = tx_write + tx_size;

        if (tx_write_next == m_tx_size) {
            tx_write_next = 0;
        }

        if (armv7m_atomic_cas(&m_tx_write_next, tx_write, tx_write_next) == tx_write) {
            memcpy(&m_tx_data[tx_write], &buffer[count], tx_size);

            count += tx_size;

            if (!tx_lock) {
                m_tx_write = tx_write_next;
            }
        }
    }
    while (count < size);

finish:
    if (!tx_lock) {
        if (!m_tx_busy) {
            tx_read = m_tx_read;
            tx_write = m_tx_write;
            
            if (tx_write >= tx_read) {
                tx_count = tx_write - tx_read;
            } else {
                tx_count = m_tx_size - tx_read;
            }
            
            if (tx_count > UART_TX_PACKET_SIZE) {
                tx_count = UART_TX_PACKET_SIZE;
            }
            
            if (tx_count) {
                if (armv7m_atomic_casb((volatile uint8_t*)&m_tx_busy, false, true) == false) {
                    m_tx_count = tx_count;
                    
                    if (!stm32wb_uart_transmit(m_uart, &m_tx_data[tx_read], tx_count, (stm32wb_uart_done_callback_t)Uart::transmitCallback, (void*)this)) {
                        m_tx_count = 0;
                        m_tx_read = tx_write;
                        
                        m_tx_busy = false;
                    }
                }
            }
        }

        m_tx_lock = 0;
    }

    return count;
}

bool Uart::write(const uint8_t *buffer, size_t size, void(*callback)(void)) {
    return write(buffer, size, Callback(callback));
}

bool Uart::write(const uint8_t *buffer, size_t size, Callback callback) {
    bool success = true;
  
    if (!m_enabled) {
        return false;
    }

    if (size == 0) {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&m_tx2_data, (uint32_t)nullptr, (uint32_t)buffer) != (uint32_t)nullptr)
    {
        return false;
    }

    m_transmit_callback = callback;
    
    m_tx2_count = size;

    if (armv7m_atomic_casb((volatile uint8_t*)&m_tx_busy, false, true) == false) {
        if (!stm32wb_uart_transmit(m_uart, m_tx2_data, m_tx2_count, (stm32wb_uart_done_callback_t)Uart::transmitCallback, (void*)this)) {
            m_tx2_count = 0;
            m_tx2_data = nullptr;

            m_tx_busy = false;

            success = false;
        }
    }

    return success;
}

bool Uart::busy() {
    return (m_tx2_data != nullptr);
}

bool Uart::cts() {
    return stm32wb_uart_cts_state(m_uart);
}

void Uart::setNonBlocking(bool enable) {
    m_nonblocking = enable;
}

void Uart::onReceive(void(*callback)(void)) {
    m_receive_callback = Callback(callback);
}

void Uart::onReceive(Callback callback) {
    m_receive_callback = callback;
}

Uart::operator bool() {
    return m_enabled;
}

void Uart::transmitCallback(class Uart *self) {
    uint32_t tx_read, tx_write, tx_count;

    k_sem_release(&self->m_sem);
    
    tx_count = self->m_tx_count;
    
    if (tx_count) {
        tx_read = self->m_tx_read;
        tx_write = self->m_tx_write;
        
        tx_read += tx_count;
        if (tx_read == self->m_tx_size) {
            tx_read = 0;
        }
        
        self->m_tx_read = tx_read;
        
        if (tx_read != tx_write) {
            if (tx_write >= tx_read) {
                tx_count = tx_write - tx_read;
            } else {
                tx_count = (self->m_tx_size - tx_read);
            }

            if (tx_count > UART_TX_PACKET_SIZE) {
                tx_count = UART_TX_PACKET_SIZE;
            }
            
            self->m_tx_count = tx_count;

            if (!stm32wb_uart_transmit(self->m_uart, &self->m_tx_data[tx_read], tx_count, (stm32wb_uart_done_callback_t)Uart::transmitCallback, (void*)self)) {
                self->m_tx_count = 0;
                self->m_tx_read = tx_write;

                self->m_tx_busy = false;
            }

            return;
        }
        else
        {
            self->m_tx_count = 0;
        }
    } else {
        self->m_tx2_count = 0;

        armv7m_atomic_or(&self->m_mask, UART_EVENT_TRANSMIT);
        
        k_work_submit(&self->m_work);
    }

    if (self->m_tx2_count) {
        if (!stm32wb_uart_transmit(self->m_uart, self->m_tx2_data, self->m_tx2_count, (stm32wb_uart_done_callback_t)Uart::transmitCallback, (void*)self)) {
            self->m_tx2_count = 0;
                    
            self->m_tx_busy = false;

            armv7m_atomic_or(&self->m_mask, UART_EVENT_TRANSMIT);
            
            k_work_submit(&self->m_work);
        }

        return;
    }

    self->m_tx_busy = false;
}

void Uart::eventCallback(class Uart *self, uint32_t events) {
    if (events & STM32WB_UART_EVENT_RECEIVE) {
        armv7m_atomic_or(&self->m_mask, UART_EVENT_RECEIVE);

        k_work_submit(&self->m_work);
    }
}

void Uart::notifyRoutine(class Uart *self) {
    uint32_t events;

    events = armv7m_atomic_swap(&self->m_mask, 0);

    if (events & UART_EVENT_RECEIVE) {
        self->m_receive_callback();
    }

    if (events & UART_EVENT_TRANSMIT) {
        self->m_tx2_data = nullptr;

        self->m_transmit_callback();
    }
}
