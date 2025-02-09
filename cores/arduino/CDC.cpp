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
#include "USBAPI.h"
#include "wiring_private.h"

#if defined(USBCON)

#define CDC_EVENT_RECEIVE  0x00000001
#define CDC_EVENT_TRANSMIT 0x00000002

CDC::CDC(void (*serialEventRun)(void)) {
    m_enabled = false;
    m_nonblocking = false;

    m_rx_size = CDC_RX_BUFFER_SIZE;
    m_tx_size = CDC_TX_BUFFER_SIZE;
    
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
    
    m_work = K_WORK_INIT((k_work_routine_t)&CDC::notifyRoutine, this);

    m_sem = K_SEM_INIT(0, 1);
}

void CDC::begin(unsigned long baudrate) {
    begin(baudrate, (uint8_t)SERIAL_8N1);
}

void CDC::begin(unsigned long baudrate __attribute((unused)), uint32_t config __attribute((unused))) {
    end();

    m_enabled = stm32wb_usbd_cdc_enable(&m_rx_data[0], sizeof(m_rx_data), (stm32wb_usbd_cdc_event_callback_t)CDC::eventCallback, (void*)this);
}

void CDC::end() {
    if (m_enabled) {
        flush();
        
        m_enabled = false;
        
        stm32wb_usbd_cdc_disable();
    }
}

int CDC::available() {
    return stm32wb_usbd_cdc_count();
}

int CDC::availableForWrite(void) {
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

int CDC::peek() {
    return stm32wb_usbd_cdc_peek();
}

int CDC::read() {
    uint8_t data;

    if (!stm32wb_usbd_cdc_read(&data, 1)) {
        return -1;
    }

    return data;
}

int CDC::read(uint8_t *buffer, size_t size) {
    return stm32wb_usbd_cdc_read((uint8_t*)buffer, size);
}

void CDC::flush() {
    if (!armv7m_core_is_in_interrupt()) {
        while (m_tx_busy) {
            if (k_sem_acquire(&m_sem, K_TIMEOUT_FOREVER) != K_NO_ERROR) {
                break;
            }
        }
    }
}

size_t CDC::write(const uint8_t data) {
    return write(&data, 1);
}

size_t CDC::write(const uint8_t *buffer, size_t size) {
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
                if (m_nonblocking || armv7m_core_is_in_interrupt()) {
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

                    if (tx_count > CDC_TX_PACKET_SIZE) {
                        tx_count = CDC_TX_PACKET_SIZE;
                    }
                    
                    if (tx_count) {
                        if (armv7m_atomic_casb((volatile uint8_t*)&m_tx_busy, false, true) == false) {
                            m_tx_count = tx_count;

                            if (!stm32wb_usbd_cdc_transmit(&m_tx_data[tx_read], tx_count, (stm32wb_usbd_cdc_done_callback_t)CDC::transmitCallback, (void*)this)) {
                                m_tx_count = 0;
                                m_tx_read = tx_write;
                                
                                m_tx_busy = false;

                                goto finish;
                            }
                        }
                    }
                }

                if (m_tx_busy) {
                    if (k_sem_acquire(&m_sem, K_TIMEOUT_FOREVER) != K_NO_ERROR) {
                        goto finish;
                    }
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
            
            if (tx_count > CDC_TX_PACKET_SIZE) {
                tx_count = CDC_TX_PACKET_SIZE;
            }
            
            if (tx_count) {
                if (armv7m_atomic_casb((volatile uint8_t*)&m_tx_busy, false, true) == false) {
                    m_tx_count = tx_count;
                    
                    if (!stm32wb_usbd_cdc_transmit(&m_tx_data[tx_read], tx_count, (stm32wb_usbd_cdc_done_callback_t)CDC::transmitCallback, (void*)this)) {
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

bool CDC::write(const uint8_t *buffer, size_t size, void(*callback)(void)) {
    return write(buffer, size, Callback(callback));
}

bool CDC::write(const uint8_t *buffer, size_t size, Callback callback) {
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
        if (!stm32wb_usbd_cdc_transmit(m_tx2_data, m_tx2_count, (stm32wb_usbd_cdc_done_callback_t)CDC::transmitCallback, (void*)this)) {
            m_tx2_count = 0;
            m_tx2_data = nullptr;

            m_tx_busy = false;

            success = false;
        }
    }

    return success;
}

bool CDC::busy() {
    return (m_tx2_data != nullptr);
}

void CDC::setNonBlocking(bool enabled) {
    m_nonblocking = enabled;
}

void CDC::onReceive(void(*callback)(void)) {
    m_receive_callback = Callback(callback);
}

void CDC::onReceive(Callback callback) {
    m_receive_callback = callback;
}

CDC::operator bool() {
    return (m_enabled && (stm32wb_usbd_cdc_line_state() & USB_CDC_LINE_STATE_DTR));
}

unsigned long CDC::baud() {
    stm32wb_usbd_cdc_line_coding_t line_coding;

    if (m_enabled) {
        stm32wb_usbd_cdc_line_coding(&line_coding);

        return line_coding.dwDTERate;
    }

    return STM32WB_USBD_CDC_LINE_CODING_DTE_RATE;
}

uint8_t CDC::stopbits() {
    stm32wb_usbd_cdc_line_coding_t line_coding;

    if (m_enabled) {
        stm32wb_usbd_cdc_line_coding(&line_coding);

        return line_coding.bCharFormat;
    }

    return STM32WB_USBD_CDC_LINE_CODING_CHAR_FORMAT;
}

uint8_t CDC::paritytype() {
    stm32wb_usbd_cdc_line_coding_t line_coding;

    if (m_enabled) {
        stm32wb_usbd_cdc_line_coding(&line_coding);

        return line_coding.bParityType;
    }

    return STM32WB_USBD_CDC_LINE_CODING_PARITY_TYPE;
}

uint8_t CDC::numbits() {
    stm32wb_usbd_cdc_line_coding_t line_coding;

    if (m_enabled) {
        stm32wb_usbd_cdc_line_coding(&line_coding);

        return line_coding.bDataBits;
    }

    return STM32WB_USBD_CDC_LINE_CODING_DATA_BITS;
}

bool CDC::dtr() {
    return (m_enabled && (stm32wb_usbd_cdc_line_state() & USB_CDC_LINE_STATE_DTR));
}

bool CDC::rts() {
    return (m_enabled && (stm32wb_usbd_cdc_line_state() & USB_CDC_LINE_STATE_RTS));
}

void CDC::transmitCallback(class CDC *self) {
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

            if (tx_count > CDC_TX_PACKET_SIZE) {
                tx_count = CDC_TX_PACKET_SIZE;
            }
            
            self->m_tx_count = tx_count;

            if (!stm32wb_usbd_cdc_transmit(&self->m_tx_data[tx_read], tx_count, (stm32wb_usbd_cdc_done_callback_t)CDC::transmitCallback, (void*)self)) {
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

        armv7m_atomic_or(&self->m_mask, CDC_EVENT_TRANSMIT);
        
        k_work_submit(&self->m_work);
    }

    if (self->m_tx2_count) {
        if (!stm32wb_usbd_cdc_transmit(self->m_tx2_data, self->m_tx2_count, (stm32wb_usbd_cdc_done_callback_t)CDC::transmitCallback, (void*)self)) {
            self->m_tx2_count = 0;
                    
            self->m_tx_busy = false;

            armv7m_atomic_or(&self->m_mask, CDC_EVENT_TRANSMIT);
            
            k_work_submit(&self->m_work);
        }

        return;
    }

    self->m_tx_busy = false;
}

void CDC::eventCallback(class CDC *self, uint32_t events) {
    if (events & STM32WB_USBD_CDC_EVENT_RECEIVE) {
        armv7m_atomic_or(&self->m_mask, CDC_EVENT_RECEIVE);

        k_work_submit(&self->m_work);
    }
}

void CDC::notifyRoutine(class CDC *self) {
    uint32_t events;

    events = armv7m_atomic_swap(&self->m_mask, 0);

    if (events & CDC_EVENT_RECEIVE) {
        self->m_receive_callback();
    }

    if (events & CDC_EVENT_TRANSMIT) {
        self->m_tx2_data = nullptr;
      
        self->m_transmit_callback();
    }
}

#endif
