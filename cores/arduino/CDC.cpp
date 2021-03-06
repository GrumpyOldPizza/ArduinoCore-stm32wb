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
#include "USBAPI.h"
#include "wiring_private.h"

#if defined(USBCON)

#define CDC_EVENT_RECEIVE  0x00000001
#define CDC_EVENT_TRANSMIT 0x00000002

extern int (*stm32wb_stdio_put)(char, FILE*);

static int serialusb_stdio_put(char data, FILE *fp) {
    (void)fp;

    return SerialUSB.write(&data, 1);
}

CDC::CDC(void (*serialEventRun)(void)) {
    m_tx_read = 0;
    m_tx_write = 0;
    m_tx_count = 0;
    m_tx_size = 0;

    m_tx_data2 = NULL;
    m_tx_size2 = 0;

    m_enabled = false;
    m_nonblocking = false;
    m_tx_busy = false;
    
    if (serialEventRun) {
        g_serialEventRun = serialEventRun;
    }

    m_transmit_callback = Callback(__emptyCallback);
    m_receive_callback = Callback(__wakeupCallback);

    k_work_create(&m_work, (k_work_routine_t)&CDC::workRoutine, this);
}

void CDC::begin(unsigned long baudrate) {
    begin(baudrate, (uint8_t)SERIAL_8N1);
}

void CDC::begin(unsigned long baudrate __attribute((unused)), uint32_t config __attribute((unused))) {
    /* If USBD_CDC has already been enabled/initialized by STDIO, just add the notify.
     */

    if (m_enabled) {
        flush();
    }

    m_enabled = stm32wb_usbd_cdc_enable(&m_rx_data[0], sizeof(m_rx_data), (stm32wb_usbd_cdc_event_callback_t)CDC::eventCallback, (void*)this);

    if (m_enabled) {
        if (stm32wb_stdio_put == NULL) {
            stm32wb_stdio_put = serialusb_stdio_put;
        }
    }
}

void CDC::end() {
    if (m_enabled) {
        flush();

        if (stm32wb_stdio_put == serialusb_stdio_put) {
            stm32wb_stdio_put = NULL;
        }
        
        stm32wb_usbd_cdc_disable();
        
        m_enabled = false;
    }
}

int CDC::available() {
    return stm32wb_usbd_cdc_count();
}

int CDC::availableForWrite(void) {
    if (!m_enabled) {
        return 0;
    }

    if (m_tx_size2 != 0) {
        return 0;
    }

    return CDC_TX_BUFFER_SIZE - m_tx_count;
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
    if (armv7m_core_is_in_thread() && !k_work_is_in_progress()) {
        while (m_tx_busy) {
            __WFE();
        }
    }
}

size_t CDC::write(const uint8_t data) {
    return write(&data, 1);
}

size_t CDC::write(const uint8_t *buffer, size_t size) {
    unsigned int tx_read, tx_write, tx_count, tx_size;
    size_t count;

    if (!m_enabled) {
        return 0;
    }

    if (size == 0) {
        return 0;
    }

    if (m_tx_size2 != 0) {
        if (m_nonblocking || !armv7m_core_is_in_thread() || !k_work_is_in_progress()) {
            return 0;
        }
        
        while (m_tx_size2 != 0) {
            __WFE();
        }
    }

    count = 0;

    while (count < size) {
        tx_count = CDC_TX_BUFFER_SIZE - m_tx_count;

        if (tx_count == 0) {
            if (m_nonblocking || !armv7m_core_is_in_thread() || k_work_is_in_progress()) {
                break;
            }

            if (!m_tx_busy) {
                tx_size = m_tx_count;
                tx_read = m_tx_read;

                if (tx_size > (CDC_TX_BUFFER_SIZE - tx_read)) {
                    tx_size = (CDC_TX_BUFFER_SIZE - tx_read);
                }

                if (tx_size > CDC_TX_PACKET_SIZE) {
                    tx_size = CDC_TX_PACKET_SIZE;
                }
                
                m_tx_size = tx_size;
                m_tx_busy = true;

                if (!stm32wb_usbd_cdc_transmit(&m_tx_data[tx_read], tx_size, &m_tx_status, (stm32wb_usbd_cdc_done_callback_t)CDC::transmitCallback, (void*)this)) {
                    m_tx_busy = false;

                    m_tx_size = 0;
                    m_tx_count = 0;
                    m_tx_read = m_tx_write;
                }
            }

            while (CDC_TX_BUFFER_SIZE == m_tx_count) {
                __WFE();
            }

            tx_count = CDC_TX_BUFFER_SIZE - m_tx_count;
        }

        tx_write = m_tx_write;

        if (tx_count > (CDC_TX_BUFFER_SIZE - tx_write)) {
            tx_count = (CDC_TX_BUFFER_SIZE - tx_write);
        }

        if (tx_count > (size - count)) {
            tx_count = (size - count);
        }

        memcpy(&m_tx_data[tx_write], &buffer[count], tx_count);
        count += tx_count;
      
        m_tx_write = (unsigned int)(tx_write + tx_count) & (CDC_TX_BUFFER_SIZE -1);

        armv7m_atomic_add(&m_tx_count, tx_count);
    }

    if (!m_tx_busy) {
        tx_size = m_tx_count;
        
        if (tx_size) {
            tx_read = m_tx_read;
        
            if (tx_size > (CDC_TX_BUFFER_SIZE - tx_read)) {
                tx_size = (CDC_TX_BUFFER_SIZE - tx_read);
            }
            
            if (tx_size > CDC_TX_PACKET_SIZE) {
                tx_size = CDC_TX_PACKET_SIZE;
            }
            
            m_tx_size = tx_size;
            m_tx_busy = true;
            
            if (!stm32wb_usbd_cdc_transmit(&m_tx_data[tx_read], tx_size, &m_tx_status, (stm32wb_usbd_cdc_done_callback_t)CDC::transmitCallback, (void*)this)) {
                m_tx_busy = false;
                
                m_tx_size = 0;
                m_tx_count = 0;
                m_tx_read = m_tx_write;
            }
        }
    }

    return count;
}

bool CDC::write(const uint8_t *buffer, size_t size, volatile uint8_t &status) {
    return write(buffer, size, status, Callback(__wakeupCallback));
}

bool CDC::write(const uint8_t *buffer, size_t size, volatile uint8_t &status, void(*callback)(void)) {
    return write(buffer, size, status, Callback(callback));
}

bool CDC::write(const uint8_t *buffer, size_t size, volatile uint8_t &status, Callback callback) {
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

        if (!stm32wb_usbd_cdc_transmit(m_tx_data2, m_tx_size2, m_tx_status2, (stm32wb_usbd_cdc_done_callback_t)CDC::transmitCallback, (void*)this)) {
            m_tx_busy = false;

            m_tx_size2 = 0;
            m_tx_data2 = NULL;

            return false;
        }
    }

    return true;
}

void CDC::setNonBlocking(bool enabled) {
    m_nonblocking = enabled;
}

void CDC::onReceive(Callback callback) {
    m_receive_callback = callback ? callback : Callback(__wakeupCallback);
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

void CDC::workRoutine(class CDC *self) {
    uint32_t events;

    events = armv7m_atomic_swap(&self->m_events, 0);

    if (events & CDC_EVENT_RECEIVE) {
        self->m_receive_callback();
    }

    if (events & CDC_EVENT_TRANSMIT) {
        self->m_transmit_callback();
    }
}

void CDC::transmitCallback(class CDC *self) {
    unsigned int tx_read, tx_size;

    self->m_tx_busy = false;

    tx_size = self->m_tx_size;
    
    if (tx_size != 0) {
        self->m_tx_read = (self->m_tx_read + tx_size) & (CDC_TX_BUFFER_SIZE -1);
      
        armv7m_atomic_sub(&self->m_tx_count, tx_size);
      
        self->m_tx_size = 0;

        if (self->m_tx_count != 0) {
            tx_size = self->m_tx_count;
            tx_read = self->m_tx_read;
                    
            if (tx_size > (CDC_TX_BUFFER_SIZE - tx_read)) {
                tx_size = (CDC_TX_BUFFER_SIZE - tx_read);
            }
                    
            if (tx_size > CDC_TX_PACKET_SIZE) {
                tx_size = CDC_TX_PACKET_SIZE;
            }
            
            self->m_tx_size = tx_size;
            self->m_tx_busy = true;
                    
            if (!stm32wb_usbd_cdc_transmit(&self->m_tx_data[tx_read], tx_size, &self->m_tx_status, (stm32wb_usbd_cdc_done_callback_t)CDC::transmitCallback, (void*)self)) {
                self->m_tx_busy = false;

                self->m_tx_size = 0;
                self->m_tx_count = 0;
                self->m_tx_read = self->m_tx_write;

                if (self->m_tx_size2 != 0) {
                    self->m_tx_size2 = 0;
                    self->m_tx_data2 = NULL;

                    armv7m_atomic_or(&self->m_events, CDC_EVENT_TRANSMIT);
                    
                    k_work_submit(&self->m_work);
                }
            }
        } else {
            if (self->m_tx_size2 != 0) {
                self->m_tx_busy = true;

                if (!stm32wb_usbd_cdc_transmit(self->m_tx_data2, self->m_tx_size2, self->m_tx_status2, (stm32wb_usbd_cdc_done_callback_t)CDC::transmitCallback, (void*)self)) {
                    self->m_tx_busy = false;

                    self->m_tx_size2 = 0;
                    self->m_tx_data2 = NULL;
                    
                    armv7m_atomic_or(&self->m_events, CDC_EVENT_TRANSMIT);

                    k_work_submit(&self->m_work);
                }
            }
        }
    } else {
        self->m_tx_size2 = 0;
        self->m_tx_data2 = NULL;
        
        armv7m_atomic_or(&self->m_events, CDC_EVENT_TRANSMIT);

        k_work_submit(&self->m_work);
    }
}

void CDC::eventCallback(class CDC *self, uint32_t events) {
    if (events & STM32WB_USBD_CDC_EVENT_RECEIVE) {
        armv7m_atomic_or(&self->m_events, CDC_EVENT_RECEIVE);

        k_work_submit(&self->m_work);
    }
}

#endif
