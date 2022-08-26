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

#pragma once

#include "HardwareSerial.h"

#define SERIAL_SBUS     (STM32WB_UART_OPTION_DATA_SIZE_8 | STM32WB_UART_OPTION_PARITY_EVEN | STM32WB_UART_OPTION_STOP_2 | STM32WB_UART_OPTION_RX_INVERT | STM32WB_UART_OPTION_TX_INVERT)
#define SERIAL_WAKEUP   (STM32WB_UART_OPTION_WAKEUP)

#define SERIAL_STATUS_SUCCESS 0
#define SERIAL_STATUS_FAILURE 1
#define SERIAL_STATUS_BUSY    2


#define UART_RX_BUFFER_SIZE 128
#define UART_TX_BUFFER_SIZE 128
#define UART_TX_PACKET_SIZE 32

#define UART_STATUS_SUCCESS 0
#define UART_STATUS_FAILURE 1
#define UART_STATUS_BUSY    2

class Uart : public HardwareSerial
{
public:
    enum FlowControl: uint32_t {
        DISABLED           = 0,
        RTS                = 1,
        CTS                = 2,
        RTS_CTS            = 3,
        XONOFF             = 4,
    };
  
    Uart(struct _stm32wb_uart_t *uart, const struct _stm32wb_uart_params_t *params, void (*serialEventRun)(void));
    void begin(unsigned long baudRate);
    void begin(unsigned long baudrate, uint32_t config);
    void end();
    virtual int available();
    virtual int availableForWrite(void);
    virtual int peek();
    virtual int read();
    virtual int read(uint8_t *buffer, size_t size);
    virtual void flush();
    virtual size_t write(const uint8_t data);
    virtual size_t write(const uint8_t *buffer, size_t size);
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool() { return true; }

    // STM32WB EXTENSION: CTS state
    bool cts();

    // STM32WB EXTENSTION: asynchronous write with callback
    bool write(const uint8_t *buffer, size_t size, volatile uint8_t &status);
    bool write(const uint8_t *buffer, size_t size, volatile uint8_t &status, void(*callback)(void));
    bool write(const uint8_t *buffer, size_t size, volatile uint8_t &status, Callback callback);
    
    // STM32WB EXTENSION: enable/disable non-blocking writes
    void setNonBlocking(bool enable);

    // STM32WB EXTENSION: configure flow control
    void setFlowControl(enum FlowControl mode);

    // STM32WB EXTENSION: asynchronous receive
    void onReceive(Callback callback);
    void onReceive(void(*callback)(void)) { onReceive(Callback(callback)); }
  
 private:
    struct _stm32wb_uart_t *m_uart;
    uint32_t m_baudrate;
    uint32_t m_option;
    uint8_t m_rx_data[UART_RX_BUFFER_SIZE];
    uint8_t m_tx_data[UART_TX_BUFFER_SIZE];
    volatile uint16_t m_tx_write;
    volatile uint16_t m_tx_read;
    volatile uint32_t m_tx_count;
    volatile uint32_t m_tx_size;

    volatile uint8_t *m_tx_status2;
    const uint8_t *m_tx_data2;
    volatile uint32_t m_tx_size2;

    bool m_enabled;
    bool m_nonblocking;
    volatile bool m_tx_busy;
    volatile uint8_t m_tx_status;
    
    volatile uint32_t m_events;
    
    Callback m_transmit_callback;
    Callback m_receive_callback;
    
    k_work_t m_work;

    static void workRoutine(class Uart *self);
    static void transmitCallback(class Uart *self);
    static void eventCallback(class Uart *self, uint32_t events);

    friend class GNSSClass;
};
