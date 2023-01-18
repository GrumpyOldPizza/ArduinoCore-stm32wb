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

class USBDeviceClass
{
public:
    USBDeviceClass();

    // USB Device API
    bool begin();
    void end();
    void start();
    void stop();
    void wakeup();

    bool attached();
    bool connected();
    bool suspended();

    void onAttach(void(*callback)(void));
    void onAttach(Callback callback);
    void onDetach(void(*callback)(void));
    void onDetach(Callback callback);
    void onConnect(void(*callback)(void));
    void onConnect(Callback callback);
    void onSuspend(void(*callback)(void));
    void onSuspend(Callback callback);
    void onResume(void(*callback)(void));
    void onResume(Callback callback);
    
private:
    volatile uint32_t m_events;
    
    Callback m_attach_callback;
    Callback m_detach_callback;
    Callback m_connect_callback;
    Callback m_suspend_callback;
    Callback m_resume_callback;

    k_work_t m_work;
    
    static void eventCallback(class USBDeviceClass *self, uint32_t events);
    static void notifyRoutine(class USBDeviceClass *self);
};

extern USBDeviceClass USBDevice;

#define CDC_RX_BUFFER_SIZE 512
#define CDC_TX_BUFFER_SIZE 512
#define CDC_TX_PACKET_SIZE 128

class CDC : public HardwareSerial
{
public:
    CDC(void (*serialEventRun)(void));
    void begin(unsigned long baudRate);
    void begin(unsigned long baudrate, uint32_t config);
    void end(void);

    virtual int available(void);
    virtual int availableForWrite(void);
    virtual int peek(void);
    virtual int read(void);
    virtual int read(uint8_t *buffer, size_t size);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buffer, size_t size);
    virtual operator bool();
    using Print::write; // pull in write(str) from Print

    // These return the settings specified by the USB host for the
    // serial port. These aren't really used, but are offered here
    // in case a sketch wants to act on these settings.
    uint32_t baud();
    uint8_t stopbits();
    uint8_t paritytype();
    uint8_t numbits();
    bool dtr();
    bool rts();

    enum {
        ONE_STOP_BIT = 0,
        ONE_AND_HALF_STOP_BIT = 1,
        TWO_STOP_BITS = 2,
    };
    enum {
        NO_PARITY = 0,
        ODD_PARITY = 1,
        EVEN_PARITY = 2,
        MARK_PARITY = 3,
        SPACE_PARITY = 4,
    };

    // STM32WB EXTENSTION: asynchronous write with callback
    bool write(const uint8_t *buffer, size_t size, void(*callback)(void));
    bool write(const uint8_t *buffer, size_t size, Callback callback);
    bool busy();
  
    // STM32WB EXTENSTION: enable/disable non-blocking writes
    void setNonBlocking(bool enable);

    // STM32WB EXTENSTION: receive callback
    void onReceive(void(*callback)(void));
    void onReceive(Callback callback);
  
private:
    bool m_enabled;
    bool m_nonblocking;
    volatile uint8_t m_tx_busy;
    volatile uint8_t m_tx_lock;

    uint8_t m_rx_data[CDC_RX_BUFFER_SIZE];
    uint8_t m_tx_data[CDC_TX_BUFFER_SIZE];
    uint32_t m_rx_size;
    uint32_t m_tx_size;
    volatile uint32_t m_tx_read;
    volatile uint32_t m_tx_write;
    volatile uint32_t m_tx_write_next;
    volatile uint32_t m_tx_count;

    const uint8_t * volatile m_tx2_data;
    volatile uint32_t m_tx2_count;
    
    volatile uint32_t m_mask;
      
    Callback m_receive_callback;
    Callback m_transmit_callback;
    
    k_work_t m_work;
    k_sem_t m_sem;

    static void transmitCallback(class CDC *self);
    static void eventCallback(class CDC *self, uint32_t events);
    static void notifyRoutine(class CDC *self);
};
