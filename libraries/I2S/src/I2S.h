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

#ifndef _I2S_H_INCLUDED
#define _I2S_H_INCLUDED

#include <Arduino.h>

#define I2S_BUFFER_COUNT 2
#define I2S_BUFFER_SIZE  512

enum I2SMode {
    I2S_PHILIPS_MODE,
    I2S_RIGHT_JUSTIFIED_MODE,
    I2S_LEFT_JUSTIFIED_MODE
};

class I2SClass
{
public:
    I2SClass(struct _stm32wb_sai_t *sai, const struct _stm32wb_sai_params_t *params, volatile uint8_t *data);

    int begin(I2SMode mode, long sampleRate, int bitsPerSample, bool transmit = false);
    void end();

    int available();
    int peek();
    int read();
    int read(void* buffer, size_t size);

    size_t availableForWrite();
    size_t write(const void *buffer, size_t size);
    void flush();
    
    void onReceive(void(*callback)(void));
    void onReceive(Callback callback);
    void onTransmit(void(*callback)(void));
    void onTransmit(Callback callback);

private:
    struct _stm32wb_sai_t *m_sai;
    volatile uint8_t m_state;
    uint8_t m_bitsPerSample;
    uint32_t m_sampleRate;
    uint32_t m_option;
    volatile uint8_t m_i2s_busy;
    volatile uint8_t m_i2s_head;
    volatile uint8_t m_i2s_tail;
    volatile uint8_t m_i2s_count;
    volatile uint32_t m_i2s_read;
    volatile uint16_t m_i2s_size[I2S_BUFFER_COUNT];
    volatile uint8_t *m_i2s_data[I2S_BUFFER_COUNT];

    Callback m_receive_callback;
    Callback m_transmit_callback;

    k_work_t m_work;
    
    void workCallback();
    static void eventCallback(class I2SClass *self, uint32_t events);
};

#if I2S_INTERFACES_COUNT > 0
extern I2SClass I2S;
#endif

#endif
