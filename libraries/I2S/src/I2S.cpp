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
#include "I2S.h"
#include "wiring_private.h"

#define I2S_STATE_NONE             0
#define I2S_STATE_TRANSMIT         1
#define I2S_STATE_RECEIVE          2

I2SClass::I2SClass(struct _stm32wb_sai_t *sai, const struct _stm32wb_sai_params_t *params, volatile uint8_t *data)
{
    m_sai = sai;

    stm32wb_sai_create(sai, params);

    m_receive_callback = Callback();
    m_transmit_callback = Callback();

    Callback work_callback = Callback(&I2SClass::workCallback, this);

    k_work_create(&m_work, work_callback.callback(), work_callback.context());
    
    m_state = I2S_STATE_NONE;

    m_i2s_data[0] = data;
    m_i2s_data[1] = data + I2S_BUFFER_SIZE;
}

int I2SClass::begin(I2SMode mode, long sampleRate, int bitsPerSample, bool transmit) {
    uint32_t option;
    uint8_t i2s_head;
        
    if (m_state != I2S_STATE_NONE) {
        return 0;
    }

    switch (mode) {
    case I2S_PHILIPS_MODE:
        option = STM32WB_SAI_OPTION_FORMAT_I2S;
        break;
    case I2S_RIGHT_JUSTIFIED_MODE:
        option = STM32WB_SAI_OPTION_FORMAT_RIGHT_JUSTIFIED;
        break;
    case I2S_LEFT_JUSTIFIED_MODE:
        option = STM32WB_SAI_OPTION_FORMAT_LEFT_JUSTIFIED;
        break;
    default:
        return 0;
    }
    
    switch (bitsPerSample) {
    case 16:
        option |= ((mode == I2S_PHILIPS_MODE) ? STM32WB_SAI_OPTION_SIZE_32 : STM32WB_SAI_OPTION_SIZE_16);
        break;
    case 24:
    case 32:
        option |= STM32WB_SAI_OPTION_SIZE_32;
        break;
    default:
        return 0;
    }

    switch (sampleRate) {
    case     0:
    case  8000:
    case 16000:
    case 24000:
    case 32000:
    case 44100:
    case 48000:
        break;
    default:
        return 0;
    }

    option |= (transmit ? STM32WB_SAI_OPTION_DIRECTION_TRANSMIT : STM32WB_SAI_OPTION_DIRECTION_RECEIVE);

    if (!stm32wb_sai_enable(m_sai, sampleRate, bitsPerSample, option, (stm32wb_sai_event_callback_t)I2SClass::eventCallback, (void*)this)) {
        return 0;
    }

    m_bitsPerSample = bitsPerSample;
    m_sampleRate = sampleRate;

    m_i2s_busy = false;
    m_i2s_head = 0;
    m_i2s_tail = 0;
    m_i2s_count = 0;
    m_i2s_read = 0;

    if (transmit) {
        m_state = I2S_STATE_TRANSMIT;
    } else {
        m_state = I2S_STATE_RECEIVE;
        m_i2s_busy = true;
        
        i2s_head = m_i2s_head;
        m_i2s_head = (i2s_head +1) & (I2S_BUFFER_COUNT -1);
        
        stm32wb_sai_receive(m_sai, (uint8_t*)m_i2s_data[i2s_head], I2S_BUFFER_SIZE);
    }
        
    return 1;
}

void I2SClass::end() {
    if (m_state != I2S_STATE_NONE) {
        stm32wb_sai_disable(m_sai);

        m_state = I2S_STATE_NONE;
    }
}

void I2SClass::flush(void) {
}

__attribute__((optimize("O3"))) int I2SClass::available() {
    if (m_state != I2S_STATE_RECEIVE) {
        return 0;
    }
    
    if (m_i2s_count == 0) {
        return 0;
    }
    
    return (I2S_BUFFER_SIZE - m_i2s_read);
}

int I2SClass::peek() {
    uint32_t i2s_read;
    uint8_t i2s_tail;
    int i2s_data;
    
    if (m_state != I2S_STATE_RECEIVE) {
        return 0;
    }

    if (m_i2s_count == 0) {
        return 0;
    }
    
    i2s_tail = m_i2s_tail;
    i2s_read = m_i2s_read;

    if (m_bitsPerSample == 16) {
        i2s_data = *((int16_t*)((uint8_t*)m_i2s_data[i2s_tail] + i2s_read));
    } else {
        i2s_data = *((int32_t*)((uint8_t*)m_i2s_data[i2s_tail] + i2s_read));
    }
    
    return i2s_data;
}

int I2SClass::read() {
    uint32_t i2s_read;
    uint8_t i2s_head, i2s_tail;
    int i2s_data;

    if (m_state != I2S_STATE_RECEIVE) {
        return 0;
    }

    if (m_i2s_count == 0) {
        return 0;
    }
    
    i2s_tail = m_i2s_tail;
    i2s_read = m_i2s_read;

    if (m_bitsPerSample == 16) {
        i2s_data = *((int16_t*)((uint8_t*)m_i2s_data[i2s_tail] + i2s_read));
        i2s_read += 2;
    } else {
        i2s_data = *((int32_t*)((uint8_t*)m_i2s_data[i2s_tail] + i2s_read));
        i2s_read += 4;
    }

    if (i2s_read == I2S_BUFFER_SIZE) {
        armv7m_atomic_decb(&m_i2s_count);
        
        m_i2s_tail = (i2s_tail +1) & (I2S_BUFFER_COUNT -1);
        m_i2s_read = 0;

        if (!m_i2s_busy) {
            m_i2s_busy = true;
            
            i2s_head = m_i2s_head;
            m_i2s_head = (i2s_head +1) & (I2S_BUFFER_COUNT -1);

            stm32wb_sai_receive(m_sai, (uint8_t*)m_i2s_data[i2s_head], I2S_BUFFER_SIZE);
        }
    } else {
        m_i2s_read = i2s_read;
    }
    
    return i2s_data;
}

__attribute__((optimize("O3"))) int I2SClass::read(void* buffer, size_t size) {
    uint32_t i2s_read;
    uint8_t i2s_head, i2s_tail;
    
    if (m_state != I2S_STATE_RECEIVE) {
        return 0;
    }

    if (m_i2s_count == 0) {
        return 0;
    }

    if (m_bitsPerSample == 16) { size &= ~1; }
    else                       { size &= ~3; }
    
    i2s_tail = m_i2s_tail;
    i2s_read = m_i2s_read;

    if (size > (I2S_BUFFER_SIZE - i2s_read)) {
        size = (I2S_BUFFER_SIZE - i2s_read);
    }

    if (size) {
        memcpy(buffer, (uint8_t*)m_i2s_data[i2s_tail] + i2s_read, size);
        
        i2s_read += size;

        if (i2s_read == I2S_BUFFER_SIZE) {
            armv7m_atomic_decb(&m_i2s_count);
            
            m_i2s_tail = (i2s_tail +1) & (I2S_BUFFER_COUNT -1);
            m_i2s_read = 0;

            if (!m_i2s_busy) {
                m_i2s_busy = true;
                
                i2s_head = m_i2s_head;
                m_i2s_head = (i2s_head +1) & (I2S_BUFFER_COUNT -1);
                
                stm32wb_sai_receive(m_sai, (uint8_t*)m_i2s_data[i2s_head], I2S_BUFFER_SIZE);
            }
        } else {
            m_i2s_read = i2s_read;
        }
    }
    
    return size;
}

size_t I2SClass::availableForWrite() {
    if (m_state != I2S_STATE_TRANSMIT) {
        return 0;
    }

    if (m_i2s_count == I2S_BUFFER_COUNT) {
        return 0;
    }
    
    return I2S_BUFFER_SIZE;
}

__attribute__((optimize("O3"))) size_t I2SClass::write(const void *buffer, size_t size) {
    uint8_t i2s_head, i2s_tail;
    
    if (m_state != I2S_STATE_TRANSMIT) {
        return 0;
    }

    if (m_i2s_count == I2S_BUFFER_COUNT) {
        return 0;
    }
    
    if (m_bitsPerSample == 16) { size &= ~1; }
    else                       { size &= ~3; }
    
    if (size > I2S_BUFFER_SIZE) {
        size = I2S_BUFFER_SIZE;
    }
    
    if (size) {
        i2s_head = m_i2s_head;
    
        memcpy((uint8_t*)m_i2s_data[i2s_head], buffer, size);
        m_i2s_size[i2s_head] = size;
        
        m_i2s_head = (i2s_head +1) & (I2S_BUFFER_COUNT -1);

        armv7m_atomic_incb(&m_i2s_count);
        
        if (!m_i2s_busy) {
            m_i2s_busy = true;
            
            i2s_tail = m_i2s_tail;
            m_i2s_tail = (i2s_tail +1) & (I2S_BUFFER_COUNT -1);
            
            stm32wb_sai_transmit(m_sai, (const uint8_t*)m_i2s_data[i2s_tail], m_i2s_size[i2s_tail]);
        }
    }
    
    return size;
}

void I2SClass::onReceive(void(*callback)(void)) {
    m_receive_callback = Callback(callback);
}

void I2SClass::onReceive(Callback callback) {
    m_receive_callback = callback;
}

void I2SClass::onTransmit(void(*callback)(void)) {
    m_transmit_callback = Callback(callback);
}

void I2SClass::onTransmit(Callback callback) {
    m_transmit_callback = callback;
}

void I2SClass::workCallback() {
    if (m_state == I2S_STATE_TRANSMIT) {
        m_transmit_callback();
    } else {
        m_receive_callback();
    }
}

void I2SClass::eventCallback(class I2SClass *self, uint32_t events) {
    uint8_t i2s_head, i2s_tail;
    
    if (events & STM32WB_SAI_EVENT_TRANSMIT) {
        if (armv7m_atomic_decb(&self->m_i2s_count) != 1) {
            i2s_tail = self->m_i2s_tail;
            self->m_i2s_tail = (i2s_tail +1) & (I2S_BUFFER_COUNT -1);
            
            stm32wb_sai_transmit(self->m_sai, (const uint8_t*)self->m_i2s_data[i2s_tail], self->m_i2s_size[i2s_tail]);
        } else {
            self->m_i2s_busy = false;
        }
        
        k_work_submit(&self->m_work);
    }

    if (events & STM32WB_SAI_EVENT_RECEIVE) {
        if (armv7m_atomic_incb(&self->m_i2s_count) != (I2S_BUFFER_COUNT -1)) {
            i2s_head = self->m_i2s_head;
            self->m_i2s_head = (i2s_head +1) & (I2S_BUFFER_COUNT -1);
            
            stm32wb_sai_receive(self->m_sai, (uint8_t*)self->m_i2s_data[i2s_head], I2S_BUFFER_SIZE);
        } else {
            self->m_i2s_busy = false;
        }
        
        k_work_submit(&self->m_work);
    }
}

#if I2S_INTERFACES_COUNT > 0

extern const stm32wb_sai_params_t g_I2SParams;

static __attribute__((aligned(4), section(".dma"))) uint8_t g_I2S_Data[I2S_BUFFER_COUNT * I2S_BUFFER_SIZE];

static stm32wb_sai_t g_I2S;

I2SClass I2S(&g_I2S, &g_I2SParams, &g_I2S_Data[0]);

#endif
