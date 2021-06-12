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
#include "PDM.h"
#include "wiring_private.h"

#define PDM_STATE_NONE             0
#define PDM_STATE_RECEIVE          1

PDMClass::PDMClass(struct _stm32wb_sai_t *sai, const struct _stm32wb_sai_params_t *params, volatile uint8_t *pdm_data, volatile uint8_t *cic_data, volatile uint8_t *hbf_data, volatile uint8_t *fir_data)
{
    m_sai = sai;

    stm32wb_sai_create(sai, params);

    m_receive_callback = Callback(__wakeupCallback);

    Callback work_callback = Callback(&PDMClass::workCallback, this);

    k_work_create(&m_work, work_callback.callback(), work_callback.context());
    
    m_state = PDM_STATE_NONE;

    m_pdm_data[0] = pdm_data;
    m_pdm_data[1] = m_pdm_data[0] + PDM_BUFFER_SIZE;

    m_cic_data[0] = cic_data;
    m_cic_data[1] = m_cic_data[0] + CIC_BUFFER_SIZE;
    m_cic_data[2] = m_cic_data[1] + CIC_BUFFER_SIZE;
    m_cic_data[3] = m_cic_data[2] + CIC_BUFFER_SIZE;

    m_hbf_data = hbf_data;

    m_fir_data[0] = fir_data;
    m_fir_data[1] = m_fir_data[0] + FIR_BUFFER_SIZE;

    m_fir_state[0].iir_gain = m_fir_state[1].iir_gain = (int32_t)(1.0f * (float)ARMV7M_PDM_GAIN_SCALE);
}

int PDMClass::begin(int channels, long sampleRate)
{
    uint32_t width;
    int32_t a1, b0, b1;
    uint8_t cic_head;

    if (m_state != PDM_STATE_NONE) {
        return 0;
    }

    switch (channels) {
    case 1:
        width = 8;
        break;
    case 2:
        width = 16;
        break;
    default:
        return 0;
    }

    switch (sampleRate) {
    case 8000:
        a1 = (int32_t)(  0.98055532 * (float)ARMV7M_PDM_IIR_SCALE);
        b0 = (int32_t)(  0.99027766 * (float)ARMV7M_PDM_IIR_SCALE);
        b1 = (int32_t)( -0.99027766 * (float)ARMV7M_PDM_IIR_SCALE);
        break;
    case 16000:
        a1 = (int32_t)(  0.99023040 * (float)ARMV7M_PDM_IIR_SCALE);
        b0 = (int32_t)(  0.99511520 * (float)ARMV7M_PDM_IIR_SCALE);
        b1 = (int32_t)( -0.99511520 * (float)ARMV7M_PDM_IIR_SCALE);
        break;
    case 24000:
        a1 = (int32_t)(  0.99347634 * (float)ARMV7M_PDM_IIR_SCALE);
        b0 = (int32_t)(  0.99673817 * (float)ARMV7M_PDM_IIR_SCALE);
        b1 = (int32_t)( -0.99673817 * (float)ARMV7M_PDM_IIR_SCALE);
        break;
     case 32000:
        a1 = (int32_t)(  0.99510327 * (float)ARMV7M_PDM_IIR_SCALE);
        b0 = (int32_t)(  0.99755164 * (float)ARMV7M_PDM_IIR_SCALE);
        b1 = (int32_t)( -0.99755164 * (float)ARMV7M_PDM_IIR_SCALE);
        break;
    case 44100:
        a1 = (int32_t)(  0.99644443 * (float)ARMV7M_PDM_IIR_SCALE);
        b0 = (int32_t)(  0.99822222 * (float)ARMV7M_PDM_IIR_SCALE);
        b1 = (int32_t)( -0.99822222 * (float)ARMV7M_PDM_IIR_SCALE);
        break;
    case 48000:
        a1 = (int32_t)(  0.99673285 * (float)ARMV7M_PDM_IIR_SCALE);
        b0 = (int32_t)(  0.99836643 * (float)ARMV7M_PDM_IIR_SCALE);
        b1 = (int32_t)( -0.99836643 * (float)ARMV7M_PDM_IIR_SCALE);
        break;
    default:
        return 0;
    }

    if (!stm32wb_sai_enable(m_sai, sampleRate, width, (STM32WB_SAI_OPTION_DIRECTION_RECEIVE | STM32WB_SAI_OPTION_FORMAT_PDM), (stm32wb_sai_event_callback_t)PDMClass::eventCallback, (void*)this)) {
        return 0;
    }

    m_state = PDM_STATE_RECEIVE;
    m_channels = channels;
    m_sampleRate = sampleRate;
    
    m_pdm_head = 0;
    m_pdm_tail = 0;
    m_pdm_count = 0;
    m_pdm_read = 0;
    m_pdm_write = 0;

    m_cic_busy = false;
    m_cic_head = 0;
    m_cic_tail = 0;
    m_cic_count = 0;

    memset((uint8_t*)m_fir_data[0], 0, FIR_BUFFER_SIZE);
    memset((uint8_t*)m_fir_data[1], 0, FIR_BUFFER_SIZE);

    memset(&m_cic_state, 0, sizeof(m_cic_state));
    memset(&m_hbf_state, 0, sizeof(m_hbf_state));

    m_fir_state[0].iir_y = m_fir_state[1].iir_y = 0;
    m_fir_state[0].iir_x = m_fir_state[1].iir_x = 0;
    m_fir_state[0].iir_a1 = m_fir_state[1].iir_a1 = a1;
    m_fir_state[0].iir_b0 = m_fir_state[1].iir_b0 = b0;
    m_fir_state[0].iir_b1 = m_fir_state[1].iir_b1 = b1;

    m_cic_busy = true;

    cic_head = m_cic_head;
    m_cic_head = (cic_head +1) & (CIC_BUFFER_COUNT -1);
        
    stm32wb_sai_receive(m_sai, (uint8_t*)m_cic_data[cic_head], CIC_BUFFER_SIZE);
    
    return true;
}

void PDMClass::end()
{
    if (m_state == PDM_STATE_NONE) {
        return;
    }

    stm32wb_sai_disable(m_sai);

    m_state = PDM_STATE_NONE;
}

void PDMClass::setGain(float gain)
{
    float scale;

    scale = powf(10.0f, gain * (1.0f / 20.0f));

    if (scale > 256.0) { scale = 256.0; }

    m_fir_state[0].iir_gain = m_fir_state[1].iir_gain = (int32_t)(scale * (float)ARMV7M_PDM_GAIN_SCALE);
}

int PDMClass::available()
{
    if (m_state == PDM_STATE_NONE) {
        return 0;
    }

    if (m_pdm_count == 0) {
        return 0;
    }
    
    return (PDM_BUFFER_SIZE - m_pdm_read);
}

__attribute__((optimize("O3"))) int PDMClass::peek()
{
    uint32_t pdm_read;
    uint8_t pdm_tail;
    int pdm_data;
    
    if (m_state == PDM_STATE_NONE) {
        return 0;
    }

    if (m_pdm_count == 0) {
        return 0;
    }

    pdm_tail = m_pdm_tail;
    pdm_read = m_pdm_read;

    pdm_data = *((int16_t*)((uint8_t*)m_pdm_data[pdm_tail] + pdm_read));
    
    return pdm_data;
}

__attribute__((optimize("O3"))) int PDMClass::read()
{
    uint32_t pdm_read;
    uint8_t pdm_tail;
    int pdm_data;
    
    if (m_state == PDM_STATE_NONE) {
        return 0;
    }

    if (m_pdm_count == 0) {
        return 0;
    }

    pdm_tail = m_pdm_tail;
    pdm_read = m_pdm_read;

    pdm_data = *((int16_t*)((uint8_t*)m_pdm_data[pdm_tail] + pdm_read));
    pdm_read += 2;

    if (pdm_read == PDM_BUFFER_SIZE) {
        armv7m_atomic_decb(&m_pdm_count);
            
        m_pdm_tail = (pdm_tail +1) & (PDM_BUFFER_COUNT -1);
        m_pdm_read = 0;
    } else {
        m_pdm_read = pdm_read;
    }
    
    return pdm_data;
}

__attribute__((optimize("O3"))) int PDMClass::read(void* buffer, size_t size)
{
    uint32_t pdm_read;
    uint8_t pdm_tail;
    
    if (m_state == PDM_STATE_NONE) {
        return 0;
    }

    if (m_pdm_count == 0) {
        return 0;
    }

    size &= ~1;
    
    pdm_tail = m_pdm_tail;
    pdm_read = m_pdm_read;

    if (size > (PDM_BUFFER_SIZE - pdm_read)) {
        size = (PDM_BUFFER_SIZE - pdm_read);
    }

    if (size) {
        memcpy(buffer, (uint8_t*)m_pdm_data[pdm_tail] + pdm_read, size);
        
        pdm_read += size;

        if (pdm_read == PDM_BUFFER_SIZE) {
            armv7m_atomic_decb(&m_pdm_count);
            
            m_pdm_tail = (pdm_tail +1) & (PDM_BUFFER_COUNT -1);
            m_pdm_read = 0;
        } else {
            m_pdm_read = pdm_read;
        }
    }
    
    return size;
}

void PDMClass::onReceive(Callback callback)
{
    m_receive_callback = callback ? callback : Callback(__wakeupCallback);
}

void PDMClass::workCallback()
{
    uint32_t pdm_write;
    uint8_t cic_head, cic_tail, pdm_head;
    
    if (m_cic_count) {
        cic_tail = m_cic_tail;
        m_cic_tail = (cic_tail +1) & (CIC_BUFFER_COUNT -1);

        if (m_channels == 2) {
            armv7m_pdm_cic4_x16_be_left((int16_t*)m_hbf_data, (const uint8_t*)m_cic_data[cic_tail], CIC_BUFFER_SIZE, &m_cic_state[0]);
            armv7m_pdm_cic4_x16_be_right((int16_t*)(m_hbf_data + (HBF_BUFFER_SIZE / 2)), (const uint8_t*)m_cic_data[cic_tail], CIC_BUFFER_SIZE, &m_cic_state[1]);
            armv7m_atomic_decb(&m_cic_count);
            
            if (!(cic_tail & 1)) {
                armv7m_pdm_hbf15_x2((int16_t*)(m_fir_data[0] + 63 * 2), (const int16_t*)m_hbf_data, 256, &m_hbf_state[0]);
                armv7m_pdm_hbf15_x2((int16_t*)(m_fir_data[1] + 63 * 2), (const int16_t*)(m_hbf_data + (HBF_BUFFER_SIZE / 2)), 256, &m_hbf_state[1]);
            } else {
                armv7m_pdm_hbf15_x2((int16_t*)(m_fir_data[0] + 63 * 2 + 128 * 2), (const int16_t*)m_hbf_data, 256, &m_hbf_state[0]);
                armv7m_pdm_hbf15_x2((int16_t*)(m_fir_data[1] + 63 * 2 + 128 * 2), (const int16_t*)(m_hbf_data + (HBF_BUFFER_SIZE / 2)), 256, &m_hbf_state[1]);
            }
        } else {
            armv7m_pdm_cic4_x16_be((int16_t*)m_hbf_data, (const uint8_t*)m_cic_data[cic_tail], CIC_BUFFER_SIZE, &m_cic_state[0]);
            armv7m_atomic_decb(&m_cic_count);
                
            armv7m_pdm_hbf15_x2((int16_t*)(m_fir_data[0] + 63 * 2), (const int16_t*)m_hbf_data, 512, &m_hbf_state[0]);
        }
        
        if (m_pdm_count != PDM_BUFFER_COUNT) {
            pdm_head = m_pdm_head;
            pdm_write = m_pdm_write;

            if (m_channels == 2) {
                if (cic_tail & 1) {
                    armv7m_pdm_fir64_x2_stereo((int16_t*)(m_pdm_data[pdm_head] + pdm_write), (const int16_t*)m_fir_data[0], 256, &m_fir_state[0]);
                    armv7m_pdm_fir64_x2_stereo((int16_t*)(m_pdm_data[pdm_head] + pdm_write + 2), (const int16_t*)m_fir_data[1], 256, &m_fir_state[1]);
                    
                    pdm_write += (256 * 2);
                }
            } else {
                armv7m_pdm_fir64_x2_mono((int16_t*)(m_pdm_data[pdm_head] + pdm_write), (const int16_t*)m_fir_data[0], 256, &m_fir_state[0]);
                
                pdm_write += (128 * 2);
            }
            
            if (pdm_write == PDM_BUFFER_SIZE) {
                m_pdm_write = 0;
            
                armv7m_atomic_incb(&m_pdm_count);
            
                m_pdm_head = (pdm_head +1) & (PDM_BUFFER_COUNT -1);
            
                m_receive_callback();
            } else {
                m_pdm_write = pdm_write;
            }
        }

        if (!m_cic_busy) {
            cic_head = m_cic_head;
            m_cic_head = (cic_head +1) & (CIC_BUFFER_COUNT -1);
            
            stm32wb_sai_receive(m_sai, (uint8_t*)m_cic_data[cic_head], CIC_BUFFER_SIZE);
        }
    }
    
    if (m_cic_count) {
        k_work_submit(&m_work);
    }
}

void PDMClass::eventCallback(class PDMClass *self, uint32_t events)
{
    uint8_t cic_head;

    if (events & STM32WB_SAI_EVENT_RECEIVE) {
        if (self->m_cic_busy) {
            if (armv7m_atomic_incb(&self->m_cic_count) != (CIC_BUFFER_COUNT -1)) {
                cic_head = self->m_cic_head;
                self->m_cic_head = (cic_head +1) & (CIC_BUFFER_COUNT -1);
                
                stm32wb_sai_receive(self->m_sai, (uint8_t*)self->m_cic_data[cic_head], CIC_BUFFER_SIZE);
            } else {
                self->m_cic_busy = false;
            }
        
            k_work_submit(&self->m_work);
        }
    }
}

#if PDM_INTERFACES_COUNT > 0

extern const stm32wb_sai_params_t g_PDMParams;

static __attribute__((aligned(4), section(".noinit"))) uint8_t g_PDM_Data[PDM_BUFFER_COUNT * PDM_BUFFER_SIZE];
static __attribute__((aligned(4), section(".noinit"))) uint8_t g_CIC_Data[CIC_BUFFER_COUNT * CIC_BUFFER_SIZE];
static __attribute__((aligned(4), section(".noinit"))) uint8_t g_HBF_Data[HBF_BUFFER_SIZE];
static __attribute__((aligned(4), section(".noinit"))) uint8_t g_FIR_Data[FIR_BUFFER_SIZE * 2];

static stm32wb_sai_t g_PDM;

PDMClass PDM(&g_PDM, &g_PDMParams, &g_PDM_Data[0], &g_CIC_Data[0], &g_HBF_Data[0], &g_FIR_Data[0]);

#endif
