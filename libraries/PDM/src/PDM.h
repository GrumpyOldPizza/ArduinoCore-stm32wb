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

#ifndef _PDM_H_INCLUDED
#define _PDM_H_INCLUDED

#include <Arduino.h>
#include "armv7m_pdm.h"

#define PDM_BUFFER_COUNT   2
#define PDM_BUFFER_SIZE    512

#define CIC_BUFFER_COUNT   4
#define CIC_BUFFER_SIZE    1024
#define HBF_BUFFER_SIZE    (512 * 2)
#define FIR_BUFFER_SIZE    ((64 + 256) * 2)

class PDMClass
{
public:
    PDMClass(struct _stm32wb_sai_t *sai, const struct _stm32wb_sai_params_t *params, volatile uint8_t *pdm_data, volatile uint8_t *cic_data, volatile uint8_t *hbf_data, volatile uint8_t *fir_data);

    int begin(int channels, long sampleRate);
    void end();

    void setGain(float gain);
    
    int available();
    int peek();
    int read();
    int read(void* buffer, size_t size);
    
    void onReceive(Callback callback);
    void onReceive(void(*callback)(void)) { onReceive(Callback(callback)); }

private:
    struct _stm32wb_sai_t *m_sai;
    volatile uint8_t m_state;
    uint8_t m_channels;
    uint32_t m_sampleRate;
    volatile uint8_t m_pdm_head;
    volatile uint8_t m_pdm_tail;
    volatile uint8_t m_pdm_count;
    volatile uint16_t m_pdm_read;
    volatile uint16_t m_pdm_write;
    volatile uint8_t *m_pdm_data[PDM_BUFFER_COUNT];

    volatile uint8_t m_cic_busy;
    volatile uint8_t m_cic_head;
    volatile uint8_t m_cic_tail;
    volatile uint8_t m_cic_count;
    volatile uint8_t *m_cic_data[CIC_BUFFER_COUNT];
    volatile uint8_t *m_hbf_data;
    volatile uint8_t *m_fir_data[2];
    
    armv7m_pdm_cic_state_t m_cic_state[2];
    armv7m_pdm_hbf_state_t m_hbf_state[2];
    armv7m_pdm_fir_state_t m_fir_state[2];
    
    Callback m_receive_callback;

    k_work_t m_work;
    
    void workCallback();
    static void eventCallback(class PDMClass *self, uint32_t events);
};

#if PDM_INTERFACES_COUNT > 0
extern PDMClass PDM;
#endif

#endif
