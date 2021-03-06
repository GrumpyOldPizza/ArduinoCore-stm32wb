/*
 * Copyright (c) 2019-2020 Thomas Roell.  All rights reserved.
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

#if !defined(_ARMV7M_PDM_H)
#define _ARMV7M_PDM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _armv7m_pdm_cic_state_t {
    int32_t        cic_accum[4];
    int32_t        cic_delay[4];
} armv7m_pdm_cic_state_t;

typedef struct _armv7m_pdm_hbf_state_t {
    int16_t        hbf_delay[14];
} armv7m_pdm_hbf_state_t;

typedef struct _armv7m_pdm_fir_state_t {
    int32_t        iir_y;
    int32_t        iir_x;
    int32_t        iir_a1;
    int32_t        iir_b0;
    int32_t        iir_b1;
    int32_t        iir_gain;
} armv7m_pdm_fir_state_t;

#define ARMV7M_PDM_IIR_SCALE  0x3fffffff /* S1Q30 */
#define ARMV7M_PDM_GAIN_SCALE 0x007fffff /* S8Q23 */
  
extern __attribute__((long_call)) void armv7m_pdm_cic4_x16_be(int16_t *output, const uint8_t *input, uint32_t count, armv7m_pdm_cic_state_t *state);
extern __attribute__((long_call)) void armv7m_pdm_cic4_x16_be_left(int16_t *output, const uint8_t *input, uint32_t count, armv7m_pdm_cic_state_t *state);
extern __attribute__((long_call)) void armv7m_pdm_cic4_x16_be_right(int16_t *output, const uint8_t *input, uint32_t count, armv7m_pdm_cic_state_t *state);
extern __attribute__((long_call)) void armv7m_pdm_cic4_x16_le(int16_t *output, const uint8_t *input, uint32_t count, armv7m_pdm_cic_state_t *state);
extern __attribute__((long_call)) void armv7m_pdm_hbf15_x2(int16_t *output, const int16_t *input, uint32_t count, armv7m_pdm_hbf_state_t *state);
extern __attribute__((long_call)) void armv7m_pdm_fir64_x2_mono(int16_t *output, const int16_t *input, uint32_t count, armv7m_pdm_fir_state_t *state);
extern __attribute__((long_call)) void armv7m_pdm_fir64_x2_stereo(int16_t *output, const int16_t *input, uint32_t count, armv7m_pdm_fir_state_t *state);
extern __attribute__((long_call)) void armv7m_pdm_fir64_x2_discard(int16_t *output, const int16_t *input, uint32_t count, armv7m_pdm_fir_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* _ARMV7M_PDM_H */
