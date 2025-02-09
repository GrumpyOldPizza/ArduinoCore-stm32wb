/*
 * Copyright (c) 2022-2023 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32WB_OTP_H)
#define _STM32WB_OTP_H

#include "armv7m.h"
#include "stm32wbxx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32WB_OTP_ID_MIN              0x00
#define STM32WB_OTP_ID_MAX              0x3e

#define STM32WB_OTP_ID_HSE_TUNE         0x00
#define STM32WB_OTP_ID_LSE_CALIBRATION  0x30
#define STM32WB_OTP_ID_LSE_COMPENSATION 0x31

typedef struct __attribute__((packed)) _stm32wb_otp_hse_tune_t {
    uint8_t   ble_address[6];
    uint8_t   hse_tune;
} stm32wb_otp_hse_tune_t;


typedef struct __attribute__((packed)) _stm32wb_otp_lse_calibration_t {
    uint8_t   clock[3];          /* scaled by 256                  */
    int16_t   temp;              /* scaled by 1e2                  */
    uint16_t  tsense;            /* tsense_data translated to 3v0  */
} stm32wb_otp_lse_calibration_t;

typedef struct __attribute__((packed)) _stm32wb_otp_lse_compensation_t {
    int16_t   temp_lo;           /* scaled by 1e2                  */
    int16_t   temp_hi;           /* scaled by 1e2                  */
    uint8_t   coeff[3];          /* scaled by 1e10, 12 bits packed */
} stm32wb_otp_lse_compensation_t;
  
extern bool stm32wb_otp_read(uint8_t id, uint8_t *data, uint32_t size, uint32_t *plength_return);
extern bool stm32wb_otp_program(uint8_t id, const uint8_t *data, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_OTA_H */
