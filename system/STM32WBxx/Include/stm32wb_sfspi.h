/*
 * Copyright (c) 2014-2024 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32WB_SFSPI_H)
#define _STM32WB_SFSPI_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32wb_spi.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct _stm32wb_sfspi_pins_t {
    uint8_t                     cs;
} stm32wb_sfspi_pins_t;

typedef struct _stm32wb_sfspi_params_t {
    stm32wb_sfspi_pins_t        pins;
} stm32wb_sfspi_params_t;

extern bool stm32wb_sfspi_initialize(stm32wb_spi_t *spi, const stm32wb_sfspi_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /*_STM32WB_SFSPI_H */
