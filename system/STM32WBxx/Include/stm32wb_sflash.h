/*
 * Copyright (c) 2020-2021 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32WB_SFLASH_H)
#define _STM32WB_SFLASH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STM32WB_SFLASH_STATUS_SUCCESS           0
#define STM32WB_SFLASH_STATUS_FAILURE           1
#define STM32WB_SFLASH_STATUS_BUSY              255

typedef struct _stm32wb_sflash_interface_t {
    bool                    (*acquire)(void);
    bool                    (*release)(void);
    bool                    (*busy)(void);
    bool                    (*suspend)(void);
    bool                    (*resume)(void);
    bool                    (*erase)(uint32_t address, volatile uint8_t *p_status_return);
    bool                    (*program)(uint32_t address, const uint8_t *data, uint32_t size, volatile uint8_t *p_status_return);
    bool                    (*read)(uint32_t address, uint8_t *data, uint32_t size);
} stm32wb_sflash_interface_t;

typedef struct _stm32wb_sflash_info_t {
    uint8_t                 mid;
    uint16_t                did;
    uint32_t                capacity;
    uint32_t                block_size;
    uint32_t                page_size;
} stm32wb_sflash_info_t;
  
extern void __stm32wb_sflash_initialize(const stm32wb_sflash_interface_t *interface, const stm32wb_sflash_info_t *info);

extern bool stm32wb_sflash_query(const stm32wb_sflash_interface_t **p_interface_return, const stm32wb_sflash_info_t **p_info_return, uint32_t *p_size_return);
extern bool stm32wb_sflash_resize(uint32_t size);
  
#ifdef __cplusplus
}
#endif

#endif /*_STM32WB_SFLASH_H */
