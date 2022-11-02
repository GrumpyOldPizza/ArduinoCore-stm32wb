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

#if !defined(_STM32WB_SAI_H)
#define _STM32WB_SAI_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32wbxx.h"

#include "stm32wb_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    STM32WB_SAI_INSTANCE_SAI1 = 0,
    STM32WB_SAI_INSTANCE_COUNT
};

#define STM32WB_SAI_CONFIG_BLOCK_A                0x0001
#define STM32WB_SAI_CONFIG_BLOCK_B                0x0002
#define STM32WB_SAI_CONFIG_PDM_DI1                0x0004
#define STM32WB_SAI_CONFIG_PDM_DI2                0x0008
  
#define STM32WB_SAI_OPTION_FORMAT_MASK            0x00000007
#define STM32WB_SAI_OPTION_FORMAT_SHIFT           0
#define STM32WB_SAI_OPTION_FORMAT_I2S             0x00000000
#define STM32WB_SAI_OPTION_FORMAT_LEFT_JUSTIFIED  0x00000001
#define STM32WB_SAI_OPTION_FORMAT_RIGHT_JUSTIFIED 0x00000002
#define STM32WB_SAI_OPTION_FORMAT_PDM             0x00000004
#define STM32WB_SAI_OPTION_DIRECTION_MASK         0x00000030
#define STM32WB_SAI_OPTION_DIRECTION_SHIFT        4
#define STM32WB_SAI_OPTION_DIRECTION_TRANSMIT     0x00000010
#define STM32WB_SAI_OPTION_DIRECTION_RECEIVE      0x00000020
#define STM32WB_SAI_OPTION_SIZE_MASK              0x00000040
#define STM32WB_SAI_OPTION_SIZE_SHIFT             6
#define STM32WB_SAI_OPTION_SIZE_16                0x00000000
#define STM32WB_SAI_OPTION_SIZE_32                0x00000040

#define STM32WB_SAI_EVENT_TRANSMIT                0x00000001
#define STM32WB_SAI_EVENT_RECEIVE                 0x00000002
#define STM32WB_SAI_EVENT_STOP                    0x00000004

typedef void (*stm32wb_sai_event_callback_t)(void *context, uint32_t events);

#define STM32WB_SAI_STATE_NONE                 0
#define STM32WB_SAI_STATE_INIT                 1
#define STM32WB_SAI_STATE_NOT_READY            2
#define STM32WB_SAI_STATE_READY                3
#define STM32WB_SAI_STATE_TRANSMIT             4
#define STM32WB_SAI_STATE_RECEIVE              5

typedef struct _stm32wb_sai_pins_t {
    uint16_t                     sck;
    uint16_t                     sd;
    uint16_t                     fs;
    uint16_t                     mck;
} stm32wb_sai_pins_t;

typedef struct _stm32wb_sai_params_t {
    uint8_t                      instance;
    uint8_t                      priority;
    uint16_t                     dma;
    uint16_t                     config;                     
    stm32wb_sai_pins_t           pins;
} stm32wb_sai_params_t;

typedef struct _stm32wb_sai_t {
    SAI_Block_TypeDef            *SAIx;
    volatile uint8_t             state;
    uint8_t                      instance;
    uint16_t                     config;
    stm32wb_sai_pins_t           pins;
    uint8_t                      interrupt;
    uint8_t                      priority;
    uint16_t                     dma;
    volatile uint8_t             busy;
    uint8_t                      width;
    uint16_t                     option;
    uint32_t                     cr1;
    uint32_t                     cr2;
    uint32_t                     frcr;
    uint32_t                     slotr;
    stm32wb_sai_event_callback_t ev_callback;
    void                         *ev_context;
} stm32wb_sai_t;

extern bool stm32wb_sai_create(stm32wb_sai_t *sai, const stm32wb_sai_params_t *params);
extern bool stm32wb_sai_destroy(stm32wb_sai_t *sai);
extern bool stm32wb_sai_enable(stm32wb_sai_t *sai, uint32_t clock, uint32_t width, uint32_t option, stm32wb_sai_event_callback_t callback, void *context);
extern bool stm32wb_sai_disable(stm32wb_sai_t *sai);
extern bool stm32wb_sai_transmit(stm32wb_sai_t *sai, const uint8_t *tx_data, uint16_t tx_count);
extern bool stm32wb_sai_receive(stm32wb_sai_t *sai, uint8_t *rx_data, uint16_t rx_count);

#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_SAI_H */
