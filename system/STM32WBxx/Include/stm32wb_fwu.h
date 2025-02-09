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

#if !defined(_STM32WB_FWU_H)
#define _STM32WB_FWU_H

#include "armv7m.h"

#ifdef __cplusplus
extern "C" {
#endif
    
#define STM32WB_FWU_STATE_READY                  0
#define STM32WB_FWU_STATE_WRITING                1
#define STM32WB_FWU_STATE_CANDIDATE              2
#define STM32WB_FWU_STATE_STAGED                 3
#define STM32WB_FWU_STATE_FAILED                 4
#define STM32WB_FWU_STATE_TRIAL                  5
#define STM32WB_FWU_STATE_REJECTED               6
#define STM32WB_FWU_STATE_UPDATED                7
  
#define STM32WB_FWU_STATUS_NO_ERROR              0
#define STM32WB_FWU_STATUS_ERR_TARGET            1
#define STM32WB_FWU_STATUS_ERR_FILE              2
#define STM32WB_FWU_STATUS_ERR_WRITE             3
#define STM32WB_FWU_STATUS_ERR_ERASE             4
#define STM32WB_FWU_STATUS_ERR_CHECK_ERASED      5
#define STM32WB_FWU_STATUS_ERR_PROGRAM           6
#define STM32WB_FWU_STATUS_ERR_VERIFY            7
#define STM32WB_FWU_STATUS_ERR_ADDRESS           8
#define STM32WB_FWU_STATUS_ERR_RESET             13
#define STM32WB_FWU_STATUS_ERR_INTERNAL          14
#define STM32WB_FWU_STATUS_BUSY                  255
  
#define STM32WB_FWU_COMPONENT_NONE               0
#define STM32WB_FWU_COMPONENT_APPLICATION        1
#define STM32WB_FWU_COMPONENT_WIRELESS           2

#define STM32WB_FWU_MODE_ACCEPT                  0
#define STM32WB_FWU_MODE_TRIAL                   1

typedef struct _stm32wb_fwu_info_t {
    uint8_t                    state;
    uint8_t                    status;
} stm32wb_fwu_info_t;

#define STM32WB_FWU_STATUS_SUCCESS               0
#define STM32WB_FWU_STATUS_FAILURE               1
  
#define STM32WB_FWU_CONTROL_ACCEPT               0x01
#define STM32WB_FWU_CONTROL_REJECT               0x02
#define STM32WB_FWU_CONTROL_CANCEL               0x04
#define STM32WB_FWU_CONTROL_CLEAN                0x08
#define STM32WB_FWU_CONTROL_START                0x10
#define STM32WB_FWU_CONTROL_WRITE                0x20
#define STM32WB_FWU_CONTROL_FINISH               0x40
#define STM32WB_FWU_CONTROL_INSTALL              0x80
  
typedef void (*stm32wb_fwu_done_callback_t)(void *context);

typedef struct _stm32wb_fwu_request_t {
    volatile uint8_t                status;
    uint8_t                         control;
    uint8_t                         component;
    uint8_t                         mode;
    uint32_t                        offset;
    const uint8_t                   *data;
    uint32_t                        size;
    stm32wb_fwu_done_callback_t     callback;
    void                            *context;
} stm32wb_fwu_request_t;
  

extern uint32_t stm32wb_fwu_state(void);
extern bool stm32wb_fwu_query(stm32wb_fwu_info_t *info);
extern bool stm32wb_fwu_request(stm32wb_fwu_request_t *request);

#define STM32WB_FWU_STATUS_OFFSET_READY            0
#define STM32WB_FWU_STATUS_OFFSET_WRITING          8
#define STM32WB_FWU_STATUS_OFFSET_CANDIDATE        16
#define STM32WB_FWU_STATUS_OFFSET_STAGED           24
#define STM32WB_FWU_STATUS_OFFSET_TRIAL            32
#define STM32WB_FWU_STATUS_OFFSET_REJECTED         40
#define STM32WB_FWU_STATUS_OFFSET_FAILED           48
#define STM32WB_FWU_STATUS_OFFSET_UPDATED          56

#define STM32WB_FWU_STATUS_OFFSET_WIRELESS         64
#define STM32WB_FWU_STATUS_OFFSET_APPLICATION      72
#define STM32WB_FWU_STATUS_OFFSET_RANDOM           80
#define STM32WB_FWU_STATUS_OFFSET_PREFIX           96
  
#define STM32WB_FWU_STATUS_OFFSET_SWAP             1024
#define STM32WB_FWU_STATUS_OFFSET_COPY             2560
    
#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_FWU_H */
