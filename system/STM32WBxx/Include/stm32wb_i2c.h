/*
 * Copyright (c) 2017-2022 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32WB_I2C_H)
#define _STM32WB_I2C_H

#include "armv7m.h"
#include "stm32wbxx.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    STM32WB_I2C_INSTANCE_I2C1  = 0,
    STM32WB_I2C_INSTANCE_I2C3,
    STM32WB_I2C_INSTANCE_COUNT
};

#define STM32WB_I2C_OPTION_MODE_MASK                 0x00000003
#define STM32WB_I2C_OPTION_MODE_SHIFT                0
#define STM32WB_I2C_OPTION_MODE_100K                 0x00000000
#define STM32WB_I2C_OPTION_MODE_400K                 0x00000001
#define STM32WB_I2C_OPTION_MODE_1000K                0x00000002
#define STM32WB_I2C_OPTION_GENERAL_CALL              0x00000004
#define STM32WB_I2C_OPTION_WAKEUP                    0x00000008
#define STM32WB_I2C_OPTION_ADDRESS_MASK              0xffff0000
#define STM32WB_I2C_OPTION_ADDRESS_SHIFT             16

#define STM32WB_I2C_EVENT_TRANSMIT_REQUEST           0x00000001
#define STM32WB_I2C_EVENT_TRANSMIT_DONE              0x00000002
#define STM32WB_I2C_EVENT_RECEIVE_REQUEST            0x00000004
#define STM32WB_I2C_EVENT_RECEIVE_DONE               0x00000008
#define STM32WB_I2C_EVENT_COUNT_MASK                 0xffffff00
#define STM32WB_I2C_EVENT_COUNT_SHIFT                8
#define STM32WB_I2C_EVENT_ADDRESS_MASK               0xffff0000
#define STM32WB_I2C_EVENT_ADDRESS_SHIFT              16

#define STM32WB_I2C_STATUS_BUSY                      0
#define STM32WB_I2C_STATUS_SUCCESS                   1
#define STM32WB_I2C_STATUS_ADDRESS_NACK              2
#define STM32WB_I2C_STATUS_DATA_NACK                 3
#define STM32WB_I2C_STATUS_FAILURE                   4
#define STM32WB_I2C_STATUS_TIMEOUT                   5

#define STM32WB_I2C_CONTROL_RESTART                  0x01
#define STM32WB_I2C_CONTROL_DIRECTION                0x02

typedef void (*stm32wb_i2c_event_callback_t)(void *context, uint32_t events);

typedef void (*stm32wb_i2c_suspend_callback_t)(void *context);

typedef void (*stm32wb_i2c_done_callback_t)(void *context);

#define STM32WB_I2C_STATE_NONE                       0
#define STM32WB_I2C_STATE_INIT                       1
#define STM32WB_I2C_STATE_NOT_READY                  2
#define STM32WB_I2C_STATE_READY                      3
#define STM32WB_I2C_STATE_SUSPENDED                  4
#define STM32WB_I2C_STATE_MASTER_STOP                5
#define STM32WB_I2C_STATE_MASTER_RESTART             6
#define STM32WB_I2C_STATE_MASTER_TRANSMIT            7
#define STM32WB_I2C_STATE_MASTER_RECEIVE             8
#define STM32WB_I2C_STATE_SLAVE_TRANSMIT             9
#define STM32WB_I2C_STATE_SLAVE_RECEIVE              10

typedef struct _stm32wb_i2c_pins_t {
    uint16_t                             scl;
    uint16_t                             sda;
} stm32wb_i2c_pins_t;

typedef struct _stm32wb_i2c_params_t {
    uint8_t                              instance;
    uint8_t                              priority;
    uint16_t                             rx_dma;
    uint16_t                             tx_dma;
    stm32wb_i2c_pins_t                   pins;
} stm32wb_i2c_params_t;

typedef struct _stm32wb_i2c_transaction_t {
    struct _stm32wb_i2c_transaction_t * volatile next;
    volatile uint8_t                     status;
    uint8_t                              control;
    uint16_t                             address;
    const uint8_t                        *tx_data;
    uint8_t                              *rx_data;
    uint16_t                             tx_count;
    uint16_t                             rx_count;
    volatile stm32wb_i2c_done_callback_t callback;
    void * volatile                      context;
} stm32wb_i2c_transaction_t;

typedef struct _stm32wb_i2c_t {
    I2C_TypeDef                          *I2C;
    volatile uint8_t                     state;
    uint8_t                              instance;
    uint8_t                              interrupt;
    uint8_t                              priority;
    uint16_t                             rx_dma;
    uint16_t                             tx_dma;
    stm32wb_i2c_pins_t                   pins;
    uint32_t                             option;
    uint32_t                             timeout;
    stm32wb_i2c_event_callback_t         ev_callback;
    void                                 *ev_context;
    stm32wb_i2c_suspend_callback_t       rq_callback;
    void                                 *rq_context;
    volatile uint8_t                     rq_sync;
    stm32wb_i2c_transaction_t            *xf_head;
    stm32wb_i2c_transaction_t            *xf_tail;
    stm32wb_i2c_transaction_t * volatile xf_submit;
    stm32wb_i2c_transaction_t            *xf_transaction;
    uint8_t                              xf_control;
    uint16_t                             xf_address;
    uint32_t                             xf_count;
    const uint8_t                        *tx_data;
    const uint8_t                        *tx_data_e;
    uint8_t                              *rx_data;
    uint8_t                              *rx_data_e;
} stm32wb_i2c_t;

extern bool stm32wb_i2c_create(stm32wb_i2c_t *i2c, const stm32wb_i2c_params_t *params);
extern bool stm32wb_i2c_destroy(stm32wb_i2c_t *i2c);
extern bool stm32wb_i2c_enable(stm32wb_i2c_t *i2c, uint32_t option, uint32_t timeout, stm32wb_i2c_event_callback_t callback, void *context);
extern bool stm32wb_i2c_disable(stm32wb_i2c_t *i2c);
extern bool stm32wb_i2c_configure(stm32wb_i2c_t *i2c, uint32_t option, uint32_t timeout);
extern bool stm32wb_i2c_suspend(stm32wb_i2c_t *i2c, stm32wb_i2c_suspend_callback_t callback, void *context);
extern void stm32wb_i2c_resume(stm32wb_i2c_t *i2c);
extern bool stm32wb_i2c_reset(stm32wb_i2c_t *i2c);
extern bool stm32wb_i2c_receive(stm32wb_i2c_t *i2c, uint8_t *rx_data, uint16_t rx_count);
extern bool stm32wb_i2c_transmit(stm32wb_i2c_t *i2c, uint8_t *tx_data, uint16_t tx_count);
extern bool stm32wb_i2c_submit(stm32wb_i2c_t *i2c, stm32wb_i2c_transaction_t *transaction);
  
#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_I2C_H */
