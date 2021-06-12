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

#if !defined(_STM32WB_SPI_H)
#define _STM32WB_SPI_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32wbxx.h"

#include "stm32wb_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    STM32WB_SPI_INSTANCE_SPI1 = 0,
    STM32WB_SPI_INSTANCE_SPI2,
    STM32WB_SPI_INSTANCE_COUNT
};

#define STM32WB_SPI_CONTROL_CPHA               0x00000001
#define STM32WB_SPI_CONTROL_CPOL               0x00000002
#define STM32WB_SPI_CONTROL_LSB_FIRST          0x00000080
  
#define STM32WB_SPI_STATE_NONE                 0
#define STM32WB_SPI_STATE_INIT                 1
#define STM32WB_SPI_STATE_NOT_READY            2
#define STM32WB_SPI_STATE_READY                3
#define STM32WB_SPI_STATE_DATA                 4
#define STM32WB_SPI_STATE_DATA_DMA             5

#define STM32WB_SPI_STATUS_SUCCESS             0
#define STM32WB_SPI_STATUS_FAILURE             1
#define STM32WB_SPI_STATUS_BUSY                255

typedef struct _stm32wb_spi_pins_t {
    uint16_t                    mosi;
    uint16_t                    miso;
    uint16_t                    sck;
} stm32wb_spi_pins_t;

typedef struct _stm32wb_spi_params_t {
    uint8_t                     instance;
    uint8_t                     priority;
    uint16_t                    rx_dma;
    uint16_t                    tx_dma;
    stm32wb_spi_pins_t          pins;
} stm32wb_spi_params_t;

typedef void (*stm32wb_spi_done_callback_t)(void *context);

  typedef struct _stm32wb_spi_t {
    SPI_TypeDef                          *SPI;
    volatile uint8_t                     state;
    uint8_t                              instance;
    uint8_t                              priority;
    uint8_t                              option;
    uint16_t                             rx_dma;
    uint16_t                             tx_dma; 
    uint16_t                             rx_none;
    uint16_t                             tx_default;
    stm32wb_spi_pins_t                   pins;
    uint32_t                             clock;
    uint16_t                             control;
    uint16_t                             mask;
    uint32_t                             cr1;
    uint32_t                             cr2;
    volatile uint8_t * volatile          xf_status;
    stm32wb_spi_done_callback_t volatile xf_callback;
    void * volatile                      xf_context;
    uint8_t                              *rx_data;
} stm32wb_spi_t;

extern bool stm32wb_spi_create(stm32wb_spi_t *spi, const stm32wb_spi_params_t *params);
extern bool stm32wb_spi_destroy(stm32wb_spi_t *spi);
extern bool stm32wb_spi_enable(stm32wb_spi_t *spi);
extern bool stm32wb_spi_disable(stm32wb_spi_t *spi);
extern bool stm32wb_spi_block(stm32wb_spi_t *spi, uint16_t pin);
extern bool stm32wb_spi_unblock(stm32wb_spi_t *spi, uint16_t pin);
extern bool stm32wb_spi_acquire(stm32wb_spi_t *spi, uint32_t clock, uint32_t control);
extern bool stm32wb_spi_release(stm32wb_spi_t *spi);
extern uint8_t stm32wb_spi_data(stm32wb_spi_t *spi, uint8_t data);
extern uint16_t stm32wb_spi_data16le(stm32wb_spi_t *spi, uint16_t data);
extern uint16_t stm32wb_spi_data16be(stm32wb_spi_t *spi, uint16_t data);
extern void stm32wb_spi_data_receive(stm32wb_spi_t *spi, uint8_t *rx_data, uint32_t rx_count);
extern void stm32wb_spi_data_transmit(stm32wb_spi_t *spi, const uint8_t *tx_data, uint32_t tx_count);
extern void stm32wb_spi_data_transfer(stm32wb_spi_t *spi, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count);
extern bool stm32wb_spi_data_dma_receive(stm32wb_spi_t *spi, uint8_t *rx_data, uint32_t rx_count, volatile uint8_t *p_status_return, stm32wb_spi_done_callback_t callback, void *context);
extern bool stm32wb_spi_data_dma_transmit(stm32wb_spi_t *spi, const uint8_t *tx_data, uint32_t tx_count, volatile uint8_t *p_status_return, stm32wb_spi_done_callback_t callback, void *context);
extern bool stm32wb_spi_data_dma_transfer(stm32wb_spi_t *spi, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count, volatile uint8_t *p_status_return, stm32wb_spi_done_callback_t callback, void *context);
extern uint32_t stm32wb_spi_data_dma_cancel(stm32wb_spi_t *spi);
extern bool stm32wb_spi_data_dma_busy(stm32wb_spi_t *spi);

static inline __attribute__((optimize("O3"),always_inline)) uint8_t STM32WB_SPI_READ_8(SPI_TypeDef *SPI)
{
    return *((volatile uint8_t*)(&SPI->DR));
}

static inline __attribute__((optimize("O3"),always_inline)) void STM32WB_SPI_WRITE_8(SPI_TypeDef *SPI, uint8_t data)
{
    *((volatile uint8_t*)(&SPI->DR)) = data;
}
  
static inline __attribute__((optimize("O3"),always_inline)) uint8_t STM32WB_SPI_DATA_8(SPI_TypeDef *SPI, uint8_t data)
{
    STM32WB_SPI_WRITE_8(SPI, data);

    while (!(SPI->SR & SPI_SR_RXNE))
    {
    }

    return STM32WB_SPI_READ_8(SPI);
}

static inline __attribute__((optimize("O3"),always_inline)) void STM32WB_SPI_WRITE_16(SPI_TypeDef *SPI, uint16_t data)
{
    SPI->DR = data;
}

static inline __attribute__((optimize("O3"),always_inline)) uint16_t STM32WB_SPI_READ_16(SPI_TypeDef *SPI)
{
    return SPI->DR;
}

static inline __attribute__((optimize("O3"),always_inline)) uint16_t STM32WB_SPI_DATA_16(SPI_TypeDef *SPI, uint16_t data)
{
    STM32WB_SPI_WRITE_16(SPI, data);

    while (!(SPI->SR & SPI_SR_RXNE))
    {
    }

    return STM32WB_SPI_READ_16(SPI);
}

typedef struct _stm32wb_spi_interface_t {
    bool     (*enable)(void *device);
    bool     (*disable)(void *device);
    bool     (*block)(void *device, uint16_t pin);
    bool     (*unblock)(void *device, uint16_t pin);
    bool     (*acquire)(void *device, uint32_t clock, uint32_t control);
    bool     (*release)(void *device);
    uint8_t  (*data)(void *device, uint8_t data);
    uint16_t (*data16le)(void *device, uint16_t data);
    uint16_t (*data16be)(void *device, uint16_t data);
    void     (*data_receive)(void *device, uint8_t *rx_data, uint32_t rx_count);
    void     (*data_transmit)(void *device, const uint8_t *tx_data, uint32_t tx_count);
    void     (*data_transfer)(void *device, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count);
    bool     (*data_dma_receive)(void *device, uint8_t *rx_data, uint32_t rx_count, volatile uint8_t *p_status_return, stm32wb_spi_done_callback_t callback, void *context);
    bool     (*data_dma_transmit)(void *device, const uint8_t *tx_data, uint32_t tx_count, volatile uint8_t *p_status_return, stm32wb_spi_done_callback_t callback, void *context);
    bool     (*data_dma_transfer)(void *device, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count, volatile uint8_t *p_status_return, stm32wb_spi_done_callback_t callback, void *context);
    uint32_t (*data_dma_cancel)(void *device);
} stm32wb_spi_interface_t;

#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_SPI_H */
