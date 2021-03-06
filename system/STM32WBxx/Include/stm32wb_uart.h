/*
 * Copyright (c) 2017-2020 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32WB_UART_H)
#define _STM32WB_UART_H

#include "armv7m.h"
#include "stm32wbxx.h"

#include "stm32wb_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    STM32WB_UART_INSTANCE_USART1 = 0,
    STM32WB_UART_INSTANCE_LPUART1,
    STM32WB_UART_INSTANCE_COUNT
};

#define STM32WB_UART_FIFO_SUPPORTED     1
#define STM32WB_UART_RTS_SUPPORTED      1
#define STM32WB_UART_CTS_SUPPORTED      1
#define STM32WB_UART_XONOFF_SUPPORTED   1
#define STM32WB_UART_BREAK_SUPPORTED    1
#define STM32WB_UART_SPI_SUPPORTED      1
  
#define STM32WB_UART_OPTION_STOP_MASK        0x00000001
#define STM32WB_UART_OPTION_STOP_SHIFT       0
#define STM32WB_UART_OPTION_STOP_1           0x00000000
#define STM32WB_UART_OPTION_STOP_2           0x00000001
#define STM32WB_UART_OPTION_PARITY_MASK      0x00000006
#define STM32WB_UART_OPTION_PARITY_SHIFT     1
#define STM32WB_UART_OPTION_PARITY_NONE      0x00000000
#define STM32WB_UART_OPTION_PARITY_EVEN      0x00000002
#define STM32WB_UART_OPTION_PARITY_ODD       0x00000004
#define STM32WB_UART_OPTION_DATA_SIZE_MASK   0x00000008
#define STM32WB_UART_OPTION_DATA_SIZE_SHIFT  3
#define STM32WB_UART_OPTION_DATA_SIZE_7      0x00000000
#define STM32WB_UART_OPTION_DATA_SIZE_8      0x00000008
#define STM32WB_UART_OPTION_XONOFF           0x00000010
#define STM32WB_UART_OPTION_RTS              0x00000020
#define STM32WB_UART_OPTION_CTS              0x00000040
#define STM32WB_UART_OPTION_WAKEUP           0x00000080
#define STM32WB_UART_OPTION_RX_INVERT        0x00000100
#define STM32WB_UART_OPTION_TX_INVERT        0x00000200
#define STM32WB_UART_OPTION_DATA_INVERT      0x00000400
#if (STM32WB_UART_SPI_SUPPORTED == 1)
#define STM32WB_UART_OPTION_SPI              0x00000800
#endif /* (STM32WB_UART_SPI_SUPPORTED == 1) */

#define STM32WB_UART_EVENT_NOISE             0x00000001
#define STM32WB_UART_EVENT_PARITY            0x00000002
#define STM32WB_UART_EVENT_FRAMING           0x00000004
#define STM32WB_UART_EVENT_BREAK             0x00000008
#define STM32WB_UART_EVENT_OVERRUN           0x00000010
#define STM32WB_UART_EVENT_RECEIVE           0x00000040

typedef void (*stm32wb_uart_event_callback_t)(void *context, uint32_t events);
typedef void (*stm32wb_uart_done_callback_t)(void *context);

#define STM32WB_UART_STATE_NONE              0
#define STM32WB_UART_STATE_INIT              1
#define STM32WB_UART_STATE_NOT_READY         2
#define STM32WB_UART_STATE_READY             3
#define STM32WB_UART_STATE_TRANSMIT_START    4
#define STM32WB_UART_STATE_TRANSMIT_RESTART  5
#define STM32WB_UART_STATE_TRANSMIT_DATA     6
#define STM32WB_UART_STATE_TRANSMIT_STOP     7
#define STM32WB_UART_STATE_TRANSMIT_SUSPEND  8
#define STM32WB_UART_STATE_TRANSMIT_RESUME   9
#if (STM32WB_UART_SPI_SUPPORTED == 1)
#define STM32WB_UART_STATE_DATA              10
#define STM32WB_UART_STATE_DATA_DMA          11
#endif /* (STM32WB_UART_SPI_SUPPORTED == 1) */
  
#define STM32WB_UART_STATUS_SUCCESS          0
#define STM32WB_UART_STATUS_FAILURE          1
#define STM32WB_UART_STATUS_BUSY             255

#if (STM32WB_UART_FIFO_SUPPORTED == 1)
#define USART_ISR_RXFNE USART_ISR_RXNE
#define USART_ISR_TXFNF USART_ISR_TXE
#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */
  
typedef struct _stm32wb_uart_pins_t {
    uint16_t                      rx;
    uint16_t                      tx;
    uint16_t                      rts;
    uint16_t                      cts;
} stm32wb_uart_pins_t;

typedef struct _stm32wb_uart_params_t {
    uint8_t                       instance;
    uint8_t                       priority;
    uint16_t                      rx_dma;
    uint16_t                      tx_dma;
    uint8_t                       *rx_fifo;
    uint16_t                      rx_entries;
    stm32wb_uart_pins_t           pins;
} stm32wb_uart_params_t;

typedef struct _stm32wb_uart_t {
    USART_TypeDef                 *USART;
    volatile uint8_t              state;
    uint8_t                       instance;
    uint8_t                       interrupt;
    uint8_t                       priority;
    uint16_t                      rx_dma;
    uint16_t                      tx_dma;
    stm32wb_uart_pins_t           pins;
    uint32_t                      clock;
    uint32_t                      option;
#if (STM32WB_UART_SPI_SUPPORTED == 1)
    uint16_t                      control;
    uint16_t                      mask;
#endif /* (STM32WB_UART_SPI_SUPPORTED == 1) */
    uint32_t                      cr1;
    uint32_t                      cr2;
    uint32_t                      cr3;
    uint32_t                      brr;
    volatile uint32_t             mode;
    stm32wb_uart_event_callback_t ev_callback;
    void                          *ev_context;
    uint8_t                       *rx_fifo;
    uint16_t                      rx_entries;
    uint16_t                      rx_head;
    uint16_t                      rx_tail;
    uint8_t                       *rx_data;
    uint16_t                      rx_size;
    uint16_t                      rx_read;
    uint16_t                      rx_write;
    uint16_t                      rx_event;
    volatile uint16_t             rx_count;
    uint16_t                      rx_threshold[3];
    const uint8_t * volatile      tx_data;
    const uint8_t * volatile      tx_data_e;
    volatile uint8_t * volatile   xf_status;
    stm32wb_uart_done_callback_t  xf_callback;
    void                          *xf_context;
} stm32wb_uart_t;

extern bool stm32wb_uart_create(stm32wb_uart_t *uart, const stm32wb_uart_params_t *params);
extern bool stm32wb_uart_destroy(stm32wb_uart_t *uart);
extern bool stm32wb_uart_enable(stm32wb_uart_t *uart, uint32_t baudrate, uint32_t option, uint8_t *rx_data, uint32_t rx_size, uint32_t rx_threshold[3], stm32wb_uart_event_callback_t callback, void *context);
extern bool stm32wb_uart_disable(stm32wb_uart_t *uart);
extern bool stm32wb_uart_configure(stm32wb_uart_t *uart, uint32_t baudrate, uint32_t option, uint32_t rx_threshold[3]);
extern bool stm32wb_uart_break_state(stm32wb_uart_t *uart);
extern bool stm32wb_uart_cts_state(stm32wb_uart_t *uart);
extern uint32_t stm32wb_uart_count(stm32wb_uart_t *uart);
extern uint32_t stm32wb_uart_read(stm32wb_uart_t *uart, uint8_t *rx_data, uint32_t rx_count);
extern int32_t stm32wb_uart_peek(stm32wb_uart_t *uart);
extern bool stm32wb_uart_break(stm32wb_uart_t *uart, bool onoff);
extern bool stm32wb_uart_transmit(stm32wb_uart_t *uart, const uint8_t *tx_data, uint32_t tx_count, volatile uint8_t *p_status_return, stm32wb_uart_done_callback_t callback, void *context);
  extern bool stm32wb_uart_busy(stm32wb_uart_t *uart);

#if (STM32WB_UART_SPI_SUPPORTED == 1)

#define STM32WB_UART_SPI_CONTROL_CPHA        0x00000001
#define STM32WB_UART_SPI_CONTROL_CPOL        0x00000002
#define STM32WB_UART_SPI_CONTROL_LSB_FIRST   0x00000080

extern bool stm32wb_uart_spi_block(stm32wb_uart_t *uart, uint16_t pin);
extern bool stm32wb_uart_spi_unblock(stm32wb_uart_t *uart, uint16_t pin);
extern bool stm32wb_uart_spi_acquire(stm32wb_uart_t *uart, uint32_t clock, uint32_t control);
extern bool stm32wb_uart_spi_release(stm32wb_uart_t *uart);
extern uint8_t stm32wb_uart_spi_data(stm32wb_uart_t *uart, uint8_t data);
extern uint16_t stm32wb_uart_spi_data16le(stm32wb_uart_t *uart, uint16_t data);
extern uint16_t stm32wb_uart_spi_data16be(stm32wb_uart_t *uart, uint16_t data);
extern void stm32wb_uart_spi_data_receive(stm32wb_uart_t *uart, uint8_t *rx_data, uint32_t rx_count);
extern void stm32wb_uart_spi_data_transmit(stm32wb_uart_t *uart, const uint8_t *tx_data, uint32_t tx_count);
extern void stm32wb_uart_spi_data_transfer(stm32wb_uart_t *uart, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count);
extern bool stm32wb_uart_spi_data_dma_receive(stm32wb_uart_t *uart, uint8_t *rx_data, uint32_t rx_count, volatile uint8_t *p_status_return, stm32wb_uart_done_callback_t callback, void *context);
extern bool stm32wb_uart_spi_data_dma_transmit(stm32wb_uart_t *uart, const uint8_t *tx_data, uint32_t tx_count, volatile uint8_t *p_status_return, stm32wb_uart_done_callback_t callback, void *context);
extern bool stm32wb_uart_spi_data_dma_transfer(stm32wb_uart_t *uart, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count, volatile uint8_t *p_status_return, stm32wb_uart_done_callback_t callback, void *context);
extern uint32_t stm32wb_uart_spi_data_dma_cancel(stm32wb_uart_t *uart);
extern bool stm32wb_uart_spi_data_dma_busy(stm32wb_uart_t *uart);

static inline __attribute__((optimize("O3"),always_inline)) uint8_t STM32WB_UART_SPI_READ_8(USART_TypeDef *USART)
{
    return *((volatile uint8_t*)(&USART->RDR));
}

static inline __attribute__((optimize("O3"),always_inline)) void STM32WB_UART_SPI_WRITE_8(USART_TypeDef *USART, uint8_t data)
{
    *((volatile uint8_t*)(&USART->TDR)) = data;
}
  
static inline __attribute__((optimize("O3"),always_inline)) uint8_t STM32WB_UART_SPI_DATA_8(USART_TypeDef *USART, uint8_t data)
{
    STM32WB_UART_SPI_WRITE_8(USART, data);

#if (STM32WB_UART_FIFO_SUPPORTED == 1)
    while (!(USART->ISR & USART_ISR_RXFNE))
    {
    }
#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */
    while (!(USART->ISR & USART_ISR_RXNE))
    {
    }
#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */

    return STM32WB_UART_SPI_READ_8(USART);
}

#endif /* (STM32WB_UART_SPI_SUPPORTED == 1) */
  
#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_UART_H */
