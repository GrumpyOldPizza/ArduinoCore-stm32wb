/*
 * Copyright (c) 2017-2021 Thomas Roell.  All rights reserved.
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

#include "armv7m.h"
#include "stm32wbxx.h"

#include "stm32wb_gpio.h"
#include "stm32wb_uart.h"
#include "stm32wb_dma.h"
#include "stm32wb_exti.h"
#include "stm32wb_system.h"

typedef struct _stm32wb_uart_device_t {
    stm32wb_system_notify_t   notify;
    volatile uint32_t         wakeup;
    stm32wb_uart_t * volatile instances[STM32WB_UART_INSTANCE_COUNT];
} stm32wb_uart_device_t;

static stm32wb_uart_device_t stm32wb_uart_device;

#define STM32WB_UART_OPTION_SYSCLK             0x20000000
#define STM32WB_UART_OPTION_RX_DMA             0x40000000
#define STM32WB_UART_OPTION_TX_DMA             0x80000000

#define STM32WB_UART_MODE_RX_IDLE              0x0001
#define STM32WB_UART_MODE_RX_SUSPENDED         0x0002
#define STM32WB_UART_MODE_RX_BREAK             0x0004
#define STM32WB_UART_MODE_RX_SUSPEND_REQUEST   0x0008

#define STM32WB_UART_RX_DMA_OPTION               \
    (STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE |    \
     STM32WB_DMA_OPTION_EVENT_TRANSFER_HALF |    \
     STM32WB_DMA_OPTION_CIRCULAR |               \
     STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |   \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |     \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |  \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_UART_TX_DMA_OPTION               \
    (STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |   \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |     \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |  \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#if (STM32WB_UART_SPI_SUPPORTED == 1)

#define STM32WB_UART_SPI_RX_DMA_OPTION_RECEIVE    \
    (STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 |  \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_UART_SPI_RX_DMA_OPTION_TRANSMIT   \
    (STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 |  \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_UART_SPI_RX_DMA_OPTION_TRANSFER   \
    (STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 |  \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_UART_SPI_TX_DMA_OPTION_RECEIVE    \
    (STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 |  \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_UART_SPI_TX_DMA_OPTION_TRANSMIT   \
    (STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 |  \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_UART_SPI_TX_DMA_OPTION_TRANSFER   \
    (STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 |  \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#endif /* (STM32WB_UART_SPI_SUPPORTED == 1) */

static USART_TypeDef * const stm32wb_uart_xlate_USART[STM32WB_UART_INSTANCE_COUNT] = {
    USART1,
    LPUART1,
};

static const IRQn_Type stm32wb_uart_xlate_IRQn[STM32WB_UART_INSTANCE_COUNT] = {
    USART1_IRQn,
    LPUART1_IRQn,
};

static const uint32_t stm32wb_uart_xlate_IMR[STM32WB_UART_INSTANCE_COUNT] = {
    EXTI_IMR1_IM24,
    EXTI_IMR1_IM25,
};

static inline void stm32wb_uart_stop_enter(stm32wb_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;

    if (uart->option & STM32WB_UART_OPTION_RX_DMA)
    {
        USART->CR3 &= ~USART_CR3_DMAR;
    }
    
    USART->CR1 |= USART_CR1_UESM;

    USART->CR3 |= USART_CR3_WUFIE;
}

static inline void stm32wb_uart_stop_leave(stm32wb_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;

    USART->CR3 &= ~USART_CR3_WUFIE;

    if (USART->ISR & USART_ISR_WUF)
    {
        armv7m_atomic_and(&uart->mode, ~STM32WB_UART_MODE_RX_IDLE);
                            
        stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP_1);

    }

    USART->CR1 &= ~USART_CR1_UESM;

    if (uart->option & STM32WB_UART_OPTION_RX_DMA)
    {
        USART->CR3 |= USART_CR3_DMAR;
    }
}

static __attribute__((optimize("O3"))) void stm32wb_uart_notify_callback(void *context, uint32_t notify)
{
    if (notify & STM32WB_SYSTEM_NOTIFY_STOP_ENTER)
    {
        if (stm32wb_uart_device.wakeup & (1 << STM32WB_UART_INSTANCE_USART1))
        {
            stm32wb_uart_stop_enter(stm32wb_uart_device.instances[STM32WB_UART_INSTANCE_USART1]);
        }
        
        if (stm32wb_uart_device.wakeup & (1 << STM32WB_UART_INSTANCE_LPUART1))
        {
            stm32wb_uart_stop_enter(stm32wb_uart_device.instances[STM32WB_UART_INSTANCE_LPUART1]);
        }
    }

    if (notify & STM32WB_SYSTEM_NOTIFY_STOP_LEAVE)
    {
        if (stm32wb_uart_device.wakeup & (1 << STM32WB_UART_INSTANCE_USART1))
        {
            stm32wb_uart_stop_leave(stm32wb_uart_device.instances[STM32WB_UART_INSTANCE_USART1]);
        }
        
        if (stm32wb_uart_device.wakeup & (1 << STM32WB_UART_INSTANCE_LPUART1))
        {
            stm32wb_uart_stop_leave(stm32wb_uart_device.instances[STM32WB_UART_INSTANCE_LPUART1]);
        }
    }

#if (STM32WB_UART_SPI_SUPPORTED == 1)
    if (notify & STM32WB_SYSTEM_NOTIFY_CLOCKS)
    {
        if (stm32wb_uart_device.instances[STM32WB_UART_INSTANCE_USART1])
        {
            stm32wb_uart_device.instances[STM32WB_UART_INSTANCE_USART1]->control = ~0;
        }
    }
#endif /* (STM32WB_UART_SPI_SUPPORTED == 1) */
}

static __attribute__((noinline, optimize("O3"))) void stm32wb_uart_request(stm32wb_uart_t *uart)
{ 
    uint32_t mode, rx_count;
    
    if (uart->mode & STM32WB_UART_MODE_RX_SUSPEND_REQUEST)
    {
        mode = armv7m_atomic_and(&uart->mode, ~STM32WB_UART_MODE_RX_SUSPEND_REQUEST);
        
        rx_count = uart->rx_count;
        
        if (mode & STM32WB_UART_MODE_RX_SUSPENDED)
        {
            if (rx_count < uart->rx_threshold[1])
            {
                armv7m_atomic_and(&uart->mode, ~STM32WB_UART_MODE_RX_SUSPENDED);
                
#if (STM32WB_UART_RTS_SUPPORTED == 1)
                if (uart->option & STM32WB_UART_OPTION_RTS)
                {
                    stm32wb_gpio_pin_write(uart->pins.rts, 0);
                }
#endif /* STM32WB_UART_RTS_SUPPORTED == 1 */
            }
        }
        else
        {
            if (rx_count >= uart->rx_threshold[2])
            {
                armv7m_atomic_or(&uart->mode, STM32WB_UART_MODE_RX_SUSPENDED);
                
#if (STM32WB_UART_RTS_SUPPORTED == 1)
                if (uart->option & STM32WB_UART_OPTION_RTS)
                {
                    stm32wb_gpio_pin_write(uart->pins.rts, 1);
                }
#endif /* STM32WB_UART_RTS_SUPPORTED == 1 */
            }
        }
    }
}

static __attribute__((noinline, optimize("O3"))) uint32_t stm32wb_uart_dma_receive(stm32wb_uart_t *uart, uint32_t rx_head)
{ 
    uint32_t mode, events, rx_tail, rx_count, rx_count_previous, rx_write;
    uint8_t rx_data;
    
    uart->rx_head = rx_head;

    events = 0;

    rx_tail = uart->rx_tail;

    if (rx_head != rx_tail)
    {
        rx_count = rx_count_previous = uart->rx_count;
        rx_write = uart->rx_write;

        do
        {
            rx_data = uart->rx_fifo[rx_tail];
            
            rx_tail++;

            if (rx_tail == uart->rx_entries)
            {
                rx_tail = 0;
            }

            mode = uart->mode & (STM32WB_UART_MODE_RX_BREAK | STM32WB_UART_MODE_RX_IDLE);
            
            if (mode)
            {
                armv7m_atomic_and(&uart->mode, ~(STM32WB_UART_MODE_RX_BREAK | STM32WB_UART_MODE_RX_IDLE));

                if (mode & STM32WB_UART_MODE_RX_IDLE)
                {
                    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP_1);
                }
                
#if (STM32WB_UART_BREAK_SUPPORTED == 1)
                if (mode & STM32WB_UART_MODE_RX_BREAK)
                {
                    events |= STM32WB_UART_EVENT_BREAK;

                    if (rx_write != uart->rx_event)
                    {
                        uart->rx_event = rx_write;
                        
                        events |= STM32WB_UART_EVENT_RECEIVE;
                    }
                    
                    (*uart->ev_callback)(uart->ev_context, events);
                    
                    events = 0;
                }
#endif /* STM32WB_UART_BREAK_SUPPORTED == 1 */
            }
            
            if (rx_count == uart->rx_size)
            {
                if (rx_count_previous < uart->rx_size)
                {
                    events |= STM32WB_UART_EVENT_OVERRUN;
                }
            }
            else
            {
                uart->rx_data[rx_write] = rx_data;
                        
                rx_write++;
                
                if (rx_write == uart->rx_size)
                {
                    rx_write = 0;
                }
                
                rx_count++;
            }
        }
        while (rx_head != rx_tail);

        uart->rx_tail = rx_tail;
        
        if (rx_count != rx_count_previous)
        {
            uart->rx_write = rx_write;
            
            if ((rx_count_previous < uart->rx_threshold[0]) && (rx_count >= uart->rx_threshold[0]))
            {
                events |= STM32WB_UART_EVENT_RECEIVE;
            }
            
            armv7m_atomic_addh(&uart->rx_count, (rx_count - rx_count_previous));

#if (STM32WB_UART_RTS_SUPPORTED == 1)
            if (uart->option & STM32WB_UART_OPTION_RTS)
            {
                if ((rx_count >= uart->rx_threshold[2]) && !(uart->mode & STM32WB_UART_MODE_RX_SUSPENDED))
                {
                    armv7m_atomic_or(&uart->mode, STM32WB_UART_MODE_RX_SUSPEND_REQUEST);
                }
            }
#endif /* STM32WB_UART_RTS_SUPPORTED == 1 */
        }
    }
    
    return events;
}

static __attribute__((optimize("O3"))) void stm32wb_uart_dma_callback(void *context, uint32_t events)
{
    stm32wb_uart_t *uart = (stm32wb_uart_t *)context;
    
    events = stm32wb_uart_dma_receive(uart, stm32wb_dma_count(uart->rx_dma));
    
    if (events)
    {
        (*uart->ev_callback)(uart->ev_context, events);
    }

#if (STM32WB_UART_RTS_SUPPORTED == 1)
    if (uart->mode & STM32WB_UART_MODE_RX_SUSPEND_REQUEST)
    {
        stm32wb_uart_request(uart);
    }
#endif /* STM32WB_UART_RTS_SUPPORTED == 1 */
}

static __attribute__((noinline, optimize("O3"))) void stm32wb_uart_interrupt(stm32wb_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;
    uint32_t events, mode, state, usart_isr, rx_count, rx_count_previous, rx_write, rx_head;
    uint8_t rx_data;
    volatile uint8_t *p_status_return;
    stm32wb_uart_done_callback_t callback;
    void *context;

    usart_isr = USART->ISR;
    
    if (usart_isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE | USART_ISR_IDLE | USART_ISR_RXNE))
    {
        events = 0;

        if (uart->option & STM32WB_UART_OPTION_RX_DMA)
        {
            rx_head = stm32wb_dma_count(uart->rx_dma);
            
            events |= stm32wb_uart_dma_receive(uart, rx_head);

            if (usart_isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE))
            {
                USART->CR3 &= ~USART_CR3_DMAR;
                    
                USART->ICR = (USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF);

                rx_data = USART->RDR;
            
#if (STM32WB_UART_BREAK_SUPPORTED == 1)
                if ((usart_isr & USART_ISR_FE) && (rx_data == 0x00))
                {
                    if (!(uart->mode & STM32WB_UART_MODE_RX_BREAK))
                    {
                        armv7m_atomic_or(&uart->mode, STM32WB_UART_MODE_RX_BREAK);
                        
                        events |= STM32WB_UART_EVENT_BREAK;
                    }

                    if (!(uart->mode & STM32WB_UART_MODE_RX_IDLE))
                    {
                        armv7m_atomic_or(&uart->mode, STM32WB_UART_MODE_RX_IDLE);

                        stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP_1);
                    }
                }
                else
#endif /* STM32WB_UART_BREAK_SUPPORTED == 1 */
                {
                    if (usart_isr & USART_ISR_FE)
                    {
                        events |= STM32WB_UART_EVENT_FRAMING;
                    }
                    
                    if (usart_isr & USART_ISR_NE)
                    {
                        events |= STM32WB_UART_EVENT_NOISE;
                    }
                    
                    if (usart_isr & USART_ISR_PE)
                    {
                        events |= STM32WB_UART_EVENT_PARITY;
                    }
                }

                USART->CR3 |= USART_CR3_DMAR;

                if (events & (STM32WB_UART_EVENT_BREAK | STM32WB_UART_EVENT_FRAMING | STM32WB_UART_EVENT_NOISE | STM32WB_UART_EVENT_PARITY))
                {
                    if (uart->rx_write != uart->rx_event)
                    {
                        uart->rx_event = uart->rx_write;
                        
                        events |= STM32WB_UART_EVENT_RECEIVE;
                    }
                    
                    (*uart->ev_callback)(uart->ev_context, events);
                    
                    events = 0;
                }
            }
        }
        else
        {
            if (usart_isr & USART_ISR_RXNE)
            {
                rx_count = rx_count_previous = uart->rx_count;
                rx_write = uart->rx_write;
                
                do
                {
                    if (usart_isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE))
                    {
                        USART->ICR = (USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF);
                        
                        rx_data = USART->RDR;
                        
                        if ((usart_isr & USART_ISR_FE) && (rx_data == 0x00))
                        {
#if (STM32WB_UART_BREAK_SUPPORTED == 1)
                            if (!(uart->mode & STM32WB_UART_MODE_RX_BREAK))
                            {
                                armv7m_atomic_or(&uart->mode, STM32WB_UART_MODE_RX_BREAK);
                                
                                events |= STM32WB_UART_EVENT_BREAK;
                            }

                            if (!(uart->mode & STM32WB_UART_MODE_RX_IDLE))
                            {
                                armv7m_atomic_or(&uart->mode, STM32WB_UART_MODE_RX_IDLE);
                                
                                stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP_1);
                            }
                        }
                        else
#endif /* STM32WB_UART_BREAK_SUPPORTED == 1 */
                        {
                            if (usart_isr & USART_ISR_FE)
                            {
                                events |= STM32WB_UART_EVENT_FRAMING;
                            }
                            
                            if (usart_isr & USART_ISR_NE)
                            {
                                events |= STM32WB_UART_EVENT_NOISE;
                            }
                            
                            if (usart_isr & USART_ISR_PE)
                            {
                                events |= STM32WB_UART_EVENT_PARITY;
                            }
                        }
                        
                        if (events & (STM32WB_UART_EVENT_BREAK | STM32WB_UART_EVENT_FRAMING | STM32WB_UART_EVENT_NOISE | STM32WB_UART_EVENT_PARITY))
                        {
                            if (rx_write != uart->rx_event)
                            {
                                uart->rx_event = rx_write;
                                
                                events |= STM32WB_UART_EVENT_RECEIVE;
                            }
                            
                            (*uart->ev_callback)(uart->ev_context, events);
                            
                            events = 0;
                        }
                    }
                    else
                    {
                        mode = uart->mode & (STM32WB_UART_MODE_RX_BREAK | STM32WB_UART_MODE_RX_IDLE);
            
                        if (mode)
                        {
                            armv7m_atomic_and(&uart->mode, ~(STM32WB_UART_MODE_RX_BREAK | STM32WB_UART_MODE_RX_IDLE));
                            
                            if (mode & STM32WB_UART_MODE_RX_IDLE)
                            {
                                stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP_1);
                            }
                        
#if (STM32WB_UART_BREAK_SUPPORTED == 1)
                            if (mode & STM32WB_UART_MODE_RX_BREAK)
                            {
                                events |= STM32WB_UART_EVENT_BREAK;
                            
                                if (rx_write != uart->rx_event)
                                {
                                    uart->rx_event = rx_write;
                                    
                                    events |= STM32WB_UART_EVENT_RECEIVE;
                                }
                                
                                (*uart->ev_callback)(uart->ev_context, events);
                                
                                events = 0;
                            }
#endif /* STM32WB_UART_BREAK_SUPPORTED == 1 */
                        }
                        
                        rx_data = USART->RDR;
                        
                        if (rx_count == uart->rx_size)
                        {
                            if (rx_count_previous < uart->rx_size)
                            {
                                events |= STM32WB_UART_EVENT_OVERRUN;
                            }
                        }
                        else
                        {
                            uart->rx_data[rx_write] = rx_data;
                            
                            rx_write++;
                            
                            if (rx_write == uart->rx_size)
                            {
                                rx_write = 0;
                            }
                            
                            rx_count++;
                        }
                    }
                    
                    usart_isr = USART->ISR;
                }
                while (usart_isr & USART_ISR_RXNE);
                
                if (rx_count != rx_count_previous)
                {
                    uart->rx_write = rx_write;
                    
                    if ((rx_count_previous < uart->rx_threshold[0]) && (rx_count >= uart->rx_threshold[0]))
                    {
                        events |= STM32WB_UART_EVENT_RECEIVE;
                    }
                    
                    armv7m_atomic_addh(&uart->rx_count, (rx_count - rx_count_previous));
                    
#if (STM32WB_UART_RTS_SUPPORTED == 1)
                    if (uart->option & STM32WB_UART_OPTION_RTS)
                    {
                        if ((rx_count >= uart->rx_threshold[2]) && !(uart->mode & STM32WB_UART_MODE_RX_SUSPENDED))
                        {
                            armv7m_atomic_or(&uart->mode, STM32WB_UART_MODE_RX_SUSPEND_REQUEST);
                        }
                    }
#endif /* STM32WB_UART_RTS_SUPPORTED == 1 */
                }
            }
        }

        USART->ICR = USART_ICR_WUCF | USART_ICR_IDLECF | USART_ICR_ORECF;

        if (usart_isr & USART_ISR_IDLE)
        {
            if (!(uart->mode & STM32WB_UART_MODE_RX_IDLE))
            {
                armv7m_atomic_or(&uart->mode, STM32WB_UART_MODE_RX_IDLE);

                stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP_1);

                if ((uart->rx_write != uart->rx_event) && (uart->rx_count < uart->rx_threshold[0]))
                {
                    events |= STM32WB_UART_EVENT_RECEIVE;
                }
            }
        }
        
        if (usart_isr & USART_ISR_ORE)
        {
            events |= STM32WB_UART_EVENT_OVERRUN;
        }

        if (events)
        {
            uart->rx_event = uart->rx_write;

            (*uart->ev_callback)(uart->ev_context, events);
        }
    }

#if (STM32WB_UART_RTS_SUPPORTED == 1)
    if (uart->mode & STM32WB_UART_MODE_RX_SUSPEND_REQUEST)
    {
        stm32wb_uart_request(uart);
    }
#endif /* STM32WB_UART_RTS_SUPPORTED == 1  */

    state = uart->state;

    if (state >= STM32WB_UART_STATE_TRANSMIT_START)
    {
        if (state == STM32WB_UART_STATE_TRANSMIT_DATA)
        {
            usart_isr = USART->ISR;
            
            if (usart_isr & USART_ISR_TXE)
            {
                do
                {
                    USART->TDR = *uart->tx_data++;

                    usart_isr = USART->ISR;

                    if (uart->tx_data == uart->tx_data_e)
                    {
#if (STM32WB_UART_FIFO_SUPPORTED == 1)
                        USART->CR3 &= ~USART_CR3_TXFTIE;
                        USART->CR1 |= USART_CR1_TCIE;
#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */
                        USART->CR1 = (USART->CR1 & ~USART_CR1_TXEIE) | USART_CR1_TCIE;
#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */
                    
                        state = STM32WB_UART_STATE_TRANSMIT_STOP;

                        break;
                    }
                }
                while (usart_isr & USART_ISR_TXE);
            }
        }
        
        if (state == STM32WB_UART_STATE_TRANSMIT_STOP)
        {
            usart_isr = USART->ISR;

            if (usart_isr & USART_ISR_TC)
            {
                USART->CR1 &= ~USART_CR1_TCIE;

                USART->ICR = USART_ICR_TCCF;
            
                if (uart->option & STM32WB_UART_OPTION_TX_DMA)
                {
                    USART->CR3 &= ~USART_CR3_DMAT;
                
                    stm32wb_dma_stop(uart->tx_dma);
                }

                stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP_1);

                p_status_return = uart->xf_status;
                    
                armv7m_atomic_store((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL);

                callback = uart->xf_callback;
                context = uart->xf_context;

                uart->state = STM32WB_UART_STATE_READY; 
                    
                *p_status_return = STM32WB_UART_STATUS_SUCCESS;
                    
                if (callback)
                {
                    (*callback)(context);
                }

                state = uart->state;
                
                if (state != STM32WB_UART_STATE_TRANSMIT_START)
                {
                    if (uart->option & STM32WB_UART_OPTION_TX_DMA)
                    {
                        stm32wb_dma_disable(uart->tx_dma);

                        uart->option &= ~STM32WB_UART_OPTION_TX_DMA;
                    }
                }
            }
        }

        if (state == STM32WB_UART_STATE_TRANSMIT_START)
        {
            stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP_1);

            if (uart->tx_dma != STM32WB_DMA_CHANNEL_NONE)
            {
                if (!(uart->option & STM32WB_UART_OPTION_TX_DMA))
                {
                    if (stm32wb_dma_enable(uart->tx_dma, uart->priority, NULL, NULL))
                    {
                        uart->option |= STM32WB_UART_OPTION_TX_DMA;
                    }
                }
            }
        
            if (uart->option & STM32WB_UART_OPTION_TX_DMA)
            {
                stm32wb_dma_start(uart->tx_dma, (uint32_t)&USART->TDR, (uint32_t)uart->tx_data, ((uint32_t)uart->tx_data_e - (uint32_t)uart->tx_data), STM32WB_UART_TX_DMA_OPTION);

                USART->CR3 |= USART_CR3_DMAT;
                USART->CR1 |= USART_CR1_TCIE;
            
                state = STM32WB_UART_STATE_TRANSMIT_STOP;
            }
            else
            {
                do
                {
                    USART->TDR = *uart->tx_data++;

                    usart_isr = USART->ISR;

                    if (uart->tx_data == uart->tx_data_e)
                    {
                        USART->CR1 |= USART_CR1_TCIE;
                    
                        state = STM32WB_UART_STATE_TRANSMIT_STOP;

                        break;
                    }
                }
                while (usart_isr & USART_ISR_TXE);

                if (state != STM32WB_UART_STATE_TRANSMIT_STOP)
                {
#if (STM32WB_UART_FIFO_SUPPORTED == 1)
                    USART->CR3 |= USART_CR3_TXFTIE;
#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */
                    USART->CR1 |= USART_CR1_TXEIE;
#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */

                    state = STM32WB_UART_STATE_TRANSMIT_DATA;
                }
            }
        }
        
        if (state == STM32WB_UART_STATE_BREAK_ON)
        {
            stm32wb_gpio_pin_output(uart->pins.tx);

            p_status_return = uart->xf_status;
                
            armv7m_atomic_store((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL);
                
            callback = uart->xf_callback;
            context = uart->xf_context;
                
            uart->state = STM32WB_UART_STATE_BREAK; 
                
            *p_status_return = STM32WB_UART_STATUS_SUCCESS;
                
            if (callback)
            {
                (*callback)(context);
            }
            
            state = uart->state;
        }

        if (state == STM32WB_UART_STATE_BREAK_OFF)
        {
            stm32wb_gpio_pin_alternate(uart->pins.tx);

            p_status_return = uart->xf_status;
                
            armv7m_atomic_store((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL);
                
            callback = uart->xf_callback;
            context = uart->xf_context;
                
            uart->state = STM32WB_UART_STATE_READY; 
                
            *p_status_return = STM32WB_UART_STATUS_SUCCESS;
                
            if (callback)
            {
                (*callback)(context);
            }
            
            state = uart->state;
        }
            
        uart->state = state;
    }
}

static __attribute__((noinline, optimize("O3"))) void stm32wb_uart_start(stm32wb_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;
    uint32_t uartclk;
    
    if (uart->option & STM32WB_UART_OPTION_SYSCLK)
    {
        stm32wb_system_reference(STM32WB_SYSTEM_REFERENCE_USART1 << uart->instance);

        uartclk = stm32wb_system_sysclk();
    }
    else
    {
        stm32wb_system_hsi16_enable();

        if (uart->instance == STM32WB_UART_INSTANCE_LPUART1)
        {
            uartclk = 16000000u * 256u;
        }
        else
        {
            uartclk = 16000000u;
        }
    }
    
    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_USART1 + uart->instance);

    USART->CR1 = uart->cr1;
    USART->CR2 = uart->cr2;
    USART->CR3 = uart->cr3;
    USART->BRR = (uartclk + (uart->clock -1)) / uart->clock;
    USART->ICR = ~0;

    USART->CR1 |= USART_CR1_UE;

    USART->CR1 |= USART_CR1_TE;

    if (uart->option & STM32WB_UART_OPTION_WAKEUP)
    {
        if (uart->clock > 38400)
        {
            stm32wb_system_lock(STM32WB_SYSTEM_LOCK_STOP_1);
        }
        else
        {
            if (uart->instance != STM32WB_UART_INSTANCE_LPUART1)
            {
                stm32wb_system_lock(STM32WB_SYSTEM_LOCK_STOP_1);
            }
        }

        armv7m_atomic_or(&stm32wb_uart_device.wakeup, (1u << uart->instance));

        armv7m_atomic_or(&EXTI->IMR1, stm32wb_uart_xlate_IMR[uart->instance]);
    }
    
    USART->CR1 |= USART_CR1_RE;
    
    while (!(USART->ISR & USART_ISR_REACK))
    {
    }

    USART->RQR = USART_RQR_RXFRQ;

    if (uart->rx_dma != STM32WB_DMA_CHANNEL_NONE)
    {
        if (stm32wb_dma_enable(uart->rx_dma, uart->priority, stm32wb_uart_dma_callback, uart))
        {
            uart->option |= STM32WB_UART_OPTION_RX_DMA;
        }
    }

    if (uart->option & STM32WB_UART_OPTION_RX_DMA)
    {
        uart->rx_head = 0;
        uart->rx_tail = 0;

        stm32wb_dma_start(uart->rx_dma, (uint32_t)uart->rx_fifo, (uint32_t)&USART->RDR, uart->rx_entries, STM32WB_UART_RX_DMA_OPTION);
        
        USART->CR3 |= (USART_CR3_DMAR | USART_CR3_EIE);
    }
    else
    {
#if (STM32WB_UART_FIFO_SUPPORTED == 1)
        USART->CR3 |= USART_CR3_RXFTIE;
#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */
        USART->CR1 |= USART_CR1_RXNEIE;
#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */
    }
}

static __attribute__((noinline, optimize("O3"))) void stm32wb_uart_stop(stm32wb_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;

    USART->CR1 &= ~USART_CR1_TE;
    
    if (uart->option & STM32WB_UART_OPTION_RX_DMA)
    {
        stm32wb_dma_stop(uart->rx_dma);
        stm32wb_dma_disable(uart->rx_dma);
    }

    USART->CR1 &= ~USART_CR1_RE;
    
    if (uart->option & STM32WB_UART_OPTION_WAKEUP)
    {
        if (uart->clock > 38400)
        {
            stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_STOP_1);
        }
        else
        {
            if (uart->instance != STM32WB_UART_INSTANCE_LPUART1)
            {
                stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_STOP_1);
            }
        }
        
        armv7m_atomic_and(&EXTI->IMR1, ~stm32wb_uart_xlate_IMR[uart->instance]);
        
        armv7m_atomic_and(&stm32wb_uart_device.wakeup, ~(1u << uart->instance));
    }

    USART->CR1 &= ~USART_CR1_UE;
    USART->ICR = ~0;
    
    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_USART1 + uart->instance);

    if (uart->option & STM32WB_UART_OPTION_SYSCLK)
    {
        stm32wb_system_unreference(STM32WB_SYSTEM_REFERENCE_USART1 << uart->instance);
    }
    else
    {
        stm32wb_system_hsi16_disable();
    }
}

bool stm32wb_uart_create(stm32wb_uart_t *uart, const stm32wb_uart_params_t *params)
{
    if (params->instance >= STM32WB_UART_INSTANCE_COUNT)
    {
        return false;
    }

    uart->USART = stm32wb_uart_xlate_USART[params->instance];
    uart->instance = params->instance;
    uart->interrupt = stm32wb_uart_xlate_IRQn[params->instance];
    uart->priority = params->priority;
    uart->rx_dma = params->rx_dma;
    uart->tx_dma = params->tx_dma;
    uart->pins = params->pins;

    uart->rx_fifo = params->rx_fifo;
    uart->rx_entries = params->rx_entries;

    uart->state = STM32WB_UART_STATE_INIT;

    if (!stm32wb_uart_device.notify.callback)
    {
#if (STM32WB_UART_SPI_SUPPORTED == 1)
        stm32wb_system_register(&stm32wb_uart_device.notify, stm32wb_uart_notify_callback, NULL, (STM32WB_SYSTEM_NOTIFY_STOP_ENTER | STM32WB_SYSTEM_NOTIFY_STOP_LEAVE | STM32WB_SYSTEM_NOTIFY_CLOCKS));
#else /* (STM32WB_UART_SPI_SUPPORTED == 1) */
        stm32wb_system_register(&stm32wb_uart_device.notify, stm32wb_uart_notify_callback, NULL, (STM32WB_SYSTEM_NOTIFY_STOP_ENTER | STM32WB_SYSTEM_NOTIFY_STOP_LEAVE));
#endif /* (STM32WB_UART_SPI_SUPPORTED == 1) */
    }

    return true;
}

bool stm32wb_uart_destroy(stm32wb_uart_t *uart)
{
    if (uart->state != STM32WB_UART_STATE_INIT)
    {
        return false;
    }

    uart->state = STM32WB_UART_STATE_NONE;

    return true;
}

bool stm32wb_uart_enable(stm32wb_uart_t *uart, uint32_t baudrate, uint32_t option, uint8_t *rx_data, uint32_t rx_size, uint32_t rx_threshold[3], stm32wb_uart_event_callback_t callback, void *context)
{
    if (uart->state != STM32WB_UART_STATE_INIT)
    {
        return false;
    }

#if 0    
    if ((rx_data == NULL) || (rx_size < 16))
    {
        return false;
    }
#endif
    
    if (armv7m_atomic_cas((volatile uint32_t*)&stm32wb_uart_device.instances[uart->instance], (uint32_t)NULL, (uint32_t)uart) != (uint32_t)NULL)
    {
        return false;
    }
    
    uart->state = STM32WB_UART_STATE_NOT_READY;
    
    armv7m_atomic_store(&uart->mode, STM32WB_UART_MODE_RX_IDLE);
    
    uart->rx_data = rx_data;
    uart->rx_size = rx_size;
    uart->rx_read = 0;
    uart->rx_write = 0;
    uart->rx_event = 0;
    armv7m_atomic_storeh(&uart->rx_count, 0);

    uart->tx_data = NULL;
    uart->tx_data_e = NULL;

    armv7m_atomic_store((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL);
    uart->xf_callback = NULL;
    uart->xf_context = NULL;

    uart->ev_callback = callback;
    uart->ev_context = context;

    if (!stm32wb_uart_configure(uart, baudrate, option, rx_threshold))
    {
        armv7m_atomic_store((volatile uint32_t*)&stm32wb_uart_device.instances[uart->instance], (uint32_t)NULL);

        uart->state = STM32WB_UART_STATE_INIT;

        return false;
    }

    uart->state = STM32WB_UART_STATE_READY;
      
    NVIC_SetPriority(uart->interrupt, uart->priority);
    NVIC_EnableIRQ(uart->interrupt);

    return true;
}

bool stm32wb_uart_disable(stm32wb_uart_t *uart)
{
    if ((uart->state != STM32WB_UART_STATE_READY) && (uart->state != STM32WB_UART_STATE_BREAK))
    {
        return false;
    }

    NVIC_DisableIRQ(uart->interrupt);

#if (STM32WB_UART_SPI_SUPPORTED == 1)
    if (!(uart->option & STM32WB_UART_OPTION_SPI))
#endif /* STM32WB_UART_SPI_SUPPORTED == 1 */
    {
        stm32wb_uart_stop(uart);
    }
    
    stm32wb_gpio_pin_configure(uart->pins.rx, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_MODE_ANALOG));
    stm32wb_gpio_pin_configure(uart->pins.tx, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_MODE_ANALOG));

#if (STM32WB_UART_RTS_SUPPORTED == 1)
    if (uart->option & STM32WB_UART_OPTION_RTS)
    {
        stm32wb_gpio_pin_configure(uart->pins.rts, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_MODE_ANALOG));
    }
#endif /* STM32WB_UART_RTS_SUPPORTED == 1 */
    
#if (STM32WB_UART_CTS_SUPPORTED == 1)
    if (uart->option & STM32WB_UART_OPTION_CTS)
    {
        stm32wb_gpio_pin_configure(uart->pins.cts, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_MODE_ANALOG));
    }
#endif /* STM32WB_UART_CTS_SUPPORTED == 1 */

#if (STM32WB_UART_SPI_SUPPORTED == 1)
    if (uart->option & STM32WB_UART_OPTION_SPI)
    {
        stm32wb_gpio_pin_configure(uart->pins.ck, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_MODE_ANALOG));
    }
#endif /* STM32WB_UART_SPI_SUPPORTED == 1 */
    
    armv7m_atomic_store((volatile uint32_t*)&stm32wb_uart_device.instances[uart->instance], (uint32_t)NULL);

    uart->state = STM32WB_UART_STATE_INIT;

    return true;
}

bool stm32wb_uart_configure(stm32wb_uart_t *uart, uint32_t baudrate, uint32_t option, uint32_t rx_threshold[3])
{
    uint32_t usart_cr1, usart_cr2, usart_cr3;

    if ((uart->state != STM32WB_UART_STATE_NOT_READY) && (uart->state != STM32WB_UART_STATE_READY))
    {
        return false;
    }
        
#if (STM32WB_UART_SPI_SUPPORTED == 1)
    if ((uart->state != STM32WB_UART_STATE_NOT_READY) && ((option ^ uart->option) & STM32WB_UART_OPTION_SPI))
    {
        return false;
    }

    if (option & STM32WB_UART_OPTION_SPI)
    {
        if (uart->instance != STM32WB_UART_INSTANCE_USART1)
        {
            return false;
        }

        if (!(stm32wb_system_options() & STM32WB_SYSTEM_OPTION_USART1_SYSCLK))
        {
            return false;
        }
        
#if (STM32WB_UART_FIFO_SUPPORTED == 1)
        usart_cr1 = USART_CR1_OVER8 | USART_CR1_FIFOEN;
        usart_cr2 = 0;
        usart_cr3 = USART_CR3_ONEBIT | USART_CR3_TXFTCFG_1 | USART_CR3_RXFTCFG_1; /* 1/2 TXFIFO, 1/2 RXFIFO */
#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */
        usart_cr1 = USART_CR1_OVER8;
        usart_cr2 = 0;
        usart_cr3 = USART_CR3_ONEBIT;
#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */

        stm32wb_gpio_pin_configure(uart->pins.rx, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
        stm32wb_gpio_pin_configure(uart->pins.tx, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
        stm32wb_gpio_pin_configure(uart->pins.ck, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));

        uart->clock = 0;
        uart->option = option;
        uart->control = ~0l;
        
        uart->cr1 = usart_cr1;
        uart->cr2 = usart_cr2;
        uart->cr3 = usart_cr3;
    }
    else
#endif /* STM32WB_UART_SPI_SUPPORTED == 1 */
    {
        if ((baudrate == 0) || (baudrate > 1000000))
        {
            return false;
        }
        
        if (rx_threshold && ((rx_threshold[0] > uart->rx_size) || (rx_threshold[1] > uart->rx_size) || (rx_threshold[2] > uart->rx_size) || (rx_threshold[1] > rx_threshold[2])))
        {
            return false;
        }
        
        if (uart->state == STM32WB_UART_STATE_READY)
        {
            stm32wb_uart_stop(uart);
        }
        
        if (rx_threshold)
        {
            uart->rx_threshold[0] = rx_threshold[0];
            uart->rx_threshold[1] = rx_threshold[1];
            uart->rx_threshold[2] = rx_threshold[2];
        }
        else
        {
            uart->rx_threshold[0] = 8;
            uart->rx_threshold[1] = uart->rx_size - (uart->rx_entries ? uart->rx_entries : 8);
            uart->rx_threshold[2] = uart->rx_size - (uart->rx_entries ? uart->rx_entries : 8);
        }

        if (uart->instance == STM32WB_UART_INSTANCE_USART1)
        {
            if (stm32wb_system_options() & STM32WB_SYSTEM_OPTION_USART1_SYSCLK)
            {
                option |= STM32WB_UART_OPTION_SYSCLK;
            }
        }
        
#if (STM32WB_UART_FIFO_SUPPORTED == 1)
        usart_cr1 = USART_CR1_FIFOEN | USART_CR1_IDLEIE;
        usart_cr2 = 0;
        usart_cr3 = USART_CR3_EIE | USART_CR3_TXFTCFG_1 | USART_CR3_RXFTCFG_1; /* 1/2 TXFIFO, 1/2 RXFIFO */
#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */
        usart_cr1 = USART_CR1_IDLEIE;
        usart_cr2 = 0;
        usart_cr3 = USART_CR3_EIE;
#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */

        if (option & (STM32WB_UART_OPTION_STOP_1_5 | STM32WB_UART_OPTION_STOP_2))
        {
            usart_cr2 |= USART_CR2_STOP_1;

            if (option & STM32WB_UART_OPTION_STOP_1_5)
            {
                usart_cr2 |= USART_CR2_STOP_0;
            }
        }
            
        if (option & (STM32WB_UART_OPTION_PARITY_EVEN | STM32WB_UART_OPTION_PARITY_ODD))
        {
            usart_cr1 |= (USART_CR1_PCE | USART_CR1_PEIE);
            
            if (option & STM32WB_UART_OPTION_PARITY_ODD)
            {
                usart_cr1 |= USART_CR1_PS;
            }
            
            usart_cr1 |= ((option & STM32WB_UART_OPTION_DATA_SIZE_8) ? USART_CR1_M0 : 0);
        }
        else
        {
            usart_cr1 |= ((option & STM32WB_UART_OPTION_DATA_SIZE_8) ? 0 : USART_CR1_M1);
        }
        
        if (option & STM32WB_UART_OPTION_WAKEUP)
        {
            usart_cr3 |= (USART_CR3_WUS_1 | USART_CR3_ONEBIT);
        }
        
        if (option & STM32WB_UART_OPTION_RX_INVERT)
        {
            usart_cr2 |= USART_CR2_RXINV;
        }
        
        if (option & STM32WB_UART_OPTION_TX_INVERT)
        {
            usart_cr2 |= USART_CR2_TXINV;
        }

        if (option & STM32WB_UART_OPTION_DATA_INVERT)
        {
            usart_cr2 |= USART_CR2_DATAINV;
        }

        stm32wb_gpio_pin_configure(uart->pins.rx, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
        stm32wb_gpio_pin_configure(uart->pins.tx, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
        stm32wb_gpio_pin_write(uart->pins.tx, 0);
        
#if (STM32WB_UART_RTS_SUPPORTED == 1)
        if (option & STM32WB_UART_OPTION_RTS)
        {
            stm32wb_gpio_pin_configure(uart->pins.rts, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));
            stm32wb_gpio_pin_write(uart->pins.rts, ((uart->mode & STM32WB_UART_MODE_RX_SUSPENDED) ? 1 : 0));
        }
        else
        {
            if (uart->option & STM32WB_UART_OPTION_RTS)
            {
                stm32wb_gpio_pin_configure(uart->pins.rts, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_MODE_ANALOG));
            }
        }
#endif /* STM32WB_UART_RTS_SUPPORTED == 1 */
        
#if (STM32WB_UART_CTS_SUPPORTED == 1)
        if (option & STM32WB_UART_OPTION_CTS)
        {
            stm32wb_gpio_pin_configure(uart->pins.cts, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_PULLDOWN | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
            
            usart_cr3 |= USART_CR3_CTSE;
        }
        else
        {
            if (uart->option & STM32WB_UART_OPTION_CTS)
            {
                stm32wb_gpio_pin_configure(uart->pins.cts, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_MODE_ANALOG));
            }
        }
#endif /* STM32WB_UART_CTS_SUPPORTED == 1 */
        
        if (uart->rx_dma != STM32WB_DMA_CHANNEL_NONE)
        {
            usart_cr3 |= USART_CR3_DDRE;
        }
        
        uart->clock = baudrate;
        uart->option = option;
        uart->control = ~0l;
        
        uart->cr1 = usart_cr1;
        uart->cr2 = usart_cr2;
        uart->cr3 = usart_cr3;
        
        stm32wb_uart_start(uart);
    }
    
    return true;
}

bool stm32wb_uart_break_state(stm32wb_uart_t *uart)
{
#if (STM32WB_UART_BREAK_SUPPORTED == 1)
    if (uart->state < STM32WB_UART_STATE_READY)
    {
        return false;
    }

    return !!(uart->mode & STM32WB_UART_MODE_RX_BREAK);
#else /* STM32WB_UART_BREAK_SUPPORTED == 1 */
    return false;
#endif /* STM32WB_UART_BREAK_SUPPORTED == 1 */
}

uint32_t stm32wb_uart_count(stm32wb_uart_t *uart)
{
    if (uart->state < STM32WB_UART_STATE_READY)
    {
        return 0;
    }

    return uart->rx_count;
}

uint32_t stm32wb_uart_read(stm32wb_uart_t *uart, uint8_t *rx_data, uint32_t rx_count)
{
    uint32_t rx_size, rx_read;
    uint32_t rx_count_previous;

    if (uart->state < STM32WB_UART_STATE_READY)
    {
        return 0;
    }

    rx_size = uart->rx_count;

    if (rx_count > rx_size)
    {
        rx_count = rx_size;
    }

    if (rx_count)
    {
        rx_read = uart->rx_read;
        rx_size = rx_count;

        if ((rx_read + rx_size) > uart->rx_size)
        {
            rx_size = uart->rx_size - rx_read;
        }

        memcpy(rx_data, &uart->rx_data[rx_read], rx_size);

        rx_data += rx_size;
        rx_read += rx_size;

        if (rx_read == uart->rx_size)
        {
            rx_read = 0;
        }

        if (rx_count != rx_size)
        {
            rx_size = rx_count - rx_size;

            memcpy(rx_data, &uart->rx_data[rx_read], rx_size);

            rx_data += rx_size;
            rx_read += rx_size;
        }

        uart->rx_read = rx_read;

#if (STM32WB_UART_RTS_SUPPORTED == 1)
        rx_count_previous = armv7m_atomic_subh(&uart->rx_count, rx_count);

        if (uart->option & STM32WB_UART_OPTION_RTS)
        {
            if (((rx_count_previous - rx_count) < uart->rx_threshold[1]) && (uart->mode & STM32WB_UART_MODE_RX_SUSPENDED))
            {
                armv7m_atomic_or(&uart->mode, STM32WB_UART_MODE_RX_SUSPEND_REQUEST);
                
                if (__current_irq() != uart->interrupt)
                {
                    NVIC_SetPendingIRQ(uart->interrupt);
                }
            }
        }
#else /* STM32WB_UART_RTS_SUPPORTED == 1 */
        armv7m_atomic_subh(&uart->rx_count, rx_count);
#endif /* STM32WB_UART_RTS_SUPPORTED == 1 */
    }
    
    return rx_count;
}

int32_t stm32wb_uart_peek(stm32wb_uart_t *uart)
{
    if (uart->state < STM32WB_UART_STATE_READY)
    {
        return -1;
    }

    if (uart->rx_count == 0)
    {
        return -1;
    }

    return uart->rx_data[uart->rx_read];
}

bool stm32wb_uart_transmit(stm32wb_uart_t *uart, const uint8_t *tx_data, uint32_t tx_count, volatile uint8_t *p_status_return, stm32wb_uart_done_callback_t callback, void *context)
{
    if (armv7m_atomic_cas((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL, (uint32_t)p_status_return) != (uint32_t)NULL)
    {
        return false;
    }

    if (uart->state != STM32WB_UART_STATE_READY)
    {
        armv7m_atomic_store((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL);

        return false;
    }

    *p_status_return = STM32WB_UART_STATUS_BUSY;
    
    uart->xf_callback = callback;
    uart->xf_context = context;

    uart->tx_data = tx_data;
    uart->tx_data_e = tx_data + tx_count;

    uart->state = STM32WB_UART_STATE_TRANSMIT_START;

    if (__current_irq() != uart->interrupt)
    {
        NVIC_SetPendingIRQ(uart->interrupt);
    }

    return true;
}

bool stm32wb_uart_break(stm32wb_uart_t *uart, bool onoff, volatile uint8_t *p_status_return, stm32wb_uart_done_callback_t callback, void *context)
{
    if (armv7m_atomic_cas((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL, (uint32_t)p_status_return) != (uint32_t)NULL)
    {
        return false;
    }

    if (uart->state != (onoff ? STM32WB_UART_STATE_READY : STM32WB_UART_STATE_BREAK))
    {
        armv7m_atomic_store((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL);

        return false;
    }

    *p_status_return = STM32WB_UART_STATUS_BUSY;
    
    uart->xf_callback = callback;
    uart->xf_context = context;

    uart->state = (onoff ? STM32WB_UART_STATE_BREAK_ON : STM32WB_UART_STATE_BREAK_OFF);

    if (__current_irq() != uart->interrupt)
    {
        NVIC_SetPendingIRQ(uart->interrupt);
    }

    return true;
}

#if (STM32WB_UART_SPI_SUPPORTED == 1)

#include "stm32wb_spi.h"

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

static void stm32wb_uart_spi_dma_callback(stm32wb_uart_t *uart, uint32_t events)
{
    USART_TypeDef *USART = uart->USART;
    stm32wb_uart_done_callback_t callback;
    void *context;
    volatile uint8_t *p_status_return;

    p_status_return = uart->xf_status;

    if (!p_status_return)
    {
        return;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&uart->xf_status, (uint32_t)p_status_return, (uint32_t)NULL) != (uint32_t)p_status_return)
    {
        return;
    }

    stm32wb_dma_stop(uart->tx_dma);
    stm32wb_dma_stop(uart->rx_dma);

    USART->CR3 = uart->cr3;

    callback = uart->xf_callback;
    context = uart->xf_context;

    uart->state = STM32WB_UART_STATE_DATA;
    
    *p_status_return = STM32WB_UART_STATUS_SUCCESS;

    if (callback)
    {
        (*callback)(context);
    }
}

bool stm32wb_uart_spi_enable(stm32wb_uart_t *uart)
{
    return stm32wb_uart_enable(uart, 0, STM32WB_UART_OPTION_SPI, NULL, 0, NULL, NULL, NULL);
}

bool stm32wb_uart_spi_disable(stm32wb_uart_t *uart)
{
    return stm32wb_uart_disable(uart);
}

bool stm32wb_uart_spi_block(stm32wb_uart_t *uart, uint16_t pin)
{
    if (uart->state != STM32WB_UART_STATE_READY)
    {
        return false;
    }

    uart->mask |= (1ul << ((pin & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT));

    return true;
}

bool stm32wb_uart_spi_unblock(stm32wb_uart_t *uart, uint16_t pin)
{
    if (uart->state != STM32WB_UART_STATE_READY)
    {
        return false;
    }

    uart->mask &= ~(1ul << ((pin & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT));

    return true;
}

bool stm32wb_uart_spi_acquire(stm32wb_uart_t *uart, uint32_t clock, uint32_t control)
{
    USART_TypeDef *USART = uart->USART;
    uint32_t uartclk, uartdiv;

    if (uart->state != STM32WB_UART_STATE_READY)
    {
        return false;
    }

    if (!(uart->option & STM32WB_UART_OPTION_SPI))
    {
        return false;
    }
    
    if (uart->mask)
    {
        stm32wb_exti_block(uart->mask);
    }

    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP_0);
    stm32wb_system_reference(STM32WB_SYSTEM_REFERENCE_USART1 << uart->instance);

    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_USART1 + uart->instance);

    if ((uart->clock != clock) || (uart->control != control))
    {
        uartclk = stm32wb_system_sysclk();

        uartdiv = ((uartclk + (clock -1)) / clock) * 2;

        if (uartdiv < 16)
        {
            uartdiv = 16;
        }

        uart->cr2 = ((USART_CR2_CLKEN | USART_CR2_LBCL) |
                     ((control & STM32WB_UART_SPI_CONTROL_LSB_FIRST) ? 0 : USART_CR2_MSBFIRST) |
                     ((control & STM32WB_UART_SPI_CONTROL_CPOL) ? USART_CR2_CPOL : 0) |
                     ((control & STM32WB_UART_SPI_CONTROL_CPHA) ? USART_CR2_CPHA : 0));


        USART->CR1 = uart->cr1;
        USART->CR2 = uart->cr2;
        USART->CR3 = uart->cr3;
        USART->BRR = (uartdiv & ~15) | ((uartdiv & 15) >> 1);
        
        uart->clock = clock;
        uart->control = control;
    }

    USART->CR1 = uart->cr1 | USART_CR1_UE;
    USART->CR1 = uart->cr1 | USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    
    while (!(USART->ISR & USART_ISR_REACK))
    {
    }

    USART->RQR = USART_RQR_RXFRQ;

    uart->state = STM32WB_UART_STATE_DATA;

    return true;
}

bool stm32wb_uart_spi_release(stm32wb_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;

    if (uart->state != STM32WB_UART_STATE_DATA)
    {
        return false;
    }
    
    USART->CR1 = uart->cr1 | USART_CR1_UE;
    USART->CR1 = uart->cr1;

    if (uart->option & (STM32WB_UART_OPTION_RX_DMA | STM32WB_UART_OPTION_TX_DMA))
    {
        if (uart->option & STM32WB_UART_OPTION_RX_DMA)
        {
            stm32wb_dma_disable(uart->rx_dma);
        }

        if (uart->option & STM32WB_UART_OPTION_TX_DMA)
        {
            stm32wb_dma_disable(uart->tx_dma);
        }
        
        uart->option &= ~(STM32WB_UART_OPTION_RX_DMA | STM32WB_UART_OPTION_TX_DMA);
    }

    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_USART1 + uart->instance);

    stm32wb_system_unreference(STM32WB_SYSTEM_REFERENCE_USART1 << uart->instance);
    stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP_0);

    uart->state = STM32WB_UART_STATE_READY;

    if (uart->mask)
    {
        stm32wb_exti_unblock(uart->mask, false);
    }

    return true;
}

__attribute__((optimize("O3"))) uint8_t stm32wb_uart_spi_data(stm32wb_uart_t *uart, uint8_t tx_data)
{
    USART_TypeDef *USART = uart->USART;
    uint8_t rx_data;

    rx_data = STM32WB_UART_SPI_DATA_8(USART, tx_data);

    return rx_data;
}

uint16_t stm32wb_uart_spi_data16le(stm32wb_uart_t *uart, uint16_t tx_data)
{
    USART_TypeDef *USART = uart->USART;
    uint16_t rx_data;
    
#if (STM32WB_UART_FIFO_SUPPORTED == 1)
    STM32WB_UART_SPI_WRITE_8(USART, tx_data >> 0);
    STM32WB_UART_SPI_WRITE_8(USART, tx_data >> 8);

    while (!(USART->ISR & USART_ISR_RXFNE))
    {
    }

    rx_data = (STM32WB_UART_SPI_READ_8(USART) << 0);

    while (!(USART->ISR & USART_ISR_RXFNE))
    {
    }

    rx_data |= (STM32WB_UART_SPI_READ_8(USART) << 8);
#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */
    rx_data = (STM32WB_UART_SPI_DATA_8(USART, tx_data >> 0) << 0);
    rx_data |= (STM32WB_UART_SPI_DATA_8(USART, tx_data >> 8) << 8);
#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */

    return rx_data;
}

uint16_t stm32wb_uart_spi_data16be(stm32wb_uart_t *uart, uint16_t tx_data)
{
    USART_TypeDef *USART = uart->USART;
    uint16_t rx_data;

#if (STM32WB_UART_FIFO_SUPPORTED == 1)
    STM32WB_UART_SPI_WRITE_8(USART, tx_data >> 8);
    STM32WB_UART_SPI_WRITE_8(USART, tx_data >> 0);

    while (!(USART->ISR & USART_ISR_RXFNE))
    {
    }

    rx_data = (STM32WB_UART_SPI_READ_8(USART) << 8);

    while (!(USART->ISR & USART_ISR_RXFNE))
    {
    }

    rx_data |= (STM32WB_UART_SPI_READ_8(USART) << 0);
#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */
    rx_data = (STM32WB_UART_SPI_DATA_8(USART, tx_data >> 8) << 8);
    rx_data |= (STM32WB_UART_SPI_DATA_8(USART, tx_data >> 0) << 0);
#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */

    return rx_data;
}

void stm32wb_uart_spi_data_receive(stm32wb_uart_t *uart, uint8_t *rx_data, uint32_t rx_count)
{
    USART_TypeDef *USART = uart->USART;
    uint8_t *rx_data_e;
    const uint8_t tx_default = 0xff;
    
#if (STM32WB_UART_FIFO_SUPPORTED == 1)

    if (rx_count < 4)
    {
        if (rx_count == 3)
        {
            STM32WB_UART_SPI_WRITE_8(USART, tx_default);
            STM32WB_UART_SPI_WRITE_8(USART, tx_default);
            STM32WB_UART_SPI_WRITE_8(USART, tx_default);
            
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }
            
            rx_data[0] = STM32WB_UART_SPI_READ_8(USART);
            
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }
            
            rx_data[1] = STM32WB_UART_SPI_READ_8(USART);

            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }
            
            rx_data[2] = STM32WB_UART_SPI_READ_8(USART);
        }
        else if (rx_count == 2)
        {
            STM32WB_UART_SPI_WRITE_8(USART, tx_default);
            STM32WB_UART_SPI_WRITE_8(USART, tx_default);
            
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }
            
            rx_data[0] = STM32WB_UART_SPI_READ_8(USART);
            
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }
            
            rx_data[1] = STM32WB_UART_SPI_READ_8(USART);
        }
        else
        {
            STM32WB_UART_SPI_WRITE_8(USART, tx_default);
            
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }
            
            rx_data[0] = STM32WB_UART_SPI_READ_8(USART);
        }
    }
    else
    {
        rx_data_e = rx_data + rx_count - 4;
        
        STM32WB_UART_SPI_WRITE_8(USART, tx_default);
        STM32WB_UART_SPI_WRITE_8(USART, tx_default);
        STM32WB_UART_SPI_WRITE_8(USART, tx_default);
        STM32WB_UART_SPI_WRITE_8(USART, tx_default);
        
        while (rx_data != rx_data_e)
        {
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }
            
            *rx_data++ = STM32WB_UART_SPI_READ_8(USART);
            
            STM32WB_UART_SPI_WRITE_8(USART, tx_default);
        }
        
        while (!(USART->ISR & USART_ISR_RXFNE))
        {
        }
        
        rx_data[0] = STM32WB_UART_SPI_READ_8(USART);
        
        while (!(USART->ISR & USART_ISR_RXFNE))
        {
        }
        
        rx_data[1] = STM32WB_UART_SPI_READ_8(USART);
        
        while (!(USART->ISR & USART_ISR_RXFNE))
        {
        }
        
        rx_data[2] = STM32WB_UART_SPI_READ_8(USART);

        while (!(USART->ISR & USART_ISR_RXFNE))
        {
        }
        
        rx_data[3] = STM32WB_UART_SPI_READ_8(USART);
    }

#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */

    rx_data_e = rx_data + rx_count;

    while (rx_data != rx_data_e)
    {
        *rx_data++ = STM32WB_UART_SPI_DATA_8(USART, tx_default);
    }

#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */
}

void stm32wb_uart_spi_data_transmit(stm32wb_uart_t *uart, const uint8_t *tx_data, uint32_t tx_count)
{
    USART_TypeDef *USART = uart->USART;
    const uint8_t *tx_data_e;

#if (STM32WB_UART_FIFO_SUPPORTED == 1)

    if (tx_count < 4)
    {
        if (tx_count == 3)
        {
            STM32WB_UART_SPI_WRITE_8(USART, tx_data[0]);
            STM32WB_UART_SPI_WRITE_8(USART, tx_data[1]);
            STM32WB_UART_SPI_WRITE_8(USART, tx_data[2]);
                    
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            STM32WB_UART_SPI_READ_8(USART);
                    
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            STM32WB_UART_SPI_READ_8(USART);

            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            STM32WB_UART_SPI_READ_8(USART);
        }
        else if (tx_count == 2)
        {
            STM32WB_UART_SPI_WRITE_8(USART, tx_data[0]);
            STM32WB_UART_SPI_WRITE_8(USART, tx_data[1]);
                    
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            STM32WB_UART_SPI_READ_8(USART);
                    
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            STM32WB_UART_SPI_READ_8(USART);
        }
        else
        {
            STM32WB_UART_SPI_WRITE_8(USART, tx_data[0]);
                    
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            STM32WB_UART_SPI_READ_8(USART);
        }
    }
    else
    {
        tx_data_e = tx_data + tx_count;
        
        STM32WB_UART_SPI_WRITE_8(USART, tx_data[0]);
        STM32WB_UART_SPI_WRITE_8(USART, tx_data[1]);
        STM32WB_UART_SPI_WRITE_8(USART, tx_data[2]);
        STM32WB_UART_SPI_WRITE_8(USART, tx_data[3]);
        tx_data += 4;
                
        while (tx_data != tx_data_e)
        {
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            STM32WB_UART_SPI_READ_8(USART);
            STM32WB_UART_SPI_WRITE_8(USART, *tx_data++);
        }
                
        while (!(USART->ISR & USART_ISR_RXFNE))
        {
        }

        STM32WB_UART_SPI_READ_8(USART);
                
        while (!(USART->ISR & USART_ISR_RXFNE))
        {
        }

        STM32WB_UART_SPI_READ_8(USART);
                
        while (!(USART->ISR & USART_ISR_RXFNE))
        {
        }

        STM32WB_UART_SPI_READ_8(USART);

        while (!(USART->ISR & USART_ISR_RXFNE))
        {
        }

        STM32WB_UART_SPI_READ_8(USART);
    }

#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */

    tx_data_e = tx_data + tx_count;

    while (tx_data != tx_data_e)
    {
        STM32WB_UART_SPI_DATA_8(USART, *tx_data++);
    }

#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */
}

void stm32wb_uart_spi_data_transfer(stm32wb_uart_t *uart, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count)
{
    USART_TypeDef *USART = uart->USART;
    const uint8_t *tx_data_e;

#if (STM32WB_UART_FIFO_SUPPORTED == 1)

    if (xf_count < 4)
    {
        if (xf_count == 3)
        {
            STM32WB_UART_SPI_WRITE_8(USART, tx_data[0]);
            STM32WB_UART_SPI_WRITE_8(USART, tx_data[1]);
            STM32WB_UART_SPI_WRITE_8(USART, tx_data[2]);
                    
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            rx_data[0] = STM32WB_UART_SPI_READ_8(USART);
                    
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            rx_data[1] = STM32WB_UART_SPI_READ_8(USART);

            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            rx_data[2] = STM32WB_UART_SPI_READ_8(USART);
        }
        else if (xf_count == 2)
        {
            STM32WB_UART_SPI_WRITE_8(USART, tx_data[0]);
            STM32WB_UART_SPI_WRITE_8(USART, tx_data[1]);
                    
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            rx_data[0] = STM32WB_UART_SPI_READ_8(USART);
                    
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            rx_data[1] = STM32WB_UART_SPI_READ_8(USART);
        }
        else
        {
            STM32WB_UART_SPI_WRITE_8(USART, tx_data[0]);
                    
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            rx_data[0] = STM32WB_UART_SPI_READ_8(USART);
        }
    }
    else
    {
        tx_data_e = tx_data + xf_count;
        
        STM32WB_UART_SPI_WRITE_8(USART, tx_data[0]);
        STM32WB_UART_SPI_WRITE_8(USART, tx_data[1]);
        STM32WB_UART_SPI_WRITE_8(USART, tx_data[2]);
        STM32WB_UART_SPI_WRITE_8(USART, tx_data[3]);
        tx_data += 4;

        while (tx_data != tx_data_e)
        {
            while (!(USART->ISR & USART_ISR_RXFNE))
            {
            }

            *rx_data++ = STM32WB_UART_SPI_READ_8(USART);
            STM32WB_UART_SPI_WRITE_8(USART, *tx_data++);
        }
                
        while (!(USART->ISR & USART_ISR_RXFNE))
        {
        }

        rx_data[0] = STM32WB_UART_SPI_READ_8(USART);
                
        while (!(USART->ISR & USART_ISR_RXFNE))
        {
        }

        rx_data[1] = STM32WB_UART_SPI_READ_8(USART);
                
        while (!(USART->ISR & USART_ISR_RXFNE))
        {
        }

        rx_data[2] = STM32WB_UART_SPI_READ_8(USART);

        while (!(USART->ISR & USART_ISR_RXFNE))
        {
        }

        rx_data[3] = STM32WB_UART_SPI_READ_8(USART);
    }

#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */

    tx_data_e = tx_data + xf_count;

    while (tx_data != tx_data_e)
    {
        *rx_data++ = STM32WB_UART_SPI_DATA_8(USART, *tx_data++);
    }

#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */
}

__attribute__((optimize("O3"))) bool stm32wb_uart_spi_data_dma_receive(stm32wb_uart_t *uart, uint8_t *rx_data, uint32_t rx_count, volatile uint8_t *p_status_return, stm32wb_uart_done_callback_t callback, void *context)
{
    USART_TypeDef *USART = uart->USART;

    if (uart->state != STM32WB_UART_STATE_DATA)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL, (uint32_t)p_status_return) != (uint32_t)NULL)
    {
        return false;
    }

    if (!(uart->option & STM32WB_UART_OPTION_RX_DMA))
    {
        if (!stm32wb_dma_enable(uart->rx_dma, uart->priority, (stm32wb_dma_callback_t)stm32wb_uart_spi_dma_callback, uart))
        {
            armv7m_atomic_store((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL);
            
            return false;
        }
        
        uart->option |= STM32WB_UART_OPTION_RX_DMA;
    }
    
    if (!(uart->option & STM32WB_UART_OPTION_TX_DMA))
    {
        if (!stm32wb_dma_enable(uart->tx_dma, uart->priority, NULL, NULL))
        {
            armv7m_atomic_store((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL);
            
            return false;
        }
        
        uart->option |= STM32WB_UART_OPTION_TX_DMA;
    }

    uart->state = STM32WB_UART_STATE_DATA_DMA;
    
    *p_status_return = STM32WB_UART_STATUS_BUSY;
    
    uart->xf_status = p_status_return;
    uart->xf_callback = callback;
    uart->xf_context = context;
    uart->rx_data = rx_data;

    USART->CR3 = uart->cr3 | (USART_CR3_DMAR | USART_CR3_DMAT);

    stm32wb_dma_start(uart->rx_dma, (uint32_t)rx_data, (uint32_t)&USART->RDR, rx_count, STM32WB_UART_SPI_RX_DMA_OPTION_RECEIVE | STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE);
    stm32wb_dma_start(uart->tx_dma, (uint32_t)&USART->TDR, (uint32_t)&uart->tx_default, rx_count, STM32WB_UART_SPI_TX_DMA_OPTION_RECEIVE);
        
    return true;
}

__attribute__((optimize("O3"))) bool stm32wb_uart_spi_data_dma_transmit(stm32wb_uart_t *uart, const uint8_t *tx_data, uint32_t tx_count, volatile uint8_t *p_status_return, stm32wb_uart_done_callback_t callback, void *context)
{
    USART_TypeDef *USART = uart->USART;
    
    if (uart->state != STM32WB_UART_STATE_DATA)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL, (uint32_t)p_status_return) != (uint32_t)NULL)
    {
        return false;
    }

    if (!(uart->option & STM32WB_UART_OPTION_RX_DMA))
    {
        if (!stm32wb_dma_enable(uart->rx_dma, uart->priority, (stm32wb_dma_callback_t)stm32wb_uart_spi_dma_callback, uart))
        {
            armv7m_atomic_store((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL);
            
            return false;
        }
        
        uart->option |= STM32WB_UART_OPTION_RX_DMA;
    }
    
    if (!(uart->option & STM32WB_UART_OPTION_TX_DMA))
    {
        if (!stm32wb_dma_enable(uart->tx_dma, uart->priority, NULL, NULL))
        {
            armv7m_atomic_store((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL);
            
            return false;
        }
        
        uart->option |= STM32WB_UART_OPTION_TX_DMA;
    }

    uart->state = STM32WB_UART_STATE_DATA_DMA;
    
    *p_status_return = STM32WB_UART_STATUS_BUSY;

    uart->xf_status = p_status_return;
    uart->xf_callback = callback;
    uart->xf_context = context;
    uart->rx_data = NULL;

    USART->CR3 = uart->cr3 | (USART_CR3_DMAR | USART_CR3_DMAT);
                
    stm32wb_dma_start(uart->rx_dma, (uint32_t)&uart->rx_none, (uint32_t)&USART->RDR, tx_count, STM32WB_UART_SPI_RX_DMA_OPTION_RECEIVE | STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE);
    stm32wb_dma_start(uart->tx_dma, (uint32_t)&USART->TDR, (uint32_t)tx_data, tx_count, STM32WB_UART_SPI_TX_DMA_OPTION_RECEIVE);

    return true;
}

__attribute__((optimize("O3"))) bool stm32wb_uart_spi_data_dma_transfer(stm32wb_uart_t *uart, const uint8_t *tx_data, uint8_t *rx_data, uint32_t xf_count, volatile uint8_t *p_status_return, stm32wb_uart_done_callback_t callback, void *context)
{
    USART_TypeDef *USART = uart->USART;
    
    if (uart->state != STM32WB_UART_STATE_DATA)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL, (uint32_t)p_status_return) != (uint32_t)NULL)
    {
        return false;
    }

    if (!(uart->option & STM32WB_UART_OPTION_RX_DMA))
    {
        if (!stm32wb_dma_enable(uart->rx_dma, uart->priority, (stm32wb_dma_callback_t)stm32wb_uart_spi_dma_callback, uart))
        {
            armv7m_atomic_store((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL);
            
            return false;
        }
        
        uart->option |= STM32WB_UART_OPTION_RX_DMA;
    }
    
    if (!(uart->option & STM32WB_UART_OPTION_TX_DMA))
    {
        if (!stm32wb_dma_enable(uart->tx_dma, uart->priority, NULL, NULL))
        {
            armv7m_atomic_store((volatile uint32_t*)&uart->xf_status, (uint32_t)NULL);
            
            return false;
        }
        
        uart->option |= STM32WB_UART_OPTION_TX_DMA;
    }

    uart->state = STM32WB_UART_STATE_DATA_DMA;
    
    *p_status_return = STM32WB_UART_STATUS_BUSY;

    uart->xf_status = p_status_return;
    uart->xf_callback = callback;
    uart->xf_context = context;

    uart->rx_data = rx_data;

    USART->CR3 = uart->cr3 | (USART_CR3_DMAR | USART_CR3_DMAT);

    stm32wb_dma_start(uart->rx_dma, (uint32_t)rx_data, (uint32_t)&USART->RDR, xf_count, STM32WB_UART_SPI_RX_DMA_OPTION_TRANSFER | STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE);
    stm32wb_dma_start(uart->tx_dma, (uint32_t)&USART->TDR, (uint32_t)tx_data, xf_count, STM32WB_UART_SPI_TX_DMA_OPTION_TRANSFER);

    return true;
}

uint32_t stm32wb_uart_spi_data_dma_cancel(stm32wb_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;
    uint32_t xf_count;
    volatile uint8_t *p_status_return;

    p_status_return = uart->xf_status;

    if (!p_status_return)
    {
        return 0;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&uart->xf_status, (uint32_t)p_status_return, (uint32_t)NULL) != (uint32_t)p_status_return)
    {
        return 0;
    }

    stm32wb_dma_stop(uart->tx_dma);

    USART->CR3 = uart->cr3 | USART_CR3_DMAR;
    
#if (STM32WB_UART_FIFO_SUPPORTED == 1)
    while ((USART->ISR & (USART_ISR_TXFE | USART_ISR_TC)) != (USART_ISR_TXFE | USART_ISR_TC))
    {
    }
#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */
    while ((USART->ISR & (USART_ISR_TXE | USART_ISR_TC)) != (USART_ISR_TXE | USART_ISR_TC))
    {
    }
#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */
    
    xf_count = stm32wb_dma_stop(uart->rx_dma);

    USART->CR3 = uart->cr3;
    
#if (STM32WB_UART_FIFO_SUPPORTED == 1)
    if (uart->rx_data)
    {
        while (USART->ISR & USART_ISR_RXFNE)
        {
            uart->rx_data[xf_count++] = STM32WB_UART_SPI_READ_8(USART);
        }
    }
    else
    {
        while (USART->ISR & USART_ISR_RXFNE)
        {
            STM32WB_UART_SPI_READ_8(USART);
        }
    }
#else /* STM32WB_UART_FIFO_SUPPORTED == 1 */
    if (uart->rx_data)
    {
        if (USART->ISR & USART_ISR_RXNE)
        {
            uart->rx_data[xf_count++] = STM32WB_UART_SPI_READ_8(USART);
        }
    }
    else
    {
        if (USART->ISR & USART_ISR_RXNE)
        {
            STM32WB_UART_SPI_READ_8(USART);
        }
    }
#endif /* STM32WB_UART_FIFO_SUPPORTED == 1 */

    uart->state = STM32WB_UART_STATE_DATA;
    
    *p_status_return = STM32WB_UART_STATUS_FAILURE;
    
    return xf_count;
}

bool stm32wb_uart_spi_data_dma_busy(stm32wb_uart_t *uart)
{
    return (uart->state == STM32WB_UART_STATE_DATA_DMA);
}

#endif /* (STM32WB_UART_UART_SUPPORTED == 1) */

void USART1_IRQHandler(void)
{
    stm32wb_uart_interrupt(stm32wb_uart_device.instances[STM32WB_UART_INSTANCE_USART1]);

    __DSB();
}

void LPUART1_IRQHandler(void)
{
    stm32wb_uart_interrupt(stm32wb_uart_device.instances[STM32WB_UART_INSTANCE_LPUART1]);

    __DSB();
}
