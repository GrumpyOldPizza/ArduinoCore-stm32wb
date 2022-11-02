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

#include "armv7m.h"
#include "stm32wbxx.h"

#include "stm32wb_gpio.h"
#include "stm32wb_exti.h"
#include "stm32wb_i2c.h"
#include "stm32wb_dma.h"
#include "stm32wb_system.h"

typedef struct _stm32wb_i2c_device_t {
    stm32wb_i2c_t *          instances[STM32WB_I2C_INSTANCE_COUNT];
} stm32wb_i2c_device_t;

static stm32wb_i2c_device_t stm32wb_i2c_device;

#define STM32WB_I2C_TX_DMA_OPTION \
    (STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL |   \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |     \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |  \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32WB_I2C_RX_DMA_OPTION \
    (STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY |   \
     STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 | \
     STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8 |     \
     STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT |  \
     STM32WB_DMA_OPTION_PRIORITY_MEDIUM)

#define I2C_CR1_DNF_SHIFT    8
#define I2C_CR2_NBYTES_MAX   255
#define I2C_CR2_NBYTES_SHIFT 16
#define I2C_CR2_NBYTES_MASK  0x00ff0000

#define STM32WB_I2C_TRANSACTION_SENTINEL ((stm32wb_i2c_transaction_t*)0x00000001)

#define STM32WB_I2C_SUSPEND_CALLBACK ((stm32wb_i2c_suspend_callback_t)2)

static I2C_TypeDef * const stm32wb_i2c_xlate_I2C[STM32WB_I2C_INSTANCE_COUNT] = {
    I2C1,
    I2C3,
};

static const IRQn_Type stm32wb_i2c_xlate_IRQn[STM32WB_I2C_INSTANCE_COUNT] = {
    I2C1_EV_IRQn,
    I2C3_EV_IRQn,
};

static const uint32_t stm32wb_i2c_xlate_FMP[STM32WB_I2C_INSTANCE_COUNT] = {
    SYSCFG_CFGR1_I2C1_FMP,
    SYSCFG_CFGR1_I2C3_FMP,
};

static const uint32_t stm32wb_i2c_xlate_IMR[STM32WB_I2C_INSTANCE_COUNT] = {
    EXTI_IMR1_IM22,
    EXTI_IMR1_IM23,
};

static void stm32wb_i2c_start(stm32wb_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;

    stm32wb_system_hsi16_enable();

    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_I2C1 + i2c->instance);

    if (i2c->option & STM32WB_I2C_OPTION_WAKEUP)
    {
        if (i2c->instance != STM32WB_I2C_INSTANCE_I2C3)
        {
            stm32wb_system_lock(STM32WB_SYSTEM_LOCK_STOP_1);
        }
    }

    I2C->CR1 |= I2C_CR1_PE;
}

static void stm32wb_i2c_stop(stm32wb_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;

    /* WAR for ERRATA 2.6.1 */
    I2C->CR1 &= ~I2C_CR1_PE;

    if (i2c->option & STM32WB_I2C_OPTION_WAKEUP)
    {
        if (i2c->instance != STM32WB_I2C_INSTANCE_I2C3)
        {
            stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_STOP_1);
        }
    }

    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_I2C1 + i2c->instance);

    stm32wb_system_hsi16_disable();
}

static void stm32wb_i2c_sync(stm32wb_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;
    uint32_t i2c_cr1, i2c_cr2, i2c_oar1, i2c_oar2, i2c_timingr, i2c_timeoutr;

    i2c->rq_sync = false;

    if (i2c->option & STM32WB_I2C_OPTION_ADDRESS_MASK)
    {
        stm32wb_i2c_stop(i2c);

        armv7m_atomic_and(&EXTI->IMR1, ~stm32wb_i2c_xlate_IMR[i2c->instance]);
    }
    
    armv7m_atomic_and(&SYSCFG->CFGR1, ~stm32wb_i2c_xlate_FMP[i2c->instance]);

    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_I2C1 + i2c->instance);

    I2C->CR1 = 0;
    
    i2c_cr1 = 0;
    i2c_cr2 = 0;
    i2c_oar1 = 0;
    i2c_oar2 = 0;
    i2c_timingr = 0;
    i2c_timeoutr = 0;

    if (i2c->option & STM32WB_I2C_OPTION_ADDRESS_MASK)
    {
        i2c_oar1 = I2C_OAR1_OA1EN | (((i2c->option & STM32WB_I2C_OPTION_ADDRESS_MASK) >> STM32WB_I2C_OPTION_ADDRESS_SHIFT) << 1);

        if (i2c->option & STM32WB_I2C_OPTION_GENERAL_CALL)
        {
            i2c_cr1 |= I2C_CR1_GCEN;
        }

        i2c_cr1 |= I2C_CR1_ADDRIE;

        if (i2c->option & STM32WB_I2C_OPTION_WAKEUP)
        {
            i2c_cr1 |= I2C_CR1_WUPEN;

            armv7m_atomic_or(&EXTI->IMR1, stm32wb_i2c_xlate_IMR[i2c->instance]);
        }

        I2C->TIMINGR = 0x00300000;
        I2C->TIMEOUTR = 0;
    }
    else
    {
        if  (i2c->option & STM32WB_I2C_OPTION_MODE_1000K)
        {
            armv7m_atomic_or(&SYSCFG->CFGR1, stm32wb_i2c_xlate_FMP[i2c->instance]);
        }
        
        /* tRise =  62ns, tFall = 10ns, no AF, DNF = 1 (FMP)    */
        /* tRise = 125ns, tFall = 13ns, no AF, DNF = 1 (FM, SM) */
        i2c_cr1 |= (I2C_CR1_ANFOFF | (1 << I2C_CR1_DNF_SHIFT));
        
        if      (i2c->option & STM32WB_I2C_OPTION_MODE_1000K) { i2c_timingr = 0x00100106; }
        else if (i2c->option & STM32WB_I2C_OPTION_MODE_400K)  { i2c_timingr = 0x00300618; }
        else                                                  { i2c_timingr = 0x00503c5a; }
        
        if (i2c->timeout) 
        {
            i2c_timeoutr = ((i2c->timeout * 125) + 15) / 16 -1;
            
            if (i2c_timeoutr > 4095)
            {
                i2c_timeoutr = 4095;
            }
            
            i2c_timeoutr |= I2C_TIMEOUTR_TIMOUTEN;
        }
        
        I2C->TIMINGR = i2c_timingr;
        I2C->TIMEOUTR = i2c_timeoutr;
    }

    I2C->OAR2 = i2c_oar2;
    I2C->OAR1 = i2c_oar1;
    I2C->CR2 = i2c_cr2;
    I2C->CR1 = i2c_cr1;

    if (i2c->option & STM32WB_I2C_OPTION_WAKEUP)
    {
        stm32wb_gpio_pin_configure(i2c->pins.scl, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_ALTERNATE));
        stm32wb_gpio_pin_configure(i2c->pins.sda, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_ALTERNATE));
    }
    else
    {
        stm32wb_gpio_pin_configure(i2c->pins.scl, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_ALTERNATE));
        stm32wb_gpio_pin_configure(i2c->pins.sda, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_ALTERNATE));
    }

    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_I2C1 + i2c->instance);

    if (i2c->option & STM32WB_I2C_OPTION_ADDRESS_MASK) 
    {
        stm32wb_i2c_start(i2c);
    }
}

static void stm32wb_i2c_master_transmit(stm32wb_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;
    uint32_t i2c_cr2, count;
    bool tx_dma;

    i2c->state = STM32WB_I2C_STATE_MASTER_TRANSMIT;

    count = i2c->tx_data_e - i2c->tx_data;

    i2c_cr2 = (i2c->xf_address << 1) | I2C_CR2_START;

    if (count > I2C_CR2_NBYTES_MAX)
    {
        count = I2C_CR2_NBYTES_MAX;

        i2c_cr2 |= I2C_CR2_RELOAD;
    }
    else
    {
        if (i2c->xf_control & STM32WB_I2C_CONTROL_DIRECTION)
        {
            i2c_cr2 |= I2C_CR2_RELOAD;
        }
        else
        {
            if (!i2c->rx_data && !(i2c->xf_control & STM32WB_I2C_CONTROL_RESTART))
            {
                i2c_cr2 |= I2C_CR2_AUTOEND;
            }
        }
    }

    tx_dma = false;

    if ((count > 1) && (i2c->tx_dma != STM32WB_DMA_CHANNEL_NONE))
    {
        tx_dma = stm32wb_dma_channel(i2c->tx_dma);
        
        if (!tx_dma)
        {
	    tx_dma = stm32wb_dma_enable(i2c->tx_dma, i2c->priority, NULL, NULL);
        }
    }

    if ((count > 1) && tx_dma)
    {
        I2C->CR1 |= I2C_CR1_TXDMAEN;

        stm32wb_dma_start(i2c->tx_dma, (uint32_t)&I2C->TXDR, (uint32_t)i2c->tx_data, (i2c->tx_data_e - i2c->tx_data), STM32WB_I2C_TX_DMA_OPTION);

        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));

        I2C->CR1 |= (I2C_CR1_STOPIE | I2C_CR1_TCIE);

        I2C->ISR |= I2C_ISR_TXE;

        i2c->tx_data += count;
    }
    else
    {
        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));

	I2C->CR1 |= (I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);

        if (count)
        {
            I2C->ISR |= I2C_ISR_TXE;

            I2C->TXDR = *(i2c->tx_data)++;
        }
    }
}

static void stm32wb_i2c_master_receive(stm32wb_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;
    uint32_t i2c_cr2, count;
    bool rx_dma = false;

    i2c->state = STM32WB_I2C_STATE_MASTER_RECEIVE;

    count = i2c->rx_data_e - i2c->rx_data;

    i2c_cr2 = (i2c->xf_address << 1) | I2C_CR2_RD_WRN | I2C_CR2_START;

    /* WAR for ERRATA if NBYTES != 1 and RELOAD is set. This case is handled by using
     * interrupt driven receive mode with NBYTES set to 1 for each byte. This is
     * only affecting receive operations with more than I2C_CR2_NBYTES_MAX bytes.
     */

    if (count > I2C_CR2_NBYTES_MAX)
    {
        count = 1;

        i2c_cr2 |= I2C_CR2_RELOAD;
    }
    else
    {
        if (!(i2c->xf_control & STM32WB_I2C_CONTROL_RESTART))
        {
            i2c_cr2 |= I2C_CR2_AUTOEND;
        }
    }

    if ((count > 1) && (i2c->rx_dma != STM32WB_DMA_CHANNEL_NONE))
    {
        rx_dma = stm32wb_dma_channel(i2c->rx_dma);

        if (!rx_dma)
        {
            rx_dma = stm32wb_dma_enable(i2c->rx_dma, i2c->priority, NULL, NULL);
        }
    }

    if ((count > 1) && rx_dma)
    {
        I2C->CR1 |= I2C_CR1_RXDMAEN;

        stm32wb_dma_start(i2c->rx_dma, (uint32_t)i2c->rx_data, (uint32_t)&I2C->RXDR, (i2c->rx_data_e - i2c->rx_data), STM32WB_I2C_RX_DMA_OPTION);

        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));

        I2C->CR1 |= (I2C_CR1_STOPIE | I2C_CR1_TCIE);

        i2c->rx_data += count;
    }
    else
    {
        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));

        if (count <= 1)
        {
            /* No need to trigger an extra RXNE interrupt if only one byte 
             * is read. TCR/TC will cover that.
             */
            I2C->CR1 |= (I2C_CR1_STOPIE | I2C_CR1_TCIE);
        }
        else
        {
            I2C->CR1 |= (I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);
        }
    }
}

static void stm32wb_i2c_master_check(stm32wb_i2c_t *i2c)
{
    stm32wb_i2c_transaction_t *transaction, *transaction_next, *transaction_previous, *transaction_head, *transaction_tail;

    if (i2c->xf_submit != STM32WB_I2C_TRANSACTION_SENTINEL)
    {
        transaction = (stm32wb_i2c_transaction_t*)armv7m_atomic_swap((volatile uint32_t*)&i2c->xf_submit, (uint32_t)STM32WB_I2C_TRANSACTION_SENTINEL);

        /* Revert the submit queue and update head/tail
         */
        for (transaction_head = STM32WB_I2C_TRANSACTION_SENTINEL, transaction_tail = transaction; transaction != STM32WB_I2C_TRANSACTION_SENTINEL; transaction = transaction_next)
        {
            transaction_next = transaction->next;
            
            transaction->next = transaction_head;
     
            transaction_head = transaction;
        }

        if (i2c->xf_head == STM32WB_I2C_TRANSACTION_SENTINEL)
        {
            i2c->xf_head = transaction_head;
        }
        else
        {
            i2c->xf_tail->next = transaction_head;
        }

        i2c->xf_tail = transaction_tail;
    }

    if ((i2c->state != STM32WB_I2C_STATE_SUSPENDED) && ((i2c->state == STM32WB_I2C_STATE_MASTER_RESTART) || !i2c->rq_callback))
    {
        transaction = i2c->xf_head;

        if (transaction != STM32WB_I2C_TRANSACTION_SENTINEL)
        {
            if (i2c->state == STM32WB_I2C_STATE_MASTER_RESTART)
            {
                for (transaction_previous = STM32WB_I2C_TRANSACTION_SENTINEL; transaction != STM32WB_I2C_TRANSACTION_SENTINEL; transaction_previous = transaction, transaction = transaction->next)
                {
                    if (transaction->address == i2c->xf_address)
                    {
                        if (transaction == i2c->xf_head)
                        {
                            i2c->xf_head = transaction->next;
                        }
                        else
                        {
			    transaction_previous->next = transaction->next;
                        }
                        
                        if (transaction == i2c->xf_tail)
                        {
                            i2c->xf_tail = transaction_previous;
                        }
                        break;
                    }
                }
            }
            else
            {
                if (i2c->xf_head == i2c->xf_tail)
                {
                    i2c->xf_head = STM32WB_I2C_TRANSACTION_SENTINEL;
                    i2c->xf_tail = STM32WB_I2C_TRANSACTION_SENTINEL;
                }
                else
                {
                    i2c->xf_head = transaction->next;
                }
            }

            if (transaction != STM32WB_I2C_TRANSACTION_SENTINEL)
            {
                if (i2c->state == STM32WB_I2C_STATE_READY)
                {
                    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
                    
                    stm32wb_i2c_start(i2c);
                }

                i2c->xf_transaction = transaction;
                
                i2c->xf_address = transaction->address;
                i2c->xf_control = transaction->control;
                
                i2c->tx_data = NULL;
                i2c->tx_data_e = NULL;
		i2c->rx_data = NULL;
		i2c->rx_data_e = NULL;

                if (transaction->tx_count)
                {
                    i2c->tx_data = transaction->tx_data;
                    i2c->tx_data_e = transaction->tx_data + transaction->tx_count;
                }
                
                if (transaction->rx_count)
                {
                    if (!(i2c->xf_control & STM32WB_I2C_CONTROL_DIRECTION))
                    {
                        i2c->rx_data = transaction->rx_data;
                        i2c->rx_data_e = transaction->rx_data + transaction->rx_count;
                    }
                    
                    if (transaction->tx_count)
                    {
                        stm32wb_i2c_master_transmit(i2c);
                    }
                    else
                    {
                        stm32wb_i2c_master_receive(i2c);
                    }
                }
                else
                {
                    /* This here allows for a 0 length transfer to ping the bus address.
                     */

                    i2c->xf_control &= ~STM32WB_I2C_CONTROL_DIRECTION;

                    stm32wb_i2c_master_transmit(i2c);
                }
            }
        }
    }
    
    if (!i2c->xf_transaction)
    {
        if (i2c->state == STM32WB_I2C_STATE_MASTER_STOP)
        {
            if (stm32wb_dma_channel(i2c->rx_dma))
            {
                stm32wb_dma_disable(i2c->rx_dma);
            }

            if (stm32wb_dma_channel(i2c->tx_dma))
            {
                stm32wb_dma_disable(i2c->tx_dma);
            }
            
            stm32wb_i2c_stop(i2c);

            stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);

            i2c->state = STM32WB_I2C_STATE_READY;
        }

        if (i2c->state == STM32WB_I2C_STATE_READY)
        {
            if (i2c->rq_sync)
            {
                stm32wb_i2c_sync(i2c);
            }

            if (i2c->rq_callback)
            {
                stm32wb_gpio_pin_configure(i2c->pins.scl, (STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_INPUT));
                stm32wb_gpio_pin_configure(i2c->pins.sda, (STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_INPUT));
                
                i2c->state = STM32WB_I2C_STATE_SUSPENDED;

                NVIC_DisableIRQ(i2c->interrupt);

                if (i2c->rq_callback != STM32WB_I2C_SUSPEND_CALLBACK)
                {
                    (*i2c->rq_callback)(i2c->rq_context);
                }

                i2c->rq_callback = NULL;
            }
        }
    }
}

static void stm32wb_i2c_slave_check(stm32wb_i2c_t *i2c)
{
    if (i2c->rq_sync)
    {
        stm32wb_i2c_sync(i2c);
    }

    if (i2c->rq_callback)
    {
        stm32wb_i2c_stop(i2c);

        stm32wb_gpio_pin_configure(i2c->pins.scl, (STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_INPUT));
        stm32wb_gpio_pin_configure(i2c->pins.sda, (STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_INPUT));
        
        i2c->state = STM32WB_I2C_STATE_SUSPENDED;

        NVIC_DisableIRQ(i2c->interrupt);

        if (i2c->rq_callback != STM32WB_I2C_SUSPEND_CALLBACK)
        {
            (*i2c->rq_callback)(i2c->rq_context);
        }
        
        i2c->rq_callback = NULL;
    }
}

static void stm32wb_i2c_slave_address(stm32wb_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;

    i2c->xf_address = (I2C->ISR >> 17) & 0x7f;
    i2c->xf_count = 0;

    if (I2C->ISR & I2C_ISR_DIR)
    {
	i2c->state = STM32WB_I2C_STATE_SLAVE_TRANSMIT;

        i2c->tx_data = NULL;
        i2c->tx_data_e = NULL;

        (*i2c->ev_callback)(i2c->ev_context, STM32WB_I2C_EVENT_TRANSMIT_REQUEST | (i2c->xf_address << STM32WB_I2C_EVENT_ADDRESS_SHIFT));

        I2C->CR2 = 0;
            
        I2C->CR1 |= (I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);

        I2C->CR1 &= ~I2C_CR1_SBC;

        I2C->ISR |= I2C_ISR_TXE;
            
        if (i2c->tx_data)
        {
            I2C->TXDR = *(i2c->tx_data)++;

            if (i2c->tx_data == i2c->tx_data_e)
            {
                i2c->tx_data = NULL;
                i2c->tx_data_e = NULL;
            }
        }
        else
        {
            I2C->TXDR = 0xff;
        }
        
        i2c->xf_count++;
    }
    else
    {
        i2c->state = STM32WB_I2C_STATE_SLAVE_RECEIVE;

        i2c->rx_data = NULL;
        i2c->rx_data_e = NULL;

        (*i2c->ev_callback)(i2c->ev_context, STM32WB_I2C_EVENT_RECEIVE_REQUEST | (i2c->xf_address << STM32WB_I2C_EVENT_ADDRESS_SHIFT));

        I2C->CR2 = (I2C_CR2_RELOAD | (1 << I2C_CR2_NBYTES_SHIFT));
                
        I2C->CR1 |= (I2C_CR1_SBC | I2C_CR1_STOPIE | I2C_CR1_TCIE);
    }

    I2C->ICR = I2C_ICR_ADDRCF;
}

static __attribute__((optimize("O3"))) void stm32wb_i2c_interrupt(stm32wb_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;
    stm32wb_i2c_transaction_t *transaction = i2c->xf_transaction;
    uint32_t i2c_isr, i2c_cr2, count;
    bool tx_dma;
    uint8_t status;
    stm32wb_i2c_done_callback_t callback;
    void *context;
    
    i2c_isr = I2C->ISR;

    switch (i2c->state) {

    case STM32WB_I2C_STATE_NONE:
    case STM32WB_I2C_STATE_INIT:
        break;

    case STM32WB_I2C_STATE_READY:
        if (i2c->option & STM32WB_I2C_OPTION_ADDRESS_MASK)
        {
            if (i2c_isr & I2C_ISR_ADDR)
            {
                stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);

                stm32wb_i2c_slave_address(i2c);
            }
            else
            {
                stm32wb_i2c_slave_check(i2c);
            }
        }
        else
        {
            if (i2c->rq_sync)
            {
                stm32wb_i2c_sync(i2c);
            }

            stm32wb_i2c_master_check(i2c);
        }
        break;

    case STM32WB_I2C_STATE_SUSPENDED:
        break;
        
    case STM32WB_I2C_STATE_MASTER_STOP:
    case STM32WB_I2C_STATE_MASTER_RESTART:
        stm32wb_i2c_master_check(i2c);
        break;

    case STM32WB_I2C_STATE_MASTER_TRANSMIT:
        if (i2c_isr & I2C_ISR_TXIS)
        {
            if (!(I2C->CR1 & I2C_CR1_TXDMAEN))
            {
                I2C->TXDR = *(i2c->tx_data)++;
            }
        }

        if (i2c_isr & (I2C_ISR_TCR | I2C_ISR_TC | I2C_ISR_STOPF))
        {
            if (i2c_isr & (I2C_ISR_TC | I2C_ISR_STOPF))
            {
                I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);
                
                I2C->ICR = I2C_ICR_STOPCF;
                
                if (I2C->CR1 & I2C_CR1_TXDMAEN)
                {
                    i2c->tx_data = transaction->tx_data + stm32wb_dma_stop(i2c->tx_dma);
                    
                    I2C->CR1 &= ~I2C_CR1_TXDMAEN;
                }

                if (i2c_isr & (I2C_ISR_TIMEOUT | I2C_ISR_ARLO | I2C_ISR_NACKF))
                {
                    I2C->ICR = I2C_ICR_TIMOUTCF | I2C_ICR_ARLOCF | I2C_ICR_NACKCF;
                    
                    if (i2c_isr & I2C_ISR_NACKF)
                    {
                        if (!(i2c_isr & I2C_ISR_TXE))
                        {
                            i2c->tx_data--;
                        }

                        status = ((i2c->tx_data == transaction->tx_data) ? STM32WB_I2C_STATUS_ADDRESS_NACK : STM32WB_I2C_STATUS_DATA_NACK);
                    }
                    else
                    {
                        if (i2c_isr & I2C_ISR_ARLO)
                        {
                            status = STM32WB_I2C_STATUS_FAILURE;
                        }
                        else
                        {
                            status = STM32WB_I2C_STATUS_TIMEOUT;
                        }
                    }

                    i2c->state = STM32WB_I2C_STATE_MASTER_STOP;
                }
                else
                {
                    if (i2c->rx_data)
                    {
                        stm32wb_i2c_master_receive(i2c);
                        
                        status = STM32WB_I2C_STATUS_BUSY;
                    }
                    else
                    {
                        if (i2c->xf_control & STM32WB_I2C_CONTROL_RESTART)
                        {
                            i2c->state = STM32WB_I2C_STATE_MASTER_RESTART;
                        }
                        else
                        {
                            i2c->state = STM32WB_I2C_STATE_MASTER_STOP;
                        }
                        
                        status = STM32WB_I2C_STATUS_SUCCESS;
                    }
                }

                if (status != STM32WB_I2C_STATUS_BUSY)
                {
                    i2c->xf_transaction = NULL;

                    callback = transaction->callback;
                    context = transaction->context;

                    transaction->next = NULL;
                    transaction->status = status;
                    
                    if (callback)
                    {
                        (*callback)(context);
                    }
                    
                    stm32wb_i2c_master_check(i2c);
                }
            }
            else /* I2C_ISR_TCR */
            {
                i2c_cr2 = I2C->CR2 & ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD | I2C_CR2_NBYTES | I2C_CR2_NACK | I2C_CR2_STOP | I2C_CR2_START);
                
                count = i2c->tx_data_e - i2c->tx_data;

                if (count)
                {
                    if (count > I2C_CR2_NBYTES_MAX)
                    {
                        count = I2C_CR2_NBYTES_MAX;
                        
                        i2c_cr2 |= I2C_CR2_RELOAD;
                    }
                    else
                    {
                        if (i2c->xf_control & STM32WB_I2C_CONTROL_DIRECTION)
                        {
                            i2c_cr2 |= I2C_CR2_RELOAD;
                        }
                        else
                        {
                            if (!i2c->rx_data && !(i2c->xf_control & STM32WB_I2C_CONTROL_RESTART))
                            {
                                i2c_cr2 |= I2C_CR2_AUTOEND;
                            }
                        }
                    }
                    
                    I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));
                    
                    if (I2C->CR1 & I2C_CR1_TXDMAEN)
                    {
                        i2c->tx_data += count;
                    }
                    else
                    {
                        if (I2C->ISR & I2C_ISR_TXE)
                        {
                            I2C->TXDR = *(i2c->tx_data)++;
                        }
                    }
                }
                else
                {
                    if (i2c->rx_data) { __BKPT(); }
                    
                    i2c->xf_control &= ~STM32WB_I2C_CONTROL_DIRECTION;

                    i2c->tx_data = transaction->rx_data;
                    i2c->tx_data_e = transaction->rx_data + transaction->rx_count;

                    count = i2c->tx_data_e - i2c->tx_data;

                    if (count > I2C_CR2_NBYTES_MAX)
                    {
                        count = I2C_CR2_NBYTES_MAX;
                        
                        i2c_cr2 |= I2C_CR2_RELOAD;
                    }
                    else
                    {
                        if (!(i2c->xf_control & STM32WB_I2C_CONTROL_RESTART))
                        {
                            i2c_cr2 |= I2C_CR2_AUTOEND;
                        }
                    }

                    tx_dma = false;
    
                    if ((count > 1) && (i2c->tx_dma != STM32WB_DMA_CHANNEL_NONE))
                    {
                        tx_dma = stm32wb_dma_channel(i2c->tx_dma);
                        
                        if (!tx_dma)
                        {
                            tx_dma = stm32wb_dma_enable(i2c->tx_dma, i2c->priority, NULL, NULL);
                        }
                    }

                    if ((count > 1) && tx_dma)
                    {
                        I2C->CR1 = (I2C->CR1 & ~I2C_CR1_TXIE) | I2C_CR1_TXDMAEN;

                        stm32wb_dma_start(i2c->tx_dma, (uint32_t)&I2C->TXDR, (uint32_t)i2c->tx_data, (i2c->tx_data_e - i2c->tx_data), STM32WB_I2C_TX_DMA_OPTION);

                        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));

                        i2c->tx_data += count;
                    }
                    else
                    {
                        I2C->CR1 &= ~I2C_CR1_TXDMAEN;

                        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));
                        
                        I2C->CR1 |= I2C_CR1_TXIE;

                        if (I2C->ISR & I2C_ISR_TXE)
                        {
                            I2C->TXDR = *(i2c->tx_data)++;
                        }
                    }
                }
            }
        }
        break;

    case STM32WB_I2C_STATE_MASTER_RECEIVE:
        if (i2c_isr & I2C_ISR_RXNE)
        {
            if (!(I2C->CR1 & I2C_CR1_RXDMAEN))
            {
                *(i2c->rx_data)++ = I2C->RXDR;
            }
        }
        
        if (i2c_isr & (I2C_ISR_TCR | I2C_ISR_TC | I2C_ISR_STOPF))
        {
            if (i2c_isr & (I2C_ISR_TC | I2C_ISR_STOPF))
            {
                I2C->CR1 &= ~(I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);

                I2C->ICR = I2C_ICR_STOPCF;
                
                if (I2C->CR1 & I2C_CR1_RXDMAEN)
                {
                    stm32wb_dma_stop(i2c->rx_dma);
                    
                    I2C->CR1 &= ~I2C_CR1_RXDMAEN;
                }
                
                if (i2c_isr & (I2C_ISR_TIMEOUT | I2C_ISR_ARLO | I2C_ISR_NACKF))
                {
                    I2C->ICR = I2C_ICR_TIMOUTCF | I2C_ICR_ARLOCF | I2C_ICR_NACKCF;
                    
                    if (i2c_isr & I2C_ISR_NACKF)
                    {
                        status = STM32WB_I2C_STATUS_ADDRESS_NACK;
                    }
                    else
                    {
                        if (i2c_isr & I2C_ISR_ARLO)
                        {
                            status = STM32WB_I2C_STATUS_FAILURE;
                        }
                        else
                        {
                            status = STM32WB_I2C_STATUS_TIMEOUT;
                        }
                    }

                    i2c->state = STM32WB_I2C_STATE_MASTER_STOP;
                }
                else
                {
                    if (i2c->xf_control & STM32WB_I2C_CONTROL_RESTART)
                    {
                        i2c->state = STM32WB_I2C_STATE_MASTER_RESTART;
                    }
                    else
                    {
                        i2c->state = STM32WB_I2C_STATE_MASTER_STOP;
                    }
                    
                    status = STM32WB_I2C_STATUS_SUCCESS;
                }

                if (status != STM32WB_I2C_STATUS_BUSY)
                {
                    i2c->xf_transaction = NULL;

                    callback = transaction->callback;
                    context = transaction->context;

                    transaction->next = NULL;
                    transaction->status = status;
                    
                    if (callback)
                    {
                        (*callback)(context);
                    }
                    
                    stm32wb_i2c_master_check(i2c);
                }
            }
            else /* I2C_ISR_TCR */
            {
                i2c_cr2 = I2C->CR2 & ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD | I2C_CR2_NBYTES | I2C_CR2_NACK |  I2C_CR2_STOP | I2C_CR2_START);
                
                count = (i2c->rx_data_e - i2c->rx_data);
                
                if (count > 1)
                {
                    i2c_cr2 |= I2C_CR2_RELOAD;
                }
                else
                {
                    if (!(i2c->xf_control & STM32WB_I2C_CONTROL_RESTART))
                    {
                        i2c_cr2 |= I2C_CR2_AUTOEND;
                    }
                }
                        
                I2C->CR2 = (i2c_cr2 | (1 << I2C_CR2_NBYTES_SHIFT));
            }
        }
        break;

    case STM32WB_I2C_STATE_SLAVE_TRANSMIT:
        /* A slave transmit is terminated by a NACK followed by a STOP of the master receiver.
         */

        if (i2c_isr & (I2C_ISR_ADDR | I2C_ISR_STOPF))
        {
            I2C->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

            I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_STOPIE);
            
            /* If I2C_TXDR is not empty, there there is an unsent byte.
             */
            if (!(I2C->ISR & I2C_ISR_TXE))
            {
                i2c->xf_count--;
            }
            
            i2c->state = STM32WB_I2C_STATE_READY;
                
            (*i2c->ev_callback)(i2c->ev_context, STM32WB_I2C_EVENT_TRANSMIT_DONE | (i2c->xf_count << STM32WB_I2C_EVENT_COUNT_SHIFT));

            if (I2C->ISR & I2C_ISR_ADDR)
            {
                stm32wb_i2c_slave_address(i2c);
            }
            else
            {
                stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);

                stm32wb_i2c_slave_check(i2c);
            }
        }
        else
        {
            if (i2c_isr & I2C_ISR_TXIS)
            {
                if (i2c->tx_data)
                {
                    I2C->TXDR = *(i2c->tx_data)++;

                    if (i2c->tx_data == i2c->tx_data_e)
                    {
                        i2c->tx_data = NULL;
                        i2c->tx_data_e = NULL;
                    }
                }
                else
                {
                    I2C->TXDR = 0xff;
                }
                
                i2c->xf_count++;
            }
        }
        break;

    case STM32WB_I2C_STATE_SLAVE_RECEIVE:
        /* A slave receive is terminated either by STOP, or by a repeated
         * start, i.e. an ADDR match.
         */

        if (i2c_isr & (I2C_ISR_ADDR | I2C_ISR_STOPF))
        {
            I2C->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

            I2C->CR1 &= ~(I2C_CR1_STOPIE | I2C_CR1_TCIE);
            
            i2c->state = STM32WB_I2C_STATE_READY;
                
            (*i2c->ev_callback)(i2c->ev_context, STM32WB_I2C_EVENT_RECEIVE_DONE | (i2c->xf_count << STM32WB_I2C_EVENT_COUNT_SHIFT));

            if (i2c_isr & I2C_ISR_ADDR)
            {
                stm32wb_i2c_slave_address(i2c);
            }
            else
            {
                stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);

                stm32wb_i2c_slave_check(i2c);
            }
        }
        else
        {
            if (i2c_isr & I2C_ISR_TCR)
            {
                i2c_cr2 = I2C->CR2 & ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD | I2C_CR2_NBYTES | I2C_CR2_NACK |  I2C_CR2_STOP | I2C_CR2_START);

                if (i2c->rx_data)
                {
                    *(i2c->rx_data)++ = I2C->RXDR;

                    i2c->xf_count++;
                    
                    if (i2c->rx_data == i2c->rx_data_e)
                    {
                        i2c->rx_data = NULL;
                        i2c->rx_data_e = NULL;
                    }
                }
                else
                {
                    I2C->RXDR;
                    
                    i2c_cr2 |= I2C_CR2_NACK;
                }

                I2C->CR2 = i2c_cr2 | I2C_CR2_RELOAD | (1 << I2C_CR2_NBYTES_SHIFT);
            }
        }
        break;
    }
}

bool stm32wb_i2c_create(stm32wb_i2c_t *i2c, const stm32wb_i2c_params_t *params)
{
    if (i2c->state != STM32WB_I2C_STATE_NONE)
    {
        return false;
    }

    i2c->I2C = stm32wb_i2c_xlate_I2C[params->instance];
    i2c->instance = params->instance;
    i2c->interrupt = stm32wb_i2c_xlate_IRQn[params->instance];
    i2c->priority = params->priority;
    i2c->rx_dma = params->rx_dma;
    i2c->tx_dma = params->tx_dma;
    i2c->pins = params->pins;

    i2c->state = STM32WB_I2C_STATE_INIT;

    return true;
}

bool stm32wb_i2c_destroy(stm32wb_i2c_t *i2c)
{
    if (i2c->state != STM32WB_I2C_STATE_INIT)
    {
        return false;
    }

    i2c->state = STM32WB_I2C_STATE_NONE;

    return true;
}

bool stm32wb_i2c_enable(stm32wb_i2c_t *i2c, uint32_t option, uint32_t timeout, stm32wb_i2c_event_callback_t callback, void *context)
{
    if (i2c->state != STM32WB_I2C_STATE_INIT)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&stm32wb_i2c_device.instances[i2c->instance], (uint32_t)NULL, (uint32_t)i2c) != (uint32_t)NULL)
    {
        return false;
    }
    
    i2c->state = STM32WB_I2C_STATE_NOT_READY;

    i2c->ev_callback = callback;
    i2c->ev_context = context;

    i2c->xf_head = STM32WB_I2C_TRANSACTION_SENTINEL;
    i2c->xf_tail = STM32WB_I2C_TRANSACTION_SENTINEL;
    i2c->xf_submit = STM32WB_I2C_TRANSACTION_SENTINEL;
    i2c->xf_transaction = NULL;

    if (!stm32wb_i2c_configure(i2c, option, timeout))
    {
	stm32wb_i2c_device.instances[i2c->instance] = NULL;
	
        i2c->state = STM32WB_I2C_STATE_INIT;
        
        return false;
    }

    i2c->state = STM32WB_I2C_STATE_READY;

    NVIC_SetPriority(i2c->interrupt, i2c->priority);
    NVIC_EnableIRQ(i2c->interrupt);

    return true;
}

bool stm32wb_i2c_disable(stm32wb_i2c_t *i2c)
{
    if (i2c->state != STM32WB_I2C_STATE_READY)
    {
        return false;
    }
    
    NVIC_DisableIRQ(i2c->interrupt);

    if (i2c->option & STM32WB_I2C_OPTION_ADDRESS_MASK)
    {
        stm32wb_i2c_stop(i2c);

        armv7m_atomic_and(&EXTI->IMR1, ~stm32wb_i2c_xlate_IMR[i2c->instance]);
    }

    armv7m_atomic_and(&SYSCFG->CFGR1, ~stm32wb_i2c_xlate_FMP[i2c->instance]);

    stm32wb_i2c_device.instances[i2c->instance] = NULL;

    i2c->state = STM32WB_I2C_STATE_INIT;

    return true;
}

bool stm32wb_i2c_configure(stm32wb_i2c_t *i2c, uint32_t option, uint32_t timeout)
{
    if (i2c->state < STM32WB_I2C_STATE_NOT_READY)
    {
        return false;
    }

    i2c->option = option;
    i2c->timeout = timeout;

    i2c->rq_sync = true;

    if (__current_irq() != i2c->interrupt)
    {
        NVIC_SetPendingIRQ(i2c->interrupt);
    }

    return true;
}

bool stm32wb_i2c_suspend(stm32wb_i2c_t *i2c, stm32wb_i2c_suspend_callback_t callback, void *context)
{
    if (i2c->rq_callback)
    {
        return false;
    }

    if (i2c->state < STM32WB_I2C_STATE_READY)
    {
        return false;
    }

    if (callback)
    {
        i2c->rq_context = context;
        i2c->rq_callback = callback;
    }
    else
    {
        i2c->rq_callback = STM32WB_I2C_SUSPEND_CALLBACK;
    }

    if (__current_irq() != i2c->interrupt)
    {
        NVIC_SetPendingIRQ(i2c->interrupt);
    }

    return true;
}

void stm32wb_i2c_resume(stm32wb_i2c_t *i2c)
{
    if (i2c->state == STM32WB_I2C_STATE_SUSPENDED)
    {
        i2c->rq_callback = NULL;

        if (i2c->option & STM32WB_I2C_OPTION_WAKEUP)
        {
            stm32wb_gpio_pin_configure(i2c->pins.scl, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_ALTERNATE));
            stm32wb_gpio_pin_configure(i2c->pins.sda, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_ALTERNATE));
        }
        else
        {
            stm32wb_gpio_pin_configure(i2c->pins.scl, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_ALTERNATE));
            stm32wb_gpio_pin_configure(i2c->pins.sda, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_ALTERNATE));
        }
        
        if (i2c->option & STM32WB_I2C_OPTION_ADDRESS_MASK) 
        {
            stm32wb_i2c_start(i2c);
        }
        
        i2c->state = STM32WB_I2C_STATE_READY;
        
        NVIC_EnableIRQ(i2c->interrupt);
        
        NVIC_SetPendingIRQ(i2c->interrupt);
    }
}

bool stm32wb_i2c_reset(stm32wb_i2c_t *i2c)
{
    uint32_t pin_scl, pin_sda, count;

    if (i2c->state != STM32WB_I2C_STATE_SUSPENDED)
    {
        return false;
    }

    pin_scl = i2c->pins.scl;
    pin_sda = i2c->pins.sda;

    stm32wb_gpio_pin_configure(pin_scl, (STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_INPUT));
    stm32wb_gpio_pin_configure(pin_sda, (STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_OPENDRAIN | STM32WB_GPIO_MODE_INPUT));
    stm32wb_gpio_pin_write(pin_scl, 0);
    stm32wb_gpio_pin_write(pin_sda, 1);
    stm32wb_gpio_pin_output(pin_scl);
    stm32wb_gpio_pin_output(pin_sda);
    
    /* Clock 16 SCL cycles to force a slave to release SDA and then issue a manual STOP condition.
     * The spec suggest 9 SCL cycles, but in practice more are needed. 
     */
    
    for (count = 0; count < 16; count++)
    {
        armv7m_core_udelay(6);    
        
        /* Set SCL to H */ 
        stm32wb_gpio_pin_write(pin_scl, 1);
        armv7m_core_udelay(4);    
        
        /* Set SCL to L */ 
        stm32wb_gpio_pin_write(pin_scl, 0);
    }
    
    // Send a STOP condition (SCL = 1, SDA = 0 -> 1)
    armv7m_core_udelay(1);    
    stm32wb_gpio_pin_write(pin_sda, 0);
    armv7m_core_udelay(1);    
    stm32wb_gpio_pin_write(pin_scl, 1);
    armv7m_core_udelay(2);    
    stm32wb_gpio_pin_write(pin_sda, 1);
    armv7m_core_udelay(40);    

    stm32wb_gpio_pin_input(pin_sda);
    stm32wb_gpio_pin_input(pin_scl);

    return true;
}

bool stm32wb_i2c_receive(stm32wb_i2c_t *i2c, uint8_t *rx_data, uint16_t rx_count)
{
    if (i2c->state != STM32WB_I2C_STATE_SLAVE_RECEIVE)
    {
        return false;
    }

    if (rx_count)
    {
        i2c->rx_data = rx_data;
        i2c->rx_data_e = rx_data + rx_count;
    }

    return true;
}

bool stm32wb_i2c_transmit(stm32wb_i2c_t *i2c, uint8_t *tx_data, uint16_t tx_count)
{
    if (i2c->state != STM32WB_I2C_STATE_SLAVE_TRANSMIT)
    {
        return false;
    }

    if (tx_count)
    {
        i2c->tx_data = tx_data;
        i2c->tx_data_e = tx_data + tx_count;
    }

    return true;
}

bool stm32wb_i2c_submit(stm32wb_i2c_t *i2c, stm32wb_i2c_transaction_t *transaction)
{
    stm32wb_i2c_transaction_t *transaction_submit;
    
    if (i2c->state < STM32WB_I2C_STATE_READY)
    {
        return false;
    }

    if (i2c->option & STM32WB_I2C_OPTION_ADDRESS_MASK)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&transaction->next, (uint32_t)NULL, (uint32_t)STM32WB_I2C_TRANSACTION_SENTINEL) != (uint32_t)NULL)
    {
        return false;
    }

    transaction->status = STM32WB_I2C_STATUS_BUSY;

    do
    {
	transaction_submit = i2c->xf_submit;

	transaction->next = transaction_submit;
    }
    while ((stm32wb_i2c_transaction_t*)armv7m_atomic_cas((volatile uint32_t*)&i2c->xf_submit, (uint32_t)transaction_submit, (uint32_t)transaction) != transaction_submit);

    if (__current_irq() != i2c->interrupt)
    {
        NVIC_SetPendingIRQ(i2c->interrupt);
    }
    
    return true;
}

void I2C1_EV_IRQHandler(void)
{
    stm32wb_i2c_interrupt(stm32wb_i2c_device.instances[STM32WB_I2C_INSTANCE_I2C1]);

    __DSB();
}

void I2C3_EV_IRQHandler(void)
{
    stm32wb_i2c_interrupt(stm32wb_i2c_device.instances[STM32WB_I2C_INSTANCE_I2C3]);

    __DSB();
}
