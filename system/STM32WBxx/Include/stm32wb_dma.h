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

#if !defined(_STM32WB_DMA_H)
#define _STM32WB_DMA_H

#include "armv7m.h"
#include "stm32wbxx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32WB_DMA_CHANNEL_NONE                      0x0000
#define STM32WB_DMA_CHANNEL_MASK                      0x0fff

#define STM32WB_DMA_CHANNEL_DMA1_CH1_INDEX            0
#define STM32WB_DMA_CHANNEL_DMA1_CH2_INDEX            1
#define STM32WB_DMA_CHANNEL_DMA1_CH3_INDEX            2
#define STM32WB_DMA_CHANNEL_DMA1_CH4_INDEX            3
#define STM32WB_DMA_CHANNEL_DMA1_CH5_INDEX            4
#define STM32WB_DMA_CHANNEL_DMA1_CH6_INDEX            5
#define STM32WB_DMA_CHANNEL_DMA1_CH7_INDEX            6
#define STM32WB_DMA_CHANNEL_DMA2_CH1_INDEX            7
#define STM32WB_DMA_CHANNEL_DMA2_CH2_INDEX            8
#define STM32WB_DMA_CHANNEL_DMA2_CH3_INDEX            9
#define STM32WB_DMA_CHANNEL_DMA2_CH4_INDEX            10
#define STM32WB_DMA_CHANNEL_DMA2_CH5_INDEX            11
#define STM32WB_DMA_CHANNEL_DMA2_CH6_INDEX            12
#define STM32WB_DMA_CHANNEL_DMA2_CH7_INDEX            13


#if defined(STM32WB55xx)

#define STM32WB_DMA_CHANNEL_SELECT_NONE               0x0000
#define STM32WB_DMA_CHANNEL_SELECT_DMAMUX_REQ_GEN0    0x0010
#define STM32WB_DMA_CHANNEL_SELECT_DMAMUX_REQ_GEN1    0x0020
#define STM32WB_DMA_CHANNEL_SELECT_DMAMUX_REQ_GEN2    0x0030
#define STM32WB_DMA_CHANNEL_SELECT_DMAMUX_REQ_GEN3    0x0040
#define STM32WB_DMA_CHANNEL_SELECT_ADC1               0x0050
#define STM32WB_DMA_CHANNEL_SELECT_SPI1_RX            0x0060
#define STM32WB_DMA_CHANNEL_SELECT_SPI1_TX            0x0070
#define STM32WB_DMA_CHANNEL_SELECT_SPI2_RX            0x0080
#define STM32WB_DMA_CHANNEL_SELECT_SPI2_TX            0x0090
#define STM32WB_DMA_CHANNEL_SELECT_I2C1_RX            0x00a0
#define STM32WB_DMA_CHANNEL_SELECT_I2C1_TX            0x00b0
#define STM32WB_DMA_CHANNEL_SELECT_I2C3_RX            0x00c0
#define STM32WB_DMA_CHANNEL_SELECT_I2C3_TX            0x00d0
#define STM32WB_DMA_CHANNEL_SELECT_USART1_RX          0x00e0
#define STM32WB_DMA_CHANNEL_SELECT_USART1_TX          0x00f0
#define STM32WB_DMA_CHANNEL_SELECT_LPUART1_RX         0x0100
#define STM32WB_DMA_CHANNEL_SELECT_LPUART1_TX         0x0110
#define STM32WB_DMA_CHANNEL_SELECT_SAI1_A             0x0120
#define STM32WB_DMA_CHANNEL_SELECT_SAI1_B             0x0130
#define STM32WB_DMA_CHANNEL_SELECT_QUADSPI            0x0140
#define STM32WB_DMA_CHANNEL_SELECT_TIM1_CH1           0x0150
#define STM32WB_DMA_CHANNEL_SELECT_TIM1_CH2           0x0160
#define STM32WB_DMA_CHANNEL_SELECT_TIM1_CH3           0x0170
#define STM32WB_DMA_CHANNEL_SELECT_TIM1_CH4           0x0180
#define STM32WB_DMA_CHANNEL_SELECT_TIM1_UP            0x0190
#define STM32WB_DMA_CHANNEL_SELECT_TIM1_TRIG          0x01a0
#define STM32WB_DMA_CHANNEL_SELECT_TIM1_COM           0x01b0
#define STM32WB_DMA_CHANNEL_SELECT_TIM2_CH1           0x01c0
#define STM32WB_DMA_CHANNEL_SELECT_TIM2_CH2           0x01d0
#define STM32WB_DMA_CHANNEL_SELECT_TIM2_CH3           0x01e0
#define STM32WB_DMA_CHANNEL_SELECT_TIM2_CH4           0x01f0
#define STM32WB_DMA_CHANNEL_SELECT_TIM2_UP            0x0200
#define STM32WB_DMA_CHANNEL_SELECT_TIM16_CH1          0x0210
#define STM32WB_DMA_CHANNEL_SELECT_TIM16_UP           0x0220
#define STM32WB_DMA_CHANNEL_SELECT_TIM17_CH1          0x0230
#define STM32WB_DMA_CHANNEL_SELECT_TIM17_UP           0x0240
#define STM32WB_DMA_CHANNEL_SELECT_AES1_IN            0x0250
#define STM32WB_DMA_CHANNEL_SELECT_AES1_OUT           0x0260
#define STM32WB_DMA_CHANNEL_SELECT_AES2_IN            0x0270
#define STM32WB_DMA_CHANNEL_SELECT_AES2_OUT           0x0280
  
#endif /* STM32WB55xx */
  
#define STM32WB_DMA_OPTION_EVENT_TRANSFER_DONE        0x00000002
#define STM32WB_DMA_OPTION_EVENT_TRANSFER_HALF        0x00000004
#define STM32WB_DMA_OPTION_EVENT_TRANSFER_ERROR       0x00000008

#define STM32WB_DMA_OPTION_PERIPHERAL_TO_MEMORY       0x00000000
#define STM32WB_DMA_OPTION_MEMORY_TO_PERIPHERAL       0x00000010
#define STM32WB_DMA_OPTION_CIRCULAR                   0x00000020
#define STM32WB_DMA_OPTION_PERIPHERAL_DATA_INCREMENT  0x00000040
#define STM32WB_DMA_OPTION_MEMORY_DATA_INCREMENT      0x00000080
#define STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_MASK  0x00000300
#define STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_SHIFT 8
#define STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_8     0x00000000
#define STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_16    0x00000100
#define STM32WB_DMA_OPTION_PERIPHERAL_DATA_SIZE_32    0x00000200
#define STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_MASK      0x00000c00
#define STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_SHIFT     10
#define STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_8         0x00000000
#define STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_16        0x00000400
#define STM32WB_DMA_OPTION_MEMORY_DATA_SIZE_32        0x00000800
#define STM32WB_DMA_OPTION_PRIORITY_MASK              0x00003000
#define STM32WB_DMA_OPTION_PRIORITY_SHIFT             12
#define STM32WB_DMA_OPTION_PRIORITY_LOW               0x00000000
#define STM32WB_DMA_OPTION_PRIORITY_MEDIUM            0x00001000
#define STM32WB_DMA_OPTION_PRIORITY_HIGH              0x00002000
#define STM32WB_DMA_OPTION_PRIORITY_VERY_HIGH         0x00003000
#define STM32WB_DMA_OPTION_MEMORY_TO_MEMORY           0x00004000

#define STM32WB_DMA_EVENT_TRANSFER_DONE               0x00000002
#define STM32WB_DMA_EVENT_TRANSFER_HALF               0x00000004
#define STM32WB_DMA_EVENT_TRANSFER_ERROR              0x00000008

typedef void (*stm32wb_dma_callback_t)(void *context, uint32_t events);
  
extern void __stm32wb_dma_initialize(void);
extern bool stm32wb_dma_channel(uint16_t channel);
extern bool stm32wb_dma_enable(uint16_t channel, uint8_t priority, stm32wb_dma_callback_t callback, void *context);
extern void stm32wb_dma_disable(uint16_t channel);
extern void stm32wb_dma_start(uint16_t channel, uint32_t tx_data, uint32_t rx_data, uint16_t xf_count, uint32_t option);
extern uint16_t stm32wb_dma_stop(uint16_t channel);
extern uint16_t stm32wb_dma_count(uint16_t channel);
extern bool stm32wb_dma_done(uint16_t channel);

#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_DMA_H */
