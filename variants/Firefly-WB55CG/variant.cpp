/*
 * Copyright (c) 2020 Thomas Roell.  All rights reserved.
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

#include "Arduino.h"
#include "wiring_private.h"

#define PWM_INSTANCE_TIM1      0
#define PWM_INSTANCE_TIM2      1
#define PWM_INSTANCE_TIM16     2

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[PINS_COUNT] =
{
    // 0..13 - Digital pins
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA10), STM32WB_GPIO_PIN_PA10,           0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA9),  STM32WB_GPIO_PIN_PA9,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB5),  STM32WB_GPIO_PIN_PB5,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB4),  STM32WB_GPIO_PIN_PB4,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA4),  STM32WB_GPIO_PIN_PA4,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB3),  STM32WB_GPIO_PIN_PB3_TIM2_CH2,   0,                                PWM_INSTANCE_TIM2,  PWM_CHANNEL_2,    ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB2),  STM32WB_GPIO_PIN_PB2,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB1),  STM32WB_GPIO_PIN_PB1,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB8),  STM32WB_GPIO_PIN_PB8_TIM1_CH2N,  (PIN_ATTR_EXTI),                  PWM_INSTANCE_TIM1,  PWM_CHANNEL_2,    ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB9),  STM32WB_GPIO_PIN_PB9_TIM1_CH3N,  (PIN_ATTR_EXTI),                  PWM_INSTANCE_TIM1,  PWM_CHANNEL_3,    ADC_CHANNEL_NONE },
#if (STORAGE_TYPE == 1) || (STORAGE_TYPE == 2)
    { NULL,  0,                                            STM32WB_GPIO_PIN_NONE,           0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  0,                                            STM32WB_GPIO_PIN_NONE,           0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  0,                                            STM32WB_GPIO_PIN_NONE,           0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  0,                                            STM32WB_GPIO_PIN_NONE,           0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
#else /* (STORAGE_TYPE == 1) || (STORAGE_TYPE == 2) */
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA15), STM32WB_GPIO_PIN_PA15_TIM2_CH1,  (PIN_ATTR_EXTI),                  PWM_INSTANCE_TIM2,  PWM_CHANNEL_1,    ADC_CHANNEL_NONE },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA7),  STM32WB_GPIO_PIN_PA7_TIM1_CH1N,  (PIN_ATTR_EXTI),                  PWM_INSTANCE_TIM1,  PWM_CHANNEL_1,    ADC_CHANNEL_NONE },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA6),  STM32WB_GPIO_PIN_PA6_TIM16_CH1,  (PIN_ATTR_EXTI),                  PWM_INSTANCE_TIM16, PWM_CHANNEL_1,    ADC_CHANNEL_NONE },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA5),  STM32WB_GPIO_PIN_PA5,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
#endif /* (STORAGE_TYPE == 1) || (STORAGE_TYPE == 2) */

    // 14..15 - I2C pins (SDA,SCL)
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB7),  STM32WB_GPIO_PIN_PB7,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB6),  STM32WB_GPIO_PIN_PB6,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 16..21 - Analog pins
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA0),  STM32WB_GPIO_PIN_PA0,            (PIN_ATTR_TAMP | PIN_ATTR_WKUP1), PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_5    },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA1),  STM32WB_GPIO_PIN_PA1,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_6    },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA2),  STM32WB_GPIO_PIN_PA2,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_7    },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA3),  STM32WB_GPIO_PIN_PA3,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_8    },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA8),  STM32WB_GPIO_PIN_PA8,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_15   },
    { NULL,  0,                                            STM32WB_GPIO_PIN_NONE,           0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 22..25 - SWCLK/SWDIO/LED/BUTTON
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA14), STM32WB_GPIO_PIN_PA14,           (PIN_ATTR_EXTI | PIN_ATTR_SWD),   PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA13), STM32WB_GPIO_PIN_PA13,           (PIN_ATTR_EXTI | PIN_ATTR_SWD),   PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOE, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PE4),  STM32WB_GPIO_PIN_PE4,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PH3),  STM32WB_GPIO_PIN_PH3,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
};

extern const unsigned int g_PWMInstances[PWM_INSTANCE_COUNT] = {
    STM32WB_TIM_INSTANCE_TIM1,
    STM32WB_TIM_INSTANCE_TIM2,
    STM32WB_TIM_INSTANCE_TIM16,
};

static uint8_t stm32wb_usart1_rx_fifo[32];

extern const stm32wb_uart_params_t g_Serial1Params = {
    STM32WB_UART_INSTANCE_USART1,
    STM32WB_UART_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA2_CH1_INDEX | STM32WB_DMA_CHANNEL_SELECT_USART1_RX),
    (STM32WB_DMA_CHANNEL_DMA2_CH2_INDEX | STM32WB_DMA_CHANNEL_SELECT_USART1_TX),
    &stm32wb_usart1_rx_fifo[0],
    sizeof(stm32wb_usart1_rx_fifo),
    {
        STM32WB_GPIO_PIN_PA10_USART1_RX,
        STM32WB_GPIO_PIN_PA9_USART1_TX,
        STM32WB_GPIO_PIN_PB5_USART1_CK,
        STM32WB_GPIO_PIN_PB4_USART1_CTS,
        STM32WB_GPIO_PIN_NONE,
    },
};

extern const stm32wb_uart_params_t g_Serial2Params = {
    STM32WB_UART_INSTANCE_LPUART1,
    STM32WB_UART_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA2_CH3_INDEX | STM32WB_DMA_CHANNEL_SELECT_LPUART1_RX),
    (STM32WB_DMA_CHANNEL_DMA2_CH4_INDEX | STM32WB_DMA_CHANNEL_SELECT_LPUART1_TX),
    NULL,
    0,
    {
        STM32WB_GPIO_PIN_PA3_LPUART1_RX,
        STM32WB_GPIO_PIN_PA2_LPUART1_TX,
        STM32WB_GPIO_PIN_NONE,
        STM32WB_GPIO_PIN_NONE,
        STM32WB_GPIO_PIN_NONE,
    },
};

extern const stm32wb_spi_params_t g_SPIParams = {
    STM32WB_SPI_INSTANCE_SPI1,
    STM32WB_SPI_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA2_CH5_INDEX | STM32WB_DMA_CHANNEL_SELECT_SPI1_RX),
    (STM32WB_DMA_CHANNEL_DMA2_CH6_INDEX | STM32WB_DMA_CHANNEL_SELECT_SPI1_TX),
    {
        STM32WB_GPIO_PIN_PA7_SPI1_MOSI,
        STM32WB_GPIO_PIN_PA6_SPI1_MISO,
        STM32WB_GPIO_PIN_PA5_SPI1_SCK,
    },
};

extern const stm32wb_i2c_params_t g_WireParams = {
    STM32WB_I2C_INSTANCE_I2C1,
    STM32WB_I2C_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA2_CH7_INDEX | STM32WB_DMA_CHANNEL_SELECT_I2C1_RX),
    STM32WB_DMA_CHANNEL_NONE,
    {
        STM32WB_GPIO_PIN_PB6_I2C1_SCL,
        STM32WB_GPIO_PIN_PB7_I2C1_SDA,
    },
};

extern const stm32wb_sai_params_t g_I2SParams = {
    STM32WB_SAI_INSTANCE_SAI1,
    STM32WB_SAI_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA1_CH2_INDEX | STM32WB_DMA_CHANNEL_SELECT_SAI1_B),
    (STM32WB_SAI_CONFIG_BLOCK_B),
    {
        STM32WB_GPIO_PIN_PB3_SAI1_SCK_B,
        STM32WB_GPIO_PIN_PB5_SAI1_SD_B,
        STM32WB_GPIO_PIN_PA4_SAI1_FS_B,
        STM32WB_GPIO_PIN_PB4_SAI1_MCLK_B,
    },
};

extern const stm32wb_sai_params_t g_PDMParams = {
    STM32WB_SAI_INSTANCE_SAI1,
    STM32WB_SAI_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA1_CH1_INDEX | STM32WB_DMA_CHANNEL_SELECT_SAI1_A),
    (STM32WB_SAI_CONFIG_BLOCK_A | STM32WB_SAI_CONFIG_PDM_DI2),
    {
        STM32WB_GPIO_PIN_PB8_SAI1_PDM_CK1,
        STM32WB_GPIO_PIN_PB9_SAI1_PDM_DI2,
        STM32WB_GPIO_PIN_NONE,
        STM32WB_GPIO_PIN_NONE,
    },
};

extern const stm32wb_sdspi_params_t g_SDSPIParams =
{
    {
        STM32WB_GPIO_PIN_PA15,
    },
};

extern const stm32wb_sfspi_params_t g_SFSPIParams =
{
    {
        STM32WB_GPIO_PIN_PA15,
    },
};
