/*
 * Copyright (c) 2021 Thomas Roell.  All rights reserved.
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

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[PINS_COUNT] =
{
    // 0..13 - Digital pins
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB10), STM32WB_GPIO_PIN_PB10_TIM2_CH3,  0,                                PWM_INSTANCE_TIM2,  PWM_CHANNEL_3,    ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB11), STM32WB_GPIO_PIN_PB11_TIM2_CH4,  0,                                PWM_INSTANCE_TIM2,  PWM_CHANNEL_4,    ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB12), STM32WB_GPIO_PIN_PB12,           (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB13), STM32WB_GPIO_PIN_PB13_TIM1_CH1N, (PIN_ATTR_EXTI),                  PWM_INSTANCE_TIM1,  PWM_CHANNEL_1,    ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB2),  STM32WB_GPIO_PIN_PB2,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC6),  STM32WB_GPIO_PIN_PC6,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC7),  STM32WB_GPIO_PIN_PC7,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC10), STM32WB_GPIO_PIN_PC10,           (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC11), STM32WB_GPIO_PIN_PC11,           (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA15), STM32WB_GPIO_PIN_PA15,           (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB14), STM32WB_GPIO_PIN_PB14,           (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB15), STM32WB_GPIO_PIN_PB15,           0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC2),  STM32WB_GPIO_PIN_PC2,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOD, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PD1),  STM32WB_GPIO_PIN_PD1,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 14..15 - I2C pins (SDA,SCL)
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC1),  STM32WB_GPIO_PIN_PC1,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC0),  STM32WB_GPIO_PIN_PC0,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 16..21 - Analog pins
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA0),  STM32WB_GPIO_PIN_PA0_TIM2_CH1,   (PIN_ATTR_EXTI | PIN_ATTR_WKUP1), PWM_INSTANCE_TIM2,  PWM_CHANNEL_1,    ADC_CHANNEL_5    },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA1),  STM32WB_GPIO_PIN_PA1_TIM2_CH2,   (PIN_ATTR_EXTI),                  PWM_INSTANCE_TIM2,  PWM_CHANNEL_2,    ADC_CHANNEL_6    },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA9),  STM32WB_GPIO_PIN_PA9_TIM1_CH2,   (PIN_ATTR_EXTI),                  PWM_INSTANCE_TIM1,  PWM_CHANNEL_2,    ADC_CHANNEL_16   },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA4),  STM32WB_GPIO_PIN_PA4,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_9    },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC5),  STM32WB_GPIO_PIN_PC5,            (PIN_ATTR_EXTI | PIN_ATTR_WKUP5), PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_14   },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC4),  STM32WB_GPIO_PIN_PC4,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_13   },

    // 22..23 - LED/BUTTON
    { GPIOE, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PE1),  STM32WB_GPIO_PIN_PE1,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PH3),  STM32WB_GPIO_PIN_PH3,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
};

extern const unsigned int g_PWMInstances[PWM_INSTANCE_COUNT] = {
    STM32WB_TIM_INSTANCE_TIM1,
    STM32WB_TIM_INSTANCE_TIM2,
};

extern const stm32wb_uart_params_t g_Serial1Params = {
    STM32WB_UART_INSTANCE_LPUART1,
    STM32WB_UART_IRQ_PRIORITY,
    STM32WB_DMA_CHANNEL_NONE,
    STM32WB_DMA_CHANNEL_NONE,
    NULL,
    0,
    {
        STM32WB_GPIO_PIN_PB10_LPUART1_RX,
        STM32WB_GPIO_PIN_PB11_LPUART1_TX,
        STM32WB_GPIO_PIN_PB12_LPUART1_RTS_DE,
        STM32WB_GPIO_PIN_PB13_LPUART1_CTS,
        STM32WB_GPIO_PIN_NONE,
    },
};

extern const stm32wb_spi_params_t g_SPIParams = {
    STM32WB_SPI_INSTANCE_SPI2,
    STM32WB_SPI_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA2_CH5_INDEX | STM32WB_DMA_CHANNEL_SELECT_SPI2_RX),
    (STM32WB_DMA_CHANNEL_DMA2_CH6_INDEX | STM32WB_DMA_CHANNEL_SELECT_SPI2_TX),
    {
        STM32WB_GPIO_PIN_PB15_SPI2_MOSI,
        STM32WB_GPIO_PIN_PC2_SPI2_MISO,
        STM32WB_GPIO_PIN_PD1_SPI2_SCK,
    },
};

extern const stm32wb_uart_params_t g_SPI1Params = {
    STM32WB_UART_INSTANCE_USART1,
    STM32WB_UART_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA1_CH2_INDEX | STM32WB_DMA_CHANNEL_SELECT_USART1_RX),
    (STM32WB_DMA_CHANNEL_DMA1_CH3_INDEX | STM32WB_DMA_CHANNEL_SELECT_USART1_TX),
    NULL,
    0,
    {
        STM32WB_GPIO_PIN_PB7_USART1_RX,
        STM32WB_GPIO_PIN_PB6_USART1_TX,
        STM32WB_GPIO_PIN_NONE,
        STM32WB_GPIO_PIN_NONE,
        STM32WB_GPIO_PIN_PB5_USART1_CK,
    },
};

extern const stm32wb_i2c_params_t g_WireParams = {
    STM32WB_I2C_INSTANCE_I2C3,
    STM32WB_I2C_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA1_CH7_INDEX | STM32WB_DMA_CHANNEL_SELECT_I2C3_RX),
    STM32WB_DMA_CHANNEL_NONE,
    {
        STM32WB_GPIO_PIN_PC0_I2C3_SCL,
        STM32WB_GPIO_PIN_PC1_I2C3_SDA,
    },
};

extern const stm32wb_i2c_params_t g_Wire1Params = {
    STM32WB_I2C_INSTANCE_I2C1,
    STM32WB_I2C_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA1_CH6_INDEX | STM32WB_DMA_CHANNEL_SELECT_I2C1_RX),
    STM32WB_DMA_CHANNEL_NONE,
    {
        STM32WB_GPIO_PIN_PB8_I2C1_SCL,
        STM32WB_GPIO_PIN_PA10_I2C1_SDA,
    },
};

static const stm32wb_spi_params_t g_SPI2Params = {
    STM32WB_SPI_INSTANCE_SPI1,
    STM32WB_SPI_IRQ_PRIORITY,
    STM32WB_DMA_CHANNEL_NONE,
    STM32WB_DMA_CHANNEL_NONE,
    {
        STM32WB_GPIO_PIN_PA7_SPI1_MOSI,
        STM32WB_GPIO_PIN_PA6_SPI1_MISO,
        STM32WB_GPIO_PIN_PA5_SPI1_SCK,
    },
};

extern const stm32wb_sai_params_t g_PDMParams = {
    STM32WB_SAI_INSTANCE_SAI1,
    STM32WB_SAI_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA2_CH1_INDEX | STM32WB_DMA_CHANNEL_SELECT_SAI1_A),
    (STM32WB_SAI_CONFIG_BLOCK_A | STM32WB_SAI_CONFIG_PDM_DI1),
    {
        STM32WB_GPIO_PIN_PA8_SAI1_PDM_CK2,
        STM32WB_GPIO_PIN_PC3_SAI1_PDM_DI1,
        STM32WB_GPIO_PIN_NONE,
        STM32WB_GPIO_PIN_NONE,
    },
};

static const stm32wb_sdspi_params_t g_SDSPI2Params =
{
    {
        STM32WB_GPIO_PIN_PB3,
    },
};

static stm32wb_spi_t g_SPI2;


extern const stm32wb_system_info_t stm32wb_system_info =
{
    .version           	= STM32WB_SYSTEM_VERSION,
    .options            = (STM32WB_SYSTEM_OPTION_DFU_USB | STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_10uH | STM32WB_SYSTEM_OPTION_SMPS_CURRENT_220mA | STM32WB_SYSTEM_OPTION_LSE_MODE_2),
    .hseclk             = 32000000,
    .lseclk             = 32774,
    .pins               = {
        .status         = STM32WB_GPIO_PIN_NONE,
        .boost          = STM32WB_GPIO_PIN_NONE,
        .dfu            = STM32WB_GPIO_PIN_PH3,
        .usb_vbus       = STM32WB_GPIO_PIN_PVM1,
        .uart_rx        = STM32WB_GPIO_PIN_NONE,
        .uart_tx        = STM32WB_GPIO_PIN_NONE,
        .sflash_cs      = STM32WB_GPIO_PIN_PD3_QUADSPI_NCS,
        .sflash_clk     = STM32WB_GPIO_PIN_PA3_QUADSPI_CLK,
        .sflash_mosi    = STM32WB_GPIO_PIN_PD4_QUADSPI_IO0,
        .sflash_miso    = STM32WB_GPIO_PIN_PD5_QUADSPI_IO1,
        .sflash_wp      = STM32WB_GPIO_PIN_PD6_QUADSPI_IO2,
        .sflash_hold    = STM32WB_GPIO_PIN_PD7_QUADSPI_IO3,
        .sflash_enable  = STM32WB_GPIO_PIN_NONE
    }
};

void initVariant()
{
    // stm32wb_spi_create(&g_SPI2, &g_SPI2Params);

    // stm32wb_sdspi_initialize(&g_SPI2, &g_SDSPI2Params);

    stm32wb_sfsqi_initialize();
}

