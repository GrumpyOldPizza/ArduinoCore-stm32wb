/*
 * Copyright (c) 2024 Thomas Roell.  All rights reserved.
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
 * D0          PA0   BMA400_INT1
 * D1          PB12  BMA400_INT2
 * D2          PC10  LSM6DSM_INT1
 * D3          PC11  LSM6DSM_INT2
 *
 * A0          PC0
 * A1          PC1
 * A2          PC2
 * A3          PC3
 * A4          PA4
 * A5          PA5
 * A6          PA6
 * A7          PA7
 * A8          PA8
 * A9          PA9
 *
 * EXT_SCL     PB13
 * EXT_SDA     PB14
 *
 * LED_GREEN   PD10  LED
 * LED_RED     PD15
 * LED_BLUE    PD8
 * BUTTON      PC13  BUTTON/BOOT
 *
 * -           PC9   BAT_L_ENABLE
 * -           PD14  VBAT_L_ENABLE
 * -           PA1   VBAT_L_SENSE
 * -           PD0   BAT_S_ENABLE
 * -           PH1   VBAT_S_ENABLE
 * -           PC4   VBAT_S_SENSE
 * -           PD1   SENS_ENABLE
 *
 * INT_SCL     PB8
 * INT_SDA     PA10  
 *
 * EXTI 0      PC0     A0
 * EXTI 1      PC1     A1
 * EXTI 2      PC2     A2
 * EXTI 3      PC3     A3
 * EXTI 4      PA4     A4
 * EXTI 5      PA5     A5
 * EXTI 6      PA6     A6
 * EXTI 7      PA7     A7
 * EXTI 8      PA8     A8
 * EXTI 9      PA9     A9
 * EXTI 10     PC10    LSM6DSM_INT1
 * EXTI 11     PC11    LSM6DSM_INT1
 * EXTI 12     PB12    BMA400_INT2
 * EXTI 13     PB13    EXT_SCL
 * EXTI 14     PB14    EXT_SDA
 * EXTI 15     PA15    USB_VBUS
 *
 * TAMP 1      PC13    BUTTON/BOOT 
 * TAMP 2      PA0     BMA400_INT1
 * TAMP 3      PC12    -
 */

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[PINS_COUNT] =
{
    // 0..3 - Digital pins (D0, D1, D2, D3)
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA0),  STM32WB_GPIO_PIN_PA0,            (PIN_ATTR_TAMP | PIN_ATTR_WKUP1), PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB12), STM32WB_GPIO_PIN_PB12,           (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC10), STM32WB_GPIO_PIN_PC10,           (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC11), STM32WB_GPIO_PIN_PC11,           (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 4..13 - Analog pins (A0-A9)
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC0),  STM32WB_GPIO_PIN_PC0,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_1    },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC1),  STM32WB_GPIO_PIN_PC1,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_2    },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC2),  STM32WB_GPIO_PIN_PC2,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_3    },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC3),  STM32WB_GPIO_PIN_PC3,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_4    },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA4),  STM32WB_GPIO_PIN_PA4,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_9    },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA5),  STM32WB_GPIO_PIN_PA5,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_TIM2,  PWM_CHANNEL_1,    ADC_CHANNEL_10   },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA6),  STM32WB_GPIO_PIN_PA6,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_11   },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA7),  STM32WB_GPIO_PIN_PA7,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_12   },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA8),  STM32WB_GPIO_PIN_PA8,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_TIM1,  PWM_CHANNEL_1,    ADC_CHANNEL_15   },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA9),  STM32WB_GPIO_PIN_PA9,            (PIN_ATTR_EXTI),                  PWM_INSTANCE_TIM1,  PWM_CHANNEL_2,    ADC_CHANNEL_16   },

    // 14..15 I2C3 pins (SCL, SDA)
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB13), STM32WB_GPIO_PIN_PB13,           (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB14), STM32WB_GPIO_PIN_PB14,           (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    
    // 16..19 - LED_GREEN/LED_RED/LED_BLUE/BUTTON
    { GPIOD, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PD10), STM32WB_GPIO_PIN_PD10,           0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOD, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PD15), STM32WB_GPIO_PIN_PD15,           0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOD, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PD8),  STM32WB_GPIO_PIN_PD8,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC13), STM32WB_GPIO_PIN_PC13,           (PIN_ATTR_TAMP | PIN_ATTR_WKUP2), PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_1    },

    // 20..26 - BAT_L_ENABLE / VBAT_L_ENABLE / VBAT_L_SENSE / BAT_S_ENABLE / VBAT_S_ENABLE / VBAT_S_SENSE / I2C_ENABLE
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC9),  STM32WB_GPIO_PIN_PC9,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOD, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PD14), STM32WB_GPIO_PIN_PD14,           0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA1),  STM32WB_GPIO_PIN_PA1,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_6    },
    { GPIOD, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PD0),  STM32WB_GPIO_PIN_PD0,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOH, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PH1),  STM32WB_GPIO_PIN_PH1,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PC4),  STM32WB_GPIO_PIN_PC4,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_13   },
    { GPIOD, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PD1),  STM32WB_GPIO_PIN_PD1,            0,                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 27..28 I2C1 pins (SCL, SDA)
    { GPIOB, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PB8),  STM32WB_GPIO_PIN_PB8,           (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, STM32WB_GPIO_PIN_MASK(STM32WB_GPIO_PIN_PA10), STM32WB_GPIO_PIN_PA10,          (PIN_ATTR_EXTI),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
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
        STM32WB_GPIO_PIN_PC0_LPUART1_RX,
        STM32WB_GPIO_PIN_PC1_LPUART1_TX,
        STM32WB_GPIO_PIN_NONE,
        STM32WB_GPIO_PIN_NONE,
        STM32WB_GPIO_PIN_NONE,
    },
};

extern const stm32wb_spi_params_t g_SPIParams = {
    STM32WB_SPI_INSTANCE_SPI1,
    STM32WB_SPI_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA2_CH3_INDEX | STM32WB_DMA_CHANNEL_SELECT_SPI1_RX),
    (STM32WB_DMA_CHANNEL_DMA2_CH4_INDEX | STM32WB_DMA_CHANNEL_SELECT_SPI1_TX),
    {
        STM32WB_GPIO_PIN_PA7_SPI1_MOSI,
        STM32WB_GPIO_PIN_PA6_SPI1_MISO,
        STM32WB_GPIO_PIN_PA5_SPI1_SCK,
    },
};

extern const stm32wb_i2c_params_t g_WireParams = {
    STM32WB_I2C_INSTANCE_I2C3,
    STM32WB_I2C_IRQ_PRIORITY,
    (STM32WB_DMA_CHANNEL_DMA1_CH7_INDEX | STM32WB_DMA_CHANNEL_SELECT_I2C3_RX),
    STM32WB_DMA_CHANNEL_NONE,
    {
        STM32WB_GPIO_PIN_PB13_I2C3_SCL,
        STM32WB_GPIO_PIN_PB14_I2C3_SDA,
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

extern const stm32wb_system_info_t stm32wb_system_info =
{
    .version           	= STM32WB_SYSTEM_VERSION,
    .options            = (STM32WB_SYSTEM_OPTION_DFU_USB | STM32WB_SYSTEM_OPTION_SFLASH_BOOST | STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_10uH | STM32WB_SYSTEM_OPTION_SMPS_CURRENT_220mA | STM32WB_SYSTEM_OPTION_LSE_MODE_2),
    .hseclk             = 32000000,
    .lseclk             = 32774,
    .pins               = {
        .status         = STM32WB_GPIO_PIN_PC9,
        .boost          = STM32WB_GPIO_PIN_PC8,
        .dfu            = STM32WB_GPIO_PIN_PH3,
        .usb_vbus       = STM32WB_GPIO_PIN_PA15,
        .uart_rx        = STM32WB_GPIO_PIN_NONE,
        .uart_tx        = STM32WB_GPIO_PIN_NONE,
        .sflash_cs      = STM32WB_GPIO_PIN_PD3_QUADSPI_NCS,
        .sflash_clk     = STM32WB_GPIO_PIN_PA3_QUADSPI_CLK,
        .sflash_mosi    = STM32WB_GPIO_PIN_PD4_QUADSPI_IO0,
        .sflash_miso    = STM32WB_GPIO_PIN_PD5_QUADSPI_IO1,
        .sflash_wp      = STM32WB_GPIO_PIN_PD6_QUADSPI_IO2,
        .sflash_hold    = STM32WB_GPIO_PIN_PD7_QUADSPI_IO3,
        .sflash_enable  = STM32WB_GPIO_PIN_PB2
    }
};

void initVariant()
{
    stm32wb_sfsqi_initialize();
}


