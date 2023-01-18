/*
 * Copyright (c) 2014-2020 Thomas Roell.  All rights reserved.
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

#include "stm32wb_gpio.h"

typedef struct _stm32wb_gpio_device_t {
    volatile uint32_t       enables[STM32WB_GPIO_PORT_COUNT];
    volatile uint32_t       mask[STM32WB_GPIO_PORT_COUNT];
    volatile uint32_t       mode[2][STM32WB_GPIO_PORT_COUNT];
    volatile uint32_t       pupd[2][STM32WB_GPIO_PORT_COUNT];
} stm32wb_gpio_device_t;

static stm32wb_gpio_device_t stm32wb_gpio_device;

void __stm32wb_gpio_initialize(void)
{
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA15, STM32WB_GPIO_MODE_ANALOG);
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PB3,  STM32WB_GPIO_MODE_ANALOG);
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PB4,  STM32WB_GPIO_MODE_ANALOG);
}

void __stm32wb_gpio_swd_enable(void)
{
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA13_SWDIO, (STM32WB_GPIO_PUPD_PULLUP   | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA14_SWCLK, (STM32WB_GPIO_PUPD_PULLDOWN | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
}

void __stm32wb_gpio_swd_disable(void)
{
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA13, STM32WB_GPIO_MODE_ANALOG);
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA14, STM32WB_GPIO_MODE_ANALOG);
}

__attribute__((optimize("O3"))) void __stm32wb_gpio_stop_enter(void)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, port, mask, mode, pupd;

    for (port = 0; port < STM32WB_GPIO_PORT_COUNT; port++)
    {
        mask = stm32wb_gpio_device.mask[port];
	
	if (mask)
	{
	    group = ((port == (STM32WB_GPIO_PORT_COUNT -1)) ? (STM32WB_GPIO_GROUP_COUNT -1) : port);
	    
	    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);
	    
	    mode = GPIO->MODER;
	    pupd = GPIO->PUPDR;
	    
	    GPIO->MODER = (mode & ~mask) | (stm32wb_gpio_device.mode[0][port] & mask);
	    GPIO->PUPDR = (pupd & ~mask) | (stm32wb_gpio_device.pupd[0][port] & mask);
	    
	    stm32wb_gpio_device.mode[1][port] = mode;
	    stm32wb_gpio_device.pupd[1][port] = pupd;
	}
    }
}

__attribute__((optimize("O3"))) void __stm32wb_gpio_stop_leave(void)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, port, mask;

    for (port = 0; port < STM32WB_GPIO_PORT_COUNT; port++)
    {
        mask = stm32wb_gpio_device.mask[port];
	
	if (mask)
	{
	    group = ((port == (STM32WB_GPIO_PORT_COUNT -1)) ? (STM32WB_GPIO_GROUP_COUNT -1) : port);

	    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);
	    
	    GPIO->MODER = stm32wb_gpio_device.mode[1][port];
	    GPIO->PUPDR = stm32wb_gpio_device.pupd[1][port];
	}
    }
}

__attribute__((optimize("O3"))) void __stm32wb_gpio_pin_write(uint32_t pin, uint32_t data)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index;

    group = (pin & STM32WB_GPIO_PIN_GROUP_MASK) >> STM32WB_GPIO_PIN_GROUP_SHIFT;
    index = (pin & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT;

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (data)
    {
        GPIO->BSRR = (1 << index);
    }
    else
    {
        GPIO->BRR = (1 << index);
    }
}

__attribute__((optimize("O3"))) uint32_t __stm32wb_gpio_pin_read(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index;

    group = (pin & STM32WB_GPIO_PIN_GROUP_MASK) >> STM32WB_GPIO_PIN_GROUP_SHIFT;
    index = (pin & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT;

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    return ((GPIO->IDR >> index) & 1);
}

__attribute__((optimize("O3"))) void stm32wb_gpio_pin_configure(uint32_t pin, uint32_t mode)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, port, afsel, mask, index_2, mask_2;

    afsel = (pin >> 8) & 15;
    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port = ((group >= STM32WB_GPIO_PORT_COUNT) ? (STM32WB_GPIO_PORT_COUNT -1) : group);

    mask = (1 << index);
    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (!(stm32wb_gpio_device.enables[port] & mask))
    {
        armv7m_atomic_or(&stm32wb_gpio_device.enables[port], mask);
        armv7m_atomic_or(&RCC->AHB2ENR, (RCC_AHB2ENR_GPIOAEN << group));
	RCC->AHB2ENR;
    }

    /* If the mode is ANALOG, set MODER first */
    if (((mode & STM32WB_GPIO_MODE_MASK) >> STM32WB_GPIO_MODE_SHIFT) == STM32WB_GPIO_MODE_ANALOG)
    {
	armv7m_atomic_modify(&GPIO->MODER, mask_2, ((STM32WB_GPIO_MODE_ANALOG >> STM32WB_GPIO_MODE_SHIFT) << index_2));
	
        armv7m_atomic_and(&stm32wb_gpio_device.enables[port], ~mask);
        armv7m_atomic_andz(&RCC->AHB2ENR, ~(RCC_AHB2ENR_GPIOAEN << group), &stm32wb_gpio_device.enables[port]);
    }
    else
    {
        /* Set AFRL/AFRH */
        armv7m_atomic_modify(&GPIO->AFR[index >> 3], (0x0000000f << ((index & 7) << 2)), (afsel << ((index & 7) << 2)));
        
        /* Set OPSPEEDR */
        armv7m_atomic_modify(&GPIO->OSPEEDR, mask_2, (((mode & STM32WB_GPIO_OSPEED_MASK) >> STM32WB_GPIO_OSPEED_SHIFT) << index_2));
        
        /* Set OPTYPER */
        armv7m_atomic_modify(&GPIO->OTYPER, mask, (((mode & STM32WB_GPIO_OTYPE_MASK) >> STM32WB_GPIO_OTYPE_SHIFT) << index));
        
        /* If the mode is OUTPUT, or OUTPUT OPENDRAIN with a ODR of 0. then first switch MODER and then PUPDR
         * to avoid spurious edges. N.b. ALTERNATE is assumed to be INPUT before the peripheral drives it.
         */ 
        if (((mode & STM32WB_GPIO_MODE_MASK) == STM32WB_GPIO_MODE_OUTPUT) &&
            (((mode & STM32WB_GPIO_OTYPE_MASK) != STM32WB_GPIO_OTYPE_OPENDRAIN) || !(GPIO->ODR & mask)))
        {
            /* Set MODE */
            armv7m_atomic_modify(&GPIO->MODER, mask_2, (((mode & STM32WB_GPIO_MODE_MASK) >> STM32WB_GPIO_MODE_SHIFT) << index_2));
            
            /* Set PUPD */
            armv7m_atomic_modify(&GPIO->PUPDR, mask_2, (((mode & STM32WB_GPIO_PUPD_MASK) >> STM32WB_GPIO_PUPD_SHIFT) << index_2));
        }
        else
        {
            /* Set PUPD */
            armv7m_atomic_modify(&GPIO->PUPDR, mask_2, (((mode & STM32WB_GPIO_PUPD_MASK) >> STM32WB_GPIO_PUPD_SHIFT) << index_2));
            
            /* Set MODE */
            armv7m_atomic_modify(&GPIO->MODER, mask_2, (((mode & STM32WB_GPIO_MODE_MASK) >> STM32WB_GPIO_MODE_SHIFT) << index_2));
        }
    }

    if ((mode & STM32WB_GPIO_PARK_MASK) == STM32WB_GPIO_PARK_NONE)
    {
        armv7m_atomic_and(&stm32wb_gpio_device.mask[port], ~mask_2);
    }
    else
    {
        armv7m_atomic_or(&stm32wb_gpio_device.mask[port], mask_2);

        if ((mode & STM32WB_GPIO_PARK_MASK) == STM32WB_GPIO_PARK_HIZ)
        {
            armv7m_atomic_modify(&stm32wb_gpio_device.mode[0][port], mask_2, ((STM32WB_GPIO_MODE_ANALOG >> STM32WB_GPIO_MODE_SHIFT) << index_2));
            armv7m_atomic_and(&stm32wb_gpio_device.pupd[0][port], ~mask_2);
        }
        else
        {
            armv7m_atomic_modify(&stm32wb_gpio_device.mode[0][port], mask_2, ((STM32WB_GPIO_MODE_INPUT >> STM32WB_GPIO_MODE_SHIFT) << index_2));
            armv7m_atomic_modify(&stm32wb_gpio_device.pupd[0][port], mask_2, (((mode & STM32WB_GPIO_PARK_MASK) >> STM32WB_GPIO_PARK_SHIFT) << index_2));
        }
    }
}

__attribute__((optimize("O3"))) void stm32wb_gpio_pin_input(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, port, mask, index_2, mask_2;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port = ((group >= STM32WB_GPIO_PORT_COUNT) ? (STM32WB_GPIO_PORT_COUNT -1) : group);

    mask = (1 << index);
    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (!(stm32wb_gpio_device.enables[port] & mask))
    {
        armv7m_atomic_or(&stm32wb_gpio_device.enables[port], mask);
        armv7m_atomic_or(&RCC->AHB2ENR, (RCC_AHB2ENR_GPIOAEN << group));
	RCC->AHB2ENR;
    }
    
    armv7m_atomic_modify(&GPIO->MODER, mask_2, ((STM32WB_GPIO_MODE_INPUT >> STM32WB_GPIO_MODE_SHIFT) << index_2));
}

__attribute__((optimize("O3"))) void stm32wb_gpio_pin_output(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, port, mask, index_2, mask_2;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port = ((group >= STM32WB_GPIO_PORT_COUNT) ? (STM32WB_GPIO_PORT_COUNT -1) : group);

    mask = (1 << index);
    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (!(stm32wb_gpio_device.enables[port] & mask))
    {
        armv7m_atomic_or(&stm32wb_gpio_device.enables[port], mask);
        armv7m_atomic_or(&RCC->AHB2ENR, (RCC_AHB2ENR_GPIOAEN << group));
	RCC->AHB2ENR;
    }
    
    armv7m_atomic_modify(&GPIO->MODER, mask_2, ((STM32WB_GPIO_MODE_OUTPUT >> STM32WB_GPIO_MODE_SHIFT) << index_2));
}

__attribute__((optimize("O3"))) void stm32wb_gpio_pin_alternate(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, port, mask, index_2, mask_2;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port = ((group >= STM32WB_GPIO_PORT_COUNT) ? (STM32WB_GPIO_PORT_COUNT -1) : group);

    mask = (1 << index);
    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (!(stm32wb_gpio_device.enables[port] & mask))
    {
        armv7m_atomic_or(&stm32wb_gpio_device.enables[port], mask);
        armv7m_atomic_or(&RCC->AHB2ENR, (RCC_AHB2ENR_GPIOAEN << group));
	RCC->AHB2ENR;
    }
    
    armv7m_atomic_modify(&GPIO->MODER, mask_2, ((STM32WB_GPIO_MODE_ALTERNATE >> STM32WB_GPIO_MODE_SHIFT) << index_2));
}

__attribute__((optimize("O3"))) void stm32wb_gpio_pin_analog(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, port, mask, index_2, mask_2;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port = ((group >= STM32WB_GPIO_PORT_COUNT) ? (STM32WB_GPIO_PORT_COUNT -1) : group);

    mask = (1 << index);
    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (!(stm32wb_gpio_device.enables[port] & mask))
    {
        armv7m_atomic_or(&stm32wb_gpio_device.enables[port], mask);
        armv7m_atomic_or(&RCC->AHB2ENR, (RCC_AHB2ENR_GPIOAEN << group));
	RCC->AHB2ENR;
    }

    armv7m_atomic_modify(&GPIO->MODER, mask_2, ((STM32WB_GPIO_MODE_ANALOG >> STM32WB_GPIO_MODE_SHIFT) << index_2));

    armv7m_atomic_and(&stm32wb_gpio_device.enables[port], ~mask);
    armv7m_atomic_andz(&RCC->AHB2ENR, ~(RCC_AHB2ENR_GPIOAEN << group), &stm32wb_gpio_device.enables[port]);
}
