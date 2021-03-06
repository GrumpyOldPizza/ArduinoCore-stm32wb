/*
 * Copyright (c) 2016-2020 Thomas Roell.  All rights reserved.
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

#include <stdio.h>

#include "stm32wbxx.h"

#include "armv7m.h"

#include "stm32wb_gpio.h"
#include "stm32wb_tim.h"
#include "stm32wb_system.h"

typedef struct _stm32wb_tim_device_t {
    stm32wb_tim_t * volatile instances[STM32WB_TIM_INSTANCE_COUNT];
} stm32wb_tim_device_t;

static stm32wb_tim_device_t stm32wb_tim_device;

static TIM_TypeDef * const stm32wb_tim_xlate_TIM[STM32WB_TIM_INSTANCE_COUNT] = {
    TIM1,
    TIM2,
    TIM16,
    TIM17,
};

static void stm32wb_tim_interrupt(stm32wb_tim_t *tim)
{
    TIM_TypeDef *TIM = tim->TIM;
    uint32_t tim_sr, events;

    tim_sr = TIM->SR;

    TIM->SR = 0;

    events = 0;

    if (tim->callback)
    {
	if (tim_sr & TIM_SR_UIF)
	{
	    events |= STM32WB_TIM_EVENT_PERIOD;
	}

	if (tim_sr & (TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF))
	{
	    if (tim_sr & TIM_SR_CC1IF)
	    {
		events |= STM32WB_TIM_EVENT_CHANNEL_1;
	    }

	    if (tim_sr & TIM_SR_CC2IF)
	    {
		events |= STM32WB_TIM_EVENT_CHANNEL_2;
	    }

	    if (tim_sr & TIM_SR_CC3IF)
	    {
		events |= STM32WB_TIM_EVENT_CHANNEL_3;
	    }

	    if (tim_sr & TIM_SR_CC4IF)
	    {
		events |= STM32WB_TIM_EVENT_CHANNEL_4;
	    }
	}
    
	(*tim->callback)(tim->context, events);
    }
}

bool stm32wb_tim_create(stm32wb_tim_t *tim, unsigned int instance, unsigned int priority)
{
    if (instance >= STM32WB_TIM_INSTANCE_COUNT)
    {
	return false;
    }

    tim->TIM = stm32wb_tim_xlate_TIM[instance];
    tim->instance = instance;
    tim->priority = priority;
    tim->callback = NULL;
    tim->context = NULL;

    tim->state = STM32WB_TIM_STATE_INIT;

    return true;
}

bool stm32wb_tim_destroy(stm32wb_tim_t *tim)
{
    if (tim->state != STM32WB_TIM_STATE_INIT)
    {
	return false;
    }

    tim->state = STM32WB_TIM_STATE_NONE;

    return true;
}

uint32_t stm32wb_tim_clock(stm32wb_tim_t *tim)
{
    uint32_t hclk, pclk;

    hclk = stm32wb_system_hclk();

    if (tim->instance == STM32WB_TIM_INSTANCE_TIM2)
    {
	pclk = stm32wb_system_pclk1();
    }
    else
    {
	pclk = stm32wb_system_pclk2();
    }

    return ((hclk == pclk) ? hclk : (2 * pclk));
}

bool stm32wb_tim_enable(stm32wb_tim_t *tim, uint32_t option, stm32wb_tim_callback_t callback, void *context)
{
    if (tim->state != STM32WB_TIM_STATE_INIT)
    {
	return false;
    }

    switch (tim->instance) {
    case STM32WB_TIM_INSTANCE_TIM1:
	if (stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM16] &&
	    (stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM16]->priority != tim->priority))
	{
	    return false;
	}

	if (stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM17] &&
	    (stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM17]->priority != tim->priority))
	{
	    return false;
	}
	break;

    case STM32WB_TIM_INSTANCE_TIM2:
	break;

    case STM32WB_TIM_INSTANCE_TIM16:
    case STM32WB_TIM_INSTANCE_TIM17:
	if (stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM1] &&
	    (stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM1]->priority != tim->priority))
	{
	    return false;
	}
	break;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&stm32wb_tim_device.instances[tim->instance], (uint32_t)NULL, (uint32_t)tim) != (uint32_t)NULL)
    {
	return false;
    }

    tim->state = STM32WB_TIM_STATE_NOT_READY;

    tim->callback = callback;
    tim->context = context;

    if (!stm32wb_tim_configure(tim, option))
    {
	armv7m_atomic_store((volatile uint32_t*)&stm32wb_tim_device.instances[tim->instance], (uint32_t)NULL);

	tim->state = STM32WB_TIM_STATE_INIT;

	return false;
    }

    stm32wb_system_reference(STM32WB_SYSTEM_REFERENCE_TIM1 << tim->instance);
    
    tim->state = STM32WB_TIM_STATE_READY;

    switch (tim->instance) {
    case STM32WB_TIM_INSTANCE_TIM1:
	NVIC_SetPriority(TIM1_BRK_IRQn, tim->priority);
	NVIC_SetPriority(TIM1_CC_IRQn, tim->priority);
	NVIC_SetPriority(TIM1_UP_TIM16_IRQn, tim->priority);
	NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, tim->priority);
	NVIC_EnableIRQ(TIM1_BRK_IRQn);
	NVIC_EnableIRQ(TIM1_CC_IRQn);
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
	break;

    case STM32WB_TIM_INSTANCE_TIM2:
	NVIC_SetPriority(TIM2_IRQn, tim->priority);
	NVIC_EnableIRQ(TIM2_IRQn);
	break;

    case STM32WB_TIM_INSTANCE_TIM16:
	NVIC_SetPriority(TIM1_UP_TIM16_IRQn, tim->priority);
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	break;

    case STM32WB_TIM_INSTANCE_TIM17:
	NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, tim->priority);
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
	break;
    }

    return true;
}

bool stm32wb_tim_disable(stm32wb_tim_t *tim)
{
    if (tim->state != STM32WB_TIM_STATE_READY)
    {
	return false;
    }

    stm32wb_system_unreference(STM32WB_SYSTEM_REFERENCE_TIM1 << tim->instance);
    
    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_TIM1 + tim->instance);

    switch (tim->instance) {
    case STM32WB_TIM_INSTANCE_TIM1:
	NVIC_DisableIRQ(TIM1_BRK_IRQn);
	NVIC_DisableIRQ(TIM1_CC_IRQn);

	if (!stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM16])
	{
	    NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
	}

	if (!stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM17])
	{
	    NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn);
	}
	break;

    case STM32WB_TIM_INSTANCE_TIM2:
	NVIC_DisableIRQ(TIM2_IRQn);
	break;

    case STM32WB_TIM_INSTANCE_TIM16:
	if (!stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM1])
	{
	    NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
	}
	break;

    case STM32WB_TIM_INSTANCE_TIM17:
	if (!stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM1])
	{
	    NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn);
	}
	break;
    }

    armv7m_atomic_store((volatile uint32_t*)&stm32wb_tim_device.instances[tim->instance], (uint32_t)NULL);

    tim->state = STM32WB_TIM_STATE_INIT;

    return true;
}

bool stm32wb_tim_configure(stm32wb_tim_t *tim, uint32_t option)
{
    TIM_TypeDef *TIM = tim->TIM;
    uint32_t tim_cr1;

    if ((tim->state != STM32WB_TIM_STATE_NOT_READY) && (tim->state != STM32WB_TIM_STATE_READY))
    {
	return false;
    }
    
    if (tim->state == STM32WB_TIM_STATE_NOT_READY)
    {
        stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_TIM1 + tim->instance);
    }

    tim_cr1 = 0;

    if (option & STM32WB_TIM_OPTION_CONTINOUS)
    {
	tim_cr1 |= TIM_CR1_ARPE;
    }
    
    TIM->CR1  = tim_cr1;

    TIM->BDTR = TIM_BDTR_MOE;
#if 0
    if (tim->instance != STM32WB_TIM_INSTANCE_TIM2)
    {
	TIM->RCR = 0;
    }
#endif
    
    if (tim->instance == STM32WB_TIM_INSTANCE_TIM16)
    {
	TIM->OR = (option & STM32WB_TIM_OPTION_TIM16_CHANNEL_1_INPUT_MASK) >> STM32WB_TIM_OPTION_TIM16_CHANNEL_1_INPUT_SHIFT;
    }

    if (tim->instance == STM32WB_TIM_INSTANCE_TIM17)
    {
	TIM->OR = (option & STM32WB_TIM_OPTION_TIM17_CHANNEL_1_INPUT_MASK) >> STM32WB_TIM_OPTION_TIM17_CHANNEL_1_INPUT_SHIFT;
    }
    
    return true;
}

bool stm32wb_tim_start(stm32wb_tim_t *tim, uint32_t prescale, uint32_t reload)
{
    TIM_TypeDef *TIM = tim->TIM;

    if (tim->state < STM32WB_TIM_STATE_READY)
    {
	return false;
    }

    if (tim->state == STM32WB_TIM_STATE_READY)
    {
        stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP_0);

	if (tim->callback)
	{
	    armv7m_atomic_or(&TIM->DIER, TIM_DIER_UIE);
	}
    
	tim->state = STM32WB_TIM_STATE_ACTIVE;
    }
    else
    {
	armv7m_atomic_and(&TIM->CR1, ~TIM_CR1_CEN);
    }
    
    TIM->SR = 0;
    TIM->PSC = prescale -1;
    TIM->ARR = reload;

    armv7m_atomic_or(&TIM->CR1, TIM_CR1_CEN);

    return true;
}

bool stm32wb_tim_restart(stm32wb_tim_t *tim, uint32_t reload)
{
    TIM_TypeDef *TIM = tim->TIM;

    if (tim->state != STM32WB_TIM_STATE_ACTIVE)
    {
	return false;
    }

    TIM->ARR = reload;

    return true;
}

bool stm32wb_tim_stop(stm32wb_tim_t *tim)
{
    TIM_TypeDef *TIM = tim->TIM;

    if (tim->state != STM32WB_TIM_STATE_ACTIVE)
    {
	return false;
    }

    armv7m_atomic_and(&TIM->CR1, ~TIM_CR1_CEN);
    armv7m_atomic_and(&TIM->DIER, ~TIM_DIER_UIE);

    stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP_0);

    tim->state = STM32WB_TIM_STATE_READY;

    return true;
}

uint32_t stm32wb_tim_count(stm32wb_tim_t *tim)
{
    TIM_TypeDef *TIM = tim->TIM;

    if (tim->state == STM32WB_TIM_STATE_ACTIVE)
    {
	return 0;
    }

    return TIM->CNT;
}

bool stm32wb_tim_channel(stm32wb_tim_t *tim, unsigned int channel, uint32_t compare, uint32_t control)
{
    TIM_TypeDef *TIM = tim->TIM;
    uint32_t tim_ccmr, tim_ccer;

    if (tim->state < STM32WB_TIM_STATE_READY)
    {
	return false;
    }
    
    armv7m_atomic_and(&TIM->CCER, ~(TIM_CCER_CC1E << (channel * 4)));
    armv7m_atomic_and(&TIM->DIER, ~(TIM_DIER_CC1IE << channel));
    
    tim_ccmr = 0;
    tim_ccer = 0;

    if (control & (STM32WB_TIM_CONTROL_CAPTURE_RISING_EDGE | STM32WB_TIM_CONTROL_CAPTURE_FALLING_EDGE | STM32WB_TIM_CONTROL_COMPARE | STM32WB_TIM_CONTROL_PWM | STM32WB_TIM_CONTROL_PWM_INVERTED))
    {
	if (control & (STM32WB_TIM_CONTROL_CAPTURE_RISING_EDGE | STM32WB_TIM_CONTROL_CAPTURE_FALLING_EDGE))
	{
	    tim_ccmr |= ((control & STM32WB_TIM_CONTROL_CAPTURE_ALTERNATE ? TIM_CCMR1_CC1S_1 : TIM_CCMR1_CC1S_0) |
			 (((control & STM32WB_TIM_CONTROL_CAPTURE_PRESCALE_MASK) >> STM32WB_TIM_CONTROL_CAPTURE_PRESCALE_SHIFT) << 2) |
			 (((control & STM32WB_TIM_CONTROL_CAPTURE_FILTER_MASK) >> STM32WB_TIM_CONTROL_CAPTURE_FILTER_SHIFT) << 4));

	    switch (control & (STM32WB_TIM_CONTROL_CAPTURE_RISING_EDGE | STM32WB_TIM_CONTROL_CAPTURE_FALLING_EDGE)) {
	    case STM32WB_TIM_CONTROL_CAPTURE_RISING_EDGE:
		tim_ccer |= (TIM_CCER_CC1E);
		break;
	    case STM32WB_TIM_CONTROL_CAPTURE_FALLING_EDGE:
		tim_ccer |= (TIM_CCER_CC1E | TIM_CCER_CC1P);
		break;
	    case STM32WB_TIM_CONTROL_CAPTURE_FALLING_EDGE | STM32WB_TIM_CONTROL_CAPTURE_RISING_EDGE:
		tim_ccer |= (TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP);
		break;
	    }
 	}
	else
	{
	    if (control & STM32WB_TIM_CONTROL_COMPARE)
	    {
		tim_ccer |= TIM_CCER_CC1E;
	    }
	    else
	    {
		if (control & STM32WB_TIM_CONTROL_PWM_INVERTED)
		{
		    tim_ccmr |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0);
		}
		else
		{
		    tim_ccmr |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
		}
		
		tim_ccer |= (TIM_CCER_CC1E |
			     TIM_CCER_CC1NE |
			     ((control & STM32WB_TIM_CONTROL_PWM_COMPLEMENTARY) ? 0 : TIM_CCER_CC1NP));
	    }

	    switch (channel) {
	    case STM32WB_TIM_CHANNEL_1: TIM->CCR1 = compare; break;
	    case STM32WB_TIM_CHANNEL_2: TIM->CCR2 = compare; break;
	    case STM32WB_TIM_CHANNEL_3: TIM->CCR3 = compare; break;
	    case STM32WB_TIM_CHANNEL_4: TIM->CCR4 = compare; break;
	    default:
		break;
	    }
	}

	switch (channel) {
	case STM32WB_TIM_CHANNEL_1: armv7m_atomic_modify(&TIM->CCMR1, 0x000100ff, (tim_ccmr << 0)); break;
	case STM32WB_TIM_CHANNEL_2: armv7m_atomic_modify(&TIM->CCMR1, 0x0100ff00, (tim_ccmr << 8)); break;
	case STM32WB_TIM_CHANNEL_3: armv7m_atomic_modify(&TIM->CCMR2, 0x000100ff, (tim_ccmr << 0)); break;
	case STM32WB_TIM_CHANNEL_4: armv7m_atomic_modify(&TIM->CCMR2, 0x0100ff00, (tim_ccmr << 8)); break;
	}

	if (control & (STM32WB_TIM_CONTROL_CAPTURE_RISING_EDGE | STM32WB_TIM_CONTROL_CAPTURE_FALLING_EDGE | STM32WB_TIM_CONTROL_COMPARE))
	{
	    if (tim->callback)
	    {
		armv7m_atomic_or(&TIM->DIER, (TIM_DIER_CC1IE << channel));
	    }
	}
	
	armv7m_atomic_modify(&TIM->CCER, (0x0000000f << (channel * 4)), (tim_ccer << (channel * 4)));
    }
	
    return true;
}

bool stm32wb_tim_compare(stm32wb_tim_t *tim, unsigned int channel, uint32_t compare)
{
    TIM_TypeDef *TIM = tim->TIM;

    if (tim->state < STM32WB_TIM_STATE_READY)
    {
	return false;
    }
    
    switch (channel) {
    case STM32WB_TIM_CHANNEL_1: TIM->CCR1 = compare; break;
    case STM32WB_TIM_CHANNEL_2: TIM->CCR2 = compare; break;
    case STM32WB_TIM_CHANNEL_3: TIM->CCR3 = compare; break;
    case STM32WB_TIM_CHANNEL_4: TIM->CCR4 = compare; break;
    default:
	break;
    }
    
    return true;
}

uint32_t stm32wb_tim_capture(stm32wb_tim_t *tim, unsigned int channel)
{
    TIM_TypeDef *TIM = tim->TIM;

    if (tim->state < STM32WB_TIM_STATE_READY)
    {
	return 0;
    }

    switch (channel) {
    case STM32WB_TIM_CHANNEL_1: return TIM->CCR1;
    case STM32WB_TIM_CHANNEL_2: return TIM->CCR2;
    case STM32WB_TIM_CHANNEL_3: return TIM->CCR3;
    case STM32WB_TIM_CHANNEL_4: return TIM->CCR4;
    default:
	return 0;
    }
}

void TIM1_BRK_IRQHandler(void)
{
    stm32wb_tim_interrupt(stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM1]);

    __DSB();
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    if (stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM1])
    {
	stm32wb_tim_interrupt(stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM1]);
    }

    if (stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM16])
    {
	stm32wb_tim_interrupt(stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM16]);
    }

    __DSB();
}

void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
    if (stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM1])
    {
	stm32wb_tim_interrupt(stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM1]);
    }

    if (stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM17])
    {
	stm32wb_tim_interrupt(stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM17]);
    }

    __DSB();
}

void TIM1_CC_IRQHandler(void)
{
    stm32wb_tim_interrupt(stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM1]);

    __DSB();
}

void TIM2_IRQHandler(void)
{
    stm32wb_tim_interrupt(stm32wb_tim_device.instances[STM32WB_TIM_INSTANCE_TIM2]);

    __DSB();
}
