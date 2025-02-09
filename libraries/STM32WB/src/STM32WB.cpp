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

#include "Arduino.h"
#include "STM32WB.h"
#include "wiring_private.h"

typedef struct _stm32wb_callbacks_t {
    stm32wb_system_notify_t notify;
    Callback                standby_callback;  
    Callback                shutdown_callback;  
    Callback                reset_callback;  
} stm32wb_callbacks_t;

static bool __stm32wb_boost_enable = false;

static stm32wb_callbacks_t stm32wb_callbacks;

static void stm32wb_notify_callback(void *context, uint32_t event)
{
    stm32wb_callbacks_t *callbacks = (stm32wb_callbacks_t*)context;

    if (event & STM32WB_SYSTEM_EVENT_STANDBY) {
        callbacks->standby_callback();
    }

    if (event & STM32WB_SYSTEM_EVENT_SHUTDOWN) {
        callbacks->shutdown_callback();
    }

    if (event & STM32WB_SYSTEM_EVENT_RESET) {
        callbacks->reset_callback();
    }
}
  
void STM32WBClass::getUID(uint32_t uid[3])
{
    stm32wb_system_uid(uid);
}

float STM32WBClass::readBattery()
{
#if defined(STM32WB_CONFIG_VBAT_SENSE_CHANNEL)
    uint8_t vrefint_channel, vbat_channel;
    uint16_t vrefint_data, vbat_data;
    float vref, battery;

    if (armv7m_core_is_in_interrupt()) {
        return 0;
    }

    vrefint_channel = STM32WB_ADC_CHANNEL_VREFINT;

    stm32wb_adc_convert(&vrefint_channel, &vrefint_data, 1, STM32WB_ADC_VREFINT_PERIOD, (STM32WB_ADC_OPTION_RATIO_1 | STM32WB_ADC_OPTION_WIDTH_12));
    
    vref = (float)(STM32WB_ADC_VREFINT_VREF * STM32WB_ADC_VREFINT_CAL) / (float)vrefint_data;
    
#if defined(STM32WB_CONFIG_PIN_VBAT_SWITCH)
#if defined(STM32WB_CONFIG_PIN_BUTTON) && (STM32WB_CONFIG_PIN_BUTTON == STM32WB_CONFIG_PIN_VBAT_SWITCH)
    armv7m_atomic_modify(&g_pinButton, (PIN_BUTTON_DATA_MASK | PIN_BUTTON_READ_BATTERY), ((stm32wb_gpio_pin_read(STM32WB_CONFIG_PIN_BUTTON) << PIN_BUTTON_DATA_SHIFT) | PIN_BUTTON_READ_BATTERY));
    stm32wb_exti_block(1 << ((STM32WB_CONFIG_PIN_BUTTON & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT));
#endif /* defined(STM32WB_CONFIG_PIN_BUTTON) && (STM32WB_CONFIG_PIN_BUTTON == STM32WB_CONFIG_PIN_VBAT_SWITCH) */

    stm32wb_gpio_pin_configure(STM32WB_CONFIG_PIN_VBAT_SWITCH, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_MEDIUM | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));
    stm32wb_gpio_pin_write(STM32WB_CONFIG_PIN_VBAT_SWITCH, 1);
#endif /* defined(STM32WB_CONFIG_PIN_VBAT_SWITCH) */
        
#if defined(STM32WB_CONFIG_VBAT_SENSE_DELAY)
    armv7m_core_udelay(STM32WB_CONFIG_VBAT_SENSE_DELAY);
#endif /* defined(STM32WB_CONFIG_VBAT_SENSE_DELAY) */

    vbat_channel = STM32WB_CONFIG_VBAT_SENSE_CHANNEL;
    
    stm32wb_adc_convert(&vbat_channel, &vbat_data, 1, STM32WB_CONFIG_VBAT_SENSE_PERIOD, (STM32WB_ADC_OPTION_RATIO_1 | STM32WB_ADC_OPTION_WIDTH_12 | STM32WB_ADC_OPTION_STOP));

    battery = ((vref * STM32WB_CONFIG_VBAT_SENSE_SCALE) * (float)vbat_data) / 4095.0;

#if defined(STM32WB_CONFIG_PIN_VBAT_SWITCH)
#if defined(STM32WB_CONFIG_PIN_BUTTON) && (STM32WB_CONFIG_PIN_BUTTON == STM32WB_CONFIG_PIN_VBAT_SWITCH)
    {
        uint32_t pinButton;

        stm32wb_gpio_pin_write(STM32WB_CONFIG_PIN_BUTTON, ((g_pinButton & PIN_BUTTON_DATA_MASK) >> PIN_BUTTON_DATA_SHIFT));
        stm32wb_exti_unblock(1 << ((STM32WB_CONFIG_PIN_BUTTON & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT), true);
        
        do {
            pinButton = g_pinButton;
            
            stm32wb_gpio_pin_configure(STM32WB_CONFIG_PIN_BUTTON, g_pinModeConfiguration[(pinButton & PIN_BUTTON_MODE_MASK) >> PIN_BUTTON_MODE_SHIFT]);
        } while (armv7m_atomic_cas(&g_pinButton, pinButton, (pinButton & ~PIN_BUTTON_READ_BATTERY)) != pinButton);
    }
#else /* defined(STM32WB_CONFIG_PIN_BUTTON) && (STM32WB_CONFIG_PIN_BUTTON == STM32WB_CONFIG_PIN_VBAT_SWITCH) */
    stm32wb_gpio_pin_configure(STM32WB_CONFIG_PIN_VBAT_SWITCH, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_MODE_ANALOG));
#endif /* defined(STM32WB_CONFIG_PIN_BUTTON) && (STM32WB_CONFIG_PIN_BUTTON == STM32WB_CONFIG_PIN_VBAT_SWITCH) */
#endif /* defined(STM32WB_CONFIG_PIN_VBAT_SWITCH) */
        
    return battery;
#else /* defined(STM32WB_CONFIG_VBAT_SENSE_CHANNEL) */
    return 0;
#endif /* defined(STM32WB_CONFIG_VBAT_SENSE_CHANNEL) */
}

float STM32WBClass::readTemperature()
{
    uint8_t vrefint_channel, tsense_channel;
    uint16_t vrefint_data, tsense_data;
    float vref, tsense, temperature;

    if (armv7m_core_is_in_interrupt()) {
        return 0;
    }

    vrefint_channel = STM32WB_ADC_CHANNEL_VREFINT;
    
    stm32wb_adc_convert(&vrefint_channel, &vrefint_data, 1, STM32WB_ADC_VREFINT_PERIOD, (STM32WB_ADC_OPTION_RATIO_1 | STM32WB_ADC_OPTION_WIDTH_12));

    tsense_channel = STM32WB_ADC_CHANNEL_TSENSE;
    
    stm32wb_adc_convert(&tsense_channel, &tsense_data, 1, STM32WB_ADC_TSENSE_PERIOD, (STM32WB_ADC_OPTION_RATIO_1 | STM32WB_ADC_OPTION_WIDTH_12 | STM32WB_ADC_OPTION_STOP));

    vref = (float)(STM32WB_ADC_VREFINT_VREF * STM32WB_ADC_VREFINT_CAL) / (float)vrefint_data;

    tsense = (vref * (float)tsense_data) / (float)STM32WB_ADC_TSENSE_CAL_VREF;
  
    temperature = ((((STM32WB_ADC_TSENSE_CAL2_TEMP - STM32WB_ADC_TSENSE_CAL1_TEMP) /(float)(STM32WB_ADC_TSENSE_CAL2 - STM32WB_ADC_TSENSE_CAL1))
                    * (tsense - STM32WB_ADC_TSENSE_CAL1))
                   + STM32WB_ADC_TSENSE_CAL1_TEMP);

    return temperature;
}

float STM32WBClass::readVREF()
{
    uint8_t vrefint_channel;
    uint16_t vrefint_data;
    float vref;

    if (armv7m_core_is_in_interrupt()) {
        return 0;
    }

    vrefint_channel = STM32WB_ADC_CHANNEL_VREFINT;
    
    stm32wb_adc_convert(&vrefint_channel, &vrefint_data, 1, STM32WB_ADC_VREFINT_PERIOD, (STM32WB_ADC_OPTION_RATIO_1 | STM32WB_ADC_OPTION_WIDTH_12 | STM32WB_ADC_OPTION_STOP));

    vref = (float)(STM32WB_ADC_VREFINT_VREF * STM32WB_ADC_VREFINT_CAL) / (float)vrefint_data;

    return vref;
}

uint32_t STM32WBClass::resetCause()
{
    return stm32wb_system_reset_cause();
}

uint32_t STM32WBClass::wakeupReason()
{
    return stm32wb_system_wakeup_reason();
}

uint32_t STM32WBClass::faultReason(const armv7m_fault_info_t * &info)
{
    info = &armv7m_fault_info;

    return stm32wb_system_reset_reason();
}

uint32_t STM32WBClass::assertReason(const char * &assertion, const char * &file, uint32_t &line)
{
    assertion = armv7m_assert_info.assertion;
    file = armv7m_assert_info.file;
    line = armv7m_assert_info.line;
  
    return stm32wb_system_reset_reason();
}

uint32_t STM32WBClass::panicReason()
{
    return stm32wb_system_reset_reason();
}

bool STM32WBClass::setClocks(uint32_t hclk)
{
    return stm32wb_system_sysclk_configure(0, hclk, 0, 0);
}

bool STM32WBClass::setClocks(uint32_t sysclk, uint32_t hclk, uint32_t pclk1, uint32_t pclk2)
{
    return stm32wb_system_sysclk_configure(sysclk, hclk, pclk1, pclk2);
}

bool STM32WBClass::boostEnable()
{
    if (__stm32wb_boost_enable || stm32wb_system_boost_enable()) {
        __stm32wb_boost_enable = true;

        return true;
    }
    
    return false;
}

bool STM32WBClass::boostDisable()
{
    if (!__stm32wb_boost_enable || stm32wb_system_boost_disable()) {
        __stm32wb_boost_enable = false;

        return true;
    }

    return false;
}

void STM32WBClass::getClocks(uint32_t &sysclk, uint32_t &hclk, uint32_t &pclk1, uint32_t &pclk2)
{
    sysclk = stm32wb_system_sysclk();
    hclk = stm32wb_system_hclk();
    pclk1 = stm32wb_system_pclk1();
    pclk2 = stm32wb_system_pclk2();
}

void STM32WBClass::wakeup()
{
    k_event_send(__arduino_task, WIRING_EVENT_WAKEUP);
}

bool STM32WBClass::sleep()
{
    return sleep(POLICY_SLEEP, 0xffffffff);
}

bool STM32WBClass::sleep(uint32_t timeout)
{
    return sleep(POLICY_SLEEP, timeout);
}

bool STM32WBClass::sleep(uint32_t policy, uint32_t timeout)
{
    uint32_t mask;
    
    if ((policy < POLICY_RUN) || (policy > POLICY_STOP)) {
        return false;
    }

    if (armv7m_core_is_in_interrupt() || k_system_is_locked()) {
        return false;
    }
    
    if (k_task_self() != __arduino_task) {
        return false;
    }

    mask = 0;
    
    policy = stm32wb_system_policy(policy);

    k_event_receive(WIRING_EVENT_WAKEUP, (K_EVENT_ANY | K_EVENT_CLEAR), timeout, &mask);
    
    stm32wb_system_policy(policy);
    
    return !!mask;
}

bool STM32WBClass::stop()
{
    return sleep(POLICY_STOP, 0xffffffff);
}

bool STM32WBClass::stop(uint32_t timeout)
{
    return sleep(POLICY_STOP, timeout);
}

bool STM32WBClass::delay(uint32_t policy, uint32_t delay)
{
    if (!delay) {
        return false;
    }
    
    if ((policy < POLICY_RUN) || (policy > POLICY_STOP)) {
        return false;
    }

    if (armv7m_core_is_in_interrupt() || k_system_is_locked()) {
        return false;
    }

    if (k_task_self() != __arduino_task) {
        return false;
    }

    policy = stm32wb_system_policy(policy);

    k_task_delay(delay);
    
    stm32wb_system_policy(policy);
    
    return true;
}

void STM32WBClass::standby()
{
    standby(0xffffffff);
}

void STM32WBClass::standby(uint32_t timeout)
{
    stm32wb_system_standby(0, timeout);
}

void STM32WBClass::standby(uint32_t pin, uint32_t mode)
{
    standby(pin, mode, 0xffffffff);
}

void STM32WBClass::standby(uint32_t pin, uint32_t mode, uint32_t timeout)
{
    uint32_t wakeup;

    if ((pin >= PINS_COUNT) || !(g_APinDescription[pin].attr & PIN_ATTR_WKUP)) {
        return;
    }

    wakeup = 0;

    if (mode & RISING) {
        wakeup |= STM32WB_SYSTEM_WAKEUP_PIN_1_RISING;
    }

    if (mode & FALLING) {
        wakeup |= STM32WB_SYSTEM_WAKEUP_PIN_1_FALLING;
    }

    wakeup <<= ((g_APinDescription[pin].attr & PIN_ATTR_WKUP) - PIN_ATTR_WKUP1);
    
    stm32wb_system_standby(wakeup, timeout);
}

void STM32WBClass::shutdown()
{
    stm32wb_system_shutdown(0);
}

void STM32WBClass::shutdown(uint32_t pin, uint32_t mode)
{
    uint32_t wakeup;

    if ((pin >= PINS_COUNT) || !(g_APinDescription[pin].attr & PIN_ATTR_WKUP)) {
        return;
    }

    wakeup = 0;

    if (mode & RISING) {
        wakeup |= STM32WB_SYSTEM_WAKEUP_PIN_1_RISING;
    }

    if (mode & FALLING) {
        wakeup |= STM32WB_SYSTEM_WAKEUP_PIN_1_FALLING;
    }

    wakeup <<= ((g_APinDescription[pin].attr & PIN_ATTR_WKUP) - PIN_ATTR_WKUP1);
    
    stm32wb_system_shutdown(wakeup);
}

void STM32WBClass::reset()
{
    stm32wb_system_reset();
}

void STM32WBClass::dfu()
{
    stm32wb_system_dfu();
}

void STM32WBClass::onStandby(void(*callback)(void))
{
    onStandby(Callback(callback));
}

void STM32WBClass::onStandby(Callback callback)
{
    stm32wb_callbacks.standby_callback = callback;

    if (!stm32wb_callbacks.notify.callback)
    {
        stm32wb_system_notify(&stm32wb_callbacks.notify, stm32wb_notify_callback, (void*)&stm32wb_callbacks, STM32WB_SYSTEM_EVENT_STANDBY);
    }

    armv7m_atomic_or((volatile uint32_t*)&stm32wb_callbacks.notify.mask, STM32WB_SYSTEM_EVENT_STANDBY);
}

void STM32WBClass::onShutdown(void(*callback)(void))
{
    onShutdown(Callback(callback));
}

void STM32WBClass::onShutdown(Callback callback)
{
    stm32wb_callbacks.shutdown_callback = callback;

    if (!stm32wb_callbacks.notify.callback)
    {
        stm32wb_system_notify(&stm32wb_callbacks.notify, stm32wb_notify_callback, (void*)&stm32wb_callbacks, STM32WB_SYSTEM_EVENT_SHUTDOWN);
    }

    armv7m_atomic_or((volatile uint32_t*)&stm32wb_callbacks.notify.mask, STM32WB_SYSTEM_EVENT_SHUTDOWN);
}

void STM32WBClass::onReset(void(*callback)(void))
{
    onReset(Callback(callback));
}

void STM32WBClass::onReset(Callback callback)
{
    stm32wb_callbacks.reset_callback = callback;

    if (!stm32wb_callbacks.notify.callback)
    {
        stm32wb_system_notify(&stm32wb_callbacks.notify, stm32wb_notify_callback, (void*)&stm32wb_callbacks, STM32WB_SYSTEM_EVENT_RESET);
    }

    armv7m_atomic_or((volatile uint32_t*)&stm32wb_callbacks.notify.mask, STM32WB_SYSTEM_EVENT_RESET);
}

void STM32WBClass::swdEnable()
{
    stm32wb_system_swd_enable();
}

void STM32WBClass::swdDisable()
{
    stm32wb_system_swd_disable();
}

void STM32WBClass::wdtEnable(uint32_t timeout)
{
    stm32wb_iwdg_enable(timeout);
}

void STM32WBClass::wdtReset()
{
    stm32wb_iwdg_reset();
}

bool STM32WBClass::flashErase(uint32_t address, uint32_t count)
{
    stm32wb_flash_request_t request;
    
    if ((address & 4095) || (count & 4095) || !count) {
        return false;
    }

    if ((address < FLASHSTART) || ((address + count) > FLASHEND)) {
        return false;
    }

    request.control = STM32WB_FLASH_CONTROL_ERASE | STM32WB_FLASH_CONTROL_FLUSH;
    request.address = address;
    request.count = count;
    request.data = NULL;
    request.callback = NULL;
    request.context = NULL;
    
    if (!stm32wb_flash_request(&request)) {
        return false;
    }

    while (request.status == STM32WB_FLASH_STATUS_BUSY) {
        __WFE();
    }
    
    return (request.status == STM32WB_FLASH_STATUS_SUCCESS);
}

bool STM32WBClass::flashProgram(uint32_t address, const void *data, uint32_t count)
{
    stm32wb_flash_request_t request;

    if ((address & 3) || (count & 3) || !count) {
        return false;
    }

    if ((address < FLASHSTART) || ((address + count) > FLASHEND)) {
        return false;
    }

    request.control = STM32WB_FLASH_CONTROL_PROGRAM | STM32WB_FLASH_CONTROL_FLUSH;
    request.address = address;
    request.count = count;
    request.data = (const uint8_t*)data;
    request.callback = NULL;
    request.context = NULL;
    
    if (!stm32wb_flash_request(&request)) {
        return false;
    }

    while (request.status == STM32WB_FLASH_STATUS_BUSY) {
        __WFE();
    }
    
    return (request.status == STM32WB_FLASH_STATUS_SUCCESS);
}

STM32WBClass STM32WB;
