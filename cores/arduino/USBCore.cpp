/*
 * Copyright (c) 2016-2022 Thomas Roell.  All rights reserved.
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
#include "USBAPI.h"
#include "wiring_private.h"

#if defined(USBCON)

#if !defined(USB_TYPE)
#define USB_VID 0x1209
#define USB_PID 0x6671
#define USB_DID 0x0100
#define USB_MANUFACTURER "Tlera Corporation"
#define USB_PRODUCT "Firefly"
#define USB_TYPE 1
#endif

#if (USB_TYPE != 0)
#if (USB_TYPE == 1)
#define USB_INFO stm32wb_usbd_dfu_cdc_info
#endif

#if (USB_TYPE == 2)
#define USB_INFO stm32wb_usbd_dfu_cdc_msc_info
#endif

static const stm32wb_usbd_device_t g_USBDevice =
{
    USB_VID,
    USB_PID,
    USB_DID,
    USB_MANUFACTURER,
    USB_PRODUCT,
};

static const stm32wb_usbd_params_t g_USBParams =
{
    STM32WB_USB_IRQ_PRIORITY,
    STM32WB_CONFIG_PIN_VBUS,
};

#endif

USBDeviceClass::USBDeviceClass() {
    m_events = 0;
    
    m_attach_callback = Callback();
    m_detach_callback = Callback();
    m_connect_callback = Callback();
    m_suspend_callback = Callback();
    m_resume_callback = Callback();

    m_work = K_WORK_INIT(&USBDeviceClass::notifyRoutine, (void*)this);

#if (USB_TYPE == 2)
#if (STM32WB_CONFIG_SFLASH == 1) || (STORAGE_TYPE == 1)
    dosfs_sflash_initialize();
#endif  
#endif

#if (USB_TYPE != 0)
    stm32wb_usbd_configure(&g_USBDevice, &USB_INFO, &g_USBParams);
#endif
}

bool USBDeviceClass::begin() {
#if (USB_TYPE != 0)
    return stm32wb_usbd_enable((stm32wb_usbd_event_callback_t)&USBDeviceClass::eventCallback, (void*)this);
#endif

    return false;
}

void USBDeviceClass::end() {
#if (USB_TYPE != 0)
    stm32wb_usbd_disable();
#endif    
}

void USBDeviceClass::start() {
#if (USB_TYPE != 0)
    stm32wb_usbd_start();
#endif
}

void USBDeviceClass::stop() {
#if (USB_TYPE != 0)
    stm32wb_usbd_stop();
#endif
}

void USBDeviceClass::wakeup() {
#if (USB_TYPE != 0)
    stm32wb_usbd_wakeup();
#endif
}
    
bool USBDeviceClass::attached() {
#if (USB_TYPE != 0)
    return (stm32wb_usbd_state() >= USB_STATE_ATTACHED);
#endif

    return false;
}

bool USBDeviceClass::connected() {
#if (USB_TYPE != 0)
    return (stm32wb_usbd_state() >= USB_STATE_DEFAULT);
#endif

    return false;
}

bool USBDeviceClass::suspended() {
#if (USB_TYPE != 0)
    return stm32wb_usbd_is_suspended();
#endif

    return false;
}

void USBDeviceClass::onAttach(void(*callback)(void)) {
    m_attach_callback = Callback(callback);
}

void USBDeviceClass::onAttach(Callback callback) {
    m_attach_callback = callback;
}

void USBDeviceClass::onDetach(void(*callback)(void)) {
    m_detach_callback = Callback(callback);
}

void USBDeviceClass::onDetach(Callback callback) {
    m_detach_callback = callback;
}

void USBDeviceClass::onConnect(void(*callback)(void)) {
    m_connect_callback = Callback(callback);
}

void USBDeviceClass::onConnect(Callback callback) {
    m_connect_callback = callback;
}

void USBDeviceClass::onSuspend(void(*callback)(void)) {
    m_suspend_callback = Callback(callback);
}

void USBDeviceClass::onSuspend(Callback callback) {
    m_suspend_callback = callback;
}

void USBDeviceClass::onResume(void(*callback)(void)) {
    m_resume_callback = Callback(callback);
}

void USBDeviceClass::onResume(Callback callback) {
    m_resume_callback = callback;
}

void USBDeviceClass::eventCallback(class USBDeviceClass *self, uint32_t events) {
    armv7m_atomic_or(&self->m_events, events);

    k_work_submit(&self->m_work);
}

void USBDeviceClass::notifyRoutine(class USBDeviceClass *self) {
    uint32_t events;

    events = armv7m_atomic_swap(&self->m_events, 0);

    if (events & STM32WB_USBD_EVENT_ATTACH) {
        self->m_attach_callback();
    }

    if (events & STM32WB_USBD_EVENT_DETACH) {
        self->m_attach_callback();
    }
    
    if (events & STM32WB_USBD_EVENT_CONNECT) {
        self->m_connect_callback();
    }

    if (events & STM32WB_USBD_EVENT_SUSPEND) {
        self->m_suspend_callback();
    }

    if (events & STM32WB_USBD_EVENT_RESUME) {
        self->m_resume_callback();
    }
}

USBDeviceClass USBDevice;

#endif /* USBCON */

