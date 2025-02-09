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
#define USB_PID 0x6675
#define USB_MANUFACTURER "Tlera Corporation"
#define USB_PRODUCT "Firefly"
//#define USB_TYPE 1
#define USB_TYPE 2
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
    0,
    USB_MANUFACTURER,
    USB_PRODUCT,
};

#endif

struct USBDeviceClass::USBDeviceCallbacks USBDeviceClass::m_callbacks = {
    .events = 0,
    .attach_callback = Callback(),
    .detach_callback = Callback(),
    .connect_callback = Callback(),
    .suspend_callback = Callback(),
    .resume_callback = Callback(),
    .work = K_WORK_INIT(&USBDeviceClass::notifyRoutine, NULL)
};

USBDeviceClass::USBDeviceClass() {
#if (USB_TYPE == 2)
#if (STM32WB_CONFIG_SFLASH == 1) || (STORAGE_TYPE == 1)
    dosfs_sflash_initialize();
#endif  
#endif

#if (USB_TYPE != 0)
    stm32wb_usbd_configure(&g_USBDevice, &USB_INFO);
#endif
}

bool USBDeviceClass::begin() {
#if (USB_TYPE != 0)
    return stm32wb_usbd_enable();
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
    onAttach(Callback(callback));
}

void USBDeviceClass::onAttach(Callback callback) {
    stm32wb_usbd_notify((stm32wb_usbd_event_callback_t)&USBDeviceClass::eventCallback, (void*)NULL);

    m_callbacks.attach_callback = callback;
}

void USBDeviceClass::onDetach(void(*callback)(void)) {
    onDetach(Callback(callback));
}

void USBDeviceClass::onDetach(Callback callback) {
    stm32wb_usbd_notify((stm32wb_usbd_event_callback_t)&USBDeviceClass::eventCallback, (void*)NULL);

    m_callbacks.detach_callback = callback;
}

void USBDeviceClass::onConnect(void(*callback)(void)) {
    onConnect(Callback(callback));
}

void USBDeviceClass::onConnect(Callback callback) {
    stm32wb_usbd_notify((stm32wb_usbd_event_callback_t)&USBDeviceClass::eventCallback, (void*)NULL);

    m_callbacks.connect_callback = callback;
}

void USBDeviceClass::onSuspend(void(*callback)(void)) {
    onSuspend(Callback(callback));
}

void USBDeviceClass::onSuspend(Callback callback) {
    stm32wb_usbd_notify((stm32wb_usbd_event_callback_t)&USBDeviceClass::eventCallback, (void*)NULL);

    m_callbacks.suspend_callback = callback;
}

void USBDeviceClass::onResume(void(*callback)(void)) {
    onResume(Callback(callback));
}

void USBDeviceClass::onResume(Callback callback) {
    stm32wb_usbd_notify((stm32wb_usbd_event_callback_t)&USBDeviceClass::eventCallback, (void*)NULL);

    m_callbacks.resume_callback = callback;
}

void USBDeviceClass::eventCallback(void *context, uint32_t events) {
    armv7m_atomic_or(&m_callbacks.events, events);

    k_work_submit(&m_callbacks.work);
}

void USBDeviceClass::notifyRoutine(void *context) {
    uint32_t events;

    events = armv7m_atomic_swap(&m_callbacks.events, 0);

    if (events & STM32WB_USBD_EVENT_ATTACH) {
        m_callbacks.attach_callback();
    }

    if (events & STM32WB_USBD_EVENT_DETACH) {
        m_callbacks.attach_callback();
    }
    
    if (events & STM32WB_USBD_EVENT_CONNECT) {
        m_callbacks.connect_callback();
    }

    if (events & STM32WB_USBD_EVENT_SUSPEND) {
        m_callbacks.suspend_callback();
    }

    if (events & STM32WB_USBD_EVENT_RESUME) {
        m_callbacks.resume_callback();
    }
}

USBDeviceClass USBDevice;

#endif /* USBCON */

