/*
 * Copyright (c) 2023 Thomas Roell.  All rights reserved.
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
#include "USBStorage.h"
#include "stm32wb_usbd.h"
#include "stm32wb_usbd_msc.h"
#include "dosfs_device.h"

USBStorageObject::USBStorageObject(const char *name, bool readOnly, uint32_t size, USBStorageObjectReadCallback onRead, USBStorageObjectWriteCallback onWrite, USBStorageObjectDeleteCallback onDelete) {
    m_object.name = name;
    m_object.read_only = readOnly;
    m_object.size = size;
    m_object.read_callback = onRead ? USBStorageObject::readCallback : nullptr;
    m_object.write_callback = onWrite ? USBStorageObject::writeCallback : nullptr;
    m_object.delete_callback = onDelete ? USBStorageObject::deleteCallback : nullptr;

    m_read_callback = onRead;
    m_write_callback = onWrite;
    m_delete_callback = onDelete;
}

void USBStorageObject::readCallback(dosfs_object_t *object, uint32_t offset, uint8_t *data, uint32_t size) {
    class USBStorageObject *self = (class USBStorageObject*)((uint8_t*)object - offsetof(USBStorageObject, m_object));
    
    (*self->m_read_callback)(self, offset, data, size);
}

void USBStorageObject::writeCallback(dosfs_object_t *object, uint32_t offset, const uint8_t *data, uint32_t size) {
    class USBStorageObject *self = (class USBStorageObject*)((uint8_t*)object - offsetof(USBStorageObject, m_object));
    
    (*self->m_write_callback)(self, offset, data, size);
}

void USBStorageObject::deleteCallback(dosfs_object_t *object) {
    class USBStorageObject *self = (class USBStorageObject*)((uint8_t*)object - offsetof(USBStorageObject, m_object));

    (*self->m_delete_callback)(self);
}

USBStorageMedium::USBStorageMedium(const char *name, bool writeProtected) {
    m_name = name;
    m_write_protected = writeProtected;
    m_attached = false;
    m_count = 0;
}

bool USBStorageMedium::addObject(class USBStorageObject &object) {
    if (m_count == 8) {
        return false;
    }
    
    m_objects[m_count++] = &object.m_object;

    return true;
}
    
USBStorageClass::USBStorageClass() {
    m_state = USBStorageState::Detached;

    m_events = 0;
    
    m_attach_callback = Callback();
    m_detach_callback = Callback();
    m_start_callback = Callback();
    m_stop_callback = Callback();
    m_eject_callback = Callback();

    m_work = K_WORK_INIT(&USBStorageClass::eventRoutine, (void*)this);
}

bool USBStorageClass::attached() {
    return (m_state >= USBStorageState::Attached);
}

bool USBStorageClass::started() {
    return (m_state == USBStorageState::Started);
}

bool USBStorageClass::stopped() {
    return (m_state == USBStorageState::Stopped);
}

bool USBStorageClass::ejected() {
    return (m_state == USBStorageState::Ejected);
}

bool USBStorageClass::attach(class USBStorageMedium &medium) {
    if (m_state != USBStorageState::Detached) {
        return false;
    }

    if (!dosfs_storage_init(&medium.m_storage, medium.m_write_protected, medium.m_name, medium.m_count, &medium.m_objects[0])) {
        return false;
    }

    if (!stm32wb_usbd_msc_attach(&medium.m_storage.device, (stm32wb_usbd_msc_event_callback_t)&USBStorageClass::eventCallback, (void*)this)) {
        return false;
    }

    medium.m_attached = true;
    
    return true;
}
  

bool USBStorageClass::attach() {
    if (m_state != USBStorageState::Detached) {
        return false;
    }

    if (!dosfs_device.interface) {
        return false;
    }

    if (!stm32wb_usbd_msc_attach(&dosfs_device, (stm32wb_usbd_msc_event_callback_t)&USBStorageClass::eventCallback, (void*)this)) {
        return false;
    }

    return true;
}

bool USBStorageClass::detach() {
    if (m_state == USBStorageState::Detached) {
        return false;
    }
    
    return stm32wb_usbd_msc_detach();
}

void USBStorageClass::onAttach(void(*callback)(void)) {
    m_attach_callback = Callback(callback);
}

void USBStorageClass::onAttach(Callback callback) {
    m_attach_callback = callback;
}

void USBStorageClass::onDetach(void(*callback)(void)) {
    m_detach_callback = Callback(callback);
}

void USBStorageClass::onDetach(Callback callback) {
    m_detach_callback = callback;
}

void USBStorageClass::onStart(void(*callback)(void)) {
    m_start_callback = Callback(callback);
}

void USBStorageClass::onStart(Callback callback) {
    m_start_callback = callback;
}

void USBStorageClass::onStop(void(*callback)(void)) {
    m_stop_callback = Callback(callback);
}

void USBStorageClass::onStop(Callback callback) {
    m_stop_callback = callback;
}

void USBStorageClass::onEject(void(*callback)(void)) {
    m_eject_callback = Callback(callback);
}

void USBStorageClass::onEject(Callback callback) {
    m_eject_callback = callback;
}

void USBStorageClass::eventRoutine(class USBStorageClass *self) {
    uint32_t events;

    events = armv7m_atomic_swap(&self->m_events, 0);

    if (events & STM32WB_USBD_MSC_EVENT_ATTACH) {
        self->m_state = USBStorageState::Attached;
        
        self->m_attach_callback();
    }

    if (events & STM32WB_USBD_MSC_EVENT_DETACH) {
        self->m_state = USBStorageState::Detached;

        self->m_attach_callback();
    }

    if (events & STM32WB_USBD_MSC_EVENT_START) {
        self->m_state = USBStorageState::Started;

        self->m_start_callback();
    }

    if (events & STM32WB_USBD_MSC_EVENT_STOP) {
        self->m_state = USBStorageState::Stopped;

        self->m_stop_callback();
    }

    if (events & STM32WB_USBD_MSC_EVENT_EJECT) {
        self->m_state = USBStorageState::Ejected;

        self->m_eject_callback();
    }
}

void USBStorageClass::eventCallback(class USBStorageClass *self, uint32_t events) {
    armv7m_atomic_or(&self->m_events, events);

    k_work_submit(&self->m_work);
}

USBStorageClass USBStorage;
