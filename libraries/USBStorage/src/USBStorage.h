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

#ifndef _USBSTORAGE_H_INCLUDED
#define _USBSTORAGE_H_INCLUDED

#include <Arduino.h>
#include "dosfs_storage.h"

typedef void (*USBStorageObjectReadCallback)(class USBStorageObject *object, uint32_t offset, uint8_t *data, uint32_t size);
typedef void (*USBStorageObjectWriteCallback)(class USBStorageObject *object, uint32_t offset, const uint8_t *data, uint32_t size);
typedef void (*USBStorageObjectDeleteCallback)(class USBStorageObject *object);

class USBStorageObject {
public:
    USBStorageObject(const char *name, bool readOnly, uint32_t size, USBStorageObjectReadCallback onRead, USBStorageObjectWriteCallback onWrite, USBStorageObjectDeleteCallback onDelete);
    USBStorageObject(const USBStorageObject&) = delete;
    USBStorageObject& operator=(const USBStorageObject&) = delete;

private:
    dosfs_object_t m_object;

    USBStorageObjectReadCallback m_read_callback;
    USBStorageObjectWriteCallback m_write_callback;
    USBStorageObjectDeleteCallback m_delete_callback;

    static void readCallback(dosfs_object_t *object, uint32_t offset, uint8_t *data, uint32_t size);
    static void writeCallback(dosfs_object_t *object, uint32_t offset, const uint8_t *data, uint32_t size);
    static void deleteCallback(dosfs_object_t *object);

    friend class USBStorageMedium;
};

class USBStorageMedium {
public:
    USBStorageMedium(const char *name, bool writeProtected = true);
    USBStorageMedium(const USBStorageMedium&) = delete;
    USBStorageMedium& operator=(const USBStorageMedium&) = delete;

    bool addObject(class USBStorageObject &object);
    bool removeObject(class USBStorageObject &object);

private:
    const char     *m_name;
    bool           m_write_protected;
    bool           m_attached;
    int            m_count;
    dosfs_object_t *m_objects[8];
    dosfs_storage_t m_storage;
  
    friend class USBStorageClass;
};
  
class USBStorageClass {
public:
    USBStorageClass();
    USBStorageClass(const USBStorageClass&) = delete;
    USBStorageClass& operator=(const USBStorageClass&) = delete;

    bool attached();
    bool started();
    bool stopped();
    bool ejected();
    
    bool attach(class USBStorageMedium &medium);
    bool attach();
    bool detach();

    void onAttach(void(*callback)(void));
    void onAttach(Callback callback);
    void onDetach(void(*callback)(void));
    void onDetach(Callback callback);
    void onStart(void(*callback)(void));
    void onStart(Callback callback);
    void onStop(void(*callback)(void));
    void onStop(Callback callback);
    void onEject(void(*callback)(void));
    void onEject(Callback callback);
    
private:
    enum USBStorageState {
        Detached = 0,
        Attached,
        Started,
        Stopped,
        Ejected
    };

    volatile enum USBStorageState m_state;
    volatile uint32_t m_events;
    
    Callback m_attach_callback;
    Callback m_detach_callback;
    Callback m_start_callback;
    Callback m_stop_callback;
    Callback m_eject_callback;

    k_work_t m_work;
    
    static void eventRoutine(class USBStorageClass *self);
    static void eventCallback(class USBStorageClass *self, uint32_t events);
};

extern USBStorageClass USBStorage;

#endif
