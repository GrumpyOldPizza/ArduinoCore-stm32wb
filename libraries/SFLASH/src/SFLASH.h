/*
 * Copyright (c) 2020-2021 Thomas Roell.  All rights reserved.
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

#ifndef SFLASH_H
#define SFLASH_H

#include <Arduino.h>

enum SFLASHStatus {
  SFLASH_STATUS_SUCCESS = 0,
  SFLASH_STATUS_FAILURE = 1,
  SFLASH_STATUS_BUSY    = 255
};


class SFLASHClass {
public:
    SFLASHClass();
    SFLASHClass(const SFLASHClass&) = delete;
    SFLASHClass& operator=(const SFLASHClass&) = delete;
    SFLASHClass(const SFLASHClass&&) = delete;
    SFLASHClass& operator=(const SFLASHClass&&) = delete;

    bool begin();
    void end();

    bool identify(uint8_t &MID, uint16_t &DID) const;
    uint32_t capacity() const;
    uint32_t blockSize() const;
    uint32_t pageSize() const;

    uint32_t length() const;

    bool busy();
    SFLASHStatus status();
    
    bool erase(uint32_t address);
    bool program(uint32_t address, const void *buffer, size_t count);
    bool read(uint32_t address, void *buffer, size_t count);

private:
    bool _busy;
    volatile uint8_t _status;
    const struct _stm32wb_sflash_interface_t *_interface;
    const struct _stm32wb_sflash_info_t *_info;
    uint32_t _size;
};

extern SFLASHClass SFLASH;

#endif // SFLASH_H
