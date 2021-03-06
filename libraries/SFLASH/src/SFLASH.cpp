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

#include "Arduino.h"
#include "SFLASH.h"
#include "wiring_private.h"

SFLASHClass::SFLASHClass()
{
}

bool SFLASHClass::begin()
{
    if (_interface) {
        return true;
    }

    if (!stm32wb_sflash_query(&_interface, &_info, &_size))
    {
        return false;
    }

    _busy = false;
    _status = STM32WB_SFLASH_STATUS_SUCCESS;
    
    return true;
}

void SFLASHClass::end()
{
}

bool SFLASHClass::identify(uint8_t &MID, uint16_t &DID) const
{
    if (!_interface) {
        return false;
    }

    MID = _info->mid;
    DID = _info->did;

    return true;
}

uint32_t SFLASHClass::capacity() const
{
    if (!_interface) {
        return 0;
    }

    return _info->capacity;
}

uint32_t SFLASHClass::blockSize() const
{
    if (!_interface) {
        return 0;
    }

    return _info->block_size;
}

uint32_t SFLASHClass::pageSize() const
{
    if (!_interface) {
        return 0;
    }

    return _info->page_size;
}


uint32_t SFLASHClass::length() const
{
    if (!_interface) {
        return 0;
    }

    return _size;
}

bool SFLASHClass::busy()
{
    if (!_interface) {
        return false;
    }

    if (!_busy) {
        return false;
    }

    (*_interface->acquire)();

    _busy = ((*_interface->busy)());
    
    (*_interface->release)();
    
    return _busy;
}

SFLASHStatus SFLASHClass::status()
{
    if (!_interface) {
        return SFLASH_STATUS_SUCCESS;
    }
    
    return (SFLASHStatus)_status;
}

bool SFLASHClass::erase(uint32_t address)
{
    if (!_interface) {
        return false;
    }

    if (address & (_info->block_size -1)) {
        return false;
    }

    if (address >= _size) {
        return false;
    }

    (*_interface->acquire)();

    while ((*_interface->busy)()) { }

    (*_interface->erase)(address, &_status);

    (*_interface->release)();

    _busy = true;
    
    return true;
}

bool SFLASHClass::program(uint32_t address, const void *buffer, size_t count)
{
    if (!_interface) {
        return false;
    }

    if (!count) {
        return false;
    }

    if ((address & ~(_info->page_size -1)) != ((address + count -1) & ~(_info->page_size -1))) {
        return false;
    }

    if ((address + count) >= _size) {
        return false;
    }
    
    (*_interface->acquire)();

    while ((*_interface->busy)()) { }

    (*_interface->program)(address, (const uint8_t*)buffer, count, &_status);

    (*_interface->release)();

    _busy = true;

    return true;
}

bool SFLASHClass::read(uint32_t address, void *buffer, size_t count)
{
    if (!_interface) {
        return false;
    }

    if (!count) {
        return false;
    }

    if ((address + count) >= _size) {
        return false;
    }
    
    (*_interface->acquire)();

    while ((*_interface->busy)()) { }

    (*_interface->read)(address, (uint8_t*)buffer, count);

    (*_interface->release)();

    _busy = false;

    _status = STM32WB_SFLASH_STATUS_SUCCESS;
    
    return true;
}

SFLASHClass SFLASH;
