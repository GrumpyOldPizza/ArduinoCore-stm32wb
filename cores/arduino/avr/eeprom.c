/*
 * Copyright (c) 2016-2021 Thomas Roell.  All rights reserved.
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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "stm32wbxx.h"
#include "stm32wb_eeprom.h"

#include "avr/eeprom.h"

uint8_t eeprom_read_byte(const uint8_t *address)
{
    uint8_t data;

    stm32wb_eeprom_read((uint32_t)address, (uint8_t*)&data, sizeof(data));

    return data;
}

uint16_t eeprom_read_word(const uint16_t *address)
{
    uint16_t data;

    stm32wb_eeprom_read((uint32_t)address, (uint8_t*)&data, sizeof(data));

    return data;
}

uint32_t eeprom_read_dword(const uint32_t *address)
{
    uint32_t data;

    stm32wb_eeprom_read((uint32_t)address, (uint8_t*)&data, sizeof(data));

    return data;
}

float eeprom_read_float(const float *address)
{
    float data;

    stm32wb_eeprom_read((uint32_t)address, (uint8_t*)&data, sizeof(data));

    return data;
}

void eeprom_read_block(void *data, const void *address, uint32_t count)
{
    stm32wb_eeprom_read((uint32_t)address, (uint8_t*)data, count);
}

void eeprom_write_byte(uint8_t *address, uint8_t data)
{
    stm32wb_eeprom_write((uint32_t)address, (const uint8_t*)&data, sizeof(data));
}

void eeprom_write_word(uint16_t *address, uint16_t data)
{
    stm32wb_eeprom_write((uint32_t)address, (const uint8_t*)&data, sizeof(data));
}

void eeprom_write_dword(uint32_t *address, uint32_t data)
{
    stm32wb_eeprom_write((uint32_t)address, (const uint8_t*)&data, sizeof(data));
}

void eeprom_write_float(float *address, float data)
{
    stm32wb_eeprom_write((uint32_t)address, (const uint8_t*)&data, sizeof(data));
}

void eeprom_write_block(const void *data, void *address, uint32_t count)
{
    stm32wb_eeprom_write((uint32_t)address, (const uint8_t*)data, count);
}

int eeprom_is_ready(void)
{
    return stm32wb_eeprom_is_ready();
}
