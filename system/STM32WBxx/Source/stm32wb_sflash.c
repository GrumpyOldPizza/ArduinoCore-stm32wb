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

#include "armv7m.h"
#include "stm32wb_sflash.h"

#include "dosfs_sflash.h"

typedef struct _stm32wb_sflash_device_t {
  const stm32wb_sflash_interface_t *interface;
  const stm32wb_sflash_info_t      *info;
  uint32_t                         size;
} stm32wb_sflash_device_t;

static stm32wb_sflash_device_t stm32wb_sflash_device;

void __stm32wb_sflash_initialize(const stm32wb_sflash_interface_t *interface, const stm32wb_sflash_info_t *info)
{
    uint32_t address, info_head[DOSFS_SFLASH_INFO_HEAD_COUNT], info_tail[DOSFS_SFLASH_INFO_TAIL_COUNT];

    stm32wb_sflash_device.interface = interface;
    stm32wb_sflash_device.info = info;

    stm32wb_sflash_device.size = info->capacity;

    (*interface->acquire)();
    
    for (address = info->capacity - info->block_size; address >= info->capacity - 4 * info->block_size; address -= info->block_size)
    {
	(*interface->read)(address + DOSFS_SFLASH_INFO_HEAD_OFFSET, (uint8_t*)&info_head[0], DOSFS_SFLASH_INFO_HEAD_SIZE);
	(*interface->read)(address + DOSFS_SFLASH_INFO_TAIL_OFFSET, (uint8_t*)&info_tail[0], DOSFS_SFLASH_INFO_TAIL_SIZE);
	
	if ((info_head[DOSFS_SFLASH_INFO_HEAD_MAGIC] == DOSFS_SFLASH_MAGIC_HEAD) &&
	    ((info_head[DOSFS_SFLASH_INFO_HEAD_DATA_START] + info_head[DOSFS_SFLASH_INFO_HEAD_DATA_SIZE]) == info->capacity) &&
	    (info_tail[DOSFS_SFLASH_INFO_TAIL_MAGIC] == DOSFS_SFLASH_MAGIC_TAIL))
	{
	    stm32wb_sflash_device.size = info_head[DOSFS_SFLASH_INFO_HEAD_DATA_START];
	    
	    break;
	}
    }
	
    (*interface->release)();
}

bool stm32wb_sflash_query(const stm32wb_sflash_interface_t **p_interface_return, const stm32wb_sflash_info_t **p_info_return, uint32_t *p_size_return)
{
    if (!stm32wb_sflash_device.interface)
    {
	return false;
    }

    if (p_interface_return)
    {
	*p_interface_return = stm32wb_sflash_device.interface;
    }

    if (p_info_return)
    {
	*p_info_return = stm32wb_sflash_device.info;
    }

    if (p_size_return)
    {
	*p_size_return = stm32wb_sflash_device.size;
    }
    
    return true;
}

bool stm32wb_sflash_resize(uint32_t size)
{
    const stm32wb_sflash_interface_t *interface;
    const stm32wb_sflash_info_t *info;
    uint32_t address;
    volatile uint8_t status;
    
    if (!stm32wb_sflash_device.interface)
    {
	return false;
    }
    
    if (stm32wb_sflash_device.size < size)
    {
	interface = stm32wb_sflash_device.interface;
	info = stm32wb_sflash_device.info;
	
	(*interface->acquire)();

	for (address = stm32wb_sflash_device.size; address < size; address += info->block_size)
	{
	    (*interface->erase)(address, &status);

	    while ((*interface->busy)()) { }
	}

	(*interface->release)();
    }

    stm32wb_sflash_device.size = size;

    return true;
}
