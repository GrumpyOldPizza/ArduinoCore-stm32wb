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
#include "stm32wb_flash.h"
#include "stm32wb_eeprom.h"
#include "stm32wb_system.h"

/*******************************************************************/

#define STM32WB_EEPROM_MAGIC_1 0xaa55ee77
#define STM32WB_EEPROM_MAGIC_2 0x77eeaa55

extern uint32_t __eeprom_start__, __eeprom_end__;

#define STM32WB_EEPROM_STATE_NONE      0
#define STM32WB_EEPROM_STATE_NOT_READY 1
#define STM32WB_EEPROM_STATE_READY     2
#define STM32WB_EEPROM_STATE_PROGRAM   3
#define STM32WB_EEPROM_STATE_ERASE     4
#define STM32WB_EEPROM_STATE_COPY      5

typedef struct _stm32wb_eeprom_device_t {
    volatile uint8_t                   state;
    uint32_t                           flash;
    uint32_t                           sequence;
    uint32_t                           size;
    uint32_t                           offset;
    stm32wb_flash_request_t            request;
    uint8_t                            program[16];
    volatile uint32_t                  mask_1;
    volatile uint32_t                  mask_2[STM32WB_EEPROM_SIZE / 128];
    volatile uint32_t                  data[STM32WB_EEPROM_SIZE / 4];
} stm32wb_eeprom_device_t;

static stm32wb_eeprom_device_t stm32wb_eeprom_device;

/* CRC16 with 0x8005 polynom, without final bit reverse.
 */
static uint32_t stm32wb_eeprom_crc16(const uint8_t *data, unsigned int count)
{
    uint32_t crc16 = 0;
    uint8_t c;

    static const uint16_t stm32wb_eeprom_crc16_lut[16] = {
        0x0000, 0xcc01, 0xd801, 0x1400,
        0xf001, 0x3c00, 0x2800, 0xe401,
        0xa001, 0x6c00, 0x7800, 0xb401,
        0x5000, 0x9c01, 0x8801, 0x4400,
    };

    while (count--)
    {
        c = *data++;

        crc16 = (crc16 >> 4) ^ stm32wb_eeprom_crc16_lut[(crc16 ^ c       ) & 15];
        crc16 = (crc16 >> 4) ^ stm32wb_eeprom_crc16_lut[(crc16 ^ (c >> 4)) & 15];
    }

    return crc16;
}

static void stm32wb_eeprom_callback(void *context)
{
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_EEPROM);
}

bool __stm32wb_eeprom_reset(void)
{
    uint8_t *program;

    if (stm32wb_eeprom_device.request.status == STM32WB_FLASH_STATUS_BUSY)
    {
        return false;
    }

    stm32wb_eeprom_device.flash = (uint32_t)&__eeprom_start__;
    stm32wb_eeprom_device.sequence = 1;
    stm32wb_eeprom_device.size = ((uint32_t)&__eeprom_end__ - (uint32_t)&__eeprom_start__) / 2;
    stm32wb_eeprom_device.offset = 0;
    
    stm32wb_eeprom_device.request.control = STM32WB_FLASH_CONTROL_ERASE;
    stm32wb_eeprom_device.request.address = (uint32_t)stm32wb_eeprom_device.flash;
    stm32wb_eeprom_device.request.count = 2 * stm32wb_eeprom_device.size;
    stm32wb_eeprom_device.request.data = NULL;
    stm32wb_eeprom_device.request.callback = NULL;
    stm32wb_eeprom_device.request.context = NULL;
    
    if (!stm32wb_flash_request(&stm32wb_eeprom_device.request))
    {
        return false;
    }

    while (stm32wb_eeprom_device.request.status == STM32WB_FLASH_STATUS_BUSY)
    {
        __WFE();
    }

    if (stm32wb_eeprom_device.request.status != STM32WB_FLASH_STATUS_SUCCESS)
    {
        return false;
    }
	
    program = &stm32wb_eeprom_device.program[0];
	
    program[ 0] = (uint8_t)(stm32wb_eeprom_device.sequence >>  0);
    program[ 1] = (uint8_t)(stm32wb_eeprom_device.sequence >>  8);
    program[ 2] = (uint8_t)(stm32wb_eeprom_device.sequence >> 16);
    program[ 3] = (uint8_t)(stm32wb_eeprom_device.sequence >> 24);
    program[ 4] = (uint8_t)(stm32wb_eeprom_device.sequence >>  0) ^ 0xff;
    program[ 5] = (uint8_t)(stm32wb_eeprom_device.sequence >>  8) ^ 0xff;
    program[ 6] = (uint8_t)(stm32wb_eeprom_device.sequence >> 16) ^ 0xff;
    program[ 7] = (uint8_t)(stm32wb_eeprom_device.sequence >> 24) ^ 0xff;
    program[ 8] = (uint8_t)(STM32WB_EEPROM_MAGIC_1 >>  0);
    program[ 9] = (uint8_t)(STM32WB_EEPROM_MAGIC_1 >>  8);
    program[10] = (uint8_t)(STM32WB_EEPROM_MAGIC_1 >> 16);
    program[11] = (uint8_t)(STM32WB_EEPROM_MAGIC_1 >> 24);
    program[12] = (uint8_t)(STM32WB_EEPROM_MAGIC_2 >>  0);
    program[13] = (uint8_t)(STM32WB_EEPROM_MAGIC_2 >>  8);
    program[14] = (uint8_t)(STM32WB_EEPROM_MAGIC_2 >> 16);
    program[15] = (uint8_t)(STM32WB_EEPROM_MAGIC_2 >> 24);
    
    stm32wb_eeprom_device.request.control = STM32WB_FLASH_CONTROL_PROGRAM;
    stm32wb_eeprom_device.request.address = (uint32_t)stm32wb_eeprom_device.flash + stm32wb_eeprom_device.size - 16;
    stm32wb_eeprom_device.request.count = 16;
    stm32wb_eeprom_device.request.data = &stm32wb_eeprom_device.program[0];
    
    if (!stm32wb_flash_request(&stm32wb_eeprom_device.request))
    {
        return false;
    }

    while (stm32wb_eeprom_device.request.status == STM32WB_FLASH_STATUS_BUSY)
    {
        __WFE();
    }

    if (stm32wb_eeprom_device.request.status != STM32WB_FLASH_STATUS_SUCCESS)
    {
        return false;
    }

    memset((uint8_t*)&stm32wb_eeprom_device.data[0], 0xff, STM32WB_EEPROM_SIZE);

    stm32wb_eeprom_device.mask_1 = 0;
    memset((uint8_t*)&stm32wb_eeprom_device.mask_2[0], 0x00, (STM32WB_EEPROM_SIZE / 128));

    stm32wb_eeprom_device.request.callback = stm32wb_eeprom_callback;
    stm32wb_eeprom_device.request.context = NULL;

    return true;
}

bool __stm32wb_eeprom_flush(bool erase)
{
    uint32_t index, data, crc16;
    const uint32_t *flash, *flash_s, *flash_e;

    if (stm32wb_eeprom_device.request.status == STM32WB_FLASH_STATUS_BUSY)
    {
        return false;
    }

    stm32wb_eeprom_device.request.callback = NULL;
    stm32wb_eeprom_device.request.context = NULL;
    
    if (stm32wb_eeprom_device.sequence == 1)
    {
        memset((uint8_t*)&stm32wb_eeprom_device.data[0], 0xff, STM32WB_EEPROM_SIZE);
        
        flash_s = (const uint32_t*)stm32wb_eeprom_device.flash;
    }
    else
    {
        memcpy((uint8_t*)(&stm32wb_eeprom_device.data[0]), (const uint8_t*)stm32wb_eeprom_device.flash, STM32WB_EEPROM_SIZE);
        
        flash_s = (const uint32_t*)(stm32wb_eeprom_device.flash + STM32WB_EEPROM_SIZE);
    }
    
    flash_e = (const uint32_t*)(stm32wb_eeprom_device.flash + (stm32wb_eeprom_device.size - 16));
    
    for (flash = flash_s; flash < flash_e; flash += 2)
    {
        if ((flash[0] == 0xffffffff) && (flash[1] == 0xffffffff))
        {
            break;
        }
        
        data = flash[0];
        index = (flash[1] >>  0) & ((STM32WB_EEPROM_SIZE / 4) - 1);
        crc16 = (flash[1] >> 16) & 0xffff;
	
	    
        if (crc16 != stm32wb_eeprom_crc16((const uint8_t*)flash, 6))
        {
            continue;
        }
	
        stm32wb_eeprom_device.data[index] = data;
    }
    
    stm32wb_eeprom_device.offset = (uint32_t)flash - stm32wb_eeprom_device.flash;
    
    if (erase)
    {
        if (stm32wb_eeprom_device.flash == (uint32_t)&__eeprom_start__)
        {
            flash_s = (const uint32_t*)(stm32wb_eeprom_device.flash + stm32wb_eeprom_device.size);
        }
        else
        {
            flash_s = (const uint32_t*)(stm32wb_eeprom_device.flash - stm32wb_eeprom_device.size);
        }

        flash_e = (const uint32_t*)((uint32_t)flash_s + stm32wb_eeprom_device.size);

        for (flash = flash_s; flash < flash_e; flash += 2)
        {
            if ((flash[0] != 0xffffffff) || (flash[1] != 0xffffffff))
            {
                break;
            }
        }
        
        if (flash != flash_e)
        {
            stm32wb_eeprom_device.request.control = STM32WB_FLASH_CONTROL_ERASE;
            stm32wb_eeprom_device.request.address = (uint32_t)flash_s;
            stm32wb_eeprom_device.request.count = stm32wb_eeprom_device.size;
            stm32wb_eeprom_device.request.data = NULL;
	    
            if (!stm32wb_flash_request(&stm32wb_eeprom_device.request))
            {
                return false;
            }
	    
            while (stm32wb_eeprom_device.request.status == STM32WB_FLASH_STATUS_BUSY)
            {
                __WFE();
            }
            
            if (stm32wb_eeprom_device.request.status != STM32WB_FLASH_STATUS_SUCCESS)
            {
                return false;
            }
        }
    }

    stm32wb_eeprom_device.mask_1 = 0;
    memset((uint8_t*)&stm32wb_eeprom_device.mask_2[0], 0x00, (STM32WB_EEPROM_SIZE / 128));
    
    stm32wb_eeprom_device.request.callback = stm32wb_eeprom_callback;
    stm32wb_eeprom_device.request.context = NULL;

    return true;
}

void __stm32wb_eeprom_initialize(void)
{
    uint32_t size, offset, sequence_1, sequence_2;
    const uint32_t *flash_1, *flash_2;

    stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_NOT_READY;

    stm32wb_eeprom_device.request.next = NULL;
    stm32wb_eeprom_device.request.status = STM32WB_FLASH_STATUS_SUCCESS;
    
    size = ((uint32_t)&__eeprom_end__ - (uint32_t)&__eeprom_start__) / 2;
    flash_1 = (const uint32_t*)((uint32_t)&__eeprom_start__);
    flash_2 = (const uint32_t*)((uint32_t)&__eeprom_start__ + size);

    sequence_1 = 0;
    sequence_2 = 0;

    offset = (size - 16) / 4;

    if ((flash_1[offset+0] == ~flash_1[offset+1]) &&
	(flash_1[offset+2] == STM32WB_EEPROM_MAGIC_1) &&
	(flash_1[offset+3] == STM32WB_EEPROM_MAGIC_2))
    {
	sequence_1 = flash_1[offset+0];
    }

    if ((flash_2[offset+0] == ~flash_2[offset+1]) &&
	(flash_2[offset+2] == STM32WB_EEPROM_MAGIC_1) &&
	(flash_2[offset+3] == STM32WB_EEPROM_MAGIC_2))
    {
	sequence_2 = flash_2[offset+0];
    }

    if ((sequence_1 == 0) && (sequence_2 == 0))
    {
	stm32wb_eeprom_device.flash = (uint32_t)flash_1;
	stm32wb_eeprom_device.sequence = 1;
	stm32wb_eeprom_device.size = size;
	stm32wb_eeprom_device.offset = 0;

        if (!__stm32wb_eeprom_reset())
        {
            return;
        }
    }
    else
    {
	if (sequence_1 > sequence_2)
	{
	    stm32wb_eeprom_device.flash = (uint32_t)flash_1;
	    stm32wb_eeprom_device.size = size;
	    stm32wb_eeprom_device.sequence = sequence_1;
	}
	else
	{
	    stm32wb_eeprom_device.flash = (uint32_t)flash_2;
	    stm32wb_eeprom_device.size = size;
	    stm32wb_eeprom_device.sequence = sequence_2;
	}

        if (!__stm32wb_eeprom_flush(stm32wb_system_reset_cause() != STM32WB_SYSTEM_RESET_STANDBY))
        {
            return;
	}
    }

    stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_READY;
}

void stm32wb_eeprom_read(uint32_t address, uint8_t *data, uint32_t count)
{
    uint32_t offset;
    volatile uint8_t *eeprom;

    if (stm32wb_eeprom_device.state < STM32WB_EEPROM_STATE_READY)
    {
	memset(data, 0xff, count);
    }
    else
    {
	eeprom = (volatile uint8_t*)&stm32wb_eeprom_device.data[0];
	
	for (offset = 0; offset < count; offset++, address++)
	{
	    address &= (STM32WB_EEPROM_SIZE -1);
	    
	    data[offset] = eeprom[address];
	}
    }
}

void stm32wb_eeprom_write(uint32_t address, const uint8_t *data, uint32_t count)
{
    uint32_t offset, index;
    volatile uint8_t *eeprom;
    bool flush;
    
    if (stm32wb_eeprom_device.state >= STM32WB_EEPROM_STATE_READY)
    {
	flush = false;

	eeprom = (volatile uint8_t*)&stm32wb_eeprom_device.data[0];
	
	for (offset = 0; offset < count; offset++, address++)
	{
	    address &= (STM32WB_EEPROM_SIZE -1);
	    
	    if (eeprom[address] != data[offset])
	    {
		eeprom[address] = data[offset];
		
		index = address >> 2;
		
		armv7m_atomic_or(&stm32wb_eeprom_device.mask_2[index >> 5], (1 << (index & 31)));
		armv7m_atomic_or(&stm32wb_eeprom_device.mask_1, (1 << (index >> 5)));

		flush = true;
	    }
	}

	if (flush)
	{
	    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_EEPROM);
	}
    }
}

bool stm32wb_eeprom_is_ready(void)
{
    return ((stm32wb_eeprom_device.state < STM32WB_EEPROM_STATE_READY) || !stm32wb_eeprom_device.mask_1);
}

static void stm32wb_eeprom_process(void)
{
    uint32_t slot, index, data, crc16;
    const uint32_t *flash, *flash_s, *flash_e;
    uint8_t *program;

    if (stm32wb_eeprom_device.state > STM32WB_EEPROM_STATE_READY)
    {
	if (stm32wb_eeprom_device.request.status != STM32WB_FLASH_STATUS_BUSY)
	{
	    if (stm32wb_eeprom_device.request.status != STM32WB_FLASH_STATUS_SUCCESS)
	    {
		stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_NOT_READY;
	    }
	    else
	    {
		if (stm32wb_eeprom_device.state == STM32WB_EEPROM_STATE_PROGRAM)
		{
		    stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_READY;
		}

		if (stm32wb_eeprom_device.state == STM32WB_EEPROM_STATE_COPY)
		{
		    program = &stm32wb_eeprom_device.program[0];

		    program[ 0] = (uint8_t)(stm32wb_eeprom_device.sequence >>  0);
		    program[ 1] = (uint8_t)(stm32wb_eeprom_device.sequence >>  8);
		    program[ 2] = (uint8_t)(stm32wb_eeprom_device.sequence >> 16);
		    program[ 3] = (uint8_t)(stm32wb_eeprom_device.sequence >> 24);
		    program[ 4] = (uint8_t)(stm32wb_eeprom_device.sequence >>  0) ^ 0xff;
		    program[ 5] = (uint8_t)(stm32wb_eeprom_device.sequence >>  8) ^ 0xff;
		    program[ 6] = (uint8_t)(stm32wb_eeprom_device.sequence >> 16) ^ 0xff;
		    program[ 7] = (uint8_t)(stm32wb_eeprom_device.sequence >> 24) ^ 0xff;
		    program[ 8] = (uint8_t)(STM32WB_EEPROM_MAGIC_1 >>  0);
		    program[ 9] = (uint8_t)(STM32WB_EEPROM_MAGIC_1 >>  8);
		    program[10] = (uint8_t)(STM32WB_EEPROM_MAGIC_1 >> 16);
		    program[11] = (uint8_t)(STM32WB_EEPROM_MAGIC_1 >> 24);
		    program[12] = (uint8_t)(STM32WB_EEPROM_MAGIC_2 >>  0);
		    program[13] = (uint8_t)(STM32WB_EEPROM_MAGIC_2 >>  8);
		    program[14] = (uint8_t)(STM32WB_EEPROM_MAGIC_2 >> 16);
		    program[15] = (uint8_t)(STM32WB_EEPROM_MAGIC_2 >> 24);
		    
		    stm32wb_eeprom_device.request.control = STM32WB_FLASH_CONTROL_PROGRAM;
		    stm32wb_eeprom_device.request.address = stm32wb_eeprom_device.flash + stm32wb_eeprom_device.size - 16;
		    stm32wb_eeprom_device.request.count = 16;
		    stm32wb_eeprom_device.request.data = &stm32wb_eeprom_device.program[0];

		    stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_PROGRAM;
		    
		    if (!stm32wb_flash_request(&stm32wb_eeprom_device.request))
		    {
			stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_NOT_READY;
		    }
		}

		if (stm32wb_eeprom_device.state == STM32WB_EEPROM_STATE_ERASE)
		{
		    stm32wb_eeprom_device.request.control = STM32WB_FLASH_CONTROL_PROGRAM;
		    stm32wb_eeprom_device.request.address = stm32wb_eeprom_device.flash;
		    stm32wb_eeprom_device.request.count = STM32WB_EEPROM_SIZE;
		    stm32wb_eeprom_device.request.data = (const uint8_t*)&stm32wb_eeprom_device.data[0];
		    
		    stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_COPY;
		    
		    if (!stm32wb_flash_request(&stm32wb_eeprom_device.request))
		    {
			stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_NOT_READY;
		    }
		}
	    }
	}
    }
    
    if (stm32wb_eeprom_device.state == STM32WB_EEPROM_STATE_READY)
    {
	if (stm32wb_eeprom_device.mask_1)
	{
	    slot = __builtin_ctz(stm32wb_eeprom_device.mask_1);
	    index = __builtin_ctz(stm32wb_eeprom_device.mask_2[slot]);
	    
	    armv7m_atomic_and(&stm32wb_eeprom_device.mask_2[slot], ~(1 << index));
	    armv7m_atomic_andz(&stm32wb_eeprom_device.mask_1, ~(1 << slot), &stm32wb_eeprom_device.mask_2[slot]);

	    index = (slot * 32) + index;

	    if (stm32wb_eeprom_device.offset != (stm32wb_eeprom_device.size - 16))
	    {
		data = stm32wb_eeprom_device.data[index];

		program = &stm32wb_eeprom_device.program[0];

		program[0] = (data >> 0);
		program[1] = (data >> 8);
		program[2] = (data >> 16);
		program[3] = (data >> 24);
		program[4] = (index >> 0);
		program[5] = (index >> 8);
		    
		crc16 = stm32wb_eeprom_crc16(&program[0], 6);
		    
		program[6] = (crc16 >> 0);
		program[7] = (crc16 >> 8);

		stm32wb_eeprom_device.request.control = STM32WB_FLASH_CONTROL_PROGRAM;
		stm32wb_eeprom_device.request.address = stm32wb_eeprom_device.flash + stm32wb_eeprom_device.offset;
		stm32wb_eeprom_device.request.count = 8;
		stm32wb_eeprom_device.request.data = &stm32wb_eeprom_device.program[0]; 

		stm32wb_eeprom_device.offset += 8;
		stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_PROGRAM;
		    
		if (!stm32wb_flash_request(&stm32wb_eeprom_device.request))
		{
		    stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_NOT_READY;
		}
	    }
	    else
	    {
		if (stm32wb_eeprom_device.flash == (uint32_t)&__eeprom_start__)
		{
		    stm32wb_eeprom_device.flash += stm32wb_eeprom_device.size;
		}
		else
		{
		    stm32wb_eeprom_device.flash -= stm32wb_eeprom_device.size;
		}
		    
		stm32wb_eeprom_device.sequence += 1;
		stm32wb_eeprom_device.offset = STM32WB_EEPROM_SIZE;

		flash_s = (const uint32_t*)stm32wb_eeprom_device.flash;
		flash_e = (const uint32_t*)((uint32_t)flash_s + stm32wb_eeprom_device.size);

		for (flash = flash_s; flash < flash_e; flash += 2)
		{
		    if ((flash[0] != 0xffffffff) || (flash[1] != 0xffffffff))
		    {
			break;
		    }
		}

		if (flash != flash_e)
		{
		    stm32wb_eeprom_device.request.control = STM32WB_FLASH_CONTROL_ERASE;
		    stm32wb_eeprom_device.request.address = (uint32_t)stm32wb_eeprom_device.flash;
		    stm32wb_eeprom_device.request.count = stm32wb_eeprom_device.size;
		    stm32wb_eeprom_device.request.data = NULL;

		    stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_ERASE;
		    
		    if (!stm32wb_flash_request(&stm32wb_eeprom_device.request))
		    {
			stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_NOT_READY;
		    }
		}
		else
		{
		    stm32wb_eeprom_device.request.control = STM32WB_FLASH_CONTROL_PROGRAM;
		    stm32wb_eeprom_device.request.address = (uint32_t)stm32wb_eeprom_device.flash;
		    stm32wb_eeprom_device.request.count = STM32WB_EEPROM_SIZE;
		    stm32wb_eeprom_device.request.data = (const uint8_t*)&stm32wb_eeprom_device.data[0];

		    stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_COPY;
			
		    if (!stm32wb_flash_request(&stm32wb_eeprom_device.request))
		    {
			stm32wb_eeprom_device.state = STM32WB_EEPROM_STATE_NOT_READY;
		    }
		}
	    }
	}
    }

}

void EEPROM_SWIHandler(void)
{
    stm32wb_eeprom_process();
}
