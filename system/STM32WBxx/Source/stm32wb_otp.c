/*
 * Copyright (c) 2022-2023 Thomas Roell.  All rights reserved.
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
#include "stm32wb_otp.h"
#include "stm32wb_flash.h"


/*  
 * Simple entry:  00xx xxxx <data>
 * Complex entry: 01xx xxxx <data> ... 11xx xxxx <data> ... 10xx xxxx <data>
 *
 * A ID of 0x3f is invalid.
 *
 * Each slot if <id-8> <data-56> (i.e. 64 bits). <data> is lsb byte order, 7 bytes per slot.
 */
#define STM32WB_OTP_TAG_ID_MASK                 0x3f
#define STM32WB_OTP_TAG_NOT_FIRST               0x80
#define STM32WB_OTP_TAG_NOT_LAST                0x40

#define STM32WB_OTP_BASE                        0x1fff7000
#define STM32WB_OTP_LIMIT                       0x1fff7400

bool stm32wb_otp_read(uint8_t id, uint8_t *data, uint32_t size, uint32_t *p_count_return)
{
    uint32_t address, slots, count, index, offset;

    if (id > STM32WB_OTP_ID_MAX)
    {
        return false;
    }
    
    for (address = STM32WB_OTP_LIMIT - 8, slots = 0; address >= STM32WB_OTP_BASE; address -= 8)
    {
        if ((((const uint8_t*)address)[7] & STM32WB_OTP_TAG_ID_MASK) == id)
        {
            if (((const uint8_t*)address)[7] & (STM32WB_OTP_TAG_NOT_FIRST | STM32WB_OTP_TAG_NOT_LAST))
            {
                if ((((const uint8_t*)address)[7] & (STM32WB_OTP_TAG_NOT_FIRST | STM32WB_OTP_TAG_NOT_LAST)) == STM32WB_OTP_TAG_NOT_FIRST)
                {
                    slots = 1;

                    for (address -= 8; address >= STM32WB_OTP_BASE; address -= 8)
                    {
                        if ((((const uint8_t*)address)[7] & STM32WB_OTP_TAG_ID_MASK) == id)
                        {
                            slots++;
                            
                            if ((((const uint8_t*)address)[7] & (STM32WB_OTP_TAG_NOT_FIRST | STM32WB_OTP_TAG_NOT_LAST)) == (STM32WB_OTP_TAG_NOT_FIRST | STM32WB_OTP_TAG_NOT_LAST))
                            {
                                continue;
                            }
                            
                            if ((((const uint8_t*)address)[7] & (STM32WB_OTP_TAG_NOT_FIRST | STM32WB_OTP_TAG_NOT_LAST)) != (STM32WB_OTP_TAG_NOT_LAST))
                            {
                                /* Tag out of sequence.
                                 */
                                slots = 0;
                            }
                        }
                        else
                        {
                            /* Switched to different ID.
                             */ 
                            slots = 0;
                        }
                            
                        break;
                    }

                    if (address < STM32WB_OTP_BASE)
                    {
                        slots = 0;
                    }
                    
                    if (slots)
                    {
                        break;
                    }
                }
            }
            else
            {
                slots = 1;

                break;
            }
        }
    }
    
    if (!slots)
    {
        return false;
    }

    count = size;

    if (count > (slots * 7))
    {
        count = (slots * 7);
    }

    for (index = 0, offset = 0; index < count; index++)
    {
        data[index] = ((const uint8_t*)address)[offset];

        offset++;

        if ((offset & 7) == 7)
        {
            offset++;
        }
    }

    if (p_count_return)
    {
        *p_count_return = count;
    }
    
    return true;
}

bool stm32wb_otp_program(uint8_t id, const uint8_t *data, uint32_t count)
{
    uint32_t address, slots, index, offset, limit;
    stm32wb_flash_request_t request;
    uint8_t program[8];
    
    if (!armv7m_core_is_in_thread())
    {
        return false;
    }

    if (id > STM32WB_OTP_ID_MAX)
    {
        return false;
    }

    for (address = STM32WB_OTP_LIMIT - 8, limit = STM32WB_OTP_LIMIT, slots = 0; address >= STM32WB_OTP_BASE; address -= 8)
    {
        if ((((const uint32_t*)address)[0] == 0xffffffff) && (((const uint32_t*)address)[1] == 0xffffffff))
        {
            limit = address;

            continue;
        }

        if ((((const uint8_t*)address)[7] & STM32WB_OTP_TAG_ID_MASK) == id)
        {
            if (((const uint8_t*)address)[7] & (STM32WB_OTP_TAG_NOT_FIRST | STM32WB_OTP_TAG_NOT_LAST))
            {
                if ((((const uint8_t*)address)[7] & (STM32WB_OTP_TAG_NOT_FIRST | STM32WB_OTP_TAG_NOT_LAST)) == STM32WB_OTP_TAG_NOT_FIRST)
                {
                    slots = 1;

                    for (address -= 8; address >= STM32WB_OTP_BASE; address -= 8)
                    {
                        if ((((const uint8_t*)address)[7] & STM32WB_OTP_TAG_ID_MASK) == id)
                        {
                            slots++;
                            
                            if ((((const uint8_t*)address)[7] & (STM32WB_OTP_TAG_NOT_FIRST | STM32WB_OTP_TAG_NOT_LAST)) == (STM32WB_OTP_TAG_NOT_FIRST | STM32WB_OTP_TAG_NOT_LAST))
                            {
                                continue;
                            }
                            
                            if ((((const uint8_t*)address)[7] & (STM32WB_OTP_TAG_NOT_FIRST | STM32WB_OTP_TAG_NOT_LAST)) != (STM32WB_OTP_TAG_NOT_LAST))
                            {
                                /* Tag out of sequence.
                                 */
                                slots = 0;
                            }
                        }
                        else
                        {
                            /* Switched to different ID.
                             */ 
                            slots = 0;
                        }
                            
                        break;
                    }

                    if (address < STM32WB_OTP_BASE)
                    {
                        slots = 0;
                    }
                    
                    if (slots)
                    {
                        break;
                    }
                }
            }
            else
            {
                slots = 1;

                break;
            }
        }
    }

    if (slots)
    {
        if (count <= (slots * 7))
        {
            for (index = 0, offset = 0; index < count; index++)
            {
                if (data[index] != ((const uint8_t*)address)[offset])
                {
                    break;
                }
                
                offset++;
                
                if ((offset & 7) == 7)
                {
                    offset++;
                }
            }
            
            if (index == count)
            {
                return true;
            }
        }
    }

    slots = ((count + 6) / 7);

    address = limit;
    
    if ((address + count * 8) > STM32WB_OTP_LIMIT)
    {
        return false;
    }

    memset(&program[0], 0, sizeof(program));

    for (index = 0, offset = 0; index < count; index++)
    {
        program[offset] = data[index];

        offset++;
            
        if (offset == 7)
        {
            program[7] = id | ((slots == 1) ? 0 : (((index >= 7) ? STM32WB_OTP_TAG_NOT_FIRST : 0) | ((index != (count -1)) ? STM32WB_OTP_TAG_NOT_LAST : 0)));

            request.next = NULL;
            request.control = STM32WB_FLASH_CONTROL_PROGRAM;
            request.address = address;
            request.count = 8;
            request.data = &program[0];
            request.callback = NULL;
            request.context = NULL;

            if (!stm32wb_flash_request(&request))
            {
                return false;
            }
            
            while (request.status == STM32WB_FLASH_STATUS_BUSY)
            {
                __WFE();
            }
            
            if (request.status != STM32WB_FLASH_STATUS_SUCCESS)
            {
                return false;
            }
            
            address += 8;
            
            memset(&program[0], 0, sizeof(program));

            offset = 0;
        }
    }
        
    if (offset & 7)
    {
        program[7] = id | ((slots == 1) ? 0 : STM32WB_OTP_TAG_NOT_FIRST);

        request.next = NULL;
        request.control = STM32WB_FLASH_CONTROL_PROGRAM;
        request.address = address;
        request.count = 8;
        request.data = &program[0];
        request.callback = NULL;
        request.context = NULL;
        
        if (!stm32wb_flash_request(&request))
        {
            return false;
        }
        
        while (request.status == STM32WB_FLASH_STATUS_BUSY)
        {
            __WFE();
        }
        
        if (request.status != STM32WB_FLASH_STATUS_SUCCESS)
        {
            return false;
        }
    }
    
    return true;
}
