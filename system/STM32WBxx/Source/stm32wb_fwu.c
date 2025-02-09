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

#include "stm32wb_boot.h"
#include "stm32wb_fwu.h"
#include "stm32wb_flash.h"
#include "stm32wb_system.h"
#include "stm32wb_ipcc.h"

extern uint32_t __application_base__, __application_limit__, __fwu_base__, __fwu_limit__, __fwu_status__;

extern const stm32wb_application_info_t stm32wb_application_info;

typedef struct _stm32wb_fwu_control_t {
    uint8_t                     control;
    uint8_t                     status;
    uint8_t                     component;
    uint32_t                    address;
    uint32_t                    size;
    uint32_t                    data[2];
    stm32wb_fwu_request_t       *request;
    stm32wb_flash_request_t     flash;
} stm32wb_fwu_control_t;

static stm32wb_fwu_control_t stm32wb_fwu_control;

static uint32_t stm32wb_fwu_check_application(void);
static uint32_t stm32wb_fwu_check_wireless(void);

uint32_t stm32wb_fwu_state(void)
{
    uint32_t fwu_status = (uint32_t)&__fwu_status__;
    
    if (((const uint32_t*)(fwu_status + STM32WB_FWU_STATUS_OFFSET_UPDATED))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_UPDATED;
    }

    if (((const uint32_t*)(fwu_status + STM32WB_FWU_STATUS_OFFSET_FAILED))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_FAILED;
    }

    if (((const uint32_t*)(fwu_status + STM32WB_FWU_STATUS_OFFSET_REJECTED))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_REJECTED;
    }

    if (((const uint32_t*)(fwu_status + STM32WB_FWU_STATUS_OFFSET_TRIAL))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_TRIAL;
    }

    if (((const uint32_t*)(fwu_status + STM32WB_FWU_STATUS_OFFSET_STAGED))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_STAGED;
    }

    if (((const uint32_t*)(fwu_status + STM32WB_FWU_STATUS_OFFSET_CANDIDATE))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_CANDIDATE;
    }

    if (((const uint32_t*)(fwu_status + STM32WB_FWU_STATUS_OFFSET_WRITING))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_WRITING;
    }

    if (((const uint32_t*)(fwu_status + STM32WB_FWU_STATUS_OFFSET_READY))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_READY;
    }

    return STM32WB_FWU_STATE_FAILED;
}

static uint32_t stm32wb_fwu_crc32(const uint8_t *data, uint32_t size, uint32_t crc32)
{
    uint8_t c;

    static const uint32_t stm32wb_fwu_crc32_lut[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
    };
    
    while (size--)
    {
        c = *data++;
        
        crc32 = (crc32 >> 4) ^ stm32wb_fwu_crc32_lut[(crc32 ^ c       ) & 15];
        crc32 = (crc32 >> 4) ^ stm32wb_fwu_crc32_lut[(crc32 ^ (c >> 4)) & 15];
    }
    
    return crc32;
}

static uint32_t stm32wb_fwu_check_application(void)
{
    const stm32wb_application_info_t *candidate_info, *application_info;
    uint32_t candidate_magic, candidate_base, candidate_size, candidate_length, candidate_crc32;
    uint32_t application_base, application_limit, fwu_base, signature_size;
    
    application_base = (uint32_t)&__application_base__;
    application_limit = (uint32_t)&__application_limit__;

    application_info = (const stm32wb_application_info_t*)(application_base + ((const stm32wb_application_vectors_t*)application_base)->size - sizeof(stm32wb_application_info_t));
    
    fwu_base = (uint32_t)&__fwu_base__;

    candidate_magic = ((const stm32wb_fwu_prefix_t*)fwu_base)->magic;
    candidate_base = ((const stm32wb_fwu_prefix_t*)fwu_base)->base;
    candidate_size = ((const stm32wb_fwu_prefix_t*)fwu_base)->size;
    candidate_length = ((const stm32wb_fwu_prefix_t*)fwu_base)->length;

    if ((candidate_magic & ~STM32WB_FWU_TYPE_MASK) != ((((const stm32wb_application_vectors_t*)application_base)->magic & ~(STM32WB_APPLICATION_TAG_MASK | STM32WB_APPLICATION_TYPE_MASK)) | STM32WB_FWU_TAG_FWU))
    {
        return STM32WB_FWU_STATUS_ERR_TARGET;
    }

    if (candidate_base != STM32WB_APPLICATION_BASE)
    {
        return STM32WB_FWU_STATUS_ERR_TARGET;
    }
    
    if (candidate_magic & STM32WB_FWU_OPTION_SIGNATURE_MASK)
    {
        if (candidate_magic & STM32WB_FWU_OPTION_SIGNATURE_RSA2048)
        {
            signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_RSA2048;
        }
        else
        {
            signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_ECC256;
        }
    }
    else
    {
        signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_NONE;
    }
    
    if ((candidate_size < (sizeof(stm32wb_application_vectors_t) + sizeof(stm32wb_application_info_t))) || ((candidate_size +  + signature_size) > (application_limit - application_base)) || (candidate_size & 15))
    {
        return STM32WB_FWU_STATUS_ERR_FILE;
    }

    if (candidate_length & 15)
    {
        return STM32WB_FWU_STATUS_ERR_FILE;
    }

    candidate_crc32 = stm32wb_fwu_crc32((const uint8_t*)fwu_base, offsetof(stm32wb_fwu_prefix_t, crc32), 0xffffffff);
    candidate_crc32 = stm32wb_fwu_crc32((const uint8_t*)(fwu_base + sizeof(stm32wb_fwu_prefix_t)), candidate_length, candidate_crc32);

    if (((const stm32wb_fwu_prefix_t*)fwu_base)->crc32 != candidate_crc32)
    {
        return STM32WB_FWU_STATUS_ERR_FILE;
    }

    candidate_info = (const stm32wb_application_info_t*)(fwu_base + sizeof(stm32wb_fwu_prefix_t) + candidate_length - (sizeof(stm32wb_application_info_t) + signature_size));

    if (memcmp(&application_info->uuid, &candidate_info->uuid, 16))
    {
        return STM32WB_FWU_STATUS_ERR_TARGET;
    }

    if (application_info->sequence > candidate_info->sequence)
    {
        return STM32WB_FWU_STATUS_ERR_FILE;
    }

    return STM32WB_FWU_STATUS_NO_ERROR;
}

static uint32_t stm32wb_fwu_check_wireless(void)
{
    const stm32wb_ipcc_fus_signature_t *signature;
    uint32_t magic;
    uint32_t fwu_base, fwu_limit;

    fwu_base = (uint32_t)&__fwu_base__;
    fwu_limit = (uint32_t)&__fwu_limit__;

    signature = (const stm32wb_ipcc_fus_signature_t*)(fwu_limit - sizeof(stm32wb_ipcc_fus_signature_t));
    
    while (fwu_base <= (uint32_t)signature)
    {
        if (signature->Magic == STM32WB_IPCC_FUS_MAGIC_STM_SIGNATURE)
        {
            magic = ((const stm32wb_ipcc_fus_image_t*)((const uint8_t*)signature - (signature->MemorySize & 0x000000ff) - sizeof(stm32wb_ipcc_fus_image_t)))->Magic;

            if ((magic == STM32WB_IPCC_FUS_MAGIC_WIRELESS_IMAGE) || (magic == STM32WB_IPCC_FUS_MAGIC_FUS_IMAGE))
            {
                return STM32WB_FWU_STATUS_NO_ERROR;
            }
        }

        signature = (const stm32wb_ipcc_fus_signature_t*)((const uint8_t*)signature - 4);
    }

    return STM32WB_FWU_STATUS_ERR_FILE;
}

static void stm32wb_fwu_process()
{
    stm32wb_fwu_request_t *request = stm32wb_fwu_control.request;
    stm32wb_fwu_done_callback_t callback;
    void *context;
    uint32_t address, control, offset, status;
    uint32_t fwu_base, fwu_limit, fwu_status;
    
    fwu_base = (uint32_t)&__fwu_base__;
    fwu_limit = (uint32_t)&__fwu_limit__;
    fwu_status = (uint32_t)&__fwu_status__;
    
    if (stm32wb_fwu_control.flash.status != STM32WB_FLASH_STATUS_SUCCESS)
    {
        if (stm32wb_fwu_control.flash.control == STM32WB_FLASH_CONTROL_ERASE)
        {
            status = STM32WB_FWU_STATUS_ERR_ERASE;
        }
        else
        {
            status = STM32WB_FWU_STATUS_ERR_PROGRAM;
        }

        stm32wb_fwu_control.control = 0;
    }
    else
    {
        status = stm32wb_fwu_control.status;

        if (status != STM32WB_FWU_STATUS_NO_ERROR)
        {
            stm32wb_fwu_control.control = 0;
        }
        else
        {
            if (stm32wb_fwu_control.control & (STM32WB_FWU_CONTROL_ACCEPT | STM32WB_FWU_CONTROL_REJECT | STM32WB_FWU_CONTROL_CANCEL))
            {
                stm32wb_fwu_control.data[0] = 0x00000000;
                stm32wb_fwu_control.data[1] = 0x00000000;
            
                if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_ACCEPT)
                {
                    control = STM32WB_FWU_CONTROL_ACCEPT;
                
                    offset = STM32WB_FWU_STATUS_OFFSET_UPDATED;
                }
                else if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_REJECT)
                {
                    control = STM32WB_FWU_CONTROL_REJECT;
                
                    offset = STM32WB_FWU_STATUS_OFFSET_REJECTED;
                }
                else
                {
                    control = STM32WB_FWU_CONTROL_CANCEL;
                
                    offset = STM32WB_FWU_STATUS_OFFSET_FAILED;
                }
            
                stm32wb_fwu_control.flash.control = STM32WB_FLASH_CONTROL_PROGRAM;
                stm32wb_fwu_control.flash.address = fwu_status + offset;
                stm32wb_fwu_control.flash.count = 8;
                stm32wb_fwu_control.flash.data = (const uint8_t*)&stm32wb_fwu_control.data[0];
                stm32wb_fwu_control.flash.callback = (stm32wb_flash_done_callback_t)stm32wb_fwu_process;
                stm32wb_fwu_control.flash.context = NULL;
            
                if (stm32wb_flash_request(&stm32wb_fwu_control.flash))
                {
                    stm32wb_fwu_control.control &= ~control;
                
                    return;
                }

                stm32wb_fwu_control.control = 0;

                status = STM32WB_FWU_STATUS_ERR_INTERNAL;
            }

            if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_CLEAN)
            {
                address = stm32wb_fwu_control.address;
            
                if (address != (fwu_status + FLASH_PAGE_SIZE))
                {
                    stm32wb_fwu_control.flash.control = STM32WB_FLASH_CONTROL_ERASE;
                    stm32wb_fwu_control.flash.address = address;
                    stm32wb_fwu_control.flash.count = FLASH_PAGE_SIZE;
                    stm32wb_fwu_control.flash.data = NULL;
                    stm32wb_fwu_control.flash.callback = (stm32wb_flash_done_callback_t)stm32wb_fwu_process;
                    stm32wb_fwu_control.flash.context = NULL;
                
                    if (stm32wb_flash_request(&stm32wb_fwu_control.flash))
                    {
                        address = address + FLASH_PAGE_SIZE;
                    
                        if (address == fwu_limit)
                        {
                            stm32wb_fwu_control.address = fwu_status;
                        }
                        else
                        {
                            stm32wb_fwu_control.address = address;
                        }
                    
                        return;
                    }
                
                    stm32wb_fwu_control.control = 0;
                
                    status = STM32WB_FWU_STATUS_ERR_INTERNAL;
                }
                else
                {
                    stm32wb_fwu_control.data[0] = 0x00000000;
                    stm32wb_fwu_control.data[1] = 0x00000000;
                
                    stm32wb_fwu_control.flash.control = STM32WB_FLASH_CONTROL_PROGRAM;
                    stm32wb_fwu_control.flash.address = fwu_status + STM32WB_FWU_STATUS_OFFSET_READY;
                    stm32wb_fwu_control.flash.count = 8;
                    stm32wb_fwu_control.flash.data = (const uint8_t*)&stm32wb_fwu_control.data[0];
                    stm32wb_fwu_control.flash.callback = (stm32wb_flash_done_callback_t)stm32wb_fwu_process;
                    stm32wb_fwu_control.flash.context = NULL;
                
                    if (stm32wb_flash_request(&stm32wb_fwu_control.flash))
                    {
                        stm32wb_fwu_control.control &= ~STM32WB_FWU_CONTROL_CLEAN;
                    
                        return;
                    }
                
                    stm32wb_fwu_control.control = 0;
                
                    status = STM32WB_FWU_STATUS_ERR_INTERNAL;
                }
            }
        
            if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_START)
            {
                offset = STM32WB_FWU_STATUS_OFFSET_WRITING;

                stm32wb_fwu_control.size = 0;
            
                stm32wb_fwu_control.data[0] = request->component;
                stm32wb_fwu_control.data[1] = 0x00000000;
            
                stm32wb_fwu_control.flash.control = STM32WB_FLASH_CONTROL_PROGRAM;
                stm32wb_fwu_control.flash.address = fwu_status + STM32WB_FWU_STATUS_OFFSET_WRITING;
                stm32wb_fwu_control.flash.count = 8;
                stm32wb_fwu_control.flash.data = (const uint8_t*)&stm32wb_fwu_control.data[0];
                stm32wb_fwu_control.flash.callback = (stm32wb_flash_done_callback_t)stm32wb_fwu_process;
                stm32wb_fwu_control.flash.context = NULL;
                
                if (stm32wb_flash_request(&stm32wb_fwu_control.flash))
                {
                    stm32wb_fwu_control.component = request->component;

                    stm32wb_fwu_control.control &= ~STM32WB_FWU_CONTROL_START;
                
                    return;
                }
            
                stm32wb_fwu_control.control = 0;
            
                status = STM32WB_FWU_STATUS_ERR_INTERNAL;
            }

            if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_WRITE)
            {
                if (stm32wb_fwu_control.component == STM32WB_FWU_COMPONENT_WIRELESS)
                {
                    address = fwu_base + STM32WB_BOOT_WIRELESS_PREFIX_SIZE + request->offset;
                }
                else
                {
                    address = fwu_base + request->offset;
                }

                if (((address + request->size) > fwu_limit) || (request->offset & 15) || (request->size & 15))
                {
                    stm32wb_fwu_control.control = 0;
                    
                    status = STM32WB_FWU_STATUS_ERR_ADDRESS;
                }
                else
                {
                    if (stm32wb_fwu_control.size < ((address + request->size) - fwu_base))
                    {
                        stm32wb_fwu_control.size = (address + request->size) - fwu_base;
                    }
                        
                    stm32wb_fwu_control.flash.control = STM32WB_FLASH_CONTROL_PROGRAM;
                    stm32wb_fwu_control.flash.address = address;
                    stm32wb_fwu_control.flash.count = request->size;
                    stm32wb_fwu_control.flash.data = request->data;
                    stm32wb_fwu_control.flash.callback = (stm32wb_flash_done_callback_t)stm32wb_fwu_process;
                    stm32wb_fwu_control.flash.context = NULL;
                    
                    if (stm32wb_flash_request(&stm32wb_fwu_control.flash))
                    {
                        stm32wb_fwu_control.control &= ~STM32WB_FWU_CONTROL_WRITE;
                        
                        return;
                    }
                    
                    stm32wb_fwu_control.control = 0;
                    
                    status = STM32WB_FWU_STATUS_ERR_INTERNAL;
                }
            }
        
            if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_FINISH)
            {
                stm32wb_fwu_control.data[0] = stm32wb_fwu_control.size;
                stm32wb_fwu_control.data[1] = 0;
            
                stm32wb_fwu_control.flash.control = STM32WB_FLASH_CONTROL_PROGRAM;
                stm32wb_fwu_control.flash.address = fwu_status + STM32WB_FWU_STATUS_OFFSET_CANDIDATE;
                stm32wb_fwu_control.flash.count = 8;
                stm32wb_fwu_control.flash.data = (const uint8_t*)&stm32wb_fwu_control.data[0];
                stm32wb_fwu_control.flash.callback = (stm32wb_flash_done_callback_t)stm32wb_fwu_process;
                stm32wb_fwu_control.flash.context = NULL;
            
                if (stm32wb_flash_request(&stm32wb_fwu_control.flash))
                {
                    stm32wb_fwu_control.control &= ~STM32WB_FWU_CONTROL_FINISH;
                
                    return;
                }
            
                stm32wb_fwu_control.control = 0;
            
                status = STM32WB_FWU_STATUS_ERR_INTERNAL;
            }

            if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_INSTALL)
            {
                stm32wb_fwu_control.data[0] = STM32WB_FWU_STATUS_ERR_INTERNAL;
                stm32wb_fwu_control.data[1] = 0x00000000;
            
                offset = STM32WB_FWU_STATUS_OFFSET_FAILED;
            
                if (((const uint32_t*)(fwu_status + STM32WB_FWU_STATUS_OFFSET_WRITING))[0] == STM32WB_FWU_COMPONENT_APPLICATION)
                {
                    stm32wb_fwu_control.status = stm32wb_fwu_check_application();
                  
                    if (stm32wb_fwu_control.status == STM32WB_FWU_STATUS_NO_ERROR)
                    {
                        offset = STM32WB_FWU_STATUS_OFFSET_STAGED;
                    
                        stm32wb_fwu_control.data[0] = request->mode;
                    }
                    else
                    {
                        stm32wb_fwu_control.data[0] = stm32wb_fwu_control.status;
                    }
                }
            
                if (((const uint32_t*)(fwu_status + STM32WB_FWU_STATUS_OFFSET_WRITING))[0] == STM32WB_FWU_COMPONENT_WIRELESS)
                {
                    stm32wb_fwu_control.status = stm32wb_fwu_check_wireless();

                    if (stm32wb_fwu_control.status == STM32WB_FWU_STATUS_NO_ERROR)
                    {
                        offset = STM32WB_FWU_STATUS_OFFSET_STAGED;

                        stm32wb_fwu_control.data[0] = 0;
                    }
                    else
                    {
                        stm32wb_fwu_control.data[0] = stm32wb_fwu_control.status;
                    }
                }
            
                stm32wb_fwu_control.flash.control = STM32WB_FLASH_CONTROL_PROGRAM;
                stm32wb_fwu_control.flash.address = fwu_status + offset;
                stm32wb_fwu_control.flash.count = 8;
                stm32wb_fwu_control.flash.data = (const uint8_t*)&stm32wb_fwu_control.data[0];
                stm32wb_fwu_control.flash.callback = (stm32wb_flash_done_callback_t)stm32wb_fwu_process;
                stm32wb_fwu_control.flash.context = NULL;
            
                if (stm32wb_flash_request(&stm32wb_fwu_control.flash))
                {
                    stm32wb_fwu_control.control &= ~STM32WB_FWU_CONTROL_INSTALL;
                
                    return;
                }
            
                stm32wb_fwu_control.control = 0;
            
                status = STM32WB_FWU_STATUS_ERR_INTERNAL;
            }
        }
    }

    if (!stm32wb_fwu_control.control)
    {
        callback = request->callback;
        context = request->context;

        stm32wb_fwu_control.request = NULL;
        
        request->status = status;

        if (callback)
        {
            (*callback)(context);
        }
    }
}


static bool __svc_stm32wb_fwu_query(stm32wb_fwu_info_t *info)
{
    uint32_t fwu_status = (uint32_t)&__fwu_status__;

    if (!info)
    {
        return false;
    }

    info->state = stm32wb_fwu_state();
    info->status = STM32WB_FWU_STATUS_NO_ERROR;

    if (((const uint32_t*)(fwu_status + STM32WB_FWU_STATUS_OFFSET_FAILED))[0] != 0xffffffff)
    {
        info->status = ((const uint8_t*)(fwu_status + STM32WB_FWU_STATUS_OFFSET_FAILED))[0];
    }
    
    return true;
}

static uint32_t __svc_stm32wb_fwu_request(stm32wb_fwu_request_t *request)
{
    uint8_t state;
    uint32_t fwu_base;
    
    fwu_base = (uint32_t)&__fwu_base__;
        
    if (stm32wb_fwu_control.request)
    {
        return false;
    }

    state = stm32wb_fwu_state();

    if (request->control & STM32WB_FWU_CONTROL_ACCEPT)
    {
        if (state != STM32WB_FWU_STATE_TRIAL)
        {
            return false;
        }

        state = STM32WB_FWU_STATE_UPDATED;
    }

    if (request->control & STM32WB_FWU_CONTROL_REJECT)
    {
        if (state != STM32WB_FWU_STATE_TRIAL)
        {
            return false;
        }

        state = STM32WB_FWU_STATE_REJECTED;
    }
    
    if (request->control & STM32WB_FWU_CONTROL_CANCEL)
    {
        if ((state != STM32WB_FWU_STATE_WRITING) && (state != STM32WB_FWU_STATE_CANDIDATE) && (state != STM32WB_FWU_STATE_STAGED))
        {
            return false;
        }

        state = STM32WB_FWU_STATE_FAILED;
    }
    
    if (request->control & STM32WB_FWU_CONTROL_CLEAN)
    {
        if ((state != STM32WB_FWU_STATE_FAILED) && (state != STM32WB_FWU_STATE_UPDATED))
        {
            return false;
        }
        
        state = STM32WB_FWU_STATE_READY;
        
        stm32wb_fwu_control.address = fwu_base;
    }

    if (request->control & STM32WB_FWU_CONTROL_START)
    {
        if (state != STM32WB_FWU_STATE_READY)
        {
            return false;
        }

        state = STM32WB_FWU_STATE_WRITING;
    }

    if (request->control & STM32WB_FWU_CONTROL_WRITE)
    {
        if (state != STM32WB_FWU_STATE_WRITING)
        {
            return false;
        }
    }

    if (request->control & STM32WB_FWU_CONTROL_FINISH)
    {
        if (state != STM32WB_FWU_STATE_WRITING)
        {
            return false;
        }

        state = STM32WB_FWU_STATE_CANDIDATE;
    }

    if (request->control & STM32WB_FWU_CONTROL_INSTALL)
    {
        if (state != STM32WB_FWU_STATE_CANDIDATE)
        {
            return false;
        }

        state = STM32WB_FWU_STATE_STAGED;
    }
    
    stm32wb_fwu_control.request = request;
    stm32wb_fwu_control.control = request->control;
    stm32wb_fwu_control.status = STM32WB_FWU_STATUS_NO_ERROR;
    
    stm32wb_fwu_control.flash.status = STM32WB_FLASH_STATUS_SUCCESS;
    
    request->status = STM32WB_FWU_STATUS_BUSY;

    stm32wb_fwu_process();

    return true;
}

bool stm32wb_fwu_query(stm32wb_fwu_info_t *info)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_1((uint32_t)&__svc_stm32wb_fwu_query, (uint32_t)info);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_fwu_query(info);
    }

    return false;
}

bool stm32wb_fwu_request(stm32wb_fwu_request_t *request)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_1((uint32_t)&__svc_stm32wb_fwu_request, (uint32_t)request);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_fwu_request(request);
    }

    return false;
}
