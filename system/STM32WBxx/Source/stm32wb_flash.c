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

#include "armv7m.h"
#include "stm32wb_flash.h"
#include "stm32wb_hsem.h"
#include "stm32wb_ipcc.h"
#include "stm32wb_system.h"

#define STM32WB_FLASH_REQUEST_SENTINEL ((stm32wb_flash_request_t*)0x00000001)

#define STM32WB_FLASH_ACTIVITY_NONE     0
#define STM32WB_FLASH_ACTIVITY_WIRELESS 1
#define STM32WB_FLASH_ACTIVITY_ON       2
#define STM32WB_FLASH_ACTIVITY_OFF      3

typedef struct _stm32wb_flash_device_t {
    bool                               busy;
    bool                               suspended;
    bool                               success;
    uint8_t                            activity;
    stm32wb_flash_request_t            *head;
    stm32wb_flash_request_t            *tail;
    stm32wb_flash_request_t * volatile submit;
    stm32wb_flash_request_t            *request;
    uint32_t                           address;
    uint32_t                           count;
    const uint8_t                      *data;
    stm32wb_ipcc_sys_command_t         command;
} stm32wb_flash_device_t;

static stm32wb_flash_device_t stm32wb_flash_device;

static const uint8_t SetFlashActivityControl = STM32WB_IPCC_SYS_SET_FLASH_ACTIVITY_CONTROL_HSEM;
static const uint8_t FlashEraseActivitityOn = STM32WB_IPCC_SYS_ERASE_ACTIVITY_ON;
static const uint8_t FlashEraseActivitityOff = STM32WB_IPCC_SYS_ERASE_ACTIVITY_OFF;

static void stm32wb_flash_routine(void);
static void stm32wb_flash_activity_wireless(void);
static void stm32wb_flash_activity_on(void);
static void stm32wb_flash_activity_off(void);

void __stm32wb_flash_initialize(void)
{
    stm32wb_flash_device.busy = false;
    stm32wb_flash_device.suspended = false;
    stm32wb_flash_device.activity = STM32WB_FLASH_ACTIVITY_NONE;
    stm32wb_flash_device.head = STM32WB_FLASH_REQUEST_SENTINEL;
    stm32wb_flash_device.tail = STM32WB_FLASH_REQUEST_SENTINEL;
    stm32wb_flash_device.submit = STM32WB_FLASH_REQUEST_SENTINEL;
    stm32wb_flash_device.request = NULL;
}

static void __svc_stm32wb_flash_suspend(void)
{
    stm32wb_flash_device.suspended = true;

    stm32wb_hsem_lock(STM32WB_HSEM_INDEX_FLASH_CPU1, STM32WB_HSEM_PROCID_FLASH_CPU1);
}

static void __svc_stm32wb_flash_resume(void)
{
    if (stm32wb_flash_device.suspended)
    {
	armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_FLASH);

        stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_FLASH_CPU1, STM32WB_HSEM_PROCID_FLASH_CPU1);

        stm32wb_flash_device.suspended = false;
    }
}

bool stm32wb_flash_request(stm32wb_flash_request_t *request)
{
    stm32wb_flash_request_t *request_submit;

    if (armv7m_atomic_cas((volatile uint32_t*)&request->next, (uint32_t)NULL, (uint32_t)STM32WB_FLASH_REQUEST_SENTINEL) != (uint32_t)NULL)
    {
	return false;
    }

    request->status = STM32WB_FLASH_STATUS_BUSY;

    do
    {
	request_submit = stm32wb_flash_device.submit;

	request->next = request_submit;
    }
    while ((stm32wb_flash_request_t*)armv7m_atomic_cas((volatile uint32_t*)&stm32wb_flash_device.submit, (uint32_t)request_submit, (uint32_t)request) != request_submit);
    
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_FLASH);

    return true;
}

void stm32wb_flash_suspend(void)
{
    if (armv7m_core_is_in_thread())
    {
        armv7m_svcall_0((uint32_t)&__svc_stm32wb_flash_suspend);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
	__svc_stm32wb_flash_suspend();
    }
}

void stm32wb_flash_resume(void)
{
    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_0((uint32_t)&__svc_stm32wb_flash_resume);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
	__svc_stm32wb_flash_resume();
    }
}

bool stm32wb_flash_is_suspended(void)
{
    return stm32wb_hsem_is_locked(STM32WB_HSEM_INDEX_FLASH_CPU1, STM32WB_HSEM_PROCID_FLASH_CPU1);
}

static void stm32wb_flash_routine(void)
{
    stm32wb_flash_request_t *request, *request_submit, *request_next, *request_head, *request_tail;
    uint32_t flash_sr, flash_acr, page, address, count, data_0, data_1;
    const uint8_t *data;
    stm32wb_flash_done_callback_t callback;
    void *context;

    if (stm32wb_flash_device.suspended)
    {
        return;
    }
    
    if (stm32wb_flash_device.submit != STM32WB_FLASH_REQUEST_SENTINEL)
    {
	request_submit = (stm32wb_flash_request_t*)armv7m_atomic_swap((volatile uint32_t*)&stm32wb_flash_device.submit, (uint32_t)STM32WB_FLASH_REQUEST_SENTINEL);

	for (request_head = STM32WB_FLASH_REQUEST_SENTINEL, request_tail = request_submit; request_submit != STM32WB_FLASH_REQUEST_SENTINEL; request_submit = request_next)
        {
            request_next = request_submit->next;

            request_submit->next = request_head;

            request_head = request_submit;
        }
	
        if (stm32wb_flash_device.head == STM32WB_FLASH_REQUEST_SENTINEL)
        {
            stm32wb_flash_device.head = request_head;
        }
        else
        {
	    stm32wb_flash_device.tail->next = request_head;
        }
	
        stm32wb_flash_device.tail = request_tail;

        if (!stm32wb_flash_device.request)
        {
	    request = stm32wb_flash_device.head;
	    
	    if (stm32wb_flash_device.head == stm32wb_flash_device.tail)
	    {
		stm32wb_flash_device.head = STM32WB_FLASH_REQUEST_SENTINEL;
		stm32wb_flash_device.tail = STM32WB_FLASH_REQUEST_SENTINEL;
	    }
	    else
	    {
		stm32wb_flash_device.head = request->next;
	    }
	    
	    stm32wb_flash_device.request = request;
	    stm32wb_flash_device.success = true;
	    stm32wb_flash_device.address = request->address;
	    stm32wb_flash_device.count = request->count;
	    stm32wb_flash_device.data = request->data;
	}
    }

    if (stm32wb_flash_device.request)
    {
        request = stm32wb_flash_device.request;

        if (stm32wb_system_wireless() && (stm32wb_flash_device.activity == STM32WB_FLASH_ACTIVITY_NONE))
        {
            stm32wb_flash_device.command.next = NULL;
            stm32wb_flash_device.command.opcode = STM32WB_IPCC_SYS_OPCODE_SET_FLASH_ACTIVITY_CONTROL;
            stm32wb_flash_device.command.event = 0;
            stm32wb_flash_device.command.cparam = &SetFlashActivityControl;
            stm32wb_flash_device.command.clen = sizeof(SetFlashActivityControl);
            stm32wb_flash_device.command.rparam = NULL;
            stm32wb_flash_device.command.rsize = 0;
            stm32wb_flash_device.command.callback = (stm32wb_ipcc_sys_command_callback_t)stm32wb_flash_activity_wireless;
            stm32wb_flash_device.command.context = NULL;
            
            stm32wb_ipcc_sys_command(&stm32wb_flash_device.command);
	    
            return;
        }

        if (!stm32wb_flash_device.busy)
        {
            if (!stm32wb_hsem_lock(STM32WB_HSEM_INDEX_FLASH, STM32WB_HSEM_PROCID_FLASH))
            {
                return;
            }
            
            stm32wb_system_hsi16_enable();

            FLASH->KEYR = 0x45670123;
            FLASH->KEYR = 0xcdef89ab;
            
            FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);

            stm32wb_flash_device.busy = true;
        }

        if (stm32wb_flash_device.activity == STM32WB_FLASH_ACTIVITY_WIRELESS)
        {
            stm32wb_flash_device.command.next = NULL;
            stm32wb_flash_device.command.opcode = STM32WB_IPCC_SYS_OPCODE_FLASH_ERASE_ACTIVITY;
            stm32wb_flash_device.command.event = 0;
            stm32wb_flash_device.command.cparam = &FlashEraseActivitityOn;
            stm32wb_flash_device.command.clen = sizeof(FlashEraseActivitityOn);
            stm32wb_flash_device.command.rparam = NULL;
            stm32wb_flash_device.command.rsize = 0;
            stm32wb_flash_device.command.callback = (stm32wb_ipcc_sys_command_callback_t)stm32wb_flash_activity_on;
            stm32wb_flash_device.command.context = NULL;
            
            stm32wb_ipcc_sys_command(&stm32wb_flash_device.command);
            
            return;
        }

        if (stm32wb_flash_device.count)
        {
            if (!stm32wb_hsem_lock(STM32WB_HSEM_INDEX_FLASH_CPU2, STM32WB_HSEM_PROCID_FLASH_CPU2))
            {
                return;
            }

            if (request->control & STM32WB_FLASH_CONTROL_ERASE)
            {
                page = (stm32wb_flash_device.address - FLASH_BASE) / FLASH_PAGE_SIZE;

                __asm__ volatile("": : : "memory");
                
                FLASH->CR = FLASH_CR_PER | ((page << 3) & FLASH_CR_PNB) | FLASH_CR_STRT;
                __DMB();
                    
                __asm__ volatile("": : : "memory");
                
                do
                {
                    flash_sr = FLASH->SR;
                }
                while (flash_sr & FLASH_SR_BSY);
                
                FLASH->CR = 0;
                FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);
                
                stm32wb_flash_device.address += FLASH_PAGE_SIZE;
                stm32wb_flash_device.count -= FLASH_PAGE_SIZE;
                
                if (flash_sr & (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR))
                {
                    stm32wb_flash_device.success = false;
                    stm32wb_flash_device.count = 0;
                }
            }

            if (request->control & STM32WB_FLASH_CONTROL_PROGRAM)
            {
                if (stm32wb_flash_device.activity >= STM32WB_FLASH_ACTIVITY_WIRELESS)
                {
                    count = 8;
                }
                else
                {
                    count = stm32wb_flash_device.count;

                    if (count > 64)
                    {
                        count = 64;
                    }
                }

                address = stm32wb_flash_device.address;
                data = stm32wb_flash_device.data;
                
                stm32wb_flash_device.address += count;
                stm32wb_flash_device.count -= count;
                stm32wb_flash_device.data += count;

                do
                {
                    data_0 = (data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
                    data_1 = (data[4] << 0) | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);
                    
                    __asm__ volatile("": : : "memory");
                    
                    FLASH->CR = FLASH_CR_PG;
                    __DMB();
                    
                    ((volatile uint32_t *)address)[0] = data_0;
                    ((volatile uint32_t *)address)[1] = data_1;
                    __DMB();
                    
                    __asm__ volatile("": : : "memory");
                    
                    do
                    {
                        flash_sr = FLASH->SR;
                    }
                    while (flash_sr & FLASH_SR_BSY);
                    
                    FLASH->CR = 0;
                    FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);

                    if (flash_sr & (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR))
                    {
                        stm32wb_flash_device.success = false;
                        stm32wb_flash_device.count = 0;

                        break;
                    }

                    address += 8;
                    count -= 8;
                    data += 8;
                }
                while (count);
            }
        }

        if (stm32wb_flash_device.count)
        {
            armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_FLASH);
        }
        else
        {
            if (request->control & STM32WB_FLASH_CONTROL_FLUSH)
            {
                flash_acr = FLASH->ACR;
                
                FLASH->ACR = (flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN));
                FLASH->ACR = (flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN)) | (FLASH_ACR_ICRST | FLASH_ACR_DCRST);
                FLASH->ACR = flash_acr;
            }

            stm32wb_flash_device.request = NULL;

            callback = request->callback;
            context = request->context;

            request->next = NULL;

            request->status = stm32wb_flash_device.success ? STM32WB_FLASH_STATUS_SUCCESS : STM32WB_FLASH_STATUS_FAIL;

            if (callback)
            {
                (*callback)(context);
            }
            
            if (stm32wb_flash_device.head != STM32WB_FLASH_REQUEST_SENTINEL)
            {
                request = stm32wb_flash_device.head;
                
                if (stm32wb_flash_device.head == stm32wb_flash_device.tail)
                {
                    stm32wb_flash_device.head = STM32WB_FLASH_REQUEST_SENTINEL;
                    stm32wb_flash_device.tail = STM32WB_FLASH_REQUEST_SENTINEL;
                }
                else
                {
                    stm32wb_flash_device.head = request->next;
                }
                
                stm32wb_flash_device.request = request;
                stm32wb_flash_device.success = true;
                stm32wb_flash_device.address = request->address;
                stm32wb_flash_device.count = request->count;
                stm32wb_flash_device.data = request->data;
            }
        }
    }

    if (stm32wb_flash_device.request)
    {
        armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_FLASH);
    }
    else
    {
        if (!armv7m_pendsv_is_pending(ARMV7M_PENDSV_SWI_FLASH))
        {
            if (stm32wb_flash_device.activity == STM32WB_FLASH_ACTIVITY_ON)
            {
                stm32wb_flash_device.command.next = NULL;
                stm32wb_flash_device.command.opcode = STM32WB_IPCC_SYS_OPCODE_FLASH_ERASE_ACTIVITY;
                stm32wb_flash_device.command.event = 0;
                stm32wb_flash_device.command.cparam = &FlashEraseActivitityOff;
                stm32wb_flash_device.command.clen = sizeof(FlashEraseActivitityOff);
                stm32wb_flash_device.command.rparam = NULL;
                stm32wb_flash_device.command.rsize = 0;
                stm32wb_flash_device.command.callback = (stm32wb_ipcc_sys_command_callback_t)stm32wb_flash_activity_off;
                stm32wb_flash_device.command.context = NULL;
                
                stm32wb_ipcc_sys_command(&stm32wb_flash_device.command);
                
                return;
            }
            
            if (stm32wb_flash_device.activity == STM32WB_FLASH_ACTIVITY_OFF)
            {
                stm32wb_flash_device.activity = STM32WB_FLASH_ACTIVITY_WIRELESS;
            }
            
            if (stm32wb_flash_device.busy)
            {
                stm32wb_flash_device.busy = false;
                
                FLASH->CR = FLASH_CR_LOCK;
                
                stm32wb_system_hsi16_disable();
                
                stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_FLASH, STM32WB_HSEM_PROCID_FLASH);
            }
        }
    }
}

static void stm32wb_flash_activity_wireless(void)
{
    stm32wb_flash_device.activity = STM32WB_FLASH_ACTIVITY_WIRELESS;
    
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_FLASH);
}

static void stm32wb_flash_activity_on(void)
{
    stm32wb_flash_device.activity = STM32WB_FLASH_ACTIVITY_ON;
    
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_FLASH);
}

static void stm32wb_flash_activity_off(void)
{
    stm32wb_flash_device.activity = STM32WB_FLASH_ACTIVITY_OFF;
    
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_FLASH);
}

void FLASH_SWIHandler(void)
{
    stm32wb_flash_routine();
}

void FLASH_HSEMHandler(void)
{
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_FLASH);
}

void FLASH_CPU1_HSEMHandler(void)
{
    if (stm32wb_flash_device.suspended)
    {
        stm32wb_hsem_lock(STM32WB_HSEM_INDEX_FLASH_CPU1, STM32WB_HSEM_PROCID_FLASH_CPU1);
    }
}

void FLASH_CPU2_HSEMHandler(void)
{
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_FLASH);
}
