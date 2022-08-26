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
#include "stm32wb_hsem.h"
#include "stm32wb_ipcc.h"
#include "stm32wb_system.h"

#define STM32WB_FLASH_REQUEST_SENTINEL ((stm32wb_flash_request_t*)0xffffffff)

typedef struct _stm32wb_flash_device_t {
    bool                               busy;
    bool                               wireless;
    bool                               activity;
    bool                               suspended;
    stm32wb_flash_request_t            *head;
    stm32wb_flash_request_t            *tail;
    stm32wb_flash_request_t * volatile submit;
    stm32wb_flash_request_t            *request;
    k_work_t                           work;
    uint32_t                           address;
    uint32_t                           count;
    const uint8_t                      *data;
} stm32wb_flash_device_t;

static stm32wb_flash_device_t stm32wb_flash_device;

static void stm32wb_flash_routine(void);

void __stm32wb_flash_initialize(void)
{
    stm32wb_flash_device.busy = false;
    stm32wb_flash_device.wireless = false;
    stm32wb_flash_device.activity = false;
    stm32wb_flash_device.suspended = false;
    stm32wb_flash_device.head = STM32WB_FLASH_REQUEST_SENTINEL;
    stm32wb_flash_device.tail = STM32WB_FLASH_REQUEST_SENTINEL;
    stm32wb_flash_device.submit = STM32WB_FLASH_REQUEST_SENTINEL;
    stm32wb_flash_device.request = NULL;

    k_work_create(&stm32wb_flash_device.work, (k_work_routine_t)stm32wb_flash_routine, NULL);
}

static bool __svc_stm32wb_flash_suspend(void)
{
    if (!stm32wb_flash_device.suspended)
    {
	if (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_FLASH_CPU1, STM32WB_HSEM_PROCID_NONE))
	{
	    return false;
	}

	stm32wb_flash_device.suspended = true;
    }

    return true;
}

static bool __svc_stm32wb_flash_resume(void)
{
    if (stm32wb_flash_device.suspended)
    {
	stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_FLASH_CPU1, STM32WB_HSEM_PROCID_NONE);

	stm32wb_flash_device.suspended = false;

	armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_FLASH);
    }

    return true;
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

	armv7m_atomic_store((volatile uint32_t*)&request->next, (uint32_t)request_submit);
    }
    while ((stm32wb_flash_request_t*)armv7m_atomic_cas((volatile uint32_t*)&stm32wb_flash_device.submit, (uint32_t)request_submit, (uint32_t)request) != request_submit);
    
    if (request_submit == STM32WB_FLASH_REQUEST_SENTINEL)
    {
	armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_FLASH);
    }

    return true;
}

bool stm32wb_flash_suspend(void)
{
    if (armv7m_core_is_in_thread())
    {
	return armv7m_svcall_0((uint32_t)&__svc_stm32wb_flash_suspend);
    }

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
	return __svc_stm32wb_flash_suspend();
    }

    return false;
}

bool stm32wb_flash_resume(void)
{
    if (armv7m_core_is_in_thread())
    {
	return armv7m_svcall_0((uint32_t)&__svc_stm32wb_flash_resume);
    }

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
	return __svc_stm32wb_flash_resume();
    }

    return false;
}

static void stm32wb_flash_routine(void)
{
    stm32wb_flash_request_t *request;
    uint32_t flash_sr, flash_acr, page, address, count, data_0, data_1;
    const uint8_t *data;
    bool done, success;
    stm32wb_flash_done_callback_t callback;
    void *context;

    request = stm32wb_flash_device.request;
    
    if (!request)
    {
	return;
    }

    done = false;
    success = true;
	
    if (stm32wb_system_wireless())
    {
	if (!stm32wb_flash_device.wireless)
	{
	    uint8_t SetFlashActivityControl = STM32WB_IPCC_SYS_SET_FLASH_ACTIVITY_CONTROL_HSEM;
		
	    stm32wb_ipcc_sys_command(STM32WB_IPCC_SYS_OPCODE_SET_FLASH_ACTIVITY_CONTROL, &SetFlashActivityControl, sizeof(SetFlashActivityControl), NULL, 0);
	    
	    stm32wb_flash_device.wireless = true;
	}
    }

    if (!stm32wb_flash_device.busy)
    {
	stm32wb_flash_device.busy = true;

	stm32wb_system_hsi16_enable();
	    
	while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_FLASH, STM32WB_HSEM_PROCID_NONE))
	{
	}

	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xcdef89ab;

	FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);
    }

    if (request->control & STM32WB_FLASH_CONTROL_ERASE)
    {
	if (stm32wb_flash_device.wireless && !stm32wb_flash_device.activity)
	{
	    uint8_t FlashEraseActivitity = STM32WB_IPCC_SYS_ERASE_ACTIVITY_ON;

	    stm32wb_ipcc_sys_command(STM32WB_IPCC_SYS_OPCODE_FLASH_ERASE_ACTIVITY, &FlashEraseActivitity, sizeof(FlashEraseActivitity), NULL, 0);
	    
	    stm32wb_flash_device.activity = true;
	}

	if (!stm32wb_hsem_lock(STM32WB_HSEM_INDEX_FLASH_CPU2, STM32WB_HSEM_PROCID_NONE))
	{
	    return;
	}

	page = (stm32wb_flash_device.address - FLASH_BASE) / 4096;
	    
	FLASH->CR = FLASH_CR_PER | ((page << 3) & FLASH_CR_PNB) | FLASH_CR_STRT;
    
	do
	{
	    flash_sr = FLASH->SR;
	}
	while (flash_sr & FLASH_SR_BSY);
    
	FLASH->CR = 0;
	FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);

	stm32wb_flash_device.address += 4096;
	stm32wb_flash_device.count -= 4096;
	    
	done = (stm32wb_flash_device.count == 0);

	if (flash_sr & (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR))
	{
	    done = true;
	    success = false;
	}

	stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_FLASH_CPU2, STM32WB_HSEM_PROCID_NONE);
	    
	if (stm32wb_flash_device.wireless && stm32wb_flash_device.activity)
	{
	    uint8_t FlashEraseActivitity = STM32WB_IPCC_SYS_ERASE_ACTIVITY_OFF;
	  
	    stm32wb_ipcc_sys_command(STM32WB_IPCC_SYS_OPCODE_FLASH_ERASE_ACTIVITY, &FlashEraseActivitity, sizeof(FlashEraseActivitity), NULL, 0);
	    
	    stm32wb_flash_device.activity = false;
	}
    }

    if (request->control & STM32WB_FLASH_CONTROL_PROGRAM)
    {
	address = stm32wb_flash_device.address;
	count = stm32wb_flash_device.count;
	data = stm32wb_flash_device.data;

	do
	{
	    if (!stm32wb_hsem_lock(STM32WB_HSEM_INDEX_FLASH_CPU2, STM32WB_HSEM_PROCID_NONE))
	    {
		stm32wb_flash_device.address = address;
		stm32wb_flash_device.count = count;
		stm32wb_flash_device.data = (const uint8_t*)data;

		return;
	    }

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
		
	    address += 8;
	    count -= 8;
	    data += 8;

	    done = (count == 0);
		
	    if (flash_sr & (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR))
	    {
		done = true;
		success = false;
	    }

	    stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_FLASH_CPU2, STM32WB_HSEM_PROCID_NONE);
	}
	while (!done && (address & 0x0000007f));

	stm32wb_flash_device.address = address;
	stm32wb_flash_device.count = count;
	stm32wb_flash_device.data = (const uint8_t*)data;
    }
	
    if (stm32wb_flash_device.busy)
    {
	FLASH->CR = FLASH_CR_LOCK;

	if (request->control & STM32WB_FLASH_CONTROL_FLUSH)
	{
	    flash_acr = FLASH->ACR;
		
	    FLASH->ACR = (flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN));
	    FLASH->ACR = (flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN)) | (FLASH_ACR_ICRST | FLASH_ACR_DCRST);
	    FLASH->ACR = flash_acr;
	}

	stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_FLASH, STM32WB_HSEM_PROCID_NONE);

	stm32wb_system_hsi16_disable();

	stm32wb_flash_device.busy = false;
    }

    if (done)
    {
	stm32wb_flash_device.request = NULL;

	callback = request->callback;
	context = request->context;

	armv7m_atomic_store((volatile uint32_t*)&request->next, (uint32_t)NULL);

	request->status = success ? STM32WB_FLASH_STATUS_SUCCESS : STM32WB_FLASH_STATUS_FAIL;

	if (callback)
	{
	    (*callback)(context);
	}

	armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_FLASH);
    }
    else
    {
	k_work_submit(&stm32wb_flash_device.work);
    }
}

static void stm32wb_flash_process(void)
{
    stm32wb_flash_request_t *request, *request_submit, *request_next, *request_head, *request_tail;

    if (stm32wb_flash_device.submit != STM32WB_FLASH_REQUEST_SENTINEL)
    {
	request_submit = (stm32wb_flash_request_t*)armv7m_atomic_swap((volatile uint32_t*)&stm32wb_flash_device.submit, (uint32_t)STM32WB_FLASH_REQUEST_SENTINEL);

	for (request_head = STM32WB_FLASH_REQUEST_SENTINEL, request_tail = request_submit; request_submit != STM32WB_FLASH_REQUEST_SENTINEL; request_submit = request_next)
        {
            request_next = request_submit->next;

            armv7m_atomic_store((volatile uint32_t*)&request_submit->next, (uint32_t)request_head);

            request_head = request_submit;
        }
	
        if (stm32wb_flash_device.head == STM32WB_FLASH_REQUEST_SENTINEL)
        {
            stm32wb_flash_device.head = request_head;
        }
        else
        {
	    armv7m_atomic_store((volatile uint32_t*)&stm32wb_flash_device.tail->next, (uint32_t)request_head);
        }
	
        stm32wb_flash_device.tail = request_tail;
    }

    if (!stm32wb_flash_device.request)
    {
	if ((stm32wb_flash_device.head != STM32WB_FLASH_REQUEST_SENTINEL) && !stm32wb_flash_device.suspended)
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
	    stm32wb_flash_device.address = request->address;
	    stm32wb_flash_device.count = request->count;
	    stm32wb_flash_device.data = request->data;

	    k_work_submit(&stm32wb_flash_device.work);
	}
    }
}

void FLASH_SWIHandler(void)
{
    stm32wb_flash_process();
}

void FLASH_CPU2_HSEMHandler(void)
{
    k_work_submit(&stm32wb_flash_device.work);
}
