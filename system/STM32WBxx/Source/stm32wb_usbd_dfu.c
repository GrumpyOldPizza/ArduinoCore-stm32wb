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


#include "armv7m.h"
#include "stm32wb_system.h"
#include "stm32wb_usbd.h"
#include "stm32wb_usbd_dcd.h"
#include "stm32wb_usbd_dfu.h"

typedef struct _stm32wb_usbd_dfu_runtime_control_t {
    uint8_t                 interface;  
} stm32wb_usbd_dfu_runtime_control_t;

static stm32wb_usbd_dfu_runtime_control_t stm32wb_usbd_dfu_runtime_control;

static void stm32wb_usbd_dfu_runtime_configure(void *context, uint8_t interface)
{
    stm32wb_usbd_dfu_runtime_control.interface = interface;
}

static void stm32wb_usbd_dfu_runtime_start(void *context)
{
}

static void stm32wb_usbd_dfu_runtime_stop(void *context)
{
}

static void stm32wb_usbd_dfu_runtime_detach()
{
    stm32wb_usbd_disable();
    
    stm32wb_system_dfu();
}

static int stm32wb_usbd_dfu_runtime_request(void *context, int state, const stm32wb_usbd_request_t *request, uint8_t *data, const uint8_t **p_data_return, uint16_t *p_length_return, stm32wb_usbd_status_routine_t *p_status_routine_return)
{
    uint16_t interface;
    int status;

    status = STM32WB_USBD_REQUEST_STATUS_UNHANDLED;
    
    switch (request->bmRequestType & USB_REQ_RECIPIENT_MASK) {
    case USB_REQ_RECIPIENT_INTERFACE: {
	interface = request->wIndex;
	
	if (interface == stm32wb_usbd_dfu_runtime_control.interface)
	{
	    switch (request->bmRequestType & USB_REQ_TYPE_MASK) {
	    case USB_REQ_TYPE_CLASS: {
		switch (request->bRequest) {
		case USB_REQ_DFU_DETACH: {
                    *p_status_routine_return = stm32wb_usbd_dfu_runtime_detach;

                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
		    break;
		}

		default:
		    break;
		}
		break;
	    }

	    default:
		break;
	    }
	}
	break;
    }
	
    default:
	break;
    }

    return status;
}

static void stm32wb_usbd_dfu_runtime_suspend(void *context)
{
}

static void stm32wb_usbd_dfu_runtime_resume(void *context)
{
}

const stm32wb_usbd_class_t stm32wb_usbd_dfu_runtime_class = {
    stm32wb_usbd_dfu_runtime_configure,
    stm32wb_usbd_dfu_runtime_start,
    stm32wb_usbd_dfu_runtime_stop,
    stm32wb_usbd_dfu_runtime_request,
    stm32wb_usbd_dfu_runtime_suspend,
    stm32wb_usbd_dfu_runtime_resume,
    NULL,
    STM32WB_USBD_DFU_INTERFACE_COUNT,
    0,
};
