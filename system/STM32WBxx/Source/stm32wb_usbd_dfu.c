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
  
#define DFU_REQUEST_DETACH               0x00
#define DFU_REQUEST_DNLOAD               0x01
#define DFU_REQUEST_UPLOAD               0x02
#define DFU_REQUEST_GETSTATUS            0x03
#define DFU_REQUEST_CLRSTATUS            0x04
#define DFU_REQUEST_GETSTATE             0x05
#define DFU_REQUEST_ABORT                0x06

#define DFU_STATUS_OK                    0x00
#define DFU_STATUS_ERR_TARGET            0x01
#define DFU_STATUS_ERR_FILE              0x02
#define DFU_STATUS_ERR_WRITE             0x03
#define DFU_STATUS_ERR_ERASE             0x04
#define DFU_STATUS_ERR_CHECK_ERASED      0x05
#define DFU_STATUS_ERR_PROG              0x06
#define DFU_STATUS_ERR_VERIFY            0x07
#define DFU_STATUS_ERR_ADDRESS           0x08
#define DFU_STATUS_ERR_NOTDONE           0x09
#define DFU_STATUS_ERR_FIRMWARE          0x0a
#define DFU_STATUS_ERR_VENDOR            0x0b
#define DFU_STATUS_ERR_USBR              0x0c
#define DFU_STATUS_ERR_POR               0x0d
#define DFU_STATUS_ERR_UNKNOWN           0x0e
#define DFU_STATUS_ERR_STALLEDPKT        0x0f

#define DFU_STATE_APP_IDLE               0
#define DFU_STATE_APP_DETACH             1
#define DFU_STATE_IDLE                   2
#define DFU_STATE_DNLOAD_SYNC            3
#define DFU_STATE_DNBUSY                 4
#define DFU_STATE_DNLOAD_IDLE            5
#define DFU_STATE_MANIFEST_SYNC          6
#define DFU_STATE_MANIFEST               7
#define DFU_STATE_MANIFEST_WAIT_RESET    8
#define DFU_STATE_UPLOAD_IDLE            9
#define DFU_STATE_ERROR                  10

typedef struct _stm32wb_usbd_dfu_control_t {
    uint8_t                 interface;  
} stm32wb_usbd_dfu_control_t;

static stm32wb_usbd_dfu_control_t stm32wb_usbd_dfu_control;

static void stm32wb_usbd_dfu_configure(void *context, uint8_t interface)
{
    stm32wb_usbd_dfu_control.interface = interface;
}

static void stm32wb_usbd_dfu_start(void *context)
{
}

static void stm32wb_usbd_dfu_stop(void *context)
{
}

static void stm32wb_usbd_dfu_detach()
{
    stm32wb_usbd_disable();
    
    stm32wb_system_dfu();
}

static int stm32wb_usbd_dfu_request(void *context, int state, const stm32wb_usbd_request_t *request, uint8_t *data, const uint8_t **p_data_return, uint16_t *p_length_return, stm32wb_usbd_status_routine_t *p_status_routine_return)
{
    uint16_t interface;
    int status;

    status = STM32WB_USBD_REQUEST_STATUS_UNHANDLED;
    
    switch (request->bmRequestType & USB_REQ_RECIPIENT_MASK) {
    case USB_REQ_RECIPIENT_INTERFACE: {
	interface = request->wIndex;
	
	if (interface == stm32wb_usbd_dfu_control.interface)
	{
	    switch (request->bmRequestType & USB_REQ_TYPE_MASK) {
	    case USB_REQ_TYPE_CLASS: {
		switch (request->bRequest) {
		case DFU_REQUEST_DETACH: {
                    *p_status_routine_return = stm32wb_usbd_dfu_detach;

                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
		    break;
		}

		case DFU_REQUEST_GETSTATUS: {
                    *p_data_return = data;
                    *p_length_return = 6;

                    data[0] = DFU_STATUS_OK;
                    data[1] = 0;
                    data[2] = 0;
                    data[3] = 0;
                    data[4] = DFU_STATE_APP_IDLE;
                    data[5] = 0;

                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
		    break;
		}

		case DFU_REQUEST_GETSTATE: {
                    *p_data_return = data;
                    *p_length_return = 1;

                    data[0] = DFU_STATE_APP_IDLE;

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

static void stm32wb_usbd_dfu_suspend(void *context)
{
}

static void stm32wb_usbd_dfu_resume(void *context)
{
}

const stm32wb_usbd_class_t stm32wb_usbd_dfu_class = {
    stm32wb_usbd_dfu_configure,
    stm32wb_usbd_dfu_start,
    stm32wb_usbd_dfu_stop,
    stm32wb_usbd_dfu_request,
    stm32wb_usbd_dfu_suspend,
    stm32wb_usbd_dfu_resume,
    NULL,
    STM32WB_USBD_DFU_INTERFACE_COUNT,
    0,
};
