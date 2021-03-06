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
#include "stm32wb_system.h"
#include "stm32wb_usbd.h"
#include "stm32wb_usbd_cdc.h"
#include "stm32wb_usbd_dcd.h"

#define STM32WB_USBD_CDC_STATE_NONE       0
#define STM32WB_USBD_CDC_STATE_INIT       1
#define STM32WB_USBD_CDC_STATE_NOT_READY  2
#define STM32WB_USBD_CDC_STATE_READY      3
#define STM32WB_USBD_CDC_STATE_CONNECTED  4

#define STM32WB_USBD_CDC_INT_IDLE         0
#define STM32WB_USBD_CDC_INT_BUSY         1

#define STM32WB_USBD_CDC_TX_IDLE          0
#define STM32WB_USBD_CDC_TX_BUSY          1
#define STM32WB_USBD_CDC_TX_ZLP           2

typedef struct _stm32wb_usbd_cdc_class_t {
    volatile uint8_t                        state;
    volatile uint8_t                        suspended;
    uint16_t                                interface;    
    struct {
        stm32wb_usbd_request_t                  request;
        uint8_t                                 data[2];
    }                                       notification;
    volatile uint16_t                       serial_state;
    volatile uint16_t                       break_state;
    volatile uint16_t                       line_state;
    volatile stm32wb_usbd_cdc_line_coding_t line_coding;
    stm32wb_usbd_cdc_event_callback_t       ev_callback;
    void                                    *ev_context;
    volatile uint8_t                        int_busy;
    volatile uint8_t                        int_request;
    volatile uint8_t                        tx_busy;
    volatile uint8_t                        tx_zlp;
    const uint8_t *                         tx_data;
    uint32_t                                tx_count;
    volatile uint8_t * volatile             tx_status;
    stm32wb_usbd_cdc_done_callback_t        tx_callback;
    void                                    *tx_context;
    uint8_t                                 *rx_data;
    uint32_t                                rx_size;
    volatile uint16_t                       rx_write;
    volatile uint16_t                       rx_read;
    volatile uint16_t                       rx_count;
    volatile uint8_t                        rx_event;
    uint8_t                                 rx_index;
    uint8_t                                 rx_fifo[2][STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE];
} stm32wb_usbd_cdc_class_t;

static stm32wb_usbd_cdc_class_t stm32wb_usbd_cdc_class;

static void stm32wb_usbd_cdc_rx_callback(void *context, uint8_t ep_addr, uint16_t length)
{
    uint32_t rx_count, rx_size, rx_write, events;

    stm32wb_usbd_dcd_ep_receive(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR, &stm32wb_usbd_cdc_class.rx_fifo[stm32wb_usbd_cdc_class.rx_index ^ 1][0], STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE, stm32wb_usbd_cdc_rx_callback, NULL);

    if ((stm32wb_usbd_cdc_class.state >= STM32WB_USBD_CDC_STATE_READY) && (stm32wb_usbd_cdc_class.line_state & USB_CDC_LINE_STATE_DTR))
    {
	events = 0;

        while (length)
        {
	    rx_count = length;
	    rx_size = stm32wb_usbd_cdc_class.rx_size - stm32wb_usbd_cdc_class.rx_count;
	    rx_write = stm32wb_usbd_cdc_class.rx_write;

            if (rx_count > rx_size)
            {
                rx_count = rx_size;
            }

	    if (!rx_count)
	    {
		break;
	    }
	    
            rx_size = rx_count;
            
            if (rx_size > (stm32wb_usbd_cdc_class.rx_size - rx_write))
            {
                rx_size = stm32wb_usbd_cdc_class.rx_size - rx_write;
            }
            
            if (rx_size)
            {
                memcpy(&stm32wb_usbd_cdc_class.rx_data[rx_write], &stm32wb_usbd_cdc_class.rx_fifo[stm32wb_usbd_cdc_class.rx_index][0], rx_size);
                
                rx_count -= rx_size;
                rx_write += rx_size;
                
                if (rx_write == stm32wb_usbd_cdc_class.rx_size)
                {
                    rx_write = 0;
                }
            }
            
            if (rx_count)
            {
                memcpy(&stm32wb_usbd_cdc_class.rx_data[rx_write], &stm32wb_usbd_cdc_class.rx_fifo[stm32wb_usbd_cdc_class.rx_index][rx_size], rx_count);
                
                rx_write += rx_count;
                
                if (rx_write == stm32wb_usbd_cdc_class.rx_size)
                {
                    rx_write = 0;
                }
            }

	    armv7m_atomic_addh(&stm32wb_usbd_cdc_class.rx_count, (rx_size + rx_count));

	    stm32wb_usbd_cdc_class.rx_write = rx_write;
	    
	    length -= (rx_size + rx_count);

            if (stm32wb_usbd_cdc_class.rx_event)
            {
                stm32wb_usbd_cdc_class.rx_event = false;
                
                events = STM32WB_USBD_CDC_EVENT_RECEIVE;
            }
	}

	if (length)
	{
	    events |= STM32WB_USBD_CDC_EVENT_OVERRUN;
	}
	
	if (events)
	{
	    if (stm32wb_usbd_cdc_class.ev_callback)
	    {
		(*stm32wb_usbd_cdc_class.ev_callback)(stm32wb_usbd_cdc_class.ev_context, events);
	    }
        }
    }

    stm32wb_usbd_cdc_class.rx_index ^= 1;
}

static void stm32wb_usbd_cdc_tx_callback(void *context, uint8_t ep_addr)
{      
    stm32wb_usbd_cdc_done_callback_t callback;
    volatile uint8_t *p_status_return;

    if (stm32wb_usbd_cdc_class.tx_busy == STM32WB_USBD_CDC_TX_ZLP)
    {
	if (stm32wb_usbd_cdc_class.tx_status)
	{
	    armv7m_atomic_storeb(&stm32wb_usbd_cdc_class.tx_busy, STM32WB_USBD_CDC_TX_BUSY);

	    stm32wb_usbd_cdc_class.tx_zlp = stm32wb_usbd_cdc_class.tx_count && !(stm32wb_usbd_cdc_class.tx_count & (STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE -1));

            stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, (uint8_t*)stm32wb_usbd_cdc_class.tx_data, stm32wb_usbd_cdc_class.tx_count, stm32wb_usbd_cdc_tx_callback, NULL);
	}
	else
	{
	    armv7m_atomic_storeb(&stm32wb_usbd_cdc_class.tx_busy, STM32WB_USBD_CDC_TX_IDLE);
	}
    }
    else
    {
	p_status_return = stm32wb_usbd_cdc_class.tx_status;

	callback = stm32wb_usbd_cdc_class.tx_callback;
	context = stm32wb_usbd_cdc_class.tx_context;

	stm32wb_usbd_cdc_class.tx_status = NULL;
	    
	*p_status_return = STM32WB_USBD_CDC_STATUS_SUCCESS;
	    
	if (callback)
	{
	    (*callback)(context);
	}
	
	if (stm32wb_usbd_cdc_class.tx_status)
	{
	    stm32wb_usbd_cdc_class.tx_zlp = stm32wb_usbd_cdc_class.tx_count && !(stm32wb_usbd_cdc_class.tx_count & (STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE -1));

            stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, (uint8_t*)stm32wb_usbd_cdc_class.tx_data, stm32wb_usbd_cdc_class.tx_count, stm32wb_usbd_cdc_tx_callback, NULL);
	}
	else
	{
	    if (stm32wb_usbd_cdc_class.tx_zlp)
	    {
		armv7m_atomic_storeb(&stm32wb_usbd_cdc_class.tx_busy, STM32WB_USBD_CDC_TX_ZLP);

		stm32wb_usbd_cdc_class.tx_zlp = false;
		
		stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, NULL, 0, stm32wb_usbd_cdc_tx_callback, NULL);
	    }
	    else
	    {
		armv7m_atomic_storeb(&stm32wb_usbd_cdc_class.tx_busy, STM32WB_USBD_CDC_TX_IDLE);
	    }
	}
    }
}

static void stm32wb_usbd_cdc_tx_flush(void)
{
    stm32wb_usbd_cdc_done_callback_t callback;
    void *context;
    volatile uint8_t *p_status_return;

    stm32wb_usbd_dcd_ep_flush(STM32WB_USBD_CDC_DATA_IN_EP_ADDR);

    stm32wb_usbd_cdc_class.tx_zlp = false;

    armv7m_atomic_storeb(&stm32wb_usbd_cdc_class.tx_busy, STM32WB_USBD_CDC_TX_IDLE);
    
    if (stm32wb_usbd_cdc_class.tx_status)
    {
	p_status_return = stm32wb_usbd_cdc_class.tx_status;
	
	callback = stm32wb_usbd_cdc_class.tx_callback;
	context = stm32wb_usbd_cdc_class.tx_context;
	
	stm32wb_usbd_cdc_class.tx_status = NULL;
	
	*p_status_return = STM32WB_USBD_CDC_STATUS_FAILURE;
	
	if (callback)
	{
	    (*callback)(context);
	}
    }
}    

static void stm32wb_usbd_cdc_int_callback(void *context, uint8_t ep_addr)
{
    uint16_t serial_state;

    stm32wb_usbd_cdc_class.serial_state &= ~(USB_CDC_SERIAL_STATE_DCD | USB_CDC_SERIAL_STATE_DSR | USB_CDC_SERIAL_STATE_BREAK | USB_CDC_SERIAL_STATE_RI);
    
    if ((stm32wb_usbd_cdc_class.line_state & USB_CDC_LINE_STATE_DTR) && stm32wb_usbd_cdc_class.int_request)
    {
	stm32wb_usbd_cdc_class.int_request = false;

	serial_state = stm32wb_usbd_cdc_class.serial_state;
	
	stm32wb_usbd_cdc_class.notification.data[0] = serial_state & 255;
	stm32wb_usbd_cdc_class.notification.data[1] = serial_state >> 8;
            
	stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_CONTROL_EP_ADDR, (uint8_t*)&stm32wb_usbd_cdc_class.notification, 10, stm32wb_usbd_cdc_int_callback, NULL);
    }
    else
    {
	armv7m_atomic_storeb(&stm32wb_usbd_cdc_class.int_busy, STM32WB_USBD_CDC_INT_IDLE);
    }

    if (stm32wb_usbd_cdc_class.state == STM32WB_USBD_CDC_STATE_READY)
    {
	stm32wb_usbd_cdc_class.state = STM32WB_USBD_CDC_STATE_CONNECTED;

	if (stm32wb_usbd_cdc_class.tx_status)
	{
	    armv7m_atomic_storeb(&stm32wb_usbd_cdc_class.tx_busy, STM32WB_USBD_CDC_TX_BUSY);

	    stm32wb_usbd_cdc_class.tx_zlp = stm32wb_usbd_cdc_class.tx_count && !(stm32wb_usbd_cdc_class.tx_count & (STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE -1));
		
	    stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, (uint8_t*)stm32wb_usbd_cdc_class.tx_data, stm32wb_usbd_cdc_class.tx_count, stm32wb_usbd_cdc_tx_callback, NULL);
	}
    }
}

static void stm32wb_usbd_cdc_int_request(void)
{
    uint16_t serial_state;

    stm32wb_usbd_cdc_class.int_request = true;

    if (!stm32wb_usbd_cdc_class.suspended && (stm32wb_usbd_cdc_class.line_state & USB_CDC_LINE_STATE_DTR))
    {
	if (armv7m_atomic_casb(&stm32wb_usbd_cdc_class.int_busy, STM32WB_USBD_CDC_INT_IDLE, STM32WB_USBD_CDC_INT_BUSY) == STM32WB_USBD_CDC_INT_IDLE)
	{
	    stm32wb_usbd_cdc_class.int_request = false;

	    serial_state = stm32wb_usbd_cdc_class.serial_state;
	
	    stm32wb_usbd_cdc_class.notification.data[0] = serial_state & 255;
	    stm32wb_usbd_cdc_class.notification.data[1] = serial_state >> 8;
            
	    stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_CONTROL_EP_ADDR, (uint8_t*)&stm32wb_usbd_cdc_class.notification, 10, stm32wb_usbd_cdc_int_callback, NULL);
	}
    }
}

static void stm32wb_usbd_cdc_int_flush(void)
{
    stm32wb_usbd_dcd_ep_flush(STM32WB_USBD_CDC_CONTROL_EP_ADDR);

    armv7m_atomic_storeb(&stm32wb_usbd_cdc_class.int_busy, STM32WB_USBD_CDC_INT_IDLE);
}

static void stm32wb_usbd_cdc_configure(void *context, uint16_t if_base, uint16_t options)
{
    stm32wb_usbd_cdc_class.suspended = true;
    stm32wb_usbd_cdc_class.interface = if_base;

    stm32wb_usbd_cdc_class.notification.request.bmRequestType = USB_REQ_DIRECTION_DEVICE_TO_HOST | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
    stm32wb_usbd_cdc_class.notification.request.bRequest = USB_REQ_CDC_SERIAL_STATE;
    stm32wb_usbd_cdc_class.notification.request.wValue = 0;
    stm32wb_usbd_cdc_class.notification.request.wIndex = stm32wb_usbd_cdc_class.interface;
    stm32wb_usbd_cdc_class.notification.request.wLength = 2;

    stm32wb_usbd_cdc_class.serial_state = 0;
    stm32wb_usbd_cdc_class.break_state = 0;
    stm32wb_usbd_cdc_class.line_state = 0;
    stm32wb_usbd_cdc_class.line_coding.dwDTERate = STM32WB_USBD_CDC_LINE_CODING_DTE_RATE;
    stm32wb_usbd_cdc_class.line_coding.bCharFormat = STM32WB_USBD_CDC_LINE_CODING_CHAR_FORMAT;
    stm32wb_usbd_cdc_class.line_coding.bParityType = STM32WB_USBD_CDC_LINE_CODING_PARITY_TYPE;
    stm32wb_usbd_cdc_class.line_coding.bDataBits = STM32WB_USBD_CDC_LINE_CODING_DATA_BITS;
    
    armv7m_atomic_storeb(&stm32wb_usbd_cdc_class.int_busy, STM32WB_USBD_CDC_INT_IDLE);
    armv7m_atomic_storeb(&stm32wb_usbd_cdc_class.tx_busy, STM32WB_USBD_CDC_TX_IDLE);

    stm32wb_usbd_dcd_ep_configure(STM32WB_USBD_CDC_CONTROL_EP_ADDR, STM32WB_USBD_EP_TYPE_INTERRUPT, STM32WB_USBD_CDC_CONTROL_MAX_PACKET_SIZE);
    stm32wb_usbd_dcd_ep_configure(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, STM32WB_USBD_EP_TYPE_BULK, STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE);
    stm32wb_usbd_dcd_ep_configure(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR, STM32WB_USBD_EP_TYPE_BULK, STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE);

    stm32wb_usbd_cdc_class.state = STM32WB_USBD_CDC_STATE_INIT;
}

static void stm32wb_usbd_cdc_start(void *context)
{
    stm32wb_usbd_dcd_ep_enable(STM32WB_USBD_CDC_CONTROL_EP_ADDR);
    stm32wb_usbd_dcd_ep_enable(STM32WB_USBD_CDC_DATA_IN_EP_ADDR);
    stm32wb_usbd_dcd_ep_enable(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR);

    stm32wb_usbd_cdc_class.suspended = false;

    stm32wb_usbd_dcd_ep_receive(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR, &stm32wb_usbd_cdc_class.rx_fifo[0][0], STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE, stm32wb_usbd_cdc_rx_callback, NULL);
}

static void stm32wb_usbd_cdc_stop(void *context)
{
    stm32wb_usbd_cdc_class.suspended = true;

    if (stm32wb_usbd_cdc_class.state == STM32WB_USBD_CDC_STATE_CONNECTED)
    {
	stm32wb_usbd_cdc_class.state = STM32WB_USBD_CDC_STATE_READY;
    }
    
    stm32wb_usbd_cdc_int_flush();
    stm32wb_usbd_cdc_tx_flush();
    
    stm32wb_usbd_cdc_class.serial_state &= (USB_CDC_SERIAL_STATE_DCD | USB_CDC_SERIAL_STATE_DSR);
    stm32wb_usbd_cdc_class.break_state = 0;
    stm32wb_usbd_cdc_class.line_state = 0;
    stm32wb_usbd_cdc_class.line_coding.dwDTERate = STM32WB_USBD_CDC_LINE_CODING_DTE_RATE;
    stm32wb_usbd_cdc_class.line_coding.bCharFormat = STM32WB_USBD_CDC_LINE_CODING_CHAR_FORMAT;
    stm32wb_usbd_cdc_class.line_coding.bParityType = STM32WB_USBD_CDC_LINE_CODING_PARITY_TYPE;
    stm32wb_usbd_cdc_class.line_coding.bDataBits = STM32WB_USBD_CDC_LINE_CODING_DATA_BITS;

    stm32wb_usbd_dcd_ep_disable(STM32WB_USBD_CDC_CONTROL_EP_ADDR);
    stm32wb_usbd_dcd_ep_disable(STM32WB_USBD_CDC_DATA_IN_EP_ADDR);
    stm32wb_usbd_dcd_ep_disable(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR);
}

static bool stm32wb_usbd_cdc_request(void *context, int state, const stm32wb_usbd_request_t *request, uint8_t *data, const uint8_t **p_data_return, uint32_t *p_length_return)
{
    uint16_t line_state;
    bool success;

    success = false;
    
    switch (request->bmRequestType & USB_REQ_RECIPIENT_MASK) {
    case USB_REQ_RECIPIENT_INTERFACE: {
	if (request->wIndex == stm32wb_usbd_cdc_class.interface)
	{
	    switch (request->bmRequestType & USB_REQ_TYPE_MASK) {
	    case USB_REQ_TYPE_CLASS: {
		switch (request->bRequest) {
		case USB_REQ_CDC_SET_LINE_CODING: {
		    stm32wb_usbd_cdc_class.line_coding.dwDTERate = (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
		    stm32wb_usbd_cdc_class.line_coding.bCharFormat = data[4];
		    stm32wb_usbd_cdc_class.line_coding.bParityType = data[5];
		    stm32wb_usbd_cdc_class.line_coding.bDataBits = data[6];
		    
		    if (stm32wb_usbd_cdc_class.state >= STM32WB_USBD_CDC_STATE_READY)
		    {
			if (stm32wb_usbd_cdc_class.ev_callback)
			{
			    (*stm32wb_usbd_cdc_class.ev_callback)(stm32wb_usbd_cdc_class.ev_context, STM32WB_USBD_CDC_EVENT_LINE_CODING);
			}
		    }

		    success = true;
		    break;
		}
		    
		case USB_REQ_CDC_GET_LINE_CODING: {
		    *p_data_return = data;
		    *p_length_return = 7;

		    data[0] = (uint8_t)(stm32wb_usbd_cdc_class.line_coding.dwDTERate >> 0);
		    data[1] = (uint8_t)(stm32wb_usbd_cdc_class.line_coding.dwDTERate >> 8);
		    data[2] = (uint8_t)(stm32wb_usbd_cdc_class.line_coding.dwDTERate >> 16);
		    data[3] = (uint8_t)(stm32wb_usbd_cdc_class.line_coding.dwDTERate >> 24);
		    data[4] = stm32wb_usbd_cdc_class.line_coding.bCharFormat;
		    data[5] = stm32wb_usbd_cdc_class.line_coding.bParityType;
		    data[6] = stm32wb_usbd_cdc_class.line_coding.bDataBits;     
		    
		    success = true;
		    break;
		}
		    
		case USB_REQ_CDC_SET_CONTROL_LINE_STATE: {
		    line_state = stm32wb_usbd_cdc_class.line_state;
        
		    stm32wb_usbd_cdc_class.line_state = request->wValue;

		    if (stm32wb_usbd_cdc_class.state >= STM32WB_USBD_CDC_STATE_READY)
		    {
			if ((line_state & USB_CDC_LINE_STATE_DTR) && !(stm32wb_usbd_cdc_class.line_state & USB_CDC_LINE_STATE_DTR))
			{
			    if (stm32wb_usbd_cdc_class.state == STM32WB_USBD_CDC_STATE_CONNECTED)
			    {
				stm32wb_usbd_cdc_class.state = STM32WB_USBD_CDC_STATE_READY;
			    }
    
			    stm32wb_usbd_cdc_int_flush();
			    stm32wb_usbd_cdc_tx_flush();
			}
		    
			if (stm32wb_usbd_cdc_class.ev_callback)
			{
			    (*stm32wb_usbd_cdc_class.ev_callback)(stm32wb_usbd_cdc_class.ev_context, STM32WB_USBD_CDC_EVENT_LINE_STATE);
			}
		    
			if (!(line_state & USB_CDC_LINE_STATE_DTR) && (stm32wb_usbd_cdc_class.line_state & USB_CDC_LINE_STATE_DTR))
			{
			    stm32wb_usbd_cdc_int_request();
			}
		    }

		    success = true;
		    break;
		}

		case USB_REQ_CDC_SEND_BREAK: {
		    stm32wb_usbd_cdc_class.break_state = request->wValue;

		    if (stm32wb_usbd_cdc_class.state >= STM32WB_USBD_CDC_STATE_READY)
		    {
			if (stm32wb_usbd_cdc_class.ev_callback)
			{
			    (*stm32wb_usbd_cdc_class.ev_callback)(stm32wb_usbd_cdc_class.ev_context, STM32WB_USBD_CDC_EVENT_BREAK_STATE);
			}
		    }
		    
		    success = true;
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

    return success;
}

static void stm32wb_usbd_cdc_suspend(void *context)
{
    stm32wb_usbd_cdc_class.suspended = true;

    if (stm32wb_usbd_cdc_class.state == STM32WB_USBD_CDC_STATE_CONNECTED)
    {
	stm32wb_usbd_cdc_class.state = STM32WB_USBD_CDC_STATE_READY;
    }
    
    stm32wb_usbd_cdc_int_flush();
    stm32wb_usbd_cdc_tx_flush();
}

static void stm32wb_usbd_cdc_resume(void *context)
{
    stm32wb_usbd_cdc_class.suspended = false;

    stm32wb_usbd_cdc_int_request();
}

static void stm32wb_usbd_cdc_sof(void *context)
{
}

const stm32wb_usbd_class_interface_t stm32wb_usbd_cdc_interface = {
    stm32wb_usbd_cdc_configure,
    stm32wb_usbd_cdc_start,
    stm32wb_usbd_cdc_stop,
    stm32wb_usbd_cdc_request,
    stm32wb_usbd_cdc_suspend,
    stm32wb_usbd_cdc_resume,
    stm32wb_usbd_cdc_sof,
};

/****************************************************************************************************************/    

static bool __svc_stm32wb_usbd_cdc_enable(uint8_t *rx_data, uint32_t rx_size, stm32wb_usbd_cdc_event_callback_t callback, void *context)
{
    if (stm32wb_usbd_cdc_class.state != STM32WB_USBD_CDC_STATE_INIT)
    {
	return false;
    }

    stm32wb_usbd_cdc_class.state = STM32WB_USBD_CDC_STATE_NOT_READY;

    stm32wb_usbd_cdc_class.ev_callback = callback;
    stm32wb_usbd_cdc_class.ev_context = context;

    stm32wb_usbd_cdc_class.rx_data = rx_data;
    stm32wb_usbd_cdc_class.rx_size = rx_size;
    stm32wb_usbd_cdc_class.rx_read = 0;
    stm32wb_usbd_cdc_class.rx_write = 0;
    stm32wb_usbd_cdc_class.rx_count = 0;
    stm32wb_usbd_cdc_class.rx_event = false;
    
    stm32wb_usbd_cdc_class.state = STM32WB_USBD_CDC_STATE_READY;

    stm32wb_usbd_cdc_class.serial_state |= (USB_CDC_SERIAL_STATE_DCD | USB_CDC_SERIAL_STATE_DSR);

    stm32wb_usbd_cdc_int_request();
    
    return true;
}

static bool __svc_stm32wb_usbd_cdc_disable(void)
{
    if (stm32wb_usbd_cdc_class.state < STM32WB_USBD_CDC_STATE_READY)
    {
	return false;
    }
    
    stm32wb_usbd_cdc_class.state = STM32WB_USBD_CDC_STATE_INIT;

    stm32wb_usbd_cdc_tx_flush();

    stm32wb_usbd_cdc_class.serial_state &= ~(USB_CDC_SERIAL_STATE_DCD | USB_CDC_SERIAL_STATE_DSR);

    stm32wb_usbd_cdc_int_request();

    return true;
}

static void __svc_stm32wb_usbd_cdc_serial_state(uint16_t serial_state)
{
    serial_state &= ~(USB_CDC_SERIAL_STATE_DCD | USB_CDC_SERIAL_STATE_DSR);
    serial_state |= (stm32wb_usbd_cdc_class.serial_state & ~(USB_CDC_SERIAL_STATE_BREAK | USB_CDC_SERIAL_STATE_RI));

    if (stm32wb_usbd_cdc_class.serial_state != serial_state)
    {
	stm32wb_usbd_cdc_class.serial_state = serial_state;

	stm32wb_usbd_cdc_int_request();
    }
}

static bool __svc_stm32wb_usbd_cdc_transmit(void)
{
    volatile uint8_t *p_status_return;

    if (stm32wb_usbd_cdc_class.suspended || !(stm32wb_usbd_cdc_class.line_state & USB_CDC_LINE_STATE_DTR))
    {
	p_status_return = stm32wb_usbd_cdc_class.tx_status;

	stm32wb_usbd_cdc_class.tx_status = NULL;

	*p_status_return = STM32WB_USBD_CDC_STATUS_FAILURE;
	    
	return false;
    }

    if (stm32wb_usbd_cdc_class.state == STM32WB_USBD_CDC_STATE_CONNECTED)
    {
	if (armv7m_atomic_casb(&stm32wb_usbd_cdc_class.tx_busy, STM32WB_USBD_CDC_TX_IDLE, STM32WB_USBD_CDC_TX_BUSY) == STM32WB_USBD_CDC_TX_IDLE)
	{
	    stm32wb_usbd_cdc_class.tx_zlp = stm32wb_usbd_cdc_class.tx_count && !(stm32wb_usbd_cdc_class.tx_count & (STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE -1));
	    
	    stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, (uint8_t*)stm32wb_usbd_cdc_class.tx_data, stm32wb_usbd_cdc_class.tx_count, stm32wb_usbd_cdc_tx_callback, NULL);
	}
    }

    return true;
}

bool stm32wb_usbd_cdc_enable(uint8_t *rx_data, uint32_t rx_size, stm32wb_usbd_cdc_event_callback_t callback, void *context)
{
    if (armv7m_core_is_in_thread())
    {
	return armv7m_svcall_4((uint32_t)&__svc_stm32wb_usbd_cdc_enable, (uint32_t)rx_data, (uint32_t)rx_size, (uint32_t)callback, (uint32_t)context);
    }

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
	return __svc_stm32wb_usbd_cdc_enable(rx_data, rx_size, callback, context);
    }
    
    return false;
}

bool stm32wb_usbd_cdc_disable(void)
{
    if (armv7m_core_is_in_thread())
    {
	return armv7m_svcall_0((uint32_t)&__svc_stm32wb_usbd_cdc_disable);
    }

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
	return __svc_stm32wb_usbd_cdc_disable();
    }
    
    return false;
}

void stm32wb_usbd_cdc_line_coding(stm32wb_usbd_cdc_line_coding_t *p_line_coding)
{
    p_line_coding->dwDTERate = stm32wb_usbd_cdc_class.line_coding.dwDTERate;
    p_line_coding->bCharFormat = stm32wb_usbd_cdc_class.line_coding.bCharFormat;
    p_line_coding->bParityType = stm32wb_usbd_cdc_class.line_coding.bParityType;
    p_line_coding->bDataBits = stm32wb_usbd_cdc_class.line_coding.bDataBits;
}

uint16_t stm32wb_usbd_cdc_line_state(void)
{
    return stm32wb_usbd_cdc_class.line_state;
}

uint16_t stm32wb_usbd_cdc_break_state(void)
{
    return stm32wb_usbd_cdc_class.break_state;
}

bool stm32wb_usbd_cdc_serial_state(uint16_t serial_state)
{
    if (stm32wb_usbd_cdc_class.state < STM32WB_USBD_CDC_STATE_READY)
    {
        return false;
    }

    if (armv7m_core_is_in_thread())
    {
	armv7m_svcall_1((uint32_t)&__svc_stm32wb_usbd_cdc_serial_state, (uint32_t)serial_state);
    }
    else
    {
	__svc_stm32wb_usbd_cdc_serial_state(serial_state);
    }
    
    return true;
}

uint32_t stm32wb_usbd_cdc_count(void)
{
    if (stm32wb_usbd_cdc_class.state < STM32WB_USBD_CDC_STATE_READY)
    {
        return 0;
    }

    return stm32wb_usbd_cdc_class.rx_count;
}


int32_t stm32wb_usbd_cdc_peek(void)
{
    if (stm32wb_usbd_cdc_class.state < STM32WB_USBD_CDC_STATE_READY)
    {
        return -1;
    }

    if (stm32wb_usbd_cdc_class.rx_count == 0)
    {
        return -1;
    }

    return stm32wb_usbd_cdc_class.rx_data[stm32wb_usbd_cdc_class.rx_read];
}

uint32_t stm32wb_usbd_cdc_read(uint8_t *rx_data, uint32_t rx_size)
{
    uint32_t rx_total, rx_count, rx_read, rx_read_next;
    
    if (stm32wb_usbd_cdc_class.state < STM32WB_USBD_CDC_STATE_READY)
    {
        return 0;
    }

    if (stm32wb_usbd_cdc_class.rx_count == 0)
    {
        return 0;
    }

    rx_total = 0;

    while (rx_total < rx_size)
    {
	do
	{
	    rx_count = stm32wb_usbd_cdc_class.rx_count;
	    rx_read  = stm32wb_usbd_cdc_class.rx_read;
	    
	    if (rx_count > (stm32wb_usbd_cdc_class.rx_size - rx_read))
	    {
		rx_count = stm32wb_usbd_cdc_class.rx_size - rx_read;
	    }
	    
	    if (rx_count > (rx_size - rx_total))
	    {
		rx_count = rx_size - rx_total;
	    }

	    rx_read_next = rx_read + rx_count;

	    if (rx_read_next == stm32wb_usbd_cdc_class.rx_size)
	    {
		rx_read_next = 0;
	    }
        }
	while (armv7m_atomic_cash(&stm32wb_usbd_cdc_class.rx_read, rx_read, rx_read_next) != rx_read);

	if (!rx_count)
	{
	    break;
	}
	
	memcpy(&rx_data[rx_total], &stm32wb_usbd_cdc_class.rx_data[rx_read], rx_count);
	rx_total += rx_count;
	
	armv7m_atomic_subh(&stm32wb_usbd_cdc_class.rx_count, rx_count);
    }
    
    if (rx_total)
    {
	stm32wb_usbd_cdc_class.rx_event = true;
    }

    return rx_total;
}

bool stm32wb_usbd_cdc_transmit(const uint8_t *tx_data, uint32_t tx_count, volatile uint8_t *p_status_return, stm32wb_usbd_cdc_done_callback_t callback, void *context)
{
    if (stm32wb_usbd_cdc_class.state < STM32WB_USBD_CDC_STATE_READY)
    {
        return false;
    }

    if (armv7m_atomic_cas((volatile uint32_t*)&stm32wb_usbd_cdc_class.tx_status, (uint32_t)NULL, (uint32_t)p_status_return) != (uint32_t)NULL)
    {
        return false;
    }

    *p_status_return = STM32WB_USBD_CDC_STATUS_BUSY;
    
    stm32wb_usbd_cdc_class.tx_data = tx_data;
    stm32wb_usbd_cdc_class.tx_count = tx_count;
    stm32wb_usbd_cdc_class.tx_status = p_status_return;
    stm32wb_usbd_cdc_class.tx_callback = callback;
    stm32wb_usbd_cdc_class.tx_context = context;

    if (armv7m_core_is_in_thread())
    {
	return armv7m_svcall_0((uint32_t)&__svc_stm32wb_usbd_cdc_transmit);
    }
    else
    {
	return __svc_stm32wb_usbd_cdc_transmit();
    }
}
