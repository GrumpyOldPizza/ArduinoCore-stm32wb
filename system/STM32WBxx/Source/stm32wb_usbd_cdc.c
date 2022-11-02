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

typedef struct _stm32wb_usbd_cdc_control_t {
    volatile uint8_t                        state;
    volatile uint8_t                        started;
    volatile uint8_t                        suspended;
    uint8_t                                 interface;    
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
    const uint8_t * volatile                tx_data;
    uint32_t                                tx_count;
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
} stm32wb_usbd_cdc_control_t;

static stm32wb_usbd_cdc_control_t stm32wb_usbd_cdc_control;

typedef struct _stm32wb_usbd_cdc_transmit_params_t {
    const uint8_t                           *tx_data;
    uint32_t                                tx_count;
    stm32wb_usbd_cdc_done_callback_t        callback;
    void                                    *context;
} stm32wb_usbd_cdc_transmit_params_t;

static void stm32wb_usbd_cdc_rx_callback(void *context, uint8_t ep_addr, uint16_t length)
{
    uint32_t rx_count, rx_size, rx_write, events;

    stm32wb_usbd_dcd_ep_receive(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR, &stm32wb_usbd_cdc_control.rx_fifo[stm32wb_usbd_cdc_control.rx_index ^ 1][0], STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE, stm32wb_usbd_cdc_rx_callback, NULL);

    if (stm32wb_usbd_cdc_control.state >= STM32WB_USBD_CDC_STATE_READY)
    {
	events = 0;

        while (length)
        {
	    rx_count = length;
	    rx_size = stm32wb_usbd_cdc_control.rx_size - stm32wb_usbd_cdc_control.rx_count;
	    rx_write = stm32wb_usbd_cdc_control.rx_write;

            if (rx_count > rx_size)
            {
                rx_count = rx_size;
            }

	    if (!rx_count)
	    {
		break;
	    }
	    
            rx_size = rx_count;
            
            if (rx_size > (stm32wb_usbd_cdc_control.rx_size - rx_write))
            {
                rx_size = stm32wb_usbd_cdc_control.rx_size - rx_write;
            }
            
            if (rx_size)
            {
                memcpy(&stm32wb_usbd_cdc_control.rx_data[rx_write], &stm32wb_usbd_cdc_control.rx_fifo[stm32wb_usbd_cdc_control.rx_index][0], rx_size);
                
                rx_count -= rx_size;
                rx_write += rx_size;
                
                if (rx_write == stm32wb_usbd_cdc_control.rx_size)
                {
                    rx_write = 0;
                }
            }
            
            if (rx_count)
            {
                memcpy(&stm32wb_usbd_cdc_control.rx_data[rx_write], &stm32wb_usbd_cdc_control.rx_fifo[stm32wb_usbd_cdc_control.rx_index][rx_size], rx_count);
                
                rx_write += rx_count;
                
                if (rx_write == stm32wb_usbd_cdc_control.rx_size)
                {
                    rx_write = 0;
                }
            }

	    armv7m_atomic_addh(&stm32wb_usbd_cdc_control.rx_count, (rx_size + rx_count));

	    stm32wb_usbd_cdc_control.rx_write = rx_write;
	    
	    length -= (rx_size + rx_count);

            if (stm32wb_usbd_cdc_control.rx_event)
            {
                stm32wb_usbd_cdc_control.rx_event = false;
                
                events = STM32WB_USBD_CDC_EVENT_RECEIVE;
            }
	}

	if (length)
	{
	    events |= STM32WB_USBD_CDC_EVENT_OVERRUN;
	}
	
	if (events)
	{
	    if (stm32wb_usbd_cdc_control.ev_callback)
	    {
		(*stm32wb_usbd_cdc_control.ev_callback)(stm32wb_usbd_cdc_control.ev_context, events);
	    }
        }
    }

    stm32wb_usbd_cdc_control.rx_index ^= 1;
}

static void stm32wb_usbd_cdc_tx_callback(void *context, uint8_t ep_addr)
{      
    stm32wb_usbd_cdc_done_callback_t callback;

    if (stm32wb_usbd_cdc_control.tx_busy == STM32WB_USBD_CDC_TX_ZLP)
    {
	if (stm32wb_usbd_cdc_control.tx_data)
	{
	    stm32wb_usbd_cdc_control.tx_busy = STM32WB_USBD_CDC_TX_BUSY;

	    stm32wb_usbd_cdc_control.tx_zlp = stm32wb_usbd_cdc_control.tx_count && !(stm32wb_usbd_cdc_control.tx_count & (STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE -1));
            
            stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, (uint8_t*)stm32wb_usbd_cdc_control.tx_data, stm32wb_usbd_cdc_control.tx_count, stm32wb_usbd_cdc_tx_callback, NULL);
	}
	else
	{
	    stm32wb_usbd_cdc_control.tx_busy = STM32WB_USBD_CDC_TX_IDLE;

#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
            stm32wb_usbd_dcd_lpm_unreference(STM32WB_USBD_DCD_EP_MASK(STM32WB_USBD_CDC_DATA_IN_EP_ADDR));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
	}
    }
    else
    {
	callback = stm32wb_usbd_cdc_control.tx_callback;
	context = stm32wb_usbd_cdc_control.tx_context;

	stm32wb_usbd_cdc_control.tx_data = NULL;
	    
	if (callback)
	{
	    (*callback)(context);
	}
	
	if (stm32wb_usbd_cdc_control.tx_data)
	{
	    stm32wb_usbd_cdc_control.tx_zlp = stm32wb_usbd_cdc_control.tx_count && !(stm32wb_usbd_cdc_control.tx_count & (STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE -1));

            stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, (uint8_t*)stm32wb_usbd_cdc_control.tx_data, stm32wb_usbd_cdc_control.tx_count, stm32wb_usbd_cdc_tx_callback, NULL);
	}
	else
	{
	    if (stm32wb_usbd_cdc_control.tx_zlp)
	    {
		stm32wb_usbd_cdc_control.tx_busy = STM32WB_USBD_CDC_TX_ZLP;

		stm32wb_usbd_cdc_control.tx_zlp = false;
		
                stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, NULL, 0, stm32wb_usbd_cdc_tx_callback, NULL);
	    }
	    else
	    {
		stm32wb_usbd_cdc_control.tx_busy = STM32WB_USBD_CDC_TX_IDLE;

#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
                stm32wb_usbd_dcd_lpm_unreference(STM32WB_USBD_DCD_EP_MASK(STM32WB_USBD_CDC_DATA_IN_EP_ADDR));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
	    }
	}
    }
}

static void stm32wb_usbd_cdc_tx_flush(void)
{
    stm32wb_usbd_cdc_done_callback_t callback;
    void *context;

    if (armv7m_atomic_swapb(&stm32wb_usbd_cdc_control.tx_busy, STM32WB_USBD_CDC_TX_IDLE) != STM32WB_USBD_CDC_TX_IDLE)
    {
        stm32wb_usbd_dcd_ep_flush(STM32WB_USBD_CDC_DATA_IN_EP_ADDR);

#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
        stm32wb_usbd_dcd_lpm_unreference(STM32WB_USBD_DCD_EP_MASK(STM32WB_USBD_CDC_DATA_IN_EP_ADDR));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
    }
    
    stm32wb_usbd_cdc_control.tx_zlp = false;

    if (stm32wb_usbd_cdc_control.tx_data)
    {
	callback = stm32wb_usbd_cdc_control.tx_callback;
	context = stm32wb_usbd_cdc_control.tx_context;
	
	stm32wb_usbd_cdc_control.tx_data = NULL;
	
	if (callback)
	{
	    (*callback)(context);
	}
    }
}    

static void stm32wb_usbd_cdc_int_callback(void *context, uint8_t ep_addr)
{
    uint8_t int_request;
    uint16_t serial_state;

    serial_state = armv7m_atomic_andh(&stm32wb_usbd_cdc_control.serial_state, ~(USB_CDC_SERIAL_STATE_DCD | USB_CDC_SERIAL_STATE_DSR | USB_CDC_SERIAL_STATE_BREAK | USB_CDC_SERIAL_STATE_RI));

    if ((serial_state & USB_CDC_SERIAL_STATE_DSR) && (stm32wb_usbd_cdc_control.state == STM32WB_USBD_CDC_STATE_READY))
    {
        stm32wb_usbd_cdc_control.state = STM32WB_USBD_CDC_STATE_CONNECTED;

        if (stm32wb_usbd_cdc_control.tx_data)
        {
            if (armv7m_atomic_casb(&stm32wb_usbd_cdc_control.tx_busy, STM32WB_USBD_CDC_TX_IDLE, STM32WB_USBD_CDC_TX_BUSY) == STM32WB_USBD_CDC_TX_IDLE)
            {
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
                stm32wb_usbd_dcd_lpm_reference(STM32WB_USBD_DCD_EP_MASK(STM32WB_USBD_CDC_DATA_IN_EP_ADDR));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */

                stm32wb_usbd_cdc_control.tx_zlp = stm32wb_usbd_cdc_control.tx_count && !(stm32wb_usbd_cdc_control.tx_count & (STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE -1));
                
                stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, (uint8_t*)stm32wb_usbd_cdc_control.tx_data, stm32wb_usbd_cdc_control.tx_count, stm32wb_usbd_cdc_tx_callback, NULL);
            }
        }
    }

    int_request = armv7m_atomic_swapb(&stm32wb_usbd_cdc_control.int_request, false);

    if (int_request)
    {
        serial_state = stm32wb_usbd_cdc_control.serial_state;
            
        stm32wb_usbd_cdc_control.notification.data[0] = serial_state & 255;
        stm32wb_usbd_cdc_control.notification.data[1] = serial_state >> 8;
        
        stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_CONTROL_EP_ADDR, (uint8_t*)&stm32wb_usbd_cdc_control.notification, 10, stm32wb_usbd_cdc_int_callback, NULL);
    }
    else
    {
        stm32wb_usbd_cdc_control.int_busy = STM32WB_USBD_CDC_INT_IDLE;
    }
}

static void stm32wb_usbd_cdc_int_request(void)
{
    uint16_t serial_state;

    stm32wb_usbd_cdc_control.int_request = true;

    if (!stm32wb_usbd_cdc_control.started)
    {
        return;
    }

    if (stm32wb_usbd_cdc_control.suspended)
    {
        stm32wb_usbd_wakeup();

        return;
    }
    
    if (armv7m_atomic_casb(&stm32wb_usbd_cdc_control.int_busy, STM32WB_USBD_CDC_INT_IDLE, STM32WB_USBD_CDC_INT_BUSY) != STM32WB_USBD_CDC_INT_IDLE)
    {
        return;
    }
    
    stm32wb_usbd_cdc_control.int_request = false;
            
    serial_state = stm32wb_usbd_cdc_control.serial_state;
            
    stm32wb_usbd_cdc_control.notification.data[0] = serial_state & 255;
    stm32wb_usbd_cdc_control.notification.data[1] = serial_state >> 8;
        
    stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_CONTROL_EP_ADDR, (uint8_t*)&stm32wb_usbd_cdc_control.notification, 10, stm32wb_usbd_cdc_int_callback, NULL);
}

static void stm32wb_usbd_cdc_int_flush(void)
{
    if (armv7m_atomic_swapb(&stm32wb_usbd_cdc_control.int_busy, STM32WB_USBD_CDC_INT_IDLE) != STM32WB_USBD_CDC_INT_IDLE)
    {
        stm32wb_usbd_dcd_ep_flush(STM32WB_USBD_CDC_CONTROL_EP_ADDR);

#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
        stm32wb_usbd_dcd_lpm_unreference(STM32WB_USBD_DCD_EP_MASK(STM32WB_USBD_CDC_CONTROL_EP_ADDR));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
    }
}

static void stm32wb_usbd_cdc_configure(void *context, uint8_t interface)
{
    stm32wb_usbd_cdc_control.started = false;
    stm32wb_usbd_cdc_control.suspended = false;
    stm32wb_usbd_cdc_control.interface = interface;

    stm32wb_usbd_cdc_control.notification.request.bmRequestType = USB_REQ_DIRECTION_DEVICE_TO_HOST | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
    stm32wb_usbd_cdc_control.notification.request.bRequest = USB_REQ_CDC_SERIAL_STATE;
    stm32wb_usbd_cdc_control.notification.request.wValue = 0;
    stm32wb_usbd_cdc_control.notification.request.wIndex = stm32wb_usbd_cdc_control.interface;
    stm32wb_usbd_cdc_control.notification.request.wLength = 2;

    stm32wb_usbd_cdc_control.serial_state = 0;
    stm32wb_usbd_cdc_control.break_state = 0;
    stm32wb_usbd_cdc_control.line_state = 0;
    stm32wb_usbd_cdc_control.line_coding.dwDTERate = STM32WB_USBD_CDC_LINE_CODING_DTE_RATE;
    stm32wb_usbd_cdc_control.line_coding.bCharFormat = STM32WB_USBD_CDC_LINE_CODING_CHAR_FORMAT;
    stm32wb_usbd_cdc_control.line_coding.bParityType = STM32WB_USBD_CDC_LINE_CODING_PARITY_TYPE;
    stm32wb_usbd_cdc_control.line_coding.bDataBits = STM32WB_USBD_CDC_LINE_CODING_DATA_BITS;
    
    stm32wb_usbd_cdc_control.tx_busy = STM32WB_USBD_CDC_TX_IDLE;

    stm32wb_usbd_dcd_ep_configure(STM32WB_USBD_CDC_CONTROL_EP_ADDR, STM32WB_USBD_DCD_EP_TYPE_INTERRUPT, STM32WB_USBD_CDC_CONTROL_MAX_PACKET_SIZE, 0, NULL);
    stm32wb_usbd_dcd_ep_configure(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, STM32WB_USBD_DCD_EP_TYPE_BULK, STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE, 0, NULL);
    stm32wb_usbd_dcd_ep_configure(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR, STM32WB_USBD_DCD_EP_TYPE_BULK, STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE, 0, NULL);

    stm32wb_usbd_cdc_control.state = STM32WB_USBD_CDC_STATE_INIT;
}

static void stm32wb_usbd_cdc_start(void *context)
{
    stm32wb_usbd_dcd_ep_enable(STM32WB_USBD_CDC_CONTROL_EP_ADDR);
    stm32wb_usbd_dcd_ep_enable(STM32WB_USBD_CDC_DATA_IN_EP_ADDR);
    stm32wb_usbd_dcd_ep_enable(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR);

    stm32wb_usbd_cdc_control.started = true;
    stm32wb_usbd_cdc_control.suspended = false;

    stm32wb_usbd_cdc_int_request();
    
    stm32wb_usbd_dcd_ep_receive(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR, &stm32wb_usbd_cdc_control.rx_fifo[0][0], STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE, stm32wb_usbd_cdc_rx_callback, NULL);
}

static void stm32wb_usbd_cdc_stop(void *context)
{
    stm32wb_usbd_cdc_control.started = false;
    stm32wb_usbd_cdc_control.suspended = false;
    
    stm32wb_usbd_cdc_int_flush();
    stm32wb_usbd_cdc_tx_flush();
    
    stm32wb_usbd_cdc_control.serial_state &= (USB_CDC_SERIAL_STATE_DCD | USB_CDC_SERIAL_STATE_DSR);
    stm32wb_usbd_cdc_control.break_state = 0;
    stm32wb_usbd_cdc_control.line_state = 0;
    stm32wb_usbd_cdc_control.line_coding.dwDTERate = STM32WB_USBD_CDC_LINE_CODING_DTE_RATE;
    stm32wb_usbd_cdc_control.line_coding.bCharFormat = STM32WB_USBD_CDC_LINE_CODING_CHAR_FORMAT;
    stm32wb_usbd_cdc_control.line_coding.bParityType = STM32WB_USBD_CDC_LINE_CODING_PARITY_TYPE;
    stm32wb_usbd_cdc_control.line_coding.bDataBits = STM32WB_USBD_CDC_LINE_CODING_DATA_BITS;
    
    stm32wb_usbd_dcd_ep_disable(STM32WB_USBD_CDC_CONTROL_EP_ADDR);
    stm32wb_usbd_dcd_ep_disable(STM32WB_USBD_CDC_DATA_IN_EP_ADDR);
    stm32wb_usbd_dcd_ep_disable(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR);
}

static int stm32wb_usbd_cdc_request(void *context, int state, const stm32wb_usbd_request_t *request, uint8_t *data, const uint8_t **p_data_return, uint16_t *p_length_return, stm32wb_usbd_status_routine_t *p_status_routine_return)
{
    uint16_t line_state;
    int status;

    status = STM32WB_USBD_REQUEST_STATUS_UNHANDLED;
    
    switch (request->bmRequestType & USB_REQ_RECIPIENT_MASK) {
    case USB_REQ_RECIPIENT_INTERFACE: {
	if (request->wIndex == stm32wb_usbd_cdc_control.interface)
	{
	    switch (request->bmRequestType & USB_REQ_TYPE_MASK) {
	    case USB_REQ_TYPE_CLASS: {
		switch (request->bRequest) {
		case USB_REQ_CDC_SET_LINE_CODING: {
		    stm32wb_usbd_cdc_control.line_coding.dwDTERate = (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
		    stm32wb_usbd_cdc_control.line_coding.bCharFormat = data[4];
		    stm32wb_usbd_cdc_control.line_coding.bParityType = data[5];
		    stm32wb_usbd_cdc_control.line_coding.bDataBits = data[6];

		    if (stm32wb_usbd_cdc_control.state >= STM32WB_USBD_CDC_STATE_READY)
		    {
			if (stm32wb_usbd_cdc_control.ev_callback)
			{
			    (*stm32wb_usbd_cdc_control.ev_callback)(stm32wb_usbd_cdc_control.ev_context, STM32WB_USBD_CDC_EVENT_LINE_CODING);
			}
		    }

                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
		    break;
		}
		    
		case USB_REQ_CDC_GET_LINE_CODING: {
		    *p_data_return = data;
		    *p_length_return = 7;

		    data[0] = (uint8_t)(stm32wb_usbd_cdc_control.line_coding.dwDTERate >> 0);
		    data[1] = (uint8_t)(stm32wb_usbd_cdc_control.line_coding.dwDTERate >> 8);
		    data[2] = (uint8_t)(stm32wb_usbd_cdc_control.line_coding.dwDTERate >> 16);
		    data[3] = (uint8_t)(stm32wb_usbd_cdc_control.line_coding.dwDTERate >> 24);
		    data[4] = stm32wb_usbd_cdc_control.line_coding.bCharFormat;
		    data[5] = stm32wb_usbd_cdc_control.line_coding.bParityType;
		    data[6] = stm32wb_usbd_cdc_control.line_coding.bDataBits;     

                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
		    break;
		}
		    
		case USB_REQ_CDC_SET_CONTROL_LINE_STATE: {
		    line_state = stm32wb_usbd_cdc_control.line_state;
        
		    stm32wb_usbd_cdc_control.line_state = request->wValue;

		    if (stm32wb_usbd_cdc_control.state >= STM32WB_USBD_CDC_STATE_READY)
		    {
			if ((line_state & USB_CDC_LINE_STATE_DTR) && !(stm32wb_usbd_cdc_control.line_state & USB_CDC_LINE_STATE_DTR))
			{
			    stm32wb_usbd_cdc_tx_flush();
			}
                        
			if (stm32wb_usbd_cdc_control.ev_callback)
			{
			    (*stm32wb_usbd_cdc_control.ev_callback)(stm32wb_usbd_cdc_control.ev_context, STM32WB_USBD_CDC_EVENT_LINE_STATE);
			}
		    }

                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
		    break;
		}

		case USB_REQ_CDC_SEND_BREAK: {
		    stm32wb_usbd_cdc_control.break_state = request->wValue;

		    if (stm32wb_usbd_cdc_control.state >= STM32WB_USBD_CDC_STATE_READY)
		    {
			if (stm32wb_usbd_cdc_control.ev_callback)
			{
			    (*stm32wb_usbd_cdc_control.ev_callback)(stm32wb_usbd_cdc_control.ev_context, STM32WB_USBD_CDC_EVENT_BREAK_STATE);
			}
		    }
		    
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

static void stm32wb_usbd_cdc_suspend(void *context)
{
    stm32wb_usbd_cdc_control.suspended = true;
}

static void stm32wb_usbd_cdc_resume(void *context)
{
    stm32wb_usbd_cdc_control.suspended = false;

#if 0    
    if (stm32wb_usbd_cdc_control.int_request)
    {
        stm32wb_usbd_cdc_int_request();
    }

    if (stm32wb_usbd_cdc_control.tx_data)
    {
        if (armv7m_atomic_casb(&stm32wb_usbd_cdc_control.tx_busy, STM32WB_USBD_CDC_TX_IDLE, STM32WB_USBD_CDC_TX_BUSY) == STM32WB_USBD_CDC_TX_IDLE)
        {
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
            stm32wb_usbd_dcd_lpm_reference(STM32WB_USBD_DCD_EP_MASK(STM32WB_USBD_CDC_DATA_IN_EP_ADDR));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */

            stm32wb_usbd_cdc_control.tx_zlp = stm32wb_usbd_cdc_control.tx_count && !(stm32wb_usbd_cdc_control.tx_count & (STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE -1));
            
            stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, (uint8_t*)stm32wb_usbd_cdc_control.tx_data, stm32wb_usbd_cdc_control.tx_count, stm32wb_usbd_cdc_tx_callback, NULL);
        }
    }
#endif    
}

const stm32wb_usbd_class_t stm32wb_usbd_cdc_class = {
    stm32wb_usbd_cdc_configure,
    stm32wb_usbd_cdc_start,
    stm32wb_usbd_cdc_stop,
    stm32wb_usbd_cdc_request,
    stm32wb_usbd_cdc_suspend,
    stm32wb_usbd_cdc_resume,
    NULL,
    STM32WB_USBD_CDC_INTERFACE_COUNT,
    (STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_CONTROL_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_DATA_IN_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR)),
};

/****************************************************************************************************************/    

static bool __svc_stm32wb_usbd_cdc_enable(uint8_t *rx_data, uint32_t rx_size, stm32wb_usbd_cdc_event_callback_t callback, void *context)
{
    if (stm32wb_usbd_cdc_control.state != STM32WB_USBD_CDC_STATE_INIT)
    {
	return false;
    }

    stm32wb_usbd_cdc_control.state = STM32WB_USBD_CDC_STATE_NOT_READY;

    stm32wb_usbd_cdc_control.ev_callback = callback;
    stm32wb_usbd_cdc_control.ev_context = context;

    stm32wb_usbd_cdc_control.rx_data = rx_data;
    stm32wb_usbd_cdc_control.rx_size = rx_size;
    stm32wb_usbd_cdc_control.rx_read = 0;
    stm32wb_usbd_cdc_control.rx_write = 0;
    stm32wb_usbd_cdc_control.rx_count = 0;
    stm32wb_usbd_cdc_control.rx_event = false;
    
    stm32wb_usbd_cdc_control.state = STM32WB_USBD_CDC_STATE_READY;

    armv7m_atomic_orh(&stm32wb_usbd_cdc_control.serial_state, (USB_CDC_SERIAL_STATE_DCD | USB_CDC_SERIAL_STATE_DSR));

    stm32wb_usbd_cdc_int_request();
    
    return true;
}

static bool __svc_stm32wb_usbd_cdc_disable(void)
{
    if (stm32wb_usbd_cdc_control.state < STM32WB_USBD_CDC_STATE_READY)
    {
	return false;
    }
    
    stm32wb_usbd_cdc_int_flush();
    stm32wb_usbd_cdc_tx_flush();

    stm32wb_usbd_cdc_control.state = STM32WB_USBD_CDC_STATE_INIT;

    armv7m_atomic_andh(&stm32wb_usbd_cdc_control.serial_state, ~(USB_CDC_SERIAL_STATE_DCD | USB_CDC_SERIAL_STATE_DSR));

    stm32wb_usbd_cdc_int_request();

    return true;
}

static bool __svc_stm32wb_usbd_cdc_serial_state(uint16_t serial_state)
{
    armv7m_atomic_orh(&stm32wb_usbd_cdc_control.serial_state, (serial_state & ~(USB_CDC_SERIAL_STATE_DCD | USB_CDC_SERIAL_STATE_DSR)));

    stm32wb_usbd_cdc_int_request();

    return true;
}

static bool __svc_stm32wb_usbd_cdc_transmit(const stm32wb_usbd_cdc_transmit_params_t *params)
{
    if (!stm32wb_usbd_cdc_control.started)
    {
        return false;
    }

    if (stm32wb_usbd_cdc_control.suspended)
    {
        if (!stm32wb_usbd_wakeup())
        {
            return false;
        }
    }

    if (!(stm32wb_usbd_cdc_control.line_state & USB_CDC_LINE_STATE_DTR))
    {
        return false;
    }
    
    if (armv7m_atomic_cas((volatile uint32_t*)&stm32wb_usbd_cdc_control.tx_data, (uint32_t)NULL, (uint32_t)params->tx_data) != (uint32_t)NULL)
    {
        return false;
    }

    stm32wb_usbd_cdc_control.tx_count = params->tx_count;
    stm32wb_usbd_cdc_control.tx_callback = params->callback;
    stm32wb_usbd_cdc_control.tx_context = params->context;

    if (stm32wb_usbd_cdc_control.state == STM32WB_USBD_CDC_STATE_CONNECTED)
    {
        if (armv7m_atomic_casb(&stm32wb_usbd_cdc_control.tx_busy, STM32WB_USBD_CDC_TX_IDLE, STM32WB_USBD_CDC_TX_BUSY) == STM32WB_USBD_CDC_TX_IDLE)
        {
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
            stm32wb_usbd_dcd_lpm_reference(STM32WB_USBD_DCD_EP_MASK(STM32WB_USBD_CDC_DATA_IN_EP_ADDR));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
            
            stm32wb_usbd_cdc_control.tx_zlp = stm32wb_usbd_cdc_control.tx_count && !(stm32wb_usbd_cdc_control.tx_count & (STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE -1));
            
            stm32wb_usbd_dcd_ep_transmit(STM32WB_USBD_CDC_DATA_IN_EP_ADDR, (uint8_t*)stm32wb_usbd_cdc_control.tx_data, stm32wb_usbd_cdc_control.tx_count, stm32wb_usbd_cdc_tx_callback, NULL);
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

    if (armv7m_core_is_in_svcall_or_pendsv())
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

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
	return __svc_stm32wb_usbd_cdc_disable();
    }
    
    return false;
}

void stm32wb_usbd_cdc_line_coding(stm32wb_usbd_cdc_line_coding_t *p_line_coding)
{
    p_line_coding->dwDTERate = stm32wb_usbd_cdc_control.line_coding.dwDTERate;
    p_line_coding->bCharFormat = stm32wb_usbd_cdc_control.line_coding.bCharFormat;
    p_line_coding->bParityType = stm32wb_usbd_cdc_control.line_coding.bParityType;
    p_line_coding->bDataBits = stm32wb_usbd_cdc_control.line_coding.bDataBits;
}

uint16_t stm32wb_usbd_cdc_line_state(void)
{
    return stm32wb_usbd_cdc_control.line_state;
}

uint16_t stm32wb_usbd_cdc_break_state(void)
{
    return stm32wb_usbd_cdc_control.break_state;
}

bool stm32wb_usbd_cdc_serial_state(uint16_t serial_state)
{
    if (stm32wb_usbd_cdc_control.state < STM32WB_USBD_CDC_STATE_READY)
    {
        return false;
    }

    if (armv7m_core_is_in_thread())
    {
	return armv7m_svcall_1((uint32_t)&__svc_stm32wb_usbd_cdc_serial_state, (uint32_t)serial_state);
    }
    else
    {
	return __svc_stm32wb_usbd_cdc_serial_state(serial_state);
    }
}

uint32_t stm32wb_usbd_cdc_count(void)
{
    if (stm32wb_usbd_cdc_control.state < STM32WB_USBD_CDC_STATE_READY)
    {
        return 0;
    }

    return stm32wb_usbd_cdc_control.rx_count;
}


int32_t stm32wb_usbd_cdc_peek(void)
{
    if (stm32wb_usbd_cdc_control.state < STM32WB_USBD_CDC_STATE_READY)
    {
        return -1;
    }

    if (stm32wb_usbd_cdc_control.rx_count == 0)
    {
        return -1;
    }

    return stm32wb_usbd_cdc_control.rx_data[stm32wb_usbd_cdc_control.rx_read];
}

uint32_t stm32wb_usbd_cdc_read(uint8_t *rx_data, uint32_t rx_size)
{
    uint32_t rx_total, rx_count, rx_read, rx_read_next;
    
    if (stm32wb_usbd_cdc_control.state < STM32WB_USBD_CDC_STATE_READY)
    {
        return 0;
    }

    if (stm32wb_usbd_cdc_control.rx_count == 0)
    {
        return 0;
    }

    rx_total = 0;

    while (rx_total < rx_size)
    {
	do
	{
	    rx_count = stm32wb_usbd_cdc_control.rx_count;
	    rx_read  = stm32wb_usbd_cdc_control.rx_read;
	    
	    if (rx_count > (stm32wb_usbd_cdc_control.rx_size - rx_read))
	    {
		rx_count = stm32wb_usbd_cdc_control.rx_size - rx_read;
	    }
	    
	    if (rx_count > (rx_size - rx_total))
	    {
		rx_count = rx_size - rx_total;
	    }

	    rx_read_next = rx_read + rx_count;

	    if (rx_read_next == stm32wb_usbd_cdc_control.rx_size)
	    {
		rx_read_next = 0;
	    }
        }
	while (armv7m_atomic_cash(&stm32wb_usbd_cdc_control.rx_read, rx_read, rx_read_next) != rx_read);

	if (!rx_count)
	{
	    break;
	}
	
	memcpy(&rx_data[rx_total], &stm32wb_usbd_cdc_control.rx_data[rx_read], rx_count);
	rx_total += rx_count;
	
	armv7m_atomic_subh(&stm32wb_usbd_cdc_control.rx_count, rx_count);
    }
    
    if (rx_total)
    {
	stm32wb_usbd_cdc_control.rx_event = true;
    }

    return rx_total;
}

bool stm32wb_usbd_cdc_transmit(const uint8_t *tx_data, uint32_t tx_count, stm32wb_usbd_cdc_done_callback_t callback, void *context)
{
    stm32wb_usbd_cdc_transmit_params_t params;

    if (stm32wb_usbd_cdc_control.state < STM32WB_USBD_CDC_STATE_READY)
    {
        return false;
    }
    
    params.tx_data = tx_data;
    params.tx_count = tx_count;
    params.callback = callback;
    params.context = context;

    if (armv7m_core_is_in_thread())
    {
	return armv7m_svcall_1((uint32_t)&__svc_stm32wb_usbd_cdc_transmit, (uint32_t)&params);
    }
    else
    {
	return __svc_stm32wb_usbd_cdc_transmit(&params);
    }
}

bool stm32wb_usbd_cdc_busy(void)
{
    return (stm32wb_usbd_cdc_control.tx_data != NULL);
}
