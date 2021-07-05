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
#include "stm32wb_gpio.h"
#include "stm32wb_exti.h"
#include "stm32wb_system.h"
#include "stm32wb_lptim.h"
#include "stm32wb_usbd.h"
#include "stm32wb_usbd_dcd.h"

#define STM32WB_USBD_STATE_NONE         0
#define STM32WB_USBD_STATE_INIT         1
#define STM32WB_USBD_STATE_NOT_READY    2
#define STM32WB_USBD_STATE_READY        3
#define STM32WB_USBD_STATE_ATTACHED     4
#define STM32WB_USBD_STATE_DETECTED     5
#define STM32WB_USBD_STATE_POWERED      6
#define STM32WB_USBD_STATE_DEFAULT      7
#define STM32WB_USBD_STATE_ADDRESSED    8
#define STM32WB_USBD_STATE_CONFIGURED   9

#define STM32WB_USBD_EP0_STATE_STATUS   0
#define STM32WB_USBD_EP0_STATE_REQUEST  1
#define STM32WB_USBD_EP0_STATE_DATA_IN  2
#define STM32WB_USBD_EP0_STATE_DATA_OUT 3

typedef struct _stm32wb_usbd_control_t {
    volatile uint8_t              state;
    volatile uint8_t              connect;
    volatile uint8_t              vbus_status;
    volatile uint8_t              bcd_status;
    volatile uint32_t             events;
    const stm32wb_usbd_device_t   *device;
    const stm32wb_usbd_info_t     *info;
    uint16_t                      pin_vbus;
    uint8_t                       remote_wakeup;
    uint8_t                       if_count;
    uint32_t                      ep_mask;
    stm32wb_usbd_event_callback_t evt_callback;
    void                          *evt_context;
    uint8_t                       ep0_state;
    bool                          ep0_cancel;
    bool                          ep0_status;
    bool                          ep0_zlp;
    uint8_t                       ep0_setup[8];
    stm32wb_usbd_request_t        ep0_request;
    uint8_t                       *ep0_out_data;
    uint16_t                      ep0_out_size;
    uint16_t                      ep0_out_count;
    const uint8_t                 *ep0_in_data;
    uint32_t                      ep0_in_length;
    stm32wb_lptim_timeout_t       timeout;
} stm32wb_usbd_control_t;

static stm32wb_usbd_control_t stm32wb_usbd_control;

static void stm32wb_usbd_event_callback(void *context, uint32_t events);

static void stm32wb_usbd_connect(void)
{
    stm32wb_usbd_dcd_connect();

    stm32wb_usbd_control.state = STM32WB_USBD_STATE_POWERED;
}

static void stm32wb_usbd_disconnect(void)
{
    uint32_t function;
    const stm32wb_usbd_info_t *info;

    if (stm32wb_usbd_control.state == STM32WB_USBD_STATE_CONFIGURED)
    {
        info = stm32wb_usbd_control.info;
        
        for (function = 0; function < info->function_count; function++)
        {
            (*info->function_table[function].interface->stop)(info->function_table[function].context);
        }
    }
    
    stm32wb_usbd_dcd_disconnect();

    stm32wb_usbd_dcd_disable();
    
    stm32wb_usbd_control.state = STM32WB_USBD_STATE_DETECTED;
}

static void stm32wb_usbd_vbus_timeout(void)
{
    uint8_t vbus_status;
    
    if (stm32wb_usbd_control.pin_vbus == STM32WB_GPIO_PIN_NONE)
    {
        vbus_status = stm32wb_system_pvm1_sense();
    }
    else
    {
        vbus_status = !!stm32wb_gpio_pin_read(stm32wb_usbd_control.pin_vbus);
    }

    if (vbus_status && stm32wb_usbd_control.vbus_status)
    {
        stm32wb_usbd_control.state = STM32WB_USBD_STATE_ATTACHED;

        if (stm32wb_usbd_control.evt_callback)
        {
            (*stm32wb_usbd_control.evt_callback)(stm32wb_usbd_control.evt_context, STM32WB_USBD_EVENT_ATTACH);
        }

        stm32wb_usbd_dcd_enable(&stm32wb_usbd_control.ep0_setup[0], STM32WB_USBD_DCD_OPTION_DETECT, stm32wb_usbd_event_callback, NULL);
    }
}

static void stm32wb_usbd_vbus_changed(void)
{
    uint8_t vbus_status_previous;

    vbus_status_previous = stm32wb_usbd_control.vbus_status;
    
    if (stm32wb_usbd_control.pin_vbus == STM32WB_GPIO_PIN_NONE)
    {
        stm32wb_usbd_control.vbus_status = stm32wb_system_pvm1_sense();
    }
    else
    {
        stm32wb_usbd_control.vbus_status = !!stm32wb_gpio_pin_read(stm32wb_usbd_control.pin_vbus);
    }
    
    if (stm32wb_usbd_control.vbus_status)
    {
        if (!vbus_status_previous)
        {
            stm32wb_lptim_timeout_start(&stm32wb_usbd_control.timeout, stm32wb_lptim_timeout_millis_to_ticks(40), (stm32wb_lptim_timeout_callback_t)stm32wb_usbd_vbus_timeout); /* 40ms */
        }
    }
    else
    {
        stm32wb_lptim_timeout_stop(&stm32wb_usbd_control.timeout);

        if (vbus_status_previous)
        {
            stm32wb_usbd_control.bcd_status = STM32WB_USBD_BCD_STATUS_UNKNOWN;

            if (stm32wb_usbd_control.state >= STM32WB_USBD_STATE_POWERED)
            {
                stm32wb_usbd_disconnect();
            }
            
            if (stm32wb_usbd_control.state >= STM32WB_USBD_STATE_ATTACHED)
            {
                stm32wb_usbd_control.state = STM32WB_USBD_STATE_READY;
                
                if (stm32wb_usbd_control.evt_callback)
                {
                    (*stm32wb_usbd_control.evt_callback)(stm32wb_usbd_control.evt_context, STM32WB_USBD_EVENT_DETACH);
                }
            }
        }
    }
}

static void stm32wb_usbd_pvm1_changed(void)
{
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_PVM1);
}

static bool stm32wb_usbd_device(uint8_t *data, const uint8_t **p_data_return, uint32_t *p_length_return)
{
    const stm32wb_usbd_device_t *device = stm32wb_usbd_control.device;
    const stm32wb_usbd_info_t *info = stm32wb_usbd_control.info;

    *p_data_return = data;
    *p_length_return = 18;
    
    data[0]  = 18;                                   /* bLength */
    data[1]  = USB_DESCRIPTOR_TYPE_DEVICE;           /* bDescriptorType */ 
    data[2]  = info->bos ? 0x10 : 0x00;              /* bcdUSB */
    data[3]  = 0x02;
    data[4]  = 0xef;                                 /* bDeviceClass */
    data[5]  = 0x02;                                 /* bDeviceSubClass */
    data[6]  = 0x01;                                 /* bDeviceProtocol */
    data[7]  = STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE; /* bMaxPacketSize */
    data[8]  = device->vid & 255;                    /* idVendor */
    data[9]  = device->vid >> 8;
    data[10] = device->pid & 255;                    /* idProduct */
    data[11] = device->pid >> 8;
    data[12] = device->did & 255;                    /* bcdDevice */
    data[13] = device->did >> 8;
    data[14] = USB_STRING_INDEX_MANUFACTURER;        /* Index of manufacturer string */
    data[15] = USB_STRING_INDEX_PRODUCT;             /* Index of product string */
    data[16] = USB_STRING_INDEX_SERIAL;              /* Index of serial number string */
    data[17] = 1;                                    /* bNumConfigurations */
    
    return true;
}

static bool stm32wb_usbd_configuration(uint8_t *data, const uint8_t **p_data_return, uint32_t *p_length_return)
{
    const stm32wb_usbd_info_t *info;
    const uint8_t *configuration;
    
    info = stm32wb_usbd_control.info;
    configuration = info->configuration;

    if (!configuration)
    {
        return false;
    }
    
    *p_data_return = configuration;
    *p_length_return = (configuration[2] << 0) | (configuration[3] << 8);

    return true;
}

static bool stm32wb_usbd_bos(uint8_t *data, const uint8_t **p_data_return, uint32_t *p_length_return)
{
    const stm32wb_usbd_info_t *info;
    const uint8_t *bos;
    
    info = stm32wb_usbd_control.info;
    bos = info->bos;

    if (!bos)
    {
        return false;
    }
    
    *p_data_return = bos;
    *p_length_return = (bos[2] << 0) | (bos[3] << 8);

    return true;
}

static bool stm32wb_usbd_msos20(uint8_t *data, const uint8_t **p_data_return, uint32_t *p_length_return)
{
    const stm32wb_usbd_info_t *info;
    const uint8_t *msos20;
    
    info = stm32wb_usbd_control.info;
    msos20 = info->msos20;

    if (!msos20)
    {
        return false;
    }
    
    *p_data_return = msos20;
    *p_length_return = (msos20[8] << 0) | (msos20[9] << 8);

    return true;
}

static bool stm32wb_usbd_string(const char *string, uint8_t *data, const uint8_t **p_data_return, uint32_t *p_length_return)
{
    uint32_t count, length;
    uint8_t *data_e;

    if (!string)
    {
        return false;
    }
    
    count = strlen(string);

    if (count > ((STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE / 2) -1))
    {
        count = (STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE / 2) -1;
    }

    length = (2 + (count * 2));

    *p_data_return = data;
    *p_length_return = length;
    
    data_e = data + length;

    *data++ = length;
    *data++ = USB_DESCRIPTOR_TYPE_STRING;

    while (data != data_e)
    {
        *data++ = *string++;
        *data++ = 0;
    }
    
    return true;
}

static bool stm32wb_usbd_language(uint8_t *data, const uint8_t **p_data_return, uint32_t *p_length_return)
{
    static const uint8_t language[4] = {
        4,
        USB_DESCRIPTOR_TYPE_STRING,
        (uint8_t)((1033) & 255),
        (uint8_t)((1033) >> 8)
    };

    *p_data_return = (uint8_t*)&language[0];
    *p_length_return = language[0];
    
    return true;
}

static bool stm32wb_usbd_serial(uint8_t *data, const uint8_t **p_data_return, uint32_t *p_length_return)
{
    uint64_t serial;
    uint32_t serial_l, serial_h, length;
    int shift;

    static const uint8_t hex2ascii[16] = "0123456789ABCDEF";
    
    serial = stm32wb_system_serial();

    serial_l = (uint32_t)(serial >> 0);
    serial_h = (uint32_t)(serial >> 32);

    length = (2 + (12 * 2));

    *p_data_return = data;
    *p_length_return = length;
    
    *data++ = 26;
    *data++ = USB_DESCRIPTOR_TYPE_STRING;

    for (shift = (16-4); shift >= 0; shift -= 4)
    {
        *data++ = hex2ascii[(serial_h >> shift) & 15];
        *data++ = 0;
    }

    for (shift = (32-4); shift >= 0; shift -= 4)
    {
        *data++ = hex2ascii[(serial_l >> shift) & 15];
        *data++ = 0;
    }
    
    return true;
}

bool stm32wb_usbd_request(const stm32wb_usbd_request_t *request, uint8_t *data, const uint8_t **p_data_return, uint32_t *p_length_return)
{
    uint32_t function, interface, if_base, ep_addr, ep_mask, type, index;
    int state;
    bool success;
    const stm32wb_usbd_info_t *info;

    success = false;

    info = stm32wb_usbd_control.info;
    state = stm32wb_usbd_control.state - STM32WB_USBD_STATE_POWERED;
    
    switch (request->bmRequestType & USB_REQ_RECIPIENT_MASK) {
    case USB_REQ_RECIPIENT_DEVICE:
        for (function = 0; function < info->function_count; function++)
        {
            success = (*info->function_table[function].interface->request)(info->function_table[function].context, state, request, data, p_data_return, p_length_return);

            if (success)
            {
                break;
            }
        }

        if (!success)
        {
            switch (request->bmRequestType & USB_REQ_TYPE_MASK) {
            case USB_REQ_TYPE_STANDARD: {
                switch (request->bRequest) {
                case USB_REQ_CODE_GET_STATUS: {
                    if ((state == USB_STATE_ADDRESSED) || (state == USB_STATE_CONFIGURED))
                    {
                        *p_data_return = data;
                        *p_length_return = 2;

                        /* Return "SelfPowered" to avoid frequent auto-suspend operations.
                         */
                        data[0] = stm32wb_usbd_control.remote_wakeup ? 0x03 : 0x01;
                        data[1] = 0x00;
                        
                        success = true;
                    }
                    break;
                }
                    
                case USB_REQ_CODE_CLEAR_FEATURE: {
                    if (request->wValue == USB_FEATURE_DEVICE_REMOTE_WAKEUP)
                    {
                        if ((state == USB_STATE_ADDRESSED) || (state == USB_STATE_CONFIGURED))
                        {
                            stm32wb_usbd_control.remote_wakeup = false;
                            
                            success = true;
                        }
                    }
                    break;
                }
                    
                case USB_REQ_CODE_SET_FEATURE: {
                    if (request->wValue == USB_FEATURE_DEVICE_REMOTE_WAKEUP)
                    {
                        if ((state == USB_STATE_ADDRESSED) || (state == USB_STATE_CONFIGURED))
                        {
                            stm32wb_usbd_control.remote_wakeup = true;
                        
                            success = true;
                        }
                    }
                    break;
                }
                    
                case USB_REQ_CODE_SET_ADDRESS: {
                    if ((state == USB_STATE_DEFAULT) || (state == USB_STATE_ADDRESSED))
                    {
                        stm32wb_usbd_dcd_set_address(request->wValue & 0x7f);
                        
                        stm32wb_usbd_control.state = (request->wValue & 0x7f) ? STM32WB_USBD_STATE_ADDRESSED : STM32WB_USBD_STATE_DEFAULT;

                        success = true;
                    }
                    break;
                }
                    
                case USB_REQ_CODE_GET_DESCRIPTOR: {
                    type = request->wValue >> 8;
                    index = request->wValue & 255;

                    switch (type) {
                    case USB_DESCRIPTOR_TYPE_DEVICE: {
                        success = stm32wb_usbd_device(data, p_data_return, p_length_return);
                        break;
                    }
                        
                    case USB_DESCRIPTOR_TYPE_CONFIGURATION: {
                        success = stm32wb_usbd_configuration(data, p_data_return, p_length_return);
                        break;
                    }

                    case USB_DESCRIPTOR_TYPE_STRING: {
                        switch (index) {
                        case USB_STRING_INDEX_LANGID: {
                            success = stm32wb_usbd_language(data, p_data_return, p_length_return);
                            break;
                        }

                        case USB_STRING_INDEX_MANUFACTURER: {
                            success = stm32wb_usbd_string(stm32wb_usbd_control.device->manufacturer, data, p_data_return, p_length_return);
                            break;
                        }

                        case USB_STRING_INDEX_PRODUCT: {
                            success = stm32wb_usbd_string(stm32wb_usbd_control.device->product, data, p_data_return, p_length_return);
                            break;
                        }

                        case USB_STRING_INDEX_SERIAL: {
                            success = stm32wb_usbd_serial(data, p_data_return, p_length_return);
                            break;
                        }

                        default: {
                            if ((index - USB_STRING_INDEX_CUSTOM) < info->string_count)
                            {
                                success = stm32wb_usbd_string(info->string_table[index - USB_STRING_INDEX_CUSTOM], data, p_data_return, p_length_return);
                            }
                            break;
                        }
                        }
                        break;
                    }

                    case USB_DESCRIPTOR_TYPE_BOS: {
                        success = stm32wb_usbd_bos(data, p_data_return, p_length_return);
                        break;
                    }
                        
                    default:
                        break;
                    }
                    break;
                }
                    
                case USB_REQ_CODE_SET_DESCRIPTOR: {
                    break;
                }
                    
                case USB_REQ_CODE_GET_CONFIGURATION: {
                    if ((state == USB_STATE_ADDRESSED) || (state == USB_STATE_CONFIGURED))
                    {
                        *p_data_return = data;
                        *p_length_return = 1;
                        
                        data[0] = (state == USB_STATE_ADDRESSED) ? 0 : 1;

                        success = true;
                    }
                    break;
                }
                    
                case USB_REQ_CODE_SET_CONFIGURATION: {
                    if ((state == USB_STATE_ADDRESSED) || (state == USB_STATE_CONFIGURED))
                    {
                        if (request->wValue == 0)
                        {
                            if (state == USB_STATE_CONFIGURED)
                            {
                                for (function = 0; function < info->function_count; function++)
                                {
                                    (*info->function_table[function].interface->stop)(info->function_table[function].context);
                                }
                            }
                                
                            stm32wb_usbd_control.state = STM32WB_USBD_STATE_ADDRESSED;
                        }
                        else
                        {
                            if (state == USB_STATE_ADDRESSED)
                            {
                                for (function = 0; function < info->function_count; function++)
                                {
                                    (*info->function_table[function].interface->start)(info->function_table[function].context);
                                }
                            }
                            
                            stm32wb_usbd_control.state = STM32WB_USBD_STATE_CONFIGURED;
                        }
                        
                        success = true;
                    }
                    break;
                }
                    
                default:
                    break;
                }
                break;
            }

            case USB_REQ_TYPE_VENDOR: {
                switch (request->bRequest) {
                case USB_REQ_MS_VENDOR_CODE: {
                    switch (request->wIndex) {
                    case MS_OS_2O_DESCRIPTOR_INDEX: {
                        success = stm32wb_usbd_msos20(data, p_data_return, p_length_return);
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
                break;
            }

            default:
                break;
            }
        }
        break;

    case USB_REQ_RECIPIENT_INTERFACE:
        interface = request->wIndex;

        if (interface < stm32wb_usbd_control.if_count)
        {
            for (function = 0, if_base = 0; function < info->function_count; function++)
            {
                if ((if_base <= interface) && ((if_base + info->function_table[function].if_count) > interface))
                {
                    success = (*info->function_table[function].interface->request)(info->function_table[function].context, state, request, data, p_data_return, p_length_return);
                        
                    break;
                }

                if_base += info->function_table[function].if_count;
            }
                
            if (!success)
            {
                switch (request->bmRequestType & USB_REQ_TYPE_MASK) {
                case USB_REQ_TYPE_STANDARD: {
                    switch (request->bRequest) {
                    case USB_REQ_CODE_GET_STATUS: {
                        if (((state == USB_STATE_ADDRESSED) && !interface) || (state == USB_STATE_CONFIGURED))
                        {
                            *p_data_return = data;
                            *p_length_return = 2;
                            
                            data[0] = 0x00;
                            data[1] = 0x00;
                            
                            success = true;
                        }
                        break;
                    }
                        
                    case USB_REQ_CODE_CLEAR_FEATURE: {
                        /* No standard features defined */
                        break;
                    }
                            
                    case USB_REQ_CODE_SET_FEATURE: {
                        /* No standard features defined */
                        break;
                    }
                            
                    case USB_REQ_CODE_GET_INTERFACE: {
                        /* Must be handled by the function, defaults to 0 */
                        if (state == USB_STATE_CONFIGURED)
                        {
                            *p_data_return = data;
                            *p_length_return = 1;
                            
                            data[0] = 0x00;
                            
                            success = true;
                        }
                        break;
                    }
                            
                    case USB_REQ_CODE_SET_INTERFACE: {
                        /* Must be handled by the function, or default to error */
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
        }
        break;

    case USB_REQ_RECIPIENT_ENDPOINT:
        ep_addr = request->wIndex & 0x8f;
        ep_mask = STM32WB_USBD_EP_MASK(ep_addr);

        if (ep_mask & stm32wb_usbd_control.ep_mask)
        {
            for (function = 0; function < info->function_count; function++)
            {
                if (ep_mask & info->function_table[function].ep_mask)
                {
                    success = (*info->function_table[function].interface->request)(info->function_table[function].context, state, request, data, p_data_return, p_length_return);
                        
                    break;
                }
            }
                
            if (!success)
            {
                switch (request->bmRequestType & USB_REQ_TYPE_MASK) {
                case USB_REQ_TYPE_STANDARD: {
                    switch (request->bRequest) {
                    case USB_REQ_CODE_GET_STATUS: {
                        if (((state == USB_STATE_ADDRESSED) && !(ep_addr & 0x0f)) || (state == USB_STATE_CONFIGURED))
                        {
                            *p_data_return = data;
                            *p_length_return = 2;
                            
                            data[0] = ((ep_addr & 0x0f) && stm32wb_usbd_dcd_ep_is_stalled(ep_addr)) ? 0x01 : 0x00;
                            data[1] = 0x00;
                        
                            success = true;
                        }
                        break;
                    }
                        
                    case USB_REQ_CODE_CLEAR_FEATURE: {
                        if (request->wValue == USB_FEATURE_ENDPOINT_HALT)
                        {
                            if (((state == USB_STATE_ADDRESSED) && !(ep_addr & 0x0f)) || (state == USB_STATE_CONFIGURED))
                            {
                                success = (ep_addr & 0x0f) && stm32wb_usbd_dcd_ep_unstall(ep_addr);
                            }
                        }
                        break;
                    }
                        
                    case USB_REQ_CODE_SET_FEATURE: {
                        if (request->wValue == USB_FEATURE_ENDPOINT_HALT)
                        {
                            if (((state == USB_STATE_ADDRESSED) && !(ep_addr & 0x0f)) || (state == USB_STATE_CONFIGURED))
                            {
                                success = (ep_addr & 0x0f) && stm32wb_usbd_dcd_ep_stall(ep_addr);
                            }
                        }
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
        }
        break;

    default:
        break;
    }
    
    return success;
}

static void stm32wb_usbd_event_process(void)
{
    uint32_t events, function;
    const stm32wb_usbd_info_t *info;
    
    info = stm32wb_usbd_control.info;

    events = armv7m_atomic_swap(&stm32wb_usbd_control.events, 0);
    
    if (events & STM32WB_USBD_DCD_EVENT_EP0_REQUEST)
    {
        stm32wb_usbd_control.ep0_status = stm32wb_usbd_request(&stm32wb_usbd_control.ep0_request, stm32wb_usbd_control.ep0_out_data, &stm32wb_usbd_control.ep0_in_data, &stm32wb_usbd_control.ep0_in_length);

        stm32wb_usbd_dcd_ep0_request();
    }

    if (events & STM32WB_USBD_DCD_EVENT_DETECT)
    {
        stm32wb_usbd_control.state = STM32WB_USBD_STATE_DETECTED;

        stm32wb_usbd_control.bcd_status = stm32wb_usbd_dcd_bcd_status();

        if (stm32wb_usbd_control.evt_callback)
        {
            (*stm32wb_usbd_control.evt_callback)(stm32wb_usbd_control.evt_context, STM32WB_USBD_EVENT_DETECT);
        }

        if ((stm32wb_usbd_control.bcd_status == STM32WB_USBD_BCD_STATUS_PORT) || (stm32wb_usbd_control.bcd_status == STM32WB_USBD_BCD_STATUS_PORT_AND_CHARGER))
        {
            if (stm32wb_usbd_control.connect)
            {
                stm32wb_usbd_connect();
            }
            else
            {
                stm32wb_usbd_dcd_disable();
            }
        }
        else
        {
            stm32wb_usbd_dcd_disable();
        }
    }
    
    if (events & STM32WB_USBD_DCD_EVENT_RESET)
    {
        if (stm32wb_usbd_control.state == STM32WB_USBD_STATE_CONFIGURED)
        {
            for (function = 0; function < info->function_count; function++)
            {
                (*info->function_table[function].interface->stop)(info->function_table[function].context);
            }
        }

        stm32wb_usbd_dcd_reset();

        if (stm32wb_usbd_control.state == STM32WB_USBD_STATE_POWERED)
        {
            if (stm32wb_usbd_control.evt_callback)
            {
                (*stm32wb_usbd_control.evt_callback)(stm32wb_usbd_control.evt_context, STM32WB_USBD_EVENT_CONNECT);
            }
        }
        
        stm32wb_usbd_control.state = STM32WB_USBD_STATE_DEFAULT;

        stm32wb_usbd_control.remote_wakeup = false;
    }
    
    if (events & STM32WB_USBD_DCD_EVENT_SUSPEND)
    {
        for (function = 0; function < info->function_count; function++)
        {
            (*info->function_table[function].interface->suspend)(info->function_table[function].context);
        }

        if (stm32wb_usbd_control.evt_callback)
        {
            (*stm32wb_usbd_control.evt_callback)(stm32wb_usbd_control.evt_context, STM32WB_USBD_EVENT_SUSPEND);
        }
    }

    if (events & STM32WB_USBD_DCD_EVENT_RESUME)
    {
        if (stm32wb_usbd_control.evt_callback)
        {
            (*stm32wb_usbd_control.evt_callback)(stm32wb_usbd_control.evt_context, STM32WB_USBD_EVENT_RESUME);
        }

        for (function = 0; function < info->function_count; function++)
        {
            (*info->function_table[function].interface->resume)(info->function_table[function].context);
        }
    }
    
    if (events & STM32WB_USBD_DCD_EVENT_SOF)
    {
        for (function = 0; function < info->function_count; function++)
        {
            (*info->function_table[function].interface->sof)(info->function_table[function].context);
        }
    }
}

static void stm32wb_usbd_event_callback(void *context, uint32_t events)
{
    uint32_t count;

    if (events & STM32WB_USBD_DCD_EVENT_EP0_REQUEST)
    {
        if (stm32wb_usbd_control.ep0_cancel)
        {
            stm32wb_usbd_control.ep0_cancel = false;
            
            stm32wb_usbd_control.ep0_state = STM32WB_USBD_EP0_STATE_STATUS;

            events |= STM32WB_USBD_DCD_EVENT_EP0_SETUP;
        }
        else
        {
            if (stm32wb_usbd_control.ep0_status)
            {
                if (stm32wb_usbd_control.ep0_request.bmRequestType & USB_REQ_DIRECTION_DEVICE_TO_HOST)
                {
                    if (stm32wb_usbd_control.ep0_in_length >= stm32wb_usbd_control.ep0_request.wLength)
                    {
                        stm32wb_usbd_control.ep0_in_length = stm32wb_usbd_control.ep0_request.wLength;
                        stm32wb_usbd_control.ep0_zlp = false;
                    }
                    else
                    {
                        stm32wb_usbd_control.ep0_zlp = !(stm32wb_usbd_control.ep0_in_length & (STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE -1));
                    }
                
                    if (stm32wb_usbd_control.ep0_in_length)
                    {
                        count = stm32wb_usbd_control.ep0_in_length;
                    
                        if (count > STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE)
                        {
                            count = STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE;
                        }
                    
                        stm32wb_usbd_dcd_ep0_transmit(stm32wb_usbd_control.ep0_in_data, count);
                    
                        stm32wb_usbd_control.ep0_in_data += count;
                        stm32wb_usbd_control.ep0_in_length -= count;
                    
                        stm32wb_usbd_control.ep0_state = STM32WB_USBD_EP0_STATE_DATA_IN;
                    }
                    else
                    {
                        stm32wb_usbd_dcd_ep0_transmit(NULL, 0);
                    
                        stm32wb_usbd_control.ep0_zlp = false;
                        stm32wb_usbd_control.ep0_state = STM32WB_USBD_EP0_STATE_DATA_IN;
                    }
                }
                else
                {
                    stm32wb_usbd_dcd_ep0_transmit(NULL, 0);

                    stm32wb_usbd_control.ep0_state = STM32WB_USBD_EP0_STATE_STATUS;
                }
            }
            else
            {
                stm32wb_usbd_dcd_ep0_stall();

                stm32wb_usbd_control.ep0_state = STM32WB_USBD_EP0_STATE_STATUS;
            }
        }

        events &= ~STM32WB_USBD_DCD_EVENT_EP0_REQUEST;
    }

    if (events & STM32WB_USBD_DCD_EVENT_EP0_SETUP)
    {
        if (stm32wb_usbd_control.ep0_state != STM32WB_USBD_EP0_STATE_REQUEST)
        {
            memcpy(&stm32wb_usbd_control.ep0_request, &stm32wb_usbd_control.ep0_setup, sizeof(stm32wb_usbd_control.ep0_request));

            stm32wb_usbd_control.ep0_cancel = false;
            stm32wb_usbd_control.ep0_zlp = false;
            stm32wb_usbd_control.ep0_status = false;
            stm32wb_usbd_control.ep0_out_data = stm32wb_usbd_control.info->ep0_data;
            stm32wb_usbd_control.ep0_out_size = stm32wb_usbd_control.info->ep0_size;
            stm32wb_usbd_control.ep0_out_count = 0;
            stm32wb_usbd_control.ep0_in_data = NULL;
            stm32wb_usbd_control.ep0_in_length = 0;

            if ((stm32wb_usbd_control.ep0_request.bmRequestType & USB_REQ_DIRECTION_DEVICE_TO_HOST) || (stm32wb_usbd_control.ep0_request.wLength == 0))
            {
                stm32wb_usbd_control.ep0_state = STM32WB_USBD_EP0_STATE_REQUEST;
                
                events |= STM32WB_USBD_DCD_EVENT_EP0_REQUEST;
            }
            else
            {
                if (stm32wb_usbd_control.ep0_request.wLength <= stm32wb_usbd_control.ep0_out_size)
                {
                    count = stm32wb_usbd_control.ep0_request.wLength;
                    
                    if (count > STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE)
                    {
                        count = STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE;
                    }

                    stm32wb_usbd_dcd_ep0_receive(&stm32wb_usbd_control.ep0_out_data[stm32wb_usbd_control.ep0_out_count], count);

                    stm32wb_usbd_control.ep0_state = STM32WB_USBD_EP0_STATE_DATA_OUT;
                }
                else
                {
                    stm32wb_usbd_dcd_ep0_stall();
                    
                    stm32wb_usbd_control.ep0_state = STM32WB_USBD_EP0_STATE_STATUS;
                }
            }
        }
        else
        {
            stm32wb_usbd_control.ep0_cancel = true;
        }
        
        events &= ~STM32WB_USBD_DCD_EVENT_EP0_SETUP;
    }
    
    if (events & STM32WB_USBD_DCD_EVENT_EP0_DATA_IN)
    {
        if (stm32wb_usbd_control.ep0_state == STM32WB_USBD_EP0_STATE_DATA_IN)
        {
            if (stm32wb_usbd_control.ep0_in_length)
            {
                count = stm32wb_usbd_control.ep0_in_length;
            
                if (count > STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE)
                {
                    count = STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE;
                }
            
                stm32wb_usbd_dcd_ep0_transmit(stm32wb_usbd_control.ep0_in_data, count);
            
                stm32wb_usbd_control.ep0_in_data += count;
                stm32wb_usbd_control.ep0_in_length -= count;
            }
            else if (stm32wb_usbd_control.ep0_zlp)
            {
                stm32wb_usbd_dcd_ep0_transmit(NULL, 0);

                stm32wb_usbd_control.ep0_zlp = false;
            }
            else
            {
                stm32wb_usbd_dcd_ep0_receive(NULL, 0);

                stm32wb_usbd_control.ep0_state = STM32WB_USBD_EP0_STATE_STATUS;
            }
        }

        events &= ~STM32WB_USBD_DCD_EVENT_EP0_DATA_IN;
    }

    if (events & STM32WB_USBD_DCD_EVENT_EP0_DATA_OUT)
    {
        if (stm32wb_usbd_control.ep0_state == STM32WB_USBD_EP0_STATE_DATA_OUT)
        {
            count = stm32wb_usbd_dcd_ep0_count();

            if (count && ((stm32wb_usbd_control.ep0_out_count + count) <= stm32wb_usbd_control.ep0_request.wLength))
            {
                stm32wb_usbd_control.ep0_out_count += count;
                
                if (stm32wb_usbd_control.ep0_out_count != stm32wb_usbd_control.ep0_request.wLength)
                {
                    count = (stm32wb_usbd_control.ep0_request.wLength - stm32wb_usbd_control.ep0_out_count);
                    
                    if (count > STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE)
                    {
                        count = STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE;
                    }

                    stm32wb_usbd_dcd_ep0_receive(&stm32wb_usbd_control.ep0_out_data[stm32wb_usbd_control.ep0_out_count], count);
                }
                else
                {
                    stm32wb_usbd_control.ep0_state = STM32WB_USBD_EP0_STATE_REQUEST;

                    events |= STM32WB_USBD_DCD_EVENT_EP0_REQUEST;
                }
            }
            else
            {
                stm32wb_usbd_dcd_ep0_stall();

                stm32wb_usbd_control.ep0_state = STM32WB_USBD_EP0_STATE_STATUS;
            }
        }

        events &= ~STM32WB_USBD_DCD_EVENT_EP0_DATA_OUT;
    }

    if (events)
    {
        if (!armv7m_atomic_or(&stm32wb_usbd_control.events, events))
        {
            armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_DCD);
        }
    }
}

bool stm32wb_usbd_configure(const stm32wb_usbd_device_t *device, const stm32wb_usbd_info_t *info, const stm32wb_usbd_params_t *params)
{
    uint32_t function, if_base;
    
    if (stm32wb_usbd_control.state != STM32WB_USBD_STATE_NONE)
    {
        return false;
    }

    stm32wb_lptim_timeout_create(&stm32wb_usbd_control.timeout);
    
    stm32wb_usbd_control.device = device;
    stm32wb_usbd_control.info = info;
    stm32wb_usbd_control.pin_vbus = params->pin_vbus;

    stm32wb_usbd_control.state = STM32WB_USBD_STATE_INIT;

    stm32wb_usbd_dcd_configure();
    
    for (function = 0, if_base = 0; function < info->function_count; function++)
    {
        (*info->function_table[function].interface->configure)(info->function_table[function].context, if_base, info->function_table[function].options);
        
        if_base += info->function_table[function].if_count;

        stm32wb_usbd_control.ep_mask |= info->function_table[function].ep_mask;
    }

    stm32wb_usbd_control.if_count = if_base;

    return true;
}

static bool __svc_stm32wb_usbd_enable(stm32wb_usbd_event_callback_t callback, void *context)
{
    if (stm32wb_usbd_control.state != STM32WB_USBD_STATE_INIT)
    {
        return false;
    }

    stm32wb_usbd_control.state = STM32WB_USBD_STATE_NOT_READY;

    stm32wb_usbd_control.evt_callback = callback;
    stm32wb_usbd_control.evt_context = context;

    stm32wb_usbd_control.state = STM32WB_USBD_STATE_READY;

    if (stm32wb_usbd_control.pin_vbus == STM32WB_GPIO_PIN_NONE)
    {
        stm32wb_system_pvm1_enable((stm32wb_system_pvm1_callback_t)stm32wb_usbd_pvm1_changed, NULL);
        
        stm32wb_usbd_control.vbus_status = stm32wb_system_pvm1_sense();
    }
    else
    {
        stm32wb_gpio_pin_configure(stm32wb_usbd_control.pin_vbus, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_PULLDOWN | STM32WB_GPIO_OSPEED_LOW | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_INPUT));
        stm32wb_exti_attach(stm32wb_usbd_control.pin_vbus, (STM32WB_EXTI_CONTROL_PRIORITY_LOW | STM32WB_EXTI_CONTROL_EDGE_RISING | STM32WB_EXTI_CONTROL_EDGE_FALLING), (stm32wb_exti_callback_t)stm32wb_usbd_vbus_changed, NULL);

        stm32wb_usbd_control.vbus_status = !!stm32wb_gpio_pin_read(stm32wb_usbd_control.pin_vbus);
    }
    
    if (stm32wb_usbd_control.vbus_status)
    {
        stm32wb_lptim_timeout_start(&stm32wb_usbd_control.timeout, stm32wb_lptim_timeout_millis_to_ticks(40), (stm32wb_lptim_timeout_callback_t)stm32wb_usbd_vbus_timeout); /* 40ms */
    }
    
    return true;
}

static bool __svc_stm32wb_usbd_disable(void)
{
    return true;
}

static bool __svc_stm32wb_usbd_attach(void)
{
    if (!stm32wb_usbd_control.connect)
    {
        stm32wb_usbd_control.connect = true;

        if ((stm32wb_usbd_control.state == STM32WB_USBD_STATE_DETECTED) && (stm32wb_usbd_control.bcd_status != STM32WB_USBD_BCD_STATUS_CHARGER))
        {
            stm32wb_usbd_dcd_enable(&stm32wb_usbd_control.ep0_setup[0], 0, stm32wb_usbd_event_callback, NULL);

            stm32wb_usbd_connect();
        }
    }
    
    return true;
}

static bool __svc_stm32wb_usbd_detach(void)
{
    if (stm32wb_usbd_control.connect)
    {
        stm32wb_usbd_control.connect = false;

        if (stm32wb_usbd_control.state >= STM32WB_USBD_STATE_POWERED)
        {
            stm32wb_usbd_disconnect();
        }
    }

    return true;
}

static bool __svc_stm32wb_usbd_wakeup(void)
{
    if (stm32wb_usbd_control.remote_wakeup)
    {
        return stm32wb_usbd_dcd_wakeup();
    }

    return false;
}

bool stm32wb_usbd_enable(stm32wb_usbd_event_callback_t callback, void *context)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_2((uint32_t)&__svc_stm32wb_usbd_enable, (uint32_t)callback, (uint32_t)context);
    }

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
        return __svc_stm32wb_usbd_enable(callback, context);
    }

    return false;
}

bool stm32wb_usbd_disable(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_usbd_disable);
    }

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
        return __svc_stm32wb_usbd_disable();
    }

    return false;
}

bool stm32wb_usbd_attach(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_usbd_attach);
    }

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
        return __svc_stm32wb_usbd_attach();
    }

    return false;
}

bool stm32wb_usbd_detach(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_usbd_detach);
    }

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
        return __svc_stm32wb_usbd_detach();
    }

    return false;
}

bool stm32wb_usbd_wakeup(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_usbd_wakeup);
    }

    if (armv7m_core_is_in_pendsv_or_svcall())
    {
        return __svc_stm32wb_usbd_wakeup();
    }

    return false;
}

uint32_t stm32wb_usbd_vbus_status(void)
{
    return stm32wb_usbd_control.vbus_status;
}

uint32_t stm32wb_usbd_bcd_status(void)
{
    return stm32wb_usbd_control.bcd_status;
}

void USBD_PVM1_SWIHandler(void)
{
    stm32wb_usbd_vbus_changed();
}

void USBD_DCD_SWIHandler(void)
{
    stm32wb_usbd_event_process();
}
