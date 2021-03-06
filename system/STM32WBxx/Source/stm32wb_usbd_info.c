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
#include "stm32wb_usbd.h"
#include "stm32wb_usbd_info.h"
#include "stm32wb_usbd_cdc.h"
#include "stm32wb_usbd_dfu.h"
#include "stm32wb_usbd_msc.h"

#define STM32WB_USBD_DFU_INTERFACE_COUNT        1
#define STM32WB_USBD_CDC_INTERFACE_COUNT        2
#define STM32WB_USBD_MSC_INTERFACE_COUNT        1

#define STM32WB_USBD_CONFIGURATION_SIZE         (9)
#define STM32WB_USBD_CDC_CONFIGURATION_SIZE     (STM32WB_USBD_CONFIGURATION_SIZE + (8+(9+5+5+4+5+7)+(9+7+7)))
#define STM32WB_USBD_CDC_MSC_CONFIGURATION_SIZE (STM32WB_USBD_CDC_CONFIGURATION_SIZE + (9+7+7))

#define STM32WB_USBD_CDC_STRING_COUNT           3
#define STM32WB_USBD_CDC_MSC_STRING_COUNT       4

#define STM32WB_USBD_CDC_FUNCTION_COUNT         1
#define STM32WB_USBD_CDC_MSC_FUNCTION_COUNT     2


#define STM32WB_USBD_DFU_CONFIGURATION_SIZE         (STM32WB_USBD_CONFIGURATION_SIZE + (9+9))
#define STM32WB_USBD_DFU_CDC_CONFIGURATION_SIZE     (STM32WB_USBD_DFU_CONFIGURATION_SIZE + (8+(9+5+5+4+5+7)+(9+7+7)))
#define STM32WB_USBD_DFU_CDC_MSC_CONFIGURATION_SIZE (STM32WB_USBD_DFU_CDC_CONFIGURATION_SIZE + (9+7+7))

#define STM32WB_USBD_DFU_CDC_STRING_COUNT           4
#define STM32WB_USBD_DFU_CDC_MSC_STRING_COUNT       5

#define STM32WB_USBD_DFU_CDC_FUNCTION_COUNT         2
#define STM32WB_USBD_DFU_CDC_MSC_FUNCTION_COUNT     3

#define STM32WB_USBD_DFU_CDC_INTERFACE_COUNT        3
#define STM32WB_USBD_DFU_CDC_MSC_INTERFACE_COUNT    4

static const uint8_t stm32wb_usbd_cdc_configuration[STM32WB_USBD_CDC_CONFIGURATION_SIZE] =
{
    /**** Configuration Descriptor ****/
    9,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_CONFIGURATION,                           /* bDescriptorType */
    STM32WB_USBD_CDC_CONFIGURATION_SIZE & 255,                   /* wTotalLength */
    STM32WB_USBD_CDC_CONFIGURATION_SIZE >> 8,
    0x02,                                                        /* bNumInterfaces */
    0x01,                                                        /* bConfigurationValue */
    0x00,                                                        /* iConfiguration */
    0xa0,                                                        /* bmAttributes */
    0xfa,                                                        /* bMaxPower */

    /**** IAD to associate the two CDC interfaces ****/
    8,                                                           /* bLength */
    0x0b,                                                        /* bDescriptorType */
    0x00,                                                        /* bFirstInterface */
    0x02,                                                        /* bInterfaceCount */
    0x02,                                                        /* bFunctionClass */
    0x02,                                                        /* bFunctionSubClass */
    0x01,                                                        /* bFunctionProtocol */
    0x04,                                                        /* iFunction */

    /**** CDC Control Interface ****/
    9,                                                           /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x00,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x01,                                                        /* bNumEndpoints */
    0x02,                                                        /* bInterfaceClass */
    0x02,                                                        /* bInterfaceSubClass */
    0x01,                                                        /* bInterfaceProtocol */
    0x05,                                                        /* iInterface */
  
    /**** CDC Header ****/
    5,                                                           /* bLength */
    0x24,                                                        /* bDescriptorType */
    0x00,                                                        /* bDescriptorSubtype */
    0x10,                                                        /* bcdCDC */
    0x01,
  
    /**** CDC Call Management ****/
    5,                                                           /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x01,                                                        /* bDescriptorSubtype */
    0x00,                                                        /* bmCapabilities */
    0x01,                                                        /* bDataInterface */
  
    /**** CDC ACM ****/
    4,                                                           /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x02,                                                        /* bDescriptorSubtype */
    0x06,                                                        /* bmCapabilities */
  
    /**** CDC Union ****/
    5,                                                           /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x06,                                                        /* bDescriptorSubtype */
    0x00,                                                        /* bMasterInterface */
    0x01,                                                        /* bSlaveInterface0 */
  
    /**** CDC Control Endpoint ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_CDC_CONTROL_EP_ADDR,                            /* bEndpointAddress */
    USB_EP_TYPE_INTERRUPT,                                       /* bmAttributes */
    STM32WB_USBD_CDC_CONTROL_MAX_PACKET_SIZE & 255,              /* wMaxPacketSize */
    STM32WB_USBD_CDC_CONTROL_MAX_PACKET_SIZE >> 8,
    STM32WB_USBD_CDC_CONTROL_INTERVAL,                           /* bInterval */ 

    /**** CDC Data Interface ****/
    9,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_INTERFACE,                               /* bDescriptorType */
    0x01,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x02,                                                        /* bNumEndpoints */
    0x0a,                                                        /* bInterfaceClass */
    0x00,                                                        /* bInterfaceSubClass */
    0x00,                                                        /* bInterfaceProtocol */
    0x06,                                                        /* iInterface */

    /**** CDC Data Endpoint IN ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_CDC_DATA_IN_EP_ADDR,                            /* bEndpointAddress */
    USB_EP_TYPE_BULK,                                            /* bmAttributes */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE & 255,                 /* wMaxPacketSize */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE >> 8,
    0,                                                           /* bInterval */

    /**** CDC Data Endpoint OUT ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_CDC_DATA_OUT_EP_ADDR,                           /* bEndpointAddress */
    USB_EP_TYPE_BULK,                                            /* bmAttributes */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE & 255,                 /* wMaxPacketSize */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE >> 8,
    0,                                                           /* bInterval */
};

static const char * const stm32wb_usbd_cdc_strings[STM32WB_USBD_CDC_STRING_COUNT] =
{
    "Serial",                                 // 4
    "CDC Control",                            // 5
    "CDC Data",                               // 6
};

static const stm32wb_usbd_function_t stm32wb_usbd_cdc_functions[STM32WB_USBD_CDC_FUNCTION_COUNT] =
{
    {
	&stm32wb_usbd_cdc_interface,
	NULL,
	0,
	STM32WB_USBD_CDC_INTERFACE_COUNT,
	(STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_CONTROL_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_DATA_IN_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR)),
    },
};

static uint8_t stm32wb_usbd_cdc_ep0_data[64];

const stm32wb_usbd_info_t stm32wb_usbd_cdc_info =
{
    stm32wb_usbd_cdc_ep0_data,
    sizeof(stm32wb_usbd_cdc_ep0_data),
    STM32WB_USBD_CDC_STRING_COUNT,
    STM32WB_USBD_CDC_FUNCTION_COUNT,
    stm32wb_usbd_cdc_configuration,
    NULL,
    NULL,
    stm32wb_usbd_cdc_strings,
    stm32wb_usbd_cdc_functions,
};

/********************************************************************************************************************************************/


static const uint8_t stm32wb_usbd_cdc_msc_configuration[STM32WB_USBD_CDC_MSC_CONFIGURATION_SIZE] =
{
    /**** Configuration Descriptor ****/
    9,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_CONFIGURATION,                           /* bDescriptorType */
    STM32WB_USBD_CDC_MSC_CONFIGURATION_SIZE & 255,               /* wTotalLength */
    STM32WB_USBD_CDC_MSC_CONFIGURATION_SIZE >> 8,
    0x03,                                                        /* bNumInterfaces */
    0x01,                                                        /* bConfigurationValue */
    0x00,                                                        /* iConfiguration */
    0xa0,                                                        /* bmAttributes */
    0xfa,                                                        /* bMaxPower */

    /**** IAD to associate the two CDC interfaces ****/
    8,                                                           /* bLength */
    0x0b,                                                        /* bDescriptorType */
    0x00,                                                        /* bFirstInterface */
    0x02,                                                        /* bInterfaceCount */
    0x02,                                                        /* bFunctionClass */
    0x02,                                                        /* bFunctionSubClass */
    0x01,                                                        /* bFunctionProtocol */
    0x04,                                                        /* iFunction */

    /**** CDC Control Interface ****/
    9,                                                           /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x00,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x01,                                                        /* bNumEndpoints */
    0x02,                                                        /* bInterfaceClass */
    0x02,                                                        /* bInterfaceSubClass */
    0x01,                                                        /* bInterfaceProtocol */
    0x05,                                                        /* iInterface */
  
    /**** CDC Header ****/
    5,                                                           /* bLength */
    0x24,                                                        /* bDescriptorType */
    0x00,                                                        /* bDescriptorSubtype */
    0x10,                                                        /* bcdCDC */
    0x01,
  
    /**** CDC Call Management ****/
    5,                                                           /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x01,                                                        /* bDescriptorSubtype */
    0x00,                                                        /* bmCapabilities */
    0x01,                                                        /* bDataInterface */
  
    /**** CDC ACM ****/
    4,                                                           /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x02,                                                        /* bDescriptorSubtype */
    0x06,                                                        /* bmCapabilities */
  
    /**** CDC Union ****/
    5,                                                           /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x06,                                                        /* bDescriptorSubtype */
    0x00,                                                        /* bMasterInterface */
    0x01,                                                        /* bSlaveInterface0 */
  
    /**** CDC Control Endpoint ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_CDC_CONTROL_EP_ADDR,                            /* bEndpointAddress */
    USB_EP_TYPE_INTERRUPT,                                       /* bmAttributes */
    STM32WB_USBD_CDC_CONTROL_MAX_PACKET_SIZE & 255,              /* wMaxPacketSize */
    STM32WB_USBD_CDC_CONTROL_MAX_PACKET_SIZE >> 8,
    STM32WB_USBD_CDC_CONTROL_INTERVAL,                           /* bInterval */ 

    /**** CDC Data Interface ****/
    9,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_INTERFACE,                               /* bDescriptorType */
    0x01,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x02,                                                        /* bNumEndpoints */
    0x0a,                                                        /* bInterfaceClass */
    0x00,                                                        /* bInterfaceSubClass */
    0x00,                                                        /* bInterfaceProtocol */
    0x06,                                                        /* iInterface */

    /**** CDC Data Endpoint IN ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_CDC_DATA_IN_EP_ADDR,                            /* bEndpointAddress */
    USB_EP_TYPE_BULK,                                            /* bmAttributes */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE & 255,                 /* wMaxPacketSize */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE >> 8,
    0,                                                           /* bInterval */

    /**** CDC Data Endpoint OUT ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_CDC_DATA_OUT_EP_ADDR,                           /* bEndpointAddress */
    USB_EP_TYPE_BULK,                                            /* bmAttributes */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE & 255,                 /* wMaxPacketSize */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE >> 8,
    0,                                                           /* bInterval */

    /**** MSC Interface ****/
    9,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_INTERFACE,                               /* bDescriptorType */
    0x02,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x02,                                                        /* bNumEndpoints */
    0x08,                                                        /* bInterfaceClass */
    0x06,                                                        /* bInterfaceSubClass */
    0x50,                                                        /* nInterfaceProtocol */
    0x07,                                                        /* iInterface */

    /**** MSC Endpoint IN ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_MSC_DATA_IN_EP_ADDR,                            /* bEndpointAddress */
    USB_EP_TYPE_BULK,                                            /* bmAttributes */
    STM32WB_USBD_MSC_DATA_MAX_PACKET_SIZE & 255,                 /* wMaxPacketSize */
    STM32WB_USBD_MSC_DATA_MAX_PACKET_SIZE >> 8,
    0,                                                           /* bInterval */

    /**** MSC Endpoint OUT ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_MSC_DATA_OUT_EP_ADDR,                           /* bEndpointAddress */
    USB_EP_TYPE_BULK,                                            /* bmAttributes */
    STM32WB_USBD_MSC_DATA_MAX_PACKET_SIZE & 255,                 /* wMaxPacketSize */
    STM32WB_USBD_MSC_DATA_MAX_PACKET_SIZE >> 8,
    0,                                                           /* bInterval */
};

static const char * const stm32wb_usbd_cdc_msc_strings[STM32WB_USBD_CDC_MSC_STRING_COUNT] =
{
    "Serial",                                 // 4
    "CDC Control",                            // 5
    "CDC Data",                               // 6
    "Mass Storage",                           // 7
};

static const stm32wb_usbd_function_t stm32wb_usbd_cdc_msc_functions[STM32WB_USBD_CDC_MSC_FUNCTION_COUNT] =
{
    {
	&stm32wb_usbd_cdc_interface,
	NULL,
	0,
	STM32WB_USBD_CDC_INTERFACE_COUNT,
	(STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_CONTROL_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_DATA_IN_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR)),
    },
    {
	&stm32wb_usbd_msc_interface,
	NULL,
	0,
	STM32WB_USBD_MSC_INTERFACE_COUNT,
	(STM32WB_USBD_EP_MASK(STM32WB_USBD_MSC_DATA_IN_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR)),
    },
};

static uint8_t stm32wb_usbd_cdc_msc_ep0_data[64];

const stm32wb_usbd_info_t stm32wb_usbd_cdc_msc_info =
{
    stm32wb_usbd_cdc_msc_ep0_data,
    sizeof(stm32wb_usbd_cdc_msc_ep0_data),
    STM32WB_USBD_CDC_MSC_STRING_COUNT,
    STM32WB_USBD_CDC_MSC_FUNCTION_COUNT,
    stm32wb_usbd_cdc_msc_configuration,
    NULL,
    NULL,
    stm32wb_usbd_cdc_msc_strings,
    stm32wb_usbd_cdc_msc_functions,
};

/*********************************************************************************************************************************/


static const uint8_t stm32wb_usbd_dfu_cdc_configuration[STM32WB_USBD_DFU_CDC_CONFIGURATION_SIZE] =
{
    /**** Configuration Descriptor ****/
    9,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_CONFIGURATION,                           /* bDescriptorType */
    STM32WB_USBD_DFU_CDC_CONFIGURATION_SIZE & 255,               /* wTotalLength */
    STM32WB_USBD_DFU_CDC_CONFIGURATION_SIZE >> 8,
    STM32WB_USBD_DFU_CDC_INTERFACE_COUNT,                        /* bNumInterfaces */
    0x01,                                                        /* bConfigurationValue */
    0x00,                                                        /* iConfiguration */
    0xa0,                                                        /* bmAttributes */
    0xfa,                                                        /* bMaxPower */

    /**** DFU Interface ****/
    9,                                                           /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x00,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x00,                                                        /* bNumEndpoints */
    0xfe,                                                        /* bInterfaceClass */
    0x01,                                                        /* bInterfaceSubClass */
    0x01,                                                        /* nInterfaceProtocol */
    0x04,                                                        /* iInterface */
    
    /**** DFU Descriptor ****/
    0x09,                                                        /* blength = 9 Bytes */
    0x21,                                                        /* bDescriptorType */
    0x0b,                                                        /* bmAttribute */
    0xff, 0x00,                                                  /* wDetachTimeOut= 255 ms */
    0x00, 0x04,                                                  /* wTransferSize = 1024 Bytes */
    0x1a, 0x01,                                                  /* bcdDFUVersion */
    
    /**** IAD to associate the two CDC interfaces ****/
    8,                                                           /* bLength */
    0x0b,                                                        /* bDescriptorType */
    0x01,                                                        /* bFirstInterface */
    0x02,                                                        /* bInterfaceCount */
    0x02,                                                        /* bFunctionClass */
    0x02,                                                        /* bFunctionSubClass */
    0x01,                                                        /* bFunctionProtocol */
    0x05,                                                        /* iFunction */

    /**** CDC Control Interface ****/
    9,                                                           /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x01,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x01,                                                        /* bNumEndpoints */
    0x02,                                                        /* bInterfaceClass */
    0x02,                                                        /* bInterfaceSubClass */
    0x01,                                                        /* bInterfaceProtocol */
    0x06,                                                        /* iInterface */
  
    /**** CDC Header ****/
    5,                                                           /* bLength */
    0x24,                                                        /* bDescriptorType */
    0x00,                                                        /* bDescriptorSubtype */
    0x10,                                                        /* bcdCDC */
    0x01,
  
    /**** CDC Call Management ****/
    5,                                                           /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x01,                                                        /* bDescriptorSubtype */
    0x00,                                                        /* bmCapabilities */
    0x02,                                                        /* bDataInterface */
  
    /**** CDC ACM ****/
    4,                                                           /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x02,                                                        /* bDescriptorSubtype */
    0x06,                                                        /* bmCapabilities */
  
    /**** CDC Union ****/
    5,                                                           /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x06,                                                        /* bDescriptorSubtype */
    0x01,                                                        /* bMasterInterface */
    0x02,                                                        /* bSlaveInterface0 */
  
    /**** CDC Control Endpoint ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_CDC_CONTROL_EP_ADDR,                            /* bEndpointAddress */
    USB_EP_TYPE_INTERRUPT,                                       /* bmAttributes */
    STM32WB_USBD_CDC_CONTROL_MAX_PACKET_SIZE & 255,              /* wMaxPacketSize */
    STM32WB_USBD_CDC_CONTROL_MAX_PACKET_SIZE >> 8,
    STM32WB_USBD_CDC_CONTROL_INTERVAL,                           /* bInterval */ 

    /**** CDC Data Interface ****/
    9,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_INTERFACE,                               /* bDescriptorType */
    0x02,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x02,                                                        /* bNumEndpoints */
    0x0a,                                                        /* bInterfaceClass */
    0x00,                                                        /* bInterfaceSubClass */
    0x00,                                                        /* bInterfaceProtocol */
    0x07,                                                        /* iInterface */

    /**** CDC Data Endpoint IN ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_CDC_DATA_IN_EP_ADDR,                            /* bEndpointAddress */
    USB_EP_TYPE_BULK,                                            /* bmAttributes */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE & 255,                 /* wMaxPacketSize */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE >> 8,
    0,                                                           /* bInterval */

    /**** CDC Data Endpoint OUT ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_CDC_DATA_OUT_EP_ADDR,                           /* bEndpointAddress */
    USB_EP_TYPE_BULK,                                            /* bmAttributes */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE & 255,                 /* wMaxPacketSize */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE >> 8,
    0,                                                           /* bInterval */
};

static const char * const stm32wb_usbd_dfu_cdc_strings[STM32WB_USBD_DFU_CDC_STRING_COUNT] =
{
    "STM Device in DFU Runtime",              // 4
    "Serial",                                 // 5
    "CDC Control",                            // 6
    "CDC Data",                               // 7
};

static const stm32wb_usbd_function_t stm32wb_usbd_dfu_cdc_functions[STM32WB_USBD_DFU_CDC_FUNCTION_COUNT] =
{
    {
	&stm32wb_usbd_dfu_runtime_interface,
	NULL,
	0,
	STM32WB_USBD_DFU_INTERFACE_COUNT,
	0,
    },
    {
	&stm32wb_usbd_cdc_interface,
	NULL,
	0,
	STM32WB_USBD_CDC_INTERFACE_COUNT,
	(STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_CONTROL_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_DATA_IN_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR)),
    },
};

#if 0

static const uint8_t stm32wb_usbd_dfu_cdc_bos[0x28] = {
    // BOS
    0x05,                                            // bLength
    USB_DESCRIPTOR_TYPE_BOS,                         // bDescriptorType
    0x28, 0x00,                                      // wTotalLength
    0x02,                                            // bNumDeviceCaps

    // USB 2.0 Extensions
    0x07,                                            // bLength
    USB_DESCRIPTOR_TYPE_DEVICE_CAPABILITY,           // bDescriptorType
    USB_DEVICE_CAPABILITY_TYPE_USB20_EXTENSION,      // bDevCapabilityType
    0x00, 0x00, 0x00, 0x00,                          // bmAttributes

    // Microsoft OS 2.0 platform capability
    0x1c,                                            // bLength
    USB_DESCRIPTOR_TYPE_DEVICE_CAPABILITY,           // bDescriptorType
    USB_DEVICE_CAPABILITY_TYPE_PLATFORM,             // bDevCapabilityType
    0x00,                                            // bReserved
    0xdf, 0x60, 0xdd, 0xd8, 0x89, 0x45, 0xc7, 0x4c,  // platformCapabilityUUID[16]     
    0x00, 0x00, 0x03, 0x06,                          // dwWindowsVersion (Windows 8.1)
    0xae, 0x00,                                      // wLength (MSOS20)
    USB_REQ_MS_VENDOR_CODE,                          // bMS_VendorCode
    0x00,                                            // bAltEnumCode
};

#endif

static const uint8_t stm32wb_usbd_dfu_cdc_bos[] = {
    // BOS
    0x05,                                            // bLength
    USB_DESCRIPTOR_TYPE_BOS,                         // bDescriptorType
    0x21, 0x00,                                      // wTotalLength
    0x01,                                            // bNumDeviceCaps

    // Microsoft OS 2.0 platform capability
    0x1c,                                            // bLength
    USB_DESCRIPTOR_TYPE_DEVICE_CAPABILITY,           // bDescriptorType
    USB_DEVICE_CAPABILITY_TYPE_PLATFORM,             // bDevCapabilityType
    0x00,                                            // bReserved
    0xdf, 0x60, 0xdd, 0xd8, 0x89, 0x45, 0xc7, 0x4c,  // platformCapabilityUUID[16]     
    0x9c, 0xd2, 0x65, 0x9d, 0x9e, 0x64, 0x8a, 0x9f,  // platformCapabilityUUID[16]     
    0x00, 0x00, 0x03, 0x06,                          // dwWindowsVersion (Windows 8.1)
    0xb2, 0x00,                                      // wLength (MSOS20)
    USB_REQ_MS_VENDOR_CODE,                          // bMS_VendorCode
    0x00,                                            // bAltEnumCode
};

static const uint8_t stm32wb_usbd_dfu_cdc_msos20[] = {
    // Microsoft OS 2.0 descriptor set header
    0x0a, 0x00,                                      // wLength
    0x00, 0x00,                                      // wDescriptorType
    0x00, 0x00, 0x03, 0x06,                          // dwWindowsVersion (Windows 8.1)
    0xb2, 0x00,                                      // wTotalLength

    // Microsoft OS 2.0 configuration subset header
    0x08, 0x00,                                      // wLength
    0x01, 0x00,                                      // wDescriptorType
    0x00,                                            // bConfigurationValue
    0x00,                                            // bReserved
    0xa8, 0x00,                                      // wTotalLength (size of entire configuration subset)

    // Microsoft OS 2.0 function subset header
    0x08, 0x00,                                      // wLength
    0x02, 0x00,                                      // wDescriptorType
    0x00,                                            // bFirstInterface
    0x00,                                            // bReserved
    0xa0, 0x00,                                      // wTotalLength  (size of entine function subset)

    // Microsoft OS 2.0 compatible ID
    0x14, 0x00,                                      // wLength
    0x03, 0x00,                                      // wDescriptorType
    'W',  'I',  'N',  'U',  'S',  'B',  0x00, 0x00,  // compatible ID
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // subCompatibleID
                                        
    // Microsoft OS 2.0 registry property
    0x84, 0x00,                                      // wLength
    0x04, 0x00,                                      // wDescriptorType
    0x07, 0x00,                                      // wPropertyDataType
    0x2a, 0x00,                                      // wPropertyNameLength
    'D',  0x00, 'e',  0x00, 'v',  0x00, 'i',  0x00,  // bPropertyName "DeviceInterfaceGUIDs"
    'c',  0x00, 'e',  0x00, 'I',  0x00, 'n',  0x00,    
    't',  0x00, 'e',  0x00, 'r',  0x00, 'f',  0x00,    
    'a',  0x00, 'c',  0x00, 'e',  0x00, 'G',  0x00,    
    'U',  0x00, 'I',  0x00, 'D',  0x00, 's',  0x00,        
    0x00, 0x00,
    0x50, 0x00,                                      // wPropertyDataLength
    '{',  0x00, 'D',  0x00, 'E',  0x00, '5',  0x00,  // bProperyData "{DE50DD7B-4DEF-4F9F-98AA-88144B16A383}"
    '0',  0x00, 'D',  0x00, 'D',  0x00, '7',  0x00,
    'B',  0x00, '-',  0x00, '4',  0x00, 'D',  0x00,
    'E',  0x00, 'F',  0x00, '-',  0x00, '4',  0x00,
    'F',  0x00, '9',  0x00, 'F',  0x00, '-',  0x00,
    '9',  0x00, '8',  0x00, 'A',  0x00, 'A',  0x00,
    '-',  0x00, '8',  0x00, '8',  0x00, '1',  0x00,
    '4',  0x00, '4',  0x00, 'B',  0x00, '1',  0x00,
    '6',  0x00, 'A',  0x00, '3',  0x00, '8',  0x00,
    '3',  0x00, '}',  0x00,
    0x00, 0x00, 0x00, 0x00,
};

static uint8_t stm32wb_usbd_dfu_cdc_ep0_data[64];

const stm32wb_usbd_info_t stm32wb_usbd_dfu_cdc_info =
{
    stm32wb_usbd_dfu_cdc_ep0_data,
    sizeof(stm32wb_usbd_dfu_cdc_ep0_data),
    STM32WB_USBD_DFU_CDC_STRING_COUNT,
    STM32WB_USBD_DFU_CDC_FUNCTION_COUNT,
    stm32wb_usbd_dfu_cdc_configuration,
    stm32wb_usbd_dfu_cdc_bos,
    stm32wb_usbd_dfu_cdc_msos20,
    stm32wb_usbd_dfu_cdc_strings,
    stm32wb_usbd_dfu_cdc_functions,
};

/********************************************************************************************************************************************/


static const uint8_t stm32wb_usbd_dfu_cdc_msc_configuration[STM32WB_USBD_DFU_CDC_MSC_CONFIGURATION_SIZE] =
{
    /**** Configuration Descriptor ****/
    9,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_CONFIGURATION,                           /* bDescriptorType */
    STM32WB_USBD_DFU_CDC_MSC_CONFIGURATION_SIZE & 255,           /* wTotalLength */
    STM32WB_USBD_DFU_CDC_MSC_CONFIGURATION_SIZE >> 8,
    STM32WB_USBD_DFU_CDC_MSC_INTERFACE_COUNT,                    /* bNumInterfaces */
    0x01,                                                        /* bConfigurationValue */
    0x00,                                                        /* iConfiguration */
    0xa0,                                                        /* bmAttributes */
    0xfa,                                                        /* bMaxPower */

    /**** DFU Interface ****/
    9,                                                           /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x00,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x00,                                                        /* bNumEndpoints */
    0xfe,                                                        /* bInterfaceClass */
    0x01,                                                        /* bInterfaceSubClass */
    0x01,                                                        /* nInterfaceProtocol */
    0x04,                                                        /* iInterface */
    
    /**** DFU Descriptor ****/
    0x09,                                                        /* blength = 9 Bytes */
    0x21,                                                        /* bDescriptorType */
    0x0b,                                                        /* bmAttribute */
    0xff, 0x00,                                                  /* wDetachTimeOut= 255 ms */
    0x00, 0x04,                                                  /* wTransferSize = 1024 Bytes */
    0x1a, 0x01,                                                  /* bcdDFUVersion */
    
    /**** IAD to associate the two CDC interfaces ****/
    8,                                                           /* bLength */
    0x0b,                                                        /* bDescriptorType */
    0x01,                                                        /* bFirstInterface */
    0x02,                                                        /* bInterfaceCount */
    0x02,                                                        /* bFunctionClass */
    0x02,                                                        /* bFunctionSubClass */
    0x01,                                                        /* bFunctionProtocol */
    0x05,                                                        /* iFunction */

    /**** CDC Control Interface ****/
    9,                                                           /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x01,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x01,                                                        /* bNumEndpoints */
    0x02,                                                        /* bInterfaceClass */
    0x02,                                                        /* bInterfaceSubClass */
    0x01,                                                        /* bInterfaceProtocol */
    0x06,                                                        /* iInterface */
  
    /**** CDC Header ****/
    5,                                                           /* bLength */
    0x24,                                                        /* bDescriptorType */
    0x00,                                                        /* bDescriptorSubtype */
    0x10,                                                        /* bcdCDC */
    0x01,
  
    /**** CDC Call Management ****/
    5,                                                           /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x01,                                                        /* bDescriptorSubtype */
    0x00,                                                        /* bmCapabilities */
    0x02,                                                        /* bDataInterface */
  
    /**** CDC ACM ****/
    4,                                                           /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x02,                                                        /* bDescriptorSubtype */
    0x06,                                                        /* bmCapabilities */
  
    /**** CDC Union ****/
    5,                                                           /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x06,                                                        /* bDescriptorSubtype */
    0x01,                                                        /* bMasterInterface */
    0x02,                                                        /* bSlaveInterface0 */
  
    /**** CDC Control Endpoint ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_CDC_CONTROL_EP_ADDR,                            /* bEndpointAddress */
    USB_EP_TYPE_INTERRUPT,                                       /* bmAttributes */
    STM32WB_USBD_CDC_CONTROL_MAX_PACKET_SIZE & 255,              /* wMaxPacketSize */
    STM32WB_USBD_CDC_CONTROL_MAX_PACKET_SIZE >> 8,
    STM32WB_USBD_CDC_CONTROL_INTERVAL,                           /* bInterval */ 

    /**** CDC Data Interface ****/
    9,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_INTERFACE,                               /* bDescriptorType */
    0x02,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x02,                                                        /* bNumEndpoints */
    0x0a,                                                        /* bInterfaceClass */
    0x00,                                                        /* bInterfaceSubClass */
    0x00,                                                        /* bInterfaceProtocol */
    0x07,                                                        /* iInterface */

    /**** CDC Data Endpoint IN ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_CDC_DATA_IN_EP_ADDR,                            /* bEndpointAddress */
    USB_EP_TYPE_BULK,                                            /* bmAttributes */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE & 255,                 /* wMaxPacketSize */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE >> 8,
    0,                                                           /* bInterval */

    /**** CDC Data Endpoint OUT ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_CDC_DATA_OUT_EP_ADDR,                           /* bEndpointAddress */
    USB_EP_TYPE_BULK,                                            /* bmAttributes */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE & 255,                 /* wMaxPacketSize */
    STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE >> 8,
    0,                                                           /* bInterval */
  
    /**** MSC Interface ****/
    9,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_INTERFACE,                               /* bDescriptorType */
    0x03,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x02,                                                        /* bNumEndpoints */
    0x08,                                                        /* bInterfaceClass */
    0x06,                                                        /* bInterfaceSubClass */
    0x50,                                                        /* nInterfaceProtocol */
    0x08,                                                        /* iInterface */

    /**** MSC Endpoint IN ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_MSC_DATA_IN_EP_ADDR,                            /* bEndpointAddress */
    USB_EP_TYPE_BULK,                                            /* bmAttributes */
    STM32WB_USBD_MSC_DATA_MAX_PACKET_SIZE & 255,                 /* wMaxPacketSize */
    STM32WB_USBD_MSC_DATA_MAX_PACKET_SIZE >> 8,
    0,                                                           /* bInterval */

    /**** MSC Endpoint OUT ****/
    7,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT,                                /* bDescriptorType */
    STM32WB_USBD_MSC_DATA_OUT_EP_ADDR,                           /* bEndpointAddress */
    USB_EP_TYPE_BULK,                                            /* bmAttributes */
    STM32WB_USBD_MSC_DATA_MAX_PACKET_SIZE & 255,                 /* wMaxPacketSize */
    STM32WB_USBD_MSC_DATA_MAX_PACKET_SIZE >> 8,
    0,                                                           /* bInterval */
};

static const char * const stm32wb_usbd_dfu_cdc_msc_strings[STM32WB_USBD_DFU_CDC_MSC_STRING_COUNT] =
{
    "STM Device in DFU Runtime",              // 4
    "Serial",                                 // 5
    "CDC Control",                            // 6
    "CDC Data",                               // 7
    "Mass Storage",                           // 8
};

static const stm32wb_usbd_function_t stm32wb_usbd_dfu_cdc_msc_functions[STM32WB_USBD_DFU_CDC_MSC_FUNCTION_COUNT] =
{
    {
	&stm32wb_usbd_dfu_runtime_interface,
	NULL,
	0,
	STM32WB_USBD_DFU_INTERFACE_COUNT,
	0,
    },
    {
	&stm32wb_usbd_cdc_interface,
	NULL,
	0,
	STM32WB_USBD_CDC_INTERFACE_COUNT,
	(STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_CONTROL_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_DATA_IN_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_CDC_DATA_OUT_EP_ADDR)),
    },
    {
	&stm32wb_usbd_msc_interface,
	NULL,
	0,
	STM32WB_USBD_MSC_INTERFACE_COUNT,
	(STM32WB_USBD_EP_MASK(STM32WB_USBD_MSC_DATA_IN_EP_ADDR) | STM32WB_USBD_EP_MASK(STM32WB_USBD_MSC_DATA_OUT_EP_ADDR)),
    },
};

static const uint8_t stm32wb_usbd_dfu_cdc_msc_bos[] = {
    // BOS
    0x05,                                            // bLength
    USB_DESCRIPTOR_TYPE_BOS,                         // bDescriptorType
    0x21, 0x00,                                      // wTotalLength
    0x01,                                            // bNumDeviceCaps

    // Microsoft OS 2.0 platform capability
    0x1c,                                            // bLength
    USB_DESCRIPTOR_TYPE_DEVICE_CAPABILITY,           // bDescriptorType
    USB_DEVICE_CAPABILITY_TYPE_PLATFORM,             // bDevCapabilityType
    0x00,                                            // bReserved
    0xdf, 0x60, 0xdd, 0xd8, 0x89, 0x45, 0xc7, 0x4c,  // platformCapabilityUUID[16]     
    0x9c, 0xd2, 0x65, 0x9d, 0x9e, 0x64, 0x8a, 0x9f,  // platformCapabilityUUID[16]     
    0x00, 0x00, 0x03, 0x06,                          // dwWindowsVersion (Windows 8.1)
    0xb2, 0x00,                                      // wLength (MSOS20)
    USB_REQ_MS_VENDOR_CODE,                          // bMS_VendorCode
    0x00,                                            // bAltEnumCode
};

static const uint8_t stm32wb_usbd_dfu_cdc_msc_msos20[] = {
    // Microsoft OS 2.0 descriptor set header
    0x0a, 0x00,                                      // wLength
    0x00, 0x00,                                      // wDescriptorType
    0x00, 0x00, 0x03, 0x06,                          // dwWindowsVersion (Windows 8.1)
    0xb2, 0x00,                                      // wTotalLength

    // Microsoft OS 2.0 configuration subset header
    0x08, 0x00,                                      // wLength
    0x01, 0x00,                                      // wDescriptorType
    0x00,                                            // bConfigurationValue
    0x00,                                            // bReserved
    0xa8, 0x00,                                      // wTotalLength (size of entire configuration subset)

    // Microsoft OS 2.0 function subset header
    0x08, 0x00,                                      // wLength
    0x02, 0x00,                                      // wDescriptorType
    0x00,                                            // bFirstInterface
    0x00,                                            // bReserved
    0xa0, 0x00,                                      // wTotalLength  (size of entine function subset)

    // Microsoft OS 2.0 compatible ID
    0x14, 0x00,                                      // wLength
    0x03, 0x00,                                      // wDescriptorType
    'W',  'I',  'N',  'U',  'S',  'B',  0x00, 0x00,  // compatible ID
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // subCompatibleID
                                        
    // Microsoft OS 2.0 registry property
    0x84, 0x00,                                      // wLength
    0x04, 0x00,                                      // wDescriptorType
    0x07, 0x00,                                      // wPropertyDataType
    0x2a, 0x00,                                      // wPropertyNameLength
    'D',  0x00, 'e',  0x00, 'v',  0x00, 'i',  0x00,  // bPropertyName "DeviceInterfaceGUIDs"
    'c',  0x00, 'e',  0x00, 'I',  0x00, 'n',  0x00,    
    't',  0x00, 'e',  0x00, 'r',  0x00, 'f',  0x00,    
    'a',  0x00, 'c',  0x00, 'e',  0x00, 'G',  0x00,    
    'U',  0x00, 'I',  0x00, 'D',  0x00, 's',  0x00,        
    0x00, 0x00,
    0x50, 0x00,                                      // wPropertyDataLength
    '{',  0x00, 'D',  0x00, 'E',  0x00, '5',  0x00,  // bProperyData "{DE50DD7B-4DEF-4F9F-98AA-88144B16A383}"
    '0',  0x00, 'D',  0x00, 'D',  0x00, '7',  0x00,
    'B',  0x00, '-',  0x00, '4',  0x00, 'D',  0x00,
    'E',  0x00, 'F',  0x00, '-',  0x00, '4',  0x00,
    'F',  0x00, '9',  0x00, 'F',  0x00, '-',  0x00,
    '9',  0x00, '8',  0x00, 'A',  0x00, 'A',  0x00,
    '-',  0x00, '8',  0x00, '8',  0x00, '1',  0x00,
    '4',  0x00, '4',  0x00, 'B',  0x00, '1',  0x00,
    '6',  0x00, 'A',  0x00, '3',  0x00, '8',  0x00,
    '3',  0x00, '}',  0x00,
    0x00, 0x00, 0x00, 0x00,
};

static uint8_t stm32wb_usbd_dfu_cdc_msc_ep0_data[64];

const stm32wb_usbd_info_t stm32wb_usbd_dfu_cdc_msc_info =
{
    stm32wb_usbd_dfu_cdc_msc_ep0_data,
    sizeof(stm32wb_usbd_cdc_msc_ep0_data),
    STM32WB_USBD_DFU_CDC_MSC_STRING_COUNT,
    STM32WB_USBD_DFU_CDC_MSC_FUNCTION_COUNT,
    stm32wb_usbd_dfu_cdc_msc_configuration,
    stm32wb_usbd_dfu_cdc_msc_bos,
    stm32wb_usbd_dfu_cdc_msc_msos20,
    stm32wb_usbd_dfu_cdc_msc_strings,
    stm32wb_usbd_dfu_cdc_msc_functions,
};
