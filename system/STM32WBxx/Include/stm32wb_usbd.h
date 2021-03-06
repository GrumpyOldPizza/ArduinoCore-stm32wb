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

#if !defined(_STM32WB_USBD_H)
#define _STM32WB_USBD_H

#ifdef __cplusplus
extern "C" {
#endif

#define USB_STATE_POWERED                              0
#define USB_STATE_DEFAULT                              1
#define USB_STATE_ADDRESSED                            2
#define USB_STATE_CONFIGURED                           3
  
#define USB_REQ_RECIPIENT_MASK                         0x1f    /* 0x04..0x1f are illegal */
#define USB_REQ_RECIPIENT_DEVICE                       0x00
#define USB_REQ_RECIPIENT_INTERFACE                    0x01
#define USB_REQ_RECIPIENT_ENDPOINT                     0x02
#define USB_REQ_TYPE_MASK                              0x60
#define USB_REQ_TYPE_STANDARD                          0x00    /* 0x60 is illegal */
#define USB_REQ_TYPE_CLASS                             0x20
#define USB_REQ_TYPE_VENDOR                            0x40
#define USB_REQ_DIRECTION_MASK                         0x80
#define USB_REQ_DIRECTION_HOST_TO_DEVICE               0x00
#define USB_REQ_DIRECTION_DEVICE_TO_HOST               0x80

#define USB_REQ_CODE_GET_STATUS                        0       /* DEVICE, INTERFACE, ENDPOINT */
#define USB_REQ_CODE_CLEAR_FEATURE                     1       /* DEVICE, INTERFACE, ENDPOINT */
#define USB_REQ_CODE_SET_FEATURE                       3       /* DEVICE, INTERFACE, ENDPOINT */
#define USB_REQ_CODE_SET_ADDRESS                       5       /* DEVICE                      */
#define USB_REQ_CODE_GET_DESCRIPTOR                    6       /* DEVICE                      */
#define USB_REQ_CODE_SET_DESCRIPTOR                    7       /* DEVICE                      */
#define USB_REQ_CODE_GET_CONFIGURATION                 8       /* DEVICE                      */
#define USB_REQ_CODE_SET_CONFIGURATION                 9       /* DEVICE                      */
#define USB_REQ_CODE_GET_INTERFACE                     10      /*        INTERFACE            */
#define USB_REQ_CODE_SET_INTERFACE                     11      /*        INTERFACE            */
#define USB_REQ_CODE_SYNCH_FRAME                       12      /*                    ENDPOINT */

#define USB_STATUS_DEVICE_SELF_POWERED                 0x0001  /* GET_STATUS(DEVICE) */
#define USB_STATUS_DEVICE_REMOTE_WAKEUP                0x0002  /* GET_STATUS(DEVICE) */

#define USB_STATUS_ENDPOINT_HALT                       0x0001  /* GET_STATUS(ENDPOINT) */

#define USB_FEATURE_ENDPOINT_HALT                      0
#define USB_FEATURE_DEVICE_REMOTE_WAKEUP               1
#define USB_FEATURE_DEVICE_TEST_MODE                   2

#define USB_INTERFACE_MASK                             0x00ff

#define USB_ENPOINT_NUMBER_MASK                        0x000f
#define USB_ENPOINT_DIRECTION_MASK                     0x0080
#define USB_ENPOINT_DIRECTION_IN                       0x0080
#define USB_ENPOINT_DIRECTION_OUT                      0x0000

#define USB_DESCRIPTOR_TYPE_DEVICE                     1
#define USB_DESCRIPTOR_TYPE_CONFIGURATION              2
#define USB_DESCRIPTOR_TYPE_STRING                     3
#define USB_DESCRIPTOR_TYPE_INTERFACE                  4       /* not used, return error               */
#define USB_DESCRIPTOR_TYPE_ENDPOINT                   5       /* not used, return error               */
#define USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER           6       /* MUST return error for FS only device */
#define USB_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION  7       /* MUST return error for FS only device */
#define USB_DESCRIPTOR_TYPE_BOS                        15
#define USB_DESCRIPTOR_TYPE_DEVICE_CAPABILITY          16

#define USB_DEVICE_CAPABILITY_TYPE_USB20_EXTENSION     2
#define USB_DEVICE_CAPABILITY_TYPE_PLATFORM            5
  
#define USB_EP_TYPE_CONTROL                            0
#define USB_EP_TYPE_ISO                                1
#define USB_EP_TYPE_BULK                               2
#define USB_EP_TYPE_INTERRUPT                          3
  
#define USB_STRING_INDEX_LANGID                        0
#define USB_STRING_INDEX_MANUFACTURER                  1
#define USB_STRING_INDEX_PRODUCT                       2
#define USB_STRING_INDEX_SERIAL                        3
#define USB_STRING_INDEX_CUSTOM                        4

#define USB_REQ_MS_VENDOR_CODE                         1       /* DEVICE                      */
#define MS_OS_2O_DESCRIPTOR_INDEX                      7
  
typedef struct _stm32wb_usbd_request_t {
    uint8_t   bmRequestType;
    uint8_t   bRequest;
    uint16_t  wValue;
    uint16_t  wIndex;
    uint16_t  wLength;
} stm32wb_usbd_request_t;
  
#define STM32WB_USBD_EVENT_ATTACH                 0x00000001
#define STM32WB_USBD_EVENT_DETACH                 0x00000002
#define STM32WB_USBD_EVENT_DETECT                 0x00000004
#define STM32WB_USBD_EVENT_CONNECT                0x00000008 // initial reset
#define STM32WB_USBD_EVENT_SUSPEND                0x00000010
#define STM32WB_USBD_EVENT_RESUME                 0x00000020

#define STM32WB_USBD_VBUS_STATUS_NOT_PRESENT      0
#define STM32WB_USBD_VBUS_STATUS_PRESENT          1

#define STM32WB_USBD_BCD_STATUS_UNKNOWN           0          /* UNKNOWN                     */
#define STM32WB_USBD_BCD_STATUS_PORT              1          /* USB PORT, 100mA/500mA       */
#define STM32WB_USBD_BCD_STATUS_CHARGER           2          /* CHARGER, 1500mA             */
#define STM32WB_USBD_BCD_STATUS_PORT_AND_CHARGER  3          /* USB PORT + CHARGER, 1500mA  */
  
#define STM32WB_USBD_EP_TYPE_CONTROL              0
#define STM32WB_USBD_EP_TYPE_ISO                  1
#define STM32WB_USBD_EP_TYPE_BULK                 2
#define STM32WB_USBD_EP_TYPE_INTERRUPT            3

#define STM32WB_USBD_EP_MASK(_ep_addr)            ((((_ep_addr) & 0x80) ? 0x00000001 : 0x00010000) << ((_ep_addr) & 0x0f))
  
typedef void (*stm32wb_usbd_event_callback_t)(void *context, uint32_t events);
  
typedef struct _stm32wb_usbd_device_t {
    uint16_t                             vid;
    uint16_t                             pid;
    uint16_t                             did;
    const char                           *manufacturer;
    const char                           *product;
} stm32wb_usbd_device_t;
    
typedef void (*stm32wb_usbd_configure_routine_t)(void *context, uint16_t if_base, uint16_t options);
typedef void (*stm32wb_usbd_start_routine_t)(void *context);
typedef void (*stm32wb_usbd_stop_routine_t)(void *context);
typedef bool (*stm32wb_usbd_request_routine_t)(void *context, int state, const stm32wb_usbd_request_t *request, uint8_t *data, const uint8_t **p_data_return, uint32_t *p_length_return);
typedef void (*stm32wb_usbd_suspend_routine_t)(void *context);
typedef void (*stm32wb_usbd_resume_routine_t)(void *context);
typedef void (*stm32wb_usbd_sof_routine_t)(void *context);

typedef struct _stm32wb_usbd_class_interface_t {
    stm32wb_usbd_configure_routine_t     configure;
    stm32wb_usbd_start_routine_t         start;
    stm32wb_usbd_stop_routine_t          stop;
    stm32wb_usbd_request_routine_t       request;
    stm32wb_usbd_suspend_routine_t       suspend;
    stm32wb_usbd_resume_routine_t        resume;
    stm32wb_usbd_sof_routine_t           sof;
} stm32wb_usbd_class_interface_t;

typedef struct _stm32wb_usbd_function_t {
    const stm32wb_usbd_class_interface_t *interface;
    void                                 *context;
    uint16_t                             options;
    uint16_t                             if_count;
    uint32_t                             ep_mask;
} stm32wb_usbd_function_t;

typedef struct _stm32wb_usbd_info_t {
    uint8_t                              *ep0_data;
    uint16_t                             ep0_size;
    uint8_t                              string_count;
    uint8_t                              function_count;
    const uint8_t                        *configuration;
    const uint8_t                        *bos;
    const uint8_t                        *msos20;
    const char * const                   *string_table;
    const stm32wb_usbd_function_t        *function_table;
} stm32wb_usbd_info_t;

typedef struct _stm32wb_usbd_params_t {
    uint8_t                              priority;
    uint16_t                             pin_vbus;
} stm32wb_usbd_params_t;

extern bool stm32wb_usbd_configure(const stm32wb_usbd_device_t *device, const stm32wb_usbd_info_t *info, const stm32wb_usbd_params_t *params);
extern bool stm32wb_usbd_enable(stm32wb_usbd_event_callback_t callback, void *context);
extern bool stm32wb_usbd_disable(void);
extern bool stm32wb_usbd_attach(void);
extern bool stm32wb_usbd_detach(void);
extern bool stm32wb_usbd_wakeup(void);

extern uint32_t stm32wb_usbd_vbus_status(void);
extern uint32_t stm32wb_usbd_bcd_status(void);
  
#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_USBD_H */
