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

#if !defined(_STM32WB_USBD_CDC_H)
#define _STM32WB_USBD_CDC_H

#ifdef __cplusplus
extern "C" {
#endif

#define STM32WB_USBD_CDC_CONTROL_EP_ADDR         0x81
#define STM32WB_USBD_CDC_DATA_IN_EP_ADDR         0x82
#define STM32WB_USBD_CDC_DATA_OUT_EP_ADDR        0x02

#define STM32WB_USBD_CDC_CONTROL_MAX_PACKET_SIZE 16
#define STM32WB_USBD_CDC_DATA_MAX_PACKET_SIZE    64
#define STM32WB_USBD_CDC_CONTROL_INTERVAL        50
   
#define STM32WB_USBD_CDC_LINE_CODING_DTE_RATE    115200
#define STM32WB_USBD_CDC_LINE_CODING_CHAR_FORMAT USB_CDC_CHAR_FORMAT_STOP_1
#define STM32WB_USBD_CDC_LINE_CODING_PARITY_TYPE USB_CDC_PARITY_TYPE_NONE
#define STM32WB_USBD_CDC_LINE_CODING_DATA_BITS   8
   
#define USB_REQ_CDC_SET_LINE_CODING              0x20
#define USB_REQ_CDC_GET_LINE_CODING              0x21
#define USB_REQ_CDC_SET_CONTROL_LINE_STATE       0x22
#define USB_REQ_CDC_SEND_BREAK                   0x23

#define USB_REQ_CDC_SERIAL_STATE                 0x20
   
#define USB_CDC_CHAR_FORMAT_STOP_1               0
#define USB_CDC_CHAR_FORMAT_STOP_1_5             1
#define USB_CDC_CHAR_FORMAT_STOP_2               2
   
#define USB_CDC_PARITY_TYPE_NONE                 0
#define USB_CDC_PARITY_TYPE_ODD                  1
#define USB_CDC_PARITY_TYPE_EVEN                 2
#define USB_CDC_PARITY_TYPE_MARK                 3
#define USB_CDC_PARITY_TYPE_SPACE                4
   
#define USB_CDC_LINE_STATE_DTR                   0x01
#define USB_CDC_LINE_STATE_RTS                   0x02

#define USB_CDC_SERIAL_STATE_DCD                 0x01
#define USB_CDC_SERIAL_STATE_DSR                 0x02
#define USB_CDC_SERIAL_STATE_BREAK               0x04
#define USB_CDC_SERIAL_STATE_RI                  0x08
#define USB_CDC_SERIAL_STATE_FRAMING             0x10
#define USB_CDC_SERIAL_STATE_PARITY              0x20
#define USB_CDC_SERIAL_STATE_OVERRUN             0x40

typedef struct _stm32wb_usbd_cdc_line_coding_t {
    volatile int32_t               dwDTERate;
    volatile uint8_t               bCharFormat;
    volatile uint8_t               bParityType;
    volatile uint8_t               bDataBits;
} stm32wb_usbd_cdc_line_coding_t;

#define STM32WB_USBD_CDC_EVENT_LINE_CODING       0x00000001
#define STM32WB_USBD_CDC_EVENT_LINE_STATE        0x00000002
#define STM32WB_USBD_CDC_EVENT_BREAK_STATE       0x00000004
#define STM32WB_USBD_CDC_EVENT_OVERRUN           0x00000008
#define STM32WB_USBD_CDC_EVENT_RECEIVE           0x00000010

typedef void (*stm32wb_usbd_cdc_event_callback_t)(void *context, uint32_t events);
typedef void (*stm32wb_usbd_cdc_done_callback_t)(void *context);

#define STM32WB_USBD_CDC_STATUS_SUCCESS          0
#define STM32WB_USBD_CDC_STATUS_FAILURE          1
#define STM32WB_USBD_CDC_STATUS_BUSY             255

extern bool stm32wb_usbd_cdc_enable(uint8_t *rx_data, uint32_t rx_size, stm32wb_usbd_cdc_event_callback_t callback, void *context);
extern bool stm32wb_usbd_cdc_disable(void);
extern void stm32wb_usbd_cdc_line_coding(stm32wb_usbd_cdc_line_coding_t *p_line_coding);
extern uint16_t stm32wb_usbd_cdc_line_state(void);
extern uint16_t stm32wb_usbd_cdc_break_state(void);
extern bool stm32wb_usbd_cdc_serial_state(uint16_t serial_state);
extern uint32_t stm32wb_usbd_cdc_count(void);
extern int32_t stm32wb_usbd_cdc_peek(void);
extern uint32_t stm32wb_usbd_cdc_read(uint8_t *rx_data, uint32_t rx_count);
extern bool stm32wb_usbd_cdc_transmit(const uint8_t *tx_data, uint32_t tx_count, volatile uint8_t *p_status_return, stm32wb_usbd_cdc_done_callback_t callback, void *context);

extern const stm32wb_usbd_class_interface_t stm32wb_usbd_cdc_interface;
  
#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_USBD_CDC_H */
