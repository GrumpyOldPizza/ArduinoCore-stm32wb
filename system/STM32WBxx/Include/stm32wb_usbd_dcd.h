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

#if !defined(_STM32WB_USBD_DCD_H)
#define _STM32WB_USBD_DCD_H

#ifdef __cplusplus
extern "C" {
#endif

#define STM32WB_USBD_DCD_IRQ_PRIORITY             ARMV7M_IRQ_PRIORITY_USB

#define STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE      64

#define STM32WB_USBD_DCD_OPTION_DETECT            0x00000001
  
#define STM32WB_USBD_DCD_EVENT_EP0_REQUEST        0x00000001
#define STM32WB_USBD_DCD_EVENT_EP0_SETUP          0x00000002
#define STM32WB_USBD_DCD_EVENT_EP0_DATA_IN        0x00000004
#define STM32WB_USBD_DCD_EVENT_EP0_DATA_OUT       0x00000008
#define STM32WB_USBD_DCD_EVENT_DETECT             0x00000010
#define STM32WB_USBD_DCD_EVENT_SUSPEND            0x00000020
#define STM32WB_USBD_DCD_EVENT_RESUME             0x00000040
#define STM32WB_USBD_DCD_EVENT_RESET              0x00000080
#define STM32WB_USBD_DCD_EVENT_SOF                0x00000100
  
typedef void (*stm32wb_usbd_dcd_event_callback_t)(void *context, uint32_t events);

typedef void (*stm32wb_usbd_dcd_ep_in_callback_t)(void *context, uint8_t ep_addr);

typedef void (*stm32wb_usbd_dcd_ep_out_callback_t)(void *context, uint8_t ep_addr, uint16_t count);

extern bool stm32wb_usbd_dcd_configure(void);
extern bool stm32wb_usbd_dcd_enable(uint8_t *setup, uint32_t options, stm32wb_usbd_dcd_event_callback_t callback, void *context);
extern bool stm32wb_usbd_dcd_disable(void);
extern bool stm32wb_usbd_dcd_connect(void);
extern bool stm32wb_usbd_dcd_disconnect(void);
extern uint32_t stm32wb_usbd_dcd_bcd_status(void);
extern bool stm32wb_usbd_dcd_reset(void);
extern bool stm32wb_usbd_dcd_wakeup(void);
extern bool stm32wb_usbd_dcd_set_address(uint8_t address);
extern bool stm32wb_usbd_dcd_sof_enable(void);
extern bool stm32wb_usbd_dcd_sof_disable(void);
extern bool stm32wb_usbd_dcd_ep0_transmit(const uint8_t *data, uint16_t length);
extern bool stm32wb_usbd_dcd_ep0_receive(uint8_t *data, uint16_t length);
extern uint32_t stm32wb_usbd_dcd_ep0_count(void);
extern bool stm32wb_usbd_dcd_ep0_stall(void);
extern bool stm32wb_usbd_dcd_ep0_request(void);
extern bool stm32wb_usbd_dcd_ep_configure(uint8_t ep_addr, uint8_t type, uint16_t size);
extern bool stm32wb_usbd_dcd_ep_enable(uint8_t ep_addr);
extern bool stm32wb_usbd_dcd_ep_disable(uint8_t ep_addr);
extern bool stm32wb_usbd_dcd_ep_stall(uint8_t ep_addr);
extern bool stm32wb_usbd_dcd_ep_unstall(uint8_t ep_addr);
extern bool stm32wb_usbd_dcd_ep_is_stalled(uint8_t ep_addr);
extern bool stm32wb_usbd_dcd_ep_transmit(uint8_t ep_addr, const uint8_t *data, uint16_t length, stm32wb_usbd_dcd_ep_in_callback_t callback, void *context);
extern bool stm32wb_usbd_dcd_ep_receive(uint8_t ep_addr, uint8_t *data, uint16_t length, stm32wb_usbd_dcd_ep_out_callback_t callback, void *context);
extern bool stm32wb_usbd_dcd_ep_flush(uint8_t ep_addr);

#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_USBD_DCD_H */
