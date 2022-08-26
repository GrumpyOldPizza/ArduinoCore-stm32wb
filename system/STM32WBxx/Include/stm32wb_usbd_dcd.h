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

#define STM32WB_USBD_DCD_LSE_SUPPORTED                0
#define STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED      1
#define STM32WB_USBD_DCD_LPM_SUPPORTED                1
#define STM32WB_USBD_DCD_DBL_SUPPORTED                1
#define STM32WB_USBD_DCD_SOF_SUPPORTED                0
#define STM32WB_USBD_DCD_ISO_SUPPORTED                0
  
#define STM32WB_USBD_DCD_IRQ_PRIORITY                 ARMV7M_IRQ_PRIORITY_USB

#define STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE          64

#define STM32WB_USBD_DCD_LPM_STATE_L0                 0  
#define STM32WB_USBD_DCD_LPM_STATE_L1                 1  
#define STM32WB_USBD_DCD_LPM_STATE_L2                 2  
#define STM32WB_USBD_DCD_LPM_STATE_L3                 3  
  
#define STM32WB_USBD_DCD_EVENT_CONNECT                0x00000001
#define STM32WB_USBD_DCD_EVENT_DISCONNECT             0x00000002
#define STM32WB_USBD_DCD_EVENT_SLEEP                  0x00000004
#define STM32WB_USBD_DCD_EVENT_SUSPEND                0x00000008
#define STM32WB_USBD_DCD_EVENT_RESUME                 0x00000010
#define STM32WB_USBD_DCD_EVENT_RESET                  0x00000020
#define STM32WB_USBD_DCD_EVENT_EP0_SETUP              0x00008000
#define STM32WB_USBD_DCD_EVENT_EP0_DATA_OUT           0x00010000
#define STM32WB_USBD_DCD_EVENT_EP0_DATA_IN            0x01000000

#define STM32WB_USBD_DCD_EP_TYPE_CONTROL              0x00
#define STM32WB_USBD_DCD_EP_TYPE_ISO                  0x01
#define STM32WB_USBD_DCD_EP_TYPE_BULK                 0x02
#define STM32WB_USBD_DCD_EP_TYPE_INTERRUPT            0x03
#define STM32WB_USBD_DCD_EP_TYPE_BULK_DBL             0x06

#define STM32WB_USBD_DCD_EP_MASK(_ep_addr)            ((((_ep_addr) & 0x80) ? 0x00000001 : 0x00010000) << ((_ep_addr) & 0x0f))

typedef void (*stm32wb_usbd_dcd_event_callback_t)(void *context, uint32_t events);

typedef void (*stm32wb_usbd_dcd_sof_callback_t)(void *context, uint32_t frameno);

typedef void (*stm32wb_usbd_dcd_ep_in_callback_t)(void *context, uint8_t ep_addr);

typedef void (*stm32wb_usbd_dcd_ep_out_callback_t)(void *context, uint8_t ep_addr, uint16_t count);
  
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)

typedef void (*stm32wb_usbd_dcd_ep_iso_callback_t)(void *context, uint8_t ep_addr, uint16_t count);
  
typedef struct _stm32wb_usbd_dcd_fifo_t {
    uint8_t                           *data;
    uint16_t                          size;
    volatile uint16_t                 head;
    volatile uint16_t                 tail;
    volatile uint16_t                 count;
} stm32wb_usbd_dcd_fifo_t;
  
#endif /* STM32WB_USBD_DCD_ISO_SUPPORTED == 1 */

extern bool stm32wb_usbd_dcd_configure(void);
extern bool stm32wb_usbd_dcd_enable(uint8_t *setup, stm32wb_usbd_dcd_event_callback_t callback, void *context);
extern bool stm32wb_usbd_dcd_disable(void);
extern bool stm32wb_usbd_dcd_connect(bool disconnect);
extern bool stm32wb_usbd_dcd_disconnect(void);
extern bool stm32wb_usbd_dcd_reset(void);
extern bool stm32wb_usbd_dcd_address(uint8_t address);
#if (STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1)
extern bool stm32wb_usbd_dcd_wakeup(void);
#endif /* STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1 */
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
extern bool stm32wb_usbd_dcd_lpm_reference(uint32_t reference);
extern void stm32wb_usbd_dcd_lpm_unreference(uint32_t reference);
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
extern bool stm32wb_usbd_dcd_ep0_transmit(const uint8_t *data, uint16_t length);
extern bool stm32wb_usbd_dcd_ep0_receive(uint8_t *data, uint16_t length);
extern uint32_t stm32wb_usbd_dcd_ep0_count(void);
extern bool stm32wb_usbd_dcd_ep0_stall(void);
extern bool stm32wb_usbd_dcd_ep_configure(uint8_t ep_addr, uint8_t type, uint16_t size, uint16_t pma_address, uint16_t *p_pma_address_return);
extern bool stm32wb_usbd_dcd_ep_enable(uint8_t ep_addr);
extern bool stm32wb_usbd_dcd_ep_disable(uint8_t ep_addr);
extern bool stm32wb_usbd_dcd_ep_stall(uint8_t ep_addr);
extern bool stm32wb_usbd_dcd_ep_unstall(uint8_t ep_addr);
extern bool stm32wb_usbd_dcd_ep_is_stalled(uint8_t ep_addr);
extern bool stm32wb_usbd_dcd_ep_transmit(uint8_t ep_addr, const uint8_t *data, uint16_t length, stm32wb_usbd_dcd_ep_in_callback_t callback, void *context);
extern bool stm32wb_usbd_dcd_ep_receive(uint8_t ep_addr, uint8_t *data, uint16_t length, stm32wb_usbd_dcd_ep_out_callback_t callback, void *context);
extern bool stm32wb_usbd_dcd_ep_flush(uint8_t ep_addr);
#if (STM32WB_USBD_DCD_SOF_SUPPORTED == 1)
extern bool stm32wb_usbd_dcd_sof_enable(stm32wb_usbd_dcd_sof_callback_t callback, void *context);
extern bool stm32wb_usbd_dcd_sof_disable(void);
#endif /* STM32WB_USBD_DCD_SOF_SUPPORTED == 1 */
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
extern uint16_t stm32wb_usbd_dcd_fifo_count(stm32wb_usbd_dcd_fifo_t *fifo);
extern uint16_t stm32wb_usbd_dcd_fifo_write(stm32wb_usbd_dcd_fifo_t *fifo, const uint8_t *data, uint16_t length);
extern uint16_t stm32wb_usbd_dcd_fifo_read(stm32wb_usbd_dcd_fifo_t *fifo, uint8_t *data, uint16_t length);
extern bool stm32wb_usbd_dcd_ep_fifo_attach(uint8_t ep_addr, stm32wb_usbd_dcd_fifo_t *fifo, uint8_t *data, uint16_t size);
extern bool stm32wb_usbd_dcd_ep_fifo_detach(uint8_t ep_addr);
extern bool stm32wb_usbd_dcd_ep_iso_start(uint8_t ep_addr, stm32wb_usbd_dcd_ep_iso_callback_t callback, void *context);
extern bool stm32wb_usbd_dcd_ep_iso_stop(uint8_t ep_addr);
extern bool stm32wb_usbd_dcd_ep_iso_transmit(uint8_t ep_addr, uint16_t length);
#endif /* STM32WB_USBD_DCD_ISO_SUPPORTED == 1 */

#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_USBD_DCD_H */
