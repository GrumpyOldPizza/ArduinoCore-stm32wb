/*
 * Copyright (c) 2019-2021 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32WB_USBD_MSC_H)
#define _STM32WB_USBD_MSC_H

#ifdef __cplusplus
 extern "C" {
#endif

#define STM32WB_USBD_MSC_INTERFACE_COUNT        1
   
#define STM32WB_USBD_MSC_DATA_IN_EP_ADDR         0x83
#define STM32WB_USBD_MSC_DATA_OUT_EP_ADDR        0x03

#define STM32WB_USBD_MSC_DATA_MAX_PACKET_SIZE    64

#define STM32WB_USBD_MSC_DATA_BLOCK_SIZE         512

extern void MSC_BOT_Notify(uint8_t lun, int acquire);

extern const stm32wb_usbd_class_t stm32wb_usbd_msc_class;
   
#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_USBD_MSC_H */
