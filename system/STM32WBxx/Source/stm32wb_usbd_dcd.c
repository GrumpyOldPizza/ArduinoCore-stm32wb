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
#include "stm32wb_system.h"
#include "stm32wb_lptim.h"
#include "stm32wb_hsem.h"
#include "stm32wb_usbd.h"
#include "stm32wb_usbd_dcd.h"

#define STM32WB_USBD_DCD_ISO_SUPPORTED             1

#define STM32WB_USBD_DCD_STATE_NONE                0
#define STM32WB_USBD_DCD_STATE_INIT                1
#define STM32WB_USBD_DCD_STATE_NOT_READY           2
#define STM32WB_USBD_DCD_STATE_READY               3
#define STM32WB_USBD_DCD_STATE_BUSY                4
#define STM32WB_USBD_DCD_STATE_CONNECTED           5
#define STM32WB_USBD_DCD_STATE_SUSPENDED           6

#define STM32WB_USBD_DCD_BCD_STATE_STOP            0
#define STM32WB_USBD_DCD_BCD_STATE_START           1
#define STM32WB_USBD_DCD_BCD_STATE_CONTACT_TIMEOUT 31
#define STM32WB_USBD_DCD_BCD_STATE_PRIMARY_EN      251
#define STM32WB_USBD_DCD_BCD_STATE_PRIMARY_DIS     252
#define STM32WB_USBD_DCD_BCD_STATE_SECONDARY_EN    253
#define STM32WB_USBD_DCD_BCD_STATE_SECONDARY_DIS   254
#define STM32WB_USBD_DCD_BCD_STATE_DONE            255

#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK      0x0007
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_SHIFT     0
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_BULK      0x0000
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_DBL_BULK  0x0001
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_CONTROL   0x0002
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO       0x0004
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_INTERRUPT 0x0006
#define STM32WB_USBD_DCD_EP_CONTROL_STALLED        0x0010
#define STM32WB_USBD_DCD_EP_CONTROL_INVALID        0x0020
#define STM32WB_USBD_DCD_EP_CONTROL_ADDRESS_MASK   0xffc0
#define STM32WB_USBD_DCD_EP_CONTROL_ADDRESS_SHIFT  6

#define STM32WB_USBD_DCD_EP_ADDRESS(_control)      ((_control) >> STM32WB_USBD_DCD_EP_CONTROL_ADDRESS_SHIFT)

typedef struct __stm32wb_usbd_dcd_ep_in_t {
    uint16_t                       control;
    uint16_t                       size;
    union {
        struct {
            stm32wb_usbd_dcd_ep_in_callback_t  callback;
            void                               *context;
            const uint8_t                      *data;
            uint16_t                           length;
            bool                               zlp;
        } bulk;
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        struct {
            stm32wb_usbd_dcd_ep_in_callback_t  callback[2];
            void                               *context[2];
        } iso;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
    };
} stm32wb_usbd_dcd_ep_in_t;

typedef struct __stm32wb_usbd_dcd_ep_out_t {
    uint16_t                       control;
    uint16_t                       size;
    union {
        struct {
            stm32wb_usbd_dcd_ep_out_callback_t callback;
            void                               *context;
            uint8_t                            *data;
            uint16_t                           count;
            uint16_t                           length;
        } bulk;
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        struct {
            stm32wb_usbd_dcd_ep_out_callback_t callback;
            void                               *context;
            uint8_t                            *data;
            uint32_t                           length;
        } iso;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
    };
} stm32wb_usbd_dcd_ep_out_t;

typedef struct __stm32wb_usbd_dcd_device_t {
    volatile uint8_t                  state;
    volatile uint8_t                  address;
    volatile uint8_t                  bcd_state;
    volatile uint8_t                  bcd_status;
    uint32_t                          pma_address;
    stm32wb_usbd_dcd_event_callback_t evt_callback;
    void                              *evt_context;
    uint8_t                           *ep0_setup;
    uint8_t                           *ep0_data;
    uint8_t                           ep0_count;
    uint8_t                           ep0_length;
    volatile uint8_t                  ep0_request;
    stm32wb_usbd_dcd_ep_in_t          ep_in[7];  /* TX */
    stm32wb_usbd_dcd_ep_out_t         ep_out[7]; /* RX */
    stm32wb_lptim_timeout_t           timeout;
} stm32wb_usbd_dcd_device_t;

static stm32wb_usbd_dcd_device_t stm32wb_usbd_dcd_device;

static const uint16_t stm32wb_usbd_dcd_xlate_ep_type[4] = {
    STM32WB_USBD_DCD_EP_CONTROL_TYPE_CONTROL,
    STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO,
    STM32WB_USBD_DCD_EP_CONTROL_TYPE_BULK,
    STM32WB_USBD_DCD_EP_CONTROL_TYPE_INTERRUPT,
};

#define USB_EP_BASE                                (USB1_BASE + 0x0000)
#define USB_PMA_BASE                               (USB1_BASE + 0x0400)
#define USB_PMA_SIZE                               0x0400

#define USB_EP0_SIZE                               STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE
#define USB_EP0_IN_ADDRESS                         (0x0040)
#define USB_EP0_OUT_ADDRESS                        (USB_EP0_IN_ADDRESS + USB_EP0_SIZE)

#define USB_EP_R(_ep_index)                        (*((volatile uint16_t*)(USB_EP_BASE + ((_ep_index) * 4))))
#define USB_EP_W(_ep_index, _data)                 (*((volatile uint16_t*)(USB_EP_BASE + ((_ep_index) * 4))) = (_data))

#define USB_EP_SWBUF_TX                            USB_EP_DTOG_RX
#define USB_EP_SWBUF_RX                            USB_EP_DTOG_TX

#define USB_EP_CONFIG(_ep_index, _config)          USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USB_EP_CTR_RX | USB_EP_CTR_TX)) | (_config)))
#define USB_EP_STAT_TX(_ep_index, _stat)           USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USB_EPREG_MASK | USB_EPTX_STAT)) ^ (_stat)))
#define USB_EP_STAT_TX_RESET(_ep_index, _stat)     USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & ((USB_EPREG_MASK | USB_EPTX_STAT | USB_EP_DTOG_TX) & ~USB_EP_CTR_TX)) ^ (_stat)))
#define USB_EP_STAT_TX_DBL_RESET(_ep_index, _stat) USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & ((USB_EPREG_MASK | USB_EPTX_STAT | USB_EP_DTOG_TX | USB_EP_SWBUF_TX) & ~USB_EP_CTR_TX)) ^ (_stat)))
#define USB_EP_CTR_TX_RESET(_ep_index)             USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USB_EPREG_MASK & ~USB_EP_CTR_TX))))
#define USB_EP_SWBUF_TX_TOGGLE(_ep_index)          USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & USB_EPREG_MASK) | USB_EP_SWBUF_TX))
#define USB_EP_STAT_RX(_ep_index, _stat)           USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USB_EPREG_MASK | USB_EPRX_STAT)) ^ (_stat)))
#define USB_EP_STAT_RX_RESET(_ep_index, _stat)     USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & ((USB_EPREG_MASK | USB_EPRX_STAT | USB_EP_DTOG_RX) & ~USB_EP_CTR_RX)) ^ (_stat)))
#define USB_EP_STAT_RX_DBL_RESET(_ep_index, _stat) USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & ((USB_EPREG_MASK | USB_EPRX_STAT | USB_EP_DTOG_RX | USB_EP_SWBUF_RX) & ~USB_EP_CTR_RX)) ^ (_stat)))
#define USB_EP_CTR_RX_RESET(_ep_index)             USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USB_EPREG_MASK & ~USB_EP_CTR_RX))))
#define USB_EP_SWBUF_RX_TOGGLE(_ep_index)          USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & USB_EPREG_MASK) | USB_EP_SWBUF_RX))

#define USB_PMA_TX_ADDRESS(_ep_index, _address)    (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 0)) = (_address))
#define USB_PMA_TX_COUNT(_ep_index, _count)        (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 2)) = (_count))
#define USB_PMA_TX_0_ADDRESS(_ep_index, _address)  (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 0)) = (_address))
#define USB_PMA_TX_1_ADDRESS(_ep_index, _address)  (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 4)) = (_address))
#define USB_PMA_TX_0_COUNT(_ep_index, _count)      (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 2)) = (_count))
#define USB_PMA_TX_1_COUNT(_ep_index, _count)      (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 6)) = (_count))
#define USB_PMA_RX_ADDRESS(_ep_index, _address)    (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 4)) = (_address))
#define USB_PMA_RX_SIZE(_ep_index, _size)          (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 6)) = (_size))
#define USB_PMA_RX_COUNT(_ep_index)                (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 6)) & 0x03ff)
#define USB_PMA_RX_0_ADDRESS(_ep_index, _address)  (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 0)) = (_address))
#define USB_PMA_RX_1_ADDRESS(_ep_index, _address)  (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 4)) = (_address))
#define USB_PMA_RX_0_SIZE(_ep_index, _size)        (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 2)) = (_size))
#define USB_PMA_RX_1_SIZE(_ep_index, _size)        (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 6)) = (_size))
#define USB_PMA_RX_0_COUNT(_ep_index)              (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 2)) & 0x03ff)
#define USB_PMA_RX_1_COUNT(_ep_index)              (*((volatile uint16_t*)(USB_PMA_BASE + ((_ep_index) * 8) + 6)) & 0x03ff)

static void __attribute__((optimize("O3"), noinline)) stm32wb_usbd_dcd_pma_write(uint32_t address, const uint8_t *data, uint32_t count)
{
    volatile uint16_t *pma;
    const uint16_t *data16, *data16_e;
    
    pma = (volatile uint16_t*)(USB_PMA_BASE + address);

    data16 = (const uint16_t*)(data);
    data16_e = (const uint16_t*)(data + (count & ~15));

    while (data16 != data16_e)
    {
        pma[0] = data16[0];
        pma[1] = data16[1];
        pma[2] = data16[2];
        pma[3] = data16[3];
        pma[4] = data16[4];
        pma[5] = data16[5];
        pma[6] = data16[6];
        pma[7] = data16[7];

        pma += 8;
        data16 += 8;
    }

    if (count & 15)
    {
        if (count & 8)
        {
            pma[0] = data16[0];
            pma[1] = data16[1];
            pma[2] = data16[2];
            pma[3] = data16[3];

            pma += 4;
            data16 += 4;
        }

        if (count & 4)
        {
            pma[0] = data16[0];
            pma[1] = data16[1];

            pma += 2;
            data16 += 2;
        }

        if (count & 2)
        {
            pma[0] = data16[0];

            pma += 1;
            data16 += 1;
        }

        if (count & 1)
        {
            data = (const uint8_t*)data16;
            
            pma[0] = data[0];
        }
    }
}

static void __attribute__((optimize("O3"), noinline)) stm32wb_usbd_dcd_pma_read(uint32_t address, uint8_t *data, uint32_t count)
{
    volatile uint16_t *pma;
    uint16_t *data16, *data16_e;
    
    pma = (volatile uint16_t*)(USB_PMA_BASE + address);

    data16 = (uint16_t*)(data);
    data16_e = (uint16_t*)(data + (count & ~15));

    while (data16 != data16_e)
    {
        data16[0] = pma[0];
        data16[1] = pma[1];
        data16[2] = pma[2];
        data16[3] = pma[3];
        data16[4] = pma[4];
        data16[5] = pma[5];
        data16[6] = pma[6];
        data16[7] = pma[7];

        pma += 8;
        data16 += 8;
    }

    if (count & 15)
    {
        if (count & 8)
        {
            data16[0] = pma[0];
            data16[1] = pma[1];
            data16[2] = pma[2];
            data16[3] = pma[3];

            pma += 4;
            data16 += 4;
        }

        if (count & 4)
        {
            data16[0] = pma[0];
            data16[1] = pma[1];

            pma += 2;
            data16 += 2;
        }

        if (count & 2)
        {
            data16[0] = pma[0];

            pma += 1;
            data16 += 1;
        }

        if (count & 1)
        {
            data = (uint8_t*)data16;

            data[0] = pma[0];
        }
    }
}

bool stm32wb_usbd_dcd_configure(void)
{
    uint32_t ep_index;

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_NONE)
    {
        return false;
    }
    
    stm32wb_lptim_timeout_create(&stm32wb_usbd_dcd_device.timeout);

    for (ep_index = 1; ep_index < 8; ep_index++)
    {
        stm32wb_usbd_dcd_device.ep_in[ep_index -1].control = STM32WB_USBD_DCD_EP_CONTROL_INVALID;
        stm32wb_usbd_dcd_device.ep_out[ep_index -1].control = STM32WB_USBD_DCD_EP_CONTROL_INVALID;
    }

    stm32wb_usbd_dcd_device.pma_address = 64 + (STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE * 2);

    NVIC_SetPriority(USB_HP_IRQn, STM32WB_USBD_DCD_IRQ_PRIORITY);
    NVIC_SetPriority(USB_LP_IRQn, STM32WB_USBD_DCD_IRQ_PRIORITY);
    
    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_INIT;

    return true;
}

/* This below could be done clearer, but this way it takes up less space.
 * At the end of the day what happens is:
 *
 * BCDEN | DCDEN
 *                10ms - 310ms [tDCD_DBNC + tDCD_TIMEOUT, check DCDET every 10ms to to 310ms]
 * BCDEN     
 *                10ms
 * BCDEN | PDEN
 *                25ms         [tVDMSRC_EN + 5ms]
 * BCDEN                       [check PDET]
 *                25ms         [tVDMSRC_DIS + 5ms]
 * BCDEN | SDEN
 *                40ms         [tVDMSRC_ON]
 *                             [check SDET]
 *
 * In total the whole process needs to go from VBUS detected to DPPU tSLVD_CON_PWD, 1000ms.
 */

static void stm32wb_usbd_dcd_detect()
{
    uint32_t usb_bdcr, timeout;

    timeout = 0;

    usb_bdcr = USB->BCDR;
    
    if (stm32wb_usbd_dcd_device.bcd_state <= STM32WB_USBD_DCD_BCD_STATE_CONTACT_TIMEOUT)
    {
        if (usb_bdcr & USB_BCDR_DCDET)
        {
            USB->BCDR = USB_BCDR_BCDEN;
            
            stm32wb_usbd_dcd_device.bcd_state = STM32WB_USBD_DCD_BCD_STATE_PRIMARY_EN;
            
            timeout = stm32wb_lptim_timeout_millis_to_ticks(10);
        }
        else
        {
            if (stm32wb_usbd_dcd_device.bcd_state < STM32WB_USBD_DCD_BCD_STATE_CONTACT_TIMEOUT)
            {
                stm32wb_usbd_dcd_device.bcd_state++;
                
                timeout = stm32wb_lptim_timeout_millis_to_ticks(10);
            }
        }
    }

    else if (stm32wb_usbd_dcd_device.bcd_state == STM32WB_USBD_DCD_BCD_STATE_PRIMARY_EN)
    {
        USB->BCDR = USB_BCDR_BCDEN | USB_BCDR_PDEN;

        timeout = stm32wb_lptim_timeout_millis_to_ticks(25);

        stm32wb_usbd_dcd_device.bcd_state = STM32WB_USBD_DCD_BCD_STATE_PRIMARY_DIS;
    }

    else if (stm32wb_usbd_dcd_device.bcd_state == STM32WB_USBD_DCD_BCD_STATE_PRIMARY_DIS)
    {
        USB->BCDR = USB_BCDR_BCDEN;

        if (usb_bdcr & USB_BCDR_PDET)
        {
            timeout = stm32wb_lptim_timeout_millis_to_ticks(25);

            stm32wb_usbd_dcd_device.bcd_state = STM32WB_USBD_DCD_BCD_STATE_SECONDARY_EN;
        }
        else
        {
            stm32wb_usbd_dcd_device.bcd_status = STM32WB_USBD_BCD_STATUS_PORT;
        }
    }
    
    else if (stm32wb_usbd_dcd_device.bcd_state == STM32WB_USBD_DCD_BCD_STATE_SECONDARY_EN)
    {
        USB->BCDR = USB_BCDR_BCDEN | USB_BCDR_SDEN;

        timeout = stm32wb_lptim_timeout_millis_to_ticks(40);

        stm32wb_usbd_dcd_device.bcd_state = STM32WB_USBD_DCD_BCD_STATE_SECONDARY_DIS;
    }

    else
    {
        stm32wb_usbd_dcd_device.bcd_status = ((usb_bdcr & USB_BCDR_SDET) ? STM32WB_USBD_BCD_STATUS_CHARGER : STM32WB_USBD_BCD_STATUS_PORT_AND_CHARGER);
    }
    
    if (timeout)
    {
        stm32wb_lptim_timeout_start(&stm32wb_usbd_dcd_device.timeout, timeout, (stm32wb_lptim_timeout_callback_t)stm32wb_usbd_dcd_detect);
    }
    else
    {
        stm32wb_usbd_dcd_device.bcd_state = STM32WB_USBD_DCD_BCD_STATE_DONE;
        
        USB->BCDR = 0;

        NVIC_SetPendingIRQ(USB_LP_IRQn);
        
        NVIC_EnableIRQ(USB_LP_IRQn);
        NVIC_EnableIRQ(USB_HP_IRQn);
    }
}

bool stm32wb_usbd_dcd_enable(uint8_t *setup, uint32_t options, stm32wb_usbd_dcd_event_callback_t callback, void *context)
{
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_INIT)
    {
        return false;
    }

    if (stm32wb_system_pclk1() < 10000000)
    {
        return false;
    }

    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_NOT_READY;

    stm32wb_usbd_dcd_device.evt_callback = callback;
    stm32wb_usbd_dcd_device.evt_context = context;
    stm32wb_usbd_dcd_device.ep0_setup = setup;

    armv7m_atomic_or(&PWR->CR2, PWR_CR2_USV);

    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA11_USB_DM, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA12_USB_DP, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));

    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_USB);
        
    stm32wb_system_reference(STM32WB_SYSTEM_REFERENCE_USB);

    armv7m_atomic_storeh(&USB->CNTR, USB_CNTR_FRES);
    armv7m_atomic_storeh(&USB->CNTR, 0);

    USB->ISTR = 0;
    USB->DADDR = 0;
    USB->BTABLE = 0;

    if (options & STM32WB_USBD_DCD_OPTION_DETECT)
    {
        USB->BCDR = USB_BCDR_BCDEN | USB_BCDR_DCDEN;

        stm32wb_usbd_dcd_device.bcd_status = STM32WB_USBD_BCD_STATUS_UNKNOWN;
        stm32wb_usbd_dcd_device.bcd_state = STM32WB_USBD_DCD_BCD_STATE_START;
        
        stm32wb_lptim_timeout_start(&stm32wb_usbd_dcd_device.timeout, stm32wb_lptim_timeout_millis_to_ticks(10), (stm32wb_lptim_timeout_callback_t)stm32wb_usbd_dcd_detect);
    }
    else
    {
        USB->BCDR = 0;
    }
    
    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_READY;
    
    NVIC_EnableIRQ(USB_LP_IRQn);
    NVIC_EnableIRQ(USB_HP_IRQn);
    
    return true;
}

bool stm32wb_usbd_dcd_disable(void)
{
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_READY)
    {
        return false;
    }

    stm32wb_lptim_timeout_stop(&stm32wb_usbd_dcd_device.timeout);
    
    NVIC_DisableIRQ(USB_HP_IRQn);
    NVIC_DisableIRQ(USB_LP_IRQn);

    armv7m_atomic_storeh(&USB->CNTR, (USB_CNTR_FRES | USB_CNTR_PDWN));

    USB->BCDR = 0;

    stm32wb_system_reference(STM32WB_SYSTEM_REFERENCE_USB);
    
    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_USB);
    
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA11_USB_DM, STM32WB_GPIO_MODE_ANALOG);
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA12_USB_DP, STM32WB_GPIO_MODE_ANALOG);

    armv7m_atomic_and(&PWR->CR2, ~PWR_CR2_USV);

    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_INIT;

    return true;
}

static void stm32wb_usbd_dcd_clk48(void)
{
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_BUSY)
    {
        return;
    }

    stm32wb_system_clk48_enable();

    if (!stm32wb_system_lseclk())
    {
        armv7m_atomic_or(&RCC->APB1ENR1, RCC_APB1ENR1_CRSEN);
        RCC->APB1ENR1;
        
        CRS->CFGR = (((48000 -1) << CRS_CFGR_RELOAD_Pos) | (34 << CRS_CFGR_FELIM_Pos) | CRS_CFGR_SYNCSRC_1);
        CRS->CR |= (CRS_CR_AUTOTRIMEN | CRS_CR_CEN);
    }

    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP_0);

    armv7m_atomic_storeh(&USB->CNTR, USB_CNTR_FRES);
    armv7m_atomic_storeh(&USB->CNTR, 0);
    armv7m_atomic_storeh(&USB->CNTR, (USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_RESETM));

    USB->ISTR = 0;
    USB->DADDR = 0;
    USB->BTABLE = 0;
    USB->BCDR = USB_BCDR_DPPU;

    USB_PMA_TX_ADDRESS(0, USB_EP0_IN_ADDRESS);
    USB_PMA_RX_ADDRESS(0, USB_EP0_OUT_ADDRESS);
    USB_PMA_RX_SIZE(0, 0x8400);                     // 8 = 0x1000, 16 = 0x2000, 32 = 0x4000, 64 = 0x8400

    armv7m_atomic_or(&EXTI->IMR1, EXTI_IMR1_IM28);

    stm32wb_usbd_dcd_device.address = 0;

    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_CONNECTED;
}

bool stm32wb_usbd_dcd_connect(void)
{
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_READY)
    {
        return false;
    }
    stm32wb_lptim_timeout_stop(&stm32wb_usbd_dcd_device.timeout);

    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_BUSY;

    if (!stm32wb_hsem_lock(STM32WB_HSEM_CLK48, 0))
    {
        return true;
    }

    stm32wb_usbd_dcd_clk48();
    
    return true;
}

bool stm32wb_usbd_dcd_disconnect(void)
{
    uint32_t ep_index;
    
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_BUSY)
    {
        return false;
    }

    if (stm32wb_usbd_dcd_device.state >= STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_BUSY;
    
        for (ep_index = 1; ep_index < 8; ep_index++)
        {
            stm32wb_usbd_dcd_device.ep_in[ep_index -1].control &= ~STM32WB_USBD_DCD_EP_CONTROL_STALLED;
            stm32wb_usbd_dcd_device.ep_in[ep_index -1].control |= STM32WB_USBD_DCD_EP_CONTROL_INVALID;
            
            stm32wb_usbd_dcd_device.ep_out[ep_index -1].control &= ~STM32WB_USBD_DCD_EP_CONTROL_STALLED;
            stm32wb_usbd_dcd_device.ep_out[ep_index -1].control |= STM32WB_USBD_DCD_EP_CONTROL_INVALID;
        }
        
        armv7m_atomic_and(&EXTI->IMR1, ~EXTI_IMR1_IM28);
        
        armv7m_atomic_storeh(&USB->CNTR, (USB_CNTR_FRES | USB_CNTR_PDWN));

        USB->DADDR = 0;
        USB->BCDR = 0;

        if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_SUSPENDED)
        {
            stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP_0);
        }
        
        stm32wb_system_clk48_disable();

        if (!stm32wb_system_lseclk())
        {
            CRS->CR &= ~(CRS_CR_AUTOTRIMEN | CRS_CR_CEN);
            
            armv7m_atomic_and(&RCC->APB1ENR1, ~RCC_APB1ENR1_CRSEN);
        }

        stm32wb_hsem_unlock(STM32WB_HSEM_CLK48, 0);
    }

    stm32wb_usbd_dcd_device.ep0_count = 0;
    stm32wb_usbd_dcd_device.ep0_length = 0;
    stm32wb_usbd_dcd_device.ep0_request = false;
    
    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_READY;

    return true;
}

uint32_t stm32wb_usbd_dcd_bcd_status(void)
{
    return stm32wb_usbd_dcd_device.bcd_status;
}

bool stm32wb_usbd_dcd_reset(void)
{
    uint32_t ep_index;
    
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }

    stm32wb_lptim_timeout_stop(&stm32wb_usbd_dcd_device.timeout);
        
    armv7m_atomic_orh(&USB->CNTR, USB_CNTR_FRES);

    for (ep_index = 0; ep_index < 8; ep_index++)
    {
        USB_EP_CTR_TX_RESET(ep_index);
        USB_EP_CTR_RX_RESET(ep_index);
    }
    
    armv7m_atomic_andh(&USB->CNTR, ~(USB_CNTR_LPMODE | USB_CNTR_FSUSP | USB_CNTR_RESUME | USB_CNTR_FRES));

    USB->ISTR = (uint16_t)~USB_ISTR_RESET;
    USB->DADDR = USB_DADDR_EF;

    USB_EP_W(0, USB_EP_CONTROL);
    USB_EP_STAT_TX_RESET(0, USB_EP_TX_NAK);
    USB_EP_STAT_RX_RESET(0, USB_EP_RX_NAK);

    if (stm32wb_usbd_dcd_device.state == STM32WB_USBD_DCD_STATE_SUSPENDED)
    {
        stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP_0);
    }

    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_CONNECTED;
    
    NVIC_EnableIRQ(USB_LP_IRQn);
    NVIC_EnableIRQ(USB_HP_IRQn);
    
    return true;
}

static void stm32wb_usbd_dcd_resume(void)
{
    if (USB->CNTR & USB_CNTR_RESUME)
    {
        armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_RESUME);
    }
    else
    {
        armv7m_atomic_orh(&USB->CNTR, USB_CNTR_RESUME);

        stm32wb_lptim_timeout_start(&stm32wb_usbd_dcd_device.timeout, stm32wb_lptim_timeout_millis_to_ticks(2), (stm32wb_lptim_timeout_callback_t)stm32wb_usbd_dcd_resume);
    }
}

bool stm32wb_usbd_dcd_wakeup(void)
{
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_SUSPENDED)
    {
        return false;
    }

    stm32wb_lptim_timeout_start(&stm32wb_usbd_dcd_device.timeout, stm32wb_lptim_timeout_millis_to_ticks(5), (stm32wb_lptim_timeout_callback_t)stm32wb_usbd_dcd_resume);
    
    return true;
}

bool stm32wb_usbd_dcd_set_address(uint8_t address)
{
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }

    stm32wb_usbd_dcd_device.address = (USB_DADDR_EF | address);

    return true;
}

bool stm32wb_usbd_dcd_sof_enable()
{
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }

    USB->ISTR = ~USB_ISTR_SOF;

    armv7m_atomic_orh(&USB->CNTR, USB_CNTR_SOFM);

    return true;
}

bool stm32wb_usbd_dcd_sof_disable()
{
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }

    armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_SOFM);

    USB->ISTR = ~USB_ISTR_SOF;

    return true;
}

bool stm32wb_usbd_dcd_ep0_transmit(const uint8_t *data, uint16_t length)
{
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }
    
    if (length > STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE)
    {
        return false;
    }

    if (length)
    {
        stm32wb_usbd_dcd_pma_write(USB_EP0_IN_ADDRESS, data, length);
        
        USB_PMA_TX_COUNT(0, length);
    }
    else
    {
        USB_PMA_TX_COUNT(0, 0);
    }
        
    USB_EP_STAT_TX(0, USB_EP_TX_VALID);

    return true;
}

bool stm32wb_usbd_dcd_ep0_receive(uint8_t *data, uint16_t length)
{
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }

    if (length > STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE)
    {
        return false;
    }
    
    stm32wb_usbd_dcd_device.ep0_data = data;
    stm32wb_usbd_dcd_device.ep0_count = 0;
    stm32wb_usbd_dcd_device.ep0_length = length;

    USB_EP_STAT_RX(0, USB_EP_RX_VALID);

    return true;
}

uint32_t stm32wb_usbd_dcd_ep0_count(void)
{
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return 0;
    }

    return stm32wb_usbd_dcd_device.ep0_count;
}

bool stm32wb_usbd_dcd_ep0_stall(void)
{
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }

    USB_EP_STAT_TX(0, USB_EP_TX_STALL);
    USB_EP_STAT_RX(0, USB_EP_RX_STALL);
    
    return true;
}

bool stm32wb_usbd_dcd_ep0_request(void)
{
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }

    stm32wb_usbd_dcd_device.ep0_request = true;

    NVIC_SetPendingIRQ(USB_LP_IRQn);
    
    return true;
}

bool stm32wb_usbd_dcd_ep_configure(uint8_t ep_addr, uint8_t type, uint16_t size)
{
    uint32_t ep_index, ep_type, pma_size;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_INIT)
    {
        return false;
    }

    if ((size == 0) || !((size == 8) || (size == 16) || !(size & 31)))
    {
        return false;
    }

    ep_index = ep_addr & 0x07;

    ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];
    ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

    ep_type = stm32wb_usbd_dcd_xlate_ep_type[type];
    
    if (ep_addr & 0x80)
    {
        if (ep_in->size)
        {
            return false;
        }

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        if (ep_type == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
        {
            if (ep_out->size)
            {
                return false;
            }
            
            pma_size = size * 2;
        }
        else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
        {
            if (ep_out->size && ((ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) != ep_type))
            {
                return false;
            }

            pma_size = size;
            
        }

        if (pma_size > (uint32_t)(USB_PMA_SIZE - stm32wb_usbd_dcd_device.pma_address))
        {
            return false;
        }
            
        ep_in->control = (stm32wb_usbd_dcd_device.pma_address << STM32WB_USBD_DCD_EP_CONTROL_ADDRESS_SHIFT) | STM32WB_USBD_DCD_EP_CONTROL_INVALID | ep_type;
        ep_in->size = size;
            
        stm32wb_usbd_dcd_device.pma_address += pma_size;
    }
    else
    {
        if (ep_out->size)
        {
            return false;
        }

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        if (ep_type == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
        {
            if (ep_in->size)
            {
                return false;
            }
            
            pma_size = size * 2;
        }
        else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
        {
            if (ep_in->size && ((ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) != ep_type))
            {
                return false;
            }

            pma_size = size;
        }

        if (pma_size > (uint32_t)(USB_PMA_SIZE - stm32wb_usbd_dcd_device.pma_address))
        {
            return false;
        }

        ep_out->control = (stm32wb_usbd_dcd_device.pma_address << STM32WB_USBD_DCD_EP_CONTROL_ADDRESS_SHIFT) | STM32WB_USBD_DCD_EP_CONTROL_INVALID | ep_type;
        ep_out->size = size;
            
        stm32wb_usbd_dcd_device.pma_address += pma_size;
    }
    
    return true;
}

bool stm32wb_usbd_dcd_ep_enable(uint8_t ep_addr)
{
    uint32_t ep_index, ep_type, ep_size;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }
    
    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
        ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

        if (!ep_in->size)
        {
            return false;
        }
        
        if (!(ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_INVALID))
        {
            return false;
        }

        ep_type = ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK;

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        if (ep_type == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
        {
            if (!ep_in->iso.callback[0] || !ep_in->iso.callback[1])
            {
                return false;
            }
        }
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
        
        ep_in->control &= ~STM32WB_USBD_DCD_EP_CONTROL_INVALID;

        USB_EP_CONFIG(ep_index, ((ep_type << 8) | ep_index));

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        if (ep_type == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
        {
            USB_PMA_TX_0_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_ADDRESS(ep_in->control));
            USB_PMA_TX_1_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_ADDRESS(ep_in->control) + ep_in->size);
            
            USB_EP_STAT_TX_RESET(ep_index, USB_EP_TX_VALID);
        }
        else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
        {
            USB_PMA_TX_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_ADDRESS(ep_in->control));
                
            USB_EP_STAT_TX_RESET(ep_index, USB_EP_TX_NAK);
        }
    }
    else
    {
        ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

        if (!ep_out->size)
        {
            return false;
        }

        if (!(ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_INVALID))
        {
            return false;
        }

        ep_type = ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK;
        
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        if (ep_type == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
        {
            if (!ep_out->iso.length)
            {
                return false;
            }
        }
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
        
        ep_out->control &= ~STM32WB_USBD_DCD_EP_CONTROL_INVALID;

        if (ep_out->size >= 64)
        {
            ep_size = 0x8000 | (((ep_out->size - 32) / 32) << 10);
        }
        else
        {
            ep_size = 0x0000 | ((ep_out->size / 2) << 10);
        }

        USB_EP_CONFIG(ep_index, ((ep_type << 8) | ep_index));

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        if (ep_type == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
        {
            USB_PMA_RX_0_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_ADDRESS(ep_out->control));
            USB_PMA_RX_0_SIZE(ep_index, ep_size);
            USB_PMA_RX_1_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_ADDRESS(ep_out->control) + ep_out->size);
            USB_PMA_RX_1_SIZE(ep_index, ep_size);
            
            USB_EP_STAT_RX_RESET(ep_index, USB_EP_RX_VALID);
        }
        else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
        {
            USB_PMA_RX_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_ADDRESS(ep_out->control));
            USB_PMA_RX_SIZE(ep_index, ep_size);
                
            USB_EP_STAT_RX_RESET(ep_index, USB_EP_RX_NAK);
        }
    }

    return true;
}

bool stm32wb_usbd_dcd_ep_disable(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }

    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
        ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

        if (ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_INVALID)
        {
            return true;
        }

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        if ((ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
        {
            ep_in->iso.callback[0] = NULL;
            ep_in->iso.callback[1] = NULL;
        }
        else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
        {
            ep_in->bulk.callback = NULL;
        }
        
        USB_EP_STAT_TX(ep_index, USB_EP_TX_DIS);
        
        ep_in->control &= ~STM32WB_USBD_DCD_EP_CONTROL_STALLED;
        ep_in->control |= STM32WB_USBD_DCD_EP_CONTROL_INVALID;
    }
    else
    {
        ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

        if (ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_INVALID)
        {
            return true;
        }

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        if ((ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
        {
            ep_out->iso.callback = NULL;
        }
        else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
        {
            ep_out->bulk.callback = NULL;
        }

        USB_EP_STAT_RX(ep_index, USB_EP_RX_DIS);
        
        ep_out->control &= ~STM32WB_USBD_DCD_EP_CONTROL_STALLED;
        ep_out->control |= STM32WB_USBD_DCD_EP_CONTROL_INVALID;
    }

    return true;
}

bool stm32wb_usbd_dcd_ep_stall(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }
    
    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
        ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

        if (ep_in->control & (STM32WB_USBD_DCD_EP_CONTROL_INVALID | STM32WB_USBD_DCD_EP_CONTROL_STALLED))
        {
            return false;
        }

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        if ((ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
        {
            return false;
        }
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */

        ep_in->bulk.callback = NULL;

        USB_EP_STAT_TX(ep_index, USB_EP_TX_STALL);
        
        ep_in->control |= STM32WB_USBD_DCD_EP_CONTROL_STALLED;
    }
    else
    {
        ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

        if (ep_out->control & (STM32WB_USBD_DCD_EP_CONTROL_INVALID | STM32WB_USBD_DCD_EP_CONTROL_STALLED))
        {
            return false;
        }

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        if ((ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
        {
            return false;
        }
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
        
        ep_out->bulk.callback = NULL;

        USB_EP_STAT_RX(ep_index, USB_EP_RX_STALL);

        ep_out->control |= STM32WB_USBD_DCD_EP_CONTROL_STALLED;
    }

    return true;
}

bool stm32wb_usbd_dcd_ep_unstall(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }
    
    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
        ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

        if (!(ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_STALLED))
        {
            return false;
        }
        
        ep_in->control &= ~STM32WB_USBD_DCD_EP_CONTROL_STALLED;
        
        USB_EP_STAT_TX_RESET(ep_index, USB_EP_TX_NAK);
    }
    else
    {
        ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

        if (!(ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_STALLED))
        {
            return false;
        }

        ep_out->control &= ~STM32WB_USBD_DCD_EP_CONTROL_STALLED;
        
        USB_EP_STAT_RX_RESET(ep_index, USB_EP_RX_NAK);
    }

    return true;
}

bool stm32wb_usbd_dcd_ep_is_stalled(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }
    
    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
        ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

        return !!(ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_STALLED);
    }
    else
    {
        ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

        return !!(ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_STALLED);
    }
}

bool stm32wb_usbd_dcd_ep_transmit(uint8_t ep_addr, const uint8_t *data, uint16_t length, stm32wb_usbd_dcd_ep_in_callback_t callback, void *context)
{
    uint32_t ep_index, count;
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
    uint32_t iso_index;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
    stm32wb_usbd_dcd_ep_in_t *ep_in;

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }
    
    ep_index = ep_addr & 0x07;

    ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
    if ((ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
    {
        if (ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_INVALID)
        {
            iso_index = ep_in->iso.callback[0] ? 1 : 0;
        }
        else
        {
            iso_index = (USB_EP_R(ep_index) & USB_EP_DTOG_TX) ? 0 : 1;
        }
        
        if (ep_in->iso.callback[iso_index])
        {
            return false;
        }

        if (length > ep_in->size)
        {
            return false;
        }

        ep_in->iso.callback[iso_index] = callback;
        ep_in->iso.context[iso_index] = context;
        
        stm32wb_usbd_dcd_pma_write(STM32WB_USBD_DCD_EP_ADDRESS(ep_in->control) + ((iso_index == 0) ? 0 : ep_in->size), data, length);

        if (!iso_index)
        {
            USB_PMA_TX_0_COUNT(ep_index, length);
        }
        else
        {
            USB_PMA_TX_1_COUNT(ep_index, length);
        }
    }
    else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
    {
        if (ep_in->bulk.callback)
        {
            return false;
        }

        if (ep_in->control & (STM32WB_USBD_DCD_EP_CONTROL_INVALID | STM32WB_USBD_DCD_EP_CONTROL_STALLED))
        {
            return false;
        }

        ep_in->bulk.data = data;
        ep_in->bulk.length = length;
        ep_in->bulk.callback = callback;
        ep_in->bulk.context = context;
        
        if (ep_in->bulk.length)
        {
            count = ep_in->bulk.length;
            
            if (count > ep_in->size)
            {
                count = ep_in->size;
            }
            
            stm32wb_usbd_dcd_pma_write(STM32WB_USBD_DCD_EP_ADDRESS(ep_in->control), ep_in->bulk.data, count);
            
            USB_PMA_TX_COUNT(ep_index, count);

            USB_EP_STAT_TX(ep_index, USB_EP_TX_VALID);
            
            ep_in->bulk.data += count;
            ep_in->bulk.length -= count;
        }
        else
        {
            USB_PMA_TX_COUNT(ep_index, 0);

            USB_EP_STAT_TX(ep_index, USB_EP_TX_VALID);
        }
    }
    
    return true;
}

bool stm32wb_usbd_dcd_ep_receive(uint8_t ep_addr, uint8_t *data, uint16_t length, stm32wb_usbd_dcd_ep_out_callback_t callback, void *context)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }
    
    ep_index = ep_addr & 0x07;

    ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
    if ((ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
    {
        if (ep_out->iso.callback)
        {
            return false;
        }

        if (length > ep_out->size)
        {
            return false;
        }

        ep_out->iso.data = data;
        ep_out->iso.length = length;
        ep_out->iso.callback = callback;
        ep_out->iso.context = context;
    }
    else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
    {
        if (ep_out->bulk.callback)
        {
            return false;
        }
        
        if (ep_out->control & (STM32WB_USBD_DCD_EP_CONTROL_INVALID | STM32WB_USBD_DCD_EP_CONTROL_STALLED))
        {
            return false;
        }

        ep_out->bulk.data = data;
        ep_out->bulk.count = 0;
        ep_out->bulk.length = length;
        ep_out->bulk.callback = callback;
        ep_out->bulk.context = context;
        
        USB_EP_STAT_RX(ep_index, USB_EP_RX_VALID);
    }

    return true;
}

bool stm32wb_usbd_dcd_ep_flush(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        return false;
    }
    
    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
        ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

        if (ep_in->control & (STM32WB_USBD_DCD_EP_CONTROL_INVALID | STM32WB_USBD_DCD_EP_CONTROL_STALLED))
        {
            return false;
        }

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        if ((ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
        {
            return false;
        }
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */

        ep_in->bulk.callback = NULL;

        USB_EP_STAT_TX_RESET(ep_index, USB_EP_TX_NAK);
    }
    else
    {
        ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

        if (ep_out->control & (STM32WB_USBD_DCD_EP_CONTROL_INVALID | STM32WB_USBD_DCD_EP_CONTROL_STALLED))
        {
            return false;
        }

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        if ((ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
        {
            return false;
        }
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
        
        ep_out->bulk.callback = NULL;

        USB_EP_STAT_RX_RESET(ep_index, USB_EP_RX_NAK);
    }

    return true;
}

static void __attribute__((optimize("O3"))) stm32wb_usbd_dcd_interrupt(void)
{
    uint32_t usb_istr, usb_ep, ep_index, count, events;
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
    uint32_t iso_index;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
    uint8_t address;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;
    stm32wb_usbd_dcd_ep_in_callback_t ep_in_callback;
    stm32wb_usbd_dcd_ep_out_callback_t ep_out_callback;
    void *ep_in_context, *ep_out_context;

    events = 0;
    
    usb_istr = USB->ISTR;

    if (usb_istr & USB_ISTR_CTR)
    {
        do
        {
            ep_index = usb_istr & USB_ISTR_EP_ID;

            if (ep_index == 0)
            {
                usb_ep = USB_EP_R(0);

                if (usb_istr & USB_ISTR_DIR)
                {
                    count = USB_PMA_RX_COUNT(0);

                    if (usb_ep & USB_EP_SETUP)
                    {
                        stm32wb_usbd_dcd_pma_read(USB_EP0_OUT_ADDRESS, stm32wb_usbd_dcd_device.ep0_setup, count);

                        USB_EP_CTR_RX_RESET(0);
                            
                        events |= STM32WB_USBD_DCD_EVENT_EP0_SETUP;
                    }
                    else
                    {
                        USB_EP_CTR_RX_RESET(0);

                        if (count > stm32wb_usbd_dcd_device.ep0_length)
                        {
                            count = stm32wb_usbd_dcd_device.ep0_length;
                        }

                        if (count)
                        {
                            stm32wb_usbd_dcd_pma_read(USB_EP0_OUT_ADDRESS, stm32wb_usbd_dcd_device.ep0_data, count);
                        }

                        stm32wb_usbd_dcd_device.ep0_count = count;
                        
                        events |= STM32WB_USBD_DCD_EVENT_EP0_DATA_OUT;
                    }
                }
                
                if (usb_ep & USB_EP_CTR_TX)
                {
                    USB_EP_CTR_TX_RESET(0);

                    address = stm32wb_usbd_dcd_device.address;

                    if (address & USB_DADDR_EF)
                    {
                        stm32wb_usbd_dcd_device.address = address & ~USB_DADDR_EF;
                        
                        USB->DADDR = address;
                    }
                    
                    events |= STM32WB_USBD_DCD_EVENT_EP0_DATA_IN;
                }
            }
            else
            {
                usb_ep = USB_EP_R(ep_index);

                if (usb_ep & USB_EP_CTR_RX)
                {
                    USB_EP_CTR_RX_RESET(ep_index);

                    ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
                    if ((ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
                    {
                        /* ISO */
                        
                        iso_index = (usb_ep & USB_EP_DTOG_RX) ? 0 : 1;
                        
                        ep_out_callback = ep_out->iso.callback;
                        ep_out_context = ep_out->iso.context;

                        if (ep_out_callback)
                        {
                            if (iso_index == 0)
                            {
                                count = USB_PMA_RX_0_COUNT(ep_index);
                            }
                            else
                            {
                                count = USB_PMA_RX_1_COUNT(ep_index);
                            }
                        
                            if (count > ep_out->iso.length)
                            {
                                count = ep_out->iso.length;
                            }

                            stm32wb_usbd_dcd_pma_read((STM32WB_USBD_DCD_EP_ADDRESS(ep_out->control) + ((iso_index == 0) ? 0 : ep_out->size)), ep_out->iso.data, count);
                        
                            ep_out->iso.callback = NULL;
                        
                            (*ep_out_callback)(ep_out_context, (ep_index | 0x00), count);
                        }
                    }
                    else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
                    {
                        /* BULK / INTERRUPT */

                        ep_out_callback = ep_out->bulk.callback;
                        ep_out_context = ep_out->bulk.context;
                            
                        if (ep_out_callback)
                        {
                            count = USB_PMA_RX_COUNT(ep_index);
                        
                            if (count > (uint32_t)(ep_out->bulk.length - ep_out->bulk.count))
                            {
                                count = ep_out->bulk.length - ep_out->bulk.count;
                            }
                            
                            if (count)
                            {
                                stm32wb_usbd_dcd_pma_read(STM32WB_USBD_DCD_EP_ADDRESS(ep_out->control), ep_out->bulk.data, count);
                            }
                            
                            ep_out->bulk.data += count;
                            ep_out->bulk.count += count;
                            
                            if ((count == ep_out->size) && (ep_out->bulk.count != ep_out->bulk.length))
                            {
                                USB_EP_STAT_RX(ep_index, USB_EP_RX_VALID);
                            }
                            else
                            {
                                
                                ep_out->bulk.callback = NULL;
                                
                                (*ep_out_callback)(ep_out_context, (ep_index | 0x00), ep_out->bulk.count);
                            }
                        }
                    }
                }

                if (usb_ep & USB_EP_CTR_TX)
                {
                    USB_EP_CTR_TX_RESET(ep_index);

                    ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
                    if ((ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
                    {
                        /* ISO */

                        iso_index = (usb_ep & USB_EP_DTOG_TX) ? 0 : 1;
                        
                        ep_in_callback = ep_in->iso.callback[iso_index];
                        ep_in_context = ep_in->iso.context[iso_index];

                        if (ep_in_callback)
                        {
                            ep_in->iso.callback[iso_index] = NULL;
                            
                            (*ep_in_callback)(ep_in_context, (ep_index | 0x80));
                        }
                    }
                    else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
                    {
                        ep_in_callback = ep_in->bulk.callback;
                        ep_in_context = ep_in->bulk.context;
                            
                        if (ep_in_callback)
                        {
                            if (ep_in->bulk.length)
                            {
                                count = ep_in->bulk.length;
                                
                                if (count > ep_in->size)
                                {
                                    count = ep_in->size;
                                }
                                
                                stm32wb_usbd_dcd_pma_write(STM32WB_USBD_DCD_EP_ADDRESS(ep_in->control), ep_in->bulk.data, count);
                                
                                USB_PMA_TX_COUNT(ep_index, count);
                            
                                USB_EP_STAT_TX(ep_index, USB_EP_TX_VALID);
                                
                                ep_in->bulk.data += count;
                                ep_in->bulk.length -= count;
                            }
                            else
                            {
                                ep_in->bulk.callback = NULL;
                                
                                (*ep_in_callback)(ep_in_context, (ep_index | 0x80));
                            }
                        }
                    }
                }
            }
            
            usb_istr = USB->ISTR;
        }
        while (usb_istr & USB_ISTR_CTR);
    }
    
    USB->ISTR = (uint16_t)~(USB_ISTR_PMAOVR | USB_ISTR_ERR | USB_ISTR_ESOF | USB_ISTR_L1REQ);

    if (usb_istr & (USB_ISTR_WKUP | USB_ISTR_SUSP | USB_ISTR_RESET | USB_ISTR_SOF))
    {
        if (usb_istr & USB_ISTR_WKUP)
        {
            armv7m_atomic_andh(&USB->CNTR, ~(USB_CNTR_LPMODE | USB_CNTR_FSUSP | USB_CNTR_RESUME));
            
            USB->ISTR = (uint16_t)~USB_ISTR_WKUP;
            
            stm32wb_lptim_timeout_stop(&stm32wb_usbd_dcd_device.timeout);
            
            if (stm32wb_usbd_dcd_device.state == STM32WB_USBD_DCD_STATE_SUSPENDED)
            {
                stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP_0);
                
                stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_CONNECTED;
                
                events |= STM32WB_USBD_DCD_EVENT_RESUME;
            }
        }
        
        if (usb_istr & USB_ISTR_SUSP)
        {
            armv7m_atomic_orh(&USB->CNTR, USB_CNTR_FSUSP);
            
            USB->ISTR = (uint16_t)~USB_ISTR_SUSP;

            armv7m_atomic_orh(&USB->CNTR, USB_CNTR_LPMODE);
            
            if (stm32wb_usbd_dcd_device.state == STM32WB_USBD_DCD_STATE_CONNECTED)
            {
                stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP_0);

                stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_SUSPENDED;

                events |= STM32WB_USBD_DCD_EVENT_SUSPEND;
            }
        }
        
        if (usb_istr & USB_ISTR_RESET)
        {
            armv7m_atomic_orh(&USB->CNTR, USB_CNTR_FRES);
            
            USB->ISTR = (uint16_t)~USB_ISTR_RESET;
            USB->DADDR = 0;
            
            stm32wb_lptim_timeout_stop(&stm32wb_usbd_dcd_device.timeout);
            
            armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_RESUME);
            
            NVIC_DisableIRQ(USB_HP_IRQn);
            NVIC_DisableIRQ(USB_LP_IRQn);
            
            events |= STM32WB_USBD_DCD_EVENT_RESET;
        }
        
        if (usb_istr & USB_ISTR_SOF)
        {
            USB->ISTR = (uint16_t)~USB_ISTR_SOF;
            
            if (USB->CNTR & USB_CNTR_SOFM)
            {
                events |= STM32WB_USBD_DCD_EVENT_SOF;
            }
        }
    }

    if (stm32wb_usbd_dcd_device.bcd_state == STM32WB_USBD_DCD_BCD_STATE_DONE)
    {
        stm32wb_usbd_dcd_device.bcd_state = STM32WB_USBD_DCD_BCD_STATE_STOP;
        
        events |= STM32WB_USBD_DCD_EVENT_DETECT;
    }

    if (stm32wb_usbd_dcd_device.ep0_request)
    {
        stm32wb_usbd_dcd_device.ep0_request = false;

        events |= STM32WB_USBD_DCD_EVENT_EP0_REQUEST;
    }
    
    if (events)
    {
        (*stm32wb_usbd_dcd_device.evt_callback)(stm32wb_usbd_dcd_device.evt_context, events);
    }
}

void USB_LP_IRQHandler(void)
{
    stm32wb_usbd_dcd_interrupt();

    __DSB();
}

void USB_HP_IRQHandler(void)
{
    stm32wb_usbd_dcd_interrupt();

    __DSB();
}

void CLK48_HSEMHandler(void)
{
    stm32wb_usbd_dcd_clk48();
}
