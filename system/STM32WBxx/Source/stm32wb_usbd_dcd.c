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
#include "stm32wb_rtc.h"
#include "stm32wb_hsem.h"
#include "stm32wb_usbd.h"
#include "stm32wb_usbd_dcd.h"

#define STM32WB_USBD_DCD_STATE_NONE                       0
#define STM32WB_USBD_DCD_STATE_INIT                       1
#define STM32WB_USBD_DCD_STATE_NOT_READY                  2
#define STM32WB_USBD_DCD_STATE_READY                      3
#define STM32WB_USBD_DCD_STATE_RESET                      4
#define STM32WB_USBD_DCD_STATE_CONNECTED                  5
#define STM32WB_USBD_DCD_STATE_SLEEP                      6
#define STM32WB_USBD_DCD_STATE_SUSPENDED                  7

#define STM32WB_USBD_DCD_EP_COUNT                         8

#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK             0x0003
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_SHIFT            0
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_CONTROL          0x0000
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO              0x0001
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_BULK             0x0002
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_INTERRUPT        0x0003
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_DBL              0x0004
#define STM32WB_USBD_DCD_EP_CONTROL_STALLED               0x0008
#define STM32WB_USBD_DCD_EP_CONTROL_ENABLED               0x0010
#define STM32WB_USBD_DCD_EP_CONTROL_STARTED               0x0020
#define STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED            0x0040
#define STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS_MASK      0xff80
#define STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS_SHIFT     6

#define STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(_control) (((_control) & STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS_MASK) >> STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS_SHIFT)

typedef struct _stm32wb_usbd_dcd_ep_in_t {
    uint16_t                                   control;
    uint16_t                                   size;
    union {
	struct {
	    stm32wb_usbd_dcd_ep_in_callback_t  callback;
	    void                               *context;
            const uint8_t                      *data;
	    uint16_t                           length;
            uint16_t                           count;
	} bulk;
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
	struct {
	    stm32wb_usbd_dcd_ep_iso_callback_t callback;
	    void                               *context;
            stm32wb_usbd_dcd_fifo_t            *fifo;
            uint16_t                           length[2];
	} iso;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
    };
} stm32wb_usbd_dcd_ep_in_t;

typedef struct __stm32wb_usbd_dcd_ep_out_t {
    uint16_t                                   control;
    uint16_t                                   size;
    union {
	struct {
	    stm32wb_usbd_dcd_ep_out_callback_t callback;
	    void                               *context;
	    uint8_t                            *data;
	    uint16_t                           length;
	    uint16_t                           count;
	} bulk;
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
	struct {
	    stm32wb_usbd_dcd_ep_iso_callback_t callback;
	    void                               *context;
            stm32wb_usbd_dcd_fifo_t            *fifo;
	} iso;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
    };
} stm32wb_usbd_dcd_ep_out_t;

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
typedef void (*stm32wb_usbd_dcd_iso_in_routine_t)(uint32_t ep_index, stm32wb_usbd_dcd_ep_in_t *ep_in);
typedef void (*stm32wb_usbd_dcd_iso_out_routine_t)(uint32_t ep_index, stm32wb_usbd_dcd_ep_out_t *ep_out);
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */

typedef struct __stm32wb_usbd_dcd_device_t {
    volatile uint8_t                   state;
    volatile uint8_t                   clk48_on;
    volatile uint8_t                   clk48_sync;
    volatile uint8_t                   lpm_state;
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
    volatile uint32_t                  reference;
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
    uint16_t                           pma_btable;
    uint16_t                           pma_address;
    volatile uint32_t                  events;
    stm32wb_usbd_dcd_event_callback_t  evt_callback;
    void                               *evt_context;
#if (STM32WB_USBD_DCD_SOF_SUPPORTED == 1)
    stm32wb_usbd_dcd_sof_callback_t    sof_callback;
    void                               *sof_context;
#endif /* STM32WB_USBD_DCD_SOF_SUPPORTED == 1 */
    uint8_t                            *ep0_setup;
    uint8_t                            *ep0_data;
    uint8_t                            ep0_count;
    uint8_t                            ep0_length;
    stm32wb_usbd_dcd_ep_in_t           ep_in[STM32WB_USBD_DCD_EP_COUNT-1];  /* TX */
    stm32wb_usbd_dcd_ep_out_t          ep_out[STM32WB_USBD_DCD_EP_COUNT-1]; /* RX */
  //    uint8_t                            pma_index[STM32WB_USBD_DCD_EP_COUNT-1];
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
    stm32wb_usbd_dcd_iso_in_routine_t  iso_in_routine;
    stm32wb_usbd_dcd_iso_out_routine_t iso_out_routine;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
} stm32wb_usbd_dcd_device_t;

static stm32wb_usbd_dcd_device_t stm32wb_usbd_dcd_device;

static const uint16_t stm32wb_usbd_dcd_xlate_ep_type[4] = {
    USB_EP_CONTROL,
    USB_EP_ISOCHRONOUS,
    USB_EP_BULK,
    USB_EP_INTERRUPT,
};

#define USB_EP_BASE                                (USB1_BASE + 0x0000)
#define USB_PMA_BASE                               (USB1_BASE + 0x0400)
#define USB_PMA_SIZE                               0x0400

#define USB_BTABLE_SIZE                            (STM32WB_USBD_DCD_EP_COUNT * 8)

#define USB_EP0_SIZE                               STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE
#define USB_EP0_PMA_ADDRESS                        (USB_PMA_SIZE - STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE)

#define USB_EP_R(_ep_index)                        (*((volatile uint16_t*)(USB_EP_BASE + ((_ep_index) * 4))))
#define USB_EP_W(_ep_index, _data)                 (*((volatile uint16_t*)(USB_EP_BASE + ((_ep_index) * 4))) = (_data))

#define USB_LPMCSR_LPMEN                           USB_LPMCSR_LMPEN 

#define USB_EP_SWBUF_TX                            USB_EP_DTOG_RX
#define USB_EP_SWBUF_RX                            USB_EP_DTOG_TX

#define USB_EP_STAT_RX_MASK                        USB_EPRX_STAT                             
#define USB_EP_STAT_TX_MASK                        USB_EPTX_STAT                             
#define USB_EP_EA_MASK                             USB_EPADDR_FIELD
  
#define USBP_EP_RW_MASK                            (USB_EP_TYPE_MASK | USB_EP_KIND | USB_EP_EA_MASK)

#define USB_EP_CONFIG(_ep_index, _config)          USB_EP_W((_ep_index), (USB_EP_CTR_RX | USB_EP_CTR_TX | (_config)))
#define USB_EP_STAT_TX(_ep_index, _stat)           USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USBP_EP_RW_MASK | USB_EP_STAT_TX_MASK)) ^ (USB_EP_CTR_RX | USB_EP_CTR_TX | (_stat))))
#define USB_EP_CTR_TX_RESET(_ep_index)             USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USBP_EP_RW_MASK)) ^ (USB_EP_CTR_RX)))
#define USB_EP_DTOG_TX_RESET(_ep_index)            USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USBP_EP_RW_MASK | USB_EP_DTOG_TX)) ^ (USB_EP_CTR_RX | USB_EP_CTR_TX)))
#define USB_EP_STAT_RX(_ep_index, _stat)           USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USBP_EP_RW_MASK | USB_EP_STAT_RX_MASK)) ^ (USB_EP_CTR_RX | USB_EP_CTR_TX | (_stat))))
#define USB_EP_CTR_RX_RESET(_ep_index)             USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USBP_EP_RW_MASK)) ^ (USB_EP_CTR_TX)))
#define USB_EP_DTOG_RX_RESET(_ep_index)            USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USBP_EP_RW_MASK | USB_EP_DTOG_RX)) ^ (USB_EP_CTR_RX | USB_EP_CTR_TX)))
#define USB_EP_DTOG_TX_STATUS(_ep_index)           ((USB_EP_R((_ep_index)) & (USB_EP_DTOG_TX)) ? 1 : 0)
#define USB_EP_DTOG_RX_STATUS(_ep_index)           ((USB_EP_R((_ep_index)) & (USB_EP_DTOG_RX)) ? 1 : 0)

#if 0
#define USB_EP_KIND_SET(_ep_index)                 USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USB_EP_TYPE_MASK | USB_EP_EA_MASK)) ^ (USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_KIND)))
#define USB_EP_KIND_RESET(_ep_index)               USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USB_EP_TYPE_MASK | USB_EP_EA_MASK)) ^ (USB_EP_CTR_RX | USB_EP_CTR_TX)))
#define USB_EP_DTOG_SWBUF_TX_RESET(_ep_index)      USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USBP_EP_RW_MASK | USB_EP_DTOG_TX | USB_EP_SWBUF_TX)) ^ (USB_EP_CTR_RX | USB_EP_CTR_TX)))
#define USB_EP_DTOG_SWBUF_RX_RESET(_ep_index)      USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USBP_EP_RW_MASK | USB_EP_DTOG_RX | USB_EP_SWBUF_RX)) ^ (USB_EP_CTR_RX | USB_EP_CTR_TX)))
#define USB_EP_SWBUF_TX_TOGGLE(_ep_index)          USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USBP_EP_RW_MASK)) ^ (USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_SWBUF_TX)))
#define USB_EP_SWBUF_RX_TOGGLE(_ep_index)          USB_EP_W((_ep_index), ((USB_EP_R((_ep_index)) & (USBP_EP_RW_MASK)) ^ (USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_SWBUF_RX)))
#define USB_EP_SWBUF_TX_STATUS(_ep_index)          ((USB_EP_R((_ep_index)) & (USB_EP_SWBUF_TX)) ? 1 : 0)
#define USB_EP_SWBUF_RX_STATUS(_ep_index)          ((USB_EP_R((_ep_index)) & (USB_EP_SWBUF_RX)) ? 1 : 0)
#endif

#define USB_PMA_TX_ADDRESS(_pma_index, _address)    (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 0)) = (_address))
#define USB_PMA_TX_COUNT(_pma_index, _count)        (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 2)) = (_count))
#define USB_PMA_TX_0_ADDRESS(_pma_index, _address)  (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 0)) = (_address))
#define USB_PMA_TX_1_ADDRESS(_pma_index, _address)  (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 4)) = (_address))
#define USB_PMA_TX_0_COUNT(_pma_index, _count)      (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 2)) = (_count))
#define USB_PMA_TX_1_COUNT(_pma_index, _count)      (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 6)) = (_count))
#define USB_PMA_RX_ADDRESS(_pma_index, _address)    (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 4)) = (_address))
#define USB_PMA_RX_SIZE(_pma_index, _size)          (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 6)) = (_size))
#define USB_PMA_RX_COUNT(_pma_index)                (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 6)) & 0x03ff)
#define USB_PMA_RX_0_ADDRESS(_pma_index, _address)  (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 0)) = (_address))
#define USB_PMA_RX_1_ADDRESS(_pma_index, _address)  (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 4)) = (_address))
#define USB_PMA_RX_0_SIZE(_pma_index, _size)        (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 2)) = (_size))
#define USB_PMA_RX_1_SIZE(_pma_index, _size)        (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 6)) = (_size))
#define USB_PMA_RX_0_COUNT(_pma_index)              (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 2)) & 0x03ff)
#define USB_PMA_RX_1_COUNT(_pma_index)              (*((volatile uint16_t*)(USB_PMA_BASE + ((_pma_index) * 8) + 6)) & 0x03ff)

static void __attribute__((optimize("O3"), noinline)) stm32wb_usbd_dcd_pma_write(uint32_t pma_address, const uint8_t *data, uint32_t count)
{
    volatile uint16_t *pma;
    const uint16_t *data16, *data16_e;

    pma = (volatile uint16_t*)(USB_PMA_BASE + pma_address);

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

static void __attribute__((optimize("O3"), noinline)) stm32wb_usbd_dcd_pma_read(uint32_t pma_address, uint8_t *data, uint32_t count)
{
    volatile uint16_t *pma;
    uint16_t *data16, *data16_e;

    pma = (volatile uint16_t*)(USB_PMA_BASE + pma_address);
    
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
    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_NONE)
    {
	return false;
    }

    stm32wb_usbd_dcd_device.pma_btable = 8;
    stm32wb_usbd_dcd_device.pma_address = USB_EP0_PMA_ADDRESS;

    NVIC_SetPriority(USB_HP_IRQn, STM32WB_USBD_DCD_IRQ_PRIORITY);
    NVIC_SetPriority(USB_LP_IRQn, STM32WB_USBD_DCD_IRQ_PRIORITY);
    NVIC_SetPriority(CRS_IRQn, STM32WB_USBD_DCD_IRQ_PRIORITY);
    
    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_INIT;
    stm32wb_usbd_dcd_device.clk48_on = false;
    stm32wb_usbd_dcd_device.clk48_sync = false;

    return true;
}

bool stm32wb_usbd_dcd_enable(uint8_t *setup, stm32wb_usbd_dcd_event_callback_t callback, void *context)
{
    // armv7m_rtt_printf("DCD_ENABLE\n");

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_INIT)
    {
	return false;
    }

    if (stm32wb_system_pclk1() < 10000000)
    {
	return false;
    }
    
    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_NOT_READY;
    stm32wb_usbd_dcd_device.events = 0;

    stm32wb_usbd_dcd_device.evt_callback = callback;
    stm32wb_usbd_dcd_device.evt_context = context;
    stm32wb_usbd_dcd_device.ep0_setup = setup;

    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA11_USB_DM, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA12_USB_DP, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));

    stm32wb_system_reference(STM32WB_SYSTEM_REFERENCE_USB);

    stm32wb_usbd_dcd_device.lpm_state = STM32WB_USBD_DCD_LPM_STATE_L3;
    
    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_READY;
    
    return true;
}

bool stm32wb_usbd_dcd_disable(void)
{
    // armv7m_rtt_printf("DCD_DISABLE\n");

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_READY)
    {
	return false;
    }

    stm32wb_system_unreference(STM32WB_SYSTEM_REFERENCE_USB);
    
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA11_USB_DM, STM32WB_GPIO_MODE_ANALOG);
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA12_USB_DP, STM32WB_GPIO_MODE_ANALOG);

    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_INIT;

    return true;
}

bool stm32wb_usbd_dcd_connect(bool disconnect)
{
    // armv7m_rtt_printf("DCD_CONNECT(disconnect=%d)\n", disconnect);

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_READY)
    {
	return false;
    }

    if (disconnect)
    {
        /* Signal a DISCONNECT on the DP line by driving it low for 50uS. Specs says 2.5uS, but leave some extra margin.
         */
        stm32wb_gpio_pin_write(STM32WB_GPIO_PIN_PA12_USB_DP, 0);
        stm32wb_gpio_pin_output(STM32WB_GPIO_PIN_PA12_USB_DP);
        
        armv7m_core_udelay(50);

        stm32wb_gpio_pin_alternate(STM32WB_GPIO_PIN_PA12_USB_DP);
    }
    
    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_STOP_1);

    // armv7m_rtt_printf("DCD_LOCK\n");

    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);

    armv7m_atomic_or(&PWR->CR2, PWR_CR2_USV);

    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_USB);
    
    armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_PDWN);

    armv7m_core_udelay(50);
    
    USB->DADDR = 0;
    USB->BTABLE = 0;
    USB->LPMCSR = 0;
    USB->BCDR = 0;
    
    armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_FRES);

    USB->ISTR = (uint16_t)0;
    
    armv7m_atomic_or(&EXTI->IMR1, EXTI_IMR1_IM28);
    
    NVIC_EnableIRQ(USB_LP_IRQn);
    // NVIC_EnableIRQ(USB_HP_IRQn);
    NVIC_EnableIRQ(CRS_IRQn);

    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_CONNECTED;
    
    armv7m_atomic_or(&stm32wb_usbd_dcd_device.events, STM32WB_USBD_DCD_EVENT_CONNECT);
        
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_DCD);
    
    return true;
}

bool stm32wb_usbd_dcd_disconnect(void)
{
    // armv7m_rtt_printf("DCD_DISCONNECT\n");

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_RESET)
    {
	return false;
    }

#if (STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1)
    stm32wb_rtc_wakeup_stop();
#endif /* STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1 */

    NVIC_DisableIRQ(CRS_IRQn);
    // NVIC_DisableIRQ(USB_HP_IRQn);
    NVIC_DisableIRQ(USB_LP_IRQn);

    armv7m_atomic_and(&EXTI->IMR1, ~EXTI_IMR1_IM28);

    USB->CNTR = USB_CNTR_FRES;
    USB->ISTR = (uint16_t)0;
    USB->DADDR = 0;
    USB->LPMCSR = 0;
    USB->BCDR = 0;

    USB->CNTR |= USB_CNTR_PDWN;

    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_USB);
    
    armv7m_atomic_and(&PWR->CR2, ~PWR_CR2_USV);
    
    /* Signal a DISCONNECT on the DP line by driving it low for 50uS. Specs says 2.5uS, but leave some extra margin.
     */
    stm32wb_gpio_pin_write(STM32WB_GPIO_PIN_PA12_USB_DP, 0);
    stm32wb_gpio_pin_output(STM32WB_GPIO_PIN_PA12_USB_DP);

    armv7m_core_udelay(50);

    stm32wb_gpio_pin_alternate(STM32WB_GPIO_PIN_PA12_USB_DP);

    stm32wb_usbd_dcd_device.lpm_state = STM32WB_USBD_DCD_LPM_STATE_L3;

    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_READY;

    if (stm32wb_usbd_dcd_device.state <= STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);

        // armv7m_rtt_printf("DCD_UNLOCK\n");
    }

    stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_STOP_1);
    
    stm32wb_usbd_dcd_device.events = STM32WB_USBD_DCD_EVENT_DISCONNECT;
    
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_DCD);

    return true;
}

bool stm32wb_usbd_dcd_reset(void)
{
    uint32_t ep_index;
    
    // armv7m_rtt_printf("DCD_RESET\n");

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_RESET)
    {
	return false;
    }
    
    stm32wb_usbd_dcd_device.ep0_count = 0;
    stm32wb_usbd_dcd_device.ep0_length = 0;

    USB_EP_CTR_RX_RESET(0);
    USB_EP_CTR_TX_RESET(0);
    
    for (ep_index = 1; ep_index < STM32WB_USBD_DCD_EP_COUNT; ep_index++)
    {
        stm32wb_usbd_dcd_device.ep_in[ep_index -1].control &= ~(STM32WB_USBD_DCD_EP_CONTROL_STARTED | STM32WB_USBD_DCD_EP_CONTROL_ENABLED | STM32WB_USBD_DCD_EP_CONTROL_STALLED);
        stm32wb_usbd_dcd_device.ep_in[ep_index -1].bulk.callback = NULL;
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        stm32wb_usbd_dcd_device.ep_in[ep_index -1].iso.callback = NULL;
        stm32wb_usbd_dcd_device.ep_in[ep_index -1].iso.fifo = NULL;
        stm32wb_usbd_dcd_device.ep_in[ep_index -1].iso.length[0] = 0;
        stm32wb_usbd_dcd_device.ep_in[ep_index -1].iso.length[1] = 0;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
        stm32wb_usbd_dcd_device.ep_out[ep_index -1].control &= ~(STM32WB_USBD_DCD_EP_CONTROL_STARTED | STM32WB_USBD_DCD_EP_CONTROL_ENABLED | STM32WB_USBD_DCD_EP_CONTROL_STALLED);
        stm32wb_usbd_dcd_device.ep_out[ep_index -1].bulk.callback = NULL;
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        stm32wb_usbd_dcd_device.ep_out[ep_index -1].iso.callback = NULL;
        stm32wb_usbd_dcd_device.ep_out[ep_index -1].iso.fifo = NULL;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */

        USB_EP_CTR_RX_RESET(ep_index);
        USB_EP_CTR_TX_RESET(ep_index);
    }
    
    armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_FRES);

    USB->ISTR = (uint16_t)0;
    
    USB_PMA_TX_ADDRESS(0, USB_EP0_PMA_ADDRESS);
    USB_PMA_RX_ADDRESS(0, USB_EP0_PMA_ADDRESS);
    USB_PMA_RX_SIZE(0, 0x8400);                     // 8 = 0x1000, 16 = 0x2000, 32 = 0x4000, 64 = 0x8400
    
    USB_EP_W(0, USB_EP_CONTROL);
    USB_EP_STAT_TX(0, USB_EP_TX_NAK);
    USB_EP_STAT_RX(0, USB_EP_RX_NAK);

    USB->DADDR = USB_DADDR_EF | 0x00;
    USB->LPMCSR = USB_LPMCSR_LPMEN | USB_LPMCSR_LPMACK;
    
    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_CONNECTED;
    
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
    stm32wb_usbd_dcd_device.reference = 0;

    armv7m_atomic_orh(&USB->CNTR, (USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_RESETM | USB_CNTR_L1REQM));
#else /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
    armv7m_atomic_orh(&USB->CNTR, (USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_RESETM));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */

#if (STM32WB_USBD_DCD_SOF_SUPPORTED == 1)
    if (stm32wb_usbd_dcd_device.sof_callback)
    {
        armv7m_atomic_orh(&USB->CNTR, USB_CNTR_SOFM);
    }
#endif /* STM32WB_USBD_DCD_SOF_SUPPORTED == 1 */

    return true;
}

bool stm32wb_usbd_dcd_address(uint8_t address)
{
    // armv7m_rtt_printf("DCD_ADDRESS(address=%02x)\n", address);

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }

    USB->DADDR = (USB_DADDR_EF | address);
    
    return true;
}

#if (STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1)

static void stm32wb_usbd_dcd_wakeup_unset_resume(void)
{
    uint16_t usb_cntr;
    
    // armv7m_rtt_printf("DCD_WAKEUP_UNSET_RESUME(cntr=%04x, istr=%04x)\n", USB->CNTR, USB->ISTR);

    if (stm32wb_usbd_dcd_device.lpm_state == STM32WB_USBD_DCD_LPM_STATE_L0)
    {
        do
        {
            usb_cntr = USB->CNTR;
            
            if (!(usb_cntr & USB_CNTR_FSUSP))
            {
                return;
            }
        }
        while (armv7m_atomic_cash(&USB->CNTR, usb_cntr, (usb_cntr & ~USB_CNTR_RESUME)) != usb_cntr);
        
        armv7m_atomic_or(&stm32wb_usbd_dcd_device.events, STM32WB_USBD_DCD_EVENT_RESUME);
        
        armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_DCD);
    }
}

static void stm32wb_usbd_dcd_wakeup_set_resume(void)
{
    uint16_t usb_cntr;
    
    // armv7m_rtt_printf("DCD_WAKEUP_SET_RESUME(cntr=%04x, istr=%04x)\n", USB->CNTR, USB->ISTR);

    if (stm32wb_usbd_dcd_device.lpm_state == STM32WB_USBD_DCD_LPM_STATE_L0)
    {
        do
        {
            usb_cntr = USB->CNTR;
            
            if (!(usb_cntr & USB_CNTR_FSUSP))
            {
                return;
            }
        }
        while (armv7m_atomic_cash(&USB->CNTR, usb_cntr, (usb_cntr | USB_CNTR_RESUME)) != usb_cntr);
        
        stm32wb_rtc_wakeup_start(stm32wb_rtc_millis_to_ticks(5), (stm32wb_rtc_wakeup_callback_t)stm32wb_usbd_dcd_wakeup_unset_resume, NULL);
    }
}

bool stm32wb_usbd_dcd_wakeup(void)
{
    uint8_t lpm_state;

    // armv7m_rtt_printf("DCD_WAKEUP(cntr=%04x, istr=%04x, %d)\n", USB->CNTR, USB->ISTR, (uint32_t)armv7m_systick_micros());

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }

    if (stm32wb_usbd_dcd_device.lpm_state == STM32WB_USBD_DCD_LPM_STATE_L2)
    {
        lpm_state = armv7m_atomic_swapb(&stm32wb_usbd_dcd_device.lpm_state, STM32WB_USBD_DCD_LPM_STATE_L0);

        if (lpm_state == STM32WB_USBD_DCD_LPM_STATE_L2)
        {
            // armv7m_rtt_printf("DCD_WAKEUP_RESUME\n");

            armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_LPMODE);

            // armv7m_rtt_printf("DCD_LOCK\n");

            stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
            
            stm32wb_rtc_wakeup_start(stm32wb_rtc_millis_to_ticks(2), (stm32wb_rtc_wakeup_callback_t)stm32wb_usbd_dcd_wakeup_set_resume, NULL);
        }
    }

    return true;
}

#endif /* STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1 */

#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)

bool stm32wb_usbd_dcd_lpm_reference(uint32_t reference)
{
    uint8_t lpm_state;
    uint16_t usb_cntr;

    armv7m_atomic_or(&stm32wb_usbd_dcd_device.reference, reference);
    armv7m_atomic_andh(&USB->LPMCSR, ~USB_LPMCSR_LPMACK);

    // armv7m_rtt_printf("DCD_LPM_REFERENCE(reference=%08x) = %s\n", reference, ((USB->LPMCSR & USB_LPMCSR_LPMACK) ? "ACK" : "NYET"));

    if (stm32wb_usbd_dcd_device.lpm_state == STM32WB_USBD_DCD_LPM_STATE_L1)
    {
        if (USB->LPMCSR & USB_LPMCSR_REMWAKE)
        {
            lpm_state = armv7m_atomic_swapb(&stm32wb_usbd_dcd_device.lpm_state, STM32WB_USBD_DCD_LPM_STATE_L0);
          
            if (lpm_state == STM32WB_USBD_DCD_LPM_STATE_L1)
            {
                // armv7m_rtt_printf("DCD_LPM_RESUME\n");
                
                armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_LPMODE);

                do
                {
                    usb_cntr = USB->CNTR;

                    if (!(usb_cntr & USB_CNTR_FSUSP))
                    {
                        return true;
                    }
                }
                while (armv7m_atomic_cash(&USB->CNTR, usb_cntr, (usb_cntr | USB_CNTR_L1RESUME)) != usb_cntr);

                // armv7m_rtt_printf("DCD_LOCK\n");

                stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
                
                armv7m_atomic_or(&stm32wb_usbd_dcd_device.events, STM32WB_USBD_DCD_EVENT_RESUME);
                
                armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_DCD);

                return true;
            }
        }
    }

    return (stm32wb_usbd_dcd_device.lpm_state == STM32WB_USBD_DCD_LPM_STATE_L1);
}

void stm32wb_usbd_dcd_lpm_unreference(uint32_t reference)
{
    if (armv7m_atomic_and(&stm32wb_usbd_dcd_device.reference, ~reference) == reference)
    {
        armv7m_atomic_orhz(&USB->LPMCSR, USB_LPMCSR_LPMACK, &stm32wb_usbd_dcd_device.reference, ~0ul);
    }

    // armv7m_rtt_printf("DCD_LPM_UNREFERENCE(reference=%08x) = %s\n", reference, ((USB->LPMCSR & USB_LPMCSR_LPMACK) ? "ACK" : "NYET"));
}

#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */

bool stm32wb_usbd_dcd_ep0_transmit(const uint8_t *data, uint16_t length)
{
    // armv7m_rtt_printf("DCD_EP0_TRANSMIT(data=%08x, length=%d)\n", data, length);
    
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }
    
    if (length > STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE)
    {
	return false;
    }

    if (length)
    {
	stm32wb_usbd_dcd_pma_write(USB_EP0_PMA_ADDRESS, data, length);
	
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
    // armv7m_rtt_printf("DCD_EP0_RECEIVE(data=%08x, length=%d)\n", data, length);

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
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
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return 0;
    }

    return stm32wb_usbd_dcd_device.ep0_count;
}

bool stm32wb_usbd_dcd_ep0_stall(void)
{
    // armv7m_rtt_printf("DCD_EP0_STALL()\n");

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }

    USB_EP_STAT_TX(0, USB_EP_TX_STALL);
    USB_EP_STAT_RX(0, USB_EP_RX_STALL);
    
    return true;
}

static uint32_t stm32wb_usbd_dcd_pma_allocate(uint32_t ep_index, uint32_t pma_address, uint32_t pma_size)
{
    uint32_t pma_btable;
    
    pma_btable = (ep_index + 1) * 8;

    if (pma_btable < stm32wb_usbd_dcd_device.pma_btable)
    {
        pma_btable = stm32wb_usbd_dcd_device.pma_btable;
    }
    
    if (pma_address)
    {
        if (pma_address < pma_btable)
        {
            return 0;
        }
    }
    else
    {
        if (pma_size > (uint32_t)(stm32wb_usbd_dcd_device.pma_address - pma_btable))
        {
            return 0;
        }

        stm32wb_usbd_dcd_device.pma_address -= pma_size;
            
        pma_address = stm32wb_usbd_dcd_device.pma_address;
    }
        
    stm32wb_usbd_dcd_device.pma_btable = pma_btable;

    return pma_address;
}

bool stm32wb_usbd_dcd_ep_configure(uint8_t ep_addr, uint8_t type, uint16_t size, uint16_t pma_address, uint16_t *p_pma_address_return)
{
    uint32_t ep_index, ep_type, ep_dbl, pma_size;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_CONFIGURE(ep_addr=%02x, type=%02x, size=%d)\n", ep_addr, type, size);

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_INIT)
    {
	return false;
    }

    if (size == 0)
    {
	return false;
    }

    ep_index = ep_addr & 0x07;
    ep_type = type & 3;
#if (STM32WB_USBD_DCD_DBL_SUPPORTED == 1)
    ep_dbl = type & 4;
#else /* STM32WB_USBD_DCD_DBL_SUPPORTED == 1 */
    ep_dbl = 0;
#endif /* STM32WB_USBD_DCD_DBL_SUPPORTED == 1 */
    
    ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];
    ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

    if (ep_addr & 0x80)
    {
        if (ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED)
	{
	    return false;
	}

        size = (size + 1) & ~1;
        
	if (ep_type == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
	{
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
            if (ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED)
	    {
		return false;
	    }
	    
	    pma_size = size * 2;
#else /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
            return false;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
	}
	else
	{
            if ((ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED) && ((ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) != ep_type))
            {
                return false;
            }

#if (STM32WB_USBD_DCD_DBL_SUPPORTED == 1)
            if (ep_dbl)
            {
                pma_size = size * 2;
            }
            else
#endif /* STM32WB_USBD_DCD_DBL_SUPPORTED == 1 */
            {
                pma_size = size;
            }
	}

        pma_address = stm32wb_usbd_dcd_pma_allocate(ep_index, pma_address, pma_size);

        if (!pma_address)
        {
            return false;
        }
        
	ep_in->control = (pma_address << STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS_SHIFT) | STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | ep_type | ep_dbl;
	ep_in->size = size;
    }
    else
    {
        if (ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED)
	{
	    return false;
	}

        if (size >= 64)
        {
            size = (size + 31) & ~31;
        }
        else
        {
            size = (size + 1) & ~1;
        }

	if (ep_type == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
	{
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
            if (ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED)
	    {
		return false;
	    }
	    
	    pma_size = size * 2;
#else /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
            return false;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
	}
	else
	{
            if ((ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED) && ((ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) != ep_type))
            {
                return false;
            }

#if (STM32WB_USBD_DCD_DBL_SUPPORTED == 1)
            if (ep_dbl)
            {
                pma_size = size * 2;
            }
            else
#endif /* STM32WB_USBD_DCD_DBL_SUPPORTED == 1 */
            {
                pma_size = size;
            }
	}

        pma_address = stm32wb_usbd_dcd_pma_allocate(ep_index, pma_address, pma_size);

        if (!pma_address)
        {
            return false;
        }
        
	ep_out->control = (pma_address << STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS_SHIFT) | STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | ep_type | ep_dbl;
	ep_out->size = size;
    }

    if (p_pma_address_return)
    {
        *p_pma_address_return = pma_address;
    }
    
    return true;
}

bool stm32wb_usbd_dcd_ep_enable(uint8_t ep_addr)
{
    uint32_t ep_index, ep_type, ep_size;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_ENABLE(ep_addr=%02x)\n", ep_addr);

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }
    
    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
	ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

	if ((ep_in->control & (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_ENABLED)) != (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED))
	{
	    return false;
	}

	ep_type = ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK;

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
	if (ep_type == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
	{
            return false;
	}
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
	
	ep_in->control |= STM32WB_USBD_DCD_EP_CONTROL_ENABLED;

        USB_EP_CONFIG(ep_index, (stm32wb_usbd_dcd_xlate_ep_type[ep_type] | ep_index));

#if (STM32WB_USBD_DCD_DBL_SUPPORTED == 1)
	if (!(ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_DBL))
#endif /* STM32WB_USBD_DCD_DBL_SUPPORTED == 1 */
        {
            USB_PMA_TX_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control));
        }
        
        USB_EP_DTOG_TX_RESET(ep_index);
        USB_EP_STAT_TX(ep_index, USB_EP_TX_NAK);
    }
    else
    {
	ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

	if ((ep_out->control & (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_ENABLED)) != (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED))
	{
	    return false;
	}

	ep_type = ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK;
	
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
	if (ep_type == STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
	{
            return false;
	}
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
	
	ep_out->control |= STM32WB_USBD_DCD_EP_CONTROL_ENABLED;

	if (ep_out->size >= 64)
	{
	    ep_size = 0x8000 | (((ep_out->size - 32) / 32) << 10);
	}
	else
	{
	    ep_size = 0x0000 | ((ep_out->size / 2) << 10);
	}

        USB_EP_CONFIG(ep_index, (stm32wb_usbd_dcd_xlate_ep_type[ep_type] | ep_index));

#if (STM32WB_USBD_DCD_DBL_SUPPORTED == 1)
	if (!(ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_DBL))
#endif /* STM32WB_USBD_DCD_DBL_SUPPORTED == 1 */
        {
            USB_PMA_RX_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control));
        }
        
        USB_PMA_RX_SIZE(ep_index, ep_size);
	
        USB_EP_DTOG_RX_RESET(ep_index);
        USB_EP_STAT_RX(ep_index, USB_EP_RX_NAK);
    }

    // armv7m_rtt_printf("DCD_EP_STATUS(%d) = %04x\n", ep_index, USB_EP_R(ep_index));
    
    return true;
}

bool stm32wb_usbd_dcd_ep_disable(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_DISABLE(ep_addr=%02x)\n", ep_addr);
    
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }

    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
	ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

	if (!(ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_ENABLED))
	{
	    return false;
	}
	
        USB_EP_STAT_TX(ep_index, USB_EP_TX_DIS);
        USB_EP_CTR_TX_RESET(ep_index);
        
        ep_in->bulk.callback = NULL;

	ep_in->control &= ~(STM32WB_USBD_DCD_EP_CONTROL_ENABLED | STM32WB_USBD_DCD_EP_CONTROL_STALLED);
    }
    else
    {
	ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

	if (!(ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_ENABLED))
	{
	    return false;
	}

        USB_EP_STAT_RX(ep_index, USB_EP_RX_DIS);
        USB_EP_CTR_RX_RESET(ep_index);

        ep_out->bulk.callback = NULL;

	ep_out->control &= ~(STM32WB_USBD_DCD_EP_CONTROL_ENABLED | STM32WB_USBD_DCD_EP_CONTROL_STALLED);
    }

    // armv7m_rtt_printf("DCD_EP_STATUS(%d) = %04x\n", ep_index, USB_EP_R(ep_index));

    return true;
}

bool stm32wb_usbd_dcd_ep_stall(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_STALL(ep_addr=%02x)\n", ep_addr);
    
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }
    
    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
	ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

	if ((ep_in->control & (STM32WB_USBD_DCD_EP_CONTROL_ENABLED | STM32WB_USBD_DCD_EP_CONTROL_STALLED)) != STM32WB_USBD_DCD_EP_CONTROL_ENABLED)
	{
	    return false;
	}

        USB_EP_STAT_TX(ep_index, USB_EP_TX_STALL);
        USB_EP_CTR_TX_RESET(ep_index);

	ep_in->bulk.callback = NULL;

	ep_in->control |= STM32WB_USBD_DCD_EP_CONTROL_STALLED;
    }
    else
    {
	ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

	if ((ep_out->control & (STM32WB_USBD_DCD_EP_CONTROL_ENABLED | STM32WB_USBD_DCD_EP_CONTROL_STALLED)) != STM32WB_USBD_DCD_EP_CONTROL_ENABLED)
	{
	    return false;
	}

        USB_EP_STAT_RX(ep_index, USB_EP_RX_STALL);
        USB_EP_CTR_RX_RESET(ep_index);

	ep_out->bulk.callback = NULL;

	ep_out->control |= STM32WB_USBD_DCD_EP_CONTROL_STALLED;
    }

    // armv7m_rtt_printf("DCD_EP_STATUS(%d) = %04x\n", ep_index, USB_EP_R(ep_index));
    
    return true;
}

bool stm32wb_usbd_dcd_ep_unstall(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_UNSTALL(ep_addr=%02x)\n", ep_addr);

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
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

        USB_EP_DTOG_TX_RESET(ep_index);
        USB_EP_STAT_TX(ep_index, USB_EP_TX_NAK);
    }
    else
    {
	ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

	if (!(ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_STALLED))
	{
	    return false;
	}

	ep_out->control &= ~STM32WB_USBD_DCD_EP_CONTROL_STALLED;

        USB_EP_DTOG_RX_RESET(ep_index);
        USB_EP_STAT_RX(ep_index, USB_EP_RX_NAK);
    }

    // armv7m_rtt_printf("DCD_EP_STATUS(%d) = %04x\n", ep_index, USB_EP_R(ep_index));
    
    return true;
}

bool stm32wb_usbd_dcd_ep_is_stalled(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_IS_STALLED(ep_addr=%02x)\n", ep_addr);
    
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
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
    uint32_t ep_index, pma_address, count;
#if (STM32WB_USBD_DCD_DBL_SUPPORTED == 1)
    uint32_t sequence;
#endif /* STM32WB_USBD_DCD_DBL_SUPPORTED == 1 */
    stm32wb_usbd_dcd_ep_in_t *ep_in;

    // armv7m_rtt_printf("DCD_EP_TRANSMIT(ep_addr=%02x, data=%08x, legnth=%d, callback=%08x, context=%0x)\n", ep_addr, data, length, callback, context);

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }
    
    ep_index = ep_addr & 0x07;

    ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];
    
    if ((ep_in->control & (STM32WB_USBD_DCD_EP_CONTROL_ENABLED | STM32WB_USBD_DCD_EP_CONTROL_STALLED)) != STM32WB_USBD_DCD_EP_CONTROL_ENABLED)
    {
        return false;
    }
    
    if (ep_in->bulk.callback)
    {
        return false;
    }
    
    ep_in->bulk.data = data;
    ep_in->bulk.length = length;
    ep_in->bulk.count = 0;
    ep_in->bulk.callback = callback;
    ep_in->bulk.context = context;

#if (STM32WB_USBD_DCD_DBL_SUPPORTED == 1)
    if (ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_DBL)
    {
        sequence = USB_EP_DTOG_TX_STATUS(ep_index);
            
        if (ep_in->bulk.length)
        {
            pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control) + (sequence ? ep_in->size : 0);
            
            count = ep_in->bulk.length;
            
            if (count > ep_in->size)
            {
                count = ep_in->size;
            }
            
            stm32wb_usbd_dcd_pma_write(pma_address, ep_in->bulk.data, count);
            
            ep_in->bulk.data += count;
            ep_in->bulk.length -= count;
            
            USB_PMA_TX_ADDRESS(ep_index, pma_address);
            USB_PMA_TX_COUNT(ep_index, count);
            
            if (ep_in->bulk.length)
            {
                count = ep_in->bulk.length;
                
                if (count > ep_in->size)
                {
                    count = ep_in->size;
                }
                
                if (count)
                {
                    pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control) + (sequence ? 0 : ep_in->size);
                    
                    stm32wb_usbd_dcd_pma_write(pma_address, ep_in->bulk.data, count);
                    
                    ep_in->bulk.count = count;
                }
            }
        }
        else
        {
            USB_PMA_TX_COUNT(ep_index, 0);
        }
    }
    else
#endif
    {
        if (ep_in->bulk.length)
        {
            pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control);
        
            count = ep_in->bulk.length;
        
            if (count > ep_in->size)
            {
                count = ep_in->size;
            }
        
            stm32wb_usbd_dcd_pma_write(pma_address, ep_in->bulk.data, count);
        
            ep_in->bulk.data += count;
            ep_in->bulk.length -= count;
        
            USB_PMA_TX_COUNT(ep_index, count);
        }
        else
        {
            USB_PMA_TX_COUNT(ep_index, 0);
        }
    }

    USB_EP_STAT_TX(ep_index, USB_EP_TX_VALID);
    
    return true;
}

bool stm32wb_usbd_dcd_ep_receive(uint8_t ep_addr, uint8_t *data, uint16_t length, stm32wb_usbd_dcd_ep_out_callback_t callback, void *context)
{
    uint32_t ep_index;
#if (STM32WB_USBD_DCD_DBL_SUPPORTED == 1)
    uint32_t pma_address, sequence;
#endif /* (STM32WB_USBD_DCD_DBL_SUPPORTED == 1) */
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_RECEIVE(ep_addr=%02x, data=%08x, legnth=%d, callback=%08x, context=%0x)\n", ep_addr, data, length, callback, context);

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }
    
    ep_index = ep_addr & 0x07;

    ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];
    
    if ((ep_out->control & (STM32WB_USBD_DCD_EP_CONTROL_ENABLED | STM32WB_USBD_DCD_EP_CONTROL_STALLED)) != STM32WB_USBD_DCD_EP_CONTROL_ENABLED)
    {
        return false;
    }

    if (ep_out->bulk.callback)
    {
        return false;
    }
    
    ep_out->bulk.data = data;
    ep_out->bulk.count = 0;
    ep_out->bulk.length = length;
    ep_out->bulk.callback = callback;
    ep_out->bulk.context = context;

#if (STM32WB_USBD_DCD_DBL_SUPPORTED == 1)
    if (ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_DBL)
    {
        sequence = USB_EP_DTOG_RX_STATUS(ep_index);

        pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control) + (sequence ? ep_out->size : 0);

        USB_PMA_RX_ADDRESS(ep_index, pma_address);
    }
#endif /* STM32WB_USBD_DCD_DBL_SUPPORTED == 1 */

    USB_EP_STAT_RX(ep_index, USB_EP_RX_VALID);

    return true;
}

bool stm32wb_usbd_dcd_ep_flush(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_FLUSH(ep_addr=%02x)\n", ep_addr);
    
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }
    
    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
	ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

        if ((ep_in->control & (STM32WB_USBD_DCD_EP_CONTROL_ENABLED | STM32WB_USBD_DCD_EP_CONTROL_STALLED)) != STM32WB_USBD_DCD_EP_CONTROL_ENABLED)
	{
	    return false;
	}

	ep_in->bulk.callback = NULL;

        USB_EP_STAT_TX(ep_index, USB_EP_TX_NAK);
        USB_EP_CTR_TX_RESET(ep_index);
    }
    else
    {
	ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

        if ((ep_out->control & (STM32WB_USBD_DCD_EP_CONTROL_ENABLED | STM32WB_USBD_DCD_EP_CONTROL_STALLED)) != STM32WB_USBD_DCD_EP_CONTROL_ENABLED)
	{
	    return false;
	}
	
	ep_out->bulk.callback = NULL;

        USB_EP_STAT_RX(ep_index, USB_EP_RX_NAK);
        USB_EP_CTR_RX_RESET(ep_index);
    }

    return true;
}

static void stm32wb_usbd_dcd_event(void)
{
    uint32_t events;
#if (STM32WB_USBD_DCD_LSE_SUPPORTED == 1)
    uint32_t lseclk;
#endif /* STM32WB_USBD_DCD_LSE_SUPPORTED == 1 */
    uint8_t clk48_on;

    events = armv7m_atomic_swap(&stm32wb_usbd_dcd_device.events, 0);

    // armv7m_rtt_printf("DCD_EVENT_ENTER(state=%d, lpm_state=%d, events=%08x, %d)\n", stm32wb_usbd_dcd_device.state, stm32wb_usbd_dcd_device.lpm_state, events, (uint32_t)armv7m_systick_micros());
    
    if (events & STM32WB_USBD_DCD_EVENT_RESET)
    {
        events &= ~(STM32WB_USBD_DCD_EVENT_EP0_SETUP | STM32WB_USBD_DCD_EVENT_EP0_DATA_IN | STM32WB_USBD_DCD_EVENT_EP0_DATA_OUT | STM32WB_USBD_DCD_EVENT_SLEEP | STM32WB_USBD_DCD_EVENT_SUSPEND | STM32WB_USBD_DCD_EVENT_RESUME);
    }

    clk48_on = stm32wb_usbd_dcd_device.clk48_on;

    if (events & (STM32WB_USBD_DCD_EVENT_DISCONNECT | STM32WB_USBD_DCD_EVENT_SLEEP | STM32WB_USBD_DCD_EVENT_SUSPEND))
    {
        clk48_on = false;
    }
    
    if (events & (STM32WB_USBD_DCD_EVENT_CONNECT | STM32WB_USBD_DCD_EVENT_RESUME | STM32WB_USBD_DCD_EVENT_RESET))
    {
        clk48_on = true;
    }
    
    if (stm32wb_usbd_dcd_device.clk48_on != clk48_on)
    {
        if (stm32wb_usbd_dcd_device.clk48_on)
        {
            if (!stm32wb_hsem_lock(STM32WB_HSEM_INDEX_RNG, STM32WB_HSEM_PROCID_USBD_DCD))
            {
                if (!stm32wb_hsem_is_locked(STM32WB_HSEM_INDEX_RNG, STM32WB_HSEM_PROCID_RANDOM))
                {
                    armv7m_atomic_or(&stm32wb_usbd_dcd_device.events, events);

                    // armv7m_rtt_printf("DCD_EVENT_LEAVE(CLK48_OFF)\n");
                    
                    return;
                }
            }

            CRS->CR &= ~(CRS_CR_AUTOTRIMEN | CRS_CR_CEN | CRS_CR_SYNCOKIE);

            armv7m_atomic_and(&RCC->APB1ENR1, ~RCC_APB1ENR1_CRSEN);
            
            stm32wb_system_clk48_disable();

            stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RNG, STM32WB_HSEM_PROCID_USBD_DCD);

            stm32wb_usbd_dcd_device.clk48_on = false;
            
            // armv7m_rtt_printf("DCD_CLK48_OFF(%d)\n", (uint32_t)armv7m_systick_micros());
        }
        else 
        {
            if (!stm32wb_hsem_lock(STM32WB_HSEM_INDEX_RNG, STM32WB_HSEM_PROCID_USBD_DCD))
            {
                if (!stm32wb_hsem_is_locked(STM32WB_HSEM_INDEX_RNG, STM32WB_HSEM_PROCID_RANDOM))
                {
                    armv7m_atomic_or(&stm32wb_usbd_dcd_device.events, events);

                    // armv7m_rtt_printf("DCD_EVENT_LEAVE(CLK48_ON)\n");
                    
                    return;
                }
            }

            stm32wb_system_clk48_enable();

            stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RNG, STM32WB_HSEM_PROCID_USBD_DCD);

            stm32wb_usbd_dcd_device.clk48_on = true;

            // armv7m_rtt_printf("DCD_CLK48_ON(%d)\n", (uint32_t)armv7m_systick_micros());

            armv7m_atomic_or(&RCC->APB1ENR1, RCC_APB1ENR1_CRSEN);
            RCC->APB1ENR1;

#if (STM32WB_USBD_DCD_LSE_SUPPORTED == 1)
            lseclk = stm32wb_system_lseclk();
            
            if (lseclk)
            {
                CRS->CFGR = (((((48000000 + (lseclk -1)) / lseclk) -1) << CRS_CFGR_RELOAD_Pos) | (1 << CRS_CFGR_FELIM_Pos) | CRS_CFGR_SYNCSRC_0);
            }
            else
#endif /* STM32WB_USBD_DCD_LSE_SUPPORTED == 1 */
            {
                // 0.11% step !!!
                CRS->CFGR = (((48000 -1) << CRS_CFGR_RELOAD_Pos) | (26 << CRS_CFGR_FELIM_Pos) | CRS_CFGR_SYNCSRC_1);
            }
            
            CRS->ICR = CRS_ICR_SYNCOKC;
            CRS->CR |= (CRS_CR_AUTOTRIMEN | CRS_CR_CEN | (!stm32wb_usbd_dcd_device.clk48_sync ? CRS_CR_SYNCOKIE : 0));
        }
    }
    
    if (events & STM32WB_USBD_DCD_EVENT_CONNECT)
    {
        USB->BCDR = USB_BCDR_DPPU;

        if (!stm32wb_usbd_dcd_device.clk48_sync)
        {
            armv7m_atomic_or(&stm32wb_usbd_dcd_device.events, events);

            // armv7m_rtt_printf("DCD_EVENT_LEAVE(CLK48_SYNC)\n");
                    
            return;
        }

        CRS->CR &= ~CRS_CR_SYNCOKIE;
        
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
        armv7m_atomic_orh(&USB->CNTR, (USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_RESETM | USB_CNTR_L1REQM));
#else /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
        armv7m_atomic_orh(&USB->CNTR, (USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_RESETM));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
    }

    if (events & STM32WB_USBD_DCD_EVENT_RESET)
    {
        stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_RESET;
    }
    
    if (events & (STM32WB_USBD_DCD_EVENT_SLEEP | STM32WB_USBD_DCD_EVENT_SUSPEND))
    {
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
        stm32wb_usbd_dcd_device.state = (events & STM32WB_USBD_DCD_EVENT_SLEEP) ? STM32WB_USBD_DCD_STATE_SLEEP: STM32WB_USBD_DCD_STATE_SUSPENDED;
#else /*STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
        stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_SLEEP;
#endif /*STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
    }

    if (events & STM32WB_USBD_DCD_EVENT_RESUME)
    {
        armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_FSUSP);

#if (STM32WB_USBD_DCD_SOF_SUPPORTED == 1)
        if (stm32wb_usbd_dcd_device.sof_callback)
        {
            armv7m_atomic_orh(&USB->CNTR, USB_CNTR_SOFM);
        }
#endif /* STM32WB_USBD_DCD_SOF_SUPPORTED == 1 */

        stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_CONNECTED;
    }
    
    if (events)
    {
        // armv7m_rtt_printf("DCD_EVENT_LEAVE(CALLBACK, state=%d, events=%08x)\n", stm32wb_usbd_dcd_device.state, events);

        (*stm32wb_usbd_dcd_device.evt_callback)(stm32wb_usbd_dcd_device.evt_context, events);
    }
    else
    {
        // armv7m_rtt_printf("DCD_EVENT_LEAVE(RETURN, state=%d)\n", stm32wb_usbd_dcd_device.state);
    }
}

#if (STM32WB_USBD_DCD_SOF_SUPPORTED == 1)

bool stm32wb_usbd_dcd_sof_enable(stm32wb_usbd_dcd_sof_callback_t callback, void *context)
{
    // armv7m_rtt_printf("DCD_SOF_ENABLE()\n");

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_RESET)
    {
	return false;
    }

    armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_SOFM);

    USB->ISTR = (uint16_t)~USB_ISTR_SOF;

    stm32wb_usbd_dcd_device.sof_callback = callback;
    stm32wb_usbd_dcd_device.sof_context = context;
    
    if (stm32wb_usbd_dcd_device.state == STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        armv7m_atomic_orh(&USB->CNTR, USB_CNTR_SOFM);
    }
    
    return true;
}

bool stm32wb_usbd_dcd_sof_disable()
{
    // armv7m_rtt_printf("DCD_SOF_DISABLE()\n");

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_RESET)
    {
	return false;
    }

    armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_SOFM);

    USB->ISTR = (uint16_t)~USB_ISTR_SOF;

    stm32wb_usbd_dcd_device.sof_callback = NULL;
    stm32wb_usbd_dcd_device.sof_context = NULL;
    
    return true;
}

#endif /* STM32WB_USBD_DCD_SOF_SUPPORTED == 1 */

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)

uint16_t stm32wb_usbd_dcd_fifo_count(stm32wb_usbd_dcd_fifo_t *fifo)
{
    return fifo->count;
}

uint16_t stm32wb_usbd_dcd_fifo_write(stm32wb_usbd_dcd_fifo_t *fifo, const uint8_t *data, uint16_t length)
{
    uint32_t count, tail;

    count = fifo->size - fifo->count;
    tail = fifo->tail;

    if (length > count)
    {
        length = count;
    }

    if (length)
    {
        count = length;
        
        if (count > (uint32_t)(fifo->size - tail))
        {
            count = fifo->size - tail;
        }

        memcpy(&fifo->data[tail], &data[0], count);

        if (count != length)
        {
            memcpy(&fifo->data[0], &data[count], (length - count));
        }
        
        tail += length;
    
        if (tail >= fifo->size)
        {
            tail -= fifo->size;
        }

        fifo->tail = tail;

        armv7m_atomic_addh(&fifo->count, length);
    }
    
    return length;
}

uint16_t stm32wb_usbd_dcd_fifo_read(stm32wb_usbd_dcd_fifo_t *fifo, uint8_t *data, uint16_t length)
{
    uint32_t count, head;

    count = fifo->count;
    head = fifo->head;

    if (length > count)
    {
        length = count;
    }
    
    if (length)
    {
        if (data)
        {
            count = length;
            
            if (count > (uint32_t)(fifo->size - head))
            {
                count = fifo->size - head;
            }
            
            memcpy(&data[0], &fifo->data[head], count);
            
            if (count != length)
            {
                memcpy(&data[count], &fifo->data[0], (length - count));
            }
        }
        
        head += length;
            
        if (head >= fifo->size)
        {
            head -= fifo->size;
        }
        
        fifo->head = head;

        armv7m_atomic_subh(&fifo->count, length);
    }
    
    return length;
}

static uint32_t stm32wb_usbd_dcd_fifo_pma_write(stm32wb_usbd_dcd_fifo_t *fifo, uint32_t pma_address, uint32_t length)
{
    uint32_t count, head;

    count = fifo->count;
    head = fifo->head;

    if (length > count)
    {
        length = count;
    }

    if (length)
    {
        count = length;
        
        if (count > (uint32_t)(fifo->size - head))
        {
            count = fifo->size - head;
        }

        stm32wb_usbd_dcd_pma_write(pma_address, &fifo->data[head], count);

        if (length != count)
        {
            stm32wb_usbd_dcd_pma_write(pma_address + count, &fifo->data[0], (length - count));
        }
        
        head += length;
    
        if (head >= fifo->size)
        {
            head -= fifo->size;
        }
        
        fifo->head = head;
        
        armv7m_atomic_subh(&fifo->count, length);
    }
    
    return length;
}

static uint32_t stm32wb_usbd_dcd_fifo_pma_read(stm32wb_usbd_dcd_fifo_t *fifo, uint32_t pma_address, uint32_t length)
{
    uint32_t count, tail;

    count = fifo->size - fifo->count;
    tail = fifo->tail;

    if (length > count)
    {
        length = count;
    }

    if (length)
    {
        count = length;

        if (count > (uint32_t)(fifo->size - tail))
        {
            count = fifo->size - tail;
        }

        stm32wb_usbd_dcd_pma_read(pma_address, &fifo->data[tail], count);

        if (length != count)
        {
            stm32wb_usbd_dcd_pma_read(pma_address + count, &fifo->data[0], (length - count));
        }
    
        tail += length;
    
        if (tail >= fifo->size)
        {
            tail -= fifo->size;
        }

        fifo->tail = tail;

        armv7m_atomic_addh(&fifo->count, length);
    }
    
    return length;
}

static void stm32wb_usbd_dcd_ep_iso_in_routine(uint32_t ep_index, stm32wb_usbd_dcd_ep_in_t *ep_in)
{
    uint32_t sequence, count;
    stm32wb_usbd_dcd_ep_iso_callback_t ep_iso_callback;
    void *ep_iso_context;

    sequence = USB_EP_DTOG_RX_STATUS(ep_index) ^ 1;
    count = ep_in->iso.length[sequence] & 0x03ff;

    ep_in->iso.length[sequence] = 0;

    if (sequence == 0)
    {
        USB_PMA_TX_0_COUNT(ep_index, 0);
    }
    else
    {
        USB_PMA_TX_1_COUNT(ep_index, 0);
    }
    
    // armv7m_rtt_printf("DCD_EP_ISO_IN_CALLBACK(ep_addr=%02x)\n", (ep_index | 0x80), count);
    
    ep_iso_callback = ep_in->iso.callback;
    ep_iso_context = ep_in->iso.context;

    if (ep_iso_callback)
    {
        (*ep_iso_callback)(ep_iso_context, (ep_index | 0x80), count);
    }
}

static void stm32wb_usbd_dcd_ep_iso_out_routine(uint32_t ep_index, stm32wb_usbd_dcd_ep_out_t *ep_out)
{
    uint32_t sequence, pma_address, length;
    stm32wb_usbd_dcd_ep_iso_callback_t ep_iso_callback;
    void *ep_iso_context;

    sequence = USB_EP_DTOG_TX_STATUS(ep_index) ^ 1;

    pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control) + (sequence ? ep_out->size : 0);
    
    if (sequence == 0)
    {
        length = USB_PMA_RX_0_COUNT(ep_index);
    }
    else
    {
        length = USB_PMA_RX_1_COUNT(ep_index);
    }

    if (length)
    {
        length = stm32wb_usbd_dcd_fifo_pma_read(ep_out->iso.fifo, pma_address, length);
    }

    // armv7m_rtt_printf("DCD_EP_ISO_OUT_CALLBACK(ep_addr=%02x, count=%d)\n", (ep_index | 0x00), length);
    
    ep_iso_callback = ep_out->iso.callback;
    ep_iso_context = ep_out->iso.context;

    if (ep_iso_callback)
    {
        (*ep_iso_callback)(ep_iso_context, (ep_index | 0x00), length);
    }
}

bool stm32wb_usbd_dcd_ep_fifo_attach(uint8_t ep_addr, stm32wb_usbd_dcd_fifo_t *fifo, uint8_t *data, uint16_t size)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_FIFO_ATTACH(ep_addr=%02x, fifo=%08x, data=%08x, size=%d)\n", ep_addr, fifo, data, size);

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_RESET)
    {
	return false;
    }
    
    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
	ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

	if ((ep_in->control & (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_STARTED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK)) != (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO))
	{
	    return false;
	}

        ep_in->iso.fifo = fifo;
        ep_in->iso.length[0] = 0;
        ep_in->iso.length[1] = 0;
    }
    else
    {
	ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

	if ((ep_out->control & (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_STARTED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK)) != (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO))
	{
	    return false;
	}

        ep_out->iso.fifo = fifo;
    }

    fifo->data = data;
    fifo->size = size;
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;

    return true;
}

bool stm32wb_usbd_dcd_ep_fifo_detach(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_FIFO_DEATTACH(ep_addr=%02x)\n", ep_addr);

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_RESET)
    {
	return false;
    }
    
    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
	ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

	if ((ep_in->control & (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_STARTED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK)) != (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO))
	{
	    return false;
	}

        ep_in->iso.fifo = NULL;
    }
    else
    {
	ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

	if ((ep_out->control & (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_STARTED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK)) != (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO))
	{
	    return false;
	}

        ep_out->iso.fifo = NULL;
    }

    return true;
}

bool stm32wb_usbd_dcd_ep_iso_start(uint8_t ep_addr, stm32wb_usbd_dcd_ep_iso_callback_t callback, void *context)
{
    uint32_t ep_index, ep_size;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_ENABLE(ep_addr=%02x)\n", ep_addr);

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }
    
    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
	ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

	if ((ep_in->control & (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_STARTED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK)) != (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO))
	{
	    return false;
	}

        if (!ep_in->iso.fifo || !ep_in->iso.length[0] || !ep_in->iso.length[1])
        {
            return false;
        }

        stm32wb_usbd_dcd_device.iso_in_routine = stm32wb_usbd_dcd_ep_iso_in_routine;
        
        ep_in->iso.callback = callback;
        ep_in->iso.context = context;
	ep_in->control |= STM32WB_USBD_DCD_EP_CONTROL_STARTED;

	USB_EP_CONFIG(ep_index, (USB_EP_ISOCHRONOUS | ep_index));

        USB_PMA_TX_0_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control));
        USB_PMA_TX_1_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control) + ep_in->size);

        USB_EP_DTOG_TX_RESET(ep_index);
        USB_EP_STAT_RX(ep_index, USB_EP_RX_DIS);
        USB_EP_STAT_TX(ep_index, USB_EP_TX_VALID);
    }
    else
    {
	ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

	if ((ep_out->control & (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_STARTED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK)) != (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO))
	{
	    return false;
	}

        if (!ep_out->iso.fifo)
        {
            return false;
        }

        stm32wb_usbd_dcd_device.iso_out_routine = stm32wb_usbd_dcd_ep_iso_out_routine;

        ep_out->iso.callback = callback;
        ep_out->iso.context = context;

	ep_out->control |= STM32WB_USBD_DCD_EP_CONTROL_STARTED;

	if (ep_out->size >= 64)
	{
	    ep_size = 0x8000 | (((ep_out->size - 32) / 32) << 10);
	}
	else
	{
	    ep_size = 0x0000 | ((ep_out->size / 2) << 10);
	}

	USB_EP_CONFIG(ep_index, (USB_EP_ISOCHRONOUS | ep_index));

        USB_PMA_RX_0_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control));
        USB_PMA_RX_0_SIZE(ep_index, ep_size);
        USB_PMA_RX_1_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control) + ep_out->size);
        USB_PMA_RX_1_SIZE(ep_index, ep_size);
	    
        USB_EP_DTOG_RX_RESET(ep_index);
        USB_EP_STAT_TX(ep_index, USB_EP_TX_DIS);
        USB_EP_STAT_RX(ep_index, USB_EP_RX_VALID);
    }

    return true;
}

bool stm32wb_usbd_dcd_ep_iso_stop(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_DISABLE(ep_addr=%02x)\n", ep_addr);
    
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }

    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
	ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

	if (!(ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_STARTED))
	{
	    return false;
	}
	
	USB_EP_STAT_TX(ep_index, USB_EP_TX_DIS);
        USB_EP_CTR_TX_RESET(ep_index);
	
        ep_in->iso.callback = NULL;
        ep_in->iso.fifo->head = 0;
        ep_in->iso.fifo->tail = 0;
        ep_in->iso.fifo->count = 0;
        ep_in->iso.length[0] = 0;
        ep_in->iso.length[1] = 0;

	ep_in->control &= ~STM32WB_USBD_DCD_EP_CONTROL_STARTED;
    }
    else
    {
	ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

	if (!(ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_STARTED))
	{
	    return false;
	}

	USB_EP_STAT_RX(ep_index, USB_EP_RX_DIS);
        USB_EP_CTR_RX_RESET(ep_index);
	
        ep_out->iso.callback = NULL;

	ep_out->control &= ~STM32WB_USBD_DCD_EP_CONTROL_STARTED;
    }

    return true;
}

bool stm32wb_usbd_dcd_ep_iso_transmit(uint8_t ep_addr, uint16_t length)
{
    uint32_t ep_index, sequence, pma_address;
    stm32wb_usbd_dcd_ep_in_t *ep_in;

    // armv7m_rtt_printf("DCD_EP_ISP_TRANSMIT(ep_addr=%02x, length=%d)\n", ep_addr, length);
    
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }

    ep_index = ep_addr & 0x07;

    ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

    if ((ep_in->control & (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK)) != (STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO))
    {
        return false;
    }

    if (!(ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_STARTED))
    {
        if      (!ep_in->iso.length[0]) { sequence = 0; }
        else if (!ep_in->iso.length[1]) { sequence = 1; }
        else
        {
            return false;
        }
    }
    else
    {
        sequence = USB_EP_DTOG_TX_STATUS(ep_index) ^ 1;
    }
    
    if (length)
    {
        pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control) + (sequence ? ep_in->size : 0);
        
        length = stm32wb_usbd_dcd_fifo_pma_write(ep_in->iso.fifo, pma_address, length);
    }
    
    if (sequence == 0)
    {
        USB_PMA_TX_0_COUNT(ep_index, length);
    }
    else
    {
        USB_PMA_TX_1_COUNT(ep_index, length);
    }

    ep_in->iso.length[sequence] = length | 0x8000;
    
    return true;
}

#endif /* STM32WB_USBD_DCD_ISO_SUPPORTED == 1 */

static void __attribute__((optimize("O3"))) stm32wb_usbd_dcd_interrupt()
{
    uint32_t usb_istr, usb_ep, ep_index, pma_address, count, events;
#if (STM32WB_USBD_DCD_DBL_SUPPORTED == 1)
    uint32_t sequence;
#endif /* STM32WB_USBD_DCD_DBL_SUPPORTED == 1 */
    uint8_t lpm_state;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;
    stm32wb_usbd_dcd_ep_in_callback_t ep_in_callback;
    stm32wb_usbd_dcd_ep_out_callback_t ep_out_callback;
    void *ep_in_context, *ep_out_context;

    events = 0;

    usb_istr = USB->ISTR;

    // // armv7m_rtt_printf("DCD_ISR_ENTER(cntr=%04x, istr=%04x, fnr=%08x, %d)\n", USB->CNTR, usb_istr, USB->FNR, (uint32_t)armv7m_systick_micros());
    
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
		        stm32wb_usbd_dcd_pma_read(USB_EP0_PMA_ADDRESS, stm32wb_usbd_dcd_device.ep0_setup, count);

			USB_EP_CTR_RX_RESET(0);
			    
			events |= STM32WB_USBD_DCD_EVENT_EP0_SETUP;
		    }
		    else
		    {
			if (count > stm32wb_usbd_dcd_device.ep0_length)
			{
			    count = stm32wb_usbd_dcd_device.ep0_length;
			}

			if (count)
			{
			    stm32wb_usbd_dcd_pma_read(USB_EP0_PMA_ADDRESS, stm32wb_usbd_dcd_device.ep0_data, count);
			}

			stm32wb_usbd_dcd_device.ep0_count = count;

			USB_EP_CTR_RX_RESET(0);
                        
			events |= STM32WB_USBD_DCD_EVENT_EP0_DATA_OUT;
		    }
		}
		
		if (usb_ep & USB_EP_CTR_TX)
		{
		    USB_EP_CTR_TX_RESET(0);
                    
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

                        (*stm32wb_usbd_dcd_device.iso_out_routine)(ep_index, ep_out);
		    }
		    else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
                    {
			/* BULK / INTERRUPT */

                        count = USB_PMA_RX_COUNT(ep_index);

#if (STM32WB_USBD_DCD_DBL_SUPPORTED == 1)
                        if (ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_DBL)
                        {
                            sequence = (usb_ep & USB_EP_DTOG_RX) ? 1 : 0;

                            if ((count == ep_out->size) && (ep_out->bulk.length > ep_out->size))
                            {
                                pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control) + (sequence ? ep_out->size : 0);
                            
                                USB_PMA_RX_ADDRESS(ep_index, pma_address);
                                    
                                USB_EP_STAT_RX(ep_index, USB_EP_RX_VALID);

                                pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control) + (sequence ? 0 : ep_out->size);
                                
                                stm32wb_usbd_dcd_pma_read(pma_address, ep_out->bulk.data, count);

                                ep_out->bulk.data += count;
                                ep_out->bulk.count += count;
                                ep_out->bulk.length -= count;
                            }
                            else
                            {
                                if (count > ep_out->bulk.length)
                                {
                                    count = ep_out->bulk.length;
                                }

                                if (count)
                                {
                                    pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control) + (sequence ? 0 : ep_out->size);
                                    
                                    stm32wb_usbd_dcd_pma_read(pma_address, ep_out->bulk.data, count);

                                    ep_out->bulk.data += count;
                                    ep_out->bulk.count += count;
                                }

                                count = ep_out->bulk.count;
                                
                                ep_out_callback = ep_out->bulk.callback;
                                ep_out_context = ep_out->bulk.context;
                                
                                ep_out->bulk.callback = NULL;
                                
                                // armv7m_rtt_printf("DCD_EP_OUT_CALLBACK(ep_addr=%02x, count=%d)\n", (ep_index | 0x00), ep_out->bulk.count);
                                
                                if (ep_out_callback)
                                {
                                    (*ep_out_callback)(ep_out_context, (ep_index | 0x00), count);
                                }
                            }
                        }
                        else
#endif /* STM32WB_USBD_DCD_DBL_SUPPORTED == 1 */
                        {
                            if (count > ep_out->bulk.length)
                            {
                                count = ep_out->bulk.length;
                            }
                            
                            if (count)
                            {
                                pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control);

                                stm32wb_usbd_dcd_pma_read(pma_address, ep_out->bulk.data, count);
                                
                                ep_out->bulk.data += count;
                                ep_out->bulk.count += count;
                                ep_out->bulk.length -= count;
                            }
                            
                            if ((count == ep_out->size) && ep_out->bulk.length)
                            {
                                USB_EP_STAT_RX(ep_index, USB_EP_RX_VALID);
                            }
                            else
                            {
                                ep_out_callback = ep_out->bulk.callback;
                                ep_out_context = ep_out->bulk.context;
                                
                                ep_out->bulk.callback = NULL;
                                
                                // armv7m_rtt_printf("DCD_EP_OUT_CALLBACK(ep_addr=%02x, count=%d)\n", (ep_index | 0x00), ep_out->bulk.count);
                                
                                if (ep_out_callback)
                                {
                                    (*ep_out_callback)(ep_out_context, (ep_index | 0x00), count);
                                }
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

                        (*stm32wb_usbd_dcd_device.iso_in_routine)(ep_index, ep_in);
		    }
		    else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
		    {
                        if (ep_in->bulk.length)
                        {
#if (STM32WB_USBD_DCD_DBL_SUPPORTED == 1)
                            if (ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_DBL)
                            {
                                if (ep_in->bulk.count)
                                {
                                    count = ep_in->bulk.count;

                                    sequence = (usb_ep & USB_EP_DTOG_TX) ? 1 : 0;
                                    
                                    pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control) + (sequence ? ep_in->size : 0);
                                
                                    USB_PMA_TX_ADDRESS(ep_index, pma_address);
                                    USB_PMA_TX_COUNT(ep_index, count);
                                    
                                    USB_EP_STAT_TX(ep_index, USB_EP_TX_VALID);

                                    ep_in->bulk.data += count;
                                    ep_in->bulk.length -= count;
                                    
                                    if (ep_in->bulk.length)
                                    {
                                        count = ep_in->bulk.length;
                                        
                                        if (count > ep_in->size)
                                        {
                                            count = ep_in->size;
                                        }
                                        
                                        pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control) + (sequence ? 0 : ep_in->size);

                                        stm32wb_usbd_dcd_pma_write(pma_address, ep_in->bulk.data, count);

                                        ep_in->bulk.count = count;
                                    }
                                    else
                                    {
                                        ep_in->bulk.count = 0;
                                    }
                                }
                            }
                            else
#endif /* STM32WB_USBD_DCD_DBL_SUPPORTED == 1 */
                            {
                                count = ep_in->bulk.length;
				
                                if (count > ep_in->size)
                                {
                                    count = ep_in->size;
                                }
                                
                                pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control);

                                stm32wb_usbd_dcd_pma_write(pma_address, ep_in->bulk.data, count);
                                
                                USB_PMA_TX_COUNT(ep_index, count);
                                
                                USB_EP_STAT_TX(ep_index, USB_EP_TX_VALID);
                                
                                ep_in->bulk.data += count;
                                ep_in->bulk.length -= count;
                            }
                        }
                        else
                        {
                            ep_in_callback = ep_in->bulk.callback;
                            ep_in_context = ep_in->bulk.context;
                                
                            ep_in->bulk.callback = NULL;
                            
                            // armv7m_rtt_printf("DCD_EP_IN_CALLBACK(ep_addr=%02x)\n", (ep_index | 0x80));
                            
                            if (ep_in_callback)
                            {
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

#if (STM32WB_USBD_DCD_SOF_SUPPORTED == 1)
    if (usb_istr & USB_ISTR_SOF)
    {
        if (USB->CNTR & USB_CNTR_SOFM)
        {
            if (stm32wb_usbd_dcd_device.sof_callback)
            {
                (*stm32wb_usbd_dcd_device.sof_callback)(stm32wb_usbd_dcd_device.sof_context, (USB->FNR & USB_FNR_FN));
            }
        }
        
        USB->ISTR = (uint16_t)~USB_ISTR_SOF;
    }
#endif /* STM32WB_USBD_DCD_SOF_SUPPORTED == 1 */

    if (usb_istr & (USB_ISTR_WKUP | USB_ISTR_SUSP | USB_ISTR_RESET | USB_ISTR_L1REQ))
    {
	if (usb_istr & USB_ISTR_WKUP)
	{
            if (stm32wb_usbd_dcd_device.lpm_state != STM32WB_USBD_DCD_LPM_STATE_L3)
            {
                lpm_state = armv7m_atomic_swapb(&stm32wb_usbd_dcd_device.lpm_state, STM32WB_USBD_DCD_LPM_STATE_L0);
                
                if (lpm_state != STM32WB_USBD_DCD_LPM_STATE_L0)
                {
                    armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_LPMODE);

                    // armv7m_rtt_printf("DCD_LOCK\n");

                    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
                    
                    events |= STM32WB_USBD_DCD_EVENT_RESUME;
                }
            }
            
            USB->ISTR = (uint16_t)~USB_ISTR_WKUP;
	}
	
	if (usb_istr & (USB_ISTR_SUSP | USB_ISTR_L1REQ))
	{
            if (stm32wb_usbd_dcd_device.lpm_state != STM32WB_USBD_DCD_LPM_STATE_L3)
            {
                armv7m_atomic_orh(&USB->CNTR, USB_CNTR_FSUSP);

#if (STM32WB_USBD_DCD_SOF_SUPPORTED == 1)
                armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_SOFM);
#endif /* STM32WB_USBD_DCD_SOF_SUPPORTED == 1 */
            
                armv7m_atomic_orh(&USB->CNTR, USB_CNTR_LPMODE);

                stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);
    
                // armv7m_rtt_printf("DCD_UNLOCK\n");
                
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
                if (usb_istr & USB_ISTR_L1REQ)
                {
                    // armv7m_rtt_printf("DCD_LPM_BSEL=%d\n", ((USB->LPMCSR & USB_LPMCSR_BESL) >> 4));
                    stm32wb_usbd_dcd_device.lpm_state = STM32WB_USBD_DCD_LPM_STATE_L1;
              
                    events |= STM32WB_USBD_DCD_EVENT_SLEEP;                
                }
                else
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
                {
                    stm32wb_usbd_dcd_device.lpm_state = STM32WB_USBD_DCD_LPM_STATE_L2;
              
                    events |= STM32WB_USBD_DCD_EVENT_SUSPEND;                
                }
            }
            
            USB->ISTR = (uint16_t)~(USB_ISTR_SUSP | USB_ISTR_L1REQ);
	}
        
	if (usb_istr & USB_ISTR_RESET)
	{
            USB->CNTR = USB_CNTR_FRES;
	    USB->ISTR = (uint16_t)~USB_ISTR_RESET;
            USB->DADDR = 0;

            lpm_state = armv7m_atomic_swapb(&stm32wb_usbd_dcd_device.lpm_state, STM32WB_USBD_DCD_LPM_STATE_L0);
            
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
            if ((lpm_state == STM32WB_USBD_DCD_LPM_STATE_L1) || (lpm_state == STM32WB_USBD_DCD_LPM_STATE_L2))
#else /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
            if (lpm_state == STM32WB_USBD_DCD_LPM_STATE_L2)
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
            {
                // armv7m_rtt_printf("DCD_LOCK\n");

                stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
            }

#if (STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1)
            stm32wb_rtc_wakeup_stop();
#endif /* STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1 */

	    events |= STM32WB_USBD_DCD_EVENT_RESET;
	}
    }
    
    if (events)
    {
        armv7m_atomic_or(&stm32wb_usbd_dcd_device.events, events);
        
        armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_DCD);
    }
    
    // // armv7m_rtt_printf("DCD_ISR_LEAVE(cntr=%04x, istr=%04x, lpm_state=%d, events=%08x)\n", USB->CNTR, USB->ISTR, stm32wb_usbd_dcd_device.lpm_state, events);
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

void CRS_IRQHandler(void)
{
    // // armv7m_rtt_printf("CRS_CLK48_SYNC(%d)\n", (uint32_t)armv7m_systick_micros());

    CRS->ICR = CRS_ICR_SYNCOKC;

    stm32wb_usbd_dcd_device.clk48_sync = true;
    
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_DCD);
    
    __DSB();
}

void USBD_DCD_HSEMHandler(void)
{
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_DCD);
}

void USBD_DCD_SWIHandler(void)
{
    stm32wb_usbd_dcd_event();
}
