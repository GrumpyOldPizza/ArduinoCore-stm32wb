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
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_BULK             0x0000
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_INTERRUPT        0x0001
#define STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO              0x0002
#define STM32WB_USBD_DCD_EP_CONTROL_STALLED               0x0004
#define STM32WB_USBD_DCD_EP_CONTROL_ENABLED               0x0008
#define STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED            0x0010
#define STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS_MASK      0xffe0
#define STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS_SHIFT     5

#define STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(_control) ((_control) >> STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS_SHIFT)

#if (STM32WB_USBD_DCD_EP_SUPPORTED == 1)

typedef struct _stm32wb_usbd_dcd_ep_in_t {
    uint16_t                                   control;
    uint16_t                                   size;
    union {
	struct {
	    stm32wb_usbd_dcd_ep_in_callback_t      callback;
	    void                                   *context;
            const uint8_t                          *data;
            uint32_t                               count;
	} bulk;
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
	struct {
	    stm32wb_usbd_dcd_ep_iso_in_callback_t  callback;
	    void                                   *context;
	} iso;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
    };
} stm32wb_usbd_dcd_ep_in_t;

typedef struct __stm32wb_usbd_dcd_ep_out_t {
    uint16_t                                   control;
    uint16_t                                   size;
    union {
	struct {
	    stm32wb_usbd_dcd_ep_out_callback_t     callback;
	    void                                   *context;
	    uint8_t                                *data;
	    uint16_t                               size;
	    uint16_t                               count;
	} bulk;
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
	struct {
	    stm32wb_usbd_dcd_ep_iso_out_callback_t callback;
	    void                                   *context;
	    uint8_t                                *data;
	    uint32_t                               size;
	} iso;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
    };
} stm32wb_usbd_dcd_ep_out_t;

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
typedef void (*stm32wb_usbd_dcd_iso_in_routine_t)(uint32_t ep_index, stm32wb_usbd_dcd_ep_in_t *ep_in, uint32_t sequence);
typedef void (*stm32wb_usbd_dcd_iso_out_routine_t)(uint32_t ep_index, stm32wb_usbd_dcd_ep_out_t *ep_out, uint32_t sequence);
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */

#endif /* (STM32WB_USBD_DCD_EP_SUPPORTED == 1) */

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
    uint8_t                            ep0_size;
    uint8_t                            ep0_count;
#if (STM32WB_USBD_DCD_EP_SUPPORTED == 1)
    stm32wb_usbd_dcd_ep_in_t           ep_in[STM32WB_USBD_DCD_EP_COUNT-1];  /* TX */
    stm32wb_usbd_dcd_ep_out_t          ep_out[STM32WB_USBD_DCD_EP_COUNT-1]; /* RX */
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
    stm32wb_usbd_dcd_iso_in_routine_t  iso_in_routine;
    stm32wb_usbd_dcd_iso_out_routine_t iso_out_routine;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
#endif /* (STM32WB_USBD_DCD_EP_SUPPORTED == 1) */
#if (STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1)
    volatile uint8_t                   set_resume;
    volatile uint8_t                   unset_resume;
    stm32wb_lptim_timeout_t            timeout;
#endif /* STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1 */
} stm32wb_usbd_dcd_device_t;

static stm32wb_usbd_dcd_device_t stm32wb_usbd_dcd_device;

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

#if (STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1)
static void stm32wb_usbd_dcd_wakeup_routine(void *context);
#endif /* STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1 */

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

#if (STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1)
    stm32wb_usbd_dcd_device.timeout = STM32WB_LPTIM_TIMEOUT_INIT();
#endif /* STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1 */
    
    return true;
}

bool stm32wb_usbd_dcd_enable(uint8_t *setup, stm32wb_usbd_dcd_event_callback_t callback, void *context)
{
    // armv7m_rtt_printf("DCD_ENABLE\n");

    if (stm32wb_usbd_dcd_device.state != STM32WB_USBD_DCD_STATE_INIT)
    {
	return false;
    }

#if !defined(__STM32WB_BOOT_CODE__)
    if (stm32wb_system_pclk1() < 10000000)
    {
	return false;
    }
#endif /* !defined(__STM32WB_BOOT_CODE__) */
    
    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_NOT_READY;
    stm32wb_usbd_dcd_device.events = 0;

    stm32wb_usbd_dcd_device.evt_callback = callback;
    stm32wb_usbd_dcd_device.evt_context = context;
    stm32wb_usbd_dcd_device.ep0_setup = setup;

#if !defined(__STM32WB_BOOT_CODE__)
    stm32wb_system_reference(STM32WB_SYSTEM_REFERENCE_USB);
#endif /* !defined(__STM32WB_BOOT_CODE__) */

    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA11_USB_DM, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA12_USB_DP, (STM32WB_GPIO_PARK_NONE | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));

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
    
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA11_USB_DM, STM32WB_GPIO_MODE_ANALOG);
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PA12_USB_DP, STM32WB_GPIO_MODE_ANALOG);

#if !defined(__STM32WB_BOOT_CODE__)
    stm32wb_system_unreference(STM32WB_SYSTEM_REFERENCE_USB);
#endif /* !defined(__STM32WB_BOOT_CODE__) */

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
    
#if !defined(__STM32WB_BOOT_CODE__)
    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_STOP_1);

    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
#endif /* !defined(__STM32WB_BOOT_CODE__) */

    armv7m_atomic_or(&PWR->CR2, PWR_CR2_USV);

    armv7m_atomic_or(&RCC->APB1ENR1, RCC_APB1ENR1_USBEN);
    RCC->APB1ENR1;
    
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
        
#if !defined(__STM32WB_BOOT_CODE__)
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_DCD);
#else /* !defined(__STM32WB_BOOT_CODE__) */
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
#endif /* !defined(__STM32WB_BOOT_CODE__) */
    
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
    stm32wb_lptim_timeout_stop(&stm32wb_usbd_dcd_device.timeout);
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

    armv7m_atomic_and(&RCC->APB1ENR1, ~RCC_APB1ENR1_USBEN);
    
    armv7m_atomic_and(&PWR->CR2, ~PWR_CR2_USV);
    
    /* Signal a DISCONNECT on the DP line by driving it low for 50uS. Specs says 2.5uS, but leave some extra margin.
     */

    stm32wb_gpio_pin_write(STM32WB_GPIO_PIN_PA12_USB_DP, 0);
    stm32wb_gpio_pin_output(STM32WB_GPIO_PIN_PA12_USB_DP);

    armv7m_core_udelay(50);

    stm32wb_gpio_pin_alternate(STM32WB_GPIO_PIN_PA12_USB_DP);

    stm32wb_usbd_dcd_device.lpm_state = STM32WB_USBD_DCD_LPM_STATE_L3;

    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_READY;

#if !defined(__STM32WB_BOOT_CODE__)
    if (stm32wb_usbd_dcd_device.state <= STM32WB_USBD_DCD_STATE_CONNECTED)
    {
        stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);

        // armv7m_rtt_printf("DCD_UNLOCK\n");
    }

    stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_STOP_1);
#endif /* !defined(__STM32WB_BOOT_CODE__) */
    
    stm32wb_usbd_dcd_device.events = STM32WB_USBD_DCD_EVENT_DISCONNECT;
    
#if !defined(__STM32WB_BOOT_CODE__)
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_DCD);
#else /* !defined(__STM32WB_BOOT_CODE__) */
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
#endif /* !defined(__STM32WB_BOOT_CODE__) */

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
    
    stm32wb_usbd_dcd_device.ep0_size = 0;
    stm32wb_usbd_dcd_device.ep0_count = 0;

    USB_EP_CTR_RX_RESET(0);
    USB_EP_CTR_TX_RESET(0);
    
    for (ep_index = 1; ep_index < STM32WB_USBD_DCD_EP_COUNT; ep_index++)
    {
#if (STM32WB_USBD_DCD_EP_SUPPORTED == 1)
        stm32wb_usbd_dcd_device.ep_in[ep_index -1].control &= ~(STM32WB_USBD_DCD_EP_CONTROL_ENABLED | STM32WB_USBD_DCD_EP_CONTROL_STALLED);
        stm32wb_usbd_dcd_device.ep_in[ep_index -1].bulk.callback = NULL;
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        stm32wb_usbd_dcd_device.ep_in[ep_index -1].iso.callback = NULL;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
        stm32wb_usbd_dcd_device.ep_out[ep_index -1].control &= ~(STM32WB_USBD_DCD_EP_CONTROL_ENABLED | STM32WB_USBD_DCD_EP_CONTROL_STALLED);
        stm32wb_usbd_dcd_device.ep_out[ep_index -1].bulk.callback = NULL;
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
        stm32wb_usbd_dcd_device.ep_out[ep_index -1].iso.callback = NULL;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
#endif /* (STM32WB_USBD_DCD_EP_SUPPORTED == 1) */

        USB_EP_CTR_RX_RESET(ep_index);
        USB_EP_CTR_TX_RESET(ep_index);
    }

    // armv7m_rtt_printf("DCD_RESET-1 cntr=%04x, istr=%04x\n", USB->CNTR, USB->ISTR);
    
    armv7m_atomic_andh(&USB->CNTR, ~USB_CNTR_FRES);

    // armv7m_rtt_printf("DCD_RESET-2 cntr=%04x, istr=%04x\n", USB->CNTR, USB->ISTR);
    
    USB->ISTR = (uint16_t)0;

    // armv7m_rtt_printf("DCD_RESET-3 cntr=%04x, istr=%04x\n", USB->CNTR, USB->ISTR);
    
    USB_PMA_TX_ADDRESS(0, USB_EP0_PMA_ADDRESS);
    USB_PMA_RX_ADDRESS(0, USB_EP0_PMA_ADDRESS);
    USB_PMA_RX_SIZE(0, 0x8400);                     // 8 = 0x1000, 16 = 0x2000, 32 = 0x4000, 64 = 0x8400
    
    USB_EP_W(0, USB_EP_CONTROL);
    USB_EP_STAT_TX(0, USB_EP_TX_NAK);
    USB_EP_STAT_RX(0, USB_EP_RX_NAK);

    // armv7m_rtt_printf("DCD_RESET-4 cntr=%04x, istr=%04x\n", USB->CNTR, USB->ISTR);
    
    USB->DADDR = USB_DADDR_EF | 0x00;
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
    USB->LPMCSR = USB_LPMCSR_LPMEN | USB_LPMCSR_LPMACK;
#else /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
    USB->LPMCSR = 0;
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
    
    stm32wb_usbd_dcd_device.state = STM32WB_USBD_DCD_STATE_CONNECTED;

    // armv7m_rtt_printf("DCD_RESET-5 cntr=%04x, istr=%04x\n", USB->CNTR, USB->ISTR);
    
#if (STM32WB_USBD_DCD_LPM_SUPPORTED == 1)
    stm32wb_usbd_dcd_device.reference = 0;

    armv7m_atomic_orh(&USB->CNTR, (USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_RESETM | USB_CNTR_L1REQM));
#else /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */
    armv7m_atomic_orh(&USB->CNTR, (USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_RESETM));
#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */

    // armv7m_rtt_printf("DCD_RESET-6 cntr=%04x, istr=%04x\n", USB->CNTR, USB->ISTR);
    
#if (STM32WB_USBD_DCD_SOF_SUPPORTED == 1)
    if (stm32wb_usbd_dcd_device.sof_callback)
    {
        armv7m_atomic_orh(&USB->CNTR, USB_CNTR_SOFM);
    }
#endif /* STM32WB_USBD_DCD_SOF_SUPPORTED == 1 */

    // armv7m_rtt_printf("DCD_RESET-LEAVE cntr=%04x, istr=%04x\n", USB->CNTR, USB->ISTR);
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

static void stm32wb_usbd_dcd_wakeup_routine(void *context)
{
    uint16_t usb_cntr;
    
    if (stm32wb_usbd_dcd_device.lpm_state == STM32WB_USBD_DCD_LPM_STATE_L0)
    {
        if (stm32wb_usbd_dcd_device.set_resume)
        {
            // armv7m_rtt_printf("DCD_WAKEUP_SET_RESUME(cntr=%04x, istr=%04x)\n", USB->CNTR, USB->ISTR);

            stm32wb_usbd_dcd_device.set_resume = 0;
            
            do
            {
                usb_cntr = USB->CNTR;
                
                if (!(usb_cntr & USB_CNTR_FSUSP))
                {
                    return;
                }
            }
            while (armv7m_atomic_cash(&USB->CNTR, usb_cntr, (usb_cntr | USB_CNTR_RESUME)) != usb_cntr);

            stm32wb_usbd_dcd_device.unset_resume = 1;
            
            stm32wb_lptim_timeout_start(&stm32wb_usbd_dcd_device.timeout, STM32WB_LPTIM_TIMEOUT_MILLIS_TO_TICKS(5), stm32wb_usbd_dcd_wakeup_routine, NULL);
        }
        
        if (stm32wb_usbd_dcd_device.unset_resume)
        {
            // armv7m_rtt_printf("DCD_WAKEUP_UNSET_RESUME(cntr=%04x, istr=%04x)\n", USB->CNTR, USB->ISTR);

            stm32wb_usbd_dcd_device.unset_resume = 0;

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
}

bool stm32wb_usbd_dcd_wakeup(void)
{
    uint8_t lpm_state;

    // armv7m_rtt_printf("DCD_WAKEUP(cntr=%04x, istr=%04x)\n", USB->CNTR, USB->ISTR);

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

            stm32wb_usbd_dcd_device.set_resume = 1;
            stm32wb_usbd_dcd_device.unset_resume = 0;
            
            stm32wb_lptim_timeout_start(&stm32wb_usbd_dcd_device.timeout, STM32WB_LPTIM_TIMEOUT_MILLIS_TO_TICKS(2), stm32wb_usbd_dcd_wakeup_routine, NULL);
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
        armv7m_atomic_orhz(&USB->LPMCSR, USB_LPMCSR_LPMACK, &stm32wb_usbd_dcd_device.reference);
    }

    // armv7m_rtt_printf("DCD_LPM_UNREFERENCE(reference=%08x) = %s\n", reference, ((USB->LPMCSR & USB_LPMCSR_LPMACK) ? "ACK" : "NYET"));
}

#endif /* STM32WB_USBD_DCD_LPM_SUPPORTED == 1 */

bool stm32wb_usbd_dcd_ep0_transmit(const uint8_t *data, uint32_t count)
{
    // armv7m_rtt_printf("DCD_EP0_TRANSMIT(data=%08x, count=%d)\n", data, count);
    
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }
    
    if (count > STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE)
    {
	return false;
    }

    if (count)
    {
	stm32wb_usbd_dcd_pma_write(USB_EP0_PMA_ADDRESS, data, count);
	
	USB_PMA_TX_COUNT(0, count);
    }
    else
    {
	USB_PMA_TX_COUNT(0, 0);
    }
	
    USB_EP_STAT_TX(0, USB_EP_TX_VALID);

    return true;
}

bool stm32wb_usbd_dcd_ep0_receive(uint8_t *data, uint32_t size)
{
    // armv7m_rtt_printf("DCD_EP0_RECEIVE(data=%08x, size=%d)\n", data, size);

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }

    if (size > STM32WB_USBD_DCD_EP0_MAX_PACKET_SIZE)
    {
	return false;
    }
    
    stm32wb_usbd_dcd_device.ep0_data = data;
    stm32wb_usbd_dcd_device.ep0_size = size;
    stm32wb_usbd_dcd_device.ep0_count = 0;

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

#if (STM32WB_USBD_DCD_EP_SUPPORTED == 1)

static uint32_t stm32wb_usbd_dcd_pma_allocate(uint32_t ep_index, uint32_t pma_size)
{
    uint32_t pma_btable;
    
    pma_btable = (ep_index + 1) * 8;

    if (pma_btable < stm32wb_usbd_dcd_device.pma_btable)
    {
        pma_btable = stm32wb_usbd_dcd_device.pma_btable;
    }
    
    if (pma_size > (uint32_t)(stm32wb_usbd_dcd_device.pma_address - pma_btable))
    {
        return 0;
    }
    stm32wb_usbd_dcd_device.pma_btable = pma_btable;
    
    stm32wb_usbd_dcd_device.pma_address -= pma_size;
    
    return stm32wb_usbd_dcd_device.pma_address;
}

bool stm32wb_usbd_dcd_ep_configure(uint8_t ep_addr, uint8_t type, uint16_t size)
{
    uint32_t ep_index, ep_type, pma_size;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;
    uint32_t pma_address;
    
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
    
    ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];
    ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

    if (ep_addr & 0x80)
    {
        if (ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED)
	{
	    return false;
	}
        
	if (type == STM32WB_USBD_DCD_EP_TYPE_ISO)
	{
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
            ep_type = STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO;

            if (ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED)
	    {
		return false;
	    }

            size = (size + 31) & ~31;
	    
	    pma_size = size * 2;
#else /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
            return false;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
	}
	else
	{
            ep_type = (type == STM32WB_USBD_DCD_EP_TYPE_INTERRUPT) ? STM32WB_USBD_DCD_EP_CONTROL_TYPE_INTERRUPT : STM32WB_USBD_DCD_EP_CONTROL_TYPE_BULK;

            if ((ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED) && ((ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) != ep_type))
            {
                return false;
            }

            size = (size + 1) & ~1;
            
            pma_size = size;
	}

        pma_address = stm32wb_usbd_dcd_pma_allocate(ep_index, pma_size);

        if (!pma_address)
        {
            return false;
        }
        
	ep_in->control = (pma_address << STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS_SHIFT) | STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | ep_type;
	ep_in->size = size;
    }
    else
    {
        if (ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED)
	{
	    return false;
	}

	if (type == STM32WB_USBD_DCD_EP_TYPE_ISO)
	{
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
            ep_type = STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO;

            if (ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED)
	    {
		return false;
	    }

            size = (size + 31) & ~31;
	    
	    pma_size = size * 2;
#else /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
            return false;
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
	}
	else
	{
            ep_type = (type == STM32WB_USBD_DCD_EP_TYPE_INTERRUPT) ? STM32WB_USBD_DCD_EP_CONTROL_TYPE_INTERRUPT : STM32WB_USBD_DCD_EP_CONTROL_TYPE_BULK;

            if ((ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED) && ((ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) != ep_type))
            {
                return false;
            }

            size = (size + 1) & ~1;

            pma_size = size;
	}

        pma_address = stm32wb_usbd_dcd_pma_allocate(ep_index, pma_size);

        if (!pma_address)
        {
            return false;
        }
        
	ep_out->control = (pma_address << STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS_SHIFT) | STM32WB_USBD_DCD_EP_CONTROL_CONFIGURED | ep_type;
	ep_out->size = size;
    }
    
    return true;
}

bool stm32wb_usbd_dcd_ep_enable(uint8_t ep_addr)
{
    uint32_t ep_index, ep_type, pma_size;
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

        USB_EP_CONFIG(ep_index, (((ep_type == STM32WB_USBD_DCD_EP_CONTROL_TYPE_INTERRUPT) ? USB_EP_INTERRUPT : USB_EP_BULK) | ep_index));

        USB_PMA_TX_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control));
        
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

        pma_size = (ep_out->size == 64) ? 0x8400 : (ep_out->size << (10-1));

        USB_EP_CONFIG(ep_index, (((ep_type == STM32WB_USBD_DCD_EP_CONTROL_TYPE_INTERRUPT) ? USB_EP_INTERRUPT : USB_EP_BULK) | ep_index));

        USB_PMA_RX_ADDRESS(ep_index, STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control));
        USB_PMA_RX_SIZE(ep_index, pma_size);
	
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

bool stm32wb_usbd_dcd_ep_transmit(uint8_t ep_addr, const uint8_t *data, uint32_t count, stm32wb_usbd_dcd_ep_in_callback_t callback, void *context)
{
    uint32_t ep_index, pma_address;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    
    // armv7m_rtt_printf("DCD_EP_TRANSMIT(ep_addr=%02x, data=%08x, count=%d, callback=%08x, context=%0x)\n", ep_addr, data, count, callback, context);

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
    
    if (ep_in->bulk.callback || !callback)
    {
        return false;
    }

    ep_in->bulk.data = data;
    ep_in->bulk.count = count;
    ep_in->bulk.callback = callback;
    ep_in->bulk.context = context;

    if (count)
    {
        pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control);
        
        if (count > ep_in->size)
        {
            count = ep_in->size;
        }
        
        stm32wb_usbd_dcd_pma_write(pma_address, ep_in->bulk.data, count);

        ep_in->bulk.data += count;
        ep_in->bulk.count -= count;
    }

    USB_PMA_TX_COUNT(ep_index, count);

    USB_EP_STAT_TX(ep_index, USB_EP_TX_VALID);
    
    return true;
}

bool stm32wb_usbd_dcd_ep_receive(uint8_t ep_addr, uint8_t *data, uint32_t size, stm32wb_usbd_dcd_ep_out_callback_t callback, void *context)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_RECEIVE(ep_addr=%02x, data=%08x, size=%d, callback=%08x, context=%0x)\n", ep_addr, data, size, callback, context);

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

    if (ep_out->bulk.callback || !callback)
    {
        return false;
    }
    
    ep_out->bulk.data = data;
    ep_out->bulk.size = size;
    ep_out->bulk.count = 0;
    ep_out->bulk.callback = callback;
    ep_out->bulk.context = context;

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

#endif /* STM32WB_USBD_DCD_EP_SUPPORTED == 1 */

static void __attribute((used)) stm32wb_usbd_dcd_event(void)
{
    uint32_t events;
#if (STM32WB_USBD_DCD_LSE_SUPPORTED == 1)
    uint32_t lseclk;
#endif /* STM32WB_USBD_DCD_LSE_SUPPORTED == 1 */
    uint8_t clk48_on;

    events = armv7m_atomic_swap(&stm32wb_usbd_dcd_device.events, 0);

    // armv7m_rtt_printf("DCD_EVENT_ENTER(state=%d, lpm_state=%d, events=%08x)\n", stm32wb_usbd_dcd_device.state, stm32wb_usbd_dcd_device.lpm_state, events);
    
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
#if !defined(__STM32WB_BOOT_CODE__)
            if (!stm32wb_hsem_lock(STM32WB_HSEM_INDEX_RNG, STM32WB_HSEM_PROCID_USBD_DCD))
            {
                if (!stm32wb_hsem_is_locked(STM32WB_HSEM_INDEX_RNG, STM32WB_HSEM_PROCID_RNG))
                {
                    armv7m_atomic_or(&stm32wb_usbd_dcd_device.events, events);

                    // armv7m_rtt_printf("DCD_EVENT_LEAVE(CLK48_OFF)\n");
                    
                    return;
                }
            }
#endif /* !defined(__STM32WB_BOOT_CODE__) */

            CRS->CR &= ~(CRS_CR_AUTOTRIMEN | CRS_CR_CEN | CRS_CR_SYNCOKIE);

            armv7m_atomic_and(&RCC->APB1ENR1, ~RCC_APB1ENR1_CRSEN);
            
#if !defined(__STM32WB_BOOT_CODE__)
            stm32wb_system_clk48_disable();

            stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RNG, STM32WB_HSEM_PROCID_USBD_DCD);
#endif /* !defined(__STM32WB_BOOT_CODE__) */

            stm32wb_usbd_dcd_device.clk48_on = false;
            
            // armv7m_rtt_printf("DCD_CLK48_OFF()\n");
        }
        else 
        {
#if !defined(__STM32WB_BOOT_CODE__)
            if (!stm32wb_hsem_lock(STM32WB_HSEM_INDEX_RNG, STM32WB_HSEM_PROCID_USBD_DCD))
            {
                if (!stm32wb_hsem_is_locked(STM32WB_HSEM_INDEX_RNG, STM32WB_HSEM_PROCID_RNG))
                {
                    armv7m_atomic_or(&stm32wb_usbd_dcd_device.events, events);

                    // armv7m_rtt_printf("DCD_EVENT_LEAVE(CLK48_ON)\n");
                    
                    return;
                }
            }
#endif /* !defined(__STM32WB_BOOT_CODE__) */

#if !defined(__STM32WB_BOOT_CODE__)
            stm32wb_system_clk48_enable();

            stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RNG, STM32WB_HSEM_PROCID_USBD_DCD);
#endif /* !defined(__STM32WB_BOOT_CODE__) */

            stm32wb_usbd_dcd_device.clk48_on = true;

            // armv7m_rtt_printf("DCD_CLK48_ON()\n");

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

#if (STM32WB_USBD_DCD_EP_SUPPORTED == 1)
#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)

static void stm32wb_usbd_dcd_ep_iso_in_routine(uint32_t ep_index, stm32wb_usbd_dcd_ep_in_t *ep_in, uint32_t sequence)
{
    uint32_t pma_address;
    const uint8_t *data;
    uint32_t count;
    stm32wb_usbd_dcd_ep_iso_in_callback_t ep_iso_in_callback;
    void *ep_iso_in_context;

    count = 0;
    
    ep_iso_in_callback = ep_in->iso.callback;
    ep_iso_in_context = ep_in->iso.context;
    
    // armv7m_rtt_printf("DCD_EP_ISO_IN_CALLBACK(ep_addr=%02x, p_data_return=%08x, p_count_return=%08x)\n", (ep_index | 0x80), &data, &count);

    if (ep_iso_in_callback)
    {
        (*ep_iso_in_callback)(ep_iso_in_context, (ep_index | 0x80), &data, &count);
    }

    pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control) + ((sequence == 0) ? 0 : ep_in->size);

    if (count > ep_in->size)
    {
        count = ep_in->size;
    }

    if (count)
    {
        stm32wb_usbd_dcd_pma_write(pma_address, data, count);
    }
    
    if (sequence == 0)
    {
        USB_PMA_TX_0_COUNT(ep_index, count);
    }
    else
    {
        USB_PMA_TX_1_COUNT(ep_index, count);
    }
}

static void stm32wb_usbd_dcd_ep_iso_out_routine(uint32_t ep_index, stm32wb_usbd_dcd_ep_out_t *ep_out, uint32_t sequence)
{
    uint32_t pma_address;
    uint16_t count;
    stm32wb_usbd_dcd_ep_iso_out_callback_t ep_iso_out_callback;
    void *ep_iso_out_context;

    if (sequence == 0)
    {
        count = USB_PMA_RX_0_COUNT(ep_index);
    }
    else
    {
        count = USB_PMA_RX_1_COUNT(ep_index);
    }

    pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control) + ((sequence == 0) ? 0 : ep_out->size);

    if (count > ep_out->iso.size)
    {
        count = ep_out->iso.size;
    }

    if (count)
    {
        stm32wb_usbd_dcd_pma_read(pma_address, ep_out->iso.data, count);
    }

    ep_out->iso.size = 0;
    
    ep_iso_out_callback = ep_out->iso.callback;
    ep_iso_out_context = ep_out->iso.context;
    
    // armv7m_rtt_printf("DCD_EP_ISO_OUT_CALLBACK(ep_addr=%02x, count=%d, p_data_return=%08x, p_size_return=%08x)\n", (ep_index | 0x80), count, &ep_out->iso.data, &ep_out->iso.size);

    if (ep_iso_out_callback)
    {
        (*ep_iso_out_callback)(ep_iso_out_context, (ep_index | 0x00), count, &ep_out->iso.data, &ep_out->iso.size);
    }
}

bool stm32wb_usbd_dcd_ep_iso_transmit(uint8_t ep_addr, const uint8_t *data_0, uint32_t count_0, const uint8_t *data_1, uint32_t count_1, stm32wb_usbd_dcd_ep_iso_in_callback_t callback, void *context)
{
    uint32_t ep_index, pma_address_0, pma_address_1;
    stm32wb_usbd_dcd_ep_in_t *ep_in;

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }
    
    ep_index = ep_addr & 0x07;

    ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

    if ((ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) != STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
    {
        return false;
    }
    
    if (ep_in->iso.callback || !callback)
    {
        return false;
    }

    stm32wb_usbd_dcd_device.iso_in_routine = stm32wb_usbd_dcd_ep_iso_in_routine;
    
    ep_in->iso.callback = callback;
    ep_in->iso.context = context;
    
    pma_address_0 = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control);

    if (count_0 > ep_in->size)
    {
        count_0 = ep_in->size;
    }

    if (count_0)
    {
        stm32wb_usbd_dcd_pma_write(pma_address_0, data_0, count_0);
    }

    USB_PMA_TX_0_ADDRESS(ep_index, pma_address_0);
    USB_PMA_TX_0_COUNT(ep_index, count_0);
    
    pma_address_1 = pma_address_0 + ep_in->size;

    if (count_1 > ep_in->size)
    {
        count_1 = ep_in->size;
    }

    if (count_1)
    {
        stm32wb_usbd_dcd_pma_write(pma_address_1, data_1, count_1);
    }

    USB_PMA_TX_1_ADDRESS(ep_index, pma_address_1);
    USB_PMA_TX_1_COUNT(ep_index, count_1);

    USB_EP_CONFIG(ep_index, (USB_EP_ISOCHRONOUS | ep_index));

    USB_EP_DTOG_TX_RESET(ep_index);
    USB_EP_STAT_RX(ep_index, USB_EP_RX_DIS);
    USB_EP_STAT_TX(ep_index, USB_EP_TX_VALID);
    
    return true;
}

bool stm32wb_usbd_dcd_ep_iso_receive(uint8_t ep_addr, uint8_t *data, uint32_t size, stm32wb_usbd_dcd_ep_iso_out_callback_t callback, void *context)
{
    uint32_t ep_index, pma_address_0, pma_address_1, pma_size;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }
    
    ep_index = ep_addr & 0x07;

    ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

    if ((ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_MASK) != STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
    {
        return false;
    }
    
    if (ep_out->iso.callback || !callback)
    {
        return false;
    }

    stm32wb_usbd_dcd_device.iso_out_routine = stm32wb_usbd_dcd_ep_iso_out_routine;
    
    ep_out->iso.callback = callback;
    ep_out->iso.context = context;
    ep_out->iso.data = data;
    ep_out->iso.size = size;
    
    pma_address_0 = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control);
    pma_address_1 = pma_address_0 + ep_out->size;
    pma_size = 0x8000 | ((ep_out->size - 32) << (10-5));

    USB_PMA_RX_0_ADDRESS(ep_index, pma_address_0);
    USB_PMA_RX_0_SIZE(ep_index, pma_size);
    USB_PMA_RX_1_ADDRESS(ep_index, pma_address_1);
    USB_PMA_RX_1_SIZE(ep_index, pma_size);

    USB_EP_CONFIG(ep_index, (USB_EP_ISOCHRONOUS | ep_index));

    USB_EP_DTOG_RX_RESET(ep_index);
    USB_EP_STAT_TX(ep_index, USB_EP_TX_DIS);
    USB_EP_STAT_RX(ep_index, USB_EP_RX_VALID);
    
    return true;
}

bool stm32wb_usbd_dcd_ep_iso_stop(uint8_t ep_addr)
{
    uint32_t ep_index;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;

    // armv7m_rtt_printf("DCD_EP_ISO_STOP(ep_addr=%02x)\n", ep_addr);
    
    if (stm32wb_usbd_dcd_device.state < STM32WB_USBD_DCD_STATE_CONNECTED)
    {
	return false;
    }

    ep_index = ep_addr & 0x07;

    if (ep_addr & 0x80)
    {
	ep_in = &stm32wb_usbd_dcd_device.ep_in[ep_index -1];

        if (!(ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO))
        {
            return false;
        }
    
        if (!ep_in->iso.callback)
        {
            return false;
        }
	
	USB_EP_STAT_TX(ep_index, USB_EP_TX_DIS);
        USB_EP_CTR_TX_RESET(ep_index);

        ep_in->iso.callback = NULL;
    }
    else
    {
	ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

        if (!(ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO))
        {
            return false;
        }
    
        if (!ep_out->iso.callback)
        {
            return false;
        }

	USB_EP_STAT_RX(ep_index, USB_EP_RX_DIS);
        USB_EP_CTR_RX_RESET(ep_index);
	
        ep_out->iso.callback = NULL;
    }

    return true;
}

#endif /* STM32WB_USBD_DCD_ISO_SUPPORTED == 1 */
#endif /* STM32WB_USBD_DCD_EP_SUPPORTED == 1 */

static void __attribute__((optimize("O3"), used)) stm32wb_usbd_dcd_usb_interrupt()
{
    uint32_t usb_istr, usb_ep, ep_index, count, events;
    uint8_t lpm_state;
#if (STM32WB_USBD_DCD_EP_SUPPORTED == 1)
    uint32_t pma_address;
    stm32wb_usbd_dcd_ep_in_t *ep_in;
    stm32wb_usbd_dcd_ep_out_t *ep_out;
    stm32wb_usbd_dcd_ep_in_callback_t ep_in_callback;
    stm32wb_usbd_dcd_ep_out_callback_t ep_out_callback;
    void *ep_in_context, *ep_out_context;
#endif /* STM32WB_USBD_DCD_EP_SUPPORTED == 1 */

    events = 0;

    usb_istr = USB->ISTR;

    // armv7m_rtt_printf("DCD_ISR_ENTER(cntr=%04x, istr=%04x, fnr=%08x)\n", USB->CNTR, usb_istr, USB->FNR);
    
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
			if (count > stm32wb_usbd_dcd_device.ep0_size)
			{
			    count = stm32wb_usbd_dcd_device.ep0_size;
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
#if (STM32WB_USBD_DCD_EP_SUPPORTED == 1)
	    else
	    {
		usb_ep = USB_EP_R(ep_index);

		if (usb_ep & USB_EP_CTR_RX)
		{
                    USB_EP_CTR_RX_RESET(ep_index);

		    ep_out = &stm32wb_usbd_dcd_device.ep_out[ep_index -1];

#if (STM32WB_USBD_DCD_ISO_SUPPORTED == 1)
		    if (ep_out->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
		    {
			/* ISO */

                        (*stm32wb_usbd_dcd_device.iso_out_routine)(ep_index, ep_out, ((usb_ep & USB_EP_DTOG_TX) ? 0 : 1));
		    }
		    else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
                    {
			/* BULK / INTERRUPT */

                        count = USB_PMA_RX_COUNT(ep_index);

                        if (count > ep_out->bulk.size)
                        {
                            count = ep_out->bulk.size;
                        }
                        
                        if (count)
                        {
                            pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_out->control);
                            
                            stm32wb_usbd_dcd_pma_read(pma_address, ep_out->bulk.data, count);
                            
                            ep_out->bulk.data += count;
                            ep_out->bulk.size -= count;
                            ep_out->bulk.count += count;
                        }
                        
                        if ((count == ep_out->size) && ep_out->bulk.size)
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
		    if (ep_in->control & STM32WB_USBD_DCD_EP_CONTROL_TYPE_ISO)
		    {
			/* ISO */

                        (*stm32wb_usbd_dcd_device.iso_in_routine)(ep_index, ep_in, ((usb_ep & USB_EP_DTOG_RX) ? 0 : 1));
		    }
		    else
#endif /* (STM32WB_USBD_DCD_ISO_SUPPORTED == 1) */
		    {
                        count = ep_in->bulk.count;

                        if (count)
                        {
                            pma_address = STM32WB_USBD_DCD_EP_CONTROL_PMA_ADDRESS(ep_in->control);

                            if (count > ep_in->size)
                            {
                                count = ep_in->size;
                            }

                            stm32wb_usbd_dcd_pma_write(pma_address, ep_in->bulk.data, count);
                                
                            ep_in->bulk.data += count;
                            ep_in->bulk.count -= count;

                            USB_PMA_TX_COUNT(ep_index, count);
                                
                            USB_EP_STAT_TX(ep_index, USB_EP_TX_VALID);
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
#endif /* STM32WB_USBD_DCD_EP_SUPPORTED == 1 */
	    
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

#if !defined(__STM32WB_BOOT_CODE__)
                    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
#endif /* !defined(__STM32WB_BOOT_CODE__) */
                    
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

#if !defined(__STM32WB_BOOT_CODE__)
                stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);
#endif /* !defined(__STM32WB_BOOT_CODE__) */
    
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

#if !defined(__STM32WB_BOOT_CODE__)
                stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
#endif /* !defined(__STM32WB_BOOT_CODE__) */
            }

#if (STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1)
            stm32wb_lptim_timeout_stop(&stm32wb_usbd_dcd_device.timeout);
#endif /* STM32WB_USBD_DCD_REMOTE_WAKEUP_SUPPORTED == 1 */

	    events |= STM32WB_USBD_DCD_EVENT_RESET;
	}
    }
    
    if (events)
    {
        armv7m_atomic_or(&stm32wb_usbd_dcd_device.events, events);
        
#if !defined(__STM32WB_BOOT_CODE__)
        armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_DCD);
#else /* !defined(__STM32WB_BOOT_CODE__) */
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
#endif /* !defined(__STM32WB_BOOT_CODE__) */
    }

    // armv7m_rtt_printf("DCD_ISR_LEAVE(cntr=%04x, istr=%04x, lpm_state=%d, events=%08x)\n", USB->CNTR, USB->ISTR, stm32wb_usbd_dcd_device.lpm_state, events);
}

static void __attribute__((optimize("O3"), used)) stm32wb_usbd_dcd_crs_interrupt()
{
    CRS->ICR = CRS_ICR_SYNCOKC;

    stm32wb_usbd_dcd_device.clk48_sync = true;
    
#if !defined(__STM32WB_BOOT_CODE__)
    armv7m_pendsv_raise(ARMV7M_PENDSV_SWI_USBD_DCD);
#else /* !defined(__STM32WB_BOOT_CODE__) */
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
#endif /* !defined(__STM32WB_BOOT_CODE__) */
}

#if !defined(__STM32WB_BOOT_CODE__)

void USB_LP_IRQHandler(void)
{
    stm32wb_usbd_dcd_usb_interrupt();
    
    __DSB();
}

void USB_HP_IRQHandler(void)
{
    stm32wb_usbd_dcd_usb_interrupt();

    __DSB();
}

void CRS_IRQHandler(void)
{
    stm32wb_usbd_dcd_crs_interrupt();
    
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

#endif /* !defined(__STM32WB_BOOT_CODE__) */
