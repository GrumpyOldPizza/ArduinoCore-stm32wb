/*
 * Copyright (c) 2017-2024 Thomas Roell.  All rights reserved.
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
#include "stm32wb_sfspi.h"
#include "stm32wb_sflash.h"
#include "stm32wb_system.h"

#define STM32WB_SFSPI_INSN_WRSR                  0x01
#define STM32WB_SFSPI_INSN_PAGE_PROGRAM          0x02
#define STM32WB_SFSPI_INSN_READ                  0x03 /* up to 33MHz SPI clock */
#define STM32WB_SFSPI_INSN_FAST_READ             0x0b
#define STM32WB_SFSPI_INSN_RDSR                  0x05
#define STM32WB_SFSPI_INSN_WREN                  0x06
#define STM32WB_SFSPI_INSN_RES                   0x90
#define STM32WB_SFSPI_INSN_RDID                  0x9f
#define STM32WB_SFSPI_INSN_SECTOR_ERASE          0x20 /* 4k ERASE */
#define STM32WB_SFSPI_INSN_BLOCK_ERASE           0xd8 /* 64K ERASE */

#define STM32WB_SFSPI_SR_WIP                     0x01 /* write in progress */
#define STM32WB_SFSPI_SR_WEL                     0x02 /* write enable latch */

#define STM32WB_SFSPI_MID_SPANSION               0x01
#define STM32WB_SFSPI_MID_MACRONIX               0xc2
#define STM32WB_SFSPI_MID_WINBOND                0xef

/* MACRONIX */
#define STM32WB_SFSPI_INSN_RDCONF                0x15
#define STM32WB_SFSPI_INSN_RDSCUR                0x2b
#define STM32WB_SFSPI_INSN_WRSCUR                0x2f
#define STM32WB_SFSPI_INSN_WREAR                 0xc5
#define STM32WB_SFSPI_INSN_PERSUS                0xb0
#define STM32WB_SFSPI_INSN_PERRSM                0x30
#define STM32WB_SFSPI_INSN_RSTEN                 0x66
#define STM32WB_SFSPI_INSN_RST                   0x99
#define STM32WB_SFSPI_INSN_RDPD                  0xab
#define STM32WB_SFSPI_INSN_DPD                   0xb9
#define STM32WB_SFSPI_INSN_EN4B                  0xb7
#define STM32WB_SFSPI_INSN_EX4B                  0xe9

#define STM32WB_SFSPI_SR_QE                      0x40
#define STM32WB_SFSPI_CONF2_HPM                  0x02
#define STM32WB_SFSPI_SCUR_PSB                   0x08
#define STM32WB_SFSPI_SCUR_ESB                   0x10
#define STM32WB_SFSPI_SCUR_P_FAIL                0x20
#define STM32WB_SFSPI_SCUR_E_FAIL                0x40

/* SPANSION */
#define STM32WB_SFSPI_INSN_RDSR2                 0x07
#define STM32WB_SFSPI_INSN_RDCR                  0x35
#define STM32WB_SFSPI_INSN_CLSR                  0x30
#define STM32WB_SFSPI_INSN_BRWR                  0x17
#define STM32WB_SFSPI_INSN_ERSP                  0x75
#define STM32WB_SFSPI_INSN_ERRS                  0x7a
#define STM32WB_SFSPI_INSN_PGSP                  0x85
#define STM32WB_SFSPI_INSN_PGRS                  0x8a
#define STM32WB_SFSPI_INSN_RESET                 0xf0

#define STM32WB_SFSPI_SR_P_ERR                   0x40
#define STM32WB_SFSPI_SR_E_ERR                   0x20
#define STM32WB_SFSPI_CR_QE                      0x02
#define STM32WB_SFSPI_SR2_PS                     0x01
#define STM32WB_SFSPI_SR2_ES                     0x02

#define STM32WB_SFSPI_BR_EXTADD                  0x80

/* WINBOND */
#define STM32WB_SFSPI_INSN_WREN_VOLATILE         0x50
#define STM32WB_SFSPI_INSN_WREAR                 0xc5
#define STM32WB_SFSPI_INSN_EPS                   0x75
#define STM32WB_SFSPI_INSN_EPR                   0x7a
#define STM32WB_SFSPI_INSN_RSTEN                 0x66
#define STM32WB_SFSPI_INSN_RST                   0x99
#define STM32WB_SFSPI_INSN_RDPD                  0xab
#define STM32WB_SFSPI_INSN_DPD                   0xb9
#define STM32WB_SFSPI_INSN_EN4B                  0xb7
#define STM32WB_SFSPI_INSN_EX4B                  0xe9

#define STM32WB_SFSPI_SR2_QE                     0x02
#define STM32WB_SFSPI_SR2_SUS                    0x80

#define STM32WB_SFSPI_STATE_NONE                 0
#define STM32WB_SFSPI_STATE_NOT_READY            1
#define STM32WB_SFSPI_STATE_READY                2
#define STM32WB_SFSPI_STATE_DATA                 3

#define STM32WB_SFSPI_BUSY_ERASE                 0x01
#define STM32WB_SFSPI_BUSY_PROGRAM               0x02
#define STM32WB_SFSPI_BUSY_SLEEP                 0x04
#define STM32WB_SFSPI_BUSY_SUSPENDED_ERASE       0x08
#define STM32WB_SFSPI_BUSY_SUSPENDED_PROGRAM     0x10

typedef struct _stm32wb_sfspi_device_t {
    stm32wb_spi_t                  *spi;
    volatile uint8_t               state;
    uint8_t                        busy;
    volatile uint8_t               refcount;
    stm32wb_sfspi_pins_t           pins; 
    stm32wb_sflash_info_t          info;
    stm32wb_system_notify_t        notify;
    SPI_TypeDef                    *SPI;
    GPIO_TypeDef                   *GPIO;
    uint16_t                       mask;
    uint8_t                        address;
    uint32_t                       clock;
    uint8_t                        insn_erase_suspend;
    uint8_t                        insn_erase_resume;
    uint8_t                        insn_program_suspend;
    uint8_t                        insn_program_resume;
    volatile uint8_t               *p_status_return;
} stm32wb_sfspi_device_t;

static stm32wb_sfspi_device_t stm32wb_sfspi_device;

static void stm32wb_sfspi_status(uint8_t status);
static bool stm32wb_sfspi_busy(void);

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfspi_command(uint8_t insn)
{
    SPI_TypeDef *SPI = stm32wb_sfspi_device.SPI;
    
    stm32wb_sfspi_device.GPIO->BRR = stm32wb_sfspi_device.mask;
    STM32WB_SPI_DATA_8(SPI, insn);
    stm32wb_sfspi_device.GPIO->BSRR = stm32wb_sfspi_device.mask;
}

static inline __attribute__((optimize("O3"),always_inline)) uint8_t stm32wb_sfspi_command_read_1(uint8_t insn)
{
    SPI_TypeDef *SPI = stm32wb_sfspi_device.SPI;
    uint8_t data_0;

    stm32wb_sfspi_device.GPIO->BRR = stm32wb_sfspi_device.mask;
    STM32WB_SPI_DATA_8(SPI, insn);
    data_0 = STM32WB_SPI_DATA_8(SPI, 0xff);
    stm32wb_sfspi_device.GPIO->BSRR = stm32wb_sfspi_device.mask;

    return data_0;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfspi_command_read_2(uint8_t insn, uint8_t *p_data_0, uint8_t *p_data_1)
{
    SPI_TypeDef *SPI = stm32wb_sfspi_device.SPI;

    stm32wb_sfspi_device.GPIO->BRR = stm32wb_sfspi_device.mask;
    STM32WB_SPI_DATA_8(SPI, insn);
    *p_data_0 = STM32WB_SPI_DATA_8(SPI, 0xff);
    *p_data_1 = STM32WB_SPI_DATA_8(SPI, 0xff);
    stm32wb_sfspi_device.GPIO->BSRR = stm32wb_sfspi_device.mask;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfspi_command_read_3(uint8_t insn, uint8_t *p_data_0, uint8_t *p_data_1, uint8_t *p_data_2)
{
    SPI_TypeDef *SPI = stm32wb_sfspi_device.SPI;

    stm32wb_sfspi_device.GPIO->BRR = stm32wb_sfspi_device.mask;
    STM32WB_SPI_DATA_8(SPI, insn);
    *p_data_0 = STM32WB_SPI_DATA_8(SPI, 0xff);
    *p_data_1 = STM32WB_SPI_DATA_8(SPI, 0xff);
    *p_data_2 = STM32WB_SPI_DATA_8(SPI, 0xff);
    stm32wb_sfspi_device.GPIO->BSRR = stm32wb_sfspi_device.mask;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfspi_command_write_1(uint8_t insn, uint8_t data_0)
{
    SPI_TypeDef *SPI = stm32wb_sfspi_device.SPI;

    stm32wb_sfspi_device.GPIO->BRR = stm32wb_sfspi_device.mask;
    STM32WB_SPI_DATA_8(SPI, insn);
    STM32WB_SPI_DATA_8(SPI, data_0);
    stm32wb_sfspi_device.GPIO->BSRR = stm32wb_sfspi_device.mask;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfspi_command_write_2(uint8_t insn, uint8_t data_0, uint8_t data_1)
{
    SPI_TypeDef *SPI = stm32wb_sfspi_device.SPI;

    stm32wb_sfspi_device.GPIO->BRR = stm32wb_sfspi_device.mask;
    STM32WB_SPI_DATA_8(SPI, insn);
    STM32WB_SPI_DATA_8(SPI, data_0);
    STM32WB_SPI_DATA_8(SPI, data_1);
    stm32wb_sfspi_device.GPIO->BSRR = stm32wb_sfspi_device.mask;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfspi_command_write_3(uint8_t insn, uint8_t data_0, uint8_t data_1, uint8_t data_2)
{
    SPI_TypeDef *SPI = stm32wb_sfspi_device.SPI;

    stm32wb_sfspi_device.GPIO->BRR = stm32wb_sfspi_device.mask;
    STM32WB_SPI_DATA_8(SPI, insn);
    STM32WB_SPI_DATA_8(SPI, data_0);
    STM32WB_SPI_DATA_8(SPI, data_1);
    STM32WB_SPI_DATA_8(SPI, data_2);
    stm32wb_sfspi_device.GPIO->BSRR = stm32wb_sfspi_device.mask;
}

static inline __attribute__((optimize("O3"),always_inline)) uint8_t stm32wb_sfspi_command_signature(void)
{
    SPI_TypeDef *SPI = stm32wb_sfspi_device.SPI;
    uint8_t signature;
    
    stm32wb_sfspi_device.GPIO->BRR = stm32wb_sfspi_device.mask;
    STM32WB_SPI_DATA_8(SPI, STM32WB_SFSPI_INSN_RES);
    STM32WB_SPI_DATA_8(SPI, 0x00);
    STM32WB_SPI_DATA_8(SPI, 0x00);
    STM32WB_SPI_DATA_8(SPI, 0x00);
    signature = STM32WB_SPI_DATA_8(SPI, 0xff);
    stm32wb_sfspi_device.GPIO->BSRR = stm32wb_sfspi_device.mask;

    return signature;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfspi_command_erase(uint32_t address)
{
    SPI_TypeDef *SPI = stm32wb_sfspi_device.SPI;

    stm32wb_sfspi_device.GPIO->BRR = stm32wb_sfspi_device.mask;
    STM32WB_SPI_DATA_8(SPI, ((address < 0x00010000) ? STM32WB_SFSPI_INSN_SECTOR_ERASE : STM32WB_SFSPI_INSN_BLOCK_ERASE));
    STM32WB_SPI_DATA_8(SPI, (address >> 16));
    STM32WB_SPI_DATA_8(SPI, (address >> 8));
    STM32WB_SPI_DATA_8(SPI, (address >> 0));
    stm32wb_sfspi_device.GPIO->BSRR = stm32wb_sfspi_device.mask;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfspi_command_program(uint32_t address, const uint8_t *data, uint32_t count)
{
    SPI_TypeDef *SPI = stm32wb_sfspi_device.SPI;
    
    stm32wb_sfspi_device.GPIO->BRR = stm32wb_sfspi_device.mask;
    STM32WB_SPI_DATA_8(SPI, STM32WB_SFSPI_INSN_PAGE_PROGRAM);
    STM32WB_SPI_DATA_8(SPI, (address >> 16));
    STM32WB_SPI_DATA_8(SPI, (address >> 8));
    STM32WB_SPI_DATA_8(SPI, (address >> 0));
    stm32wb_spi_data_transmit(stm32wb_sfspi_device.spi, data, count);
    stm32wb_sfspi_device.GPIO->BSRR = stm32wb_sfspi_device.mask;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfspi_command_read(uint32_t address, uint8_t *data, uint32_t count)
{
    SPI_TypeDef *SPI = stm32wb_sfspi_device.SPI;

    stm32wb_sfspi_device.GPIO->BRR = stm32wb_sfspi_device.mask;
    STM32WB_SPI_DATA_8(SPI, STM32WB_SFSPI_INSN_READ);
    STM32WB_SPI_DATA_8(SPI, (address >> 16));
    STM32WB_SPI_DATA_8(SPI, (address >> 8));
    STM32WB_SPI_DATA_8(SPI, (address >> 0));
    stm32wb_spi_data_receive(stm32wb_sfspi_device.spi, data, count);
    stm32wb_sfspi_device.GPIO->BSRR = stm32wb_sfspi_device.mask;
}

static __attribute__((optimize("O3"),noinline)) uint8_t stm32wb_sfspi_write_enable(void)
{
    uint8_t status;

    do
    {
	stm32wb_sfspi_command(STM32WB_SFSPI_INSN_WREN);

	status = stm32wb_sfspi_command_read_1(STM32WB_SFSPI_INSN_RDSR);
    }
    while (!(status & STM32WB_SFSPI_SR_WEL));

    return status;
}

static __attribute__((optimize("O3"),noinline)) void stm32wb_sfspi_address(uint32_t address)
{
    if (stm32wb_sfspi_device.address != (address >> 24))
    {
	stm32wb_sfspi_device.address = (address >> 24);
	
	if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_WINBOND)
	{
	    stm32wb_sfspi_command(STM32WB_SFSPI_INSN_WREN_VOLATILE);
	}
	
	stm32wb_sfspi_command_write_1(STM32WB_SFSPI_INSN_WREAR, stm32wb_sfspi_device.address);
    }
}

static __attribute__((noinline)) bool stm32wb_sfspi_enable(void)
{
    if (stm32wb_sfspi_device.refcount >= 1)
    {
        stm32wb_sfspi_device.refcount++;

	return true;
    }

    if (stm32wb_sfspi_device.state != STM32WB_SFSPI_STATE_NOT_READY)
    {
	return false;
    }

    stm32wb_sfspi_device.refcount = 1;
    
    if (stm32wb_system_info.options & STM32WB_SYSTEM_OPTION_SFLASH_BOOST)
    {
        stm32wb_system_boost_enable();
    }

    stm32wb_sfspi_device.state = STM32WB_SFSPI_STATE_READY;

    return true;
}

static __attribute__((noinline)) bool stm32wb_sfspi_disable(void)
{
    if (stm32wb_sfspi_device.refcount > 1)
    {
        stm32wb_sfspi_device.refcount--;

	return true;
    }

    if (stm32wb_sfspi_device.state != STM32WB_SFSPI_STATE_READY)
    {
	return false;
    }

    if (stm32wb_sfspi_device.busy & (STM32WB_SFSPI_BUSY_ERASE | STM32WB_SFSPI_BUSY_PROGRAM | STM32WB_SFSPI_BUSY_SUSPENDED_ERASE | STM32WB_SFSPI_BUSY_SUSPENDED_PROGRAM)) 
    {
	return false;
    }

    stm32wb_sfspi_device.refcount = 0;
    
    if (stm32wb_system_info.options & STM32WB_SYSTEM_OPTION_SFLASH_BOOST)
    {
        stm32wb_system_boost_disable();
    }
    
    stm32wb_sfspi_device.state = STM32WB_SFSPI_STATE_NOT_READY;
    
    return true;
}

static bool stm32wb_sfspi_acquire(void)
{
    if (stm32wb_sfspi_device.state != STM32WB_SFSPI_STATE_READY)
    {
	return false;
    }

    if (!stm32wb_spi_acquire(stm32wb_sfspi_device.spi, stm32wb_sfspi_device.clock, 0))
    {
	return false;
    }

    stm32wb_gpio_pin_output(stm32wb_sfspi_device.pins.cs);

    if (stm32wb_sfspi_device.busy & STM32WB_SFSPI_BUSY_SLEEP)
    {
        stm32wb_sfspi_command(STM32WB_SFSPI_INSN_RDPD);
        
        armv7m_core_udelay(50);

	stm32wb_sfspi_device.busy &= ~STM32WB_SFSPI_BUSY_SLEEP;
    }
    
    stm32wb_sfspi_device.state = STM32WB_SFSPI_STATE_DATA;

    return true;
}

static bool stm32wb_sfspi_release(void)
{
    if (stm32wb_sfspi_device.state != STM32WB_SFSPI_STATE_DATA)
    {
	return false;
    }

    stm32wb_sfspi_device.state = STM32WB_SFSPI_STATE_READY;

    stm32wb_spi_release(stm32wb_sfspi_device.spi);

    stm32wb_gpio_pin_analog(stm32wb_sfspi_device.pins.cs);
    
    return true;
}

static void stm32wb_sfspi_sleep(void)
{
    if (stm32wb_sfspi_device.state != STM32WB_SFSPI_STATE_READY)
    {
	return;
    }

    if (stm32wb_sfspi_device.busy & STM32WB_SFSPI_BUSY_SLEEP)
    {
	return;
    }

    if ((stm32wb_sfspi_device.info.mid != STM32WB_SFSPI_MID_MACRONIX) && (stm32wb_sfspi_device.info.mid != STM32WB_SFSPI_MID_WINBOND))
    {
        return;
    }

    if (!stm32wb_sfspi_acquire())
    {
	return;
    }

    if (!stm32wb_sfspi_busy())
    {
        stm32wb_sfspi_command(STM32WB_SFSPI_INSN_DPD);

        stm32wb_sfspi_device.busy |= STM32WB_SFSPI_BUSY_SLEEP;
    }
    
    stm32wb_sfspi_release();
}

static __attribute__((optimize("O3"),noinline)) void stm32wb_sfspi_status(uint8_t status)
{
    uint8_t scur;
    bool success;
    
    success = true;

    if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_MACRONIX)
    {
	scur = stm32wb_sfspi_command_read_1(STM32WB_SFSPI_INSN_RDSCUR);

	if (stm32wb_sfspi_device.busy & STM32WB_SFSPI_BUSY_ERASE)
	{
	    if (scur & STM32WB_SFSPI_SCUR_E_FAIL)
	    {
		success = false;
	    }
	}
	else
	{
	    if (scur & STM32WB_SFSPI_SCUR_P_FAIL)
	    {
		success = false;
	    }
	}
    }
    
    if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_SPANSION)
    {
	if (status & (STM32WB_SFSPI_SR_E_ERR | STM32WB_SFSPI_SR_P_ERR))
	{
	    stm32wb_sfspi_command(STM32WB_SFSPI_INSN_CLSR);
	    
	    success = false;
	}
    }
    
    if (stm32wb_sfspi_device.p_status_return)
    {
	*stm32wb_sfspi_device.p_status_return = success ? STM32WB_SFLASH_STATUS_SUCCESS : STM32WB_SFLASH_STATUS_FAILURE;
    }
    
    stm32wb_sfspi_device.busy &= ~(STM32WB_SFSPI_BUSY_ERASE | STM32WB_SFSPI_BUSY_PROGRAM);
}

static __attribute__((optimize("O3"),noinline)) bool stm32wb_sfspi_busy(void)
{
    uint8_t status;
    
    if (stm32wb_sfspi_device.state != STM32WB_SFSPI_STATE_DATA)
    {
	return false;
    }

    if (!(stm32wb_sfspi_device.busy & (STM32WB_SFSPI_BUSY_ERASE | STM32WB_SFSPI_BUSY_PROGRAM)))
    {
	return false;
    }

    status = stm32wb_sfspi_command_read_1(STM32WB_SFSPI_INSN_RDSR);

    if (status & (STM32WB_SFSPI_SR_WEL | STM32WB_SFSPI_SR_WIP))
    {
	return true;
    }

    stm32wb_sfspi_status(status);

    return false;
}

static bool stm32wb_sfspi_suspend(void)
{
    uint8_t status, status2, scur;
    bool success;
    
    if (stm32wb_sfspi_device.state != STM32WB_SFSPI_STATE_DATA)
    {
	return false;
    }

    if (stm32wb_sfspi_device.busy & (STM32WB_SFSPI_BUSY_SUSPENDED_ERASE | STM32WB_SFSPI_BUSY_SUSPENDED_PROGRAM))
    {
	return false;
    }
    
    if (!stm32wb_sfspi_busy())
    {
	return false;
    }
    
    stm32wb_sfspi_command((stm32wb_sfspi_device.busy & STM32WB_SFSPI_BUSY_ERASE) ? stm32wb_sfspi_device.insn_erase_suspend : stm32wb_sfspi_device.insn_program_suspend);

    do
    {
	status = stm32wb_sfspi_command_read_1(STM32WB_SFSPI_INSN_RDSR);
    }
    while (status & (STM32WB_SFSPI_SR_WEL | STM32WB_SFSPI_SR_WIP));

    success = true;

    if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_SPANSION)
    {
	status2 = stm32wb_sfspi_command_read_1(STM32WB_SFSPI_INSN_RDSR2);
	
	if (!(status2 & (STM32WB_SFSPI_SR2_ES | STM32WB_SFSPI_SR2_PS)))
	{
	    success = false;
	}
    }
    
    if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_MACRONIX)
    {
	scur = stm32wb_sfspi_command_read_1(STM32WB_SFSPI_INSN_RDSCUR);
	
	if (!(scur & (STM32WB_SFSPI_SCUR_ESB | STM32WB_SFSPI_SCUR_PSB)))
	{
	    success = false;
	}
    }

    if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_WINBOND)
    {
	stm32wb_sfspi_command_read_2(STM32WB_SFSPI_INSN_RDSR, &status, &status2);

	if (!(status2 & STM32WB_SFSPI_SR2_SUS))
	{
	    success = false;
	}
    }
    
    if (success)
    {
	if (stm32wb_sfspi_device.busy & STM32WB_SFSPI_BUSY_ERASE)
	{
	    stm32wb_sfspi_device.busy = (stm32wb_sfspi_device.busy & ~STM32WB_SFSPI_BUSY_ERASE) | STM32WB_SFSPI_BUSY_SUSPENDED_ERASE;
	}
	else
	{
	    stm32wb_sfspi_device.busy = (stm32wb_sfspi_device.busy & ~STM32WB_SFSPI_BUSY_PROGRAM) | STM32WB_SFSPI_BUSY_SUSPENDED_PROGRAM;
	}

	return true;
    }
    else
    {
	stm32wb_sfspi_status(status);

	return false;
    }
}

static bool stm32wb_sfspi_resume(void)
{
    if (stm32wb_sfspi_device.state != STM32WB_SFSPI_STATE_DATA)
    {
	return false;
    }

    if (!(stm32wb_sfspi_device.busy & (STM32WB_SFSPI_BUSY_SUSPENDED_ERASE | STM32WB_SFSPI_BUSY_SUSPENDED_PROGRAM)))
    {
	return false;
    }
    
    stm32wb_sfspi_command((stm32wb_sfspi_device.busy & STM32WB_SFSPI_BUSY_SUSPENDED_ERASE) ? stm32wb_sfspi_device.insn_erase_resume : stm32wb_sfspi_device.insn_program_resume);
    
    if (stm32wb_sfspi_device.busy & STM32WB_SFSPI_BUSY_SUSPENDED_PROGRAM)
    {
	stm32wb_sfspi_device.busy = (stm32wb_sfspi_device.busy & ~STM32WB_SFSPI_BUSY_SUSPENDED_PROGRAM) | STM32WB_SFSPI_BUSY_PROGRAM;
    }
    else
    {
	stm32wb_sfspi_device.busy = (stm32wb_sfspi_device.busy & ~STM32WB_SFSPI_BUSY_SUSPENDED_ERASE) | STM32WB_SFSPI_BUSY_ERASE;
    }
    
    return true;
}

static bool stm32wb_sfspi_erase(uint32_t address, volatile uint8_t *p_status_return)
{
    if (stm32wb_sfspi_device.state != STM32WB_SFSPI_STATE_DATA)
    {
	return false;
    }

    if (stm32wb_sfspi_device.busy & (STM32WB_SFSPI_BUSY_SUSPENDED_ERASE | STM32WB_SFSPI_BUSY_SUSPENDED_PROGRAM | STM32WB_SFSPI_BUSY_ERASE | STM32WB_SFSPI_BUSY_PROGRAM))
    {
	return false;
    }

    if (p_status_return)
    {
	*p_status_return = STM32WB_SFLASH_STATUS_BUSY;
    }
    
    stm32wb_sfspi_device.p_status_return = p_status_return;
    stm32wb_sfspi_device.busy = STM32WB_SFSPI_BUSY_ERASE;
    
    if (stm32wb_sfspi_device.info.capacity > 0x01000000)
    {
	stm32wb_sfspi_address(address);
    }
    
    stm32wb_sfspi_write_enable();

    stm32wb_sfspi_command_erase(address);
    
    return true;
}

static __attribute__((optimize("O3"),noinline)) bool stm32wb_sfspi_program(uint32_t address, const uint8_t *data, uint32_t count, volatile uint8_t *p_status_return)
{
    if (stm32wb_sfspi_device.state != STM32WB_SFSPI_STATE_DATA)
    {
	return false;
    }

    if (stm32wb_sfspi_device.busy & (STM32WB_SFSPI_BUSY_SUSPENDED_ERASE | STM32WB_SFSPI_BUSY_SUSPENDED_PROGRAM | STM32WB_SFSPI_BUSY_ERASE | STM32WB_SFSPI_BUSY_PROGRAM))
    {
	return false;
    }
    
    if (p_status_return)
    {
	*p_status_return = STM32WB_SFLASH_STATUS_BUSY;
    }
    
    stm32wb_sfspi_device.p_status_return = p_status_return;
    stm32wb_sfspi_device.busy = STM32WB_SFSPI_BUSY_PROGRAM;

    if (stm32wb_sfspi_device.info.capacity > 0x01000000)
    {
	stm32wb_sfspi_address(address);
    }

    stm32wb_sfspi_write_enable();

    stm32wb_sfspi_command_program(address, data, count);
    
    return true;
}

static __attribute__((optimize("O3"),noinline)) bool stm32wb_sfspi_read(uint32_t address, uint8_t *data, uint32_t count)
{
    if (stm32wb_sfspi_device.state != STM32WB_SFSPI_STATE_DATA)
    {
	return false;
    }

    if (stm32wb_sfspi_device.busy & (STM32WB_SFSPI_BUSY_ERASE | STM32WB_SFSPI_BUSY_PROGRAM))
    {
	return false;
    }

    if (stm32wb_sfspi_device.info.capacity > 0x01000000)
    {
	stm32wb_sfspi_address(address);
    }
    
    stm32wb_sfspi_command_read(address, data, count);
    
    return true;
}

static const stm32wb_sflash_interface_t stm32wb_sfspi_interface = {
    stm32wb_sfspi_enable,
    stm32wb_sfspi_disable,
    stm32wb_sfspi_acquire,
    stm32wb_sfspi_release,
    stm32wb_sfspi_busy,
    stm32wb_sfspi_suspend,
    stm32wb_sfspi_resume,
    stm32wb_sfspi_erase,
    stm32wb_sfspi_program,
    stm32wb_sfspi_read,
};

bool stm32wb_sfspi_initialize(stm32wb_spi_t *spi, const stm32wb_sfspi_params_t *params)
{
    uint32_t group, index;
    uint8_t MID, DID[2], status;
    bool success;
    
    if (stm32wb_sfspi_device.state != STM32WB_SFSPI_STATE_NONE)
    {
	return false;
    }
    
    if (spi->state == STM32WB_SPI_STATE_NONE)
    {
	return false;
    }

    stm32wb_sfspi_device.state = STM32WB_SFSPI_STATE_NOT_READY;
	
    stm32wb_sfspi_device.spi = spi;
    stm32wb_sfspi_device.pins = params->pins;

    stm32wb_sfspi_device.SPI = spi->SPI;

    group = (params->pins.cs & STM32WB_GPIO_PIN_GROUP_MASK) >> STM32WB_GPIO_PIN_GROUP_SHIFT;
    index = (params->pins.cs & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT;
        
    stm32wb_sfspi_device.GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);
    stm32wb_sfspi_device.mask = (1 << index);

    stm32wb_sfspi_device.clock = 8000000;
    
    stm32wb_gpio_pin_configure(stm32wb_sfspi_device.pins.cs, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_ODATA_1 | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));

    stm32wb_sfspi_enable();
    
    stm32wb_sfspi_acquire();
    
    stm32wb_sfspi_command(STM32WB_SFSPI_INSN_RDPD);
    
    armv7m_core_udelay(200);

    stm32wb_sfspi_device.info.mid = stm32wb_sfspi_command_signature();

    success = false;

    if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_SPANSION)
    {
	success = true;
    }
    
    if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_MACRONIX)
    {
	success = true;
    }

    if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_WINBOND)
    {
	success = true;
    }

    if (success)
    {
	status = stm32wb_sfspi_command_read_1(STM32WB_SFSPI_INSN_RDSR);
	
	if (status & (STM32WB_SFSPI_SR_WEL | STM32WB_SFSPI_SR_WIP))
	{
	    if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_SPANSION)
	    {
		stm32wb_sfspi_command(STM32WB_SFSPI_INSN_RESET);
	    }
	    else
	    {
		stm32wb_sfspi_command(STM32WB_SFSPI_INSN_RSTEN);
		stm32wb_sfspi_command(STM32WB_SFSPI_INSN_RST);
	    }
	    
	    armv7m_core_udelay(100);
	    
	    do
	    {
		status = stm32wb_sfspi_command_read_1(STM32WB_SFSPI_INSN_RDSR);
	    } 
	    while (status & (STM32WB_SFSPI_SR_WEL | STM32WB_SFSPI_SR_WIP));
	}

	stm32wb_sfspi_command_read_3(STM32WB_SFSPI_INSN_RDID, &MID, &DID[0], &DID[1]);

	if ((stm32wb_sfspi_device.info.mid != MID) || ((DID[0] == 0x00) && (DID[1] == 0x00)) || ((DID[0] == 0xff) && (DID[1] == 0xff)))
	{
	    success = false;
	}
    }

    if (success)
    {
        stm32wb_sfspi_device.info.did = (DID[0] << 8) | (DID[1] << 0);
        stm32wb_sfspi_device.info.capacity = (1 << DID[1]);
        stm32wb_sfspi_device.info.block_size = 65536;
        stm32wb_sfspi_device.info.page_size = 256;

	stm32wb_sfspi_device.clock = 32000000;

	if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_SPANSION)
	{
	    stm32wb_sfspi_device.insn_erase_suspend = STM32WB_SFSPI_INSN_ERSP;
	    stm32wb_sfspi_device.insn_erase_resume = STM32WB_SFSPI_INSN_ERRS;
	    stm32wb_sfspi_device.insn_program_suspend = STM32WB_SFSPI_INSN_PGSP;
	    stm32wb_sfspi_device.insn_program_resume = STM32WB_SFSPI_INSN_PGRS;
	}

	if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_MACRONIX)
	{
	    stm32wb_sfspi_device.insn_erase_suspend = STM32WB_SFSPI_INSN_PERSUS;
	    stm32wb_sfspi_device.insn_erase_resume = STM32WB_SFSPI_INSN_PERRSM;
	    stm32wb_sfspi_device.insn_program_suspend = STM32WB_SFSPI_INSN_PERSUS;
	    stm32wb_sfspi_device.insn_program_resume = STM32WB_SFSPI_INSN_PERRSM;
	}

	if (stm32wb_sfspi_device.info.mid == STM32WB_SFSPI_MID_WINBOND)
	{
	    stm32wb_sfspi_device.insn_erase_suspend = STM32WB_SFSPI_INSN_EPS;
	    stm32wb_sfspi_device.insn_erase_resume = STM32WB_SFSPI_INSN_EPR;
	    stm32wb_sfspi_device.insn_program_suspend = STM32WB_SFSPI_INSN_EPS;
	    stm32wb_sfspi_device.insn_program_resume = STM32WB_SFSPI_INSN_EPR;
	}
	
	if (stm32wb_sfspi_device.info.capacity > 0x01000000)
	{
	    stm32wb_sfspi_device.address = 0xff;

	    stm32wb_sfspi_address(0x00000000);
	}

	stm32wb_sfspi_release();

	stm32wb_sfspi_disable();
    
	stm32wb_system_notify(&stm32wb_sfspi_device.notify, (stm32wb_system_callback_t)stm32wb_sfspi_sleep, NULL, STM32WB_SYSTEM_EVENT_STOP_PREPARE);
	
	__stm32wb_sflash_initialize(&stm32wb_sfspi_interface, &stm32wb_sfspi_device.info);
    }
    else
    {
        stm32wb_sfspi_release();

	stm32wb_spi_disable(spi);

	stm32wb_gpio_pin_configure(stm32wb_sfspi_device.pins.cs, STM32WB_GPIO_MODE_ANALOG);

	stm32wb_sfspi_device.state = STM32WB_SFSPI_STATE_NONE;
    }

    return success;
}
