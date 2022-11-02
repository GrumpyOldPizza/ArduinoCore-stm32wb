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

#include "armv7m.h"
#include "stm32wb_gpio.h"
#include "stm32wb_sfsqi.h"
#include "stm32wb_sflash.h"
#include "stm32wb_system.h"

#define STM32WB_SFSQI_CCR_DDR                    (QUADSPI_CCR_DDRM)
#define STM32WB_SFSQI_CCR_WRITE                  (0)
#define STM32WB_SFSQI_CCR_READ                   (QUADSPI_CCR_FMODE_0)
#define STM32WB_SFSQI_CCR_WAIT                   (QUADSPI_CCR_FMODE_1)
#define STM32WB_SFSQI_CCR_DATA_SINGLE            (QUADSPI_CCR_DMODE_0)
#define STM32WB_SFSQI_CCR_DATA_DUAL              (QUADSPI_CCR_DMODE_1)
#define STM32WB_SFSQI_CCR_DATA_QUAD              (QUADSPI_CCR_DMODE_0 | QUADSPI_CCR_DMODE_1)
#define STM32WB_SFSQI_CCR_MODE_SINGLE            (QUADSPI_CCR_ABMODE_0)
#define STM32WB_SFSQI_CCR_MODE_DUAL              (QUADSPI_CCR_ABMODE_1)
#define STM32WB_SFSQI_CCR_MODE_QUAD              (QUADSPI_CCR_ABMODE_0 | QUADSPI_CCR_ABMODE_1)
#define STM32WB_SFSQI_CCR_MODE_SINGLE_8          (QUADSPI_CCR_ABMODE_0)
#define STM32WB_SFSQI_CCR_MODE_SINGLE_16         (QUADSPI_CCR_ABMODE_0 | QUADSPI_CCR_ABSIZE_0)
#define STM32WB_SFSQI_CCR_MODE_SINGLE_24         (QUADSPI_CCR_ABMODE_0 | QUADSPI_CCR_ABSIZE_1)
#define STM32WB_SFSQI_CCR_MODE_SINGLE_32         (QUADSPI_CCR_ABMODE_0 | QUADSPI_CCR_ABSIZE_0 | QUADSPI_CCR_ABSIZE_1)
#define STM32WB_SFSQI_CCR_MODE_DUAL_8            (QUADSPI_CCR_ABMODE_1)
#define STM32WB_SFSQI_CCR_MODE_DUAL_16           (QUADSPI_CCR_ABMODE_1 | QUADSPI_CCR_ABSIZE_0)
#define STM32WB_SFSQI_CCR_MODE_DUAL_24           (QUADSPI_CCR_ABMODE_1 | QUADSPI_CCR_ABSIZE_1)
#define STM32WB_SFSQI_CCR_MODE_DUAL_32           (QUADSPI_CCR_ABMODE_1 | QUADSPI_CCR_ABSIZE_0 | QUADSPI_CCR_ABSIZE_1)
#define STM32WB_SFSQI_CCR_MODE_QUAD_8            (QUADSPI_CCR_ABMODE_0 | QUADSPI_CCR_ABMODE_1)
#define STM32WB_SFSQI_CCR_MODE_QUAD_16           (QUADSPI_CCR_ABMODE_0 | QUADSPI_CCR_ABMODE_1 | QUADSPI_CCR_ABSIZE_0)
#define STM32WB_SFSQI_CCR_MODE_QUAD_24           (QUADSPI_CCR_ABMODE_0 | QUADSPI_CCR_ABMODE_1 | QUADSPI_CCR_ABSIZE_1)
#define STM32WB_SFSQI_CCR_MODE_QUAD_32           (QUADSPI_CCR_ABMODE_0 | QUADSPI_CCR_ABMODE_1 | QUADSPI_CCR_ABSIZE_0 | QUADSPI_CCR_ABSIZE_1)
#define STM32WB_SFSQI_CCR_DCYCLE_2               (2 << QUADSPI_CCR_DCYC_Pos)
#define STM32WB_SFSQI_CCR_DCYCLE_3               (3 << QUADSPI_CCR_DCYC_Pos)
#define STM32WB_SFSQI_CCR_DCYCLE_4               (4 << QUADSPI_CCR_DCYC_Pos)
#define STM32WB_SFSQI_CCR_DCYCLE_5               (5 << QUADSPI_CCR_DCYC_Pos)
#define STM32WB_SFSQI_CCR_DCYCLE_6               (6 << QUADSPI_CCR_DCYC_Pos)
#define STM32WB_SFSQI_CCR_DCYCLE_7               (7 << QUADSPI_CCR_DCYC_Pos)
#define STM32WB_SFSQI_CCR_DCYCLE_8               (8 << QUADSPI_CCR_DCYC_Pos)
#define STM32WB_SFSQI_CCR_DCYCLE_10              (10 << QUADSPI_CCR_DCYC_Pos)
#define STM32WB_SFSQI_CCR_ADDR_SINGLE_8          (QUADSPI_CCR_ADMODE_0)
#define STM32WB_SFSQI_CCR_ADDR_SINGLE_16         (QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_ADSIZE_0)
#define STM32WB_SFSQI_CCR_ADDR_SINGLE_24         (QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_ADSIZE_1)
#define STM32WB_SFSQI_CCR_ADDR_SINGLE_32         (QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_ADSIZE_0 | QUADSPI_CCR_ADSIZE_1)
#define STM32WB_SFSQI_CCR_ADDR_DUAL_8            (QUADSPI_CCR_ADMODE_1)
#define STM32WB_SFSQI_CCR_ADDR_DUAL_16           (QUADSPI_CCR_ADMODE_1 | QUADSPI_CCR_ADSIZE_0)
#define STM32WB_SFSQI_CCR_ADDR_DUAL_24           (QUADSPI_CCR_ADMODE_1 | QUADSPI_CCR_ADSIZE_1)
#define STM32WB_SFSQI_CCR_ADDR_DUAL_32           (QUADSPI_CCR_ADMODE_1 | QUADSPI_CCR_ADSIZE_0 | QUADSPI_CCR_ADSIZE_1)
#define STM32WB_SFSQI_CCR_ADDR_QUAD_8            (QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_ADMODE_1)
#define STM32WB_SFSQI_CCR_ADDR_QUAD_16           (QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_ADMODE_1 | QUADSPI_CCR_ADSIZE_0)
#define STM32WB_SFSQI_CCR_ADDR_QUAD_24           (QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_ADMODE_1 | QUADSPI_CCR_ADSIZE_1)
#define STM32WB_SFSQI_CCR_ADDR_QUAD_32           (QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_ADMODE_1 | QUADSPI_CCR_ADSIZE_0 | QUADSPI_CCR_ADSIZE_1)
#define STM32WB_SFSQI_CCR_INSN_SINGLE            (QUADSPI_CCR_IMODE_0)
#define STM32WB_SFSQI_CCR_INSN_DUAL              (QUADSPI_CCR_IMODE_1)
#define STM32WB_SFSQI_CCR_INSN_QUAD              (QUADSPI_CCR_IMODE_0 | QUADSPI_CCR_IMODE_1)

#define STM32WB_SFSQI_INSN_WRSR                  0x01
#define STM32WB_SFSQI_INSN_RDSR                  0x05
#define STM32WB_SFSQI_INSN_WREN                  0x06
#define STM32WB_SFSQI_INSN_RES                   0x90
#define STM32WB_SFSQI_INSN_RDID                  0x9f

#define STM32WB_SFSQI_SR_WIP                     0x01 /* write in progress */
#define STM32WB_SFSQI_SR_WEL                     0x02 /* write enable latch */

#define STM32WB_SFSQI_CCR_SECTOR_ERASE           (0x20 | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_SINGLE_24)
#define STM32WB_SFSQI_CCR_BLOCK_ERASE            (0xd8 | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_SINGLE_24)

#define STM32WB_SFSQI_CCR_1_1_1_PAGE_PROGRAM     (0x02 | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_SINGLE_24 | STM32WB_SFSQI_CCR_DATA_SINGLE)
#define STM32WB_SFSQI_CCR_1_1_2_PAGE_PROGRAM     (0xa2 | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_SINGLE_24 | STM32WB_SFSQI_CCR_DATA_DUAL)
#define STM32WB_SFSQI_CCR_1_2_2_PAGE_PROGRAM     (0xd2 | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_DUAL_24 | STM32WB_SFSQI_CCR_DATA_DUAL)
#define STM32WB_SFSQI_CCR_1_1_4_PAGE_PROGRAM     (0x32 | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_SINGLE_24 | STM32WB_SFSQI_CCR_DATA_QUAD)
#define STM32WB_SFSQI_CCR_1_4_4_PAGE_PROGRAM     (0x38 | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_QUAD_24 | STM32WB_SFSQI_CCR_DATA_QUAD)

#define STM32WB_SFSQI_CCR_1_1_1_READ             (0x03 | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_SINGLE_24 | STM32WB_SFSQI_CCR_DATA_SINGLE | STM32WB_SFSQI_CCR_READ)
#define STM32WB_SFSQI_CCR_1_1_1_FAST_READ        (0x0b | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_SINGLE_24 | STM32WB_SFSQI_CCR_DCYCLE_8 | STM32WB_SFSQI_CCR_DATA_SINGLE | STM32WB_SFSQI_CCR_READ)
#define STM32WB_SFSQI_CCR_1_1_2_D8_READ          (0x3b | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_SINGLE_24 | STM32WB_SFSQI_CCR_DCYCLE_8 | STM32WB_SFSQI_CCR_DATA_DUAL | STM32WB_SFSQI_CCR_READ)
#define STM32WB_SFSQI_CCR_1_1_4_D8_READ          (0x6b | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_SINGLE_24 | STM32WB_SFSQI_CCR_DCYCLE_8 | STM32WB_SFSQI_CCR_DATA_QUAD | STM32WB_SFSQI_CCR_READ)
#define STM32WB_SFSQI_CCR_1_2_2_M2D2_READ        (0xbb | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_DUAL_24 | STM32WB_SFSQI_CCR_MODE_QUAD_8 | STM32WB_SFSQI_CCR_DCYCLE_2 | STM32WB_SFSQI_CCR_DATA_DUAL | STM32WB_SFSQI_CCR_READ)
#define STM32WB_SFSQI_CCR_1_4_4_M2D4_READ        (0xeb | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_QUAD_24 | STM32WB_SFSQI_CCR_MODE_QUAD_8 | STM32WB_SFSQI_CCR_DCYCLE_4 | STM32WB_SFSQI_CCR_DATA_QUAD | STM32WB_SFSQI_CCR_READ)

#define STM32WB_SFSQI_MID_SPANSION               0x01
#define STM32WB_SFSQI_MID_MACRONIX               0xc2
#define STM32WB_SFSQI_MID_WINBOND                0xef

/* MACRONIX */
#define STM32WB_SFSQI_INSN_RDCONF                0x15
#define STM32WB_SFSQI_INSN_WRSCUR                0x2f
#define STM32WB_SFSQI_INSN_RDSCUR                0x2b
#define STM32WB_SFSQI_INSN_WREAR                 0xc5
#define STM32WB_SFSQI_INSN_PERSUS                0xb0
#define STM32WB_SFSQI_INSN_PERRSM                0x30
#define STM32WB_SFSQI_INSN_RSTEN                 0x66
#define STM32WB_SFSQI_INSN_RST                   0x99
#define STM32WB_SFSQI_INSN_RDPD                  0xab
#define STM32WB_SFSQI_INSN_DPD                   0xb9
#define STM32WB_SFSQI_INSN_EN4B                  0xb7
#define STM32WB_SFSQI_INSN_EX4B                  0xe9

#define STM32WB_SFSQI_SR_QE                      0x40
#define STM32WB_SFSQI_CONF2_HPM                  0x02
#define STM32WB_SFSQI_SCUR_PSB                   0x08
#define STM32WB_SFSQI_SCUR_ESB                   0x10
#define STM32WB_SFSQI_SCUR_P_FAIL                0x20
#define STM32WB_SFSQI_SCUR_E_FAIL                0x40

/* SPANSION */
#define STM32WB_SFSQI_INSN_RDSR2                 0x07
#define STM32WB_SFSQI_INSN_RDCR                  0x35
#define STM32WB_SFSQI_INSN_CLSR                  0x30
#define STM32WB_SFSQI_INSN_BRWR                  0x17
#define STM32WB_SFSQI_INSN_ERSP                  0x75
#define STM32WB_SFSQI_INSN_ERRS                  0x7a
#define STM32WB_SFSQI_INSN_PGSP                  0x85
#define STM32WB_SFSQI_INSN_PGRS                  0x8a
#define STM32WB_SFSQI_INSN_RESET                 0xf0

#define STM32WB_SFSQI_SR_P_ERR                   0x40
#define STM32WB_SFSQI_SR_E_ERR                   0x20
#define STM32WB_SFSQI_CR_QE                      0x02
#define STM32WB_SFSQI_SR2_PS                     0x01
#define STM32WB_SFSQI_SR2_ES                     0x02

#define STM32WB_SFSQI_BR_EXTADD                  0x80

/* WINBOND */
#define STM32WB_SFSQI_INSN_WREN_VOLATILE         0x50
#define STM32WB_SFSQI_INSN_WREAR                 0xc5
#define STM32WB_SFSQI_INSN_EPS                   0x75
#define STM32WB_SFSQI_INSN_EPR                   0x7a
#define STM32WB_SFSQI_INSN_RSTEN                 0x66
#define STM32WB_SFSQI_INSN_RST                   0x99
#define STM32WB_SFSQI_INSN_RDPD                  0xab
#define STM32WB_SFSQI_INSN_DPD                   0xb9
#define STM32WB_SFSQI_INSN_EN4B                  0xb7
#define STM32WB_SFSQI_INSN_EX4B                  0xe9

#define STM32WB_SFSQI_SR2_QE                     0x02
#define STM32WB_SFSQI_SR2_SUS                    0x80


#define STM32WB_SFSQI_STATE_NONE                 0
#define STM32WB_SFSQI_STATE_NOT_READY            1
#define STM32WB_SFSQI_STATE_READY                2
#define STM32WB_SFSQI_STATE_DATA                 3
#define STM32WB_SFSQI_STATE_WAIT                 4

#define STM32WB_SFSQI_BUSY_ERASE                 0x01
#define STM32WB_SFSQI_BUSY_PROGRAM               0x02
#define STM32WB_SFSQI_BUSY_SLEEP                 0x04
#define STM32WB_SFSQI_BUSY_SUSPENDED_ERASE       0x08
#define STM32WB_SFSQI_BUSY_SUSPENDED_PROGRAM     0x10


typedef struct _stm32wb_sfsqi_device_t {
    volatile uint8_t               state;
    uint8_t                        busy;
    stm32wb_sfsqi_pins_t           pins;
    stm32wb_sflash_info_t          info;
    stm32wb_system_notify_t        notify;
    QUADSPI_TypeDef                *QSPI;
    uint32_t                       clock;
    uint32_t                       hclk;
    uint32_t                       sclk;
    uint32_t                       cr;
    uint32_t                       ccr_erase;
    uint32_t                       ccr_program;
    uint32_t                       ccr_read;
    uint8_t                        insn_erase_suspend;
    uint8_t                        insn_erase_resume;
    uint8_t                        insn_program_suspend;
    uint8_t                        insn_program_resume;
    volatile uint8_t               *p_status_return;
} stm32wb_sfsqi_device_t;

static stm32wb_sfsqi_device_t stm32wb_sfsqi_device;

static void stm32wb_sfsqi_status(uint8_t status);
static bool stm32wb_sfsqi_busy(void);

static inline __attribute__((optimize("O3"),always_inline)) uint8_t STM32WB_SFSQI_READ_8(QUADSPI_TypeDef *QSPI)
{
    volatile uint8_t *fifo = (volatile uint8_t*)((uint32_t)&QSPI->DR);

    return *fifo;
}

static inline __attribute__((optimize("O3"),always_inline)) void STM32WB_SFSQI_WRITE_8(QUADSPI_TypeDef *QSPI, const uint8_t data)
{
    volatile uint8_t *fifo = (volatile uint8_t*)((uint32_t)&QSPI->DR);

    *fifo = data;
}

static inline __attribute__((optimize("O3"),always_inline)) uint16_t STM32WB_SFSQI_READ_16(QUADSPI_TypeDef *QSPI)
{
    volatile uint16_t *fifo = (volatile uint16_t*)((uint32_t)&QSPI->DR);

    return *fifo;
}

static inline __attribute__((optimize("O3"),always_inline)) void STM32WB_SFSQI_WRITE_16(QUADSPI_TypeDef *QSPI, const uint16_t data)
{
    volatile uint16_t *fifo = (volatile uint16_t*)((uint32_t)&QSPI->DR);

    *fifo = data;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t STM32WB_SFSQI_READ_32(QUADSPI_TypeDef *QSPI)
{
    volatile uint32_t *fifo = (volatile uint32_t*)((uint32_t)&QSPI->DR);

    return *fifo;
}

static inline __attribute__((optimize("O3"),always_inline)) void STM32WB_SFSQI_WRITE_32(QUADSPI_TypeDef *QSPI, const uint32_t data)
{
    volatile uint32_t *fifo = (volatile uint32_t*)((uint32_t)&QSPI->DR);

    *fifo = data;
}

static inline void stm32wb_sfsqi_start(void)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;
    uint32_t hclk, presc;
    
    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);

    hclk = stm32wb_system_hclk();

    if (stm32wb_sfsqi_device.hclk != hclk)
    {
	stm32wb_sfsqi_device.hclk = hclk;

	if (stm32wb_sfsqi_device.clock >= hclk)
	{
	    presc = 1;
	}
	else
	{
	    presc = (hclk + (stm32wb_sfsqi_device.clock -1)) / stm32wb_sfsqi_device.clock;
	}
	
	stm32wb_sfsqi_device.sclk = hclk / presc;
	stm32wb_sfsqi_device.cr = ((presc -1) << QUADSPI_CR_PRESCALER_Pos) | ((8 -1) << QUADSPI_CR_FTHRES_Pos) | QUADSPI_CR_APMS | QUADSPI_CR_SSHIFT | QUADSPI_CR_SMIE | QUADSPI_CR_EN;
    }
    
    stm32wb_system_periph_enable(STM32WB_SYSTEM_PERIPH_QSPI);
    
    QSPI->DCR = QUADSPI_DCR_FSIZE | ((2 -1) << QUADSPI_DCR_CSHT_Pos);
    QSPI->ABR = 0x88;
    QSPI->CR = stm32wb_sfsqi_device.cr;
}

static inline void stm32wb_sfsqi_stop(void)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;

    while (QSPI->SR & QUADSPI_SR_BUSY) { }
    
    QSPI->CR = 0;
    
    stm32wb_system_periph_disable(STM32WB_SYSTEM_PERIPH_QSPI);

    stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfsqi_command(uint8_t insn)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;

    while (QSPI->SR & QUADSPI_SR_BUSY) { }

    QSPI->CCR = insn | STM32WB_SFSQI_CCR_INSN_SINGLE;
}

static inline __attribute__((optimize("O3"),always_inline)) uint8_t stm32wb_sfsqi_command_read_1(uint8_t insn)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;
    uint8_t data_0;

    while (QSPI->SR & QUADSPI_SR_BUSY) { }

    QSPI->DLR = 0;
    QSPI->CCR = insn | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_DATA_SINGLE | STM32WB_SFSQI_CCR_READ;

    while (!(QSPI->SR & QUADSPI_SR_TCF)) { }

    data_0 = STM32WB_SFSQI_READ_8(QSPI);

    QSPI->FCR = QUADSPI_FCR_CTCF;

    return data_0;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfsqi_command_read_2(uint8_t insn, uint8_t *p_data_0, uint8_t *p_data_1)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;

    while (QSPI->SR & QUADSPI_SR_BUSY) { }

    QSPI->DLR = 1;
    QSPI->CCR = insn | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_DATA_SINGLE | STM32WB_SFSQI_CCR_READ;

    while (!(QSPI->SR & QUADSPI_SR_TCF)) { }

    *p_data_0 = STM32WB_SFSQI_READ_8(QSPI);
    *p_data_1 = STM32WB_SFSQI_READ_8(QSPI);

    QSPI->FCR = QUADSPI_FCR_CTCF;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfsqi_command_read_3(uint8_t insn, uint8_t *p_data_0, uint8_t *p_data_1, uint8_t *p_data_2)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;

    while (QSPI->SR & QUADSPI_SR_BUSY) { }

    QSPI->DLR = 2;
    QSPI->CCR = insn | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_DATA_SINGLE | STM32WB_SFSQI_CCR_READ;

    while (!(QSPI->SR & QUADSPI_SR_TCF)) { }

    *p_data_0 = STM32WB_SFSQI_READ_8(QSPI);
    *p_data_1 = STM32WB_SFSQI_READ_8(QSPI);
    *p_data_2 = STM32WB_SFSQI_READ_8(QSPI);

    QSPI->FCR = QUADSPI_FCR_CTCF;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfsqi_command_write_1(uint8_t insn, uint8_t data_0)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;

    while (QSPI->SR & QUADSPI_SR_BUSY) { }

    QSPI->DLR = 0;
    QSPI->CCR = insn | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_DATA_SINGLE;

    STM32WB_SFSQI_WRITE_8(QSPI, data_0);

    while (!(QSPI->SR & QUADSPI_SR_TCF)) { }

    QSPI->FCR = QUADSPI_FCR_CTCF;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfsqi_command_write_2(uint8_t insn, uint8_t data_0, uint8_t data_1)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;

    while (QSPI->SR & QUADSPI_SR_BUSY) { }

    QSPI->DLR = 1;
    QSPI->CCR = insn | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_DATA_SINGLE;

    STM32WB_SFSQI_WRITE_8(QSPI, data_0);
    STM32WB_SFSQI_WRITE_8(QSPI, data_1);

    while (!(QSPI->SR & QUADSPI_SR_TCF)) { }

    QSPI->FCR = QUADSPI_FCR_CTCF;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfsqi_command_write_3(uint8_t insn, uint8_t data_0, uint8_t data_1, uint8_t data_2)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;

    while (QSPI->SR & QUADSPI_SR_BUSY) { }

    QSPI->DLR = 2;
    QSPI->CCR = insn | STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_DATA_SINGLE;

    STM32WB_SFSQI_WRITE_8(QSPI, data_0);
    STM32WB_SFSQI_WRITE_8(QSPI, data_1);
    STM32WB_SFSQI_WRITE_8(QSPI, data_2);

    while (!(QSPI->SR & QUADSPI_SR_TCF)) { }

    QSPI->FCR = QUADSPI_FCR_CTCF;
}

static inline __attribute__((optimize("O3"),always_inline)) uint8_t stm32wb_sfsqi_command_signature(void)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;
    uint8_t signature;
    
    while (QSPI->SR & QUADSPI_SR_BUSY) { }

    QSPI->DLR = 0;
    QSPI->CCR = STM32WB_SFSQI_INSN_RES | (STM32WB_SFSQI_CCR_INSN_SINGLE | STM32WB_SFSQI_CCR_ADDR_SINGLE_24 | STM32WB_SFSQI_CCR_DATA_SINGLE | STM32WB_SFSQI_CCR_READ);
    QSPI->AR  = 0;    

    while (!(QSPI->SR & QUADSPI_SR_TCF)) { }

    signature = STM32WB_SFSQI_READ_8(QSPI);

    QSPI->FCR = QUADSPI_FCR_CTCF;

    return signature;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfsqi_command_erase(uint32_t address)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;

    while (QSPI->SR & QUADSPI_SR_BUSY) { }

    QSPI->CCR = stm32wb_sfsqi_device.ccr_erase;
    QSPI->AR  = address;    
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfsqi_command_program(uint32_t address, const uint8_t *data, uint32_t count)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;
    uint32_t data_0, data_1, data_2, data_3;
    const uint32_t *data32, *data32_e;
    const uint16_t *data16;
    const uint8_t *data8, *data_e;
    
    while (QSPI->SR & QUADSPI_SR_BUSY) { }

    QSPI->DLR = count -1;
    QSPI->CCR = stm32wb_sfsqi_device.ccr_program;
    QSPI->AR  = address;    

    if (count > 16)
    {
	if ((uint32_t)data & 3)
	{
	    data_e = data + count;

	    if ((uint32_t)data & 1)
	    {
		data8 = (const uint8_t*)data;
		
		STM32WB_SFSQI_WRITE_8(QSPI, *data8++);
		
		data = (const uint8_t*)data8;
	    }
	    
	    if ((uint32_t)data & 2)
	    {
		data16 = (const uint16_t*)data;
		
		STM32WB_SFSQI_WRITE_16(QSPI, *data16++);
		
		data = (const uint8_t*)data16;
	    }
	    
	    count = data_e - data;
	}
	
        data32 = (const uint32_t*)(data);
	data32_e = (const uint32_t*)(data + (count & ~7));

	do
	{
	    data_0 = data32[0];
	    data_1 = data32[1];

	    __asm__ volatile("": : : "memory");

	    while (!(QSPI->SR & QUADSPI_SR_FTF)) { }
	    
	    STM32WB_SFSQI_WRITE_32(QSPI, data_0);
	    STM32WB_SFSQI_WRITE_32(QSPI, data_1);

	    __asm__ volatile("": : : "memory");

	    data32 += 2;
	}
	while (data32 != data32_e);

	if (count & 7)
	{
	    data = (const uint8_t*)data32;

	    while (!(QSPI->SR & QUADSPI_SR_FTF)) { }

	    if (count & 4)
	    {
		data32 = (const uint32_t*)data;

		STM32WB_SFSQI_WRITE_32(QSPI, *data32++);
	    
		data = (const uint8_t*)data32;
	    }

	    if (count & 2)
	    {
		data16 = (const uint16_t*)data;
		
		STM32WB_SFSQI_WRITE_16(QSPI, *data16++);
		
		data = (const uint8_t*)data16;
	    }
	    
	    if (count & 1)
	    {
		data8 = (const uint8_t*)data;

		STM32WB_SFSQI_WRITE_8(QSPI, *data8);
	    }
	}
    }
    else
    {
	if (count > 3)
	{
	    if ((uint32_t)data & 3)
	    {
		data_e = data + count;

		if ((uint32_t)data & 1)
		{
		    data8 = (const uint8_t*)data;
		    
		    STM32WB_SFSQI_WRITE_8(QSPI, *data8++);
		    
		    data = (const uint8_t*)data8;
		}
		
		if ((uint32_t)data & 2)
		{
		    data16 = (const uint16_t*)data;
		    
		    STM32WB_SFSQI_WRITE_16(QSPI, *data16++);
		    
		    data = (const uint8_t*)data16;
		}
		
		count = data_e - data;
	    }
	    
	    if (count & 16)
	    {
		data32 = (const uint32_t*)data;

		data_0 = data32[0];
		data_1 = data32[1];
		data_2 = data32[2];
		data_3 = data32[3];
		
		__asm__ volatile("": : : "memory");
	    
		STM32WB_SFSQI_WRITE_32(QSPI, data_0);
		STM32WB_SFSQI_WRITE_32(QSPI, data_1);
		STM32WB_SFSQI_WRITE_32(QSPI, data_2);
		STM32WB_SFSQI_WRITE_32(QSPI, data_3);
		
		__asm__ volatile("": : : "memory");
	    }
	    else
	    {
		if (count & 8)
		{
		    data32 = (const uint32_t*)data;

		    data_0 = data32[0];
		    data_1 = data32[1];
		    
		    __asm__ volatile("": : : "memory");
	    
		    STM32WB_SFSQI_WRITE_32(QSPI, data_0);
		    STM32WB_SFSQI_WRITE_32(QSPI, data_1);
		    
		    __asm__ volatile("": : : "memory");
		    
		    data32 += 2;
		    
		    data = (const uint8_t*)data32;
		}
		
		if (count & 4)
		{
		    data32 = (const uint32_t*)data;
		    
		    STM32WB_SFSQI_WRITE_32(QSPI, *data32++);
		    
		    data = (const uint8_t*)data32;
		}
		
		if (count & 2)
		{
		    data16 = (const uint16_t*)data;
	    
		    STM32WB_SFSQI_WRITE_16(QSPI, *data16++);
		    
		    data = (const uint8_t*)data16;
		}
		
		if (count & 1)
		{
		    data8 = (const uint8_t*)data;
		    
		    STM32WB_SFSQI_WRITE_8(QSPI, *data8);
		}
	    }
	}
	else
	{
	    if (count >= 3)
	    {
		STM32WB_SFSQI_WRITE_8(QSPI, *data++);
	    }

	    if (count >= 2)
	    {
		STM32WB_SFSQI_WRITE_8(QSPI, *data++);
	    }

	    STM32WB_SFSQI_WRITE_8(QSPI, *data);
	}
    }
    
    while (!(QSPI->SR & QUADSPI_SR_TCF)) { }
    
    QSPI->FCR = QUADSPI_FCR_CTCF;
}

static inline __attribute__((optimize("O3"),always_inline)) void stm32wb_sfsqi_command_read(uint32_t address, uint8_t *data, uint32_t count)
{
    QUADSPI_TypeDef *QSPI = stm32wb_sfsqi_device.QSPI;
    uint32_t data_0, data_1, data_2, data_3;
    uint32_t *data32, *data32_e;
    uint16_t *data16;
    uint8_t *data8, *data_e;

    while (QSPI->SR & QUADSPI_SR_BUSY) { }

    QSPI->DLR = count -1;
    QSPI->CCR = stm32wb_sfsqi_device.ccr_read;
    QSPI->AR  = address;
    
    if (count > 16)
    {
	if ((uint32_t)data & 3)
	{
	    data_e = data + count;

	    while (!(QSPI->SR & QUADSPI_SR_FTF)) { }

	    if ((uint32_t)data & 1)
	    {
		data8 = (uint8_t*)data;
		
		*data8++ = STM32WB_SFSQI_READ_8(QSPI);
		
		data = (uint8_t*)data8;
	    }
	    
	    if ((uint32_t)data & 2)
	    {
		data16 = (uint16_t*)data;
		
		*data16++ = STM32WB_SFSQI_READ_16(QSPI);
		
		data = (uint8_t*)data16;
	    }
	    
	    count = data_e - data;
	}
	
	data32 = (uint32_t*)(data);
	data32_e = (uint32_t*)(data + (count & ~7));

	do
	{
	    __asm__ volatile("": : : "memory");

	    while (!(QSPI->SR & QUADSPI_SR_FTF)) { }
		
	    data_0 = STM32WB_SFSQI_READ_32(QSPI);
	    data_1 = STM32WB_SFSQI_READ_32(QSPI);

	    __asm__ volatile("": : : "memory");
	    
	    data32[0] = data_0;
	    data32[1] = data_1;

	    data32 += 2;
	}
	while (data32 != data32_e);

	while (!(QSPI->SR & QUADSPI_SR_TCF)) { }
	
	if (count & 7)
	{
	    data = (void*)data32;

	    if (count & 4)
	    {
		data32 = (uint32_t*)data;

		*data32++ = STM32WB_SFSQI_READ_32(QSPI);
		
		data = (uint8_t*)data32;
	    }

	    if (count & 2)
	    {
		data16 = (uint16_t*)data;

		*data16++ = STM32WB_SFSQI_READ_16(QSPI);
		
		data = (uint8_t*)data16;
	    }

	    if (count & 1)
	    {
		data8 = (uint8_t*)data;

		*data8 = STM32WB_SFSQI_READ_8(QSPI);
	    }
	}
    }
    else
    {
	while (!(QSPI->SR & QUADSPI_SR_TCF)) { }

	if (count > 3)
	{
	    if ((uint32_t)data & 3)
	    {
		data_e = data + count;

		if ((uint32_t)data & 1)
		{
		    data8 = (uint8_t*)data;
		    
		    *data8++ = STM32WB_SFSQI_READ_8(QSPI);
		    
		    data = (uint8_t*)data8;
		}
		
		if ((uint32_t)data & 2)
		{
		    data16 = (uint16_t*)data;
		    
		    *data16++ = STM32WB_SFSQI_READ_16(QSPI);
		    
		    data = (uint8_t*)data16;
		}
		
		count = data_e - data;
	    }
	    
	    if (count & 16)
	    {
		data32 = (uint32_t*)data;
		    
		__asm__ volatile("": : : "memory");
		    
		data_0 = STM32WB_SFSQI_READ_32(QSPI);
		data_1 = STM32WB_SFSQI_READ_32(QSPI);
		data_2 = STM32WB_SFSQI_READ_32(QSPI);
		data_3 = STM32WB_SFSQI_READ_32(QSPI);
		    
		__asm__ volatile("": : : "memory");
		    
		data32[0] = data_0;
		data32[1] = data_1;
		data32[2] = data_2;
		data32[3] = data_3;
	    }
	    else
	    {
		if (count & 8)
		{
		    data32 = (uint32_t*)data;
	    
		    __asm__ volatile("": : : "memory");
			
		    data_0 = STM32WB_SFSQI_READ_32(QSPI);
		    data_1 = STM32WB_SFSQI_READ_32(QSPI);
			
		    __asm__ volatile("": : : "memory");
			
		    data32[0] = data_0;
		    data32[1] = data_1;
			
		    data32 += 2;
			
		    data = (uint8_t*)data32;
		}
		    
		if (count & 4)
		{
		    data32 = (uint32_t*)data;
	    
		    *data32++ = STM32WB_SFSQI_READ_32(QSPI);
			
		    data = (uint8_t*)data32;
		}
	
		if (count & 2)
		{
		    data16 = (uint16_t*)data;
	    
		    *data16++ = STM32WB_SFSQI_READ_16(QSPI);
			
		    data = (uint8_t*)data16;
		}
		    
		if (count & 1)
		{
		    data8 = (uint8_t*)data;
			
		    *data8 = STM32WB_SFSQI_READ_8(QSPI);
		}
	    }
	}
	else
	{
	    if (count > 2)
	    {
		*data++ = STM32WB_SFSQI_READ_8(QSPI);
	    }

	    if (count >= 2)
	    {
		*data++ = STM32WB_SFSQI_READ_8(QSPI);
	    }
	    
	    *data++ = STM32WB_SFSQI_READ_8(QSPI);
	}
    }
    
    QSPI->FCR = QUADSPI_FCR_CTCF;
}

static __attribute__((optimize("O3"),noinline)) uint8_t stm32wb_sfsqi_write_enable(void)
{
    uint8_t status;
    
    do
    {
	stm32wb_sfsqi_command(STM32WB_SFSQI_INSN_WREN);

	status = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDSR);
    }
    while (!(status & STM32WB_SFSQI_SR_WEL));

    return status;
}

static bool stm32wb_sfsqi_acquire(void)
{
    if (stm32wb_sfsqi_device.state < STM32WB_SFSQI_STATE_NOT_READY)
    {
	return false;
    }

    stm32wb_sfsqi_start();

    if (stm32wb_sfsqi_device.busy & STM32WB_SFSQI_BUSY_SLEEP)
    {
	stm32wb_sfsqi_command(STM32WB_SFSQI_INSN_RDPD);
	
        armv7m_core_udelay(50);

	stm32wb_sfsqi_device.busy &= ~STM32WB_SFSQI_BUSY_SLEEP;
    }
    
    if (stm32wb_sfsqi_device.state != STM32WB_SFSQI_STATE_NOT_READY)
    {
	stm32wb_sfsqi_device.state = STM32WB_SFSQI_STATE_DATA;
    }

    return true;
}

static bool stm32wb_sfsqi_release(void)
{
    if (stm32wb_sfsqi_device.state < STM32WB_SFSQI_STATE_NOT_READY)
    {
	return false;
    }

    stm32wb_sfsqi_stop();
    
    if (stm32wb_sfsqi_device.state != STM32WB_SFSQI_STATE_NOT_READY)
    {
	stm32wb_sfsqi_device.state = STM32WB_SFSQI_STATE_READY;
    }
    
    return true;
}

static void stm32wb_sfsqi_sleep(void)
{
    bool success;
    
    if (stm32wb_sfsqi_device.state != STM32WB_SFSQI_STATE_READY)
    {
	return;
    }

    if (stm32wb_sfsqi_device.busy & STM32WB_SFSQI_BUSY_SLEEP)
    {
	return;
    }

    if (stm32wb_sfsqi_device.info.mid == STM32WB_SFSQI_MID_SPANSION)
    {
	return;
    }

    if (!stm32wb_sfsqi_acquire())
    {
	return;
    }

    success = true;
    
    if (stm32wb_sfsqi_busy())
    {
	success = false;
    }

    if (success)
    {
	stm32wb_sfsqi_command(STM32WB_SFSQI_INSN_DPD);

	stm32wb_sfsqi_device.busy |= STM32WB_SFSQI_BUSY_SLEEP;
    }
    
    stm32wb_sfsqi_release();
}

static __attribute__((optimize("O3"),noinline)) void stm32wb_sfsqi_status(uint8_t status)
{
    uint8_t scur;
    bool success;
    
    success = true;

    if (stm32wb_sfsqi_device.info.mid == STM32WB_SFSQI_MID_MACRONIX)
    {
	scur = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDSCUR);

	if (stm32wb_sfsqi_device.busy & STM32WB_SFSQI_BUSY_ERASE)
	{
	    if (scur & STM32WB_SFSQI_SCUR_E_FAIL)
	    {
		success = false;
	    }
	}
	else
	{
	    if (scur & STM32WB_SFSQI_SCUR_P_FAIL)
	    {
		success = false;
	    }
	}
    }
    
    if (stm32wb_sfsqi_device.info.mid == STM32WB_SFSQI_MID_SPANSION)
    {
	if (status & (STM32WB_SFSQI_SR_E_ERR | STM32WB_SFSQI_SR_P_ERR))
	{
	    stm32wb_sfsqi_command(STM32WB_SFSQI_INSN_CLSR);
	    
	    success = false;
	}
    }
    
    if (stm32wb_sfsqi_device.p_status_return)
    {
	*stm32wb_sfsqi_device.p_status_return = success ? STM32WB_SFLASH_STATUS_SUCCESS : STM32WB_SFLASH_STATUS_FAILURE;
    }
    
    stm32wb_sfsqi_device.busy &= ~(STM32WB_SFSQI_BUSY_ERASE | STM32WB_SFSQI_BUSY_PROGRAM);
}

static __attribute__((optimize("O3"),noinline)) bool stm32wb_sfsqi_busy(void)
{
    uint8_t status;
    
    if (stm32wb_sfsqi_device.state != STM32WB_SFSQI_STATE_DATA)
    {
	return false;
    }

    if (!(stm32wb_sfsqi_device.busy & (STM32WB_SFSQI_BUSY_ERASE | STM32WB_SFSQI_BUSY_PROGRAM)))
    {
	return false;
    }

    status = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDSR);

    if (status & (STM32WB_SFSQI_SR_WEL | STM32WB_SFSQI_SR_WIP))
    {
	return true;
    }

    stm32wb_sfsqi_status(status);

    return false;
}

static bool stm32wb_sfsqi_suspend(void)
{
    uint8_t status, status2, scur;
    bool success;
    
    if (stm32wb_sfsqi_device.state != STM32WB_SFSQI_STATE_DATA)
    {
	return false;
    }

    if (stm32wb_sfsqi_device.busy & (STM32WB_SFSQI_BUSY_SUSPENDED_ERASE | STM32WB_SFSQI_BUSY_SUSPENDED_PROGRAM))
    {
	return false;
    }

    if (!stm32wb_sfsqi_busy())
    {
	return false;
    }
    
    stm32wb_sfsqi_command((stm32wb_sfsqi_device.busy & STM32WB_SFSQI_BUSY_ERASE) ? stm32wb_sfsqi_device.insn_erase_suspend : stm32wb_sfsqi_device.insn_program_suspend);

    do
    {
	status = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDSR);
    }
    while (status & (STM32WB_SFSQI_SR_WEL | STM32WB_SFSQI_SR_WIP));

    success = true;

    if (stm32wb_sfsqi_device.info.mid == STM32WB_SFSQI_MID_SPANSION)
    {
	status2 = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDSR2);
	
	if (!(status2 & (STM32WB_SFSQI_SR2_ES | STM32WB_SFSQI_SR2_PS)))
	{
	    success = false;
	}
    }
    
    if (stm32wb_sfsqi_device.info.mid == STM32WB_SFSQI_MID_MACRONIX)
    {
	scur = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDSCUR);
	
	if (!(scur & (STM32WB_SFSQI_SCUR_ESB | STM32WB_SFSQI_SCUR_PSB)))
	{
	    success = false;
	}
    }

    if (stm32wb_sfsqi_device.info.mid == STM32WB_SFSQI_MID_WINBOND)
    {
	stm32wb_sfsqi_command_read_2(STM32WB_SFSQI_INSN_RDSR, &status, &status2);

	if (!(status2 & STM32WB_SFSQI_SR2_SUS))
	{
	    success = false;
	}
    }
    
    if (success)
    {
	if (stm32wb_sfsqi_device.busy & STM32WB_SFSQI_BUSY_ERASE)
	{
	    stm32wb_sfsqi_device.busy = (stm32wb_sfsqi_device.busy & ~STM32WB_SFSQI_BUSY_ERASE) | STM32WB_SFSQI_BUSY_SUSPENDED_ERASE;
	}
	else
	{
	    stm32wb_sfsqi_device.busy = (stm32wb_sfsqi_device.busy & ~STM32WB_SFSQI_BUSY_PROGRAM) | STM32WB_SFSQI_BUSY_SUSPENDED_PROGRAM;
	}

	return true;
    }
    else
    {
	stm32wb_sfsqi_status(status);

	return false;
    }
}

static bool stm32wb_sfsqi_resume(void)
{
    if (stm32wb_sfsqi_device.state != STM32WB_SFSQI_STATE_DATA)
    {
	return false;
    }

    if (!(stm32wb_sfsqi_device.busy & (STM32WB_SFSQI_BUSY_SUSPENDED_ERASE | STM32WB_SFSQI_BUSY_SUSPENDED_PROGRAM)))
    {
	return false;
    }

    stm32wb_sfsqi_command((stm32wb_sfsqi_device.busy & STM32WB_SFSQI_BUSY_SUSPENDED_ERASE) ? stm32wb_sfsqi_device.insn_erase_resume : stm32wb_sfsqi_device.insn_program_resume);
    
    if (stm32wb_sfsqi_device.busy & STM32WB_SFSQI_BUSY_SUSPENDED_PROGRAM)
    {
	stm32wb_sfsqi_device.busy = (stm32wb_sfsqi_device.busy & ~STM32WB_SFSQI_BUSY_SUSPENDED_PROGRAM) | STM32WB_SFSQI_BUSY_PROGRAM;
    }
    else
    {
	stm32wb_sfsqi_device.busy = (stm32wb_sfsqi_device.busy & ~STM32WB_SFSQI_BUSY_SUSPENDED_ERASE) | STM32WB_SFSQI_BUSY_ERASE;
    }
    
    return true;
}

static bool stm32wb_sfsqi_erase(uint32_t address, volatile uint8_t *p_status_return)
{
    if (stm32wb_sfsqi_device.state != STM32WB_SFSQI_STATE_DATA)
    {
	return false;
    }

    if (stm32wb_sfsqi_device.busy & (STM32WB_SFSQI_BUSY_SUSPENDED_ERASE | STM32WB_SFSQI_BUSY_SUSPENDED_PROGRAM | STM32WB_SFSQI_BUSY_ERASE | STM32WB_SFSQI_BUSY_PROGRAM))
    {
	return false;
    }

    if (p_status_return)
    {
	*p_status_return = STM32WB_SFLASH_STATUS_BUSY;
    }
    
    stm32wb_sfsqi_device.p_status_return = p_status_return;
    stm32wb_sfsqi_device.busy = STM32WB_SFSQI_BUSY_ERASE;
    
    stm32wb_sfsqi_write_enable();

    stm32wb_sfsqi_command_erase(address);
    
    return true;
}

static bool stm32wb_sfsqi_program(uint32_t address, const uint8_t *data, uint32_t count, volatile uint8_t *p_status_return)
{
    if (stm32wb_sfsqi_device.state != STM32WB_SFSQI_STATE_DATA)
    {
	return false;
    }

    if (stm32wb_sfsqi_device.busy & (STM32WB_SFSQI_BUSY_SUSPENDED_ERASE | STM32WB_SFSQI_BUSY_SUSPENDED_PROGRAM | STM32WB_SFSQI_BUSY_ERASE | STM32WB_SFSQI_BUSY_PROGRAM))
    {
	return false;
    }
    
    if (p_status_return)
    {
	*p_status_return = STM32WB_SFLASH_STATUS_BUSY;
    }
    
    stm32wb_sfsqi_device.p_status_return = p_status_return;
    stm32wb_sfsqi_device.busy = STM32WB_SFSQI_BUSY_PROGRAM;

    stm32wb_sfsqi_write_enable();

    stm32wb_sfsqi_command_program(address, data, count);
    
    return true;
}

static bool stm32wb_sfsqi_read(uint32_t address, uint8_t *data, uint32_t count)
{
    if (stm32wb_sfsqi_device.state != STM32WB_SFSQI_STATE_DATA)
    {
	return false;
    }

    if (stm32wb_sfsqi_device.busy & (STM32WB_SFSQI_BUSY_ERASE | STM32WB_SFSQI_BUSY_PROGRAM))
    {
	return false;
    }
    
    stm32wb_sfsqi_command_read(address, data, count);
    
    return true;
}

static const stm32wb_sflash_interface_t stm32wb_sfsqi_interface = {
    stm32wb_sfsqi_acquire,
    stm32wb_sfsqi_release,
    stm32wb_sfsqi_busy,
    stm32wb_sfsqi_suspend,
    stm32wb_sfsqi_resume,
    stm32wb_sfsqi_erase,
    stm32wb_sfsqi_program,
    stm32wb_sfsqi_read,
};

bool stm32wb_sfsqi_initialize(const stm32wb_sfsqi_params_t *params)
{
    uint8_t MID, DID[2], status, status2, control, config1, config2;
    bool success, quad;
    
    if (stm32wb_sfsqi_device.state != STM32WB_SFSQI_STATE_NONE)
    {
	return false;
    }

    stm32wb_sfsqi_device.state = STM32WB_SFSQI_STATE_NOT_READY;
    stm32wb_sfsqi_device.pins = params->pins;

    stm32wb_sfsqi_device.QSPI = QUADSPI;

    stm32wb_sfsqi_device.clock = 8000000;
    stm32wb_sfsqi_device.hclk = 0;
    stm32wb_sfsqi_device.sclk = 0;
    
    stm32wb_gpio_pin_configure(stm32wb_sfsqi_device.pins.clk, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    stm32wb_gpio_pin_configure(stm32wb_sfsqi_device.pins.cs, (STM32WB_GPIO_PARK_PULLUP | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    stm32wb_gpio_pin_configure(stm32wb_sfsqi_device.pins.io0, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
    stm32wb_gpio_pin_configure(stm32wb_sfsqi_device.pins.io1, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));

    if ((stm32wb_sfsqi_device.pins.io2 != STM32WB_GPIO_PIN_NONE) && (stm32wb_sfsqi_device.pins.io3 != STM32WB_GPIO_PIN_NONE))
    {
	stm32wb_gpio_pin_configure(stm32wb_sfsqi_device.pins.io2, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
	stm32wb_gpio_pin_configure(stm32wb_sfsqi_device.pins.io3, (STM32WB_GPIO_PARK_HIZ | STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));

	quad = true;
    }
    else
    {
	quad = false;
    }

    stm32wb_sfsqi_acquire();

    stm32wb_sfsqi_command(STM32WB_SFSQI_INSN_RDPD);
    
    armv7m_core_udelay(200);

    stm32wb_sfsqi_device.info.mid = stm32wb_sfsqi_command_signature();

    success = false;

    if (stm32wb_sfsqi_device.info.mid == STM32WB_SFSQI_MID_SPANSION)
    {
	success = true;
    }
    
    if (stm32wb_sfsqi_device.info.mid == STM32WB_SFSQI_MID_MACRONIX)
    {
	success = true;
    }

    if (stm32wb_sfsqi_device.info.mid == STM32WB_SFSQI_MID_WINBOND)
    {
	success = true;
    }

    if (success)
    {
	status = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDSR);
	
	if (status & (STM32WB_SFSQI_SR_WEL | STM32WB_SFSQI_SR_WIP))
	{
	    if (stm32wb_sfsqi_device.info.mid == STM32WB_SFSQI_MID_SPANSION)
	    {
		stm32wb_sfsqi_command(STM32WB_SFSQI_INSN_RESET);
	    }
	    else
	    {
		stm32wb_sfsqi_command(STM32WB_SFSQI_INSN_RSTEN);
		stm32wb_sfsqi_command(STM32WB_SFSQI_INSN_RST);
	    }
	    
	    armv7m_core_udelay(100);
	    
	    do
	    {
		status = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDSR);
	    } 
	    while (status & (STM32WB_SFSQI_SR_WEL | STM32WB_SFSQI_SR_WIP));
	}

	stm32wb_sfsqi_command_read_3(STM32WB_SFSQI_INSN_RDID, &MID, &DID[0], &DID[1]);

	if ((stm32wb_sfsqi_device.info.mid != MID) || ((DID[0] == 0x00) && (DID[1] == 0x00)) || ((DID[0] == 0xff) && (DID[1] == 0xff)))
	{
	    success = false;
	}
    }

    if (success)
    {
        stm32wb_sfsqi_device.info.did = (DID[0] << 8) | (DID[1] << 0);
        stm32wb_sfsqi_device.info.capacity = (1 << DID[1]);
        stm32wb_sfsqi_device.info.block_size = 65536;
        stm32wb_sfsqi_device.info.page_size = 256;
	
        stm32wb_sfsqi_device.clock = 32000000;
	
	stm32wb_sfsqi_device.ccr_erase = STM32WB_SFSQI_CCR_BLOCK_ERASE;
	stm32wb_sfsqi_device.ccr_program = STM32WB_SFSQI_CCR_1_1_1_PAGE_PROGRAM;
	stm32wb_sfsqi_device.ccr_read = STM32WB_SFSQI_CCR_1_1_1_FAST_READ;
	
	if (MID == STM32WB_SFSQI_MID_SPANSION)
	{
	    stm32wb_sfsqi_device.insn_erase_suspend = STM32WB_SFSQI_INSN_ERSP;
	    stm32wb_sfsqi_device.insn_erase_resume = STM32WB_SFSQI_INSN_ERRS;
	    stm32wb_sfsqi_device.insn_program_suspend = STM32WB_SFSQI_INSN_PGSP;
	    stm32wb_sfsqi_device.insn_program_resume = STM32WB_SFSQI_INSN_PGRS;

	    if (quad)
	    {
	        stm32wb_sfsqi_device.ccr_program = STM32WB_SFSQI_CCR_1_1_4_PAGE_PROGRAM;
		stm32wb_sfsqi_device.ccr_read = STM32WB_SFSQI_CCR_1_4_4_M2D4_READ;

		control = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDCR);
		
		if (!(control & STM32WB_SFSQI_CR_QE))
		{
		    status = stm32wb_sfsqi_write_enable();
		    
		    stm32wb_sfsqi_command_write_2(STM32WB_SFSQI_INSN_WRSR, status, (control | STM32WB_SFSQI_CR_QE));
		    
		    do
		    {
			status = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDSR);
		    } 
		    while (status & (STM32WB_SFSQI_SR_WEL | STM32WB_SFSQI_SR_WIP));
		}
	    }
	    else
	    {
		stm32wb_sfsqi_device.ccr_read = STM32WB_SFSQI_CCR_1_2_2_M2D2_READ;
	    }
	}

	if (MID == STM32WB_SFSQI_MID_MACRONIX)
	{
	    stm32wb_sfsqi_device.insn_erase_suspend = STM32WB_SFSQI_INSN_PERSUS;
	    stm32wb_sfsqi_device.insn_erase_resume = STM32WB_SFSQI_INSN_PERRSM;
	    stm32wb_sfsqi_device.insn_program_suspend = STM32WB_SFSQI_INSN_PERSUS;
	    stm32wb_sfsqi_device.insn_program_resume = STM32WB_SFSQI_INSN_PERRSM;

	    if (quad)
	    {
		if (DID[0] == 0x20)
		{
		    stm32wb_sfsqi_device.ccr_read = STM32WB_SFSQI_CCR_1_1_4_D8_READ;
		}
		else
		{
		    stm32wb_sfsqi_device.ccr_program = STM32WB_SFSQI_CCR_1_4_4_PAGE_PROGRAM;
		    stm32wb_sfsqi_device.ccr_read = STM32WB_SFSQI_CCR_1_4_4_M2D4_READ;
		}
			
		status = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDSR);

		if (!(status & STM32WB_SFSQI_SR_QE))
		{
		    status = stm32wb_sfsqi_write_enable();
		    
		    stm32wb_sfsqi_command_write_1(STM32WB_SFSQI_INSN_WRSR, (status | STM32WB_SFSQI_SR_QE));
		    
		    do
		    {
			status = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDSR);
		    } 
		    while (status & (STM32WB_SFSQI_SR_WEL | STM32WB_SFSQI_SR_WIP));
		}
	    }
	    else
	    {
		stm32wb_sfsqi_device.ccr_read = STM32WB_SFSQI_CCR_1_2_2_M2D2_READ;
	    }
	    
	    if (DID[0] == 0x28)
	    {
		stm32wb_sfsqi_command_read_2(STM32WB_SFSQI_INSN_RDCONF, &config1, &config2);

		if (!(config2 & STM32WB_SFSQI_CONF2_HPM))
		{
		    status = stm32wb_sfsqi_write_enable();
		    
		    stm32wb_sfsqi_command_write_3(STM32WB_SFSQI_INSN_WRSR, status, config1, config2 | STM32WB_SFSQI_CONF2_HPM);
		    
		    do
		    {
			status = stm32wb_sfsqi_command_read_1(STM32WB_SFSQI_INSN_RDSR);
		    } 
		    while (status & (STM32WB_SFSQI_SR_WEL | STM32WB_SFSQI_SR_WIP));
		}
	    }
	}

	if (MID == STM32WB_SFSQI_MID_WINBOND)
	{
	    stm32wb_sfsqi_device.insn_erase_suspend = STM32WB_SFSQI_INSN_EPS;
	    stm32wb_sfsqi_device.insn_erase_resume = STM32WB_SFSQI_INSN_EPR;
	    stm32wb_sfsqi_device.insn_program_suspend = STM32WB_SFSQI_INSN_EPS;
	    stm32wb_sfsqi_device.insn_program_resume = STM32WB_SFSQI_INSN_EPR;

	    if (quad)
	    {
		stm32wb_sfsqi_device.ccr_program = STM32WB_SFSQI_CCR_1_1_4_PAGE_PROGRAM;
		stm32wb_sfsqi_device.ccr_read = STM32WB_SFSQI_CCR_1_4_4_M2D4_READ;

		stm32wb_sfsqi_command_read_2(STM32WB_SFSQI_INSN_RDSR, &status, &status2);

		if (!(status2 & STM32WB_SFSQI_SR2_QE))
		{
		    stm32wb_sfsqi_command(STM32WB_SFSQI_INSN_WREN_VOLATILE);
		    
		    stm32wb_sfsqi_command_write_2(STM32WB_SFSQI_INSN_WRSR, status, (status2 | STM32WB_SFSQI_SR2_QE));
		}
	    }
	    else
	    {
		stm32wb_sfsqi_device.ccr_read = STM32WB_SFSQI_CCR_1_2_2_M2D2_READ;
	    }
	}

	if (stm32wb_sfsqi_device.info.capacity > 0x01000000)
	{
	    if (MID == STM32WB_SFSQI_MID_SPANSION)
	    {
		stm32wb_sfsqi_command_write_1(STM32WB_SFSQI_INSN_BRWR, STM32WB_SFSQI_BR_EXTADD);
	    }
	    else
	    {
		stm32wb_sfsqi_command(STM32WB_SFSQI_INSN_EN4B);
	    }

	    stm32wb_sfsqi_device.ccr_erase |= QUADSPI_CCR_ADSIZE_0;
	    stm32wb_sfsqi_device.ccr_program |= QUADSPI_CCR_ADSIZE_0;
	    stm32wb_sfsqi_device.ccr_read |= QUADSPI_CCR_ADSIZE_0;
	}
	
	stm32wb_sfsqi_release();

	stm32wb_sfsqi_device.state = STM32WB_SFSQI_STATE_READY;
	stm32wb_sfsqi_device.hclk = 0;
	stm32wb_sfsqi_device.sclk = 0;

	stm32wb_system_register(&stm32wb_sfsqi_device.notify, (stm32wb_system_callback_t)stm32wb_sfsqi_sleep, NULL, STM32WB_SYSTEM_NOTIFY_STOP_PREPARE);
	
	__stm32wb_sflash_initialize(&stm32wb_sfsqi_interface, &stm32wb_sfsqi_device.info);
    }
    else
    {
	stm32wb_sfsqi_release();

	stm32wb_gpio_pin_configure(stm32wb_sfsqi_device.pins.clk, STM32WB_GPIO_MODE_ANALOG);
	stm32wb_gpio_pin_configure(stm32wb_sfsqi_device.pins.cs, STM32WB_GPIO_MODE_ANALOG);
	stm32wb_gpio_pin_configure(stm32wb_sfsqi_device.pins.io0, STM32WB_GPIO_MODE_ANALOG);
	stm32wb_gpio_pin_configure(stm32wb_sfsqi_device.pins.io1, STM32WB_GPIO_MODE_ANALOG);
	
	if ((stm32wb_sfsqi_device.pins.io2 != STM32WB_GPIO_PIN_NONE) && (stm32wb_sfsqi_device.pins.io3 != STM32WB_GPIO_PIN_NONE))
	{
	    stm32wb_gpio_pin_configure(stm32wb_sfsqi_device.pins.io2, STM32WB_GPIO_MODE_ANALOG);
	    stm32wb_gpio_pin_configure(stm32wb_sfsqi_device.pins.io3, STM32WB_GPIO_MODE_ANALOG);
	}

	stm32wb_sfsqi_device.state = STM32WB_SFSQI_STATE_NONE;
    }
    
    return success;
}
