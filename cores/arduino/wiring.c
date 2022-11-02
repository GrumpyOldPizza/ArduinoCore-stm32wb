/*
 * Copyright (c) 2016-2020 Thomas Roell.  All rights reserved.
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

#include "Arduino.h"
#include "wiring_private.h"
#include "dosfs_api.h"
#include "dosfs_sflash.h"

#ifdef __cplusplus
extern "C" {
#endif

#undef ARDUINO_RV2

k_event_t g_wakeup_event = K_EVENT_INIT();
  
#if defined(USBCON)
stm32wb_uart_t g_Serial1;
extern const stm32wb_uart_params_t g_Serial1Params;
#else
stm32wb_uart_t g_Serial;
extern const stm32wb_uart_params_t g_SerialParams;
#endif
  
stm32wb_spi_t g_SPI;
extern const stm32wb_spi_params_t g_SPIParams;

stm32wb_i2c_t g_Wire;
extern const stm32wb_i2c_params_t g_WireParams;

#if (STORAGE_TYPE == 1)
extern const stm32wb_sfspi_params_t g_SFSPIParams;
#endif

#if (STORAGE_TYPE == 2)
extern const stm32wb_sdspi_params_t g_SDSPIParams;
#endif

extern void __libc_init_array(void);
#if defined(ARDUINO_RV2)
extern void cmsis_rv2(void);
#endif

void __runtime_start(void)
{
    stm32wb_system_initialize(0, __SYSTEM_CORE_CLOCK__, 0, 0, STM32WB_CONFIG_LSECLK, STM32WB_CONFIG_HSECLK, STM32WB_CONFIG_SYSOPT);

#if defined(ARDUINO_RV2)
    cmsis_rv2();
#endif

    // k_system_initialize(&armv7m_rtt_hook_table);
    k_system_initialize(NULL);
    
    __libc_init_array();

    k_system_start((k_task_routine_t)main, NULL); 
}
  
void init(void)
{
#if (STORAGE_TYPE == 1)
    if (g_SPI.state == STM32WB_SPI_STATE_NONE) {
        stm32wb_spi_create(&g_SPI, &g_SPIParams);
    }

    stm32wb_sfspi_initialize(&g_SPI, &g_SFSPIParams);
#endif
    

#if (STORAGE_TYPE == 2)
    if (g_SPI.state == STM32WB_SPI_STATE_NONE) {
        stm32wb_spi_create(&g_SPI, &g_SPIParams);
    }

    stm32wb_sdspi_initialize(&g_SPI, &g_SDSPIParams);
#endif
}

#ifdef __cplusplus
}
#endif
