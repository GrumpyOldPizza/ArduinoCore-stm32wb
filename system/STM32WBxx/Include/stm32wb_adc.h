/*
 * Copyright (c) 2016-2017 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32WB_ADC_H)
#define _STM32WB_ADC_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32wbxx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32WB_ADC_CHANNEL_VREFINT                 0
#define STM32WB_ADC_CHANNEL_1                       1   /* PC0 */
#define STM32WB_ADC_CHANNEL_2                       2   /* PC1 */
#define STM32WB_ADC_CHANNEL_3                       3   /* PC2 */
#define STM32WB_ADC_CHANNEL_4                       4   /* PC3 */
#define STM32WB_ADC_CHANNEL_5                       5   /* PA0 */
#define STM32WB_ADC_CHANNEL_6                       6   /* PA1 */
#define STM32WB_ADC_CHANNEL_7                       7   /* PA2 */
#define STM32WB_ADC_CHANNEL_8                       8   /* PA3 */
#define STM32WB_ADC_CHANNEL_9                       9   /* PA4 */
#define STM32WB_ADC_CHANNEL_10                      10  /* PA5 */
#define STM32WB_ADC_CHANNEL_11                      11  /* PA6 */
#define STM32WB_ADC_CHANNEL_12                      12  /* PA7 */
#define STM32WB_ADC_CHANNEL_13                      13  /* PC4 */
#define STM32WB_ADC_CHANNEL_14                      14  /* PC5 */
#define STM32WB_ADC_CHANNEL_15                      15  /* PA8 */
#define STM32WB_ADC_CHANNEL_16                      16  /* PA9 */
#define STM32WB_ADC_CHANNEL_TSENSE                  17  /* TEMPERATURE SENSOR */
#define STM32WB_ADC_CHANNEL_VBAT                    18  /* VBAT/3 */

#define STM32WB_ADC_OPTION_WIDTH_SHIFT              0
#define STM32WB_ADC_OPTION_WIDTH_MASK               0x00000007
#define STM32WB_ADC_OPTION_WIDTH_12                 0x00000000
#define STM32WB_ADC_OPTION_WIDTH_13                 0x00000001
#define STM32WB_ADC_OPTION_WIDTH_14                 0x00000002
#define STM32WB_ADC_OPTION_WIDTH_15                 0x00000003
#define STM32WB_ADC_OPTION_WIDTH_16                 0x00000004
#define STM32WB_ADC_OPTION_RATIO_SHIFT              3
#define STM32WB_ADC_OPTION_RATIO_MASK               0x00000078
#define STM32WB_ADC_OPTION_RATIO_1                  0x00000000
#define STM32WB_ADC_OPTION_RATIO_2                  0x00000008
#define STM32WB_ADC_OPTION_RATIO_4                  0x00000010
#define STM32WB_ADC_OPTION_RATIO_8                  0x00000018
#define STM32WB_ADC_OPTION_RATIO_16                 0x00000020
#define STM32WB_ADC_OPTION_RATIO_32                 0x00000028
#define STM32WB_ADC_OPTION_RATIO_64                 0x00000030
#define STM32WB_ADC_OPTION_RATIO_128                0x00000038
#define STM32WB_ADC_OPTION_RATIO_256                0x00000040
#define STM32WB_ADC_OPTION_STOP                     0x80000000
  
#define STM32WB_ADC_VREFINT_CAL                     (*((const uint16_t*)0x1fff75aa))
#define STM32WB_ADC_VREFINT_VREF                    (3.6)
#define STM32WB_ADC_TSENSE_CAL1                     (*((const uint16_t*)0x1fff75a8))
#define STM32WB_ADC_TSENSE_CAL2                     (*((const uint16_t*)0x1fff75ca))
#define STM32WB_ADC_TSENSE_CAL1_TEMP                (30.0)
#define STM32WB_ADC_TSENSE_CAL2_TEMP                (130.0)
#define STM32WB_ADC_TSENSE_CAL_VREF                 (3.0)
  
#define STM32WB_ADC_VREFINT_PERIOD                  4000   /* nS */
#define STM32WB_ADC_TSENSE_PERIOD                   5000   /* nS */
#define STM32WB_ADC_VBAT_PERIOD                     12000  /* nS */

#define STM32WB_ADC_IRQ_PRIORITY                    ARMV7M_IRQ_PRIORITY_ADC
  
extern void __stm32wb_adc_initialize(void);

extern bool stm32wb_adc_start(const uint8_t *channels, uint32_t count, uint32_t period, uint32_t option);
extern bool stm32wb_adc_stop(void);
extern bool stm32wb_adc_done(void);
extern uint32_t stm32wb_adc_data(uint16_t *data, uint32_t count);
extern uint32_t stm32wb_adc_convert(const uint8_t *channels, uint16_t *data, uint32_t count, uint32_t period, uint32_t option);

#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_ADC_H */
