/*
 * Copyright (c) 2020 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32WB_HSEM_H)
#define _STM32WB_HSEM_H

#ifdef __cplusplus
extern "C" {
#endif

#define STM32WB_HSEM_IRQ_PRIORITY   ARMV7M_IRQ_PRIORITY_HSEM
  
#define HSEM_R_COREID_CPU1   (4 << HSEM_R_COREID_Pos)
#define HSEM_RLR_COREID_CPU1 (4 << HSEM_R_COREID_Pos)

#define STM32WB_HSEM_INDEX_RNG         0
#define STM32WB_HSEM_INDEX_PKA         1
#define STM32WB_HSEM_INDEX_FLASH       2
#define STM32WB_HSEM_INDEX_RCC         3
#define STM32WB_HSEM_INDEX_DEEPSLEEP   4
#define STM32WB_HSEM_INDEX_CLK48       5
#define STM32WB_HSEM_INDEX_FLASH_CPU1  6
#define STM32WB_HSEM_INDEX_FLASH_CPU2  7

#define STM32WB_HSEM_PROCID_NONE       0
#define STM32WB_HSEM_PROCID_RANDOM     1
#define STM32WB_HSEM_PROCID_USBD_DCD   2
  
extern void __stm32wb_hsem_initialize(void);
extern bool __stm32wb_hsem_lock(uint32_t index, uint32_t procid);

static inline bool stm32wb_hsem_trylock(uint32_t index, uint32_t procid)
{
    if (__builtin_constant_p(procid) && (procid == STM32WB_HSEM_PROCID_NONE))
    {
        if (HSEM->RLR[index] == (HSEM_RLR_LOCK | HSEM_RLR_COREID_CPU1))
        {
            // armv7m_rtt_printf("HSEM_TRYLOCK(%d [%d])\n", index, procid);

            return true;
        }
    }
    else
    {
	HSEM->R[index] = (HSEM_RLR_LOCK | HSEM_R_COREID_CPU1 | procid);

	if (HSEM->R[index] == (HSEM_RLR_LOCK | HSEM_R_COREID_CPU1 | procid))
        {
            // armv7m_rtt_printf("HSEM_TRYLOCK(%d [%d])\n", index, procid);

            return true;
        }
    }

    return false;
}

static inline bool stm32wb_hsem_lock(uint32_t index, uint32_t procid)
{
    if (__builtin_constant_p(procid) && (procid == STM32WB_HSEM_PROCID_NONE))
    {
	if (HSEM->RLR[index] == (HSEM_RLR_LOCK | HSEM_RLR_COREID_CPU1))
	{
            // armv7m_rtt_printf("HSEM_LOCK(%d [%d])\n", index, procid);

            return true;
	}
    }
    else
    {
	HSEM->R[index] = (HSEM_RLR_LOCK | HSEM_R_COREID_CPU1 | procid);

	if (HSEM->R[index] == (HSEM_RLR_LOCK | HSEM_R_COREID_CPU1 | procid))
	{
            // armv7m_rtt_printf("HSEM_LOCK(%d [%d])\n", index, procid);

          return true;
	}
    }

    return __stm32wb_hsem_lock(index, procid);
}
    
static inline void stm32wb_hsem_unlock(uint32_t index, uint32_t procid)
{
    // armv7m_rtt_printf("HSEM_UNLOCK(%d [%d])\n", index, procid);
    
    HSEM->R[index] = (HSEM_R_COREID_CPU1 | procid);
}

static inline bool stm32wb_hsem_is_locked(uint32_t index, uint32_t procid)
{
    return (HSEM->R[index] == (HSEM_RLR_LOCK | HSEM_R_COREID_CPU1 | procid));
}
  
extern void HSEM0_HSEMHandler(void);
extern void HSEM1_HSEMHandler(void);
extern void HSEM2_HSEMHandler(void);
extern void HSEM3_HSEMHandler(void);
extern void HSEM4_HSEMHandler(void);
extern void HSEM5_HSEMHandler(void);
extern void HSEM6_HSEMHandler(void);
extern void HSEM7_HSEMHandler(void);
extern void HSEM8_HSEMHandler(void);
extern void HSEM9_HSEMHandler(void);
extern void HSEM10_HSEMHandler(void);
extern void HSEM11_HSEMHandler(void);
extern void HSEM12_HSEMHandler(void);
extern void HSEM13_HSEMHandler(void);
extern void HSEM14_HSEMHandler(void);
extern void HSEM15_HSEMHandler(void);
extern void HSEM16_HSEMHandler(void);
extern void HSEM17_HSEMHandler(void);
extern void HSEM18_HSEMHandler(void);
extern void HSEM19_HSEMHandler(void);
extern void HSEM20_HSEMHandler(void);
extern void HSEM21_HSEMHandler(void);
extern void HSEM22_HSEMHandler(void);
extern void HSEM23_HSEMHandler(void);
extern void HSEM24_HSEMHandler(void);
extern void HSEM25_HSEMHandler(void);
extern void HSEM26_HSEMHandler(void);
extern void HSEM27_HSEMHandler(void);
extern void HSEM28_HSEMHandler(void);
extern void HSEM29_HSEMHandler(void);
extern void HSEM30_HSEMHandler(void);
extern void HSEM31_HSEMHandler(void);
    
#define RNG_HSEMHandler                HSEM0_HSEMHandler
#define PKA_HSEMHandler                HSEM1_HSEMHandler
#define FLASH_CPU2_HSEMHandler         HSEM7_HSEMHandler    
    
#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_HSEM_H */
