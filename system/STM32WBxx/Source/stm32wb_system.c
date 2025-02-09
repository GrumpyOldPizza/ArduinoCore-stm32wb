/*
 * Copyright (c) 2016-2023 Thomas Roell.  All rights reserved.
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

#include <stdlib.h>

#include "armv7m.h"

#include "stm32wb_boot.h"
#include "stm32wb_system.h"
#include "stm32wb_gpio.h"
#include "stm32wb_exti.h"
#include "stm32wb_dma.h"
#include "stm32wb_rtc.h"
#include "stm32wb_lptim.h"
#include "stm32wb_hsem.h"
#include "stm32wb_ipcc.h"
#include "stm32wb_adc.h"
#include "stm32wb_flash.h"
#include "stm32wb_eeprom.h"
#include "stm32wb_random.h"
#include "stm32wb_otp.h"

/* CPU2 reserves the first 16 bytes of SRAM2A for Hardfault
   and security attack magic values. AN5289 4.8.2
*/

#define STM32WB_SYSTEM_REFERENCE_RANGE_1_2      \
    (STM32WB_SYSTEM_REFERENCE_HSI48 |           \
     STM32WB_SYSTEM_REFERENCE_WIRELESS |        \
     STM32WB_SYSTEM_REFERENCE_USB |             \
     STM32WB_SYSTEM_REFERENCE_RNG |             \
     STM32WB_SYSTEM_REFERENCE_SYSCLK_RANGE_1 |  \
     STM32WB_SYSTEM_REFERENCE_SYSCLK_RANGE_2 |  \
     STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_1 |  \
     STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_2)

#define STM32WB_SYSTEM_REFERENCE_RANGE_1        \
    (STM32WB_SYSTEM_REFERENCE_HSI48 |           \
     STM32WB_SYSTEM_REFERENCE_WIRELESS |        \
     STM32WB_SYSTEM_REFERENCE_USB |             \
     STM32WB_SYSTEM_REFERENCE_RNG |             \
     STM32WB_SYSTEM_REFERENCE_SYSCLK_RANGE_1 |  \
     STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_1)

extern uint8_t __Vectors[];
extern uint8_t __StackTop[];

extern uint8_t __cctext_start__[];
extern uint8_t __cctext_end__[];
extern uint8_t __cctext_flash__[];
extern uint8_t __ccram_base__[];
extern uint8_t __ccram_limit__[];

typedef struct _stm32wb_system_device_t {
    uint32_t                        options;
    uint32_t                        lseclk;
    uint32_t                        hseclk;
    uint32_t                        sysclk;
    uint32_t                        hclk;
    uint32_t                        pclk1;
    uint32_t                        pclk2;
    uint32_t                        saiclk; 
    uint8_t                         mco;
    uint8_t                         lsco;
    uint8_t                         hse;
    uint8_t                         lsi;
    uint8_t                         msi;
    uint8_t                         hsi16;
    uint8_t                         clk48;
    uint8_t                         pllsys;
    uint8_t                         latency;
    uint8_t                         smps;
    uint8_t                         palevel;
    uint8_t                         boost;
    uint32_t                        msirange;
    uint32_t                        c2hpre;
    stm32wb_system_pvm1_callback_t  pvm1_callback;
    void                            *pvm1_context;
    stm32wb_system_notify_t         *notify;
    volatile uint8_t                policy;
    volatile uint8_t                events;
    volatile uint8_t                lock[STM32WB_SYSTEM_LOCK_COUNT];
    volatile uint32_t               reference;
} stm32wb_system_device_t;

static stm32wb_system_device_t stm32wb_system_device;

static volatile uint32_t * const stm32wb_system_xlate_RSTR[STM32WB_SYSTEM_PERIPH_COUNT] = {
    &RCC->AHB2RSTR,           /* STM32WB_SYSTEM_PERIPH_ADC */
    &RCC->APB1RSTR1,          /* STM32WB_SYSTEM_PERIPH_USB */
    &RCC->APB2RSTR,           /* STM32WB_SYSTEM_PERIPH_USART1 */
    &RCC->APB1RSTR2,          /* STM32WB_SYSTEM_PERIPH_LPUART1 */
    &RCC->APB1RSTR1,          /* STM32WB_SYSTEM_PERIPH_I2C1 */
    &RCC->APB1RSTR1,          /* STM32WB_SYSTEM_PERIPH_I2C3 */
    &RCC->APB2RSTR,           /* STM32WB_SYSTEM_PERIPH_SPI1 */
    &RCC->APB1RSTR1,          /* STM32WB_SYSTEM_PERIPH_SPI2 */
    &RCC->AHB3RSTR,           /* STM32WB_SYSTEM_PERIPH_QSPI */
    &RCC->APB2RSTR,           /* STM32WB_SYSTEM_PERIPH_SAI1 */
    &RCC->APB2RSTR,           /* STM32WB_SYSTEM_PERIPH_TIM1 */
    &RCC->APB1RSTR1,          /* STM32WB_SYSTEM_PERIPH_TIM2 */
    &RCC->APB2RSTR,           /* STM32WB_SYSTEM_PERIPH_TIM16 */
    &RCC->APB2RSTR,           /* STM32WB_SYSTEM_PERIPH_TIM17 */
    &RCC->APB1RSTR1,          /* STM32WB_SYSTEM_PERIPH_LPTIM1 */
    &RCC->APB1RSTR2,          /* STM32WB_SYSTEM_PERIPH_LPTIM2 */
    &RCC->AHB1RSTR,           /* STM32WB_SYSTEM_PERIPH_TSC */
    &RCC->APB1RSTR1,          /* STM32WB_SYSTEM_PERIPH_LCD */
    &RCC->AHB3RSTR,           /* STM32WB_SYSTEM_PERIPH_HSEM */
    &RCC->AHB3RSTR,           /* STM32WB_SYSTEM_PERIPH_IPCC */
    &RCC->AHB1RSTR,           /* STM32WB_SYSTEM_PERIPH_CRC */
    &RCC->AHB3RSTR,           /* STM32WB_SYSTEM_PERIPH_RNG */
    &RCC->AHB2RSTR,           /* STM32WB_SYSTEM_PERIPH_AES1 */
    &RCC->AHB3RSTR,           /* STM32WB_SYSTEM_PERIPH_AES2 */
    &RCC->AHB3RSTR,           /* STM32WB_SYSTEM_PERIPH_PKA */
};

static uint32_t const stm32wb_system_xlate_RSTMSK[STM32WB_SYSTEM_PERIPH_COUNT] = {
    RCC_AHB2RSTR_ADCRST,      /* STM32WB_SYSTEM_PERIPH_ADC */
    RCC_APB1RSTR1_USBRST,     /* STM32WB_SYSTEM_PERIPH_USB */
    RCC_APB2RSTR_USART1RST,   /* STM32WB_SYSTEM_PERIPH_USART1 */
    RCC_APB1RSTR2_LPUART1RST, /* STM32WB_SYSTEM_PERIPH_LPUART1 */
    RCC_APB1RSTR1_I2C1RST,    /* STM32WB_SYSTEM_PERIPH_I2C1 */
    RCC_APB1RSTR1_I2C3RST,    /* STM32WB_SYSTEM_PERIPH_I2C3 */
    RCC_APB2RSTR_SPI1RST,     /* STM32WB_SYSTEM_PERIPH_SPI1 */
    RCC_APB1RSTR1_SPI2RST,    /* STM32WB_SYSTEM_PERIPH_SPI2 */
    RCC_AHB3RSTR_QUADSPIRST,  /* STM32WB_SYSTEM_PERIPH_QSPI */
    RCC_APB2RSTR_SAI1RST,     /* STM32WB_SYSTEM_PERIPH_SAI1 */
    RCC_APB2RSTR_TIM1RST,     /* STM32WB_SYSTEM_PERIPH_TIM1 */
    RCC_APB1RSTR1_TIM2RST,    /* STM32WB_SYSTEM_PERIPH_TIM2 */
    RCC_APB2RSTR_TIM16RST,    /* STM32WB_SYSTEM_PERIPH_TIM16 */
    RCC_APB2RSTR_TIM17RST,    /* STM32WB_SYSTEM_PERIPH_TIM17 */
    RCC_APB1RSTR1_LPTIM1RST,  /* STM32WB_SYSTEM_PERIPH_LPTIM1 */
    RCC_APB1RSTR2_LPTIM2RST,  /* STM32WB_SYSTEM_PERIPH_LPTIM2 */
    RCC_AHB1RSTR_TSCRST,      /* STM32WB_SYSTEM_PERIPH_TSC */
    RCC_APB1RSTR1_LCDRST,     /* STM32WB_SYSTEM_PERIPH_LCD */
    RCC_AHB3RSTR_HSEMRST,     /* STM32WB_SYSTEM_PERIPH_HSEM */
    RCC_AHB3RSTR_IPCCRST,     /* STM32WB_SYSTEM_PERIPH_IPCC */
    RCC_AHB1RSTR_CRCRST,      /* STM32WB_SYSTEM_PERIPH_CRC */
    RCC_AHB3RSTR_RNGRST,      /* STM32WB_SYSTEM_PERIPH_RNG */
    RCC_AHB2RSTR_AES1RST,     /* STM32WB_SYSTEM_PERIPH_AES1 */
    RCC_AHB3RSTR_AES2RST,     /* STM32WB_SYSTEM_PERIPH_AES2 */
    RCC_AHB3RSTR_PKARST,      /* STM32WB_SYSTEM_PERIPH_PKA */
};

static volatile uint32_t * const stm32wb_system_xlate_ENR[STM32WB_SYSTEM_PERIPH_COUNT] = {
    &RCC->AHB2ENR,            /* STM32WB_SYSTEM_PERIPH_ADC */
    &RCC->APB1ENR1,           /* STM32WB_SYSTEM_PERIPH_USB */
    &RCC->APB2ENR,            /* STM32WB_SYSTEM_PERIPH_USART1 */
    &RCC->APB1ENR2,           /* STM32WB_SYSTEM_PERIPH_LPUART1 */
    &RCC->APB1ENR1,           /* STM32WB_SYSTEM_PERIPH_I2C1 */
    &RCC->APB1ENR1,           /* STM32WB_SYSTEM_PERIPH_I2C3 */
    &RCC->APB2ENR,            /* STM32WB_SYSTEM_PERIPH_SPI1 */
    &RCC->APB1ENR1,           /* STM32WB_SYSTEM_PERIPH_SPI2 */
    &RCC->AHB3ENR,            /* STM32WB_SYSTEM_PERIPH_QSPI */
    &RCC->APB2ENR,            /* STM32WB_SYSTEM_PERIPH_SAI1 */
    &RCC->APB2ENR,            /* STM32WB_SYSTEM_PERIPH_TIM1 */
    &RCC->APB1ENR1,           /* STM32WB_SYSTEM_PERIPH_TIM2 */
    &RCC->APB2ENR,            /* STM32WB_SYSTEM_PERIPH_TIM16 */
    &RCC->APB2ENR,            /* STM32WB_SYSTEM_PERIPH_TIM17 */
    &RCC->APB1ENR1,           /* STM32WB_SYSTEM_PERIPH_LPTIM1 */
    &RCC->APB1ENR2,           /* STM32WB_SYSTEM_PERIPH_LPTIM2 */
    &RCC->AHB1ENR,            /* STM32WB_SYSTEM_PERIPH_TSC */
    &RCC->APB1ENR1,           /* STM32WB_SYSTEM_PERIPH_LCD */
    &RCC->AHB3ENR,            /* STM32WB_SYSTEM_PERIPH_HSEM */
    &RCC->AHB3ENR,            /* STM32WB_SYSTEM_PERIPH_IPCC */
    &RCC->AHB1ENR,            /* STM32WB_SYSTEM_PERIPH_CRC */
    &RCC->AHB3ENR,            /* STM32WB_SYSTEM_PERIPH_RNG */
    &RCC->AHB2ENR,            /* STM32WB_SYSTEM_PERIPH_AES1 */
    &RCC->AHB3ENR,            /* STM32WB_SYSTEM_PERIPH_AES2 */
    &RCC->AHB3ENR,            /* STM32WB_SYSTEM_PERIPH_PKA */
};

static uint32_t const stm32wb_system_xlate_ENMSK[STM32WB_SYSTEM_PERIPH_COUNT] = {
    RCC_AHB2ENR_ADCEN,        /* STM32WB_SYSTEM_PERIPH_ADC */
    RCC_APB1ENR1_USBEN,       /* STM32WB_SYSTEM_PERIPH_USB */
    RCC_APB2ENR_USART1EN,     /* STM32WB_SYSTEM_PERIPH_USART1 */
    RCC_APB1ENR2_LPUART1EN,   /* STM32WB_SYSTEM_PERIPH_LPUART1 */
    RCC_APB1ENR1_I2C1EN,      /* STM32WB_SYSTEM_PERIPH_I2C1 */
    RCC_APB1ENR1_I2C3EN,      /* STM32WB_SYSTEM_PERIPH_I2C3 */
    RCC_APB2ENR_SPI1EN,       /* STM32WB_SYSTEM_PERIPH_SPI1 */
    RCC_APB1ENR1_SPI2EN,      /* STM32WB_SYSTEM_PERIPH_SPI2 */
    RCC_AHB3ENR_QUADSPIEN,    /* STM32WB_SYSTEM_PERIPH_QSPI */
    RCC_APB2ENR_SAI1EN,       /* STM32WB_SYSTEM_PERIPH_SAI1 */
    RCC_APB2ENR_TIM1EN,       /* STM32WB_SYSTEM_PERIPH_TIM1 */
    RCC_APB1ENR1_TIM2EN,      /* STM32WB_SYSTEM_PERIPH_TIM2 */
    RCC_APB2ENR_TIM16EN,      /* STM32WB_SYSTEM_PERIPH_TIM16 */
    RCC_APB2ENR_TIM17EN,      /* STM32WB_SYSTEM_PERIPH_TIM17 */
    RCC_APB1ENR1_LPTIM1EN,    /* STM32WB_SYSTEM_PERIPH_LPTIM1 */
    RCC_APB1ENR2_LPTIM2EN,    /* STM32WB_SYSTEM_PERIPH_LPTIM2 */
    RCC_AHB1ENR_TSCEN,        /* STM32WB_SYSTEM_PERIPH_TSC */
    RCC_APB1ENR1_LCDEN,       /* STM32WB_SYSTEM_PERIPH_LCD */
    RCC_AHB3ENR_HSEMEN,       /* STM32WB_SYSTEM_PERIPH_HSEM */
    RCC_AHB3ENR_IPCCEN,       /* STM32WB_SYSTEM_PERIPH_IPCC */
    RCC_AHB1ENR_CRCEN,        /* STM32WB_SYSTEM_PERIPH_CRC */
    RCC_AHB3ENR_RNGEN,        /* STM32WB_SYSTEM_PERIPH_RNG */
    RCC_AHB2ENR_AES1EN,       /* STM32WB_SYSTEM_PERIPH_AES1 */
    RCC_AHB3ENR_AES2EN,       /* STM32WB_SYSTEM_PERIPH_AES2 */
    RCC_AHB3ENR_PKAEN,        /* STM32WB_SYSTEM_PERIPH_PKA */
};

static void stm32wb_system_event(uint32_t event);

static __attribute__((used)) uint32_t __SECTION_FATAL  __stm32wb_system_reset_cause;
static __attribute__((used)) uint32_t __SECTION_FATAL  __stm32wb_system_reset_reason;
static __attribute__((used)) uint32_t __SECTION_NOINIT __stm32wb_system_wakeup_reason;

uint32_t __SECTION_NOINIT SystemCoreClock;

void SystemInit(void)
{
    uint32_t flash_acr;
    volatile uint32_t *cctext, *cctext_e;
    const uint32_t *cctext_f;

    RCC->CIER = 0x00000000;

    RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;

    /* Switch to Main Flash @ 0x00000000. Make sure the I/D CACHE is
     * disabled to avoid stale data in the cache.
     */

    flash_acr = FLASH->ACR;

    FLASH->ACR = flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
        
    SYSCFG->MEMRMP = 0;

    PWR->CR1 |= PWR_CR1_DBP;
    
    while (!(PWR->CR1 & PWR_CR1_DBP))
    {
    }
    
#if 0      
    if (RCC->BDCR & RCC_BDCR_RTCEN)
    {
        if ((((RTC->BKP16R & STM32WB_RTC_BKP16R_DATA_MASK) >> STM32WB_RTC_BKP16R_DATA_SHIFT) != ((~RTC->BKP16R & STM32WB_RTC_BKP16R_NOT_DATA_MASK) >> STM32WB_RTC_BKP16R_NOT_DATA_SHIFT)) ||
            ((RTC->BKP16R & STM32WB_RTC_BKP16R_REVISION_MASK) != STM32WB_RTC_BKP16R_REVISION_CURRENT))
        {
            /* Reset RTC after a BKP mismatch, but also go throu a CPU reset to start off clean.
             */
            RCC->BDCR &= ~RCC_BDCR_RTCEN;
            RCC->BDCR |= RCC_BDCR_BDRST;
            RCC->BDCR &= ~RCC_BDCR_BDRST;

            NVIC_SystemReset();
            
            while (1)
            {
            }
        }
    }
#endif
    
    if (PWR->EXTSCR & PWR_EXTSCR_C1SBF)
    {
        __stm32wb_system_reset_cause = STM32WB_SYSTEM_RESET_CAUSE_WAKEUP;
        __stm32wb_system_reset_reason = 0;
        __stm32wb_system_wakeup_reason = (PWR->SR1 & PWR_SR1_WUF);

        if ((RCC->BDCR & RCC_BDCR_RTCEN) && (RTC->ISR & RTC_ISR_ALRAF))
        {
            __stm32wb_system_wakeup_reason |= STM32WB_SYSTEM_WAKEUP_REASON_TIMEOUT;
        }

        if (RCC->CSR & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF))
        {
            __stm32wb_system_wakeup_reason |= STM32WB_SYSTEM_WAKEUP_REASON_WATCHDOG;
        }
        
        if (RCC->CSR & (RCC_CSR_BORRSTF | RCC_CSR_PINRSTF))
        {
            __stm32wb_system_wakeup_reason |= STM32WB_SYSTEM_WAKEUP_REASON_RESET;
        }
    }
    else
    {
        /* The STM32 BOOTLOADER does not do a reset when jumping to the newly flashed
         * code. Hence CPU2 might be booted. If so, do a software reset.
         */
        if (PWR->CR4 & PWR_CR4_C2BOOT)
        {
            NVIC_SystemReset();
            
            while (1)
            {
            }
        }

        __stm32wb_system_wakeup_reason = 0;

        if (RCC->CSR & RCC_CSR_SFTRSTF)
        {
            if (!((RCC->BDCR & RCC_BDCR_RTCEN) && (RTC->BKP16R & STM32WB_RTC_BKP16R_FATAL)))
            {
                __stm32wb_system_reset_cause = STM32WB_SYSTEM_RESET_CAUSE_SOFTWARE;
                __stm32wb_system_reset_reason = 0;
            }
        }
        else
        {
            if (RCC->CSR & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF))
            {
                __stm32wb_system_reset_cause = STM32WB_SYSTEM_RESET_CAUSE_WATCHDOG;
                __stm32wb_system_reset_reason = 0;

                if (RCC->BDCR & RCC_BDCR_RTCEN)
                {
                    RTC->WPR = 0xca;
                    RTC->WPR = 0x53;
                    
                    RTC->BKP16R = (RTC->BKP16R & ~((STM32WB_RTC_BKP16R_FATAL << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT) | STM32WB_RTC_BKP16R_DFU)) | ((STM32WB_RTC_BKP16R_DFU << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT) | STM32WB_RTC_BKP16R_FATAL);

                    RTC->WPR = 0x00;
                }
            }
            else
            {
                __stm32wb_system_reset_cause = STM32WB_SYSTEM_RESET_CAUSE_HARDWARE;
                __stm32wb_system_reset_reason = 0;
            }
        }
    }
    
    RCC->CSR |= RCC_CSR_RMVF;
    RCC->CSR &= ~RCC_CSR_RMVF;

    PWR->SCR = (PWR_SCR_CWUF5 | PWR_SCR_CWUF4 | PWR_SCR_CWUF3 | PWR_SCR_CWUF2 | PWR_SCR_CWUF1);
    PWR->EXTSCR = PWR_EXTSCR_C1CSSF;
    
    /****************************************************************************************************************************************************************/
    
    /* We should be at a 4MHz MSI clock, so switch to HSI16 for the
     * init code to be half way fast.
     */

    FLASH->ACR = (flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN)) | (FLASH_ACR_ICRST | FLASH_ACR_DCRST);
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_0WS;
    
    if (!(RCC->CR & RCC_CR_HSION))
    {
        RCC->CR |= RCC_CR_HSION;
        
        while (!(RCC->CR & RCC_CR_HSIRDY))
        {
        }
    }

    if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
            
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
        {
        }
    }

    SystemCoreClock = 16000000;
    
    if (&__cctext_start__[0] != &__cctext_end__[0])
    {
        cctext = (uint32_t*)&__cctext_start__[0];
        cctext_e = (uint32_t*)&__cctext_end__[0];
        cctext_f = (const uint32_t*)&__cctext_flash__[0];
        
        do
        {
            *cctext++ = *cctext_f++;
        }
        while (cctext != cctext_e);
    }

    /* Write protect .cctext in SRAM2b */
    SYSCFG->SWPR2 = (0xffffffff >> (32 - (((uint32_t)__cctext_end__ - (uint32_t)__cctext_start__ + 1023) / 1024))) << (((uint32_t)__ccram_base__ - SRAM2B_BASE + 1023) / 1024);

    __armv7m_core_initialize();
}

void stm32wb_system_initialize(uint32_t sysclk, uint32_t hclk, uint32_t pclk1, uint32_t pclk2)
{
    uint32_t options, hseclk, lseclk, count;
    stm32wb_otp_hse_tune_t hse_tune;

    options = stm32wb_system_info.options;
    hseclk = stm32wb_system_info.hseclk;
    lseclk = stm32wb_system_info.lseclk;
    
    __disable_irq();

    if (RCC->BDCR & RCC_BDCR_RTCEN)
    {
        if (!(RCC->BDCR & RCC_BDCR_LSEON))
        {
            lseclk = 0;
        }

        if (!(RTC->BKP17R & STM32WB_RTC_BKP17R_HSECLK))
        {
            hseclk = 0;
        }
    }
    else
    {
        if (lseclk)
        {
            if (options & STM32WB_SYSTEM_OPTION_LSE_BYPASS)
            {
                RCC->BDCR |= RCC_BDCR_LSEBYP;
            }
            else
            {
                if (!(RCC->BDCR & RCC_BDCR_LSEON))
                {
                    /* Add LSEDRV before turning on LSE.
                     */
                    RCC->BDCR |= (((options & STM32WB_SYSTEM_OPTION_LSE_MODE_MASK) >> STM32WB_SYSTEM_OPTION_LSE_MODE_SHIFT) << RCC_BDCR_LSEDRV_Pos);

                    RCC->BDCR |= RCC_BDCR_LSEON;

                    /* The loop below take about 8 cycles per iteration. The startup time for
                     * LSE is 5000ms. At 16MHz this corresponds to about 10000000 iterations.
                     */
                    count = 0;
                
                    while (!(RCC->BDCR & RCC_BDCR_LSERDY))
                    {
                        if (++count >= 10000000)
                        {
                            lseclk = 0;
                        
                            RCC->BDCR &= ~RCC_BDCR_LSEON;

                            break;
                        }
                    }
                }
            }
        }

        if (hseclk)
        {
            if (!(options & STM32WB_SYSTEM_OPTION_HSE_BYPASS))
            {
                RCC->CR |= RCC_CR_HSEON;
                
                /* The loop below take about 8 cycles per iteration. The startup time for
                 * HSE is 100ms. At 16MHz this corresponds to about 200000 iterations.
                 */
                count = 0;
                
                while (!(RCC->CR & RCC_CR_HSERDY))
                {
                    if (++count >= 200000)
                    {
                        hseclk = 0;

                        break;
                    }
                }
            }
        }
    
        RCC->CR &= ~RCC_CR_HSEON;
    }

    if (lseclk)
    {
        RCC->CSR = (RCC->CSR & ~(RCC_CSR_RFWKPSEL | RCC_CSR_LSI1ON)) | (RCC_CSR_RFWKPSEL_0);
    }
    else
    {
        RCC->CSR = (RCC->CSR & ~(RCC_CSR_RFWKPSEL | RCC_CSR_LSI1ON)) | (RCC_CSR_LSI1ON | RCC_CSR_RFWKPSEL_0 | RCC_CSR_RFWKPSEL_1);
        
        while (!(RCC->CSR & RCC_CSR_LSI1RDY))
        {
        }
        
        stm32wb_system_device.lsi = 0x80;
    }

    if (stm32wb_otp_read(STM32WB_OTP_ID_HSE_TUNE, (uint8_t*)&hse_tune, sizeof(hse_tune), NULL))
    {
        RCC->HSECR = 0xcafecafe;
        RCC->HSECR = (RCC->HSECR & ~RCC_HSECR_HSETUNE) | (hse_tune.hse_tune << RCC_HSECR_HSETUNE_Pos);
    }

    if (options & STM32WB_SYSTEM_OPTION_VBAT_CHARGING)
    {
        PWR->CR4 |= PWR_CR4_VBE;
    }

    /* Keep SRAM2a alive in STANDBY */
    PWR->CR3 |= PWR_CR3_RRS;

    if (__stm32wb_system_reset_cause != STM32WB_SYSTEM_RESET_CAUSE_WAKEUP)
    {
        /* Punt CPU2 all the way to SHUTDOWN.
         */
        PWR->C2CR1 = (PWR->C2CR1 & ~PWR_C2CR1_LPMS) | PWR_C2CR1_LPMS_SHUTDOWN;
    }
    
    /* Disable SMPS for now ... */
    PWR->CR5 &= ~(PWR_CR5_SMPSEN | PWR_CR5_BORHC);
    
    if (options & (STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_10uH | STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_2_2uH))
    {
        RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSDIV) | ((options & STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_10uH) ? RCC_SMPSCR_SMPSDIV_0 : 0);

        PWR->CR5 = (PWR->CR5 & ~PWR_CR5_SMPSSC) | (((options & STM32WB_SYSTEM_OPTION_SMPS_CURRENT_MASK) >> STM32WB_SYSTEM_OPTION_SMPS_CURRENT_SHIFT) << PWR_CR5_SMPSSC_Pos);
    }
    
    /* Setup default SLEEP mode settings */
    RCC->AHB1SMENR  = (RCC_AHB1SMENR_DMA1SMEN |
                       RCC_AHB1SMENR_DMA2SMEN |
                       RCC_AHB1SMENR_DMAMUX1SMEN |
                       RCC_AHB1SMENR_CRCSMEN |
                       RCC_AHB1SMENR_TSCSMEN);

    RCC->AHB2SMENR  = (RCC_AHB2SMENR_ADCSMEN |
                       RCC_AHB2SMENR_AES1SMEN);

    RCC->AHB3SMENR  = (RCC_AHB3SMENR_QUADSPISMEN |
                       RCC_AHB3SMENR_PKASMEN |
                       RCC_AHB3SMENR_AES2SMEN |
                       RCC_AHB3SMENR_RNGSMEN);
    
    RCC->APB1SMENR1 = (RCC_APB1SMENR1_TIM2SMEN |
                       RCC_APB1SMENR1_LCDSMEN |
                       RCC_APB1SMENR1_SPI2SMEN |
                       RCC_APB1SMENR1_I2C1SMEN |
                       RCC_APB1SMENR1_I2C3SMEN |
                       RCC_APB1SMENR1_CRSSMEN |
                       RCC_APB1SMENR1_USBSMEN |
                       RCC_APB1SMENR1_LPTIM1SMEN);
                       
    RCC->APB1SMENR2 = (RCC_APB1SMENR2_LPUART1SMEN |
                       RCC_APB1SMENR2_LPTIM2SMEN);

    RCC->APB2SMENR  = (RCC_APB2SMENR_TIM1SMEN |
                       RCC_APB2SMENR_SPI1SMEN |
                       RCC_APB2SMENR_USART1SMEN |
                       RCC_APB2SMENR_TIM16SMEN |
                       RCC_APB2SMENR_TIM17SMEN |
                       RCC_APB2SMENR_SAI1SMEN);
    
    /* Setup the independent clocks for the peripherals to HSI16, except
     * LUPART which is LSE. CLK48 is driven by HSI48 with CRS.
     */
    RCC->CCIPR = ((0 |                                               /* RNG is CLK48           */
                   RCC_CCIPR_ADCSEL_1 | RCC_CCIPR_ADCSEL_0 |         /* ADC is SYSCLK          */
                   0 |                                               /* CLK48 is HSI48         */
                   0 |                                               /* SAI1SEL is PLLSAI1 "P" */
                   RCC_CCIPR_LPTIM2SEL_1 |                           /* LPTIM2 is HSI16        */
                   RCC_CCIPR_I2C3SEL_1 |                             /* I2C3 is HSI16          */
                   RCC_CCIPR_I2C1SEL_1 |                             /* I2C1 is HSI16          */
                   RCC_CCIPR_LPUART1SEL_1) |                         /* LPUART1 is HSI16       */
                  ((options & STM32WB_SYSTEM_OPTION_USART1_SYSCLK)
                   ? (RCC_CCIPR_USART1SEL_0)                         /* USART1 is SYSCLK       */
                   : (RCC_CCIPR_USART1SEL_1)) |                      /* USART1 is HSI16        */
                  (lseclk
                   ? (RCC_CCIPR_LPTIM1SEL_1 | RCC_CCIPR_LPTIM1SEL_0) /* LPTIM1 is LSE          */
                   : (RCC_CCIPR_LPTIM1SEL_0)));                      /* LPTIM1 is LSI          */


#if 0    
    EXTI->IMR2 |= EXTI_IMR2_IM48;
    EXTI->C2IMR2 |= EXTI_C2IMR2_IM48;
#endif
    
    EXTI->IMR2 &= ~EXTI_IMR2_IM48;
    EXTI->C2IMR2 &= ~EXTI_C2IMR2_IM48;
    
    stm32wb_system_device.options = options;
    stm32wb_system_device.lseclk = lseclk;
    stm32wb_system_device.hseclk = hseclk;
    stm32wb_system_device.reference = 0;
    stm32wb_system_device.policy = STM32WB_SYSTEM_POLICY_SLEEP;
    stm32wb_system_device.events = 0;
    
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
    {
        DBGMCU->CR       = DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;
        DBGMCU->APB1FZR1 = (DBGMCU_APB1FZR1_DBG_RTC_STOP |
                            DBGMCU_APB1FZR1_DBG_IWDG_STOP |
                            DBGMCU_APB1FZR1_DBG_I2C1_STOP |
                            DBGMCU_APB1FZR1_DBG_I2C3_STOP |
                            DBGMCU_APB1FZR1_DBG_LPTIM1_STOP);
        DBGMCU->APB1FZR2 = (DBGMCU_APB1FZR2_DBG_LPTIM2_STOP);
        DBGMCU->APB2FZR  = 0;

        // RTT in SRAM2
        RCC->AHB3SMENR |= RCC_AHB3SMENR_SRAM2SMEN;

#if 0        
        EXTI->IMR2 |= EXTI_IMR2_IM48;
        EXTI->C2IMR2 |= EXTI_C2IMR2_IM48;
#endif
        
        stm32wb_system_device.reference |= STM32WB_SYSTEM_REFERENCE_SWD;
    }
         
    NVIC_SetPriority(PVD_PVM_IRQn, STM32WB_PVD_PVM_IRQ_PRIORITY);
    NVIC_EnableIRQ(PVD_PVM_IRQn);

#if 0    
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PE2,  (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_ODATA_0 | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PE1,  (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_ODATA_0 | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PE0,  (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_ODATA_0 | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PD12, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_ODATA_0 | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PD13, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_ODATA_0 | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PB5,  (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_ODATA_0 | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PB7,  (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_ODATA_0 | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));
    stm32wb_gpio_pin_configure(STM32WB_GPIO_PIN_PD9,  (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_ODATA_0 | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));

    stm32wb_gpio_pin_write(STM32WB_GPIO_PIN_PE2, 1);
    armv7m_core_udelay(10000);
    stm32wb_gpio_pin_write(STM32WB_GPIO_PIN_PE2, 0);
    armv7m_core_udelay(10000);
    stm32wb_gpio_pin_write(STM32WB_GPIO_PIN_PE2, 1);
#endif
    
    __stm32wb_hsem_initialize();
    __stm32wb_gpio_initialize();
    __stm32wb_exti_initialize();
    __stm32wb_dma_initialize();
    __stm32wb_rtc_initialize();
    __stm32wb_lptim_initialize();

    __stm32wb_adc_initialize();
    
    __enable_irq();
    
    stm32wb_system_sysclk_configure(sysclk, hclk, pclk1, pclk2);

    __stm32wb_flash_initialize();
    __stm32wb_eeprom_initialize();
    __stm32wb_random_initialize();
}

static void stm32wb_system_sysclk_ext_compute(uint32_t sysclk, uint32_t hclk, uint32_t *p_latency_return, uint32_t *p_c2hpre_return, uint32_t *p_shdhpre_return)
{
    uint32_t hclk2, hclk4, latency, c2hpre, shdhpre;

    if      (stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_WIRELESS) { hclk2 = 32000000; }
    else if (stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_CPU2)     { hclk2 = 16000000; }
    else                                                                          { hclk2 = 0;        }

    if      (hclk2 >= (sysclk / 1))   { hclk2 = sysclk / 1;   c2hpre = RCC_EXTCFGR_C2HPRE_DIV1;   }
    else if (hclk2 >= (sysclk / 2))   { hclk2 = sysclk / 2;   c2hpre = RCC_EXTCFGR_C2HPRE_DIV2;   }
    else if (hclk2 >= (sysclk / 4))   { hclk2 = sysclk / 4;   c2hpre = RCC_EXTCFGR_C2HPRE_DIV4;   }
    else if (hclk2 >= (sysclk / 8))   { hclk2 = sysclk / 8;   c2hpre = RCC_EXTCFGR_C2HPRE_DIV8;   }
    else if (hclk2 >= (sysclk / 16))  { hclk2 = sysclk / 16;  c2hpre = RCC_EXTCFGR_C2HPRE_DIV16;  }
    else if (hclk2 >= (sysclk / 32))  { hclk2 = sysclk / 32;  c2hpre = RCC_EXTCFGR_C2HPRE_DIV32;  }
    else if (hclk2 >= (sysclk / 64))  { hclk2 = sysclk / 64;  c2hpre = RCC_EXTCFGR_C2HPRE_DIV64;  }
    else if (hclk2 >= (sysclk / 128)) { hclk2 = sysclk / 128; c2hpre = RCC_EXTCFGR_C2HPRE_DIV128; }
    else if (hclk2 >= (sysclk / 256)) { hclk2 = sysclk / 256; c2hpre = RCC_EXTCFGR_C2HPRE_DIV256; }
    else                              { hclk2 = sysclk / 512; c2hpre = RCC_EXTCFGR_C2HPRE_DIV512; }

    hclk4 = (hclk >= hclk2) ? hclk : hclk2;

    if      (hclk4 >= (sysclk / 1))   { hclk4 = sysclk / 1;   shdhpre = RCC_EXTCFGR_SHDHPRE_DIV1;   }
    else if (hclk4 >= (sysclk / 2))   { hclk4 = sysclk / 2;   shdhpre = RCC_EXTCFGR_SHDHPRE_DIV2;   }
    else if (hclk4 >= (sysclk / 4))   { hclk4 = sysclk / 4;   shdhpre = RCC_EXTCFGR_SHDHPRE_DIV4;   }
    else if (hclk4 >= (sysclk / 8))   { hclk4 = sysclk / 8;   shdhpre = RCC_EXTCFGR_SHDHPRE_DIV8;   }
    else if (hclk4 >= (sysclk / 16))  { hclk4 = sysclk / 16;  shdhpre = RCC_EXTCFGR_SHDHPRE_DIV16;  }
    else if (hclk4 >= (sysclk / 32))  { hclk4 = sysclk / 32;  shdhpre = RCC_EXTCFGR_SHDHPRE_DIV32;  }
    else if (hclk4 >= (sysclk / 64))  { hclk4 = sysclk / 64;  shdhpre = RCC_EXTCFGR_SHDHPRE_DIV64;  }
    else if (hclk4 >= (sysclk / 128)) { hclk4 = sysclk / 128; shdhpre = RCC_EXTCFGR_SHDHPRE_DIV128; }
    else if (hclk4 >= (sysclk / 256)) { hclk4 = sysclk / 256; shdhpre = RCC_EXTCFGR_SHDHPRE_DIV256; }
    else                              { hclk4 = sysclk / 512; shdhpre = RCC_EXTCFGR_SHDHPRE_DIV512; }
    
    if (hclk4 <= 16000000)
    {
        if      (hclk4 <=  60000000) { latency = FLASH_ACR_LATENCY_0WS; }
        else if (hclk4 <= 120000000) { latency = FLASH_ACR_LATENCY_1WS; }
        else                         { latency = FLASH_ACR_LATENCY_2WS; }
    } 
    else
    {
        if      (hclk4 <=  16000000) { latency = FLASH_ACR_LATENCY_0WS; }
        else if (hclk4 <=  32000000) { latency = FLASH_ACR_LATENCY_1WS; }
        else if (hclk4 <=  48000000) { latency = FLASH_ACR_LATENCY_2WS; }
        else                         { latency = FLASH_ACR_LATENCY_3WS; }
    }

    *p_latency_return = latency;
    *p_c2hpre_return  = c2hpre;
    *p_shdhpre_return = shdhpre;
}

static void stm32wb_system_sysclk_ext_configure(uint32_t sysclk, uint32_t hclk)
{
    uint32_t primask, latency, c2hpre, shdhpre;

    stm32wb_system_sysclk_ext_compute(sysclk, hclk, &latency, &c2hpre, &shdhpre);
    
    primask = __get_PRIMASK();
        
    __disable_irq();
        
    while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
    {
        __set_PRIMASK(primask);
        
        __disable_irq();
    }
    
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS;
    FLASH->ACR;

    RCC->EXTCFGR = (RCC->EXTCFGR & ~(RCC_EXTCFGR_SHDHPRE | RCC_EXTCFGR_C2HPRE | RCC_EXTCFGR_SHDHPREF | RCC_EXTCFGR_C2HPREF)) | shdhpre | c2hpre;
    
    while ((RCC->EXTCFGR & (RCC_EXTCFGR_SHDHPREF | RCC_EXTCFGR_C2HPREF)) != (RCC_EXTCFGR_SHDHPREF | RCC_EXTCFGR_C2HPREF))
    {
    }

    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | latency;
    FLASH->ACR;
    
    stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

    __set_PRIMASK(primask);

    stm32wb_system_device.latency = latency;
    stm32wb_system_device.c2hpre = c2hpre;
}

static bool __svc_stm32wb_system_sysclk_configure(uint32_t sysclk, uint32_t hclk, uint32_t pclk1, uint32_t pclk2)
{
    uint32_t primask, pllcfg, msirange, hpre, ppre1, ppre2, c2hpre, shdhpre, latency, smpsvos;
    bool hse, pllsys, smps;

    if (!sysclk)
    {
        sysclk = hclk;
    }
    
    if (stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_WIRELESS)
    {
        /* Use HSE */

        hse = true;
        
        if (sysclk >= 64000000)
        {
            sysclk = 64000000;
            
            msirange = RCC_CR_MSIRANGE_8;

            pllcfg = ((1 << RCC_PLLCFGR_PLLR_Pos) | (64000000 / (4000000 / 2)) << RCC_PLLCFGR_PLLN_Pos) | (((8-1) << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLSRC_HSE);

            pllsys = true;
        }
        else
        {
            sysclk = 32000000;

            msirange = RCC_CR_MSIRANGE_10;
            
            pllcfg = (((8-1) << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLSRC_HSE);
            
            pllsys = false;
        }
    }
    else
    {
        /* Use MSI */

        hse = false;

        if (sysclk >= 64000000)
        {
            sysclk = 64000000;
            
            msirange = RCC_CR_MSIRANGE_8;
                
            pllcfg = ((1 << RCC_PLLCFGR_PLLR_Pos) | (64000000 / (4000000 / 2)) << RCC_PLLCFGR_PLLN_Pos) | (((4-1) << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLSRC_MSI);

            pllsys = true;
        }
        else
        {
            if      (sysclk >= 32000000) { sysclk = 32000000; msirange = RCC_CR_MSIRANGE_10; pllcfg = (((8-1) << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLSRC_MSI); }
            else if (sysclk >= 16000000) { sysclk = 16000000; msirange = RCC_CR_MSIRANGE_8;  pllcfg = (((4-1) << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLSRC_MSI); }
            else                         { sysclk =  2000000; msirange = RCC_CR_MSIRANGE_5;  pllcfg = RCC_PLLCFGR_PLLSRC_NONE;                                    }
            
            pllsys = false;
        }
    }
    
    if      (hclk >= (sysclk / 1))   { hclk = sysclk / 1;   hpre = RCC_CFGR_HPRE_DIV1;   }
    else if (hclk >= (sysclk / 2))   { hclk = sysclk / 2;   hpre = RCC_CFGR_HPRE_DIV2;   }
    else if (hclk >= (sysclk / 4))   { hclk = sysclk / 4;   hpre = RCC_CFGR_HPRE_DIV4;   }
    else if (hclk >= (sysclk / 8))   { hclk = sysclk / 8;   hpre = RCC_CFGR_HPRE_DIV8;   }
    else if (hclk >= (sysclk / 16))  { hclk = sysclk / 16;  hpre = RCC_CFGR_HPRE_DIV16;  }
    else if (hclk >= (sysclk / 32))  { hclk = sysclk / 32;  hpre = RCC_CFGR_HPRE_DIV32;  }
    else if (hclk >= (sysclk / 64))  { hclk = sysclk / 64;  hpre = RCC_CFGR_HPRE_DIV64;  }
    else if (hclk >= (sysclk / 128)) { hclk = sysclk / 128; hpre = RCC_CFGR_HPRE_DIV128; }
    else if (hclk >= (sysclk / 256)) { hclk = sysclk / 256; hpre = RCC_CFGR_HPRE_DIV256; }
    else                             { hclk = sysclk / 512; hpre = RCC_CFGR_HPRE_DIV512; }
    
    if (pclk1)
    {
        if      (pclk1 >= (hclk / 1)) { pclk1 = hclk / 1;  ppre1 = RCC_CFGR_PPRE1_DIV1;  }
        else if (pclk1 >= (hclk / 2)) { pclk1 = hclk / 2;  ppre1 = RCC_CFGR_PPRE1_DIV2;  }
        else if (pclk1 >= (hclk / 4)) { pclk1 = hclk / 4;  ppre1 = RCC_CFGR_PPRE1_DIV4;  }
        else if (pclk1 >= (hclk / 8)) { pclk1 = hclk / 8;  ppre1 = RCC_CFGR_PPRE1_DIV8;  }
        else                          { pclk1 = hclk / 16; ppre1 = RCC_CFGR_PPRE1_DIV16; }
    }
    else
    {
        if      (hclk >= 64000000)    { pclk1 = hclk / 4;  ppre1 = RCC_CFGR_PPRE1_DIV4;  }
        else if (hclk >= 32000000)    { pclk1 = hclk / 2;  ppre1 = RCC_CFGR_PPRE1_DIV2;  }
        else                          { pclk1 = hclk / 1;  ppre1 = RCC_CFGR_PPRE1_DIV1;  }
    }

    if (pclk2)
    {
        if      (pclk2 >= (hclk / 1)) { pclk2 = hclk / 1;  ppre2 = RCC_CFGR_PPRE2_DIV1;  }
        else if (pclk2 >= (hclk / 2)) { pclk2 = hclk / 2;  ppre2 = RCC_CFGR_PPRE2_DIV2;  }
        else if (pclk2 >= (hclk / 4)) { pclk2 = hclk / 4;  ppre2 = RCC_CFGR_PPRE2_DIV4;  }
        else if (pclk2 >= (hclk / 8)) { pclk2 = hclk / 8;  ppre2 = RCC_CFGR_PPRE2_DIV8;  }
        else                          { pclk2 = hclk / 16; ppre2 = RCC_CFGR_PPRE2_DIV16; }
    }
    else
    {
        if      (hclk >= 32000000)    { pclk2 = hclk / 2;  ppre2 = RCC_CFGR_PPRE2_DIV2;  }
        else                          { pclk2 = hclk / 1;  ppre2 = RCC_CFGR_PPRE2_DIV1;  }
    }

    stm32wb_system_sysclk_ext_compute(sysclk, hclk, &latency, &c2hpre, &shdhpre);

    if ((sysclk >= 32000000) && (stm32wb_system_device.options & (STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_10uH | STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_2_2uH)))
    {
        smps = true;
        smpsvos = ((*((const uint32_t*)0x1fff7558) >> 8) & 15);

        if (stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_WIRELESS)
        {
            if      (stm32wb_system_device.palevel == 0x1f) { smpsvos += 5; } /* 1750mV */
            else if (stm32wb_system_device.palevel == 0x1e) { smpsvos += 2; } /* 1600mV */
            else                                            { smpsvos -= 1; } /* 1450mV */
        }
        else
        {
            smpsvos -= 1; /* 1450mV */
        }
    }
    else
    {
        smps = false;
        smpsvos = 8;
    }

    primask = __get_PRIMASK();
    
    __disable_irq();
    
    if (stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_CLOCKS] ||
        (stm32wb_system_device.reference &
         (STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_1 |
          STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_2)) ||
        ((stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_WIRELESS) && (sysclk < 32000000)) ||
        ((stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_CPU2) && (sysclk < 16000000)) ||
        ((stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_USB) && (pclk1 < 16000000)))
    {
        __set_PRIMASK(primask);
        
        return false;
    }

    stm32wb_system_event(STM32WB_SYSTEM_EVENT_CLOCKS_PROLOGUE);

    armv7m_systick_disable();
    
    if (sysclk <= STM32WB_SYSTEM_FREQUENCY_RANGE_2)
    {
        if (sysclk <= STM32WB_SYSTEM_FREQUENCY_LPRUN)
        {
            stm32wb_system_device.reference = stm32wb_system_device.reference & ~(STM32WB_SYSTEM_REFERENCE_SYSCLK_RANGE_1 | STM32WB_SYSTEM_REFERENCE_SYSCLK_RANGE_2);
        }
        else
        {
            stm32wb_system_device.reference = (stm32wb_system_device.reference & ~STM32WB_SYSTEM_REFERENCE_SYSCLK_RANGE_1) | STM32WB_SYSTEM_REFERENCE_SYSCLK_RANGE_2;
        }
    }
    else
    {
        stm32wb_system_device.reference = (stm32wb_system_device.reference & ~STM32WB_SYSTEM_REFERENCE_SYSCLK_RANGE_2) | STM32WB_SYSTEM_REFERENCE_SYSCLK_RANGE_1;
    }

    if (stm32wb_system_device.sysclk < sysclk)
    {
        /* Leave LPrun */
        if (PWR->CR1 & PWR_CR1_LPR)
        {
            PWR->CR1 &= ~PWR_CR1_LPR;
            
            while (PWR->SR2 & PWR_SR2_REGLPF)
            {
            }
        }
        
        /* Leave to Range 2 */
        if ((PWR->CR1 & PWR_CR1_VOS) == PWR_CR1_VOS_RANGE_2)
        {
            PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_RANGE_1;
            
            while (PWR->SR2 & PWR_SR2_VOSF)
            {
            }
        }
    }

    if (stm32wb_system_device.smps && !smps)
    {
        PWR->CR5 = (PWR->CR5 & ~(PWR_CR5_SMPSEN | PWR_CR5_BORHC | PWR_CR5_SMPSVOS)) | (8 << PWR_CR5_SMPSVOS_Pos);
    }

    while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
    {
    }
    
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS;
    FLASH->ACR;
    
    if (stm32wb_system_device.sysclk != sysclk)
    {
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_HPREF | RCC_CFGR_PPRE1F | RCC_CFGR_PPRE2F)) | (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1);

        while ((RCC->CFGR & (RCC_CFGR_HPREF | RCC_CFGR_PPRE1F | RCC_CFGR_PPRE2F)) != (RCC_CFGR_HPREF | RCC_CFGR_PPRE1F | RCC_CFGR_PPRE2F))
        {
        }

        RCC->EXTCFGR = (RCC->EXTCFGR & ~(RCC_EXTCFGR_C2HPRE | RCC_EXTCFGR_C2HPREF)) | RCC_EXTCFGR_C2HPRE_DIV2;

        while ((RCC->EXTCFGR & RCC_EXTCFGR_C2HPREF) != RCC_EXTCFGR_C2HPREF)
        {
        }

        if (stm32wb_system_device.pllsys)
        {
            if (!stm32wb_system_device.hsi16)
            {
                RCC->CR |= RCC_CR_HSION;
                
                while (!(RCC->CR & RCC_CR_HSIRDY))
                {
                }
            }

            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
                
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
            {
            }

            RCC->CR &= ~RCC_CR_PLLON;
            RCC->CR;
        }
        
        if (hse)
        {
            if (!stm32wb_system_device.hse)
            {
                RCC->CR |= RCC_CR_HSEON;

                stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);
                
                while (!(RCC->CR & RCC_CR_HSERDY))
                {
                }

                while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
                {
                }
            }

            RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSSEL) | RCC_SMPSCR_SMPSSEL_HSE;
            
            /* HSI16 is wakeup clock */
            RCC->CFGR |= RCC_CFGR_STOPWUCK;
            
            stm32wb_system_device.hse |= 0x80;
            stm32wb_system_device.msi &= ~0x80;
        }
        else
        {
            RCC->CR = (RCC->CR & ~(RCC_CR_MSIRANGE | RCC_CR_MSIPLLEN)) | msirange | RCC_CR_MSION; 

            stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

            __DSB();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            
            while (!(RCC->CR & RCC_CR_MSIRDY))
            {
            }

            while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
            {
            }
            
            if (stm32wb_system_device.lseclk)
            {
                RCC->CR |= RCC_CR_MSIPLLEN;
            }

            RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSSEL) | RCC_SMPSCR_SMPSSEL_MSI;

            /* Here HSI16 is selected as wakeup clock to allow CSS at some point.
             */

            /* HSI16 is wakeup clock */
            RCC->CFGR |= RCC_CFGR_STOPWUCK;
            
            stm32wb_system_device.hse &= ~0x80;
            stm32wb_system_device.msi |= 0x80;
        }

        RCC->PLLCFGR = pllcfg;

        if (pllsys)
        {
            RCC->CR |= RCC_CR_PLLON;

            stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);
            
            while (!(RCC->CR & RCC_CR_PLLRDY))
            {
            }
    
            while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
            {
            }

            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
            
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
            {
            }
        }
        else
        {
            if (hse)
            {
                RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSE;
                
                while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE)
                {
                }
            }
            else
            {
                RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_MSI;
                
                while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI)
                {
                }
            }
        }
    }
    
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_HPREF | RCC_CFGR_PPRE1F | RCC_CFGR_PPRE2F)) | hpre | ppre1 | ppre2;
    
    while ((RCC->CFGR & (RCC_CFGR_HPREF | RCC_CFGR_PPRE1F | RCC_CFGR_PPRE2F)) != (RCC_CFGR_HPREF | RCC_CFGR_PPRE1F | RCC_CFGR_PPRE2F))
    {
    }
    
    RCC->EXTCFGR = (RCC->EXTCFGR & ~(RCC_EXTCFGR_SHDHPRE | RCC_EXTCFGR_C2HPRE | RCC_EXTCFGR_SHDHPREF | RCC_EXTCFGR_C2HPREF)) | shdhpre | c2hpre;
    
    while ((RCC->EXTCFGR & (RCC_EXTCFGR_SHDHPREF | RCC_EXTCFGR_C2HPREF)) != (RCC_EXTCFGR_SHDHPREF | RCC_EXTCFGR_C2HPREF))
    {
    }

    if (!stm32wb_system_device.hsi16)
    {
        RCC->CR &= ~RCC_CR_HSION;
    }

    if (!stm32wb_system_device.msi)
    {
        RCC->CR &= ~(RCC_CR_MSIPLLEN | RCC_CR_MSION);
    }

    if (!stm32wb_system_device.hse)
    {
        RCC->CR &= ~RCC_CR_HSEON;
    }

    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | latency;
    FLASH->ACR;
    
    stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

    if (smps)
    {
        PWR->CR5 = (PWR->CR5 & ~PWR_CR5_SMPSVOS) | (PWR_CR5_SMPSEN | PWR_CR5_BORHC | (smpsvos << PWR_CR5_SMPSVOS_Pos));
    }
    
    /* Enter Range 2 */
    if (!(stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_RANGE_1))
    {
        if ((PWR->CR1 & PWR_CR1_VOS) == PWR_CR1_VOS_RANGE_1)
        {
            PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_RANGE_2;
        }
    }
    
    /* Enter LPrun */
    if (!(stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_RANGE_1_2))
    {
        PWR->CR1 |= PWR_CR1_LPR;
    }

    SystemCoreClock = hclk;
    
    stm32wb_system_device.sysclk   = sysclk;
    stm32wb_system_device.hclk     = hclk;
    stm32wb_system_device.pclk1    = pclk1;
    stm32wb_system_device.pclk2    = pclk2;

    stm32wb_system_device.pllsys   = pllsys;
    stm32wb_system_device.latency  = latency;
    stm32wb_system_device.smps     = smps;
    stm32wb_system_device.msirange = msirange;
    stm32wb_system_device.c2hpre   = c2hpre;

    armv7m_core_configure();
    armv7m_systick_configure();
    
    armv7m_systick_enable();

    stm32wb_system_event(STM32WB_SYSTEM_EVENT_CLOCKS_EPILOGUE);

    __set_PRIMASK(primask);
    
    return true;
}

static void stm32wb_system_lsi_enable(void)
{
    stm32wb_system_device.lsi++;

    if (stm32wb_system_device.lsi == 1)
    {
        RCC->CSR |= RCC_CSR_LSI1ON;
    
        while (!(RCC->CSR & RCC_CSR_LSI1RDY))
        {
        }
    }
}

static void stm32wb_system_lsi_disable(void)
{
    if (stm32wb_system_device.lsi == 1)
    {
        RCC->CSR &= ~RCC_CSR_LSI1ON;
    }

    stm32wb_system_device.lsi--;
}

static void stm32wb_system_voltage_increase(void)
{
    /* Leave LPrun */
    if (stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_RANGE_1_2)
    {
        if (PWR->CR1 & PWR_CR1_LPR)
        {
            PWR->CR1 &= ~PWR_CR1_LPR;
        
            while (PWR->SR2 & PWR_SR2_REGLPF)
            {
            }
        }
    }
    
    /* Leave Range 2 */
    if (stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_RANGE_1)
    {
        if ((PWR->CR1 & PWR_CR1_VOS) == PWR_CR1_VOS_RANGE_2)
        {
            PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_RANGE_1;
            
            while (PWR->SR2 & PWR_SR2_VOSF)
            {
            }
        }
    }
}

static void stm32wb_system_voltage_decrease(void)
{
    /* Enter Range 2 */
    if (!(stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_RANGE_1))
    {
        if ((PWR->CR1 & PWR_CR1_VOS) == PWR_CR1_VOS_RANGE_1)
        {
            PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_RANGE_2;
        }
    }
    
    /* Enter LPrun */
    if (!(stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_RANGE_1_2))
    {
        PWR->CR1 |= PWR_CR1_LPR;
    }
}

static void stm32wb_system_wireless_enter(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
        
    __disable_irq();
        
    while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
    {
        __set_PRIMASK(primask);
        
        __disable_irq();
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSE;
                
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE)
    {
    }
    
    if (stm32wb_system_device.pllsys)
    {
        RCC->CR &= ~RCC_CR_PLLON;
        RCC->CR;

        RCC->PLLCFGR = ((1 << RCC_PLLCFGR_PLLR_Pos) | (64000000 / (4000000 / 2)) << RCC_PLLCFGR_PLLN_Pos) | (((8-1) << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLSRC_HSE);
        
        RCC->CR |= RCC_CR_PLLON;

        stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

        __set_PRIMASK(primask);
        
        while (!(RCC->CR & RCC_CR_PLLRDY))
        {
        }

        __disable_irq();
        
        while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
        {
            __set_PRIMASK(primask);
            
            __disable_irq();
        }
        
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
        
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        {
        }
    }
    else
    {
        RCC->PLLCFGR = (((8-1) << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLSRC_HSE);
    }

    RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSSEL) | RCC_SMPSCR_SMPSSEL_HSE;
    
    stm32wb_system_device.msi &= ~0x80;
    
    if (!stm32wb_system_device.msi)
    {
        RCC->CR &= ~(RCC_CR_MSIPLLEN | RCC_CR_MSION);
    }

    stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

    __set_PRIMASK(primask);
    
}

static void stm32wb_system_wireless_leave(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
        
    __disable_irq();
        
    while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
    {
        __set_PRIMASK(primask);
        
        __disable_irq();
    }

    RCC->CR = (RCC->CR & ~(RCC_CR_MSIRANGE | RCC_CR_MSIPLLEN)) | stm32wb_system_device.msirange | RCC_CR_MSION; 

    stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

    __set_PRIMASK(primask);

    __DSB();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    
    while (!(RCC->CR & RCC_CR_MSIRDY))
    {
    }

    __disable_irq();
        
    while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
    {
        __set_PRIMASK(primask);
        
        __disable_irq();
    }
    
    if (stm32wb_system_device.lseclk)
    {
        RCC->CR |= RCC_CR_MSIPLLEN;
    }
    
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_MSI;
    
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI)
    {
    }
    
    if (stm32wb_system_device.pllsys)
    {
        RCC->CR &= ~RCC_CR_PLLON;
        RCC->CR;
        
        RCC->PLLCFGR = ((1 << RCC_PLLCFGR_PLLR_Pos) | (64000000 / (4000000 / 2)) << RCC_PLLCFGR_PLLN_Pos) | (((4-1) << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLSRC_MSI);
        
        RCC->CR |= RCC_CR_PLLON;

        stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

        __set_PRIMASK(primask);

        while (!(RCC->CR & RCC_CR_PLLRDY))
        {
        }

        __disable_irq();
        
        while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
        {
            __set_PRIMASK(primask);
            
            __disable_irq();
        }
        
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
        
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        {
        }
    }
    else
    {
        RCC->PLLCFGR = (((8-1) << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLSRC_MSI);
    }
    
    RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSSEL) | RCC_SMPSCR_SMPSSEL_MSI;
    
    stm32wb_system_device.hse &= ~0x80;
    stm32wb_system_device.msi |= 0x80;
    
    if (!stm32wb_system_device.hse)
    {
        RCC->CR &= ~RCC_CR_HSEON;
    }

    stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

    __set_PRIMASK(primask);
}

static bool __svc_stm32wb_system_saiclk_configure(uint32_t saiclk)
{
    uint32_t primask, reference, pllcfg;

    reference = 0;
    pllcfg = 0;
    
    if (saiclk != STM32WB_SYSTEM_SAICLK_NONE)
    {
        if (stm32wb_system_device.sysclk <= STM32WB_SYSTEM_FREQUENCY_RANGE_2)
        {
            /* 4MHz PLL input */
            switch (saiclk) {
            case STM32WB_SYSTEM_SAICLK_11289600:
                reference = STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_2;
                pllcfg = (((11 -1) << RCC_PLLSAI1CFGR_PLLP_Pos) | (31 << RCC_PLLSAI1CFGR_PLLN_Pos) | RCC_PLLSAI1CFGR_PLLPEN);
                break;

            case STM32WB_SYSTEM_SAICLK_24576000:
                reference = STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_2;
                pllcfg = (((5 -1) << RCC_PLLSAI1CFGR_PLLP_Pos) | (31 << RCC_PLLSAI1CFGR_PLLN_Pos) | RCC_PLLSAI1CFGR_PLLPEN);
                break;

            default:
                return false;
            }
        }
        else
        {
            /* 4MHz PLL input */
            switch (saiclk) {
            case STM32WB_SYSTEM_SAICLK_11289600:
                reference = STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_1;
                pllcfg = (((17 -1) << RCC_PLLSAI1CFGR_PLLP_Pos) | (48 << RCC_PLLSAI1CFGR_PLLN_Pos) | RCC_PLLSAI1CFGR_PLLPEN);
                break;
                    
            case STM32WB_SYSTEM_SAICLK_24576000:
                reference = STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_1;
                pllcfg = (((7 -1) << RCC_PLLSAI1CFGR_PLLP_Pos) | (43 << RCC_PLLSAI1CFGR_PLLN_Pos) | RCC_PLLSAI1CFGR_PLLPEN);
                break;
                    
            default:
                return false;
            }
        }
            
    }

    stm32wb_system_unreference(STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_1 | STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_2);

    if (reference)
    {
        stm32wb_system_reference(reference);
    }
        
    primask = __get_PRIMASK();
            
    __disable_irq();
            
    while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
    {
        __set_PRIMASK(primask);
            
        __disable_irq();
    }
        
    RCC->CR &= ~RCC_CR_PLLSAI1ON;
    RCC->CR;
        
    stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);
        
    __set_PRIMASK(primask);
        
    if (stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_WIRELESS)
    {
        if (stm32wb_system_device.msi & 0x80)
        {
            stm32wb_system_wireless_enter();
        }
    }
    else
    {
        if (stm32wb_system_device.hse & 0x80)
        {
            stm32wb_system_wireless_leave();
        }
    }

    if (saiclk != STM32WB_SYSTEM_SAICLK_NONE)
    {
        if (stm32wb_system_device.saiclk < saiclk)
        {
            stm32wb_system_voltage_increase();
        }
            
        primask = __get_PRIMASK();
            
        __disable_irq();
            
        while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
        {
            __set_PRIMASK(primask);
                
            __disable_irq();
        }

        RCC->PLLSAI1CFGR = pllcfg;

        RCC->CR |= RCC_CR_PLLSAI1ON;

        stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

        __set_PRIMASK(primask);
            
        while (!(RCC->CR & RCC_CR_PLLSAI1RDY))
        {
        }

        if (stm32wb_system_device.saiclk > saiclk)
        {
            stm32wb_system_voltage_decrease();
        }
    }
    else
    {
        stm32wb_system_voltage_decrease();
    }
        
    stm32wb_system_device.saiclk = saiclk;

    return true;
}

static bool __svc_stm32wb_system_mco_configure(uint32_t mco)
{
    uint32_t primask, mcosel, mcopre;
    
    switch (mco & STM32WB_SYSTEM_MCO_SOURCE_MASK) {
    case STM32WB_SYSTEM_MCO_SOURCE_NONE:
    case STM32WB_SYSTEM_MCO_SOURCE_SYSCLK:
    case STM32WB_SYSTEM_MCO_SOURCE_MSI:
    case STM32WB_SYSTEM_MCO_SOURCE_HSE:
    case STM32WB_SYSTEM_MCO_SOURCE_PLL:
    case STM32WB_SYSTEM_MCO_SOURCE_LSE:
    case STM32WB_SYSTEM_MCO_SOURCE_HSI48:
        break;
        
    case STM32WB_SYSTEM_MCO_SOURCE_LSI:
        stm32wb_system_lsi_enable();
        break;
        
    case STM32WB_SYSTEM_MCO_SOURCE_HSI16:
        stm32wb_system_hsi16_enable();
        break;
    }

    mcosel = (mco & STM32WB_SYSTEM_MCO_SOURCE_MASK) >> STM32WB_SYSTEM_MCO_SOURCE_SHIFT;
    mcopre = (mco & STM32WB_SYSTEM_MCO_DIVIDE_MASK) >> STM32WB_SYSTEM_MCO_DIVIDE_SHIFT;

    primask = __get_PRIMASK();
            
    __disable_irq();
            
    while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
    {
        __set_PRIMASK(primask);
        
        __disable_irq();
    }

    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE)) | (((mcosel << RCC_CFGR_MCOSEL_Pos) | (mcopre << RCC_CFGR_MCOPRE_Pos)));

    stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

    __set_PRIMASK(primask);
    
    switch (stm32wb_system_device.mco & STM32WB_SYSTEM_MCO_SOURCE_MASK) {
    case STM32WB_SYSTEM_MCO_SOURCE_NONE:
    case STM32WB_SYSTEM_MCO_SOURCE_SYSCLK:
    case STM32WB_SYSTEM_MCO_SOURCE_MSI:
    case STM32WB_SYSTEM_MCO_SOURCE_HSE:
    case STM32WB_SYSTEM_MCO_SOURCE_PLL:
    case STM32WB_SYSTEM_MCO_SOURCE_LSE:
    case STM32WB_SYSTEM_MCO_SOURCE_HSI48:
        break;
        
    case STM32WB_SYSTEM_MCO_SOURCE_LSI:
        stm32wb_system_lsi_disable();
        break;
        
    case STM32WB_SYSTEM_MCO_SOURCE_HSI16:
        stm32wb_system_hsi16_disable();
        break;
    }
    
    stm32wb_system_device.mco = mco;

    return true;
}

static bool __svc_stm32wb_system_lsco_configure(uint32_t lsco)
{
    if (lsco == STM32WB_SYSTEM_LSCO_SOURCE_LSI)
    {
        if (stm32wb_system_device.lsco != STM32WB_SYSTEM_LSCO_SOURCE_LSI)
        {
            stm32wb_system_lsi_enable();
        }
        
        RCC->BDCR = (RCC->BDCR & ~(RCC_BDCR_LSCOEN | RCC_BDCR_LSCOSEL)) | (RCC_BDCR_LSCOEN | RCC_BDCR_LSCOSEL);
    }
    else
    {
        if (lsco == STM32WB_SYSTEM_LSCO_SOURCE_LSE)
        {
            RCC->BDCR = (RCC->BDCR & ~(RCC_BDCR_LSCOEN | RCC_BDCR_LSCOSEL)) | (RCC_BDCR_LSCOEN);
        }
        else
        {
            RCC->BDCR = (RCC->BDCR & ~(RCC_BDCR_LSCOEN | RCC_BDCR_LSCOSEL));
        }

        if (stm32wb_system_device.lsco == STM32WB_SYSTEM_LSCO_SOURCE_LSI)
        {
            stm32wb_system_lsi_disable();
        }
    }
    
    stm32wb_system_device.lsco = lsco;

    return true;
}

static bool __svc_stm32wb_system_smps_configure(uint32_t palevel)
{
    uint32_t smpsvos;
    
    stm32wb_system_device.palevel = palevel;

    if (stm32wb_system_device.smps)
    {
        if (stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_WIRELESS)
        {
            smpsvos = ((*((const uint32_t*)0x1fff7558) >> 8) & 15);

            if      (palevel == 0x1f) { smpsvos += 5; } /* 1750mV */
            else if (palevel == 0x1e) { smpsvos += 2; } /* 1600mV */
            else                      { smpsvos -= 1; } /* 1450mV */

            PWR->CR5 = (PWR->CR5 & ~PWR_CR5_SMPSVOS) | (PWR_CR5_SMPSEN | PWR_CR5_BORHC | (smpsvos << PWR_CR5_SMPSVOS_Pos));
        }
    }

    return true;
}

static bool __svc_stm32wb_system_vref_configure(uint32_t vref)
{
    if (!(stm32wb_system_device.options & STM32WB_SYSTEM_OPTION_VREF) || (vref > STM32WB_SYSTEM_VREF_2V5))
    {
        return false;
    }

    VREFBUF->CSR = VREFBUF_CSR_HIZ;

    if (vref != STM32WB_SYSTEM_VREF_EXTERNAL)
    {
        if (vref == STM32WB_SYSTEM_VREF_2V0)
        {
            VREFBUF->CCR = *((const uint8_t*)0x1fff75f0);
        }
        else
        {
            VREFBUF->CCR = *((const uint8_t*)0x1fff7530);
            VREFBUF->CSR |= VREFBUF_CSR_VRS;
        }

        VREFBUF->CSR = (VREFBUF->CSR & ~VREFBUF_CSR_HIZ) | VREFBUF_CSR_ENVR;

        while (!(VREFBUF->CSR & VREFBUF_CSR_VRR))
        {
        }
    }

    return true;
}


static bool __svc_stm32wb_system_cpu2_boot(void)
{
    if (stm32wb_system_device.sysclk < 16000000)
    {
        return false;
    }

    if (armv7m_atomic_or(&stm32wb_system_device.reference, STM32WB_SYSTEM_REFERENCE_CPU2) & STM32WB_SYSTEM_REFERENCE_CPU2)
    {
        return true;
    }
   
    stm32wb_system_sysclk_ext_configure(stm32wb_system_device.sysclk, stm32wb_system_device.hclk);

    PWR->C2CR1 &= ~PWR_C2CR1_LPMS;

    PWR->CR4 |= PWR_CR4_C2BOOT;
    PWR->CR4;

    return true;
}

static bool __svc_stm32wb_system_wireless_enable(void)
{
    uint32_t primask, smpsvos;
    
    if (!stm32wb_system_device.lseclk || !stm32wb_system_device.hseclk || (stm32wb_system_device.sysclk < 32000000))
    {
        return false;
    }

    if (!(stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_CPU2))
    {
        return false;
    }
    
    if (armv7m_atomic_or(&stm32wb_system_device.reference, STM32WB_SYSTEM_REFERENCE_WIRELESS) & STM32WB_SYSTEM_REFERENCE_WIRELESS)
    {
        return true;
    }

    stm32wb_system_sysclk_ext_configure(stm32wb_system_device.sysclk, stm32wb_system_device.hclk);
    
    if (stm32wb_system_device.smps)
    {
        smpsvos = ((*((const uint32_t*)0x1fff7558) >> 8) & 15);

        if      (stm32wb_system_device.palevel == 0x1f) { smpsvos += 5; } /* 1750mV */
        else if (stm32wb_system_device.palevel == 0x1e) { smpsvos += 2; } /* 1600mV */
        else                                            { smpsvos -= 1; } /* 1450mV */

        PWR->CR5 = (PWR->CR5 & ~PWR_CR5_SMPSVOS) | (PWR_CR5_SMPSEN | PWR_CR5_BORHC | (smpsvos << PWR_CR5_SMPSVOS_Pos));
    }

    if (!stm32wb_system_device.hse)
    {
        primask = __get_PRIMASK();
        
        __disable_irq();
        
        while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
        {
            __set_PRIMASK(primask);
            
            __disable_irq();
        }
        
        RCC->CR |= RCC_CR_HSEON;

        stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

        __set_PRIMASK(primask);
        
        while (!(RCC->CR & RCC_CR_HSERDY))
        {
        }
    }

    stm32wb_system_device.hse |= 0x80;

    if (stm32wb_system_device.saiclk == STM32WB_SYSTEM_SAICLK_NONE)
    {
        stm32wb_system_wireless_enter();
    }
    
    stm32wb_system_lock(STM32WB_SYSTEM_LOCK_STOP_2);
    
    return true;
}

static bool __svc_stm32wb_system_wireless_disable(void)
{
    uint32_t smpsvos;

    if (!(armv7m_atomic_and(&stm32wb_system_device.reference, ~STM32WB_SYSTEM_REFERENCE_WIRELESS) & STM32WB_SYSTEM_REFERENCE_WIRELESS))
    {
        return true;
    }

    stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_STOP_2);

    if (stm32wb_system_device.saiclk == STM32WB_SYSTEM_SAICLK_NONE)
    {
        stm32wb_system_wireless_leave();
    }
    
    if (stm32wb_system_device.smps)
    {
        smpsvos = ((*((const uint32_t*)0x1fff7558) >> 8) & 15);

        smpsvos -= 1; /* 1450mV */

        PWR->CR5 = (PWR->CR5 & ~PWR_CR5_SMPSVOS) | (PWR_CR5_SMPSEN | PWR_CR5_BORHC | (smpsvos << PWR_CR5_SMPSVOS_Pos));
    }

    stm32wb_system_sysclk_ext_configure(stm32wb_system_device.sysclk, stm32wb_system_device.hclk);
    
    return true;
}

static bool __svc_stm32wb_system_pvm1_enable(stm32wb_system_pvm1_callback_t callback, void *context)
{
    armv7m_atomic_and(&EXTI->IMR1, ~EXTI_IMR1_IM31);

    EXTI->PR1 = EXTI_PR1_PIF31;

    PWR->CR2 |= PWR_CR2_PVME1;

    /* allow PVM / VREFINT to power up */
    armv7m_core_udelay(20);

    stm32wb_system_device.pvm1_callback = callback;
    stm32wb_system_device.pvm1_context  = context;

    armv7m_atomic_or(&EXTI->RTSR1, EXTI_RTSR1_RT31);
    armv7m_atomic_or(&EXTI->FTSR1, EXTI_FTSR1_FT31);
    armv7m_atomic_or(&EXTI->IMR1, EXTI_IMR1_IM31);

    return true;
}

static bool __svc_stm32wb_system_pvm1_disable(void)
{
    armv7m_atomic_and(&EXTI->IMR1, ~EXTI_IMR1_IM31);

    EXTI->PR1 = EXTI_PR1_PIF31;

    PWR->CR2 &= ~PWR_CR2_PVME1;

    stm32wb_system_device.pvm1_callback = NULL;
    stm32wb_system_device.pvm1_context = NULL;

    return true;
}

static bool __svc_stm32wb_system_clk48_enable(void)
{
    stm32wb_system_device.clk48++;

    if (stm32wb_system_device.clk48 == 1)
    {
        stm32wb_system_voltage_increase();
        
        RCC->CRRCR |= RCC_CRRCR_HSI48ON;
        
        while(!(RCC->CRRCR & RCC_CRRCR_HSI48RDY))
        {
        }
        
        stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_CLK48, STM32WB_HSEM_PROCID_NONE);
            
        stm32wb_system_reference(STM32WB_SYSTEM_REFERENCE_HSI48);

        stm32wb_system_lock(STM32WB_SYSTEM_LOCK_SLEEP);
    }
    
    return true;
}

static bool __svc_stm32wb_system_clk48_disable(void)
{
    if (stm32wb_system_device.clk48 == 1)
    {
        stm32wb_system_unlock(STM32WB_SYSTEM_LOCK_SLEEP);

        stm32wb_system_unreference(STM32WB_SYSTEM_REFERENCE_HSI48);

        stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_CLK48, STM32WB_HSEM_PROCID_NONE);
        
        RCC->CRRCR &= ~RCC_CRRCR_HSI48ON;

        stm32wb_system_voltage_decrease();
    }

    stm32wb_system_device.clk48--;

    return true;
}

static bool __svc_stm32wb_system_boost_enable(void)
{
    stm32wb_system_device.boost++;

    if (stm32wb_system_device.boost == 1)
    {
        if (stm32wb_system_info.pins.boost != STM32WB_GPIO_PIN_NONE)
        {
            stm32wb_gpio_pin_write(stm32wb_system_info.pins.boost, 1);

            armv7m_core_udelay(2000);
        }
    }
    
    return true;
}

static bool __svc_stm32wb_system_boost_disable(void)
{
    if (stm32wb_system_device.boost == 1)
    {
        if (stm32wb_system_info.pins.boost != STM32WB_GPIO_PIN_NONE)
        {
            stm32wb_gpio_pin_write(stm32wb_system_info.pins.boost, 0);
        }
    }

    stm32wb_system_device.boost--;

    return true;
}

/*****************************************************************************************************/

bool stm32wb_system_sysclk_configure(uint32_t sysclk, uint32_t hclk, uint32_t pclk1, uint32_t pclk2)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_4((uint32_t)&__svc_stm32wb_system_sysclk_configure, sysclk, hclk, pclk1, pclk2);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_sysclk_configure(sysclk, hclk, pclk1, pclk2);
    }

    return false;
}

bool stm32wb_system_saiclk_configure(uint32_t saiclk)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_1((uint32_t)&__svc_stm32wb_system_saiclk_configure, saiclk);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_saiclk_configure(saiclk);
    }

    return false;
}

bool stm32wb_system_mco_configure(uint32_t mco)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_1((uint32_t)&__svc_stm32wb_system_mco_configure, mco);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_mco_configure(mco);
    }

    return false;
}

bool stm32wb_system_lsco_configure(uint32_t lsco)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_1((uint32_t)&__svc_stm32wb_system_lsco_configure, lsco);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_lsco_configure(lsco);
    }

    return false;
}

bool stm32wb_system_smps_configure(uint32_t palevel)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_1((uint32_t)&__svc_stm32wb_system_smps_configure, palevel);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_smps_configure(palevel);
    }

    return false;
}

bool stm32wb_system_vref_configure(uint32_t vref)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_1((uint32_t)&__svc_stm32wb_system_vref_configure, vref);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_vref_configure(vref);
    }

    return false;
}

bool stm32wb_system_cpu2_boot(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_system_cpu2_boot);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_cpu2_boot();
    }

    return false;
}

bool stm32wb_system_wireless_enable(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_system_wireless_enable);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_wireless_enable();
    }

    return false;
}

bool stm32wb_system_wireless_disable(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_system_wireless_disable);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_wireless_disable();
    }

    return false;
}

bool stm32wb_system_pvm1_enable(stm32wb_system_pvm1_callback_t callback, void *context)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_2((uint32_t)&__svc_stm32wb_system_pvm1_enable, (uint32_t)callback, (uint32_t)context);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_pvm1_enable(callback, context);
    }

    return false;
}

bool stm32wb_system_pvm1_disable(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_system_pvm1_disable);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_pvm1_disable();
    }

    return false;
}

bool stm32wb_system_pvm1_sense(void)
{
    return !(PWR->SR2 & PWR_SR2_PVMO1);
}

bool stm32wb_system_clk48_enable(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_system_clk48_enable);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_clk48_enable();
    }

    return false;
}

bool stm32wb_system_clk48_disable(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_system_clk48_disable);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_clk48_disable();
    }

    return false;
}

void stm32wb_system_hsi16_enable(void)
{
    uint32_t primask;

    armv7m_atomic_incb(&stm32wb_system_device.hsi16);
    
    if (!(stm32wb_system_device.hsi16 & 0x80))
    {
        primask = __get_PRIMASK();
        
        __disable_irq();
        
        while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
        {
            __set_PRIMASK(primask);
            
            __disable_irq();
        }

        RCC->CR |= RCC_CR_HSION;
        
        stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

        __set_PRIMASK(primask);
            
        while (!(RCC->CR & RCC_CR_HSIRDY))
        {
        }
    }
}

void stm32wb_system_hsi16_disable(void)
{
    uint32_t primask;

    armv7m_atomic_incb(&stm32wb_system_device.hsi16);
    
    if (!(stm32wb_system_device.hsi16 & 0x80))
    {
        primask = __get_PRIMASK();
        
        __disable_irq();
        
        while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
        {
            __set_PRIMASK(primask);
            
            __disable_irq();
        }

        if (!stm32wb_system_device.hsi16)
        {
            RCC->CR &= ~RCC_CR_HSION;
        }
        
        stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

        __set_PRIMASK(primask);
    }
}

bool stm32wb_system_boost_enable(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_system_boost_enable);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_boost_enable();
    }

    return false;
}

bool stm32wb_system_boost_disable(void)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_0((uint32_t)&__svc_stm32wb_system_boost_disable);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_system_boost_disable();
    }

    return false;
}

uint32_t stm32wb_system_reset_cause(void)
{
    armv7m_atomic_modify(&RTC->BKP16R, STM32WB_RTC_BKP16R_FATAL, (STM32WB_RTC_BKP16R_FATAL << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT));
    
    return __stm32wb_system_reset_cause;
}

uint32_t stm32wb_system_reset_reason(void)
{
    return __stm32wb_system_reset_reason;
}

uint32_t stm32wb_system_wakeup_reason(void)
{
    return __stm32wb_system_wakeup_reason;
}

uint32_t stm32wb_system_options(void)
{
    return stm32wb_system_device.options;
}

uint32_t stm32wb_system_lseclk(void)
{
    return stm32wb_system_device.lseclk;
}

uint32_t stm32wb_system_hseclk(void)
{
    return stm32wb_system_device.hseclk;
}

uint32_t stm32wb_system_sysclk(void)
{
    return stm32wb_system_device.sysclk;
}

uint32_t stm32wb_system_hclk(void)
{
    return stm32wb_system_device.hclk;
}

uint32_t stm32wb_system_pclk1(void)
{
    return stm32wb_system_device.pclk1;
}

uint32_t stm32wb_system_pclk2(void)
{
    return stm32wb_system_device.pclk2;
}

uint32_t stm32wb_system_saiclk(void)
{
    return stm32wb_system_device.saiclk;
}

bool stm32wb_system_cpu2(void)
{
    return !!(PWR->CR4 & PWR_CR4_C2BOOT);
}

bool stm32wb_system_wireless(void)
{
    return (stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_WIRELESS);
}

void stm32wb_system_uid(uint32_t *uid)
{
    uid[0] = *((const uint32_t*)(UID_BASE + 0x00));
    uid[1] = *((const uint32_t*)(UID_BASE + 0x04));
    uid[2] = *((const uint32_t*)(UID_BASE + 0x08));
}

void stm32wb_system_periph_reset(uint32_t periph)
{
    __armv7m_atomic_or(stm32wb_system_xlate_RSTR[periph], stm32wb_system_xlate_RSTMSK[periph]);
    __armv7m_atomic_and(stm32wb_system_xlate_RSTR[periph], ~stm32wb_system_xlate_RSTMSK[periph]);
    *stm32wb_system_xlate_RSTR[periph];
}

void stm32wb_system_periph_enable(uint32_t periph)
{
    __armv7m_atomic_or(stm32wb_system_xlate_ENR[periph], stm32wb_system_xlate_ENMSK[periph]);
    *stm32wb_system_xlate_ENR[periph];
}

void stm32wb_system_periph_disable(uint32_t periph)
{
    __armv7m_atomic_and(stm32wb_system_xlate_ENR[periph], ~stm32wb_system_xlate_ENMSK[periph]);
    *stm32wb_system_xlate_ENR[periph];
}

void stm32wb_system_swd_enable(void)
{
    DBGMCU->CR       = DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;
    DBGMCU->APB1FZR1 = (DBGMCU_APB1FZR1_DBG_RTC_STOP |
                        DBGMCU_APB1FZR1_DBG_IWDG_STOP |
                        DBGMCU_APB1FZR1_DBG_I2C1_STOP |
                        DBGMCU_APB1FZR1_DBG_I2C3_STOP |
                        DBGMCU_APB1FZR1_DBG_LPTIM1_STOP);
    DBGMCU->APB1FZR2 = (DBGMCU_APB1FZR2_DBG_LPTIM2_STOP);
    DBGMCU->APB2FZR  = 0;

    // RTT in SRAM2
    armv7m_atomic_or(&RCC->AHB3SMENR, RCC_AHB3SMENR_SRAM2SMEN);
    
#if 0        
    armv7m_atomic_or(&EXTI->IMR2, EXTI_IMR2_IM48);
    armv7m_atomic_or(&EXTI->C2IMR2, EXTI_C2IMR2_IM48);
#endif
    
    __stm32wb_gpio_swd_enable();

    stm32wb_system_reference(STM32WB_SYSTEM_REFERENCE_SWD);
}

void stm32wb_system_swd_disable(void)
{
    stm32wb_system_unreference(STM32WB_SYSTEM_REFERENCE_SWD);

    __stm32wb_gpio_swd_disable();

#if 0        
    armv7m_atomic_and(&EXTI->IMR2, ~EXTI_IMR2_IM48);
    armv7m_atomic_and(&EXTI->C2IMR2, ~EXTI_C2IMR2_IM48);
#endif
    
    // RTT in SRAM2
    armv7m_atomic_and(&RCC->AHB3SMENR, ~RCC_AHB3SMENR_SRAM2SMEN);
    
    DBGMCU->CR       = 0;
    DBGMCU->APB1FZR1 = 0;
    DBGMCU->APB1FZR2 = 0;
    DBGMCU->APB2FZR  = 0;
}

void stm32wb_system_notify(stm32wb_system_notify_t *notify, stm32wb_system_callback_t callback, void *context, uint32_t mask)
{
    stm32wb_system_notify_t **pp_entry, *entry;

    do
    {
        for (pp_entry = &stm32wb_system_device.notify, entry = *pp_entry; entry; pp_entry = &entry->next, entry = *pp_entry)
        {
            if (entry == notify)
            {
                goto found;
            }
        }
        
        notify->next = NULL;
    } while (armv7m_atomic_cas((volatile uint32_t*)pp_entry, (uint32_t)NULL, (uint32_t)notify) != (uint32_t)NULL);

found:    
    notify->callback = callback;
    notify->context = context;
    notify->mask = mask;
}

static void stm32wb_system_event(uint32_t event)
{
    stm32wb_system_notify_t *entry;

    for (entry = stm32wb_system_device.notify; entry; entry = entry->next)
    {
        if (entry->mask & event)
        {
            (*entry->callback)(entry->context, entry->mask & event);
        }
    }
}

void stm32wb_system_lock(uint32_t lock)
{
    __armv7m_atomic_incb(&stm32wb_system_device.lock[lock]);
}

void stm32wb_system_unlock(uint32_t lock)
{
    __armv7m_atomic_decb(&stm32wb_system_device.lock[lock]);
}

void stm32wb_system_reference(uint32_t reference)
{
    __armv7m_atomic_or(&stm32wb_system_device.reference, reference);
}

void stm32wb_system_unreference(uint32_t reference)
{
    __armv7m_atomic_and(&stm32wb_system_device.reference, ~reference);
}

uint32_t stm32wb_system_policy(uint32_t policy)
{
    if (policy == STM32WB_SYSTEM_POLICY_CURRENT)
    {
        return stm32wb_system_device.policy;
    }
    else
    {
        if (policy > STM32WB_SYSTEM_POLICY_STOP)
        {
            policy = STM32WB_SYSTEM_POLICY_STOP;
        }
        
        return armv7m_atomic_swapb(&stm32wb_system_device.policy, policy);
    }
}

void stm32wb_system_sleep()
{
    uint32_t policy, primask, lpms;
    bool hsi16;

    if (!stm32wb_system_device.events)
    {
        policy = stm32wb_system_device.policy;

        if (policy <= STM32WB_SYSTEM_POLICY_RUN)
        {
            while (!stm32wb_system_device.events)
            {
                __NOP();
            }
        }
        else
        {
            if (policy <= STM32WB_SYSTEM_POLICY_SLEEP)
            {
                while (!stm32wb_system_device.events)
                {
                    __WFE();
                }
            }
            else
            {
                stm32wb_system_event(STM32WB_SYSTEM_EVENT_STOP_PREPARE);

                while (!stm32wb_system_device.events)
                {
                    primask = __get_PRIMASK();

                    __disable_irq();

                    if (stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_SLEEP])
                    {
                        __WFI();
                    }
                    else
                    {
                        stm32wb_system_event(STM32WB_SYSTEM_EVENT_STOP_ENTER);

                        if (!(SCB->ICSR & SCB_ICSR_VECTPENDING_Msk))
                        {
                            armv7m_systick_disable();

                            __stm32wb_gpio_stop_enter();

                            if (stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_STOP_0])
                            {
                                lpms = PWR_CR1_LPMS_STOP0;
                            }
                            else
                            {
                                if (stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_STOP_1] || !(stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_RANGE_1_2))
                                {
                                    lpms = PWR_CR1_LPMS_STOP1;
                                }
                                else
                                {
                                    lpms = PWR_CR1_LPMS_STOP2;
                                }
                            }

                            PWR->CR1 = (PWR->CR1 & ~PWR_CR1_LPMS) | lpms;

                            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
                                        
                            if (!(SCB->ICSR & SCB_ICSR_VECTPENDING_Msk))
                            {
                                while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
                                {
                                }

                                if (stm32wb_system_device.reference & STM32WB_SYSTEM_REFERENCE_WIRELESS)
                                {
                                    hsi16 = !stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_DEEPSLEEP, STM32WB_HSEM_PROCID_NONE);
                                }
                                else
                                {
                                    hsi16 = true;
                                }
                                        
                                if (hsi16)
                                {
                                    if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
                                    {
                                        if (!(RCC->CR & RCC_CR_HSION))
                                        {
                                            RCC->CR |= RCC_CR_HSION;
                                            
                                            while (!(RCC->CR & RCC_CR_HSIRDY))
                                            {
                                            }
                                        }
                                        
                                        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
                                        
                                        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
                                        {
                                        }
                                        
                                        RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSSEL) | RCC_SMPSCR_SMPSSEL_HSI;

                                        FLASH->ACR = FLASH->ACR & ~FLASH_ACR_LATENCY;
                                    }
                                }

                                if (stm32wb_system_device.msi)
                                {
                                    RCC->CR &= ~RCC_CR_MSIPLLEN;
                                }
                                        
                                if (lpms == PWR_CR1_LPMS_STOP0)
                                {
                                    RCC->CR |= RCC_CR_HSIKERON;
                                            
                                    while (!(RCC->CR & RCC_CR_HSIKERDY))
                                    {
                                    }
                                }

                                stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

                                __DSB();
                                __WFI();
                                __NOP();
                                __NOP();
                                __NOP();
                                __NOP();
                                        
                                stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_DEEPSLEEP, STM32WB_HSEM_PROCID_NONE);
                                        
                                while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
                                {
                                }
                                
                                FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | stm32wb_system_device.latency;
                                        
                                if (stm32wb_system_device.hse)
                                {
                                    if (!(RCC->CR & RCC_CR_HSEON))
                                    {
                                        RCC->CR |= RCC_CR_HSEON;
                                                
                                        while (!(RCC->CR & RCC_CR_HSERDY))
                                        {
                                        }
                                    }

                                    if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE)
                                    {
                                        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSE;
                                        
                                        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE)
                                        {
                                        }
                                    }
                                    
                                    RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSSEL) | RCC_SMPSCR_SMPSSEL_HSE;
                                }
                                else
                                {
                                    if (!(RCC->CR & RCC_CR_MSION))
                                    {
                                        RCC->CR |= RCC_CR_MSION;
                                                
                                        while (!(RCC->CR & RCC_CR_MSIRDY))
                                        {
                                        }
                                    }
                                                
                                    if (stm32wb_system_device.lseclk)
                                    {
                                        RCC->CR |= RCC_CR_MSIPLLEN;
                                    }
                                    
                                    if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI)
                                    {
                                        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_MSI;
                                        
                                        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI)
                                        {
                                        }
                                    }
                                    
                                    RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSSEL) | RCC_SMPSCR_SMPSSEL_MSI;
                                }
                                
                                if (stm32wb_system_device.pllsys)
                                {
                                    RCC->CR |= RCC_CR_PLLON;
                                }

                                stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

                                /* Give CM0+ a chance to deal with radio activity with the minimal HSE wakeup time.
                                 */

                                if (stm32wb_system_device.pllsys)
                                {
                                    while (!(RCC->CR & RCC_CR_PLLRDY))
                                    {
                                    }
                                }
                                
                                while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
                                {
                                }
                                
                                RCC->EXTCFGR = (RCC->EXTCFGR & ~(RCC_EXTCFGR_C2HPRE | RCC_EXTCFGR_C2HPREF)) | stm32wb_system_device.c2hpre;
                                
                                while ((RCC->EXTCFGR & RCC_EXTCFGR_C2HPREF) != RCC_EXTCFGR_C2HPREF)
                                {
                                }
                                
                                if (stm32wb_system_device.pllsys)
                                {
                                    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
                                    
                                    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
                                    {
                                    }
                                }

                                if (lpms == PWR_CR1_LPMS_STOP0)
                                {
                                    RCC->CR &= ~RCC_CR_HSIKERON;
                                }
                                        
                                if (!stm32wb_system_device.hsi16)
                                {
                                    RCC->CR &= ~RCC_CR_HSION;
                                }
                                        
                                stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

                                PWR->EXTSCR = PWR_EXTSCR_C1CSSF;
                            }

                            SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

                            __stm32wb_gpio_stop_leave();

                            armv7m_systick_enable();
                        }
                                
                        stm32wb_system_event(STM32WB_SYSTEM_EVENT_STOP_LEAVE);
                    }
                              
                    __set_PRIMASK(primask);
                }
            }
        }
    }

    stm32wb_system_device.events = 0;
}

void stm32wb_system_wakeup(void)
{
    stm32wb_system_device.events = 1;
}

static __attribute__((noreturn)) void stm32wb_system_halt(uint32_t wakeup, bool shutdown)
{
    while (!stm32wb_hsem_trylock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE))
    {
    }

    if (!stm32wb_system_device.hsi16)
    {
        RCC->CR |= RCC_CR_HSION;
        
        while (!(RCC->CR & RCC_CR_HSIRDY))
        {
        }
    }
    
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {
    }

    RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSSEL) | RCC_SMPSCR_SMPSSEL_HSI;

    if (stm32wb_system_device.msi)
    {
        RCC->CR &= ~(RCC_CR_MSIPLLEN | RCC_CR_MSION);
    }
    
    stm32wb_hsem_unlock(STM32WB_HSEM_INDEX_RCC, STM32WB_HSEM_PROCID_NONE);

    if (shutdown)
    {
        if (stm32wb_system_device.smps)
        {
            PWR->CR5 &= ~(PWR_CR5_SMPSEN | PWR_CR5_BORHC);
        }

        PWR->CR1 = (PWR->CR1 & ~PWR_CR1_LPMS) | PWR_CR1_LPMS_SHUTDOWN;
    }
    else
    {
        PWR->CR1 = (PWR->CR1 & ~PWR_CR1_LPMS) | PWR_CR1_LPMS_STANDBY;
    }
    
    PWR->CR3 = ((PWR->CR3 & ~PWR_CR3_EWUP) |
                (((wakeup >> STM32WB_SYSTEM_WAKEUP_PIN_RISING_SHIFT) | (wakeup >> STM32WB_SYSTEM_WAKEUP_PIN_FALLING_SHIFT)) & PWR_CR3_EWUP) | PWR_CR3_EIWUL);
    PWR->CR4 = ((PWR->CR4 & ~(PWR_CR4_WP1 | PWR_CR4_WP2 | PWR_CR4_WP3 | PWR_CR4_WP4 | PWR_CR4_WP5)) |
                ((wakeup >> STM32WB_SYSTEM_WAKEUP_PIN_FALLING_SHIFT) & (PWR_CR4_WP1 | PWR_CR4_WP2 | PWR_CR4_WP3 | PWR_CR4_WP4 | PWR_CR4_WP5)));

    RTC->BKP16R = (RTC->BKP16R & ~(STM32WB_RTC_BKP16R_DFU | STM32WB_RTC_BKP16R_FATAL)) | ((STM32WB_RTC_BKP16R_DFU | STM32WB_RTC_BKP16R_FATAL) << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT);
    
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __DSB();

    __SEV();
    __WFE();
    
    while (1)
    {
        __WFE();
    }
}

void stm32wb_system_standby(uint32_t wakeup, uint32_t timeout)
{
    uint32_t primask;

    if (!timeout)
    {
        return;
    }
    
    primask = __get_PRIMASK();
    
    __disable_irq();
    
    if (stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_SLEEP] ||
        stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_STOP_0] ||
        stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_STOP_1] ||
        stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_STOP_2])
    {
        __set_PRIMASK(primask);
                        
        return;
    }
    
    stm32wb_system_event(STM32WB_SYSTEM_EVENT_STANDBY);

    stm32wb_rtc_standby(timeout);

    stm32wb_system_halt(wakeup, false);
}

void stm32wb_system_shutdown(uint32_t wakeup)
{
    uint32_t primask;
    
    primask = __get_PRIMASK();
    
    __disable_irq();
    
    if (stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_SLEEP] ||
        stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_STOP_0] ||
        stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_STOP_1] ||
        stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_STOP_2] ||
        stm32wb_system_device.lock[STM32WB_SYSTEM_LOCK_STANDBY])
    {
        __set_PRIMASK(primask);
                        
        return;
    }

    stm32wb_system_event(STM32WB_SYSTEM_EVENT_SHUTDOWN);
    
    stm32wb_rtc_reset();

    stm32wb_system_halt(wakeup, true);
}

void __attribute__((noreturn)) stm32wb_system_reset(void)
{
    __disable_irq();

    stm32wb_system_event(STM32WB_SYSTEM_EVENT_RESET);

    RTC->BKP16R = (RTC->BKP16R & ~(STM32WB_RTC_BKP16R_DFU | STM32WB_RTC_BKP16R_FATAL)) | ((STM32WB_RTC_BKP16R_DFU | STM32WB_RTC_BKP16R_FATAL) << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT);

    stm32wb_rtc_reset();

    NVIC_SystemReset();
    
    while (1)
    {
    }
}

void __attribute__((noreturn)) stm32wb_system_dfu(void)
{
    __disable_irq();
    
    stm32wb_system_event(STM32WB_SYSTEM_EVENT_DFU);

    RTC->BKP16R = (RTC->BKP16R & ~((STM32WB_RTC_BKP16R_DFU << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT) | STM32WB_RTC_BKP16R_FATAL)) | ((STM32WB_RTC_BKP16R_FATAL << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT) | STM32WB_RTC_BKP16R_DFU);

    stm32wb_rtc_reset();

    NVIC_SystemReset();
    
    while (1)
    {
    }
}

void __attribute__((noreturn)) stm32wb_system_fatal(uint32_t cause, uint32_t reason)
{
    __disable_irq();

    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
    {
        __BKPT();
    }

    if (RTC->BKP16R & STM32WB_RTC_BKP16R_FATAL)
    {
        // double FATAL escalates to DFU
        RTC->BKP16R = (RTC->BKP16R & ~((STM32WB_RTC_BKP16R_DFU << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT) | STM32WB_RTC_BKP16R_FATAL)) | ((STM32WB_RTC_BKP16R_FATAL << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT) | STM32WB_RTC_BKP16R_DFU);
    }
    else
    {
        __stm32wb_system_reset_cause = cause;
        __stm32wb_system_reset_reason = reason;
        
        RTC->BKP16R = (RTC->BKP16R & ~((STM32WB_RTC_BKP16R_FATAL << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT) | STM32WB_RTC_BKP16R_DFU)) | ((STM32WB_RTC_BKP16R_DFU << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT) | STM32WB_RTC_BKP16R_FATAL);
    }
    
    stm32wb_rtc_reset();

    NVIC_SystemReset();
    
    while (1)
    {
    }
}

void PVD_PVM_IRQHandler(void)
{
    if (EXTI->PR1 & EXTI_PR1_PIF31)
    {
        EXTI->PR1 = EXTI_PR1_PIF31;

        if (stm32wb_system_device.pvm1_callback)
        {
            (*stm32wb_system_device.pvm1_callback)(stm32wb_system_device.pvm1_context);
        }
    }
}

static void __empty() { }

void __stm32wb_dma_initialize(void) __attribute__ ((weak, alias("__empty")));
void __stm32wb_lptim_initialize(void) __attribute__ ((weak, alias("__empty")));
void __stm32wb_adc_initialize(void) __attribute__ ((weak, alias("__empty")));

void __stm32wb_flash_initialize(void) __attribute__ ((weak, alias("__empty")));
void __stm32wb_eeprom_initialize(void) __attribute__ ((weak, alias("__empty")));
void __stm32wb_random_initialize(void) __attribute__ ((weak, alias("__empty")));

void stm32wb_flash_suspend(void) __attribute__ ((weak, alias("__empty")));
void stm32wb_flash_resume(void) __attribute__ ((weak, alias("__empty")));

static void Default_IRQHandler(void)
{
    stm32wb_system_fatal(STM32WB_SYSTEM_RESET_CAUSE_PANIC, (STM32WB_SYSTEM_PANIC_REASON_SPURIOUS | __current_irq()));
}

void WWDG_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void TAMP_STAMP_LSECSS_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void RTC_WKUP_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void FLASH_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void RCC_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void EXTI0_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void EXTI1_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void EXTI2_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void EXTI3_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void EXTI4_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA1_Channel1_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA1_Channel2_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA1_Channel3_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA1_Channel4_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA1_Channel5_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA1_Channel6_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA1_Channel7_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void ADC1_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void USB_HP_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void USB_LP_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void C2SEV_PWR_C2H_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void COMP_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void EXTI9_5_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void TIM1_BRK_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void TIM1_UP_TIM16_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void TIM1_TRG_COM_TIM17_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void TIM1_CC_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void TIM2_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void PKA_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void I2C1_EV_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void I2C1_ER_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void I2C3_EV_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void I2C3_ER_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void SPI1_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void SPI2_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void USART1_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void LPUART1_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void SAI1_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void TSC_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void EXTI15_10_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void RTC_Alarm_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void CRS_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void PWR_SOTF_BLEACT_802ACT_RFPHASE_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void IPCC_C1_RX_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void IPCC_C1_TX_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void HSEM_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void LPTIM1_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void LPTIM2_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void LCD_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void QUADSPI_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void AES1_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void AES2_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void RNG_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void FPU_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA2_Channel1_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA2_Channel2_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA2_Channel3_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA2_Channel4_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA2_Channel5_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA2_Channel6_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMA2_Channel7_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
void DMAMUX1_OVR_IRQHandler(void) __attribute__ ((weak, alias("Default_IRQHandler")));
