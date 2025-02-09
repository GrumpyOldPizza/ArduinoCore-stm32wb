/*
 * Copyright (c) 2022-2024 Thomas Roell.  All rights reserved.
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

#define __STM32WB_BOOT_CODE__

#include "stm32wb_boot.h"
#include "stm32wb_fwu.h"
#include "stm32wb_rtc.h"
#include "stm32wb_gpio.h"
#include "stm32wb_system.h"

#define STM32WB_BOOT_FWU_CONFIG_APPLICATION 1 // allow APPLICATION updates via FWU
#define STM32WB_BOOT_FWU_CONFIG_WIRELESS    1 // allow WIRELESS updates via FWU
#define STM32WB_BOOT_FWU_CONFIG_SFLASH      1 // allow FWU area in SFLASH

#define STM32WB_BOOT_DFU_CONFIG_STANDALONE  1 // do not use the FWU path for DFU
#define STM32WB_BOOT_DFU_CONFIG_WIRELESS    1 // allow WIRELESS updates via DFU

/************************************************************************************************************************************/
static void __attribute__((noreturn)) stm32wb_boot_reset(void);
       void __attribute__((noreturn)) stm32wb_boot_enter(void);
static void __attribute__((noreturn)) stm32wb_boot_leave(uint32_t vtor_address);
static void __attribute__((noreturn)) stm32wb_boot_application(void);


static void stm32wb_boot_nmi(void);
static void stm32wb_boot_hardfault(void);
static void stm32wb_boot_svcall(void);
static void stm32wb_boot_pendsv(void);

static void stm32wb_boot_usbd_dcd_event(void);
static void stm32wb_boot_usbd_dcd_usb_interrupt(void);
static void stm32wb_boot_usbd_dcd_crs_interrupt(void);

/************************************************************************************************************************************/

static void stm32wb_boot_memset(void *d, uint8_t c, size_t n);
static void stm32wb_boot_memcpy(void *d, const void *s, size_t n);
static int stm32wb_boot_memcmp(const void *a, const void *b, size_t n);

/************************************************************************************************************************************/

static void stm32wb_boot_gpio_pin_configure(uint32_t pin, uint32_t mode);
static void stm32wb_boot_gpio_pin_input(uint32_t pin);
static void stm32wb_boot_gpio_pin_output(uint32_t pin);
static void stm32wb_boot_gpio_pin_alternate(uint32_t pin);
static void stm32wb_boot_gpio_pin_write(uint32_t pin, uint32_t data);
static uint32_t stm32wb_boot_gpio_pin_read(uint32_t pin);

/************************************************************************************************************************************/

#define STM32WB_FLASH_PAGE_SIZE 4096

#define STM32WB_BOOT_FLASH_ERASE_SIZE    4096
#define STM32WB_BOOT_FLASH_ERASE_TIME    22     // mS
#define STM32WB_BOOT_FLASH_PROGRAM_SIZE  512
#define STM32WB_BOOT_FLASH_PROGRAM_TIME  5200   // uS

static bool stm32wb_boot_flash_erase(uint32_t address);
static bool stm32wb_boot_flash_program(uint32_t address, const uint8_t *data, uint32_t count);

/************************************************************************************************************************************/

#if (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1)

#define STM32WB_BOOT_SFLASH_BASE         0x00000000
#define STM32WB_BOOT_SFLASH_LIMIT        0x08000000

#define STM32WB_BOOT_SFLASH_FEATURE_QUAD 0x02

typedef struct _stm32wb_boot_sflash_info_t {
    uint8_t                        did[3];
    uint8_t                        features;
    uint32_t                       capacity;
    uint32_t                       erase_size;
    uint32_t                       page_size;
} stm32wb_boot_sflash_info_t;

static stm32wb_boot_sflash_info_t * stm32wb_boot_sflash_init(void);
static bool stm32wb_boot_sflash_erase(uint32_t address);
static void stm32wb_boot_sflash_program(uint32_t address, const uint8_t *data, uint32_t count);
static void stm32wb_boot_sflash_read(uint32_t address, uint8_t *data, uint32_t count);
static bool stm32wb_boot_sflash_verify(uint32_t address, const uint8_t *data, uint32_t count);

#endif /* (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1) */

/************************************************************************************************************************************/

#define STM32WB_BOOT_NVM_PROGRAM_SIZE 512

static void stm32wb_boot_nvm_set_key(const uint32_t *key);
static bool stm32wb_boot_nvm_erase(uint32_t address);
static bool stm32wb_boot_nvm_program(uint32_t address, const uint8_t *data, uint32_t count, bool encrypted);
static void stm32wb_boot_nvm_read(uint32_t address, uint8_t *data, uint32_t count, bool encrypted);

/************************************************************************************************************************************/

static void stm32wb_boot_random(uint8_t *data, uint32_t count);

/************************************************************************************************************************************/

static uint32_t stm32wb_boot_crc32(const uint8_t *data, uint32_t size, uint32_t crc32);

/************************************************************************************************************************************/

#define SWAP(_x)      (uint32_t)((((_x) >> 24) & 0x000000ff) | (((_x) >> 8) & 0x0000ff00) | (((_x) << 8) & 0x00ff0000) |  (((_x) << 24) & 0xff000000))

#define STM32WB_BOOT_AES128_BLOCK_SIZE 16

typedef struct _stm32wb_boot_aes128_context_t {
    uint32_t                   K[4];
} stm32wb_boot_aes128_context_t;

typedef void (*stm32wb_boot_aes128_get_key_routine_t)(uint32_t *key);

static void stm32wb_boot_aes128_get_key(const uint32_t *nonce, uint32_t *key);
static void stm32wb_boot_aes128_set_key(stm32wb_boot_aes128_context_t *aes128_ctx, const uint32_t *key);
static void stm32wb_boot_aes128_ecb_encrypt(stm32wb_boot_aes128_context_t *aes128_ctx, const uint32_t *in, uint32_t *out, uint32_t count);
static void stm32wb_boot_aes128_ctr_encrypt(stm32wb_boot_aes128_context_t *aes128_ctx, const uint32_t *iv, const uint32_t *in, uint32_t *out, uint32_t count);

/************************************************************************************************************************************/

#define STM32WB_BOOT_SHA256_BLOCK_SIZE  64
#define STM32WB_BOOT_SHA256_HASH_SIZE   32    

#define STM32WB_BOOT_SHA256_NUM_BITS    256
#define STM32WB_BOOT_SHA256_NUM_BYTES   32
#define STM32WB_BOOT_SHA256_NUM_WORDS   8

typedef struct _stm32wb_boot_sha256_context_t {
    uint32_t                   hash[STM32WB_BOOT_SHA256_HASH_SIZE / 4];
    uint32_t                   length;
    uint32_t                   index;
    uint8_t                    data[STM32WB_BOOT_SHA256_BLOCK_SIZE];
} stm32wb_boot_sha256_context_t;

static void stm32wb_boot_sha256_init(stm32wb_boot_sha256_context_t *sha256_ctx);
static void stm32wb_boot_sha256_update(stm32wb_boot_sha256_context_t *sha256_ctx, const uint8_t *data, size_t size);
static void stm32wb_boot_sha256_final(stm32wb_boot_sha256_context_t *sha256_ctx, uint32_t *hash);

/************************************************************************************************************************************/

static bool stm32wb_boot_ecc256_verify(const stm32wb_boot_ecc256_key_t *key, const uint32_t *signature, const uint32_t *digest);

/************************************************************************************************************************************/

#if (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1) || (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1)

static uint32_t stm32wb_boot_fus_install(uint32_t image_base, uint32_t image_size);

#endif /* (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1) || (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */

/************************************************************************************************************************************/

#if (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1) || (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1)

static uint32_t stm32wb_boot_fwu_state(void);
static uint32_t stm32wb_boot_fwu_install(const stm32wb_application_info_t *application_info);
static void stm32wb_boot_fwu_rollback(uint32_t status);

#endif /* (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1) || (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1) */

/************************************************************************************************************************************/

static void __attribute__((noreturn)) stm32wb_boot_dfu(const stm32wb_application_info_t *application_info);

/************************************************************************************************************************************/

extern uint32_t __boot_data_start__[];
extern uint32_t __boot_data_end__[];
extern uint32_t __boot_data_flash__[];
extern uint32_t __boot_bss_start__[];
extern uint32_t __boot_bss_end__[];

static uint8_t __attribute__((section(".noinit"), aligned(4))) stm32wb_boot_io2_data[2048]; /* EP0_DATA, LZFWU_TEMP */
static uint8_t __attribute__((section(".noinit"), aligned(4))) stm32wb_boot_io8_data[8192]; /* DFU_DATA, LZFWU_DICT */

extern const stm32wb_boot_info_t stm32wb_boot_info;

static const __attribute__((used)) stm32wb_boot_ecc256_key_t stm32wb_boot_ecc256_key;

static const __attribute__((used)) stm32wb_boot_info_t __stm32wb_boot_info__ = { .version = STM32WB_BOOT_VERSION, .ecc256_key = (uint32_t)&stm32wb_boot_ecc256_key };

extern uint32_t stm32wb_boot_stack[];
extern const uint32_t stm32wb_boot_vectors[];

static uint8_t __attribute__((section(".stack"), used, aligned(32))) __stm32wb_boot_stack__[4096];

static const uint32_t __attribute__((used)) __stm32wb_boot_vectors__[16+64] =
{
    (uint32_t)stm32wb_boot_stack,                                /* Top of Stack */
    (uint32_t)stm32wb_boot_enter,                                /* Reset Handler */
    (uint32_t)stm32wb_boot_nmi,                                  /* NMI Handler */
    (uint32_t)stm32wb_boot_hardfault,                            /* Hard Fault Handler */
    0,                                                           /* MPU Fault Handler */
    0,                                                           /* Bus Fault Handler */
    0,                                                           /* Usage Fault Handler */
    0,                                                           /* Reserved */
    0x54561495,                                                  /* Reserved (magic) */
    STM32WB_BOOT_BASE,                                           /* Reserved (base) */
    STM32WB_BOOT_SIZE,                                           /* Reserved (size) */
    (uint32_t)stm32wb_boot_svcall,                               /* SVCall Handler */
    0,                                                           /* Debug Monitor Handler */
    (uint32_t)&stm32wb_boot_info - STM32WB_BOOT_BASE,            /* Reserved (offset) */
    (uint32_t)stm32wb_boot_pendsv,                               /* PendSV Handler */
    0,                                                           /* SysTick Handler */

    /* External interrupts */
    0,                                                           /* WWDG_IRQHandler */
    0,                                                           /* PVD_PVM_IRQHandler */
    0,                                                           /* TAMP_STAMP_LSECSS_IRQHandler */
    0,                                                           /* RTC_WKUP_IRQHandler */
    0,                                                           /* FLASH_IRQHandler */
    0,                                                           /* RCC_IRQHandler */
    0,                                                           /* EXTI0_IRQHandler */
    0,                                                           /* EXTI1_IRQHandler */
    0,                                                           /* EXTI2_IRQHandler */
    0,                                                           /* EXTI3_IRQHandler */
    0,                                                           /* EXTI4_IRQHandler */
    0,                                                           /* DMA1_Channel1_IRQHandler */
    0,                                                           /* DMA1_Channel2_IRQHandler */
    0,                                                           /* DMA1_Channel3_IRQHandler */
    0,                                                           /* DMA1_Channel4_IRQHandler */
    0,                                                           /* DMA1_Channel5_IRQHandler */
    0,                                                           /* DMA1_Channel6_IRQHandler */
    0,                                                           /* DMA1_Channel7_IRQHandler */
    0,                                                           /* ADC1_IRQHandler */
    0,                                                           /* USB_HP_IRQHandler */
    (uint32_t)stm32wb_boot_usbd_dcd_usb_interrupt,               /* USB_LP_IRQHandler */
    0,                                                           /* C2SEV_PWR_C2H_IRQHandler */
    0,                                                           /* COMP_IRQHandler */
    0,                                                           /* EXTI9_5_IRQHandler */
    0,                                                           /* TIM1_BRK_IRQHandler */
    0,                                                           /* TIM1_UP_TIM16_IRQHandler */
    0,                                                           /* TIM1_TRG_COM_TIM17_IRQHandler */
    0,                                                           /* TIM1_CC_IRQHandler */
    0,                                                           /* TIM2_IRQHandler */
    0,                                                           /* PKA_IRQHandler */
    0,                                                           /* I2C1_EV_IRQHandler */
    0,                                                           /* I2C1_ER_IRQHandler */
    0,                                                           /* I2C3_EV_IRQHandler */
    0,                                                           /* I2C3_ER_IRQHandler */
    0,                                                           /* SPI1_IRQHandler */
    0,                                                           /* SPI2_IRQHandler */
    0,                                                           /* USART1_IRQHandler */
    0,                                                           /* LPUART1_IRQHandler */
    0,                                                           /* SAI1_IRQHandler */
    0,                                                           /* TSC_IRQHandler */
    0,                                                           /* EXTI15_10_IRQHandler */
    0,                                                           /* RTC_Alarm_IRQHandler */
    (uint32_t)stm32wb_boot_usbd_dcd_crs_interrupt,               /* CRS_IRQHandler */
    0,                                                           /* PWR_SOTF_BLEACT_802ACT_RFPHASE_IRQHandler */
    0,                                                           /* IPCC_C1_RX_IRQHandler */
    0,                                                           /* IPCC_C1_TX_IRQHandler */
    0,                                                           /* HSEM_IRQHandler */
    0,                                                           /* LPTIM1_IRQHandler */
    0,                                                           /* LPTIM2_IRQHandler */
    0,                                                           /* LCD_IRQHandler */
    0,                                                           /* QUADSPI_IRQHandler */
    0,                                                           /* AES1_IRQHandler */
    0,                                                           /* AES2_IRQHandler */
    0,                                                           /* RNG_IRQHandler */
    0,                                                           /* FPU_IRQHandler */
    0,                                                           /* DMA2_Channel1_IRQHandler */
    0,                                                           /* DMA2_Channel2_IRQHandler */
    0,                                                           /* DMA2_Channel3_IRQHandler */
    0,                                                           /* DMA2_Channel4_IRQHandler */
    0,                                                           /* DMA2_Channel5_IRQHandler */
    0,                                                           /* DMA2_Channel6_IRQHandler */
    0,                                                           /* DMA2_Channel7_IRQHandler */
    0,                                                           /* DMAMUX1_OVR_IRQHandler */
};

/************************************************************************************************************************************/

static void __attribute__((noreturn, noinline)) stm32wb_boot_reset(void)
{
    SCB->AIRCR = 0x05fa0004;
    __DSB();
    __ISB();

    while (1)
    {
    }
}

void __attribute__((noreturn, noinline)) stm32wb_boot_enter(void)
{
    FLASH->ACR = FLASH_ACR_ICRST | FLASH_ACR_DCRST;
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS;
    FLASH->ACR;
    
    RCC->CR |= RCC_CR_HSION;
        
    while (!(RCC->CR & RCC_CR_HSIRDY))
    {
    }

    RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSSEL) | RCC_SMPSCR_SMPSSEL_HSI;

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
            
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {
    }

    if ((PWR->CR1 & PWR_CR1_VOS) != PWR_CR1_VOS_RANGE_1)
    {
        PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_RANGE_1;
            
        while (PWR->SR2 & PWR_SR2_VOSF)
        {
        }
    }

    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_HPREF | RCC_CFGR_PPRE1F | RCC_CFGR_PPRE2F)) | (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2);

    while ((RCC->CFGR & (RCC_CFGR_HPREF | RCC_CFGR_PPRE1F | RCC_CFGR_PPRE2F)) != (RCC_CFGR_HPREF | RCC_CFGR_PPRE1F | RCC_CFGR_PPRE2F))
    {
    }

    RCC->EXTCFGR = (RCC->EXTCFGR & ~(RCC_EXTCFGR_SHDHPRE | RCC_EXTCFGR_C2HPRE | RCC_EXTCFGR_SHDHPREF | RCC_EXTCFGR_C2HPREF)) | RCC_EXTCFGR_SHDHPRE_DIV1 | RCC_EXTCFGR_C2HPRE_DIV2;
    
    while ((RCC->EXTCFGR & (RCC_EXTCFGR_SHDHPREF | RCC_EXTCFGR_C2HPREF)) != (RCC_EXTCFGR_SHDHPREF | RCC_EXTCFGR_C2HPREF))
    {
    }

    stm32wb_boot_application();
}

static void __attribute__((noreturn, noinline)) stm32wb_boot_leave(uint32_t vtor_address)
{
    register uint32_t stack_address __asm__("r0");
    register uint32_t entry_address __asm__("r1");
    register uint32_t zero_base __asm__("r2");
    register uint32_t zero_limit __asm__("r3");

    MPU->RBAR = 0x08000000 | MPU_RBAR_VALID_Msk | (0 << MPU_RBAR_REGION_Pos);
    MPU->RASR = MPU_RASR_XN_Msk | (1 << MPU_RASR_SRD_Pos) | (13 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;
    MPU->CTRL = (MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_HFNMIENA_Msk | MPU_CTRL_ENABLE_Msk);
    
    SCB->VTOR = vtor_address;
    
    __DSB();
    __ISB();
    
    stack_address = ((volatile uint32_t * volatile )vtor_address)[0];
    entry_address = ((volatile uint32_t * volatile )vtor_address)[1];
    
    zero_base  = (uint32_t)0x20000000;
    zero_limit = (uint32_t)0x20004000;
    
    __asm__ volatile (
        "   mov     sp, %0                                  \n"
        "   mov     lr, %1                                  \n"
        "   movs    %0, #0                                  \n"
        "   movs    %1, #0                                  \n"
        "1: strd    %0, %1, [%2], #4                        \n"
        "   cmp     %2, %3                                  \n"
        "   bne     1b                                      \n"
        "   mov     r2, r0                                  \n"
        "   mov     r3, r1                                  \n"
        "   mov     r4, r2                                  \n"
        "   mov     r5, r2                                  \n"
        "   mov     r6, r2                                  \n"
        "   mov     r7, r2                                  \n"
        "   mov     r8, r2                                  \n"
        "   mov     r9, r2                                  \n"
        "   mov     r10, r2                                 \n"
        "   mov     r11, r2                                 \n"
        "   mov     r12, r2                                 \n"
        "   bx      lr                                      \n"
        :
        : "l" (stack_address), "l" (entry_address), "l" (zero_base), "l" (zero_limit)
        );
    
    while (1)
    {
    }
}

/************************************************************************************************************************************/

static void __attribute__((noinline)) stm32wb_boot_nmi(void)
{
    while (1)
    {
        __WFE();
    }
}

static void __attribute__((noinline)) stm32wb_boot_hardfault(void)
{
    while (1)
    {
        __WFE();
    }
}

static void __attribute__((naked, noinline)) stm32wb_boot_svcall(void)
{
    __asm__(
        "   mov     r2, sp                                  \n"
        "   push    { r2, lr }                              \n"
        "   ldmia   r2, { r0, r1, r2, r3 }                  \n"
        "   blx     r7                                      \n"
        "   pop     { r2, r3 }                              \n"
        "   str     r0, [r2, #0]                            \n"
        "   bx      r3                                      \n"
        :
        :
        );
}

static void __attribute__((noinline)) stm32wb_boot_pendsv(void)
{
    stm32wb_boot_usbd_dcd_event();
}

/************************************************************************************************************************************/

static void __attribute__((naked, noinline)) stm32wb_boot_memset(void *d, uint8_t c, size_t n)
{
    __asm__(
        "   cmp     r2, #16                                 \n"
        "   bls     2f                                      \n"
        "   lsls    r3, r0, #30                             \n"
        "   bne     2f                                      \n"
        "   uxtb    r1, r1                                  \n"
	"   orr     r1, r1, r1, lsl #8                      \n"
	"   orr     r1, r1, r1, lsl #16                     \n"
        "   bic     r3, r2, #3                              \n"
        "   add     r3, r0                                  \n"
        "1: str     r1, [r0], #4                            \n"
        "   cmp     r3, r0                                  \n"
        "   bne     1b                                      \n"
        "   lsrs    r3, r2, #2                              \n"
        "   it      cs                                      \n"
        "   strhcs  r1, [r0], #2                            \n"
        "   lsrs    r3, r2, #1                              \n"
        "   it      cs                                      \n"
        "   strbcs  r1, [r0], #1                            \n"
        "   bx      lr                                      \n"
        "2: cbz     r2, 4f                                  \n"
        "   add     r2, r0                                  \n"
        "3: strb    r1, [r0], #1                            \n"
        "   cmp     r2, r0                                  \n"
        "   bne     3b                                      \n"
        "4: bx      lr                                      \n"
        );
}

static void __attribute__((naked, noinline)) stm32wb_boot_memcpy(void *d, const void *s, size_t n)
{
    __asm__(
        "   cmp     r2, #16                                 \n"
        "   bls     2f                                      \n"
        "   lsls    r3, r0, #30                             \n"
        "   bne     2f                                      \n"
        "   bic     r3, r2, #3                              \n"
        "   add     r3, r0                                  \n"
        "1: ldr     r12, [r1], #4                           \n"
        "   str     r12, [r0], #4                           \n"
        "   cmp     r3, r0                                  \n"
        "   bne     1b                                      \n"
        "   lsrs    r3, r2, #2                              \n"
        "   itt     cs                                      \n"
        "   ldrhcs  r12, [r1], #2                           \n"
        "   strhcs  r12, [r0], #2                           \n"
        "   lsrs    r3, r2, #1                              \n"
        "   itt     cs                                      \n"
        "   ldrbcs  r12, [r1], #1                           \n"
        "   strbcs  r12, [r0], #1                           \n"
        "   bx      lr                                      \n"
        "2: cbz     r2, 4f                                  \n"
        "   add     r2, r0                                  \n"
        "3: ldrb    r12, [r1], #1                           \n"
        "   strb    r12, [r0], #1                           \n"
        "   cmp     r2, r0                                  \n"
        "   bne     3b                                      \n"
        "4: bx      lr                                      \n"
        );
}

static int __attribute__((noinline)) stm32wb_boot_memcmp(const void *a, const void *b, size_t n)
{
    size_t i;
    
    if (n)
    {
        for (i = 0; i < n; i++)
        {
            if (((const uint8_t*)a)[i] != ((const uint8_t*)b)[i])
            {
                return (((const uint8_t*)a)[i] - ((const uint8_t*)b)[i]);
            }
        }
    }

    return 0;
}
/************************************************************************************************************************************/

#define armv7m_atomic_and              stm32wb_boot_atomic_and
#define armv7m_atomic_andh             stm32wb_boot_atomic_andh
#define armv7m_atomic_or               stm32wb_boot_atomic_or
#define armv7m_atomic_orh              stm32wb_boot_atomic_orh 
#define armv7m_atomic_swap             stm32wb_boot_atomic_swap
#define armv7m_atomic_swapb            stm32wb_boot_atomic_swapb

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_and(volatile uint32_t *p_data, uint32_t data)
{
    return __armv7m_atomic_and(p_data, data);
}

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_andh(volatile uint16_t *p_data, uint32_t data)
{
    return __armv7m_atomic_andh(p_data, data);
}

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_or(volatile uint32_t *p_data, uint32_t data)
{
    return __armv7m_atomic_or(p_data, data);
}

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_orh(volatile uint16_t *p_data, uint32_t data)
{
    return __armv7m_atomic_orh(p_data, data);
}

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_swap(volatile uint32_t *p_data, uint32_t data)
{
    return __armv7m_atomic_swap(p_data, data);
}

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_swapb(volatile uint8_t *p_data, uint32_t data)
{
    return __armv7m_atomic_swapb(p_data, data);
}

/************************************************************************************************************************************/

#define armv7m_core_udelay             stm32wb_boot_udelay

static void __attribute__((noinline)) stm32wb_boot_udelay(uint32_t delay)
{
    uint32_t n;

    n = (delay * (64000000 / 15625) + 255) / 256;

    __asm__ __volatile__(
                         "1: subs %0, #1 \n"
                         "   nop         \n"
                         "   bne  1b     \n"
                         : "+r" (n));
}

/************************************************************************************************************************************/

static void __attribute__((noinline)) stm32wb_boot_gpio_pin_configure(uint32_t pin, uint32_t mode)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, afsel, mask, index_2, mask_2;

    afsel = (pin >> 8) & 15;
    group = (pin >> 4) & 15;
    index = (pin >> 0) & 15;

    mask = (1 << index);
    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN << group);
    RCC->AHB2ENR;

    /* If the mode is ANALOG, set MODER first */
    if ((mode & STM32WB_GPIO_MODE_MASK) == STM32WB_GPIO_MODE_ANALOG)
    {
        GPIO->MODER = (GPIO->MODER & ~mask_2) | ((STM32WB_GPIO_MODE_ANALOG >> STM32WB_GPIO_MODE_SHIFT) << index_2);
    }

    /* Set AFRL/AFRH */
    GPIO->AFR[index >> 3] = (GPIO->AFR[index >> 3] & ~(15 << ((index & 7) << 2))) | (afsel << ((index & 7) << 2));
        
    /* Set OPTYPER */
    GPIO->OTYPER = (GPIO->OTYPER & ~mask) | (((mode & STM32WB_GPIO_OTYPE_MASK) >> STM32WB_GPIO_OTYPE_SHIFT) << index);
    
    /* Set OPSPEEDR */
    GPIO->OSPEEDR = (GPIO->OSPEEDR & ~mask_2) | (((mode & STM32WB_GPIO_OSPEED_MASK) >> STM32WB_GPIO_OSPEED_SHIFT) << index_2);

    if (mode & STM32WB_GPIO_ODATA_0)
    {
        GPIO->BRR = mask;
    }
    
    if (mode & STM32WB_GPIO_ODATA_1)
    {
        GPIO->BSRR = mask;
    }
    
    /* Set MODE */
    GPIO->MODER = (GPIO->MODER & ~mask_2) | (((mode & STM32WB_GPIO_MODE_MASK) >> STM32WB_GPIO_MODE_SHIFT) << index_2);
        
    /* Set PUPD */
    GPIO->PUPDR = (GPIO->PUPDR & ~mask_2) | (((mode & STM32WB_GPIO_PUPD_MASK) >> STM32WB_GPIO_PUPD_SHIFT) << index_2);
}

static void __attribute__((noinline)) stm32wb_boot_gpio_pin_input(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, index_2, mask_2;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;

    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    GPIO->MODER = (GPIO->MODER & ~mask_2) | ((STM32WB_GPIO_MODE_INPUT >> STM32WB_GPIO_MODE_SHIFT) << index_2);
}

static void __attribute__((noinline)) stm32wb_boot_gpio_pin_output(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, index_2, mask_2;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;

    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    GPIO->MODER = (GPIO->MODER & ~mask_2) | ((STM32WB_GPIO_MODE_OUTPUT >> STM32WB_GPIO_MODE_SHIFT) << index_2);
}

static void __attribute__((noinline)) stm32wb_boot_gpio_pin_alternate(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, index_2, mask_2;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;

    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    GPIO->MODER = (GPIO->MODER & ~mask_2) | ((STM32WB_GPIO_MODE_ALTERNATE >> STM32WB_GPIO_MODE_SHIFT) << index_2);
}

static void __attribute__((noinline, optimize("O3"))) stm32wb_boot_gpio_pin_write(uint32_t pin, uint32_t data)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index;

    group = (pin & STM32WB_GPIO_PIN_GROUP_MASK) >> STM32WB_GPIO_PIN_GROUP_SHIFT;
    index = (pin & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT;

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (data)
    {
        GPIO->BSRR = (1 << index);
    }
    else
    {
        GPIO->BRR = (1 << index);
    }
}

static uint32_t __attribute__((noinline, optimize("O3"))) stm32wb_boot_gpio_pin_read(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index;

    group = (pin & STM32WB_GPIO_PIN_GROUP_MASK) >> STM32WB_GPIO_PIN_GROUP_SHIFT;
    index = (pin & STM32WB_GPIO_PIN_INDEX_MASK) >> STM32WB_GPIO_PIN_INDEX_SHIFT;

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    return ((GPIO->IDR >> index) & 1);
}

/************************************************************************************************************************************/

static bool __attribute__((noinline)) stm32wb_boot_flash_erase(uint32_t address)
{
    uint32_t flash_sr, flash_acr;

    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xcdef89ab;
    
    FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);
    FLASH->CR = FLASH_CR_STRT | FLASH_CR_PER | (((address - FLASH_BASE) / STM32WB_FLASH_PAGE_SIZE) << 3);

    do
    {
        flash_sr = FLASH->SR;
    }
    while (flash_sr & FLASH_SR_BSY);

    FLASH->CR = FLASH_CR_LOCK;
    
    if (flash_sr & (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR))
    {
        return false;
    }

    flash_acr = FLASH->ACR;

    FLASH->ACR = flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    FLASH->ACR = (flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN)) | (FLASH_ACR_ICRST | FLASH_ACR_DCRST);
    FLASH->ACR = flash_acr;
    
    return true;
}

static bool stm32wb_boot_flash_program(uint32_t address, const uint8_t *data, uint32_t count)
{
    uint32_t flash_sr, flash_acr, data_0, data_1;
    bool success = true;

    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xcdef89ab;
    
    FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);
    FLASH->CR = FLASH_CR_PG;

    for (; count; address += 8, data += 8, count -= 8)
    {
        data_0 = ((const uint32_t*)data)[0];
        data_1 = ((const uint32_t*)data)[1];

        __asm__ volatile("": : : "memory");
        
        ((volatile uint32_t*)address)[0] = data_0;
        ((volatile uint32_t*)address)[1] = data_1;
        
        do
        {
            flash_sr = FLASH->SR;
        }
        while (flash_sr & FLASH_SR_BSY);

        if (flash_sr & (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR))
        {
            success = false;

            break;
        }
    }
    
    FLASH->CR = FLASH_CR_LOCK;
    
    flash_acr = FLASH->ACR;

    FLASH->ACR = flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    FLASH->ACR = (flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN)) | (FLASH_ACR_ICRST | FLASH_ACR_DCRST);
    FLASH->ACR = flash_acr;
    
    return success;
}

/************************************************************************************************************************************/

#if (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1)

#define QSPI_CCR_SDR                               (0)
#define QSPI_CCR_DDR                               QUADSPI_CCR_DDRM
#define QSPI_CCR_FMODE_MASK                        QUADSPI_CCR_FMODE_Msk
#define QSPI_CCR_FMODE_SHIFT                       QUADSPI_CCR_FMODE_Pos
#define QSPI_CCR_FMODE_WRITE                       (0)
#define QSPI_CCR_FMODE_READ                        (QUADSPI_CCR_FMODE_0)
#define QSPI_CCR_FMODE_WAIT                        (QUADSPI_CCR_FMODE_1)
#define QSPI_CCR_DMODE_MASK                        QUADSPI_CCR_DMODE_Msk
#define QSPI_CCR_DMODE_SHIFT                       QUADSPI_CCR_DMODE_Pos
#define QSPI_CCR_DMODE_NONE                        (0)
#define QSPI_CCR_DMODE_SINGLE                      (QUADSPI_CCR_DMODE_0)
#define QSPI_CCR_DMODE_DUAL                        (QUADSPI_CCR_DMODE_1)
#define QSPI_CCR_DMODE_QUAD                        (QUADSPI_CCR_DMODE_1 | QUADSPI_CCR_DMODE_0)
#define QSPI_CCR_DCYCLE(_n)                        (((_n) << QUADSPI_CCR_DCYC_Pos) & QUADSPI_CCR_DCYC_Msk)
#define QSPI_CCR_ABSIZE_MASK                       QUADSPI_CCR_ABSIZE_Msk
#define QSPI_CCR_ABSIZE_SHIFT                      QUADSPI_CCR_ABSIZE_Pos
#define QSPI_CCR_ABSIZE_8                          (0)
#define QSPI_CCR_ABSIZE_16                         (QUADSPI_CCR_ABSIZE_0)
#define QSPI_CCR_ABSIZE_24                         (QUADSPI_CCR_ABSIZE_1)
#define QSPI_CCR_ABSIZE_32                         (QUADSPI_CCR_ABSIZE_1 | QUADSPI_CCR_ABSIZE_0)
#define QSPI_CCR_ABMODE_MASK                       QUADSPI_CCR_ABMODE_Msk
#define QSPI_CCR_ABMODE_SHIFT                      QUADSPI_CCR_ABMODE_Pos
#define QSPI_CCR_ABMODE_NONE                       (0)
#define QSPI_CCR_ABMODE_SINGLE                     (QUADSPI_CCR_ABMODE_0)
#define QSPI_CCR_ABMODE_DUAL                       (QUADSPI_CCR_ABMODE_1)
#define QSPI_CCR_ABMODE_QUAD                       (QUADSPI_CCR_ABMODE_1 | QUADSPI_CCR_ABMODE_0)
#define QSPI_CCR_ABMODE_DUAL_8                     (QSPI_CCR_ABMODE_DUAL | QSPI_CCR_ABSIZE_8)
#define QSPI_CCR_ABMODE_DUAL_16                    (QSPI_CCR_ABMODE_DUAL | QSPI_CCR_ABSIZE_16)
#define QSPI_CCR_ABMODE_DUAL_24                    (QSPI_CCR_ABMODE_DUAL | QSPI_CCR_ABSIZE_24)
#define QSPI_CCR_ABMODE_DUAL_32                    (QSPI_CCR_ABMODE_DUAL | QSPI_CCR_ABSIZE_32)
#define QSPI_CCR_ABMODE_QUAD_8                     (QSPI_CCR_ABMODE_QUAD | QSPI_CCR_ABSIZE_8)
#define QSPI_CCR_ABMODE_QUAD_16                    (QSPI_CCR_ABMODE_QUAD | QSPI_CCR_ABSIZE_16)
#define QSPI_CCR_ABMODE_QUAD_24                    (QSPI_CCR_ABMODE_QUAD | QSPI_CCR_ABSIZE_24)
#define QSPI_CCR_ABMODE_QUAD_32                    (QSPI_CCR_ABMODE_QUAD | QSPI_CCR_ABSIZE_32)
#define QSPI_CCR_ADSIZE_MASK                       QUADSPI_CCR_ADSIZE_Msk
#define QSPI_CCR_ADSIZE_SHIFT                      QUADSPI_CCR_ADSIZE_Pos
#define QSPI_CCR_ADSIZE_16                         (QUADSPI_CCR_ADSIZE_0)
#define QSPI_CCR_ADSIZE_8                          (0)
#define QSPI_CCR_ADSIZE_16                         (QUADSPI_CCR_ADSIZE_0)
#define QSPI_CCR_ADSIZE_24                         (QUADSPI_CCR_ADSIZE_1)
#define QSPI_CCR_ADSIZE_32                         (QUADSPI_CCR_ADSIZE_1 | QUADSPI_CCR_ADSIZE_0)
#define QSPI_CCR_ADMODE_MASK                       QUADSPI_CCR_ADMODE_Msk
#define QSPI_CCR_ADMODE_SHIFT                      QUADSPI_CCR_ADMODE_Pos
#define QSPI_CCR_ADMODE_NONE                       (0)
#define QSPI_CCR_ADMODE_SINGLE                     (QUADSPI_CCR_ADMODE_0)
#define QSPI_CCR_ADMODE_DUAL                       (QUADSPI_CCR_ADMODE_1)
#define QSPI_CCR_ADMODE_QUAD                       (QUADSPI_CCR_ADMODE_1 | QUADSPI_CCR_ADMODE_0)
#define QSPI_CCR_ADMODE_SINGLE_8                   (QSPI_CCR_ADMODE_SINGLE | QSPI_CCR_ADSIZE_8)
#define QSPI_CCR_ADMODE_SINGLE_16                  (QSPI_CCR_ADMODE_SINGLE | QSPI_CCR_ADSIZE_16)
#define QSPI_CCR_ADMODE_SINGLE_24                  (QSPI_CCR_ADMODE_SINGLE | QSPI_CCR_ADSIZE_24)
#define QSPI_CCR_ADMODE_SINGLE_32                  (QSPI_CCR_ADMODE_SINGLE | QSPI_CCR_ADSIZE_32)
#define QSPI_CCR_ADMODE_DUAL_8                     (QSPI_CCR_ADMODE_DUAL | QSPI_CCR_ADSIZE_8)
#define QSPI_CCR_ADMODE_DUAL_16                    (QSPI_CCR_ADMODE_DUAL | QSPI_CCR_ADSIZE_16)
#define QSPI_CCR_ADMODE_DUAL_24                    (QSPI_CCR_ADMODE_DUAL | QSPI_CCR_ADSIZE_24)
#define QSPI_CCR_ADMODE_DUAL_32                    (QSPI_CCR_ADMODE_DUAL | QSPI_CCR_ADSIZE_32)
#define QSPI_CCR_ADMODE_QUAD_8                     (QSPI_CCR_ADMODE_QUAD | QSPI_CCR_ADSIZE_8)
#define QSPI_CCR_ADMODE_QUAD_16                    (QSPI_CCR_ADMODE_QUAD | QSPI_CCR_ADSIZE_16)
#define QSPI_CCR_ADMODE_QUAD_24                    (QSPI_CCR_ADMODE_QUAD | QSPI_CCR_ADSIZE_24)
#define QSPI_CCR_ADMODE_QUAD_32                    (QSPI_CCR_ADMODE_SINGLE | QSPI_CCR_ADSIZE_32)
#define QSPI_CCR_IMODE_NONE                        (0)
#define QSPI_CCR_IMODE_SINGLE                      (QUADSPI_CCR_IMODE_0)
#define QSPI_CCR_IMODE_DUAL                        (QUADSPI_CCR_IMODE_1)
#define QSPI_CCR_IMODE_QUAD                        (QUADSPI_CCR_IMODE_1 | QUADSPI_CCR_IMODE_0)

#define STM32WB_BOOT_QSPI_MID_SPANSION             0x01
#define STM32WB_BOOT_QSPI_MID_MACRONIX             0xc2
#define STM32WB_BOOT_QSPI_MID_WINBOND              0xef

#define STM32WB_BOOT_QSPI_CCR_COMMAND              (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_NONE   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_NONE      | QSPI_CCR_IMODE_SINGLE)
#define STM32WB_BOOT_QSPI_CCR_COMMAND_STATUS       (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_SINGLE | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_NONE      | QSPI_CCR_IMODE_SINGLE)
#define STM32WB_BOOT_QSPI_CCR_COMMAND_WRITE_1      (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_NONE   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_8  | QSPI_CCR_IMODE_SINGLE)
#define STM32WB_BOOT_QSPI_CCR_COMMAND_WRITE_2      (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_NONE   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_16 | QSPI_CCR_IMODE_SINGLE)
#define STM32WB_BOOT_QSPI_CCR_COMMAND_WRITE_3      (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_NONE   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_24 | QSPI_CCR_IMODE_SINGLE)
#define STM32WB_BOOT_QSPI_CCR_COMMAND_WRITE_4      (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_NONE   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_32 | QSPI_CCR_IMODE_SINGLE)
#define STM32WB_BOOT_QSPI_CCR_COMMAND_READ         (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_SINGLE | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_NONE      | QSPI_CCR_IMODE_SINGLE)
#define STM32WB_BOOT_QSPI_CCR_COMMAND_QPI          (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_NONE   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_NONE      | QSPI_CCR_IMODE_QUAD  )
#define STM32WB_BOOT_QSPI_CCR_COMMAND_QPI_STATUS   (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_QUAD   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_NONE      | QSPI_CCR_IMODE_QUAD  )
#define STM32WB_BOOT_QSPI_CCR_COMMAND_QPI_WRITE_1  (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_NONE   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_QUAD_8    | QSPI_CCR_IMODE_QUAD  )
#define STM32WB_BOOT_QSPI_CCR_COMMAND_QPI_WRITE_2  (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_NONE   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_QUAD_16   | QSPI_CCR_IMODE_QUAD  )
#define STM32WB_BOOT_QSPI_CCR_COMMAND_QPI_WRITE_3  (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_NONE   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_QUAD_24   | QSPI_CCR_IMODE_QUAD  )
#define STM32WB_BOOT_QSPI_CCR_COMMAND_QPI_WRITE_4  (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_NONE   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_QUAD_32   | QSPI_CCR_IMODE_QUAD  )

#define STM32WB_BOOT_QSPI_CCR_RDID                 (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_SINGLE | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_NONE      | QSPI_CCR_IMODE_SINGLE | 0x9f)
#define STM32WB_BOOT_QSPI_CCR_RDSFDP               (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_SINGLE | QSPI_CCR_DCYCLE(8) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_24 | QSPI_CCR_IMODE_SINGLE | 0x5a)

#define STM32WB_BOOT_QSPI_CCR_ERASE_64K            (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_NONE   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_24 | QSPI_CCR_IMODE_SINGLE | 0xd8)
#define STM32WB_BOOT_QSPI_CCR_ERASE_4K             (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_NONE   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_24 | QSPI_CCR_IMODE_SINGLE | 0x20)
#define STM32WB_BOOT_QSPI_CCR_PROGRAM_1_1_1        (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_SINGLE | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_24 | QSPI_CCR_IMODE_SINGLE | 0x02)
#define STM32WB_BOOT_QSPI_CCR_PROGRAM_1_1_4        (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_QUAD   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_24 | QSPI_CCR_IMODE_SINGLE | 0x32)
#define STM32WB_BOOT_QSPI_CCR_PROGRAM_1_4_4        (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_QUAD   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_QUAD_24   | QSPI_CCR_IMODE_SINGLE | 0x38)
#define STM32WB_BOOT_QSPI_CCR_PROGRAM_4_4_4        (QSPI_CCR_SDR | QSPI_CCR_FMODE_WRITE | QSPI_CCR_DMODE_QUAD   | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_QUAD_24   | QSPI_CCR_IMODE_QUAD   | 0x02)
#define STM32WB_BOOT_QSPI_CCR_READ_1_1_1           (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_SINGLE | QSPI_CCR_DCYCLE(0) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_24 | QSPI_CCR_IMODE_SINGLE | 0x03)
#define STM32WB_BOOT_QSPI_CCR_READ_1_1_1_D8        (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_SINGLE | QSPI_CCR_DCYCLE(8) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_24 | QSPI_CCR_IMODE_SINGLE | 0x0b)
#define STM32WB_BOOT_QSPI_CCR_READ_1_1_2_D8        (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_DUAL   | QSPI_CCR_DCYCLE(8) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_24 | QSPI_CCR_IMODE_SINGLE | 0x3b)
#define STM32WB_BOOT_QSPI_CCR_READ_1_1_4_D8        (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_QUAD   | QSPI_CCR_DCYCLE(8) | QSPI_CCR_ABMODE_NONE    | QSPI_CCR_ADMODE_SINGLE_24 | QSPI_CCR_IMODE_SINGLE | 0x6b)
#define STM32WB_BOOT_QSPI_CCR_READ_1_2_2_M2D2      (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_DUAL   | QSPI_CCR_DCYCLE(2) | QSPI_CCR_ABMODE_QUAD_8  | QSPI_CCR_ADMODE_DUAL_24   | QSPI_CCR_IMODE_SINGLE | 0xbb)
#define STM32WB_BOOT_QSPI_CCR_READ_1_2_2_M2D2_DTR  (QSPI_CCR_DDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_DUAL   | QSPI_CCR_DCYCLE(2) | QSPI_CCR_ABMODE_QUAD_16 | QSPI_CCR_ADMODE_DUAL_24   | QSPI_CCR_IMODE_SINGLE | 0xbd)
#define STM32WB_BOOT_QSPI_CCR_READ_1_2_2_M2D4_DTR  (QSPI_CCR_DDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_DUAL   | QSPI_CCR_DCYCLE(4) | QSPI_CCR_ABMODE_QUAD_16 | QSPI_CCR_ADMODE_DUAL_24   | QSPI_CCR_IMODE_SINGLE | 0xbd)
#define STM32WB_BOOT_QSPI_CCR_READ_1_4_4_M2D4      (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_QUAD   | QSPI_CCR_DCYCLE(4) | QSPI_CCR_ABMODE_QUAD_8  | QSPI_CCR_ADMODE_QUAD_24   | QSPI_CCR_IMODE_SINGLE | 0xeb)
#define STM32WB_BOOT_QSPI_CCR_READ_1_4_4_M2D4_DTR  (QSPI_CCR_DDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_QUAD   | QSPI_CCR_DCYCLE(4) | QSPI_CCR_ABMODE_QUAD_16 | QSPI_CCR_ADMODE_QUAD_24   | QSPI_CCR_IMODE_SINGLE | 0xed)
#define STM32WB_BOOT_QSPI_CCR_READ_1_4_4_M2D6_DTR  (QSPI_CCR_DDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_QUAD   | QSPI_CCR_DCYCLE(6) | QSPI_CCR_ABMODE_QUAD_16 | QSPI_CCR_ADMODE_QUAD_24   | QSPI_CCR_IMODE_SINGLE | 0xed)
#define STM32WB_BOOT_QSPI_CCR_READ_4_4_4_D2        (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_QUAD   | QSPI_CCR_DCYCLE(2) | QSPI_CCR_ABMODE_QUAD_8  | QSPI_CCR_ADMODE_QUAD_24   | QSPI_CCR_IMODE_QUAD   | 0x0b)
#define STM32WB_BOOT_QSPI_CCR_READ_4_4_4_M2D4      (QSPI_CCR_SDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_QUAD   | QSPI_CCR_DCYCLE(4) | QSPI_CCR_ABMODE_QUAD_8  | QSPI_CCR_ADMODE_QUAD_24   | QSPI_CCR_IMODE_QUAD   | 0xeb)
#define STM32WB_BOOT_QSPI_CCR_READ_4_4_4_M2D4_DTR  (QSPI_CCR_DDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_QUAD   | QSPI_CCR_DCYCLE(4) | QSPI_CCR_ABMODE_QUAD_16 | QSPI_CCR_ADMODE_QUAD_24   | QSPI_CCR_IMODE_QUAD   | 0xed)
#define STM32WB_BOOT_QSPI_CCR_READ_4_4_4_M2D6_DTR  (QSPI_CCR_DDR | QSPI_CCR_FMODE_READ  | QSPI_CCR_DMODE_QUAD   | QSPI_CCR_DCYCLE(6) | QSPI_CCR_ABMODE_QUAD_16 | QSPI_CCR_ADMODE_QUAD_24   | QSPI_CCR_IMODE_QUAD   | 0xed)


#define STM32WB_BOOT_QSPI_OPCODE_WRSR              0x01
#define STM32WB_BOOT_QSPI_OPCODE_RDSR              0x05
#define STM32WB_BOOT_QSPI_OPCODE_WREN              0x06
#define STM32WB_BOOT_QSPI_OPCODE_WRDI              0x04
#define STM32WB_BOOT_QSPI_OPCODE_RDID              0x9f
#define STM32WB_BOOT_QSPI_OPCODE_RDSFDP            0x5a
#define STM32WB_BOOT_QSPI_OPCODE_RSTEN             0x66
#define STM32WB_BOOT_QSPI_OPCODE_RST               0x99
#define STM32WB_BOOT_QSPI_OPCODE_DPD               0xb9
#define STM32WB_BOOT_QSPI_OPCODE_RDPD              0xab

#define STM32WB_BOOT_QSPI_SR_WIP                   0x01
#define STM32WB_BOOT_QSPI_SR_WEL                   0x02

typedef struct _stm32wb_boot_sflash_device_t {
    stm32wb_boot_sflash_info_t     info;
    uint32_t                       ccr_erase;
    uint32_t                       ccr_program;
    uint32_t                       ccr_read;
} stm32wb_boot_sflash_device_t;

static stm32wb_boot_sflash_device_t stm32wb_boot_sflash_device =
{
    .info = {
        .did = { 0, 0, 0 },
        .features = 0,
        .capacity = 0,
        .erase_size = 65536,
        .page_size = 256,
    },
    .ccr_erase = STM32WB_BOOT_QSPI_CCR_ERASE_64K,
    .ccr_program = STM32WB_BOOT_QSPI_CCR_PROGRAM_1_1_1,
    .ccr_read = STM32WB_BOOT_QSPI_CCR_READ_1_1_2_D8,
};

static inline __attribute__((optimize("O3"),always_inline)) uint8_t STM32WB_BOOT_QSPI_READ_8(QUADSPI_TypeDef *QSPI)
{
    volatile uint8_t *fifo = (volatile uint8_t*)((uint32_t)&QSPI->DR);

    return *fifo;
}

static inline __attribute__((optimize("O3"),always_inline)) uint16_t STM32WB_BOOT_QSPI_READ_16(QUADSPI_TypeDef *QSPI)
{
    volatile uint16_t *fifo = (volatile uint16_t*)((uint32_t)&QSPI->DR);

    return *fifo;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t STM32WB_BOOT_QSPI_READ_32(QUADSPI_TypeDef *QSPI)
{
    volatile uint32_t *fifo = (volatile uint32_t*)((uint32_t)&QSPI->DR);

    return *fifo;
}

static inline __attribute__((optimize("O3"),always_inline)) void STM32WB_BOOT_QSPI_WRITE_8(QUADSPI_TypeDef *QSPI, const uint8_t data)
{
    volatile uint8_t *fifo = (volatile uint8_t*)((uint32_t)&QSPI->DR);

    *fifo = data;
}

static inline __attribute__((optimize("O3"),always_inline)) void STM32WB_BOOT_QSPI_WRITE_16(QUADSPI_TypeDef *QSPI, const uint16_t data)
{
    volatile uint16_t *fifo = (volatile uint16_t*)((uint32_t)&QSPI->DR);

    *fifo = data;
}

static inline __attribute__((optimize("O3"),always_inline)) void STM32WB_BOOT_QSPI_WRITE_32(QUADSPI_TypeDef *QSPI, const uint32_t data)
{
    volatile uint32_t *fifo = (volatile uint32_t*)((uint32_t)&QSPI->DR);

    *fifo = data;
}

static void __attribute__((noinline)) stm32wb_boot_qspi_command(uint32_t ccr, uint32_t address)
{
    QUADSPI_TypeDef *QSPI = QUADSPI;

    while (QSPI->SR & QUADSPI_SR_BUSY)
    {
    }

    QSPI->CCR = ccr;

    if (ccr & QUADSPI_CCR_ADMODE)
    {
        QSPI->AR  = address;    
    }
}

static uint8_t __attribute__((noinline)) stm32wb_boot_qspi_command_status(uint32_t ccr, uint32_t address)
{
    QUADSPI_TypeDef *QSPI = QUADSPI;
    uint8_t status;
  
    while (QSPI->SR & QUADSPI_SR_BUSY)
    {
    }

    QSPI->DLR = 1 -1;
    QSPI->CCR = ccr;

    if (ccr & QUADSPI_CCR_ADMODE)
    {
        QSPI->AR  = address;    
    }

    while (!(QSPI->SR & QUADSPI_SR_TCF))
    {
    }

    status = STM32WB_BOOT_QSPI_READ_8(QSPI);

    QSPI->FCR = QUADSPI_FCR_CTCF;

    return status;
}

static void __attribute__((noinline)) stm32wb_boot_qspi_command_program(uint32_t ccr, uint32_t address, const uint8_t *data, uint32_t count)
{
    QUADSPI_TypeDef *QSPI = QUADSPI;
    uint32_t data_out;
    const uint32_t *data32, *data32_e;
    const uint16_t *data16;
    const uint8_t *data8;

    while (QSPI->SR & QUADSPI_SR_BUSY)
    {
    }
    
    QSPI->DLR = count -1;
    QSPI->CCR = ccr;

    if (ccr & QUADSPI_CCR_ADMODE)
    {
        QSPI->AR  = address;    
    }

    data32 = (const uint32_t*)(data);

    if (count & ~3)
    {
        data32_e = (const uint32_t*)(data + (count & ~3));

        do
        {
            while (!(QSPI->SR & QUADSPI_SR_FTF))
            {
            }
            
            __asm__ volatile("": : : "memory");
            
            data_out = data32[0];
            
            data32 += 1;
            
            __asm__ volatile("": : : "memory");
            
            STM32WB_BOOT_QSPI_WRITE_32(QSPI, data_out);

            __asm__ volatile("": : : "memory");
        }
        while (data32 != data32_e);
    }

    if (count & 3)
    {
        while (!(QSPI->SR & QUADSPI_SR_FTF))
        {
        }

        data16 = (const uint16_t*)data32;
        
        if (count & 2)
        {
            __asm__ volatile("": : : "memory");
            
            data_out = data16[0];
            
            data16 += 1;

            __asm__ volatile("": : : "memory");
            
            STM32WB_BOOT_QSPI_WRITE_16(QSPI, data_out);
            
            __asm__ volatile("": : : "memory");
        }

        data8 = (const uint8_t*)data16;
        
        if (count & 1)
        {
            __asm__ volatile("": : : "memory");
            
            data_out = data8[0];
            
            data8 += 1;
            
            __asm__ volatile("": : : "memory");
            
            STM32WB_BOOT_QSPI_WRITE_8(QSPI, data_out);
            
            __asm__ volatile("": : : "memory");
        }
    }
    
    QSPI->FCR = QUADSPI_FCR_CTCF;
}

static void __attribute__((noinline)) stm32wb_boot_qspi_command_read(uint32_t ccr, uint32_t address, uint8_t *data, uint32_t count)
{
    QUADSPI_TypeDef *QSPI = QUADSPI;
    uint32_t data_in;
    uint32_t *data32, *data32_e;
    uint16_t *data16;
    uint8_t *data8;

    while (QSPI->SR & QUADSPI_SR_BUSY)
    {
    }
    
    QSPI->DLR = count -1;
    QSPI->CCR = ccr;

    if (ccr & QUADSPI_CCR_ADMODE)
    {
        QSPI->AR  = address;    
    }

    data32 = (uint32_t*)(data);

    if (count & ~3)
    {
        data32_e = (uint32_t*)(data + (count & ~3));
        
        do
        {
            while (!(QSPI->SR & QUADSPI_SR_FTF))
            {
            }

            __asm__ volatile("": : : "memory");
            
            data_in = STM32WB_BOOT_QSPI_READ_32(QSPI);
            
            __asm__ volatile("": : : "memory");
            
            data32[0] = data_in;
            
            data32 += 1;

            __asm__ volatile("": : : "memory");
        }
        while (data32 != data32_e);
    }

    if (count & 3)
    {
        while (!(QSPI->SR & QUADSPI_SR_TCF))
        {
        }

        data16 = (uint16_t*)data32;
        
        if (count & 2)
        {
            __asm__ volatile("": : : "memory");
            
            data_in = STM32WB_BOOT_QSPI_READ_16(QSPI);
            
            __asm__ volatile("": : : "memory");
            
            data16[0] = data_in;
            
            data16 += 1;

            __asm__ volatile("": : : "memory");
        }

        data8 = (uint8_t*)data16;
        
        if (count & 1)
        {
            __asm__ volatile("": : : "memory");
            
            data_in = STM32WB_BOOT_QSPI_READ_8(QSPI);
            
            __asm__ volatile("": : : "memory");
            
            data8[0] = data_in;
            
            data8 += 1;

            __asm__ volatile("": : : "memory");
        }
    }

    QSPI->FCR = QUADSPI_FCR_CTCF;
}

static bool __attribute__((noinline)) stm32wb_boot_qspi_command_verify(uint32_t ccr, uint32_t address, const uint8_t *data, uint32_t count)
{
    QUADSPI_TypeDef *QSPI = QUADSPI;
    uint32_t data_in, data_verify;
    uint32_t *data32, *data32_e;
    uint16_t *data16;
    uint8_t *data8;

    while (QSPI->SR & QUADSPI_SR_BUSY)
    {
    }
    
    QSPI->DLR = count -1;
    QSPI->CCR = ccr;

    if (ccr & QUADSPI_CCR_ADMODE)
    {
        QSPI->AR  = address;    
    }

    data_verify = 0;
    
    data32 = (uint32_t*)(data);

    if (count & ~3)
    {
        data32_e = (uint32_t*)(data + (count & ~3));
        
        do
        {
            while (!(QSPI->SR & QUADSPI_SR_FTF))
            {
            }

            __asm__ volatile("": : : "memory");
            
            data_in = STM32WB_BOOT_QSPI_READ_32(QSPI);
            
            __asm__ volatile("": : : "memory");
            
            data_verify |= (data32[0] ^ data_in);
            
            data32 += 1;

            __asm__ volatile("": : : "memory");
        }
        while (data32 != data32_e);
    }

    if (count & 3)
    {
        while (!(QSPI->SR & QUADSPI_SR_TCF))
        {
        }

        data16 = (uint16_t*)data32;
        
        if (count & 2)
        {
            __asm__ volatile("": : : "memory");
            
            data_in = STM32WB_BOOT_QSPI_READ_16(QSPI);
            
            __asm__ volatile("": : : "memory");
            
            data_verify |= (data16[0] ^ data_in);
            
            data16 += 1;

            __asm__ volatile("": : : "memory");
        }

        data8 = (uint8_t*)data16;
        
        if (count & 1)
        {
            __asm__ volatile("": : : "memory");
            
            data_in = STM32WB_BOOT_QSPI_READ_8(QSPI);
            
            __asm__ volatile("": : : "memory");
            
            data_verify |= (data8[0] ^ data_in);
            
            data8 += 1;

            __asm__ volatile("": : : "memory");
        }
    }

    QSPI->FCR = QUADSPI_FCR_CTCF;

    return (data_verify == 0);
}

static void stm32wb_boot_sflash_wait()
{
    uint8_t sr;
    
    do
    {
        sr = stm32wb_boot_qspi_command_status((STM32WB_BOOT_QSPI_CCR_COMMAND_STATUS | STM32WB_BOOT_QSPI_OPCODE_RDSR), 0);
    }
    while (sr & STM32WB_BOOT_QSPI_SR_WIP);
}

static stm32wb_boot_sflash_info_t * __attribute__((noinline)) stm32wb_boot_sflash_init(void)
{
    QUADSPI_TypeDef *QSPI = QUADSPI;
    uint8_t did[3], sr, sr1, sr2, cr[2];
    bool busy;
    uint32_t features;
    
    if (!stm32wb_boot_sflash_device.info.did[0])
    {
        if (stm32wb_boot_info.options & STM32WB_SYSTEM_OPTION_SFLASH_BOOST)
        {
            stm32wb_boot_gpio_pin_write(stm32wb_boot_info.pins.boost, 1);

            armv7m_core_udelay(400);
        }
        
        stm32wb_boot_gpio_pin_configure(stm32wb_boot_info.pins.sflash_cs,   (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
        stm32wb_boot_gpio_pin_configure(stm32wb_boot_info.pins.sflash_clk,  (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
        stm32wb_boot_gpio_pin_configure(stm32wb_boot_info.pins.sflash_mosi, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
        stm32wb_boot_gpio_pin_configure(stm32wb_boot_info.pins.sflash_miso, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));

        if ((stm32wb_boot_info.pins.sflash_wp != STM32WB_GPIO_PIN_NONE) && (stm32wb_boot_info.pins.sflash_hold != STM32WB_GPIO_PIN_NONE))
        {
            stm32wb_boot_gpio_pin_configure(stm32wb_boot_info.pins.sflash_wp,   (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));
            stm32wb_boot_gpio_pin_configure(stm32wb_boot_info.pins.sflash_hold, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_OSPEED_VERY_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_ALTERNATE));

            features = STM32WB_BOOT_SFLASH_FEATURE_QUAD;

            stm32wb_boot_sflash_device.ccr_read = STM32WB_BOOT_QSPI_CCR_READ_1_1_4_D8;
        }
        else
        {
            features = 0;
        }

        /* Max QUADSPI clk is 60MHz for SDR and 48MHz for DDR, we are running at 64MHz, so prescaling is needed.
         */
    
        RCC->AHB3ENR |= RCC_AHB3ENR_QUADSPIEN;
        RCC->AHB3ENR;
    
        QSPI->DCR = QUADSPI_DCR_FSIZE | ((2 -1) << QUADSPI_DCR_CSHT_Pos);
        QSPI->ABR = 0xfffffff;
        QSPI->CR = ((2-1) << QUADSPI_CR_PRESCALER_Pos) | ((8-1) << QUADSPI_CR_FTHRES_Pos) | QUADSPI_CR_EN;
        
        if (stm32wb_boot_info.pins.sflash_enable != STM32WB_GPIO_PIN_NONE)
        {
            stm32wb_boot_gpio_pin_configure(stm32wb_boot_info.pins.sflash_enable, (STM32WB_GPIO_PUPD_NONE | STM32WB_GPIO_ODATA_1 | STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));

            armv7m_core_udelay(3000);
        }
        
        /* A tad messy. First a possible QPI mode needs to be exited. And the of course
         * before that Deep Power Down needs to be released before that (QPI) and then
         * afterwards (non-QPI).
         */
        stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND_QPI | STM32WB_BOOT_QSPI_OPCODE_RDPD), 0);

        armv7m_core_udelay(250);

        stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND_QPI | STM32WB_BOOT_QSPI_OPCODE_RSTEN), 0);
        stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND_QPI | STM32WB_BOOT_QSPI_OPCODE_RST), 0);

        armv7m_core_udelay(100);
        
        stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND | STM32WB_BOOT_QSPI_OPCODE_RDPD), 0);
    
        armv7m_core_udelay(250);

        stm32wb_boot_qspi_command_read(STM32WB_BOOT_QSPI_CCR_RDID, 0, &did[0], 3);

        if (did[0] == 0x00)
        {
            return NULL;
        }

        /* Configure MX25Rxx35 for "High Performance Mode" ...
         */
        if ((did[0] == STM32WB_BOOT_QSPI_MID_MACRONIX) && (did[1] == 0x28))
        {
            stm32wb_boot_qspi_command_read((STM32WB_BOOT_QSPI_CCR_COMMAND_READ | 0x15), 0, &cr[0], 2);
            
            if (!(cr[1] & 0x02))
            {
                sr = stm32wb_boot_qspi_command_status((STM32WB_BOOT_QSPI_CCR_COMMAND_STATUS | STM32WB_BOOT_QSPI_OPCODE_RDSR), 0);

                stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND | STM32WB_BOOT_QSPI_OPCODE_WREN), 0);
                stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND_WRITE_3 | STM32WB_BOOT_QSPI_OPCODE_WRSR), ((sr << 16) | (cr[0] << 8) | (cr[1] | 0x02)));
		
                stm32wb_boot_sflash_wait();
	    }
        }

        if (features & STM32WB_BOOT_SFLASH_FEATURE_QUAD)
        {
            busy = false;
            
            if (did[0] == STM32WB_BOOT_QSPI_MID_MACRONIX)
            {
                sr1 = stm32wb_boot_qspi_command_status((STM32WB_BOOT_QSPI_CCR_COMMAND_STATUS | STM32WB_BOOT_QSPI_OPCODE_RDSR), 0);
                
                if (!(sr1 & 0x40))
                {
                    stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND | STM32WB_BOOT_QSPI_OPCODE_WREN), 0);
                    stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND_WRITE_1 | STM32WB_BOOT_QSPI_OPCODE_WRSR), (sr1 | 0x40));
                    
                    busy = true;
                }
            }
            else
            {
                sr1 = stm32wb_boot_qspi_command_status((STM32WB_BOOT_QSPI_CCR_COMMAND_STATUS | STM32WB_BOOT_QSPI_OPCODE_RDSR), 0);
                sr2 = stm32wb_boot_qspi_command_status((STM32WB_BOOT_QSPI_CCR_COMMAND_STATUS | 0x35), 0);
                    
                if (!(sr2 & 0x02))
                {
                    stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND | STM32WB_BOOT_QSPI_OPCODE_WREN), 0);
                    stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND_WRITE_2 | STM32WB_BOOT_QSPI_OPCODE_WRSR), ((sr1 << 8) | (sr2 | 0x02)));
                        
                    busy = true;
                }
            }

            if (busy)
            {
                do
                {
                    sr = stm32wb_boot_qspi_command_status((STM32WB_BOOT_QSPI_CCR_COMMAND_STATUS | STM32WB_BOOT_QSPI_OPCODE_RDSR), 0);
                }
                while (sr & STM32WB_BOOT_QSPI_SR_WIP);
            }

            stm32wb_boot_sflash_device.info.capacity = 1 << (did[2] & 0x1f);
        
            if (stm32wb_boot_sflash_device.info.capacity > 0x01000000)
            {
                stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND | 0xe9), 0x00);
                stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND_WRITE_1 | 0xc5), 0x00);
            }
        }
        
        stm32wb_boot_sflash_device.info.did[0] = did[0];
        stm32wb_boot_sflash_device.info.did[1] = did[1];
        stm32wb_boot_sflash_device.info.did[2] = did[2];
        stm32wb_boot_sflash_device.info.features = features;
    }

    return &stm32wb_boot_sflash_device.info;
}

static bool stm32wb_boot_sflash_erase(uint32_t address)
{
    stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND | STM32WB_BOOT_QSPI_OPCODE_WREN), 0);
    
    stm32wb_boot_qspi_command(stm32wb_boot_sflash_device.ccr_erase, address);

    stm32wb_boot_sflash_wait();
    
    return true;
}

static void stm32wb_boot_sflash_program(uint32_t address, const uint8_t *data, uint32_t count)
{
    stm32wb_boot_qspi_command((STM32WB_BOOT_QSPI_CCR_COMMAND | STM32WB_BOOT_QSPI_OPCODE_WREN), 0);
    
    stm32wb_boot_qspi_command_program(stm32wb_boot_sflash_device.ccr_program, address, data, count);

    stm32wb_boot_sflash_wait();
}

static void stm32wb_boot_sflash_read(uint32_t address, uint8_t *data, uint32_t count)
{
    stm32wb_boot_qspi_command_read(stm32wb_boot_sflash_device.ccr_read, address, data, count);
}

static bool stm32wb_boot_sflash_verify(uint32_t address, const uint8_t *data, uint32_t count)
{
    return stm32wb_boot_qspi_command_verify(stm32wb_boot_sflash_device.ccr_read, address, data, count);
}

#endif /* (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1) */

/************************************************************************************************************************************/

#if (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1)

typedef struct _stm32wb_boot_nvm_control_t {
    stm32wb_boot_aes128_context_t  aes128_ctx;
    uint32_t                       aes128_iv[4];
} stm32wb_boot_nvm_control_t;

static stm32wb_boot_nvm_control_t stm32wb_boot_nvm_control;

static void __attribute__((noinline)) stm32wb_boot_nvm_set_key(const uint32_t *key)
{
    stm32wb_boot_aes128_set_key(&stm32wb_boot_nvm_control.aes128_ctx, key);

    stm32wb_boot_nvm_control.aes128_iv[0] = 0;
    stm32wb_boot_nvm_control.aes128_iv[1] = 0;
    stm32wb_boot_nvm_control.aes128_iv[2] = 0;
    stm32wb_boot_nvm_control.aes128_iv[3] = 0;
    
}

static bool __attribute__((noinline)) stm32wb_boot_nvm_erase(uint32_t address)
{
    if (address < STM32WB_BOOT_SFLASH_LIMIT)
    {
        return stm32wb_boot_sflash_erase(address);
    }

    return stm32wb_boot_flash_erase(address);
}

static bool __attribute__((noinline)) stm32wb_boot_nvm_program(uint32_t address, const uint8_t *data, uint32_t count, bool encrypted)
{
    uint32_t program_data[STM32WB_BOOT_NVM_PROGRAM_SIZE / 4];

    if (address < STM32WB_BOOT_SFLASH_LIMIT)
    {
        if (encrypted)
        {
            stm32wb_boot_nvm_control.aes128_iv[3] = SWAP(address / 16);
            
            stm32wb_boot_aes128_ctr_encrypt(&stm32wb_boot_nvm_control.aes128_ctx, &stm32wb_boot_nvm_control.aes128_iv[0], (const uint32_t*)data, (uint32_t*)&program_data[0], count);
            
            data = (const uint8_t*)&program_data[0];
        }
        
        stm32wb_boot_sflash_program(address, data, count);

        return stm32wb_boot_sflash_verify(address, data, count);
    }

    return stm32wb_boot_flash_program(address, data, count);
}

static void __attribute__((noinline)) stm32wb_boot_nvm_read(uint32_t address, uint8_t *data, uint32_t count, bool encrypted)
{
    if (address < STM32WB_BOOT_SFLASH_LIMIT)
    {
        stm32wb_boot_sflash_read(address, data, count);

        if (encrypted)
        {
            stm32wb_boot_nvm_control.aes128_iv[3] = SWAP(address / 16);
            
            stm32wb_boot_aes128_ctr_encrypt(&stm32wb_boot_nvm_control.aes128_ctx, &stm32wb_boot_nvm_control.aes128_iv[0], (const uint32_t*)data, (uint32_t*)data, count);
        }
        
        return;
    }

    stm32wb_boot_memcpy(data, (const uint8_t*)address, count);
}

#else /* (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1) */

static bool stm32wb_boot_nvm_erase(uint32_t address)
{
    return stm32wb_boot_flash_erase(address);
}

static bool stm32wb_boot_nvm_program(uint32_t address, const uint8_t *data, uint32_t count, bool encrypted)
{
    return stm32wb_boot_flash_program(address, data, count);
}

static void stm32wb_boot_nvm_read(uint32_t address, uint8_t *data, uint32_t count, bool encrypted)
{
    stm32wb_boot_memcpy(data, (const uint8_t*)address, count);
}

#endif /* (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1) */

/************************************************************************************************************************************/

static void __attribute__((noinline)) stm32wb_boot_random(uint8_t *data, uint32_t count)
{
    uint32_t random[4];
    uint32_t sr;
    uint8_t *data_e;
    
    RCC->AHB3ENR |= RCC_AHB3ENR_RNGEN;
    RCC->AHB3ENR;

    RNG->CR |= RNG_CR_RNGEN;

    data_e = data + count;

    while (data < data_e)
    {
        do
        {
            sr = RNG->SR;
        }
        while (!(sr & (RNG_SR_SECS | RNG_SR_DRDY)));

        random[0] = RNG->DR;
        random[1] = RNG->DR;
        random[2] = RNG->DR;
        random[3] = RNG->DR;

        if (!(sr & RNG_SR_SECS))
        {
            count = sizeof(random);
            
            if (count > (uint32_t)(data_e - data))
            {
                count = (uint32_t)(data_e - data);
            }
            
            stm32wb_boot_memcpy(data, (const uint8_t*)&random[0], count);
            
            data += count;
        }
    }
    
    RNG->CR &= ~RNG_CR_RNGEN;
    
    RCC->AHB3ENR &= ~RCC_AHB3ENR_RNGEN;
}

/************************************************************************************************************************************/

static uint32_t __attribute__((noinline)) stm32wb_boot_crc32(const uint8_t *data, uint32_t size, uint32_t crc32)
{
    const uint32_t *in, *in_e;
    
    if (size)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;

        in = (const uint32_t*)data;
        in_e = (const uint32_t*)(data + size);

        CRC->INIT = __RBIT(crc32);
        CRC->POL  = 0x04c11db7;
        CRC->CR   = CRC_CR_RESET | CRC_CR_REV_IN_0 | CRC_CR_REV_IN_1 | CRC_CR_REV_OUT;

        do
        {
            CRC->DR = *in++;
        }
        while (in != in_e);

        crc32 = CRC->DR;
        
        RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
    }

    return crc32;
}

/************************************************************************************************************************************/

static void  __attribute__((noinline)) stm32wb_boot_aes128_get_key(const uint32_t *nonce, uint32_t *key)
{
    stm32wb_boot_aes128_context_t aes128_ctx;
    uint32_t aes128_key[4];
    stm32wb_boot_sha256_context_t sha256_ctx;
    uint32_t sha256_hash[STM32WB_BOOT_SHA256_HASH_SIZE / 4];
    uint32_t idcode;
    
    ((stm32wb_boot_aes128_get_key_routine_t)(STM32WB_BOOT_AES128_KEY | 1))(&aes128_key[0]);

    stm32wb_boot_aes128_set_key(&aes128_ctx, &aes128_key[0]);
    
    idcode = STM32WB_FWU_IDCODE_STM32WB55;
            
    stm32wb_boot_sha256_init(&sha256_ctx);
    stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)&stm32wb_boot_info.uuid[0], 16);
    stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)&idcode, 4);
    stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)&nonce[0], 12);
    stm32wb_boot_sha256_final(&sha256_ctx, sha256_hash);
    
    stm32wb_boot_aes128_ecb_encrypt(&aes128_ctx, &sha256_hash[0], &key[0], 16);
}

static void __attribute__((noinline)) stm32wb_boot_aes128_set_key(stm32wb_boot_aes128_context_t *aes128_ctx, const uint32_t *key)
{
    aes128_ctx->K[0] = SWAP(key[3]);
    aes128_ctx->K[1] = SWAP(key[2]);
    aes128_ctx->K[2] = SWAP(key[1]);
    aes128_ctx->K[3] = SWAP(key[0]);
}

static void __attribute__((noinline)) stm32wb_boot_aes128_encrypt(stm32wb_boot_aes128_context_t *aes128_ctx, uint32_t mode, const uint32_t *iv, const uint32_t *in, uint32_t *out, uint32_t count)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_AES1EN;
    RCC->AHB2ENR;

    AES1->KEYR0 = aes128_ctx->K[0];
    AES1->KEYR1 = aes128_ctx->K[1];
    AES1->KEYR2 = aes128_ctx->K[2];
    AES1->KEYR3 = aes128_ctx->K[3];
    
    AES1->CR = AES_CR_DATATYPE_1 | mode;

    if (iv)
    {
        AES1->IVR0 = SWAP(iv[3]);
        AES1->IVR1 = SWAP(iv[2]);
        AES1->IVR2 = SWAP(iv[1]);
        AES1->IVR3 = SWAP(iv[0]);
    }

    AES1->CR = AES_CR_EN | AES_CR_DATATYPE_1 | mode;

    for (; count >= 16; count -= 16, in += 4, out += 4)
    {
        AES1->DINR = in[0];
        AES1->DINR = in[1];
        AES1->DINR = in[2];
        AES1->DINR = in[3];
        
        while (!(AES1->SR & AES_SR_CCF))
        {
        }
        
        AES1->CR |= AES_CR_CCFC;
        
        out[0] = AES1->DOUTR;
        out[1] = AES1->DOUTR;
        out[2] = AES1->DOUTR;
        out[3] = AES1->DOUTR;
    }
    
    AES1->CR &= ~AES_CR_EN;
    
    AES1->KEYR0 = 0;
    AES1->KEYR1 = 0;
    AES1->KEYR2 = 0;
    AES1->KEYR3 = 0;
    
    RCC->AHB2ENR &= ~RCC_AHB2ENR_AES1EN;
}

static void stm32wb_boot_aes128_ecb_encrypt(stm32wb_boot_aes128_context_t *aes128_ctx, const uint32_t *in, uint32_t *out, uint32_t count)
{
    stm32wb_boot_aes128_encrypt(aes128_ctx, (0), NULL, in, out, count);
}

static void stm32wb_boot_aes128_ctr_encrypt(stm32wb_boot_aes128_context_t *aes128_ctx, const uint32_t *iv, const uint32_t *in, uint32_t *out, uint32_t count)
{
    stm32wb_boot_aes128_encrypt(aes128_ctx, (AES_CR_CHMOD_1), iv, in, out, count);
}

/************************************************************************************************************************************/

static uint32_t __attribute__((used)) stm32wb_boot_sha256_const_K[64] =
{
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

static const uint32_t stm32wb_boot_sha256_const_H[8] =
{
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19, 
};

static void __attribute__((naked, noinline)) stm32wb_boot_sha256_compress(uint32_t *state, const uint32_t *data, const uint32_t *data_e)
{
    register uint32_t _state  __asm__("r0") = (uint32_t)state;
    register uint32_t _data   __asm__("r1") = (uint32_t)data;
    register uint32_t _data_e __asm__("r2") = (uint32_t)data_e;

    __asm__(
        "    push    { r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, r11, lr }    \n"
        "    mov     lr, sp                                                  \n"
        "    mov     r8, r1                                                  \n"
        "                                                                    \n"
        "1:  sub     sp, #256                                                \n"
        "    mov     r9, sp                                                  \n"
        "                                                                    \n"
        "    ldmia   r8!, { r0, r1, r2, r3, r4, r5, r6, r7 }                 \n"
        "    rev     r0, r0                                                  \n"
        "    rev     r1, r1                                                  \n"
        "    rev     r2, r2                                                  \n"
        "    rev     r3, r3                                                  \n"
        "    rev     r4, r4                                                  \n"
        "    rev     r5, r5                                                  \n"
        "    rev     r6, r6                                                  \n"
        "    rev     r7, r7                                                  \n"
        "    stmia   r9!, { r0, r1, r2, r3, r4, r5, r6, r7 }                 \n"
        "    ldmia   r8!, { r0, r1, r2, r3, r4, r5, r6, r7 }                 \n"
        "    rev     r0, r0                                                  \n"
        "    rev     r1, r1                                                  \n"
        "    rev     r2, r2                                                  \n"
        "    rev     r3, r3                                                  \n"
        "    rev     r4, r4                                                  \n"
        "    rev     r5, r5                                                  \n"
        "    rev     r6, r6                                                  \n"
        "    rev     r7, r7                                                  \n"
        "    stmia   r9!, { r0, r1, r2, r3, r4, r5, r6, r7 }                 \n"
        "    str     r8, [sp, #260]                                          \n"
        "                                                                    \n"
        "    mov     r0, sp                                                  \n"
        "    add     r1, r0, #192                                            \n"
        "    ldrd    r4, r5, [r0, #56]                                       \n"
        "                                                                    \n"
        "2:  ror     r2, r4, #17                                             \n"
        "    eor     r2, r2, r4, ror #19                                     \n"
        "    eor     r2, r2, r4, lsr #10                                     \n"
        "    ror     r3, r5, #17                                             \n"
        "    eor     r3, r3, r5, ror #19                                     \n"
        "    eor     r3, r3, r5, lsr #10                                     \n"
        "    ldrd    r4, r5, [r0, #36]                                       \n"
        "    adds    r2, r4                                                  \n"
        "    adds    r3, r5                                                  \n"
        "    ldmia   r0, { r6, r7, r8 }                                      \n"
        "    ror     r4, r7, #7                                              \n"
        "    eor     r4, r4, r7, ror #18                                     \n"
        "    eor     r4, r4, r7, lsr #3                                      \n"
        "    adds    r4, r2                                                  \n"
        "    ror     r5, r8, #7                                              \n"
        "    eor     r5, r5, r8, ror #18                                     \n"
        "    eor     r5, r5, r8, lsr #3                                      \n"
        "    adds    r5, r3                                                  \n"
        "    adds    r4, r6                                                  \n"
        "    adds    r5, r7                                                  \n"
        "    strd    r4, r5, [r0, #64]                                       \n"
        "    adds    r0, #8                                                  \n"
        "    cmp     r0, r1                                                  \n"
        "    bne     2b                                                      \n"
        "                                                                    \n"
        "    ldr     r12, [sp, #256]                                         \n"
        "    ldmia   r12, { r2, r3, r4, r5, r6, r7, r8, r9 }                 \n"
        "    movw    r12, #:lower16:stm32wb_boot_sha256_const_K              \n"
        "    movt    r12, #:upper16:stm32wb_boot_sha256_const_K              \n"
        "                                                                    \n"
        "3:  ror     r0, r6, #6                                              \n"
        "    eor     r0, r0, r6, ror #11                                     \n"
        "    eor     r0, r0, r6, ror #25                                     \n"
        "    add     r9, r0                                                  \n"
        "    eor     r0, r7, r8                                              \n"
        "    ands    r0, r6                                                  \n"
        "    eor     r0, r8                                                  \n"
        "    add     r9, r0                                                  \n"
        "    ldrd    r0, r10, [r12], #8                                      \n"
        "    add     r9, r0                                                  \n"
        "    ldrd    r1, r11, [sp], #8                                       \n"
        "    add     r9, r1                                                  \n"
        "    add     r5, r9                                                  \n"
        "    ror     r0, r2, #2                                              \n"
        "    eor     r0, r0, r2, ror #13                                     \n"
        "    eor     r0, r0, r2, ror #22                                     \n"
        "    add     r9, r0                                                  \n"
        "    orr     r0, r3, r4                                              \n"
        "    and     r1, r3, r4                                              \n"
        "    ands    r0, r2                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    add     r9, r0                                                  \n"
        "    ror     r0, r5, #6                                              \n"
        "    eor     r0, r0, r5, ror #11                                     \n"
        "    eor     r0, r0, r5, ror #25                                     \n"
        "    add     r8, r0                                                  \n"
        "    eor     r0, r6, r7                                              \n"
        "    ands    r0, r5                                                  \n"
        "    eors    r0, r7                                                  \n"
        "    add     r8, r0                                                  \n"
        "    add     r8, r10                                                 \n"
        "    add     r8, r11                                                 \n"
        "    add     r4, r8                                                  \n"
        "    ror     r0, r9, #2                                              \n"
        "    eor     r0, r0, r9, ror #13                                     \n"
        "    eor     r0, r0, r9, ror #22                                     \n"
        "    add     r8, r0                                                  \n"
        "    orr     r0, r2, r3                                              \n"
        "    and     r1, r2, r3                                              \n"
        "    and     r0, r9                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    add     r8, r0                                                  \n"
        "    ror     r0, r4, #6                                              \n"
        "    eor     r0, r0, r4, ror #11                                     \n"
        "    eor     r0, r0, r4, ror #25                                     \n"
        "    adds    r7, r0                                                  \n"
        "    eor     r0, r5, r6                                              \n"
        "    ands    r0, r4                                                  \n"
        "    eors    r0, r6                                                  \n"
        "    adds    r7, r0                                                  \n"
        "    ldrd    r0, r10, [r12], #8                                      \n"
        "    adds    r7, r0                                                  \n"
        "    ldrd    r1, r11, [sp], #8                                       \n"
        "    adds    r7, r1                                                  \n"
        "    adds    r3, r7                                                  \n"
        "    ror     r0, r8, #2                                              \n"
        "    eor     r0, r0, r8, ror #13                                     \n"
        "    eor     r0, r0, r8, ror #22                                     \n"
        "    adds    r7, r0                                                  \n"
        "    orr     r0, r9, r2                                              \n"
        "    and     r1, r9, r2                                              \n"
        "    and     r0, r8                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    adds    r7, r0                                                  \n"
        "    ror     r0, r3, #6                                              \n"
        "    eor     r0, r0, r3, ror #11                                     \n"
        "    eor     r0, r0, r3, ror #25                                     \n"
        "    adds    r6, r0                                                  \n"
        "    eor     r0, r4, r5                                              \n"
        "    ands    r0, r3                                                  \n"
        "    eors    r0, r5                                                  \n"
        "    adds    r6, r0                                                  \n"
        "    add     r6, r10                                                 \n"
        "    add     r6, r11                                                 \n"
        "    adds    r2, r6                                                  \n"
        "    ror     r0, r7, #2                                              \n"
        "    eor     r0, r0, r7, ror #13                                     \n"
        "    eor     r0, r0, r7, ror #22                                     \n"
        "    adds    r6, r0                                                  \n"
        "    orr     r0, r8, r9                                              \n"
        "    and     r1, r8, r9                                              \n"
        "    ands    r0, r7                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    adds    r6, r0                                                  \n"
        "    ror     r0, r2, #6                                              \n"
        "    eor     r0, r0, r2, ror #11                                     \n"
        "    eor     r0, r0, r2, ror #25                                     \n"
        "    adds    r5, r0                                                  \n"
        "    eor     r0, r3, r4                                              \n"
        "    ands    r0, r2                                                  \n"
        "    eors    r0, r4                                                  \n"
        "    adds    r5, r0                                                  \n"
        "    ldrd    r0, r10, [r12], #8                                      \n"
        "    adds    r5, r0                                                  \n"
        "    ldrd    r1, r11, [sp], #8                                       \n"
        "    adds    r5, r1                                                  \n"
        "    add     r9, r5                                                  \n"
        "    ror     r0, r6, #2                                              \n"
        "    eor     r0, r0, r6, ror #13                                     \n"
        "    eor     r0, r0, r6, ror #22                                     \n"
        "    adds    r5, r0                                                  \n"
        "    orr     r0, r7, r8                                              \n"
        "    and     r1, r7, r8                                              \n"
        "    ands    r0, r6                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    adds    r5, r0                                                  \n"
        "    ror     r0, r9, #6                                              \n"
        "    eor     r0, r0, r9, ror #11                                     \n"
        "    eor     r0, r0, r9, ror #25                                     \n"
        "    adds    r4, r0                                                  \n"
        "    eor     r0, r2, r3                                              \n"
        "    and     r0, r9                                                  \n"
        "    eors    r0, r3                                                  \n"
        "    adds    r4, r0                                                  \n"
        "    add     r4, r10                                                 \n"
        "    add     r4, r11                                                 \n"
        "    add     r8, r4                                                  \n"
        "    ror     r0, r5, #2                                              \n"
        "    eor     r0, r0, r5, ror #13                                     \n"
        "    eor     r0, r0, r5, ror #22                                     \n"
        "    adds    r4, r0                                                  \n"
        "    orr     r0, r6, r7                                              \n"
        "    and     r1, r6, r7                                              \n"
        "    ands    r0, r5                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    adds    r4, r0                                                  \n"
        "    ror     r0, r8, #6                                              \n"
        "    eor     r0, r0, r8, ror #11                                     \n"
        "    eor     r0, r0, r8, ror #25                                     \n"
        "    adds    r3, r0                                                  \n"
        "    eor     r0, r9, r2                                              \n"
        "    and     r0, r8                                                  \n"
        "    eors    r0, r2                                                  \n"
        "    adds    r3, r0                                                  \n"
        "    ldrd    r0, r10, [r12], #8                                      \n"
        "    adds    r3, r0                                                  \n"
        "    ldrd    r1, r11, [sp], #8                                       \n"
        "    adds    r3, r1                                                  \n"
        "    adds    r7, r3                                                  \n"
        "    ror     r0, r4, #2                                              \n"
        "    eor     r0, r0, r4, ror #13                                     \n"
        "    eor     r0, r0, r4, ror #22                                     \n"
        "    adds    r3, r0                                                  \n"
        "    orr     r0, r5, r6                                              \n"
        "    and     r1, r5, r6                                              \n"
        "    ands    r0, r4                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    adds    r3, r0                                                  \n"
        "    ror     r0, r7, #6                                              \n"
        "    eor     r0, r0, r7, ror #11                                     \n"
        "    eor     r0, r0, r7, ror #25                                     \n"
        "    adds    r2, r0                                                  \n"
        "    eor     r0, r8, r9                                              \n"
        "    ands    r0, r7                                                  \n"
        "    eor     r0, r9                                                  \n"
        "    adds    r2, r0                                                  \n"
        "    add     r2, r10                                                 \n"
        "    add     r2, r11                                                 \n"
        "    adds    r6, r2                                                  \n"
        "    ror     r0, r3, #2                                              \n"
        "    eor     r0, r0, r3, ror #13                                     \n"
        "    eor     r0, r0, r3, ror #22                                     \n"
        "    adds    r2, r0                                                  \n"
        "    orr     r0, r4, r5                                              \n"
        "    and     r1, r4, r5                                              \n"
        "    ands    r0, r3                                                  \n"
        "    orrs    r0, r1                                                  \n"
        "    adds    r2, r0                                                  \n"
        "    cmp     sp, lr                                                  \n"
        "    bne     3b                                                      \n"
        "                                                                    \n"
        "    ldr     r12, [sp, #0]                                           \n"
        "    ldmia   r12, { r0, r1, r10, r11 }                               \n"
        "    adds    r2, r0                                                  \n"
        "    adds    r3, r1                                                  \n"
        "    add     r4, r10                                                 \n"
        "    add     r5, r11                                                 \n"
        "    stmia   r12!, { r2, r3, r4, r5 }                                \n"
        "    ldmia   r12, { r2, r3, r4, r5 }                                 \n"
        "    adds    r2, r6                                                  \n"
        "    adds    r3, r7                                                  \n"
        "    add     r4, r8                                                  \n"
        "    add     r5, r9                                                  \n"
        "    stmia   r12!, { r2, r3, r4, r5 }                                \n"
        "                                                                    \n"
        "    ldrd    r8, r9, [sp, #4]                                        \n"
        "    cmp     r8, r9                                                  \n"
        "    bne     1b                                                      \n"
        "                                                                    \n"
        "    pop     { r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, r11, pc }    \n"
        :
        : "r" (_state), "r" (_data), "r" (_data_e)
        );
}

static void __attribute__((noinline)) stm32wb_boot_sha256_init(stm32wb_boot_sha256_context_t *sha256_ctx)
{
    sha256_ctx->length = 0;
    sha256_ctx->index = 0;

    stm32wb_boot_memcpy(&sha256_ctx->hash[0], &stm32wb_boot_sha256_const_H[0], 32);
}

static void __attribute__((noinline)) stm32wb_boot_sha256_update(stm32wb_boot_sha256_context_t *sha256_ctx, const uint8_t *data, size_t size)
{
    uint32_t index, count;
  
    sha256_ctx->length += size;

    index = sha256_ctx->index;
    
    while (size)
    {
        if ((index == 0) && !((uint32_t)data & 3) && (size & ~63))
        {
            count = size & ~63;

            stm32wb_boot_sha256_compress(&sha256_ctx->hash[0], (const uint32_t*)data, (const uint32_t*)(data + count));

            data += count;
            size -= count;
        }
        else
        {
            sha256_ctx->data[index++] = *data++;
            
            size--;
            
            if (index == 64)
            {
                stm32wb_boot_sha256_compress(&sha256_ctx->hash[0], (const uint32_t*)&sha256_ctx->data[0], (const uint32_t*)&sha256_ctx->data[64]);
                
                index = 0;
            }
        }
    }
    
    sha256_ctx->index = index;
}

static void __attribute__((noinline)) stm32wb_boot_sha256_final(stm32wb_boot_sha256_context_t *sha256_ctx, uint32_t *hash)
{
    sha256_ctx->data[sha256_ctx->index++] = 0x80;

    if (sha256_ctx->index > (64-8))
    {
        stm32wb_boot_memset(&sha256_ctx->data[sha256_ctx->index], 0x00, 64 - sha256_ctx->index);
        
        stm32wb_boot_sha256_compress(&sha256_ctx->hash[0], (const uint32_t*)&sha256_ctx->data[0], (const uint32_t*)&sha256_ctx->data[64]);

        sha256_ctx->index = 0;
    }

    if (sha256_ctx->index != (64-8+3))
    {
        stm32wb_boot_memset(&sha256_ctx->data[sha256_ctx->index], 0x00, (64-8+3) - sha256_ctx->index);
    }

    sha256_ctx->data[59] = sha256_ctx->length >> (32-3);
    sha256_ctx->data[60] = sha256_ctx->length >> (24-3);
    sha256_ctx->data[61] = sha256_ctx->length >> (16-3);
    sha256_ctx->data[62] = sha256_ctx->length >>  (8-3);
    sha256_ctx->data[63] = sha256_ctx->length <<    (3);

    stm32wb_boot_sha256_compress(&sha256_ctx->hash[0], (const uint32_t*)&sha256_ctx->data[0], (const uint32_t*)&sha256_ctx->data[64]);

    hash[0] = SWAP(sha256_ctx->hash[0]);
    hash[1] = SWAP(sha256_ctx->hash[1]);
    hash[2] = SWAP(sha256_ctx->hash[2]);
    hash[3] = SWAP(sha256_ctx->hash[3]);
    hash[4] = SWAP(sha256_ctx->hash[4]);
    hash[5] = SWAP(sha256_ctx->hash[5]);
    hash[6] = SWAP(sha256_ctx->hash[6]);
    hash[7] = SWAP(sha256_ctx->hash[7]);    
}

/************************************************************************************************************************************/

#define STM32WB_BOOT_ECC256_NUM_BITS   256
#define STM32WB_BOOT_ECC256_NUM_BYTES  32
#define STM32WB_BOOT_ECC256_NUM_WORDS  8

static const uint32_t stm32wb_boot_ecc256_in_coeff_a_sign = 1;

static const uint32_t stm32wb_boot_ecc256_in_coeff_a[] = {
    0x00000003, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

static const uint32_t stm32wb_boot_ecc256_in_mod_gf[] = {
    0xffffffff, 0xffffffff, 0xffffffff, 0x00000000,
    0x00000000, 0x00000000, 0x00000001, 0xffffffff,
};

static const uint32_t stm32wb_boot_ecc256_in_initial_point_x[] = {
    0xd898c296, 0xf4a13945, 0x2deb33a0, 0x77037d81,
    0x63a440f2, 0xf8bce6e5, 0xe12c4247, 0x6b17d1f2,
};

static const uint32_t stm32wb_boot_ecc256_in_initial_point_y[] = {
    0x37bf51f5, 0xcbb64068, 0x6b315ece, 0x2bce3357,
    0x7c0f9e16, 0x8ee7eb4a, 0xfe1a7f9b, 0x4fe342e2,
};

static const uint32_t stm32wb_boot_ecc256_in_order_n[] = {
    0xfc632551, 0xf3b9cac2, 0xa7179e84, 0xbce6faad,
    0xffffffff, 0xffffffff, 0x00000000, 0xffffffff,
};

static void __attribute__((noinline)) stm32wb_boot_ecc256_param(uint32_t offset, const uint32_t *param, uint32_t count)
{
    uint32_t index;

    for (index = 0; index < count; index++)
    {
        PKA->RAM[offset + index] = param[index];
    }
    PKA->RAM[offset + count] = 0;
}

static bool __attribute__((noinline)) stm32wb_boot_ecc256_verify(const stm32wb_boot_ecc256_key_t *key, const uint32_t *signature, const uint32_t *digest)
{
    uint32_t index;
    bool success;

    RCC->AHB3ENR |= RCC_AHB3ENR_PKAEN;
    RCC->AHB3ENR;

    PKA->CR = PKA_CR_EN;

    PKA->CLRFR = ~0ul;

    PKA->RAM[PKA_ECDSA_VERIF_IN_ORDER_NB_BITS] = STM32WB_BOOT_ECC256_NUM_BITS;
    PKA->RAM[PKA_ECDSA_VERIF_IN_MOD_NB_BITS] = STM32WB_BOOT_ECC256_NUM_BITS;
    PKA->RAM[PKA_ECDSA_VERIF_IN_A_COEFF_SIGN] = stm32wb_boot_ecc256_in_coeff_a_sign;

    stm32wb_boot_ecc256_param(PKA_ECDSA_VERIF_IN_A_COEFF, stm32wb_boot_ecc256_in_coeff_a, STM32WB_BOOT_ECC256_NUM_WORDS);
    stm32wb_boot_ecc256_param(PKA_ECDSA_VERIF_IN_MOD_GF, stm32wb_boot_ecc256_in_mod_gf, STM32WB_BOOT_ECC256_NUM_WORDS);
    stm32wb_boot_ecc256_param(PKA_ECDSA_VERIF_IN_INITIAL_POINT_X, stm32wb_boot_ecc256_in_initial_point_x, STM32WB_BOOT_ECC256_NUM_WORDS);
    stm32wb_boot_ecc256_param(PKA_ECDSA_VERIF_IN_INITIAL_POINT_Y, stm32wb_boot_ecc256_in_initial_point_y, STM32WB_BOOT_ECC256_NUM_WORDS);
    stm32wb_boot_ecc256_param(PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_X, key->x, STM32WB_BOOT_ECC256_NUM_WORDS);
    stm32wb_boot_ecc256_param(PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_Y, key->y, STM32WB_BOOT_ECC256_NUM_WORDS);
    stm32wb_boot_ecc256_param(PKA_ECDSA_VERIF_IN_SIGNATURE_R, ((const uint32_t*)&signature[0]), STM32WB_BOOT_ECC256_NUM_WORDS);
    stm32wb_boot_ecc256_param(PKA_ECDSA_VERIF_IN_SIGNATURE_S, ((const uint32_t*)&signature[8]), STM32WB_BOOT_ECC256_NUM_WORDS);
    stm32wb_boot_ecc256_param(PKA_ECDSA_VERIF_IN_ORDER_N, stm32wb_boot_ecc256_in_order_n, STM32WB_BOOT_ECC256_NUM_WORDS);
    
    for (index = 0; index < STM32WB_BOOT_SHA256_NUM_WORDS; index++)
    {
        PKA->RAM[PKA_ECDSA_VERIF_IN_HASH_E + index] = SWAP(((const uint32_t*)&digest[0])[(STM32WB_BOOT_SHA256_NUM_WORDS-1) - index]);
    }
    PKA->RAM[PKA_ECDSA_VERIF_IN_HASH_E + STM32WB_BOOT_SHA256_NUM_WORDS] = 0;

    PKA->CR = (PKA->CR & ~PKA_CR_MODE) | (0x26 << PKA_CR_MODE_Pos);
    
    PKA->CR |= PKA_CR_START;

    while (!(PKA->SR & PKA_SR_PROCENDF)) { }

    PKA->CLRFR = ~0ul;

    success = (PKA->RAM[PKA_ECDSA_VERIF_OUT_RESULT] == 0);

    PKA->CR = 0;
    
    RCC->AHB3ENR &= ~RCC_AHB3ENR_PKAEN;

    return success;
}

/************************************************************************************************************************************/

static const stm32wb_application_info_t * __attribute__((noinline)) stm32wb_boot_application_info(void)
{
    const stm32wb_application_vectors_t *application_vectors;
    const stm32wb_application_info_t *application_info;
    uint32_t application_base, application_size, signature_size;
    stm32wb_boot_sha256_context_t sha256_ctx;
    uint32_t sha256_hash[STM32WB_BOOT_SHA256_HASH_SIZE / 4];
    uint32_t crc32;
    
    application_base = stm32wb_boot_info.application_base;

    application_vectors = (const stm32wb_application_vectors_t*)application_base;

    if (application_vectors->magic != ((((const stm32wb_boot_vectors_t*)stm32wb_boot_vectors)->magic & ~STM32WB_BOOT_TYPE_MASK) | STM32WB_APPLICATION_TYPE_APPLICATION))
    {
        return NULL;
    }

    if (((const stm32wb_application_vectors_t*)stm32wb_boot_vectors)->magic & STM32WB_BOOT_OPTION_SIGNATURE_MASK)
    {
        signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_ECC256;
    }
    else
    {
        signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_NONE;
    }
        
    if (application_vectors->base != application_base)
    {
        return NULL;
    }
    
    application_size = application_vectors->size;

    if (((application_size + signature_size) > (stm32wb_boot_info.application_limit - application_base)) || (application_size & 15))
    {
        return NULL;
    }
    
    application_info = (const stm32wb_application_info_t*)(application_base + application_size - sizeof(stm32wb_application_info_t));

    if (stm32wb_boot_memcmp(&stm32wb_boot_info.uuid[0], &application_info->uuid[0], sizeof(stm32wb_boot_info.uuid)))
    {
        return NULL;
    }

    if (signature_size)
    {
        stm32wb_boot_sha256_init(&sha256_ctx);
        stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)application_base, application_size);
        stm32wb_boot_sha256_final(&sha256_ctx, sha256_hash);

        if (!stm32wb_boot_ecc256_verify((const stm32wb_boot_ecc256_key_t *)stm32wb_boot_info.ecc256_key, (const uint32_t*)((const uint8_t*)application_base + application_size), sha256_hash))
        {
            return NULL;
        }
    }
    else
    {
        crc32 = stm32wb_boot_crc32((const uint8_t*)application_base, (application_size - sizeof(uint32_t)), 0xffffffff);

        if (application_info->crc32 != crc32)
        {
            return NULL;
        }
    }
    
    return application_info;
}

void __attribute__((noreturn, noinline)) stm32wb_boot_application(void)
{
    const stm32wb_application_info_t *application_info;
    uint32_t fwu_state;
    uint32_t boot_dfu;

    if (stm32wb_boot_info.pins.status != STM32WB_GPIO_PIN_NONE)
    {
        stm32wb_boot_gpio_pin_configure(stm32wb_boot_info.pins.status, (STM32WB_GPIO_PUPD_PULLUP | STM32WB_GPIO_ODATA_1 |STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));
    }

    if (stm32wb_boot_info.pins.boost != STM32WB_GPIO_PIN_NONE)
    {
        stm32wb_boot_gpio_pin_configure(stm32wb_boot_info.pins.boost, (STM32WB_GPIO_PUPD_PULLDOWN | STM32WB_GPIO_ODATA_0 |STM32WB_GPIO_OSPEED_HIGH | STM32WB_GPIO_OTYPE_PUSHPULL | STM32WB_GPIO_MODE_OUTPUT));
    }
    
    if (PWR->EXTSCR & PWR_EXTSCR_C1SBF)
    {
        stm32wb_boot_leave(stm32wb_boot_info.application_base);
    }
    
    /* The STM32 BOOTLOADER does not do a reset when jumping to the newly flashed
     * code. Hence CPU2 might be booted. If so, do a software reset.
     */
    if (PWR->CR4 & PWR_CR4_C2BOOT)
    {
        stm32wb_boot_reset();
    }

    RCC->CR &= ~RCC_CR_PLLON;

    RCC->PLLCFGR = ((1 << RCC_PLLCFGR_PLLR_Pos) | (64000000 / (16000000 / 2)) << RCC_PLLCFGR_PLLN_Pos) | (((1-1) << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLSRC_HSI);
    
    RCC->CR |= RCC_CR_PLLON;

    while (!(RCC->CR & RCC_CR_PLLRDY))
    {
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
            
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
    }
    
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;

    while(!(RCC->CRRCR & RCC_CRRCR_HSI48RDY))
    {
    }

    SCB->CCR = 0;
    SCB->SHCSR = 0;
    
    NVIC_SetPriority(SVCall_IRQn, ARMV7M_IRQ_PRIORITY_SVCALL);
    NVIC_SetPriority(PendSV_IRQn, ARMV7M_IRQ_PRIORITY_PENDSV);
    
    RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;
    
    PWR->CR1 |= PWR_CR1_DBP;
    
    while (!(PWR->CR1 & PWR_CR1_DBP))
    {
    }

    stm32wb_boot_memcpy((uint8_t*)__boot_data_start__, (const uint8_t*)__boot_data_flash__, (uint32_t)__boot_data_end__ - (uint32_t)__boot_data_start__);
    stm32wb_boot_memset((uint8_t*)__boot_bss_start__, 0, (uint32_t)__boot_bss_end__ - (uint32_t)__boot_bss_start__);

    if ((FLASH->OPTR & FLASH_OPTR_RDP) == 0xaa)
    {
        stm32wb_boot_sha256_context_t sha256_ctx;
        uint32_t sha256_hash[STM32WB_BOOT_SHA256_HASH_SIZE / 4];
      
        stm32wb_boot_sha256_init(&sha256_ctx);
        stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)STM32WB_BOOT_BASE, (STM32WB_BOOT_HASH - STM32WB_BOOT_BASE));
        stm32wb_boot_sha256_final(&sha256_ctx, sha256_hash);

        if (stm32wb_boot_memcmp((const uint8_t*)STM32WB_BOOT_HASH, sha256_hash, sizeof(sha256_hash)))
        {
            while (1)
            {
            }
        }

        /* Add PCROP/WPROT, BOOT/PH3 and switch to RDP Level 1 if PROTECTED.
         */
        if (((const stm32wb_application_vectors_t*)stm32wb_boot_vectors)->magic & STM32WB_BOOT_OPTION_PROTECTED)
        {
            FLASH->KEYR      = 0x45670123;
            FLASH->KEYR      = 0xcdef89ab;
            
            FLASH->OPTKEYR   = 0x08192a3b;
            FLASH->OPTKEYR   = 0x4c5d6e7f;

            FLASH->OPTR      = (FLASH->OPTR & ~(FLASH_OPTR_nBOOT0 | FLASH_OPTR_nSWBOOT0 | FLASH_OPTR_RDP)) | (FLASH_OPTR_nBOOT0 | 0xbb);
            FLASH->WRP1AR    = 0x00030000;            
            FLASH->PCROP1ASR = 0x00000001;
            FLASH->PCROP1AER = 0x80000007;

            __DSB();
            __ISB();
            
            FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);
            FLASH->CR = FLASH_CR_OPTSTRT;
            
            while (FLASH->SR & FLASH_SR_BSY)
            {
            }
            
            FLASH->CR |= FLASH_CR_OBL_LAUNCH;
            
            __DSB();
            __ISB();
            
            FLASH->CR |= (FLASH_CR_LOCK | FLASH_CR_OPTLOCK);
            
            stm32wb_boot_reset();
        }
    }
    
#if (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1) || (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1)
    fwu_state = stm32wb_boot_fwu_state();

#if (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1)
    if ((fwu_state == STM32WB_FWU_STATE_TRIAL) || (fwu_state == STM32WB_FWU_STATE_REJECTED))
    {
        stm32wb_boot_fwu_rollback(STM32WB_FWU_STATUS_ERR_RESET);
    }
#endif /* (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1) */
    
    application_info = stm32wb_boot_application_info();

    if (fwu_state == STM32WB_FWU_STATE_STAGED)
    {
        stm32wb_boot_fwu_install(application_info);

        application_info = stm32wb_boot_application_info();
    }
#else /* (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1) || (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1) */
    application_info = stm32wb_boot_application_info();
#endif /* (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1) || (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1) */
    
    if (!application_info)
    {
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
#if (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1)
        if (((const uint32_t*)(stm32wb_boot_info.application_base + STM32WB_BOOT_WIRELESS_PREFIX_OFFSET_MAGIC))[0] == STM32WB_BOOT_WIRELESS_PREFIX_MAGIC)
        {
            stm32wb_boot_fus_install(stm32wb_boot_info.application_base, ((const uint32_t*)(stm32wb_boot_info.application_base + STM32WB_BOOT_WIRELESS_PREFIX_OFFSET_SIZE))[0]);
        }
#endif /* (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
        
        stm32wb_boot_dfu(NULL);
    }

    if (stm32wb_boot_info.pins.dfu != STM32WB_GPIO_PIN_NONE)
    {
        stm32wb_boot_gpio_pin_configure(stm32wb_boot_info.pins.dfu, (STM32WB_GPIO_PUPD_PULLDOWN | STM32WB_GPIO_MODE_INPUT));
        
        boot_dfu = stm32wb_boot_gpio_pin_read(stm32wb_boot_info.pins.dfu);
        
        stm32wb_boot_gpio_pin_configure(stm32wb_boot_info.pins.dfu, (STM32WB_GPIO_MODE_ANALOG));
    
        if (boot_dfu)
        {
            stm32wb_boot_dfu(application_info);
        }
    }

    if (RCC->BDCR & RCC_BDCR_RTCEN)
    {
        if (RTC->BKP16R & STM32WB_RTC_BKP16R_DFU)
        {
            stm32wb_boot_dfu(application_info);
        }
    }

    RCC->APB1ENR1 &= ~RCC_APB1ENR1_RTCAPBEN;

    PWR->CR1 &= ~PWR_CR1_DBP;
    
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
            
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {
    }

    RCC->CR &= ~RCC_CR_PLLON;

    RCC->CRRCR &= ~RCC_CRRCR_HSI48ON;
    
    stm32wb_boot_leave(stm32wb_boot_info.application_base);
}

/************************************************************************************************************************************/

#if (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1)

#define STM32WB_BOOT_FWU_DICT_SIZE 8192

typedef struct _stm32wb_boot_fwu_context_t {
    bool                          encrypted;
    bool                          compressed;
    bool                          signature;
    bool                          restore;
    uint32_t                      in_current;
    uint32_t                      in_finish;
    uint32_t                      in_base;
    uint32_t                      in_limit;
    uint8_t                       in_size;
    uint8_t                       in_index;
    uint8_t                       in_shift;
    uint32_t                      in_bits;
    uint32_t                      in_data[64];
    uint32_t                      out_base;
    uint32_t                      out_limit;
    uint32_t                      swap_start;
    uint32_t                      swap_base;
    uint32_t                      swap_limit;
    const uint8_t                 *copy_current;
    const uint8_t                 *copy_finish;
    uint8_t                       *copy_base;
    uint8_t                       *dict_current;
    uint8_t                       *dict_base;
    uint8_t                       *dict_limit;
    stm32wb_boot_aes128_context_t aes128_ctx;
    uint32_t                      aes128_iv[4];
} stm32wb_boot_fwu_context_t;

static void __attribute__((noinline)) stm32wb_boot_fwu_data(stm32wb_boot_fwu_context_t *fwu_ctx, uint8_t *data, uint32_t count)
{
    uint32_t in_current = fwu_ctx->in_current;
    
    // address multiple of 16, count multiple of 16

    if (count > (fwu_ctx->in_limit - in_current))
    {
        count = fwu_ctx->in_limit - in_current;
    }

    if (count)
    {
        fwu_ctx->in_current += count;

        stm32wb_boot_nvm_read(in_current, data, count, false);
    }
}

static void __attribute__((noinline)) stm32wb_boot_fwu_decrypt(stm32wb_boot_fwu_context_t *fwu_ctx, uint8_t *data, uint32_t count)
{
    uint32_t in_current = fwu_ctx->in_current;
    uint32_t block;

    // address multiple of 16, count multiple of 16

    if (count > (fwu_ctx->in_limit - in_current))
    {
        count = fwu_ctx->in_limit - in_current;
    }
    
    if (count)
    {
        fwu_ctx->in_current += count;

        stm32wb_boot_nvm_read(in_current, data, count, false);

        if (fwu_ctx->encrypted)
        {
            block = (in_current - fwu_ctx->in_base) / 16;
        
            fwu_ctx->aes128_iv[3] = SWAP(block);
        
            stm32wb_boot_aes128_ctr_encrypt(&fwu_ctx->aes128_ctx, &fwu_ctx->aes128_iv[0], (const uint32_t*)data, (uint32_t*)data, count);
        }
    }
}

static void __attribute__((noinline)) stm32wb_boot_fwu_init(stm32wb_boot_fwu_context_t *fwu_ctx, uint32_t in_finish, uint32_t in_base, uint32_t in_limit, uint32_t out_base, uint32_t out_limit, uint32_t swap_start, uint32_t swap_base, uint32_t swap_limit)
{
    fwu_ctx->in_finish = in_finish;
    fwu_ctx->in_base = in_base;
    fwu_ctx->in_limit = in_limit;

    fwu_ctx->out_base = out_base;
    fwu_ctx->out_limit = out_limit;
    
    fwu_ctx->swap_start = swap_start;
    fwu_ctx->swap_base  = swap_base;
    fwu_ctx->swap_limit = swap_limit;

    fwu_ctx->copy_base = &stm32wb_boot_io2_data[0];
    fwu_ctx->dict_base = &stm32wb_boot_io8_data[0];
    fwu_ctx->dict_limit = &stm32wb_boot_io8_data[STM32WB_BOOT_FWU_DICT_SIZE];
}

static void __attribute__((noinline)) stm32wb_boot_fwu_save(stm32wb_boot_fwu_context_t *fwu_ctx, uint32_t * p_in_current_return, uint8_t * p_in_size_return, uint8_t * p_in_index_return, uint8_t * p_in_shift_return)
{
    *p_in_current_return = fwu_ctx->in_current;
    *p_in_size_return = fwu_ctx->in_size;
    *p_in_index_return = fwu_ctx->in_index;
    *p_in_shift_return = fwu_ctx->in_shift;
}

static void __attribute__((noinline)) stm32wb_boot_fwu_restore(stm32wb_boot_fwu_context_t *fwu_ctx, uint32_t in_current, uint8_t in_size, uint8_t in_index, uint8_t in_shift, uint32_t out_current)
{
    uint32_t count;
    
    fwu_ctx->in_current = in_current;
    fwu_ctx->in_size = in_size;
    fwu_ctx->in_index = in_index;
    fwu_ctx->in_shift = in_shift;

    // starting condition is in_index == 63 and in_shift == 32.
    if (!((fwu_ctx->in_shift == 32) && (fwu_ctx->in_index == (fwu_ctx->in_size -1))))
    {
        fwu_ctx->in_current -= (fwu_ctx->in_size * 4);
    
        stm32wb_boot_fwu_decrypt(fwu_ctx, (uint8_t*)&fwu_ctx->in_data[0], (fwu_ctx->in_size * 4));

        if (fwu_ctx->in_shift != 32)
        {
            fwu_ctx->in_bits = fwu_ctx->in_data[fwu_ctx->in_index] << fwu_ctx->in_shift;
        }
    }
    
    fwu_ctx->copy_current = NULL;
    fwu_ctx->copy_finish = NULL;

    count = STM32WB_BOOT_FWU_DICT_SIZE;

    if (count > (out_current - fwu_ctx->out_base))
    {
        count = (out_current - fwu_ctx->out_base);
    }

    stm32wb_boot_memcpy(fwu_ctx->dict_base, (const uint8_t*)(out_current - count), count);

    fwu_ctx->dict_current = fwu_ctx->dict_base + count;

    if (fwu_ctx->dict_current == fwu_ctx->dict_limit)
    {
        fwu_ctx->dict_current = fwu_ctx->dict_base;
    }
}

static uint32_t __attribute__((noinline, optimize("O3"))) stm32wb_boot_fwu_bits(stm32wb_boot_fwu_context_t *fwu_ctx, uint32_t count)
{
    uint32_t bits, size;

    if (fwu_ctx->in_shift == 32)
    {
        fwu_ctx->in_shift = 0;
        fwu_ctx->in_index++;

        if (fwu_ctx->in_index == fwu_ctx->in_size)
        {
            fwu_ctx->in_size = 64;
            fwu_ctx->in_index = 0;
            
            if (fwu_ctx->in_size > ((fwu_ctx->in_finish - fwu_ctx->in_current) / 4))
            {
                fwu_ctx->in_size = ((fwu_ctx->in_finish - fwu_ctx->in_current) / 4);
            }
            
            if (fwu_ctx->in_size)
            {
                stm32wb_boot_fwu_decrypt(fwu_ctx, (uint8_t*)&fwu_ctx->in_data[0], (fwu_ctx->in_size * 4));
            }
        }
        
        fwu_ctx->in_bits = fwu_ctx->in_data[fwu_ctx->in_index];

        bits = fwu_ctx->in_bits >> (32 - count);

        fwu_ctx->in_bits <<= count;
        fwu_ctx->in_shift += count;
    }
    else
    {
        size = 32 - fwu_ctx->in_shift;

        if (size > count)
        {
            size = count;
        }
        
        bits = fwu_ctx->in_bits >> (32 - size);

        fwu_ctx->in_bits <<= size;
        fwu_ctx->in_shift += size;

        if (size != count)
        {
            fwu_ctx->in_shift = 0;
            fwu_ctx->in_index++;
            
            if (fwu_ctx->in_index == fwu_ctx->in_size)
            {
                fwu_ctx->in_size = 64;
                fwu_ctx->in_index = 0;
                
                if (fwu_ctx->in_size > ((fwu_ctx->in_finish - fwu_ctx->in_current) / 4))
                {
                    fwu_ctx->in_size = ((fwu_ctx->in_finish - fwu_ctx->in_current) / 4);
                }
                
                if (fwu_ctx->in_size)
                {
                    stm32wb_boot_fwu_decrypt(fwu_ctx, (uint8_t*)&fwu_ctx->in_data[0], (fwu_ctx->in_size * 4));
                }
            }
            
            fwu_ctx->in_bits = fwu_ctx->in_data[fwu_ctx->in_index];
            
            bits = (bits << (count - size)) | (fwu_ctx->in_bits >> (32 - (count - size)));
            
            fwu_ctx->in_bits <<= (count - size);
            fwu_ctx->in_shift += (count - size);
        }
    }
    
    return bits;
}

static void __attribute__((noinline, optimize("O3"))) stm32wb_boot_fwu_uncompress(stm32wb_boot_fwu_context_t *fwu_ctx, uint8_t *data, uint32_t count, uint32_t swap_size)
{
    uint32_t code, literal, length, distance, offset, align;
    const uint8_t *copy_current;
    uint8_t *dict_current;
    uint8_t *data_e;

    if (!fwu_ctx->compressed)
    {
        stm32wb_boot_fwu_decrypt(fwu_ctx, data, count);

        return;
    }
    
    data_e = data + count;

    copy_current = fwu_ctx->copy_current;
    dict_current = fwu_ctx->dict_current;

    while (data < data_e)
    {
        if (copy_current != fwu_ctx->copy_finish)
        {
            literal = *copy_current++;

            if (copy_current == fwu_ctx->dict_limit)
            {
                copy_current = fwu_ctx->dict_base;
            }

            *data++ = literal;

            *dict_current++ = literal;

            if (dict_current == fwu_ctx->dict_limit)
            {
                dict_current = fwu_ctx->dict_base;
            }
            
            continue;
        }
        
        code = stm32wb_boot_fwu_bits(fwu_ctx, 3);

        if (code < 4)
        {
            literal = (code << 6) | stm32wb_boot_fwu_bits(fwu_ctx, 6);

            *data++ = literal;

            *dict_current++ = literal;

            if (dict_current == fwu_ctx->dict_limit)
            {
                dict_current = fwu_ctx->dict_base;
            }
        }
        else
        {
            if (code < 7)
            {
                length = (code - 4) + 2;
            }
            else
            {
                code = stm32wb_boot_fwu_bits(fwu_ctx, 2);

                if (code < 2)
                {
                    length = (code + 5);
                }
                else
                {
                    if (code == 2)
                    {
                        length = stm32wb_boot_fwu_bits(fwu_ctx, 3) + 7;
                    }
                    else
                    {
                        code = stm32wb_boot_fwu_bits(fwu_ctx, 7);

                        if (code < 112)
                        {
                            length = code + 15;
                        }
                        else
                        {
                            length = (((code - 112) << 7) + stm32wb_boot_fwu_bits(fwu_ctx, 7)) + 127;
                        }
                    }
                }
            }
            
            code = stm32wb_boot_fwu_bits(fwu_ctx, 9);
            
            if (code < 256)
            {
                distance = code + 1;
            }
            else
            {
                if (length == 2)
                {
                    distance = (((code - 256) << 3) + stm32wb_boot_fwu_bits(fwu_ctx, 3)) + 257;
                }
                else
                {
                    if (code < 384)
                    {
                        distance = (((code - 256) << 4) + stm32wb_boot_fwu_bits(fwu_ctx, 4)) + 257;
                    }
                    else
                    {
                        code = ((code - 384) << 6) + stm32wb_boot_fwu_bits(fwu_ctx, 6);
                        
                        if (code < 6144)
                        {
                            distance = code + 2305;
                        }
                        else
                        {
                            offset = (((code - 6144) << 11) + stm32wb_boot_fwu_bits(fwu_ctx, 11));
                            
                            if ((offset + length) > (fwu_ctx->out_limit - fwu_ctx->out_base))
                            {
                                // Avoid overfetching, but also avoid in/out size mismatches ... So fetch random stuff
                                copy_current = (const uint8_t*)fwu_ctx->out_base;
                            }
                            else
                            {
                                if (offset >= swap_size)
                                {
                                    copy_current = (const uint8_t*)(fwu_ctx->out_base + offset);
                                }
                                else
                                {
                                    if (offset < (fwu_ctx->swap_limit - fwu_ctx->swap_start))
                                    {
                                        copy_current = (const uint8_t*)(fwu_ctx->swap_start + offset);
                                    }
                                    else
                                    {
                                        copy_current = (const uint8_t*)(fwu_ctx->swap_base + (offset - (fwu_ctx->swap_limit - fwu_ctx->swap_start)));
                                    }
                                }

                                align = ((uint32_t)copy_current & 15);
                                
                                stm32wb_boot_nvm_read((uint32_t)copy_current - align, fwu_ctx->copy_base, ((align + length + 15) & ~15), true);

                                copy_current = fwu_ctx->copy_base + align;
                            }

                            fwu_ctx->copy_finish = copy_current + length;

                            continue;
                        }
                    }
                }
            }

            copy_current = dict_current - distance;

            if (copy_current < fwu_ctx->dict_base)
            {
                copy_current += STM32WB_BOOT_FWU_DICT_SIZE;
            }
            
            fwu_ctx->copy_finish = copy_current + length;

            if (fwu_ctx->copy_finish >= fwu_ctx->dict_limit)
            {
                fwu_ctx->copy_finish -= STM32WB_BOOT_FWU_DICT_SIZE;
            }
        }
    }

    fwu_ctx->copy_current = copy_current;
    fwu_ctx->dict_current = dict_current;
}

static uint32_t __attribute__((noinline)) stm32wb_boot_fwu_application(const stm32wb_application_info_t *application_info)
{
    stm32wb_fwu_prefix_t candidate_prefix;
    const stm32wb_application_vectors_t *candidate_vectors;
    const stm32wb_application_info_t *candidate_info;
    stm32wb_boot_fwu_context_t fwu_ctx;
    stm32wb_boot_sha256_context_t sha256_ctx;
    uint32_t sha256_hash[STM32WB_BOOT_SHA256_HASH_SIZE / 4];
    uint32_t aes128_key[4];
    uint32_t program_data[STM32WB_BOOT_NVM_PROGRAM_SIZE /4];
    uint32_t candidate_base, candidate_size, candidate_length, signature_size, program_size, program_count, size, offset, slot, crc32;
    uint32_t application_base, application_limit, application_size;
    uint32_t image_base, image_limit;
    uint8_t image_size, image_index, image_shift;
    uint32_t swap_start, swap_base, swap_limit, swap_current, swap_size, nvm_erase_size, nvm_program_size;

#if (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1)
    if (stm32wb_boot_info.fwu_base < STM32WB_BOOT_SFLASH_LIMIT)
    {
        const stm32wb_boot_sflash_info_t *sflash_info;

        while ((((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_RANDOM))[0] &
                ((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_RANDOM))[1] &
                ((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_RANDOM))[2] &
                ((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_RANDOM))[3]) == 0xffffffff)
        {
            stm32wb_boot_random((uint8_t*)&aes128_key[0], sizeof(aes128_key));
            
            stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_RANDOM + 0, (const uint8_t*)&aes128_key[0] + 0, 8);
            stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_RANDOM + 8, (const uint8_t*)&aes128_key[0] + 8, 8);
        }

        stm32wb_boot_nvm_set_key((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_RANDOM));
        
        sflash_info = stm32wb_boot_sflash_init();

        nvm_erase_size = sflash_info->erase_size;
        nvm_program_size = sflash_info->page_size;
    }
    else
#endif /* (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1) */
    {
        nvm_erase_size = STM32WB_BOOT_FLASH_ERASE_SIZE;
        nvm_program_size = sizeof(program_data);
    }

    if (((const stm32wb_application_vectors_t*)stm32wb_boot_vectors)->magic & STM32WB_BOOT_OPTION_SIGNATURE_MASK)
    {
        signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_ECC256;
    }
    else
    {
        signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_NONE;
    }
            
    if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_SWAP))[0] == 0xffffffff)
    {
        stm32wb_boot_nvm_read(stm32wb_boot_info.fwu_base, (uint8_t*)&candidate_prefix, sizeof(candidate_prefix), false);
            
        if ((candidate_prefix.magic & ~STM32WB_FWU_TYPE_MASK) == ((((const stm32wb_boot_vectors_t*)stm32wb_boot_vectors)->magic & ~(STM32WB_BOOT_TAG_MASK | STM32WB_BOOT_TYPE_MASK)) | STM32WB_FWU_TAG_FWU))
        {
            candidate_base = candidate_prefix.base;
            candidate_size = candidate_prefix.size;
            candidate_length = candidate_prefix.length;

            if (candidate_base != stm32wb_boot_info.application_base)
            {
                return STM32WB_FWU_STATUS_ERR_TARGET;
            }

            if ((candidate_size < (sizeof(stm32wb_application_vectors_t) + sizeof(stm32wb_application_info_t))) || ((candidate_size + signature_size) > (stm32wb_boot_info.application_limit - stm32wb_boot_info.application_base)) || (candidate_size & 15))
            {
                return STM32WB_FWU_STATUS_ERR_FILE;
            }

            if (candidate_length & 15)
            {
                return STM32WB_FWU_STATUS_ERR_FILE;
            }
        }
        else
        {
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0)
            stm32wb_boot_nvm_read(stm32wb_boot_info.fwu_base, (uint8_t*)&program_data[0], sizeof(stm32wb_application_vectors_t), false);

            candidate_vectors = (const stm32wb_application_vectors_t*)(&program_data[0]);

            if (candidate_vectors->magic == ((((const stm32wb_boot_vectors_t*)stm32wb_boot_vectors)->magic & ~STM32WB_BOOT_TYPE_MASK) | STM32WB_APPLICATION_TYPE_APPLICATION))
            {
                candidate_base = candidate_vectors->base;
                candidate_size = candidate_vectors->size;
            
                if (candidate_base != stm32wb_boot_info.application_base)
                {
                    return STM32WB_FWU_STATUS_ERR_TARGET;
                }
                
                if ((candidate_size < (sizeof(stm32wb_application_vectors_t) + sizeof(stm32wb_application_info_t))) || ((candidate_size + signature_size) > (stm32wb_boot_info.application_limit - stm32wb_boot_info.application_base)) || (candidate_size & 15))
                {
                    return STM32WB_FWU_STATUS_ERR_FILE;
                }
                
                stm32wb_boot_memset(&candidate_prefix, 0, sizeof(candidate_prefix));
                
                candidate_prefix.size = candidate_size;
                candidate_prefix.length = candidate_size + signature_size;
            }
            else
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0) */
            {
                return STM32WB_FWU_STATUS_ERR_FILE;
            }
        }
        
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0)
        if (!candidate_prefix.magic)
        {
            image_base = stm32wb_boot_info.fwu_base;
        }
        else
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0) */
        {
            image_base = stm32wb_boot_info.fwu_base + sizeof(stm32wb_fwu_prefix_t);
        }

        image_limit = image_base + candidate_prefix.length; 

        image_size = 64;
        image_index = 63;
        image_shift = 32;

        fwu_ctx.encrypted = !!(candidate_prefix.magic & STM32WB_FWU_OPTION_SIGNATURE_MASK);
        fwu_ctx.compressed = !!(candidate_prefix.magic & STM32WB_FWU_TYPE_COMPRESSED);
        fwu_ctx.signature = !!(((const stm32wb_application_vectors_t*)stm32wb_boot_vectors)->magic & STM32WB_BOOT_OPTION_SIGNATURE_MASK);
        fwu_ctx.restore = true;
        
        if (fwu_ctx.encrypted)
        {
            stm32wb_boot_aes128_get_key(&candidate_prefix.nonce[0], &aes128_key[0]);

            stm32wb_boot_aes128_set_key(&fwu_ctx.aes128_ctx, &aes128_key[0]);

            fwu_ctx.aes128_iv[0] = 0;
            fwu_ctx.aes128_iv[1] = 0;
            fwu_ctx.aes128_iv[2] = 0;
            fwu_ctx.aes128_iv[3] = 0;
        }

        if (fwu_ctx.signature)
        {
            stm32wb_boot_sha256_init(&sha256_ctx);
        }
        else
        {
            crc32 = 0xffffffff;
        }
        
        stm32wb_boot_fwu_init(&fwu_ctx, (image_limit - (sizeof(stm32wb_application_info_t) + signature_size)), image_base, image_limit, stm32wb_boot_info.application_base, stm32wb_boot_info.application_limit, 0, 0, 0);
        stm32wb_boot_fwu_restore(&fwu_ctx, image_base, image_size, image_index, image_shift, stm32wb_boot_info.application_base);

        for (offset = 0, application_size = candidate_size - sizeof(stm32wb_application_info_t); application_size; application_size -= program_size, offset += program_size)
        {
            program_size = sizeof(program_data);

            if (program_size > application_size)
            {
                program_size = application_size;
            }

            stm32wb_boot_fwu_uncompress(&fwu_ctx, (uint8_t*)&program_data[0], program_size, 0);

            if (fwu_ctx.signature)
            {
                stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)&program_data[0], program_size);
            }
            else
            {
                crc32 = stm32wb_boot_crc32((const uint8_t*)&program_data[0], program_size, crc32);
            }
            
            if (offset == 0)
            {
                candidate_vectors = (const stm32wb_application_vectors_t*)&program_data[0];

                if ((candidate_vectors->magic != ((((const stm32wb_boot_vectors_t*)stm32wb_boot_vectors)->magic & ~STM32WB_BOOT_TYPE_BOOT) | STM32WB_APPLICATION_TYPE_APPLICATION)) ||
                    (candidate_vectors->base != candidate_prefix.base) ||
                    (candidate_vectors->size != candidate_prefix.size))
                {
                    return STM32WB_FWU_STATUS_ERR_FILE;
                }
            }
        }

        stm32wb_boot_fwu_data(&fwu_ctx, (uint8_t*)&program_data[0], (sizeof(stm32wb_application_info_t) + signature_size));

        if (candidate_prefix.length != (fwu_ctx.in_current - fwu_ctx.in_base))
        {
            return STM32WB_FWU_STATUS_ERR_FILE;
        }
        
        candidate_info = (const stm32wb_application_info_t*)&program_data[0];
        
        if (stm32wb_boot_memcmp(&stm32wb_boot_info.uuid[0], &candidate_info->uuid[0], sizeof(stm32wb_boot_info.uuid)))
        {
            return STM32WB_FWU_STATUS_ERR_TARGET;
        }

        if (application_info)
        {
            if (application_info->sequence > candidate_info->sequence)
            {
                return STM32WB_FWU_STATUS_ERR_FILE;
            }
        }

        if (fwu_ctx.signature)
        {
            stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)&program_data[0], sizeof(stm32wb_application_info_t));
            stm32wb_boot_sha256_final(&sha256_ctx, sha256_hash);

            if (!stm32wb_boot_ecc256_verify((const stm32wb_boot_ecc256_key_t *)stm32wb_boot_info.ecc256_key, (const uint32_t*)((const uint8_t*)candidate_info + sizeof(stm32wb_application_info_t)), sha256_hash))
            {
                return STM32WB_FWU_STATUS_ERR_FILE;
            }
        }
        else
        {
            crc32 = stm32wb_boot_crc32((const uint8_t*)&program_data[0], (sizeof(stm32wb_application_info_t) - sizeof(uint32_t)), crc32);

            if (candidate_info->crc32 != crc32)
            {
                return STM32WB_FWU_STATUS_ERR_FILE;
            }
        }
        
        for (offset = 0; offset < sizeof(candidate_prefix); offset += 8)
        {
            stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_PREFIX + offset, (const uint8_t*)&candidate_prefix + offset, 8);
        }

        if (stm32wb_boot_memcmp((const uint8_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_PREFIX), (const uint8_t*)&candidate_prefix, sizeof(candidate_prefix)))
        {
            return STM32WB_FWU_STATUS_ERR_PROGRAM;
        }
    }
    
    /**********************************************************************************************************************************/

    stm32wb_boot_memcpy(&candidate_prefix, (const uint8_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_PREFIX), sizeof(candidate_prefix));

#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0)
    if (!candidate_prefix.magic)
    {
        image_base = stm32wb_boot_info.fwu_base;
    }
    else
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0) */
    {
        image_base = stm32wb_boot_info.fwu_base + sizeof(stm32wb_fwu_prefix_t);
    }

    image_limit = image_base + candidate_prefix.length;
    
    image_size = 64;
    image_index = 63;
    image_shift = 32;

    fwu_ctx.encrypted = !!(candidate_prefix.magic & STM32WB_FWU_OPTION_SIGNATURE_MASK);
    fwu_ctx.compressed = !!(candidate_prefix.magic & STM32WB_FWU_TYPE_COMPRESSED);
    fwu_ctx.signature = !!(((const stm32wb_application_vectors_t*)stm32wb_boot_vectors)->magic & STM32WB_BOOT_OPTION_SIGNATURE_MASK);
    fwu_ctx.restore = true;

    if (fwu_ctx.encrypted)
    {
        stm32wb_boot_aes128_get_key(&candidate_prefix.nonce[0], &aes128_key[0]);
        
        stm32wb_boot_aes128_set_key(&fwu_ctx.aes128_ctx, &aes128_key[0]);
        
        fwu_ctx.aes128_iv[0] = 0;
        fwu_ctx.aes128_iv[1] = 0;
        fwu_ctx.aes128_iv[2] = 0;
        fwu_ctx.aes128_iv[3] = 0;
    }
    
    swap_start = (image_limit + (nvm_erase_size-1)) & ~(nvm_erase_size-1);
    swap_base  = stm32wb_boot_info.fwu_base;
    swap_limit = stm32wb_boot_info.fwu_limit;

    swap_current = swap_start;
    swap_size = 0;

    stm32wb_boot_fwu_init(&fwu_ctx, (image_limit - (sizeof(stm32wb_application_info_t) + signature_size)), image_base, image_limit, stm32wb_boot_info.application_base, stm32wb_boot_info.application_limit, swap_start, swap_base, swap_limit);

    application_base = stm32wb_boot_info.application_base;
    application_limit = stm32wb_boot_info.application_base + candidate_prefix.size - sizeof(stm32wb_application_info_t);
    application_size = candidate_prefix.size + signature_size;

    for (slot = 0; application_size; application_size -= size, application_base += size, slot += 8)
    {
        size = STM32WB_BOOT_FLASH_ERASE_SIZE;

        if (size > application_size)
        {
            size = application_size;
        }

        if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_SWAP + slot))[0] == 0xffffffff)
        {
            if (!(swap_current & (nvm_erase_size-1)))
            {
                if (!stm32wb_boot_nvm_erase(swap_current))
                {
                    return STM32WB_FWU_STATUS_ERR_ERASE;
                }
            }

            for (offset = 0; offset < size; offset += nvm_program_size)
            {
                if (!stm32wb_boot_nvm_program(swap_current + offset, ((const uint8_t*)application_base + offset), nvm_program_size, true))
                {
                    return STM32WB_FWU_STATUS_ERR_PROGRAM;
                }
            }
            
            program_data[0] = swap_current;
            program_data[1] = 0;
            
            if (!stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_SWAP + slot, (const uint8_t*)program_data, 8))
            {
                return STM32WB_FWU_STATUS_ERR_PROGRAM;
            }
        }
        
        swap_current += STM32WB_BOOT_FLASH_ERASE_SIZE;
        swap_size += STM32WB_BOOT_FLASH_ERASE_SIZE;
        
        if (swap_current == swap_limit)
        {
            swap_current = swap_base;
        }
        
        if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_COPY + slot))[0] == 0xffffffff)
        {
            if (!stm32wb_boot_flash_erase(application_base))
            {
                return STM32WB_FWU_STATUS_ERR_ERASE;
            }

            if (fwu_ctx.restore)
            {
                fwu_ctx.restore = false;

                stm32wb_boot_fwu_restore(&fwu_ctx, image_base, image_size, image_index, image_shift, application_base);
            }
                
            for (offset = 0; offset < size; offset += program_size)
            {
                program_size = sizeof(program_data);

                if (program_size > (size - offset))
                {
                    program_size = (size - offset);
                }

                if ((application_base + offset) < application_limit)
                {
                    program_count = program_size;
                
                    if (program_count > ((application_limit - application_base) - offset))
                    {
                        program_count = ((application_limit - application_base) - offset);
                    }

                    if (program_count)
                    {
                        stm32wb_boot_fwu_uncompress(&fwu_ctx, (uint8_t*)&program_data[0], program_count, swap_size);
                    }
                }
                else
                {
                    program_count = 0;
                }

                if (program_count != program_size)
                {
                    stm32wb_boot_fwu_data(&fwu_ctx, (uint8_t*)&program_data[0] + program_count, (program_size - program_count));
                }
                
                if (!stm32wb_boot_flash_program(application_base + offset, (const uint8_t*)program_data, program_size))
                {
                    return STM32WB_FWU_STATUS_ERR_PROGRAM;
                }
            }

            stm32wb_boot_fwu_save(&fwu_ctx, &image_base, &image_size, &image_index, &image_shift);
            
            program_data[0] = image_base;
            program_data[1] = (image_size << 0) | (image_index << 8) | (image_shift << 16);
            
            if (!stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_COPY + slot, (const uint8_t*)program_data, 8))
            {
                return STM32WB_FWU_STATUS_ERR_PROGRAM;
            }
        }
        else
        {
            program_data[0] = ((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_COPY + slot))[0];
            program_data[1] = ((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_COPY + slot))[1];

            image_base = program_data[0];
            image_size  = (program_data[1] >> 0) & 0xff;
            image_index = (program_data[1] >> 8) & 0xff;
            image_shift = (program_data[1] >> 16) & 0xff;
        }
    }
    
    return STM32WB_FWU_STATUS_NO_ERROR;
}

static void stm32wb_boot_fwu_rollback(uint32_t status)
{
    uint32_t application_base, application_limit, swap_current, slot;
    uint32_t program_data[2];

#if (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1)
    if (stm32wb_boot_info.fwu_base < STM32WB_BOOT_SFLASH_LIMIT)
    {
        stm32wb_boot_nvm_set_key((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_RANDOM));

        stm32wb_boot_sflash_init();
    }
#endif /* (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1) */
    
    application_base = stm32wb_boot_info.application_base;
    application_limit = stm32wb_boot_info.application_limit;

    for (slot = 0; application_base < application_limit; application_base += STM32WB_FLASH_PAGE_SIZE, slot += 8)
    {
        if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_SWAP + slot))[0] == 0xffffffff)
        {
            break;
        }

        swap_current = ((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_SWAP + slot))[0];
        
        stm32wb_boot_flash_erase(application_base);

        stm32wb_boot_nvm_read(swap_current, (uint8_t*)&stm32wb_boot_io8_data[0], STM32WB_BOOT_FLASH_ERASE_SIZE, true);

        stm32wb_boot_flash_program(application_base, (const uint8_t*)&stm32wb_boot_io8_data[0], STM32WB_BOOT_FLASH_ERASE_SIZE);
    }
        
    program_data[0] = status;
    program_data[1] = 0x00000000;

    stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_FAILED, (const uint8_t*)&program_data[0], 8);
}

#endif /* (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1) */

#if (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1) || (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1)

static uint32_t stm32wb_boot_fwu_state(void)
{
    if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_UPDATED))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_UPDATED;
    }

    if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_FAILED))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_FAILED;
    }

    if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_REJECTED))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_REJECTED;
    }

    if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_TRIAL))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_TRIAL;
    }

    if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_STAGED))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_STAGED;
    }

    if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_CANDIDATE))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_CANDIDATE;
    }

    if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_WRITING))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_WRITING;
    }

    if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_READY))[0] != 0xffffffff)
    {
        return STM32WB_FWU_STATE_READY;
    }

    return STM32WB_FWU_STATE_FAILED;
}

static uint32_t stm32wb_boot_fwu_install(const stm32wb_application_info_t *application_info)
{
    uint32_t status, offset, data[2];

    status = STM32WB_FWU_STATUS_ERR_INTERNAL;

#if (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1)
    if (((const uint8_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_WRITING))[0] == STM32WB_FWU_COMPONENT_APPLICATION)
    {
        status = stm32wb_boot_fwu_application(application_info);

        data[0] = status;
        data[1] = 0x00000000;
        
        if (status == STM32WB_FWU_STATUS_NO_ERROR)
        {
            if (((const uint8_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_STAGED))[0] == STM32WB_FWU_MODE_TRIAL)
            {
                offset = STM32WB_FWU_STATUS_OFFSET_TRIAL;
            }
            else
            {
                offset = STM32WB_FWU_STATUS_OFFSET_UPDATED;
            }
            
            stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + offset, (const uint8_t*)(&data[0]), 8);
        }
        else
        {
            stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_REJECTED, (const uint8_t*)(&data[0]), 8);

            stm32wb_boot_fwu_rollback(status);
        }
    }
#endif /* (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1) */
    
#if (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1)
    if (((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_WRITING))[0] == STM32WB_FWU_COMPONENT_WIRELESS)
    {
        status = stm32wb_boot_fus_install(stm32wb_boot_info.fwu_base, ((const uint32_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_CANDIDATE))[0]);

        data[0] = status;
        data[1] = 0x00000000;
        
        if (status == STM32WB_FWU_STATUS_NO_ERROR)
        {
            offset = STM32WB_FWU_STATUS_OFFSET_UPDATED;
        }
        else
        {
            offset = STM32WB_FWU_STATUS_OFFSET_FAILED;
        }
        
        stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + offset, (const uint8_t*)(&data[0]), 8);

        stm32wb_boot_reset();
    }
#endif /* (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1) */

    return status;
}

#endif /* (STM32WB_BOOT_FWU_CONFIG_APPLICATION == 1) || (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1) */

/************************************************************************************************************************************/

#if (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1) || (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1)

#define stm32wb_ipcc_device            stm32wb_boot_ipcc_device

#define stm32wb_ipcc_sys_enable        stm32wb_boot_ipcc_sys_enable
#define stm32wb_ipcc_sys_disable       stm32wb_boot_ipcc_sys_disable
#define stm32wb_ipcc_sys_state         stm32wb_boot_ipcc_sys_state
#define stm32wb_ipcc_sys_info          stm32wb_boot_ipcc_sys_info
#define stm32wb_ipcc_sys_command       stm32wb_boot_ipcc_sys_command

#define stm32wb_ipcc_fus_state         stm32wb_boot_ipcc_fus_state
#define stm32wb_ipcc_fus_command       stm32wb_boot_ipcc_fus_command

#define MB_QueueInit                   stm32wb_boot_ipcc_queue_init
#define MB_QueueIsEmpty                stm32wb_boot_ipcc_queue_is_empty
#define MB_QueueInsert                 stm32wb_boot_ipcc_queue_insert
#define MB_QueueRemove                 stm32wb_boot_ipcc_queue_remove
#define MB_QueueCopy                   stm32wb_boot_ipcc_queue_copy

#include "stm32wb_ipcc.c"

/************************************************************************************************************************************/

static bool stm32wb_boot_fus_image(uint32_t image_base, uint32_t image_size, stm32wb_ipcc_fus_image_t * p_fus_image_return)
{
    const stm32wb_ipcc_fus_signature_t *signature;
    uint32_t magic, count;

    image_size = (image_size + 15) & ~15;
    
    count = 1024;
    
    if (count > image_size)
    {
        count = image_size;
    }

    stm32wb_boot_nvm_read((image_base + image_size - count), (uint8_t*)&stm32wb_boot_io8_data[0], count, false);
    
    for (signature = (const stm32wb_ipcc_fus_signature_t*)((uint8_t*)&stm32wb_boot_io8_data[0] + count - sizeof(stm32wb_ipcc_fus_signature_t));
         ((uint8_t*)&stm32wb_boot_io8_data[0] <= (uint8_t*)signature);
         signature = (const stm32wb_ipcc_fus_signature_t*)((const uint8_t*)signature - 4)) 
    {
        if (signature->Magic == STM32WB_IPCC_FUS_MAGIC_STM_SIGNATURE)
        {
            magic = ((const stm32wb_ipcc_fus_image_t*)((const uint8_t*)signature - (signature->MemorySize & 0x000000ff) - sizeof(stm32wb_ipcc_fus_image_t)))->Magic;
            
            if ((magic == STM32WB_IPCC_FUS_MAGIC_WIRELESS_IMAGE) || (magic == STM32WB_IPCC_FUS_MAGIC_FUS_IMAGE))
            {
                if (p_fus_image_return)
                {
                    stm32wb_boot_memcpy(p_fus_image_return, (const stm32wb_ipcc_fus_image_t*)((const uint8_t*)signature - (signature->MemorySize & 0x000000ff) - sizeof(stm32wb_ipcc_fus_image_t)), sizeof(stm32wb_ipcc_fus_image_t));
                }

                return true;
            }
        }
    }

    return false;
}

static bool stm32wb_boot_fus_state(stm32wb_ipcc_fus_state_t *p_state_return)
{
    stm32wb_ipcc_sys_command_t command;
    
    command.opcode = STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE;
    command.cparam = NULL;
    command.clen = 0;
    command.rparam = (void*)p_state_return;
    command.rsize = sizeof(stm32wb_ipcc_fus_state_t);

    stm32wb_ipcc_sys_command(&command);

    return (command.status == STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS);
}

static bool stm32wb_boot_fus_command(uint32_t opcode)
{
    stm32wb_ipcc_sys_command_t command;
    
    command.opcode = opcode;
    command.cparam = NULL;
    command.clen = 0;
    command.rparam = NULL;
    command.rsize = 0;

    stm32wb_ipcc_sys_command(&command);

    return (command.status == STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS);
}

static uint32_t stm32wb_boot_fus_upgrade(uint32_t image_base, uint32_t image_size, const stm32wb_ipcc_fus_image_t *fus_image)
{
    stm32wb_ipcc_fus_state_t fus_state;
    stm32wb_ipcc_sys_info_t sys_info;
    uint32_t sys_state, fus_address, fus_base, fus_limit, image_address, program_size;
    uint32_t program_data[STM32WB_BOOT_NVM_PROGRAM_SIZE / 4];
    
    stm32wb_boot_ipcc_sys_enable();

    if (!stm32wb_boot_fus_state(&fus_state))
    {
        return STM32WB_FWU_STATUS_ERR_INTERNAL;
    }
    
    if (!stm32wb_boot_ipcc_sys_info(&sys_info))
    {
        return STM32WB_FWU_STATUS_ERR_INTERNAL;
    }

    sys_state = STM32WB_IPCC_SYS_STATE_FUS;
    
    do
    {
        if (fus_state.State == STM32WB_IPCC_FUS_STATE_ERROR)
        {
            if (fus_state.ErrorCode == STM32WB_IPCC_FUS_ERROR_CODE_NOT_RUNNING)
            {
                sys_state = STM32WB_IPCC_SYS_STATE_WIRELESS;
                
                break;
            }

            return STM32WB_FWU_STATUS_ERR_INTERNAL;
        }

        if (fus_state.State != STM32WB_IPCC_FUS_STATE_IDLE)
        {
            stm32wb_boot_udelay(100000);
            
            if (!stm32wb_boot_fus_state(&fus_state))
            {
                return STM32WB_FWU_STATUS_ERR_INTERNAL;
            }
        }
    }
    while (fus_state.State != STM32WB_IPCC_FUS_STATE_IDLE);
    
    if (((volatile uint32_t*)(image_base + STM32WB_BOOT_WIRELESS_PREFIX_OFFSET_STATUS))[0] == 0xffffffff)
    {
        // fus_limit = 0x08000000 + (FLASH->SFR & FLASH_SFR_SFSA) * STM32WB_FLASH_PAGE_SIZE;
        fus_limit = 0x080f4000;
        
        if (fus_image->Magic == STM32WB_IPCC_FUS_MAGIC_FUS_IMAGE)
        {
            if ((fus_image->Version & 0xffffff00) < (sys_info.FusVersion & 0xffffff00))
            {
                return STM32WB_FWU_STATUS_ERR_FILE;
            }

            fus_base = 0x080ec000; // max 32k
        }
        else
        {
            if (fus_image->Info1 == 0x5afeb007)
            {
                if ((fus_image->Version & 0xffffff00) < (sys_info.SafeBootVersion & 0xffffff00))
                {
                    return STM32WB_FWU_STATUS_ERR_FILE;
                }

                fus_base = 0x080f0000; // max 16k
            }
            else
            {
                fus_base = fus_limit - (fus_image->MemorySize & 0x000000ff) * STM32WB_FLASH_PAGE_SIZE;

                if (fus_base < stm32wb_boot_info.wireless_base)
                {
                    return STM32WB_FWU_STATUS_ERR_FILE;
                }
            }
        }

        if (sys_state == STM32WB_IPCC_SYS_STATE_WIRELESS)
        {
            /* Reboot into FUS.
             */
            
            if (!stm32wb_boot_fus_state(&fus_state))
            {
                return STM32WB_FWU_STATUS_ERR_INTERNAL;
            }
            
            while (1)
            {
            }
        }
        
        if (sys_info.WirelessStackMemorySize)
        {
            if (!stm32wb_boot_fus_command(STM32WB_IPCC_SYS_OPCODE_FUS_FW_DELETE))
            {
                return STM32WB_FWU_STATUS_ERR_INTERNAL;
            }

            do
            {
                stm32wb_boot_udelay(100000);
                
                if (!stm32wb_ipcc_fus_state(&fus_state))
                {
                    return STM32WB_FWU_STATUS_ERR_INTERNAL;
                }
                
                if (fus_state.State == STM32WB_IPCC_FUS_STATE_ERROR)
                {
                    return STM32WB_FWU_STATUS_ERR_INTERNAL;
                }
            }
            while (fus_state.State != STM32WB_IPCC_FUS_STATE_IDLE);

            /* If we get here throu an idle-state, CM0+ for some reason did not reset.
             * Hence use CM4 to do so.
             */
            stm32wb_boot_reset();
        }

        /* FUS does something odd with erased pages, so that a simple blank detection will
         * not work. Just brute force erase the whole area instead of trying to be smart.
         */
        for (fus_address = fus_base; fus_address < fus_limit; fus_address += STM32WB_BOOT_FLASH_ERASE_SIZE)
        {
            if (!stm32wb_boot_flash_erase(fus_address))
            {
                return STM32WB_FWU_STATUS_ERR_ERASE;
            }
        }

        for (fus_address = fus_base, image_address = image_base; fus_address < fus_limit; fus_address += program_size, image_address += program_size)
        {
            program_size = sizeof(program_data);

            if (program_size > (fus_limit - fus_address))
            {
                program_size = (fus_limit - fus_address);
            }
            
            stm32wb_boot_nvm_read(image_address, (uint8_t*)&program_data[0], program_size, false);

            if (!stm32wb_boot_flash_program(fus_address, (const uint8_t *)&program_data[0], program_size))
            {
                return STM32WB_FWU_STATUS_ERR_PROGRAM;
            }
        }
        
        /* Ok, all the prepwork had been done. The image has been verified (sort of), FUS is booted, and the
         * previous wireless stack has been deleted.
         */
        program_data[0] = 0x00000000;
        program_data[1] = 0x00000000;

        if (!stm32wb_boot_flash_program(image_base + STM32WB_BOOT_WIRELESS_PREFIX_OFFSET_STATUS, (const uint8_t*)&program_data[0], 8))
        {
            return STM32WB_FWU_STATUS_ERR_PROGRAM;
        }

        if (!stm32wb_boot_fus_command(STM32WB_IPCC_SYS_OPCODE_FUS_FW_UPGRADE))
        {
            return STM32WB_FWU_STATUS_ERR_INTERNAL;
        }

        do
        {
            stm32wb_boot_udelay(100000);

            if (!stm32wb_ipcc_fus_state(&fus_state))
            {
                return STM32WB_FWU_STATUS_ERR_INTERNAL;
            }
            
            if (fus_state.State == STM32WB_IPCC_FUS_STATE_ERROR)
            {
                return STM32WB_FWU_STATUS_ERR_INTERNAL;
            }
        }
        while (fus_state.State != STM32WB_IPCC_FUS_STATE_IDLE);

        /* If we get here throu an idle-state, CM0+ for some reason did not reset.
         * Hence use CM4 to do so.
         */
        stm32wb_boot_reset();
    }

    if (sys_info.WirelessStackMemorySize)
    {
        if (sys_state != STM32WB_IPCC_SYS_STATE_WIRELESS)
        {
            if(!stm32wb_boot_fus_command(STM32WB_IPCC_SYS_OPCODE_FUS_START_WS))
            {
                return STM32WB_FWU_STATUS_ERR_INTERNAL;
            }
        }
    }

    return STM32WB_FWU_STATUS_NO_ERROR;
}

static uint32_t __attribute__((noinline)) stm32wb_boot_fus_install(uint32_t image_base, uint32_t image_size)
{
    stm32wb_ipcc_fus_image_t fus_image;
    uint32_t status;
    uint32_t program_data[2];
    
#if (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1)
    if (image_base < STM32WB_BOOT_SFLASH_LIMIT)
    {
        stm32wb_boot_sflash_init();
    }
#endif /* (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1) */

    if (!stm32wb_boot_fus_image(image_base, image_size, &fus_image))
    {
        status = STM32WB_FWU_STATUS_ERR_FILE;
    }
    else
    {
        status = stm32wb_boot_fus_upgrade(image_base, image_size, &fus_image);
    }

#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
    /* On a DFU STANDALONE config, nuke the MAGIC, so that stm32wb_boot_application() is not
     * retrying an installion (whether successful or not).
     */
    program_data[0] = 0x00000000;
    program_data[1] = 0x00000000;

    stm32wb_boot_flash_program(image_base + STM32WB_BOOT_WIRELESS_PREFIX_OFFSET_MAGIC, (const uint8_t*)&program_data[0], 8);
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */

    return status;
}

#endif /* (STM32WB_BOOT_FWU_CONFIG_WIRELESS == 1) || (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */

/************************************************************************************************************************************/

/************************************************************************************************************************************/

#define stm32wb_gpio_pin_configure     stm32wb_boot_gpio_pin_configure
#define stm32wb_gpio_pin_input         stm32wb_boot_gpio_pin_input
#define stm32wb_gpio_pin_output        stm32wb_boot_gpio_pin_output
#define stm32wb_gpio_pin_alternate     stm32wb_boot_gpio_pin_alternate
#define stm32wb_gpio_pin_write         stm32wb_boot_gpio_pin_write
#define stm32wb_gpio_pin_read          stm32wb_boot_gpio_pin_read

#define stm32wb_usbd_dcd_device        stm32wb_boot_usbd_dcd_device

#define stm32wb_usbd_dcd_address       stm32wb_boot_usbd_dcd_address
#define stm32wb_usbd_dcd_configure     stm32wb_boot_usbd_dcd_configure
#define stm32wb_usbd_dcd_connect       stm32wb_boot_usbd_dcd_connect
#define stm32wb_usbd_dcd_disable       stm32wb_boot_usbd_dcd_disable
#define stm32wb_usbd_dcd_disconnect    stm32wb_boot_usbd_dcd_disconnect
#define stm32wb_usbd_dcd_enable        stm32wb_boot_usbd_dcd_enable
#define stm32wb_usbd_dcd_event         stm32wb_boot_usbd_dcd_event
#define stm32wb_usbd_dcd_ep0_count     stm32wb_boot_usbd_dcd_ep0_count
#define stm32wb_usbd_dcd_ep0_receive   stm32wb_boot_usbd_dcd_ep0_receive
#define stm32wb_usbd_dcd_ep0_stall     stm32wb_boot_usbd_dcd_ep0_stall
#define stm32wb_usbd_dcd_ep0_transmit  stm32wb_boot_usbd_dcd_ep0_transmit
#define stm32wb_usbd_dcd_pma_read      stm32wb_boot_usbd_dcd_pma_read
#define stm32wb_usbd_dcd_pma_write     stm32wb_boot_usbd_dcd_pma_write
#define stm32wb_usbd_dcd_reset         stm32wb_boot_usbd_dcd_reset

#define stm32wb_usbd_dcd_usb_interrupt stm32wb_boot_usbd_dcd_usb_interrupt
#define stm32wb_usbd_dcd_crs_interrupt stm32wb_boot_usbd_dcd_crs_interrupt

#include "stm32wb_usbd_dcd.c"

#define stm32wb_usbd_control           stm32wb_boot_usbd_control

#define stm32wb_usbd_bos               stm32wb_boot_usbd_bos
#define stm32wb_usbd_configuration     stm32wb_boot_usbd_configuration
#define stm32wb_usbd_configure         stm32wb_boot_usbd_configure
#define stm32wb_usbd_connect           stm32wb_boot_usbd_connect
#define stm32wb_usbd_device            stm32wb_boot_usbd_device
#define stm32wb_usbd_disable           stm32wb_boot_usbd_disable
#define stm32wb_usbd_disconnect        stm32wb_boot_usbd_disconnect
#define stm32wb_usbd_enable            stm32wb_boot_usbd_enable
#define stm32wb_usbd_event_callback    stm32wb_boot_usbd_event_callback
#define stm32wb_usbd_is_suspended      stm32wb_boot_usbd_is_suspended
#define stm32wb_usbd_language          stm32wb_boot_usbd_language
#define stm32wb_usbd_msos20            stm32wb_boot_usbd_msos20
#define stm32wb_usbd_request           stm32wb_boot_usbd_request
#define stm32wb_usbd_serial            stm32wb_boot_usbd_serial
#define stm32wb_usbd_set_address       stm32wb_boot_usbd_set_address
#define stm32wb_usbd_start             stm32wb_boot_usbd_start
#define stm32wb_usbd_state             stm32wb_boot_usbd_state
#define stm32wb_usbd_stop              stm32wb_boot_usbd_stop
#define stm32wb_usbd_string            stm32wb_boot_usbd_string
#define __svc_stm32wb_usbd_disable     __svc_stm32wb_boot_usbd_disable
#define __svc_stm32wb_usbd_enable      __svc_stm32wb_boot_usbd_enable
#define __svc_stm32wb_usbd_start       __svc_stm32wb_boot_usbd_start
#define __svc_stm32wb_usbd_stop        __svc_stm32wb_boot_usbd_stop

#include "stm32wb_usbd.h"

static void stm32wb_boot_dfu_configure(void *context, uint8_t interface);
static void stm32wb_boot_dfu_start(void *context);
static void stm32wb_boot_dfu_stop(void *context);
static int stm32wb_boot_dfu_request(void *context, int state, const stm32wb_usbd_request_t *request, uint8_t *data, const uint8_t **p_data_return, uint16_t *p_length_return, stm32wb_usbd_status_routine_t *p_status_routine_return);

#if (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1)
#define STM32WB_BOOT_DFU_CONFIGURATION_SIZE         (9 + (9+9+9))
#define STM32WB_BOOT_DFU_STRING_COUNT               2
#else /* (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */
#define STM32WB_BOOT_DFU_CONFIGURATION_SIZE         (9 + (9+9))
#define STM32WB_BOOT_DFU_STRING_COUNT               1
#endif /* (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */
#define STM32WB_BOOT_DFU_CLASS_COUNT                1
#define STM32WB_BOOT_DFU_INTERFACE_COUNT            1

static const uint8_t stm32wb_boot_dfu_configuration[STM32WB_BOOT_DFU_CONFIGURATION_SIZE] =
{
    /**** Configuration Descriptor ****/
    9,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_CONFIGURATION,                           /* bDescriptorType */
    STM32WB_BOOT_DFU_CONFIGURATION_SIZE & 255,                   /* wTotalLength */
    STM32WB_BOOT_DFU_CONFIGURATION_SIZE >> 8,
    1,                                                           /* bNumInterfaces */
    0x01,                                                        /* bConfigurationValue */
    0x02,                                                        /* iConfiguration */
    0x80,                                                        /* bmAttributes */
    0xfa,                                                        /* bMaxPower */

    /**** DFU Interface (Application) ****/
    9,                                                           /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x00,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x00,                                                        /* bNumEndpoints */
    0xfe,                                                        /* bInterfaceClass */
    0x01,                                                        /* bInterfaceSubClass */
    0x02,                                                        /* nInterfaceProtocol */
    0x04,                                                        /* iInterface */

#if (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1)
    /**** DFU Interface (Wireless) ****/
    9,                                                           /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x00,                                                        /* bInterfaceNumber */
    0x01,                                                        /* bAlternateSetting */
    0x00,                                                        /* bNumEndpoints */
    0xfe,                                                        /* bInterfaceClass */
    0x01,                                                        /* bInterfaceSubClass */
    0x02,                                                        /* nInterfaceProtocol */
    0x05,                                                        /* iInterface */
#endif /* (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */
    
    /**** DFU Descriptor ****/
    0x09,                                                        /* blength = 9 Bytes */
    0x21,                                                        /* bDescriptorType */
    0x0d,                                                        /* bmAttribute */
    0xff, 0xff,                                                  /* wDetachTimeOut= 255 ms */
    0x00, 0x04,                                                  /* wTransferSize = 1024 Bytes */
    0x10, 0x01,                                                  /* bcdDFUVersion */
};

static const char * const stm32wb_boot_dfu_strings[STM32WB_BOOT_DFU_STRING_COUNT] =
{
    "STM32WB Application",                    // 4
#if (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1)
    "STM32WB Wireless",                       // 5
#endif /* (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */
};

static const uint8_t stm32wb_boot_dfu_bos[] = {
    // BOS
    0x05,                                            // bLength
    USB_DESCRIPTOR_TYPE_BOS,                         // bDescriptorType
    0x21, 0x00,                                      // wTotalLength
    0x01,                                            // bNumDeviceCaps

    // Microsoft OS 2.0 platform capability
    0x1c,                                            // bLength
    USB_DESCRIPTOR_TYPE_DEVICE_CAPABILITY,           // bDescriptorType
    USB_DEVICE_CAPABILITY_TYPE_PLATFORM,             // bDevCapabilityType
    0x00,                                            // bReserved
    0xdf, 0x60, 0xdd, 0xd8, 0x89, 0x45, 0xc7, 0x4c,  // platformCapabilityUUID[16]     
    0x9c, 0xd2, 0x65, 0x9d, 0x9e, 0x64, 0x8a, 0x9f,  // platformCapabilityUUID[16]     
    0x00, 0x00, 0x03, 0x06,                          // dwWindowsVersion (Windows 8.1)
    0xb2, 0x00,                                      // wLength (MSOS20)
    USB_REQ_MS_VENDOR_CODE,                          // bMS_VendorCode
    0x00,                                            // bAltEnumCode
};

static const uint8_t stm32wb_boot_dfu_msos20[] = {
    // Microsoft OS 2.0 descriptor set header
    0x0a, 0x00,                                      // wLength
    0x00, 0x00,                                      // wDescriptorType
    0x00, 0x00, 0x03, 0x06,                          // dwWindowsVersion (Windows 8.1)
    0xb2, 0x00,                                      // wTotalLength

    // Microsoft OS 2.0 configuration subset header
    0x08, 0x00,                                      // wLength
    0x01, 0x00,                                      // wDescriptorType
    0x00,                                            // bConfigurationValue
    0x00,                                            // bReserved
    0xa8, 0x00,                                      // wTotalLength (size of entire configuration subset)

    // Microsoft OS 2.0 function subset header
    0x08, 0x00,                                      // wLength
    0x02, 0x00,                                      // wDescriptorType
    0x00,                                            // bFirstInterface
    0x00,                                            // bReserved
    0xa0, 0x00,                                      // wTotalLength  (size of entine function subset)

    // Microsoft OS 2.0 compatible ID
    0x14, 0x00,                                      // wLength
    0x03, 0x00,                                      // wDescriptorType
    'W',  'I',  'N',  'U',  'S',  'B',  0x00, 0x00,  // compatible ID
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // subCompatibleID
                                        
    // Microsoft OS 2.0 registry property
    0x84, 0x00,                                      // wLength
    0x04, 0x00,                                      // wDescriptorType
    0x07, 0x00,                                      // wPropertyDataType
    0x2a, 0x00,                                      // wPropertyNameLength
    'D',  0x00, 'e',  0x00, 'v',  0x00, 'i',  0x00,  // bPropertyName "DeviceInterfaceGUIDs"
    'c',  0x00, 'e',  0x00, 'I',  0x00, 'n',  0x00,    
    't',  0x00, 'e',  0x00, 'r',  0x00, 'f',  0x00,    
    'a',  0x00, 'c',  0x00, 'e',  0x00, 'G',  0x00,    
    'U',  0x00, 'I',  0x00, 'D',  0x00, 's',  0x00,        
    0x00, 0x00,
    0x50, 0x00,                                      // wPropertyDataLength
    '{',  0x00, 'D',  0x00, 'E',  0x00, '5',  0x00,  // bProperyData "{DE50DD7B-4DEF-4F9F-98AA-88144B16A383}"
    '0',  0x00, 'D',  0x00, 'D',  0x00, '7',  0x00,
    'B',  0x00, '-',  0x00, '4',  0x00, 'D',  0x00,
    'E',  0x00, 'F',  0x00, '-',  0x00, '4',  0x00,
    'F',  0x00, '9',  0x00, 'F',  0x00, '-',  0x00,
    '9',  0x00, '8',  0x00, 'A',  0x00, 'A',  0x00,
    '-',  0x00, '8',  0x00, '8',  0x00, '1',  0x00,
    '4',  0x00, '4',  0x00, 'B',  0x00, '1',  0x00,
    '6',  0x00, 'A',  0x00, '3',  0x00, '8',  0x00,
    '3',  0x00, '}',  0x00,
    0x00, 0x00, 0x00, 0x00,
};

static const stm32wb_usbd_device_t stm32wb_boot_dfu_device =
{
    0x0483,
    0xdf11,
    0x0200,
    "STMicroelectronics",
    "STM32WB in DFU Mode",
};

#define stm32wb_boot_dfu_ep0_data stm32wb_boot_io2_data

#include "stm32wb_usbd.c"

#define DFU_ALTSETTING_APPLICATION       0
#define DFU_ALTSETTING_WIRELESS          1

#define DFU_REQUEST_DETACH               0x00
#define DFU_REQUEST_DNLOAD               0x01
#define DFU_REQUEST_UPLOAD               0x02
#define DFU_REQUEST_GETSTATUS            0x03
#define DFU_REQUEST_CLRSTATUS            0x04
#define DFU_REQUEST_GETSTATE             0x05
#define DFU_REQUEST_ABORT                0x06

#define DFU_STATUS_OK                    0x00
#define DFU_STATUS_ERR_TARGET            0x01
#define DFU_STATUS_ERR_FILE              0x02
#define DFU_STATUS_ERR_WRITE             0x03
#define DFU_STATUS_ERR_ERASE             0x04
#define DFU_STATUS_ERR_CHECK_ERASED      0x05
#define DFU_STATUS_ERR_PROG              0x06
#define DFU_STATUS_ERR_VERIFY            0x07
#define DFU_STATUS_ERR_ADDRESS           0x08
#define DFU_STATUS_ERR_NOTDONE           0x09
#define DFU_STATUS_ERR_FIRMWARE          0x0a
#define DFU_STATUS_ERR_VENDOR            0x0b
#define DFU_STATUS_ERR_USBR              0x0c
#define DFU_STATUS_ERR_POR               0x0d
#define DFU_STATUS_ERR_UNKNOWN           0x0e
#define DFU_STATUS_ERR_STALLEDPKT        0x0f

#define DFU_STATE_APP_IDLE               0
#define DFU_STATE_APP_DETACH             1
#define DFU_STATE_IDLE                   2
#define DFU_STATE_DNLOAD_SYNC            3
#define DFU_STATE_DNBUSY                 4
#define DFU_STATE_DNLOAD_IDLE            5
#define DFU_STATE_MANIFEST_SYNC          6
#define DFU_STATE_MANIFEST               7
#define DFU_STATE_MANIFEST_WAIT_RESET    8
#define DFU_STATE_UPLOAD_IDLE            9
#define DFU_STATE_ERROR                  10

typedef struct _stm32wb_boot_dfu_control_t {
    uint8_t                       interface;
    uint8_t                       altsetting;
    volatile uint8_t              status;
    volatile uint8_t              state;

    volatile uint32_t             events;

    uint32_t                      erase_address;
    uint32_t                      erase_size;
    uint32_t                      erase_time;
    uint32_t                      program_address;
    uint32_t                      program_size;
    uint32_t                      program_time;
    uint32_t                      boot_size;
    uint32_t                      application_size;
    uint32_t                      image_size;
    uint32_t                      image_count;
    uint8_t                       *image_data;
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
    uint32_t                      application_info;
    stm32wb_boot_aes128_context_t aes128_ctx;
    uint32_t                      aes128_iv[4];
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
} stm32wb_boot_dfu_control_t;

static stm32wb_boot_dfu_control_t stm32wb_boot_dfu_control;

#define STM32WB_DFU_PROGRAM_SIZE       1024

#define STM32WB_DFU_EVENT_START        0x00000001
#define STM32WB_DFU_EVENT_WRITE        0x00000002
#define STM32WB_DFU_EVENT_FINISH       0x00000004
#define STM32WB_DFU_EVENT_DETACH       0x00000008

static void stm32wb_boot_dfu_detach()
{
    __armv7m_atomic_or(&stm32wb_boot_dfu_control.events, STM32WB_DFU_EVENT_DETACH);
}

static void stm32wb_boot_dfu_dnload()
{
    uint32_t events;

    events = STM32WB_DFU_EVENT_WRITE;
    
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
    if (stm32wb_boot_dfu_control.erase_address == stm32wb_boot_info.application_base)
#else /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
    if (stm32wb_boot_dfu_control.erase_address == stm32wb_boot_info.fwu_base)
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
    {
        events |= STM32WB_DFU_EVENT_START;
    }

    __armv7m_atomic_or(&stm32wb_boot_dfu_control.events, events);
}

static void stm32wb_boot_dfu_manifest()
{
    uint32_t events;

    events = STM32WB_DFU_EVENT_FINISH;

    if (stm32wb_boot_dfu_control.image_count)
    {
        events |= STM32WB_DFU_EVENT_WRITE;
    }

    __armv7m_atomic_or(&stm32wb_boot_dfu_control.events, events);
}

static void stm32wb_boot_dfu_configure(void *context, uint8_t interface)
{
    stm32wb_boot_dfu_control.interface = interface;
}

static void stm32wb_boot_dfu_start(void *context)
{
    stm32wb_boot_dfu_control.altsetting = DFU_ALTSETTING_APPLICATION;
}

static void stm32wb_boot_dfu_stop(void *context)
{
}

static int stm32wb_boot_dfu_request(void *context, int state, const stm32wb_usbd_request_t *request, uint8_t *data, const uint8_t **p_data_return, uint16_t *p_length_return, stm32wb_usbd_status_routine_t *p_status_routine_return)
{
    uint16_t interface;
    int status, dfu_state, dfu_status, dfu_timeout;
    uint32_t application_size, application_length, image_offset, image_skip, signature_size;
    const stm32wb_fwu_prefix_t *application_prefix;
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
    uint32_t aes128_key[4];
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */

    status = STM32WB_USBD_REQUEST_STATUS_UNHANDLED;
    
    switch (request->bmRequestType & USB_REQ_RECIPIENT_MASK) {
    case USB_REQ_RECIPIENT_INTERFACE: {
	interface = request->wIndex;
	
	if (interface == stm32wb_boot_dfu_control.interface)
	{
	    switch (request->bmRequestType & USB_REQ_TYPE_MASK) {
            case USB_REQ_TYPE_STANDARD: {
                switch (request->bRequest) {
                case USB_REQ_CODE_GET_INTERFACE: {
                    *p_data_return = data;
                    *p_length_return = 1;
                        
                    data[0] = stm32wb_boot_dfu_control.altsetting;
                    
                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    break;
                }
                            
                case USB_REQ_CODE_SET_INTERFACE: {
#if (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1)
                    if ((request->wValue == 0) || (request->wValue == 1))
#else /* (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */
                    if (request->wValue == 0)
#endif /* (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */
                    {
                        stm32wb_boot_dfu_control.altsetting = request->wValue;

                        status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    }
                    break;
                }

                default:
                    break;
                }
                break;
            }
                
	    case USB_REQ_TYPE_CLASS: {
                dfu_state = DFU_STATE_ERROR;
                dfu_status = DFU_STATUS_ERR_STALLEDPKT;
                
		switch (request->bRequest) {
                    
                case DFU_REQUEST_DETACH: {
                    *p_status_routine_return = stm32wb_boot_dfu_detach;
                        
                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    break;
                }
                    
                case DFU_REQUEST_DNLOAD: {     
                    if ((stm32wb_boot_dfu_control.state == DFU_STATE_IDLE) || (stm32wb_boot_dfu_control.state == DFU_STATE_DNLOAD_IDLE))
                    {
                        if (stm32wb_boot_dfu_control.state == DFU_STATE_IDLE)
                        {
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
                            stm32wb_boot_dfu_control.erase_address = stm32wb_boot_info.application_base;
                            stm32wb_boot_dfu_control.program_address = stm32wb_boot_info.application_base;
#else /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
                            stm32wb_boot_dfu_control.erase_address = stm32wb_boot_info.fwu_base;
                            stm32wb_boot_dfu_control.program_address = stm32wb_boot_info.fwu_base;
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
                            stm32wb_boot_dfu_control.boot_size = 0;
                            stm32wb_boot_dfu_control.application_size = 0;
                            stm32wb_boot_dfu_control.image_size = 0;
                            stm32wb_boot_dfu_control.image_count = 0;
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
                            stm32wb_boot_dfu_control.application_info = 0;
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
                        }

                        if (request->wLength)
                        {
                            stm32wb_boot_memcpy(&stm32wb_boot_dfu_control.image_data[stm32wb_boot_dfu_control.image_count], data, request->wLength);

                            stm32wb_boot_dfu_control.image_size += request->wLength;
                            stm32wb_boot_dfu_control.image_count += request->wLength;

                            image_offset = stm32wb_boot_dfu_control.image_size - stm32wb_boot_dfu_control.image_count;

                            if (stm32wb_boot_dfu_control.altsetting == DFU_ALTSETTING_APPLICATION)
                            {
                                if ((image_offset == stm32wb_boot_dfu_control.boot_size) && (stm32wb_boot_dfu_control.image_count >= sizeof(stm32wb_application_vectors_t)))
                                {
                                    if ((image_offset == 0) &&
                                        (((const stm32wb_boot_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->magic == ((const stm32wb_boot_vectors_t*)stm32wb_boot_vectors)->magic) &&
                                        (((const stm32wb_boot_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->base == ((const stm32wb_boot_vectors_t*)stm32wb_boot_vectors)->base) &&
                                        (((const stm32wb_boot_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->size == ((const stm32wb_boot_vectors_t*)stm32wb_boot_vectors)->size))
                                    {
                                        stm32wb_boot_dfu_control.boot_size = ((const stm32wb_boot_vectors_t*)stm32wb_boot_vectors)->size;
                                    }
                                    else
                                    {
                                        if (((const stm32wb_application_vectors_t*)stm32wb_boot_vectors)->magic & STM32WB_BOOT_OPTION_SIGNATURE_MASK)
                                        {
                                            signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_ECC256;
                                        }
                                        else
                                        {
                                            signature_size = STM32WB_APPLICATION_SIGNATURE_SIZE_NONE;
                                        }
                                      
                                        if ((((const stm32wb_application_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->magic == ((((const stm32wb_boot_vectors_t*)stm32wb_boot_vectors)->magic & ~STM32WB_BOOT_TYPE_MASK) | STM32WB_APPLICATION_TYPE_APPLICATION)) &&
                                            (((const stm32wb_application_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->base == stm32wb_boot_info.application_base))
                                        {
                                            application_size = ((const stm32wb_application_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->size;
                                            
                                            if ((application_size >= (sizeof(stm32wb_application_vectors_t) + sizeof(stm32wb_application_info_t))) &&
                                                ((application_size + signature_size) <= (stm32wb_boot_info.application_limit - stm32wb_boot_info.application_base)) &&
                                                !(application_size & 15))
                                            {
                                                stm32wb_boot_dfu_control.application_size = application_size + signature_size;
                                            }
                                        }
                                        
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
                                        if (((const stm32wb_fwu_prefix_t*)&stm32wb_boot_dfu_control.image_data[0])->magic == ((((const stm32wb_boot_vectors_t*)stm32wb_boot_vectors)->magic & ~(STM32WB_BOOT_TAG_MASK | STM32WB_BOOT_TYPE_MASK)) | STM32WB_FWU_TAG_FWU))
#else /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
                                            if ((((const stm32wb_fwu_prefix_t*)&stm32wb_boot_dfu_control.image_data[0])->magic & ~STM32WB_FWU_TYPE_MASK) == ((((const stm32wb_boot_vectors_t*)stm32wb_boot_vectors)->magic & ~(STM32WB_BOOT_TAG_MASK | STM32WB_BOOT_TYPE_MASK)) | STM32WB_FWU_TAG_FWU))
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
                                        {
                                            application_prefix = ((const stm32wb_fwu_prefix_t*)&stm32wb_boot_dfu_control.image_data[0]);
                                            
                                            application_size = application_prefix->size;
                                            application_length = application_prefix->length; 
                                            
                                            if ((application_size >= (sizeof(stm32wb_application_vectors_t) + sizeof(stm32wb_application_info_t))) &&
                                                ((application_size + signature_size) <= (stm32wb_boot_info.application_limit - stm32wb_boot_info.application_base)) &&
                                                !(application_size & 15) &&
                                                !(application_length & 15))
                                            {
                                                stm32wb_boot_dfu_control.application_size = sizeof(stm32wb_fwu_prefix_t) + application_length;

#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
                                                if (signature_size)
                                                {
                                                    stm32wb_boot_dfu_control.application_info = stm32wb_boot_info.application_base;
                                                    
                                                    stm32wb_boot_dfu_control.application_info += (application_size - sizeof(stm32wb_application_info_t));
                                                    
                                                    stm32wb_boot_aes128_get_key(&application_prefix->nonce[0], &aes128_key[0]);
                                                    
                                                    stm32wb_boot_aes128_set_key(&stm32wb_boot_dfu_control.aes128_ctx, &aes128_key[0]);
                                                    
                                                    stm32wb_boot_dfu_control.aes128_iv[0] = 0;
                                                    stm32wb_boot_dfu_control.aes128_iv[1] = 0;
                                                    stm32wb_boot_dfu_control.aes128_iv[2] = 0;
                                                    stm32wb_boot_dfu_control.aes128_iv[3] = 0;
                                                }
                                                else
                                                {
                                                    stm32wb_boot_dfu_control.application_info = 0x00000000;
                                                }
                                                
                                                stm32wb_boot_dfu_control.image_count -= sizeof(stm32wb_fwu_prefix_t);
                                                
                                                stm32wb_boot_memcpy(&stm32wb_boot_dfu_control.image_data[0], &stm32wb_boot_dfu_control.image_data[sizeof(stm32wb_fwu_prefix_t)], stm32wb_boot_dfu_control.image_count);
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
                                            }
                                        }
                                    }
                                    
                                    if ((stm32wb_boot_dfu_control.boot_size == 0) && (stm32wb_boot_dfu_control.application_size == 0))
                                    {
                                        dfu_state = DFU_STATE_ERROR;
                                        dfu_status = DFU_STATUS_ERR_FILE;
                                        
                                        status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                                    }
                                }
                                    
                                if ((stm32wb_boot_dfu_control.boot_size != 0) || (stm32wb_boot_dfu_control.application_size != 0))
                                {
                                    if (status != STM32WB_USBD_REQUEST_STATUS_FAILURE)
                                    {
                                        if (stm32wb_boot_dfu_control.image_size > (stm32wb_boot_dfu_control.boot_size + stm32wb_boot_dfu_control.application_size))
                                        {
                                            dfu_state = DFU_STATE_ERROR;
                                            dfu_status = DFU_STATUS_ERR_ADDRESS;
                                                
                                            status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                                        }
                                    }
                                        
                                    if (status != STM32WB_USBD_REQUEST_STATUS_FAILURE)
                                    {
                                        if (stm32wb_boot_dfu_control.boot_size > image_offset)
                                        {
                                            image_skip = (stm32wb_boot_dfu_control.boot_size - image_offset);

                                            if (image_skip > stm32wb_boot_dfu_control.image_count)
                                            {
                                                image_skip = stm32wb_boot_dfu_control.image_count;
                                            }
                                            
                                            stm32wb_boot_dfu_control.image_count -= image_skip;
                                                
                                            if (stm32wb_boot_dfu_control.image_count)
                                            {
                                                stm32wb_boot_memcpy(&stm32wb_boot_dfu_control.image_data[0],
                                                                    &stm32wb_boot_dfu_control.image_data[image_skip],
                                                                    stm32wb_boot_dfu_control.image_count);
                                            }
                                                
                                            dfu_state = DFU_STATE_DNLOAD_IDLE;
                                            dfu_status = DFU_STATUS_OK;
                                        }
                                        else
                                        {
                                            dfu_state = DFU_STATE_DNLOAD_SYNC;
                                            dfu_status = DFU_STATUS_OK;
                                        }
                                            
                                        status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                                    }
                                }
                            }
#if (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1)
                            else
                            {
                                if (((const stm32wb_boot_vectors_t*)stm32wb_boot_vectors)->magic & STM32WB_BOOT_OPTION_PROTECTED)
                                {
                                    dfu_state = DFU_STATE_ERROR;
                                    dfu_status = DFU_STATUS_ERR_FILE;
                                    
                                    status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                                }
                                else
                                {
                                    if (stm32wb_boot_dfu_control.image_size > (stm32wb_boot_info.fwu_limit - stm32wb_boot_info.fwu_base - STM32WB_BOOT_WIRELESS_PREFIX_SIZE))
                                    {
                                        dfu_state = DFU_STATE_ERROR;
                                        dfu_status = DFU_STATUS_ERR_ADDRESS;
                                        
                                        status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                                    }
                                    else
                                    {
                                        dfu_state = DFU_STATE_DNLOAD_SYNC;
                                        dfu_status = DFU_STATUS_OK;
                                        
                                        status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                                    }
                                }
                            }
#endif /* (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */
                            
                        }
                        else
                        {
                            if (stm32wb_boot_dfu_control.altsetting == DFU_ALTSETTING_APPLICATION)
                            {
                                if (stm32wb_boot_dfu_control.application_size == 0)
                                {
                                    dfu_state = DFU_STATE_ERROR;
                                    dfu_status = DFU_STATUS_ERR_FILE;
                                    
                                    status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                                }
                                else
                                {
                                    if (stm32wb_boot_dfu_control.image_size != (stm32wb_boot_dfu_control.boot_size + stm32wb_boot_dfu_control.application_size))
                                    {
                                        dfu_state = DFU_STATE_ERROR;
                                        dfu_status = DFU_STATUS_ERR_NOTDONE;
                                        
                                        status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                                    }
                                }
                            }
                                
                            if (status != STM32WB_USBD_REQUEST_STATUS_FAILURE)
                            {
                                dfu_state = DFU_STATE_MANIFEST_SYNC;
                                dfu_status = DFU_STATUS_OK;
                                    
                                status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                            }
                        }
                    }
                    else
                    {
                        status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                    }
                    break;
                }

                case DFU_REQUEST_UPLOAD: {     
                    status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                    break;
                }
                    
                case DFU_REQUEST_GETSTATUS: {
                    dfu_state = stm32wb_boot_dfu_control.state;
                    dfu_status = stm32wb_boot_dfu_control.status;
                    dfu_timeout = 0;
                    
                    *p_data_return = data;
                    *p_length_return = 6;

                    if (dfu_state == DFU_STATE_DNBUSY)
                    {
                        dfu_timeout = 5;
                    }

                    if (dfu_state == DFU_STATE_DNLOAD_SYNC)
                    {
                        if (stm32wb_boot_dfu_control.image_count >= STM32WB_DFU_PROGRAM_SIZE)
                        {
                            dfu_state = DFU_STATE_DNBUSY;
                        
                            *p_status_routine_return = stm32wb_boot_dfu_dnload;
                            
                            dfu_timeout += stm32wb_boot_dfu_control.program_time;
                            
                            if (stm32wb_boot_dfu_control.erase_address == stm32wb_boot_info.fwu_base)
                            {
                                dfu_timeout += STM32WB_BOOT_FLASH_ERASE_TIME;
                            }
                            
                            if (stm32wb_boot_dfu_control.erase_address == stm32wb_boot_dfu_control.program_address)
                            {
                                dfu_timeout += stm32wb_boot_dfu_control.erase_time;
                            }
                        }
                        else
                        {
                            dfu_state = DFU_STATE_DNLOAD_IDLE;
                        }
                    }

                    if (dfu_state == DFU_STATE_MANIFEST)
                    {
                        dfu_timeout = 250;
                    }

                    if (dfu_state == DFU_STATE_MANIFEST_SYNC)
                    {
                        *p_status_routine_return = stm32wb_boot_dfu_manifest;

                        dfu_state = DFU_STATE_MANIFEST;

                        dfu_timeout = 250;
                    }

                    data[0] = dfu_status;
                    data[1] = dfu_timeout >> 0;
                    data[2] = dfu_timeout >> 8;
                    data[3] = dfu_timeout >> 16;
                    data[4] = dfu_state;
                    data[5] = 0;

                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    break;
                }
                    
                case DFU_REQUEST_CLRSTATUS: {
                    if (stm32wb_boot_dfu_control.state == DFU_STATE_ERROR)
                    {
                        dfu_state = DFU_STATE_IDLE;
                        dfu_status = DFU_STATUS_OK;

                        status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    }
                    else
                    {
                        dfu_state = DFU_STATE_ERROR;
                        dfu_status = stm32wb_boot_dfu_control.status;
                        
                        status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                    }
                    break;
                }
                    
                case DFU_REQUEST_GETSTATE: {
                    dfu_state = stm32wb_boot_dfu_control.state;
                    dfu_status = stm32wb_boot_dfu_control.status;

                    *p_data_return = data;
                    *p_length_return = 1;

                    data[0] = dfu_state;
                    
                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    break;
                }
                    
                case DFU_REQUEST_ABORT: {
                    if ((stm32wb_boot_dfu_control.state == DFU_STATE_IDLE) || (stm32wb_boot_dfu_control.state == DFU_STATE_DNLOAD_IDLE))
                    {
                        dfu_state = DFU_STATE_IDLE;
                        dfu_status = DFU_STATUS_OK;

                        status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    }
                    else
                    {
                        status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                    }
                    break;
                }
                    
		default:
		    break;
		}

                if (status != STM32WB_USBD_REQUEST_STATUS_UNHANDLED)
                {
                    stm32wb_boot_dfu_control.state = dfu_state;
                    stm32wb_boot_dfu_control.status = dfu_status;
                }
		break;
	    }

	    default:
		break;
	    }
	}
	break;
    }
	
    default:
	break;
    }
    
    return status;
}

static void __attribute__((noreturn, noinline)) stm32wb_boot_dfu(const stm32wb_application_info_t *application_info)
{
    uint32_t events, size, count, data[8];
    uint8_t dfu_state, dfu_status;
    
    RTC->WPR = 0xca;
    RTC->WPR = 0x53;
                    
    RTC->BKP16R = ((RTC->BKP16R & ~(STM32WB_RTC_BKP16R_DFU)) | ((STM32WB_RTC_BKP16R_DFU) << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT));
    
    RTC->WPR = 0x00;
    
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0)
#if (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1)
    if (stm32wb_boot_info.fwu_base < STM32WB_BOOT_SFLASH_LIMIT)
    {
        const stm32wb_boot_sflash_info_t *sflash_info;

        sflash_info = stm32wb_boot_sflash_init();

        stm32wb_boot_dfu_control.erase_size = sflash_info->erase_size;
        stm32wb_boot_dfu_control.program_size = sflash_info->page_size;
    }
    else
#endif /* (STM32WB_BOOT_FWU_CONFIG_SFLASH == 1) */
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0) */
    {
        stm32wb_boot_dfu_control.erase_size = STM32WB_BOOT_FLASH_ERASE_SIZE;
        stm32wb_boot_dfu_control.program_size = STM32WB_BOOT_FLASH_PROGRAM_SIZE;
    }

    stm32wb_boot_dfu_control.erase_time = STM32WB_BOOT_FLASH_ERASE_TIME;
    stm32wb_boot_dfu_control.program_time = (((STM32WB_DFU_PROGRAM_SIZE / STM32WB_BOOT_FLASH_PROGRAM_SIZE) * STM32WB_BOOT_FLASH_PROGRAM_TIME) + 999) / 1000;
    
    if (application_info)
    {
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0)
        if (stm32wb_boot_fwu_state() == STM32WB_FWU_STATE_FAILED)
        {
            stm32wb_boot_dfu_control.status = ((const uint8_t*)(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_FAILED))[0];
            stm32wb_boot_dfu_control.state = DFU_STATE_ERROR;
        }
        else
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0) */
        {
            stm32wb_boot_dfu_control.status = DFU_STATUS_OK;
            stm32wb_boot_dfu_control.state = DFU_STATE_IDLE;
        }
    }
    else
    {
        stm32wb_boot_dfu_control.status = DFU_STATUS_ERR_FIRMWARE;
        stm32wb_boot_dfu_control.state = DFU_STATE_ERROR;
    }

    stm32wb_boot_dfu_control.image_data = &stm32wb_boot_io8_data[0];
    
    stm32wb_usbd_configure(&stm32wb_boot_dfu_device, NULL);
    
    stm32wb_usbd_enable();
    
    while (1)
    {
        __WFE();
        
        events = __armv7m_atomic_swap(&stm32wb_boot_dfu_control.events, 0);
        
        if (events)
        {
            dfu_state = stm32wb_boot_dfu_control.state;
            dfu_status = stm32wb_boot_dfu_control.status;
            
            if (events & STM32WB_DFU_EVENT_START)
            {
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0)
                if (!stm32wb_boot_flash_erase(stm32wb_boot_info.fwu_status))
                {
                    dfu_status = DFU_STATUS_ERR_ERASE;
                }

                if (dfu_status == DFU_STATUS_OK)
                {
                    data[0] = 0x00000000;
                    data[1] = 0x00000000;
                        
                    if (!stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_READY, (const uint8_t*)&data[0], 8))
                    {
                        dfu_status = DFU_STATUS_ERR_PROG;
                    }
                }

                if (dfu_status == DFU_STATUS_OK)
                {
#if (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1)
                    data[0] = (stm32wb_boot_dfu_control.altsetting == DFU_ALTSETTING_APPLICATION) ? STM32WB_FWU_COMPONENT_APPLICATION : STM32WB_FWU_COMPONENT_WIRELESS;
#else /* (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */
                    data[0] = STM32WB_FWU_COMPONENT_APPLICATION;
#endif /* (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */
                    data[1] = 0x00000000;
                        
                    if (!stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_WRITING, (const uint8_t*)&data[0], 8))
                    {
                        dfu_status = DFU_STATUS_ERR_PROG;
                    }
                }
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0) */
            }

            if (events & STM32WB_DFU_EVENT_WRITE)
            {
                size = STM32WB_DFU_PROGRAM_SIZE;
                
                if (size > stm32wb_boot_dfu_control.image_count)
                {
                    size = stm32wb_boot_dfu_control.image_count;
                }
                
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
                if (stm32wb_boot_dfu_control.altsetting == DFU_ALTSETTING_APPLICATION)
                {
                    if (stm32wb_boot_dfu_control.program_address < stm32wb_boot_dfu_control.application_info)
                    {
                        count = size;
                        
                        if (count > (stm32wb_boot_dfu_control.application_info - stm32wb_boot_dfu_control.program_address))
                        {
                            count = (stm32wb_boot_dfu_control.application_info - stm32wb_boot_dfu_control.program_address);
                        }
                        
                        if (count)
                        {
                            stm32wb_boot_dfu_control.aes128_iv[3] = SWAP((stm32wb_boot_dfu_control.program_address - stm32wb_boot_info.application_base) / 16);
                            
                            stm32wb_boot_aes128_ctr_encrypt(&stm32wb_boot_dfu_control.aes128_ctx, &stm32wb_boot_dfu_control.aes128_iv[0], (const uint32_t*)&stm32wb_boot_dfu_control.image_data[0], (uint32_t*)&stm32wb_boot_dfu_control.image_data[0], count);
                        }
                    }
                }
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
                
                if (dfu_status == DFU_STATUS_OK)
                {
                    if (stm32wb_boot_dfu_control.erase_address == stm32wb_boot_dfu_control.program_address)
                    {
                        if (!stm32wb_boot_nvm_erase(stm32wb_boot_dfu_control.erase_address))
                        {
                            dfu_status = DFU_STATUS_ERR_ERASE;
                        }
                        else
                        {
                            stm32wb_boot_dfu_control.erase_address += stm32wb_boot_dfu_control.erase_size;
                        }
                    }
                }

                if (dfu_status == DFU_STATUS_OK)
                {
#if (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1)
                    if (stm32wb_boot_dfu_control.altsetting == DFU_ALTSETTING_WIRELESS)
                    {
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
                        if (stm32wb_boot_dfu_control.program_address == stm32wb_boot_info.application_base)
#else /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
                        if (stm32wb_boot_dfu_control.program_address == stm32wb_boot_info.fwu_base)
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
                        {
                            stm32wb_boot_dfu_control.program_address += STM32WB_BOOT_WIRELESS_PREFIX_SIZE;

                            size -= STM32WB_BOOT_WIRELESS_PREFIX_SIZE;
                        }
                    }
#endif /* (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */
                    
                    dfu_state = DFU_STATE_DNLOAD_IDLE;
                    
                    for (; size; size -= count)
                    {
                        count = stm32wb_boot_dfu_control.program_size;

                        if (count > size)
                        {
                            count = size;
                        }
                    
                        if (!stm32wb_boot_nvm_program(stm32wb_boot_dfu_control.program_address, &stm32wb_boot_dfu_control.image_data[0], ((count + 15) & ~15), false))
                        {
                            dfu_status = DFU_STATUS_ERR_PROG;

                            break;
                        }

                        stm32wb_boot_dfu_control.program_address += count;
                        
                        stm32wb_boot_dfu_control.image_count -= count;
                        
                        if (stm32wb_boot_dfu_control.image_count)
                        {
                            stm32wb_boot_memcpy(&stm32wb_boot_dfu_control.image_data[0],
                                                &stm32wb_boot_dfu_control.image_data[count],
                                                stm32wb_boot_dfu_control.image_count);
                        }
                    }
                }
            }
            
            if (events & STM32WB_DFU_EVENT_FINISH)
            {
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0)
                if (dfu_status == DFU_STATUS_OK)
                {
                    data[0] = stm32wb_boot_dfu_control.image_size;
                    data[1] = 0;
                    
                    if (!stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + STM32WB_FWU_STATUS_OFFSET_CANDIDATE, (const uint8_t*)&data[0], 8))
                    {
                        dfu_status = DFU_STATUS_ERR_PROG;
                    }
                }
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0) */
            
                if (dfu_status == DFU_STATUS_OK)
                {
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
#if (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1)
                    if (stm32wb_boot_dfu_control.altsetting == DFU_ALTSETTING_WIRELESS)
                    {
                        data[0] = stm32wb_boot_dfu_control.image_size;
                        data[1] = STM32WB_BOOT_WIRELESS_PREFIX_MAGIC;
                    
                        if (!stm32wb_boot_flash_program(stm32wb_boot_info.application_base, (const uint8_t*)&data[0], 8))
                        {
                            dfu_status = DFU_STATUS_ERR_PROG;
                        }
                    }
#endif /* (STM32WB_BOOT_DFU_CONFIG_WIRELESS == 1) */
#else /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
                    data[0] = dfu_status; // DFU_STATUSOK == STM32WB_FWU_MODE_ACCEPT ...
                    data[1] = 0x00000000; 
                    
                    if (!stm32wb_boot_flash_program(stm32wb_boot_info.fwu_status + ((dfu_status == DFU_STATUS_OK) ? STM32WB_FWU_STATUS_OFFSET_STAGED : STM32WB_FWU_STATUS_OFFSET_FAILED), (const uint8_t*)&data[0], 8))
                    {
                        dfu_status = DFU_STATUS_ERR_PROG;
                    }
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 0) */
                }
                    
                if (dfu_status == DFU_STATUS_OK)
                {
                    if (stm32wb_boot_dfu_control.altsetting == DFU_ALTSETTING_APPLICATION)
                    {
#if (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1)
                        if (!stm32wb_boot_application_info())
                        {
                            dfu_status = DFU_STATUS_ERR_FILE;
                        }
#else /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
                        dfu_status = stm32wb_boot_fwu_install(application_info);
#endif /* (STM32WB_BOOT_DFU_CONFIG_STANDALONE == 1) */
                    }
                }
                
                dfu_state = DFU_STATE_IDLE;
            }
            
            if (events & STM32WB_DFU_EVENT_DETACH)
            {
                stm32wb_usbd_disable();
                
                stm32wb_boot_reset();
            }
            
            if (dfu_status != DFU_STATUS_OK)
            {
                dfu_state = DFU_STATE_ERROR;
            }
            
            stm32wb_boot_dfu_control.status = dfu_status;
            stm32wb_boot_dfu_control.state = dfu_state;
        }
    }
}

/************************************************************************************************************************************/
