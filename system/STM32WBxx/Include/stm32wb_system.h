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

#if !defined(_STM32WB_SYSTEM_H)
#define _STM32WB_SYSTEM_H

#include "armv7m.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    STM32WB_SYSTEM_PERIPH_ADC,
    STM32WB_SYSTEM_PERIPH_USB,
    STM32WB_SYSTEM_PERIPH_USART1,
    STM32WB_SYSTEM_PERIPH_LPUART1,
    STM32WB_SYSTEM_PERIPH_I2C1,
    STM32WB_SYSTEM_PERIPH_I2C3,
    STM32WB_SYSTEM_PERIPH_SPI1,
    STM32WB_SYSTEM_PERIPH_SPI2,
    STM32WB_SYSTEM_PERIPH_QSPI,
    STM32WB_SYSTEM_PERIPH_SAI1,
    STM32WB_SYSTEM_PERIPH_TIM1,
    STM32WB_SYSTEM_PERIPH_TIM2,
    STM32WB_SYSTEM_PERIPH_TIM16,
    STM32WB_SYSTEM_PERIPH_TIM17,
    STM32WB_SYSTEM_PERIPH_LPTIM1,
    STM32WB_SYSTEM_PERIPH_LPTIM2,
    STM32WB_SYSTEM_PERIPH_TSC,
    STM32WB_SYSTEM_PERIPH_LCD,
    STM32WB_SYSTEM_PERIPH_HSEM,
    STM32WB_SYSTEM_PERIPH_IPCC,
    STM32WB_SYSTEM_PERIPH_CRC,
    STM32WB_SYSTEM_PERIPH_RNG,
    STM32WB_SYSTEM_PERIPH_AES1,
    STM32WB_SYSTEM_PERIPH_AES2,
    STM32WB_SYSTEM_PERIPH_PKA,
    STM32WB_SYSTEM_PERIPH_COUNT
};

#define STM32WB_SYSTEM_OPTION_LSE_BYPASS            0x00000001
#define STM32WB_SYSTEM_OPTION_LSE_MODE_MASK         0x00000006
#define STM32WB_SYSTEM_OPTION_LSE_MODE_SHIFT        1
#define STM32WB_SYSTEM_OPTION_LSE_MODE_0            0x00000000
#define STM32WB_SYSTEM_OPTION_LSE_MODE_1            0x00000002
#define STM32WB_SYSTEM_OPTION_LSE_MODE_2            0x00000004
#define STM32WB_SYSTEM_OPTION_LSE_MODE_3            0x00000006
#define STM32WB_SYSTEM_OPTION_HSE_BYPASS            0x00000008
#define STM32WB_SYSTEM_OPTION_VBAT_CHARGING         0x00000010
#define STM32WB_SYSTEM_OPTION_USART1_SYSCLK         0x00000020
#define STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_MASK    0x000000c0
#define STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_SHIFT   6
#define STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_10uH    0x00000040
#define STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_2_2uH   0x00000080
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_MASK     0x00000700
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_SHIFT    8
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_80mA     0x00000000
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_100mA    0x00000100
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_120mA    0x00000200
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_140mA    0x00000300
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_160mA    0x00000400
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_180mA    0x00000500
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_200mA    0x00000600
#define STM32WB_SYSTEM_OPTION_SMPS_CURRENT_220mA    0x00000700

#define STM32WB_SYSTEM_LOCK_CLOCKS                  0
#define STM32WB_SYSTEM_LOCK_SLEEP                   1
#define STM32WB_SYSTEM_LOCK_STOP_0                  2
#define STM32WB_SYSTEM_LOCK_STOP_1                  3
#define STM32WB_SYSTEM_LOCK_STOP_2                  4
#define STM32WB_SYSTEM_LOCK_STANDBY                 5
#define STM32WB_SYSTEM_LOCK_COUNT                   6

#define STM32WB_SYSTEM_FREQUENCY_LPRUN              2000000
#define STM32WB_SYSTEM_FREQUENCY_RANGE_2            16000000
#define STM32WB_SYSTEM_FREQUENCY_RANGE_1            64000000
  
#define STM32WB_SYSTEM_REFERENCE_HSI48              0x00000001 /* RANGE 1 */
#define STM32WB_SYSTEM_REFERENCE_CPU2               0x00000002 /* RANGE 1, HCLK  >= 32MHz */
#define STM32WB_SYSTEM_REFERENCE_WIRELESS           0x00000004 /* HSE vs MSI */
#define STM32WB_SYSTEM_REFERENCE_USB                0x00000008 /* RANGE 1, PCLK1 >= 16MHz */
#define STM32WB_SYSTEM_REFERENCE_RNG                0x00000010 /* RANGE 1 */
#define STM32WB_SYSTEM_REFERENCE_SWD                0x08000000
#define STM32WB_SYSTEM_REFERENCE_SYSCLK_RANGE_1     0x10000000 /* RANGE 1 */ 
#define STM32WB_SYSTEM_REFERENCE_SYSCLK_RANGE_2     0x20000000 /* RANGE 2 */
#define STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_1     0x40000000 /* RANGE 1 */
#define STM32WB_SYSTEM_REFERENCE_SAICLK_RANGE_2     0x80000000 /* RANGE 2 */

#define STM32WB_SYSTEM_NOTIFY_CLOCKS_PROLOGUE       0x00000001
#define STM32WB_SYSTEM_NOTIFY_CLOCKS_EPILOGUE       0x00000002
#define STM32WB_SYSTEM_NOTIFY_STOP_PREPARE          0x00000004
#define STM32WB_SYSTEM_NOTIFY_STOP_ENTER            0x00000008
#define STM32WB_SYSTEM_NOTIFY_STOP_LEAVE            0x00000010
#define STM32WB_SYSTEM_NOTIFY_STANDBY               0x00000020
#define STM32WB_SYSTEM_NOTIFY_SHUTDOWN              0x00000040
#define STM32WB_SYSTEM_NOTIFY_FATAL                 0x00000080
#define STM32WB_SYSTEM_NOTIFY_DFU                   0x00000100
#define STM32WB_SYSTEM_NOTIFY_OTA                   0x00000200
#define STM32WB_SYSTEM_NOTIFY_RESET                 0x00000400

typedef void (*stm32wb_system_pvm1_callback_t)(void *context);

typedef void (*stm32wb_system_callback_t)(void *context, uint32_t notify);

typedef struct _stm32wb_system_notify_t {
    struct _stm32wb_system_notify_t *next;
    stm32wb_system_callback_t       callback;
    void                            *context;
    uint32_t                        mask;
} stm32wb_system_notify_t;

#define STM32WB_SYSTEM_SAICLK_NONE               0
#define STM32WB_SYSTEM_SAICLK_11289600           11289600 /*  44100 * 256 */
#define STM32WB_SYSTEM_SAICLK_24576000           24576000 /*  96000 * 256 */
  
#define STM32WB_SYSTEM_MCO_SOURCE_MASK           0x0000000f
#define STM32WB_SYSTEM_MCO_SOURCE_SHIFT          0
#define STM32WB_SYSTEM_MCO_SOURCE_NONE           0x00000000
#define STM32WB_SYSTEM_MCO_SOURCE_SYSCLK         0x00000001
#define STM32WB_SYSTEM_MCO_SOURCE_MSI            0x00000002
#define STM32WB_SYSTEM_MCO_SOURCE_HSI16          0x00000003
#define STM32WB_SYSTEM_MCO_SOURCE_HSE            0x00000004
#define STM32WB_SYSTEM_MCO_SOURCE_PLL            0x00000005
#define STM32WB_SYSTEM_MCO_SOURCE_LSI            0x00000006
#define STM32WB_SYSTEM_MCO_SOURCE_LSE            0x00000008
#define STM32WB_SYSTEM_MCO_SOURCE_HSI48          0x00000009

#define STM32WB_SYSTEM_MCO_DIVIDE_MASK           0x00000070
#define STM32WB_SYSTEM_MCO_DIVIDE_SHIFT          4
#define STM32WB_SYSTEM_MCO_DIVIDE_BY_1           0x00000000
#define STM32WB_SYSTEM_MCO_DIVIDE_BY_2           0x00000010
#define STM32WB_SYSTEM_MCO_DIVIDE_BY_4           0x00000020
#define STM32WB_SYSTEM_MCO_DIVIDE_BY_8           0x00000030
#define STM32WB_SYSTEM_MCO_DIVIDE_BY_16          0x00000040

#define STM32WB_SYSTEM_LSCO_SOURCE_MASK          0x00000003
#define STM32WB_SYSTEM_LSCO_SOURCE_SHIFT         0
#define STM32WB_SYSTEM_LSCO_SOURCE_NONE          0x00000000
#define STM32WB_SYSTEM_LSCO_SOURCE_LSI           0x00000001 /* LSI1/LSI2 */
#define STM32WB_SYSTEM_LSCO_SOURCE_LSE           0x00000002

#define STM32WB_SYSTEM_RESET_PIN                 0
#define STM32WB_SYSTEM_RESET_POWERON             1
#define STM32WB_SYSTEM_RESET_INTERNAL            2
#define STM32WB_SYSTEM_RESET_SOFTWARE            3
#define STM32WB_SYSTEM_RESET_FIREWALL            4
#define STM32WB_SYSTEM_RESET_WATCHDOG            5
#define STM32WB_SYSTEM_RESET_STANDBY             6
#define STM32WB_SYSTEM_RESET_SHUTDOWN            7
#define STM32WB_SYSTEM_RESET_FATAL               8
#define STM32WB_SYSTEM_RESET_DFU                 9
#define STM32WB_SYSTEM_RESET_OTA                 10


#define STM32WB_SYSTEM_WAKEUP_NONE               0x00000000
#define STM32WB_SYSTEM_WAKEUP_PIN_1              0x00000001
#define STM32WB_SYSTEM_WAKEUP_PIN_2              0x00000002
#define STM32WB_SYSTEM_WAKEUP_PIN_3              0x00000004
#define STM32WB_SYSTEM_WAKEUP_PIN_4              0000000008
#define STM32WB_SYSTEM_WAKEUP_PIN_5              0x00000010
#define STM32WB_SYSTEM_WAKEUP_TIMEOUT            0x00000100
#define STM32WB_SYSTEM_WAKEUP_WATCHDOG           0x00000200
#define STM32WB_SYSTEM_WAKEUP_RESET              0x00000400

#define STM32WB_SYSTEM_POLICY_RUN                1
#define STM32WB_SYSTEM_POLICY_SLEEP              2
#define STM32WB_SYSTEM_POLICY_STOP               3
  
#define STM32WB_SYSTEM_WAKEUP_PIN_RISING_SHIFT   0
#define STM32WB_SYSTEM_WAKEUP_PIN_1_RISING       0x00000001
#define STM32WB_SYSTEM_WAKEUP_PIN_2_RISING       0x00000002
#define STM32WB_SYSTEM_WAKEUP_PIN_3_RISING       0x00000004
#define STM32WB_SYSTEM_WAKEUP_PIN_4_RISING       0x00000008
#define STM32WB_SYSTEM_WAKEUP_PIN_5_RISING       0x00000010
#define STM32WB_SYSTEM_WAKEUP_PIN_FALLING_SHIFT  8
#define STM32WB_SYSTEM_WAKEUP_PIN_1_FALLING      0x00000100
#define STM32WB_SYSTEM_WAKEUP_PIN_2_FALLING      0x00000200
#define STM32WB_SYSTEM_WAKEUP_PIN_3_FALLING      0x00000400
#define STM32WB_SYSTEM_WAKEUP_PIN_4_FALLING      0x00000800
#define STM32WB_SYSTEM_WAKEUP_PIN_5_FALLING      0x00001000
  
extern void     stm32wb_system_initialize(uint32_t sysclk, uint32_t hclk, uint32_t pclk1, uint32_t pclk2, uint32_t lseclk, uint32_t hseclk, uint32_t option);
extern bool     stm32wb_system_sysclk_configure(uint32_t sysclk, uint32_t hclk, uint32_t pclk1, uint32_t pclk2);
extern bool     stm32wb_system_saiclk_configure(uint32_t saiclk);
extern bool     stm32wb_system_mco_configure(uint32_t mco);
extern bool     stm32wb_system_lsco_configure(uint32_t lsco);
extern bool     stm32wb_system_smps_configure(uint32_t palevel);
extern bool     stm32wb_system_cpu2_boot(void);
extern bool     stm32wb_system_wireless_enable(void);
extern bool     stm32wb_system_wireless_disable(void);
extern bool     stm32wb_system_pvm1_enable(stm32wb_system_pvm1_callback_t callback, void *context);
extern bool     stm32wb_system_pvm1_disable(void);
extern bool     stm32wb_system_pvm1_sense(void);
extern bool     stm32wb_system_clk48_enable(void);
extern bool     stm32wb_system_clk48_disable(void);
extern void     stm32wb_system_hsi16_enable(void);
extern void     stm32wb_system_hsi16_disable(void);
extern uint32_t stm32wb_system_reset_cause(void);
extern uint32_t stm32wb_system_wakeup_reason(void);
extern uint32_t stm32wb_system_options(void);
extern uint32_t stm32wb_system_lseclk(void);
extern uint32_t stm32wb_system_hseclk(void);
extern uint32_t stm32wb_system_sysclk(void);
extern uint32_t stm32wb_system_hclk(void);
extern uint32_t stm32wb_system_pclk1(void);
extern uint32_t stm32wb_system_pclk2(void);
extern uint32_t stm32wb_system_saiclk(void);
extern bool     stm32wb_system_cpu2(void);
extern bool     stm32wb_system_wireless(void);
extern uint64_t stm32wb_system_serial(void);
extern void     stm32wb_system_uid(uint32_t *uid);
extern void     stm32wb_system_periph_reset(uint32_t periph);
extern void     stm32wb_system_periph_enable(uint32_t periph);
extern void     stm32wb_system_periph_disable(uint32_t periph);
extern void     stm32wb_system_swd_enable(void);
extern void     stm32wb_system_swd_disable(void);
extern void     stm32wb_system_register(stm32wb_system_notify_t *notify, stm32wb_system_callback_t callback, void *context, uint32_t mask); 
extern void     stm32wb_system_unregister(stm32wb_system_notify_t *notify);
extern void     stm32wb_system_notify(uint32_t notify); 
extern void     stm32wb_system_lock(uint32_t lock); 
extern void     stm32wb_system_unlock(uint32_t lock);
extern void     stm32wb_system_reference(uint32_t reference); 
extern void     stm32wb_system_unreference(uint32_t reference); 
extern void     stm32wb_system_sleep(uint32_t policy);
extern void     stm32wb_system_wakeup(void);
extern void     stm32wb_system_standby(uint32_t wakeup, uint32_t timeout);
extern void     stm32wb_system_shutdown(uint32_t wakeup);
extern void     stm32wb_system_fatal(void) __attribute__((noreturn));
extern void     stm32wb_system_reset(void) __attribute__((noreturn));
extern void     stm32wb_system_dfu(void) __attribute__((noreturn));

#define STM32WB_PVD_PVM_IRQ_PRIORITY ARMV7M_IRQ_PRIORITY_MEDIUM
  
extern void WWDG_IRQHandler(void);
extern void PVD_PVM_IRQHandler(void);
extern void TAMP_STAMP_LSECSS_IRQHandler(void);
extern void RTC_WKUP_IRQHandler(void);
extern void FLASH_IRQHandler(void);
extern void RCC_IRQHandler(void);
extern void EXTI0_IRQHandler(void);
extern void EXTI1_IRQHandler(void);
extern void EXTI2_IRQHandler(void);
extern void EXTI3_IRQHandler(void);
extern void EXTI4_IRQHandler(void);
extern void DMA1_Channel1_IRQHandler(void);
extern void DMA1_Channel2_IRQHandler(void);
extern void DMA1_Channel3_IRQHandler(void);
extern void DMA1_Channel4_IRQHandler(void);
extern void DMA1_Channel5_IRQHandler(void);
extern void DMA1_Channel6_IRQHandler(void);
extern void DMA1_Channel7_IRQHandler(void);
extern void ADC1_IRQHandler(void);
extern void USB_HP_IRQHandler(void);
extern void USB_LP_IRQHandler(void);
extern void C2SEV_PWR_C2H_IRQHandler(void);
extern void COMP_IRQHandler(void);
extern void EXTI9_5_IRQHandler(void);
extern void TIM1_BRK_IRQHandler(void);
extern void TIM1_UP_TIM16_IRQHandler(void);
extern void TIM1_TRG_COM_TIM17_IRQHandler(void);
extern void TIM1_CC_IRQHandler(void);
extern void TIM2_IRQHandler(void);
extern void PKA_IRQHandler(void);
extern void I2C1_EV_IRQHandler(void);
extern void I2C1_ER_IRQHandler(void);
extern void I2C3_EV_IRQHandler(void);
extern void I2C3_ER_IRQHandler(void);
extern void SPI1_IRQHandler(void);
extern void SPI2_IRQHandler(void);
extern void USART1_IRQHandler(void);
extern void LPUART1_IRQHandler(void);
extern void SAI1_IRQHandler(void);
extern void TSC_IRQHandler(void);
extern void EXTI15_10_IRQHandler(void);
extern void RTC_Alarm_IRQHandler(void);
extern void CRS_IRQHandler(void);
extern void PWR_SOTF_BLEACT_802ACT_RFPHASE_IRQHandler(void);
extern void IPCC_C1_RX_IRQHandler(void);
extern void IPCC_C1_TX_IRQHandler(void);
extern void HSEM_IRQHandler(void);
extern void LPTIM1_IRQHandler(void);
extern void LPTIM2_IRQHandler(void);
extern void LCD_IRQHandler(void);
extern void QUADSPI_IRQHandler(void);
extern void AES1_IRQHandler(void);
extern void AES2_IRQHandler(void);
extern void RNG_IRQHandler(void);
extern void FPU_IRQHandler(void);
extern void DMA2_Channel1_IRQHandler(void);
extern void DMA2_Channel2_IRQHandler(void);
extern void DMA2_Channel3_IRQHandler(void);
extern void DMA2_Channel4_IRQHandler(void);
extern void DMA2_Channel5_IRQHandler(void);
extern void DMA2_Channel6_IRQHandler(void);
extern void DMA2_Channel7_IRQHandler(void);
extern void DMAMUX1_OVR_IRQHandler(void);
  
#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_SYSTEM_H */
