/*
 * Copyright (c) 2017-2021 Thomas Roell.  All rights reserved.
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

#if !defined(_ARMV7M_CORE_H)
#define _ARMV7M_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#define ARMV7M_IRQ_PRIORITY_FAULT      0  // MemManage, BusFault, UsageFault
#define ARMV7M_IRQ_PRIORITY_CRITICAL   1  // EXTI with time stamping, DebugMon
#define ARMV7M_IRQ_PRIORITY_REALTIME   2  // SYSTICK, SERVO, TONE, short but low jitter
#define ARMV7M_IRQ_PRIORITY_HIGH       3  // UART
#define ARMV7M_IRQ_PRIORITY_SAI        4  // SAI
#define ARMV7M_IRQ_PRIORITY_USB        5  // USB
#define ARMV7M_IRQ_PRIORITY_MEDIUM     6  // HSEM, IPCC, I2C, SPI, RTC, LPTIM, RANDOM, QSPI
#define ARMV7M_IRQ_PRIORITY_LOW        7  // SVCALL, PENDSV

#define ARMV7M_IRQ_PRIORITY_UART       ARMV7M_IRQ_PRIORITY_HIGH
#define ARMV7M_IRQ_PRIORITY_HSEM       ARMV7M_IRQ_PRIORITY_MEDIUM
#define ARMV7M_IRQ_PRIORITY_IPCC       ARMV7M_IRQ_PRIORITY_MEDIUM
#define ARMV7M_IRQ_PRIORITY_I2C        ARMV7M_IRQ_PRIORITY_MEDIUM
#define ARMV7M_IRQ_PRIORITY_SPI        ARMV7M_IRQ_PRIORITY_MEDIUM
#define ARMV7M_IRQ_PRIORITY_RTC        ARMV7M_IRQ_PRIORITY_MEDIUM
#define ARMV7M_IRQ_PRIORITY_LPTIM      ARMV7M_IRQ_PRIORITY_MEDIUM
#define ARMV7M_IRQ_PRIORITY_RANDOM     ARMV7M_IRQ_PRIORITY_MEDIUM
  
#define ARMV7M_IRQ_PRIORITY_MEMMANAGE  ARMV7M_IRQ_PRIORITY_FAULT
#define ARMV7M_IRQ_PRIORITY_BUSFAULT   ARMV7M_IRQ_PRIORITY_FAULT
#define ARMV7M_IRQ_PRIORITY_USAGEFAULT ARMV7M_IRQ_PRIORITY_FAULT
#define ARMV7M_IRQ_PRIORITY_DEBUGMON   ARMV7M_IRQ_PRIORITY_FAULT
#define ARMV7M_IRQ_PRIORITY_SYSTICK    ARMV7M_IRQ_PRIORITY_REALTIME
#define ARMV7M_IRQ_PRIORITY_PENDSV     ARMV7M_IRQ_PRIORITY_LOW
#define ARMV7M_IRQ_PRIORITY_SVCALL     ARMV7M_IRQ_PRIORITY_LOW

#define Reset_IRQn        ((IRQn_Type)-16)
#define NMI_IRQn          ((IRQn_Type)-14)
#define HardFault_IRQn    ((IRQn_Type)-13)
#define MemManage_IRQn    ((IRQn_Type)-12)
#define BusFault_IRQn     ((IRQn_Type)-11)
#define UsageFault_IRQn   ((IRQn_Type)-10)
#define SVCall_IRQn       ((IRQn_Type)-5)
#define DebugMon_IRQn     ((IRQn_Type)-4)
#define PendSV_IRQn       ((IRQn_Type)-2)
#define SysTick_IRQn      ((IRQn_Type)-1)

typedef enum {
    ThreadMode_EXCn             = 0,
    NMI_EXCn                    = 2,      /*!< 2 Cortex-M4 NMI Exception                              */
    HardFault_EXCn              = 3,      /*!< 3 Cortex-M4 Hard Fault Exception                       */
    MemManage_EXCn              = 4,      /*!< 4 Cortex-M4 Memory Management Exception                */
    BusFault_EXCn               = 5,      /*!< 5 Cortex-M4 Bus Fault Exception                        */
    UsageFault_EXCn             = 6,      /*!< 6 Cortex-M4 Usage Fault Exception                      */
    SVCall_EXCn                 = 11,     /*!< 11 Cortex-M4 SV Call Exception                         */
    DebugMon_EXCn               = 12,     /*!< 12 Cortex-M4 Debug Monitor Exception                   */
    PendSV_EXCn                 = 14,     /*!< 14 Cortex-M4 Pend SV Exception                         */
    SysTick_EXCn                = 15,     /*!< 15 Cortex-M4 System Tick Exception                     */
    IRQ0_EXCn                   = 16,
} EXCn_Type;
    
#define ARMV7M_BITBAND_SRAM_ADDRESS(_address,_index) \
  ((volatile uint32_t*)(0x22000000 + ((uint32_t)(_address) - 0x20000000) * 32 + ((_index) * 4)))

#define ARMV7M_BITBAND_PERIPHERAL_ADDRESS(_address,_index) \
  ((volatile uint32_t*)(0x42000000 + ((uint32_t)(_address) - 0x40000000) * 32 + ((_index) * 4)))
  
typedef void (*armv7m_core_routine_t)(void *context);

static inline IRQn_Type __current_irq(void)
{
    return (IRQn_Type)(__get_IPSR() - 16);
}
  
extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void MemManage_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);
extern void SVC_Handler(void);
extern void DebugMon_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

typedef struct _armv7m_exception_t {
    uint32_t            r0;
    uint32_t            r1;
    uint32_t            r2;
    uint32_t            r3;
    uint32_t            r12;
    uint32_t            lr;
    uint32_t            pc;
    uint32_t            xpsr;
} armv7m_exception_t;

#if defined (__VFP_FP__) && !defined(__SOFTFP__)

typedef struct _armv7m_exception_fpu_t {
    uint32_t            r0;
    uint32_t            r1;
    uint32_t            r2;
    uint32_t            r3;
    uint32_t            r12;
    uint32_t            lr;
    uint32_t            pc;
    uint32_t            xpsr;
    uint64_t            d0;
    uint64_t            d1;
    uint64_t            d2;
    uint64_t            d3;
    uint64_t            d4;
    uint64_t            d5;
    uint64_t            d6;
    uint64_t            d7;
    uint32_t            fpscr;
    uint32_t            reserved;
} armv7m_exception_fpu_t;

#endif /* __VFP_FP__ && !__SOFTFP__ */
  
typedef struct _armv7m_core_state_t {
    uint32_t            r0;
    uint32_t            r1;
    uint32_t            r2;
    uint32_t            r3;
    uint32_t            r4;
    uint32_t            r5;
    uint32_t            r6;
    uint32_t            r7;
    uint32_t            r8;
    uint32_t            r9;
    uint32_t            r10;
    uint32_t            r11;
    uint32_t            r12;
    uint32_t            lr;
    uint32_t            pc;
    uint32_t            xpsr;
    uint32_t            msp;
    uint32_t            psp;
    uint32_t            primask;
    uint32_t            basepri;
    uint32_t            faultmask;
    uint32_t            control;
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    uint64_t            d0;
    uint64_t            d1;
    uint64_t            d2;
    uint64_t            d3;
    uint64_t            d4;
    uint64_t            d5;
    uint64_t            d6;
    uint64_t            d7;
    uint64_t            d8;
    uint64_t            d9;
    uint64_t            d10;
    uint64_t            d11;
    uint64_t            d12;
    uint64_t            d13;
    uint64_t            d14;
    uint64_t            d15;
    uint32_t            fpscr;
#endif /* __VFP_FP__ && !__SOFTFP__ */
} armv7m_core_state_t;

typedef struct _armv7m_core_info_t {
    uint32_t                  code;
    const char                *file;
    uint32_t                  line;
    const armv7m_core_state_t *state;
} armv7m_core_info_t;
    
static inline bool armv7m_interrupt_is_in_progress(void)
{
    return (__get_IPSR() != ThreadMode_EXCn);
}
  
static inline bool armv7m_core_is_in_thread(void)
{
    uint32_t ipsr = __get_IPSR();
    
    return (ipsr == ThreadMode_EXCn);
}

static inline bool armv7m_core_is_in_pendsv(void)
{
    uint32_t ipsr = __get_IPSR();

    return (ipsr == PendSV_EXCn);
}

static inline bool armv7m_core_is_in_svcall(void)
{
    uint32_t ipsr = __get_IPSR();

    return (ipsr == SVCall_EXCn);
}

static inline bool armv7m_core_is_in_pendsv_or_svcall(void)
{
    uint32_t ipsr = __get_IPSR();

    return ((ipsr == SVCall_EXCn) || (ipsr == PendSV_EXCn));
}

extern void __armv7m_core_initialize(void);
extern void armv7m_core_system_clock(uint32_t clock);
extern void armv7m_core_udelay(uint32_t udelay);

typedef void (*armv7m_core_callback_t)(void *context);  

extern void armv7m_core_cxx_method(const void *method, const void *object, armv7m_core_callback_t *p_callback_return, void **p_context_return);

extern void armv7m_core_fatal(uint32_t code, const char *file, uint32_t line, const armv7m_core_state_t *state) __attribute__((noreturn));
    
#ifdef __cplusplus
}
#endif

#endif /* _ARMV7M_CORE_H */
