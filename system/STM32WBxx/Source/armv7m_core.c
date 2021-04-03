/*
 * Copyright (c) 2014-2021 Thomas Roell.  All rights reserved.
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
#include "stm32wb_system.h"

typedef struct _armv7m_core_control_t {
    uint32_t                         udelay_scale;
    armv7m_core_fault_callback_t     fault_callback;
    volatile uint32_t                hardfault_address;
    const void * volatile            hardfault_epilogue;
} armv7m_core_control_t;

static __attribute__((section(".noinit"))) armv7m_core_control_t armv7m_core_control;

static __attribute__((section(".noinit2a"), used)) armv7m_core_info_t armv7m_core_info;
static __attribute__((section(".noinit2a"), used)) armv7m_core_state_t armv7m_core_state;

static void armv7m_core_fatal_callback(armv7m_core_state_t *state);

void __armv7m_core_initialize(void)
{
    armv7m_core_control.udelay_scale = SystemCoreClock / 15625;
    armv7m_core_control.fault_callback = armv7m_core_fatal_callback;
    armv7m_core_control.hardfault_address = 0;
    armv7m_core_control.hardfault_epilogue = NULL;

    NVIC_SetPriority(MemManage_IRQn, ARMV7M_IRQ_PRIORITY_MEMMANAGE);
    NVIC_SetPriority(BusFault_IRQn, ARMV7M_IRQ_PRIORITY_BUSFAULT);
    NVIC_SetPriority(UsageFault_IRQn, ARMV7M_IRQ_PRIORITY_USAGEFAULT);
    NVIC_SetPriority(DebugMon_IRQn, ARMV7M_IRQ_PRIORITY_DEBUGMON);

    SCB->CCR = 0;
    SCB->SHCSR = (SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk);
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    SCB->CPACR = 0x00f00000;
#endif /* __VFP_FP__ && !__SOFTFP__ */

    __DSB();
    
    __armv7m_pendsv_initialize();
    __armv7m_svcall_initialize();
    __armv7m_systick_initialize();
    __armv7m_rtos_initialize();
}

void armv7m_core_configure(void)
{
    armv7m_core_control.udelay_scale = SystemCoreClock / 15625;
}

void armv7m_core_udelay(uint32_t delay)
{
    uint32_t n;

    n = (delay * armv7m_core_control.udelay_scale + 255) / 256;

    __asm__ __volatile__(
                         "1: subs %0, #1 \n"
                         "   nop         \n"
                         "   bne  1b     \n"
                         : "+r" (n));
}

void armv7m_core_fault_callback(armv7m_core_fault_callback_t callback)
{
    armv7m_core_control.fault_callback = callback ? callback : armv7m_core_fatal_callback;
}

void armv7m_core_hardfault_try(const void *epilogue)
{
    armv7m_core_control.hardfault_address = 0;
    armv7m_core_control.hardfault_epilogue = epilogue;
}

uint32_t armv7m_core_hardfault_catch(void)
{
    armv7m_core_control.hardfault_epilogue = NULL;

    return armv7m_core_control.hardfault_address;
}

/* The format of a member pointer is defined in the C++ ABI.
 * For ARM that document is IHI0041D, which describes the differences
 * to the Itanium C++ ABI.
 */

void armv7m_core_cxx_method(const void *method, const void *object, armv7m_core_callback_t *p_callback_return, void **p_context_return)
{
    void *ptr = (void*)(((const uint32_t*)method)[0]);
    ptrdiff_t adj = ((ptrdiff_t)(((const uint32_t*)method)[1]) >> 1);

    if (!((((const uint32_t*)method)[1]) & 1))
    {
        /* non-virtual function */
        *p_callback_return = ptr;
    }
    else
    {
        /* virtual function */
        void *vptr = *((void**)((uintptr_t)object + adj)); 
        *p_callback_return = *((void**)((uint8_t*)vptr + (ptrdiff_t)ptr));
    }

    *p_context_return = (void*)((uintptr_t)object + adj);
}

void __attribute__((noreturn)) armv7m_core_fatal(uint32_t code, const char *file, uint32_t line, const armv7m_core_state_t *state)
{
    armv7m_core_info.code = code;
    armv7m_core_info.file = file;
    armv7m_core_info.line = line;
    armv7m_core_info.state = state;

    stm32wb_system_fatal(&armv7m_core_info);
}

static void __attribute__((noreturn)) armv7m_core_fatal_callback(armv7m_core_state_t *state)
{
    armv7m_core_fatal(__get_IPSR(), NULL, 0, state);
}

static __attribute__((naked, used)) void armv7m_core_fault(void)
{
    __asm__(
        "    mrs      r0, PRIMASK                              \n"
        "    cpsid    i                                        \n"
        "    movw     r3, #:lower16:armv7m_core_state          \n"
        "    movt     r3, #:upper16:armv7m_core_state          \n"
        "    strb     r0, [r3, %[offset_PRIMASK]]              \n"
        "    mrs      r0, CONTROL                              \n"
        "    tst      lr, #0x00000004                          \n" // LR bit 2 is SPSEL
        "    ite      ne                                       \n"
        "    mrsne    r2, PSP                                  \n"
        "    moveq    r2, sp                                   \n"
        "    add      r1, r3, %[offset_R4]                     \n"
        "    stmia    r1, { r4-r11 }                           \n"
        "    ldmia    r2!, { r4-r11 }                          \n" // r4-r11 = { r0, r1, r2, r3, r12, lr, pc, xpsr }
        "    str      r4, [r3, %[offset_R0]]                   \n"
        "    str      r5, [r3, %[offset_R1]]                   \n"
        "    str      r6, [r3, %[offset_R2]]                   \n"
        "    str      r7, [r3, %[offset_R3]]                   \n"
        "    str      r8, [r3, %[offset_R12]]                  \n"
        "    str      r9, [r3, %[offset_LR]]                   \n"
        "    str      r10, [r3, %[offset_PC]]                  \n"
        "    str      r11, [r3, %[offset_XPSR]]                \n"
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
        "    tst      lr, #0x00000010                          \n" // LR bit 4 is nFPCA
        "    bne.n    1f                                       \n"
        "    add      r1, r3, %[offset_D8]                     \n"
        "    vstmia   r1, { d8-d15 }                           \n"
        "    add      r1, r3, %[offset_D0]                     \n"
        "    vldmia   r2!, { d0-d7 }                           \n"
        "    vstmia   r1, { d0-d7 }                            \n"
        "    ldmia    r2!, { r4-r5 }                           \n" // r4-r5 = { fpscr, reserved }
        "    str      r4, [r3, %[offset_FPSCR]]                \n"
        "    msr      CONTROL, r0                              \n"
        "    orr      r0, #0x00000004                          \n"
#endif /* __VFP_FP__ && !__SOFTFP__ */
        "1:  and      r1, r11, #0x00000200                     \n" // drop aligner if needed
        "    add      r2, r2, r1, ROR #7                       \n"
        "    str      r2, [r3, %[offset_SP]]                   \n"
        "    tst      lr, #0x00000004                          \n" // LR bit 2 is SPSEL
        "    itte     ne                                       \n"
        "    msrne    PSP, r2                                  \n"
        "    orrne    r0, #0x00000002                          \n"
        "    moveq    sp, r2                                   \n"
        "    strb     r0, [r3, %[offset_CONTROL]]              \n"
        "    mrs      r0, MSP                                  \n"
        "    str      r0, [r3, %[offset_MSP]]                  \n"
        "    mrs      r0, PSP                                  \n"
        "    str      r0, [r3, %[offset_PSP]]                  \n"
        "    mrs      r0, BASEPRI                              \n"
        "    strb     r0, [r3, %[offset_BASEPRI]]              \n"
        "    mrs      r0, FAULTMASK                            \n"
        "    strb     r0, [r3, %[offset_FAULTMASK]]            \n"

        "    mov      r0, r3                                   \n"
        "    movw     r3, #:lower16:armv7m_core_control        \n"
        "    movt     r3, #:upper16:armv7m_core_control        \n"
        "    ldr      r1, [r3, %[offset_FAULT_CALLBACK]]       \n"      
        "    blx      r1                                       \n"

        "    movw     r3, #:lower16:armv7m_core_state          \n"
        "    movt     r3, #:upper16:armv7m_core_state          \n"
        "    mov      lr, 0xfffffff9                           \n" // reconstruct LR
        "    ldrb     r0, [r3, %[offset_CONTROL]]              \n"
        "    bic      r1, r0, #0x00000006                      \n"
        "    msr      CONTROL, r1                              \n"
        "    and      r1, r0, #0x00000002                      \n"
        "    orr      lr, lr, r1, LSL #1                       \n" // LR bit 2 is SPSEL
        "    and      r1, r0, #0x00000004                      \n"
        "    bic      lr, lr, r1, LSL #2                       \n" // LR bit 4 is nFPCA
        "    ldr      r11, [r3, %[offset_XPSR]]                \n"
        "    lsl      r1, r11, #23                             \n"
        "    it       eq                                       \n"
        "    biceq    lr, 0x000000008                          \n"
        "    ldr      r2, [r3, %[offset_SP]]                   \n" // reconstruct SP
        "    and      r1, r2, #0x00000004                      \n" // insert aligner if needed
        "    sub      r2, r1                                   \n"
        "    orr      r11, r11, r1, LSL #7                     \n"
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
        "    tst      lr, #0x00000010                          \n" // LR bit 4 is nFPCA
        "    bne.n    2f                                       \n"
        "    ldr      r4, [r3, %[offset_FPSCR]]                \n"
        "    mov      r5, #0                                   \n"
        "    stmdb    r2!, { r4-r5 }                           \n" // r4-r5 = { fpscr, reserved }
        "    add      r1, r3, %[offset_D0]                     \n"
        "    vldmia   r1, { d0-d7 }                            \n"
        "    vstmdb   r2!, { d0-d7 }                           \n"
        "    add      r1, r3, %[offset_D8]                     \n"
        "    vldmia   r1, { d8-d15 }                           \n"
#endif /* __VFP_FP__ && !__SOFTFP__ */
        "2:  ldr      r4, [r1, %[offset_R0]]                   \n"
        "    ldr      r5, [r1, %[offset_R1]]                   \n"
        "    ldr      r6, [r1, %[offset_R2]]                   \n"
        "    ldr      r7, [r1, %[offset_R3]]                   \n"
        "    ldr      r8, [r1, %[offset_R12]]                  \n"
        "    ldr      r9, [r1, %[offset_LR]]                   \n"
        "    ldr      r10, [r1, %[offset_PC]]                  \n"
        "    stmdb    r2!, { r4-r11 }                          \n"
        "    add      r1, r3, %[offset_R4]                     \n"
        "    ldmia    r1, { r4-r11 }                           \n"
        "    tst      lr, #0x00000004                          \n" // LR bit 2 is SPSEL
        "    itt      eq                                       \n"
        "    moveq    sp, r2                                   \n"
        "    ldreq    r2, [r3, %[offset_PSP]]                  \n"
        "    msr      PSP, r2                                  \n"
        "    ldrb     r0, [r3, %[offset_BASEPRI]]              \n"
        "    msr      BASEPRI, r0                              \n"
        "    ldrb     r0, [r3, %[offset_FAULTMASK]]            \n"
        "    msr      FAULTMASK, r0                            \n"
        "    ldrb     r0, [r3, %[offset_PRIMASK]]              \n"
        "    msr      PRIMASK, r0                              \n"
        "    bx       lr                                       \n"

        :
        : [offset_R0]             "I" (offsetof(armv7m_core_state_t, r0)),
          [offset_R1]             "I" (offsetof(armv7m_core_state_t, r1)),
          [offset_R2]             "I" (offsetof(armv7m_core_state_t, r2)),
          [offset_R3]             "I" (offsetof(armv7m_core_state_t, r3)),
          [offset_R4]             "I" (offsetof(armv7m_core_state_t, r4)),
          [offset_R12]            "I" (offsetof(armv7m_core_state_t, r12)),
          [offset_SP]             "I" (offsetof(armv7m_core_state_t, sp)),
          [offset_LR]             "I" (offsetof(armv7m_core_state_t, lr)),        
          [offset_PC]             "I" (offsetof(armv7m_core_state_t, pc)),
          [offset_XPSR]           "I" (offsetof(armv7m_core_state_t, xpsr)),
          [offset_MSP]            "I" (offsetof(armv7m_core_state_t, msp)),
          [offset_PSP]            "I" (offsetof(armv7m_core_state_t, psp)),
          [offset_PRIMASK]        "I" (offsetof(armv7m_core_state_t, primask)),
          [offset_BASEPRI]        "I" (offsetof(armv7m_core_state_t, basepri)),
          [offset_FAULTMASK]      "I" (offsetof(armv7m_core_state_t, faultmask)),
          [offset_CONTROL]        "I" (offsetof(armv7m_core_state_t, control)),
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
          [offset_D0]             "I" (offsetof(armv7m_core_state_t, d0)),
          [offset_D8]             "I" (offsetof(armv7m_core_state_t, d8)),
          [offset_FPSCR]          "I" (offsetof(armv7m_core_state_t, fpscr)),
#endif /* __VFP_FP__ && !__SOFTFP__ */
          [offset_FAULT_CALLBACK] "I" (offsetof(armv7m_core_control_t, fault_callback))
        );
}

void MemManage_Handler(void) __attribute__ ((alias("armv7m_core_fault")));
void BusFault_Handler(void) __attribute__ ((alias("armv7m_core_fault")));
void UsageFault_Handler(void) __attribute__ ((alias("armv7m_core_fault")));
void DebugMon_Handler(void) __attribute__ ((alias("armv7m_core_fault")));

void HardFault_Handler(void)
{
    __asm__(
        "    movw     r3, #:lower16:armv7m_core_control        \n"
        "    movt     r3, #:upper16:armv7m_core_control        \n"
        "    ldr      r0, [r3, %[offset_HARDFAULT_EPILOGUE]]   \n"
        "    cbnz.n   r0, 1f                                   \n"
        "    b        armv7m_core_fault                        \n"
        "1:  tst      lr, #0x00000004                          \n" // LR bit 2 is SPSEL
        "    ite      ne                                       \n"
        "    mrsne    r2, PSP                                  \n"
        "    moveq    r2, sp                                   \n"
        "    ldr      r1, [r2, #0x18]                          \n"
        "    bic      r0, r0, #1                               \n"
        "    str      r0, [r2, #0x18]                          \n"
        "    str      r1, [r3, %[offset_HARDFAULT_ADDRESS]]    \n"
        "    mov      r0, #0                                   \n"
        "    str      r0, [r3, %[offset_HARDFAULT_EPILOGUE]]   \n"
        "    bx       lr                                       \n"
        :
        : [offset_HARDFAULT_ADDRESS]  "I" (offsetof(armv7m_core_control_t, hardfault_address)),
          [offset_HARDFAULT_EPILOGUE] "I" (offsetof(armv7m_core_control_t, hardfault_epilogue))
        );
}

void NMI_Handler(void) __attribute__ ((weak, alias("armv7m_core_fault")));
        
