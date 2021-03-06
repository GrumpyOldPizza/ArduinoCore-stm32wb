/*
 * Copyright (c) 2017-2020 Thomas Roell.  All rights reserved.
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

#if !defined(_ARMV7M_ATOMIC_H)
#define _ARMV7M_ATOMIC_H

#ifdef __cplusplus
extern "C" {
#endif

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_add(volatile uint32_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   add     %1, %0, %4    \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}


static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_sub(volatile uint32_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   sub     %1, %0, %4    \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_and(volatile uint32_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   and     %1, %0, %4    \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_or(volatile uint32_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   orr     %1, %0, %4    \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_xor(volatile uint32_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   eor     %1, %0, %4    \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_inc(volatile uint32_t *p_data, uint32_t data_limit)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   cmp     %0, %4        \n"
        "   ite     eq            \n"
        "   moveq   %1, %0        \n"
        "   addne   %1, %0, #1    \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data_limit)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_dec(volatile uint32_t *p_data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   cmp     %0, #0        \n"
        "   ite     eq            \n"
        "   moveq   %1, %0        \n"
        "   subne   %1, %0, #1    \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data)
	: "memory"
	);

    return data_return;
}


static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_swap(volatile uint32_t *p_data, uint32_t data)
{
    uint32_t data_return, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "+&r" (data), "=&r" (success)
	: "r" (_p_data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_cas(volatile uint32_t *p_data, uint32_t data_expected, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   cmp     %0, %4        \n"
        "   ite     ne            \n"
        "   movne   %1, %0        \n"
        "   moveq   %1, %5        \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	"2:                       \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data_expected), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_addh(volatile uint16_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint16_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexh  %0, [%3]      \n"
        "   add     %1, %0, %4    \n"
        "   strexh  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}


static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_subh(volatile uint16_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint16_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexh  %0, [%3]      \n"
        "   sub     %1, %0, %4    \n"
        "   strexh  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_andh(volatile uint16_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint16_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexh  %0, [%3]      \n"
        "   and     %1, %0, %4    \n"
        "   strexh  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_orh(volatile uint16_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint16_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexh  %0, [%3]      \n"
        "   orr     %1, %0, %4    \n"
        "   strexh  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_xorh(volatile uint16_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint16_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexh  %0, [%3]      \n"
        "   eor     %1, %0, %4    \n"
        "   strexh  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_inch(volatile uint16_t *p_data, uint32_t data_limit)
{
    uint32_t data_return, data_temp, success;
    register volatile uint16_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexh  %0, [%3]      \n"
        "   cmp     %0, %4        \n"
        "   ite     eq            \n"
        "   moveq   %1, %0        \n"
        "   addne   %1, %0, #1    \n"
        "   strexh  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data_limit)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_dech(volatile uint16_t *p_data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint16_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexh  %0, [%3]      \n"
        "   cmp     %0, #0        \n"
        "   it      eq            \n"
        "   moveq   %1, %0        \n"
        "   subne   %1, %0, #1    \n"
        "   strexh  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data)
	: "memory"
	);

    return data_return;
}


static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_swaph(volatile uint16_t *p_data, uint32_t data)
{
    uint32_t data_return, success;
    register volatile uint16_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexh  %0, [%3]      \n"
        "   strexh  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "+&r" (data), "=&r" (success)
	: "r" (_p_data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_cash(volatile uint16_t *p_data, uint32_t data_expected, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint16_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexh  %0, [%3]      \n"
        "   cmp     %0, %4        \n"
        "   ite     ne            \n"
        "   movne   %1, %0        \n"
        "   moveq   %1, %5        \n"
        "   strexh  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	"2:                       \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data_expected), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_addb(volatile uint8_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint8_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexb  %0, [%3]      \n"
        "   add     %1, %0, %4    \n"
        "   strexb  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}


static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_subb(volatile uint8_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint8_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexb  %0, [%3]      \n"
        "   sub     %1, %0, %4    \n"
        "   strexb  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_andb(volatile uint8_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint8_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexb  %0, [%3]      \n"
        "   and     %1, %0, %4    \n"
        "   strexb  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_orb(volatile uint8_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint8_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexb  %0, [%3]      \n"
        "   orr     %1, %0, %4    \n"
        "   strexb  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_xorb(volatile uint8_t *p_data, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint8_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexb  %0, [%3]      \n"
        "   eor     %1, %0, %4    \n"
        "   strexb  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_incb(volatile uint8_t *p_data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint8_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexb  %0, [%3]      \n"
        "   cmp     %0, #255      \n"
        "   ite     eq            \n"
        "   moveq   %1, %0        \n"
        "   addne   %1, %0, #1    \n"
        "   strexb  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_decb(volatile uint8_t *p_data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint8_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexb  %0, [%3]      \n"
        "   cmp     %0, #0        \n"
        "   ite     eq            \n"
        "   moveq   %1, %0        \n"
        "   subne   %1, %0, #1    \n"
        "   strexb  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data)
	: "memory"
	);

    return data_return;
}


static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_swapb(volatile uint8_t *p_data, uint32_t data)
{
    uint32_t data_return, success;
    register volatile uint8_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexb  %0, [%3]      \n"
        "   strexb  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "+&r" (data), "=&r" (success)
	: "r" (_p_data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_casb(volatile uint8_t *p_data, uint32_t data_expected, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint8_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrexb  %0, [%3]      \n"
        "   cmp     %0, %4        \n"
        "   ite     ne            \n"
        "   movne   %1, %0        \n"
        "   moveq   %1, %5        \n"
        "   strexb  %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	"2:                       \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data_expected), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_modify(volatile uint32_t *p_data, uint32_t mask, uint32_t data)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   bic     %1, %0, %4    \n"
        "   eor     %1, %1, %5    \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (mask), "r" (data)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_modifyz(volatile uint32_t *p_data, uint32_t mask, uint32_t data, volatile uint32_t *p_zero, uint32_t bits)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   ldr     %1, [%6]      \n"
        "   ands    %1, %7        \n"
        "   itte    eq            \n"
        "   biceq   %1, %0, %4    \n"
        "   eoreq   %1, %1, %5    \n"
        "   movne   %1, %0        \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (mask), "r" (data), "r" (p_zero), "r" (bits)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_modifyzb(volatile uint32_t *p_data, uint32_t mask, uint32_t data, volatile uint8_t *p_zero)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   ldrb    %1, [%6]      \n"
        "   cmp     %1, #0        \n"
        "   itte    eq            \n"
        "   biceq   %1, %0, %4    \n"
        "   eoreq   %1, %1, %5    \n"
        "   movne   %1, %0        \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (mask), "r" (data), "r" (p_zero)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_andz(volatile uint32_t *p_data, uint32_t data, volatile uint32_t *p_zero, uint32_t bits)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   ldr     %1, [%5]      \n"
        "   ands    %1, %6        \n"
        "   ite     eq            \n"
        "   andeq   %1, %0, %4    \n"
        "   movne   %1, %0        \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data), "r" (p_zero), "r" (bits)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_andzb(volatile uint32_t *p_data, uint32_t data, volatile uint8_t *p_zero)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   ldrb    %1, [%5]      \n"
        "   cmp     %1, #0        \n"
        "   ite     eq            \n"
        "   andeq   %1, %0, %4    \n"
        "   movne   %1, %0        \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data), "r" (p_zero)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_andzh(volatile uint32_t *p_data, uint32_t data, volatile uint16_t *p_zero, uint32_t bits)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   ldrh    %1, [%5]      \n"
        "   ands    %1, %6        \n"
        "   ite     eq            \n"
        "   andeq   %1, %0, %4    \n"
        "   movne   %1, %0        \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data), "r" (p_zero), "r" (bits)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_orz(volatile uint32_t *p_data, uint32_t data, volatile uint32_t *p_zero, uint32_t bits)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   ldr     %1, [%5]      \n"
        "   ands    %1, %6        \n"
        "   ite     eq            \n"
        "   orreq   %1, %0, %4    \n"
        "   movne   %1, %0        \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data), "r" (p_zero), "r" (bits)
	: "memory"
	);

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv7m_atomic_orzb(volatile uint32_t *p_data, uint32_t data, volatile uint8_t *p_zero)
{
    uint32_t data_return, data_temp, success;
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "1: ldrex   %0, [%3]      \n"
        "   ldrb    %1, [%5]      \n"
        "   cmp     %1, #0        \n"
        "   ite     eq            \n"
        "   orreq   %1, %0, %4    \n"
        "   movne   %1, %0        \n"
        "   strex   %2, %1, [%3]  \n"
	"   cmp     %2, #0        \n"
	"   bne.n   1b            \n"
	: "=&r" (data_return), "=&r" (data_temp), "=&r" (success)
	: "r" (_p_data), "r" (data), "r" (p_zero)
	: "memory"
	);

    return data_return;
}

extern uint32_t armv7m_atomic_add(volatile uint32_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_sub(volatile uint32_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_and(volatile uint32_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_or(volatile uint32_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_xor(volatile uint32_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_inc(volatile uint32_t *p_data, uint32_t data_limit);
extern uint32_t armv7m_atomic_dec(volatile uint32_t *p_data);
extern uint32_t armv7m_atomic_swap(volatile uint32_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_cas(volatile uint32_t *p_data, uint32_t data_expected, uint32_t data);

extern uint32_t armv7m_atomic_addh(volatile uint16_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_subh(volatile uint16_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_andh(volatile uint16_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_orh(volatile uint16_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_swaph(volatile uint16_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_cash(volatile uint16_t *p_data, uint32_t data_expected, uint32_t data);
extern uint32_t armv7m_atomic_addb(volatile uint8_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_subb(volatile uint8_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_andb(volatile uint8_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_orb(volatile uint8_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_decb(volatile uint8_t *p_data);
extern uint32_t armv7m_atomic_incb(volatile uint8_t *p_data);
extern uint32_t armv7m_atomic_swapb(volatile uint8_t *p_data, uint32_t data);
extern uint32_t armv7m_atomic_casb(volatile uint8_t *p_data, uint32_t data_expected, uint32_t data);

/* *p_data = (*p_data & ~mask) ^ data */
extern uint32_t armv7m_atomic_modify(volatile uint32_t *p_data, uint32_t mask, uint32_t data);

/* The nested conditional updates below work, because ARMV7M has a single address range monitor.
 * Hence if either *p_data or *p_zero get changed the CPU will fail the update to *p_data
 * and force a retry.
 */

/* *p_data = (*p_zero == 0) ? ((*p_data & ~mask) ^ data) : *p_data */
extern uint32_t armv7m_atomic_modifyz(volatile uint32_t *p_data, uint32_t mask, uint32_t data, volatile uint32_t *p_zero, uint32_t bits);
extern uint32_t armv7m_atomic_modifyzb(volatile uint32_t *p_data, uint32_t mask, uint32_t data, volatile uint8_t *p_zero);

/* *p_data = (*p_zero == 0) ? (*p_data & mask) : *p_data */
extern uint32_t armv7m_atomic_andz(volatile uint32_t *p_data, uint32_t data, volatile uint32_t *p_zero, uint32_t bits);
extern uint32_t armv7m_atomic_andzb(volatile uint32_t *p_data, uint32_t data, volatile uint8_t *p_zero);

/* *p_data = (*p_zero == 0) ? (*p_data | mask) : *p_data */
extern uint32_t armv7m_atomic_orz(volatile uint32_t *p_data, uint32_t data, volatile uint32_t *p_zero, uint32_t bits);
extern uint32_t armv7m_atomic_orzb(volatile uint32_t *p_data, uint32_t data, volatile uint8_t *p_zero);

/* *p_data = data
 * 
 * This is more involved as one might guess. There is only one exclusive monitor. Hence if
 * a data item is stored in a location that is also used by LDREX/STREX primitives, the
 * exclusive monitor has to be cleared, so that LDREX/STREX see the out of band update.
 * Assembly code had been picked so that the matching CLREX direct preceeds the store.
 */

static inline __attribute__((optimize("O3"),always_inline)) void armv7m_atomic_store(volatile uint32_t *p_data, uint32_t data)
{
    register volatile uint32_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "   clrex                 \n"
        "   str     %1, [%0]      \n"
	:
	: "r" (_p_data), "r" (data)
	: "memory"
	);
}

static inline __attribute__((optimize("O3"),always_inline)) void armv7m_atomic_storeb(volatile uint8_t *p_data, uint8_t data)
{
    register volatile uint8_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "   clrex                 \n"
        "   strb    %1, [%0]      \n"
	:
	: "r" (_p_data), "r" (data)
	: "memory"
	);
}

static inline __attribute__((optimize("O3"),always_inline)) void armv7m_atomic_storeh(volatile uint16_t *p_data, uint16_t data)
{
    register volatile uint16_t *_p_data __asm__("r12") = p_data;

    __asm__ volatile (
        "   clrex                 \n"
        "   strh    %1, [%0]      \n"
	:
	: "r" (_p_data), "r" (data)
	: "memory"
	);
}

static inline __attribute__((optimize("O3"),always_inline)) void armv7m_atomic_store_2(volatile uint32_t *p_data, uint32_t data_0, uint32_t data_1)
{
    uint32_t _primask;
    
    __asm__ volatile (
        "   mrs     %0, primask   \n"
	"   cpsid   i             \n"
        "   clrex                 \n"
        "   strd    %2, %3, [%1]  \n"
        "   msr     primask, %0   \n"
	: "=&r" (_primask)
	: "r" (p_data), "r" (data_0), "r" (data_1)
	: "memory"
	);
}

static inline __attribute__((optimize("O3"),always_inline)) void armv7m_atomic_store_2_restart(volatile uint32_t *p_data, uint32_t data_0, uint32_t data_1)
{
    register uint32_t _a0 __asm__("r0") = data_0;
    register uint32_t _a1 __asm__("r1") = data_1;
    
    __asm__ volatile ("stm %0!, { %1, %2 }"
                      : "+&l" (p_data)
                      : "r" (_a0), "r" (_a1)
                      : "memory");
}

static inline __attribute__((optimize("O3"),always_inline)) void armv7m_atomic_store_3_restart(volatile uint32_t *p_data, uint32_t data_0, uint32_t data_1, uint32_t data_2)
{
    register uint32_t _a0 __asm__("r0") = data_0;
    register uint32_t _a1 __asm__("r1") = data_1;
    register uint32_t _a2 __asm__("r2") = data_2;
    
    __asm__ volatile ("stm %0!, { %1, %2, %3 }"
                      : "+&l" (p_data)
                      : "r" (_a0), "r" (_a1), "r" (_a2)
                      : "memory");
}

static inline __attribute__((optimize("O3"),always_inline)) void armv7m_atomic_store_4_restart(volatile uint32_t *p_data, uint32_t data_0, uint32_t data_1, uint32_t data_2, uint32_t data_3)
{
    register uint32_t _a0 __asm__("r0") = data_0;
    register uint32_t _a1 __asm__("r1") = data_1;
    register uint32_t _a2 __asm__("r2") = data_2;
    register uint32_t _a3 __asm__("r3") = data_3;
    
    __asm__ volatile ("stm %0!, { %1, %2, %3, %4 }"
                      : "+&l" (p_data)
                      : "r" (_a0), "r" (_a1), "r" (_a2), "r" (_a3)
                      : "memory");
}

static inline __attribute__((optimize("O3"),always_inline)) void armv7m_atomic_load_2(volatile uint32_t *p_data, uint32_t *p_data_0, uint32_t *p_data_1)
{
    register uint32_t _a0 __asm__("r0");
    register uint32_t _a1 __asm__("r1");
    
    __asm__ volatile ("ldm %0!, { %1, %2 }"
                      : "+&l" (p_data), "=r" (_a0), "=r" (_a1)
                      :
                      : "memory");

    *p_data_0 = _a0;
    *p_data_1 = _a1;
}

static inline __attribute__((optimize("O3"),always_inline)) void armv7m_atomic_load_3(volatile uint32_t *p_data, uint32_t *p_data_0, uint32_t *p_data_1, uint32_t *p_data_2)
{
    register uint32_t _a0 __asm__("r0");
    register uint32_t _a1 __asm__("r1");
    register uint32_t _a2 __asm__("r2");
    
    __asm__ volatile ("ldm %0!, { %1, %2, %3 }"
                      : "+&l" (p_data), "=r" (_a0), "=r" (_a1), "=r" (_a2)
                      :
                      : "memory");

    *p_data_0 = _a0;
    *p_data_1 = _a1;
    *p_data_2 = _a2;
}

static inline __attribute__((optimize("O3"),always_inline)) void armv7m_atomic_load_4(volatile uint32_t *p_data, uint32_t *p_data_0, uint32_t *p_data_1, uint32_t *p_data_2, uint32_t *p_data_3)
{
    register uint32_t _a0 __asm__("r0");
    register uint32_t _a1 __asm__("r1");
    register uint32_t _a2 __asm__("r2");
    register uint32_t _a3 __asm__("r3");
    
    __asm__ volatile ("ldm %0!, { %1, %2, %3, %4 }"
                      : "+&l" (p_data), "=r" (_a0), "=r" (_a1), "=r" (_a2), "=r" (_a3)
                      :
                      : "memory");

    *p_data_0 = _a0;
    *p_data_1 = _a1;
    *p_data_2 = _a2;
    *p_data_3 = _a3;
}

#ifdef __cplusplus
}
#endif

#endif /* _ARMV7M_ATOMIC_H */
