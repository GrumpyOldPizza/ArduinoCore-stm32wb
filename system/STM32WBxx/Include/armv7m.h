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

#if !defined(_ARMV7M_H)
#define _ARMV7M_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "stm32wbxx.h"

#include "armv7m_atomic.h"
#include "armv7m_core.h"
#include "armv7m_pendsv.h"
#include "armv7m_rtos.h"
#include "armv7m_rtt.h"
#include "armv7m_svcall.h"
#include "armv7m_systick.h"

#define __CONCAT1(x,y) x ## y
#define __CONCAT(x,y)  __CONCAT1(x,y)
#define __STRING(x)    #x
#define __XSTRING(x)   __STRING(x)

#define __SECTION_TEXT    __attribute__((section(".text."    __XSTRING(__COUNTER__))))
#define __SECTION_RODATA  __attribute__((section(".rodata."  __XSTRING(__COUNTER__))))

#define __SECTION_CCTEXT  __attribute__((section(".cctext."  __XSTRING(__COUNTER__))))

#define __SECTION_DATA    __attribute__((section(".data."    __XSTRING(__COUNTER__))))
#define __SECTION_BSS     __attribute__((section(".bss."     __XSTRING(__COUNTER__))))
#define __SECTION_NOINIT  __attribute__((section(".noinit."  __XSTRING(__COUNTER__))))
#define __SECTION_DMA     __attribute__((section(".dma."     __XSTRING(__COUNTER__))))

#define __SECTION_DATA2   __attribute__((section(".data2."   __XSTRING(__COUNTER__))))
#define __SECTION_BSS2    __attribute__((section(".bss2."    __XSTRING(__COUNTER__))))
#define __SECTION_NOINIT2 __attribute__((section(".noinit2." __XSTRING(__COUNTER__))))

#endif /* _ARMV7M_H */
