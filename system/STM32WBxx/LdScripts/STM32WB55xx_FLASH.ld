/*
 * Copyright (c) 2008-2020 Thomas Roell.  All rights reserved.
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
 * THE SOFTWARE IS PROVINCED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */


/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 * 
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __copy_table_start__
 *   __copy_table_end__
 *   __zero_table_start__
 *   __zero_table_end__
 *   __etext
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_start__
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackTop
 *   __stack
 */

MEMORY
{
   BOOT        (rx)  : ORIGIN = 0x08000000, LENGTH = 0x00004000 /* FLASH   16k                          */
   APPLICATION (rx)  : ORIGIN = 0x08004000, LENGTH = 0x00060000 /* FLASH   384k                         */
   FWU         (rx)  : ORIGIN = 0x08064000, LENGTH = 0x00062000 /* FLASH   392k                         */
   EEPROM      (r)   : ORIGIN = 0x080c6000, LENGTH = 0x00008000 /* FLASH   32k                          */
   WIRELESS          : ORIGIN = 0x080ce000, LENGTH = 0x00032000 /* FLASH   200k (Wireless/SafeBoot/FUS) */
   CCRAM       (rx)  : ORIGIN = 0x10008000, LENGTH = 0x00001000 /* SRAM2b  4k                           */
   SRAM        (rw)  : ORIGIN = 0x20000000, LENGTH = 0x00030000 /* SRAM1   192k                         */
   IPCC        (rw)  : ORIGIN = 0x20030000, LENGTH = 0x00001000 /* SRAM2a  4k                           */
   BBRAM       (rw)  : ORIGIN = 0x20031000, LENGTH = 0x00001800 /* SRAM2a  6k                           */
   RTT         (rw)  : ORIGIN = 0x20039000, LENGTH = 0x00002800 /* SRAM2b  10k                          */
}

GROUP(libgcc.a libc.a libm.a)

ENTRY(Reset_Handler)

PHDRS
{
    boot        PT_LOAD  FLAGS(5);
    text        PT_LOAD  FLAGS(5);
    rodata      PT_LOAD  FLAGS(4);
    stack       PT_NULL  FLAGS(6);
    data        PT_LOAD  FLAGS(6);
    bss         PT_NULL  FLAGS(6);
    noinit      PT_NULL  FLAGS(6);
    cctext      PT_LOAD  FLAGS(5);
    ipcc        PT_NULL  FLAGS(6);
    bbram       PT_NULL  FLAGS(6);
    rtt         PT_NULL  FLAGS(6);
    info        PT_LOAD  FLAGS(4);
}

SECTIONS
{
   .boot :
   {
       KEEP(*:boot_stm32wb55xx.o(.boot))
   } > BOOT :boot

   .text :
   {
     __application_start__ = .;
     __Vectors = .;
       KEEP(*(.isr_vectors))
       . = 0x140;
       
       *(.text .text.* .gnu.linkonce.t.*)

       KEEP(*(.init))
       KEEP(*(.fini))

       /* .ctors */
       *crtbegin.o(.ctors)
       *crtbegin?.o(.ctors)
       *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
       *(SORT(.ctors.*))
       *(.ctors)

       /* .dtors */
       *crtbegin.o(.dtors)
       *crtbegin?.o(.dtors)
       *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
       *(SORT(.dtors.*))
       *(.dtors)
       . = ALIGN(8);
   } > APPLICATION :text

   __etext = . ;

   .rodata :
   {
       *(.rodata .rodata.* .gnu.linkonce.r.*)

       KEEP(*(.eh_frame*))
       . = ALIGN(4);

       /* preinit data */
       PROVIDE_HIDDEN (__preinit_array_start = .);
       KEEP(*(.preinit_array))
       PROVIDE_HIDDEN (__preinit_array_end = .);
       . = ALIGN(4);

       /* init data */
       PROVIDE_HIDDEN (__init_array_start = .);
       KEEP(*(SORT(.init_array.*)))
       KEEP(*(.init_array))
       PROVIDE_HIDDEN (__init_array_end = .);
       . = ALIGN(4);

       /* finit data */
       PROVIDE_HIDDEN (__fini_array_start = .);
       KEEP(*(SORT(.fini_array.*)))
       KEEP(*(.fini_array))
       PROVIDE_HIDDEN (__fini_array_end = .);
       . = ALIGN(4);

     __system_info__ = .;
       KEEP(*(.info.system))

     __rtc_info__ = .;
       LONG  (DEFINED(__RTC_EPOCH__) ? __RTC_EPOCH__ : 0)
       SHORT (((DEFINED(__RTC_ZONE__) ? __RTC_ZONE__ : 0) + 43200) / 900)
       BYTE  ((DEFINED(__RTC_DST__) ? __RTC_DST__ : 0) / 3600)
       BYTE  ((DEFINED(__RTC_LEAP_SECONDS__) ? __RTC_LEAP_SECONDS__ : 18) + 128)

    __copy_table_start__ = .;
       LONG (LOADADDR(.data))
       LONG (__data_start__)
       LONG (__data_end__ - __data_start__)
     __copy_table_end__ = .;

    __zero_table_start__ = .;
       LONG (__bss_start__)
       LONG (__bss_end__ - __bss_start__)
     __zero_table_end__ = .;
       . = ALIGN(8);
   } > APPLICATION :rodata

   .ARM.extab :
   {
       *(.ARM.extab* .gnu.linkonce.armextab.*)
       . = ALIGN(8);
   } > APPLICATION :rodata

   .ARM.exidx :
   {
       PROVIDE_HIDDEN(__exidx_start = .);
       *(.ARM.exidx* .gnu.linkonce.armexidx.*)
       PROVIDE_HIDDEN(__exidx_end = .);
       . = ALIGN(8);
   } > APPLICATION :rodata

   .stack (NOLOAD) :
   {
       KEEP(*(.isr_stack))
       . = ALIGN(32);
     __StackTop = .;
   } > SRAM :stack

   .data :
   {
     __data_start__ = .;
       *(vtable)
       *(.data .data.* .gnu.linkonce.d.*)
       . = ALIGN(8);
     __data_end__ = .;
       . = ALIGN(8);
   } > SRAM AT >APPLICATION :data

   .bss (NOLOAD) :
   {
     __bss_start__ = .;
       *(.bss .bss.* .gnu.linkonce.b.*)
       *(COMMON)
       . = ALIGN(8);
     __bss_end__ = .;
   } > SRAM :bss

   .noinit (NOLOAD) :
   {
       *(.noinit .noinit.*)
       . = ALIGN(8);
   } > SRAM :noinit

   __end__ = .;
   PROVIDE(end = .);

   .heap (NOLOAD) :
   {
     __HeapBase = .;
       *(.heap .heap.*)
       . = ALIGN(32);
     __HeapLimit = .;
   } > SRAM

   /* Set stack top to end of RAM, and stack limit move down by
    * size of stack_dummy section */
   .stack_dummy (NOLOAD):
   {
       *(.stack .stack.*)
       . = ALIGN(32);
   } > SRAM

   __StackBase = ORIGIN(SRAM) + LENGTH(SRAM);
   __StackLimit = __StackTop - SIZEOF(.stack_dummy);
   PROVIDE(__HeapLimit = __StackLimit);

   .ipcc (NOLOAD) :
   {
     __ipcc_start__ = .;
       KEEP(*(.ipcc .ipcc.*))
       . = ALIGN(8);
     __ipcc_end__ = .;
   } >IPCC :ipcc

   .bbram (NOLOAD) :
   {
     __bbram_start__ = .;
       *(.bbram .bbram.*)
       . = ALIGN(8);
     __bbram_end__ = .;
   } > BBRAM :bbram

   .cctext :
   {
     __cctext_start__ = .;
       *(.cctext .cctext.*)
       . = ALIGN(8);
     __cctext_end__ = .;
   } >CCRAM AT >APPLICATION :cctext

   __cctext_flash__ = LOADADDR(.cctext);

   .rtt (NOLOAD) :
   {
       KEEP(*(.rtt_control))
       KEEP(*(.rtt_data .rtt_data.*))
   } > RTT :rtt
   
   .info : ALIGN(16)
   {
     __application_info__ = .;
     stm32wb_application_info = .;
       KEEP(*(.info.uuid))          /* UUID      */
       . = 16;                      /* VERSION   */
       KEEP(*(.info.version))
       . = 20;                      /* SEQUENCE  */
       KEEP(*(.info.sequence))
       . = 24;                      /* EPOCH     */
       . = 28;                      /* CRC32     */
       . = 32;                      /* SIGNATURE */
       . = 96 - 4;
       LONG(0)
     __application_end__ = .;
   } > APPLICATION :info

   __FlashBase = ALIGN(LOADADDR(.info) + SIZEOF(.info), 4096);
   __FlashLimit = ORIGIN(APPLICATION) + LENGTH(APPLICATION);

   __ccram_base__ = ORIGIN(CCRAM);
   __ccram_limit__ = ORIGIN(CCRAM) + LENGTH(CCRAM);

   __application_base__ = ORIGIN(APPLICATION);
   __application_limit__ = ORIGIN(APPLICATION) + LENGTH(APPLICATION);

   __fwu_base__ = ORIGIN(FWU);
   __fwu_limit__ = ORIGIN(FWU) + LENGTH(FWU) - 4096;
   __fwu_status__ = ORIGIN(FWU) + LENGTH(FWU);

   __eeprom_base__ = ORIGIN(EEPROM);
   __eeprom_limit__ = ORIGIN(EEPROM) + LENGTH(EEPROM);

   __wireless_base__ = ORIGIN(WIRELESS);
}
