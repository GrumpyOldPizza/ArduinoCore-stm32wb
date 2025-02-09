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

ENTRY(stm32wb_boot_enter)

PHDRS
{
    boot_text   PT_LOAD  FLAGS(5);
    boot_rodata PT_LOAD  FLAGS(4);
    boot_stack  PT_NULL  FLAGS(6);
    boot_data   PT_LOAD  FLAGS(6);
    boot_bss    PT_NULL  FLAGS(6);
    boot_noinit PT_NULL  FLAGS(6);
}

SECTIONS
{
   .boot_rodata :
   {
     __boot_base__ = .;
     stm32wb_boot_vectors = .;
       KEEP(*stm32wb_boot.o(.rodata.__stm32wb_boot_vectors__))
     stm32wb_boot_info = .;
       KEEP(*stm32wb_boot.o(.rodata.__stm32wb_boot_info__))
       . = ALIGN(8);
       KEEP(*stm32wb_boot.o(.text.stm32wb_boot_enter))
       KEEP(*stm32wb_boot.o(.text.stm32wb_boot_leave))
/*       KEEP(*stm32wb_boot.o(.text.stm32wb_boot_reset)) */
/*       KEEP(*stm32wb_boot.o(.text.stm32wb_boot_nmi)) */
/*       KEEP(*stm32wb_boot.o(.text.stm32wb_boot_hardfault)) */
       *stm32wb_boot.o(.rodata .rodata.*)
       . = ALIGN(8);
} > BOOT :boot_rodata

   .boot_stack (NOLOAD) :
   {
       KEEP(*stm32wb_boot.o(.stack))
       . = ALIGN(32);
     stm32wb_boot_stack = .;
   } > SRAM :boot_stack

   .boot_data :
   {
     __boot_data_start__ = .;
       *stm32wb_boot.o(.data .data.* .gnu.linkonce.d.*)
       . = ALIGN(8);
     __boot_data_end__ = .;
   } > SRAM AT >BOOT :boot_data

  __boot_data_flash__ = LOADADDR(.boot_data);

   .boot_bss (NOLOAD) :
   {
     __boot_bss_start__ = .;
       *stm32wb_boot.o(.bss .bss.* .gnu.linkonce.b.*)
       *stm32wb_boot.o(COMMON)
       . = ALIGN(8);
     __boot_bss_end__ = .;
   } > SRAM :boot_bss

   .boot_noinit (NOLOAD) :
   {
     __boot_noinit_start__ = .;
       *stm32wb_boot.o(.noinit .noinit.*)
       . = ALIGN(8);
     __boot_noinit_end__ = .;
   } > SRAM :boot_noinit

   .boot_text : ALIGN(2048)
   {
       *stm32wb_boot.o(.text .text.*)
       . = ALIGN(8);
     __boot_limit__ = .;
       . = ALIGN(16384) - 80;
       LONG(0)
       . += 76;
   } > BOOT :boot_text

   __application_start__ = ORIGIN(APPLICATION);
   __application_end__ = ORIGIN(APPLICATION) + LENGTH(APPLICATION);

   __fwu_start__ = ORIGIN(FWU);
   __fwu_end__ = ORIGIN(FWU) + LENGTH(FWU) - 4096;
   __fwu_status__ = ORIGIN(FWU) + LENGTH(FWU);

   __eeprom_start__ = ORIGIN(EEPROM);
   __eeprom_end__ = ORIGIN(EEPROM) + LENGTH(EEPROM);

   __wireless_start__ = ORIGIN(WIRELESS);
}
