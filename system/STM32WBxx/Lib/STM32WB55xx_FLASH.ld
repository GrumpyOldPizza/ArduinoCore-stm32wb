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
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
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
   FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 0x000c0000 /* 768k (BLE starts @ 0x000cb000)  */
   EEPROM (r)   : ORIGIN = 0x080c0000, LENGTH = 0x00008000 /* 32k  (BLE starts @ 0x000cb000)  */
   RAM    (rw)  : ORIGIN = 0x20000000, LENGTH = 0x00030000 /* 192k                            */
   IPCRAM (rw)  : ORIGIN = 0x20030000, LENGTH = 0x00001800 /* 6k                              */
   RETRAM (rw)  : ORIGIN = 0x20031800, LENGTH = 0x00001000 /* 4k                              */
   CCMRAM (rx)  : ORIGIN = 0x20038000, LENGTH = 0x00002000 /* 8k                              */
}

GROUP(libgcc.a libc.a libm.a libnosys.a)

ENTRY(Reset_Handler)

SECTIONS
{
   .text :
   {
     __Vectors = .;
       KEEP(*(.vectors))

       . = ORIGIN(FLASH) + 0x140;
     __tag_ota_start__ = .;
       LONG (__tag_ota_end__)
       
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
   } > FLASH

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

     __copy_table_start__ = .;
       LONG (LOADADDR(.data))
       LONG (__data_start__)
       LONG (__data_end__ - __data_start__)
       LONG (LOADADDR(.ccmram))
       LONG (__ccmram_start__)
       LONG (__ccmram_end__ - __ccmram_start__)
     __copy_table_end__ = .;
     __zero_table_start__ = .;
       LONG (__bss_start__)
       LONG (__bss_end__ - __bss_start__)
     __zero_table_end__ = .;
       . = ALIGN(8);
   } > FLASH

   .stack (NOLOAD) :
   {
     __StackLimit = .;
       KEEP(*(.stack .stack.*))
     . = ALIGN(1024);
     __StackTop = .;
     PROVIDE(__stack = __StackTop);
   } > RAM

   .data :
   {
     __data_start__ = .;
       *(vtable)
       *(.data .data.* .gnu.linkonce.d.*)
       . = ALIGN(8);
     __data_end__ = .;
   } > RAM AT >FLASH

   .bss (NOLOAD) :
   {
     __bss_start__ = .;
       *(.bss .bss.* .gnu.linkonce.b.*)
       *(COMMON)
       . = ALIGN(8);
     __bss_end__ = .;
   } > RAM

   .noinit (NOLOAD) :
   {
     __noinit_start__ = .;
       *(.noinit .noinit.*)
       . = ALIGN(8);
     __noinit_end__ = .;
   } > RAM

   __end__ = .;
   PROVIDE(end = .);

   .ipcram (NOLOAD) :
   {
     __ipcram_start__ = .;
       KEEP(*(.ipcram.ref_table))
       KEEP(*(.ipcram.sys_state))
       KEEP(*(.ipcram.info_table))
       KEEP(*(.ipcram.ble_table))
       KEEP(*(.ipcram.sys_table))
       KEEP(*(.ipcram.mm_table))
       *(.ipcram .ipcram.*)
       . = ALIGN(8);
     __ipcram_end__ = .;
   } >IPCRAM

   .retram :
   {
     __retram_start__ = .;
       *(.retram .retram.*)
       . = ALIGN(8);
     __retram_end__ = .;
   } > RETRAM AT >FLASH

   __retram_flash__ = LOADADDR(.retram);

   .ccmram :
   {
     __ccmram_start__ = .;
       *(.ccmram .ccmram.*)
       . = ALIGN(8);
     __ccmram_end__ = .;
   } >CCMRAM AT >FLASH

   .ARM.extab :
   {
       *(.ARM.extab* .gnu.linkonce.armextab.*)
       . = ALIGN(8);
   } > FLASH

   .ARM.exidx :
   {
       PROVIDE_HIDDEN(__exidx_start = .);
       *(.ARM.exidx* .gnu.linkonce.armexidx.*)
       PROVIDE_HIDDEN(__exidx_end = .);
       . = ALIGN(8);
   } > FLASH

   /* Align the .ota_signature, so that the final image always ends at a 1024 byte boundary */
   .ota_signature (ALIGN(LOADADDR(.ARM.exidx) + SIZEOF(.ARM.exidx) + 4, 1024) - 4) :
   {
     __tag_ota_end__ = .;
       LONG (0x94448A29)
   } > FLASH

   __HeapLimit = ORIGIN(RAM) + LENGTH(RAM);
   __FlashBase = ALIGN(LOADADDR(.ota_signature) + SIZEOF(.ota_signature), 4096);
   __FlashLimit = ORIGIN(FLASH) + LENGTH(FLASH);

   __eeprom_start = ORIGIN(EEPROM);
   __eeprom_end = ORIGIN(EEPROM) + LENGTH(EEPROM);

   /* Stabs debugging sections.  */
   .stab          0 : { *(.stab) }
   .stabstr       0 : { *(.stabstr) }
   .stab.excl     0 : { *(.stab.excl) }
   .stab.exclstr  0 : { *(.stab.exclstr) }
   .stab.index    0 : { *(.stab.index) }
   .stab.indexstr 0 : { *(.stab.indexstr) }
   .comment       0 : { *(.comment) }
   /* DWARF debug sections.
      Symbols in the DWARF debugging sections are relative to the beginning
      of the section so we begin them at 0.  */
   /* DWARF 1 */
   .debug          0 : { *(.debug) }
   .line           0 : { *(.line) }
   /* GNU DWARF 1 extensions */
   .debug_srcinfo  0 : { *(.debug_srcinfo) }
   .debug_sfnames  0 : { *(.debug_sfnames) }
   /* DWARF 1.1 and DWARF 2 */
   .debug_aranges  0 : { *(.debug_aranges) }
   .debug_pubnames 0 : { *(.debug_pubnames) }
   /* DWARF 2 */
   .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
   .debug_abbrev   0 : { *(.debug_abbrev) }
   .debug_line     0 : { *(.debug_line) }
   .debug_frame    0 : { *(.debug_frame) }
   .debug_str      0 : { *(.debug_str) }
   .debug_loc      0 : { *(.debug_loc) }
   .debug_macinfo  0 : { *(.debug_macinfo) }
   /* SGI/MIPS DWARF 2 extensions */
   .debug_weaknames 0 : { *(.debug_weaknames) }
   .debug_funcnames 0 : { *(.debug_funcnames) }
   .debug_typenames 0 : { *(.debug_typenames) }
   .debug_varnames  0 : { *(.debug_varnames) }
   /* DWARF 3 */
   .debug_pubtypes 0 : { *(.debug_pubtypes) }
   .debug_ranges   0 : { *(.debug_ranges) }
   /* DWARF Extension.  */
   .debug_macro    0 : { *(.debug_macro) }
   .ARM.attributes 0 : { KEEP (*(.ARM.attributes)) KEEP (*(.gnu.attributes)) }
   .note.gnu.arm.ident 0 : { KEEP (*(.note.gnu.arm.ident)) }
   /DISCARD/ : { *(.note.GNU-stack) *(.gnu_debuglink) *(.gnu.lto_*) }
}
