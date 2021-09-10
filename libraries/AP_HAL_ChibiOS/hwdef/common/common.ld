/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
    this file is included by the board specific ldscript.ld which is
    generated from hwdef.dat
*/

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts*/
REGION_ALIAS("MAIN_STACK_RAM", ram0);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram0);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram0);

__ram0_start__          = ORIGIN(ram0);
__ram0_size__           = LENGTH(ram0);
__ram0_end__            = __ram0_start__ + __ram0_size__;

ENTRY(Reset_Handler)

SECTIONS
{
    . = 0;
    _text = .;

    startup : ALIGN(16) SUBALIGN(16)
    {
        KEEP(*(.vectors))
    } > flash

    constructors : ALIGN(4) SUBALIGN(4)
    {
        __init_array_base__ = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end__ = .;
    } > flash

    destructors : ALIGN(4) SUBALIGN(4)
    {
        __fini_array_base__ = .;
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        __fini_array_end__ = .;
    } > flash

    .text : ALIGN(4) SUBALIGN(4)
    {
        /* we want app_descriptor near the start of flash so a false
           positive isn't found by the bootloader (eg. ROMFS) */
        KEEP(*(.app_descriptor));
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > flash

    .ARM.exidx : {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
     } > flash

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > flash

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > flash

    .textalign : ONLY_IF_RO
    {
        . = ALIGN(8);
    } > flash

    /* Legacy symbol, not used anywhere.*/
    . = ALIGN(4);
    PROVIDE(_etext = .);

    /* Special section for exceptions stack.*/
    .mstack :
    {
        . = ALIGN(8);
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(8);
        __main_stack_end__ = .;
    } > MAIN_STACK_RAM

    /* Special section for process stack.*/
    .pstack :
    {
        __process_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
        __main_thread_stack_end__ = .;
    } > PROCESS_STACK_RAM

    .data : ALIGN(4)
    {
        . = ALIGN(4);
        PROVIDE(_textdata = LOADADDR(.data));
        PROVIDE(_data = .);
        __textdata_base__ = LOADADDR(.data);
        __data_base__ = .;
        *(.data)
        *(.data.*)
        *(.ramtext)
        . = ALIGN(4);
        PROVIDE(_edata = .);
        __data_end__ = .;
    } > DATA_RAM AT > flash

    .bss (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        __bss_base__ = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
        PROVIDE(end = .);
    } > BSS_RAM

    .ram0_init : ALIGN(4)
    {
        . = ALIGN(4);
        __ram0_init_text__ = LOADADDR(.ram0_init);
        __ram0_init__ = .;
        *(.ram0_init)
        *(.ram0_init.*)
        . = ALIGN(4);
    } > ram0 AT > flash

    .ram0 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        __ram0_clear__ = .;
        *(.ram0_clear)
        *(.ram0_clear.*)
        . = ALIGN(4);
        __ram0_noinit__ = .;
        *(.ram0)
        *(.ram0.*)
        . = ALIGN(4);
        __ram0_free__ = .;
    } > ram0

    .ram1_init : ALIGN(4)
    {
        __ram1_init_text__ = LOADADDR(.ram1_init);
        __ram1_init__ = .;
        KEEP(*(.ram1_init))
        KEEP(*(.ram1_init.*))
        . = ALIGN(4);
    } > ram1 AT > RAM_INIT_FLASH_LMA

    .ram1 (NOLOAD) : ALIGN(4)
    {
        __ram1_clear__ = .;
        *(.ram1_clear)
        *(.ram1_clear.*)
        . = ALIGN(4);
        __ram1_noinit__ = .;
        *(.ram1)
        *(.ram1.*)
        . = ALIGN(4);
        __ram1_free__ = .;
    } > ram1

    .ram2_init : ALIGN(4)
    {
        __ram2_init_text__ = LOADADDR(.ram2_init);
        __ram2_init__ = .;
        KEEP(*(.ram2_init))
        KEEP(*(.ram2_init.*))
        . = ALIGN(4);
    } > ram2 AT > RAM_INIT_FLASH_LMA

    .ram2 (NOLOAD) : ALIGN(4)
    {
        __ram2_clear__ = .;
        *(.ram2_clear)
        *(.ram2_clear.*)
        . = ALIGN(4);
        __ram2_noinit__ = .;
        *(.ram2)
        *(.ram2.*)
        . = ALIGN(4);
        __ram2_free__ = .;
    } > ram2

    .ram3_init : ALIGN(4)
    {
        __ram3_init_text__ = LOADADDR(.ram3_init);
        __ram3_init__ = .;
        KEEP(*(.ram3_init))
        KEEP(*(.ram3_init.*))
        . = ALIGN(4);
    } > ram3 AT > RAM_INIT_FLASH_LMA

    .ram3 (NOLOAD) : ALIGN(4)
    {
        __ram3_clear__ = .;
        *(.ram3_clear)
        *(.ram3_clear.*)
        . = ALIGN(4);
        __ram3_noinit__ = .;
        *(.ram3)
        *(.ram3.*)
        . = ALIGN(4);
        __ram3_free__ = .;
    } > ram3

    .ram4_init : ALIGN(4)
    {
        __ram4_init_text__ = LOADADDR(.ram4_init);
        __ram4_init__ = .;
        KEEP(*(.ram4_init))
        KEEP(*(.ram4_init.*))
        . = ALIGN(4);
    } > ram4 AT > RAM_INIT_FLASH_LMA

    .ram4 (NOLOAD) : ALIGN(4)
    {
        __ram4_clear__ = .;
        *(.ram4_clear)
        *(.ram4_clear.*)
        . = ALIGN(4);
        __ram4_noinit__ = .;
        *(.ram4)
        *(.ram4.*)
        . = ALIGN(4);
        __ram4_free__ = .;
    } > ram4

    .ram5_init : ALIGN(4)
    {
        __ram5_init_text__ = LOADADDR(.ram5_init);
        __ram5_init__ = .;
        KEEP(*(.ram5_init))
        KEEP(*(.ram5_init.*))
        . = ALIGN(4);
    } > ram5 AT > RAM_INIT_FLASH_LMA

    .ram5 (NOLOAD) : ALIGN(4)
    {
        __ram5_clear__ = .;
        *(.ram5_clear)
        *(.ram5_clear.*)
        . = ALIGN(4);
        __ram5_noinit__ = .;
        *(.ram5)
        *(.ram5.*)
        . = ALIGN(4);
        __ram5_free__ = .;
    } > ram5

    .ram6_init : ALIGN(4)
    {
        __ram6_init_text__ = LOADADDR(.ram6_init);
        __ram6_init__ = .;
        KEEP(*(.ram6_init))
        KEEP(*(.ram6_init.*))
        . = ALIGN(4);
    } > ram6 AT > RAM_INIT_FLASH_LMA

    .ram6 (NOLOAD) : ALIGN(4)
    {
        __ram6_clear__ = .;
        *(.ram6_clear)
        *(.ram6_clear.*)
        . = ALIGN(4);
        __ram6_noinit__ = .;
        *(.ram6)
        *(.ram6.*)
        . = ALIGN(4);
        __ram6_free__ = .;
    } > ram6

    .ram7_init : ALIGN(4)
    {
        __ram7_init_text__ = LOADADDR(.ram7_init);
        __ram7_init__ = .;
        KEEP(*(.ram7_init))
        KEEP(*(.ram7_init.*))
        . = ALIGN(4);
    } > ram7 AT > RAM_INIT_FLASH_LMA

    .ram7 (NOLOAD) : ALIGN(4)
    {
        __ram7_clear__ = .;
        *(.ram7_clear)
        *(.ram7_clear.*)
        . = ALIGN(4);
        __ram7_noinit__ = .;
        *(.ram7)
        *(.ram7.*)
        . = ALIGN(4);
        __ram7_free__ = .;
    } > ram7

    /* The default heap uses the (statically) unused part of a RAM section.*/
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        __heap_base__ = .;
        . = ORIGIN(HEAP_RAM) + LENGTH(HEAP_RAM);
        __heap_end__ = .;
    } > HEAP_RAM
}
