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

    /* External Flash area if defined */
    .extflash : ALIGN(4) SUBALIGN(4)
    {
        *(.ap_romfs*)
        /* moving GCS_MAVLink to ext_flash */
        lib/lib*.a:GCS*.*(.text* .rodata*)
        *(.extflash)
    } > ext_flash

    .text : ALIGN(4) SUBALIGN(4)
    {
        /* we want app_descriptor near the start of flash so a false
           positive isn't found by the bootloader (eg. ROMFS) */
        KEEP(*(.apsec_data));
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

    /* The default heap uses the (statically) unused part of a RAM section.*/
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        __heap_base__ = .;
        . = ORIGIN(HEAP_RAM) + LENGTH(HEAP_RAM);
        __heap_end__ = .;
    } > HEAP_RAM

    /* The crash log uses the unused part of a flash section.*/
    .crash_log (NOLOAD) :
    {
        . = ALIGN(32);
        __crash_log_base__ = .;
        . = ORIGIN(flash) + LENGTH(flash);
        __crash_log_end__ = .;
    } > flash

}
