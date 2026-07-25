/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

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

__stacks_total_size__	    = __und_stack_size__ + __abt_stack_size__ + __fiq_stack_size__ + __irq_stack_size__ + __svc_stack_size__ + __sys_stack_size__;

__ram0_base__           = ORIGIN(ram0);
__ram0_size__           = LENGTH(ram0);
__ram0_end__            = __ram0_base__ + __ram0_size__;
__ram1_base__           = ORIGIN(ram1);
__ram1_size__           = LENGTH(ram1);
__ram1_end__            = __ram1_base__ + __ram1_size__;
__ram2_base__           = ORIGIN(ram2);
__ram2_size__           = LENGTH(ram2);
__ram2_end__            = __ram2_base__ + __ram2_size__;
__ram3_base__           = ORIGIN(ram3);
__ram3_size__           = LENGTH(ram3);
__ram3_end__            = __ram3_base__ + __ram3_size__;
__ram4_base__           = ORIGIN(ram4);
__ram4_size__           = LENGTH(ram4);
__ram4_end__            = __ram4_base__ + __ram4_size__;
__ram5_base__           = ORIGIN(ram5);
__ram5_size__           = LENGTH(ram5);
__ram5_end__            = __ram5_base__ + __ram5_size__;
__ram6_base__           = ORIGIN(ram6);
__ram6_size__           = LENGTH(ram6);
__ram6_end__            = __ram6_base__ + __ram6_size__;
__ram7_base__           = ORIGIN(ram7);
__ram7_size__           = LENGTH(ram7);
__ram7_end__            = __ram7_base__ + __ram7_size__;

ENTRY(Reset_Handler)

SECTIONS
{
    . = 0;
    _text = .;

    startup : ALIGN(16) 
    {
        KEEP(*(.vectors))
        KEEP(*(.boot))
    } > flash

    constructors : ALIGN(4) 
    {
        PROVIDE(__init_array_base__ = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end__ = .);
    } > flash

    destructors : ALIGN(4) 
    {
        PROVIDE(__fini_array_base__ = .);
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        PROVIDE(__fini_array_end__ = .);
    } > flash

    .text : ALIGN_WITH_INPUT
    {
       __text_base__ = .;
        *(.text)
        *(.text.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
        __text_end__ = .;
    } > flash

    .rodata : ALIGN(4)
    {
        __rodata_base__ = .;
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4);
        __rodata_end__ = .;
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > flash

    .ARM.exidx : {
        __exidx_base__ = .;
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end__ = .;
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

    . = ALIGN(4);
    _etext = .;

    .stacks (NOLOAD) :
    {
        . = ALIGN(8);
        __stacks_base__ = .;
        __main_thread_stack_base__ = .;
        . += __sys_stack_size__;
       . = ALIGN(8);
        __main_thread_stack_end__ = .;
        . += __stacks_total_size__ - __sys_stack_size__;
        . = ALIGN(8);
        __stacks_end__ = .;
    } > STACKS_RAM

    .data : ALIGN(4)
    {
        . = ALIGN(4);
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
        . = ALIGN(4);
        __bss_end__ = .;
        PROVIDE(end = .);
    } > BSS_RAM    

    .ram0 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram0)
        *(.ram0.*)
        . = ALIGN(4);
        __ram0_free__ = .;
    } > ram0

    .ram1 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram1)
        *(.ram1.*)
        . = ALIGN(4);
        __ram1_free__ = .;
    } > ram1

    .ram2 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram2)
        *(.ram2.*)
        . = ALIGN(4);
        __ram2_free__ = .;
    } > ram2

    .ram3 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram3)
        *(.ram3.*)
        . = ALIGN(4);
        __ram3_free__ = .;
    } > ram3

    .ram4 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram4)
        *(.ram4.*)
        . = ALIGN(4);
        __ram4_free__ = .;
    } > ram4

    .ram5 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram5)
        *(.ram5.*)
        . = ALIGN(4);
        __ram5_free__ = .;
    } > ram5

    .ram6 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram6)
        *(.ram6.*)
        . = ALIGN(4);
        __ram6_free__ = .;
    } > ram6

    .ram7 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram7)
        *(.ram7.*)
        . = ALIGN(4);
        __ram7_free__ = .;
    } > ram7
}

/* Heap default boundaries, it is defaulted to be the non-used part
   of ram0 region.*/
__heap_base__   = __ram0_free__;
__heap_end__    = __ram0_end__;
