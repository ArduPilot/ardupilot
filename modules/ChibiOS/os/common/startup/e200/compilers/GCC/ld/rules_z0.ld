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

__ram_size__            = LENGTH(ram);
__ram_start__           = ORIGIN(ram);
__ram_end__             = ORIGIN(ram) + LENGTH(ram);

ENTRY(_reset_address)

SECTIONS
{
    . = ORIGIN(flash);
    .boot0 : ALIGN(16)
    {
        KEEP(*(.boot))
    } > flash

    .boot1 : ALIGN(16)
    {
        KEEP(*(.handlers))
        KEEP(*(.crt0))
        /* The vectors table requires a 2kB alignment.*/
        . = ALIGN(0x800);
        KEEP(*(.vectors))
        /* The IVPR register requires a 4kB alignment.*/
        . = ALIGN(0x1000);
        __ivpr_base__ = .;
        KEEP(*(.ivors))
    } > flash

    constructors : ALIGN(4)
    {
        PROVIDE(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end = .);
    } > flash

    destructors : ALIGN(4)
    {
        PROVIDE(__fini_array_start = .);
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        PROVIDE(__fini_array_end = .);
    } > flash

    .text_vle : ALIGN(16)
    {
        *(.text_vle)
        *(.text_vle.*)
        *(.gnu.linkonce.t_vle.*)
    } > flash

    .text : ALIGN(16)
    {
        *(.text)
        *(.text.*)
        *(.gnu.linkonce.t.*)
    } > flash

    .rodata : ALIGN(16)
    {
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
        *(.rodata)
        *(.rodata.*)
        *(.rodata1)
    } > flash

    .sdata2 : ALIGN(16)
    {
        __sdata2_start__ = . + 0x8000;
        *(.sdata2)
        *(.sdata2.*)
        *(.gnu.linkonce.s2.*)
        *(.sbss2)
        *(.sbss2.*)
        *(.gnu.linkonce.sb2.*)
    } > flash

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > flash

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > flash

    .romdata : ALIGN(16)
    {
        __romdata_start__ = .;
    } > flash

    .stacks (NOLOAD) : ALIGN(16)
    {
        . = ALIGN(8);
        __irq_stack_base__ = .;
        . += __irq_stack_size__;
        . = ALIGN(8);
        __irq_stack_end__ = .;
        __process_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
        __main_thread_stack_end__ = .;
    } > ram

    .data : AT(__romdata_start__)
    {
        . = ALIGN(4);
        __data_start__ = .;
        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d.*)
        __sdata_start__ = . + 0x8000;
        *(.sdata)
        *(.sdata.*)
        *(.gnu.linkonce.s.*)
        __data_end__ = .;
    } > ram

    .sbss (NOLOAD) :
    {
        __bss_start__ = .;
        *(.sbss)
        *(.sbss.*)
        *(.gnu.linkonce.sb.*)
        *(.scommon)
    } > ram

    .bss (NOLOAD) :
    {
        *(.bss)
        *(.bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        __bss_end__ = .;
    } > ram

    __heap_base__   = __bss_end__;
    __heap_end__    = __ram_end__;
}
