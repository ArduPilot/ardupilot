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

/*
 * SPC560B50 memory setup.
 */
MEMORY
{
    flash     : org = 0x00000000, len = 512k
    dataflash : org = 0x00800000, len = 64k
    ram       : org = 0x40000000, len = 32k
}

OPTION ("-e=_reset_address")

SECTIONS
{
    .boot0 ALIGN(16) :
    {
        *(.boot)
        *(.handlers)
        *(.crt0)
    } > flash

    .vectors ALIGN(0x800) :
    {
        /* The vectors table requires a 2kB alignment.*/
        *(.vectors)
    } > flash

    .ivors ALIGN(0x1000) :
    {
        /* The IVORs table requires a 4kB alignment.*/
        __ivpr_base__ = .;
        *(.ivors)
    } > flash

    constructors ALIGN(4) :
    {
        PROVIDE(__init_array_start = .);
        "*(.init_array.*)"
        *(.init_array)
        PROVIDE(__init_array_end = .);
    } > flash

    destructors ALIGN(4) :
    {
        PROVIDE(__fini_array_start = .);
        *(.fini_array)
        "*(.fini_array.*)"
        PROVIDE(__fini_array_end = .);
    } > flash

    .vletext ALIGN(16) :
    {
        *(.vletext)
        "*(.vletext.*)"
    } > flash

    .text ALIGN(16) :
    {
        *(.text)
        "*(.text.*)"
    } > flash

    .rodata ALIGN(16) :
    {
        *(.rodata)
        "*(.rodata.*)"
        *(.rodata1)
    } > flash

    .sdata2 ALIGN(16) :
    {
        __sdata2_start__ = . + 0x8000;
        *(.sdata2)
        "*(.sdata2.*)"
        *(.sbss2)
        "*(.sbss2.*)"
    } > flash

    .stacks ALIGN(16) :
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

    .romdatastart ALIGN(16) :
    {
        __romdata_start__ = .;
    } > flash

    .data ALIGN(4) : AT(__romdata_start__)
    {
        __data_start__ = .;
        *(.data)
        "*(.data.*)"
        . = ALIGN(4);
        *(.ramtext)
        . = ALIGN(4);
        __data_end__ = .;
    } > ram

    __romsdata_start__ = (__data_end__ - __data_start__) + __romdata_start__;

    .sdata ALIGN(4) : AT(__romsdata_start__)
    {
         __sdata_start__ = . + 0x8000;
        *(.sdata)
        "*(.sdata.*)"
    } > ram

    .sbss ALIGN(4) :
    {
        __bss_start__ = .;
        *(.sbss)
        "*(.sbss.*)"
        *(.scommon)
    } > ram

    .bss ALIGN(4) :
    {
        *(.bss)
        "*(.bss.*)"
        *(COMMON)
        __bss_end__ = .;
    } > ram

    __flash_size__          = SIZEOF(flash);
    __flash_start__         = ADDR(flash);
    __flash_end__           = ENDADDR(flash);

    __dataflash_size__      = SIZEOF(dataflash);
    __dataflash_start__     = ADDR(dataflash);
    __dataflash_end__       = ENDADDR(dataflash);

    __ram_size__            = SIZEOF(ram);
    __ram_start__           = ADDR(ram);
    __ram_end__             = ENDADDR(ram);

    __heap_base__           = __bss_end__;
    __heap_end__            = __ram_end__;
}
