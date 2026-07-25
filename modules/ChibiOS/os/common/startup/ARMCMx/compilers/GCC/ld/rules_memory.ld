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

__flash0_base__         = ORIGIN(flash0);
__flash0_size__         = LENGTH(flash0);
__flash0_end__          = __flash0_base__ + __flash0_size__;
__flash1_base__         = ORIGIN(flash1);
__flash1_size__         = LENGTH(flash1);
__flash1_end__          = __flash1_base__ + __flash1_size__;
__flash2_base__         = ORIGIN(flash2);
__flash2_size__         = LENGTH(flash2);
__flash2_end__          = __flash2_base__ + __flash2_size__;
__flash3_base__         = ORIGIN(flash3);
__flash3_size__         = LENGTH(flash3);
__flash3_end__          = __flash3_base__ + __flash3_size__;
__flash4_base__         = ORIGIN(flash4);
__flash4_size__         = LENGTH(flash4);
__flash4_end__          = __flash4_base__ + __flash4_size__;
__flash5_base__         = ORIGIN(flash5);
__flash5_size__         = LENGTH(flash5);
__flash5_end__          = __flash5_base__ + __flash5_size__;
__flash6_base__         = ORIGIN(flash6);
__flash6_size__         = LENGTH(flash6);
__flash6_end__          = __flash6_base__ + __flash6_size__;
__flash7_base__         = ORIGIN(flash7);
__flash7_size__         = LENGTH(flash7);
__flash7_end__          = __flash7_base__ + __flash7_size__;

SECTIONS
{
    .ram0_init : ALIGN(4)
    {
        __ram0_init_text__ = LOADADDR(.ram0_init);
        __ram0_init__ = .;
        KEEP(*(.ram0_init))
        KEEP(*(.ram0_init.*))
        . = ALIGN(4);
    } > ram0 AT > RAM_INIT_FLASH_LMA

    .ram0 (NOLOAD) : ALIGN(4)
    {
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

    .flash0 : ALIGN(4)
    {
        __flash0_init__ = .;
        KEEP(*(.flash0_init))
        KEEP(*(.flash0_init.*))
        __flash0_free__ = .;
    } > flash0

    .flash1 : ALIGN(4)
    {
        __flash1_init__ = .;
        KEEP(*(.flash1_init))
        KEEP(*(.flash1_init.*))
        __flash1_free__ = .;
    } > flash1

    .flash2 : ALIGN(4)
    {
        __flash2_init__ = .;
        KEEP(*(.flash2_init))
        KEEP(*(.flash2_init.*))
        __flash2_free__ = .;
    } > flash2

    .flash3 : ALIGN(4)
    {
        __flash3_init__ = .;
        KEEP(*(.flash3_init))
        KEEP(*(.flash3_init.*))
        __flash3_free__ = .;
    } > flash3

    .flash4 : ALIGN(4)
    {
        __flash4_init__ = .;
        KEEP(*(.flash4_init))
        KEEP(*(.flash4_init.*))
        __flash4_free__ = .;
    } > flash4

    .flash5 : ALIGN(4)
    {
        __flash5_init__ = .;
        KEEP(*(.flash5_init))
        KEEP(*(.flash5_init.*))
        __flash5_free__ = .;
    } > flash5

    .flash6 : ALIGN(4)
    {
        __flash6_init__ = .;
        KEEP(*(.flash6_init))
        KEEP(*(.flash6_init.*))
        __flash6_free__ = .;
    } > flash6

    .flash7 : ALIGN(4)
    {
        __flash7_init__ = .;
        KEEP(*(.flash7_init))
        KEEP(*(.flash7_init.*))
        __flash7_free__ = .;
    } > flash7

    /* The default heap uses the (statically) unused part of a RAM section.*/
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        __heap_base__ = .;
        . = ORIGIN(HEAP_RAM) + LENGTH(HEAP_RAM);
        __heap_end__ = .;
    } > HEAP_RAM
}
