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

/* RAM region to be used for fast code. */
REGION_ALIAS("FASTCODE_RAM", ram1)

/* stack areas are configured to be in AXI RAM (ram1) to ensure the SSBL will load the image */
/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts*/
REGION_ALIAS("MAIN_STACK_RAM", ram1);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram1);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram0);

__ram0_start__          = ORIGIN(ram0);
__ram0_size__           = LENGTH(ram0);
__ram0_end__            = __ram0_start__ + __ram0_size__;

/* AXI */
__ram1_start__          = ORIGIN(ram1);
__ram1_size__           = LENGTH(ram1);
__ram1_end__            = __ram1_start__ + __ram1_size__;

/* DTCM */
__ram2_start__          = ORIGIN(ram2);
__ram2_size__           = LENGTH(ram2);
__ram2_end__            = __ram2_start__ + __ram2_size__;

/* ITCM */
__instram_start__      = ORIGIN(instram);
__instram_size__       = LENGTH(instram);
__instram_end__        = __instram_start__ + __instram_size__;

ENTRY(Reset_Handler)

SECTIONS
{
    . = 0;
    _text = .;

    startup : ALIGN(16) SUBALIGN(16)
    {
        KEEP(*(.vectors))
    } > default_flash

    constructors : ALIGN(4) SUBALIGN(4)
    {
        __init_array_base__ = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end__ = .;
    } > default_flash

    destructors : ALIGN(4) SUBALIGN(4)
    {
        __fini_array_base__ = .;
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        __fini_array_end__ = .;
    } > default_flash

    /* INSTRUCTION_RAM area is fast-access ITCM used for RAM-based code, 64k on H7 */
    .fastramfunc : ALIGN(4) SUBALIGN(4)
    {
        . = ALIGN(4);
        __instram_init_text__ = LOADADDR(.fastramfunc);
        __instram_init__ = .;
        /* ChibiOS won't boot unless these are excluded */
        EXCLUDE_FILE (*vectors.o *crt0_v7m.o *crt1.o)
        /* performance critical sections of ChibiOS */
        *libch.a:ch*.*(.text*)
        *libch.a:nvic.*(.text*)
        *libch.a:bouncebuffer.*(.text*)
        *libch.a:stm32_util.*(.text*)
        *libch.a:stm32_dma.*(.text*)
        *libch.a:memstreams.*(.text*)
        *libch.a:malloc.*(.text*)
        *libch.a:hrt.*(.text*)
        *libch.a:hal*.*(.text*)
        /* a selection of performance critical functions driven CPUInfo results */
        lib/lib*.a:Semaphores.*(.text*)
        lib/lib*.a:AP_Math.*(.text*)
        lib/lib*.a:vector3.*(.text*)
        lib/lib*.a:matrix3.*(.text*)
        /* only used on debug builds */
        *libg_nano.a:*memset*(.text*)
        *libg_nano.a:*memcpy*(.text*)
        *libm.a:*(.text*)
        /* For some reason boards won't boot if libc is in RAM, but will with debug on */
        /**libc_nano.a:*(.text* .rodata*)
        *libstdc++_nano.a:(.text* .rodata*)*/
        *(.fastramfunc)
        . = ALIGN(4);
        __instram_end__ = .;
    } > instram AT > default_flash

    /* FLASH_RAM area is primarily used for RAM-based code and data, 256k allocation on H7 */
    .ramfunc : ALIGN(4) SUBALIGN(4)
    {
        . = ALIGN(4);
        __ram1_init_text__ = LOADADDR(.ramfunc);
        __ram1_init__ = .;
        /* ChibiOS won't boot unless these are excluded */
        EXCLUDE_FILE (*vectors.o *crt0_v7m.o *crt1.o)
        /*libch.a:*(.text* .rodata* .glue_7t .glue_7 .gcc*)*/
        /* a selection of larger performance critical functions */
        lib/lib*.a:*Filter.*(.text* .rodata*)
        lib/lib*.a:*Filter2p.*(.text* .rodata*)
        lib/lib*.a:SPIDevice.*(.text* .rodata*)
        lib/lib*.a:I2CDevice.*(.text* .rodata*)
        lib/lib*.a:Util.*(.text* .rodata*)
        lib/lib*.a:Device.*(.text* .rodata*)
        lib/lib*.a:Scheduler.*(.text* .rodata*)
        lib/lib*.a:shared_dma.*(.text* .rodata*)
        lib/lib*.a:RingBuffer.*(.text* .rodata*)
        lib/lib*.a:crc.*(.text* .rodata*)
        lib/lib*.a:matrixN.*(.text* .rodata*)
        lib/lib*.a:matrix_alg.*(.text* .rodata*)
        lib/lib*.a:AP_NavEKF3*.*(.text* .rodata*)
        lib/lib*.a:AP_NavEKF_*.*(.text* .rodata*)
        lib/lib*.a:EKF*.*(.text* .rodata*)
        lib/lib*.a:AP_Compass_Backend.*(.text* .rodata*)
        lib/lib*.a:AP_RCProtocol_Backend.*(.text* .rodata*)
        lib/lib*.a:AP_RCProtocol.*(.text* .rodata*)
        lib/lib*.a:AP_RCProtocol_CRSF.*(.text* .rodata*)
        lib/lib*.a:AP_RCTelemetry.*(.text* .rodata*)
        lib/lib*.a:AP_CRSF_Telem.*(.text* .rodata*)
        lib/lib*.a:vector2.*(.text* .rodata*)
        lib/lib*.a:quaternion.*(.text* .rodata*)
        lib/lib*.a:polygon.*(.text* .rodata*)
        lib/lib*.a:flash.*(.text* .rodata*) /* flash ops in RAM so that both banks can be erased */
        /* uncomment these to test CPUInfo in FLASH_RAM */
        /*Tools/CPUInfo/CPUInfo.*(.text* .rodata*)
        Tools/CPUInfo/EKF_Maths.*(.text* .rodata*)*/
        *(.ramfunc*)
        . = ALIGN(4);
    } > ram1 AT > default_flash

    .ram1 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        __ram1_clear__ = .;
        . = ALIGN(4);
        __ram1_noinit__ = .;
        *(.ram1*)
        . = ALIGN(4);
        __ram1_free__ = .;
    } > ram1

    /* DATA_RAM area is DTCM primarily used for RAM-based data, e.g. vtables */
    .ramdata : ALIGN(4)
    {
        . = ALIGN(4);
        __ram2_init_text__ = LOADADDR(.ramdata);
        __ram2_init__ = .;
        /* ChibiOS won't boot unless these are excluded */
        EXCLUDE_FILE (*vectors.o *crt0_v7m.o *crt1.o)
        /* performance critical sections of ChibiOS */
        *libch.a:ch*.*(.rodata*)
        *libch.a:nvic.*(.rodata*)
        *libch.a:bouncebuffer.*(.rodata*)
        *libch.a:stm32_util.*(.rodata*)
        *libch.a:stm32_dma.*(.rodata*)
        *libch.a:memstreams.*(.rodata*)
        *libch.a:malloc.*(.rodata*)
        *libch.a:hrt.*(.rodata*)
        *libch.a:hal*.*(.rodata*)
        /* a selection of performance critical functions driven CPUInfo results */
        lib/lib*.a:Semaphores.*(.rodata*)
        lib/lib*.a:AP_Math.*(.rodata*)
        lib/lib*.a:vector3.*(.rodata*)
        lib/lib*.a:matrix3.*(.rodata*)
        *libm.a:*(.rodata*)
        *(.ramdata*)
        . = ALIGN(4);
    } > ram2 AT > default_flash

    .ram2 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        __ram2_clear__ = .;
        . = ALIGN(4);
        __ram2_noinit__ = .;
        *(.ram2*)
        . = ALIGN(4);
        __ram2_free__ = .;
    } > ram2

    .text : ALIGN(4) SUBALIGN(4)
    {
        /* we want app_descriptor near the start of flash so a false
           positive isn't found by the bootloader (eg. ROMFS) */
        KEEP(*libch.a:vectors.o);
        KEEP(*libch.a:crt0_v7m.o);
        KEEP(*(.app_descriptor));
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
    } > default_flash

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > default_flash

    .ARM.exidx : {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
     } > default_flash

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > default_flash

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > default_flash

    .textalign : ONLY_IF_RO
    {
        . = ALIGN(8);
    } > default_flash

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
    } > DATA_RAM AT > default_flash

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
    } > ram0 AT > default_flash

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
        . = ORIGIN(default_flash) + LENGTH(default_flash);
        __crash_log_end__ = .;
    } > default_flash
}
