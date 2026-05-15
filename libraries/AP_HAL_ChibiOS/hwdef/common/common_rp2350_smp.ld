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

    This is the RP2350 SMP variant of common.ld.  It adds the core1 MSP
    and PSP stack sections required when CH_CFG_SMP_MODE=TRUE and
    RP_CORE1_START=TRUE.  Both core1 stacks are placed in ram0 (the main
    520 KB SRAM).
*/

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts*/
REGION_ALIAS("MAIN_STACK_RAM", ram0);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram0);

/* Core 1 stacks — both placed in ram0 (RP2350 main SRAM).
   The RP2350 SCRATCH_X/Y per-core banks are not mapped in ldscript.ld;
   using ram0 is functionally equivalent for our use case. */
REGION_ALIAS("C1_MAIN_STACK_RAM", ram0);
REGION_ALIAS("C1_PROCESS_STACK_RAM", ram0);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram0);

__ram0_start__          = ORIGIN(ram0);
__ram0_size__           = LENGTH(ram0);
__ram0_end__            = __ram0_start__ + __ram0_size__;
/*
 * RP2350 __RAMFUNC__ guard.
 * Older fixed-size guards were conservative but could trip even when total SRAM headroom was still healthy.
 */
PROVIDE(__rp2350_min_heap__ = 0x00020000);          /* 128 KB minimum heap */

ENTRY(Reset_Handler)
/*
 * Force rp2350_imagedef.o to be pulled from libch.a so the PICOBIN IMAGE_DEF block lands in flash.
 * On non-RP2350 boards the symbol simply doesn't exist and this directive is silently ignored.
 */
EXTERN(__embedded_block)

SECTIONS
{
    . = 0;
    _text = .;

/*
 * Cortex-M33 VTOR requires bits[6:0] = 0 (128-byte alignment).
 * Placing startup first means the vector table is at flash_base+0x0000,
 * which satisfies the constraint without padding.
 */
    startup : ALIGN(4) SUBALIGN(16)
    {
        __vectors_base__ = .;
        KEEP(*(.vectors))
    } > flash

/*
 * RP2350 PICOBIN image definition block.
 * The bootrom scans the first 4KB of flash for PICOBIN_BLOCK_MARKER_START (0xffffded3).
 * Placing it after the vector table (~272 bytes) keeps it well within that window.
 */
    embedded_block : ALIGN(4)
    {
        KEEP(*(.embedded_block))
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
        KEEP(*(.apsec_data));
        KEEP(*(.app_descriptor));
        *(.text)
        *(.rodata)
        *(.rodata.*)
    } > flash

    .data : ALIGN(4)
    {
        . = ALIGN(4);
        PROVIDE(_textdata = LOADADDR(.data));
        PROVIDE(_data = .);
        __textdata_base__ = LOADADDR(.data);
        __data_base__ = .;
        *(.data)
        *(.data.*)
        . = ALIGN(16);
        __ramfunc_start__ = .;
        *(.ramtext)
        *(.ramtext.*)
        INCLUDE rp2350_ramfunc2_sections.ld
        . = ALIGN(16);
        __ramfunc_end__ = .;
        __ramfunc_size__ = __ramfunc_end__ - __ramfunc_start__;
        . = ALIGN(4);
        PROVIDE(_edata = .);
        __data_end__ = .;
    } > DATA_RAM AT > flash

    .text_rest : ALIGN(4) SUBALIGN(4)
    {
        *(.text.*)
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

    /* Core 1 MSP exception stack — same size as core0 MSP. */
    __c1_main_stack_size__    = __main_stack_size__;
    /* Core1 PSP: 16 KB to give EKF CovariancePrediction (large float-heavy
       function) sufficient headroom.  Core0 uses 7 KB __process_stack_size__;
       core1 needs more because CovariancePrediction runs on the PSP here. */
    __c1_process_stack_size__ = 0x4000;

    .c1_mstack (NOLOAD) :
    {
        . = ALIGN(8);
        __c1_main_stack_base__ = .;
        . += __c1_main_stack_size__;
        . = ALIGN(8);
        __c1_main_stack_end__ = .;
    } > C1_MAIN_STACK_RAM

    /* Core 1 PSP process/thread stack. */
    .c1_pstack (NOLOAD) :
    {
        . = ALIGN(8);
        __c1_process_stack_base__    = .;
        __c1_main_thread_stack_base__ = .;
        . += __c1_process_stack_size__;
        . = ALIGN(8);
        __c1_process_stack_end__     = .;
        __c1_main_thread_stack_end__ = .;
    } > C1_PROCESS_STACK_RAM

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
        __heap_size__ = __heap_end__ - __heap_base__;

        /* Explicit SRAM accounting for link-time diagnostics:
           - allowed: max static SRAM usage while preserving min heap
           - used:    current static SRAM usage before heap
           - over:    how far static usage exceeds allowed budget */
        __rp2350_sram_allowed__ = LENGTH(HEAP_RAM) - __rp2350_min_heap__;
        __rp2350_sram_used__ = __heap_base__ - ORIGIN(HEAP_RAM);
        __rp2350_sram_over_limit__ = (__rp2350_sram_used__ > __rp2350_sram_allowed__) ?
            (__rp2350_sram_used__ - __rp2350_sram_allowed__) : 0;
    } > HEAP_RAM

    /* The crash log uses the unused part of a flash section.*/
    .crash_log (NOLOAD) :
    {
        . = ALIGN(32);
        __crash_log_base__ = .;
        . = ORIGIN(flash) + LENGTH(flash);
        __crash_log_end__ = .;
    } > flash

        ASSERT(__rp2350_sram_used__ <= __rp2350_sram_allowed__,
            "RP2350: SRAM over limit after RAMFUNC placement; see __rp2350_sram_allowed__, __rp2350_sram_used__, __rp2350_sram_over_limit__")

}
