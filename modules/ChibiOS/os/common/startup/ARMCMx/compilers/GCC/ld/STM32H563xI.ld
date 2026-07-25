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
 * STM32H563xI memory setup.
 */
MEMORY
{
    flash0 (rx) : org = 0x08000000, len = 2M    /* Flash NS */
    flash1 (rx) : org = 0x0C000000, len = 2M    /* Flash NSC */
    flash2 (rx) : org = 0x00000000, len = 0
    flash3 (rx) : org = 0x00000000, len = 0
    flash4 (rx) : org = 0x00000000, len = 0
    flash5 (rx) : org = 0x00000000, len = 0
    flash6 (rx) : org = 0x00000000, len = 0
    flash7 (rx) : org = 0x00000000, len = 0
    ram0   (wx) : org = 0x20000000, len = 640k  /* SRAM1+SRAM2+SRAM3 NS */
    ram1   (wx) : org = 0x20000000, len = 256k  /* SRAM1 NS */
    ram2   (wx) : org = 0x20040000, len = 64k   /* SRAM2 NS */
    ram3   (wx) : org = 0x20050000, len = 320k  /* SRAM3 NS */
    ram4   (wx) : org = 0x30000000, len = 640k  /* SRAM1+SRAM2+SRAM3 NSC */
    ram5   (wx) : org = 0x30000000, len = 256k  /* SRAM1 NSC */
    ram6   (wx) : org = 0x30040000, len = 64k   /* SRAM2 NSC */
    ram7   (wx) : org = 0x30050000, len = 320k  /* SRAM3 NSC */
}

/* For each data/text section two region are defined, a virtual region
   and a load region (_LMA suffix).*/

/* Flash region to be used for exception vectors.*/
REGION_ALIAS("VECTORS_FLASH", flash0);
REGION_ALIAS("VECTORS_FLASH_LMA", flash0);

/* Flash region to be used for constructors and destructors.*/
REGION_ALIAS("XTORS_FLASH", flash0);
REGION_ALIAS("XTORS_FLASH_LMA", flash0);

/* Flash region to be used for code text.*/
REGION_ALIAS("TEXT_FLASH", flash0);
REGION_ALIAS("TEXT_FLASH_LMA", flash0);

/* Flash region to be used for read only data.*/
REGION_ALIAS("RODATA_FLASH", flash0);
REGION_ALIAS("RODATA_FLASH_LMA", flash0);

/* Flash region to be used for various.*/
REGION_ALIAS("VARIOUS_FLASH", flash0);
REGION_ALIAS("VARIOUS_FLASH_LMA", flash0);

/* Flash region to be used for RAM(n) initialization data.*/
REGION_ALIAS("RAM_INIT_FLASH_LMA", flash0);

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts.*/
REGION_ALIAS("MAIN_STACK_RAM", ram0);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram0);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);
REGION_ALIAS("DATA_RAM_LMA", flash0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram0);

/* RAM region to be used for DMA3 link structures.*/
REGION_ALIAS("DMA3_RAM", ram0);

SECTIONS
{
    /* Special section for GPDMA links, it must not exceed 64kB in size.*/
    .dma3 (NOLOAD) : ALIGN(16)
    {
        __dma3_base__ = .;
        *(.dma3)
        *(.dma3.*)
        *(.bss.__dma3_*)
        . = ALIGN(4);
        __dma3_end__ = .;
    } > DMA3_RAM
}

/* Generic rules inclusion.*/
INCLUDE rules.ld
