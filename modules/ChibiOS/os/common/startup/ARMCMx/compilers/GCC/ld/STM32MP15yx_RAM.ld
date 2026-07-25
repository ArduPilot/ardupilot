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
 * STM32MP15yx memory setup.
 * NOTE: Vectors in RETRAM, M4 application in SRAM1..SRAM4 combined, SYSRAM
 *       not used.
 */
MEMORY
{
    flash0 (rx) : org = 0x08000000, len = 0
    flash1 (rx) : org = 0x00000000, len = 0
    flash2 (rx) : org = 0x00000000, len = 0
    flash3 (rx) : org = 0x00000000, len = 0
    flash4 (rx) : org = 0x00000000, len = 0
    flash5 (rx) : org = 0x00000000, len = 0
    flash6 (rx) : org = 0x00000000, len = 0
    flash7 (rx) : org = 0x00000000, len = 0
    ram0   (wx) : org = 0x00000000, len = 64k   /* RETRAM */
    ram1   (wx) : org = 0x10000000, len = 128k  /* SRAM1 */
    ram2   (wx) : org = 0x10020000, len = 128k  /* SRAM2 */
    ram3   (wx) : org = 0x10040000, len = 64k   /* SRAM3 */
    ram4   (wx) : org = 0x10050000, len = 64k   /* SRAM4 */
    ram5   (wx) : org = 0x10000000, len = 384k  /* SRAM1+SRAM2+SRAM3+SRAM4 */
    ram6   (wx) : org = 0x2FFC0000, len = 256k  /* SYSRAM */
    ram7   (wx) : org = 0x00000000, len = 0
}

/* For each data/text section two region are defined, a virtual region
   and a load region (_LMA suffix).*/

/* Flash region to be used for exception vectors.*/
REGION_ALIAS("VECTORS_FLASH", ram0);
REGION_ALIAS("VECTORS_FLASH_LMA", ram0);

/* Flash region to be used for constructors and destructors.*/
REGION_ALIAS("XTORS_FLASH", ram5);
REGION_ALIAS("XTORS_FLASH_LMA", ram5);

/* Flash region to be used for code text.*/
REGION_ALIAS("TEXT_FLASH", ram5);
REGION_ALIAS("TEXT_FLASH_LMA", ram5);

/* Flash region to be used for read only data.*/
REGION_ALIAS("RODATA_FLASH", ram5);
REGION_ALIAS("RODATA_FLASH_LMA", ram5);

/* Flash region to be used for various.*/
REGION_ALIAS("VARIOUS_FLASH", ram5);
REGION_ALIAS("VARIOUS_FLASH_LMA", ram5);

/* Flash region to be used for RAM(n) initialization data.*/
REGION_ALIAS("RAM_INIT_FLASH_LMA", ram5);

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts.*/
REGION_ALIAS("MAIN_STACK_RAM", ram5);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram5);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);
REGION_ALIAS("DATA_RAM_LMA", ram5);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram5);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram5);

/* Generic rules inclusion.*/
INCLUDE rules.ld
