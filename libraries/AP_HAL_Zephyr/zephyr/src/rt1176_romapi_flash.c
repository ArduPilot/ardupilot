/*
 * ROM-API flash primitives for mr_vmu_rt1176 parameter storage.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* WHY THE ROM API AND NOT ZEPHYR'S FLASH DRIVER: the Zephyr FlexSPI driver
 * reconfigures the controller the CPU is executing through under XIP, which
 * traps the core in the BootROM at pc=0x00223104. */

#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>
#include <string.h>

#include <fsl_romapi.h>

#include "rt1176_romapi_flash.h"

/* The FCB the BootROM already used to bring this flash up, reused as the ROM
 * API's config so the two agree on timing and geometry. */
#define RT1176_FCB_ADDR (RT1176_FLASH_MEMMAP_BASE + 0x400U)  // FCB offset in boot image
#define RT1176_FCB_TAG  0x42464346U   // "FCFB" LE, NXP FCB magic
#define ROM_API_INSTANCE 1U           // FlexSPI1; inst 0 = InvalidArgument

static flexspi_nor_config_t romapi_config;
static bool romapi_ready;

/* WHY A RUNTIME CHECK AND NOT A BUILD OPTION: the same image has to run on
 * silicon, where the BootROM flash API lives in the 256 KB ROM at 0x00200000,
 * and under Renode, where nothing is mapped there and every read returns zero.
 * Reading the API tree pointer the way ROM_API_Init() does (fsl_romapi.c)
 * tells the two apart at first use. Without a ROM the primitives below write
 * straight into the FlexSPI memory-mapped window, which the emulator backs
 * with plain RAM and treats with NOR semantics here (erase sets 0xFF, program
 * only clears bits). On silicon that window is read-only, and this path is
 * never taken. */
#define RT1176_ROM_BASE         0x00200000U
#define RT1176_ROM_END          0x00240000U   /* ROMCP: 256 KB */
#define RT1176_ROM_TREE_PTR_A0  0x0020001CU   /* MISC_DIFPROG == 0x001170a0 */
#define RT1176_ROM_TREE_PTR     0x0021001CU   /* every other revision */
static bool romapi_memmap;                    /* true: no ROM, write the window */

static bool bootrom_present(void)
{
	/* The same selection ROM_API_Init() makes, so we test the word it will use. */
	const uintptr_t slot = (ANADIG_MISC->MISC_DIFPROG == 0x001170a0U)
			       ? RT1176_ROM_TREE_PTR_A0 : RT1176_ROM_TREE_PTR;
	const uint32_t tree = *(const volatile uint32_t *)slot;

	return tree >= RT1176_ROM_BASE && tree < RT1176_ROM_END;
}

/* The ROM flash API is not reentrant, and since 2026-08-14 it has two callers,
 * so every entry point takes the same lock. */
static K_MUTEX_DEFINE(romapi_mutex);

/* Bring the BootROM's flash API up once, before any erase or program. */
static bool romapi_ensure_init(void)
{
	/* WHAT: run the body only once.
	 * WHY:  ROM_FLEXSPI_NorFlash_Init() reconfigures the live XIP controller.
	 *       Repeating it per call would be needless risk on a peripheral we
	 *       are fetching instructions through, and it is not idempotent in
	 *       any guaranteed way. */
	if (romapi_ready) {
		return true;
	}

	if (!bootrom_present()) {
		romapi_memmap = true;
		romapi_ready = true;
		printk("rt1176 flash: no BootROM at 0x%08x, writing the memory-mapped window (emulator)\n",
		       (unsigned)RT1176_ROM_BASE);
		return true;
	}

	const uint8_t *fcb = (const uint8_t *)(uintptr_t)RT1176_FCB_ADDR;
	uint32_t tag;

	/* Read the first word of the FlexSPI Configuration Block already in flash. */
	memcpy(&tag, fcb, sizeof(tag));
	if (tag != RT1176_FCB_TAG) {
		return false;
	}

	/* Take a private RAM copy of the FCB rather than pointing the ROM at the one in
	 * flash it is about to reprogram. */
	memcpy(&romapi_config, fcb, sizeof(romapi_config));

	/* WHAT: initialise the ROM API's own bookkeeping. */
	ROM_API_Init();

	/* Initialise the NOR driver behind the ROM API. */
	(void)ROM_FLEXSPI_NorFlash_Init(ROM_API_INSTANCE, &romapi_config);

	romapi_ready = true;
	return true;
}

int rt1176_flash_init(void)
{
	return romapi_ensure_init() ? 0 : -1;
}

/* Range erase, NOT EraseBlock: this part's FCB sets is_uniform_block_size, which
 * the block call does not honour. */
__ramfunc int rt1176_flash_erase(uint32_t offset, uint32_t size)
{
	if (!romapi_ensure_init()) {
		return -1;
	}

	k_mutex_lock(&romapi_mutex, K_FOREVER);
	if (romapi_memmap) {
		memset((void *)(uintptr_t)(RT1176_FLASH_MEMMAP_BASE + offset), 0xFF, size);
		k_mutex_unlock(&romapi_mutex);
		return 0;
	}

	/* Erase in CHUNKS, releasing interrupts between each: a whole-region erase with
	 * interrupts locked starves the watchdog feeder and the SoC resets mid-erase. */
	uint32_t done = 0;
	while (done < size) {
		const uint32_t chunk = MIN(RT1176_FLASH_ERASE_CHUNK, size - done);

		const unsigned int key = irq_lock();
		status_t status = ROM_FLEXSPI_NorFlash_Erase(ROM_API_INSTANCE,
							     &romapi_config,
							     offset + done, chunk);
		irq_unlock(key);

		if (status != kStatus_Success) {
			k_mutex_unlock(&romapi_mutex);
			return -1;
		}
		done += chunk;
	}
	k_mutex_unlock(&romapi_mutex);

	return 0;
}

/* Program exactly one page per ROM call; bytes the caller omits are left 0xff. */
__ramfunc int rt1176_flash_program(uint32_t offset, const uint8_t *data, uint32_t len)
{
	if (!romapi_ensure_init()) {
		return -1;
	}

	k_mutex_lock(&romapi_mutex, K_FOREVER);
	if (romapi_memmap) {
		/* NOR can only clear bits without an erase; keep the emulator honest. */
		uint8_t *dst = (uint8_t *)(uintptr_t)(RT1176_FLASH_MEMMAP_BASE + offset);
		for (uint32_t i = 0; i < len; i++) {
			dst[i] &= data[i];
		}
		k_mutex_unlock(&romapi_mutex);
		return 0;
	}
	while (len > 0U) {
		const uint32_t page_base = offset & ~(RT1176_FLASH_PAGE_SIZE - 1U);
		const uint32_t in_page = offset - page_base;
		const uint32_t this_page = MIN(len, RT1176_FLASH_PAGE_SIZE - in_page);

		uint8_t page[RT1176_FLASH_PAGE_SIZE];

		memset(page, 0xFF, sizeof(page));
		memcpy(&page[in_page], data, this_page);

		const unsigned int key = irq_lock();
		status_t status = ROM_FLEXSPI_NorFlash_ProgramPage(
			ROM_API_INSTANCE, &romapi_config, page_base,
			(const uint32_t *)page);
		irq_unlock(key);

		if (status != kStatus_Success) {
			k_mutex_unlock(&romapi_mutex);
			return -1;
		}

		offset += this_page;
		data += this_page;
		len -= this_page;
	}
	k_mutex_unlock(&romapi_mutex);

	return 0;
}
