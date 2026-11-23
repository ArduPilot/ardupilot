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
#include <zephyr/cache.h>
#include <zephyr/toolchain.h>
#include <string.h>

#include <fsl_romapi.h>

#include "rt1176_romapi_flash.h"
#include "ap_hooks.h"

/* The FCB the BootROM already used to bring this flash up, reused as the ROM
 * API's config so the two agree on timing and geometry. */
#define RT1176_FCB_ADDR (RT1176_FLASH_MEMMAP_BASE + 0x400U)  // FCB offset in boot image
#define RT1176_FCB_TAG  0x42464346U   // "FCFB" LE, NXP FCB magic
#define ROM_API_INSTANCE 1U           // FlexSPI1; inst 0 = InvalidArgument

/* WHY MIRROR THE SDK's PRIVATE STRUCTS: the erase/program primitives below are
 * __ramfunc because the flash they touch is the one the CPU fetches from. The
 * SDK's ROM_FLEXSPI_NorFlash_* entry points are ordinary functions that link
 * into XIP flash, so calling them put an XIP fetch INSIDE the window where the
 * ROM has taken the FlexSPI controller away - the fetch returns garbage, the
 * core branches through it and takes an illegal-EPSR UsageFault (Zephyr fatal
 * reason 35). Measured on silicon: repeated resets, every record naming the
 * AP_storage thread at priority 10.
 *
 * Those entry points only dereference the bootloader API tree, so we read the
 * tree ourselves, cache the NOR interface in RAM, and call through it. The
 * pointers in it aim at ROM (0x0020xxxx), which stays readable throughout.
 *
 * Layout copied verbatim from the SDK's fsl_romapi.c, which is in modules/ and
 * must not be modified. Its own comment on the tree reads "The order of
 * existing fields must not be changed", so this mirror is stable. */
typedef struct {
	uint32_t version;
	status_t (*init)(uint32_t instance, flexspi_nor_config_t *config);
	status_t (*page_program)(uint32_t instance, flexspi_nor_config_t *config,
				 uint32_t dst_addr, const uint32_t *src);
	status_t (*erase_all)(uint32_t instance, flexspi_nor_config_t *config);
	status_t (*erase)(uint32_t instance, flexspi_nor_config_t *config,
			  uint32_t start, uint32_t length);
	status_t (*read)(uint32_t instance, flexspi_nor_config_t *config,
			 uint32_t *dst, uint32_t start, uint32_t bytes);
	void (*clear_cache)(uint32_t instance);
	status_t (*xfer)(uint32_t instance, void *xfer);
	status_t (*update_lut)(uint32_t instance, uint32_t seqIndex,
			       const uint32_t *lutBase, uint32_t numberOfSeq);
	status_t (*get_config)(uint32_t instance, flexspi_nor_config_t *config,
			       serial_nor_config_option_t *option);
	status_t (*erase_sector)(uint32_t instance, flexspi_nor_config_t *config,
				 uint32_t address);
	status_t (*erase_block)(uint32_t instance, flexspi_nor_config_t *config,
				uint32_t address);
	const uint32_t reserved0;
	status_t (*wait_busy)(uint32_t instance, flexspi_nor_config_t *config,
			      bool isParallelMode, uint32_t address);
	const uint32_t reserved1[2];
} ap_rom_nor_iface_t;

typedef struct {
	void (*runBootloader)(void *arg);
	uint32_t version;                            /* standard_version_t, 4 bytes */
	const char *copyright;
	const ap_rom_nor_iface_t *flexSpiNorDriver;
	const uint32_t reserved[8];
} ap_rom_tree_t;

/* Breadcrumbs for the WDG line. These paths lock interrupts around a ROM call,
 * so if the SoC freezes inside one, nothing below interrupt level can report it
 * afterwards - the monitor thread never runs again. Written here and sampled by
 * Scheduler.cpp's monitor every 100 ms, so the LAST saved snapshot before a
 * watchdog reset says whether a flash operation was in progress. */
volatile uint32_t g_ap_flash_busy;   /* 1 = erase, 2 = program, 0 = idle */
volatile uint32_t g_ap_flash_ops;    /* completed operations, to spot a stuck one */

/* STICKY markers, written by this path itself rather than sampled by the monitor
 * every 100 ms. The monitor's snapshot is up to 100 ms stale, which is far too
 * coarse: at ~0.8 flash operations per second it catches one about 8% of the
 * time, so a "no flash operation in progress" reading proves nothing. These are
 * set on entry and only cleared on successful completion, so a freeze that
 * happens INSIDE an operation leaves them set for the next boot to read. */
/* Seeded once via g_ap_flash_seed. These are __noinit so they survive a reset,
 * which also means they start as RAM GARBAGE on the first boot after a flash -
 * enter minus exit then reads as a fixed nonsense offset (observed: 174). */
#define AP_FLASH_SEED_MAGIC 0x464c5348u   /* 'FLSH' */
__noinit volatile uint32_t g_ap_flash_seed;
__noinit volatile uint32_t g_ap_flash_enter;   /* ops started */
__noinit volatile uint32_t g_ap_flash_exit;    /* ops completed */
__noinit volatile uint32_t g_ap_flash_last_off;/* offset of the op in progress */
__noinit volatile uint32_t g_ap_flash_last_op; /* 1 = erase, 2 = program */

static flexspi_nor_config_t romapi_config;
static bool romapi_ready;
/* The NOR interface, cached in RAM so the __ramfunc paths never fetch from
 * flash to reach it. NULL until romapi_ensure_init() has validated it. */
static const ap_rom_nor_iface_t *rom_nor;

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

	/* Cache the NOR interface in RAM for the __ramfunc paths. Read the same
	 * tree slot ROM_API_Init() selected, and require every pointer we will
	 * call to land in ROM - a bad mirror offset would otherwise hand us a
	 * plausible-looking pointer and fault identically to the bug this
	 * replaces. */
	{
		const uintptr_t slot = (ANADIG_MISC->MISC_DIFPROG == 0x001170a0U)
				       ? RT1176_ROM_TREE_PTR_A0 : RT1176_ROM_TREE_PTR;
		const ap_rom_tree_t *tree =
			(const ap_rom_tree_t *)(uintptr_t)*(const volatile uint32_t *)slot;
		const ap_rom_nor_iface_t *nor = tree->flexSpiNorDriver;
		const uintptr_t e = (uintptr_t)(void *)nor->erase;
		const uintptr_t p = (uintptr_t)(void *)nor->page_program;

		if (e < RT1176_ROM_BASE || e >= RT1176_ROM_END ||
		    p < RT1176_ROM_BASE || p >= RT1176_ROM_END) {
			printk("rt1176 flash: ROM NOR iface implausible (erase=%08x program=%08x)\n",
			       (unsigned)e, (unsigned)p);
			return false;
		}
		rom_nor = nor;
	}

	if (g_ap_flash_seed != AP_FLASH_SEED_MAGIC) {
		/* First boot after a flash: clear the garbage so enter-minus-exit is a
		   real in-flight count from here on. Deliberately NOT done on every
		   boot - the difference has to survive a reset to be evidence. */
		g_ap_flash_enter = 0;
		g_ap_flash_exit = 0;
		g_ap_flash_last_op = 0;
		g_ap_flash_last_off = 0;
		g_ap_flash_seed = AP_FLASH_SEED_MAGIC;
	}

	romapi_ready = true;
	return true;
}

int rt1176_flash_init(void)
{
	return romapi_ensure_init() ? 0 : -1;
}

/* WHY A CONTROLLER RESET AFTER EVERY ROM ERASE AND PROGRAM: the ROM drives the
 * flash with IP commands and leaves the FlexSPI AHB buffer holding whatever it
 * had prefetched beforehand. The buffer is what serves instruction fetches
 * under XIP, so the first fetch after the ROM returns can be answered out of
 * stale bytes; the core then branches through them and takes an illegal-EPSR
 * UsageFault (Zephyr fatal reason 35). Whether it hits depends on which lines
 * the buffer happens to hold, which is why it presented as resets at
 * unpredictable uptimes naming whichever thread was next to run.
 *
 * NXP's answer is ROM_FLEXSPI_NorFlash_ClearCache(), which fsl_romapi.c places
 * in RAM (AT_QUICKACCESS_SECTION_CODE) and leaves for the caller to invoke -
 * the SDK never calls it itself. Its register sequence is mirrored here rather
 * than called, for the same reason the erase and program calls go through the
 * cached ROM interface: the SDK entry point is an ordinary function that links
 * into XIP, so calling it would put an XIP fetch inside the very window this
 * has to repair.
 *
 * Sequence copied from fsl_romapi.c ROM_FLEXSPI_NorFlash_ClearCache(), which
 * is in modules/ and must not be modified. */
__ramfunc static void romapi_flexspi_reset(void)
{
	FLEXSPI_Type *base = (ROM_API_INSTANCE == 2U) ? FLEXSPI2 : FLEXSPI1;

	base->MCR0 |= FLEXSPI_MCR0_SWRESET_MASK;
	while (base->MCR0 & FLEXSPI_MCR0_SWRESET_MASK) {
	}

	/* no instruction may be fetched until the reset has settled */
	__ISB();
}

/* Drop any cached view of a range that the flash underneath has just changed.
 * The FlexSPI software reset above clears the CONTROLLER's AHB buffer, but the
 * M7 has its own 32 KB data cache (CONFIG_DCACHE=y) and Storage's
 * _flash_read_data() reads this NOR through the memory-mapped window with a
 * plain memcpy. Without this, a read after a write can be answered from a line
 * cached before the write and return the old bytes - which for AP_FlashStorage
 * means reading back a sector header or record it has just replaced.
 *
 * Called after the mutex is released, where XIP is sound again, because
 * sys_cache_data_invd_range() is not itself RAM-resident. The mapped window is
 * never written by the CPU, so no line here can be dirty and nothing is lost. */
static void romapi_invalidate_mapped(uint32_t offset, uint32_t size)
{
	if (romapi_memmap || size == 0U) {
		return;   /* emulator path writes the window directly, nothing stale */
	}
	(void)sys_cache_data_invd_range(
		(void *)(uintptr_t)(RT1176_FLASH_MEMMAP_BASE + offset), size);
}

/* Range erase, NOT EraseBlock: this part's FCB sets is_uniform_block_size, which
 * the block call does not honour. */
__ramfunc int rt1176_flash_erase(uint32_t offset, uint32_t size)
{
	/* romapi_ready is a RAM flag: in steady state this returns without
	 * branching into romapi_ensure_init(), which links into XIP. */
	if (!romapi_ready && !romapi_ensure_init()) {
		return -1;
	}
	if (!romapi_memmap && rom_nor == NULL) {
		return -1;
	}

	k_mutex_lock(&romapi_mutex, K_FOREVER);
	g_ap_flash_busy = 1;
	g_ap_flash_last_op = 1;
	g_ap_flash_last_off = offset;
	g_ap_flash_enter++;
	ap_persistent_flash_mark(1, offset, g_ap_flash_enter - g_ap_flash_exit);
	if (romapi_memmap) {
		memset((void *)(uintptr_t)(RT1176_FLASH_MEMMAP_BASE + offset), 0xFF, size);
		g_ap_flash_busy = 0;
		k_mutex_unlock(&romapi_mutex);
		return 0;
	}

	/* Erase in CHUNKS, releasing interrupts between each: a whole-region erase with
	 * interrupts locked starves the watchdog feeder and the SoC resets mid-erase. */
	uint32_t done = 0;
	while (done < size) {
		const uint32_t chunk = MIN(RT1176_FLASH_ERASE_CHUNK, size - done);

		const unsigned int key = irq_lock();
		status_t status = rom_nor->erase(ROM_API_INSTANCE,
						 &romapi_config,
						 offset + done, chunk);
		/* WAIT FOR THE DEVICE, still inside the lock. erase() issuing the
		   command is not the same as the NOR having finished it, which is
		   why the ROM exposes wait_busy() separately. Without this the
		   lock is released while the flash is still erasing, and the very
		   next instruction fetch - the first interrupt to be taken, USB
		   being the frequent one - reaches a busy device. That read does
		   not fault, it stalls the bus, so the core simply stops with
		   interrupts enabled and no thread able to run: the monitor never
		   feeds the watchdog, the watchdog resets the SoC, and nothing is
		   recorded because nothing executed. */
		if (status == kStatus_Success && rom_nor->wait_busy != NULL) {
			status = rom_nor->wait_busy(ROM_API_INSTANCE, &romapi_config,
						    false, offset + done);
		}
		/* inside the lock: nothing may fetch from XIP between the ROM
		   call and the reset that makes XIP trustworthy again */
		romapi_flexspi_reset();
		irq_unlock(key);

		if (status != kStatus_Success) {
			g_ap_flash_busy = 0;
			k_mutex_unlock(&romapi_mutex);
			return -1;
		}
		done += chunk;
	}
	g_ap_flash_busy = 0;
	g_ap_flash_ops++;
	g_ap_flash_exit++;
	ap_persistent_flash_mark(1, offset, g_ap_flash_enter - g_ap_flash_exit);
	k_mutex_unlock(&romapi_mutex);
	romapi_invalidate_mapped(offset, size);

	return 0;
}

/* Program exactly one page per ROM call; bytes the caller omits are left 0xff. */
__ramfunc int rt1176_flash_program(uint32_t offset, const uint8_t *data, uint32_t len)
{
	/* romapi_ready is a RAM flag: in steady state this returns without
	 * branching into romapi_ensure_init(), which links into XIP. */
	if (!romapi_ready && !romapi_ensure_init()) {
		return -1;
	}
	if (!romapi_memmap && rom_nor == NULL) {
		return -1;
	}

	const uint32_t prog_start = offset;
	const uint32_t prog_len = len;

	k_mutex_lock(&romapi_mutex, K_FOREVER);
	g_ap_flash_busy = 2;
	g_ap_flash_last_op = 2;
	g_ap_flash_last_off = offset;
	g_ap_flash_enter++;
	ap_persistent_flash_mark(2, offset, g_ap_flash_enter - g_ap_flash_exit);
	if (romapi_memmap) {
		/* NOR can only clear bits without an erase; keep the emulator honest. */
		uint8_t *dst = (uint8_t *)(uintptr_t)(RT1176_FLASH_MEMMAP_BASE + offset);
		for (uint32_t i = 0; i < len; i++) {
			dst[i] &= data[i];
		}
		g_ap_flash_busy = 0;
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
		status_t status = rom_nor->page_program(
			ROM_API_INSTANCE, &romapi_config, page_base,
			(const uint32_t *)page);
		/* same reason as the erase path: the program has to have LANDED
		   before the lock is released and something fetches from here. */
		if (status == kStatus_Success && rom_nor->wait_busy != NULL) {
			status = rom_nor->wait_busy(ROM_API_INSTANCE, &romapi_config,
						    false, page_base);
		}
		romapi_flexspi_reset();
		irq_unlock(key);

		if (status != kStatus_Success) {
			g_ap_flash_busy = 0;
			k_mutex_unlock(&romapi_mutex);
			return -1;
		}

		offset += this_page;
		data += this_page;
		len -= this_page;
	}
	g_ap_flash_busy = 0;
	g_ap_flash_ops++;
	g_ap_flash_exit++;
	ap_persistent_flash_mark(2, prog_start, g_ap_flash_enter - g_ap_flash_exit);
	k_mutex_unlock(&romapi_mutex);
	romapi_invalidate_mapped(prog_start, prog_len);

	return 0;
}
