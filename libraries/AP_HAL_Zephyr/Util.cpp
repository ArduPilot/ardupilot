/*
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
 *
 * Code by @davidbuzz and Claude
 */

#include "Util.h"
#if defined(CONFIG_HWINFO)
#include <zephyr/drivers/hwinfo.h>
#endif
#include <AP_Common/ExpandingString.h>   // thread_info() writes into one
/* Diagnostic-only layering exception: the HAL reaching up into AP_Scheduler.
   Justified because this exists solely to render @SYS/tasks.txt somewhere the
   debugger can reach it, and it compiles out with the diagnostic. */
#include <AP_Scheduler/AP_Scheduler.h>
#if AP_FILESYSTEM_FATFS_ENABLED
/* For the SD listing + marker write in ap_sysinfo_capture(). Also brings in
   <fcntl.h> for O_WRONLY/O_CREAT/O_TRUNC and struct dirent. */
#include <AP_Filesystem/AP_Filesystem.h>
#endif

/* @SYS/threads.txt without MAVFTP: rendered into a plain global so it can be read
 * over SWD when MAVFTP will not serve it. */
#define AP_SYSINFO_BUF_SIZE 8192
extern "C" {
char g_ap_sysinfo[AP_SYSINFO_BUF_SIZE];
volatile uint32_t g_ap_sysinfo_len;
volatile uint32_t g_ap_sysinfo_seq;
}

/* extern "C" deliberately: the caller is Zephyr::Scheduler::_io_thread_fn, and
   a plain `extern void ap_sysinfo_capture();` declared inside that namespace
   binds to Zephyr::ap_sysinfo_capture(), which this definition is not
   (`using namespace Zephyr;` does not place definitions INTO the namespace).
   C linkage sidesteps the mismatch. */
extern "C" void ap_sysinfo_capture(void)
{
    extern const AP_HAL::HAL &hal;
    ExpandingString str(g_ap_sysinfo, sizeof(g_ap_sysinfo));

    /* @SYS/threads.txt - per-thread CPU LOAD% and stack high-water. */
    hal.util->thread_info(str);

    /* @SYS/tasks.txt - per-scheduler-task timing, in the same TasksV2 format ChibiOS
     * emits so captures from the two HALs compare column for column. */
    static uint8_t captures;
    if (captures < 255) {
        captures++;
    }
    if (captures == 2) {
        AP::scheduler().update_logging();
    }
    AP::scheduler().task_info(str);

    /* SD card contents. There is no other way to see them on this board:
       MAVFTP does not work here, and the console is unreliable. Listing the
       filesystem into the same SWD-readable buffer turns "did anything
       actually get written to the card" into a direct readout. */
#if AP_FILESYSTEM_FATFS_ENABLED
    str.printf("SDCARD\n");
    /* Positive write test, once. Proves the card is WRITEABLE, independently of
       whether AP_Logger ever starts a log - on this board most scheduled tasks
       are starved, so waiting for a log file would confound "SD is broken" with
       "the logger never ran". */
    static bool wrote_marker;
    if (!wrote_marker) {
        wrote_marker = true;
        AP::FS().mkdir("/APM");
        int fd = AP::FS().open("/APM/ZEPHYR.TXT", O_WRONLY | O_CREAT | O_TRUNC);
        if (fd >= 0) {
            char msg[96];
            const int n = snprintf(msg, sizeof(msg),
                                   "AP_HAL_Zephyr mr_vmu_rt1176 SD write OK, uptime %u ms\n",
                                   (unsigned)AP_HAL::millis());
            const int32_t w = AP::FS().write(fd, msg, n);
            AP::FS().close(fd);
            str.printf("  marker write: fd=%d wrote=%ld of %d\n", fd, (long)w, n);
        } else {
            str.printf("  marker write FAILED: open() = %d\n", fd);
        }
    }
    /* Mount status only. An earlier version listed /APM and /APM/LOGS, which cost a
     * directory walk every time the file was read. */
    struct stat st;
    const bool have_apm = (AP::FS().stat("/APM", &st) == 0);
    str.printf("  /APM %s\n", have_apm ? "present" : "NOT present (card not mounted?)");
    /* Report the marker EVERY capture, not just the one that wrote it - the
       write happens once and its result scrolled out of the buffer long before
       anyone reads it. A non-zero size here is the proof that this HAL wrote to
       the physical card; /APM alone is not, since a card used in another
       autopilot already has that directory. */
    struct stat mst;
    if (AP::FS().stat("/APM/ZEPHYR.TXT", &mst) == 0) {
        str.printf("  /APM/ZEPHYR.TXT present, %ld bytes  <== WRITE VERIFIED\n",
                   (long)mst.st_size);
    } else {
        str.printf("  /APM/ZEPHYR.TXT MISSING - no successful write yet\n");
    }
#endif

    /* length last, then seq: a reader that samples seq, reads, and re-samples
       seq can detect a capture that landed mid-read. */
    g_ap_sysinfo_len = str.get_length();
    g_ap_sysinfo_seq++;
}

// CONFIG_SYS_HEAP_RUNTIME_STATS is injected by -imacros autoconf.h for all
// Zephyr board builds.  __ZEPHYR__ is NOT defined in autoconf.h so cannot be
// used as a guard here.  available_memory is out-of-line so it is compiled
// once with the correct CONFIG_* macros visible, ensuring the reference to
// malloc_runtime_stats_get survives --gc-sections.
#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS) && !defined(CONFIG_EXTERNAL_LIBC)
#include <zephyr/sys/mem_stats.h>
extern "C" int malloc_runtime_stats_get(struct sys_memory_stats *stats);
#endif

using namespace Zephyr;

uint32_t Util::available_memory(void)
{
#if defined(CONFIG_EXTERNAL_LIBC)
    /* Host libc (native_sim): Zephyr's common-libc heap does not exist, so
       malloc_runtime_stats_get() never links. Report a fixed generous figure
       - the host has effectively unbounded memory and AP's low-memory arming
       checks should never trip in simulation. */
    return 256 * 1024;
#elif defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
    struct sys_memory_stats stats;
    if (malloc_runtime_stats_get(&stats) == 0) {
        return (uint32_t)stats.free_bytes;
    }
    return 0;
#else
    return 0;
#endif
}

#if HAL_ENABLE_THREAD_STATISTICS
/* Display thread stack usage and CPU load as text for @SYS/threads.txt. */
struct thread_info_ctx {
    ExpandingString *str;
    uint64_t total_cycles;
};

static void thread_info_accumulate(const struct k_thread *thread, void *user_data)
{
#ifdef CONFIG_THREAD_RUNTIME_STATS
    auto *ctx = (thread_info_ctx *)user_data;
    k_thread_runtime_stats_t stats;
    if (k_thread_runtime_stats_get((k_tid_t)thread, &stats) == 0) {
        ctx->total_cycles += stats.execution_cycles;
    }
#else
    (void)thread;
    (void)user_data;
#endif
}

static void thread_info_print(const struct k_thread *thread, void *user_data)
{
    auto *ctx = (thread_info_ctx *)user_data;
    const char *name = k_thread_name_get((k_tid_t)thread);
    if (name == nullptr) {
        name = "unnamed";
    }

    size_t unused = 0;
    size_t total_stack = 0;
    void *stack_start = nullptr;
#ifdef CONFIG_THREAD_STACK_INFO
    total_stack = thread->stack_info.size;
    stack_start = (void *)thread->stack_info.start;
    if (k_thread_stack_space_get(thread, &unused) != 0) {
        unused = 0;
    }
#endif

#ifdef CONFIG_THREAD_RUNTIME_STATS
    k_thread_runtime_stats_t stats;
    if (ctx->total_cycles > 0 &&
        k_thread_runtime_stats_get((k_tid_t)thread, &stats) == 0) {
        ctx->str->printf("%-13.13s PRI=%3d sp=%p STACK=%4u/%4u LOAD=%4.1f%%\n",
                         name, (int)k_thread_priority_get((k_tid_t)thread),
                         stack_start,
                         (unsigned)unused, (unsigned)total_stack,
                         100.0f * float(stats.execution_cycles) / float(ctx->total_cycles));
        return;
    }
#endif
    ctx->str->printf("%-13.13s PRI=%3d sp=%p STACK=%4u/%4u\n",
                     name, (int)k_thread_priority_get((k_tid_t)thread),
                     stack_start,
                     (unsigned)unused, (unsigned)total_stack);
}

void Util::thread_info(ExpandingString &str)
{
    thread_info_ctx ctx { &str, 0 };

    // Two passes: the LOAD% denominator is the sum over all threads, so it has to be
    // totalled before any row can be rendered.
    k_thread_foreach_unlocked(thread_info_accumulate, &ctx);

    // header allowing machine parsers to determine format - same tag ChibiOS
    // emits, so existing parsers need no Zephyr-specific case
    str.printf("ThreadsV2\n");

    k_thread_foreach_unlocked(thread_info_print, &ctx);

    /* ExpandingString grows via mem_realloc()->malloc(), and on failure it sets
       a flag and silently keeps going. Without this line a truncated or empty
       dump is indistinguishable from "this board has no threads", which is a
       genuinely misleading thing for a diagnostic to report. Say so instead. */
    if (str.has_failed_allocation()) {
        str.printf("WARNING: out of memory - thread list above is TRUNCATED\n");
    }
}
#endif // HAL_ENABLE_THREAD_STATISTICS

/* Report whether the last reset was a watchdog reset, for Util::was_watchdog_reset(). */
bool Util::was_watchdog_reset() const
{
#if defined(CONFIG_HWINFO)
    static bool checked;
    static bool was_wdt;
    if (!checked) {
        checked = true;
        uint32_t cause = 0;
        if (hwinfo_get_reset_cause(&cause) == 0) {
            was_wdt = (cause & RESET_WATCHDOG) != 0;
            /* clear so the NEXT boot reports its own cause, not an accumulation
               - SRSR latches every source until explicitly cleared (p.2093). */
            (void)hwinfo_clear_reset_cause();
        }
    }
    return was_wdt;
#else
    return false;
#endif
}

/* Factory board unique ID - see Util.h's override comment. */
bool Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
#if defined(CONFIG_HWINFO)
    uint8_t id[16] {};
    const ssize_t n = hwinfo_get_device_id(id, sizeof(id));
    if (n <= 0) {
        return false;
    }
    len = MIN((uint8_t)n, len);
    memcpy(buf, id, len);
    return true;
#else
    (void)buf; (void)len;
    return false;
#endif
}

bool Util::get_system_id(char buf[50])
{
    uint8_t serialid[12] {};
    uint8_t len = sizeof(serialid);
    if (!get_system_id_unformatted(serialid, len)) {
        return false;
    }
    /* Same rendering as AP_HAL_ChibiOS::Util::get_system_id() (which in
       turn matches HAL_PX4's format): board name + three 4-byte hex
       groups, bytes within each group printed high-to-low. */
    char board_name[24];
    strncpy(board_name, HAL_BOARD_NAME, 23);
    board_name[23] = 0;
    snprintf(buf, 50, "%s %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
             board_name,
             (unsigned)serialid[3], (unsigned)serialid[2], (unsigned)serialid[1], (unsigned)serialid[0],
             (unsigned)serialid[7], (unsigned)serialid[6], (unsigned)serialid[5], (unsigned)serialid[4],
             (unsigned)serialid[11], (unsigned)serialid[10], (unsigned)serialid[9], (unsigned)serialid[8]);
    buf[49] = 0;
    return true;
}

/* DMA-safe allocation - THE PREREQUISITE FOR SPI DMA. A buffer the DMA engine
 * cannot reach, or that shares a cache line, corrupts silently. */
#include <zephyr/sys/sys_heap.h>

/* MEMORY REGIONS - a direct port of AP_HAL_ChibiOS/hwdef/common/malloc.c, so the
 * region table and its semantics match ChibiOS rather than inventing a new model. */
#define MEM_REGION_FLAG_DMA_OK 1
#define MEM_REGION_FLAG_FAST   2

struct memory_region {
    void *address;
    uint32_t size;
    uint32_t flags;
};

/* Region 0 is the DEFAULT heap, handled by calloc()/free() rather than a k_heap. */
#define ZEPHYR_DMA_POOL_SIZE  16384
/* 16 KB, not the original 65536. MEASURED 2026-08-07: the only MEM_FAST consumer
 * never approached the larger size, and DTCM is scarce. */
/* 16384 -> 14336 on 2026-08-09: CONFIG_SPI_RTIO's per-instance context
   pushed .dtcm_noinit 1536 B past the 32 KB DTCM; the FAST pool is the
   only elastic tenant in that region. */
#define ZEPHYR_FAST_POOL_SIZE 14336

#ifdef CONFIG_NOCACHE_MEMORY
static __nocache uint8_t dma_pool_mem[ZEPHYR_DMA_POOL_SIZE] __aligned(32);
#else
/* No nocache region: buffers are still cache-line isolated, but CPU/DMA
   coherency is NOT solved here and the driver or SoC must handle it. */
static uint8_t dma_pool_mem[ZEPHYR_DMA_POOL_SIZE] __aligned(32);
#endif

/* The FAST region. __dtcm_bss_section puts it in DTCM, this SoC's fastest RAM. */
#if defined(CONFIG_ARM)
static __dtcm_bss_section uint8_t fast_pool_mem[ZEPHYR_FAST_POOL_SIZE] __aligned(8);
#else
static uint8_t fast_pool_mem[ZEPHYR_FAST_POOL_SIZE] __aligned(8);
#endif

static const struct memory_region memory_regions[] = {
    /* region 0: default heap, backed by calloc() - address/size unused */
    { nullptr,       0,                     0 },
    { dma_pool_mem,  ZEPHYR_DMA_POOL_SIZE,  MEM_REGION_FLAG_DMA_OK },
    { fast_pool_mem, ZEPHYR_FAST_POOL_SIZE, MEM_REGION_FLAG_FAST },
};
#define NUM_MEMORY_REGIONS (sizeof(memory_regions)/sizeof(memory_regions[0]))

/* k_heap, NOT sys_heap. This is not a style choice: k_heap takes a timeout and
 * blocks, which is what ArduPilot's allocator contract expects. */
static struct k_heap heaps[NUM_MEMORY_REGIONS];
static bool heaps_ready;

/*
  DMA reserve heap, mirroring ChibiOS's DMA_RESERVE_SIZE (malloc.c:66-72). Carved
  out of DMA memory at init so that a late DMA allocation cannot fail merely
  because ordinary traffic drained the pool first. Same 6144 B default.
 */
#define ZEPHYR_DMA_RESERVE_SIZE 6144
static struct k_heap dma_reserve_heap;
static bool dma_reserve_ready;

/* Counts DMA allocations that fell through to the cached heap. Replaces a
   printk() on that path - see the note at the fallback itself. */
static uint32_t dma_pool_exhausted_count;

static void *malloc_dma(size_t size);   /* used by init_heaps() to carve the reserve */

static void init_heaps(void)
{
    if (heaps_ready) {
        return;
    }
    for (uint8_t i=1; i<NUM_MEMORY_REGIONS; i++) {
        k_heap_init(&heaps[i], memory_regions[i].address, memory_regions[i].size);
    }
    /* must be set BEFORE the malloc_dma() below, which re-enters malloc_flags() */
    heaps_ready = true;

    /*
      ChibiOS malloc.c:103-111 verbatim in behaviour: ask for the reserve, and on
      failure shrink by 7/8 and retry, so a small pool still yields SOME reserve
      rather than none.
     */
    uint32_t reserve_size = ZEPHYR_DMA_RESERVE_SIZE;
    while (reserve_size > 0) {
        void *dma_reserve = malloc_dma(reserve_size);
        if (dma_reserve != nullptr) {
            k_heap_init(&dma_reserve_heap, dma_reserve, reserve_size);
            dma_reserve_ready = true;
            break;
        }
        reserve_size = (reserve_size * 7) / 8;
    }
}

/* Initialise at PRE_KERNEL_1 rather than lazily on first malloc: the lazy form
 * races with allocations made from other PRE_KERNEL_1 initialisers. */
static int init_heaps_sys(void)
{
    init_heaps();
    return 0;
}
SYS_INIT(init_heaps_sys, PRE_KERNEL_1, 0);

/*
  Allocate with flags, mirroring ChibiOS malloc_flags(): try regions whose flags
  satisfy the request, then - for non-DMA requests only - fall back to any heap
  and finally the default heap.
 */
static void *malloc_flags(size_t size, uint32_t flags)
{
    if (size == 0) {
        return nullptr;
    }
    init_heaps();

    /* 32-byte alignment for DMA so a buffer never shares a D-cache line with
       anything else; natural alignment otherwise. */
    const size_t align = (flags & MEM_REGION_FLAG_DMA_OK) ? 32 : 8;

    /* DIVERGENCE FIXED: round the SIZE up to the DMA alignment too, not just the
     * address - a tail sharing a cache line with other data is not DMA-safe. */
    if (flags & MEM_REGION_FLAG_DMA_OK) {
        size = (size + (align - 1)) & ~(align - 1);
    }

    for (uint8_t i=1; i<NUM_MEMORY_REGIONS; i++) {
        if ((flags & MEM_REGION_FLAG_DMA_OK) &&
            !(memory_regions[i].flags & MEM_REGION_FLAG_DMA_OK)) {
            continue;
        }
        if ((flags & MEM_REGION_FLAG_FAST) &&
            !(memory_regions[i].flags & MEM_REGION_FLAG_FAST)) {
            continue;
        }
        void *p = k_heap_aligned_alloc(&heaps[i], align, size, K_NO_WAIT);
        if (p != nullptr) {
            memset(p, 0, size);
            return p;
        }
    }

    /* Not a DMA request: fall back to another heap, then the default heap. */
    if (!(flags & MEM_REGION_FLAG_DMA_OK)) {
        for (uint8_t i=1; i<NUM_MEMORY_REGIONS; i++) {
            if (memory_regions[i].flags & MEM_REGION_FLAG_DMA_OK) {
                continue;
            }
            void *p = k_heap_aligned_alloc(&heaps[i], align, size, K_NO_WAIT);
            if (p != nullptr) {
                memset(p, 0, size);
                return p;
            }
        }
        return calloc(1, size);
    }

    /* Last resort for a DMA request: the reserve heap, as ChibiOS malloc.c does. */
    if (dma_reserve_ready) {
        void *p = k_heap_aligned_alloc(&dma_reserve_heap, align, size, K_NO_WAIT);
        if (p != nullptr) {
            memset(p, 0, size);
            return p;
        }
    }

    /* DIVERGENCE FIXED: this used to printk() here. ChibiOS prints nothing from the
     * allocator, and printing from a failed allocation can recurse. */
    dma_pool_exhausted_count++;
    return calloc(1, size);
}

static void *malloc_dma(size_t size)     { return malloc_flags(size, MEM_REGION_FLAG_DMA_OK); }
static void *malloc_fastmem(size_t size) { return malloc_flags(size, MEM_REGION_FLAG_FAST); }

/* Routing copied verbatim from AP_HAL_ChibiOS/Util.cpp::malloc_type(). */
void *Util::malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    if (mem_type == AP_HAL::Util::MEM_DMA_SAFE) {
        return malloc_dma(size);
    } else if (mem_type == AP_HAL::Util::MEM_FAST) {
        return malloc_fastmem(size);
    } else if (mem_type == AP_HAL::Util::MEM_FILESYSTEM) {
        return malloc_dma(size);
    } else {
        return calloc(1, size);
    }
}

/* ChibiOS's free_type() is simply free(ptr) - its allocator unifies every region,
 * so it can ignore the type argument. We cannot: the region a pointer came from
 * is what decides which heap frees it. */
void Util::free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    (void)size;
    (void)mem_type;
    if (ptr == nullptr) {
        return;
    }
    if (heaps_ready) {
        for (uint8_t i=1; i<NUM_MEMORY_REGIONS; i++) {
            uint8_t *base = (uint8_t *)memory_regions[i].address;
            if ((uint8_t *)ptr >= base && (uint8_t *)ptr < base + memory_regions[i].size) {
                k_heap_free(&heaps[i], ptr);
                return;
            }
        }
    }
    free(ptr);
}

/* ChibiOS's mem_is_dma_safe() equivalent: is this buffer in memory the DMA engine
 * can actually reach? */
extern "C" bool mem_is_dma_safe(const void *buf, uint32_t size)
{
    if (buf == nullptr || !heaps_ready) {
        return false;
    }
    const uint8_t *p = (const uint8_t *)buf;
    for (uint8_t i=1; i<NUM_MEMORY_REGIONS; i++) {
        if (!(memory_regions[i].flags & MEM_REGION_FLAG_DMA_OK)) {
            continue;
        }
        const uint8_t *base = (const uint8_t *)memory_regions[i].address;
        if (p >= base && (p + size) <= (base + memory_regions[i].size)) {
            return true;
        }
    }
    return false;
}

/* ISR entry counter for the SPI DMA metric in Tools/CPUInfo. Costs one increment
 * per interrupt; do not ship it enabled. */
#ifdef CONFIG_TRACING_USER
extern "C" volatile uint32_t g_ap_isr_count;
extern "C" volatile uint32_t g_ap_isr_cycles;
volatile uint32_t g_ap_isr_count;
volatile uint32_t g_ap_isr_cycles;

/* Counting ISR entries says how OFTEN interrupts fire, not how much CPU they cost -
 * the two diverge sharply once DMA reduces the count but not the work. */
static uint32_t isr_t0;
static uint8_t isr_depth;

/* Per-vector composition, added 2026-08-08 for the switch/IRQ-rate work. */
extern "C" volatile uint32_t g_ap_isr_vec[256];
volatile uint32_t g_ap_isr_vec[256];

extern "C" void sys_trace_isr_enter_user(void)
{
    if (isr_depth++ == 0) {
        isr_t0 = k_cycle_get_32();
    }
    g_ap_isr_count++;
    uint32_t vec = (*(volatile uint32_t *)0xE000ED04u) & 0x1FFu;
    if (vec >= 256u) {
        vec = 255u;
    }
    g_ap_isr_vec[vec]++;
}

extern "C" void sys_trace_isr_exit_user(void)
{
    if (isr_depth > 0 && --isr_depth == 0) {
        g_ap_isr_cycles += k_cycle_get_32() - isr_t0;
    }
}
#endif  // CONFIG_TRACING_USER

#if AP_CRASHDUMP_ENABLED
/* On-board crash dump retrieval, readable after a reset. See Util.h. */
#include "zephyr/src/rt1176_coredump.h"
#include "zephyr/src/rt1176_romapi_flash.h"

size_t Util::last_crash_dump_size() const
{
#ifdef CONFIG_AP_RT1176_ROMAPI_FLASH
    const struct rt1176_coredump_hdr *hdr = (const struct rt1176_coredump_hdr *)
        (uintptr_t)(RT1176_FLASH_MEMMAP_BASE + RT1176_COREDUMP_PARTITION_OFFSET);
    if (hdr->magic != RT1176_COREDUMP_MAGIC) {
        return 0;
    }
    return hdr->size;
#else
    return 0;
#endif
}

void *Util::last_crash_dump_ptr() const
{
#ifdef CONFIG_AP_RT1176_ROMAPI_FLASH
    if (last_crash_dump_size() == 0) {
        return nullptr;
    }
    return (void *)(uintptr_t)(RT1176_FLASH_MEMMAP_BASE + RT1176_COREDUMP_PARTITION_OFFSET +
                                RT1176_COREDUMP_HDR_SIZE);
#else
    return nullptr;
#endif
}
#endif  // AP_CRASHDUMP_ENABLED

#if AP_BOOTLOADER_FLASHING_ENABLED
/* In-app bootloader update, ChibiOS parity: reads bootloader.bin from ROMFS and
 * programs it, so a board can be updated without SWD. */
#ifndef CONFIG_AP_RT1176_ROMAPI_FLASH
#error "AP_BOOTLOADER_FLASHING_ENABLED needs the RT1176 ROM-API flash backend"
#endif
#include <AP_ROMFS/AP_ROMFS.h>
#include <zephyr/cache.h>
#include <zephyr/sys/printk.h>
#include "zephyr/src/rt1176_romapi_flash.h"

#if HAL_GCS_ENABLED
#include <GCS_MAVLink/GCS.h>
#define BL_FLASH_DEBUG(fmt, args ...)  do { GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } while (0)
#else
#define BL_FLASH_DEBUG(fmt, args ...)  do { printk(fmt "\n", ## args); } while (0)
#endif

extern const AP_HAL::HAL &hal;

Util::FlashBootloader Util::flash_bootloader()
{
    uint32_t fw_size;
    const char *fw_name = "bootloader.bin";

    EXPECT_DELAY_MS(11000);

    const uint8_t *fw = AP_ROMFS::find_decompress(fw_name, fw_size);
    if (fw == nullptr) {
        BL_FLASH_DEBUG("failed to find %s", fw_name);
        return FlashBootloader::NOT_AVAILABLE;
    }

    // FLASH_BOOTLOADER_LOAD_KB - the resident bootloader's slot
    const uint32_t bl_region = 128U * 1024U;
    uint32_t fcb_tag = 0;
    if (fw_size >= 0x404U) {
        memcpy(&fcb_tag, &fw[0x400], sizeof(fcb_tag));
    }
    if (fw_size > bl_region || fcb_tag != 0x42464346UL) {   // "FCFB" LE
        BL_FLASH_DEBUG("bad bootloader image (%u bytes)", (unsigned)fw_size);
        AP_ROMFS::free(fw);
        return FlashBootloader::FAIL;
    }

    const uint8_t *cur = (const uint8_t *)(uintptr_t)RT1176_FLASH_MEMMAP_BASE;
    sys_cache_data_invd_range((void *)cur, fw_size);
    if (memcmp(fw, cur, fw_size) == 0) {
        BL_FLASH_DEBUG("Bootloader up-to-date");
        AP_ROMFS::free(fw);
        return FlashBootloader::NO_CHANGE;
    }

    /* Slice the erase and pace it (hardware, 2026-08-14): back-to-back irq-locked
     * ROM flash calls starve the watchdog feeder and the SoC resets mid-flash. */
    static constexpr uint32_t SLICE = 4096;

    auto erase_region = [&]() -> bool {
        BL_FLASH_DEBUG("Erasing bootloader");
        for (uint32_t off = 0; off < bl_region; off += SLICE) {
            EXPECT_DELAY_MS(1000);
            if (rt1176_flash_erase(off, SLICE) != 0) {
                return false;
            }
            hal.scheduler->delay(2);
        }
        return true;
    };

    if (!erase_region()) {
        BL_FLASH_DEBUG("Erase failed");
        AP_ROMFS::free(fw);
        return FlashBootloader::FAIL;
    }

    BL_FLASH_DEBUG("Flashing %s (%u bytes)", fw_name, (unsigned)fw_size);
    const uint8_t max_attempts = 10;
    for (uint8_t i = 0; i < max_attempts; i++) {
        bool write_ok = true;
        for (uint32_t off = 0; off < fw_size; off += SLICE) {
            EXPECT_DELAY_MS(1000);
            const uint32_t n = MIN(SLICE, fw_size - off);
            if (rt1176_flash_program(off, &fw[off], n) != 0) {
                write_ok = false;
                break;
            }
            hal.scheduler->delay(2);
        }
        if (!write_ok) {
            BL_FLASH_DEBUG("Flash failed! (attempt=%u/%u)", i + 1, max_attempts);
            hal.scheduler->delay(100);
            continue;
        }
        sys_cache_data_invd_range((void *)cur, fw_size);
        if (memcmp(fw, cur, fw_size) != 0) {
            // NOR programming can only clear bits: a failed verify cannot be
            // fixed by re-programming over the top - erase and retry.
            BL_FLASH_DEBUG("Verify failed! (attempt=%u/%u)", i + 1, max_attempts);
            if (!erase_region()) {
                break;
            }
            continue;
        }
        BL_FLASH_DEBUG("Flash OK");
        AP_ROMFS::free(fw);
        return FlashBootloader::OK;
    }

    BL_FLASH_DEBUG("Flash failed after %u attempts", max_attempts);
    AP_ROMFS::free(fw);
    return FlashBootloader::FAIL;
}
#endif  // AP_BOOTLOADER_FLASHING_ENABLED

/* Wall clock (get/set_hw_rtc). RT1176 backs it with the SNVS LP SRTC. */
#ifdef CONFIG_SOC_SERIES_IMXRT11XX
#include "zephyr/src/rt1176_snvs_rtc.h"
#endif

void Util::set_hw_rtc(uint64_t time_utc_usec)
{
    _rtc_usec = time_utc_usec;
#ifdef CONFIG_SOC_SERIES_IMXRT11XX
    rt1176_snvs_srtc_set_seconds((uint32_t)(time_utc_usec / 1000000ULL));
#endif
}

uint64_t Util::get_hw_rtc() const
{
#ifdef CONFIG_SOC_SERIES_IMXRT11XX
    const uint32_t sec = rt1176_snvs_srtc_get_seconds();
    /* 1600000000 = 2020-09-13: anything below is a never-set counter that
       started from zero, not a real wall clock - fall through to RAM. */
    if (sec > 1600000000u) {
        return (uint64_t)sec * 1000000ULL;
    }
#endif
    return _rtc_usec;
}

/* @SYS/mem.txt in ChibiOS MemInfoV1 format so existing parsers work. */
void Util::mem_info(ExpandingString &str)
{
    str.printf("MemInfoV1\n");
    str.printf("START=%p LEN=%3uk FREE=%6u LRG=%6u TYPE=%1u\n",
               (void*)nullptr, 0U, (unsigned)available_memory(), 0U, 0U);
#if defined(__ZEPHYR__) && defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
    for (uint8_t i = 1; i < NUM_MEMORY_REGIONS; i++) {
        struct sys_memory_stats stats;
        if (sys_heap_runtime_stats_get(&heaps[i].heap, &stats) != 0) {
            continue;
        }
        str.printf("START=%p LEN=%3uk FREE=%6u LRG=%6u TYPE=%1u\n",
                   memory_regions[i].address,
                   (unsigned)(memory_regions[i].size / 1024),
                   (unsigned)stats.free_bytes, 0U,
                   (unsigned)memory_regions[i].flags);
    }
#endif
}
