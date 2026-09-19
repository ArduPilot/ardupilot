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
#if defined(__ZEPHYR__)
/* for Z_MALLOC_PARTITION_EXISTS, which decides whether mem_info() can work out
   where the libc malloc arena starts, and for the _end symbol it starts past */
#include <zephyr/sys/libc-hooks.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/devicetree.h>
#include <zephyr/devicetree/sram.h>
#endif
#include <AP_Common/ExpandingString.h>   // thread_info() writes into one
#include "UARTDriver.h"                  // @SYS/uarts.txt and dma.txt
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

/* The two @SYS files individually, for a debugger. Each is a pointer into
   g_ap_sysinfo plus a length, so `x/s g_threads_txt` in gdb prints threads.txt
   and `dump binary memory f g_tasks_txt g_tasks_txt+g_tasks_txt_len` saves
   tasks.txt, over SWD, and nothing printed on any port. (The core IS halted
   for the attach-to-detach window of each read - measured at up to ~0.6 s
   per read through a Black Magic Probe, gdb start-up included - which is why
   the scripts refuse an armed board.) No
   second buffer and no second render: the sections are contiguous in
   g_ap_sysinfo and there is no NUL between them, which is why the lengths
   exist. Tools/zephyr/zephyr_sysinfo_gdb.py reads them through the Black
   Magic Probe; zephyr_sysinfo.py keeps reading the whole buffer with pyocd.
   Valid once g_ap_sysinfo_seq is non-zero and EVEN; a reader that samples seq
   before and after can tell if a capture landed mid-read or was in progress. */
const char *volatile g_threads_txt;
volatile uint32_t g_threads_txt_len;
const char *volatile g_tasks_txt;
volatile uint32_t g_tasks_txt_len;

/* soft_armed as one byte a debugger can read before it halts anything. */
volatile uint8_t g_ap_soft_armed;

/* Count of MEM_DMA_SAFE allocations served from ordinary heap memory.
   Defined further down beside the allocator; declared here because
   ap_sysinfo_capture() below reports it. */
extern volatile uint32_t g_dma_pool_exhausted;
}

void Zephyr::Util::set_soft_armed(const bool b)
{
    AP_HAL::Util::set_soft_armed(b);
    g_ap_soft_armed = b ? 1 : 0;
}

/* extern "C" deliberately: the caller is Zephyr::Scheduler::_io_thread_fn, and
   a plain `extern void ap_sysinfo_capture();` declared inside that namespace
   binds to Zephyr::ap_sysinfo_capture(), which this definition is not
   (`using namespace Zephyr;` does not place definitions INTO the namespace).
   C linkage sidesteps the mismatch. */
extern "C" void ap_sysinfo_capture(void)
{
    extern const AP_HAL::HAL &hal;
    /* seq is ODD while a capture is being rendered and EVEN when the buffer is
       whole: a reader that samples it before and after must see the same EVEN
       value. Sampling before and after alone did not catch a debugger halting
       the board inside this function (review, 2026-09-12): the buffer was
       already cleared, seq had not moved, and a half-rendered capture read as
       a good one. */
    g_ap_sysinfo_seq++;
    ExpandingString str(g_ap_sysinfo, sizeof(g_ap_sysinfo));

    /* @SYS/threads.txt - per-thread CPU LOAD% and stack high-water. */
    hal.util->thread_info(str);
    const uint32_t threads_len = str.get_length();

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
    const uint32_t tasks_len = str.get_length() - threads_len;

    /* Per-file views for a debugger, set together so a reader never sees one
       file's pointer with the other's length. The buffer address is fixed;
       only the lengths change between captures. */
    g_threads_txt = g_ap_sysinfo;
    g_threads_txt_len = threads_len;
    g_tasks_txt = g_ap_sysinfo + threads_len;
    g_tasks_txt_len = tasks_len;

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

    /* DMA pool health. Non-zero means at least one MEM_DMA_SAFE request was
       served from ordinary heap memory, which a DMA engine may not be able to
       reach - see the fallback in malloc_flags(). */
    str.printf("DMA pool exhausted: %lu\n", (unsigned long)g_dma_pool_exhausted);

    /* length last, then seq back to even: the capture is whole. */
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
        ctx->str->printf("%-13.13s PRI=%3d sp=%p STACK=%4u/%4u LOAD=%4.1f%%",
                         name, (int)k_thread_priority_get((k_tid_t)thread),
                         stack_start,
                         (unsigned)unused, (unsigned)total_stack,
                         100.0f * float(stats.execution_cycles) / float(ctx->total_cycles));
#ifdef CONFIG_SCHED_THREAD_USAGE_ANALYSIS
        /* One window is one scheduled-in..scheduled-out run, timed by the
           kernel's own context-switch hooks. For the idle thread a window is
           one uninterrupted idle episode, so this is how long the CPU had
           nothing to do at a stretch - min/mean/max since boot.
           The max is a since-boot extreme; the mean is cumulative, so a
           grapher wanting a windowed mean differences LOAD's cycle total.
           The min is always 0: Zephyr's kernel tracks only .peak_cycles and
           .average_cycles, and a real trough would need a counter added to
           modules/zephyr, which is upstream and not ours to carry. The field
           is kept so the layout matches ChibiOS's time_measurement_t
           best/cumulative/worst triple and so parsers see a fixed shape. */
        ctx->str->printf(" SLICE=%lu/%lu/%luus",
                         0UL,
                         (unsigned long)k_cyc_to_us_floor64(stats.average_cycles),
                         (unsigned long)k_cyc_to_us_floor64(stats.peak_cycles));
#endif
        ctx->str->printf("\n");
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
/* for LINKER_DT_NODE_REGION_NAME(), which names the linker region a
   zephyr,memory-region devicetree node generates */
#include <zephyr/linker/devicetree_regions.h>

/* MEMORY REGIONS - a direct port of AP_HAL_ChibiOS/hwdef/common/malloc.c, so the
 * region table and its semantics match ChibiOS rather than inventing a new model.
 * Flag VALUES are ChibiOS's, so @SYS/memory.txt's TYPE column means the same
 * thing on both HALs. AXI_BUS and ETH_SAFE tag regions but are not yet request
 * types here - nothing in this HAL asks for them, and ChibiOS's ETH_SAFE path
 * additionally forces alignment==size and a power-of-two size, which would be
 * untested code with no caller. */
#define MEM_REGION_FLAG_DMA_OK   1
#define MEM_REGION_FLAG_FAST     2
#define MEM_REGION_FLAG_AXI_BUS  4
#define MEM_REGION_FLAG_ETH_SAFE 8

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

/*
  THE SoC RAM MAP.

  ChibiOS builds its heaps from RAM_MAP in
  libraries/AP_HAL_ChibiOS/hwdef/scripts/<MCU>.py and manages every bank the
  part has. This port managed none of them. Zephyr's system heap lives in the
  single bank named by /chosen/zephyr,sram and the rest of the SoC's RAM was
  simply never referenced - on an H743 that is about 543 KB of 1055 KB sitting
  idle, and it is why tridge saw @SYS/memory.txt list three regions on a
  CubeOrangeZephyr where a ChibiOS CubeOrange lists six.

  Each bank is claimed by a pool array placed in the linker region Zephyr
  generates for that devicetree node, so it is the linker, not a comment, that
  guarantees nothing else lands there - if it ever does, the build fails
  instead of two owners quietly sharing addresses. Those sections are NOLOAD,
  so the arrays cost nothing in flash and the startup code does not touch them.

  Geometry and flags follow STM32H743xx.py's RAM_MAP line for line:

     (0x30000000, 256, 8)  SRAM1+SRAM2  ETH_SAFE   merged in the board DTS
     (0x20000000, 128, 2)  DTCM         FAST       no DMA engine reaches it
     (0x24000000, 512, 4)  AXI SRAM     AXI_BUS    region 0, Zephyr's own heap
     (0x00000400,  63, 2)  ITCM         FAST       first 1 KB deliberately unused
     (0x30040000,  32, 8)  SRAM3        ETH_SAFE
     (0x38000000,  64, 1)  SRAM4        DMA_OK

  Region 0 is AXI SRAM on this port rather than SRAM1 as in ChibiOS's map. That
  is not a choice: region 0 is whatever /chosen/zephyr,sram named, because that
  is where the linker has already put .data and .bss.
*/
#if defined(CONFIG_SOC_SERIES_STM32H7X)
#define AP_ZEPHYR_SOC_RAM_MAP 1
#else
#define AP_ZEPHYR_SOC_RAM_MAP 0
#endif

/* A bank is ours to claim only if the devicetree declares it with a linker
   region, it is not the bank Zephyr already runs its own heap in, and it is
   still enabled - disabling a node is how the board DTS folds SRAM2 into
   SRAM1. */
#define AP_BANK_CLAIMABLE(label)                                        \
    (DT_NODE_HAS_STATUS(DT_NODELABEL(label), okay) &&                   \
     DT_NODE_HAS_PROP(DT_NODELABEL(label), zephyr_memory_region) &&     \
     !DT_SAME_NODE(DT_NODELABEL(label), DT_CHOSEN(zephyr_sram)))

#define AP_DEFINE_BANK(label)                                           \
    static uint8_t label##_bank[DT_REG_SIZE(DT_NODELABEL(label))]       \
        Z_GENERIC_SECTION(LINKER_DT_NODE_REGION_NAME(DT_NODELABEL(label))) __aligned(32)

/* The TCMs go to the ArduPilot allocator only if the board has NOT handed them
   to the kernel. A board that sets /chosen/zephyr,dtcm wants __dtcm_* sections
   there, and both owners in one region would overflow it. mr_vmu_rt1176 is
   such a board, which is why it keeps the fixed FAST pool below. */
#if AP_ZEPHYR_SOC_RAM_MAP && AP_BANK_CLAIMABLE(dtcm) && !DT_HAS_CHOSEN(zephyr_dtcm)
#define AP_DTCM_CLAIMABLE 1
#else
#define AP_DTCM_CLAIMABLE 0
#endif
#if AP_ZEPHYR_SOC_RAM_MAP && AP_BANK_CLAIMABLE(itcm) && !DT_HAS_CHOSEN(zephyr_itcm)
#define AP_ITCM_CLAIMABLE 1
#else
#define AP_ITCM_CLAIMABLE 0
#endif

/* ChibiOS starts its ITCM region at 0x400 rather than 0, leaving the first
   1 KB out of the heap on purpose so a null-pointer dereference cannot land in
   valid RAM. Same here - the array still covers the bank, so nothing else can
   claim the low kilobyte, the allocator just never hands it out. */
#define AP_ITCM_NULL_GUARD 1024

#if AP_ZEPHYR_SOC_RAM_MAP
#if AP_BANK_CLAIMABLE(sram1)
AP_DEFINE_BANK(sram1);
#endif
#if AP_BANK_CLAIMABLE(sram2)
AP_DEFINE_BANK(sram2);
#endif
#if AP_BANK_CLAIMABLE(sram3)
AP_DEFINE_BANK(sram3);
#endif
#if AP_BANK_CLAIMABLE(sram4)
AP_DEFINE_BANK(sram4);
#endif
#if AP_DTCM_CLAIMABLE
AP_DEFINE_BANK(dtcm);
#endif
#if AP_ITCM_CLAIMABLE
AP_DEFINE_BANK(itcm);
#endif
#endif  /* AP_ZEPHYR_SOC_RAM_MAP */

/* The fixed FAST pool, for boards with no claimable TCM bank. __dtcm_bss_section
   puts it in DTCM on a board that declares one; without /chosen/zephyr,dtcm the
   section lands in ordinary RAM, see the note in DeviceBus.cpp. */
#if !AP_DTCM_CLAIMABLE
#if defined(CONFIG_ARM) && DT_HAS_CHOSEN(zephyr_dtcm)
static __dtcm_bss_section uint8_t fast_pool_mem[ZEPHYR_FAST_POOL_SIZE] __aligned(8);
#else
static uint8_t fast_pool_mem[ZEPHYR_FAST_POOL_SIZE] __aligned(8);
#endif
#endif

static const struct memory_region memory_regions[] = {
    /* region 0: Zephyr's own heap, in whatever bank /chosen/zephyr,sram named.
       On an H743 that is AXI SRAM, so it carries ChibiOS's AXI_BUS flag. The
       address and size are filled in by mem_info(); malloc_flags() never
       allocates from this entry, calloc() does. */
#if AP_ZEPHYR_SOC_RAM_MAP
    { nullptr,       0,                     MEM_REGION_FLAG_AXI_BUS },
#else
    { nullptr,       0,                     0 },
#endif
    { dma_pool_mem,  ZEPHYR_DMA_POOL_SIZE,  MEM_REGION_FLAG_DMA_OK },
#if AP_ZEPHYR_SOC_RAM_MAP
#if AP_BANK_CLAIMABLE(sram1)
    { sram1_bank,    sizeof(sram1_bank),    MEM_REGION_FLAG_ETH_SAFE },
#endif
#if AP_BANK_CLAIMABLE(sram2)
    { sram2_bank,    sizeof(sram2_bank),    MEM_REGION_FLAG_ETH_SAFE },
#endif
#if AP_BANK_CLAIMABLE(sram3)
    { sram3_bank,    sizeof(sram3_bank),    MEM_REGION_FLAG_ETH_SAFE },
#endif
#if AP_BANK_CLAIMABLE(sram4)
    { sram4_bank,    sizeof(sram4_bank),    MEM_REGION_FLAG_DMA_OK },
#endif
#if AP_DTCM_CLAIMABLE
    { dtcm_bank,     sizeof(dtcm_bank),     MEM_REGION_FLAG_FAST },
#endif
#if AP_ITCM_CLAIMABLE
    { itcm_bank + AP_ITCM_NULL_GUARD,
                     sizeof(itcm_bank) - AP_ITCM_NULL_GUARD,
                                            MEM_REGION_FLAG_FAST },
#endif
#endif  /* AP_ZEPHYR_SOC_RAM_MAP */
#if !AP_DTCM_CLAIMABLE
    { fast_pool_mem, ZEPHYR_FAST_POOL_SIZE, MEM_REGION_FLAG_FAST },
#endif
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
/* Where the reserve block itself lives. It is carved OUT of a DMA region, so
 * every pointer the reserve heap hands out also falls inside that region's
 * address range and free_type()'s region walk would otherwise return it to the
 * wrong heap. Kept so that walk can be short-circuited. */
static uint8_t *dma_reserve_base;
static size_t dma_reserve_bytes;

/* Counts DMA allocations that fell through to ordinary heap memory. Replaces a
   printk() on that path - see the note at the fallback itself.

   extern "C" and volatile so it can be read over SWD and, below, reported in
   the @SYS capture. It used to be a file-static that nothing ever read, so a
   board quietly handing NON-DMA-SAFE memory to a caller that asked for
   MEM_DMA_SAFE left no trace at all. The fallback itself is kept - ChibiOS's
   allocator does the same rather than failing the allocation - but a driver
   DMAing into it is a real hazard, so the count has to be visible. */
volatile uint32_t g_dma_pool_exhausted;
#define dma_pool_exhausted_count g_dma_pool_exhausted

static void *malloc_dma(size_t size);   /* used by init_heaps() to carve the reserve */

#if AP_ZEPHYR_SOC_RAM_MAP
#include <stm32_ll_bus.h>

/*
  Bring the SoC's other RAM banks online, before anything writes to them.

  ChibiOS does this in stm32_clock_init() - ChibiOS/os/hal/ports/STM32/
  STM32H7xx/hal_lld.c, rccEnableSRAM1/2/3 - because the D2 SRAMs come out of
  reset with their clocks GATED. Zephyr never turns them on, since in a stock
  Zephyr build nothing is placed in them. Read over SWD on a running
  CubeOrangeZephyr before this change: RCC->AHB2ENR was 0x00000000, all three
  bits clear. An access to 0x30000000 in that state is a bus fault, so this has
  to happen before k_heap_init() writes the first heap header - which is why it
  is called from init_heaps() rather than racing it as another SYS_INIT.
  SRAM4 sits in D3 and has no clock gate on this SoC; ChibiOS enables nothing
  for it either.

  The TCMs are then zero-filled at the access widths ST's AN5342 specifies for
  the ECC init - 64-bit for ITCM, 32-bit for DTCM - which is exactly what
  Zephyr's own soc_reset_hook() does for a board that hands its TCM to the
  kernel. This board does not, so nothing had initialised them, and on the
  parts whose TCMs carry ECC a caller reading a freshly allocated block before
  writing it would take an ECC error. Costs a few hundred microseconds once.
*/
static void soc_ram_banks_init(void)
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_D2SRAM1 |
                             LL_AHB2_GRP1_PERIPH_D2SRAM2 |
                             LL_AHB2_GRP1_PERIPH_D2SRAM3);

#if AP_ITCM_CLAIMABLE
    {
        volatile uint64_t *p = (volatile uint64_t *)itcm_bank;
        volatile uint64_t *end = p + sizeof(itcm_bank) / sizeof(uint64_t);
        while (p < end) {
            *p++ = 0;
        }
    }
#endif
#if AP_DTCM_CLAIMABLE
    {
        volatile uint32_t *p = (volatile uint32_t *)dtcm_bank;
        volatile uint32_t *end = p + sizeof(dtcm_bank) / sizeof(uint32_t);
        while (p < end) {
            *p++ = 0;
        }
    }
#endif
}
#else
static void soc_ram_banks_init(void) {}
#endif  /* AP_ZEPHYR_SOC_RAM_MAP */

static void init_heaps(void)
{
    if (heaps_ready) {
        return;
    }
    /* MUST precede the k_heap_init() loop - some of those heaps live in banks
       that are unclocked until this returns. */
    soc_ram_banks_init();
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
            dma_reserve_base = (uint8_t *)dma_reserve;
            dma_reserve_bytes = reserve_size;
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

    /* As ChibiOS's malloc_flags(): a DMA-safe request that neither the DMA
       pool nor the reserve can serve gets NULL, never memory from the default
       heap - that memory would be cacheable and unaligned, and a DMA engine
       would then read or write it wrongly, silently. Callers already handle
       NULL (the UART driver falls back to its interrupt path). Counted, and
       nothing is printed: printing from a failed allocation can recurse. */
    dma_pool_exhausted_count++;
    return nullptr;
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
    /* The reserve BEFORE the region walk. The reserve block was carved out of a
     * DMA region with malloc_dma(), so a pointer from the reserve heap is also
     * inside memory_regions[i] and the walk below would hand it to heaps[i] -
     * a heap that never allocated it. That corrupts both: heaps[i] takes a
     * block it does not own, and the reserve loses it for good, which defeats
     * the whole point of holding memory back for late DMA allocations. */
    if (dma_reserve_ready &&
        (uint8_t *)ptr >= dma_reserve_base &&
        (uint8_t *)ptr < dma_reserve_base + dma_reserve_bytes) {
        k_heap_free(&dma_reserve_heap, ptr);
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

/* The other declaration of this in the file sits inside a feature guard, so
   declare it here too - a repeated extern of the same type is legal and this
   block must not depend on whether that feature is built. */
extern const AP_HAL::HAL &hal;

#if HAL_UART_STATS_ENABLED
/*
  @SYS/uarts.txt, the aggregator. AP_HAL_ChibiOS/Util.cpp::uart_info() is the
  reference, including the UARTV1 header and the SERIALn prefix in front of
  whatever the driver prints for itself.

  dt_ms is the gap since the LAST call, not since boot: every figure in this
  file is traffic during that window, which is why the trackers have to live
  here rather than in the driver - one set per port, persisting across calls.
 */
void Util::uart_info(ExpandingString &str)
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t dt_ms = now_ms - sys_uart_stats.last_ms;
    sys_uart_stats.last_ms = now_ms;

    // a header to allow for machine parsers to determine format
    str.printf("UARTV1\n");
    for (uint8_t i = 0; i < HAL_UART_NUM_SERIAL_PORTS; i++) {
        auto *uart = hal.serial(i);
        if (uart != nullptr) {
            str.printf("SERIAL%u ", i);
            uart->uart_info(str, sys_uart_stats.serial[i], dt_ms);
        }
    }
}
#endif  // HAL_UART_STATS_ENABLED

/* @SYS/timers.txt. ChibiOS's is the same single delegation. */
void Util::timer_info(ExpandingString &str)
{
    hal.rcout->timer_info(str);
}

/*
  @SYS/dma.txt, byte-for-byte in AP_HAL_ChibiOS/shared_dma.cpp::dma_info()'s
  DMAV1 columns, with ZERO wherever the quantity does not exist on this HAL.

  The zeros are load-bearing and worth understanding before reading anything
  into them. ChibiOS's ULCK, CLCK and CONT come from Shared_DMA, a lock
  manager that exists because several peripherals CONTEND for one DMA stream:
  they count uncontended and contended lock acquisitions and the ratio between
  them. Zephyr binds a stream to a peripheral in the devicetree at build time,
  so nothing arbitrates at runtime and there is nothing to contend over.

  CONT=0.0% on this HAL therefore means "no contention is possible", not "we
  measured contention and found none". A ChibiOS board showing 0.0% is the
  second thing; they print identically and do not mean the same thing.

  DMA=c:s is 0:0 because the controller and stream numbers live in the
  devicetree dmas property and are not recoverable from the driver at runtime.
  TX is a real transaction count for the ports this HAL drives itself. SPI and
  I2C DMA is run inside the Zephyr drivers, which keep no counters this HAL
  can read, so those streams do not appear at all rather than appear as zero.
 */
void Util::dma_info(ExpandingString &str)
{
    // a header to allow for machine parsers to determine format
    str.printf("DMAV1\n");

#if HAL_UART_STATS_ENABLED
    for (uint8_t i = 0; i < HAL_UART_NUM_SERIAL_PORTS; i++) {
        auto *uart = hal.serial(i);
        if (uart == nullptr) {
            continue;
        }
        uint32_t transactions = 0;
        if (!Zephyr::UARTDriver::dma_counters(uart, transactions)) {
            continue;   /* not on the async path - owns no DMA stream */
        }
        str.printf("DMA=%1u:%1u TX=%8u ULCK=%8u CLCK=%8u CONT=%4.1f%%\n",
                   0U, 0U, unsigned(transactions), 0U, 0U, 0.0f);
    }
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

/*
  @SYS/memory.txt, in ChibiOS's MemInfoV1 format so the same GCS tooling reads
  both HALs - AP_HAL_ChibiOS/Util.cpp::mem_info() is the reference.

  ONE uniform loop over every heap this HAL manages, index 0 first, as ChibiOS
  does. The previous version special-cased region 0 into a hardcoded line
  reading "START=(nil) LEN=  0k", which told a reader the main heap was a
  zero-length region at address nil. Both numbers are knowable: with
  CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE -1 the arena is "the RAM left over",
  which is the span from _end to the end of the chosen SRAM, and that is the
  region in the sense ChibiOS means - so report that span and take FREE from
  the runtime stats.

  LRG is 0 on this HAL and that is a genuine gap, not an oversight. ChibiOS
  fills it from chHeapStatus(), which walks the free list without allocating.
  Zephyr's sys_memory_stats carries only free_bytes, allocated_bytes and
  max_allocated_bytes, and sys_heap exposes no largest-contiguous query. The
  only way to obtain it would be to probe by allocating, and taking the
  largest block out of the DMA pool of a flying vehicle to populate a
  diagnostic field is a worse trade than reporting 0.

  Comparing this against a ChibiOS CubeOrange: the SoC RAM map above puts the
  same banks under management, with ChibiOS's flag values, so the two listings
  now cover the same memory. They are not line-for-line identical and cannot
  be. Region 0 is AXI SRAM here and SRAM1 there, because region 0 is wherever
  the linker already put .data and .bss; this HAL also carries a DMA pool that
  ChibiOS has no equivalent of, since ChibiOS reaches DMA-capable memory by
  flagging whole banks instead. FREE differs too, and should: the two RTOSes
  spend different amounts of the same RAM on themselves.
*/
void Util::mem_info(ExpandingString &str)
{
    str.printf("MemInfoV1\n");

#if defined(__ZEPHYR__) && defined(CONFIG_SYS_HEAP_RUNTIME_STATS) && !defined(CONFIG_EXTERNAL_LIBC)
    for (uint8_t i = 0; i < NUM_MEMORY_REGIONS; i++) {
        struct sys_memory_stats stats = {};
        size_t len_bytes;
        uintptr_t start;

        if (i == 0) {
            /* the main heap: libc's arena, which owns whatever RAM is left */
            if (malloc_runtime_stats_get(&stats) != 0) {
                continue;
            }
#if !defined(Z_MALLOC_PARTITION_EXISTS) && CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE < 0
            /* With a negative arena size and no memory partition, Zephyr's libc
               puts the arena at the first 8-aligned address past _end and runs
               it to the end of the chosen SRAM - HEAP_BASE/HEAP_SIZE in
               lib/libc/common/source/stdlib/malloc.c. Recompute the same two
               numbers so START and LEN describe the actual region, which is
               what ChibiOS prints for its regions.

               WHY THIS LINE READS 239k ON A 512 KB BANK, and does not match
               the 512k ChibiOS prints for the same bank:

                   512.00 KB   the AXI SRAM bank itself (CONFIG_SRAM_SIZE)
                 - 272.58 KB   X, below
                 = 239.42 KB   what is left for the heap, printed as 239k

               X is the static image the linker has already placed in this
               bank: .data, .bss, noinit and Zephyr's kernel object areas -
               everything from _image_ram_start up to _end. It is the same
               number the build prints as "RAM: 279120 B ... 53.24%", and _end
               is literally this line's START. Measured on CubeOrangeZephyr
               2026-09-18: bank 0x24000000..0x24080000, _end 0x24044250, so
               X = 279120 B and the heap gets 245168 B.

               ChibiOS reports 512k here because its region 0 size comes
               straight from the RAM_MAP entry, the whole bank, even though
               its own .bss sits in that bank too - so its LEN overstates what
               the heap can ever hand out and only its FREE tells the truth.
               This prints the extent the allocator actually owns. Both are
               defensible; they are not the same measurement, and a reader
               diffing the two listings needs to know which one they are
               looking at. */
            start = ROUND_UP((uintptr_t)_end, sizeof(double));
            len_bytes = ROUND_DOWN(((uintptr_t)DT_CHOSEN_SRAM_ADDR + (size_t)DT_CHOSEN_SRAM_SIZE) - start,
                                   sizeof(double));
#else
            /* A statically sized arena is an unnamed array, and a partitioned
               one is page-aligned by a rule this code does not track. Report
               the extent the allocator admits to and no address, rather than a
               plausible-looking one that is not where the heap is. */
            start = 0U;
            len_bytes = stats.free_bytes + stats.allocated_bytes;
#endif
        } else {
            if (sys_heap_runtime_stats_get(&heaps[i].heap, &stats) != 0) {
                continue;
            }
            len_bytes = memory_regions[i].size;
            start = (uintptr_t)memory_regions[i].address;
        }

        str.printf("START=0x%08x LEN=%3uk FREE=%6u LRG=%6u TYPE=%1u\n",
                   (unsigned)start,
                   (unsigned)(len_bytes / 1024),
                   (unsigned)stats.free_bytes,
                   0U,
                   (unsigned)memory_regions[i].flags);
    }
#else
    /* No runtime heap stats (or a host libc): report what available_memory()
       can still answer rather than printing a bare header. */
    str.printf("START=0x%08x LEN=%3uk FREE=%6u LRG=%6u TYPE=%1u\n",
               0U, 0U, (unsigned)available_memory(), 0U, 0U);
#endif
}
