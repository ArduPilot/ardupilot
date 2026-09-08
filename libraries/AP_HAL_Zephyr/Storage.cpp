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
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "Storage.h"

#include <algorithm>
#include <cstring>
#include <errno.h>

#include <zephyr/sys/printk.h>

#ifdef CONFIG_ZMS
#include <zephyr/storage/flash_map.h>
#include <zephyr/devicetree/fixed-partitions.h>
#include <zephyr/devicetree/mapped-partition.h>
#endif

#ifdef CONFIG_FAT_FILESYSTEM_ELM
#include <zephyr/fs/fs.h>

static uint8_t           s_fat_fs_buf[2048] __attribute__((aligned(4)));  // FATFS work area
static struct fs_mount_t s_fat_mount = {
    .type      = FS_FATFS,
    .mnt_point = "/SD:",
    .fs_data   = s_fat_fs_buf,
};
static const char SD_FILE_PATH[] = "/SD:/ardupilot.bin";

// keep the file open across write_block() calls
static struct fs_file_t  s_fat_file;
static bool              s_fat_file_open = false;
#endif

#ifdef __ZEPHYR__
#if DT_NODE_HAS_STATUS(DT_NODELABEL(fram0), okay)
static const struct spi_dt_spec fram_spec =
    SPI_DT_SPEC_GET(DT_NODELABEL(fram0), SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0U);
#endif
#endif

using namespace Zephyr;

extern const AP_HAL::HAL& hal;

#define AP_STORAGE_PARTITION storage_partition

Storage::Storage() :
    _storage{},
    _healthy(false),
    _zms_ok(false),
    _fram_ok(false),
    _file_ok(false)
#ifdef CONFIG_ZMS
    , _zms{}
#endif
{
}

void Storage::init()
{
    _storage.fill(0);
    _zms_ok  = false;
    _fram_ok = false;
    _healthy = false;

#ifdef CONFIG_AP_RT1176_ROMAPI_FLASH
    /* --- Backend 0: external NOR via the BootROM flash API --- */
    _flash_ok = false;
    _flash_write_failed = false;
    if (rt1176_flash_init() == 0) {
        _flash_ok = _flash.init();
        if (_flash_ok) {
            printk("AP_HAL_Zephyr Storage: ROM-API flash ready at 0x%06x (2 x %u KiB)\n",
                   (unsigned)RT1176_FLASH_STORAGE_OFFSET,
                   (unsigned)(RT1176_FLASH_SECTOR_SIZE / 1024));
        } else {
            printk("AP_HAL_Zephyr Storage: ROM-API flash init failed\n");
        }
    } else {
        printk("AP_HAL_Zephyr Storage: ROM API unavailable\n");
    }
#endif

    // --- Backend 1: ZMS on internal flash ---
#ifdef CONFIG_ZMS

    // Resolve the flash device and partition offset.
    // ESP32 and other memory-mapped targets use "zephyr,mapped-partition" (DT_MAPPED_PARTITION_*).
    // STM32 and other targets use "fixed-partitions" (FIXED_PARTITION_* / DT_MTD_FROM_FIXED_PARTITION).
#if DT_MAPPED_PARTITION_EXISTS(DT_NODELABEL(AP_STORAGE_PARTITION))
    // Memory-mapped partition (ESP32, ESP32-S3, etc.)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#define _ZMS_FLASH_DEV  DEVICE_DT_GET(DT_MTD_FROM_MAPPED_PARTITION(DT_NODELABEL(AP_STORAGE_PARTITION)))
#define _ZMS_OFFSET     DT_MAPPED_PARTITION_OFFSET(DT_NODELABEL(AP_STORAGE_PARTITION))
#pragma GCC diagnostic pop
#define _ZMS_PARTITION_OKAY 1
#elif DT_NODE_HAS_STATUS(DT_MTD_FROM_FIXED_PARTITION(DT_NODELABEL(AP_STORAGE_PARTITION)), okay)
    // Fixed partition (STM32, NXP, etc.)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#define _ZMS_FLASH_DEV  FIXED_PARTITION_DEVICE(AP_STORAGE_PARTITION)
#define _ZMS_OFFSET     FIXED_PARTITION_OFFSET(AP_STORAGE_PARTITION)
#pragma GCC diagnostic pop
#define _ZMS_PARTITION_OKAY 1
#else
#define _ZMS_PARTITION_OKAY 0
#endif

#if _ZMS_PARTITION_OKAY
    {
        const struct device *flash_dev = _ZMS_FLASH_DEV;

        if (flash_dev == nullptr) {
            printk("AP_HAL_Zephyr Storage: ZMS flash device is NULL\n");
        } else if (!device_is_ready(flash_dev)) {
            printk("AP_HAL_Zephyr Storage: ZMS flash device '%s' not ready\n", flash_dev->name);
        } else {
            _zms.flash_device = flash_dev;
            _zms.offset       = _ZMS_OFFSET;
            _zms.sector_size  = ZMS_SECTOR_SIZE;
            _zms.sector_count = ZMS_SECTOR_COUNT;

            printk("AP_HAL_Zephyr Storage: mounting ZMS on '%s' offset=0x%lx "
                   "sector_size=%u sector_count=%u\n",
                   flash_dev->name, (unsigned long)_zms.offset,
                   _zms.sector_size, _zms.sector_count);

            int64_t t0 = k_uptime_get();
            int rc = zms_mount(&_zms);
            int64_t mount_ms = k_uptime_get() - t0;

            if (rc < 0) {
                printk("AP_HAL_Zephyr Storage: zms_mount() failed (%d) after %lld ms\n",
                       rc, (long long)mount_ms);
            } else {
                printk("AP_HAL_Zephyr Storage: ZMS mounted OK in %lld ms\n", (long long)mount_ms);
                _zms_ok = true;

                int64_t tread = k_uptime_get();
                for (uint16_t i = 0; i < NUM_CHUNKS; i++) {
                    uint8_t *chunk = _storage.data() + i * CHUNK_SIZE;
                    ssize_t  n     = zms_read(&_zms, (zms_id_t)(i + 1), chunk, CHUNK_SIZE);
                    (void)n;  // -ENOENT is fine on first boot
                }
                printk("AP_HAL_Zephyr Storage: read %u chunks in %lld ms\n",
                       NUM_CHUNKS, (long long)(k_uptime_get() - tread));
            }
        }
    }
#else
    printk("AP_HAL_Zephyr Storage: ZMS storage_partition not found in DTS\n");
#endif  // _ZMS_PARTITION_OKAY

#undef _ZMS_FLASH_DEV
#undef _ZMS_OFFSET
#undef _ZMS_PARTITION_OKAY

#endif  // CONFIG_ZMS

    // --- Backend 2: SPI FRAM (CubeOrange, NXP, etc.) ---
#ifdef __ZEPHYR__
#if DT_NODE_HAS_STATUS(DT_NODELABEL(fram0), okay)
    if (!_zms_ok) {
        _fram = &fram_spec;
        if (spi_is_ready_dt(_fram)) {
            // Read the full storage image from FRAM into the RAM shadow.
            _fram_ok = true;
            for (uint16_t i = 0; i < NUM_CHUNKS; i++) {
                if (!_fram_read((uint32_t)i * CHUNK_SIZE,
                                _storage.data() + (uint32_t)i * CHUNK_SIZE,
                                CHUNK_SIZE)) {
                    _fram_ok = false;
                    break;
                }
            }
            if (_fram_ok) {
                printk("AP_HAL_Zephyr Storage: FRAM loaded %u B\n", (unsigned)_storage.size());
            } else {
                printk("AP_HAL_Zephyr Storage: FRAM read failed\n");
                _fram = nullptr;
            }
        } else {
            printk("AP_HAL_Zephyr Storage: FRAM SPI not ready\n");
            _fram = nullptr;
        }
    }
#endif
#endif  // __ZEPHYR__

    // --- Backend 3: FAT filesystem (SD card) ---
#ifdef CONFIG_FAT_FILESYSTEM_ELM
    bool have_backend = _zms_ok || _fram_ok;
#ifdef CONFIG_AP_RT1176_ROMAPI_FLASH
    /* The NOTE above is not hypothetical: with ROM-API flash active this path runs
     * with interrupts locked. */
    have_backend = have_backend || _flash_ok;
#endif
    if (!have_backend) {
        _try_file_mount();
    }
#endif

    bool persistent = _zms_ok || _fram_ok || _file_ok;
#ifdef CONFIG_AP_RT1176_ROMAPI_FLASH
    persistent = persistent || _flash_ok;
#endif
    if (!persistent) {
        printk("AP_HAL_Zephyr Storage: no persistent storage -- RAM-only\n");
    }
    /* Was unconditionally true regardless of `persistent`, so AP_Arming believed
     * storage was durable on boards where it is RAM-only. */
    _healthy = persistent;
    /* Seed the drain timestamp so healthy() doesn't dip false between init
       and the storage thread's first empty-queue tick. */
    _last_empty_ms = AP_HAL::millis();
}

#ifdef CONFIG_AP_RT1176_ROMAPI_FLASH
// ChibiOS's own retry budget (AP_HAL_ChibiOS/Storage.cpp:48)
#define STORAGE_FLASH_RETRIES 5

/* AP_FlashStorage backend over the BootROM flash API. */
static inline uint32_t flash_sector_offset(uint8_t sector)
{
    return RT1176_FLASH_STORAGE_OFFSET + (uint32_t)sector * RT1176_FLASH_SECTOR_SIZE;
}

bool Storage::_flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
    /* ChibiOS parity: retry a failed flash write before giving up. */
    for (uint8_t i = 0; i < STORAGE_FLASH_RETRIES; i++) {
        EXPECT_DELAY_MS(1);
        if (rt1176_flash_program(flash_sector_offset(sector) + offset, data, length) == 0) {
            return true;
        }
        hal.scheduler->delay(1);
    }
    if (_flash_erase_ok()) {
        const uint32_t now = AP_HAL::millis();
        if (now - _last_re_init_ms > 5000) {
            _last_re_init_ms = now;
            const bool ok = _flash.re_initialise();
            printk("AP_HAL_Zephyr Storage: failed at %u:%u for %u - re-init %u\n",
                   (unsigned)sector, (unsigned)offset, (unsigned)length, (unsigned)ok);
        }
    }
    return false;
}

bool Storage::_flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    /*
      No ROM call needed. XIP is intact, so the flash is directly addressable -
      this is the whole reason ext_flash_ctrl can stay disabled.
     */
    const uint8_t *src = (const uint8_t *)(uintptr_t)
        (RT1176_FLASH_MEMMAP_BASE + flash_sector_offset(sector) + offset);
    memcpy(data, src, length);
    return true;
}

bool Storage::_flash_erase_sector(uint8_t sector)
{
    return rt1176_flash_erase(flash_sector_offset(sector), RT1176_FLASH_SECTOR_SIZE) == 0;
}

bool Storage::_flash_erase_ok(void)
{
    // only erase while disarmed - an erase stalls the whole MCU for the duration
    return !hal.util->get_soft_armed();
}
#endif  // CONFIG_AP_RT1176_ROMAPI_FLASH

bool Storage::erase()
{
    _storage.fill(0);

#ifdef CONFIG_ZMS
    if (_zms_ok) {
        zms_clear(&_zms);
        _zms_ok = (zms_mount(&_zms) == 0);
    }
#endif

#ifdef __ZEPHYR__
    if (_fram_ok) {
        _fram_write(0, _storage.data(), _storage.size());
    }
#endif

#ifdef CONFIG_FAT_FILESYSTEM_ELM
    if (_file_ok) {
        _write_file();
    }
#endif

    /* Every backend above was just rewritten synchronously and in full, so
       any chunks still marked dirty from writes that predate this erase()
       are moot - their data no longer exists. Without this, _timer_tick()
       would spend its next several ticks re-flushing chunks that erase()
       already handled. */
    _dirty_mask.clearall();

    return true;
}

void Storage::read_block(void *dst, uint16_t src, size_t n)
{
    if (dst == nullptr || src >= (uint16_t)_storage.size()) {
        return;
    }
    const size_t count = std::min<size_t>(n, _storage.size() - src);
    memcpy(dst, _storage.data() + src, count);
}

void Storage::write_block(uint16_t dst, const void *src, size_t n)
{
    if (src == nullptr || dst >= (uint16_t)_storage.size()) {
        return;
    }

    const size_t count = std::min<size_t>(n, _storage.size() - dst);
    memcpy(_storage.data() + dst, src, count);

    /* Backend writes are deferred to _timer_tick() (ChibiOS parity - see
       Storage.h's comment on _timer_tick()). This call only updates RAM
       and marks the touched chunks dirty; it must never block. */
    const uint16_t first_chunk = dst / CHUNK_SIZE;
    const uint16_t last_chunk  = (uint16_t)((dst + count - 1) / CHUNK_SIZE);
    for (uint16_t i = first_chunk; i <= last_chunk; i++) {
        _dirty_mask.set(i);
    }
}

void Storage::_timer_tick(void)
{
    /* One dirty chunk per call, matching AP_HAL_ChibiOS/Storage.cpp's own
       "write one storage line" _timer_tick() - Scheduler::_storage_thread_fn
       calls this every 1ms, so a full-storage flush (NUM_CHUNKS chunks)
       completes in well under NUM_CHUNKS milliseconds without ever holding
       up the caller that actually called write_block(). */
    const int16_t i = _dirty_mask.first_set();
    if (i < 0) {
        /* Nothing pending: every write has landed. healthy() keys off this
           timestamp going stale (ChibiOS parity - its _timer_tick does the
           same with _last_empty_ms), so a backend that stops accepting
           writes eventually fails the "Param storage failed" prearm. */
        _last_empty_ms = AP_HAL::millis();
        return;
    }
    const uint16_t chunk = (uint16_t)i;
    const uint32_t off = (uint32_t)chunk * CHUNK_SIZE;
    bool ok = true;

#ifdef CONFIG_AP_RT1176_ROMAPI_FLASH
    if (_flash_ok) {
        /* AP_FlashStorage appends the changed range to the active sector, so a write is
         * an append until the sector fills and is compacted. */
        if (!_flash.write(off, CHUNK_SIZE)) {
            ok = false;
            if (!_flash_write_failed) {
                _flash_write_failed = true;
                printk("AP_HAL_Zephyr Storage: flash write failed at %u (+%u)\n",
                       (unsigned)off, (unsigned)CHUNK_SIZE);
            }
        } else {
            _flash_write_failed = false;
        }
    }
#endif
#ifdef CONFIG_ZMS
    if (_zms_ok) {
        _write_chunk_zms(chunk);
    }
#endif
#ifdef __ZEPHYR__
    if (_fram_ok) {
        _fram_write(off, _storage.data() + off, CHUNK_SIZE);
    }
#endif
#ifdef CONFIG_FAT_FILESYSTEM_ELM
    if (_file_ok) {
        _write_file_chunk(chunk);
    }
#endif

    if (ok) {
        _dirty_mask.clear(chunk);
    }
}

bool Storage::healthy()
{
    /* ChibiOS parity: initialised AND the dirty queue has drained within the
       last 2 seconds. A backend stuck failing writes leaves its chunk dirty
       forever, _last_empty_ms goes stale, and this goes false - which is
       what arms AP_Arming's "Param storage failed" prearm check. */
    return _healthy && (AP_HAL::millis() - _last_empty_ms < 2000u);
}

// --- ZMS helpers ---

#ifdef CONFIG_ZMS
void Storage::_write_chunk_zms(uint16_t chunk_idx)
{
    if (chunk_idx >= NUM_CHUNKS) {
        return;
    }
    const uint8_t *chunk = _storage.data() + chunk_idx * CHUNK_SIZE;
    // zms_write is a no-op when data hasn't changed
    zms_write(&_zms, (zms_id_t)(chunk_idx + 1), chunk, CHUNK_SIZE);
}
#endif

// --- FRAM SPI helpers ---

#ifdef __ZEPHYR__
/* FM25V02-class parts (<= 64 KiB, e.g. CubeOrange) take 2-byte FRAM
   addresses; larger parts take 3-byte. Derived from the DTS size
   property so the same code serves both. */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(fram0), okay)
#define FRAM_ADDR_LEN ((DT_PROP(DT_NODELABEL(fram0), size) > 65536) ? 3u : 2u)  // >64KiB needs 3-byte addr
#else
#define FRAM_ADDR_LEN 3u
#endif

static size_t fram_cmd(uint8_t opcode, uint32_t addr, uint8_t cmd[4])
{
    size_t i = 0;
    cmd[i++] = opcode;
    if (FRAM_ADDR_LEN == 3u) {
        cmd[i++] = static_cast<uint8_t>((addr >> 16) & 0xFF);
    }
    cmd[i++] = static_cast<uint8_t>((addr >> 8) & 0xFF);
    cmd[i++] = static_cast<uint8_t>(addr & 0xFF);
    return i;
}

bool Storage::_fram_wren()
{
    if (_fram == nullptr || !spi_is_ready_dt(_fram)) {
        return false;
    }
    const uint8_t cmd = CMD_WREN;
    const struct spi_buf tx = { .buf = const_cast<uint8_t *>(&cmd), .len = 1 };
    const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

    struct spi_config cfg = _fram->config;
    cfg.frequency = MIN(_fram->config.frequency, 10000000U);
    return spi_write(_fram->bus, &cfg, &tx_set) == 0;
}

bool Storage::_fram_read(uint32_t addr, uint8_t *dst, size_t n)
{
    if (_fram == nullptr || dst == nullptr) {
        return false;
    }
    uint8_t cmd[4];
    const size_t cmd_len = fram_cmd(CMD_READ, addr, cmd);
    uint8_t discard[4] = {};

    const struct spi_buf tx_bufs[] = {
        { .buf = cmd,     .len = cmd_len },
        { .buf = nullptr, .len = n       },
    };
    const struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs) };

    struct spi_buf rx_bufs[] = {
        { .buf = discard, .len = cmd_len },
        { .buf = dst,     .len = n       },
    };
    const struct spi_buf_set rx_set = { .buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs) };

    struct spi_config cfg = _fram->config;
    cfg.frequency = MIN(_fram->config.frequency, 10000000U);
    return spi_transceive(_fram->bus, &cfg, &tx_set, &rx_set) == 0;
}

bool Storage::_fram_write(uint32_t addr, const uint8_t *src, size_t n)
{
    if (_fram == nullptr || src == nullptr) {
        return false;
    }
    if (!_fram_wren()) {
        return false;
    }
    uint8_t cmd[4];
    const size_t cmd_len = fram_cmd(CMD_WRITE, addr, cmd);
    const struct spi_buf tx_bufs[] = {
        { .buf = cmd,                            .len = cmd_len },
        { .buf = const_cast<uint8_t *>(src),    .len = n       },
    };
    const struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs) };

    struct spi_config cfg = _fram->config;
    cfg.frequency = MIN(_fram->config.frequency, 10000000U);
    return spi_write(_fram->bus, &cfg, &tx_set) == 0;
}
#endif  // __ZEPHYR__

// --- FAT filesystem helpers ---

void Storage::_try_file_mount()
{
#ifdef CONFIG_FAT_FILESYSTEM_ELM
    printk("### SD: fs_mount START ###\n");
    int rc = fs_mount(&s_fat_mount);
    printk("### SD: fs_mount rc=%d %s ###\n", rc, (rc == 0 || rc == -EEXIST) ? "OK" : "FAIL");
    if (rc != 0 && rc != -EEXIST) {
        return;
    }

    fs_file_t_init(&s_fat_file);
    rc = fs_open(&s_fat_file, SD_FILE_PATH, FS_O_RDWR);
    if (rc == 0) {
        ssize_t n = fs_read(&s_fat_file, _storage.data(), _storage.size());
        printk("AP_HAL_Zephyr Storage: loaded %d B from SD card\n", (int)n);
    } else {
        rc = fs_open(&s_fat_file, SD_FILE_PATH, FS_O_CREATE | FS_O_RDWR);
        if (rc != 0) {
            printk("AP_HAL_Zephyr Storage: SD file create failed (%d) -- RAM-only\n", rc);
            return;
        }
        fs_write(&s_fat_file, _storage.data(), _storage.size());
        fs_sync(&s_fat_file);
        printk("AP_HAL_Zephyr Storage: created %s (%u B)\n",
               SD_FILE_PATH, (unsigned)_storage.size());
    }

    s_fat_file_open = true;
    _file_ok        = true;
    printk("AP_HAL_Zephyr Storage: using SD card at %s\n", SD_FILE_PATH);
#endif
}

void Storage::_write_file()
{
#ifdef CONFIG_FAT_FILESYSTEM_ELM
    if (!s_fat_file_open) {
        return;
    }
    fs_seek(&s_fat_file, 0, FS_SEEK_SET);
    fs_write(&s_fat_file, _storage.data(), _storage.size());
    fs_sync(&s_fat_file);
#endif
}

void Storage::_write_file_chunk(uint16_t chunk_idx)
{
#ifdef CONFIG_FAT_FILESYSTEM_ELM
    if (!s_fat_file_open || chunk_idx >= NUM_CHUNKS) {
        return;
    }
    fs_seek(&s_fat_file, (off_t)chunk_idx * CHUNK_SIZE, FS_SEEK_SET);
    fs_write(&s_fat_file, _storage.data() + chunk_idx * CHUNK_SIZE, CHUNK_SIZE);
    fs_sync(&s_fat_file);
#endif
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
