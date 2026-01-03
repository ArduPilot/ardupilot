/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_RP2350_ENABLED

#include "AP_Filesystem.h"
#include "AP_Filesystem_RP2350.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_RTC/AP_RTC.h>

//#define AP_LFS_DEBUG
#ifdef AP_LFS_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

using namespace RP;

static int flashmem_read(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    void* buffer, lfs_size_t size
);
static int flashmem_prog(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    const void* buffer, lfs_size_t size
);
static int flashmem_erase(const struct lfs_config *cfg, lfs_block_t block);
static int flashmem_sync(const struct lfs_config *cfg);
static void get_nand_addr(uint32_t offset, uint32_t pageSize, uint32_t pagesPerBlock, uint32_t &block, uint32_t &page);

const extern AP_HAL::HAL& hal;

/* ************************************************************************* */
/* Low-level flash memory access                                             */
/* ************************************************************************* */

bool AP_Filesystem_RP2350::init_flash()
{
    WITH_SEMAPHORE(dev_sem);
    if (!wait_until_device_is_ready()) {
        return false;
    }
    return true;
}

bool AP_Filesystem_RP2350::write_enable()
{
    if (!wait_until_device_is_ready()) {
        return false;
    }

    WITH_SEMAPHORE(dev_sem);
    dev->write_enable();
    return true;
}

bool AP_Filesystem_RP2350::is_busy()
{
    WITH_SEMAPHORE(dev_sem);
    return dev->is_busy();
}

bool AP_Filesystem_RP2350::mount_filesystem()
{
    if (dead) {
        return false;
    }

    if (mounted) {
        return true;
    }

    EXPECT_DELAY_MS(3000);

    fs_cfg.context = this;
    fs_cfg.read = flashmem_read;
    fs_cfg.prog = flashmem_prog;
    fs_cfg.erase = flashmem_erase;
    fs_cfg.sync = flashmem_sync;

    HAL_RP &hal_rp = (HAL_RP&)hal;
    
    dev = hal_rp.get_nand_pio(); 
    
    if (!dev) {
        mark_dead();
        return false;
    }

    dev_sem = dev->get_semaphore();

    uint32_t id = find_block_size_and_count();

    if (!id) {
        mark_dead();
        return false;
    }
    if (!init_flash()) {
        mark_dead();
        return false;
    }
    if (lfs_mount(&fs, &fs_cfg) < 0) {
        /* maybe not formatted? try formatting it */
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Formatting flash");

        if (lfs_format(&fs, &fs_cfg) < 0) {
            mark_dead();
            return false;
        }

        /* try mounting again */
        if (lfs_mount(&fs, &fs_cfg) < 0) {
            /* cannot mount after formatting */
            mark_dead();
            return false;
        }
    }

    // try to create the root storage folder. Ignore the error code in case
    // the filesystem is corrupted or it already exists.
    if (strlen(HAL_BOARD_STORAGE_DIRECTORY) > 0) {
        lfs_mkdir(&fs, HAL_BOARD_STORAGE_DIRECTORY);
    }

    // Force garbage collection to avoid expensive operations after boot
    lfs_fs_gc(&fs);
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Mounted flash 0x%x as littlefs", unsigned(id));
    mounted = true;
    return true;
}

uint32_t AP_Filesystem_RP2350::find_block_size_and_count()
{
    if (!wait_until_device_is_ready()) {
        return false;
    }

    WITH_SEMAPHORE(dev_sem);

    // Get manufacturer ID
    uint32_t id = dev->get_manufacturer_id();

    // Let's specify the terminology here.
    //
    // 1 block = smallest unit that we _erase_ in a single operation
    // 1 page = smallest unit that we read or program in a single operation
    //
    // regardless of what the flash chip documentation refers to as a "block"
    lfs_size_t page_size = dev->get_page_size();
    lfs_size_t block_size = dev->get_block_size();

    lfs_size_t block_count = dev->get_block_count();
 
    fs_cfg.read_size = page_size;
    fs_cfg.prog_size = page_size; // we assume this is equal to read_size!
    fs_cfg.block_size = block_size;
    fs_cfg.block_count = block_count;

    // larger metadata blocks mean less frequent compaction operations, but each
    // takes longer. however, the total time spent compacting for a given file
    // size still goes down with larger metadata blocks. 4096 was chosen to
    // minimize both frequency and log buffer utilization.
    fs_cfg.metadata_max = 4096;
    fs_cfg.compact_thresh = 4096*0.88f;
    fs_cfg.metadata_max = page_size * 2;
    fs_cfg.compact_thresh = fs_cfg.metadata_max * 0.88f;

    fs_cfg.block_cycles = 75000;
    // cache entire flash state in RAM (8 blocks = 1 byte of storage) to
    // avoid scanning while logging
    fs_cfg.lookahead_size = fs_cfg.block_count/8;
    // non-inlined files require their own block, but must be copie. Generally we have requirements for tiny files
    // (scripting) and very large files (e.g. logging), but not much in-between. Setting the cache size will also
    // limit the inline size.
    fs_cfg.cache_size = fs_cfg.prog_size;

    return id;
}

#ifdef AP_LFS_DEBUG
static uint32_t page_writes;
static uint32_t last_write_msg_ms;
static uint32_t page_reads;
static uint32_t block_erases;
#endif
int AP_Filesystem_RP2350::_flashmem_read(
    lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size
) {
    EXPECT_DELAY_MS((25*size)/(fs_cfg.read_size*1000));

    // LittleFS always calls us with off aligned to read_size and size a
    // multiple of read_size
    const uint32_t page_size = fs_cfg.read_size;
    uint32_t num_pages = size / page_size;
    uint32_t address = (block * fs_cfg.block_size) + off;
    uint8_t* p = static_cast<uint8_t*>(buffer);

    while (num_pages--) {
#ifdef AP_LFS_DEBUG
        page_reads++;
#endif
        if (!wait_until_device_is_ready()) {
            return LFS_ERR_IO;
        }
        uint32_t _block, _page;
        uint32_t _pagesPerBlock = fs_cfg.block_size / page_size;
        get_nand_addr(address, page_size, _pagesPerBlock, _block, _page);
        if (_block != block) {
            debug("Block number is different : _block = %ld, block = %ld\n");
        }
        if (!dev->read_page(_block, _page, p)) {
            return LFS_ERR_IO;
        }

        address += page_size;
        p += page_size;
    }
    return LFS_ERR_OK;
}

int AP_Filesystem_RP2350::_flashmem_prog(
    lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size
) {
    EXPECT_DELAY_MS((250*size)/(fs_cfg.read_size*1000));

    // LittleFS always calls us with off aligned to prog_size and size a
    // multiple of prog_size (which we set equal to read_size)
    const uint32_t page_size = fs_cfg.read_size;
    uint32_t num_pages = size / page_size;
    uint32_t address = (block * fs_cfg.block_size) + off;
    const uint8_t* p = static_cast<const uint8_t*>(buffer);

    while (num_pages--) {
        if (!write_enable()) {
            return LFS_ERR_IO;
        }

#ifdef AP_LFS_DEBUG
        page_writes++;
        if (AP_HAL::millis() - last_write_msg_ms > 5000) {
            debug("LFS: writes %lukB/s, pages %lu/s (reads %lu/s, block erases %lu/s)",
                (page_writes*page_size)/(5*1024), page_writes/5, page_reads/5, block_erases/5);
            page_writes = 0;
            page_reads = 0;
            block_erases = 0;
            last_write_msg_ms = AP_HAL::millis();
        }
#endif

        WITH_SEMAPHORE(dev_sem);
        uint32_t _block, _page;
        uint32_t _pagesPerBlock = fs_cfg.block_size / page_size;
        get_nand_addr(address, page_size, _pagesPerBlock, _block, _page);
        if (_block != block) {
            debug("Block number is different : _block = %ld, block = %ld\n");
        }
        if (!dev->write_page(_block, _page, p)) {
            return LFS_ERR_IO;
        }

        // writing simply means the data is in the internal cache, it will take
        // some period to propagate to the flash itself

        address += page_size;
        p += page_size;
    }
    return LFS_ERR_OK;
}

int AP_Filesystem_RP2350::_flashmem_erase(lfs_block_t block) {
    if (!write_enable()) {
        return LFS_ERR_IO;
    }

#ifdef AP_LFS_DEBUG
    block_erases++;
#endif

    WITH_SEMAPHORE(dev_sem);
    dev->erase_block(static_cast<uint32_t>(block));
    // sleep so that other processes get the CPU cycles that the 4ms erase cycle needs.
    hal.scheduler->delay(4);

    return LFS_ERR_OK;
}

int AP_Filesystem_RP2350::_flashmem_sync() {
    if (wait_until_device_is_ready()) {
        return LFS_ERR_OK;
    } else {
        return LFS_ERR_IO;
    }
}

static int flashmem_read(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    void* buffer, lfs_size_t size
) {
    AP_Filesystem_RP2350* self = static_cast<AP_Filesystem_RP2350*>(cfg->context);
    return self->_flashmem_read(block, off, buffer, size);
}

static int flashmem_prog(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    const void* buffer, lfs_size_t size
) {
    AP_Filesystem_RP2350* self = static_cast<AP_Filesystem_RP2350*>(cfg->context);
    return self->_flashmem_prog(block, off, buffer, size);
}

static int flashmem_erase(const struct lfs_config *cfg, lfs_block_t block) {
    AP_Filesystem_RP2350* self = static_cast<AP_Filesystem_RP2350*>(cfg->context);
    return self->_flashmem_erase(block);
}

static int flashmem_sync(const struct lfs_config *cfg) {
    AP_Filesystem_RP2350* self = static_cast<AP_Filesystem_RP2350*>(cfg->context);
    return self->_flashmem_sync();
}

static void get_nand_addr(uint32_t offset, uint32_t pageSize, uint32_t pagesPerBlock, uint32_t &block, uint32_t &page)
{
    uint32_t absolute_page = offset / pageSize;
    block = absolute_page / pagesPerBlock;
    page = absolute_page % pagesPerBlock;
}

#endif // AP_FILESYSTEM_RP2350_ENABLED
