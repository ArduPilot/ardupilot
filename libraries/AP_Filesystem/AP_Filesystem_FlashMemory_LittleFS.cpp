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
/*
  ArduPilot filesystem interface for systems using the LittleFS filesystem in
  flash memory

  Original littlefs integration by Tamas Nepusz <ntamas@gmail.com>
  Further development by Andy Piper <github@andypiper.com>
*/
#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_LITTLEFS_ENABLED

#include "AP_Filesystem.h"
#include "AP_Filesystem_FlashMemory_LittleFS.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_RTC/AP_RTC.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "bd/lfs_filebd.h"
#include <cstdio>
#endif

// Uncomment to enable LittleFS flash I/O debug output
// #define AP_LFS_DEBUG
#ifdef AP_LFS_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

#define ENSURE_MOUNTED() do { if (!mounted && !mount_filesystem()) { errno = EIO; return -1; }} while (0)
#define ENSURE_MOUNTED_NULL() do { if (!mounted && !mount_filesystem()) { errno = EIO; return nullptr; }} while (0)
#define LFS_CHECK(func) do { int __retval = func; if (__retval < 0) { errno = errno_from_lfs_error(__retval); return -1; }} while (0)
#define LFS_CHECK_NULL(func) do { int __retval = func; if (__retval < 0) { errno = errno_from_lfs_error(__retval); return nullptr; }} while (0)
#define LFS_ATTR_MTIME 'M'

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
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
#endif
static int errno_from_lfs_error(int lfs_error);
static int lfs_flags_from_flags(int flags);

const extern AP_HAL::HAL& hal;

int AP_Filesystem_FlashMemory_LittleFS::open(const char *pathname, int flags, bool allow_absolute_path)
{
    FS_CHECK_ALLOWED(-1);
    WITH_SEMAPHORE(fs_sem);

    ENSURE_MOUNTED();

    int fd = allocate_fd();
    if (fd < 0) {
        return -1;
    }

    FileDescriptor* fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    // file is closed, see if we already have a modification time
    if (lfs_getattr(&fs, pathname, LFS_ATTR_MTIME, &fp->mtime, sizeof(fp->mtime)) != sizeof(fp->mtime)) {
        // no attribute, populate from RTC
        uint64_t utc_usec = 0;
        AP::rtc().get_utc_usec(utc_usec);
        fp->mtime = utc_usec/(1000U*1000U);
    }

    // populate the file config with the mtime attribute, will get written out on first sync or close
    fp->cfg.attrs = fp->attrs;
    fp->cfg.attr_count = 1;
    fp->attrs[0] = {
        .type = LFS_ATTR_MTIME,
        .buffer = &fp->mtime,
        .size = sizeof(fp->mtime)
    };
    fp->filename = strdup(pathname);
    if (fp->filename == nullptr) {
        errno = ENOMEM;
        free_fd(fd);
        return -1;
    }

    int retval = lfs_file_opencfg(&fs, &fp->file, pathname, lfs_flags_from_flags(flags), &fp->cfg);

    if (retval < 0) {
        errno = errno_from_lfs_error(retval);
        free_fd(fd);
        return -1;
    }


    return fd;
}

int AP_Filesystem_FlashMemory_LittleFS::close(int fileno)
{
    FS_CHECK_ALLOWED(-1);
    WITH_SEMAPHORE(fs_sem);

    FileDescriptor* fp = lfs_file_from_fd(fileno);
    if (fp == nullptr) {
        return -1;
    }

    int retval = lfs_file_close(&fs, &(fp->file));
    if (retval < 0) {
        free_fd(fileno);   // ignore error code, we have something else to report
        errno = errno_from_lfs_error(retval);
        return -1;
    }

    if (free_fd(fileno) < 0) {
        return -1;
    }

    return 0;
}

int32_t AP_Filesystem_FlashMemory_LittleFS::read(int fd, void *buf, uint32_t count)
{
    FS_CHECK_ALLOWED(-1);
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    FileDescriptor* fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    lfs_ssize_t read = lfs_file_read(&fs, &(fp->file), buf, count);
    if (read < 0) {
        errno = errno_from_lfs_error(read);
        return -1;
    }

    return read;
}

int32_t AP_Filesystem_FlashMemory_LittleFS::write(int fd, const void *buf, uint32_t count)
{
    FS_CHECK_ALLOWED(-1);
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    FileDescriptor* fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    lfs_ssize_t written = lfs_file_write(&fs, &(fp->file), buf, count);
    if (written < 0) {
        errno = errno_from_lfs_error(written);
        return -1;
    }

    return written;
}

int AP_Filesystem_FlashMemory_LittleFS::fsync(int fd)
{
    FS_CHECK_ALLOWED(-1);
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    FileDescriptor* fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    if (fp->file.off != fs_cfg.block_size) {
        debug("misaligned fsync: %lu\n", fp->file.off);
    }

    LFS_CHECK(lfs_file_sync(&fs, &(fp->file)));
    return 0;
}

int32_t AP_Filesystem_FlashMemory_LittleFS::lseek(int fd, int32_t position, int whence)
{
    FS_CHECK_ALLOWED(-1);
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    FileDescriptor* fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    int lfs_whence;
    switch (whence) {
    case SEEK_END:
        lfs_whence = LFS_SEEK_SET;
        break;
    case SEEK_CUR:
        lfs_whence = LFS_SEEK_CUR;
        break;
    case SEEK_SET:
    default:
        lfs_whence = LFS_SEEK_SET;
        break;
    }

    lfs_soff_t pos = lfs_file_seek(&fs, &(fp->file), position, lfs_whence);
    if (pos < 0) {
        errno = errno_from_lfs_error(pos);
        return -1;
    }

    return pos;
}

int AP_Filesystem_FlashMemory_LittleFS::stat(const char *name, struct stat *buf)
{
    FS_CHECK_ALLOWED(-1);
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    lfs_info info;
    LFS_CHECK(lfs_stat(&fs, name, &info));

    memset(buf, 0, sizeof(*buf));
    uint32_t mtime;
    if (lfs_getattr(&fs, name, LFS_ATTR_MTIME, &mtime, sizeof(mtime)) == sizeof(mtime)) {
        buf->st_mtime = mtime;
        buf->st_atime = mtime;
        buf->st_ctime = mtime;
    }
    buf->st_mode = (info.type == LFS_TYPE_DIR ? S_IFDIR : S_IFREG) | 0666;
    buf->st_nlink = 1;
    buf->st_size = info.size;
    buf->st_blksize = fs_cfg.read_size;
    buf->st_uid=1000;
    buf->st_gid=1000;
    buf->st_blocks = (info.size >> 9) + ((info.size & 0x1FF) > 0 ? 1 : 0);

    return 0;
}

// set modification time on a file
bool AP_Filesystem_FlashMemory_LittleFS::set_mtime(const char *filename, const uint32_t mtime_sec)
{
    // unfortunately lfs_setattr will not work while the file is open, instead
    // we need to update the file config, but that means finding the file config
    for (int fd = 0; fd < MAX_OPEN_FILES; fd++) {
        if (open_files[fd] != nullptr && strcmp(open_files[fd]->filename, filename) == 0) {
            open_files[fd]->mtime = mtime_sec;
            return true;
        }
    }
    return false;
}

int AP_Filesystem_FlashMemory_LittleFS::unlink(const char *pathname)
{
    FS_CHECK_ALLOWED(-1);
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();
    LFS_CHECK(lfs_remove(&fs, pathname));
    return 0;
}

int AP_Filesystem_FlashMemory_LittleFS::mkdir(const char *pathname)
{
    FS_CHECK_ALLOWED(-1);
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();
    LFS_CHECK(lfs_mkdir(&fs, pathname));
    return 0;
}


struct DirEntry {
    lfs_dir_t dir;
    struct dirent entry;
};

void *AP_Filesystem_FlashMemory_LittleFS::opendir(const char *pathdir)
{
    FS_CHECK_ALLOWED(nullptr);
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED_NULL();

    DirEntry *result = NEW_NOTHROW DirEntry;
    if (!result) {
        errno = ENOMEM;
        return nullptr;
    }

    int retval = lfs_dir_open(&fs, &result->dir, pathdir);
    if (retval < 0) {
        delete result;
        errno = errno_from_lfs_error(retval);
        return nullptr;
    }

    memset(&result->entry, 0, sizeof(result->entry));

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    result->entry.d_reclen = sizeof(result->entry);
#endif

    // LittleFS has issues with opendir where filesystem operations that trigger
    // writes while a directory is open can break the iteration and cause
    // LittleFS to report filesystem corruption. We hope readdir loops don't do
    // writes (none do in ArduPilot at the time of writing), but also take the
    // lock again so other threads can't write until the corresponding release
    // in closedir. This is safe here since the lock is recursive; recursion
    // also lets the thread with the directory open do reads. Hopefully this
    // will be fixed upstream so we can remove this quirk.
    fs_sem.take_blocking();

    return result;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
struct dirent *AP_Filesystem_FlashMemory_LittleFS::readdir(void *ptr)
{
    FS_CHECK_ALLOWED(nullptr);
    WITH_SEMAPHORE(fs_sem);

    DirEntry *pair = static_cast<DirEntry*>(ptr);
    if (!pair) {
        errno = EINVAL;
        return nullptr;
    }

    lfs_info info;
    int retval = lfs_dir_read(&fs, &pair->dir, &info);
    if (retval == 0) {
        /* no more entries */
        return nullptr;
    }
    if (retval < 0) {
        // failure
        errno = errno_from_lfs_error(retval);
        return nullptr;
    }

    memset(&pair->entry, 0, sizeof(pair->entry));

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    pair->entry.d_ino = 0;
    pair->entry.d_seekoff++;
#endif

    strncpy(pair->entry.d_name, info.name, MIN(strlen(info.name)+1, sizeof(pair->entry.d_name)));
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    pair->entry.d_namlen = strlen(info.name);
#endif

    pair->entry.d_type = info.type == LFS_TYPE_DIR ? DT_DIR : DT_REG;

    return &pair->entry;
}
#pragma GCC diagnostic pop

int AP_Filesystem_FlashMemory_LittleFS::closedir(void *ptr)
{
    FS_CHECK_ALLOWED(-1);
    WITH_SEMAPHORE(fs_sem);

    DirEntry *pair = static_cast<DirEntry*>(ptr);
    if (!pair) {
        errno = EINVAL;
        return 0;
    }

    // Before the close, undo the lock we did in opendir so it's released even
    // if the close fails. We don't undo it above, as the input being nullptr
    // means we didn't successfully open the directory and lock.
    fs_sem.give();

    int retval = lfs_dir_close(&fs, &pair->dir);
    delete pair;

    if (retval < 0) {
        errno = errno_from_lfs_error(retval);
        return -1;
    }

    return 0;
}

// return number of bytes that should be written before fsync for optimal
// streaming performance/robustness. if zero, any number can be written.
// LittleFS needs to copy the block contents to a new one if fsync is called
// in the middle of a block. LittleFS also is guaranteed to not remember any
// file contents until fsync is called!
uint32_t AP_Filesystem_FlashMemory_LittleFS::bytes_until_fsync(int fd)
{
    FS_CHECK_ALLOWED(0);
    WITH_SEMAPHORE(fs_sem);

    FileDescriptor* fp = lfs_file_from_fd(fd);
    if (!mounted || fp == nullptr) {
        return 0;
    }

    uint32_t file_pos = fp->file.pos;
    uint32_t block_size = fs_cfg.block_size;

    // first block exclusively stores data:
    // https://github.com/littlefs-project/littlefs/issues/564#issuecomment-2555733922
    if (file_pos < block_size) {
        return block_size - file_pos; // so block_offset is exactly file_pos
    }

    // see https://github.com/littlefs-project/littlefs/issues/564#issuecomment-2363032827
    // n = (N − w/8 ( popcount( N/(B − 2w/8) − 1) + 2))/(B − 2w/8))
    // off = N − ( B − 2w/8 ) n − w/8popcount( n )
#define BLOCK_INDEX(N, B) \
    (N - sizeof(uint32_t) * (__builtin_popcount(N/(B - 2 * sizeof(uint32_t)) -1) + 2))/(B - 2 * sizeof(uint32_t))

#define BLOCK_OFFSET(N, B, n) \
    (N - (B - 2*sizeof(uint32_t)) * n - sizeof(uint32_t) * __builtin_popcount(n))

    uint32_t block_index = BLOCK_INDEX(file_pos, block_size);
    // offset will be 4 (or bigger) through (block_size-1) as subsequent blocks
    // start with one or more pointers; offset will never equal block_size
    uint32_t block_offset = BLOCK_OFFSET(file_pos, block_size, block_index);

#undef BLOCK_INDEX
#undef BLOCK_OFFSET

    return block_size - block_offset;
}


int64_t AP_Filesystem_FlashMemory_LittleFS::disk_free(const char *path)
{
    FS_CHECK_ALLOWED(-1);
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    lfs_ssize_t alloc_size = lfs_fs_size(&fs);
    if (alloc_size < 0) {
        errno = errno_from_lfs_error(alloc_size);
        return -1;
    }

    return disk_space(path) - alloc_size;
}

int64_t AP_Filesystem_FlashMemory_LittleFS::disk_space(const char *path)
{
    return fs_cfg.block_count * fs_cfg.block_size;
}

bool AP_Filesystem_FlashMemory_LittleFS::retry_mount(void)
{
    FS_CHECK_ALLOWED(false);
    WITH_SEMAPHORE(fs_sem);

    if (!dead) {
        if (!mounted && !mount_filesystem()) {
            errno = EIO;
            return false;
        }

        return true;
    } else {
        return false;
    }
}

void AP_Filesystem_FlashMemory_LittleFS::unmount(void)
{
    WITH_SEMAPHORE(fs_sem);

    if (mounted && !dead) {
        free_all_fds();
        lfs_unmount(&fs);
        mounted = false;
    }
}

/* ************************************************************************* */
/* Private functions                                                         */
/* ************************************************************************* */

int AP_Filesystem_FlashMemory_LittleFS::allocate_fd()
{
    for (int fd = 0; fd < MAX_OPEN_FILES; fd++) {
        if (open_files[fd] == nullptr) {
            open_files[fd] = static_cast<FileDescriptor*>(calloc(1, sizeof(FileDescriptor)));
            if (open_files[fd] == nullptr) {
                errno = ENOMEM;
                return -1;
            }

            return fd;
        }
    }

    errno = ENFILE;
    return -1;
}

int AP_Filesystem_FlashMemory_LittleFS::free_fd(int fd)
{
    FileDescriptor* fp = lfs_file_from_fd(fd);
    if (!fp) {
        return -1;
    }

    free(fp->filename);
    free(fp);
    open_files[fd] = nullptr;

    return 0;
}

void AP_Filesystem_FlashMemory_LittleFS::free_all_fds()
{
    for (int fd = 0; fd < MAX_OPEN_FILES; fd++) {
        if (open_files[fd] != nullptr) {
            free_fd(fd);
        }
    }
}

AP_Filesystem_FlashMemory_LittleFS::FileDescriptor* AP_Filesystem_FlashMemory_LittleFS::lfs_file_from_fd(int fd) const
{
    if (fd < 0 || fd >= MAX_OPEN_FILES || open_files[fd] == nullptr) {
        errno = EBADF;
        return nullptr;
    }

    return open_files[fd];
}

void AP_Filesystem_FlashMemory_LittleFS::mark_dead()
{
    if (!dead) {
        printf("FlashMemory_LittleFS: dead\n");
        free_all_fds();
        dead = true;
    }
}

/* ************************************************************************* */
/* Low-level flash memory access                                             */
/* ************************************************************************* */

#define JEDEC_WRITE_ENABLE           0x06
#define JEDEC_READ_STATUS            0x05
#define JEDEC_WRITE_STATUS           0x01
#define JEDEC_READ_DATA              0x03
#define JEDEC_PAGE_DATA_READ         0x13
#define JEDEC_DEVICE_ID              0x9F
#define JEDEC_PAGE_WRITE             0x02
#define JEDEC_PROGRAM_EXECUTE        0x10

#define JEDEC_DEVICE_RESET           0xFF

#define JEDEC_BULK_ERASE             0xC7
#define JEDEC_SECTOR4_ERASE          0x20 // 4k erase
#define JEDEC_BLOCK_ERASE            0xD8

#define JEDEC_STATUS_BUSY            0x01
#define JEDEC_STATUS_WRITEPROTECT    0x02
#define JEDEC_STATUS_BP0             0x04
#define JEDEC_STATUS_BP1             0x08
#define JEDEC_STATUS_BP2             0x10
#define JEDEC_STATUS_TP              0x20
#define JEDEC_STATUS_SEC             0x40
#define JEDEC_STATUS_SRP0            0x80

/*
  flash device IDs taken from betaflight flash_m25p16.c

  Format is manufacturer, memory type, then capacity
*/
#define JEDEC_ID_UNKNOWN               0x000000
#define JEDEC_ID_MACRONIX_MX25L3206E   0xC22016
#define JEDEC_ID_MACRONIX_MX25L6406E   0xC22017
#define JEDEC_ID_MACRONIX_MX25L25635E  0xC22019
#define JEDEC_ID_MICRON_M25P16         0x202015
#define JEDEC_ID_MICRON_N25Q064        0x20BA17
#define JEDEC_ID_MICRON_N25Q128        0x20BA18
#define JEDEC_ID_WINBOND_W25Q16        0xEF4015
#define JEDEC_ID_WINBOND_W25Q32        0xEF4016
#define JEDEC_ID_WINBOND_W25X32        0xEF3016
#define JEDEC_ID_WINBOND_W25Q64        0xEF4017
#define JEDEC_ID_WINBOND_W25Q128       0xEF4018
#define JEDEC_ID_WINBOND_W25Q256       0xEF4019
#define JEDEC_ID_WINBOND_W25Q128_2     0xEF7018
#define JEDEC_ID_WINBOND_W25N01GV      0xEFAA21
#define JEDEC_ID_WINBOND_W25N02KV      0xEFAA22
#if AP_FILESYSTEM_LITTLEFS_MT29FXX_ENABLED
// Micron MT29FXX SPI NAND family - manufacturer 0x2C, device IDs vary by density
#define JEDEC_ID_MICRON_MT29FXX1G        0x2C1400    // 1Gbit (128MB), 2KB page
#define JEDEC_ID_MICRON_MT29FXX2G        0x2C2400    // 2Gbit (256MB), 2KB page
#define JEDEC_ID_MICRON_MT29FXX4G        0x2C3400    // 4Gbit (512MB), 4KB page (MT29F4G01ABAFD)
#endif
#define JEDEC_ID_CYPRESS_S25FL064L     0x016017
#define JEDEC_ID_CYPRESS_S25FL128L     0x016018
#define JEDEC_ID_GIGA_GD25Q16E         0xC84015
#define JEDEC_ID_ZBIT_ZB25VQ128        0x5E4018 
#define JEDEC_ID_FMSH_FM25Q64          0xA14017
#define JEDEC_ID_FMSH_FM25Q128A        0xA14018
#define JEDEC_ID_FMSH_FM25Q256         0xA14019

#if AP_FILESYSTEM_LITTLEFS_FLASH_IS_NAND
/*
 * SPI NAND command set used by the nand_* ops below. Opcodes shared with the
 * JEDEC set above are reused from there; only the SPI-NAND-specific ones are
 * defined here. 0x0F/0x1F (GET/SET FEATURE) are also accepted by W25N as
 * Read/Write Status Register, so they serve both families.
 */
#define NAND_CMD_GET_FEATURE     0x0F
#define NAND_CMD_SET_FEATURE     0x1F
#define NAND_REG_PROTECTION      0xA0
#define NAND_REG_CONFIG          0xB0
#define NAND_REG_STATUS          0xC0
#define NAND_STATUS_OIP          0x01    // Operation In Progress
#define NAND_STATUS_WEL          0x02    // Write Enable Latch
#define NAND_STATUS_EFAIL        0x04    // Erase Fail
#define NAND_STATUS_PFAIL        0x08    // Program Fail

// cache opcodes: quad on a WSPI bus, single-line on SPI
#if AP_FILESYSTEM_LITTLEFS_USE_WSPI
#define NAND_CMD_READ_CACHE      0x6B               // 1-1-4 quad read
#define NAND_CMD_PROGRAM_LOAD    0x32               // 1-1-4 quad write
#define NAND_CACHE_DUMMY         8
#else
#define NAND_CMD_READ_CACHE      JEDEC_READ_DATA    // 0x03, 1-1-1 single-line read
#define NAND_CMD_PROGRAM_LOAD    JEDEC_PAGE_WRITE   // 0x02, 1-1-1 single-line write
#endif

// config-register value and reset timing differ by chip
#if AP_FILESYSTEM_LITTLEFS_FLASH_TYPE == AP_FILESYSTEM_FLASH_WSPI_NAND
#define NAND_CONFIG_BITS         (1 << 4)               // ECC enable
#define NAND_RESET_DELAY_MS      2
#else
#define NAND_CONFIG_BITS         ((1 << 4) | (1 << 3))  // ECC enable + buffer read mode
#define NAND_RESET_DELAY_MS      500
#endif
#endif  // AP_FILESYSTEM_LITTLEFS_FLASH_IS_NAND

#if AP_FILESYSTEM_LITTLEFS_USE_WSPI
// Send a command with no address or data phase
bool AP_Filesystem_FlashMemory_LittleFS::wspi_command(uint8_t cmd)
{
    const AP_HAL::Device::CommandHeader hdr {
        .cmd = cmd,
        .cfg = AP_HAL::WSPI::CFG_CMD_MODE_ONE_LINE,
    };
    wspi_dev->set_cmd_header(hdr);
    return wspi_dev->transfer(nullptr, 0, nullptr, 0);
}
#endif

bool AP_Filesystem_FlashMemory_LittleFS::is_busy()
{
    WITH_SEMAPHORE(dev_sem);
    uint8_t status;
#if AP_FILESYSTEM_LITTLEFS_FLASH_IS_NAND
    // shared NAND path: W25N and MT29 both report OIP/PFAIL/EFAIL in the same
    // bit positions; nand_read_status hides the SPI-vs-WSPI difference.
    if (!nand_read_status(status)) {
        return true;  // assume busy on error
    }
    return (status & (NAND_STATUS_OIP | NAND_STATUS_PFAIL | NAND_STATUS_EFAIL)) != 0;
#else
    uint8_t cmd = JEDEC_READ_STATUS;
    dev->transfer(&cmd, 1, &status, 1);
    return (status & (JEDEC_STATUS_BUSY | JEDEC_STATUS_SRP0)) != 0;
#endif
}

// Send a command with an address phase, picking the transport from the build.
// WSPI uses a 24-bit address; SPI uses 24- or 32-bit per use_32bit_address.
bool AP_Filesystem_FlashMemory_LittleFS::send_command_addr(uint8_t command, uint32_t addr)
{
#if AP_FILESYSTEM_LITTLEFS_USE_WSPI
    const AP_HAL::Device::CommandHeader hdr {
        .cmd = command,
        .cfg = AP_HAL::WSPI::CFG_CMD_MODE_ONE_LINE |
               AP_HAL::WSPI::CFG_ADDR_MODE_ONE_LINE |
               AP_HAL::WSPI::CFG_ADDR_SIZE_24,
        .addr = addr,
    };
    wspi_dev->set_cmd_header(hdr);
    return wspi_dev->transfer(nullptr, 0, nullptr, 0);
#else
    uint8_t cmd[5];
    cmd[0] = command;

    if (use_32bit_address) {
        cmd[1] = (addr >> 24) & 0xff;
        cmd[2] = (addr >> 16) & 0xff;
        cmd[3] = (addr >>  8) & 0xff;
        cmd[4] = (addr >>  0) & 0xff;
    } else {
        cmd[1] = (addr >> 16) & 0xff;
        cmd[2] = (addr >>  8) & 0xff;
        cmd[3] = (addr >>  0) & 0xff;
        cmd[4] = 0;
    }

    return dev->transfer(cmd, use_32bit_address ? 5 : 4, nullptr, 0);
#endif
}

#if !AP_FILESYSTEM_LITTLEFS_USE_WSPI
void AP_Filesystem_FlashMemory_LittleFS::send_command_page(uint8_t command, uint32_t page)
{
    uint8_t cmd[3];
    cmd[0] = command;
    cmd[1] = (page >> 8) & 0xff;
    cmd[2] = (page >> 0) & 0xff;
    dev->transfer(cmd, 3, nullptr, 0);
}
#endif  // !AP_FILESYSTEM_LITTLEFS_USE_WSPI

#if AP_FILESYSTEM_LITTLEFS_FLASH_IS_NAND
// reset the device
bool AP_Filesystem_FlashMemory_LittleFS::nand_reset()
{
#if AP_FILESYSTEM_LITTLEFS_USE_WSPI
    return wspi_command(JEDEC_DEVICE_RESET);
#else
    uint8_t b = JEDEC_DEVICE_RESET;
    return dev->transfer(&b, 1, nullptr, 0);
#endif
}

// read the JEDEC ID and assemble it manufacturer-first (e.g. 0x2C1400, 0xEFAA21)
bool AP_Filesystem_FlashMemory_LittleFS::nand_read_id(uint32_t &id)
{
    uint8_t buf[4] {};
#if AP_FILESYSTEM_LITTLEFS_USE_WSPI
    // cmd(0x9F) + addr(0x00) + 8 dummy cycles shifts the read window by one byte,
    // so buf[0]=Device ID, buf[1]=Manufacturer ID
    const AP_HAL::Device::CommandHeader hdr {
        .cmd = JEDEC_DEVICE_ID,
        .cfg = AP_HAL::WSPI::CFG_CMD_MODE_ONE_LINE |
               AP_HAL::WSPI::CFG_ADDR_MODE_ONE_LINE |
               AP_HAL::WSPI::CFG_ADDR_SIZE_8 |
               AP_HAL::WSPI::CFG_DATA_MODE_ONE_LINE,
        .addr = 0x00,
        .dummy = 8,
    };
    wspi_dev->set_cmd_header(hdr);
    if (!wspi_dev->transfer(nullptr, 0, buf, 2)) {
        return false;
    }
#else
    uint8_t cmd = JEDEC_DEVICE_ID;
    dev->transfer(&cmd, 1, buf, 4);
#endif
#if AP_FILESYSTEM_LITTLEFS_FLASH_TYPE == AP_FILESYSTEM_FLASH_WSPI_NAND
    id = (static_cast<uint32_t>(buf[1]) << 16) | (static_cast<uint32_t>(buf[0]) << 8);
#else
    id = (static_cast<uint32_t>(buf[1]) << 16) | (static_cast<uint32_t>(buf[2]) << 8) | buf[3];
#endif
    return true;
}

// read the chip status/feature register (busy + program/erase fail bits)
bool AP_Filesystem_FlashMemory_LittleFS::nand_read_status(uint8_t &status)
{
#if AP_FILESYSTEM_LITTLEFS_USE_WSPI
    const AP_HAL::Device::CommandHeader hdr {
        .cmd = NAND_CMD_GET_FEATURE,
        .cfg = AP_HAL::WSPI::CFG_CMD_MODE_ONE_LINE |
               AP_HAL::WSPI::CFG_ADDR_MODE_ONE_LINE |
               AP_HAL::WSPI::CFG_ADDR_SIZE_8 |
               AP_HAL::WSPI::CFG_DATA_MODE_ONE_LINE,
        .addr = NAND_REG_STATUS,
    };
    wspi_dev->set_cmd_header(hdr);
    return wspi_dev->transfer(nullptr, 0, &status, 1);
#else
    uint8_t cmd[2] { NAND_CMD_GET_FEATURE, NAND_REG_STATUS };
    return dev->transfer(cmd, 2, &status, 1);
#endif
}

// write a feature/status register (protection, config)
bool AP_Filesystem_FlashMemory_LittleFS::nand_set_reg(uint8_t reg, uint8_t value)
{
#if AP_FILESYSTEM_LITTLEFS_USE_WSPI
    const AP_HAL::Device::CommandHeader hdr {
        .cmd = NAND_CMD_SET_FEATURE,
        .cfg = AP_HAL::WSPI::CFG_CMD_MODE_ONE_LINE |
               AP_HAL::WSPI::CFG_ADDR_MODE_ONE_LINE |
               AP_HAL::WSPI::CFG_ADDR_SIZE_8 |
               AP_HAL::WSPI::CFG_DATA_MODE_ONE_LINE,
        .addr = reg,
    };
    wspi_dev->set_cmd_header(hdr);
    return wspi_dev->transfer(&value, 1, nullptr, 0);
#else
    uint8_t cmd[3] { NAND_CMD_SET_FEATURE, reg, value };
    return dev->transfer(cmd, 3, nullptr, 0);
#endif
}

// load a page from the array into the chip's internal cache
bool AP_Filesystem_FlashMemory_LittleFS::nand_page_read(uint32_t row_addr)
{
    return send_command_addr(JEDEC_PAGE_DATA_READ, row_addr);
}

// clock data out of the chip's internal cache starting at a column address
bool AP_Filesystem_FlashMemory_LittleFS::nand_read_cache(uint16_t col_addr, uint8_t *buf, uint32_t len)
{
#if AP_FILESYSTEM_LITTLEFS_USE_WSPI
    const AP_HAL::Device::CommandHeader hdr {
        .cmd = NAND_CMD_READ_CACHE,
        .cfg = AP_HAL::WSPI::CFG_CMD_MODE_ONE_LINE |
               AP_HAL::WSPI::CFG_ADDR_MODE_ONE_LINE |
               AP_HAL::WSPI::CFG_ADDR_SIZE_16 |
               AP_HAL::WSPI::CFG_DATA_MODE_FOUR_LINES,
        .addr = col_addr,
        .dummy = NAND_CACHE_DUMMY,
    };
    wspi_dev->set_cmd_header(hdr);
    return wspi_dev->transfer(nullptr, 0, buf, len);
#else
    dev->set_chip_select(true);
    send_command_addr(NAND_CMD_READ_CACHE, col_addr);
    bool ok = dev->transfer(nullptr, 0, buf, len);
    dev->set_chip_select(false);
    return ok;
#endif
}

// write a page of data into the chip's internal cache at a column address
bool AP_Filesystem_FlashMemory_LittleFS::nand_program_load(uint16_t col_addr, const uint8_t *buf, uint32_t len)
{
#if AP_FILESYSTEM_LITTLEFS_USE_WSPI
    const AP_HAL::Device::CommandHeader hdr {
        .cmd = NAND_CMD_PROGRAM_LOAD,
        .cfg = AP_HAL::WSPI::CFG_CMD_MODE_ONE_LINE |
               AP_HAL::WSPI::CFG_ADDR_MODE_ONE_LINE |
               AP_HAL::WSPI::CFG_ADDR_SIZE_16 |
               AP_HAL::WSPI::CFG_DATA_MODE_FOUR_LINES,
        .addr = col_addr,
    };
    wspi_dev->set_cmd_header(hdr);
    return wspi_dev->transfer(buf, len, nullptr, 0);
#else
    dev->set_chip_select(true);
    send_command_page(NAND_CMD_PROGRAM_LOAD, col_addr);
    bool ok = dev->transfer(buf, len, nullptr, 0);
    dev->set_chip_select(false);
    return ok;
#endif
}

// commit the cache to the array at a row (page) address
bool AP_Filesystem_FlashMemory_LittleFS::nand_program_execute(uint32_t row_addr)
{
    return send_command_addr(JEDEC_PROGRAM_EXECUTE, row_addr);
}

// erase the block containing the given row (page) address
bool AP_Filesystem_FlashMemory_LittleFS::nand_block_erase(uint32_t row_addr)
{
    return send_command_addr(JEDEC_BLOCK_ERASE, row_addr);
}
#endif  // AP_FILESYSTEM_LITTLEFS_FLASH_IS_NAND

bool AP_Filesystem_FlashMemory_LittleFS::wait_until_device_is_ready()
{
    if (dead) {
        return false;
    }

    uint32_t t = AP_HAL::millis();
    while (is_busy()) {
        hal.scheduler->delay_microseconds(100);
        if (AP_HAL::millis() - t > 5000) {
            mark_dead();
            return false;
        }
    }

    return true;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
const static struct lfs_filebd_config fbd_config {
    .read_size = 2048,
    .prog_size = 2048,
    .erase_size = 131072,
    .erase_count = 256
};
#endif

uint32_t AP_Filesystem_FlashMemory_LittleFS::find_block_size_and_count() {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    if (!wait_until_device_is_ready()) {
        return false;
    }

    WITH_SEMAPHORE(dev_sem);

    // Read manufacturer ID
    uint32_t id;

#if AP_FILESYSTEM_LITTLEFS_FLASH_IS_NAND
    if (!nand_read_id(id)) {
        return 0;
    }
#else
    uint8_t buf[4];
    uint8_t cmd = JEDEC_DEVICE_ID;
    dev->transfer(&cmd, 1, buf, 4);
    id = (static_cast<uint32_t>(buf[0]) << 16) | (static_cast<uint32_t>(buf[1]) << 8) | buf[2];
#endif

    // Let's specify the terminology here.
    //
    // 1 block = smallest unit that we _erase_ in a single operation
    // 1 page = smallest unit that we read or program in a single operation
    //
    // regardless of what the flash chip documentation refers to as a "block"

#if AP_FILESYSTEM_LITTLEFS_FLASH_IS_NAND
    // most SPI NAND chips have 2048 byte pages and 128K erase blocks; parts
    // that differ (e.g. the 4Gbit MT29F) override these in the switch below
    lfs_size_t page_size = 2048;
    lfs_size_t block_size = 131072;
#else
    // typical JEDEC-ish NOR flash chips have 256 byte pages and 64K blocks.
    // many also support smaller erase sizes like 4K "sectors", but the largest
    // block size is fastest to erase in bytes per second by 3-5X so we use
    // that. be aware that worst case erase time can be seconds!
    lfs_size_t page_size = 256;
    lfs_size_t block_size = 65536;
#endif

    lfs_size_t block_count;
    switch (id) {
#if AP_FILESYSTEM_LITTLEFS_FLASH_TYPE == AP_FILESYSTEM_FLASH_WSPI_NAND
#if AP_FILESYSTEM_LITTLEFS_MT29FXX_ENABLED
    case JEDEC_ID_MICRON_MT29FXX1G:
        block_count = 1024;   /* 1Gbit = 128MB, 1024 blocks of 128KB */
        break;
    case JEDEC_ID_MICRON_MT29FXX2G:
        block_count = 2048;   /* 2Gbit = 256MB, 2048 blocks of 128KB */
        break;
    case JEDEC_ID_MICRON_MT29FXX4G:
        // MT29F4G01ABAFD uses a 4KB page and 256KB erase block (the 1G/2G
        // parts use 2KB/128KB)
        page_size = 4096;
        block_size = 256 * 1024;
        block_count = 2048;   /* 4Gbit = 512MB, 2048 blocks of 256KB */
        break;
#endif
#elif AP_FILESYSTEM_LITTLEFS_FLASH_TYPE == AP_FILESYSTEM_FLASH_W25NXX
    case JEDEC_ID_WINBOND_W25N01GV:
        block_count = 1024;   /* 128MiB */
        break;
    case JEDEC_ID_WINBOND_W25N02KV:
        block_count = 2048;   /* 256MiB */
        break;
#else
    case JEDEC_ID_WINBOND_W25Q16:
    case JEDEC_ID_MICRON_M25P16:
    case JEDEC_ID_GIGA_GD25Q16E:
        block_count = 32;     /* 2MiB */
        break;

    case JEDEC_ID_WINBOND_W25Q32:
    case JEDEC_ID_WINBOND_W25X32:
    case JEDEC_ID_MACRONIX_MX25L3206E:
        block_count = 64;     /* 4MiB */
        break;

    case JEDEC_ID_MICRON_N25Q064:
    case JEDEC_ID_WINBOND_W25Q64:
    case JEDEC_ID_FMSH_FM25Q64:
    case JEDEC_ID_MACRONIX_MX25L6406E:
    case JEDEC_ID_CYPRESS_S25FL064L:
        block_count = 128;    /* 8MiB */
        break;

    case JEDEC_ID_MICRON_N25Q128:
    case JEDEC_ID_WINBOND_W25Q128:
    case JEDEC_ID_WINBOND_W25Q128_2:
    case JEDEC_ID_CYPRESS_S25FL128L:
    case JEDEC_ID_ZBIT_ZB25VQ128:   
    case JEDEC_ID_FMSH_FM25Q128A:
        block_count = 256;    /* 16MiB */
        break;

    case JEDEC_ID_WINBOND_W25Q256:
    case JEDEC_ID_MACRONIX_MX25L25635E:
    case JEDEC_ID_FMSH_FM25Q256:
        block_count = 512;    /* 32MiB */
        use_32bit_address = true;
        break;
#endif
    default:
        block_count = 0;
        hal.scheduler->delay(2000);
        printf("Unknown SPI Flash 0x%08x\n", id);
        return 0;
    }

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
#if AP_FILESYSTEM_LITTLEFS_FLASH_IS_NAND
    // NAND flash has larger pages, adjust metadata accordingly
    fs_cfg.metadata_max = page_size * 2;
    fs_cfg.compact_thresh = fs_cfg.metadata_max * 0.88f;
#endif
#else
    // SITL config
    fs_cfg.read_size = 2048;
    fs_cfg.prog_size = 2048;
    fs_cfg.block_size = 131072;
    fs_cfg.block_count = 256;
    fs_cfg.metadata_max = 2048;

    char lfsname[L_tmpnam];
    uint32_t id = 0;
    if (std::tmpnam(lfsname)) {
        lfs_filebd_create(&fs_cfg, lfsname, &fbd_config);
        id = 0xFAFF;
    }
#endif // CONFIG_HAL_BOARD != HAL_BOARD_SITL
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

bool AP_Filesystem_FlashMemory_LittleFS::mount_filesystem() {
    if (dead) {
        return false;
    }

    if (mounted) {
        return true;
    }

    EXPECT_DELAY_MS(3000);

    fs_cfg.context = this;
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    fs_cfg.read = flashmem_read;
    fs_cfg.prog = flashmem_prog;
    fs_cfg.erase = flashmem_erase;
    fs_cfg.sync = flashmem_sync;

#if AP_FILESYSTEM_LITTLEFS_USE_WSPI
    wspi_dev = hal.wspi->get_device("dataflash");

    if (!wspi_dev) {
        mark_dead();
        return false;
    }

    dev_sem = wspi_dev->get_semaphore();
#else
    dev = hal.spi->get_device("dataflash");

    if (!dev) {
        mark_dead();
        return false;
    }

    dev_sem = dev->get_semaphore();
#endif  // AP_FILESYSTEM_LITTLEFS_USE_WSPI
#else
    fs_cfg.read = lfs_filebd_read;
    fs_cfg.prog = lfs_filebd_prog;
    fs_cfg.erase = lfs_filebd_erase;
    fs_cfg.sync = lfs_filebd_sync;
#endif

    uint32_t id = find_block_size_and_count();

    if (!id) {
        mark_dead();
        return false;
    }
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    if (!init_flash()) {
        mark_dead();
        return false;
    }
#endif
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

/*
  format sdcard
*/
bool AP_Filesystem_FlashMemory_LittleFS::format(void)
{
    WITH_SEMAPHORE(fs_sem);
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Filesystem_FlashMemory_LittleFS::format_handler, void));
    // the format is handled asynchronously, we inform user of success
    // via a text message.  format_status can be polled for progress
    format_status = FormatStatus::PENDING;
    return true;
}

/*
  format sdcard
*/
void AP_Filesystem_FlashMemory_LittleFS::format_handler(void)
{
    if (format_status != FormatStatus::PENDING) {
        return;
    }
    WITH_SEMAPHORE(fs_sem);
    format_status = FormatStatus::IN_PROGRESS;
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Formatting flash using littlefs");

    int ret = lfs_format(&fs, &fs_cfg);

    /* try mounting */
    if (ret == LFS_ERR_OK) {
        ret = lfs_mount(&fs, &fs_cfg);
    }

    // try to create the root storage folder. Ignore the error code in case
    // the filesystem is corrupted or it already exists.
    if (ret == LFS_ERR_OK && strlen(HAL_BOARD_STORAGE_DIRECTORY) > 0) {
        ret = lfs_mkdir(&fs, HAL_BOARD_STORAGE_DIRECTORY);
    }

    if (ret == LFS_ERR_OK) {
        format_status = FormatStatus::SUCCESS;
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Format flash: OK");
    } else {
        format_status = FormatStatus::FAILURE;
        mark_dead();
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Format flash: Failed (%d)", int(ret));
    }
}

// returns true if we are currently formatting the SD card:
AP_Filesystem_Backend::FormatStatus AP_Filesystem_FlashMemory_LittleFS::get_format_status(void) const
{
    // note that format_handler holds sem, so we can't take it here.
    return format_status;
}

bool AP_Filesystem_FlashMemory_LittleFS::write_enable()
{
    if (!wait_until_device_is_ready()) {
        return false;
    }

    WITH_SEMAPHORE(dev_sem);
#if AP_FILESYSTEM_LITTLEFS_USE_WSPI
    if (!wspi_command(JEDEC_WRITE_ENABLE)) {
        return false;
    }
    // verify WEL bit is set
    uint8_t status;
    if (!nand_read_status(status)) {
        return false;
    }
    return (status & NAND_STATUS_WEL) != 0;
#else
    uint8_t b = JEDEC_WRITE_ENABLE;
    return dev->transfer(&b, 1, nullptr, 0);
#endif
}

bool AP_Filesystem_FlashMemory_LittleFS::init_flash()
{
    if (!wait_until_device_is_ready()) {
        return false;
    }

#if AP_FILESYSTEM_LITTLEFS_FLASH_IS_NAND
    // reset the device
    {
        WITH_SEMAPHORE(dev_sem);
        if (!nand_reset()) {
            return false;
        }
    }
    hal.scheduler->delay(NAND_RESET_DELAY_MS);

    if (!wait_until_device_is_ready()) {
        return false;
    }

    {
        WITH_SEMAPHORE(dev_sem);
        // disable write protection, then enable ECC (W25N also needs buffer read mode)
        if (!nand_set_reg(NAND_REG_PROTECTION, 0x00)) {
            return false;
        }
        if (!nand_set_reg(NAND_REG_CONFIG, NAND_CONFIG_BITS)) {
            return false;
        }
    }
#else
    if (use_32bit_address) {
        WITH_SEMAPHORE(dev_sem);
        // enter 4-byte address mode
        const uint8_t cmd = 0xB7;
        dev->transfer(&cmd, 1, nullptr, 0);
    }
#endif

    return true;
}

#ifdef AP_LFS_DEBUG
static uint32_t page_writes;
static uint32_t last_write_msg_ms;
static uint32_t page_reads;
static uint32_t block_erases;
#endif
int AP_Filesystem_FlashMemory_LittleFS::_flashmem_read(
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
#if AP_FILESYSTEM_LITTLEFS_FLASH_IS_NAND
        // NAND: load the page into the chip's cache, then clock the cache out.
        // row address = page number within the device, column 0.
        const uint32_t row_addr = address / page_size;

        if (!wait_until_device_is_ready()) {
            return LFS_ERR_IO;
        }
        {
            WITH_SEMAPHORE(dev_sem);
            if (!nand_page_read(row_addr)) {
                return LFS_ERR_IO;
            }
        }

        if (!wait_until_device_is_ready()) {
            return LFS_ERR_IO;
        }
        {
            WITH_SEMAPHORE(dev_sem);
            if (!nand_read_cache(0, p, page_size)) {
                return LFS_ERR_IO;
            }
        }
#else
        // NOR: direct read, no page cache
        if (!wait_until_device_is_ready()) {
            return LFS_ERR_IO;
        }

        WITH_SEMAPHORE(dev_sem);

        dev->set_chip_select(true);
        send_command_addr(JEDEC_READ_DATA, address);
        dev->transfer(nullptr, 0, p, page_size);
        dev->set_chip_select(false);
#endif

        address += page_size;
        p += page_size;
    }
    return LFS_ERR_OK;
}


int AP_Filesystem_FlashMemory_LittleFS::_flashmem_prog(
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
#if AP_FILESYSTEM_LITTLEFS_FLASH_IS_NAND
        // NAND: PROGRAM LOAD writes data into the cache at column 0, PROGRAM
        // EXECUTE commits it to the page at the row address.
        const uint32_t row_addr = address / page_size;

        if (!nand_program_load(0, p, page_size)) {
            return LFS_ERR_IO;
        }
        if (!nand_program_execute(row_addr)) {
            return LFS_ERR_IO;
        }
#else
        dev->set_chip_select(true);
        send_command_addr(JEDEC_PAGE_WRITE, address);
        dev->transfer(p, page_size, nullptr, 0);
        dev->set_chip_select(false);
#endif
        // writing simply means the data is in the internal cache, it will take
        // some period to propagate to the flash itself

        address += page_size;
        p += page_size;
    }
    return LFS_ERR_OK;
}

int AP_Filesystem_FlashMemory_LittleFS::_flashmem_erase(lfs_block_t block) {
    if (!write_enable()) {
        return LFS_ERR_IO;
    }

#ifdef AP_LFS_DEBUG
    block_erases++;
#endif

    WITH_SEMAPHORE(dev_sem);

#if AP_FILESYSTEM_LITTLEFS_FLASH_IS_NAND
    // NAND: BLOCK ERASE uses the row address of the block's first page
    const uint32_t pages_per_block = fs_cfg.block_size / fs_cfg.read_size;
    if (!nand_block_erase(block * pages_per_block)) {
        return LFS_ERR_IO;
    }
#else
    send_command_addr(JEDEC_BLOCK_ERASE, block * fs_cfg.block_size);
#endif

    // sleep so that other processes get the CPU cycles that the 4ms erase cycle needs.
    hal.scheduler->delay(4);

    return LFS_ERR_OK;
}

int AP_Filesystem_FlashMemory_LittleFS::_flashmem_sync() {
    if (wait_until_device_is_ready()) {
        return LFS_ERR_OK;
    } else {
        return LFS_ERR_IO;
    }
}

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
static int flashmem_read(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    void* buffer, lfs_size_t size
) {
    AP_Filesystem_FlashMemory_LittleFS* self = static_cast<AP_Filesystem_FlashMemory_LittleFS*>(cfg->context);
    return self->_flashmem_read(block, off, buffer, size);
}

static int flashmem_prog(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    const void* buffer, lfs_size_t size
) {
    AP_Filesystem_FlashMemory_LittleFS* self = static_cast<AP_Filesystem_FlashMemory_LittleFS*>(cfg->context);
    return self->_flashmem_prog(block, off, buffer, size);
}

static int flashmem_erase(const struct lfs_config *cfg, lfs_block_t block) {
    AP_Filesystem_FlashMemory_LittleFS* self = static_cast<AP_Filesystem_FlashMemory_LittleFS*>(cfg->context);
    return self->_flashmem_erase(block);
}

static int flashmem_sync(const struct lfs_config *cfg) {
    AP_Filesystem_FlashMemory_LittleFS* self = static_cast<AP_Filesystem_FlashMemory_LittleFS*>(cfg->context);
    return self->_flashmem_sync();
}
#endif

/* ************************************************************************* */
/* LittleFS to POSIX API conversion functions                                */
/* ************************************************************************* */

static int errno_from_lfs_error(int lfs_error)
{
    switch (lfs_error) {
        case LFS_ERR_OK: return 0;
        case LFS_ERR_IO: return EIO;
        case LFS_ERR_CORRUPT: return EIO;
        case LFS_ERR_NOENT: return ENOENT;
        case LFS_ERR_EXIST: return EEXIST;
        case LFS_ERR_NOTDIR: return ENOTDIR;
        case LFS_ERR_ISDIR: return EISDIR;
        case LFS_ERR_NOTEMPTY: return ENOTEMPTY;
        case LFS_ERR_BADF: return EBADF;
        case LFS_ERR_FBIG: return EFBIG;
        case LFS_ERR_INVAL: return EINVAL;
        case LFS_ERR_NOSPC: return ENOSPC;
        case LFS_ERR_NOMEM: return ENOMEM;
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        case LFS_ERR_NOATTR: return ENOATTR;
#endif
        case LFS_ERR_NAMETOOLONG: return ENAMETOOLONG;
        default: return EIO;
    }
}

static int lfs_flags_from_flags(int flags)
{
    int outflags = 0;

    if (flags & O_WRONLY) {
        outflags |= LFS_O_WRONLY | LFS_F_WRUNCHECKED;
    } else if (flags & O_RDWR) {
        outflags |= LFS_O_RDWR;
    } else {
        outflags |= LFS_O_RDONLY;
    }

    if (flags & O_CREAT) {
        outflags |= LFS_O_CREAT;
    }

    if (flags & O_EXCL) {
        outflags |= LFS_O_EXCL;
    }

    if (flags & O_TRUNC) {
        outflags |= LFS_O_TRUNC;
    }

    if (flags & O_APPEND) {
        outflags |= LFS_O_APPEND;
    }

    return outflags;
}

#endif  // AP_FILESYSTEM_LITTLEFS_ENABLED
