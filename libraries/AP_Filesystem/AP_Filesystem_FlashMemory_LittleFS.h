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
#pragma once

#include "AP_Filesystem_backend.h"

#if AP_FILESYSTEM_LITTLEFS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include "lfs.h"

class AP_Filesystem_FlashMemory_LittleFS : public AP_Filesystem_Backend
{
public:
    // functions that closely match the equivalent posix calls
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    int close(int fd) override;
    int32_t read(int fd, void *buf, uint32_t count) override;
    int32_t write(int fd, const void *buf, uint32_t count) override;
    int fsync(int fd) override;
    int32_t lseek(int fd, int32_t offset, int whence) override;
    int stat(const char *pathname, struct stat *stbuf) override;

    int unlink(const char *pathname) override;
    int mkdir(const char *pathname) override;

    void *opendir(const char *pathname) override;
    struct dirent *readdir(void *dirp) override;
    int closedir(void *dirp) override;

    uint32_t bytes_until_fsync(int fd) override;

    int64_t disk_free(const char *path) override;
    int64_t disk_space(const char *path) override;

    // set modification time on a file
    bool set_mtime(const char *filename, const uint32_t mtime_sec) override;

    bool retry_mount(void) override;
    void unmount(void) override;
    // format flash.  This is async, monitor get_format_status for progress
    bool format(void) override;
    AP_Filesystem_Backend::FormatStatus get_format_status() const override;

    int _flashmem_read(lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size);
    int _flashmem_prog(lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size);
    int _flashmem_erase(lfs_block_t block);
    int _flashmem_sync();

private:
    // Semaphore to protect against concurrent accesses to fs
    HAL_Semaphore fs_sem;

    // The filesystem object
    lfs_t fs;

    // The configuration of the filesystem
    struct lfs_config fs_cfg;

    // Maximum number of files that may be open at the same time
    static constexpr int MAX_OPEN_FILES = 16;

    // Stores whether the filesystem is mounted
    bool mounted;

    // Stores whether the filesystem has been marked as dead
    bool dead;

    // Array of currently open file descriptors
    struct FileDescriptor {
        lfs_file_t file;
        lfs_file_config cfg;
        lfs_attr attrs[1];
        uint32_t mtime;
        char* filename;
    };

    FileDescriptor* open_files[MAX_OPEN_FILES];

    // SPI device that handles the raw flash memory
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;

    // Semaphore to protect access to the SPI device
    AP_HAL::Semaphore *dev_sem;

    // Flag to denote that the underlying flash chip uses 32-bit addresses
    bool use_32bit_address;
    FormatStatus format_status;

    int allocate_fd();
    int free_fd(int fd);
    void free_all_fds();
    FileDescriptor* lfs_file_from_fd(int fd) const;

    uint32_t find_block_size_and_count();
    bool init_flash() WARN_IF_UNUSED;
    bool write_enable() WARN_IF_UNUSED;
    bool is_busy();
    bool mount_filesystem();
    void send_command_addr(uint8_t command, uint32_t addr);
    void send_command_page(uint8_t command, uint32_t page);
    bool wait_until_device_is_ready() WARN_IF_UNUSED;
    void write_status_register(uint8_t reg, uint8_t bits);
    void format_handler(void);
    void mark_dead();
};

#endif  // #if AP_FILESYSTEM_LITTLEFS_ENABLED
