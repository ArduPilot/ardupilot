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
  stdio-like functions for buffered file writing in AP_Filesystem
 */

#include "AP_Filesystem.h"

#if AP_FILESYSTEM_FILE_WRITING_ENABLED

#include <stdarg.h>
#include <AP_Math/AP_Math.h>

#define CHECK_STREAM(stream, ret) while (stream == NULL || stream->fd < 0) { errno = EBADF; return ret; }

#define modecmp(str, pat) (strcmp(str, pat) == 0 ? 1: 0)

// 2k makes for efficient usage of a microSD
#define AP_FILESYSTEM_STDIO_WRITEBUF_SIZE 2048

/*
  do an fsync every 8k to keep directory information up to date
  we don't rely on the user calling flush() as that would lead to
  misaligned writes
*/
#define AP_FILESYSTEM_STDIO_WRITEBUF_FSYNC_SIZE (AP_FILESYSTEM_STDIO_WRITEBUF_SIZE*4)

#define EOF (-1)

/*
  map a fopen() file mode to a open() mode
 */
static int posix_fopen_modes_to_open(const char *mode)
{
    int flag = 0;

    if (modecmp(mode,"r") || modecmp(mode,"rb")) {
        flag = O_RDONLY;
        return flag;
    }
    if (modecmp(mode,"r+") || modecmp(mode, "r+b" ) || modecmp(mode, "rb+" )) {
        flag = O_RDWR | O_TRUNC;
        return flag;
    }
    if (modecmp(mode,"w") || modecmp(mode,"wb")) {
        flag = O_WRONLY | O_CREAT | O_TRUNC;
        return flag;
    }
    if (modecmp(mode,"w+") || modecmp(mode, "w+b" ) || modecmp(mode, "wb+" )) {
        flag = O_RDWR | O_CREAT | O_TRUNC;
        return flag;
    }
    if (modecmp(mode,"a") || modecmp(mode,"ab")) {
        flag = O_WRONLY | O_CREAT | O_APPEND;
        return flag;
    }
    if (modecmp(mode,"a+") || modecmp(mode, "a+b" ) || modecmp(mode, "ab+" )) {
        flag = O_RDWR | O_CREAT | O_APPEND;
        return flag;
    }
    return -1;
}

APFS_FILE *AP_Filesystem::fopen(const char *pathname, const char *mode)
{
    APFS_FILE *f = NEW_NOTHROW APFS_FILE;
    if (!f) {
        return nullptr;
    }
    const auto open_flags = posix_fopen_modes_to_open(mode);
    f->fd = open(pathname, open_flags);
    if (f->fd == -1) {
        delete f;
        return nullptr;
    }
    f->unget = -1;
    // if being opened for write only then allocate a buffer
    if (open_flags == (O_WRONLY | O_CREAT | O_TRUNC)) {
        // don't throw an error if we can't allocate
        f->writebuf = (uint8_t *)malloc(AP_FILESYSTEM_STDIO_WRITEBUF_SIZE);
    }
    return f;
}

int AP_Filesystem::fflush(APFS_FILE *f)
{
    CHECK_STREAM(f, EOF);
    WITH_SEMAPHORE(f->sem);
    const auto used = f->writebuf_used;
    f->writebuf_used = 0;
    if (used > 0) {
        if (write(f->fd, f->writebuf, used) != (int32_t)used) {
            // out of space
            errno = ENOSPC;
            fsync(f->fd);
            return EOF;
        }
    }
    if (fsync(f->fd) == 0) {
        return 0;
    }
    return EOF;
}

size_t AP_Filesystem::fwrite(const void *ptr, size_t size, size_t nmemb, APFS_FILE *f)
{
    CHECK_STREAM(f, 0);
    WITH_SEMAPHORE(f->sem);
    uint32_t total = size*nmemb;
    const uint8_t *b = (const uint8_t *)ptr;
    if (f->writebuf != nullptr && !f->error) {
        /*
          write via the buffer, keeping writes aligned to multiple of buffer size
         */
        uint32_t nwritten = 0;
        while (total > 0) {
            uint32_t n = MIN(total, AP_FILESYSTEM_STDIO_WRITEBUF_SIZE - f->writebuf_used);
            memcpy(&f->writebuf[f->writebuf_used], b, n);
            f->writebuf_used += n;
            total -= n;
            b += n;
            nwritten += n;
            if (f->writebuf_used == AP_FILESYSTEM_STDIO_WRITEBUF_SIZE) {
                const auto n2 = write(f->fd, f->writebuf, AP_FILESYSTEM_STDIO_WRITEBUF_SIZE);
                f->writebuf_used = 0;
                if (n2 != AP_FILESYSTEM_STDIO_WRITEBUF_SIZE) {
                    f->error = true;
                    break;
                }
                f->total_written += n2;
                if (f->total_written % AP_FILESYSTEM_STDIO_WRITEBUF_FSYNC_SIZE == 0) {
                    fflush(f);
                }
            }
        }
        return nwritten;
    }
    // non-buffered write
    ssize_t ret = write(f->fd, ptr, total);
    if (ret <= 0) {
        f->error = true;
        return 0;
    }
    return ret / size;
}

int AP_Filesystem::fclose(APFS_FILE *f)
{
    int ret = -1;
    CHECK_STREAM(f, EOF);
    {
        WITH_SEMAPHORE(f->sem);
        if (f->writebuf != nullptr) {
            if (f->writebuf_used != 0) {
                fflush(f);
            }
            auto *wb = f->writebuf;
            f->writebuf = nullptr;
            free(wb);
        }
        ret = close(f->fd);
        f->fd = -1;
        if (f->tmpfile_name) {
            unlink(f->tmpfile_name);
            free(f->tmpfile_name);
            f->tmpfile_name = NULL;
        }
    }
    delete f;
    return ret;
}

#endif // AP_FILESYSTEM_FILE_WRITING_ENABLED

