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
  compatibility with posix APIs using AP_Filesystem

  This implements the FILE* API from posix sufficiently well for Lua
  scripting to function. It has no buffering so is inefficient for
  single character operations. We deliberately use this implementation
  in HAL_SITL and HAL_Linux where it is not needed in order to have a
  uniform implementation across all platforms
 */

#include "AP_Filesystem.h"

#if AP_FILESYSTEM_FATFS_ENABLED || AP_FILESYSTEM_POSIX_ENABLED || AP_FILESYSTEM_ESP32_ENABLED || AP_FILESYSTEM_ROMFS_ENABLED

#include "posix_compat.h"
#include <stdarg.h>
#include <AP_Math/AP_Math.h>

#define CHECK_STREAM(stream, ret) while (stream == NULL || stream->fd < 0) { errno = EBADF; return ret; }

APFS_FILE *apfs_fopen(const char *pathname, const char *mode)
{
    return AP::FS().fopen(pathname, mode);
}

int apfs_fprintf(APFS_FILE *stream, const char *fmt, ...)
{
    CHECK_STREAM(stream, -1);
    va_list va;
    char* buf = NULL;
    int16_t len;
    va_start(va, fmt);
    len = vasprintf(&buf, fmt, va);
    va_end(va);
    if (len > 0) {
        len = apfs_fwrite(buf, 1, len, stream);
        free(buf);
    }

    return len;
}

int apfs_fflush(APFS_FILE *stream)
{
    return AP::FS().fflush(stream);
}

size_t apfs_fread(void *ptr, size_t size, size_t nmemb, APFS_FILE *stream)
{
    CHECK_STREAM(stream, 0);
    if (stream->writebuf_used != 0) {
        apfs_fflush(stream);
    }
    ssize_t ret = AP::FS().read(stream->fd, ptr, size*nmemb);
    if (ret <= 0) {
        stream->eof = true;
        return 0;
    }
    return ret / size;
}

size_t apfs_fwrite(const void *ptr, size_t size, size_t nmemb, APFS_FILE *stream)
{
    return AP::FS().fwrite(ptr, size, nmemb, stream);
}

int apfs_fputs(const char *s, APFS_FILE *stream)
{
    CHECK_STREAM(stream, EOF);
    ssize_t ret = apfs_fwrite(s, 1, strlen(s), stream);
    if (ret < 0) {
        stream->error = true;
        return EOF;
    }
    return ret;
}

#undef fgets
char *apfs_fgets(char *s, int size, APFS_FILE *stream)
{
    CHECK_STREAM(stream, NULL);
    if (stream->writebuf_used != 0) {
        apfs_fflush(stream);
    }
    auto &fs = AP::FS();
    if (!fs.fgets(s, size, stream->fd)) {
        return NULL;
    }
    return s;
}

void apfs_clearerr(APFS_FILE *stream)
{
    stream->error = false;
}

int apfs_fseek(APFS_FILE *stream, long offset, int whence)
{
    CHECK_STREAM(stream, EOF);
    if (stream->writebuf_used != 0) {
        apfs_fflush(stream);
    }
    stream->eof = false;
    AP::FS().lseek(stream->fd, offset, whence);
    return 0;
}

int apfs_ferror(APFS_FILE *stream)
{
    CHECK_STREAM(stream, EOF);
    return stream->error;
}

int apfs_fclose(APFS_FILE *stream)
{
    return AP::FS().fclose(stream);
}

APFS_FILE *apfs_tmpfile(void)
{
    char *fname = NULL;
    if (asprintf(&fname, "tmp.%03u", unsigned(get_random16()) % 1000) <= 0) {
        return NULL;
    }
    APFS_FILE *ret = apfs_fopen(fname, "w");
    if (!ret) {
        free(fname);
        return NULL;
    }
    ret->tmpfile_name = fname;
    return ret;
}

int apfs_getc(APFS_FILE *stream)
{
    CHECK_STREAM(stream, EOF);
    if (stream->writebuf_used != 0) {
        apfs_fflush(stream);
    }
    uint8_t c;
    if (stream->unget != -1) {
        c = stream->unget;
        stream->unget = -1;
        return c;
    }
    ssize_t ret = AP::FS().read(stream->fd, &c, 1);
    if (ret <= 0) {
        stream->eof = true;
        return EOF;
    }
    return c;
}

int apfs_ungetc(int c, APFS_FILE *stream)
{
    CHECK_STREAM(stream, EOF);
    if (stream->writebuf_used != 0) {
        apfs_fflush(stream);
    }
    stream->unget = c;
    stream->eof = false;
    return c;
}

int apfs_feof(APFS_FILE *stream)
{
    return stream->eof;
}

long apfs_ftell(APFS_FILE *stream)
{
    CHECK_STREAM(stream, EOF);
    if (stream->writebuf_used != 0) {
        apfs_fflush(stream);
    }
    return AP::FS().lseek(stream->fd, 0, SEEK_CUR);
}

int apfs_remove(const char *pathname)
{
    return AP::FS().unlink(pathname);
}

#endif // AP_FILESYSTEM_POSIX_ENABLED
