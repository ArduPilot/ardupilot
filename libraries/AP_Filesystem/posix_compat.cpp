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

#if HAVE_FILESYSTEM_SUPPORT

#include "posix_compat.h"
#include <stdarg.h>
#include <AP_Math/AP_Math.h>

struct apfs_file {
    int fd;
    bool error;
    bool eof;
    int16_t unget;
    char *tmpfile_name;
};

#define CHECK_STREAM(stream, ret) while (stream == NULL || stream->fd < 0) { errno = EBADF; return ret; }

#define modecmp(str, pat) (strcmp(str, pat) == 0 ? 1: 0)

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
        return -1;
    }
    return -1;
}

APFS_FILE *apfs_fopen(const char *pathname, const char *mode)
{
    APFS_FILE *f = new APFS_FILE;
    if (!f) {
        return nullptr;
    }
    f->fd = AP::FS().open(pathname, posix_fopen_modes_to_open(mode));
    f->unget = -1;
    return f;
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
        len = AP::FS().write(stream->fd, buf, len);
        free(buf);
    }

    return len;
}

int apfs_fflush(APFS_FILE *stream)
{
    CHECK_STREAM(stream, EOF);
    return 0;
}

size_t apfs_fread(void *ptr, size_t size, size_t nmemb, APFS_FILE *stream)
{
    CHECK_STREAM(stream, 0);
    ssize_t ret = AP::FS().read(stream->fd, ptr, size*nmemb);
    if (ret <= 0) {
        stream->eof = true;
        return 0;
    }
    return ret / size;
}

size_t apfs_fwrite(const void *ptr, size_t size, size_t nmemb, APFS_FILE *stream)
{
    CHECK_STREAM(stream, 0);
    ssize_t ret = AP::FS().write(stream->fd, ptr, size*nmemb);
    if (ret <= 0) {
        stream->error = true;
        return 0;
    }
    return ret / size;
}

int apfs_fputs(const char *s, APFS_FILE *stream)
{
    CHECK_STREAM(stream, EOF);
    ssize_t ret = AP::FS().write(stream->fd, s, strlen(s));
    if (ret < 0) {
        stream->error = true;
        return EOF;
    }
    return ret;
}

char *apfs_fgets(char *s, int size, APFS_FILE *stream)
{
    CHECK_STREAM(stream, NULL);
    ssize_t ret = AP::FS().read(stream->fd, s, size-1);
    if (ret < 0) {
        stream->error = true;
        return NULL;
    }
    s[ret] = 0;
    return s;
}

void apfs_clearerr(APFS_FILE *stream)
{
    stream->error = false;
}

int apfs_fseek(APFS_FILE *stream, long offset, int whence)
{
    CHECK_STREAM(stream, EOF);
    stream->eof = false;
    return AP::FS().lseek(stream->fd, offset, whence);
}

int apfs_ferror(APFS_FILE *stream)
{
    CHECK_STREAM(stream, EOF);
    return stream->error;
}

int apfs_fclose(APFS_FILE *stream)
{
    CHECK_STREAM(stream, EOF);
    int ret = AP::FS().close(stream->fd);
    stream->fd = -1;
    if (stream->tmpfile_name) {
        AP::FS().unlink(stream->tmpfile_name);
        free(stream->tmpfile_name);
        stream->tmpfile_name = NULL;
    }
    delete stream;
    return ret;
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
    stream->unget = c;
    stream->eof = false;
    return c;
}

int apfs_feof(APFS_FILE *stream)
{
    return stream->eof;
}

APFS_FILE *apfs_freopen(const char *pathname, const char *mode, APFS_FILE *stream)
{
    CHECK_STREAM(stream, NULL);
    int ret = AP::FS().close(stream->fd);
    if (ret < 0) {
        return NULL;
    }
    if (stream->tmpfile_name) {
        AP::FS().unlink(stream->tmpfile_name);
        free(stream->tmpfile_name);
        stream->tmpfile_name = NULL;
    }
    stream->fd = AP::FS().open(pathname, posix_fopen_modes_to_open(mode));
    stream->error = false;
    stream->eof = false;
    stream->unget = -1;
    return stream;
}

long apfs_ftell(APFS_FILE *stream)
{
    CHECK_STREAM(stream, EOF);
    return AP::FS().lseek(stream->fd, 0, SEEK_CUR);
}

int apfs_remove(const char *pathname)
{
    return AP::FS().unlink(pathname);
}

#endif // HAVE_FILESYSTEM_SUPPORT
