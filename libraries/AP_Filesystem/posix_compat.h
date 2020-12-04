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
 */
#pragma once

#include <sys/types.h>
#include <stdio.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
  these are here to allow lua to build on HAL_ChibiOS
 */

typedef struct apfs_file APFS_FILE;

APFS_FILE *apfs_fopen(const char *pathname, const char *mode);
int apfs_fprintf(APFS_FILE *stream, const char *format, ...);
int apfs_fflush(APFS_FILE *stream);
size_t apfs_fread(void *ptr, size_t size, size_t nmemb, APFS_FILE *stream);
size_t apfs_fwrite(const void *ptr, size_t size, size_t nmemb, APFS_FILE *stream);
int apfs_fputs(const char *s, APFS_FILE *stream);
char *apfs_fgets(char *s, int size, APFS_FILE *stream);
void apfs_clearerr(APFS_FILE *stream);
int apfs_fseek(APFS_FILE *stream, long offset, int whence);
int apfs_ferror(APFS_FILE *stream);
int apfs_fclose(APFS_FILE *stream);
APFS_FILE *apfs_tmpfile(void);
int apfs_getc(APFS_FILE *stream);
int apfs_ungetc(int c, APFS_FILE *stream);
int apfs_feof(APFS_FILE *stream);
long apfs_ftell(APFS_FILE *stream);
APFS_FILE *apfs_freopen(const char *pathname, const char *mode, APFS_FILE *stream);
int apfs_remove(const char *pathname);
int apfs_rename(const char *oldpath, const char *newpath);
char *tmpnam(char *s);

#undef stdin
#undef stdout
#undef stderr
#define stdin ((APFS_FILE*)1)
#define stdout ((APFS_FILE*)2)
#define stderr ((APFS_FILE*)3)

#undef BUFSIZ
#define BUFSIZ 256
#define EOF (-1)

#ifndef SEEK_SET
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2
#endif

#define FILE APFS_FILE
#define fopen(p,m) apfs_fopen(p,m)
#define fprintf(stream, format, args...) apfs_fprintf(stream, format, ##args)
#define fflush(s) apfs_fflush(s)
#define fread(ptr,size,nmemb, stream) apfs_fread(ptr, size, nmemb, stream)
#define fwrite(ptr, size, nmemb, stream) apfs_fwrite(ptr, size, nmemb, stream)
#define fputs(s, stream) apfs_fputs(s, stream)
#define fgets(s, size, stream) apfs_fgets(s, size, stream)
#define clearerr(stream) apfs_clearerr(stream)
#define fseek(stream, offset, whence) apfs_fseek(stream, offset, whence)
#define ferror(stream) apfs_ferror(stream)
#define fclose(stream) apfs_fclose(stream)
#define tmpfile() apfs_tmpfile()
#undef getc
#define getc(stream) apfs_getc(stream)
#define ungetc(c, stream) apfs_ungetc(c, stream)
#define feof(stream) apfs_ferror(stream)
#define ftell(stream) apfs_ftell(stream)
#define freopen(pathname, mode, stream) apfs_freopen(pathname, mode, stream)
#define remove(pathname) apfs_remove(pathname)
#define rename(oldpath, newpath) apfs_rename(oldpath, newpath)
#if !defined(__APPLE__)
int sprintf(char *str, const char *format, ...);
#endif

#ifdef __cplusplus
}
#endif
