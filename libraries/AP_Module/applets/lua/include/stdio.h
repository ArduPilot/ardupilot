// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stddef.h>
#include <stdarg.h>
#define EOF (-1)
#define BUFSIZ 512
typedef struct lua_module_file FILE;
#ifdef __cplusplus
extern "C" {
#endif
extern FILE *stdin;
extern FILE *stdout;
int fflush(FILE *);
int printf(const char *, ...);
int snprintf(char *, size_t, const char *, ...);
FILE *fopen(const char *, const char *);
FILE *freopen(const char *, const char *, FILE *);
int fclose(FILE *);
int ferror(FILE *);
int feof(FILE *);
int getc(FILE *);
size_t fread(void *, size_t, size_t, FILE *);
#ifdef __cplusplus
}
#endif
