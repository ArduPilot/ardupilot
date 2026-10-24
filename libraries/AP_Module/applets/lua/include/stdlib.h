// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stddef.h>
#define RAND_MAX 2147483647
#ifdef __cplusplus
extern "C" {
#endif
int abs(int);
int rand(void);
void srand(unsigned);
float strtof(const char *, char **);
void *malloc(size_t);
void *realloc(void *, size_t);
void free(void *);
void abort(void) __attribute__((noreturn));
#ifdef __cplusplus
}
#endif
