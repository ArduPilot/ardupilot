// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
#ifdef __arm__
    int64_t registers[13];
#else
    uint64_t registers[8];
#endif
} ap_jmp_buf[1];
int ap_setjmp(ap_jmp_buf env) __attribute__((returns_twice));
void ap_longjmp(ap_jmp_buf env, int value) __attribute__((noreturn));
#ifdef __cplusplus
}
#endif
