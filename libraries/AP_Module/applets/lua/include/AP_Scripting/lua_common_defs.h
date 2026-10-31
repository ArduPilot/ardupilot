// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void lua_abort(void) __attribute__((noreturn));
#ifdef __cplusplus
}
#endif
#define lua_getlocaledecpoint() '.'
