// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
typedef int64_t time_t;
#ifdef __cplusplus
extern "C" {
#endif
time_t time(time_t *);
#ifdef __cplusplus
}
#endif
