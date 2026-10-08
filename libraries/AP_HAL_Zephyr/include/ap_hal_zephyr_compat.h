/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by @davidbuzz and Claude
 */
/*
 * Cross-board compatibility shims for AP_HAL_Zephyr.
 * Force-included from boards.py before per-board hwdef.h, so board
 * headers can override with #ifndef guards.
 */
#pragma once

// pull toolchain <stdio.h> before Zephyr's posix <unistd.h> shadows SEEK_*
#include <stdio.h>

#include <strings.h>
// Zephyr's posix sys/stat.h pulls in kernel.h which uses CONTAINER_OF —
// suppress the resulting -Wcast-align warning from that macro in util.h.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#include <sys/stat.h>
#pragma GCC diagnostic pop
#include <time.h>

// picolibc's <time.h> doesn't always expose CLOCK_* constants without POSIX
// feature-test macros.  Define them directly with Zephyr's values.
#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME  1
#endif
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 4
#endif

// picolibc on Xtensa already provides memmem() in string.h; no shim needed.

// zephyr/sys/util.h defines ARRAY_SIZE; AP_Common redefines it with a slightly
// different form.  Drop the Zephyr definition so AP_Common.h wins cleanly.
#undef ARRAY_SIZE

// picolibc's byteswap.h provides bswap_16/32/64 but not the __bswap_* aliases
// that AP_HAL/utility/sparse-endian.h uses.  Provide them here.
#include <byteswap.h>
#ifndef __bswap_16
#define __bswap_16(x) bswap_16(x)
#define __bswap_32(x) bswap_32(x)
#define __bswap_64(x) bswap_64(x)
#endif

// Block Zephyr's posix/sys/dirent.h from defining struct dirent without d_type.
// We provide the definitive struct dirent here (with d_type for ArduPilot), and
// AP_Filesystem.h is guarded by the same macro so it won't redefine it.
#ifndef ZEPHYR_INCLUDE_POSIX_SYS_DIRENT_H_
#define ZEPHYR_INCLUDE_POSIX_SYS_DIRENT_H_
#ifndef DT_REG
#define DT_REG  0
#define DT_DIR  1
#define DT_LNK  10
#endif
/* NO `typedef void DIR;` HERE - deliberately. Zephyr's own headers define DIR,
 * and a second definition breaks every translation unit that sees both. */
struct dirent {
    unsigned int  d_ino;
    char          d_name[256];
    unsigned char d_type;
};
#endif /* ZEPHYR_INCLUDE_POSIX_SYS_DIRENT_H_ */

// renamed entry point so Zephyr's weak main() loses to app/main.cpp's strong one
#ifndef AP_MAIN
#define AP_MAIN ardupilot_entry
#endif
