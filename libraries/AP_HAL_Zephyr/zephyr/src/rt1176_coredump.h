/*
 * Layout shared between the RT1176 coredump backend (rt1176_coredump_backend.c,
 * C, runs inside z_fatal_error()) and Zephyr::Util::last_crash_dump_size()/
 * last_crash_dump_ptr() (Util.cpp, C++, runs during ordinary boot). Kept in one
 * header so the two sides of the "who wrote it" / "who reads it" boundary can't
 * drift apart.
 *
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
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* coredump_partition@3f00000, 1 MiB reserved in the board DTS. */
#define RT1176_COREDUMP_PARTITION_OFFSET  0x03F00000U

/* One erase unit - see rt1176_romapi_flash.c's range-erase primitive. */
#define RT1176_COREDUMP_ERASE_SIZE        4096U

/* Distinct from both erased NOR (0xFFFFFFFF) and zeroed/never-written RAM
 * (0x00000000), so "is there a real dump here" is a single word compare. */
#define RT1176_COREDUMP_MAGIC             0x44524358U   /* ASCII "XCRD" */

/* This backend's own tiny header, written last (once total size is known)
 * into the same erased sector the payload lives in. Distinct from Zephyr's
 * own coredump_hdr_t (debug/coredump.h), which is part of the *payload*
 * this header points at - two headers, two jobs: this one says "is there a
 * dump, and how big", Zephyr's own says "how to parse it". */
struct rt1176_coredump_hdr {
	uint32_t magic;
	uint32_t size;   /* payload bytes following this header, not including it */
};

#define RT1176_COREDUMP_HDR_SIZE    ((uint32_t)sizeof(struct rt1176_coredump_hdr))
#define RT1176_COREDUMP_PAYLOAD_MAX (RT1176_COREDUMP_ERASE_SIZE - RT1176_COREDUMP_HDR_SIZE)

#ifdef __cplusplus
}
#endif
