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
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifdef __ZEPHYR__
// The enable flag lives in hwdef_zephyr.h (ChibiOS boards get it from theirs).
#include "hwdef_zephyr.h"
#endif

#ifndef AP_BOOTLOADER_MCUBOOT_AB
#define AP_BOOTLOADER_MCUBOOT_AB 0
#endif

#if AP_BOOTLOADER_MCUBOOT_AB

#include <stdint.h>
#include <stdbool.h>

/* MCUBoot-compatible A/B, overwrite-only: slot 1 is checked and copied over slot
 * 0, so the image format matches what standard MCUBoot tooling produces. */

// MCUBoot image header (image/image.h), little-endian on this SoC.
#define MCUBOOT_IMAGE_MAGIC 0x96f3b83dU

struct mcuboot_image_header {
    uint32_t magic;
    uint32_t load_addr;
    uint16_t hdr_size;
    uint16_t protect_tlv_size;
    uint32_t img_size;
    uint32_t flags;
    struct { uint8_t major, minor; uint16_t revision; uint32_t build_num; } version;
    uint32_t pad;
};

// TLV info + entry (image/image.h)
#define MCUBOOT_TLV_INFO_MAGIC     0x6907U
#define MCUBOOT_TLV_PROT_INFO_MAGIC 0x6908U
#define MCUBOOT_TLV_SHA256   0x10U
#define MCUBOOT_TLV_KEYHASH  0x01U
#define MCUBOOT_TLV_ED25519  0x24U

/* Run the overwrite-only A/B step, called from main() BEFORE the normal protocol. */
bool mcuboot_ab_update(void);

#endif  // AP_BOOTLOADER_MCUBOOT_AB
