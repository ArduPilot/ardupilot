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
  implement a file store for embedded firmware images
 */

#include "AP_ROMFS.h"
#include "tinf.h"
#include <AP_Math/crc.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_ROMFS_config.h"

#include <string.h>

#ifdef HAL_HAVE_AP_ROMFS_PRELINKED_APMOD
/*
  Restore the canonical target offsets in a firmware-prelinked APMOD07
  module. This lets ordinary ROMFS reads return the original file while
  find_uncompressed() exposes the physical execute-in-place image.
 */
static bool canonicalise_apmod(const uint8_t *source, uint8_t *copy, uint32_t size)
{
    constexpr uint32_t header_size = 88;
    constexpr uint32_t pointer_width = 4;
    const auto word = [](const uint8_t *p) {
        return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
    };
    if (size < header_size || memcmp(source, "APMOD07\0", 8) != 0 ||
        word(source + 8) != 7 || word(source + 12) != 40 || word(source + 16) != pointer_width) {
        return false;
    }
    const uint32_t file_size = word(source + 24);
    const uint32_t code_size = word(source + 32);
    const uint32_t fixup_count = word(source + 40);
    if (file_size == 0 || file_size > size - header_size || code_size == 0 || code_size > file_size ||
        fixup_count > file_size / pointer_width ||
        fixup_count > (size - header_size - file_size) / 4U) {
        return false;
    }
    const uintptr_t code_address = reinterpret_cast<uintptr_t>(source + header_size);
    if (code_address > UINT32_MAX) {
        return false;
    }
    uint32_t previous_end = 0;
    const uint8_t *fixups = source + header_size + file_size;
    for (uint32_t i = 0; i < fixup_count; i++) {
        const uint32_t destination = word(fixups + i * 4U);
        if (destination < previous_end || destination % pointer_width != 0 ||
            destination > file_size - pointer_width) {
            return false;
        }
        previous_end = destination + pointer_width;
        if (destination >= code_size) {
            continue;
        }
        const uint32_t linked = word(source + header_size + destination);
        if (linked < code_address || linked - code_address >= code_size) {
            return false;
        }
        const uint32_t target = linked - code_address;
        for (uint8_t j = 0; j < pointer_width; j++) {
            copy[header_size + destination + j] = uint8_t(target >> (j * 8));
        }
    }
    return true;
}
#endif

#if AP_ROMFS_ENABLED
#include <ap_romfs_embedded.h>
#else
const AP_ROMFS::embedded_file AP_ROMFS::files[] = {};
#endif

/*
  find an embedded file
*/
const AP_ROMFS::embedded_file *AP_ROMFS::find_file(const char *name)
{
#if UINTPTR_MAX == UINT32_MAX
    static_assert(sizeof(embedded_file) == 20, "ROMFS table entry size changed");
#endif
    for (uint16_t i=0; i<ARRAY_SIZE(files); i++) {
        if (strcmp(name, files[i].filename) == 0) {
            return &files[i];
        }
    }
    return nullptr;
}

/*
  Find the named file and return its decompressed data and size. Caller must
  call AP_ROMFS::free() on the return value after use to free it. The data is
  guaranteed to be null-terminated such that it can be treated as a string.
  Prelinked files are restored to their canonical source representation.
*/
const uint8_t *AP_ROMFS::find_decompress(const char *name, uint32_t &size)
{
    const struct embedded_file *f = find_file(name);
    if (f == nullptr) {
        return nullptr;
    }

#ifdef HAL_ROMFS_UNCOMPRESSED
    if (!f->uncompressed()) {
        return nullptr;
    }
    size = f->decompressed_size;
    return f->contents;
#else
    // add one byte for null termination; ArduPilot's malloc will zero it.
    uint8_t *decompressed_data = (uint8_t *)malloc(f->decompressed_size+1);
    if (!decompressed_data) {
        return nullptr;
    }

    if (f->decompressed_size == 0) {
        // empty file, avoid decompression problems
        size = 0;
        return decompressed_data;
    }

    if (f->uncompressed()) {
        memcpy(decompressed_data, f->contents, f->decompressed_size);
#ifdef HAL_HAVE_AP_ROMFS_PRELINKED_APMOD
        if (f->prelinked() && !canonicalise_apmod(f->contents, decompressed_data, f->decompressed_size)) {
            ::free(decompressed_data);
            return nullptr;
        }
#endif
        if (crc32_small(0, decompressed_data, f->decompressed_size) != f->crc) {
            ::free(decompressed_data);
            return nullptr;
        }
        size = f->decompressed_size;
        return decompressed_data;
    }

    TINF_DATA *d = (TINF_DATA *)malloc(sizeof(TINF_DATA));
    if (!d) {
        ::free(decompressed_data);
        return nullptr;
    }
    uzlib_uncompress_init(d, NULL, 0);

    d->source = f->contents;
    d->source_limit = f->contents + f->stored_size();
    d->dest = decompressed_data;
    d->destSize = f->decompressed_size;

    int res = uzlib_uncompress(d);

    ::free(d);
    
    if (res != TINF_OK) {
        ::free(decompressed_data);
        return nullptr;
    }

    if (crc32_small(0, decompressed_data, f->decompressed_size) != f->crc) {
        ::free(decompressed_data);
        return nullptr;
    }
    
    size = f->decompressed_size;
    return decompressed_data;
#endif
}

/*
  Return a borrowed physical view of an uncompressed file. This is used for
  files such as native modules whose prelinked contents need to remain at
  their linked flash address. The returned data has firmware lifetime and is
  never freed.
 */
const uint8_t *AP_ROMFS::find_uncompressed(const char *name, uint32_t &size)
{
    const struct embedded_file *f = find_file(name);
    if (f == nullptr || !f->uncompressed()) {
        return nullptr;
    }
    size = f->decompressed_size;
    return f->contents;
}

// free decompressed file data
void AP_ROMFS::free(const uint8_t *data)
{
#ifndef HAL_ROMFS_UNCOMPRESSED
    ::free(const_cast<uint8_t *>(data));
#endif
}

/*
  directory listing interface. Start with ofs=0. Returns pathnames
  that match dirname prefix. Ends with nullptr return when no more
  files found
*/
const char *AP_ROMFS::dir_list(const char *dirname, uint16_t &ofs)
{
    const size_t dlen = strlen(dirname);
    for ( ; ofs < ARRAY_SIZE(files); ofs++) {
        if (strncmp(dirname, files[ofs].filename, dlen) == 0) {
            if (dlen != 0 && files[ofs].filename[dlen] != '/') {
                // only a partial match, or the name of a file rather than
                // of a directory holding it, skip
                continue;
            }
            /*
              prevent duplicate directories: compare through the
              separator, as a file such as "sub.txt" sorts just before
              "sub/" and must not be taken for it
             */
            const char *start_name = files[ofs].filename + (dlen > 0 ? dlen + 1 : 0);
            const char *slash = strchr(start_name, '/');
            if (ofs > 0 && slash != nullptr) {
                const size_t prefix_len = (slash - files[ofs].filename) + 1;
                // strncmp, not memcmp: the previous name may be shorter
                if (strncmp(files[ofs].filename, files[ofs-1].filename, prefix_len) == 0) {
                    continue;
                }
            }
            // found one
            return files[ofs++].filename;
        }
    }
    return nullptr;
}

/*
  find a compressed file and return its size
*/
bool AP_ROMFS::find_size(const char *name, uint32_t &size)
{
    const struct embedded_file *f = find_file(name);
    if (f == nullptr) {
        return false;
    }
    size = f->decompressed_size;
    return true;
}

