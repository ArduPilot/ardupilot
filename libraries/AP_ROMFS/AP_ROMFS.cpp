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

#include <string.h>

#ifdef HAL_HAVE_AP_ROMFS_EMBEDDED_H
#include <ap_romfs_embedded.h>
#else
const AP_ROMFS::embedded_file AP_ROMFS::files[] = {};
#endif

/*
  find an embedded file
*/
const AP_ROMFS::embedded_file *AP_ROMFS::find_file(const char *name)
{
    for (uint16_t i=0; i<ARRAY_SIZE(files); i++) {
        if (strcmp(name, files[i].filename) == 0) {
            return &files[i];
        }
    }
    return nullptr;
}

/*
  find a compressed file and uncompress it. Space for decompressed data comes
  from malloc. Caller must be careful to free the resulting data after use. The
  file data buffer is guaranteed to contain at least one null (though it may be
  at buf[size]).
*/
const uint8_t *AP_ROMFS::find_decompress(const char *name, uint32_t &size)
{
    const struct embedded_file *f = find_file(name);
    if (!f) {
        return nullptr;
    }

#ifdef HAL_ROMFS_UNCOMPRESSED
    size = f->decompressed_size;
    return f->contents;
#else
    uint8_t *decompressed_data = (uint8_t *)malloc(f->decompressed_size+1);
    if (!decompressed_data) {
        return nullptr;
    }

    // explicitly null-terminate the data
    decompressed_data[f->decompressed_size] = 0;

    TINF_DATA *d = (TINF_DATA *)malloc(sizeof(TINF_DATA));
    if (!d) {
        ::free(decompressed_data);
        return nullptr;
    }
    uzlib_uncompress_init(d, NULL, 0);

    d->source = f->contents;
    d->source_limit = f->contents + f->compressed_size;
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

// free returned data
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
        if (strncmp(dirname, files[ofs].filename, dlen) == 0 &&
            files[ofs].filename[dlen] == '/') {
            // found one
            return files[ofs++].filename;
        }
    }
    return nullptr;
}
