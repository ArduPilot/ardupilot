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

#ifdef HAL_HAVE_AP_ROMFS_EMBEDDED_H
#include <ap_romfs_embedded.h>
#else
const AP_ROMFS::embedded_file AP_ROMFS::files[] = {};
#endif

/*
  find an embedded file
*/
const uint8_t *AP_ROMFS::find_file(const char *name, uint32_t &size)
{
    for (uint16_t i=0; i<ARRAY_SIZE(files); i++) {
        if (strcmp(name, files[i].filename) == 0) {
            size = files[i].size;
            return files[i].contents;
        }
    }
    return nullptr;
}

/*
  find a compressed file and uncompress it. Space for decompressed
  data comes from malloc. Caller must be careful to free the resulting
  data after use. The next byte after the file data is guaranteed to
  be null
*/
const uint8_t *AP_ROMFS::find_decompress(const char *name, uint32_t &size)
{
    uint32_t compressed_size;
    const uint8_t *compressed_data = find_file(name, compressed_size);
    if (!compressed_data) {
        return nullptr;
    }

#ifdef HAL_ROMFS_UNCOMPRESSED
    return compressed_data;
#else
    // last 4 bytes of gzip file are length of decompressed data
    const uint8_t *p = &compressed_data[compressed_size-4];
    uint32_t decompressed_size = p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24;
    
    uint8_t *decompressed_data = (uint8_t *)malloc(decompressed_size + 1);
    if (!decompressed_data) {
        return nullptr;
    }

    // explicitly null terimnate the data
    decompressed_data[decompressed_size] = 0;

    TINF_DATA *d = (TINF_DATA *)malloc(sizeof(TINF_DATA));
    if (!d) {
        ::free(decompressed_data);
        return nullptr;
    }
    uzlib_uncompress_init(d, NULL, 0);

    d->source = compressed_data;
    d->source_limit = compressed_data + compressed_size - 4;

    // assume gzip format
    int res = uzlib_gzip_parse_header(d);
    if (res != TINF_OK) {
        ::free(decompressed_data);
        ::free(d);
        return nullptr;
    }

    d->dest = decompressed_data;
    d->destSize = decompressed_size;

    // we don't check CRC, as it just wastes flash space for constant
    // ROMFS data
    res = uzlib_uncompress(d);

    ::free(d);
    
    if (res != TINF_OK) {
        ::free(decompressed_data);
        return nullptr;
    }

    size = decompressed_size;
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
