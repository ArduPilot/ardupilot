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
    for (uint16_t i=0; i<ARRAY_SIZE_SIMPLE(files); i++) {
        if (strcmp(name, files[i].filename) == 0) {
            size = files[i].size;
            return files[i].contents;
        }
    }
    return nullptr;
}
