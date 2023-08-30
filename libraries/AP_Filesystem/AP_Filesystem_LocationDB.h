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

#pragma once

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_LOCATIONDB_ENABLED

#include "AP_Filesystem_backend.h"
#include <AP_LocationDB/AP_LocationDB.h>

class AP_Filesystem_LocationDB : public AP_Filesystem_Backend
{
public:
    // functions that closely match the equivalent posix calls
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    int close(int fd) override;
    int32_t read(int fd, void *buf, uint32_t count) override;
    int32_t lseek(int fd, int32_t offset, int whence) override;
    int stat(const char *pathname, struct stat *stbuf) override;

private:
    static constexpr uint16_t locationdb_magic = 0x2801;

    // header at front of the file
    struct header {
        uint16_t magic = locationdb_magic;
        uint16_t num_items;
    };

    struct rfile {
        bool open;
        uint32_t file_ofs;
        uint16_t num_items;
        uint32_t last_op_ms;
    } file;

    struct PACKED packet_header {
        uint32_t key;
        uint32_t timestamp_ms;
        uint8_t populated_fields;
        uint8_t filled_fields_count;
    };

    struct PACKED packet_t {
        packet_header pkt_header;
        float fields[11];
    };

    bool check_file_name(const char *fname);

    // get one item
    bool get_item(uint16_t idx, AP_LocationDB_Item &item) const;
    void construct_packet_from_DBItem(packet_t &ret, AP_LocationDB_Item item);
    bool skip_byte(AP_LocationDB_Item &item, uint32_t byte_num) const;
};

#endif
