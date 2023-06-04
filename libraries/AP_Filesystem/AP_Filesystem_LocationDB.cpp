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
  ArduPilot filesystem interface for LocationDB 
 */

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_LOCATIONDB_ENABLED

#include "AP_Filesystem.h"
#include "AP_Filesystem_LocationDB.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;
extern int errno;

#define IDLE_TIMEOUT_MS 30000

int AP_Filesystem_LocationDB::open(const char *fname, int flags, bool allow_absolute_paths)
{
    auto *db = AP::locationdb();
    if (!check_file_name(fname) || !db) {
        errno = ENOENT;
        return -1;
    }

    const bool readonly = ((flags & O_ACCMODE) == O_RDONLY);

    if (!readonly) { 
        // we only allow reads
        return -1;
    }

    const uint32_t now = AP_HAL::millis();
    if (now - file.last_op_ms > IDLE_TIMEOUT_MS) {
        file.open = false;
    }

    struct rfile &r = file;
    r.file_ofs = 0;
    r.open = true;
    r.num_items = db->size();
    r.last_op_ms = now;

    return 0;
}

int AP_Filesystem_LocationDB::close(int fd)
{
    if (fd < 0 || fd >= 1 || !file.open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file;
    r.open = false;
    return 0;
}

/*
  packed format:
    file header:
      uint16_t magic = 0x2801
      uint16_t total_items

    per-entry is AP_LocationDB_Item
 */

/*
  read from file handle
 */
int32_t AP_Filesystem_LocationDB::read(int fd, void *buf, uint32_t count)
{
    if (fd < 0 || fd >= 1 || !file.open) {
        errno = EBADF;
        return -1;
    }

    struct rfile &r = file;

    r.last_op_ms = AP_HAL::millis();

    size_t header_total = 0;
    uint8_t *ubuf = (uint8_t *)buf;

    if (r.file_ofs < sizeof(struct header)) {
        struct header hdr {};
        hdr.num_items = r.num_items;
        uint8_t n = MIN(sizeof(hdr) - r.file_ofs, count);
        const uint8_t *b = (const uint8_t *)&hdr;
        memcpy(ubuf, &b[r.file_ofs], n);
        count -= n;
        header_total += n;
        r.file_ofs += n;
        ubuf += n;
        if (count == 0) {
            return header_total;
        }
    }

    uint32_t data_ofs = r.file_ofs - sizeof(struct header);
    const uint8_t item_size = sizeof(packet_t);
    uint32_t total = 0;

    // using for loop to make sure we exit after finite number of iterations
    for (uint16_t iter_count = 0; iter_count < 500 && count > 0; iter_count++) {
        uint32_t item_idx = data_ofs / item_size;
        uint32_t item_ofs = data_ofs % item_size;
        AP_LocationDB_Item item;
        if (!get_item(item_idx, item)) {
            break;
        }

        packet_t packet;
        construct_packet_from_DBItem(packet, item);
        const uint8_t *ibuf = (const uint8_t *)&packet;

        const uint8_t compressed_packet_size = sizeof(packet_header) + packet.pkt_header.filled_fields_count * sizeof(float);
        
        if (item_ofs >= compressed_packet_size) {
            // this item has been transferred
            // move to next item
            data_ofs += (item_size - item_ofs);
            continue;
        }

        const uint8_t n = MIN(compressed_packet_size - item_ofs, count);
        memcpy(ubuf, &ibuf[item_ofs], n);

        ubuf += n;
        count -= n;
        total += n;
        data_ofs += n;
    }

    r.file_ofs = data_ofs + sizeof(struct header);
    return total + header_total;
}

int32_t AP_Filesystem_LocationDB::lseek(int fd, int32_t offset, int seek_from)
{
    if (fd < 0 || fd >= 1 || !file.open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file;
    switch (seek_from) {
    case SEEK_SET:
        r.file_ofs = offset;
        break;
    case SEEK_CUR:
        r.file_ofs += offset;
        break;
    case SEEK_END:
        errno = EINVAL;
        return -1;
    }
    return r.file_ofs;
}

int AP_Filesystem_LocationDB::stat(const char *name, struct stat *stbuf)
{
    auto *db = AP::locationdb();
    if (!check_file_name(name) || !db) {
        errno = ENOENT;
        return -1;
    }
    memset(stbuf, 0, sizeof(*stbuf));
    // give fixed size to avoid needing to scan entire file
    stbuf->st_size = db->size() * sizeof(AP_LocationDB_Item);
    return 0;
}

/*
  check for the right file name
 */
bool AP_Filesystem_LocationDB::check_file_name(const char *name)
{
    if (strcmp(name, "locationdb.dat") == 0) {
        return true;
    }
    return false;
}

void AP_Filesystem_LocationDB::construct_packet_from_DBItem(packet_t &ret, AP_LocationDB_Item item)
{   
    ret.pkt_header.key = item.get_key();
    ret.pkt_header.timestamp_ms = item.get_timestamp_ms();
    ret.pkt_header.populated_fields = item._populated_fields;

    Location item_loc;
    uint8_t field_index = 0;
    if (item.field_is_populated(AP_LocationDB_Item::DataField::POS)) {
        item_loc = Location(item._pos, Location::AltFrame::ABOVE_ORIGIN);
    #if AP_AHRS_ENABLED
        IGNORE_RETURN(item_loc.change_alt_frame(Location::AltFrame::ABSOLUTE));
    #endif
        ret.fields[field_index++] = item_loc.lat;
        ret.fields[field_index++] = item_loc.lng;
        ret.fields[field_index++] = item_loc.alt;
    }

    if (item.field_is_populated(AP_LocationDB_Item::DataField::VEL)) {
        ret.fields[field_index++] = item._vel.x;
        ret.fields[field_index++] = item._vel.y;
        ret.fields[field_index++] = item._vel.z;
    }

    if (item.field_is_populated(AP_LocationDB_Item::DataField::ACC)) {
        ret.fields[field_index++] = item._acc.x;
        ret.fields[field_index++] = item._acc.y;
        ret.fields[field_index++] = item._acc.z;
    }

    if (item.field_is_populated(AP_LocationDB_Item::DataField::HEADING)) {
        ret.fields[field_index++] = item._heading;
    }

    if (item.field_is_populated(AP_LocationDB_Item::DataField::RADIUS)) {
        ret.fields[field_index++] = item._radius;
    }

    ret.pkt_header.filled_fields_count = field_index;
}

/*
  get one item
 */
bool AP_Filesystem_LocationDB::get_item(uint16_t idx, AP_LocationDB_Item &item) const
{
    auto *db = AP::locationdb();
    if (!db) {
        return false;
    }

    return db->get_item_at_index(idx, item);
}

#endif
