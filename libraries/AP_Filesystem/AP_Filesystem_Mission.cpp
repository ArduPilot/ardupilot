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
  ArduPilot filesystem interface for mission/fence/rally
 */
#include "AP_Filesystem.h"
#include "AP_Filesystem_Mission.h"
#include <AP_Mission/AP_Mission.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_Rally/AP_Rally.h>
#include <GCS_MAVLink/MissionItemProtocol_Rally.h>
#include <GCS_MAVLink/MissionItemProtocol_Fence.h>

extern const AP_HAL::HAL& hal;
extern int errno;

int AP_Filesystem_Mission::open(const char *fname, int flags)
{
    enum MAV_MISSION_TYPE mtype;

    if (!check_file_name(fname, mtype)) {
        errno = ENOENT;
        return -1;
    }
    if ((flags & O_ACCMODE) != O_RDONLY) {
        return -1;
    }
    uint8_t idx;
    for (idx=0; idx<max_open_file; idx++) {
        if (!file[idx].open) {
            break;
        }
    }
    if (idx == max_open_file) {
        errno = ENFILE;
        return -1;
    }
    struct rfile &r = file[idx];
    r.file_ofs = 0;
    r.open = true;
    r.mtype = mtype;
    r.num_items = get_num_items(r.mtype);

    return idx;
}

int AP_Filesystem_Mission::close(int fd)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file[fd];
    r.open = false;
    return 0;
}

/*
  packed format:
    file header:
      uint16_t magic = 0x671b
      uint16_t data_type MAV_MISSION_TYPE_*
      uint32_t total_items

    per-entry is mavlink packed item
 */

/*
  read from file handle
 */
int32_t AP_Filesystem_Mission::read(int fd, void *buf, uint32_t count)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }

    struct rfile &r = file[fd];
    size_t header_total = 0;
    uint8_t *ubuf = (uint8_t *)buf;

    if (r.file_ofs < sizeof(struct header)) {
        struct header hdr;
        hdr.data_type = uint16_t(r.mtype);
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
    mavlink_mission_item_int_t item;
    const uint8_t item_size = MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN;
    uint32_t item_ofs = data_ofs % item_size;
    uint32_t total = 0;

    while (count > 0) {
        uint32_t item_idx = data_ofs / item_size;

        const uint8_t *ibuf = (const uint8_t *)&item;
        if (!get_item(item_idx, r.mtype, item)) {
            break;
        }
        const uint8_t n = MIN(item_size - item_ofs, count);
        memcpy(ubuf, &ibuf[item_ofs], n);
        ubuf += n;
        count -= n;
        total += n;
        item_ofs = 0;
        data_ofs += n;
    }

    r.file_ofs += total;
    return total + header_total;
}

int32_t AP_Filesystem_Mission::lseek(int fd, int32_t offset, int seek_from)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file[fd];
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

int AP_Filesystem_Mission::stat(const char *name, struct stat *stbuf)
{
    enum MAV_MISSION_TYPE mtype;
    if (!check_file_name(name, mtype)) {
        errno = ENOENT;
        return -1;
    }
    memset(stbuf, 0, sizeof(*stbuf));
    // give fixed size to avoid needing to scan entire file
    stbuf->st_size = 1024*1024;
    return 0;
}

/*
  check for the right file name
 */
bool AP_Filesystem_Mission::check_file_name(const char *name, enum MAV_MISSION_TYPE &mtype)
{
    if (strcmp(name, "mission.dat") == 0) {
        mtype = MAV_MISSION_TYPE_MISSION;
        return true;
    }
    if (strcmp(name, "fence.dat") == 0) {
        mtype = MAV_MISSION_TYPE_FENCE;
        return true;
    }
    if (strcmp(name, "rally.dat") == 0) {
        mtype = MAV_MISSION_TYPE_RALLY;
        return true;
    }
    return false;
}

/*
  get one item
 */
bool AP_Filesystem_Mission::get_item(uint32_t idx, enum MAV_MISSION_TYPE mtype, mavlink_mission_item_int_t &item)
{
    switch (mtype) {
    case MAV_MISSION_TYPE_MISSION: {
        auto *mission = AP::mission();
        if (!mission) {
            return false;
        }
        return mission->get_item(idx, item);
    }
    case MAV_MISSION_TYPE_FENCE:
        return MissionItemProtocol_Fence::get_item_as_mission_item(idx, item);

    case MAV_MISSION_TYPE_RALLY:
        return MissionItemProtocol_Rally::get_item_as_mission_item(idx, item);

    default:
        break;
    }
    return false;
}

// get number of items
uint32_t AP_Filesystem_Mission::get_num_items(enum MAV_MISSION_TYPE mtype) const
{
    switch (mtype) {
    case MAV_MISSION_TYPE_MISSION: {
        auto *mission = AP::mission();
        if (!mission) {
            return 0;
        }
        return mission->num_commands();
    }
        
    case MAV_MISSION_TYPE_FENCE: {
        auto *fence = AP::fence();
        if (fence == nullptr) {
            return 0;
        }
        return fence->polyfence().num_stored_items();
    }

    case MAV_MISSION_TYPE_RALLY: {
        auto *rally = AP::rally();
        if (rally == nullptr) {
            return 0;
        }
        return rally->get_rally_total();
    }
        
    default:
        break;
    }
    return 0;
}
