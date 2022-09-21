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

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_MISSION_ENABLED

#include "AP_Filesystem.h"
#include "AP_Filesystem_Mission.h"
#include <AP_Mission/AP_Mission.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_Rally/AP_Rally.h>
#include <GCS_MAVLink/MissionItemProtocol_Rally.h>
#include <GCS_MAVLink/MissionItemProtocol_Fence.h>

extern const AP_HAL::HAL& hal;
extern int errno;

#define IDLE_TIMEOUT_MS 30000

int AP_Filesystem_Mission::open(const char *fname, int flags, bool allow_absolute_paths)
{
    enum MAV_MISSION_TYPE mtype;

    if (!check_file_name(fname, mtype)) {
        errno = ENOENT;
        return -1;
    }
    uint8_t idx;
    bool readonly = ((flags & O_ACCMODE) == O_RDONLY);
    uint32_t now = AP_HAL::millis();
    for (idx=0; idx<max_open_file; idx++) {
        if (now - file[idx].last_op_ms > IDLE_TIMEOUT_MS) {
            file[idx].open = false;
            delete file[idx].writebuf;
            file[idx].writebuf = nullptr;
        }
        if (!readonly && file[idx].writebuf != nullptr) {
            // only one upload at a time
            return -1;
        }
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
    if (!readonly) {
        // setup for upload
        r.writebuf = new ExpandingString();
    } else {
        r.writebuf = nullptr;
    }
    r.last_op_ms = now;

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
    if (r.writebuf != nullptr) {
        bool ok = finish_upload(r);
        delete r.writebuf;
        r.writebuf = nullptr;
        if (!ok) {
            errno = EINVAL;
            return -1;
        }
    }
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

    if (r.writebuf != nullptr) {
        errno = EBADF;
        return -1;
    }

    r.last_op_ms = AP_HAL::millis();

    size_t header_total = 0;
    uint8_t *ubuf = (uint8_t *)buf;

    if (r.file_ofs < sizeof(struct header)) {
        struct header hdr {};
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
    mavlink_mission_item_int_t item {};
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
#if AP_FENCE_ENABLED
    if (strcmp(name, "fence.dat") == 0) {
        mtype = MAV_MISSION_TYPE_FENCE;
        return true;
    }
#endif
    if (strcmp(name, "rally.dat") == 0) {
        mtype = MAV_MISSION_TYPE_RALLY;
        return true;
    }
    return false;
}

/*
  get one item
 */
bool AP_Filesystem_Mission::get_item(uint32_t idx, enum MAV_MISSION_TYPE mtype, mavlink_mission_item_int_t &item) const
{
    switch (mtype) {
    case MAV_MISSION_TYPE_MISSION: {
        auto *mission = AP::mission();
        if (!mission) {
            return false;
        }
        return mission->get_item(idx, item);
    }
#if AP_FENCE_ENABLED
    case MAV_MISSION_TYPE_FENCE:
        return MissionItemProtocol_Fence::get_item_as_mission_item(idx, item);
#endif

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
#if AP_FENCE_ENABLED
        auto *fence = AP::fence();
        if (fence == nullptr) {
            return 0;
        }
        return fence->polyfence().num_stored_items();
#else
        return 0;
#endif
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

/*
  support mission upload
 */
int32_t AP_Filesystem_Mission::write(int fd, const void *buf, uint32_t count)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file[fd];
    if (r.writebuf == nullptr) {
        errno = EBADF;
        return -1;
    }
    r.last_op_ms = AP_HAL::millis();
    struct header hdr;
    if (r.file_ofs == 0 && count >= sizeof(hdr)) {
        // pre-expand the buffer to the full size when we get the header
        memcpy(&hdr, buf, sizeof(hdr));
        if (hdr.num_items < 0xFFFF) {
            const uint32_t flen = sizeof(hdr) + hdr.num_items * MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN;
            if (flen > r.writebuf->get_length()) {
                if (!r.writebuf->append(nullptr, flen - r.writebuf->get_length())) {
                    // not enough memory
                    errno = ENOSPC;
                    return -1;
                }
            }
        }
    }
    if (r.file_ofs + count > r.writebuf->get_length()) {
        if (!r.writebuf->append(nullptr, r.file_ofs + count - r.writebuf->get_length())) {
            errno = ENOSPC;
            return -1;
        }
    }
    uint8_t *b = (uint8_t *)r.writebuf->get_writeable_string();
    memcpy(&b[r.file_ofs], buf, count);
    r.file_ofs += count;
    return count;
}

// see if a block of memory is all zero
bool AP_Filesystem_Mission::all_zero(const uint8_t *b, uint8_t len) const
{
    while (len--) {
        if (*b++ != 0) {
            return false;
        }
    }
    return true;
}

/*
  finish mission upload
 */
bool AP_Filesystem_Mission::finish_upload(const rfile &r)
{
    const uint32_t flen = r.writebuf->get_length();
    const uint8_t *b = (const uint8_t *)r.writebuf->get_string();
    struct header hdr;
    if (flen < sizeof(hdr)) {
        return false;
    }
    const uint8_t item_size = MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN;
    const uint32_t nitems = (flen - sizeof(hdr)) / item_size;

    memcpy(&hdr, b, sizeof(hdr));
    if (hdr.magic != mission_magic) {
        return false;
    }
    if (nitems != hdr.num_items) {
        return false;
    }

    // if any item is all zeros then reject, it means client didn't
    // fill in the whole file
    for (uint32_t i=0; i<nitems; i++) {
        const uint8_t *b2 = b + sizeof(hdr) + i*item_size;
        if (all_zero(b2, item_size)) {
            return false;
        }
    }

    auto *mission = AP::mission();
    if (mission == nullptr) {
        return false;
    }
    WITH_SEMAPHORE(mission->get_semaphore());
    if ((hdr.options & unsigned(Options::NO_CLEAR)) == 0) {
        mission->clear();
    }
    for (uint32_t i=0; i<nitems; i++) {
        mavlink_mission_item_int_t m {};
        AP_Mission::Mission_Command cmd;
        memcpy(&m, &b[sizeof(hdr)+i*item_size], item_size);
        const MAV_MISSION_RESULT res = AP_Mission::mavlink_int_to_mission_cmd(m, cmd);
        if (res != MAV_MISSION_ACCEPTED) {
            return false;
        }
        if (cmd.id == MAV_CMD_DO_JUMP &&
            (cmd.content.jump.target >= nitems || cmd.content.jump.target == 0)) {
            return false;
        }
        uint16_t idx = i + hdr.start;
        if (idx == mission->num_commands()) {
            if (!mission->add_cmd(cmd)) {
                return false;
            }
        } else {
            if (!mission->replace_cmd(idx, cmd)) {
                return false;
            }
        }
    }
    return true;
}

#endif  // AP_FILESYSTEM_MISSION_ENABLED
