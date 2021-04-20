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

#include "AP_Filesystem_backend.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/ExpandingString.h>

class AP_Filesystem_Mission : public AP_Filesystem_Backend
{
public:
    // functions that closely match the equivalent posix calls
    int open(const char *fname, int flags) override;
    int close(int fd) override;
    int32_t read(int fd, void *buf, uint32_t count) override;
    int32_t lseek(int fd, int32_t offset, int whence) override;
    int32_t write(int fd, const void *buf, uint32_t count) override;
    int stat(const char *pathname, struct stat *stbuf) override;

private:
    // only allow up to 4 files at a time
    static constexpr uint8_t max_open_file = 4;

    static constexpr uint16_t mission_magic = 0x763d;

    enum class Options {
        NO_CLEAR = (1U<<0), // don't clear the old mission
    };

    // header at front of the file
    struct header {
        uint16_t magic = mission_magic;
        uint16_t data_type; // MAV_MISSION_TYPE_*
        uint16_t options; // optional features
        uint16_t start; // first WP num, 0 for full upload
        uint16_t num_items;
    };

    struct rfile {
        bool open;
        ExpandingString *writebuf;
        uint32_t file_ofs;
        uint32_t num_items;
        enum MAV_MISSION_TYPE mtype;
        uint32_t last_op_ms;
    } file[max_open_file];

    bool check_file_name(const char *fname, enum MAV_MISSION_TYPE &mtype);

    // get one item
    bool get_item(uint32_t idx, enum MAV_MISSION_TYPE mtype, mavlink_mission_item_int_t &item) const;

    // get number of items
    uint32_t get_num_items(enum MAV_MISSION_TYPE mtype) const;

    // finish loading items
    bool finish_upload(const rfile &r);

    // see if a block of memory is all zero
    bool all_zero(const uint8_t *b, uint8_t size) const;
};
