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
#include <AP_Common/ExpandingString.h>

#include <AP_Param/AP_Param.h>

class AP_Filesystem_Param : public AP_Filesystem_Backend
{
public:
    // functions that closely match the equivalent posix calls
    int open(const char *fname, int flags) override;
    int close(int fd) override;
    int32_t read(int fd, void *buf, uint32_t count) override;
    int32_t lseek(int fd, int32_t offset, int whence) override;
    int stat(const char *pathname, struct stat *stbuf) override;
    int32_t write(int fd, const void *buf, uint32_t count) override;

private:
    // we maintain two cursors per open file to minimise seeking
    // when filling in gaps
    static constexpr uint8_t num_cursors = 2;

    // only allow up to 4 files at a time
    static constexpr uint8_t max_open_file = 4;

    // maximum size of one packed parameter
    static constexpr uint8_t max_pack_len = AP_MAX_NAME_SIZE + 2 + 4 + 3;

    static constexpr uint16_t pmagic = 0x671b;

    // header at front of the file
    struct header {
        uint16_t magic = pmagic;
        uint16_t num_params;
        uint16_t total_params; // for upload this is total file length
    };

    struct cursor {
        AP_Param::ParamToken token;
        uint32_t token_ofs;
        char last_name[AP_MAX_NAME_SIZE+1];
        uint8_t trailer_len;
        uint8_t trailer[max_pack_len];
        uint16_t idx;
    };

    struct rfile {
        bool open;
        uint16_t read_size;
        uint16_t start;
        uint16_t count;
        uint32_t file_ofs;
        uint32_t file_size;
        struct cursor *cursors;
        ExpandingString *writebuf; // for upload
    } file[max_open_file];

    bool token_seek(const struct rfile &r, const uint32_t data_ofs, struct cursor &c);
    uint8_t pack_param(const struct rfile &r, struct cursor &c, uint8_t *buf);
    bool check_file_name(const char *fname);

    // finish uploading parameters
    bool finish_upload(const rfile &r);
    bool param_upload_parse(const rfile &r, bool &need_retry);
};
