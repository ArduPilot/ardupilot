// 2 or 3 structures, select one that is before target point, closest to target

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
  ArduPilot filesystem interface for parameters
 */
#include "AP_Filesystem.h"
#include "AP_Filesystem_Param.h"
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#define PACKED_NAME "param.pck"

extern const AP_HAL::HAL& hal;
extern int errno;

int AP_Filesystem_Param::open(const char *fname, int flags)
{
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
    r.cursors = new cursor[num_cursors];
    if (r.cursors == nullptr) {
        errno = ENOMEM;
        return -1;
    }
    r.file_ofs = 0;
    r.open = true;
    return idx;
}

int AP_Filesystem_Param::close(int fd)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file[fd];
    r.open = false;
    delete [] r.cursors;
    return 0;
}

/*
  packed format:
    uint8_t type:4;         // AP_Param type NONE=0, INT8=1, INT16=2, INT32=3, FLOAT=4
    uint8_t type_len:4;     // number of bytes in type
    uint8_t common_len:4;   // number of name bytes in common with previous entry, 0..15
    uint8_t name_len:4;     // non-common length of param name -1 (0..15)
    uint8_t name[name_len]; // name
    uint8_t data[];         // value, may be truncated by record_length
 */

/*
  pack a single parameter
 */
uint8_t AP_Filesystem_Param::pack_param(const AP_Param *ap, const char *pname, const char *last_name,
                                        enum ap_var_type ptype, uint8_t *buf, uint8_t buflen)
{
    uint8_t common_len = 0;
    while (*pname == *last_name) {
        common_len++;
        pname++;
        last_name++;
    }
    const uint8_t name_len = strlen(pname);
    uint8_t type_len = AP_Param::type_size(ptype);
    const uint8_t *pbuf = (const uint8_t *)ap;
    while (type_len > 0 && pbuf[type_len-1] == 0) {
        type_len--;
    }
    uint8_t packed_len = type_len + name_len + 2;
    if (packed_len <= buflen && buf) {
        buf[0] = uint8_t(ptype) | (type_len<<4);
        buf[1] = common_len | ((name_len-1)<<4);
        memcpy(&buf[2], pname, name_len);
        memcpy(&buf[2+name_len], ap, type_len);
    }
    return packed_len;
}

/*
  seek the token to match file offset
 */
bool AP_Filesystem_Param::token_seek(const struct rfile &r, struct cursor &c)
{
    //hal.console->printf("token_seek %u %u\n", unsigned(r.file_ofs), unsigned(c.token_ofs));

    if (r.file_ofs == 0) {
        memset(&c, 0, sizeof(c));
        return true;
    }
    if (c.token_ofs > r.file_ofs) {
        memset(&c, 0, sizeof(c));
    }

    char name[AP_MAX_NAME_SIZE+1];
    name[AP_MAX_NAME_SIZE] = 0;
    enum ap_var_type ptype;

    if (c.trailer_len > 0) {
        uint8_t n = MIN(c.trailer_len, r.file_ofs - c.token_ofs);
        if (n != c.trailer_len) {
            memmove(&c.trailer[0], &c.trailer[n], c.trailer_len - n);
        }
        c.trailer_len -= n;
        c.token_ofs += n;
    }
    
    while (r.file_ofs != c.token_ofs) {
        AP_Param *ap;

        if (c.token_ofs == 0) {
            ap = AP_Param::first(&c.token, &ptype);
        } else {
            ap = AP_Param::next_scalar(&c.token, &ptype);
        }
        if (ap == nullptr) {
            break;
        }
        ap->copy_name_token(c.token, name, AP_MAX_NAME_SIZE, true);
        uint32_t available = r.file_ofs - c.token_ofs;
        uint8_t tbuf[max_pack_len];
        uint8_t len = pack_param(ap, name, c.last_name, ptype, tbuf, max_pack_len);
        uint8_t n = MIN(len, available);
        if (len > available) {
            c.trailer_len = len - available;
            memcpy(c.trailer, &tbuf[n], c.trailer_len);
        }
        strcpy(c.last_name, name);
        c.token_ofs += n;
    }
    return r.file_ofs == c.token_ofs;
}

int32_t AP_Filesystem_Param::read(int fd, void *buf, uint32_t count)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file[fd];
    uint8_t best_i = 0;
    uint32_t best_ofs = r.cursors[0].token_ofs;

    // find the first cursor that is positioned after the file offset
    for (uint8_t i=1; i<num_cursors; i++) {
        struct cursor &c = r.cursors[i];
        if (c.token_ofs >= r.file_ofs && c.token_ofs < best_ofs) {
            best_i = i;
            best_ofs = c.token_ofs;
        }
    }
    struct cursor &c = r.cursors[best_i];

    if (r.file_ofs != c.token_ofs) {
        if (!token_seek(r, c)) {
            // must be EOF
            return 0;
        }
    }
    if (count == 0) {
        return 0;
    }
    uint8_t *ubuf = (uint8_t *)buf;

    char name[AP_MAX_NAME_SIZE+1];
    name[AP_MAX_NAME_SIZE] = 0;
    size_t total = 0;

    if (c.trailer_len > 0) {
        uint8_t n = MIN(c.trailer_len, count);
        memcpy(ubuf, c.trailer, n);
        count -= n;
        ubuf += n;
        if (n != c.trailer_len) {
            memmove(&c.trailer[0], &c.trailer[n], c.trailer_len - n);
        }
        c.trailer_len -= n;
        total += n;
        c.token_ofs += n;
    }

    while (count > 0) {
        AP_Param *ap;
        enum ap_var_type ptype;

        if (c.token_ofs == 0) {
            ap = AP_Param::first(&c.token, &ptype);
        } else {
            ap = AP_Param::next_scalar(&c.token, &ptype);
        }
        if (ap == nullptr) {
            break;
        }
        ap->copy_name_token(c.token, name, AP_MAX_NAME_SIZE, true);
        uint8_t tbuf[max_pack_len];
        uint8_t len = pack_param(ap, name, c.last_name, ptype, tbuf, sizeof(tbuf));
        uint8_t n = MIN(len, count);
        if (len > count) {
            c.trailer_len = len - count;
            memcpy(c.trailer, &tbuf[count], c.trailer_len);
        }
        memcpy(ubuf, tbuf, n);
        strcpy(c.last_name, name);
        count -= n;
        ubuf += n;
        total += n;
        c.token_ofs += n;
    }
    r.file_ofs += total;
    return total;
}

int32_t AP_Filesystem_Param::lseek(int fd, int32_t offset, int seek_from)
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

int AP_Filesystem_Param::stat(const char *name, struct stat *stbuf)
{
    if (strcmp(name, PACKED_NAME) != 0) {
        errno = ENOENT;
        return -1;
    }
    memset(stbuf, 0, sizeof(*stbuf));
    // give fixed size
    stbuf->st_size = 65535;
    return 0;
}
