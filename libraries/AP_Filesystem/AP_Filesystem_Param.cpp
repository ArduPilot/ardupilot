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
#include <ctype.h>

#define PACKED_NAME "param.pck"

extern const AP_HAL::HAL& hal;
extern int errno;

int AP_Filesystem_Param::open(const char *fname, int flags)
{
    if (!check_file_name(fname)) {
        errno = ENOENT;
        return -1;
    }
    bool read_only = ((flags & O_ACCMODE) == O_RDONLY);
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
    if (read_only) {
        r.cursors = new cursor[num_cursors];
        if (r.cursors == nullptr) {
            errno = ENOMEM;
            return -1;
        }
    }
    r.file_ofs = 0;
    r.open = true;
    r.start = 0;
    r.count = 0;
    r.read_size = 0;
    r.file_size = 0;
    r.writebuf = nullptr;
    if (!read_only) {
        // setup for upload
        r.writebuf = new ExpandingString();
        if (r.writebuf == nullptr) {
            close(idx);
            errno = ENOMEM;
            return -1;
        }
    }

    /*
      allow for URI style arguments param.pck?start=N&count=C
     */
    const char *c = strchr(fname, '?');
    while (c && *c) {
        c++;
        if (strncmp(c, "start=", 6) == 0) {
            uint32_t v = strtoul(c+6, nullptr, 10);
            if (v >= UINT16_MAX) {
                goto failed;
            }
            r.start = v;
            c += 6;
            c = strchr(c, '&');
            continue;
        }
        if (strncmp(c, "count=", 6) == 0) {
            uint32_t v = strtoul(c+6, nullptr, 10);
            if (v >= UINT16_MAX) {
                goto failed;
            }
            r.count = v;
            c += 6;
            c = strchr(c, '&');
            continue;
        }
    }

    return idx;

failed:
    delete [] r.cursors;
    r.open = false;
    errno = EINVAL;
    return -1;
}

int AP_Filesystem_Param::close(int fd)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file[fd];
    int ret = 0;
    if (r.writebuf != nullptr && !finish_upload(r)) {
        errno = EINVAL;
        ret = -1;
    }
    r.open = false;
    delete [] r.cursors;
    r.cursors = nullptr;
    delete r.writebuf;
    r.writebuf = nullptr;
    return ret;
}

/*
  packed format:
    file header:
      uint16_t magic = 0x671b
      uint16_t num_params
      uint16_t total_params

    per-parameter:

    uint8_t type:4;         // AP_Param type NONE=0, INT8=1, INT16=2, INT32=3, FLOAT=4
    uint8_t flags:4;        // for future use
    uint8_t common_len:4;   // number of name bytes in common with previous entry, 0..15
    uint8_t name_len:4;     // non-common length of param name -1 (0..15)
    uint8_t name[name_len]; // name
    uint8_t data[];         // value, length given by variable type

    Any leading zero bytes after the header should be discarded as pad
    bytes. Pad bytes are used to ensure that a parameter data[] field
    does not cross a read packet boundary
 */

/*
  pack a single parameter. The buffer must be at least of size max_pack_len
 */
uint8_t AP_Filesystem_Param::pack_param(const struct rfile &r, struct cursor &c, uint8_t *buf)
{
    char name[AP_MAX_NAME_SIZE+1];
    name[AP_MAX_NAME_SIZE] = 0;
    enum ap_var_type ptype;
    AP_Param *ap;

    if (c.token_ofs == 0) {
        c.idx = 0;
        ap = AP_Param::first(&c.token, &ptype);
        uint16_t idx = 0;
        while (idx < r.start && ap) {
            ap = AP_Param::next_scalar(&c.token, &ptype);
            idx++;
        }
    } else {
        c.idx++;
        ap = AP_Param::next_scalar(&c.token, &ptype);
    }
    if (ap == nullptr || (r.count && c.idx >= r.count)) {
        return 0;
    }
    ap->copy_name_token(c.token, name, AP_MAX_NAME_SIZE, true);

    uint8_t common_len = 0;
    const char *last_name = c.last_name;
    const char *pname = name;
    while (*pname == *last_name && *pname) {
        common_len++;
        pname++;
        last_name++;
    }
    uint8_t name_len = strlen(pname);
    if (name_len == 0) {
        name_len = 1;
        common_len--;
        pname--;
    }
    const uint8_t type_len = AP_Param::type_size(ptype);
    uint8_t packed_len = type_len + name_len + 2;
    const uint8_t flags = 0;

    /*
      see if we need to add padding to ensure that a data field never
      crosses a block boundary. This ensures that re-reading a block
      won't get a corrupt value for a parameter
     */
    if (type_len > 1) {
        const uint32_t ofs = c.token_ofs + sizeof(struct header) + packed_len;
        const uint32_t ofs_mod = ofs % r.read_size;
        if (ofs_mod > 0 && ofs_mod < type_len) {
            const uint8_t pad = type_len - ofs_mod;
            memset(buf, 0, pad);
            buf += pad;
            packed_len += pad;
        }
    }

    buf[0] = uint8_t(ptype) | (flags<<4);
    buf[1] = common_len | ((name_len-1)<<4);
    memcpy(&buf[2], pname, name_len);
    memcpy(&buf[2+name_len], ap, type_len);

    strcpy(c.last_name, name);

    return packed_len;
}

/*
  seek the token to match file offset
 */
bool AP_Filesystem_Param::token_seek(const struct rfile &r, const uint32_t data_ofs, struct cursor &c)
{
    if (data_ofs == 0) {
        memset(&c, 0, sizeof(c));
        return true;
    }
    if (c.token_ofs > data_ofs) {
        memset(&c, 0, sizeof(c));
    }

    if (c.trailer_len > 0) {
        uint8_t n = MIN(c.trailer_len, data_ofs - c.token_ofs);
        if (n != c.trailer_len) {
            memmove(&c.trailer[0], &c.trailer[n], c.trailer_len - n);
        }
        c.trailer_len -= n;
        c.token_ofs += n;
    }
    
    while (data_ofs != c.token_ofs) {
        uint32_t available = data_ofs - c.token_ofs;
        uint8_t tbuf[max_pack_len];
        uint8_t len = pack_param(r, c, tbuf);
        if (len == 0) {
            // no more parameters
            break;
        }
        uint8_t n = MIN(len, available);
        if (len > available) {
            c.trailer_len = len - available;
            memcpy(c.trailer, &tbuf[n], c.trailer_len);
        }
        c.token_ofs += n;
    }
    return data_ofs == c.token_ofs;
}

int32_t AP_Filesystem_Param::read(int fd, void *buf, uint32_t count)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }

    struct rfile &r = file[fd];
    if (r.writebuf != nullptr) {
        // no read on upload
        errno = EINVAL;
        return -1;
    }
    size_t header_total = 0;

    /*
      we only allow for a single read size. This ensures that pad
      bytes placed to avoid a data value crossing a block boundary in
      the ftp protocol do not change when filling in lost packets
     */
    if (r.read_size == 0 && count > 0) {
        r.read_size = count;
    }
    if (r.read_size != 0 && r.read_size != count) {
        errno = EINVAL;
        return -1;
    }

    if (r.file_size != 0) {
        // ensure we don't try to read past EOF
        if (r.file_ofs > r.file_size) {
            count = 0;
        } else {
            count = MIN(count, r.file_size - r.file_ofs);
        }
    }

    if (r.file_ofs < sizeof(struct header)) {
        struct header hdr;
        hdr.total_params = AP_Param::count_parameters();
        if (hdr.total_params <= r.start) {
            errno = EINVAL;
            return -1;
        }
        hdr.num_params = hdr.total_params - r.start;
        if (r.count > 0 && hdr.num_params > r.count) {
            hdr.num_params = r.count;
        }
        uint8_t n = MIN(sizeof(hdr) - r.file_ofs, count);
        const uint8_t *b = (const uint8_t *)&hdr;
        memcpy(buf, &b[r.file_ofs], n);
        count -= n;
        header_total += n;
        r.file_ofs += n;
        buf = (void *)(n + (const uint8_t *)buf);
        if (count == 0) {
            return header_total;
        }
    }

    uint32_t data_ofs = r.file_ofs - sizeof(struct header);
    uint8_t best_i = 0;
    uint32_t best_ofs = r.cursors[0].token_ofs;
    size_t total = 0;

    // find the first cursor that is positioned after the file offset
    for (uint8_t i=1; i<num_cursors; i++) {
        struct cursor &c = r.cursors[i];
        if (c.token_ofs >= data_ofs && c.token_ofs < best_ofs) {
            best_i = i;
            best_ofs = c.token_ofs;
        }
    }
    struct cursor &c = r.cursors[best_i];

    if (data_ofs != c.token_ofs) {
        if (!token_seek(r, data_ofs, c)) {
            // must be EOF
            return header_total;
        }
    }
    if (count == 0) {
        return header_total;
    }
    uint8_t *ubuf = (uint8_t *)buf;

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
        uint8_t tbuf[max_pack_len];
        uint8_t len = pack_param(r, c, tbuf);
        if (len == 0) {
            // no more params, use this to trigger EOF in later reads
            const uint32_t size = r.file_ofs + total;
            if (r.file_size == 0) {
                r.file_size = size;
            } else {
                r.file_size = MIN(size, r.file_size);
            }
            break;
        }
        uint8_t n = MIN(len, count);
        if (len > count) {
            c.trailer_len = len - count;
            memcpy(c.trailer, &tbuf[count], c.trailer_len);
        }
        memcpy(ubuf, tbuf, n);
        count -= n;
        ubuf += n;
        total += n;
        c.token_ofs += n;
    }
    r.file_ofs += total;
    return total + header_total;
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
    if (!check_file_name(name)) {
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
bool AP_Filesystem_Param::check_file_name(const char *name)
{
    const uint8_t packed_len = strlen(PACKED_NAME);
    if (strncmp(name, PACKED_NAME, packed_len) == 0 &&
        (name[packed_len] == 0 || name[packed_len] == '?')) {
        return true;
    }
    return false;
}

/*
  support param upload
 */
int32_t AP_Filesystem_Param::write(int fd, const void *buf, uint32_t count)
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
    struct header hdr;
    if (r.file_ofs == 0 && count >= sizeof(hdr)) {
        // pre-expand the buffer to the full size when we get the header
        memcpy(&hdr, buf, sizeof(hdr));
        if (hdr.magic == pmagic) {
            const uint32_t flen = hdr.total_params;
            if (flen > r.writebuf->get_length()) {
                if (!r.writebuf->append(nullptr, flen - r.writebuf->get_length())) {
                    // not enough memory
                    return -1;
                }
            }
        }
    }
    if (r.file_ofs + count > r.writebuf->get_length()) {
        if (!r.writebuf->append(nullptr, r.file_ofs + count - r.writebuf->get_length())) {
            return -1;
        }
    }
    uint8_t *b = (uint8_t *)r.writebuf->get_writeable_string();
    memcpy(&b[r.file_ofs], buf, count);
    r.file_ofs += count;
    return count;
}

/*
  parse incoming parameters
 */
bool AP_Filesystem_Param::param_upload_parse(const rfile &r, bool &need_retry)
{
    need_retry = false;

    const uint8_t *b = (const uint8_t *)r.writebuf->get_string();
    uint32_t length = r.writebuf->get_length();
    struct header hdr;
    if (length < sizeof(hdr)) {
        return false;
    }
    memcpy(&hdr, b, sizeof(hdr));
    if (hdr.magic != pmagic) {
        return false;
    }
    if (length != hdr.total_params) {
        return false;
    }
    b += sizeof(hdr);

    char last_name[17] {};

    for (uint16_t i=0; i<hdr.num_params; i++) {
        enum ap_var_type ptype = (enum ap_var_type)(b[0]&0x0F);
        uint8_t flags = (enum ap_var_type)(b[0]>>4);
        if (flags != 0) {
            return false;
        }
        uint8_t common_len = b[1]&0xF;
        uint8_t name_len = (b[1]>>4)+1;
        if (common_len + name_len > 16) {
            return false;
        }
        char name[17];
        memcpy(name, last_name, common_len);
        memcpy(&name[common_len], &b[2], name_len);
        name[common_len+name_len] = 0;

        memcpy(last_name, name, sizeof(name));
        enum ap_var_type ptype2 = AP_PARAM_NONE;
        uint16_t flags2;

        b += 2 + name_len;

        AP_Param *p = AP_Param::find(name, &ptype2, &flags2);
        if (p == nullptr) {
            if (ptype == AP_PARAM_INT8) {
                b++;
            } else if (ptype == AP_PARAM_INT16) {
                b += 2;
            } else {
                b += 4;
            }
            continue;
        }

        /*
          if we are enabling a subsystem we need a small delay between
          parameters to allow main thread to perform any allocation of
          backends
        */
        bool need_delay = ((flags2 & AP_PARAM_FLAG_ENABLE) != 0 &&
                           ptype2 == AP_PARAM_INT8 &&
                           ((AP_Int8 *)p)->get() == 0);

        if (ptype == ptype2 && ptype == AP_PARAM_INT32) {
            // special handling of int32_t to preserve all bits
            int32_t v;
            memcpy(&v, b, sizeof(v));
            ((AP_Int32 *)p)->set(v);
            b += 4;
        } else if (ptype == AP_PARAM_INT8) {
            if (need_delay && b[0] == 0) {
                need_delay = false;
            }
            p->set_float((int8_t)b[0], ptype2);
            b += 1;
        } else if (ptype == AP_PARAM_INT16) {
            int16_t v;
            memcpy(&v, b, sizeof(v));
            p->set_float(float(v), ptype2);
            b += 2;
        } else if (ptype == AP_PARAM_INT32) {
            int32_t v;
            memcpy(&v, b, sizeof(v));
            p->set_float(float(v), ptype2);
            b += 4;
        } else if (ptype == AP_PARAM_FLOAT) {
            float v;
            memcpy(&v, b, sizeof(v));
            p->set_float(v, ptype2);
            b += 4;
        }

        p->save_sync(false, false);

        if (need_delay) {
            // let main thread have some time to init backends
            need_retry = true;
            hal.scheduler->delay(100);
        }
    }
    return true;
}


/*
  parse incoming parameters
 */
bool AP_Filesystem_Param::finish_upload(const rfile &r)
{
    uint8_t loops = 0;
    while (loops++ < 4) {
        bool need_retry;
        if (!param_upload_parse(r, need_retry)) {
            return false;
        }
        if (!need_retry) {
            break;
        }
    }
    return true;
}
