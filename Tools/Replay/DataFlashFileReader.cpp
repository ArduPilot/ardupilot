#include "DataFlashFileReader.h"

#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>

bool DataFlashFileReader::open_log(const char *logfile)
{
    fd = ::open(logfile, O_RDONLY|O_CLOEXEC);
    if (fd == -1) {
        return false;
    }
    return true;
}

bool DataFlashFileReader::update(char type[5])
{
    uint8_t hdr[3];
    if (::read(fd, hdr, 3) != 3) {
        return false;
    }
    if (hdr[0] != HEAD_BYTE1 || hdr[1] != HEAD_BYTE2) {
        printf("bad log header\n");
        return false;
    }

    if (hdr[2] == LOG_FORMAT_MSG) {
        struct log_Format f;
        memcpy(&f, hdr, 3);
        if (::read(fd, &f.type, sizeof(f)-3) != sizeof(f)-3) {
            return false;
        }
        memcpy(&formats[f.type], &f, sizeof(formats[f.type]));
        strncpy(type, "FMT", 3);
        type[3] = 0;

        return handle_log_format_msg(f);
    }

    if (!done_format_msgs) {
        done_format_msgs = true;
        end_format_msgs();
    }

    const struct log_Format &f = formats[hdr[2]];
    if (f.length == 0) {
        // can't just throw these away as the format specifies the
        // number of bytes in the message
        ::printf("No format defined for type (%d)\n", hdr[2]);
        exit(1);
    }

    uint8_t msg[f.length];

    memcpy(msg, hdr, 3);
    if (::read(fd, &msg[3], f.length-3) != f.length-3) {
        return false;
    }

    strncpy(type, f.name, 4);
    type[4] = 0;

    return handle_msg(f,msg);
}
