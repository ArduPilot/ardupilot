#include "DataFlashFileReader.h"
#include <AP_Filesystem/AP_Filesystem.h>

#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <cinttypes>

#ifndef PRIu64
#define PRIu64 "llu"
#endif

AP_LoggerFileReader::AP_LoggerFileReader()
{}

AP_LoggerFileReader::~AP_LoggerFileReader()
{
    ::printf("Replay counts: %" PRIu64 " bytes  %u entries\n", bytes_read, message_count);
}

bool AP_LoggerFileReader::open_log(const char *logfile)
{
    fd = AP::FS().open(logfile, O_RDONLY);
    if (fd == -1) {
        return false;
    }
    return true;
}

ssize_t AP_LoggerFileReader::read_input(void *buffer, const size_t count)
{
    uint64_t ret = AP::FS().read(fd, buffer, count);
    bytes_read += ret;
    return ret;
}

void AP_LoggerFileReader::format_type(uint16_t type, char dest[5])
{
    const struct log_Format &f = formats[type];
    memset(dest,0,5);
    if (f.length == 0) {
        return;
    }
    strncpy(dest, f.name, 4);
}
void AP_LoggerFileReader::get_packet_counts(uint64_t dest[])
{
    memcpy(dest, packet_counts, sizeof(packet_counts));
}

bool AP_LoggerFileReader::update()
{
    uint8_t hdr[3];
    if (read_input(hdr, 3) != 3) {
        return false;
    }
    if (hdr[0] != HEAD_BYTE1 || hdr[1] != HEAD_BYTE2) {
        printf("bad log header\n");
        return false;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    // running on stm32 is slow enough it is nice to see progress
    if (message_count % 500 == 0) {
        ::printf("line %u pkt 0x%02x t=%u\n", message_count, hdr[2], AP_HAL::millis());
    }
#endif
    packet_counts[hdr[2]]++;

    if (hdr[2] == LOG_FORMAT_MSG) {
        struct log_Format f;
        memcpy(&f, hdr, 3);
        if (read_input(&f.type, sizeof(f)-3) != sizeof(f)-3) {
            return false;
        }
        memcpy(&formats[f.type], &f, sizeof(formats[f.type]));

        message_count++;
        return handle_log_format_msg(f);
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
    if (read_input(&msg[3], f.length-3) != f.length-3) {
        return false;
    }

    message_count++;
    return handle_msg(f, msg);
}
