#include "DataFlashFileReader.h"

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

// flogged from AP_Hal_Linux/system.cpp; we don't want to use stopped clock here
uint64_t now() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)));
}

AP_LoggerFileReader::AP_LoggerFileReader() :
    start_micros(now())
{}

AP_LoggerFileReader::~AP_LoggerFileReader()
{
    const uint64_t micros = now();
    const uint64_t delta = micros - start_micros;
    ::printf("Replay counts: %" PRIu64 " bytes  %u entries\n", bytes_read, message_count);
    ::printf("Replay rates: %" PRIu64 " bytes/second  %" PRIu64 " messages/second\n", bytes_read*1000000/delta, message_count*1000000/delta);
}

bool AP_LoggerFileReader::open_log(const char *logfile)
{
    fd = ::open(logfile, O_RDONLY|O_CLOEXEC);
    if (fd == -1) {
        return false;
    }
    return true;
}

ssize_t AP_LoggerFileReader::read_input(void *buffer, const size_t count)
{
    uint64_t ret = ::read(fd, buffer, count);
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

bool AP_LoggerFileReader::update(char type[5])
{
    uint8_t hdr[3];
    if (read_input(hdr, 3) != 3) {
        return false;
    }
    if (hdr[0] != HEAD_BYTE1 || hdr[1] != HEAD_BYTE2) {
        printf("bad log header\n");
        return false;
    }

    packet_counts[hdr[2]]++;

    if (hdr[2] == LOG_FORMAT_MSG) {
        struct log_Format f;
        memcpy(&f, hdr, 3);
        if (read_input(&f.type, sizeof(f)-3) != sizeof(f)-3) {
            return false;
        }
        memcpy(&formats[f.type], &f, sizeof(formats[f.type]));
        strncpy(type, "FMT", 3);
        type[3] = 0;

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

    strncpy(type, f.name, 4);
    type[4] = 0;

    message_count++;
    return handle_msg(f,msg);
}
