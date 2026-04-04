#pragma once

#include <AP_Logger/AP_Logger.h>

#define LOGREADER_MAX_FORMATS 255 // must be >= highest MESSAGE

class AP_LoggerFileReader
{
public:

    AP_LoggerFileReader();
    ~AP_LoggerFileReader();

    bool open_log(const char *logfile);
    bool update();

    virtual bool handle_log_format_msg(const struct log_Format &f) = 0;
    virtual bool handle_msg(const struct log_Format &f, uint8_t *msg) = 0;

    void format_type(uint16_t type, char dest[5]);
    void get_packet_counts(uint64_t dest[]);
    float get_percent_read(); // Get percentage of log file read

protected:
    int fd = -1;

    struct log_Format formats[LOGREADER_MAX_FORMATS] {};

private:
    ssize_t read_input(void *buf, size_t count);

    uint64_t bytes_read = 0;
    uint64_t file_size = 0; // Total size of the log file
    uint32_t message_count = 0;
    uint64_t start_micros;

    uint64_t packet_counts[LOGREADER_MAX_FORMATS] = {};
};
