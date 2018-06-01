#ifndef REPLAY_DATAFLASHREADER_H
#define REPLAY_DATAFLASHREADER_H

#include <DataFlash/DataFlash.h>

class DataFlashFileReader
{
public:
    bool open_log(const char *logfile);
    bool update(char type[5]);

    virtual bool handle_log_format_msg(const struct log_Format &f) = 0;
    virtual bool handle_msg(const struct log_Format &f, uint8_t *msg) = 0;

protected:
    int fd = -1;
    bool done_format_msgs = false;
    virtual void end_format_msgs(void) {}

#define LOGREADER_MAX_FORMATS 255 // must be >= highest MESSAGE
    struct log_Format formats[LOGREADER_MAX_FORMATS] {};
};

#endif
