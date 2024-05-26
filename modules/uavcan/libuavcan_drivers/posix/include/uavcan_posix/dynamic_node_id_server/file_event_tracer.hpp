/****************************************************************************
*
*   Copyright (c) 2015 PX4 Development Team. All rights reserved.
*      Author: Pavel Kirienko <pavel.kirienko@gmail.com>
*              David Sidrane <david_s5@usa.net>
*
****************************************************************************/

#ifndef UAVCAN_POSIX_DYNAMIC_NODE_ID_SERVER_FILE_EVENT_TRACER_HPP_INCLUDED
#define UAVCAN_POSIX_DYNAMIC_NODE_ID_SERVER_FILE_EVENT_TRACER_HPP_INCLUDED

#include <uavcan/protocol/dynamic_node_id_server/event.hpp>
#include <cstdio>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>

namespace uavcan_posix
{
namespace dynamic_node_id_server
{
/**
 * This interface implements a POSIX compliant file based IEventTracer interface
 */
class FileEventTracer : public uavcan::dynamic_node_id_server::IEventTracer
{
    /**
     * Maximum length of full path to log file
     */
    enum { MaxPathLength = 128 };

    enum { FilePermissions = 438 };     ///< 0o666

    /**
     * This type is used for the path
     */
    typedef uavcan::MakeString<MaxPathLength>::Type PathString;

    PathString path_;

protected:
    virtual void onEvent(uavcan::dynamic_node_id_server::TraceCode code, uavcan::int64_t argument)
    {
        using namespace std;

        timespec ts = timespec();               // If clock_gettime() fails, zero time will be used
        (void)clock_gettime(CLOCK_REALTIME, &ts);

        int fd = open(path_.c_str(), O_WRONLY | O_CREAT | O_APPEND, FilePermissions);
        if (fd >= 0)
        {
            const int FormatBufferLength = 63;
            char buffer[FormatBufferLength + 1];
            ssize_t remaining = snprintf(buffer, FormatBufferLength, "%ld.%06ld\t%d\t%lld\n",
                                         static_cast<long>(ts.tv_sec), static_cast<long>(ts.tv_nsec / 1000L),
                                         static_cast<int>(code), static_cast<long long>(argument));

            ssize_t total_written = 0;
            ssize_t written = 0;
            do
            {
                written = write(fd, &buffer[total_written], remaining);
                if (written > 0)
                {
                    total_written += written;
                    remaining -=  written;
                }
            }
            while (written > 0 && remaining > 0);
            (void)close(fd);
        }
    }

public:
    /**
     * Initializes the file based event tracer.
     */
    int init(const PathString& path)
    {
        using namespace std;

        int rv = -uavcan::ErrInvalidParam;

        if (path.size() > 0)
        {
            rv = 0;
            path_ = path.c_str();
            int fd = open(path_.c_str(), O_RDWR | O_CREAT | O_TRUNC, FilePermissions);
            if (fd >= 0)
            {
                (void)close(fd);
            }
        }
        return rv;
    }
};
}
}

#endif // Include guard
