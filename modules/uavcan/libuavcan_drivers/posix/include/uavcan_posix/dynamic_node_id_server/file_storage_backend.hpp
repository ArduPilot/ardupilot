/****************************************************************************
*
*   Copyright (c) 2015 PX4 Development Team. All rights reserved.
*      Author: Pavel Kirienko <pavel.kirienko@gmail.com>
*              David Sidrane <david_s5@usa.net>
*
****************************************************************************/

#ifndef UAVCAN_POSIX_DYNAMIC_NODE_ID_SERVER_FILE_STORAGE_BACKEND_HPP_INCLUDED
#define UAVCAN_POSIX_DYNAMIC_NODE_ID_SERVER_FILE_STORAGE_BACKEND_HPP_INCLUDED

#include <sys/stat.h>
#include <cstdio>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>

#include <uavcan/protocol/dynamic_node_id_server/storage_backend.hpp>

namespace uavcan_posix
{
namespace dynamic_node_id_server
{
/**
 * This interface implements a POSIX compliant IStorageBackend interface
 */
class FileStorageBackend : public uavcan::dynamic_node_id_server::IStorageBackend
{
    /**
     * Maximum length of full path including / and key max
     */
    enum { MaxPathLength = 128 };

    enum { FilePermissions = 438 };     ///< 0o666

    /**
     * This type is used for the path
     */
    typedef uavcan::MakeString<MaxPathLength>::Type PathString;

    PathString base_path;

protected:
    virtual String get(const String& key) const
    {
        using namespace std;
        PathString path = base_path.c_str();
        path += key;
        String value;
        int fd = open(path.c_str(), O_RDONLY);
        if (fd >= 0)
        {
            char buffer[MaxStringLength + 1];
            (void)memset(buffer, 0, sizeof(buffer));
            ssize_t remaining = MaxStringLength;
            ssize_t total_read = 0;
            ssize_t nread = 0;
            do
            {
                nread = ::read(fd, &buffer[total_read], remaining);
                if (nread > 0)
                {
                    remaining -= nread,
                    total_read += nread;
                }
            }
            while (nread > 0 && remaining > 0);
            (void)close(fd);
            if (total_read > 0)
            {
                for (int i = 0; i < total_read; i++)
                {
                    if (buffer[i] == ' ' || buffer[i] == '\n' || buffer[i] == '\r')
                    {
                        buffer[i] = '\0';
                        break;
                    }
                }
                value = buffer;
            }
        }
        return value;
    }

    virtual void set(const String& key, const String& value)
    {
        using namespace std;
        PathString path = base_path.c_str();
        path += key;
        int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, FilePermissions);
        if (fd >= 0)
        {
            ssize_t remaining = value.size();
            ssize_t total_written = 0;
            ssize_t written = 0;
            do
            {
                written = write(fd, &value.c_str()[total_written], remaining);
                if (written > 0)
                {
                    total_written += written;
                    remaining -=  written;
                }
            }
            while (written > 0 && remaining > 0);

            (void)fsync(fd);
            (void)close(fd);
        }
    }

public:
    /**
     * Initializes the file based backend storage by passing a path to
     * the directory where the key named files will be stored.
     * The return value should be 0 on success.
     * If it is -ErrInvalidConfiguration then the the path name is too long to
     * accommodate the trailing slash and max key length.
     */
    int init(const PathString& path)
    {
        using namespace std;

        int rv = -uavcan::ErrInvalidParam;

        if (path.size() > 0)
        {
            base_path = path.c_str();

            if (base_path.back() == '/')
            {
                base_path.pop_back();
            }

            rv = 0;
            struct stat sb;
            if (stat(base_path.c_str(), &sb) != 0 || !S_ISDIR(sb.st_mode))
            {
                // coverity[toctou]
                rv = mkdir(base_path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
            }
            if (rv >= 0)
            {
                base_path.push_back('/');
                if ((base_path.size() + MaxStringLength) > MaxPathLength)
                {
                    rv = -uavcan::ErrInvalidConfiguration;
                }
            }
        }
        return rv;
    }
};
}
}

#endif // Include guard
