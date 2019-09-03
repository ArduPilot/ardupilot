/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Siddharth Bharat Purohit
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_UAVCAN_Servers.h"

#ifdef HAS_UAVCAN_SERVERS

#include <uavcan/protocol/dynamic_node_id_server/event.hpp>
#include <uavcan/protocol/dynamic_node_id_server/storage_backend.hpp>
#include <uavcan/protocol/dynamic_node_id_server/centralized.hpp>
#include <uavcan/protocol/HardwareVersion.hpp>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#include <AP_Common/AP_Common.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Filesystem/AP_Filesystem.h>

#ifndef UAVCAN_NODE_DB_PATH
#define UAVCAN_NODE_DB_PATH HAL_BOARD_STORAGE_DIRECTORY "/UAVCAN"
#endif

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define debug_uavcan(fmt, args...) do { hal.console->printf(fmt, ##args); } while (0)

class AP_UAVCAN_CentralizedServer : public uavcan::dynamic_node_id_server::CentralizedServer
{
public:
    AP_UAVCAN_CentralizedServer(uavcan::INode& node, uavcan::dynamic_node_id_server::IStorageBackend &storage_backend, uavcan::dynamic_node_id_server::IEventTracer &tracer) :
        uavcan::dynamic_node_id_server::CentralizedServer(node, storage_backend, tracer) {}
};

class AP_UAVCAN_FileEventTracer : public uavcan::dynamic_node_id_server::IEventTracer
{
protected:
    virtual void onEvent(uavcan::dynamic_node_id_server::TraceCode code, uavcan::int64_t argument) override
    {
        AP::logger().Write("UCEV", "TimeUS,code,arg", "s--", "F--", "Qhq", AP_HAL::micros64(), code, argument);
    }
};


class AP_UAVCAN_RestartRequestHandler : public uavcan::IRestartRequestHandler {
public:
    bool handleRestartRequest(uavcan::NodeID request_source) override {
        // swiped from reboot handling in GCS_Common.cpp
        if (hal.util->get_soft_armed()) {
            // refuse reboot when armed
            return false;
        }
        AP_Notify *notify = AP_Notify::get_singleton();
        if (notify) {
            AP_Notify::flags.firmware_update = 1;
            notify->update();
        }
        // force safety on
        hal.rcout->force_safety_on();

        // flush pending parameter writes
        AP_Param::flush();

        hal.scheduler->delay(200);
        hal.scheduler->reboot(false);
        return true;
    }
};

class AP_UAVCAN_FileStorageBackend : public uavcan::dynamic_node_id_server::IStorageBackend
{
    /**
     * Maximum length of full path including / and key max
     */
    enum { MaxPathLength = 128 };

    enum { MaxNumOpens = 100 };
    /**
     * This type is used for the path
     */
    typedef uavcan::MakeString<MaxPathLength>::Type PathString;

    PathString base_path;

    static uint8_t num_opens;
protected:
    virtual String get(const String& key) const override
    {
        using namespace std;
        PathString path = base_path.c_str();
        path += key;
        String value;
        //This is to deter frequent inflight opening and closing of files during an event
        //where the device is misbehaving
        if (num_opens >= MaxNumOpens) {
            return value;
        }
        num_opens++;
        int fd = AP::FS().open(path.c_str(), O_RDONLY);
        if (fd >= 0)
        {
            char buffer[MaxStringLength + 1];
            (void)memset(buffer, 0, sizeof(buffer));
            ssize_t remaining = MaxStringLength;
            ssize_t total_read = 0;
            ssize_t nread = 0;
            do
            {
                nread = AP::FS().read(fd, &buffer[total_read], remaining);
                if (nread > 0)
                {
                    remaining -= nread,
                    total_read += nread;
                }
            }
            while (nread > 0 && remaining > 0);
            AP::FS().close(fd);
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

    virtual void set(const String& key, const String& value) override
    {
        using namespace std;
        PathString path = base_path.c_str();
        path += key;
        //This is to deter frequent inflight opening and closing of files during an event
        //where the device is misbehaving
        if (num_opens >= MaxNumOpens) {
            return;
        }
        num_opens++;
        int fd = AP::FS().open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC);
        if (fd >= 0)
        {
            ssize_t remaining = value.size();
            ssize_t total_written = 0;
            ssize_t written = 0;
            do
            {
                written = AP::FS().write(fd, &value.c_str()[total_written], remaining);
                if (written > 0)
                {
                    total_written += written;
                    remaining -=  written;
                }
            }
            while (written > 0 && remaining > 0);

            AP::FS().fsync(fd);
            AP::FS().close(fd);
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
            if (AP::FS().stat(base_path.c_str(), &sb) != 0 || !S_ISDIR(sb.st_mode))
            {
                rv = AP::FS().mkdir(base_path.c_str());
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
uint8_t AP_UAVCAN_FileStorageBackend::num_opens = 0;

bool AP_UAVCAN_Servers::init(uavcan::Node<0> &node)
{
    if (_server_instance != nullptr) {
        return true;
    }

    int ret = 0;

    _storage_backend = new AP_UAVCAN_FileStorageBackend;
    if (_storage_backend == nullptr) {
        debug_uavcan("UAVCAN_Servers: Failed to Allocate FileStorageBackend\n"); 
        goto failed;
    }

    ret = _storage_backend->init(UAVCAN_NODE_DB_PATH);
    if (ret < 0) {
        debug_uavcan("UAVCAN_Servers: FileStorageBackend init: %d, errno: %d\n", ret, errno);
        goto failed;
    }

    _tracer = new AP_UAVCAN_FileEventTracer;
    if (_tracer == nullptr) {
        debug_uavcan("UAVCAN_Servers: Failed to Allocate FileEventTracer\n"); 
        goto failed;
    }

    _server_instance = new AP_UAVCAN_CentralizedServer(node, *_storage_backend, *_tracer);
    if (_server_instance == nullptr) {
        debug_uavcan("UAVCAN_Servers: Failed to Allocate Server\n"); 
        goto failed;
    }

    {
        uavcan::dynamic_node_id_server::centralized::Storage storage(*_storage_backend);
        if (storage.getNodeIDForUniqueID(node.getHardwareVersion().unique_id) != node.getNodeID()) {
            //Node ID was changed, reseting database
            reset();
        }
    }

    if (_restart_request_handler == nullptr) {
        _restart_request_handler = new AP_UAVCAN_RestartRequestHandler();
        if (_restart_request_handler == nullptr) {
            goto failed;
        }
    }
    node.setRestartRequestHandler(_restart_request_handler);

    //Start Dynamic Node Server
    ret = _server_instance->init(node.getHardwareVersion().unique_id);
    if (ret < 0) {
        debug_uavcan("UAVCAN_Server init: %d, errno: %d\n", ret, errno); 
        goto failed;
    }

    return true;

failed:
    delete _restart_request_handler;
    delete _storage_backend;
    delete _tracer;
    delete _server_instance;
    return false;
}

void AP_UAVCAN_Servers::reset()
{
    debug_uavcan("UAVCAN_Servers: Resetting Server Database...\n");
    DIR* dp;
    struct dirent* ep;
    dp = AP::FS().opendir(UAVCAN_NODE_DB_PATH);
    char abs_filename[100];
    if (dp != NULL)
    {
        while((ep = AP::FS().readdir(dp))) {
            snprintf(abs_filename, 100, "%s/%s", UAVCAN_NODE_DB_PATH, ep->d_name);
            AP::FS().unlink(abs_filename);
        }
    }
    AP::FS().closedir(dp);
}

#endif

#endif //HAL_WITH_UAVCAN
