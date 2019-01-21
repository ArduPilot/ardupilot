#pragma once
#include <AP_HAL/AP_HAL.h>

#if (HAL_OS_POSIX_IO || HAL_OS_FATFS_IO)

#define HAS_UAVCAN_SERVERS

#include <uavcan/uavcan.hpp>

//Forward declaring classes
class AP_UAVCAN_FileEventTracer;
class AP_UAVCAN_FileStorageBackend;
class AP_UAVCAN_CentralizedServer;

class AP_UAVCAN_Servers
{
public:
    bool init(uavcan::Node<0> &node);

private:
    void reset();

    AP_UAVCAN_CentralizedServer *_server_instance;
    AP_UAVCAN_FileEventTracer *_tracer;
    AP_UAVCAN_FileStorageBackend *_storage_backend;
};

#endif
