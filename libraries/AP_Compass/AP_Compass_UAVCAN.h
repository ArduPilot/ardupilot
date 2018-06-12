#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

class AP_Compass_UAVCAN : public AP_Compass_Backend {
public:
    void        read(void) override;

    AP_Compass_UAVCAN(Compass &compass);
    ~AP_Compass_UAVCAN() override;

    static AP_Compass_Backend *probe(Compass &compass);

    bool register_uavcan_compass(uint8_t mgr, uint8_t node);

    // This method is called from UAVCAN thread
    void handle_mag_msg(Vector3f &mag);

private:
    uint8_t  _instance;
    int      _mag_fd;
    Vector3f _sum;
    uint32_t _count;

    bool _initialized;
    uint8_t _manager;

    AP_HAL::Semaphore *_sem_mag;
};
