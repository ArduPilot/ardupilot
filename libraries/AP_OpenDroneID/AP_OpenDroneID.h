#pragma once

#ifndef AP_OPENDRONEID_ENABLED
#define AP_OPENDRONEID_ENABLED 1
#endif

#include <AP_Param/AP_Param.h>

class AP_OpenDroneID {
public:
    AP_OpenDroneID();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_OpenDroneID);

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();


    // get methods
    

    // get singleton instance
    static AP_OpenDroneID *get_singleton() { return _singleton; }

private:
    static AP_OpenDroneID *_singleton;

    // parameters
    AP_Int32 _serial_number;

    uint32_t _last_send_ms;
};

namespace AP {
    AP_OpenDroneID &opendroneid();
};
