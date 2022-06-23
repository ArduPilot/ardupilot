#include "AP_OpenDroneID.h"

#if AP_OPENDRONEID_ENABLED

#include <AP_HAL/AP_HAL_Boards.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

const AP_Param::GroupInfo AP_OpenDroneID::var_info[] = {
    // @Param: SER_NUM
    // @DisplayName: Serial Number
    // @Description: Serial Number
    // @User: Advanced
    AP_GROUPINFO("SER_NUM", 1, AP_OpenDroneID, _serial_number, 0),

    AP_GROUPEND
};

// constructor
AP_OpenDroneID::AP_OpenDroneID()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("OpenDroneID must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_OpenDroneID::init()
{
    
}

void AP_OpenDroneID::update()
{
    
}

// singleton instance
AP_OpenDroneID *AP_OpenDroneID::_singleton;

namespace AP {

AP_OpenDroneID &opendroneid()
{
    return *AP_OpenDroneID::get_singleton();
}

}
#endif //AP_OPENDRONEID_ENABLED
