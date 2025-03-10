/*
 * This source file implements all functions and variables necessary for creation of an AP_Vertiq object.
 * The AP_Vertiq object provides functions necessary to communicate with Vertiq modules.
*/

#include "AP_Vertiq.h"

extern const AP_HAL::HAL& hal;

AP_Vertiq *AP_Vertiq::_singleton;

const AP_Param::GroupInfo AP_Vertiq::var_info[] = {

    // @Param: CVS
    // @DisplayName: Number of Motors
    // @Description: Sets how many motors will be sent throttle commands starting from 0 and being contiguous
    // @Range: 0 16
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("CVS", 1, AP_Vertiq, _number_cvs, 0),

    // @Param: TEL_BM
    // @DisplayName: Module IDs whose telemetry will be requested
    // @Description: Bitmask with one set for each Module ID to request telemetry from
    // @Bitmask: 0: ID 0, 1: ID 1, 2: ID 2, 3: ID 3, 4: ID 4, 5: ID 5, 6: ID 6, 7: ID 7, 8: ID 8, 9: ID 9, 10: ID 10, 11: ID 11, 12: ID 12, 13: ID 13, 14: ID 14, 15: ID 15, 16: ID 16, 17: ID 17, 18: ID 18, 19: ID 19, 20: ID 20, 21: ID 21, 22: ID 22, 23: ID 23, 24: ID 24, 25: ID 25, 26: ID 26, 27: ID 27, 28: ID 28, 29: ID 29, 30: ID 30, 31: ID 31
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("TEL_BM", 2, AP_Vertiq, _telemetry_bitmask, 0),

    AP_GROUPEND
};

AP_Vertiq::AP_Vertiq() :
    _client_manager(),
    _iquart_manager(&_iquart_interface, &_client_manager),
    _output_manager(&_iquart_interface, &_client_manager)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Vertiq::init()
{
    _iquart_manager.initSerial();

    _output_manager.Init(_telemetry_bitmask.get(), _number_cvs.get());
}

void AP_Vertiq::update()
{

    //Only do this once, and make sure you escape after you init
    if (!_init_done) {
        init();

        _init_done = true;

        return;
    }

    //Update your outputs (Control Values and telemetry byte)
    _output_manager.Update();

    //Process all serial communications
    _iquart_manager.ProcessSerial();
}