#pragma once

#include <AP_Vehicle/AP_Vehicle.h>

#include "Parameters.h"

class AP_GenericVehicle : public AP_Vehicle {
public:
    AP_GenericVehicle();

    Parameters g;
    static const AP_Param::Info var_info[];
    AP_Param param_loader {var_info};

    void load_parameters() override;

    // begin stuff it would be nice not to have in the build:

    // set_mode *must* set control_mode_reason
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override {
        return false;
    }
    uint8_t get_mode() const override { return 0; }


};

extern AP_GenericVehicle genericvehicle;
