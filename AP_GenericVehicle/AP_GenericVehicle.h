#pragma once

#include <AP_Vehicle/AP_Vehicle.h>

#include "Parameters.h"
#include "GCS_GenericVehicle.h"

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

    void init_ardupilot() override;

#if AP_SCHEDULER_ENABLED
    static const AP_Scheduler::Task scheduler_tasks[];
    uint32_t log_bitx;
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks, uint8_t &task_count, uint32_t &log_bit) override;
#endif

#if HAL_GCS_ENABLED
    // GCS selection
    GCS_GenericVehicle _gcs; // avoid using this; use gcs()
    GCS_GenericVehicle &gcs() { return _gcs; }
#endif

};

extern AP_GenericVehicle genericvehicle;
