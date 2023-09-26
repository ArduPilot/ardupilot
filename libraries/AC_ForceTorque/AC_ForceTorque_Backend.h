#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AC_ForceTorque.h"

class AC_ForceTorque_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AC_ForceTorque_Backend(ForceTorque::ForceTorque_State &_state, AC_ForceTorque_Params &_params);

    // we declare a virtual destructor so that ForceTorque drivers can
    // override with a custom destructor if need be
    virtual ~AC_ForceTorque_Backend(void) {}

    // update the state structure
    virtual void update() = 0;
    virtual void init_serial(uint8_t serial_instance) {};

    enum InstallLocation location() const { return (InstallLocation)params.location.get(); }
    float get_force_x_N() const { return state.force_N.x; }
    float get_force_y_N() const { return state.force_N.y; }
    float get_force_z_N() const { return state.force_N.z; }
    float get_torque_x_Nm() const { return state.roll_deg; }
    float get_torque_y_Nm() const { return state.roll_deg; }
    float get_torque_z_Nm() const { return state.roll_deg; }

    float get_max_force_N() const {return params.max_force_N; }
    float get_min_force_N() const {return params.min_force_N; }
    float get_max_torque_Nm() const {return params.max_torque_Nm; }
    float get_min_torque_Nm() const {return params.min_torque_Nm; }

    ForceTorque::Status status() const;
    ForceTorque::Type type() const { return (ForceTorque::Type)params.type.get(); }

    // true if sensor is returning data
    bool has_data() const;

    // return system time of last successful read from the sensor
    uint32_t last_reading_ms() const { return state.last_reading_ms; }

protected:

    // update status based on angle measurement
    void update_status();

    // set status and update valid_count
    void set_status(ForceTorque::Status status);

    ForceTorque::ForceTorque_State &state;
    AC_ForceTorque_Params &params;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    //Type Backend initialised with
    ForceTorque::Type _backend_type;
};