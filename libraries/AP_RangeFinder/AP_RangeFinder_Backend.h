/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include "AP_RangeFinder.h"

class AP_RangeFinder_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_RangeFinder_Backend(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // we declare a virtual destructor so that RangeFinder drivers can
    // override with a custom destructor if need be
    virtual ~AP_RangeFinder_Backend(void) {}

    // update the state structure
    virtual void update() = 0;
    virtual void init_serial(uint8_t serial_instance) {};

    virtual void handle_msg(const mavlink_message_t &msg) { return; }

#if AP_SCRIPTING_ENABLED
    void get_state(RangeFinder::RangeFinder_State &state_arg);

    // Returns false if scripting backing hasn't been setup.
    virtual bool handle_script_msg(float dist_m) { return false; } // legacy interface
    virtual bool handle_script_msg(const RangeFinder::RangeFinder_State &state_arg) { return false; }
#endif

#if HAL_MSP_RANGEFINDER_ENABLED
    virtual void handle_msp(const MSP::msp_rangefinder_data_message_t &pkt) { return; }
#endif

    enum Rotation orientation() const { return (Rotation)params.orientation.get(); }
    float distance() const { return state.distance_m; }
    int8_t signal_quality_pct() const  WARN_IF_UNUSED { return state.signal_quality_pct; }
    uint16_t voltage_mv() const { return state.voltage_mv; }
    virtual float max_distance() const { return params.max_distance; }
    virtual float min_distance() const { return params.min_distance; }
    float ground_clearance() const { return params.ground_clearance; }
    MAV_DISTANCE_SENSOR get_mav_distance_sensor_type() const;
    RangeFinder::Status status() const;
    RangeFinder::Status status(float distance_m) const;
    RangeFinder::Type type() const { return (RangeFinder::Type)params.type.get(); }

    // true if sensor is returning data
    bool has_data() const;

    // returns count of consecutive good readings
    uint8_t range_valid_count() const { return state.range_valid_count; }

    // return a 3D vector defining the position offset of the sensor
    // in metres relative to the body frame origin
    const Vector3f &get_pos_offset() const { return params.pos_offset; }

    // return system time of last successful read from the sensor
    uint32_t last_reading_ms() const { return state.last_reading_ms; }

#if AP_AHRS_ENABLED
    // return vehicle location at last successful read from the sensor
    Location last_reading_loc() const { return state.last_reading_loc; }
#endif

    // return sample history of the sensor
    RangeFinder::RangeFinder_State::RangeFinder_Sample *sample_history() const
    {
        return state.sample_history;
    }

    // return number of samples in the sample history of the sensor
    uint8_t sample_history_size() const { return state.sample_history_size; }

    // return index of the last captured sample in the history of the sensor
    uint8_t last_sample_history_index() const { return state.last_sample_history_index; }

    // get temperature reading in C.  returns true on success and populates temp argument
    virtual bool get_temp(float &temp) const { return false; }

    // return the actual type of the rangefinder, as opposed to the
    // parameter value which may be changed at runtime.
    RangeFinder::Type allocated_type() const { return _backend_type; }

protected:

    // update status based on distance measurement
    void update_status(RangeFinder::RangeFinder_State &state_arg) const;
    void update_status() { update_status(state); }

    // update sample history
    void update_history(RangeFinder::RangeFinder_State &state_arg) const;
    void update_history() { update_history(state); }

    // set status and update valid_count
    static void set_status(RangeFinder::RangeFinder_State &state_arg, RangeFinder::Status status);
    void set_status(RangeFinder::Status status) { set_status(state, status); }

    RangeFinder::RangeFinder_State &state;
    AP_RangeFinder_Params &params;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    //Type Backend initialised with
    RangeFinder::Type _backend_type;

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const = 0;

private:
    enum RangeFinder::Status _get_status_for_distance(float distance_m) const;

#if AP_AHRS_ENABLED
    void _calc_rngfnd_attitude_deviation_rad(float &orient_error_rad) const;
#endif
};

#endif  // AP_RANGEFINDER_ENABLED
