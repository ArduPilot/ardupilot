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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_RangeFinder_Params.h"

// Maximum number of range finder instances available on this platform
#ifndef RANGEFINDER_MAX_INSTANCES
#define RANGEFINDER_MAX_INSTANCES 10
#endif

#define RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT 10
#define RANGEFINDER_PREARM_ALT_MAX_CM           200
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define RANGEFINDER_PREARM_REQUIRED_CHANGE_CM   0
#else
#define RANGEFINDER_PREARM_REQUIRED_CHANGE_CM   50
#endif

class AP_RangeFinder_Backend;

class RangeFinder
{
    friend class AP_RangeFinder_Backend;
    //UAVCAN drivers are initialised in the Backend, hence list of drivers is needed there.
    friend class AP_RangeFinder_UAVCAN;
public:
    RangeFinder();

    /* Do not allow copies */
    RangeFinder(const RangeFinder &other) = delete;
    RangeFinder &operator=(const RangeFinder&) = delete;

    // RangeFinder driver types
    enum class Type {
        NONE   = 0,
        ANALOG = 1,
        MBI2C  = 2,
        PLI2C  = 3,
//        PX4    = 4, // no longer used, but may be in some user's parameters
        PX4_PWM= 5,
        BBB_PRU= 6,
        LWI2C  = 7,
        LWSER  = 8,
        BEBOP  = 9,
        MAVLink = 10,
        ULANDING= 11,
        LEDDARONE = 12,
        MBSER  = 13,
        TRI2C  = 14,
        PLI2CV3= 15,
        VL53L0X = 16,
        NMEA = 17,
        WASP = 18,
        BenewakeTF02 = 19,
        BenewakeTFmini = 20,
        PLI2CV3HP = 21,
        PWM = 22,
        BLPing = 23,
        UAVCAN = 24,
        BenewakeTFminiPlus = 25,
        Lanbao = 26,
        BenewakeTF03 = 27,
        VL53L1X_Short = 28,
        LeddarVu8_Serial = 29,
    };

    enum class Function {
        LINEAR    = 0,
        INVERTED  = 1,
        HYPERBOLA = 2
    };

    enum class Status {
        NotConnected = 0,
        NoData,
        OutOfRangeLow,
        OutOfRangeHigh,
        Good
    };

    // The RangeFinder_State structure is filled in by the backend driver
    struct RangeFinder_State {
        uint16_t distance_cm;           // distance: in cm
        uint16_t voltage_mv;            // voltage in millivolts, if applicable, otherwise 0
        enum RangeFinder::Status status; // sensor status
        uint8_t  range_valid_count;     // number of consecutive valid readings (maxes out at 10)
        uint32_t last_reading_ms;       // system time of last successful update from sensor

        const struct AP_Param::GroupInfo *var_info;
    };

    static const struct AP_Param::GroupInfo *backend_var_info[RANGEFINDER_MAX_INSTANCES];

    // parameters for each instance
    static const struct AP_Param::GroupInfo var_info[];

    void set_log_rfnd_bit(uint32_t log_rfnd_bit) { _log_rfnd_bit = log_rfnd_bit; }

    // Return the number of range finder instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // prearm checks
    bool prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const;

    // detect and initialise any available rangefinders
    void init(enum Rotation orientation_default);

    // update state of all rangefinders. Should be called at around
    // 10Hz from main loop
    void update(void);

    // Handle an incoming DISTANCE_SENSOR message (from a MAVLink enabled range finder)
    void handle_msg(const mavlink_message_t &msg);

    // return true if we have a range finder with the specified orientation
    bool has_orientation(enum Rotation orientation) const;

    // find first range finder instance with the specified orientation
    AP_RangeFinder_Backend *find_instance(enum Rotation orientation) const;

    AP_RangeFinder_Backend *get_backend(uint8_t id) const;

    // get rangefinder type for an ID
    Type get_type(uint8_t id) const {
        return id >= RANGEFINDER_MAX_INSTANCES? Type::NONE : Type(params[id].type.get());
    }

    // methods to return a distance on a particular orientation from
    // any sensor which can current supply it
    uint16_t distance_cm_orient(enum Rotation orientation) const;
    uint16_t voltage_mv_orient(enum Rotation orientation) const;
    int16_t max_distance_cm_orient(enum Rotation orientation) const;
    int16_t min_distance_cm_orient(enum Rotation orientation) const;
    int16_t ground_clearance_cm_orient(enum Rotation orientation) const;
    MAV_DISTANCE_SENSOR get_mav_distance_sensor_type_orient(enum Rotation orientation) const;
    RangeFinder::Status status_orient(enum Rotation orientation) const;
    bool has_data_orient(enum Rotation orientation) const;
    uint8_t range_valid_count_orient(enum Rotation orientation) const;
    const Vector3f &get_pos_offset_orient(enum Rotation orientation) const;
    uint32_t last_reading_ms(enum Rotation orientation) const;

    // indicate which bit in LOG_BITMASK indicates RFND should be logged
    void set_rfnd_bit(uint32_t log_rfnd_bit) { _log_rfnd_bit = log_rfnd_bit; }

    /*
      set an externally estimated terrain height. Used to enable power
      saving (where available) at high altitudes.
     */
    void set_estimated_terrain_height(float height) {
        estimated_terrain_height = height;
    }

    static RangeFinder *get_singleton(void) { return _singleton; }

protected:
    AP_RangeFinder_Params params[RANGEFINDER_MAX_INSTANCES];

private:
    static RangeFinder *_singleton;

    RangeFinder_State state[RANGEFINDER_MAX_INSTANCES];
    AP_RangeFinder_Backend *drivers[RANGEFINDER_MAX_INSTANCES];
    uint8_t num_instances;
    float estimated_terrain_height;
    Vector3f pos_offset_zero;   // allows returning position offsets of zero for invalid requests

    void convert_params(void);

    void detect_instance(uint8_t instance, uint8_t& serial_instance);

    bool _add_backend(AP_RangeFinder_Backend *driver);

    uint32_t _log_rfnd_bit = -1;
    void Log_RFND();
};

namespace AP {
    RangeFinder *rangefinder();
};
