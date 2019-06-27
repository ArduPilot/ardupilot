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
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_RangeFinder_Params.h"

// Maximum number of range finder instances available on this platform
#define RANGEFINDER_MAX_INSTANCES 10
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
    RangeFinder(AP_SerialManager &_serial_manager);

    /* Do not allow copies */
    RangeFinder(const RangeFinder &other) = delete;
    RangeFinder &operator=(const RangeFinder&) = delete;

    // RangeFinder driver types
    enum RangeFinder_Type {
        RangeFinder_TYPE_NONE   = 0,
        RangeFinder_TYPE_ANALOG = 1,
        RangeFinder_TYPE_MBI2C  = 2,
        RangeFinder_TYPE_PLI2C  = 3,
        RangeFinder_TYPE_PX4    = 4,
        RangeFinder_TYPE_PX4_PWM= 5,
        RangeFinder_TYPE_BBB_PRU= 6,
        RangeFinder_TYPE_LWI2C  = 7,
        RangeFinder_TYPE_LWSER  = 8,
        RangeFinder_TYPE_BEBOP  = 9,
        RangeFinder_TYPE_MAVLink = 10,
        RangeFinder_TYPE_ULANDING= 11,
        RangeFinder_TYPE_LEDDARONE = 12,
        RangeFinder_TYPE_MBSER  = 13,
        RangeFinder_TYPE_TRI2C  = 14,
        RangeFinder_TYPE_PLI2CV3= 15,
        RangeFinder_TYPE_VL53L0X = 16,
        RangeFinder_TYPE_NMEA = 17,
        RangeFinder_TYPE_WASP = 18,
        RangeFinder_TYPE_BenewakeTF02 = 19,
        RangeFinder_TYPE_BenewakeTFmini = 20,
        RangeFinder_TYPE_PLI2CV3HP = 21,
        RangeFinder_TYPE_PWM = 22,
        RangeFinder_TYPE_BLPing = 23,
        RangeFinder_TYPE_UAVCAN = 24,
        RangeFinder_TYPE_BenewakeTFminiPlus = 25,
    };

    enum RangeFinder_Function {
        FUNCTION_LINEAR    = 0,
        FUNCTION_INVERTED  = 1,
        FUNCTION_HYPERBOLA = 2
    };

    enum RangeFinder_Status {
        RangeFinder_NotConnected = 0,
        RangeFinder_NoData,
        RangeFinder_OutOfRangeLow,
        RangeFinder_OutOfRangeHigh,
        RangeFinder_Good
    };

    // The RangeFinder_State structure is filled in by the backend driver
    struct RangeFinder_State {
        uint16_t distance_cm;           // distance: in cm
        uint16_t voltage_mv;            // voltage in millivolts, if applicable, otherwise 0
        enum RangeFinder_Status status; // sensor status
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

    // detect and initialise any available rangefinders
    void init(enum Rotation orientation_default);

    // update state of all rangefinders. Should be called at around
    // 10Hz from main loop
    void update(void);

    // Handle an incoming DISTANCE_SENSOR message (from a MAVLink enabled range finder)
    void handle_msg(mavlink_message_t *msg);

    // return true if we have a range finder with the specified orientation
    bool has_orientation(enum Rotation orientation) const;

    // find first range finder instance with the specified orientation
    AP_RangeFinder_Backend *find_instance(enum Rotation orientation) const;

    AP_RangeFinder_Backend *get_backend(uint8_t id) const;

    // methods to return a distance on a particular orientation from
    // any sensor which can current supply it
    uint16_t distance_cm_orient(enum Rotation orientation) const;
    uint16_t voltage_mv_orient(enum Rotation orientation) const;
    int16_t max_distance_cm_orient(enum Rotation orientation) const;
    int16_t min_distance_cm_orient(enum Rotation orientation) const;
    int16_t ground_clearance_cm_orient(enum Rotation orientation) const;
    MAV_DISTANCE_SENSOR get_mav_distance_sensor_type_orient(enum Rotation orientation) const;
    RangeFinder_Status status_orient(enum Rotation orientation) const;
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
    AP_SerialManager &serial_manager;
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
