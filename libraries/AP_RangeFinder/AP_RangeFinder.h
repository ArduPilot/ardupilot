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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_MSP/msp.h>
#include "AP_RangeFinder_Params.h"

// Maximum number of range finder instances available on this platform
#ifndef RANGEFINDER_MAX_INSTANCES 
  #if AP_RANGEFINDER_ENABLED
  #define RANGEFINDER_MAX_INSTANCES 10
  #else
  #define RANGEFINDER_MAX_INSTANCES 1
  #endif
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
    friend class AP_RangeFinder_DroneCAN;
public:
    RangeFinder();

    /* Do not allow copies */
    CLASS_NO_COPY(RangeFinder);

    // RangeFinder driver types
    enum class Type {
        NONE   = 0,
#if AP_RANGEFINDER_ANALOG_ENABLED
        ANALOG = 1,
#endif
#if AP_RANGEFINDER_MAXSONARI2CXL_ENABLED
        MBI2C  = 2,
#endif
#if AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED
        PLI2C  = 3,
#endif
//        PX4    = 4, // no longer used, but may be in some user's parameters
#if AP_RANGEFINDER_PWM_ENABLED
        PX4_PWM= 5,
#endif
#if AP_RANGEFINDER_BBB_PRU_ENABLED
        BBB_PRU= 6,
#endif
#if AP_RANGEFINDER_LWI2C_ENABLED
        LWI2C  = 7,
#endif
#if AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED
        LWSER  = 8,
#endif
#if AP_RANGEFINDER_BEBOP_ENABLED
        BEBOP  = 9,
#endif
#if AP_RANGEFINDER_MAVLINK_ENABLED
        MAVLink = 10,
#endif
#if AP_RANGEFINDER_USD1_SERIAL_ENABLED
        USD1_Serial = 11,
#endif
#if AP_RANGEFINDER_LEDDARONE_ENABLED
        LEDDARONE = 12,
#endif
#if AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED
        MBSER  = 13,
#endif
#if AP_RANGEFINDER_TRI2C_ENABLED
        TRI2C  = 14,
#endif
#if AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED
        PLI2CV3= 15,
#endif
        VL53L0X = 16,
#if AP_RANGEFINDER_NMEA_ENABLED
        NMEA = 17,
#endif
#if AP_RANGEFINDER_WASP_ENABLED
        WASP = 18,
#endif
#if AP_RANGEFINDER_BENEWAKE_TF02_ENABLED
        BenewakeTF02 = 19,
#endif
#if AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED
        BenewakeTFmini = 20,
#endif
#if AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED
        PLI2CV3HP = 21,
#endif
#if AP_RANGEFINDER_PWM_ENABLED
        PWM = 22,
#endif
#if AP_RANGEFINDER_BLPING_ENABLED
        BLPing = 23,
#endif
#if AP_RANGEFINDER_DRONECAN_ENABLED
        UAVCAN = 24,
#endif
#if AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED
        BenewakeTFminiPlus = 25,
#endif
#if AP_RANGEFINDER_LANBAO_ENABLED
        Lanbao = 26,
#endif
#if AP_RANGEFINDER_BENEWAKE_TF03_ENABLED
        BenewakeTF03 = 27,
#endif
        VL53L1X_Short = 28,
#if AP_RANGEFINDER_LEDDARVU8_ENABLED
        LeddarVu8_Serial = 29,
#endif
#if AP_RANGEFINDER_HC_SR04_ENABLED
        HC_SR04 = 30,
#endif
#if AP_RANGEFINDER_GYUS42V2_ENABLED
        GYUS42v2 = 31,
#endif
#if HAL_MSP_RANGEFINDER_ENABLED
        MSP = 32,
#endif
#if AP_RANGEFINDER_USD1_CAN_ENABLED
        USD1_CAN = 33,
#endif
#if AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
        Benewake_CAN = 34,
#endif
#if AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED
        TeraRanger_Serial = 35,
#endif
#if AP_RANGEFINDER_LUA_ENABLED
        Lua_Scripting = 36,
#endif
#if AP_RANGEFINDER_NOOPLOOP_ENABLED
        NoopLoop_P = 37,
#endif
#if AP_RANGEFINDER_TOFSENSEP_CAN_ENABLED
        TOFSenseP_CAN = 38,
#endif
#if AP_RANGEFINDER_NRA24_CAN_ENABLED
        NRA24_CAN = 39,
#endif
#if AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED
        TOFSenseF_I2C = 40,
#endif
#if AP_RANGEFINDER_JRE_SERIAL_ENABLED
        JRE_Serial = 41,
#endif
#if AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED
        Ainstein_LR_D1 = 42,
#endif
#if AP_RANGEFINDER_RDS02UF_ENABLED
        RDS02UF = 43,
#endif
#if AP_RANGEFINDER_SIM_ENABLED
        SIM = 100,
#endif
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

    static constexpr int8_t SIGNAL_QUALITY_MIN = 0;
    static constexpr int8_t SIGNAL_QUALITY_MAX = 100;
    static constexpr int8_t SIGNAL_QUALITY_UNKNOWN = -1;

    // The RangeFinder_State structure is filled in by the backend driver
    struct RangeFinder_State {
        float distance_m;               // distance in meters
        int8_t signal_quality_pct;      // measurement quality in percent 0-100, -1 -> quality is unknown
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

    /*
      Return the number of range finder instances. Note that if users
      sets up rangefinders with a gap in the types then this is the
      index of the maximum sensor ID plus one, so this gives the value
      that should be used when iterating over all sensors
    */
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

#if HAL_MSP_RANGEFINDER_ENABLED
    // Handle an incoming DISTANCE_SENSOR message (from a MSP enabled range finder)
    void handle_msp(const MSP::msp_rangefinder_data_message_t &pkt);
#endif
    // return true if we have a range finder with the specified orientation
    bool has_orientation(enum Rotation orientation) const;

    // find first range finder instance with the specified orientation
    AP_RangeFinder_Backend *find_instance(enum Rotation orientation) const;

    AP_RangeFinder_Backend *get_backend(uint8_t id) const;

    // get rangefinder type for an ID
    Type get_type(uint8_t id) const {
        return id >= RANGEFINDER_MAX_INSTANCES? Type::NONE : Type(params[id].type.get());
    }

    // get rangefinder address (for AP_Periph CAN)
    uint8_t get_address(uint8_t id) const {
        return id >= RANGEFINDER_MAX_INSTANCES? 0 : uint8_t(params[id].address.get());
    }
    
    // methods to return a distance on a particular orientation from
    // any sensor which can current supply it
    float distance_orient(enum Rotation orientation) const;
    uint16_t distance_cm_orient(enum Rotation orientation) const;
    int8_t signal_quality_pct_orient(enum Rotation orientation) const;
    int16_t max_distance_cm_orient(enum Rotation orientation) const;
    int16_t min_distance_cm_orient(enum Rotation orientation) const;
    int16_t ground_clearance_cm_orient(enum Rotation orientation) const;
    MAV_DISTANCE_SENSOR get_mav_distance_sensor_type_orient(enum Rotation orientation) const;
    RangeFinder::Status status_orient(enum Rotation orientation) const;
    bool has_data_orient(enum Rotation orientation) const;
    uint8_t range_valid_count_orient(enum Rotation orientation) const;
    const Vector3f &get_pos_offset_orient(enum Rotation orientation) const;
    uint32_t last_reading_ms(enum Rotation orientation) const;

    // get temperature reading in C.  returns true on success and populates temp argument
    bool get_temp(enum Rotation orientation, float &temp) const;

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
    HAL_Semaphore detect_sem;
    float estimated_terrain_height;
    Vector3f pos_offset_zero;   // allows returning position offsets of zero for invalid requests

    void detect_instance(uint8_t instance, uint8_t& serial_instance);

    bool _add_backend(AP_RangeFinder_Backend *driver, uint8_t instance, uint8_t serial_instance=0);

    uint32_t _log_rfnd_bit = -1;
    void Log_RFND() const;
};

namespace AP {
    RangeFinder *rangefinder();
};
