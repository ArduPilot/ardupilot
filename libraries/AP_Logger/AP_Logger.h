/* ************************************************************ */
/* Test for AP_Logger Log library                               */
/* ************************************************************ */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_AHRS/AP_AHRS_DCM.h>
#include <AP_AHRS/AP_AHRS_NavEKF.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Logger/LogStructure.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AP_InertialSensor/AP_InertialSensor_Backend.h>
#include <AP_Vehicle/ModeReason.h>

#include <stdint.h>

#include "LoggerMessageWriter.h"

class AP_Logger_Backend;
class AP_AHRS;
class AP_AHRS_View;

// do not do anything here apart from add stuff; maintaining older
// entries means log analysis is easier
enum class LogEvent : uint8_t {
    ARMED = 10,
    DISARMED = 11,
    AUTO_ARMED = 15,
    LAND_COMPLETE_MAYBE = 17,
    LAND_COMPLETE = 18,
    NOT_LANDED = 28,
    LOST_GPS = 19,
    FLIP_START = 21,
    FLIP_END = 22,
    SET_HOME = 25,
    SET_SIMPLE_ON = 26,
    SET_SIMPLE_OFF = 27,
    SET_SUPERSIMPLE_ON = 29,
    AUTOTUNE_INITIALISED = 30,
    AUTOTUNE_OFF = 31,
    AUTOTUNE_RESTART = 32,
    AUTOTUNE_SUCCESS = 33,
    AUTOTUNE_FAILED = 34,
    AUTOTUNE_REACHED_LIMIT = 35,
    AUTOTUNE_PILOT_TESTING = 36,
    AUTOTUNE_SAVEDGAINS = 37,
    SAVE_TRIM = 38,
    SAVEWP_ADD_WP = 39,
    FENCE_ENABLE = 41,
    FENCE_DISABLE = 42,
    ACRO_TRAINER_OFF = 43,
    ACRO_TRAINER_LEVELING = 44,
    ACRO_TRAINER_LIMITED = 45,
    GRIPPER_GRAB = 46,
    GRIPPER_RELEASE = 47,
    PARACHUTE_DISABLED = 49,
    PARACHUTE_ENABLED = 50,
    PARACHUTE_RELEASED = 51,
    LANDING_GEAR_DEPLOYED = 52,
    LANDING_GEAR_RETRACTED = 53,
    MOTORS_EMERGENCY_STOPPED = 54,
    MOTORS_EMERGENCY_STOP_CLEARED = 55,
    MOTORS_INTERLOCK_DISABLED = 56,
    MOTORS_INTERLOCK_ENABLED = 57,
    ROTOR_RUNUP_COMPLETE = 58, // Heli only
    ROTOR_SPEED_BELOW_CRITICAL = 59, // Heli only
    EKF_ALT_RESET = 60,
    LAND_CANCELLED_BY_PILOT = 61,
    EKF_YAW_RESET = 62,
    AVOIDANCE_ADSB_ENABLE = 63,
    AVOIDANCE_ADSB_DISABLE = 64,
    AVOIDANCE_PROXIMITY_ENABLE = 65,
    AVOIDANCE_PROXIMITY_DISABLE = 66,
    GPS_PRIMARY_CHANGED = 67,
    WINCH_RELAXED = 68,
    WINCH_LENGTH_CONTROL = 69,
    WINCH_RATE_CONTROL = 70,
    ZIGZAG_STORE_A = 71,
    ZIGZAG_STORE_B = 72,
    LAND_REPO_ACTIVE = 73,
    STANDBY_ENABLE = 74,
    STANDBY_DISABLE = 75,

    SURFACED = 163,
    NOT_SURFACED = 164,
    BOTTOMED = 165,
    NOT_BOTTOMED = 166,
};

enum class LogDataID : uint8_t {
    AP_STATE = 7,
// SYSTEM_TIME_SET = 8,
    INIT_SIMPLE_BEARING = 9,
};

enum class LogErrorSubsystem : uint8_t {
    MAIN = 1,
    RADIO = 2,
    COMPASS = 3,
    OPTFLOW = 4,   // not used
    FAILSAFE_RADIO = 5,
    FAILSAFE_BATT = 6,
    FAILSAFE_GPS = 7,   // not used
    FAILSAFE_GCS = 8,
    FAILSAFE_FENCE = 9,
    FLIGHT_MODE = 10,
    GPS = 11,
    CRASH_CHECK = 12,
    FLIP = 13,
    AUTOTUNE = 14,  // not used
    PARACHUTES = 15,
    EKFCHECK = 16,
    FAILSAFE_EKFINAV = 17,
    BARO = 18,
    CPU = 19,
    FAILSAFE_ADSB = 20,
    TERRAIN = 21,
    NAVIGATION = 22,
    FAILSAFE_TERRAIN = 23,
    EKF_PRIMARY = 24,
    THRUST_LOSS_CHECK = 25,
    FAILSAFE_SENSORS = 26,
    FAILSAFE_LEAK = 27,
    PILOT_INPUT = 28,
    FAILSAFE_VIBE = 29,
};

// bizarrely this enumeration has lots of duplicate values, offering
// very little in the way of typesafety
enum class LogErrorCode : uint8_t {
// general error codes
    ERROR_RESOLVED  = 0,
    FAILED_TO_INITIALISE = 1,
    UNHEALTHY = 4,
// subsystem specific error codes -- radio
    RADIO_LATE_FRAME = 2,
// subsystem specific error codes -- failsafe_thr, batt, gps
    FAILSAFE_RESOLVED = 0,
    FAILSAFE_OCCURRED = 1,
// subsystem specific error codes -- main
    MAIN_INS_DELAY = 1,
// subsystem specific error codes -- crash checker
    CRASH_CHECK_CRASH = 1,
    CRASH_CHECK_LOSS_OF_CONTROL = 2,
// subsystem specific error codes -- flip
    FLIP_ABANDONED = 2,
// subsystem specific error codes -- terrain
    MISSING_TERRAIN_DATA = 2,
// subsystem specific error codes -- navigation
    FAILED_TO_SET_DESTINATION = 2,
    RESTARTED_RTL = 3,
    FAILED_CIRCLE_INIT = 4,
    DEST_OUTSIDE_FENCE = 5,
    RTL_MISSING_RNGFND = 6,

// parachute failed to deploy because of low altitude or landed
    PARACHUTE_TOO_LOW = 2,
    PARACHUTE_LANDED = 3,
// EKF check definitions
    EKFCHECK_BAD_VARIANCE = 2,
    EKFCHECK_VARIANCE_CLEARED = 0,
// Baro specific error codes
    BARO_GLITCH = 2,
    BAD_DEPTH = 3, // sub-only
// GPS specific error coces
    GPS_GLITCH = 2,
};

// fwd declarations to avoid include errors
class AC_AttitudeControl;
class AC_PosControl;

class AP_Logger
{
    friend class AP_Logger_Backend; // for _num_types

public:
    FUNCTOR_TYPEDEF(vehicle_startup_message_Writer, void);

    AP_Logger(const AP_Int32 &log_bitmask);

    /* Do not allow copies */
    AP_Logger(const AP_Logger &other) = delete;
    AP_Logger &operator=(const AP_Logger&) = delete;

    // get singleton instance
    static AP_Logger *get_singleton(void) {
        return _singleton;
    }

    // initialisation
    void Init(const struct LogStructure *structure, uint8_t num_types);
    void set_num_types(uint8_t num_types) { _num_types = num_types; }

    bool CardInserted(void);

    // erase handling
    void EraseAll();

    /* Write a block of data at current offset */
    void WriteBlock(const void *pBuffer, uint16_t size);
    /* Write an *important* block of data at current offset */
    void WriteCriticalBlock(const void *pBuffer, uint16_t size);

    // high level interface
    uint16_t find_last_log() const;
    void get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page);
    uint16_t get_num_logs(void);

    void setVehicle_Startup_Writer(vehicle_startup_message_Writer writer);

    void PrepForArming();

    void EnableWrites(bool enable) { _writes_enabled = enable; }
    bool WritesEnabled() const { return _writes_enabled; }

    void StopLogging();

    void Write_Parameter(const char *name, float value);
    void Write_Event(LogEvent id);
    void Write_Error(LogErrorSubsystem sub_system,
                     LogErrorCode error_code);
    void Write_GPS(uint8_t instance, uint64_t time_us=0);
    void Write_IMU();
    void Write_IMUDT(uint64_t time_us, uint8_t imu_mask);
    bool Write_ISBH(uint16_t seqno,
                        AP_InertialSensor::IMU_SENSOR_TYPE sensor_type,
                        uint8_t instance,
                        uint16_t multiplier,
                        uint16_t sample_count,
                        uint64_t sample_us,
                        float sample_rate_hz);
    bool Write_ISBD(uint16_t isb_seqno,
                        uint16_t seqno,
                        const int16_t x[32],
                        const int16_t y[32],
                        const int16_t z[32]);
    void Write_Vibration();
    void Write_RCIN(void);
    void Write_RCOUT(void);
    void Write_RSSI();
    void Write_Rally();
    void Write_Baro(uint64_t time_us=0);
    void Write_Power(void);
    void Write_AHRS2();
    void Write_POS();
    void Write_Radio(const mavlink_radio_t &packet);
    void Write_Message(const char *message);
    void Write_MessageF(const char *fmt, ...);
    void Write_CameraInfo(enum LogMessages msg, const Location &current_loc, uint64_t timestamp_us=0);
    void Write_Camera(const Location &current_loc, uint64_t timestamp_us=0);
    void Write_Trigger(const Location &current_loc);
    void Write_ESC(uint8_t id, uint64_t time_us, int32_t rpm, uint16_t voltage, uint16_t current, int16_t esc_temp, uint16_t current_tot, int16_t motor_temp);
    void Write_ServoStatus(uint64_t time_us, uint8_t id, float position, float force, float speed, uint8_t power_pct);
    void Write_ESCStatus(uint64_t time_us, uint8_t id, uint32_t error_count, float voltage, float current, float temperature, int32_t rpm, uint8_t power_pct);
    void Write_Attitude(const Vector3f &targets);
    void Write_AttitudeView(AP_AHRS_View &ahrs, const Vector3f &targets);
    void Write_Current();
    void Write_Compass(uint64_t time_us=0);
    void Write_Mode(uint8_t mode, const ModeReason reason);

    void Write_EntireMission();
    void Write_Command(const mavlink_command_int_t &packet, MAV_RESULT result, bool was_command_long=false);
    void Write_Mission_Cmd(const AP_Mission &mission,
                               const AP_Mission::Mission_Command &cmd);
    void Write_Origin(uint8_t origin_type, const Location &loc);
    void Write_RPM(const AP_RPM &rpm_sensor);
    void Write_Rate(const AP_AHRS_View *ahrs,
                        const AP_Motors &motors,
                        const AC_AttitudeControl &attitude_control,
                        const AC_PosControl &pos_control);
    void Write_RallyPoint(uint8_t total,
                          uint8_t sequence,
                          const RallyLocation &rally_point);
    void Write_VisualOdom(float time_delta, const Vector3f &angle_delta, const Vector3f &position_delta, float confidence);
    void Write_VisualPosition(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, float roll, float pitch, float yaw, uint8_t reset_counter);
    void Write_VisualVelocity(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter);
    void Write_AOA_SSA(AP_AHRS &ahrs);
    void Write_Beacon(AP_Beacon &beacon);
    void Write_Proximity(AP_Proximity &proximity);
    void Write_SRTL(bool active, uint16_t num_points, uint16_t max_points, uint8_t action, const Vector3f& point);
    void Write_OABendyRuler(bool active, float target_yaw, float margin, const Location &final_dest, const Location &oa_dest);
    void Write_OADijkstra(uint8_t state, uint8_t error_id, uint8_t curr_point, uint8_t tot_points, const Location &final_dest, const Location &oa_dest);

    void Write(const char *name, const char *labels, const char *fmt, ...);
    void Write(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...);
    void WriteCritical(const char *name, const char *labels, const char *fmt, ...);
    void WriteCritical(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...);
    void WriteV(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, va_list arg_list, bool is_critical=false);

    // This structure provides information on the internal member data of a PID for logging purposes
    struct PID_Info {
        float target;
        float actual;
        float error;
        float P;
        float I;
        float D;
        float FF;
    };

    void Write_PID(uint8_t msg_type, const PID_Info &info);

    // returns true if logging of a message should be attempted
    bool should_log(uint32_t mask) const;

    bool logging_started(void);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only AP_Logger_File support this:
    void flush(void);
#endif

    void handle_mavlink_msg(class GCS_MAVLINK &, const mavlink_message_t &msg);

    void periodic_tasks(); // may want to split this into GCS/non-GCS duties

    // number of blocks that have been dropped
    uint32_t num_dropped(void) const;

    // accesss to public parameters
    void set_force_log_disarmed(bool force_logging) { _force_log_disarmed = force_logging; }
    bool log_while_disarmed(void) const;
    uint8_t log_replay(void) const { return _params.log_replay; }
    
    vehicle_startup_message_Writer _vehicle_messages;

    // parameter support
    static const struct AP_Param::GroupInfo        var_info[];
    struct {
        AP_Int8 backend_types;
        AP_Int8 file_bufsize; // in kilobytes
        AP_Int8 file_disarm_rot;
        AP_Int8 log_disarmed;
        AP_Int8 log_replay;
        AP_Int8 mav_bufsize; // in kilobytes
        AP_Int16 file_timeout; // in seconds
    } _params;

    const struct LogStructure *structure(uint16_t num) const;
    const struct UnitStructure *unit(uint16_t num) const;
    const struct MultiplierStructure *multiplier(uint16_t num) const;

    // methods for mavlink SYS_STATUS message (send_sys_status)
    // these methods cover only the first logging backend used -
    // typically AP_Logger_File.
    bool logging_present() const;
    bool logging_enabled() const;
    bool logging_failed() const;

    // notify logging subsystem of an arming failure. This triggers
    // logging for HAL_LOGGER_ARM_PERSIST seconds
    void arming_failure() { _last_arming_failure_ms = AP_HAL::millis(); }

    void set_vehicle_armed(bool armed_state);
    bool vehicle_is_armed() const { return _armed; }

    void handle_log_send();
    bool in_log_download() const {
        return transfer_activity != TransferActivity::IDLE;
    }

    float quiet_nanf() const { return nanf("0x4152"); } // "AR"
    double quiet_nan() const { return nan("0x4152445550490a"); } // "ARDUPI"

    // returns true if msg_type is associated with a message
    bool msg_type_in_use(uint8_t msg_type) const;

    // calculate the length of a message using fields specified in
    // fmt; includes the message header
    int16_t Write_calc_msg_len(const char *fmt) const;

    // this structure looks much like struct LogStructure in
    // LogStructure.h, however we need to remember a pointer value for
    // efficiency of finding message types
    struct log_write_fmt {
        struct log_write_fmt *next;
        uint8_t msg_type;
        uint8_t msg_len;
        uint8_t sent_mask; // bitmask of backends sent to
        const char *name;
        const char *fmt;
        const char *labels;
        const char *units;
        const char *mults;
    } *log_write_fmts;

    // return (possibly allocating) a log_write_fmt for a name
    struct log_write_fmt *msg_fmt_for_name(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, const bool direct_comp = false);

    // output a FMT message for each backend if not already done so
    void Safe_Write_Emit_FMT(log_write_fmt *f);

protected:

    const struct LogStructure *_structures;
    uint8_t _num_types;
    const struct UnitStructure *_units = log_Units;
    const struct MultiplierStructure *_multipliers = log_Multipliers;
    const uint8_t _num_units = (sizeof(log_Units) / sizeof(log_Units[0]));
    const uint8_t _num_multipliers = (sizeof(log_Multipliers) / sizeof(log_Multipliers[0]));

    /* Write a block with specified importance */
    /* might be useful if you have a boolean indicating a message is
     * important... */
    void WritePrioritisedBlock(const void *pBuffer, uint16_t size,
                               bool is_critical);

private:
    #define LOGGER_MAX_BACKENDS 2
    uint8_t _next_backend;
    AP_Logger_Backend *backends[LOGGER_MAX_BACKENDS];
    const AP_Int32 &_log_bitmask;

    enum class Backend_Type : uint8_t {
        NONE       = 0,
        FILESYSTEM = (1<<0),
        MAVLINK    = (1<<1),
        BLOCK      = (1<<2),
    };

    /*
     * support for dynamic Write; user-supplies name, format,
     * labels and values in a single function call.
     */
    HAL_Semaphore log_write_fmts_sem;

    // return (possibly allocating) a log_write_fmt for a name
    const struct log_write_fmt *log_write_fmt_for_msg_type(uint8_t msg_type) const;

    const struct LogStructure *structure_for_msg_type(uint8_t msg_type);

    // return a msg_type which is not currently in use (or -1 if none available)
    int16_t find_free_msg_type() const;

    // fill LogStructure with information about msg_type
    bool fill_log_write_logstructure(struct LogStructure &logstruct, const uint8_t msg_type) const;

    bool _armed;

    void Write_Baro_instance(uint64_t time_us, uint8_t baro_instance, enum LogMessages type);
    void Write_IMU_instance(uint64_t time_us,
                                uint8_t imu_instance,
                                enum LogMessages type);
    void Write_Compass_instance(uint64_t time_us,
                                    uint8_t mag_instance,
                                    enum LogMessages type);
    void Write_Current_instance(uint64_t time_us, uint8_t battery_instance);
    void Write_IMUDT_instance(uint64_t time_us,
                                  uint8_t imu_instance,
                                  enum LogMessages type);

    void backend_starting_new_log(const AP_Logger_Backend *backend);

    static AP_Logger *_singleton;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    bool validate_structure(const struct LogStructure *logstructure, int16_t offset);
    void validate_structures(const struct LogStructure *logstructures, const uint8_t num_types);
    void dump_structure_field(const struct LogStructure *logstructure, const char *label, const uint8_t fieldnum);
    void dump_structures(const struct LogStructure *logstructures, const uint8_t num_types);
    void assert_same_fmt_for_name(const log_write_fmt *f,
                                  const char *name,
                                  const char *labels,
                                  const char *units,
                                  const char *mults,
                                  const char *fmt) const;
    const char* unit_name(const uint8_t unit_id);
    double multiplier_name(const uint8_t multiplier_id);
    bool seen_ids[256] = { };
    bool labels_string_is_good(const char *labels) const;
#endif

    // possibly expensive calls to start log system:
    void Prep();

    bool _writes_enabled:1;
    bool _force_log_disarmed:1;

    /* support for retrieving logs via mavlink: */

    enum class TransferActivity {
        IDLE,    // not doing anything, all file descriptors closed
        LISTING, // actively sending log_entry packets
        SENDING, // actively sending log_sending packets
    } transfer_activity = TransferActivity::IDLE;

    // next log list entry to send
    uint16_t _log_next_list_entry;

    // last log list entry to send
    uint16_t _log_last_list_entry;

    // number of log files
    uint16_t _log_num_logs;

    // log number for data send
    uint16_t _log_num_data;

    // offset in log
    uint32_t _log_data_offset;

    // size of log file
    uint32_t _log_data_size;

    // number of bytes left to send
    uint32_t _log_data_remaining;

    // start page of log data
    uint32_t _log_data_page;

    GCS_MAVLINK *_log_sending_link;
    HAL_Semaphore _log_send_sem;

    // last time arming failed, for backends
    uint32_t _last_arming_failure_ms;

    bool should_handle_log_message();
    void handle_log_message(class GCS_MAVLINK &, const mavlink_message_t &msg);

    void handle_log_request_list(class GCS_MAVLINK &, const mavlink_message_t &msg);
    void handle_log_request_data(class GCS_MAVLINK &, const mavlink_message_t &msg);
    void handle_log_request_erase(class GCS_MAVLINK &, const mavlink_message_t &msg);
    void handle_log_request_end(class GCS_MAVLINK &, const mavlink_message_t &msg);
    void handle_log_send_listing(); // handle LISTING state
    void handle_log_sending(); // handle SENDING state
    bool handle_log_send_data(); // send data chunk to client

    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);

    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);

    /* end support for retrieving logs via mavlink: */

};

namespace AP {
    AP_Logger &logger();
};
