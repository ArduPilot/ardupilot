/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <DataFlash/LogStructure.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AP_InertialSensor/AP_InertialSensor_Backend.h>

#include <stdint.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <uORB/topics/esc_status.h>
#endif

#include "DFMessageWriter.h"

class DataFlash_Backend;

enum DataFlash_Backend_Type {
    DATAFLASH_BACKEND_NONE = 0,
    DATAFLASH_BACKEND_FILE = 1,
    DATAFLASH_BACKEND_MAVLINK = 2,
    DATAFLASH_BACKEND_BOTH = 3,
};

// fwd declarations to avoid include errors
class AC_AttitudeControl;
class AC_PosControl;

class DataFlash_Class
{
    friend class DataFlash_Backend; // for _num_types

public:
    FUNCTOR_TYPEDEF(print_mode_fn, void, AP_HAL::BetterStream*, uint8_t);
    FUNCTOR_TYPEDEF(vehicle_startup_message_Log_Writer, void);

    DataFlash_Class(const char *firmware_string, const AP_Int32 &log_bitmask);

    /* Do not allow copies */
    DataFlash_Class(const DataFlash_Class &other) = delete;
    DataFlash_Class &operator=(const DataFlash_Class&) = delete;

    // get singleton instance
    static DataFlash_Class *instance(void) {
        return _instance;
    }

    void set_mission(const AP_Mission *mission);

    // initialisation
    void Init(const struct LogStructure *structure, uint8_t num_types);

    bool CardInserted(void);

    // erase handling
    void EraseAll();

    /* Write a block of data at current offset */
    void WriteBlock(const void *pBuffer, uint16_t size);
    /* Write an *important* block of data at current offset */
    void WriteCriticalBlock(const void *pBuffer, uint16_t size);

    // high level interface
    uint16_t find_last_log() const;
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    uint16_t get_num_logs(void);
    void LogReadProcess(uint16_t log_num,
                                uint16_t start_page, uint16_t end_page, 
                                print_mode_fn printMode,
                                AP_HAL::BetterStream *port);
    void DumpPageInfo(AP_HAL::BetterStream *port);
    void ShowDeviceInfo(AP_HAL::BetterStream *port);
    void ListAvailableLogs(AP_HAL::BetterStream *port);

    void setVehicle_Startup_Log_Writer(vehicle_startup_message_Log_Writer writer);

    void PrepForArming();

    void EnableWrites(bool enable) { _writes_enabled = enable; }
    bool WritesEnabled() const { return _writes_enabled; }

    void StopLogging();

    void Log_Write_Parameter(const char *name, float value);
    void Log_Write_GPS(const AP_GPS &gps, uint8_t instance, uint64_t time_us=0);
    void Log_Write_RFND(const RangeFinder &rangefinder);
    void Log_Write_IMU(const AP_InertialSensor &ins);
    void Log_Write_IMUDT(const AP_InertialSensor &ins, uint64_t time_us, uint8_t imu_mask);
    bool Log_Write_ISBH(uint16_t seqno,
                        AP_InertialSensor::IMU_SENSOR_TYPE sensor_type,
                        uint8_t instance,
                        uint16_t multiplier,
                        uint16_t sample_count,
                        uint64_t sample_us,
                        float sample_rate_hz);
    bool Log_Write_ISBD(uint16_t isb_seqno,
                        uint16_t seqno,
                        const int16_t x[32],
                        const int16_t y[32],
                        const int16_t z[32]);
    void Log_Write_Vibration(const AP_InertialSensor &ins);
    void Log_Write_RCIN(void);
    void Log_Write_RCOUT(void);
    void Log_Write_RSSI(AP_RSSI &rssi);
    void Log_Write_Baro(AP_Baro &baro, uint64_t time_us=0);
    void Log_Write_Power(void);
    void Log_Write_AHRS2(AP_AHRS &ahrs);
    void Log_Write_POS(AP_AHRS &ahrs);
#if AP_AHRS_NAVEKF_AVAILABLE
    void Log_Write_EKF(AP_AHRS_NavEKF &ahrs);
#endif
    bool Log_Write_MavCmd(uint16_t cmd_total, const mavlink_mission_item_t& mav_cmd);
    void Log_Write_Radio(const mavlink_radio_t &packet);
    void Log_Write_Message(const char *message);
    void Log_Write_MessageF(const char *fmt, ...);
    void Log_Write_CameraInfo(enum LogMessages msg, const AP_AHRS &ahrs, const Location &current_loc);
    void Log_Write_Camera(const AP_AHRS &ahrs, const Location &current_loc);
    void Log_Write_Trigger(const AP_AHRS &ahrs, const Location &current_loc);
    void Log_Write_ESC(void);
    void Log_Write_Airspeed(AP_Airspeed &airspeed);
    void Log_Write_Attitude(AP_AHRS &ahrs, const Vector3f &targets);
    void Log_Write_AttitudeView(AP_AHRS_View &ahrs, const Vector3f &targets);
    void Log_Write_Current(const AP_BattMonitor &battery);
    void Log_Write_Compass(const Compass &compass, uint64_t time_us=0);
    void Log_Write_Mode(uint8_t mode, uint8_t reason = 0);

    void Log_Write_EntireMission(const AP_Mission &mission);
    void Log_Write_Mission_Cmd(const AP_Mission &mission,
                               const AP_Mission::Mission_Command &cmd);
    void Log_Write_Origin(uint8_t origin_type, const Location &loc);
    void Log_Write_RPM(const AP_RPM &rpm_sensor);
    void Log_Write_Rate(const AP_AHRS &ahrs,
                        const AP_Motors &motors,
                        const AC_AttitudeControl &attitude_control,
                        const AC_PosControl &pos_control);
    void Log_Write_Rally(const AP_Rally &rally);
    void Log_Write_VisualOdom(float time_delta, const Vector3f &angle_delta, const Vector3f &position_delta, float confidence);
    void Log_Write_AOA_SSA(AP_AHRS &ahrs);
    void Log_Write_Beacon(AP_Beacon &beacon);
    void Log_Write_Proximity(AP_Proximity &proximity);
    void Log_Write_SRTL(bool active, uint16_t num_points, uint16_t max_points, uint8_t action, const Vector3f& point);

    void Log_Write(const char *name, const char *labels, const char *fmt, ...);
    void Log_Write(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...);
    void Log_WriteV(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, va_list arg_list);

    // This structure provides information on the internal member data of a PID for logging purposes
    struct PID_Info {
        float desired;
        float P;
        float I;
        float D;
        float FF;
        float AFF;
    };

    void Log_Write_PID(uint8_t msg_type, const PID_Info &info);

    // returns true if logging of a message should be attempted
    bool should_log(uint32_t mask) const;

    bool logging_started(void);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only DataFlash_File support this:
    void flush(void);
#endif

    void handle_mavlink_msg(class GCS_MAVLINK &, mavlink_message_t* msg);

    void periodic_tasks(); // may want to split this into GCS/non-GCS duties

    // number of blocks that have been dropped
    uint32_t num_dropped(void) const;

    // accesss to public parameters
    bool log_while_disarmed(void) const { return _params.log_disarmed != 0; }
    uint8_t log_replay(void) const { return _params.log_replay; }
    
    vehicle_startup_message_Log_Writer _vehicle_messages;

    // parameter support
    static const struct AP_Param::GroupInfo        var_info[];
    struct {
        AP_Int8 backend_types;
        AP_Int8 file_bufsize; // in kilobytes
        AP_Int8 file_disarm_rot;
        AP_Int8 log_disarmed;
        AP_Int8 log_replay;
    } _params;

    const struct LogStructure *structure(uint16_t num) const;
    const struct UnitStructure *unit(uint16_t num) const;
    const struct MultiplierStructure *multiplier(uint16_t num) const;

    // methods for mavlink SYS_STATUS message (send_extended_status1)
    // these methods cover only the first logging backend used -
    // typically DataFlash_File.
    bool logging_present() const;
    bool logging_enabled() const;
    bool logging_failed() const;

    void set_vehicle_armed(bool armed_state);
    bool vehicle_is_armed() const { return _armed; }

    void handle_log_send(class GCS_MAVLINK &);
    bool in_log_download() const { return _in_log_download; }

    float quiet_nanf() const { return nanf("0x4152"); } // "AR"
    double quiet_nan() const { return nan("0x4152445550490a"); } // "ARDUPI"

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
    #define DATAFLASH_MAX_BACKENDS 2
    uint8_t _next_backend;
    DataFlash_Backend *backends[DATAFLASH_MAX_BACKENDS];
    const char *_firmware_string;
    const AP_Int32 &_log_bitmask;

    void internal_error() const;

    /*
     * support for dynamic Log_Write; user-supplies name, format,
     * labels and values in a single function call.
     */

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
    struct log_write_fmt *msg_fmt_for_name(const char *name, const char *labels, const char *units, const char *mults, const char *fmt);

    // returns true if msg_type is associated with a message
    bool msg_type_in_use(uint8_t msg_type) const;

    // return a msg_type which is not currently in use (or -1 if none available)
    int16_t find_free_msg_type() const;

    // fill LogStructure with information about msg_type
    bool fill_log_write_logstructure(struct LogStructure &logstruct, const uint8_t msg_type) const;

    // calculate the length of a message using fields specified in
    // fmt; includes the message header
    int16_t Log_Write_calc_msg_len(const char *fmt) const;

    bool _armed;

#if AP_AHRS_NAVEKF_AVAILABLE
    void Log_Write_EKF2(AP_AHRS_NavEKF &ahrs);
    void Log_Write_EKF3(AP_AHRS_NavEKF &ahrs);
#endif

    void Log_Write_Baro_instance(AP_Baro &baro, uint64_t time_us, uint8_t baro_instance, enum LogMessages type);
    void Log_Write_IMU_instance(const AP_InertialSensor &ins,
                                uint64_t time_us,
                                uint8_t imu_instance,
                                enum LogMessages type);
    void Log_Write_Compass_instance(const Compass &compass,
                                    uint64_t time_us,
                                    uint8_t mag_instance,
                                    enum LogMessages type);
    void Log_Write_Current_instance(const AP_BattMonitor &battery,
                                    uint64_t time_us,
                                    uint8_t battery_instance,
                                    enum LogMessages type,
                                    enum LogMessages celltype);
    void Log_Write_IMUDT_instance(const AP_InertialSensor &ins,
                                  uint64_t time_us,
                                  uint8_t imu_instance,
                                  enum LogMessages type);

    void backend_starting_new_log(const DataFlash_Backend *backend);

private:
    static DataFlash_Class *_instance;

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
#endif

    void Log_Write_EKF_Timing(const char *name, uint64_t time_us, const struct ekf_timing &timing);

    // possibly expensive calls to start log system:
    void Prep();

    bool _writes_enabled;

    /* support for retrieving logs via mavlink: */
    uint8_t  _log_listing:1; // sending log list
    uint8_t  _log_sending:1; // sending log data

    // bolean replicating old vehicle in_log_download flag:
    bool _in_log_download:1;

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
    uint16_t _log_data_page;

    int8_t _log_sending_chan = -1;

    bool should_handle_log_message();
    void handle_log_message(class GCS_MAVLINK &, mavlink_message_t *msg);

    void handle_log_request_list(class GCS_MAVLINK &, mavlink_message_t *msg);
    void handle_log_request_data(class GCS_MAVLINK &, mavlink_message_t *msg);
    void handle_log_request_erase(class GCS_MAVLINK &, mavlink_message_t *msg);
    void handle_log_request_end(class GCS_MAVLINK &, mavlink_message_t *msg);
    void handle_log_send_listing(class GCS_MAVLINK &);
    bool handle_log_send_data(class GCS_MAVLINK &);

    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);

    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);

    /* end support for retrieving logs via mavlink: */

};
