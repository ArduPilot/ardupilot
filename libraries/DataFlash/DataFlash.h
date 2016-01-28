/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef DataFlash_h
#define DataFlash_h

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

class DataFlash_Class
{
    friend class DataFlash_Backend; // for _num_types

public:
    FUNCTOR_TYPEDEF(print_mode_fn, void, AP_HAL::BetterStream*, uint8_t);
    FUNCTOR_TYPEDEF(vehicle_startup_message_Log_Writer, void);
    DataFlash_Class(const char *firmware_string) :
        _firmware_string(firmware_string)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    void set_mission(const AP_Mission *mission);

    // initialisation
    void Init(const struct LogStructure *structure, uint8_t num_types);
    bool CardInserted(void);

    // erase handling
    bool NeedErase(void);
    void EraseAll();

    // possibly expensive calls to start log system:
    bool NeedPrep();
    void Prep();

    /* Write a block of data at current offset */
    void WriteBlock(const void *pBuffer, uint16_t size);
    /* Write an *important* block of data at current offset */
    void WriteCriticalBlock(const void *pBuffer, uint16_t size);

    // high level interface
    uint16_t find_last_log() const;
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
    uint16_t get_num_logs(void);
    void LogReadProcess(uint16_t log_num,
                                uint16_t start_page, uint16_t end_page, 
                                print_mode_fn printMode,
                                AP_HAL::BetterStream *port);
    void DumpPageInfo(AP_HAL::BetterStream *port);
    void ShowDeviceInfo(AP_HAL::BetterStream *port);
    void ListAvailableLogs(AP_HAL::BetterStream *port);

    void setVehicle_Startup_Log_Writer(vehicle_startup_message_Log_Writer writer);

    void StartNewLog(void);
    void EnableWrites(bool enable);

    void Log_Write_Parameter(const char *name, float value);
    void Log_Write_GPS(const AP_GPS &gps, uint8_t instance, int32_t relative_alt);
    void Log_Write_RFND(const RangeFinder &rangefinder);
    void Log_Write_IMU(const AP_InertialSensor &ins);
    void Log_Write_IMUDT(const AP_InertialSensor &ins);
    void Log_Write_Vibration(const AP_InertialSensor &ins);
    void Log_Write_RCIN(void);
    void Log_Write_RCOUT(void);
    void Log_Write_RSSI(AP_RSSI &rssi);
    void Log_Write_Baro(AP_Baro &baro);
    void Log_Write_Power(void);
    void Log_Write_AHRS2(AP_AHRS &ahrs);
    void Log_Write_POS(AP_AHRS &ahrs);
#if AP_AHRS_NAVEKF_AVAILABLE
    void Log_Write_EKF(AP_AHRS_NavEKF &ahrs, bool optFlowEnabled);
    void Log_Write_EKF2(AP_AHRS_NavEKF &ahrs, bool optFlowEnabled);
#endif
    bool Log_Write_MavCmd(uint16_t cmd_total, const mavlink_mission_item_t& mav_cmd);
    void Log_Write_Radio(const mavlink_radio_t &packet);
    void Log_Write_Message(const char *message);
    void Log_Write_CameraInfo(enum LogMessages msg, const AP_AHRS &ahrs, const AP_GPS &gps, const Location &current_loc);
    void Log_Write_Camera(const AP_AHRS &ahrs, const AP_GPS &gps, const Location &current_loc);
    void Log_Write_Trigger(const AP_AHRS &ahrs, const AP_GPS &gps, const Location &current_loc);    
    void Log_Write_ESC(void);
    void Log_Write_Airspeed(AP_Airspeed &airspeed);
    void Log_Write_Attitude(AP_AHRS &ahrs, const Vector3f &targets);
    void Log_Write_Current(const AP_BattMonitor &battery, int16_t throttle);
    void Log_Write_Compass(const Compass &compass);
    void Log_Write_Mode(uint8_t mode);

    void Log_Write_EntireMission(const AP_Mission &mission);
    void Log_Write_Mission_Cmd(const AP_Mission &mission,
                               const AP_Mission::Mission_Command &cmd);
    void Log_Write_Origin(uint8_t origin_type, const Location &loc);
    void Log_Write_RPM(const AP_RPM &rpm_sensor);

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

    bool logging_started(void);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only DataFlash_File support this:
    void flush(void);
#endif

    // for DataFlash_MAVLink:
    void remote_log_block_status_msg(mavlink_channel_t chan,
                                     mavlink_message_t* msg);
    // end for DataFlash_MAVLink:

    void periodic_tasks(); // may want to split this into GCS/non-GCS duties

    vehicle_startup_message_Log_Writer _vehicle_messages;

    // parameter support
    static const struct AP_Param::GroupInfo        var_info[];
    struct {
        AP_Int8 backend_types;
        AP_Int8 file_bufsize; // in kilobytes
    } _params;

    const struct LogStructure *structure(uint16_t num) const;

protected:

    const struct LogStructure *_structures;
    uint8_t _num_types;

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
};

#endif
