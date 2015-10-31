/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef DataFlash_h
#define DataFlash_h

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_Baro.h>
#include <AP_AHRS.h>
#include "../AP_Airspeed/AP_Airspeed.h"
#include "../AP_BattMonitor/AP_BattMonitor.h"
#include <stdint.h>

#include "LogStructure.h"

#include <GCS_MAVLink.h> // for mavlink_msg_t

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <uORB/topics/esc_status.h>
#endif

class DataFlash_MAVLink;
class DFMessageWriter_Factory;
class DataFlash_Backend;

enum DataFlash_Backend_Type {
    DATAFLASH_BACKEND_NONE = 0,
    DATAFLASH_BACKEND_FILE = 1,
    DATAFLASH_BACKEND_MAVLINK = 2,
    DATAFLASH_BACKEND_BOTH = 3,
};

class DataFlash_Class
{

public:
    DataFlash_Class() :
        _startup_messagewriter_factory(NULL),
        _next_backend(0)
        { }

    // initialisation
    void Init(const struct LogStructure *structure, uint8_t num_types,
              DFMessageWriter_Factory *factory);

    bool CardInserted(void);

    // erase handling
    bool NeedErase(void);
    void EraseAll();

    void Prep_MinSpace();

    /* Write a block of data at current offset */
    void WriteBlock(const void *pBuffer, uint16_t size);

    // high level interface
    uint16_t find_last_log(void);
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
    uint16_t get_num_logs(void);
#ifndef DATAFLASH_NO_CLI
    void LogReadProcess(uint16_t log_num,
                                uint16_t start_page, uint16_t end_page, 
                                void (*printMode)(AP_HAL::BetterStream *port, uint8_t mode),
                                AP_HAL::BetterStream *port);
    void DumpPageInfo(AP_HAL::BetterStream *port);
    void ShowDeviceInfo(AP_HAL::BetterStream *port);
    void ListAvailableLogs(AP_HAL::BetterStream *port);
#endif // DATAFLASH_NO_CLI

    void setStartupMessageWriterFactory(DFMessageWriter_Factory *messagewriter_factory);

    void StartNewLog(void);
    void AddLogFormats(const struct LogStructure *structures, uint8_t num_types);
    void EnableWrites(bool enable);
    void Log_Write_SysInfo(const prog_char_t *firmware_string);
    void Log_Write_Format(const struct LogStructure *structure);
    void Log_Write_Parameter(const char *name, float value);
    void Log_Write_GPS(const AP_GPS &gps, uint8_t instance, int32_t relative_alt);
    void Log_Write_IMU(const AP_InertialSensor &ins);
    void Log_Write_RCIN(void);
    void Log_Write_RCOUT(void);
    void Log_Write_Baro(AP_Baro &baro);
    void Log_Write_Power(void);
    void Log_Write_AHRS2(AP_AHRS &ahrs);
#if AP_AHRS_NAVEKF_AVAILABLE
    void Log_Write_EKF(AP_AHRS_NavEKF &ahrs, bool optFlowEnabled);
#endif
    void Log_Write_MavCmd(uint16_t cmd_total, const mavlink_mission_item_t& mav_cmd);
    void Log_Write_Radio(const mavlink_radio_t &packet);
    void Log_Write_Message(const char *message);
    void Log_Write_Message_P(const prog_char_t *message);
    void Log_Write_Camera(const AP_AHRS &ahrs, const AP_GPS &gps, const Location &current_loc);
    void Log_Write_ESC(void);
    void Log_Write_Airspeed(AP_Airspeed &airspeed);
    void Log_Write_Attitude(AP_AHRS &ahrs, const Vector3f &targets);
	void Log_Write_Current(const AP_BattMonitor &battery, int16_t throttle);
    void Log_Write_Compass(const Compass &compass);
    void Log_Write_Mode(uint8_t mode);

    bool logging_started(void);

    // for DataFlash_MAVLink:
    void remote_log_block_status_msg(mavlink_channel_t chan,
                                     mavlink_message_t* msg);
    // end for DataFlash_MAVLink:

    void periodic_tasks(); // may want to split this into GCS/non-GCS duties

    // this is out here for the trickle-startup-messages logging.
    // Think before calling.
    bool Log_Write_Parameter(const AP_Param *ap, const AP_Param::ParamToken &token, 
                             enum ap_var_type type);

    DFMessageWriter_Factory *_startup_messagewriter_factory;

    static const struct AP_Param::GroupInfo        var_info[];
    struct {
        AP_Int8 backend_types;
    } _params;

protected:
    /*
    read and print a log entry using the format strings from the given structure
    */
    void _print_log_entry(uint8_t msg_type, 
                          void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                          AP_HAL::BetterStream *port);
    
    void Log_Fill_Format(const struct LogStructure *structure, struct log_Format &pkt);
    void Log_Write_Parameters(void);

    const struct LogStructure *_structures;
    uint8_t _num_types;
    bool _writes_enabled;

    /*
      read a block
    */
    void ReadBlock(void *pkt, uint16_t size);

private:
    #define DATAFLASH_MAX_BACKENDS 2
    uint8_t _next_backend;
    DataFlash_Backend *backends[DATAFLASH_MAX_BACKENDS];
};

#endif
